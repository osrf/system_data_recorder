// Copyright 2021 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "sdr/sdr_component.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "rosbag2_storage/topic_metadata.hpp"
#include "rosbag2_transport/qos.hpp"
#include "yaml-cpp/yaml.h"

namespace sdr
{

SystemDataRecorder::SystemDataRecorder(
  const std::string & node_name,
  const rclcpp::NodeOptions & options,
  const std::string & bag_uri,
  const std::string & copy_destination,
  const unsigned int max_file_size,
  const std::unordered_map<std::string, std::string> & topics_and_types)
: rclcpp_lifecycle::LifecycleNode(node_name, options),
  topics_and_types_(topics_and_types),
  source_directory_(bag_uri)
{
  // Set up the storage options for writing the bag
  storage_options_.uri = bag_uri;
  // Only the "sqlite3" storage backend is supported; currently this is the only backend provided
  // by rosbag2 so this is not a problem at this time
  storage_options_.storage_id = "sqlite3";
  // Set the maximum size of each individual file
  storage_options_.max_bagfile_size = max_file_size;
  // Using a write cache is not required, but it helps avoid disc I/O becoming a bottleneck.
  // Set this option to 0 to disable the cache
  storage_options_.max_cache_size = 100*1024*1024;

  // Get the name of the bag directory from the bag URI - the bag URI should not be empty
  auto bag_directory_name = std::filesystem::path(bag_uri).filename();
  if (bag_directory_name.empty()) {
    // There was probably a slash on the end
    std::string bag_uri_copy = bag_uri;
    bag_directory_name = std::filesystem::path(
      bag_uri_copy.erase(bag_uri.size() - 1, 1)).filename();
  }
  // A directory named after the bag will be created under the copy_destination directory to put
  // the copied files into
  destination_directory_ = std::filesystem::path(copy_destination) / bag_directory_name;

  // Initially, the "last" bag file will be the first bag file
  last_bag_file_ = std::filesystem::path(bag_uri) / bag_directory_name.concat("_0.db3");
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// Lifecycle states
///////////////////////////////////////////////////////////////////////////////////////////////////

// In the on_configure state, we set up the rosbag2 infrastructure for recording by creating a
// writer and opening the storage. We also set up the worker thread that will copy files in
// parallel to the recording of data. A callback is registered with the writer so that we will get
// informed each time a new file is created in the bag.
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SystemDataRecorder::on_configure(const rclcpp_lifecycle::State & /* state */)
{
  RCLCPP_INFO(get_logger(), "Preparing to begin recording");

  RCLCPP_INFO(get_logger(), "Copying bag files to %s", destination_directory_.c_str());

  // Prepare for file-copying by creating the destination directory and setting up a worker thread
  try {
    if (!create_copy_destination()) {
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
  }
  catch (std::filesystem::filesystem_error const & ex) {
    RCLCPP_ERROR(get_logger(), "Could not create destination directory for bag backup");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }
  copy_thread_ = std::make_shared<std::thread>([this]{ copy_thread_main(); });

  // Notify the copy thread to get it into the correct state
  notify_state_change(SdrStateChange::PAUSED);

  // Prepare the rosbag2 objects for recording
  // A sequential writer is used as this is the only writer currently available
  writer_ = std::make_shared<rosbag2_cpp::Writer>(
    std::make_unique<rosbag2_cpp::writers::SequentialWriter>());
  // Add a callback for when a split occurs
  rosbag2_cpp::bag_events::WriterEventCallbacks callbacks;
  callbacks.write_split_callback =
    [this]
    (rosbag2_cpp::bag_events::WriteSplitInfo & info) {
      // Record the opened file - this will be the last file remaining to be copied when recording
      // is terminated
      last_bag_file_ = info.opened_file;
      // Notify the worker thread about the closed file being ready to copy
      notify_new_file_to_copy(info.closed_file);
    };
  writer_->add_event_callbacks(callbacks);
  // Open the bag
  writer_->open(
    storage_options_,
    {rmw_get_serialization_format(), rmw_get_serialization_format()});

  // Start receiving data - it won't be recorded yet because the node's lifecycle is not in the
  // Active state
  subscribe_to_topics();

  // Make sure cleanup happens when the on_cleanup transition occurs - this allows the node to be
  // cleaned up and then configured again (with the bag files moved away somewhere else) without
  // shutting it down
  cleaned_up = false;
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

// In the on_activate transition, we simply notify the worker thread that we are recording. Data
// will be written to the bag because the state changes to Active.
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SystemDataRecorder::on_activate(const rclcpp_lifecycle::State & /* state */)
{
  RCLCPP_INFO(get_logger(), "Starting recording");

  // Notify the copy thread
  notify_state_change(SdrStateChange::RECORDING);
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

// In the on_deactivate transition, we simply notify the worker thread that we are paused. Data
// will not be written to the bag because the state changes to Inactive.
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SystemDataRecorder::on_deactivate(const rclcpp_lifecycle::State & /* state */)
{
  RCLCPP_INFO(get_logger(), "Pausing recording");

  // Notify the copy thread
  notify_state_change(SdrStateChange::PAUSED);
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

// When cleaning up, we need to stop recording, stop receiving data (i.e. unsubscribe from topics),
// and ensure that the final files of the bag (the last data file and the metadata file, which gets
// written when the writer object destructs) are copied to the destination directory.
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SystemDataRecorder::on_cleanup(const rclcpp_lifecycle::State & /* state */)
{
  RCLCPP_INFO(get_logger(), "Stopping and finalising recording");
  // Avoid double-cleanup if on_shutdown is called after this
  cleaned_up = true;

  // Stop recording by stopping data from arriving
  unsubscribe_from_topics();
  // Clean up the writer so that the bag is closed and the metadata file is written
  writer_.reset();
  // Copy the final data file
  notify_new_file_to_copy(last_bag_file_);
  // Copy the metadata file
  notify_new_file_to_copy(source_directory_ / "metadata.yaml");

  // Notify the worker thread that we are cleaning up, so that it exits onces it's finished its
  // final work
  notify_state_change(SdrStateChange::FINISHED);
  copy_thread_->join();
  // Get rid of the worker thread
  copy_thread_.reset();

  RCLCPP_INFO(get_logger(), "Cleanup complete");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

// If not already performed, the shutdown transition performs the same functions as the cleanup
// transition.
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SystemDataRecorder::on_shutdown(const rclcpp_lifecycle::State & /* state */)
{
  RCLCPP_INFO(get_logger(), "Stopping and finalising recording (hard shutdown)");
  if (!cleaned_up) {
    // Stop recording
    unsubscribe_from_topics();
    // Clean up the writer
    writer_.reset();
    // Copy the final file
    notify_new_file_to_copy(last_bag_file_);
    // Copy the metadata file
    notify_new_file_to_copy(source_directory_ / "metadata.yaml");
  }

  // Notify the copy thread - but only if it hasn't already been cleaned up
  if (copy_thread_) {
    notify_state_change(SdrStateChange::FINISHED);
    copy_thread_->join();
    copy_thread_.reset();
  }
  RCLCPP_INFO(get_logger(), "Cleanup complete");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// Bag recording functionality
///////////////////////////////////////////////////////////////////////////////////////////////////

void SystemDataRecorder::subscribe_to_topics()
{
  for (const auto & topic_with_type : topics_and_types_) {
    subscribe_to_topic(topic_with_type.first, topic_with_type.second);
  }
}

void SystemDataRecorder::subscribe_to_topic(const std::string & topic, const std::string & type)
{
  // Get the QoS offered by the topic for saving in the bag
  auto offered_qos = get_serialised_offered_qos_for_topic(topic);
  // Find out what QoS is most appropriate to use when subscribing to the topic
  auto qos = get_appropriate_qos_for_topic(topic);

  // The metadata to pass to the writer object so it can register the topic in the bag
  auto topic_metadata = rosbag2_storage::TopicMetadata(
    {
      topic,  // Topic name
      type,  // Topic type, e.g. "example_interfaces/msg/String"
      rmw_get_serialization_format(),  // The serialization format, most likely to be "CDR"
      offered_qos  // The offered QoS profile for the topic in YAML
    }
  );
  // It is a good idea to create the topic in the writer prior to adding the subscription in case
  // data arrives after subscribing and before the topic is created. Although we should be ignoring
  // any data until the node is set to active, we maintain this good practice here for future
  // maintainability.
  writer_->create_topic(topic_metadata);

  // Create a generic subscriber. A generic subscriber received message data in serialized form,
  // which means that:
  // - No de-serialization will take place, saving that processing time, and
  // - The data type does not need to be known at compile time, so we don't need a templated
  //   callback for when message data is received.
  auto subscription = create_generic_subscription(
    topic,
    type,
    qos,
    [this, topic, type](std::shared_ptr<rclcpp::SerializedMessage> message) {
      // When a message is received, it should only be written to the bag if recording is not
      // paused (i.e. the node lifecycle state is "active"). If recording is paused, the message is
      // thrown away.
      if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        writer_->write(message, topic, type, rclcpp::Clock(RCL_SYSTEM_TIME).now());
      }
    });
  if (subscription) {
    subscriptions_.insert({topic, subscription});
    RCLCPP_INFO(get_logger(), "Subscribed to topic '%s'", topic.c_str());
  } else {
    writer_->remove_topic(topic_metadata);
    RCLCPP_ERROR(get_logger(), "Failed to subscribe to topic '%s'", topic.c_str());
  }
}

std::string SystemDataRecorder::get_serialised_offered_qos_for_topic(const std::string & topic)
{
  YAML::Node offered_qos_profiles;
  auto endpoints = get_publishers_info_by_topic(topic);
  for (const auto & endpoint : endpoints) {
    offered_qos_profiles.push_back(rosbag2_transport::Rosbag2QoS(endpoint.qos_profile()));
  }
  return YAML::Dump(offered_qos_profiles);
}

// Figure out the most appropriate QoS for a given topic. This method tries to decide if
// constrained QoS can be used, or if less-trustworthy QoS needs to be used to catch data from
// every publisher.
// Returns the QoS to use.
rclcpp::QoS SystemDataRecorder::get_appropriate_qos_for_topic(const std::string & topic)
{
  auto qos = rclcpp::QoS(rmw_qos_profile_default.depth);

  // Get the information about known publishers on this topic
  auto endpoints = get_publishers_info_by_topic(topic);
  if (endpoints.empty()) {
    // There are not yet any publishers on the topic.
    // Use the default QoS profile, as we do not know what publishers will use since there are not
    // yet any publishers on this topic.
    return qos;
  }

  // Count the number of reliable and transient-local publishers for the topic
  size_t reliability_reliable_endpoints_count = 0;
  size_t durability_transient_local_endpoints_count = 0;
  for (const auto & endpoint : endpoints) {
    const auto & profile = endpoint.qos_profile().get_rmw_qos_profile();
    if (profile.reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE) {
      ++reliability_reliable_endpoints_count;
    }
    if (profile.durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL) {
      ++durability_transient_local_endpoints_count;
    }
  }

  if (reliability_reliable_endpoints_count == endpoints.size()) {
    // All publishers are reliable, so we can use the reliable QoS
    qos.reliable();
  } else {
    if (reliability_reliable_endpoints_count > 0) {
      // There is a mix of QoS profiles amongst the publishers, so use the QoS setting that captures
      // all of them
      RCLCPP_WARN(
        get_logger(),
        "Some, but not all, publishers on topic \"%s\" are offering "
          "RMW_QOS_POLICY_RELIABILITY_RELIABLE. Falling back to "
          "RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT as it will connect to all publishers. Some "
          "messages from Reliable publishers could be dropped.",
        topic.c_str());
    }
    qos.best_effort();
  }

  if (durability_transient_local_endpoints_count == endpoints.size()) {
    // All publishers are transient local, so we can use the transient local QoS
    qos.transient_local();
  } else {
    if (durability_transient_local_endpoints_count > 0) {
      // There is a mix of QoS profiles amongst the publishers, so use the QoS setting that captures
      // all of them
      RCLCPP_WARN(
        get_logger(),
        "Some, but not all, publishers on topic \"%s\" are offering "
          "RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL. Falling back to "
          "RMW_QOS_POLICY_DURABILITY_VOLATILE as it will connect to all publishers. Previously-"
          "published latched messages will not be retrieved.",
        topic.c_str());
    }
    qos.durability_volatile();
  }

  return qos;
}

void SystemDataRecorder::unsubscribe_from_topics()
{
  // Make a note of the topics we are subscribed to
  std::vector<rosbag2_storage::TopicMetadata> topics;
  for (const auto & topic_with_type : topics_and_types_) {
    topics.push_back(rosbag2_storage::TopicMetadata(
      {
        topic_with_type.first,
        topic_with_type.second,
        rmw_get_serialization_format(),
        ""
      }));
  }
  // Unsubscribing happens automatically when the subscription objects are destroyed
  subscriptions_.clear();
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// File-copying thread
///////////////////////////////////////////////////////////////////////////////////////////////////

// The main function for the file-copying worker thread
void SystemDataRecorder::copy_thread_main()
{
  RCLCPP_INFO(get_logger(), "Copy thread: Starting");
  // Local state storage, to keep the critical section as short as possible
  SdrStateChange current_state = SdrStateChange::PAUSED;
  std::queue<std::string> local_files_to_copy;
  // Loop until receiving a state-change message indicating it is time to stop
  while (current_state != SdrStateChange::FINISHED)
  {
    { // Critical section start
      std::unique_lock<std::mutex> lock(copy_thread_mutex_);
      if (files_to_copy_.empty()) {
        // Only wait if there's nothing to copy, otherwise skip the wait and go on
        while (!copy_thread_should_wake()) {
          copy_thread_wake_cv_.wait(lock);
        }
      }

      // If the state has changed, make a note of the new state
      if (state_msg_ != SdrStateChange::NO_CHANGE) {
        current_state = state_msg_;
        // Reset the message-carrying variable so we don't get confused the next time around the
        // loop
        state_msg_ = SdrStateChange::NO_CHANGE;
      }

      // local_files_to_copy_ will be empty here, so we are swapping the received queue of files
      // and emptying the message-carrying queue at the same time
      local_files_to_copy.swap(files_to_copy_);
    } // Critical section end

    // Now copy any files that were sent from the node
    while(!local_files_to_copy.empty()) {
      std::string uri = local_files_to_copy.front();
      local_files_to_copy.pop();
      copy_bag_file(uri);
    }
  }
  RCLCPP_INFO(get_logger(), "Copy thread: Exiting");
}

// Only wake up if there are files to copy or a state-change message has been received
bool SystemDataRecorder::copy_thread_should_wake()
{
  return state_msg_ != SdrStateChange::NO_CHANGE || !files_to_copy_.empty();
}

// Notify the worker thread of a state change by sending it a message and triggering its condition
// variable
void SystemDataRecorder::notify_state_change(SdrStateChange new_state)
{
  { // Critical section start
    std::lock_guard<std::mutex> lock(copy_thread_mutex_);
    // new_state must not be NO_CHANGE or the copy thread won't wake up
    state_msg_ = new_state;
  } // Critical section end
  copy_thread_wake_cv_.notify_one();
}

// Notify the worker thread of a state change by adding the file to the queue and triggering its
// condition variable
void SystemDataRecorder::notify_new_file_to_copy(const std::string & file_uri)
{
  { // Critical section start
    std::lock_guard<std::mutex> lock(copy_thread_mutex_);
    files_to_copy_.push(file_uri);
  } // Critical section end
  copy_thread_wake_cv_.notify_one();
}

// Override that accepts a std::filesystem::path
void SystemDataRecorder::notify_new_file_to_copy(const std::filesystem::path & file_path)
{
  notify_new_file_to_copy(file_path.string());
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// File-copying functionality
///////////////////////////////////////////////////////////////////////////////////////////////////

// Create the destination directory to copy bag files to
// Returns true if the directory was created, false otherwise
bool SystemDataRecorder::create_copy_destination()
{
  if (std::filesystem::exists(destination_directory_)) {
    RCLCPP_ERROR(get_logger(), "Copy destination directory already exists");
    return false;
  }
  RCLCPP_INFO(get_logger(), "Creating destination directory %s", destination_directory_.c_str());
  return std::filesystem::create_directories(destination_directory_);
}

// Copy a bag file to the destination directory
void SystemDataRecorder::copy_bag_file(const std::string & bag_file_name)
{
  RCLCPP_INFO(
    get_logger(),
    "Copying %s to %s",
    bag_file_name.c_str(),
    destination_directory_.c_str());

  std::filesystem::copy(bag_file_name, destination_directory_);
}

}  // namespace sdr
