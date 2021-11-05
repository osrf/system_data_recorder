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

#include "rosbag2_storage/topic_metadata.hpp"

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
  storage_options_.uri = bag_uri;
  storage_options_.storage_id = "sqlite3";
  storage_options_.max_bagfile_size = max_file_size;

  auto bag_directory_name = std::filesystem::path(bag_uri).filename();
  if (bag_directory_name.empty()) {
    // There was probably a slash on the end
    std::string bag_uri_copy = bag_uri;
    bag_directory_name = std::filesystem::path(
      bag_uri_copy.erase(bag_uri.size() - 1, 1)).filename();
  }
  destination_directory_ = std::filesystem::path(copy_destination) / bag_directory_name;

  last_bag_file_ = bag_directory_name.concat("_0.db3");
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// Lifecycle states
///////////////////////////////////////////////////////////////////////////////////////////////////

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SystemDataRecorder::on_configure(const rclcpp_lifecycle::State & /* state */)
{
  RCLCPP_INFO(get_logger(), "Preparing to begin recording");

  RCLCPP_INFO(get_logger(), "Copying bag files to %s", destination_directory_.c_str());

  // Prepare for file-copying
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

  // Notify the copy thread
  notify_state_change(SdrStateChange::PAUSED);

  // Prepare the objects for recording
  writer_ = std::make_shared<rosbag2_cpp::Writer>(
    std::make_unique<rosbag2_cpp::writers::SequentialWriter>());
  // Add a callback for when a split occurs
  rosbag2_cpp::bag_events::WriterEventCallbacks callbacks;
  callbacks.output_file_split_callback =
    [this]
    (rosbag2_cpp::bag_events::OutputFileSplitInfo & info) {
      // Record the opened file - this will be the last file remaining to be copied when recording
      // is terminated
      last_bag_file_ = info.opened_file;
      // Copy the closed file
      notify_new_file_to_copy(info.closed_file);
    };
  writer_->add_event_callbacks(callbacks);
  // Open the bag
  writer_->open(
    storage_options_,
    {rmw_get_serialization_format(), rmw_get_serialization_format()});

  // Start receiving data - it won't be recorded yet
  subscribe_to_topics();

  cleaned_up = false;
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SystemDataRecorder::on_activate(const rclcpp_lifecycle::State & /* state */)
{
  RCLCPP_INFO(get_logger(), "Starting recording");
  // Hit play on recorder
  is_paused = false;

  // Notify the copy thRead
  notify_state_change(SdrStateChange::RECORDING);
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SystemDataRecorder::on_deactivate(const rclcpp_lifecycle::State & /* state */)
{
  RCLCPP_INFO(get_logger(), "Pausing recording");
  // Hit pause on recorder
  is_paused = true;

  // Notify the copy thread
  notify_state_change(SdrStateChange::PAUSED);
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SystemDataRecorder::on_cleanup(const rclcpp_lifecycle::State & /* state */)
{
  RCLCPP_INFO(get_logger(), "Stopping and finalising recording");
  cleaned_up = true;

  // Stop recording
  unsubscribe_from_topics();
  writer_.reset();
  // Copy the final file
  notify_new_file_to_copy(last_bag_file_);
  // Copy the metadata file
  notify_new_file_to_copy(source_directory_ / "metadata.yaml");

  // Notify the copy thread
  notify_state_change(SdrStateChange::FINISHED);
  copy_thread_->join();
  copy_thread_.reset();

  RCLCPP_INFO(get_logger(), "Cleanup complete");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SystemDataRecorder::on_shutdown(const rclcpp_lifecycle::State & /* state */)
{
  RCLCPP_INFO(get_logger(), "Stopping and finalising recording (hard shutdown)");
  if (!cleaned_up) {
    // Stop recording
    unsubscribe_from_topics();
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
  auto topic_metadata = rosbag2_storage::TopicMetadata(
    {
      topic,
      type,
      rmw_get_serialization_format(),
      ""
    }
  );
  // It is a good idea to create the topic in the writer prior to adding the subscription in case
  // data arrives after subscribing and before the topic is created. Although we should be ignoring
  // any data until the node is set to active, we maintain this good practice here for future
  // maintainability.
  writer_->create_topic(topic_metadata);

  auto qos = rclcpp::QoS(1);
  auto subscription = create_generic_subscription(
    topic,
    type,
    qos,
    [this, topic, type](std::shared_ptr<rclcpp::SerializedMessage> message) {
      // When a message is received, it should only be written to the bag if recording is not
      // paused (i.e. the node lifecycle state is "active"). If recording is paused, the message is
      // thrown away.
      if (!is_paused) {
        writer_->write(*message, topic, type, rclcpp::Clock(RCL_SYSTEM_TIME).now());
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

void SystemDataRecorder::unsubscribe_from_topics()
{
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
  // Now that there is no chance of new messages arriving, it is safe to remove the topics from the
  // writer
  for (const auto & topic : topics) {
    writer_->remove_topic(topic);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// File-copying thread
///////////////////////////////////////////////////////////////////////////////////////////////////

void SystemDataRecorder::copy_thread_main()
{
  RCLCPP_INFO(get_logger(), "Copy thread: Starting");
  SdrStateChange current_state = SdrStateChange::PAUSED;
  std::queue<std::string> local_files_to_copy;
  while (current_state != SdrStateChange::FINISHED)
  {
    { // Critical section start
      std::unique_lock<std::mutex> lock(copy_thread_mutex_);
      if (files_to_copy_.empty()) {
        // Only wait if there's nothing to copy
        while (!copy_thread_should_wake()) {
          copy_thread_wake_cv_.wait(lock);
        }
      }

      if (state_msg_ != SdrStateChange::NO_CHANGE) {
        current_state = state_msg_;
        state_msg_ = SdrStateChange::NO_CHANGE;
      }

      local_files_to_copy.swap(files_to_copy_);
    } // Critical section end

    while(!local_files_to_copy.empty()) {
      std::string uri = local_files_to_copy.front();
      local_files_to_copy.pop();
      copy_bag_file(uri);
    }
  }
  RCLCPP_INFO(get_logger(), "Copy thread: Exiting");
}

bool SystemDataRecorder::copy_thread_should_wake()
{
  return state_msg_ != SdrStateChange::NO_CHANGE || !files_to_copy_.empty();
}

void SystemDataRecorder::notify_state_change(SdrStateChange new_state)
{
  { // Critical section start
    std::lock_guard<std::mutex> lock(copy_thread_mutex_);
    // new_state must not be NO_CHANGE or the copy thread won't wake up
    state_msg_ = new_state;
  } // Critical section end
  copy_thread_wake_cv_.notify_one();
}

void SystemDataRecorder::notify_new_file_to_copy(const std::string & file_uri)
{
  { // Critical section start
    std::lock_guard<std::mutex> lock(copy_thread_mutex_);
    files_to_copy_.push(file_uri);
  } // Critical section end
  copy_thread_wake_cv_.notify_one();
}

void SystemDataRecorder::notify_new_file_to_copy(const std::filesystem::path & file_path)
{
  notify_new_file_to_copy(file_path.string());
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// File-copying functionality
///////////////////////////////////////////////////////////////////////////////////////////////////

bool SystemDataRecorder::create_copy_destination()
{
  if (std::filesystem::exists(destination_directory_)) {
    RCLCPP_ERROR(get_logger(), "Copy destination directory already exists");
    return false;
  }
  RCLCPP_INFO(get_logger(), "Creating destination directory %s", destination_directory_.c_str());
  return std::filesystem::create_directories(destination_directory_);
}

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
