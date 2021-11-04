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

namespace sdr
{

SystemDataRecorder::SystemDataRecorder(
  const std::string & node_name,
  const rclcpp::NodeOptions & options,
  const std::string & bag_uri,
  const std::string & copy_destination_uri)
  : rclcpp_lifecycle::LifecycleNode(node_name, options)
{
  //rosbag2_storage::StorageOptions storage_options;
  //storage_options.uri = "test_bag";
  //storage_options.storage_id = "sqlite3";
  //storage_options.max_bagfile_size = 100000;

  //rosbag2_transport::RecordOptions record_options;
  //record_options.all = true;
  //record_options.rmw_serialization_format = std::string(rmw_get_serialization_format());

  //auto writer = std::make_shared<rosbag2_cpp::Writer>(
    //std::make_unique<rosbag2_cpp::writers::SequentialWriter>());

  //auto recorder = std::make_shared<rosbag2_transport::Recorder>(
    //writer, storage_options, record_options);

  //rosbag2_cpp::bag_events::WriterEventCallbacks callbacks;
  //callbacks.output_file_split_callback =
    //[&recorder, &m, &cv, &filename]
    //(rosbag2_cpp::bag_events::OutputFileSplitInfo & info) {
      //{
        //std::lock_guard<std::mutex> lk(m);
        //filename = info.closed_file;
      //}
      //cv.notify_one();
    //};
  //writer->add_event_callbacks(callbacks);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SystemDataRecorder::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Node: Preparing to begin recording");
  // Prepare the recorder object

  // Prepare the file-copying thread
  copy_thread_ = std::make_shared<std::thread>([this]{ copy_thread_main(); });

  // Notify the copy thread
  notify_state_change(SdrState::PAUSED);
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SystemDataRecorder::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Node: Starting recording");
  // Hit play on recorder
  //recorder->record();
  // Notify the copy thread
  notify_state_change(SdrState::RECORDING);
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SystemDataRecorder::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Node: Pausing recording");
  // Hit pause on recorder
  // Notify the copy thread
  notify_state_change(SdrState::PAUSED);
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SystemDataRecorder::on_cleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Node: Stopping and finalising recording");
  // Stop recording and copy the final file

  // Notify the copy thread
  notify_state_change(SdrState::FINISHED);
  copy_thread_->join();
  copy_thread_.reset();
  RCLCPP_INFO(get_logger(), "Node: Cleanup complete");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SystemDataRecorder::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Node: Stopping and finalising recording (hard shutdown)");
  // Stop recording and copy the final file

  // Notify the copy thread - but only if it hasn't already been cleaned up
  if (copy_thread_) {
    notify_state_change(SdrState::FINISHED);
    copy_thread_->join();
    copy_thread_.reset();
  }
  RCLCPP_INFO(get_logger(), "Node: Cleanup complete");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void SystemDataRecorder::copy_thread_main()
{
  RCLCPP_INFO(get_logger(), "Copy thread: Starting");
  SdrState current_state = SdrState::PAUSED;
  std::queue<std::string> local_files_to_copy;
  while (current_state != SdrState::FINISHED)
  {
    { // Critical section start
      RCLCPP_INFO(get_logger(), "Copy thread: Start of critical section");
      std::unique_lock<std::mutex> lock(copy_thread_mutex_);
      if (files_to_copy_.empty()) {
        // Only wait if there's nothing to copy
        RCLCPP_INFO(get_logger(), "Copy thread: Sleeping");
        while (!copy_thread_should_wake()) {
          copy_thread_wake_cv_.wait(lock);
        }
        RCLCPP_INFO(get_logger(), "Copy thread: Woken up");
      }

      if (state_msg_ != SdrState::NO_CHANGE) {
        switch(state_msg_) {
          case SdrState::NO_CHANGE:
            RCLCPP_INFO(get_logger(), "Copy thread: Received NO_CHANGE message");
            break;
          case SdrState::PAUSED:
            RCLCPP_INFO(get_logger(), "Copy thread: Received PAUSED message");
            break;
          case SdrState::RECORDING:
            RCLCPP_INFO(get_logger(), "Copy thread: Received RECORDING message");
            break;
          case SdrState::FINISHED:
          default:
            RCLCPP_INFO(get_logger(), "Copy thread: Received FINISHED message");
            break;
        }
        current_state = state_msg_;
        state_msg_ = SdrState::NO_CHANGE;
      }

      local_files_to_copy.swap(files_to_copy_);
      RCLCPP_INFO(get_logger(), "Copy thread: End of critical section");
    } // Critical section end

    while(!local_files_to_copy.empty()) {
      std::string uri = local_files_to_copy.front();
      local_files_to_copy.pop();
      RCLCPP_INFO(get_logger(), "Copy thread: Copying %s", uri.c_str());
    }
  }
  RCLCPP_INFO(get_logger(), "Copy thread: Exiting");
}

bool SystemDataRecorder::copy_thread_should_wake()
{
  return state_msg_ != SdrState::NO_CHANGE || !files_to_copy_.empty();
}

void SystemDataRecorder::notify_state_change(SdrState new_state)
{
  RCLCPP_INFO(get_logger(), "Node: Grabbing mutex");
  { // Critical section start
    std::lock_guard<std::mutex> lock(copy_thread_mutex_);
    // new_state must not be NO_CHANGE or the copy thread won't wake up
    state_msg_ = new_state;
  } // Critical section end
  RCLCPP_INFO(get_logger(), "Node: Waking copy thread");
  copy_thread_wake_cv_.notify_one();
}

void SystemDataRecorder::notify_new_file_to_copy(std::string & file_uri)
{
  RCLCPP_INFO(get_logger(), "Node: Grabbing mutex");
  { // Critical section start
    std::lock_guard<std::mutex> lock(copy_thread_mutex_);
    files_to_copy_.push(file_uri);
  } // Critical section end
  RCLCPP_INFO(get_logger(), "Node: Waking copy thread");
  copy_thread_wake_cv_.notify_one();
}

}  // namespace sdr
