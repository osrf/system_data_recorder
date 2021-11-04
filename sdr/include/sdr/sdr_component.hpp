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

#ifndef SDR__SDR_COMPONENT_HPP__
#define SDR__SDR_COMPONENT_HPP__

#include <atomic>
#include <queue>
#include <string>

#include "sdr/visibility_control.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_transport/recorder.hpp"

namespace sdr
{

class SystemDataRecorder : public rclcpp_lifecycle::LifecycleNode
{
public:
  SDR_PUBLIC
  SystemDataRecorder(
    const std::string & node_name,
    const rclcpp::NodeOptions & options,
    const std::string & bag_uri,
    const std::string & copy_destination_uri);

  SDR_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & state);

  SDR_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & state);

  SDR_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & state);

  SDR_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State & state);

  SDR_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State & state);

private:
  std::shared_ptr<rosbag2_cpp::Writer> writer_;

  std::mutex copy_thread_mutex_;
  std::condition_variable copy_thread_wake_cv_;
  std::shared_ptr<std::thread> copy_thread_;

  enum class SdrState
  {
    NO_CHANGE,
    PAUSED,
    RECORDING,
    FINISHED
  };

  SdrState state_msg_ = SdrState::NO_CHANGE;
  std::queue<std::string> files_to_copy_;

  void copy_thread_main();
  bool copy_thread_should_wake();
  void notify_state_change(SdrState new_state);
  void notify_new_file_to_copy(std::string & file_uri);
};

}  // namespace sdr

#endif  // SDR__SDR_COMPONENT_HPP__
