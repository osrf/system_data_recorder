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

#include <condition_variable>
#include <mutex>
#include <thread>

#include "sdr/sdr_component.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_transport/recorder.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto topics_and_types = std::unordered_map<std::string, std::string>();
  topics_and_types.insert({"chatter", "std_msgs/msg/String"});
  auto sdr_component = std::make_shared<sdr::SystemDataRecorder>(
    "sdr",
    rclcpp::NodeOptions(),
    "test_bag",
    "copied_bag",
    100000,
    topics_and_types);

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(sdr_component->get_node_base_interface());
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
