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

#include "rclcpp/rclcpp.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_transport/recorder.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = "test_bag";
  storage_options.storage_id = "sqlite3";
  storage_options.max_bagfile_size = 100000;

  rosbag2_transport::RecordOptions record_options;
  record_options.all = true;
  record_options.rmw_serialization_format = std::string(rmw_get_serialization_format());

  auto writer = std::make_shared<rosbag2_cpp::Writer>(
    std::make_unique<rosbag2_cpp::writers::SequentialWriter>());

  auto recorder = std::make_shared<rosbag2_transport::Recorder>(
    writer, storage_options, record_options);

  std::mutex m;
  std::condition_variable cv;
  std::string filename = "";
  std::atomic_bool finished(false);

  rosbag2_cpp::bag_events::WriterEventCallbacks callbacks;
  callbacks.output_file_split_callback =
    [&recorder, &m, &cv, &filename]
    (rosbag2_cpp::bag_events::OutputFileSplitInfo & info) {
      {
        std::lock_guard<std::mutex> lk(m);
        filename = info.closed_file;
      }
      cv.notify_one();
    };
  writer->add_event_callbacks(callbacks);

  recorder->record();

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(recorder);

  auto spin_thread = std::thread([&exec, &finished, &cv]() {
      exec.spin();
      finished.store(true);
      cv.notify_one();
    });

  while (rclcpp::ok())
  {
    {
      RCLCPP_INFO(recorder->get_logger(), "Acquiring lock");
      std::unique_lock<std::mutex> lk(m);
      RCLCPP_INFO(recorder->get_logger(), "Waiting");
      cv.wait(lk, [&filename, &finished]{return filename != "" || finished;});
      RCLCPP_INFO(recorder->get_logger(), "Copying file %s", filename.c_str());
      filename = "";
    }
    RCLCPP_INFO(recorder->get_logger(), "End of loop");
  }

  exec.cancel();
  RCLCPP_INFO(recorder->get_logger(), "Joining");
  spin_thread.join();
  RCLCPP_INFO(recorder->get_logger(), "Exiting");

  return 0;
}
