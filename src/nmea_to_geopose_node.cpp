// Copyright (c) 2019 OUXT Polaris
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

// Headers in this package
#include <nmea_to_geopose/nmea_to_geopose_component.hpp>

// Headers in ROS
#include <rclcpp/rclcpp.hpp>

// Headers in Glog
#include <glog/logging.h>

// Headers in STL
#include <memory>

int main(int argc, char * argv[])
{
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<nmea_to_geopose::NmeaToGeoPoseComponent>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}
