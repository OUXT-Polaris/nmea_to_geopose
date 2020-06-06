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

#ifndef NMEA_TO_GEOPOSE__NMEA_TO_GEOPOSE_COMPONENT_HPP_
#define NMEA_TO_GEOPOSE__NMEA_TO_GEOPOSE_COMPONENT_HPP_

// Headers in Boost
#include <boost/optional.hpp>

// Headers in ROS
#include <rclcpp/rclcpp.hpp>
#include <nmea_msgs/msg/sentence.hpp>
#include <geographic_msgs/msg/geo_pose.hpp>
#include <geographic_msgs/msg/geo_pose_stamped.hpp>
#include <geographic_msgs/msg/geo_point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <quaternion_operation/quaternion_operation.h>

// Headers in STL
#include <vector>
#include <string>

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define NMEA_TO_GEOPOSE_NMEA_TO_GEOPOSE_COMPONENT_EXPORT __attribute__((dllexport))
#define NMEA_TO_GEOPOSE_NMEA_TO_GEOPOSE_COMPONENT_IMPORT __attribute__((dllimport))
#else
#define NMEA_TO_GEOPOSE_NMEA_TO_GEOPOSE_COMPONENT_EXPORT __declspec(dllexport)
#define NMEA_TO_GEOPOSE_NMEA_TO_GEOPOSE_COMPONENT_IMPORT __declspec(dllimport)
#endif
#ifdef NMEA_TO_GEOPOSE_NMEA_TO_GEOPOSE_COMPONENT_BUILDING_DLL
#define NMEA_TO_GEOPOSE_NMEA_TO_GEOPOSE_COMPONENT_PUBLIC \
  NMEA_TO_GEOPOSE_NMEA_TO_GEOPOSE_COMPONENT_EXPORT
#else
#define NMEA_TO_GEOPOSE_NMEA_TO_GEOPOSE_COMPONENT_PUBLIC \
  NMEA_TO_GEOPOSE_NMEA_TO_GEOPOSE_COMPONENT_IMPORT
#endif
#define NMEA_TO_GEOPOSE_NMEA_TO_GEOPOSE_COMPONENT_PUBLIC_TYPE \
  NMEA_TO_GEOPOSE_NMEA_TO_GEOPOSE_COMPONENT_PUBLIC
#define NMEA_TO_GEOPOSE_NMEA_TO_GEOPOSE_COMPONENT_LOCAL
#else
#define NMEA_TO_GEOPOSE_NMEA_TO_GEOPOSE_COMPONENT_EXPORT \
  __attribute__((visibility("default")))
#define NMEA_TO_GEOPOSE_NMEA_TO_GEOPOSE_COMPONENT_IMPORT
#if __GNUC__ >= 4
#define NMEA_TO_GEOPOSE_NMEA_TO_GEOPOSE_COMPONENT_PUBLIC \
  __attribute__((visibility("default")))
#define NMEA_TO_GEOPOSE_NMEA_TO_GEOPOSE_COMPONENT_LOCAL \
  __attribute__((visibility("hidden")))
#else
#define NMEA_TO_GEOPOSE_NMEA_TO_GEOPOSE_COMPONENT_PUBLIC
#define NMEA_TO_GEOPOSE_NMEA_TO_GEOPOSE_COMPONENT_LOCAL
#endif
#define NMEA_TO_GEOPOSE_NMEA_TO_GEOPOSE_COMPONENT_PUBLIC_TYPE
#endif

#if __cplusplus
}  // extern "C"
#endif

namespace nmea_to_geopose
{
class NmeaToGeoPoseComponent : public rclcpp::Node
{
public:
  NMEA_TO_GEOPOSE_NMEA_TO_GEOPOSE_COMPONENT_PUBLIC
  explicit NmeaToGeoPoseComponent(const rclcpp::NodeOptions & options);
  ~NmeaToGeoPoseComponent();

private:
  void nmeaSentenceCallback(const nmea_msgs::msg::Sentence::SharedPtr msg);
  std::string calculateChecksum(std::string sentence);
  std::string getHexString(uint8_t value);
  rclcpp::Publisher<geographic_msgs::msg::GeoPoseStamped>::SharedPtr geopose_pub_;
  rclcpp::Subscription<nmea_msgs::msg::Sentence>::SharedPtr nmea_sub_;
  std::string input_topic_;
  boost::optional<geographic_msgs::msg::GeoPoint> geopoint_;
  boost::optional<geometry_msgs::msg::Quaternion> quat_;
  bool isGprmcSentence(nmea_msgs::msg::Sentence sentence);
  bool isGphdtSentence(nmea_msgs::msg::Sentence sentence);
  std::vector<std::string> split(const std::string & s, char delim);
  std::vector<std::string> splitChecksum(std::string str);
  boost::optional<std::vector<std::string>> splitSentence(nmea_msgs::msg::Sentence sentence);
  boost::optional<rclcpp::Time> last_timestamp_;
};
}  // namespace nmea_to_geopose

#endif  // NMEA_TO_GEOPOSE__NMEA_TO_GEOPOSE_COMPONENT_HPP_
