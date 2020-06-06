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

#include <nmea_to_geopose/nmea_to_geopose_component.hpp>
#include <rclcpp_components/register_node_macro.hpp>

// Headers in STL
#include <vector>
#include <string>

namespace nmea_to_geopose
{
NmeaToGeoPoseComponent::NmeaToGeoPoseComponent(const rclcpp::NodeOptions & options)
: Node("nmea_to_geopose", options)
{
  declare_parameter("input_topic", "/nmea_sentence");
  get_parameter("input_topic", input_topic_);
  geopose_pub_ = this->create_publisher<geographic_msgs::msg::GeoPoseStamped>("geopose", 1);
  nmea_sub_ = this->create_subscription<nmea_msgs::msg::Sentence>(
    input_topic_, 10,
    std::bind(&NmeaToGeoPoseComponent::nmeaSentenceCallback, this, std::placeholders::_1));
}

NmeaToGeoPoseComponent::~NmeaToGeoPoseComponent()
{
}

void NmeaToGeoPoseComponent::nmeaSentenceCallback(const nmea_msgs::msg::Sentence::SharedPtr msg)
{
  nmea_msgs::msg::Sentence sentence = *msg;
  if (isGprmcSentence(sentence)) {
    geographic_msgs::msg::GeoPoint geopoint;
    boost::optional<std::vector<std::string>> data = splitSentence(sentence);
    if (data) {
      std::string lat_str = data.get()[3];
      std::string north_or_south_str = data.get()[4];
      double latitude = std::stod(lat_str.substr(0, 2)) + std::stod(lat_str.substr(2)) / 60.0;
      assert(north_or_south_str == "N" || north_or_south_str == "S");
      if (north_or_south_str == "S") {
        latitude = latitude * -1;
      }
      std::string lon_str = data.get()[5];
      std::string east_or_west_str = data.get()[6];
      double longitude = std::stod(lon_str.substr(0, 3)) + std::stod(lon_str.substr(3)) / 60.0;
      assert(east_or_west_str == "E" || east_or_west_str == "W");
      if (east_or_west_str == "W") {
        longitude = longitude * -1;
      }
      geopoint.latitude = latitude;
      geopoint.longitude = longitude;
      geopoint.altitude = 0.0;
      geopoint_ = geopoint;
    }
  }
  if (isGphdtSentence(sentence)) {
    boost::optional<std::vector<std::string>> data = splitSentence(sentence);
    if (data) {
      if (data.get()[2] == "T") {
        double heading = std::stod(data.get()[1]);
        geometry_msgs::msg::Vector3 vec;
        vec.x = 0.0;
        vec.y = 0.0;
        vec.z = heading / 180 * M_PI * -1;
        geometry_msgs::msg::Quaternion quat =
          quaternion_operation::convertEulerAngleToQuaternion(vec);
        quat_ = quat;
      }
    }
  }
  if (geopoint_ && quat_) {
    rclcpp::Time stamp = msg->header.stamp;
    if (!last_timestamp_ || last_timestamp_ != stamp) {
      last_timestamp_ = stamp;
      geographic_msgs::msg::GeoPoseStamped geopose;
      geopose.pose.position = geopoint_.get();
      geopose.pose.orientation = quat_.get();
      geopose.header = msg->header;
      geopose_pub_->publish(geopose);
    }
  }
}

std::vector<std::string> NmeaToGeoPoseComponent::split(const std::string & s, char delim)
{
  std::vector<std::string> elems;
  std::stringstream ss(s);
  std::string item;
  while (getline(ss, item, delim)) {
    if (!item.empty()) {
      elems.push_back(item);
    }
  }
  return elems;
}

std::vector<std::string> NmeaToGeoPoseComponent::splitChecksum(std::string str)
{
  return split(str, '*');
}

boost::optional<std::vector<std::string>> NmeaToGeoPoseComponent::splitSentence(
  nmea_msgs::msg::Sentence sentence)
{
  std::vector<std::string> data = splitChecksum(sentence.sentence);
  if (data.size() != 2) {
    return boost::none;
  }
  if (calculateChecksum(data[0]) == data[1]) {
    return split(data[0], ',');
  }
  return boost::none;
}

bool NmeaToGeoPoseComponent::isGprmcSentence(nmea_msgs::msg::Sentence sentence)
{
  std::string type = sentence.sentence.substr(0, 6);
  if (type == "$GPRMC") {
    return true;
  }
  return false;
}

bool NmeaToGeoPoseComponent::isGphdtSentence(nmea_msgs::msg::Sentence sentence)
{
  std::string type = sentence.sentence.substr(0, 6);
  if (type == "$GPHDT") {
    return true;
  }
  return false;
}

std::string NmeaToGeoPoseComponent::calculateChecksum(std::string sentence)
{
  uint8_t checksum;
  for (unsigned int i = 1; i < sentence.size(); i++) {
    int16_t c = sentence[i];
    checksum ^= c;
  }
  uint8_t rest = checksum % 16;
  uint8_t quotient = (checksum - rest) / 16;
  std::string ret = getHexString(quotient) + getHexString(rest);
  return ret;
}

std::string NmeaToGeoPoseComponent::getHexString(uint8_t value)
{
  assert(value <= 16);
  std::string ret;
  if (value == 10) {
    ret = "A";
  } else if (value == 11) {
    ret = "B";
  } else if (value == 12) {
    ret = "C";
  } else if (value == 13) {
    ret = "D";
  } else if (value == 14) {
    ret = "E";
  } else if (value == 15) {
    ret = "F";
  } else {
    ret = std::to_string(value);
  }
  return ret;
}
}  // namespace nmea_to_geopose

RCLCPP_COMPONENTS_REGISTER_NODE(nmea_to_geopose::NmeaToGeoPoseComponent)
