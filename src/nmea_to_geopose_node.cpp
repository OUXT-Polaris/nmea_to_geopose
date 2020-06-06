// Headers in this package
#include <nmea_to_geopose/nmea_to_geopose_component.hpp>

// Headers in ROS
#include <rclcpp/rclcpp.hpp>

// Headers in Glog
#include <glog/logging.h>

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
