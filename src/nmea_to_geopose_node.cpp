#include <nmea_to_geopose/nmea_to_geopose.h>

#include <ros/ros.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "nmea_to_geopose_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    NmeaToGeoPose converter(nh,pnh);
    ros::spin();
    return 0;
}