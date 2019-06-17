#ifndef NMEA_TO_GEOPOSE_NMEA_TO_GEOPOSE_H_INCLUDED
#define NMEA_TO_GEOPOSE_NMEA_TO_GEOPOSE_H_INCLUDED

// Headers in Boost
#include <boost/optional.hpp>
#include <boost/regex.hpp>

// Headers in ROS
#include <ros/ros.h>
#include <nmea_msgs/Sentence.h>

class NmeaToGeoPose
{
public:
    NmeaToGeoPose(ros::NodeHandle nh,ros::NodeHandle pnh);
    ~NmeaToGeoPose();
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    void nmeaSentenceCallback(const nmea_msgs::Sentence::ConstPtr msg);
    std::string calculateChecksum(std::string sentence);
    std::string getHexString(uint8_t value);
    ros::Subscriber nmea_sub_;
    std::string input_topic_;
    boost::optional<nmea_msgs::Sentence> gprmc_sentence_;
    boost::optional<nmea_msgs::Sentence> gphdt_sentence_;
    bool isGprmcSentence(nmea_msgs::Sentence sentence);
    bool isGphdtSentence(nmea_msgs::Sentence sentence);
};

#endif  //NMEA_TO_GEOPOSE_NMEA_TO_GEOPOSE_H_INCLUDED