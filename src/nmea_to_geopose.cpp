#include <nmea_to_geopose/nmea_to_geopose.h>

NmeaToGeoPose::NmeaToGeoPose(ros::NodeHandle nh,ros::NodeHandle pnh)
{
    nh_ = nh;
    pnh_ = pnh;
    pnh_.param<std::string>("input_topic", input_topic_, "/nmea_sentence");
    nmea_sub_ = nh_.subscribe(input_topic_,1,&NmeaToGeoPose::nmeaSentenceCallback,this);
}

NmeaToGeoPose::~NmeaToGeoPose()
{

}

void NmeaToGeoPose::nmeaSentenceCallback(const nmea_msgs::Sentence::ConstPtr msg)
{
    if(isGprmcSentence(*msg))
    {
    }
    if(isGphdtSentence(*msg))
    {
        
    }
}

bool NmeaToGeoPose::isGprmcSentence(nmea_msgs::Sentence sentence)
{
    boost::regex regex("$GPRMC*");
    boost::match_results<const char*> results;
    bool found = boost::regex_search(sentence.sentence.c_str(), results, regex);
    return found;
}

bool NmeaToGeoPose::isGphdtSentence(nmea_msgs::Sentence sentence)
{
    boost::regex regex("$GPHDT*");
    boost::match_results<const char*> results;
    bool found = boost::regex_search(sentence.sentence.c_str(), results, regex);
    return found;
}

std::string NmeaToGeoPose::calculateChecksum(std::string sentence)
{
    uint8_t checksum;
    for(int i=1; i<sentence.size(); i++)
    {
        int c = sentence[i];
        checksum ^= c;
    }
    uint8_t rest = checksum%16;
    uint8_t quotient = (checksum-rest)/16;
    std::string ret = getHexString(quotient) + getHexString(rest);
    ret = "*" + ret;
    return ret;
}

std::string NmeaToGeoPose::getHexString(uint8_t value)
{
    ROS_ASSERT(value <= 16);
    std::string ret;
    if(value == 10)
    {
        ret = "A";
    }
    else if(value == 11)
    {
        ret = "B";
    }
    else if(value == 12)
    {
        ret = "C";
    }
    else if(value == 13)
    {
        ret = "D";
    }
    else if(value == 14)
    {
        ret = "E";
    }
    else if(value == 15)
    {
        ret = "F";
    }
    else
    {
        ret = std::to_string(value);
    }
    return ret;
}