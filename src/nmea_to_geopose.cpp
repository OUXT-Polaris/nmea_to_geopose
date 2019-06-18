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
    nmea_msgs::Sentence sentence = *msg;
    if(isGprmcSentence(sentence))
    {
        boost::optional<std::vector<std::string> > data = splitSentence(sentence);
        if(data)
        {
            data.get()[0];
        }
    }
    if(isGphdtSentence(sentence))
    {
        boost::optional<std::vector<std::string> > data = splitSentence(sentence);
        if(data)
        {

        }
    }
}

std::vector<std::string> NmeaToGeoPose::split(const std::string &s,char delim)
{
    std::vector<std::string> elems;
    std::stringstream ss(s);
    std::string item;
    while (getline(ss, item, delim))
    {
        if (!item.empty())
        {
            elems.push_back(item);
        }
    }
    return elems;
}

std::vector<std::string> NmeaToGeoPose::splitChecksum(std::string str)
{
    return split(str,'*');
}

boost::optional<std::vector<std::string> > NmeaToGeoPose::splitSentence(nmea_msgs::Sentence sentence)
{
    std::vector<std::string> data = splitChecksum(sentence.sentence);
    if(data.size() != 2)
    {
        return boost::none;
    }
    if(calculateChecksum(data[0]) == data[1])
    {
        return split(data[0],',');
    }
    return boost::none;
}

bool NmeaToGeoPose::isGprmcSentence(nmea_msgs::Sentence sentence)
{
    std::string type = sentence.sentence.substr(0,6);
    if(type == "$GPRMC")
    {
        return true;
    }
    return false;
}

bool NmeaToGeoPose::isGphdtSentence(nmea_msgs::Sentence sentence)
{
    std::string type = sentence.sentence.substr(0,6);
    if(type == "$GPHDT")
    {
        return true;
    }
    return false;
}

std::string NmeaToGeoPose::calculateChecksum(std::string sentence)
{
    uint8_t checksum;
    for(int i=1; i<sentence.size(); i++)
    {
        int16_t c = sentence[i];
        checksum ^= c;
    }
    uint8_t rest = checksum%16;
    uint8_t quotient = (checksum-rest)/16;
    std::string ret = getHexString(quotient) + getHexString(rest);
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