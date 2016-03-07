//PUT_VREP_ROSPLUGIN_COPYRIGHT_NOTICE_HERE

#ifndef V_REPEXTROS_H
#define V_REPEXTROS_H

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <map>

#include <boost/lexical_cast.hpp>

#include <ros/ros.h>

#define VREP_DLLEXPORT extern "C"

// The 3 required entry points of the V-REP plugin:
VREP_DLLEXPORT unsigned char v_repStart(void* reservedPointer,int reservedInt);
VREP_DLLEXPORT void v_repEnd();
VREP_DLLEXPORT void* v_repMessage(int message,int* auxiliaryData,void* customData,int* replyData);

struct ScriptCallback
{
    int scriptId;
    std::string name;
};

struct SubscriberProxy
{
    int handle;
    std::string topicName;
    std::string topicType;
    ScriptCallback topicCallback;
    ros::Subscriber subscriber;
};

struct PublisherProxy
{
    int handle;
    std::string topicName;
    std::string topicType;
    ros::Publisher publisher;
};

#include <stubs.h>
#include <cpp_io.h>
#include <ros_msg_builtin_io.h>
#include <ros_msg_io.h>

#include <v_repLib.h>

#endif
