#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <map>

#include <boost/lexical_cast.hpp>

#include <ros/ros.h>

#include <std_msgs/Float32.h>

#include "../include/v_repLib.h"
#include "../include/scriptFunctionData.h"
#include "../include/vrep_ros_plugin/vrep_ros_plugin.h"

#define PLUGIN_VERSION 5 // 5 since 3.3.1 (using stacks to exchange data with scripts)

#define CONCAT(x,y,z) x y z
#define strConCat(x,y,z)	CONCAT(x,y,z)

LIBRARY vrepLib; // the V-REP library that we will dynamically load and bind

ros::NodeHandle *nh = NULL;

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

int subscriberProxyNextHandle = 3562;
int publisherProxyNextHandle = 7980;

std::map<int, SubscriberProxy *> subscriberProxies;
std::map<int, PublisherProxy *> publisherProxies;

bool read__bool(int stack, uint8_t *value)
{
    simBool v;
    if(simGetStackBoolValue(stack, &v) == 1)
    {
        *value = v;
        simPopStackItem(stack, 1);
        return true;
    }
    else
    {
        std::cerr << "read__bool: error: expected bool value." << std::endl;
        return false;
    }
}

bool read__int8(int stack, int8_t *value)
{
    simInt v;
    if(simGetStackInt32Value(stack, &v) == 1)
    {
        *value = v;
        simPopStackItem(stack, 1);
        return true;
    }
    else
    {
        std::cerr << "read__int8: error: expected integer value." << std::endl;
        return false;
    }
}

bool read__uint8(int stack, uint8_t *value)
{
    simInt v;
    if(simGetStackInt32Value(stack, &v) == 1)
    {
        *value = v;
        simPopStackItem(stack, 1);
        return true;
    }
    else
    {
        std::cerr << "read__uint8: error: expected integer value." << std::endl;
        return false;
    }
}

bool read__int16(int stack, int16_t *value)
{
    simInt v;
    if(simGetStackInt32Value(stack, &v) == 1)
    {
        *value = v;
        simPopStackItem(stack, 1);
        return true;
    }
    else
    {
        std::cerr << "read__int16: error: expected integer value." << std::endl;
        return false;
    }
}

bool read__uint16(int stack, uint16_t *value)
{
    simInt v;
    if(simGetStackInt32Value(stack, &v) == 1)
    {
        *value = v;
        simPopStackItem(stack, 1);
        return true;
    }
    else
    {
        std::cerr << "read__uint16: error: expected integer value." << std::endl;
        return false;
    }
}

bool read__int32(int stack, int32_t *value)
{
    simInt v;
    if(simGetStackInt32Value(stack, &v) == 1)
    {
        *value = v;
        simPopStackItem(stack, 1);
        return true;
    }
    else
    {
        std::cerr << "read__int32: error: expected integer value." << std::endl;
        return false;
    }
}

bool read__uint32(int stack, uint32_t *value)
{
    simInt v;
    if(simGetStackInt32Value(stack, &v) == 1)
    {
        *value = v;
        simPopStackItem(stack, 1);
        return true;
    }
    else
    {
        std::cerr << "read__uint32: error: expected integer value." << std::endl;
        return false;
    }
}

bool read__int64(int stack, int64_t *value)
{
    // XXX: we represent Int64 as double - possible loss of precision!
    simDouble v;
    if(simGetStackDoubleValue(stack, &v) == 1)
    {
        *value = v;
        simPopStackItem(stack, 1);
        return true;
    }
    else
    {
        std::cerr << "read__int64: error: expected integer value." << std::endl;
        return false;
    }
}

bool read__uint64(int stack, uint64_t *value)
{
    // XXX: we represent UInt64 as double - possible loss of precision!
    simDouble v;
    if(simGetStackDoubleValue(stack, &v) == 1)
    {
        *value = v;
        simPopStackItem(stack, 1);
        return true;
    }
    else
    {
        std::cerr << "read__uint64: error: expected integer value." << std::endl;
        return false;
    }
}

bool read__float32(int stack, float *value)
{
    simFloat v;
    if(simGetStackFloatValue(stack, &v) == 1)
    {
        *value = v;
        simPopStackItem(stack, 1);
        return true;
    }
    else
    {
        std::cerr << "read__float32: error: expected float value." << std::endl;
        return false;
    }
}

bool read__float64(int stack, float *value)
{
    simDouble v;
    if(simGetStackDoubleValue(stack, &v) == 1)
    {
        *value = v;
        simPopStackItem(stack, 1);
        return true;
    }
    else
    {
        std::cerr << "read__float64: error: expected float value." << std::endl;
        return false;
    }
}

bool read__string(int stack, std::string *value)
{
    simChar *str;
    simInt strSize;
    if((str = simGetStackStringValue(stack, &strSize)) != NULL && strSize > 0)
    {
        *value = std::string(str);
        simPopStackItem(stack, 1);
        return true;
    }
    else
    {
        std::cerr << "read__string: error: expected string value." << std::endl;
        return false;
    }
}

bool read__time(int stack, ros::Time *value)
{
    simDouble v;
    if(simGetStackDoubleValue(stack, &v) == 1)
    {
        *value = ros::Time(v);
        simPopStackItem(stack, 1);
        return true;
    }
    else
    {
        std::cerr << "read__time: error: expected float value." << std::endl;
        return false;
    }
}

bool read__duration(int stack, ros::Duration *value)
{
    simDouble v;
    if(simGetStackDoubleValue(stack, &v) == 1)
    {
        *value = ros::Duration(v);
        simPopStackItem(stack, 1);
        return true;
    }
    else
    {
        std::cerr << "read__duration: error: expected float value." << std::endl;
        return false;
    }
}

bool read__std_msgs__Float32(int stack, std_msgs::Float32 *msg)
{
    int i;
    if((i = simGetStackTableInfo(stack, 0)) != sim_stack_table_map)
    {
        std::cerr << "read__std_msgs__Float32: error: expected a table (simGetStackTableInfo returned " << i << ")." << std::endl;
        return false;
    }

    int sz = simGetStackSize(stack);
    simUnfoldStackTable(stack);
    int numItems = (simGetStackSize(stack) - sz + 1) / 2;

    char *str;
    int strSz;

    while(numItems >= 1)
    {
        simMoveStackItemToTop(stack, simGetStackSize(stack) - 2); // move key to top
        if((str = simGetStackStringValue(stack, &strSz)) != NULL && strSz > 0)
        {
            simPopStackItem(stack, 1); // now stack top is value

            if(strcmp(str, "data") == 0)
            {
                if(!read__float32(stack, &(msg->data)))
                {
                    std::cerr << "read__std_msgs__Float32: error: value is not float for key: " << str << "." << std::endl;
                    return false;
                }
            }
            else
            {
                std::cerr << "read__std_msgs__Float32: error: unexpected key: " << str << "." << std::endl;
                return false;
            }

            simReleaseBuffer(str);
        }
        else
        {
            std::cerr << "read__std_msgs__Float32: error: malformed table (bad key type)." << std::endl;
            return false;
        }

        numItems = (simGetStackSize(stack) - sz + 1) / 2;
    }
    
    return true;
}

bool write__bool(uint8_t value, int stack)
{
    simBool v = value;
    if(simPushBoolOntoStack(stack, v) == -1)
    {
        std::cerr << "write__bool: error: push table value (data) failed." << std::endl;
        return false;
    }
    else
    {
        return true;
    }
}

bool write__int8(int8_t value, int stack)
{
    simInt v = value;
    if(simPushInt32OntoStack(stack, v) == -1)
    {
        std::cerr << "write__int8: error: push table value (data) failed." << std::endl;
        return false;
    }
    else
    {
        return true;
    }
}

bool write__uint8(uint8_t value, int stack)
{
    simInt v = value;
    if(simPushInt32OntoStack(stack, v) == -1)
    {
        std::cerr << "write__uint8: error: push table value (data) failed." << std::endl;
        return false;
    }
    else
    {
        return true;
    }
}

bool write__int16(int16_t value, int stack)
{
    simInt v = value;
    if(simPushInt32OntoStack(stack, v) == -1)
    {
        std::cerr << "write__int16: error: push table value (data) failed." << std::endl;
        return false;
    }
    else
    {
        return true;
    }
}

bool write__uint16(uint16_t value, int stack)
{
    simInt v = value;
    if(simPushInt32OntoStack(stack, v) == -1)
    {
        std::cerr << "write__uint16: error: push table value (data) failed." << std::endl;
        return false;
    }
    else
    {
        return true;
    }
}

bool write__int32(int32_t value, int stack)
{
    simInt v = value;
    if(simPushInt32OntoStack(stack, v) == -1)
    {
        std::cerr << "write__int32: error: push table value (data) failed." << std::endl;
        return false;
    }
    else
    {
        return true;
    }
}

bool write__uint32(uint32_t value, int stack)
{
    simInt v = value;
    if(simPushInt32OntoStack(stack, v) == -1)
    {
        std::cerr << "write__uint32: error: push table value (data) failed." << std::endl;
        return false;
    }
    else
    {
        return true;
    }
}

bool write__int64(int64_t value, int stack)
{
    // XXX: we represent Int64 as double - possible loss of precision!
    simDouble v = value;
    if(simPushDoubleOntoStack(stack, v) == -1)
    {
        std::cerr << "write__int64: error: push table value (data) failed." << std::endl;
        return false;
    }
    else
    {
        return true;
    }
}

bool write__uint64(uint64_t value, int stack)
{
    // XXX: we represent UInt64 as double - possible loss of precision!
    simDouble v = value;
    if(simPushDoubleOntoStack(stack, v) == -1)
    {
        std::cerr << "write__uint64: error: push table value (data) failed." << std::endl;
        return false;
    }
    else
    {
        return true;
    }
}

bool write__float32(float value, int stack)
{
    simFloat v = value;
    if(simPushFloatOntoStack(stack, v) == -1)
    {
        std::cerr << "write__float32: error: push table value (data) failed." << std::endl;
        return false;
    }
    else
    {
        return true;
    }
}

bool write__float64(double value, int stack)
{
    simDouble v = value;
    if(simPushDoubleOntoStack(stack, v) == -1)
    {
        std::cerr << "write__float64: error: push table value (data) failed." << std::endl;
        return false;
    }
    else
    {
        return true;
    }
}

bool write__time(ros::Time value, int stack)
{
    simDouble v = value.toSec();
    if(simPushDoubleOntoStack(stack, v) == -1)
    {
        std::cerr << "write__time: error: push table value (data) failed." << std::endl;
        return false;
    }
    else
    {
        return true;
    }
}

bool write__duration(ros::Duration value, int stack)
{
    simDouble v = value.toSec();
    if(simPushDoubleOntoStack(stack, v) == -1)
    {
        std::cerr << "write__duration: error: push table value (data) failed." << std::endl;
        return false;
    }
    else
    {
        return true;
    }
}

bool write__std_msgs__Float32(const std_msgs::Float32::ConstPtr& msg, int stack)
{
    if(simPushTableOntoStack(stack) == -1)
    {
        std::cerr << "write__sta_msgs__Float32: error: push table failed." << std::endl;
        return false;
    }
    if(simPushStringOntoStack(stack, "data", 0) == -1)
    {
        std::cerr << "write__sta_msgs__Float32: error: push table key (data) failed." << std::endl;
        return false;
    }
    if(!write__float32(msg->data, stack))
    {
        std::cerr << "write__sta_msgs__Float32: error: push table value (data) failed." << std::endl;
        return false;
    }
    if(simInsertDataIntoStackTable(stack) == -1)
    {
        std::cerr << "write__sta_msgs__Float32: error: insert table failed." << std::endl;
        return false;
    }
    return true;
}

void ros_callback__std_msgs__Float32(const std_msgs::Float32::ConstPtr& msg, SubscriberProxy *proxy)
{
    int stack = simCreateStack();
    if(stack != -1)
    {
        do
        {
            if(!write__std_msgs__Float32(msg, stack))
            {
                break;
            }
            if(simCallScriptFunctionEx(proxy->topicCallback.scriptId, proxy->topicCallback.name.c_str(), stack) == -1)
            {
                std::cerr << "ros_callback__std_msgs__Float32: error: call script failed." << std::endl;
                break;
            }
        }
        while(0);
        simReleaseStack(stack);
    }
}

void simExtROS_subscribe(SScriptCallBack *p)
{
    // input arguments:

    std::string topicName;
    std::string topicType;
    std::string topicCallback;
    simInt queueSize = 0;

    // read input argument count from stack:

    int numArgs = simGetStackSize(p->stackID);
    int totalArgs = 4;
    int requiredArgs = 3;

    if(numArgs < requiredArgs || numArgs > totalArgs)
    {
        simSetLastError("simExtROS_subscribe", "wrong number of arguments");
        return;
    }

    // read input argument values from stack:

    if(numArgs >= 1)
    {
        simMoveStackItemToTop(p->stackID, 0);
        int strSize;
        simChar *str = simGetStackStringValue(p->stackID, &strSize);
        if(strSize == -1)
        {
            simSetLastError("simExtROS_subscribe", "internal error reading argument 1");
            return;
        }
        if(str == NULL && strSize == 0)
        {
            simSetLastError("simExtROS_subscribe", "type error: argument 1 must be a string");
            return;
        }
        topicName = std::string(str);
        simReleaseBuffer(str);
    }

    if(numArgs >= 2)
    {
        simMoveStackItemToTop(p->stackID, 0);
        int strSize;
        simChar *str = simGetStackStringValue(p->stackID, &strSize);
        if(strSize == -1)
        {
            simSetLastError("simExtROS_subscribe", "internal error reading argument 2");
            return;
        }
        if(str == NULL && strSize == 0)
        {
            simSetLastError("simExtROS_subscribe", "type error: argument 2 must be a string");
            return;
        }
        topicType = std::string(str);
        simReleaseBuffer(str);
    }

    if(numArgs >= 3)
    {
        simMoveStackItemToTop(p->stackID, 0);
        int strSize;
        simChar *str = simGetStackStringValue(p->stackID, &strSize);
        if(strSize == -1)
        {
            simSetLastError("simExtROS_subscribe", "internal error reading argument 3");
            return;
        }
        if(str == NULL && strSize == 0)
        {
            simSetLastError("simExtROS_subscribe", "type error: argument 3 must be a string");
            return;
        }
        topicCallback = std::string(str);
        simReleaseBuffer(str);
    }

    if(numArgs >= 4)
    {
        simMoveStackItemToTop(p->stackID, 0);
        simInt ret = simGetStackInt32Value(p->stackID, &queueSize);
        if(ret == -1)
        {
            simSetLastError("simExtROS_subscribe", "internal error reading argument 4");
            return;
        }
        if(ret == 0)
        {
            simSetLastError("simExtROS_subscribe", "type error: argument 4 must be a number");
            return;
        }
    }

    SubscriberProxy *subscriberProxy = new SubscriberProxy();
    subscriberProxy->handle = subscriberProxyNextHandle++;
    subscriberProxy->topicName = topicName;
    subscriberProxy->topicType = topicType;
    subscriberProxy->topicCallback.scriptId = p->scriptID;
    subscriberProxy->topicCallback.name = topicCallback;
    subscriberProxies[subscriberProxy->handle] = subscriberProxy;

    if(topicType == "std_msgs/Float32")
    {
        subscriberProxy->subscriber = nh->subscribe<std_msgs::Float32>(topicName, queueSize, boost::bind(ros_callback__std_msgs__Float32, _1, subscriberProxy));
    }
    else
    {
        simSetLastError("simExtROS_subscribe", "unsupported message type. please edit and recompile ROS plugin");
        return;
    }

    if(!subscriberProxy->subscriber)
    {
        simSetLastError("simExtROS_subscribe", "failed creation of ROS subscriber");
        return;
    }

    // write output arguments to stack:

    simPopStackItem(p->stackID, 0);

    {
        int ret = simPushInt32OntoStack(p->stackID, subscriberProxy->handle);
        if(ret == -1)
        {
            simSetLastError("simExtROS_subscribe", "internal error writing output argument 1");
            return;
        }
    }
}

void simExtROS_shutdownSubscriber(SScriptCallBack *p)
{
    // input arguments:

    simInt subscriberHandle;

    // read input argument count from stack:

    int numArgs = simGetStackSize(p->stackID);
    int totalArgs = 1;
    int requiredArgs = 1;

    if(numArgs < requiredArgs || numArgs > totalArgs)
    {
        simSetLastError("simExtROS_shutdownSubscriber", "wrong number of arguments");
        return;
    }

    // read input argument values from stack:

    if(numArgs >= 1)
    {
        simMoveStackItemToTop(p->stackID, 0);
        simInt ret = simGetStackInt32Value(p->stackID, &subscriberHandle);
        if(ret == -1)
        {
            simSetLastError("simExtROS_shutdownSubscriber", "internal error reading argument 1");
            return;
        }
        if(ret == 0)
        {
            simSetLastError("simExtROS_shutdownSubscriber", "type error: argument 1 must be a number");
            return;
        }
    }

    if(subscriberProxies.find(subscriberHandle) == subscriberProxies.end())
    {
        simSetLastError("simExtROS_shutdownSubscriber", "invalid subscriber handle");
        return;
    }

    SubscriberProxy *subscriberProxy = subscriberProxies[subscriberHandle];
    subscriberProxy->subscriber.shutdown();
    subscriberProxies.erase(subscriberProxy->handle);
    delete subscriberProxy;

    // write output arguments to stack:

    {
    }
}

void simExtROS_advertise(SScriptCallBack *p)
{
    // input arguments:

    std::string topicName;
    std::string topicType;
    simInt queueSize = 0;
    simBool latch = false;

    // read input argument count from stack:

    int numArgs = simGetStackSize(p->stackID);
    int totalArgs = 4;
    int requiredArgs = 2;

    if(numArgs < requiredArgs || numArgs > totalArgs)
    {
        simSetLastError("simExtROS_advertise", "wrong number of arguments");
        return;
    }

    // read input argument values from stack:

    if(numArgs >= 1)
    {
        simMoveStackItemToTop(p->stackID, 0);
        int strSize;
        simChar *str = simGetStackStringValue(p->stackID, &strSize);
        if(strSize == -1)
        {
            simSetLastError("simExtROS_advertise", "internal error reading argument 1");
            return;
        }
        if(str == NULL && strSize == 0)
        {
            simSetLastError("simExtROS_advertise", "type error: argument 1 must be a string");
            return;
        }
        topicName = std::string(str);
        simReleaseBuffer(str);
    }

    if(numArgs >= 2)
    {
        simMoveStackItemToTop(p->stackID, 0);
        int strSize;
        simChar *str = simGetStackStringValue(p->stackID, &strSize);
        if(strSize == -1)
        {
            simSetLastError("simExtROS_advertise", "internal error reading argument 2");
            return;
        }
        if(str == NULL && strSize == 0)
        {
            simSetLastError("simExtROS_advertise", "type error: argument 2 must be a string");
            return;
        }
        topicType = std::string(str);
        simReleaseBuffer(str);
    }

    if(numArgs >= 3)
    {
        simMoveStackItemToTop(p->stackID, 0);
        simInt ret = simGetStackInt32Value(p->stackID, &queueSize);
        if(ret == -1)
        {
            simSetLastError("simExtROS_advertise", "internal error reading argument 3");
            return;
        }
        if(ret == 0)
        {
            simSetLastError("simExtROS_advertise", "type error: argument 3 must be a number");
            return;
        }
    }

    if(numArgs >= 4)
    {
        simMoveStackItemToTop(p->stackID, 0);
        simInt ret = simGetStackBoolValue(p->stackID, &latch);
        if(ret == -1)
        {
            simSetLastError("simExtROS_advertise", "internal error reading argument 4");
            return;
        }
        if(ret == 0)
        {
            simSetLastError("simExtROS_advertise", "type error: argument 4 must be a bool");
            return;
        }
    }

    PublisherProxy *publisherProxy = new PublisherProxy();
    publisherProxy->handle = publisherProxyNextHandle++;
    publisherProxy->topicName = topicName;
    publisherProxy->topicType = topicType;
    publisherProxies[publisherProxy->handle] = publisherProxy;

    if(topicType == "std_msgs/Float32")
    {
        publisherProxy->publisher = nh->advertise<std_msgs::Float32>(topicName, queueSize, latch);
    }
    else
    {
        simSetLastError("simExtROS_advertise", "unsupported message type. please edit and recompile ROS plugin");
        return;
    }

    if(!publisherProxy->publisher)
    {
        simSetLastError("simExtROS_advertise", "failed creation of ROS publisher");
        return;
    }

    // write output arguments to stack:

    simPopStackItem(p->stackID, 0);

    {
        int ret = simPushInt32OntoStack(p->stackID, publisherProxy->handle);
        if(ret == -1)
        {
            simSetLastError("simExtROS_advertise", "internal error writing output argument 1");
            return;
        }
    }
}

void simExtROS_shutdownPublisher(SScriptCallBack *p)
{
    // input arguments:

    simInt publisherHandle;

    // read input argument count from stack:

    int numArgs = simGetStackSize(p->stackID);
    int totalArgs = 1;
    int requiredArgs = 1;

    if(numArgs < requiredArgs || numArgs > totalArgs)
    {
        simSetLastError("simExtROS_shutdownPublisher", "wrong number of arguments");
        return;
    }

    // read input argument values from stack:

    if(numArgs >= 1)
    {
        simMoveStackItemToTop(p->stackID, 0);
        simInt ret = simGetStackInt32Value(p->stackID, &publisherHandle);
        if(ret == -1)
        {
            simSetLastError("simExtROS_shutdownPublisher", "internal error reading argument 1");
            return;
        }
        if(ret == 0)
        {
            simSetLastError("simExtROS_shutdownPublisher", "type error: argument 1 must be a number");
            return;
        }
    }

    if(publisherProxies.find(publisherHandle) == publisherProxies.end())
    {
        simSetLastError("simExtROS_shutdownPublisher", "invalid publisher handle");
        return;
    }

    PublisherProxy *publisherProxy = publisherProxies[publisherHandle];
    publisherProxy->publisher.shutdown();
    publisherProxies.erase(publisherProxy->handle);
    delete publisherProxy;

    // write output arguments to stack:

    {
    }
}

void simExtROS_publish(SScriptCallBack *p)
{
    // input arguments:

    simInt publisherHandle;

    // read input argument count from stack:

    int numArgs = simGetStackSize(p->stackID);
    int totalArgs = 2;
    int requiredArgs = 2;

    if(numArgs < requiredArgs || numArgs > totalArgs)
    {
        simSetLastError("simExtROS_publish", "wrong number of arguments");
        return;
    }

    // read input argument values from stack:

    if(numArgs >= 1)
    {
        simMoveStackItemToTop(p->stackID, 0);
        simInt ret = simGetStackInt32Value(p->stackID, &publisherHandle);
        if(ret == -1)
        {
            simSetLastError("simExtROS_publish", "internal error reading argument 1");
            return;
        }
        if(ret == 0)
        {
            simSetLastError("simExtROS_publish", "type error: argument 1 must be a number");
            return;
        }
    }

    if(publisherProxies.find(publisherHandle) == publisherProxies.end())
    {
        simSetLastError("simExtROS_publish", "invalid publisher handle");
        return;
    }

    PublisherProxy *publisherProxy = publisherProxies[publisherHandle];

    simMoveStackItemToTop(p->stackID, 0);

    if(publisherProxy->topicType == "std_msgs/Float32")
    {
        std_msgs::Float32 msg;
        if(!read__std_msgs__Float32(p->stackID, &msg))
        {
            simSetLastError("simExtROS_publish", "invalid message format (check stderr)");
            return;
        }
        publisherProxy->publisher.publish(msg);
    }
    else
    {
        simSetLastError("simExtROS_publish", "unsupported message type. please edit and recompile ROS plugin");
        return;
    }

    // write output arguments to stack:

    {
    }
}

bool initialize()
{
    int argc = 0;
    char *argv[] = {};
    ros::init(argc, argv, "vrep_ros_plugin");

    if(!ros::master::check())
        return false;

    nh = new ros::NodeHandle("~");

    return true;
}

void shutdown()
{
    ros::shutdown();
}

bool registerScriptStuff()
{
    {
        int ret = simRegisterScriptCallbackFunction("simExtROS_subscribe@ROS", "number subscriberHandle=simExtROS_subscribe(string topicName, string topicType, string callback, number queueSize=0)", simExtROS_subscribe);
        if(ret == 0)
        {
            std::cout << "Plugin 'ROS': warning: replaced function simExtROS_subscribe" << std::endl;
        }
        else if(ret == -1)
        {
            std::cout << "Plugin 'ROS': error: failed to register function simExtROS_subscribe" << std::endl;
            return false;
        }
    }
    {
        int ret = simRegisterScriptCallbackFunction("simExtROS_shutdownSubscriber@ROS", "simExtROS_shutdownSubscriber(number subscriberHandle)", simExtROS_shutdownSubscriber);
        if(ret == 0)
        {
            std::cout << "Plugin 'ROS': warning: replaced function simExtROS_shutdownSubscriber" << std::endl;
        }
        else if(ret == -1)
        {
            std::cout << "Plugin 'ROS': error: failed to register function simExtROS_shutdownSubscriber" << std::endl;
            return false;
        }
    }
    {
        int ret = simRegisterScriptCallbackFunction("simExtROS_advertise@ROS", "number publisherHandle=simExtROS_advertise(string topicName, string topicType, bool latch=false, number queueSize=0)", simExtROS_advertise);
        if(ret == 0)
        {
            std::cout << "Plugin 'ROS': warning: replaced function simExtROS_advertise" << std::endl;
        }
        else if(ret == -1)
        {
            std::cout << "Plugin 'ROS': error: failed to register function simExtROS_advertise" << std::endl;
            return false;
        }
    }
    {
        int ret = simRegisterScriptCallbackFunction("simExtROS_shutdownPublisher@ROS", "simExtROS_shutdownPublisher(number publisherHandle)", simExtROS_shutdownPublisher);
        if(ret == 0)
        {
            std::cout << "Plugin 'ROS': warning: replaced function simExtROS_shutdownPublisher" << std::endl;
        }
        else if(ret == -1)
        {
            std::cout << "Plugin 'ROS': error: failed to register function simExtROS_shutdownPublisher" << std::endl;
            return false;
        }
    }
    {
        int ret = simRegisterScriptCallbackFunction("simExtROS_publish@ROS", "simExtROS_publish(number publisherHandle, table message)", simExtROS_publish);
        if(ret == 0)
        {
            std::cout << "Plugin 'ROS': warning: replaced function simExtROS_publish" << std::endl;
        }
        else if(ret == -1)
        {
            std::cout << "Plugin 'ROS': error: failed to register function simExtROS_publish" << std::endl;
            return false;
        }
    }
    return true;
}

// This is the plugin start routine (called just once, just after the plugin was loaded):
VREP_DLLEXPORT unsigned char v_repStart(void* reservedPointer,int reservedInt)
{
	// Dynamically load and bind V-REP functions:
	// ******************************************
	// 1. Figure out this plugin's directory:
	char curDirAndFile[1024];
	getcwd(curDirAndFile, sizeof(curDirAndFile));

	std::string currentDirAndPath(curDirAndFile);
	// 2. Append the V-REP library's name:
	std::string temp(currentDirAndPath);
	#ifdef _WIN32
		temp+="\\v_rep.dll";
	#elif defined (__linux)
		temp+="/libv_rep.so";
	#elif defined (__APPLE__)
		temp+="/libv_rep.dylib";
	#endif

	// 3. Load the V-REP library:
	vrepLib=loadVrepLibrary(temp.c_str());
	if (vrepLib==NULL)
	{
		std::cout << "Error, could not find or correctly load the V-REP library. Cannot start 'ROS' plugin.\n";
		return 0; // Means error, V-REP will unload this plugin
	}
	if (getVrepProcAddresses(vrepLib)==0)
	{
		std::cout << "Error, could not find all required functions in the V-REP library. Cannot start 'ROS' plugin.\n";
		unloadVrepLibrary(vrepLib);
		return 0; // Means error, V-REP will unload this plugin
	}
	// ******************************************

	// Check the version of V-REP:
	// ******************************************
	int vrepVer;
	simGetIntegerParameter(sim_intparam_program_version,&vrepVer);
	if (vrepVer<20605) // if V-REP version is smaller than 2.06.04
	{
		std::cout << "Sorry, your V-REP copy is somewhat old. Cannot start 'ROS' plugin.\n";
		unloadVrepLibrary(vrepLib);
		return 0; // Means error, V-REP will unload this plugin
	}
	// ******************************************
	

	if(!initialize()) 
	{
		std::cout << "ROS master is not running. Cannot start 'ROS' plugin.\n";
		return 0; //If the master is not running then the plugin is not loaded.
	}
	
	// Register script functions and variables:
    if(!registerScriptStuff())
    {
        return 0;
    }

	return PLUGIN_VERSION; // initialization went fine, we return the version number of this plugin (can be queried with simGetModuleName)
}

// This is the plugin end routine (called just once, when V-REP is ending, i.e. releasing this plugin):
VREP_DLLEXPORT void v_repEnd()
{
	shutdown();
	unloadVrepLibrary(vrepLib); // release the library
}

// This is the plugin messaging routine (i.e. V-REP calls this function very often, with various messages):
VREP_DLLEXPORT void* v_repMessage(int message,int* auxiliaryData,void* customData,int* replyData)
{ 
	// This is called quite often. Just watch out for messages/events you want to handle
	// Keep following 4 lines at the beginning and unchanged:
	int errorModeSaved;
	simGetIntegerParameter(sim_intparam_error_report_mode,&errorModeSaved);
	simSetIntegerParameter(sim_intparam_error_report_mode,sim_api_errormessage_ignore);
	void* retVal=NULL;

	// Here we can intercept many messages from V-REP (actually callbacks). Only the most important messages are listed here:

	if (message==sim_message_eventcallback_instancepass)
	{ 
		// This message is sent each time the scene was rendered (well, shortly after) (very often)
		// When a simulation is not running, but you still need to execute some commands, then put some code here
        ros::spinOnce();
	}

	if (message==sim_message_eventcallback_mainscriptabouttobecalled)
	{ 
		// Main script is about to be run (only called while a simulation is running (and not paused!))
		//
		// This is a good location to execute simulation commands
	}

	if (message==sim_message_eventcallback_simulationabouttostart)
	{ 
	    // Simulation is about to start
	}

	if (message==sim_message_eventcallback_simulationended)
	{ 
		// Simulation just ended
	}

	// Keep following unchanged:
	simSetIntegerParameter(sim_intparam_error_report_mode,errorModeSaved); // restore previous settings
	return retVal;
}

