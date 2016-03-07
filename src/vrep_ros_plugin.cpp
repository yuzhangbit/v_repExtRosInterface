#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <map>

#include <boost/lexical_cast.hpp>

#include <ros/ros.h>

#include <vrep_ros_plugin/ros_msg_builtin_io.h>
#include <vrep_ros_plugin/ros_msg_io.h>

#include "../include/v_repLib.h"
#include "../include/scriptFunctionData.h"
#include "../include/vrep_ros_plugin/vrep_ros_plugin.h"

#define PLUGIN_VERSION 5 // 5 since 3.3.1 (using stacks to exchange data with scripts)

#define CONCAT(x,y,z) x y z
#define strConCat(x,y,z)	CONCAT(x,y,z)

LIBRARY vrepLib; // the V-REP library that we will dynamically load and bind

ros::NodeHandle *nh = NULL;

int subscriberProxyNextHandle = 3562;
int publisherProxyNextHandle = 7980;

std::map<int, SubscriberProxy *> subscriberProxies;
std::map<int, PublisherProxy *> publisherProxies;

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

    if(0) {}
#include <vrep_ros_plugin/generated_sub.cpp>
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

    if(0) {}
#include <vrep_ros_plugin/generated_adv.cpp>
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

    if(0) {}
#include <vrep_ros_plugin/generated_pub.cpp>
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

