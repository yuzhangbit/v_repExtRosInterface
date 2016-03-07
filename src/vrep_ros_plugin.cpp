#include <vrep_ros_plugin.h>

#define PLUGIN_VERSION 5 // 5 since 3.3.1 (using stacks to exchange data with scripts)

#define CONCAT(x,y,z) x y z
#define strConCat(x,y,z)	CONCAT(x,y,z)

LIBRARY vrepLib; // the V-REP library that we will dynamically load and bind

ros::NodeHandle *nh = NULL;

int subscriberProxyNextHandle = 3562;
int publisherProxyNextHandle = 7980;

std::map<int, SubscriberProxy *> subscriberProxies;
std::map<int, PublisherProxy *> publisherProxies;

void subscribe(SScriptCallBack * p, const char * cmd, subscribe_in * in, subscribe_out * out)
{
    SubscriberProxy *subscriberProxy = new SubscriberProxy();
    subscriberProxy->handle = subscriberProxyNextHandle++;
    subscriberProxy->topicName = in->topicName;
    subscriberProxy->topicType = in->topicType;
    subscriberProxy->topicCallback.scriptId = p->scriptID;
    subscriberProxy->topicCallback.name = in->topicCallback;
    subscriberProxies[subscriberProxy->handle] = subscriberProxy;

    if(0) {}
#include <sub.cpp>
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

    out->subscriberHandle = subscriberProxy->handle;
}

void shutdownSubscriber(SScriptCallBack * p, const char * cmd, shutdownSubscriber_in * in, shutdownSubscriber_out * out)
{
    if(subscriberProxies.find(in->subscriberHandle) == subscriberProxies.end())
    {
        simSetLastError("simExtROS_shutdownSubscriber", "invalid subscriber handle");
        return;
    }

    SubscriberProxy *subscriberProxy = subscriberProxies[in->subscriberHandle];
    subscriberProxy->subscriber.shutdown();
    subscriberProxies.erase(subscriberProxy->handle);
    delete subscriberProxy;
}

void advertise(SScriptCallBack * p, const char * cmd, advertise_in * in, advertise_out * out)
{
    PublisherProxy *publisherProxy = new PublisherProxy();
    publisherProxy->handle = publisherProxyNextHandle++;
    publisherProxy->topicName = in->topicName;
    publisherProxy->topicType = in->topicType;
    publisherProxies[publisherProxy->handle] = publisherProxy;

    if(0) {}
#include <adv.cpp>
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

    out->publisherHandle = publisherProxy->handle;
}

void shutdownPublisher(SScriptCallBack * p, const char * cmd, shutdownPublisher_in * in, shutdownPublisher_out * out)
{
    if(publisherProxies.find(in->publisherHandle) == publisherProxies.end())
    {
        simSetLastError("simExtROS_shutdownPublisher", "invalid publisher handle");
        return;
    }

    PublisherProxy *publisherProxy = publisherProxies[in->publisherHandle];
    publisherProxy->publisher.shutdown();
    publisherProxies.erase(publisherProxy->handle);
    delete publisherProxy;
}

void publish(SScriptCallBack * p, const char * cmd, publish_in * in, publish_out * out)
{
    if(publisherProxies.find(in->publisherHandle) == publisherProxies.end())
    {
        simSetLastError("simExtROS_publish", "invalid publisher handle");
        return;
    }

    PublisherProxy *publisherProxy = publisherProxies[in->publisherHandle];

    simMoveStackItemToTop(p->stackID, 0);

    if(0) {}
#include <pub.cpp>
    else
    {
        simSetLastError("simExtROS_publish", "unsupported message type. please edit and recompile ROS plugin");
        return;
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

