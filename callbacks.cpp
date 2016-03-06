// This file is generated automatically! Do NOT edit!

#include <boost/lexical_cast.hpp>

#include "callbacks.h"

subscribe_in::subscribe_in()
{
    queueSize = -1;
}

subscribe_out::subscribe_out()
{
    subscriber = nil;
}

void subscribe(SScriptCallBack * p, subscribe_in * in, subscribe_out * out)
{
    subscribe(p, "simExtROS_subscribe", in, out);
}

std::string subscribe(SScriptCallBack * p, std::string topic, std::string type, std::string callback, int queueSize)
{
    subscribe_in in_args;
    in_args.topic = topic;
    in_args.type = type;
    in_args.callback = callback;
    in_args.queueSize = queueSize;
    subscribe_out out_args;
    subscribe(p, &in_args, &out_args);
    return out_args.subscriber;
}

void subscribe(SScriptCallBack * p, subscribe_out * out, std::string topic, std::string type, std::string callback, int queueSize)
{
    subscribe_in in_args;
    in_args.topic = topic;
    in_args.type = type;
    in_args.callback = callback;
    in_args.queueSize = queueSize;
    subscribe(p, &in_args, out);
}

const int inArgs_subscribe[] = {4, sim_script_arg_string, 0, sim_script_arg_string, 0, sim_script_arg_string, 0, sim_script_arg_int32, 0};

void subscribe_callback(SScriptCallBack * p)
{
    //p->outputArgCount = 0;
    CScriptFunctionData D;
    if(D.readDataFromStack(p->stackID, inArgs_subscribe, 3, "simExtROS_subscribe"))
    {
        std::vector<CScriptFunctionDataItem>* inData = D.getInDataPtr();
        subscribe_in in_args;
        subscribe_out out_args;
        in_args.topic = inData->at(0).stringData[0];
        in_args.type = inData->at(1).stringData[0];
        in_args.callback = inData->at(2).stringData[0];
        if(inData->size() > 3) in_args.queueSize = inData->at(3).int32Data[0];
        subscribe(p, "simExtROS_subscribe", &in_args, &out_args);
        D.pushOutData(CScriptFunctionDataItem(out_args.subscriber));
    }
    D.writeDataToStack(p->stackID);
}

advertise_in::advertise_in()
{
    latch = false;
    queueSize = -1;
    connectCallback = nil;
    disconnectCallback = nil;
}

advertise_out::advertise_out()
{
    publisher = nil;
}

void advertise(SScriptCallBack * p, advertise_in * in, advertise_out * out)
{
    advertise(p, "simExtROS_advertise", in, out);
}

std::string advertise(SScriptCallBack * p, std::string topic, std::string type, bool latch, int queueSize, std::string connectCallback, std::string disconnectCallback)
{
    advertise_in in_args;
    in_args.topic = topic;
    in_args.type = type;
    in_args.latch = latch;
    in_args.queueSize = queueSize;
    in_args.connectCallback = connectCallback;
    in_args.disconnectCallback = disconnectCallback;
    advertise_out out_args;
    advertise(p, &in_args, &out_args);
    return out_args.publisher;
}

void advertise(SScriptCallBack * p, advertise_out * out, std::string topic, std::string type, bool latch, int queueSize, std::string connectCallback, std::string disconnectCallback)
{
    advertise_in in_args;
    in_args.topic = topic;
    in_args.type = type;
    in_args.latch = latch;
    in_args.queueSize = queueSize;
    in_args.connectCallback = connectCallback;
    in_args.disconnectCallback = disconnectCallback;
    advertise(p, &in_args, out);
}

const int inArgs_advertise[] = {6, sim_script_arg_string, 0, sim_script_arg_string, 0, sim_script_arg_bool, 0, sim_script_arg_int32, 0, sim_script_arg_string, 0, sim_script_arg_string, 0};

void advertise_callback(SScriptCallBack * p)
{
    //p->outputArgCount = 0;
    CScriptFunctionData D;
    if(D.readDataFromStack(p->stackID, inArgs_advertise, 2, "simExtROS_advertise"))
    {
        std::vector<CScriptFunctionDataItem>* inData = D.getInDataPtr();
        advertise_in in_args;
        advertise_out out_args;
        in_args.topic = inData->at(0).stringData[0];
        in_args.type = inData->at(1).stringData[0];
        if(inData->size() > 2) in_args.latch = inData->at(2).boolData[0];
        if(inData->size() > 3) in_args.queueSize = inData->at(3).int32Data[0];
        if(inData->size() > 4) in_args.connectCallback = inData->at(4).stringData[0];
        if(inData->size() > 5) in_args.disconnectCallback = inData->at(5).stringData[0];
        advertise(p, "simExtROS_advertise", &in_args, &out_args);
        D.pushOutData(CScriptFunctionDataItem(out_args.publisher));
    }
    D.writeDataToStack(p->stackID);
}

publish_in::publish_in()
{

}

publish_out::publish_out()
{

}

void publish(SScriptCallBack * p, publish_in * in, publish_out * out)
{
    publish(p, "simExtROS_publish", in, out);
}

void publish(SScriptCallBack * p, publish_out * out, std::string publisher)
{
    publish_in in_args;
    in_args.publisher = publisher;
    publish(p, &in_args, out);
}

const int inArgs_publish[] = {2, sim_script_arg_string, 0, sim_script_arg_table, 0};

void publish_callback(SScriptCallBack * p)
{
    //p->outputArgCount = 0;
    CScriptFunctionData D;
    if(D.readDataFromStack(p->stackID, inArgs_publish, 2, "simExtROS_publish"))
    {
        std::vector<CScriptFunctionDataItem>* inData = D.getInDataPtr();
        publish_in in_args;
        publish_out out_args;
        in_args.publisher = inData->at(0).stringData[0];
        publish(p, "simExtROS_publish", &in_args, &out_args);
    }
    D.writeDataToStack(p->stackID);
}

shutdownPublisher_in::shutdownPublisher_in()
{

}

shutdownPublisher_out::shutdownPublisher_out()
{

}

void shutdownPublisher(SScriptCallBack * p, shutdownPublisher_in * in, shutdownPublisher_out * out)
{
    shutdownPublisher(p, "simExtROS_shutdownPublisher", in, out);
}

void shutdownPublisher(SScriptCallBack * p, shutdownPublisher_out * out, std::string publisher)
{
    shutdownPublisher_in in_args;
    in_args.publisher = publisher;
    shutdownPublisher(p, &in_args, out);
}

const int inArgs_shutdownPublisher[] = {1, sim_script_arg_string, 0};

void shutdownPublisher_callback(SScriptCallBack * p)
{
    //p->outputArgCount = 0;
    CScriptFunctionData D;
    if(D.readDataFromStack(p->stackID, inArgs_shutdownPublisher, 1, "simExtROS_shutdownPublisher"))
    {
        std::vector<CScriptFunctionDataItem>* inData = D.getInDataPtr();
        shutdownPublisher_in in_args;
        shutdownPublisher_out out_args;
        in_args.publisher = inData->at(0).stringData[0];
        shutdownPublisher(p, "simExtROS_shutdownPublisher", &in_args, &out_args);
    }
    D.writeDataToStack(p->stackID);
}

shutdownSubscriber_in::shutdownSubscriber_in()
{

}

shutdownSubscriber_out::shutdownSubscriber_out()
{

}

void shutdownSubscriber(SScriptCallBack * p, shutdownSubscriber_in * in, shutdownSubscriber_out * out)
{
    shutdownSubscriber(p, "simExtROS_shutdownSubscriber", in, out);
}

void shutdownSubscriber(SScriptCallBack * p, shutdownSubscriber_out * out, std::string subscriber)
{
    shutdownSubscriber_in in_args;
    in_args.subscriber = subscriber;
    shutdownSubscriber(p, &in_args, out);
}

const int inArgs_shutdownSubscriber[] = {1, sim_script_arg_string, 0};

void shutdownSubscriber_callback(SScriptCallBack * p)
{
    //p->outputArgCount = 0;
    CScriptFunctionData D;
    if(D.readDataFromStack(p->stackID, inArgs_shutdownSubscriber, 1, "simExtROS_shutdownSubscriber"))
    {
        std::vector<CScriptFunctionDataItem>* inData = D.getInDataPtr();
        shutdownSubscriber_in in_args;
        shutdownSubscriber_out out_args;
        in_args.subscriber = inData->at(0).stringData[0];
        shutdownSubscriber(p, "simExtROS_shutdownSubscriber", &in_args, &out_args);
    }
    D.writeDataToStack(p->stackID);
}

subscriberCallback_in::subscriberCallback_in()
{

}

subscriberCallback_out::subscriberCallback_out()
{

}

const int outArgs_subscriberCallback[] = {0};

bool subscriberCallback(simInt scriptId, const char * func, subscriberCallback_in * in, subscriberCallback_out * out)
{
    //SScriptCallBack c;
    int stackID = simCreateStack();
    CScriptFunctionData D;
    bool ret = false;
    
    D.pushOutData_scriptFunctionCall(CScriptFunctionDataItem(in->topic));
    D.pushOutData_scriptFunctionCall(CScriptFunctionDataItem(in->type));
    D.writeDataToStack_scriptFunctionCall(stackID);
    
    if(simCallScriptFunctionEx(scriptId, func, stackID) != -1)
    {
        if(D.readDataFromStack_scriptFunctionCall(stackID, outArgs_subscriberCallback, outArgs_subscriberCallback[0], func))
        {
            std::vector<CScriptFunctionDataItem> *outData = D.getOutDataPtr_scriptFunctionCall();
            ret = true;
        }
        else
        {
            simSetLastError(func, "return value size and/or type is incorrect");
        }
    }
    else
    {
        simSetLastError(func, "callback returned an error");
    }
    
    D.releaseBuffers_scriptFunctionCall(&c);
    simReleaseStack(stackID);
    return ret;
}

void registerScriptStuff()
{
    std::vector<int> inArgs;
    simRegisterScriptCallbackFunction("simExtROS_subscribe@ROS", "string subscriber=simExtROS_subscribe(string topic,string type,string callback,number queueSize=-1)", subscribe_callback);
    simRegisterScriptCallbackFunction("simExtROS_advertise@ROS", "string publisher=simExtROS_advertise(string topic,string type,bool latch=false,number queueSize=-1,string connectCallback=nil,string disconnectCallback=nil)", advertise_callback);
    simRegisterScriptCallbackFunction("simExtROS_publish@ROS", "=simExtROS_publish(string publisher,table message)", publish_callback);
    simRegisterScriptCallbackFunction("simExtROS_shutdownPublisher@ROS", "=simExtROS_shutdownPublisher(string publisher)", shutdownPublisher_callback);
    simRegisterScriptCallbackFunction("simExtROS_shutdownSubscriber@ROS", "=simExtROS_shutdownSubscriber(string subscriber)", shutdownSubscriber_callback);
}

