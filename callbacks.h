// This file is generated automatically! Do NOT edit!

#include "scriptFunctionData.h"

#include "v_repLib.h"

#include <boost/assign/list_of.hpp>

struct subscribe_in
{
    std::string topic;
    std::string type;
    std::string callback;
    int queueSize;
    
    subscribe_in();
};

struct subscribe_out
{
    std::string subscriber;
    
    subscribe_out();
};

void subscribe(SScriptCallBack * p, subscribe_in * in, subscribe_out * out);

void subscribe(SScriptCallBack * p, const char * cmd, subscribe_in * in, subscribe_out * out);

std::string subscribe(SScriptCallBack * p, std::string topic, std::string type, std::string callback, int queueSize = -1);

void subscribe(SScriptCallBack * p, subscribe_out * out, std::string topic, std::string type, std::string callback, int queueSize = -1);

void subscribe_callback(SScriptCallBack * p);

struct advertise_in
{
    std::string topic;
    std::string type;
    bool latch;
    int queueSize;
    std::string connectCallback;
    std::string disconnectCallback;
    
    advertise_in();
};

struct advertise_out
{
    std::string publisher;
    
    advertise_out();
};

void advertise(SScriptCallBack * p, advertise_in * in, advertise_out * out);

void advertise(SScriptCallBack * p, const char * cmd, advertise_in * in, advertise_out * out);

std::string advertise(SScriptCallBack * p, std::string topic, std::string type, bool latch = false, int queueSize = -1, std::string connectCallback = nil, std::string disconnectCallback = nil);

void advertise(SScriptCallBack * p, advertise_out * out, std::string topic, std::string type, bool latch = false, int queueSize = -1, std::string connectCallback = nil, std::string disconnectCallback = nil);

void advertise_callback(SScriptCallBack * p);

struct publish_in
{
    std::string publisher;
    
    publish_in();
};

struct publish_out
{
    publish_out();
};

void publish(SScriptCallBack * p, publish_in * in, publish_out * out);

void publish(SScriptCallBack * p, const char * cmd, publish_in * in, publish_out * out);

void publish(SScriptCallBack * p, publish_out * out, std::string publisher);

void publish_callback(SScriptCallBack * p);

struct shutdownPublisher_in
{
    std::string publisher;
    
    shutdownPublisher_in();
};

struct shutdownPublisher_out
{
    shutdownPublisher_out();
};

void shutdownPublisher(SScriptCallBack * p, shutdownPublisher_in * in, shutdownPublisher_out * out);

void shutdownPublisher(SScriptCallBack * p, const char * cmd, shutdownPublisher_in * in, shutdownPublisher_out * out);

void shutdownPublisher(SScriptCallBack * p, shutdownPublisher_out * out, std::string publisher);

void shutdownPublisher_callback(SScriptCallBack * p);

struct shutdownSubscriber_in
{
    std::string subscriber;
    
    shutdownSubscriber_in();
};

struct shutdownSubscriber_out
{
    shutdownSubscriber_out();
};

void shutdownSubscriber(SScriptCallBack * p, shutdownSubscriber_in * in, shutdownSubscriber_out * out);

void shutdownSubscriber(SScriptCallBack * p, const char * cmd, shutdownSubscriber_in * in, shutdownSubscriber_out * out);

void shutdownSubscriber(SScriptCallBack * p, shutdownSubscriber_out * out, std::string subscriber);

void shutdownSubscriber_callback(SScriptCallBack * p);

struct subscriberCallback_in
{
    std::string topic;
    std::string type;
    
    subscriberCallback_in();
};

struct subscriberCallback_out
{
    subscriberCallback_out();
};

bool subscriberCallback(simInt scriptId, const char * func, subscriberCallback_in * in, subscriberCallback_out * out);

void registerScriptStuff();

