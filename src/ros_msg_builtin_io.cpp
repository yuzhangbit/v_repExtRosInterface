#include <ros_msg_builtin_io.h>
#include <v_repLib.h>
#include <iostream>

void read__bool(int stack, uint8_t *value)
{
    simBool v;
    if(simGetStackBoolValueE(stack, &v) == 1)
    {
        *value = v;
        simPopStackItemE(stack, 1);
    }
    else
    {
        throw exception("expected bool");
    }
}

void read__int8(int stack, int8_t *value)
{
    simInt v;
    if(simGetStackInt32ValueE(stack, &v) == 1)
    {
        *value = v;
        simPopStackItemE(stack, 1);
    }
    else
    {
        throw exception("expected integer");
    }
}

void read__uint8(int stack, uint8_t *value)
{
    simInt v;
    if(simGetStackInt32ValueE(stack, &v) == 1)
    {
        *value = v;
        simPopStackItemE(stack, 1);
    }
    else
    {
        throw exception("expected integer");
    }
}

void read__int16(int stack, int16_t *value)
{
    simInt v;
    if(simGetStackInt32ValueE(stack, &v) == 1)
    {
        *value = v;
        simPopStackItemE(stack, 1);
    }
    else
    {
        throw exception("expected integer");
    }
}

void read__uint16(int stack, uint16_t *value)
{
    simInt v;
    if(simGetStackInt32ValueE(stack, &v) == 1)
    {
        *value = v;
        simPopStackItemE(stack, 1);
    }
    else
    {
        throw exception("expected integer");
    }
}

void read__int32(int stack, int32_t *value)
{
    simInt v;
    if(simGetStackInt32ValueE(stack, &v) == 1)
    {
        *value = v;
        simPopStackItemE(stack, 1);
    }
    else
    {
        throw exception("expected integer");
    }
}

void read__uint32(int stack, uint32_t *value)
{
    simInt v;
    if(simGetStackInt32ValueE(stack, &v) == 1)
    {
        *value = v;
        simPopStackItemE(stack, 1);
    }
    else
    {
        throw exception("expected integer");
    }
}

void read__int64(int stack, int64_t *value)
{
    // XXX: we represent Int64 as double - possible loss of precision!
    simDouble v;
    if(simGetStackDoubleValueE(stack, &v) == 1)
    {
        *value = v;
        simPopStackItemE(stack, 1);
    }
    else
    {
        throw exception("expected double");
    }
}

void read__uint64(int stack, uint64_t *value)
{
    // XXX: we represent UInt64 as double - possible loss of precision!
    simDouble v;
    if(simGetStackDoubleValueE(stack, &v) == 1)
    {
        *value = v;
        simPopStackItemE(stack, 1);
    }
    else
    {
        throw exception("expected double");
    }
}

void read__float32(int stack, float *value)
{
    simFloat v;
    if(simGetStackFloatValueE(stack, &v) == 1)
    {
        *value = v;
        simPopStackItemE(stack, 1);
    }
    else
    {
        throw exception("expected float");
    }
}

void read__float64(int stack, double *value)
{
    simDouble v;
    if(simGetStackDoubleValueE(stack, &v) == 1)
    {
        *value = v;
        simPopStackItemE(stack, 1);
    }
    else
    {
        throw exception("expected double");
    }
}

void read__string(int stack, std::string *value)
{
    simChar *str;
    simInt strSize;
    if((str = simGetStackStringValueE(stack, &strSize)) != NULL && strSize > 0)
    {
        *value = std::string(str);
        simPopStackItemE(stack, 1);
    }
    else
    {
        throw exception("expected string");
    }
}

void read__time(int stack, ros::Time *value)
{
    simDouble v;
    if(simGetStackDoubleValueE(stack, &v) == 1)
    {
        *value = ros::Time(v);
        simPopStackItemE(stack, 1);
    }
    else
    {
        throw exception("expected double");
    }
}

void read__duration(int stack, ros::Duration *value)
{
    simDouble v;
    if(simGetStackDoubleValueE(stack, &v) == 1)
    {
        *value = ros::Duration(v);
        simPopStackItemE(stack, 1);
    }
    else
    {
        throw exception("expected double");
    }
}

void write__bool(uint8_t value, int stack)
{
    simBool v = value;
    simPushBoolOntoStackE(stack, v);
}

void write__int8(int8_t value, int stack)
{
    simInt v = value;
    simPushInt32OntoStackE(stack, v);
}

void write__uint8(uint8_t value, int stack)
{
    simInt v = value;
    simPushInt32OntoStackE(stack, v);
}

void write__int16(int16_t value, int stack)
{
    simInt v = value;
    simPushInt32OntoStackE(stack, v);
}

void write__uint16(uint16_t value, int stack)
{
    simInt v = value;
    simPushInt32OntoStackE(stack, v);
}

void write__int32(int32_t value, int stack)
{
    simInt v = value;
    simPushInt32OntoStackE(stack, v);
}

void write__uint32(uint32_t value, int stack)
{
    simInt v = value;
    simPushInt32OntoStackE(stack, v);
}

void write__int64(int64_t value, int stack)
{
    // XXX: we represent Int64 as double - possible loss of precision!
    simDouble v = value;
    simPushDoubleOntoStackE(stack, v);
}

void write__uint64(uint64_t value, int stack)
{
    // XXX: we represent UInt64 as double - possible loss of precision!
    simDouble v = value;
    simPushDoubleOntoStackE(stack, v);
}

void write__float32(float value, int stack)
{
    simFloat v = value;
    simPushFloatOntoStackE(stack, v);
}

void write__float64(double value, int stack)
{
    simDouble v = value;
    simPushDoubleOntoStackE(stack, v);
}

void write__string(std::string value, int stack)
{
    const simChar *v = value.c_str();
    simPushStringOntoStackE(stack, v, 0);
}

void write__time(ros::Time value, int stack)
{
    simDouble v = value.toSec();
    simPushDoubleOntoStackE(stack, v);
}

void write__duration(ros::Duration value, int stack)
{
    simDouble v = value.toSec();
    simPushDoubleOntoStackE(stack, v);
}

