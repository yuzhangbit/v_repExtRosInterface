#include <ros_msg_builtin_io.h>
#include <v_repLib.h>
#include <iostream>

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

bool read__float64(int stack, double *value)
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

bool write__string(std::string value, int stack)
{
    const simChar *v = value.c_str();
    if(simPushStringOntoStack(stack, v, 0) == -1)
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

