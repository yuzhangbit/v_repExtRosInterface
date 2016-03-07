#include <cpp_io.h>
#include <v_repLib.h>
#include <iostream>

bool read__bool(int stack, bool *value)
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

bool read__int(int stack, int *value)
{
    int v;
    if(simGetStackInt32Value(stack, &v) == 1)
    {
        *value = v;
        simPopStackItem(stack, 1);
        return true;
    }
    else
    {
        std::cerr << "read__int: error: expected bool value." << std::endl;
        return false;
    }
}

bool read__float(int stack, float *value)
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
        std::cerr << "read__float: error: expected float value." << std::endl;
        return false;
    }
}

bool read__std__string(int stack, std::string *value)
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
        std::cerr << "read__std__string: error: expected string value." << std::endl;
        return false;
    }
}

bool write__bool(bool value, int stack)
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

bool write__int(int value, int stack)
{
    int v = value;
    if(simPushInt32OntoStack(stack, v) == -1)
    {
        std::cerr << "write__int: error: push table value (data) failed." << std::endl;
        return false;
    }
    else
    {
        return true;
    }
}

bool write__float(float value, int stack)
{
    simFloat v = value;
    if(simPushFloatOntoStack(stack, v) == -1)
    {
        std::cerr << "write__float: error: push table value (data) failed." << std::endl;
        return false;
    }
    else
    {
        return true;
    }
}

bool write__std__string(std::string value, int stack)
{
    const simChar *v = value.c_str();
    if(simPushStringOntoStack(stack, v, 0) == -1)
    {
        std::cerr << "write__std__string: error: push table value (data) failed." << std::endl;
        return false;
    }
    else
    {
        return true;
    }
}


