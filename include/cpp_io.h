#ifndef VREP_ROS_PLUGIN__CPP_IO__H
#define VREP_ROS_PLUGIN__CPP_IO__H

#include <v_repLib.h>
#include <string>

bool read__bool(int stack, bool *value);
bool read__int(int stack, int *value);
bool read__float(int stack, float *value);
bool read__std__string(int stack, std::string *value);
bool write__bool(bool value, int stack);
bool write__int(int value, int stack);
bool write__float(float value, int stack);
bool write__std__string(std::string value, int stack);

#endif // VREP_ROS_PLUGIN__CPP_IO__H
