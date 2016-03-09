#ifndef VREP_ROS_PLUGIN__ROS_MSG_BUILTIN_IO__H
#define VREP_ROS_PLUGIN__ROS_MSG_BUILTIN_IO__H

#include <stubs.h>
#include <ros/ros.h>

void read__bool(int stack, uint8_t *value);
void read__int8(int stack, int8_t *value);
void read__uint8(int stack, uint8_t *value);
void read__int16(int stack, int16_t *value);
void read__uint16(int stack, uint16_t *value);
void read__int32(int stack, int32_t *value);
void read__uint32(int stack, uint32_t *value);
void read__int64(int stack, int64_t *value);
void read__uint64(int stack, uint64_t *value);
void read__float32(int stack, float *value);
void read__float64(int stack, double *value);
void read__string(int stack, std::string *value);
void read__time(int stack, ros::Time *value);
void read__duration(int stack, ros::Duration *value);
void write__bool(uint8_t value, int stack);
void write__int8(int8_t value, int stack);
void write__uint8(uint8_t value, int stack);
void write__int16(int16_t value, int stack);
void write__uint16(uint16_t value, int stack);
void write__int32(int32_t value, int stack);
void write__uint32(uint32_t value, int stack);
void write__int64(int64_t value, int stack);
void write__uint64(uint64_t value, int stack);
void write__float32(float value, int stack);
void write__float64(double value, int stack);
void write__string(std::string value, int stack);
void write__time(ros::Time value, int stack);
void write__duration(ros::Duration value, int stack);

#endif // VREP_ROS_PLUGIN__ROS_MSG_BUILTIN_IO__H
