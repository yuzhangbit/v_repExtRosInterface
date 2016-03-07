set -e

cat /dev/null > include/vrep_ros_plugin/ros_msg_io.h
cat /dev/null > src/ros_msg_io.cpp
cat /dev/null > include/vrep_ros_plugin/generated_sub.cpp
cat /dev/null > include/vrep_ros_plugin/generated_adv.cpp
cat /dev/null > include/vrep_ros_plugin/generated_pub.cpp

cat >> src/ros_msg_io.cpp <<EOF
#include <vrep_ros_plugin/ros_msg_builtin_io.h>
#include <vrep_ros_plugin/ros_msg_io.h>
#include "../include/v_repLib.h"

EOF

cat >> include/vrep_ros_plugin/ros_msg_io.h <<EOF
#ifndef VREP_ROS_PLUGIN__ROS_MSG_IO__H
#define VREP_ROS_PLUGIN__ROS_MSG_IO__H

#include <ros/ros.h>
#include <vrep_ros_plugin/vrep_ros_plugin.h>

EOF

l=messages.txt

cat $l | while read msg; do
    echo "#include <$msg.h>" >> include/vrep_ros_plugin/ros_msg_io.h
done

echo "" >> include/vrep_ros_plugin/ros_msg_io.h

cat $l | while read msg; do
    echo Generating code for message $msg...
    rosmsg show -r $msg > tmp.msg
    python generate_msg.py h tmp.msg $msg $l >> include/vrep_ros_plugin/ros_msg_io.h
    python generate_msg.py cpp tmp.msg $msg $l >> src/ros_msg_io.cpp
    python generate_msg.py sub tmp.msg $msg $l >> include/vrep_ros_plugin/generated_sub.cpp
    python generate_msg.py adv tmp.msg $msg $l >> include/vrep_ros_plugin/generated_adv.cpp
    python generate_msg.py pub tmp.msg $msg $l >> include/vrep_ros_plugin/generated_pub.cpp
done

rm tmp.msg

cat >> include/vrep_ros_plugin/ros_msg_io.h <<EOF

#endif // VREP_ROS_PLUGIN__ROS_MSG_IO__H
EOF
