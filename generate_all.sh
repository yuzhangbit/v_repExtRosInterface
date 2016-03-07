set -e

cat /dev/null > generated/ros_msg_io.h
cat /dev/null > generated/ros_msg_io.cpp
cat /dev/null > generated/sub.cpp
cat /dev/null > generated/adv.cpp
cat /dev/null > generated/pub.cpp

cat >> generated/ros_msg_io.cpp <<EOF
#include <ros_msg_builtin_io.h>
#include <ros_msg_io.h>
#include <v_repLib.h>

EOF

cat >> generated/ros_msg_io.h <<EOF
#ifndef VREP_ROS_PLUGIN__ROS_MSG_IO__H
#define VREP_ROS_PLUGIN__ROS_MSG_IO__H

#include <ros/ros.h>
#include <vrep_ros_plugin.h>

EOF

l=messages.txt

cat $l | while read msg; do
    echo "#include <$msg.h>" >> generated/ros_msg_io.h
done

echo "" >> generated/ros_msg_io.h

cat $l | while read msg; do
    echo Generating code for message $msg...
    rosmsg show -r $msg > generated/tmp.msg
    python generate_msg.py h generated/tmp.msg $msg $l >> generated/ros_msg_io.h
    python generate_msg.py cpp generated/tmp.msg $msg $l >> generated/ros_msg_io.cpp
    python generate_msg.py sub generated/tmp.msg $msg $l >> generated/sub.cpp
    python generate_msg.py adv generated/tmp.msg $msg $l >> generated/adv.cpp
    python generate_msg.py pub generated/tmp.msg $msg $l >> generated/pub.cpp
done

rm generated/tmp.msg

cat >> generated/ros_msg_io.h <<EOF

#endif // VREP_ROS_PLUGIN__ROS_MSG_IO__H
EOF
