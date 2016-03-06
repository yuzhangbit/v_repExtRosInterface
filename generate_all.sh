set -e

cat /dev/null > include/ros_msg_io.h
cat /dev/null > src/ros_msg_io.cpp
cat /dev/null > include/generated_sub.cpp
cat /dev/null > include/generated_adv.cpp
cat /dev/null > include/generated_pub.cpp

cat > src/ros_msg_io.cpp <<EOF
#include <vrep_ros_plugin/ros_msg_builtin_io.h>
#include <vrep_ros_plugin/ros_msg_io.h>
EOF

l=messages.txt

cat $l | while read msg; do
    echo Generating code for message $msg...
    rosmsg show -r $msg > tmp.msg
    python generate_msg.py h tmp.msg $msg $l >> include/ros_msg_io.h
    python generate_msg.py cpp tmp.msg $msg $l >> src/ros_msg_io.cpp
    python generate_msg.py sub tmp.msg $msg $l >> generated_sub.cpp
    python generate_msg.py adv tmp.msg $msg $l >> generated_adv.cpp
    python generate_msg.py pub tmp.msg $msg $l >> generated_pub.cpp
done

rm tmp.msg
