Building the ROS plugin:
=========================

What you need:
    - C++ compiler
    - xsltproc

Build steps:

    NOTE: the directory containing all files (i.e. package.xml etc) must be called
          vrep_ros_interface, otherwise build will fail.

    1) Edit meta/messages.txt and meta/services.txt if you need to include more ROS messages/services.
       You need to specify the full message/service type, i.e. geometry_msgs/Twist rather than Twist.

    2) Compile the plugin using catkin tools:

        > catkin build

