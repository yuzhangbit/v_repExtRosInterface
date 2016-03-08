Building the ROS plugin:
=========================

What you need:
    - C++ compiler
    - Python interpreter (2.7 or greater)
    - lxml package for Python
    - v_repStubsGen (available from https://github.com/fferri/v_repStubsGen)
    - An XSLT Processor, such as SAXON (needed for generating documentation)

Build steps:

    1) Generate stubs for Lua callbacks:

        > mkdir generated
        > cd generated
        > python /path/to/v_repStubsGen/main-stacks.py -H stubs.h -C stubs.cpp -I vrep_ros_plugin.h ../callbacks.xml
        > cd ..

    2) Edit messages.txt and services.txt if you need to include more ROS messages/services.
       You need to specify the full message/service type, i.e. geometry_msgs/Twist rather than Twist.

    3) Generate code for reading/writing ROS messages:

        > python gen_ros_stuff.py messages.txt services.txt

    4) Now, in the directory generated/ you should have the following files:

        - stubs.cpp
        - stubs.h
        - adv.cpp
        - pub.cpp
        - sub.cpp
        - ros_msg_io.cpp
        - ros_msg_io.h
        - ros_srv_io.cpp
        - ros_srv_io.h
        - srvcall.cpp
        - srvcli.cpp
        - srvsrv.cpp

    5) Compile the plugin using catkin tools:

        > catkin build

    6) (optional) generate documentation with an XSLT processor:

        > saxon -s:callbacks.xml -a:on -o:reference.html

        The callbacks.xml document can be viewed directly into the browser.
        Any modern browser will do the XSLT processing automatically.


