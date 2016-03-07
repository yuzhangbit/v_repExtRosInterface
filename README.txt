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

        > cd generated
        > python /path/to/v_repStubsGen/main-stacks.py -H stubs.h -C stubs.cpp -I vrep_ros_plugin.h ../callbacks.xml
        > cd ..

    2) Edit messages.txt if you need to include more ROS messages.

    3) Generate code for reading/writing ROS messages:

        > python generate_msg.py messages.txt

    4) Now, in the directory generated/ you should have the following files:

        - stubs.cpp
        - stubs.h
        - adv.cpp
        - pub.cpp
        - sub.cpp
        - ros_msg_io.cpp
        - ros_msg_io.h

    5) Compile the plugin using catkin tools:

        > catkin build

    6) (optional) generate documentation with an XSLT processor:

        > saxon -s:callbacks.xml -a:on -o:reference.html

