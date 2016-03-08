Building the ROS plugin:
=========================

What you need:
    - C++ compiler
    - Python interpreter (2.7 or greater)
    - lxml package for Python
    - v_repStubsGen (available from https://github.com/fferri/v_repStubsGen)
    - An XSLT Processor, such as SAXON (needed for generating documentation)

Build steps:

    1) Edit meta/messages.txt and meta/services.txt if you need to include more ROS messages/services.
       You need to specify the full message/service type, i.e. geometry_msgs/Twist rather than Twist.

    2) Compile the plugin using catkin tools:

        > catkin build

    3) (optional) generate documentation with an XSLT processor:

        > saxon -s:callbacks.xml -a:on -o:reference.html

        The callbacks.xml document can be viewed directly into the browser.
        Any modern browser will do the XSLT processing automatically.


