# Qt Custom User Interface plugin for V-REP

### Compiling

_NOTE:_ the directory containing all files (i.e. package.xml etc) must be called vrep_ros_interface, otherwise build will fail.

1. Install required python packages for [v_repStubsGen](https://github.com/fferri/v_repStubsGen): see v_repStubsGen's [README](external/v_repStubsGen/README.md)
2. Install `xsltproc` (an XSLT processor)
3. Download and install Qt (same version as V-REP, i.e. 5.5.0)
4. Checkout
```
$ git clone --recursive https://github.com/fferri/v_repExtRosInterface.git vrep_ros_interface
```
5. Edit `meta/messages.txt` and `meta/services.txt` if you need to include more ROS messages/services. You need to specify the full message/service type, i.e. geometry_msgs/Twist rather than Twist.
6. Compile
```
$ catkin build
```
