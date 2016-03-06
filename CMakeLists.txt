cmake_minimum_required(VERSION 2.8.3)
project(vrep_ros_plugin)

find_package(catkin REQUIRED COMPONENTS
  roscpp image_transport tf vrep_common
)

catkin_package(
               CATKIN_DEPENDS roscpp rosconsole cv_bridge image_transport tf brics_actuator roslib actionlib_msgs control_msgs diagnostic_msgs geometry_msgs map_msgs nav_msgs pcl_msgs sensor_msgs shape_msgs std_msgs tf2_geometry_msgs tf2_msgs tf2_sensor_msgs trajectory_msgs visualization_msgs
               DEPENDS roscpp rosconsole cv_bridge image_transport tf brics_actuator roslib actionlib_msgs control_msgs diagnostic_msgs geometry_msgs map_msgs nav_msgs pcl_msgs sensor_msgs shape_msgs std_msgs tf2_geometry_msgs tf2_msgs tf2_sensor_msgs trajectory_msgs visualization_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})

link_directories("/opt/ros/fuerte/lib")
link_directories("/opt/ros/groovy/lib")
link_directories("/opt/ros/hydro/lib")
link_directories("/opt/ros/indigo/lib")
link_directories("/opt/ros/jade/lib")

add_library(v_repExtRos src/vrep_ros_plugin.cpp src/scriptFunctionData.cpp src/scriptFunctionDataItem.cpp src/v_repLib.cpp)

add_dependencies(v_repExtRos ${catkin_EXPORTED_TARGETS})
target_link_libraries(v_repExtRos
    pthread
    dl
    roslib
    rosconsole
    rostime
    rospack
    roscpp_serialization
    roscpp
    tf
    image_transport
    boost_system
   ${catkin_LIBRARIES}
)
