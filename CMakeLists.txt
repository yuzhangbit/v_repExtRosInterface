cmake_minimum_required(VERSION 2.8.3)
project(vrep_ros_interface)

find_package(catkin REQUIRED COMPONENTS
  roscpp rosmsg image_transport tf cv_bridge
)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/modules/")
find_package(VREP REQUIRED)

catkin_package(
   CATKIN_DEPENDS roscpp rosconsole cv_bridge image_transport tf brics_actuator roslib actionlib_msgs control_msgs diagnostic_msgs geometry_msgs map_msgs nav_msgs pcl_msgs sensor_msgs shape_msgs std_msgs tf2_geometry_msgs tf2_msgs tf2_sensor_msgs trajectory_msgs visualization_msgs
   DEPENDS roscpp rosconsole cv_bridge image_transport tf brics_actuator roslib actionlib_msgs control_msgs diagnostic_msgs geometry_msgs map_msgs nav_msgs pcl_msgs sensor_msgs shape_msgs std_msgs tf2_geometry_msgs tf2_msgs tf2_sensor_msgs trajectory_msgs visualization_msgs)

file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/generated)
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/doc)

include_directories(include ${CMAKE_BINARY_DIR}/generated ${catkin_INCLUDE_DIRS} $ENV{VREP_ROOT}/programming/include $ENV{VREP_ROOT}/programming/common)

link_directories("/opt/ros/fuerte/lib")
link_directories("/opt/ros/groovy/lib")
link_directories("/opt/ros/hydro/lib")
link_directories("/opt/ros/indigo/lib")
link_directories("/opt/ros/jade/lib")
link_directories("/opt/ros/kinetic/lib")

set(generatedFiles)
file(GLOB templateFiles RELATIVE ${CMAKE_SOURCE_DIR}/templates/ ${CMAKE_SOURCE_DIR}/templates/*)
foreach(templateFile ${templateFiles})
    add_custom_command(OUTPUT "${CMAKE_BINARY_DIR}/generated/${templateFile}" COMMAND python "${CMAKE_SOURCE_DIR}/external/v_repStubsGen/external/pycpp/pycpp.py" -p "messages_file=${CMAKE_SOURCE_DIR}/meta/messages.txt" -p "services_file=${CMAKE_SOURCE_DIR}/meta/services.txt" -i "${CMAKE_SOURCE_DIR}/templates/${templateFile}" -o "${CMAKE_BINARY_DIR}/generated/${templateFile}" -P "${CMAKE_SOURCE_DIR}/tools" DEPENDS "${CMAKE_SOURCE_DIR}/meta/messages.txt" "${CMAKE_SOURCE_DIR}/meta/services.txt" "${CMAKE_SOURCE_DIR}/templates/${templateFile}")
    set(generatedFiles ${generatedFiles} "${CMAKE_BINARY_DIR}/generated/${templateFile}")
endforeach()
add_custom_target(generate_ros_code DEPENDS ${generatedFiles})

if((EXISTS ${CMAKE_SOURCE_DIR}/include/stubs.h) AND (EXISTS ${CMAKE_SOURCE_DIR}/src/stubs.cpp) AND (EXISTS ${CMAKE_SOURCE_DIR}/doc/reference.html))
    add_library(v_repExtRosInterface src/vrep_ros_interface.cpp src/stubs.cpp src/ros_msg_builtin_io.cpp src/v_repLib_inc.cpp generated/ros_msg_io.cpp generated/ros_srv_io.cpp)
else()
  MESSAGE(STATUS "*** Generating build files ***")
  add_custom_command(OUTPUT
    ${CMAKE_BINARY_DIR}/generated/stubs.cpp ${CMAKE_BINARY_DIR}/generated/stubs.h ${CMAKE_SOURCE_DIR}/doc/reference.html
    COMMAND python "${CMAKE_SOURCE_DIR}/external/v_repStubsGen/generate.py" --xml-file "${CMAKE_SOURCE_DIR}/meta/callbacks.xml" --gen-all ${CMAKE_BINARY_DIR}/generated/
    DEPENDS ${CMAKE_SOURCE_DIR}/meta/callbacks.xml)
  add_library(v_repExtRosInterface src/vrep_ros_interface.cpp generated/stubs.cpp src/ros_msg_builtin_io.cpp src/v_repLib_inc.cpp generated/ros_msg_io.cpp generated/ros_srv_io.cpp)
endif()
add_dependencies(v_repExtRosInterface ${catkin_EXPORTED_TARGETS} generate_ros_code)
target_link_libraries(v_repExtRosInterface
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
