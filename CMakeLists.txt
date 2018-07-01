cmake_minimum_required(VERSION 2.8.3)
project(vrep_ros_interface)

set(CMAKE_CXX_STANDARD 11)
#set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DDEBUG")

set(PKG_DEPS
  roscpp
  rosconsole
  cv_bridge
  image_transport
  tf brics_actuator
  roslib
  brics_actuator
  actionlib_msgs
  control_msgs
  diagnostic_msgs
  geometry_msgs
  map_msgs
  nav_msgs
  pcl_msgs
  sensor_msgs
  shape_msgs
  std_msgs
  tf2_geometry_msgs
  tf2_msgs
  tf2_sensor_msgs
  trajectory_msgs
  visualization_msgs)

find_package(catkin REQUIRED COMPONENTS ${PKG_DEPS}
)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules/")
find_package(VREP REQUIRED)

catkin_package(
    CATKIN_DEPENDS ${PKG_DEPS}
)

file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/generated)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/doc)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/external
  ${CMAKE_CURRENT_BINARY_DIR}/generated
  ${catkin_INCLUDE_DIRS}
  ${VREP_INCLUDE}
  ${VREP_COMMON})

link_directories("/opt/ros/fuerte/lib")
link_directories("/opt/ros/groovy/lib")
link_directories("/opt/ros/hydro/lib")
link_directories("/opt/ros/indigo/lib")
link_directories("/opt/ros/jade/lib")
link_directories("/opt/ros/kinetic/lib")
link_directories("/opt/ros/melodic/lib")

set(generatedFiles)
file(GLOB templateFiles RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/templates/ ${CMAKE_CURRENT_SOURCE_DIR}/templates/*)
foreach(templateFile ${templateFiles})
    add_custom_command(OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/generated/${templateFile}" COMMAND python "${CMAKE_CURRENT_SOURCE_DIR}/external/v_repStubsGen/external/pycpp/pycpp.py" -p "messages_file=${CMAKE_CURRENT_SOURCE_DIR}/meta/messages.txt" -p "services_file=${CMAKE_CURRENT_SOURCE_DIR}/meta/services.txt" -i "${CMAKE_CURRENT_SOURCE_DIR}/templates/${templateFile}" -o "${CMAKE_CURRENT_BINARY_DIR}/generated/${templateFile}" -P "${CMAKE_CURRENT_SOURCE_DIR}/tools" DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/meta/messages.txt" "${CMAKE_CURRENT_SOURCE_DIR}/meta/services.txt" "${CMAKE_CURRENT_SOURCE_DIR}/templates/${templateFile}")
    set(generatedFiles ${generatedFiles} "${CMAKE_CURRENT_BINARY_DIR}/generated/${templateFile}")
endforeach()
add_custom_target(generate_ros_code DEPENDS ${generatedFiles})
vrep_generate_stubs(${CMAKE_CURRENT_BINARY_DIR}/generated XML_FILE ${CMAKE_CURRENT_SOURCE_DIR}/meta/callbacks.xml LUA_FILE ${CMAKE_CURRENT_SOURCE_DIR}/simExtRosInterface.lua)

set(SOURCES
    src/vrep_ros_interface.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/external/v_repPlusPlus/Plugin.cpp
    ${CMAKE_CURRENT_BINARY_DIR}/generated/stubs.cpp
    src/ros_msg_builtin_io.cpp
    ${VREP_EXPORTED_SOURCES}
    ${CMAKE_CURRENT_BINARY_DIR}/generated/ros_msg_io.cpp
    ${CMAKE_CURRENT_BINARY_DIR}/generated/ros_srv_io.cpp
)
add_library(v_repExtRosInterface ${SOURCES})
add_dependencies(v_repExtRosInterface ${catkin_EXPORTED_TARGETS} generate_ros_code)
target_link_libraries(v_repExtRosInterface
    pthread
    ${catkin_LIBRARIES}
    ${VREP_LIBRARIES}
)
## install the library to VREP_ROOT folder
install(TARGETS v_repExtRosInterface DESTINATION ${VREP_ROOT})
