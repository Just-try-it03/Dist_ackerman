cmake_minimum_required(VERSION 2.8)

project(zmq_server)


set(__DEBUG 1)

if(__DEBUG)
    SET(CMAKE_BUILD_TYPE "Debug")
    add_compile_options(-O0)
    add_compile_options(-g -lmcheck -ggdb)
else()
    #add_compile_options(-O2)
    SET(CMAKE_BUILD_TYPE "Release")
	#SET(CMAKE_BUILD_TYPE "RelWithDebInfo")
    #add_compile_options(-O2)
    #add_definitions(-O2)
    #set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")
endif()

message("PROJECT_SOURCE_DIR: ${PROJECT_SOURCE_DIR}")

LIST(APPEND CMAKE_PREFIX_PATH /opt/ros/melodic /opt/ros/noetic)
#SET(CATKIN_ENABLE_TESTING OFF)
#FIND_PACKAGE(catkin REQUIRED COMPONENTS roscpp rospy std_msgs)
#catkin_package()
#INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS})
#SET(ROS_LIBS ${catkin_LIBRARIES})
#MESSAGE(STATUS "ROS_LIBS : " ${ROS_LIBS})
# ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/ros/noetic/lib
# messages
#INCLUDE_DIRECTORIES(${EXT_DIR} ${EXT_DIR}/jarvis_msgs)

find_package(catkin REQUIRED
        #COMPONENTS
        roscpp
        #rosbag
        std_msgs
        sensor_msgs
        laser_geometry
        #tf
        )




INCLUDE_DIRECTORIES(
        ${catkin_INCLUDE_DIRS}
)


INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/master/)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/slave/)

FILE(GLOB_RECURSE HDRS ${PROJECT_SOURCE_DIR}/src/*.h*  ${PROJECT_SOURCE_DIR}/*.h*)
FILE(GLOB_RECURSE SRCS ${PROJECT_SOURCE_DIR}/src/*.c*  )

# message("HDRS: ${HDRS}")
# message("SRCS: ${SRCS}")


# build
ADD_EXECUTABLE(zmq_master master/server_node.cpp ${HDRS} ${SRCS})
#TARGET_LINK_LIBRARIES(jarvis-g ${EXT_LIBS} ${ROS_LIBS} dl  ${catkin_LIBRARIES} ${PCL_LIBRARIES})
TARGET_LINK_LIBRARIES(zmq_master
#${EXT_LIBS}
${ROS_LIBS} 
#dl
${catkin_LIBRARIES}
zmq
pthread
)

SET_TARGET_PROPERTIES(zmq_master PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")


# build
ADD_EXECUTABLE(zmq_slave slave/client_node.cpp ${HDRS} ${SRCS})
#TARGET_LINK_LIBRARIES(jarvis-g ${EXT_LIBS} ${ROS_LIBS} dl  ${catkin_LIBRARIES} ${PCL_LIBRARIES})
TARGET_LINK_LIBRARIES(zmq_slave
        #${EXT_LIBS}
        ${ROS_LIBS}
        #dl
        ${catkin_LIBRARIES}
        zmq
        pthread
)

SET_TARGET_PROPERTIES(zmq_slave PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")

# build slave1
ADD_EXECUTABLE(zmq_slave1 slave/client_node1.cpp ${HDRS} ${SRCS})
TARGET_LINK_LIBRARIES(zmq_slave1
        #${EXT_LIBS}
        ${ROS_LIBS}
        #dl
        ${catkin_LIBRARIES}
        zmq
        pthread
)

SET_TARGET_PROPERTIES(zmq_slave1 PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")

#ADD_CUSTOM_COMMAND(
#		TARGET zmq_server POST_BUILD
#                COMMAND ${CMAKE_COMMAND} -E copy ${EXECUTABLE_OUTPUT_PATH}/zmq_server ${EXECUTABLE_OUTPUT_PATH}/zmq_server.v.${JARVIS_VERSION})


