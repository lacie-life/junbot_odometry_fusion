cmake_minimum_required(VERSION 3.0.2)
project(junbot_odometry_fusion)


find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        nav_msgs
        geometry_msgs
        image_transport
        cv_bridge
        tf
        )

find_package(OpenCV 4.5 REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED NO_MODULE)
find_package(yaml-cpp REQUIRED)


catkin_package(
        INCLUDE_DIRS include
        LIBRARIES junbot_relocalization
        CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs nav_msgs tf image_transport cv_bridge
#       DEPENDS system_lib
)


include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${OpenCV_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${catkin_INCLUDE_DIRS}
        ${YAML_CPP_INCLUDE_DIR}
)


file(GLOB HEADERS
        ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
        )
file(GLOB SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
        )

source_group("Header Files" FILES ${HEADERS})
source_group("Source Files" FILES ${SOURCES})

add_executable(junbot_odometry_fusion
        ${HEADERS}
        ${SOURCES}
        )

target_link_libraries(junbot_odometry_fusion
        ${OpenCV_LIBS}
        ${catkin_LIBRARIES}
        yaml-cpp
)

install(DIRECTORY
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE)

install(TARGETS junbot_odometry_fusion
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/data
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})


