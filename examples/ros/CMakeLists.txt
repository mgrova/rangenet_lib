
find_package(catkin REQUIRED roscpp geometry_msgs sensor_msgs)
if(${catkin_FOUND})
    catkin_package(
        CATKIN_DEPENDS roscpp geometry_msgs sensor_msgs
        INCLUDE_DIRS include
    )

    add_executable(ros_node src/*.cpp src/*.hpp)

    target_include_directories(ros_node PUBLIC ${PROJECT_NAME} ${catkin_INCLUDE_DIRS})
    target_link_libraries(ros_node LINK_PUBLIC ${PROJECT_NAME} ${catkin_LIBRARIES})
    add_dependencies(ros_node ${catkin_EXPORTED_TARGETS})
endif()