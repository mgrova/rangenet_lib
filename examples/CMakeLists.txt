
if (${BUILD_SIMPLE_EXAMPLE})
find_package(OpenCV REQUIRED COMPONENTS core viz)
    if(${OpenCV_FOUND})
        message("Building simple inference example...")
        add_executable(simple_infer simple_infer/infer.cpp)
        target_link_libraries(simple_infer PRIVATE ${PROJECT_NAME} ${OpenCV_LIBS})
    endif()
endif()

if (${BUILD_ROS_NODE})
    file(GLOB_RECURSE SRCS "ros/src/*.c*" "ros/src/*.h*")
    add_executable(ros_node ${SRCS})

    target_include_directories(ros_node PRIVATE ${PROJECT_NAME} ${catkin_INCLUDE_DIRS})
    target_link_libraries(ros_node PRIVATE ${PROJECT_NAME} ${catkin_LIBRARIES})
    add_dependencies(ros_node ${catkin_EXPORTED_TARGETS})
endif()
