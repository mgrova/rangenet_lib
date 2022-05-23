cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(rangenet_lib VERSION 0.1 LANGUAGES C CXX)

option(BUILD_EXAMPLES "Building example" ON)

if( NOT CMAKE_BUILD_TYPE )
  set( CMAKE_BUILD_TYPE Release CACHE STRING
       "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
       FORCE )
endif()

set(CMAKE_CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-g -O3") 

#############################
######	  Library	     ######
#############################
file(GLOB_RECURSE HDRS "include/*.h*")
file(GLOB_RECURSE SRCS "src/*.c*")

add_library(${PROJECT_NAME} SHARED ${SRCS} ${HDRS})

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE C)

#############################
######	Third Party	   ######
#############################
find_package(yaml-cpp REQUIRED)
if(${yaml-cpp_FOUND})
  target_include_directories(${PROJECT_NAME} PUBLIC ${YAML_CPP_INCLUDE_DIR})
  target_link_libraries(${PROJECT_NAME} PUBLIC ${YAML_CPP_LIBRARIES})
endif()

find_package(Boost REQUIRED COMPONENTS program_options filesystem)
if(${Boost_FOUND})
  target_include_directories(${PROJECT_NAME} PUBLIC ${Boost_INCLUDE_DIRS})
  target_link_libraries(${PROJECT_NAME} PUBLIC ${Boost_LIBRARIES})
endif()

# first try to find cuda and tensorrt-related stuff (nvinfer, onnx, etc)
find_package(TensorRT REQUIRED PATHS cmake)
if (${TENSORRT_FOUND})
  target_include_directories(${PROJECT_NAME} PUBLIC ${TENSORRT_INCLUDE_DIRS})
  target_link_libraries(${PROJECT_NAME} PUBLIC ${TENSORRT_LIBRARIES})
endif (${TENSORRT_FOUND})

#############################
######	  Examples	   ######
#############################
if(BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()



