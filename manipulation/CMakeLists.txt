cmake_minimum_required(VERSION 2.6)
project(manipulation)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")
elseif(COMPILER_SUPPORTS_CXX0X)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

set(CMAKE_PREFIX_PATH "/home/yeshi/opencv-3.1.0/install")
set(OpenCV_FOUND TRUE)
#set(OpenCV_DIR /usr/local/lib)
set(OpenCV_DIR REQUIRED)
find_package(OpenCV REQUIRED)
message(STATUS "OpenCV Version : " ${OpenCV_VERSION})
set(OpenCV_INCLUDE_DIRS "/home/yeshi/opencv-3.1.0/install/include")
message(STATUS "OpenCV_INCLUDE_DIRS : " ${OpenCV_INCLUDE_DIRS})
#set(OpenCV_LIBS "/home/yeshi/opencv-3.1.0/install/lib")
message(STATUS "OpenCV_LIBS : " ${OpenCV_LIBS})

find_package(YARP REQUIRED)
message(STATUS "YARP_INCLUDE_DIRS : " ${YARP_INCLUDE_DIRS})
message(STATUS "YARP_LIBRARIES : " ${YARP_LIBRARIES} )

#find_package(ICUB REQUIRED)
#message(STATUS "ICUB_INCLUDE_DIRS : " ${ICUB_INCLUDE_DIRS})
#message(STATUS "ICUB_LIBRARIES : " ${ICUB_LIBRARIES})

find_package(Boost REQUIRED)
message(STATUS "BOOST_INCLUDE_DIRS : " ${Boost_INCLUDE_DIRS})
message(STATUS "BOOST_LIBRARIES : " ${Boost_LIBRARIES})

include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} -I include/)
#include_directories(${ICUB_INCLUDE_DIRS})

add_executable(manipulation src/main.cpp src/manipulation_module.cpp src/manipulation.cpp )

target_link_libraries(manipulation ${catkin_LIBRARIES} ${OpenCV_LIBS} ${YARP_LIBRARIES} ${Boost_LIBRARIES} -lm -ldl)
#target_link_libraries(manipulation ${ICUB_LIBRARIES})

#install(TARGETS manipulation RUNTIME DESTINATION bin)
