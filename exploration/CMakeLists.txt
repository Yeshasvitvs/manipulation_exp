cmake_minimum_required(VERSION 2.6)
project(exploration)

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

find_package(YARP REQUIRED)
message(STATUS "YARP_INCLUDE_DIRS : " ${YARP_INCLUDE_DIRS})
message(STATUS "YARP_LIBRARIES : " ${YARP_LIBRARIES} )

find_package(ICUB REQUIRED)
message(STATUS "ICUB_INCLUDE_DIRS : " ${ICUB_INCLUDE_DIRS})
message(STATUS "ICUB_LIBRARIES : " ${ICUB_LIBRARIES})

find_package(Boost REQUIRED)
message(STATUS "BOOST_INCLUDE_DIRS : " ${Boost_INCLUDE_DIRS})
message(STATUS "BOOST_LIBRARIES : " ${Boost_LIBRARIES})

include_directories(${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} -I include/)

add_executable(exploration src/main.cpp src/exploration_module.cpp src/exploration.cpp)

target_link_libraries(exploration ${YARP_LIBRARIES} ${ICUB_LIBRARIES} ${Boost_LIBRARIES} -lm -ldl)

#install(TARGETS exploration RUNTIME DESTINATION bin)
