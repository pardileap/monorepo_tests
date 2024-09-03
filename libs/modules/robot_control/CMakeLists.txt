cmake_minimum_required(VERSION 3.10) 
project(robot_control)

#Set the C++ standard
set(CMAKE_CXX_STANDARD 17) 
set(CMAKE_CXX_STANDARD_REQUIRED True)

#Find the Ruckig package if it has a config file installed
#find_package(PACKAGE_NAME REQUIRED)


#Include the directories where the header files are located
include_directories(include)

# Glob all files in the src directory 
file(GLOB_RECURSE SOURCES "src/*.*pp" "include/*.h*")

#Add the executable that we want to compile
add_executable(${PROJECT_NAME}_app ${SOURCES})

#Link against the Ruckig library
target_link_libraries(${PROJECT_NAME}_app PRIVATE soem)

option(ENABLE_TESTS "Enable building tests" OFF)

add_subdirectory(googletest)

if (ENABLE_TESTS) 
    add_subdirectory(test)
endif()