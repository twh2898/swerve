cmake_minimum_required(VERSION 3.25)

project(swerve
    VERSION 0.1.0
    DESCRIPTION "Swerve drive in webots"
    LANGUAGES CXX)

set(WEBOTS_HOME "/usr/local/webots")

add_subdirectory(external)

link_directories(${WEBOTS_HOME}/lib/controller)
include_directories(${WEBOTS_HOME}/include/controller/c ${WEBOTS_HOME}/include/controller/cpp)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/libraries)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/controllers)
