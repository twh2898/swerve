set(UTIL_TARGET util)

set(UTIL_HEADER_LIST
    log.hpp
    PID.hpp
    Profiler.hpp
    ramp.hpp
    sim_time.hpp
    Telemetry.hpp
    XY.hpp
)
list(TRANSFORM UTIL_HEADER_LIST PREPEND "include/${UTIL_TARGET}/")

set(UTIL_SOURCE_LIST
    log.cpp
    PID.cpp
    Profiler.cpp
    sim_time.cpp
    Telemetry.cpp
)
list(TRANSFORM UTIL_SOURCE_LIST PREPEND "src/")

add_library(${UTIL_TARGET} OBJECT ${UTIL_SOURCE_LIST} ${UTIL_HEADER_LIST} include/rclcpp/macros.hpp)

target_include_directories(${UTIL_TARGET} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

set_property(TARGET ${UTIL_TARGET} PROPERTY CXX_STANDARD 17)

target_link_libraries(${UTIL_TARGET} PRIVATE json spdlog::spdlog)
