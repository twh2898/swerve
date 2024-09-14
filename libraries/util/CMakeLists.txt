set(UTIL_TARGET util)

set(UTIL_HEADER_LIST
    Config.hpp
    log.hpp
    PID.hpp
    Profiler.hpp
    Telemetry.hpp
    XY.hpp
)
list(TRANSFORM UTIL_HEADER_LIST PREPEND "include/${UTIL_TARGET}/")

set(UTIL_SOURCE_LIST
    Config.cpp
    log.cpp
    PID.cpp
    Profiler.cpp
    Telemetry.cpp
)
list(TRANSFORM UTIL_SOURCE_LIST PREPEND "src/")

add_library(${UTIL_TARGET} OBJECT ${UTIL_SOURCE_LIST} ${UTIL_HEADER_LIST})

target_include_directories(${UTIL_TARGET} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

set_property(TARGET ${UTIL_TARGET} PROPERTY CXX_STANDARD 17)

target_link_libraries(${UTIL_TARGET} PRIVATE json spdlog::spdlog)
