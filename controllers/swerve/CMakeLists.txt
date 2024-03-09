
set(TARGET swerve)

## =============================================================================
## Utilities
## -----------------------------------------------------------------------------

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
list(TRANSFORM UTIL_SOURCE_LIST PREPEND "src/${UTIL_TARGET}/")

add_library(${UTIL_TARGET} OBJECT ${UTIL_SOURCE_LIST} ${UTIL_HEADER_LIST})

target_include_directories(${UTIL_TARGET} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

set_property(TARGET ${UTIL_TARGET} PROPERTY CXX_STANDARD 17)

## =============================================================================
## Controller
## -----------------------------------------------------------------------------

set(HEADER_LIST
)
list(TRANSFORM HEADER_LIST PREPEND "include/${TARGET}/")

set(SOURCE_LIST
    main.cpp
)
list(TRANSFORM SOURCE_LIST PREPEND "src/")

add_executable(${TARGET}
    ${SOURCE_LIST}
    ${HEADER_LIST}
    include/json.hpp
)

target_include_directories(${TARGET} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

set_property(TARGET ${TARGET} PROPERTY CXX_STANDARD 17)

target_link_libraries(${TARGET}
    $<TARGET_OBJECTS:${UTIL_TARGET}>
    # $<TARGET_OBJECTS:${BASE_TARGET}>
    spdlog::spdlog
    ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX}
    ${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX}
)

## =============================================================================
## Post-Build: Copy Executable to Controller Directory
## -----------------------------------------------------------------------------

add_custom_command(TARGET ${TARGET} POST_BUILD COMMAND ${CMAKE_COMMAND} -E
    copy ${CMAKE_BINARY_DIR}/controllers/${TARGET}/${TARGET} ${CMAKE_CURRENT_SOURCE_DIR}
)
