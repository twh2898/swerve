
set(TARGET swerve)

## =============================================================================
## Controller
## -----------------------------------------------------------------------------

file(GLOB HEADER_LIST include/**/*.hpp)

file(GLOB SOURCE_LIST src/*.cpp)

add_executable(${TARGET}
    ${SOURCE_LIST}
    ${HEADER_LIST}
)

target_include_directories(${TARGET} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

set_property(TARGET ${TARGET} PROPERTY CXX_STANDARD 17)

target_link_libraries(${TARGET}
    json
    util
    base
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
