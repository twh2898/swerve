set(UTIL_TARGET util)

file(GLOB HEADER_LIST include/**/*.hpp)

file(GLOB SOURCE_LIST src/*.cpp)

add_library(${UTIL_TARGET} OBJECT ${SOURCE_LIST} ${HEADER_LIST} include/rclcpp/macros.hpp)

target_include_directories(${UTIL_TARGET} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

set_property(TARGET ${UTIL_TARGET} PROPERTY CXX_STANDARD 17)

target_link_libraries(${UTIL_TARGET} PRIVATE json spdlog::spdlog)
