set(BASE_TARGET base)

file(GLOB HEADER_LIST include/**/*.hpp)

file(GLOB SOURCE_LIST src/*.cpp)

add_library(${BASE_TARGET} OBJECT ${SOURCE_LIST} ${HEADER_LIST})

target_include_directories(${BASE_TARGET} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

set_property(TARGET ${BASE_TARGET} PROPERTY CXX_STANDARD 17)

target_link_libraries(${BASE_TARGET}
    json
    util
)
