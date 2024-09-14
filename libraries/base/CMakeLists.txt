set(BASE_TARGET base)

set(BASE_HEADER_LIST
    Motor.hpp
)
list(TRANSFORM BASE_HEADER_LIST PREPEND "include/${BASE_TARGET}/")

set(BASE_SOURCE_LIST
    Motor.cpp
)
list(TRANSFORM BASE_SOURCE_LIST PREPEND "src/")

add_library(${BASE_TARGET} OBJECT ${BASE_SOURCE_LIST} ${BASE_HEADER_LIST})

target_include_directories(${BASE_TARGET} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

set_property(TARGET ${BASE_TARGET} PROPERTY CXX_STANDARD 17)

target_link_libraries(${BASE_TARGET}
    json
    util
)
