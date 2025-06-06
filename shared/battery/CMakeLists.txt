cmake_minimum_required(VERSION 3.13)

include(${CMAKE_CURRENT_LIST_DIR}/../../pico_sdk_import.cmake)

project(battery VERSION 1.0.0 DESCRIPTION "Battery voltage reader for Project Elijah" LANGUAGES C CXX)
pico_sdk_init()

add_library(${PROJECT_NAME} INTERFACE)

target_link_libraries(${PROJECT_NAME} INTERFACE pico_stdlib hardware_adc)

set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER include/${PROJECT_NAME}.h)
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${PROJECT_VERSION})
set_target_properties(${PROJECT_NAME} PROPERTIES SOVERSION ${PROJECT_VERSION_MAJOR})

target_include_directories(${PROJECT_NAME} INTERFACE include)

file(GLOB_RECURSE SRC_CPP CONFIGURE_DEPENDS "src/*.cpp")
file(GLOB_RECURSE SRC_C CONFIGURE_DEPENDS "src/*.c")

target_sources(${PROJECT_NAME} INTERFACE ${SRC_CPP} ${SRC_C})