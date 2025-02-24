cmake_minimum_required(VERSION 3.13)

include(${CMAKE_CURRENT_LIST_DIR}/../../pico_sdk_import.cmake)

project(elijah_state_framework VERSION 1.0.0 DESCRIPTION "State management and communication for Project Elijah" LANGUAGES C CXX)
pico_sdk_init()

add_library(${PROJECT_NAME} INTERFACE)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../shared_mutex shared_mutex-build)

pico_enable_stdio_uart(${PROJECT_NAME} 0)
pico_enable_stdio_usb(${PROJECT_NAME} 1)

target_link_libraries(${PROJECT_NAME} INTERFACE pico_stdlib shared_mutex)

set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER include/${PROJECT_NAME}.h)
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${PROJECT_VERSION})
set_target_properties(${PROJECT_NAME} PROPERTIES SOVERSION ${PROJECT_VERSION_MAJOR})

target_include_directories(${PROJECT_NAME} INTERFACE include)

file(GLOB_RECURSE SRC_CPP CONFIGURE_DEPENDS "src/*.cpp")
file(GLOB_RECURSE SRC_C CONFIGURE_DEPENDS "src/*.c")

target_sources(${PROJECT_NAME} INTERFACE ${SRC_CPP} ${SRC_C})