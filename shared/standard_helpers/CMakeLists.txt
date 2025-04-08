cmake_minimum_required(VERSION 3.13)

include(${CMAKE_CURRENT_LIST_DIR}/../../pico_sdk_import.cmake)

project(standard_flight_phase_controller VERSION 1.0.0 DESCRIPTION "Standard helpers for Project Elijah systems" LANGUAGES C CXX)
pico_sdk_init()

add_library(${PROJECT_NAME} INTERFACE)

if (NOT TARGET elijah_state_framework)
    add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../elijah_state_framework elijah_state_framework-build)
endif ()

if (NOT TARGET bmp_280)
    add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../bmp_280 bmp_280-build)
endif ()

if (NOT TARGET mpu_6050)
    add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../mpu_6050 mpu_6050-build)
endif ()

target_link_libraries(${PROJECT_NAME} INTERFACE pico_stdlib elijah_state_framework bmp_280 mpu_6050)


set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER include/${PROJECT_NAME}.h)
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${PROJECT_VERSION})
set_target_properties(${PROJECT_NAME} PROPERTIES SOVERSION ${PROJECT_VERSION_MAJOR})

target_include_directories(${PROJECT_NAME} INTERFACE include)

file(GLOB_RECURSE SRC_CPP CONFIGURE_DEPENDS "src/*.cpp")
file(GLOB_RECURSE SRC_C CONFIGURE_DEPENDS "src/*.c")

target_sources(${PROJECT_NAME} INTERFACE ${SRC_CPP} ${SRC_C})