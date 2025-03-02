cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 17)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
option(ENTER_USB_BOOT_ON_EXIT "Enter USB boot on exit" OFF)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if (WIN32)
    set(USERHOME $ENV{USERPROFILE})
else ()
    set(USERHOME $ENV{HOME})
endif ()
set(sdkVersion 2.1.0)
set(toolchainVersion 13_3_Rel1)
set(picotoolVersion 2.1.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif ()
# ====================================================================================

find_program(PICOTOOL_INSTALL_LOC "picotool")
if (NOT PICOTOOL_INSTALL_LOC)
    message(FATAL_ERROR "picotool installation not found, is it in your path?")
else()
    message("Using picotool installation at ${PICOTOOL_INSTALL_LOC}")
endif()

set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(elijah-payload C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1
file(GLOB_RECURSE elijah_SRC_CPP CONFIGURE_DEPENDS "src/*.cpp")
file(GLOB_RECURSE elijah_SRC_C CONFIGURE_DEPENDS "src/*.c")

add_executable(${PROJECT_NAME} ${elijah_SRC_CPP} ${elijah_SRC_C})
file(GLOB elijah_PIO CONFIGURE_DEPENDS "src/pio/*.pio")

pico_generate_pio_header(${PROJECT_NAME} ${elijah_PIO})

pico_set_program_name(elijah-payload "elijah-payload")
pico_set_program_version(elijah-payload "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(${PROJECT_NAME} 1)
pico_enable_stdio_usb(${PROJECT_NAME} 1)

# Add the standard library to the build
target_link_libraries(${PROJECT_NAME} pico_stdlib)

target_include_directories(${PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/src
)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/lib/no-OS-FatFS-SD-SDIO-SPI-RPi-Pico/src fast-fs-build)
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_LIST_DIR}/lib/)

if (ENTER_USB_BOOT_ON_EXIT)
    target_compile_definitions(${PROJECT_NAME} PUBLIC PICO_ENTER_USB_BOOT_ON_EXIT)
endif (ENTER_USB_BOOT_ON_EXIT)

target_compile_definitions(${PROJECT_NAME}
        PUBLIC
        $<$<CONFIG:Debug>:DEBUG>
        $<$<CONFIG:Release>:RELEASE>
)

target_compile_definitions(${PROJECT_NAME} PRIVATE W25Q64FV_FLASH_SIM)

# Add any user requested libraries
target_link_libraries(${PROJECT_NAME}
        hardware_i2c
        hardware_spi
        hardware_pio
        hardware_adc
        pico_multicore
        pico_rand
        pico_mem_ops_default
        pico_float_default
        pico_double_default
        pico_int64_ops_default
        pico_aon_timer
        no-OS-FatFS-SD-SDIO-SPI-RPi-Pico
)

pico_add_extra_outputs(${PROJECT_NAME})

file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/tmp/install.sh "#!/bin/sh\n" ${PICOTOOL_INSTALL_LOC} " load \"$1\" -fx")
file(COPY ${CMAKE_CURRENT_BINARY_DIR}/tmp/install.sh DESTINATION ${CMAKE_CURRENT_BINARY_DIR} FILE_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)


