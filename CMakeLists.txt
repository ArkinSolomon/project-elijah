cmake_minimum_required(VERSION 3.20)

# Yeah this gets undefined later idk
if (WIN32)
    set(USING_WINDOWS 1)
endif ()

set(CMAKE_C_STANDARD 17)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(ENTER_USB_BOOT_ON_EXIT "Enter USB boot on exit" OFF)
option(STATUS_SPEAKER_ENABLE "Enable speaker" ON)
option(AB_TEST "Airbrakes test" OFF)

set(PICO_EXTRAS_FETCH_FROM_GIT ON)

find_program(PICOTOOL_INSTALL_LOC "picotool")
if (NOT PICOTOOL_INSTALL_LOC)
    message(FATAL_ERROR "picotool installation not found, is it in your path?")
else ()
    message("Using picotool installation at ${PICOTOOL_INSTALL_LOC}")
endif ()

include(${CMAKE_CURRENT_LIST_DIR}/pico_sdk_import.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/lib/pico_extras_import.cmake)

project(project-elijah C CXX ASM)

pico_sdk_init()

include(FetchContent)
FetchContent_Declare(aprs_pico
        GIT_REPOSITORY "https://github.com/eleccoder/raspi-pico-aprs-tnc.git"
        GIT_TAG main
        GIT_SHALLOW ON
)
FetchContent_MakeAvailable(aprs_pico)
add_library(aprs_pico::aprs_pico ALIAS aprs_pico)

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/lib/no-OS-FatFS-SD-SDIO-SPI-RPi-Pico/src fast-fs-build)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/shared/elijah_state_framework)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/shared/bmp_280)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/shared/mpu_6050)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/shared/battery)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/shared/standard_flight_phase_controller)

function(create_elijah_target SRC_DIR_NAME)
    SET(EXEC_NAME elijah-${SRC_DIR_NAME})

    string(CONCAT CPP_GLOB ${SRC_DIR_NAME} "/*.cpp")
    string(CONCAT C_GLOB ${SRC_DIR_NAME} "/*.c")

    file(GLOB_RECURSE SRC_CPP CONFIGURE_DEPENDS ${CPP_GLOB})
    file(GLOB_RECURSE SRC_C CONFIGURE_DEPENDS ${C_GLOB})
    add_executable(${EXEC_NAME} ${SRC_CPP} ${SRC_C})

    string(CONCAT PIO_GLOB ${SRC_DIR_NAME} "/pio/*.pio")
    file(GLOB_RECURSE SRC_PIO CONFIGURE_DEPENDS ${PIO_GLOB})

    foreach (pio_src_file ${SRC_PIO})
        pico_generate_pio_header(${EXEC_NAME} ${pio_src_file})
    endforeach ()

    pico_set_program_name(${EXEC_NAME} ${EXEC_NAME})
    pico_set_program_version(${EXEC_NAME} "0.1")

    pico_enable_stdio_uart(${EXEC_NAME} 0)
    pico_enable_stdio_usb(${EXEC_NAME} 1)

    target_link_libraries(${EXEC_NAME} pico_stdlib)

    target_include_directories(${EXEC_NAME} PRIVATE
            ${CMAKE_CURRENT_LIST_DIR}/${SRC_DIR_NAME}
    )
    target_include_directories(${EXEC_NAME} PRIVATE ${CMAKE_CURRENT_LIST_DIR}/lib/)

    if (ENTER_USB_BOOT_ON_EXIT)
        target_compile_definitions(${EXEC_NAME} PUBLIC PICO_ENTER_USB_BOOT_ON_EXIT)
    endif (ENTER_USB_BOOT_ON_EXIT)

    if (STATUS_SPEAKER_ENABLE)
        target_compile_definitions(${EXEC_NAME} PRIVATE SPEAKER_ENABLE)
    endif (STATUS_SPEAKER_ENABLE)

    if (AB_TEST)
        target_compile_definitions(${EXEC_NAME} PRIVATE AIRBRAKES_TEST)
    endif (AB_TEST)

    target_compile_definitions(${EXEC_NAME}
            PUBLIC
            $<$<CONFIG:Debug>:DEBUG>
            $<$<CONFIG:Release>:RELEASE>
    )

    if (PICO_RP2040)
        target_compile_definitions(${EXEC_NAME} PRIVATE SYS_CLK_KHZ=200000)
    endif (PICO_RP2040)

    if (PICO_RP2350)
        target_compile_definitions(${EXEC_NAME} PRIVATE SYS_CLK_KHZ=150000)
    endif (PICO_RP2350)

    target_compile_definitions(${EXEC_NAME} PRIVATE CFG_TUD_CDC_TX_BUFSIZE=4096 PICO_STACK_SIZE=4096 PICO_AUDIO_PWM_MONO_DMA_CH=6)

    target_link_libraries(${EXEC_NAME}
            aprs_pico
            hardware_i2c
            hardware_spi
            hardware_pio
            hardware_adc
            hardware_pwm
            hardware_clocks
            pico_audio_pwm
            pico_multicore
            pico_rand
            pico_mem_ops_default
            pico_float_default
            pico_double_default
            pico_int64_ops_default
            pico_aon_timer
            no-OS-FatFS-SD-SDIO-SPI-RPi-Pico
            elijah_state_framework
            bmp_280
            mpu_6050
            battery
            standard_flight_phase_controller
    )

    pico_add_extra_outputs(${EXEC_NAME})

    if (USING_WINDOWS)
        set(INSTALL_FILE_NAME "${CMAKE_CURRENT_BINARY_DIR}/tmp/install.bat")
        file(WRITE ${INSTALL_FILE_NAME} "\"" ${PICOTOOL_INSTALL_LOC} "\" reboot -f -u\ntimeout /NOBREAK /T 5\n\"" ${PICOTOOL_INSTALL_LOC} "\" load \"%1\" -fx")
    else ()
        set(INSTALL_FILE_NAME "${CMAKE_CURRENT_BINARY_DIR}/tmp/install.sh")
        file(WRITE ${INSTALL_FILE_NAME} "#!/bin/sh\n" ${PICOTOOL_INSTALL_LOC} " load \"$1\" -x")
    endif ()
    file(COPY ${INSTALL_FILE_NAME} DESTINATION ${CMAKE_CURRENT_BINARY_DIR} FILE_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
endfunction()

create_elijah_target(payload)
create_elijah_target(override)
create_elijah_target(airbrakes)