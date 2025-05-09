cmake_minimum_required(VERSION 3.13)

# define "w" board to use cyw43_arch_xxx() function
if(PICO_BOARD)
    unset(PICO_BOARD)
endif()
if(PICO_PLATFORM STREQUAL rp2350)
    set(PICO_BOARD pico2_w)
else()
    set(PICO_BOARD pico_w)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
include($ENV{PICO_EXTRAS_PATH}/external/pico_extras_import.cmake)

set(project_name "pico_spdif_recorder" C CXX ASM)
project(${project_name})
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_subdirectory(lib/pico_spdif_rx pico_spdif_rx)
add_subdirectory(lib/pico_audio_i2s_32b/src/pico_audio_32b pico_audio_32b)
add_subdirectory(lib/pico_fatfs_customized pico_fatfs_customized)
add_subdirectory(lib/pico_ntp_client pico_ntp_client)
add_subdirectory(lib/pico_flash_param pico_flash_param)
add_subdirectory(lib/pico_flash_param/samples/lib/picow_config_wifi picow_config_wifi)

add_executable(${PROJECT_NAME}
    src/main.cpp
    src/spdif_rec_wav.cpp
    src/wav_file_status.cpp
    src/wav_file_cmd.cpp
    src/wav_file.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    $ENV{PICO_EXAMPLES_PATH}/pico_w/wifi/
)

target_link_libraries(${PROJECT_NAME}
    hardware_adc
    pico_aon_timer
    pico_fatfs_customized
    pico_flash_param
    pico_multicore
    pico_ntp_client
    pico_spdif_rx
    pico_stdlib
    picow_config_wifi
)

pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)

pico_add_extra_outputs(${PROJECT_NAME}) # create map/bin/hex/uf2 files
