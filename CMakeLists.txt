cmake_minimum_required(VERSION 3.13)
set(PICO_BOARD "pico_w")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
include($ENV{PICO_EXTRAS_PATH}/external/pico_extras_import.cmake)

set(project_name "pico_spdif_recorder" C CXX ASM)
project(${project_name})
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_subdirectory(lib/pico_spdif_rx pico_spdif_rx)
add_subdirectory(lib/pico_spdif_rx/samples/lib/pico_audio_32b pico_audio_32b) # for extended audio.h
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
    hardware_rtc
    pico_multicore
    pico_spdif_rx
    pico_stdlib
    pico_fatfs_customized
    pico_flash_param
    pico_ntp_client
    picow_config_wifi
)

pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)

pico_add_extra_outputs(${PROJECT_NAME}) # create map/bin/hex/uf2 files
