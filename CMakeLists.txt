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

add_subdirectory(lib/pico_spdif_rx)
add_subdirectory(lib/pico_spdif_rx/samples/lib/pico_audio_32b) # for extended audio.h
add_subdirectory(lib/pico_fatfs)

# select sample application here (choose only one)
add_executable(${PROJECT_NAME}
    src/main.cpp
    src/spdif_rec_wav.cpp
    src/wav_file_status.cpp
    src/wav_file_cmd.cpp
    src/wav_file.cpp
    src/ntp_client.cpp
    src/config_wifi.cpp
    src/UserFlash.cpp
    src/ConfigParam.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    $ENV{PICO_EXAMPLES_PATH}/pico_w/wifi/
    $ENV{PICO_EXAMPLES_PATH}/pico_w/wifi/ntp_client
)

target_link_libraries(${PROJECT_NAME}
    pico_stdlib
    pico_multicore
    hardware_adc
    pico_cyw43_arch_lwip_poll
    pico_spdif_rx
    pico_fatfs
)

pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)

pico_add_extra_outputs(${PROJECT_NAME}) # create map/bin/hex/uf2 files
