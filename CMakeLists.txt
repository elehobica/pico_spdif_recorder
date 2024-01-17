cmake_minimum_required(VERSION 3.12)

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
)

target_link_libraries(${PROJECT_NAME}
    pico_stdlib
    pico_multicore
    spdif_rx
    pico_fatfs
)

pico_add_extra_outputs(${PROJECT_NAME}) # create map/bin/hex/uf2 files