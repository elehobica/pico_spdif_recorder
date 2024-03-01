if (NOT TARGET pico_ntp_client)
    add_library(pico_ntp_client INTERFACE)

    target_sources(pico_ntp_client INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/ntp_client.cpp
    )

    target_include_directories(pico_ntp_client INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}
        $ENV{PICO_EXAMPLES_PATH}/pico_w/wifi/ntp_client
    )

    target_link_libraries(pico_ntp_client INTERFACE
        pico_stdlib
        pico_cyw43_arch_lwip_poll
    )
endif()
