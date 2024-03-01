if (NOT TARGET config_wifi)
    add_library(config_wifi INTERFACE)

    target_sources(config_wifi INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/config_wifi.cpp
    )

    target_include_directories(config_wifi INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}
    )

    target_link_libraries(config_wifi INTERFACE
        pico_stdlib
    )
endif()
