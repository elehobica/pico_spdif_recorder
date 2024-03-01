if (NOT TARGET picoConfigParam)
    add_library(picoConfigParam INTERFACE)

    target_sources(picoConfigParam INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/ConfigParam.cpp
        ${CMAKE_CURRENT_LIST_DIR}/UserFlash.cpp
    )

    target_include_directories(picoConfigParam INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}
    )

    target_link_libraries(picoConfigParam INTERFACE
        hardware_exception
        pico_stdlib
        pico_flash

    )
endif()
