if (NOT TARGET pico_fatfs_customized)
    add_subdirectory(pico_fatfs/pio/spi pio_spi)
    add_library(pico_fatfs_customized INTERFACE)

    set(FATFS_REV R0.15)

    target_sources(pico_fatfs_customized INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/pico_fatfs/fatfs/ffunicode.c  # original
        ${CMAKE_CURRENT_LIST_DIR}/pico_fatfs/tf_card.c          # original
        ${CMAKE_CURRENT_LIST_DIR}/${FATFS_REV}/ff.c             # customized for timezone
        ${CMAKE_CURRENT_LIST_DIR}/${FATFS_REV}/ffsystem.c       # customized for reentrant
        ${CMAKE_CURRENT_LIST_DIR}/tf_card_override.c            # overridden for fattime and timezone
    )

    target_include_directories(pico_fatfs_customized INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/${FATFS_REV}
        ${CMAKE_CURRENT_LIST_DIR}/pico_fatfs
        ${CMAKE_CURRENT_LIST_DIR}/pico_fatfs/fatfs
    )

    target_link_libraries(pico_fatfs_customized INTERFACE
        hardware_clocks
        hardware_spi
        pico_aon_timer
        pico_stdlib
        pio_spi
    )
endif()
