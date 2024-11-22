if (NOT FREERTOS_KERNEL_PATH AND NOT DEFINED ENV{FREERTOS_KERNEL_PATH})
    message("Skipping Pico W FreeRTOS examples as FREERTOS_KERNEL_PATH not defined")
else()
    include(FreeRTOS_Kernel_import.cmake)

    add_subdirectory(RTOS_integration)
    # add_subdirectory(RTOS_zx)
    add_subdirectory(RTOS_xh)
    add_subdirectory(Buddy1_WIFI)
    
endif()