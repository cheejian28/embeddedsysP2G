cmake_minimum_required(VERSION 3.12)

if (PICO_CYW43_SUPPORTED) # set by PICO_BOARD=pico_w
    if (NOT TARGET pico_cyw43_arch)
        message("Skipping Pico W examples as support is not available")
    else()
        add_subdirectory(communication)

        # ================= CAR ==================
        add_executable(test_car test_car.c)
        target_compile_definitions(test_car PRIVATE)
        target_link_libraries(test_car pico_stdlib wifi_communication client_communication server_communication)
        pico_add_extra_outputs(test_car)
        pico_enable_stdio_usb(test_car 1)


        # ================= REMOTE ==================
        add_executable(test_remote test_remote.c)
        target_compile_definitions(test_remote PRIVATE)
        target_link_libraries(test_remote pico_stdlib wifi_communication client_communication server_communication)
        pico_add_extra_outputs(test_remote)
        pico_enable_stdio_usb(test_remote 1)

        # ================= FREERTOS ==================
        add_executable(rtos_car rtos_car.c)
        target_compile_definitions(rtos_car PRIVATE)
        target_include_directories(rtos_car PRIVATE ${CMAKE_CURRENT_LIST_DIR})
        target_link_libraries(rtos_car pico_stdlib FreeRTOS-Kernel-Heap4 wifi_communication client_communication server_communication)
        pico_add_extra_outputs(rtos_car)
        pico_enable_stdio_usb(rtos_car 1)

        # ================= FREERTOS ==================
        add_executable(rtos_remote rtos_remote.c)
        target_compile_definitions(rtos_remote PRIVATE)
        target_include_directories(rtos_remote PRIVATE ${CMAKE_CURRENT_LIST_DIR})
        target_link_libraries(rtos_remote pico_stdlib FreeRTOS-Kernel-Heap4 wifi_communication client_communication server_communication)
        pico_add_extra_outputs(rtos_remote)
        pico_enable_stdio_usb(rtos_remote 1)
    endif()
endif()
