cmake_minimum_required(VERSION 3.12)

if (PICO_CYW43_SUPPORTED) # set by PICO_BOARD=pico_w
    if (NOT TARGET pico_cyw43_arch)
        message("Skipping Pico W examples as support is not available")
    else()
        add_subdirectory(communication)
        add_subdirectory(IR_sensor)

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

        # ================= FREERTOS ==================
        add_executable(test_dashboard test_dashboard.c)
        target_compile_definitions(test_dashboard PRIVATE)
        target_include_directories(test_dashboard PRIVATE ${CMAKE_CURRENT_LIST_DIR})
        target_link_libraries(test_dashboard pico_stdlib FreeRTOS-Kernel-Heap4 wifi_communication client_communication server_communication)
        pico_add_extra_outputs(test_dashboard)
        pico_enable_stdio_usb(test_dashboard 1)


        # ================= UDP REMOTE ==================
        add_executable(udp_remote udp_remote.c)
        target_compile_definitions(udp_remote PRIVATE)
        target_link_libraries(udp_remote pico_stdlib wifi_communication udp_client_communication)
        pico_add_extra_outputs(udp_remote)
        pico_enable_stdio_usb(udp_remote 1)


        # ================= UDP FREERTOS CAR ==================
        add_executable(udp_car udp_car.c)
        target_compile_definitions(udp_car PRIVATE)
        target_include_directories(udp_car PRIVATE ${CMAKE_CURRENT_LIST_DIR})
        target_link_libraries(udp_car pico_stdlib FreeRTOS-Kernel-Heap4 wifi_communication udp_server_communication)
        pico_add_extra_outputs(udp_car)
        pico_enable_stdio_usb(udp_car 1)

        add_executable(UpdatedAccelerometer UpdatedAccelerometer.c)
        target_compile_definitions(UpdatedAccelerometer PRIVATE)
        target_link_libraries(UpdatedAccelerometer pico_stdlib hardware_i2c wifi_communication client_communication)
        pico_add_extra_outputs(UpdatedAccelerometer)
        pico_enable_stdio_usb(UpdatedAccelerometer 1)
    endif()
endif()
