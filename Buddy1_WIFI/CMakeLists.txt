cmake_minimum_required(VERSION 3.12)

if (PICO_CYW43_SUPPORTED) # set by PICO_BOARD=pico_w
    if (NOT TARGET pico_cyw43_arch)
        message("Skipping Pico W examples as support is not available")
    else()
        add_executable(test_car test_car.c)
        add_subdirectory(communication)

        target_compile_definitions(test_car PRIVATE)

        target_link_libraries(test_car pico_stdlib wifi_communication client_communication server_communication)
        

        pico_add_extra_outputs(test_car)
        pico_enable_stdio_usb(test_car 1)



        add_executable(test_remote test_remote.c)
        target_compile_definitions(test_remote PRIVATE)

        target_link_libraries(test_remote pico_stdlib wifi_communication client_communication server_communication)
        

        pico_add_extra_outputs(test_remote)
        pico_enable_stdio_usb(test_remote 1)
    endif()
endif()
