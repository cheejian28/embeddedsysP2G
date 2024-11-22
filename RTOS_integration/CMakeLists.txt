add_executable(RTOS_integration
        motor_control.c
        wheel_encoder.c
        ultrasonic.c
        )
target_include_directories(RTOS_integration PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/../.. # for our common lwipopts
        )

# pull in common dependencies and additional pwm hardware support
target_link_libraries(RTOS_integration 
        pico_stdlib
        FreeRTOS-Kernel-Heap4 # FreeRTOS kernel and dynamic heap
        hardware_pwm
        hardware_gpio
        hardware_adc
        wifi_communication 
        client_communication 
        server_communication
        )

# Add the include directory for FreeRTOS.h
target_include_directories(RTOS_integration PRIVATE C:/FreeRTOS-Kernel-main/include)

# create map/bin/hex file etc.
pico_add_extra_outputs(RTOS_integration)

# add url via pico_set_program_url
example_auto_set_url(RTOS_integration)

# enable usb output
pico_enable_stdio_usb(RTOS_integration 1)