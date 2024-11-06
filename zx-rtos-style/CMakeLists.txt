add_executable(motor_control
        motor_control.c
        wheel_encoder.c
        ultrasonic.c
        )
target_include_directories(motor_control PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/../.. # for our common lwipopts
        )

# pull in common dependencies and additional pwm hardware support
target_link_libraries(motor_control 
        pico_stdlib
        FreeRTOS-Kernel-Heap4 # FreeRTOS kernel and dynamic heap
        hardware_pwm
        hardware_gpio
        hardware_adc
        )

# Add the include directory for FreeRTOS.h
target_include_directories(motor_control PRIVATE C:/FreeRTOS-Kernel-main/include)

# create map/bin/hex file etc.
pico_add_extra_outputs(motor_control)

# add url via pico_set_program_url
example_auto_set_url(motor_control)

# enable usb output
pico_enable_stdio_usb(motor_control 1)
