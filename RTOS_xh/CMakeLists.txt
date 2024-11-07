add_executable(RTOS_xh
        motor_control.c
        )
target_include_directories(RTOS_xh PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/../.. # for our common lwipopts
        )

# pull in common dependencies and additional pwm hardware support
target_link_libraries(RTOS_xh 
        pico_stdlib
        FreeRTOS-Kernel-Heap4 # FreeRTOS kernel and dynamic heap
        hardware_pwm
        hardware_gpio
        hardware_adc
        )

# Add the include directory for FreeRTOS.h
target_include_directories(RTOS_xh PRIVATE C:/FreeRTOS-Kernel-main/include)

# create map/bin/hex file etc.
pico_add_extra_outputs(RTOS_xh)

# add url via pico_set_program_url
example_auto_set_url(RTOS_xh)

# enable usb output
pico_enable_stdio_usb(RTOS_xh 1)
