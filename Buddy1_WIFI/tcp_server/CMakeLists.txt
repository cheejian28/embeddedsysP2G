add_executable(car_tcp_server tcp_server.c)

target_compile_definitions(car_tcp_server PRIVATE
        WIFI_SSID=\"${WIFI_SSID}\"
        WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
)

target_include_directories(car_tcp_server PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts
)

target_link_libraries(car_tcp_server
        pico_cyw43_arch_lwip_threadsafe_background
        pico_stdlib
)

pico_add_extra_outputs(car_tcp_server)
pico_enable_stdio_usb(car_tcp_server 1)