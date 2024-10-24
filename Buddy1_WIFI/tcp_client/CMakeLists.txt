
add_executable(car_tcp_client tcp_client.c)
target_compile_definitions(car_tcp_client PRIVATE
        WIFI_SSID=\"${WIFI_SSID}\"
        WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
        # TEST_TCP_SERVER_IP=\"${TEST_TCP_SERVER_IP}\"
        TEST_TCP_SERVER_IP=\"172.20.10.9\"
)

target_include_directories(car_tcp_client PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts
)

target_link_libraries(car_tcp_client
        pico_cyw43_arch_lwip_threadsafe_background
        pico_stdlib
)

pico_add_extra_outputs(car_tcp_client)
pico_enable_stdio_usb(car_tcp_client 1)
