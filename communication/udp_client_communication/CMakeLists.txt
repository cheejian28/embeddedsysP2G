
add_library(udp_client_communication STATIC udp_client_communication.c)
target_compile_definitions(udp_client_communication PUBLIC)

target_include_directories(udp_client_communication PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts
)

target_link_libraries(udp_client_communication
        pico_cyw43_arch_lwip_threadsafe_background
        pico_stdlib
)