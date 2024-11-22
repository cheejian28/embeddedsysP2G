
add_library(wifi_communication STATIC wifi_communication.c)
target_compile_definitions(wifi_communication PUBLIC)

target_include_directories(wifi_communication PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts
)

target_link_libraries(wifi_communication
        pico_cyw43_arch_lwip_threadsafe_background
        pico_stdlib
)