
add_library(server_communication STATIC server_communication.c)
target_compile_definitions(server_communication PUBLIC)

target_include_directories(server_communication PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts
)

target_link_libraries(server_communication
        pico_cyw43_arch_lwip_threadsafe_background
        pico_stdlib
)