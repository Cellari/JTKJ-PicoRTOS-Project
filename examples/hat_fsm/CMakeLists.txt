add_executable(hat_fsm
  ${CMAKE_CURRENT_LIST_DIR}/src/main.c
  ${CMAKE_CURRENT_LIST_DIR}/src/morse_utils.c
)

target_include_directories( hat_fsm PRIVATE
  include
)

target_link_libraries(hat_fsm PRIVATE
  pico_stdlib
  FreeRTOS-Kernel
  FreeRTOS-Kernel-Heap4
  usb_serial_debug
  TKJHAT_SDK
)

if (PICO_CYW43_SUPPORTED)
    target_link_libraries(hat_fsm PRIVATE pico_cyw43_arch_none)
endif()

pico_enable_stdio_usb(hat_fsm 1)
pico_enable_stdio_uart(hat_fsm 0)

pico_add_extra_outputs(hat_fsm)