# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
include(FetchContent)

if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()

set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()

set(PICO_BOARD pico_w CACHE STRING "Board type")
include(pico_sdk_import.cmake)

project(main C CXX ASM)

pico_sdk_init()

# Inclui os novos arquivos do diretório src/
add_executable(main 
    main.c
    src/mqtt.c
    src/wifi.c
)

pico_set_program_name(main "main")
pico_set_program_version(main "0.1")

pico_enable_stdio_uart(main 1)
pico_enable_stdio_usb(main 1)

# Bibliotecas padrão e específicas do projeto
target_link_libraries(main
    pico_stdlib
    hardware_pwm
    hardware_adc
    hardware_i2c
    freertos_kernel
    oled
    pico_cyw43_arch_lwip_threadsafe_background  # ESSENCIAL para Wi-Fi + lwIP com FreeRTOS
    pico_lwip_mqtt # Adiciona suporte a cliente MQTT sobre o LwIP.
    pico_lwip # A pilha Lightweight IP (LwIP), que fornece funcionalidades TCP/IP (HTTP, MQTT, UDP, DHCP, etc.).
    pico_cyw43_driver # O driver do chip Wi-Fi CYW43 (usado no Pico W).
)

# Diretórios de include
target_include_directories(main PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}         # para encontrar lwipopts.h
    ${CMAKE_CURRENT_LIST_DIR}/include  # para mqtt.h, wifi.h e FreeRTOSConfig.h
    ${CMAKE_CURRENT_LIST_DIR}/include/oled
)

add_subdirectory(include/oled)

pico_add_extra_outputs(main)

# Adicionando FreeRTOS
FetchContent_Declare(freertos_kernel
  GIT_REPOSITORY https://github.com/FreeRTOS/FreeRTOS-Kernel.git
  GIT_TAG        main
)

add_library(freertos_config INTERFACE)
target_include_directories(freertos_config SYSTEM
  INTERFACE
    include
)

set(FREERTOS_HEAP "4" CACHE STRING "" FORCE)
set(FREERTOS_PORT "GCC_RP2040" CACHE STRING "" FORCE)
if (CMAKE_CROSSCOMPILING)
  set(FREERTOS_PORT "GCC_RP2040" CACHE STRING "" FORCE)
endif()

FetchContent_MakeAvailable(freertos_kernel)
