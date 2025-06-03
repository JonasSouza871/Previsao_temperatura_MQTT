# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
# Esta seção é utilizada pela extensão do VS Code para o Raspberry Pi Pico
# e garante a correta configuração do ambiente de desenvolvimento.
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
# As versões do SDK, toolchain e picotool podem precisar ser ajustadas
# de acordo com a configuração do ambiente de desenvolvimento.
set(sdkVersion 2.1.0)
set(toolchainVersion 13_3_Rel1)
set(picotoolVersion 2.1.0)

set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================

cmake_minimum_required(VERSION 3.13)

# Configurações padrão do compilador C e CXX
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # Gera o compile_commands.json para linters e outras ferramentas

# Define a placa alvo como Raspberry Pi Pico W
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Inclui as definições e funções do SDK do Raspberry Pi Pico.
include(pico_sdk_import.cmake)

# CONFIGURAÇÃO DO FREERTOS - Aplicando a configuração que funcionou
set(FREERTOS_KERNEL_PATH "C:/Users/Jonas/FreeRTOS-Kernel")
include(${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

# ADIÇÃO PARA DIAGNÓSTICO: Verifica se o pico_sdk_import.cmake funcionou
if(NOT DEFINED PICO_SDK_INIT_CMAKE_FILE)
    message(FATAL_ERROR "ERRO CRÍTICO: PICO_SDK_INIT_CMAKE_FILE não está definido. "
                        "Isso significa que 'pico_sdk_import.cmake' falhou ao encontrar ou carregar o SDK do Pico.\n"
                        "Verifique se:\n"
                        "1. A variável de ambiente PICO_SDK_PATH está correta e aponta para a pasta do SDK (ex: C:/Users/Jonas/.pico-sdk/sdk/2.1.0).\n"
                        "2. O arquivo 'pico_sdk_import.cmake' está presente na raiz do seu projeto e é uma cópia válida do arquivo original do SDK.\n"
                        "3. A instalação do SDK no PICO_SDK_PATH está completa e não corrompida.")
endif()

# Nome do projeto
project(PicoMQTT C CXX ASM)

# Inicializa o SDK do Pico para o projeto
pico_sdk_init()

# Diretórios de inclusão para headers do projeto (usando include_directories como no exemplo que funcionou)
include_directories(
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/lib
    ${CMAKE_SOURCE_DIR}/lib/Display_Bibliotecas
    ${CMAKE_SOURCE_DIR}/lib/DS18b20
    ${CMAKE_SOURCE_DIR}/lib/Matriz_Bibliotecas
    ${CMAKE_SOURCE_DIR}/lib/Wifi
)

# Adiciona o executável principal do projeto e seus arquivos fonte.
add_executable(PicoMQTT
    main.c
    lib/Display_Bibliotecas/ssd1306.c
    lib/DS18b20/ds18b20.c
    lib/Matriz_Bibliotecas/matriz_led.c
)

# Gera o cabeçalho PIO para o WS2812
pico_generate_pio_header(PicoMQTT ${CMAKE_CURRENT_LIST_DIR}/lib/Matriz_Bibliotecas/ws2812.pio)

# Define o nome e a versão do programa (útil para identificação do firmware)
pico_set_program_name(PicoMQTT "PicoMQTT_App")
pico_set_program_version(PicoMQTT "0.1")

# Configuração da saída padrão (stdio)
pico_enable_stdio_uart(PicoMQTT 0) # Saída UART desabilitada (UART0)
pico_enable_stdio_usb(PicoMQTT 1)  # Saída USB (CDC) habilitada

# Linka as bibliotecas necessárias para o projeto (usando a ordem que funcionou)
target_link_libraries(PicoMQTT
    pico_stdlib                                # Biblioteca padrão do Pico SDK
    hardware_adc                               # Para uso do conversor Analógico-Digital
    hardware_pio                               # Necessário para usar PIO
    hardware_i2c                               # Para comunicação I2C
    pico_cyw43_arch_lwip_threadsafe_background # Suporte para Wi-Fi e LwIP no Pico W (thread-safe)
    pico_lwip_mqtt                             # Biblioteca MQTT sobre LwIP
    pico_mbedtls                               # Biblioteca mbedTLS para funcionalidades criptográficas
    pico_lwip_mbedtls                          # Integração do LwIP com mbedTLS
    FreeRTOS-Kernel                            # Kernel do FreeRTOS
    FreeRTOS-Kernel-Heap4                     # Gerenciador de memória do FreeRTOS
    hardware_pwm                               # PWM
)

# Gera arquivos de saída adicionais (ex: .uf2, .hex) para gravação no microcontrolador
pico_add_extra_outputs(PicoMQTT)
