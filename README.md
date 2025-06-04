# 🚀 PicoTemp Oracle: Previsão Inteligente de Temperatura com Visualização MQTT em Tempo Real
> *Um sistema de monitoramento e previsão de temperatura em tempo real para Raspberry Pi Pico, utilizando FreeRTOS, com display OLED, LEDs de status, matriz de LEDs e integração MQTT.*

## 📝 Descrição Breve
Este projeto implementa um sistema avançado de monitoramento e previsão de temperatura utilizando o microcontrolador Raspberry Pi Pico. Ele lê a temperatura de um sensor DS18B20, calcula previsões usando regressão linear e suavização Holt, e exibe esses dados em um display OLED SSD1306. LEDs de status e uma matriz de LEDs fornecem feedback visual imediato sobre a situação da temperatura (normal, atenção, alerta, grave) em relação a um ponto de urgência configurável. O sistema se conecta a uma rede Wi-Fi e publica todos os dados e estados via MQTT para monitoramento e controle remoto em tempo real. Um buzzer emite alertas sonoros para condições críticas. O projeto é construído sobre FreeRTOS para gerenciamento eficiente de tarefas concorrentes, como leitura de sensor, processamento de dados, atualização de display, controle de indicadores e comunicação de rede.

**Componentes principais envolvidos:**
*   Raspberry Pi Pico
*   Display OLED I2C SSD1306 (128x64)
*   Sensor de Temperatura DS18B20 (1-Wire)
*   Botões Táteis (2 para navegação/ajuste)
*   Entrada Analógica (para ajuste de parâmetro, como um joystick)
*   LEDs (Verde, Vermelho)
*   Matriz de LEDs (WS2812 ou similar)
*   Buzzer Passivo
*   Módulo Wi-Fi CYW43 (integrado no Pico W)

**Uso esperado ou aplicação prática:**
*   Monitoramento de temperatura em ambientes críticos (servidores, estufas, salas).
*   Sistema de alerta precoce para variações de temperatura.
*   Plataforma educacional para sistemas embarcados com RTOS, previsão de dados, IoT e conectividade.
*   Base para projetos de automação residencial ou industrial que exigem monitoramento ambiental.

**Tecnologias ou bibliotecas utilizadas:**
*   Linguagem: C
*   SDK/Framework: Raspberry Pi Pico SDK
*   Sistema Operacional: FreeRTOS
*   Bibliotecas de Rede: LwIP (com suporte a MQTT, DNS, TLS)
*   Bibliotecas customizadas/de terceiros: `ssd1306.h` (Display OLED), `ds18b20.h` (Sensor 1-Wire), `matriz_led.h` (Matriz de LEDs WS2812), `pico/cyw43_arch.h` (Driver Wi-Fi).

## ✨ Funcionalidades Principais
*   🌡️ **Leitura de Temperatura:** Aquisição contínua de temperatura através do sensor DS18B20.
*   📈 **Previsão de Temperatura:** Implementação de dois métodos de previsão: Regressão Linear e Suavização Exponencial de Holt, com base em um histórico de leituras.
*   🖥️ **Display OLED Informativo:** Exibição em tempo real de:
    *   Temperatura atual.
    *   Temperaturas previstas (Linear e Holt).
    *   Ponto de urgência configurado.
    *   Situação da temperatura (Normal, Atenção, Alerta, Grave).
    *   Interface de configuração para o ponto de urgência.
*   🔄 **Entrada do Usuário:** Botões dedicados para navegação entre telas e ajuste do ponto de urgência, e uma entrada analógica para ajuste fino.
*   🚥 **Feedback Visual (LEDs e Matriz de LEDs):**
    *   LEDs Verde/Vermelho indicam o estado geral da temperatura.
    *   Matriz de LEDs exibe padrões visuais (OK, Exclamação, X) correspondentes à situação da temperatura.
*   🔔 **Alertas Sonoros:** Buzzer emite bipes indicando situações de "Atenção", "Alerta" e "Grave".
*   🌐 **Conectividade Wi-Fi:** Conexão à rede local para comunicação com broker MQTT.
*   ☁️ **Publicação MQTT em Tempo Real:** Publica periodicamente no broker MQTT os seguintes dados:
    *   Temperatura atual.
    *   Temperaturas previstas (Linear e Holt).
    *   Situação da temperatura.
    *   Ponto de urgência configurado.
    *   Suporte a Last Will and Testament para indicar status online/offline.
*   ↩️ **Controle MQTT (Exemplo):** Suporte básico para receber comandos via MQTT (ex: ligar/desligar LED integrado do Pico W via `/led`).
*   🚀 **Multitarefa com FreeRTOS:** Gerenciamento eficiente de operações concorrentes (leitura de sensor, processamento de dados, I/O do usuário, atualização de display, comunicação de rede).

## ⚙️ Pré-requisitos / Hardware Necessário
### Hardware
| Componente                        | Quant. | Observações                                                              |
| :-------------------------------- | :----: | :----------------------------------------------------------------------- |
| Raspberry Pi Pico W               |   1    | Necessita da versão com Wi-Fi (CYW43) integrado. Com headers soldados é recomendado. |
| Display OLED I2C SSD1306 128x64   |   1    | Monocromático. Conectado via I2C.                                        |
| Sensor de Temperatura DS18B20     |   1    | Sensor digital 1-Wire.                                                   |
| Botão Táctil (Push Button)        |   2    | Um para "Próxima Tela" (GP5) e outro para "Tela Anterior" (GP6).         |
| Potenciômetro ou Joystick Analógico |   1    | Conectado à entrada ADC (GP26) para ajuste de temperatura de urgência.   |
| LED Verde                         |   1    | Para indicação de status.                                                |
| LED Vermelho                      |   1    | Para indicação de status.                                                |
| Matriz de LEDs 5x5 (ex: WS2812)   |   1    | Endereçável (NeoPixel). Conectado via PIO.                               |
| Buzzer Passivo                    |   1    | Controlado por PWM para alertas sonoros.                                 |
| Resistores                        | Vários | Conforme necessidade para LEDs, sensor DS18B20 (pull-up de 4.7kΩ para data line) e botões (pull-ups internos do Pico). |
| Protoboard                        |   1    | Para montagem do circuito.                                               |
| Jumpers Macho-Macho/Macho-Fêmea   | Vários | Para conexões.                                                           |
| Cabo Micro USB                    |   1    | Para alimentação e programação do Pico.                                  |

### Software / Ferramentas
*   **Raspberry Pi Pico SDK:** Versão mais recente recomendada.
*   **ARM GCC Toolchain:** (e.g., `arm-none-eabi-gcc`)
*   **CMake:** Versão 3.13 ou superior.
*   **Git:** Para clonar o repositório.
*   **IDE (Opcional):** VS Code com extensões C/C++ e CMake Tools.
*   **Sistema Operacional Testado:** Linux, macOS, Windows (com WSL2 ou ambiente Pico Toolchain).
*   **Terminal Serial:** PuTTY, minicom, Tera Term (Baud rate: **115200 bps**).
*   **Broker MQTT:** Um broker MQTT acessível pela rede (ex: Mosquitto, HiveMQ Cloud).
*   **Cliente MQTT:** Para visualizar e interagir com os tópicos (ex: MQTT Explorer, mosquitto_sub/pub).

## 🔌 Conexões / Configuração Inicial
### Pinagem resumida (Conforme `main.c`)
| Pino Pico (GP) | Componente          | Função/Conexão                                       |
| :------------- | :------------------ | :--------------------------------------------------- |
| GP5            | Botão Próx. Tela (A)| Sinal do Botão (Pull-up interno habilitado)          |
| GP6            | Botão Tela Ant. (B) | Sinal do Botão (Pull-up interno habilitado)          |
| GP26           | ADC                 | Entrada analógica (e.g., centro de potenciômetro ou joystick VRx) |
| GP14 (I2C1 SDA)| Display OLED        | SDA (Dados I2C)                                      |
| GP15 (I2C1 SCL)| Display OLED        | SCL (Clock I2C)                                      |
| GP16           | Sensor DS18B20      | Pino de Dados 1-Wire (com resistor pull-up externo de 4.7kΩ para 3.3V) |
| GP11           | LED Verde           | Sinal de controle do LED                             |
| GP13           | LED Vermelho        | Sinal de controle do LED                             |
| GP10           | Buzzer Passivo      | Sinal PWM para o Buzzer                              |
| GP07           | Matriz de LED 5x5   | Pino de Dados (DIN) da Matriz de LED (configurado na biblioteca `matriz_led.c` ou `.pio`) |
| 3V3 (OUT)      | Vários              | Alimentação 3.3V para periféricos                    |
| GND            | Vários              | Referência comum de terra para todos os componentes  |

> **Nota Importante:**
> *   Certifique-se de que todos os componentes compartilham um **GND comum** com o Raspberry Pi Pico.
> *   Verifique a tensão de alimentação correta para cada periférico (principalmente 3.3V do Pico).
> *   O sensor DS18B20 requer um resistor de pull-up (geralmente 4.7kΩ) entre o pino de dados e 3.3V.
> *   Os botões são configurados para usar o resistor de pull-up interno do Pico, simplificando a fiação (conecte o botão entre o pino GPIO e GND).
> *   A matriz de LEDs WS2812 é sensível ao timing, certifique-se de que a conexão esteja estável.

### Configuração de Software (primeira vez)
1.  **Clone o repositório:**
    ```bash
    git clone https://github.com/SeuUsuario/PicoTemp_Oracle.git
    cd PicoTemp_Oracle
    ```

2.  **Inicialize e atualize os submódulos (se o Pico SDK estiver como submódulo):**
    ```bash
    git submodule update --init --recursive
    ```
    Caso contrário, certifique-se de que a variável de ambiente `PICO_SDK_PATH` esteja configurada apontando para a raiz do seu SDK.

3.  **Verifique e ajuste as configurações no `main.c` (ou arquivos de configuração equivalentes):**
    *   **Configurações de Rede:**
        ```c
        #define WIFI_SSID       "[SEU_SSID_WIFI]"
        #define WIFI_PASSWORD   "[SUA_SENHA_WIFI]"
        #define MQTT_SERVER     "[IP_OU_HOSTNAME_DO_BROKER_MQTT]"   // Ex: "192.168.1.100" ou "broker.hivemq.com"
        #define MQTT_USERNAME   "[SEU_USUARIO_MQTT]"         // Deixe "" se broker anônimo
        #define MQTT_PASSWORD   "[SUA_SENHA_MQTT]"           // Deixe "" se broker anônimo
        ```
    *   **Ponto de Urgência Inicial:** `estado_sistema.temperatura_urgencia` (o valor padrão é 30).
    *   **Tópico Base MQTT:** `MQTT_TOPIC_BASE` (padrão: `/Temperatura_MQTT_Pico`).

## ▶️ Como Compilar e Executar
Siga estes passos para compilar o projeto:

1.  **Crie e acesse o diretório de build:**
    A partir da raiz do projeto:
    ```bash
    mkdir build
    cd build
    ```

2.  **Configure o CMake apontando para o Pico SDK:**
    Se o SDK estiver em um local padrão ou for encontrado automaticamente, `cmake ..` pode ser suficiente. Caso contrário, defina a variável de ambiente `PICO_SDK_PATH` antes:
    ```bash
    # Exemplo (ajuste para o seu caminho):
    # export PICO_SDK_PATH=/caminho/absoluto/para/pico-sdk
    cmake ..
    ```

3.  **Compile o projeto:**
    ```bash
    make -j$(nproc)  # Para Linux/macOS (usa todos os núcleos disponíveis)
    # ou
    # make -jX       # Onde X é o número de núcleos que você deseja usar
    ```
    Isso gerará um arquivo `.uf2` (e.g., `pico_temp_oracle.uf2`) dentro do diretório `build`.

**Para gravar na placa (Raspberry Pi Pico W):**
1.  Desconecte o Pico W da alimentação (USB).
2.  Pressione e mantenha pressionado o botão **BOOTSEL** no Pico W.
3.  Conecte o Pico W ao seu computador via cabo USB enquanto mantém o BOOTSEL pressionado.
4.  O Pico W aparecerá como um dispositivo de armazenamento em massa (`RPI-RP2`).
5.  Arraste e solte o arquivo `.uf2` gerado (ex: `pico_temp_oracle.uf2`) para dentro do dispositivo de armazenamento do Pico W.
6.  O Pico W irá reiniciar automaticamente e começar a executar o firmware.

**Como acessar logs/interfaces:**
*   **Logs (Serial):**
    *   Conecte-se ao Pico W usando um programa de terminal serial (PuTTY, minicom, Tera Term, etc.).
    *   Configure a porta serial correspondente ao Pico W e use uma taxa de transmissão (baud rate) de **115200 bps**.
    *   Mensagens de inicialização, status da conexão Wi-Fi/MQTT e logs de depuração podem ser visualizados aqui.

## 👤 Autor / Contato
*   **Nome:** Jonas Souza
*   **E-mail:** Jonassouza871@hotmail.com