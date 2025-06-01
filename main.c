/* AULA IoT - Ricardo Prates - 001 - Cliente MQTT - Publisher:/Temperatura; Subscribed:/led
 *
 * Material de suporte - 27/05/2025
 * * Código adaptado de: https://github.com/raspberrypi/pico-examples/tree/master/pico_w/wifi/mqtt 
 */
#include <math.h>
#include "pico/stdlib.h"        // Biblioteca da Raspberry Pi Pico para funções padrão (GPIO, temporização, etc.)
#include "pico/cyw43_arch.h"    // Biblioteca para arquitetura Wi-Fi da Pico com CYW43
#include "pico/unique_id.h"     // Biblioteca para obter o ID único da placa

#include "hardware/gpio.h"      // Biblioteca de hardware de GPIO
#include "hardware/irq.h"       // Biblioteca de hardware de interrupções
#include "hardware/adc.h"       // Biblioteca de hardware para conversão ADC

#include "lwip/apps/mqtt.h"     // Biblioteca LWIP MQTT - fornece funções e recursos para conexão MQTT
#include "lwip/apps/mqtt_priv.h"// Biblioteca que fornece funções e recursos para Geração de Conexões
#include "lwip/dns.h"           // Biblioteca que fornece funções e recursos suporte DNS
#include "lwip/altcp_tls.h"     // Biblioteca que fornece funções e recursos para conexões seguras usando TLS

// Includes para as bibliotecas customizadas do projeto (localizadas em lib/)
// Certifique-se de que o CMakeLists.txt está configurado para encontrar estes cabeçalhos.
#include "ssd1306.h"            // Para o display OLED SSD1306
#include "ds18b20.h"            // Para o sensor de temperatura DS18B20
#include "matriz_led.h"         // Para a matriz de LEDs WS2812 (via PIO)
// Se for usar FreeRTOS ativamente, inclua os cabeçalhos do FreeRTOS aqui:
// #include "FreeRTOS.h"
// #include "task.h"

#define WIFI_SSID "Nome_wifi"            // Substitua pelo nome da sua rede Wi-Fi
#define WIFI_PASSWORD "Senha_wifi"      // Substitua pela senha da sua rede Wi-Fi
#define MQTT_SERVER "ip_host"     // Substitua pelo endereço do host - broker MQTT: Ex: 192.168.1.107
#define MQTT_USERNAME "Usario_mqtt"           // Substitua pelo nome da host MQTT - Username
#define MQTT_PASSWORD "Senha_mqtt_"        // Substitua pelo Password da host MQTT - credencial de acesso - caso exista

// Definição da escala de temperatura
#ifndef TEMPERATURE_UNITS
#define TEMPERATURE_UNITS 'C' // Set to 'F' for Fahrenheit
#endif

#ifndef MQTT_SERVER
#error Need to define MQTT_SERVER
#endif

// This file includes your client certificate for client server authentication
#ifdef MQTT_CERT_INC
#include MQTT_CERT_INC
#endif

#ifndef MQTT_TOPIC_LEN
#define MQTT_TOPIC_LEN 100
#endif

//Dados do cliente MQTT
typedef struct {
    mqtt_client_t* mqtt_client_inst;
    struct mqtt_connect_client_info_t mqtt_client_info;
    char data[MQTT_OUTPUT_RINGBUF_SIZE];
    char topic[MQTT_TOPIC_LEN];
    uint32_t len;
    ip_addr_t mqtt_server_address;
    bool connect_done;
    int subscribe_count;
    bool stop_client;
} MQTT_CLIENT_DATA_T;

#ifndef DEBUG_printf
#ifndef NDEBUG
#define DEBUG_printf printf
#else
#define DEBUG_printf(...)
#endif
#endif

#ifndef INFO_printf
#define INFO_printf printf
#endif

#ifndef ERROR_printf
#define ERROR_printf printf
#endif

// Temporização da coleta de temperatura - how often to measure our temperature
#define TEMP_WORKER_TIME_S 10

// Manter o programa ativo - keep alive in seconds
#define MQTT_KEEP_ALIVE_S 60

// QoS - mqtt_subscribe
// At most once (QoS 0)
// At least once (QoS 1)
// Exactly once (QoS 2)
#define MQTT_SUBSCRIBE_QOS 1
#define MQTT_PUBLISH_QOS 1
#define MQTT_PUBLISH_RETAIN 0

// Tópico usado para: last will and testament
#define MQTT_WILL_TOPIC "/online"
#define MQTT_WILL_MSG "0"
#define MQTT_WILL_QOS 1

#ifndef MQTT_DEVICE_NAME
#define MQTT_DEVICE_NAME "pico"
#endif

// Definir como 1 para adicionar o nome do cliente aos tópicos, para suportar vários dispositivos que utilizam o mesmo servidor
#ifndef MQTT_UNIQUE_TOPIC
#define MQTT_UNIQUE_TOPIC 0
#endif

/* References for this implementation:
 * raspberry-pi-pico-c-sdk.pdf, Section '4.1.1. hardware_adc'
 * pico-examples/adc/adc_console/adc_console.c */

//Leitura de temperatura do microcotrolador
static float read_onboard_temperature(const char unit);

// Requisição para publicar
static void pub_request_cb(__unused void *arg, err_t err);

// Topico MQTT
static const char *full_topic(MQTT_CLIENT_DATA_T *state, const char *name);

// Controle do LED 
static void control_led(MQTT_CLIENT_DATA_T *state, bool on);

// Publicar temperatura
static void publish_temperature(MQTT_CLIENT_DATA_T *state);

// Requisição de Assinatura - subscribe
static void sub_request_cb(void *arg, err_t err);

// Requisição para encerrar a assinatura
static void unsub_request_cb(void *arg, err_t err);

// Tópicos de assinatura
static void sub_unsub_topics(MQTT_CLIENT_DATA_T* state, bool sub);

// Dados de entrada MQTT
static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags);

// Dados de entrada publicados
static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len);

// Publicar temperatura
static void temperature_worker_fn(async_context_t *context, async_at_time_worker_t *worker);
static async_at_time_worker_t temperature_worker = { .do_work = temperature_worker_fn };

// Conexão MQTT
static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status);

// Inicializar o cliente MQTT
static void start_client(MQTT_CLIENT_DATA_T *state);

// Call back com o resultado do DNS
static void dns_found(const char *hostname, const ip_addr_t *ipaddr, void *arg);

int main(void) {

    // Inicializa todos os tipos de bibliotecas stdio padrão presentes que estão ligados ao binário.
    stdio_init_all();
    INFO_printf("======================================\n");
    INFO_printf("      CLIENTE MQTT IoT INICIANDO      \n");
    INFO_printf("======================================\n");

    // Inicializa o conversor ADC para leitura da temperatura interna
    adc_init();
    adc_set_temp_sensor_enabled(true);
    adc_select_input(4); // Canal ADC para o sensor de temperatura interno
    INFO_printf("ADC inicializado com sucesso\n");

    // TODO: Adicionar aqui a inicialização das suas bibliotecas customizadas, se necessário
    // Exemplo:
    // ssd1306_init();
    // ds18b20_init();
    // matriz_led_init();

    // Cria registro com os dados do cliente
    static MQTT_CLIENT_DATA_T state;

    // Inicializa a arquitetura do cyw43
    INFO_printf("Inicializando módulo WiFi CYW43...\n");
    if (cyw43_arch_init()) {
        ERROR_printf("ERRO: Falha ao inicializar módulo WiFi CYW43\n");
        panic("Failed to inizialize CYW43");
    }
    INFO_printf("✓ Módulo WiFi CYW43 inicializado com sucesso\n");

    // Usa identificador único da placa para o Client ID MQTT
    char unique_id_buf[PICO_UNIQUE_BOARD_ID_SIZE_BYTES * 2 + 1]; // Buffer para string hexadecimal do ID
    pico_get_unique_board_id_string(unique_id_buf, sizeof(unique_id_buf));
    // Opcional: converter para minúsculas se desejado
    // for(int i=0; unique_id_buf[i]; i++) {
    //     unique_id_buf[i] = tolower(unique_id_buf[i]);
    // }

    // Gera nome único para o cliente MQTT, Ex: pico<ID_UNICO>
    char client_id_buf[sizeof(MQTT_DEVICE_NAME) + strlen(unique_id_buf)]; // Ajuste de tamanho
    strcpy(client_id_buf, MQTT_DEVICE_NAME);
    strcat(client_id_buf, unique_id_buf);
    INFO_printf("Nome do dispositivo (Client ID): %s\n", client_id_buf);

    state.mqtt_client_info.client_id = client_id_buf;
    state.mqtt_client_info.keep_alive = MQTT_KEEP_ALIVE_S; // Keep alive in sec
#if defined(MQTT_USERNAME) && defined(MQTT_PASSWORD)
    state.mqtt_client_info.client_user = MQTT_USERNAME;
    state.mqtt_client_info.client_pass = MQTT_PASSWORD;
#else
    state.mqtt_client_info.client_user = NULL;
    state.mqtt_client_info.client_pass = NULL;
#endif
    static char will_topic[MQTT_TOPIC_LEN];
    strncpy(will_topic, full_topic(&state, MQTT_WILL_TOPIC), sizeof(will_topic));
    will_topic[sizeof(will_topic)-1] = '\0'; // Garantir terminação nula
    state.mqtt_client_info.will_topic = will_topic;
    state.mqtt_client_info.will_msg = MQTT_WILL_MSG;
    state.mqtt_client_info.will_qos = MQTT_WILL_QOS;
    state.mqtt_client_info.will_retain = true; // MQTT v3.1.1: will_retain é bool

#if LWIP_ALTCP && LWIP_ALTCP_TLS
    // Configuração TLS
#ifdef MQTT_CERT_INC
    static const uint8_t ca_cert[] = TLS_ROOT_CERT;
    static const uint8_t client_key[] = TLS_CLIENT_KEY;
    static const uint8_t client_cert[] = TLS_CLIENT_CERT;
    // Autenticação mútua (servidor e cliente)
    state.mqtt_client_info.tls_config = altcp_tls_create_config_client_2wayauth(ca_cert, sizeof(ca_cert),
            client_key, sizeof(client_key), NULL, 0, client_cert, sizeof(client_cert));
#if ALTCP_MBEDTLS_AUTHMODE != MBEDTLS_SSL_VERIFY_REQUIRED
    WARN_printf("Warning: tls without verification is insecure\n");
#endif
#else
    // Apenas TLS, sem autenticação de cliente por certificado (o servidor ainda pode ser verificado se ca_cert for fornecido)
    // Para altcp_tls_create_config_client(NULL, 0), nenhuma verificação de certificado de servidor é feita.
    state.mqtt_client_info.tls_config = altcp_tls_create_config_client(NULL, 0); 
    WARN_printf("Warning: tls without a root CA certificate for server verification is insecure\n");
#endif
#endif

    // Conectar à rede WiFI - fazer um loop até que esteja conectado
    INFO_printf("--------------------------------------\n");
    INFO_printf("Tentando conectar ao WiFi...\n");
    INFO_printf("SSID: %s\n", WIFI_SSID);
    cyw43_arch_enable_sta_mode();
    
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        ERROR_printf("ERRO: Falha ao conectar no WiFi\n");
        ERROR_printf("Verifique:\n");
        ERROR_printf("- Nome da rede (SSID): %s\n", WIFI_SSID);
        ERROR_printf("- Senha da rede\n");
        ERROR_printf("- Sinal da rede WiFi\n");
        panic("Failed to connect");
    }
    
    INFO_printf("✓ CONECTADO AO WiFi COM SUCESSO!\n");
    INFO_printf("✓ Rede: %s\n", WIFI_SSID);
    
    // Mostra o IP obtido
    cyw43_arch_lwip_begin();
    INFO_printf("✓ Endereço IP obtido: %s\n", ipaddr_ntoa(netif_ip4_addr(netif_default))); // Usar netif_default e netif_ip4_addr
    cyw43_arch_lwip_end();
    
    INFO_printf("--------------------------------------\n");

    //Faz um pedido de DNS para o endereço IP do servidor MQTT
    INFO_printf("Resolvendo DNS do servidor MQTT: %s\n", MQTT_SERVER);
    cyw43_arch_lwip_begin();
    int err = dns_gethostbyname(MQTT_SERVER, &state.mqtt_server_address, dns_found, &state);
    cyw43_arch_lwip_end();

    // Se tiver o endereço, inicia o cliente
    if (err == ERR_OK) {
        INFO_printf("✓ DNS resolvido diretamente\n");
        start_client(&state);
    } else if (err != ERR_INPROGRESS) { // ERR_INPROGRESS means expect a callback
        ERROR_printf("ERRO: Falha na resolução DNS (código: %d)\n", err);
        panic("dns request failed");
    } else {
        INFO_printf("Aguardando resolução DNS...\n");
    }

    // Loop principal do programa
    // Mantém a conexão Wi-Fi e MQTT ativas, processa eventos
    // Este loop será interrompido se a conexão MQTT cair e não for restabelecida,
    // ou se o cliente for explicitamente parado (state.stop_client).
    while (!state.stop_client) { // Modificado para verificar state.stop_client
        cyw43_arch_poll(); // Processa eventos Wi-Fi e LwIP
        // Espera por trabalho ou timeout. cyw43_arch_wait_for_work_until pode ser usado
        // para um controle mais fino do tempo de espera, especialmente com FreeRTOS.
        // Para um loop simples, um pequeno delay pode ser usado para não sobrecarregar a CPU.
        busy_wait_ms(1); // Pequeno delay para evitar 100% de uso da CPU
                         // Em um sistema com FreeRTOS, esta lógica seria diferente (ex: vTaskDelay)

        // Verifica se a conexão MQTT ainda está ativa e se precisamos reconectar
        if (state.connect_done && !mqtt_client_is_connected(state.mqtt_client_inst)) {
            INFO_printf("⚠️ Conexão MQTT perdida. Tentando reconectar...\n");
            // Tenta resolver o DNS novamente e reconectar
            cyw43_arch_lwip_begin();
            err = dns_gethostbyname(MQTT_SERVER, &state.mqtt_server_address, dns_found, &state);
            cyw43_arch_lwip_end();
            if (err == ERR_OK) {
                start_client(&state); // Tenta reconectar
            } else if (err != ERR_INPROGRESS) {
                ERROR_printf("ERRO: Falha na resolução DNS ao tentar reconectar (código: %d)\n", err);
            }
            // Aguarda um tempo antes de tentar novamente para não sobrecarregar
            // Em um cenário real, implementar um backoff exponencial seria ideal.
            busy_wait_ms(5000); 
        }
    }

    INFO_printf("Cliente MQTT finalizando...\n");
    // Limpeza final, se necessário (desconectar MQTT, etc.)
    if (mqtt_client_is_connected(state.mqtt_client_inst)) {
        mqtt_disconnect(state.mqtt_client_inst);
    }
    // mqtt_client_free(state.mqtt_client_inst); // Liberar a instância do cliente
    // cyw43_arch_deinit(); // Desinicializar o módulo Wi-Fi

    return 0;
}

/* References for this implementation:
 * raspberry-pi-pico-c-sdk.pdf, Section '4.1.1. hardware_adc'
 * pico-examples/adc/adc_console/adc_console.c */
static float read_onboard_temperature(const char unit) {

    /* 12-bit conversion, assume max value == ADC_VREF == 3.3 V */
    const float conversionFactor = 3.3f / (1 << 12);

    float adc = (float)adc_read() * conversionFactor;
    float tempC = 27.0f - (adc - 0.706f) / 0.001721f;

    if (unit == 'C' || unit != 'F') { // Default to Celsius
        return tempC;
    } else if (unit == 'F') {
        return tempC * 9 / 5 + 32;
    }

    return -1.0f; // Should not happen
}

// Callback para requisição de publicação
static void pub_request_cb(__unused void *arg, err_t err) {
    if (err != ERR_OK) { // ERR_OK é 0
        ERROR_printf("ERRO: Falha na publicação MQTT (código: %d)\n", err);
    }
}

// Monta o tópico completo, adicionando o client_id se MQTT_UNIQUE_TOPIC estiver habilitado
static const char *full_topic(MQTT_CLIENT_DATA_T *state, const char *name) {
#if MQTT_UNIQUE_TOPIC
    static char full_topic_buf[MQTT_TOPIC_LEN]; // Buffer estático para o tópico completo
    snprintf(full_topic_buf, sizeof(full_topic_buf), "/%s%s", state->mqtt_client_info.client_id, name);
    return full_topic_buf;
#else
    return name; // Retorna o nome base do tópico
#endif
}

// Controla o LED da placa e publica seu estado
static void control_led(MQTT_CLIENT_DATA_T *state, bool on) {
    const char* message = on ? "On" : "Off";
    if (on) {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        INFO_printf("✓ LED ligado\n");
    } else {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        INFO_printf("✓ LED desligado\n");
    }

    mqtt_publish(state->mqtt_client_inst, full_topic(state, "/led/state"), message, strlen(message), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
}

// Publica a leitura da temperatura
static void publish_temperature(MQTT_CLIENT_DATA_T *state) {
    static float old_temperature = -999.0f; // Inicializa com um valor improvável
    const char *temperature_topic_name = full_topic(state, "/temperature"); // Nome do tópico de temperatura
    float temperature = read_onboard_temperature(TEMPERATURE_UNITS);
    
    // Publica apenas se a temperatura mudou significativamente (ex: 0.1 grau)
    // Isso evita publicações excessivas se a leitura flutuar minimamente.
    if (fabs(temperature - old_temperature) > 0.1f) { 
        old_temperature = temperature;
        char temp_str[16];
        snprintf(temp_str, sizeof(temp_str), "%.2f", temperature);
        INFO_printf("📊 Publicando temperatura: %s°%c no tópico: %s\n", temp_str, TEMPERATURE_UNITS, temperature_topic_name);
        mqtt_publish(state->mqtt_client_inst, temperature_topic_name, temp_str, strlen(temp_str), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
    }
}

// Callback para requisição de assinatura (subscribe)
static void sub_request_cb(void *arg, err_t err) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
    if (err != ERR_OK) {
        ERROR_printf("ERRO: Falha na assinatura (código: %d)\n", err);
        // Em um cenário real, pode-se tentar novamente ou tratar o erro de forma mais robusta.
        // panic("subscribe request failed %d", err); // Evitar panic em produção se possível
    } else {
        INFO_printf("✓ Assinatura bem-sucedida para um tópico\n");
    }
    state->subscribe_count++; // Incrementa mesmo em caso de erro para manter a lógica de parada
}

// Callback para requisição de cancelamento de assinatura (unsubscribe)
static void unsub_request_cb(void *arg, err_t err) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
    if (err != ERR_OK) {
        ERROR_printf("ERRO: Falha ao cancelar assinatura (código: %d)\n", err);
        // panic("unsubscribe request failed %d", err);
    } else {
         INFO_printf("✓ Cancelamento de assinatura bem-sucedido para um tópico\n");
    }
    state->subscribe_count--;
    if (state->subscribe_count < 0) state->subscribe_count = 0; // Evitar contagem negativa

    // Para o cliente se solicitado e todas as assinaturas foram canceladas
    if (state->subscribe_count <= 0 && state->stop_client) {
        mqtt_disconnect(state->mqtt_client_inst);
    }
}

// Assina ou cancela assinatura dos tópicos MQTT definidos
static void sub_unsub_topics(MQTT_CLIENT_DATA_T* state, bool sub) {
    mqtt_request_cb_t cb = sub ? sub_request_cb : unsub_request_cb;
    const char* action = sub ? "Assinando" : "Cancelando assinatura d"; // Ajuste na mensagem
    
    INFO_printf("%sos seguintes tópicos MQTT:\n", action);
    const char* topics_to_manage[] = {"/led", "/print", "/ping", "/exit"};
    int num_topics = sizeof(topics_to_manage) / sizeof(topics_to_manage[0]);

    for (int i = 0; i < num_topics; ++i) {
        const char* topic_full_name = full_topic(state, topics_to_manage[i]);
        INFO_printf("  - %s\n", topic_full_name);
        mqtt_sub_unsub(state->mqtt_client_inst, topic_full_name, MQTT_SUBSCRIBE_QOS, cb, state, sub);
    }
}

// Callback para dados recebidos em um tópico assinado (parte dos dados)
static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
    
    // Concatena os dados recebidos no buffer 'state->data'
    // Esta implementação simples assume que as mensagens são pequenas e cabem no buffer.
    // Para mensagens maiores, seria necessário um buffer dinâmico ou processamento em partes.
    if (state->len + len <= sizeof(state->data) -1) {
        memcpy(state->data + state->len, data, len);
        state->len += len;
        state->data[state->len] = '\0'; // Garante terminação nula
    } else {
        ERROR_printf("ERRO: Buffer de dados MQTT insuficiente para mensagem no tópico %s\n", state->topic);
        // Limpa o buffer para a próxima mensagem
        state->len = 0; 
        state->data[0] = '\0';
        return;
    }

    // Se for o último fragmento da mensagem (MQTT_PUBSUB_FLAG_LAST)
    if (flags & MQTT_DATA_FLAG_LAST) {
        INFO_printf("📩 Mensagem recebida - Tópico: %s, Dados: %s\n", state->topic, state->data);
        
        // Determina o tópico base (sem o client_id, se MQTT_UNIQUE_TOPIC estiver ativo)
        const char *basic_topic;
        #if MQTT_UNIQUE_TOPIC
            if (strncmp(state->topic, "/", 1) == 0 && 
                strncmp(state->topic + 1, state->mqtt_client_info.client_id, strlen(state->mqtt_client_info.client_id)) == 0) {
                basic_topic = state->topic + strlen(state->mqtt_client_info.client_id) + 1;
            } else {
                basic_topic = state->topic; // Não corresponde ao padrão esperado
            }
        #else
            basic_topic = state->topic;
        #endif

        if (strcmp(basic_topic, "/led") == 0) {
            if (lwip_stricmp(state->data, "On") == 0 || strcmp(state->data, "1") == 0)
                control_led(state, true);
            else if (lwip_stricmp(state->data, "Off") == 0 || strcmp(state->data, "0") == 0)
                control_led(state, false);
        } else if (strcmp(basic_topic, "/print") == 0) {
            INFO_printf("💬 Print solicitado: %s\n", state->data);
        } else if (strcmp(basic_topic, "/ping") == 0) {
            char buf[16]; // Buffer para uptime em segundos
            snprintf(buf, sizeof(buf), "%lu", to_ms_since_boot(get_absolute_time()) / 1000);
            INFO_printf("🏓 Ping recebido - Respondendo com uptime: %s segundos\n", buf);
            mqtt_publish(state->mqtt_client_inst, full_topic(state, "/uptime"), buf, strlen(buf), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
        } else if (strcmp(basic_topic, "/exit") == 0) {
            INFO_printf("🚪 Comando de saída recebido - Finalizando cliente\n");
            state->stop_client = true; // Sinaliza para parar o cliente
            sub_unsub_topics(state, false); // Cancela todas as assinaturas
            // A desconexão ocorrerá no loop principal ou no callback de unsub_request_cb
        }
        
        // Reseta o buffer para a próxima mensagem
        state->len = 0;
        state->data[0] = '\0';
    }
}

// Callback para início de publicação de dados em um tópico assinado
static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
    strncpy(state->topic, topic, sizeof(state->topic) -1);
    state->topic[sizeof(state->topic)-1] = '\0'; // Garantir terminação nula
    state->len = 0; // Prepara o buffer para receber os dados da mensagem
    // INFO_printf("Recebendo mensagem no tópico: %s (tamanho total: %lu)\n", topic, tot_len);
}

// Worker para publicar a temperatura periodicamente
static void temperature_worker_fn(async_context_t *context, async_at_time_worker_t *worker) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)worker->user_data;
    if (mqtt_client_is_connected(state->mqtt_client_inst)) { // Publica apenas se conectado
        publish_temperature(state);
    }
    // Reagenda o worker
    async_context_add_at_time_worker_in_ms(context, worker, TEMP_WORKER_TIME_S * 1000);
}

// Callback para o status da conexão MQTT
static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
    if (status == MQTT_CONNECT_ACCEPTED) {
        INFO_printf("✓ CONECTADO AO BROKER MQTT COM SUCESSO!\n");
        INFO_printf("✓ Servidor: %s\n", MQTT_SERVER); // MQTT_SERVER pode ser um hostname
        INFO_printf("✓ IP do Servidor: %s\n", ipaddr_ntoa(&state->mqtt_server_address)); // IP resolvido
        INFO_printf("✓ Cliente ID: %s\n", state->mqtt_client_info.client_id);
        
        state->connect_done = true; // Sinaliza que a conexão inicial foi bem-sucedida
        sub_unsub_topics(state, true); // Assina os tópicos

        // Publica o status "online" (last will and testament message é para offline inesperado)
        if (state->mqtt_client_info.will_topic) { // Publica "1" para indicar que está online
            mqtt_publish(state->mqtt_client_inst, state->mqtt_client_info.will_topic, "1", 1, MQTT_WILL_QOS, true, pub_request_cb, state);
            INFO_printf("✓ Status online publicado no tópico: %s\n", state->mqtt_client_info.will_topic);
        }

        // Inicia o worker para publicar a temperatura
        temperature_worker.user_data = state;
        async_context_add_at_time_worker_in_ms(cyw43_arch_async_context(), &temperature_worker, TEMP_WORKER_TIME_S * 1000); // Publica após o primeiro intervalo
        INFO_printf("✓ Monitoramento de temperatura iniciado (intervalo: %d segundos)\n", TEMP_WORKER_TIME_S);
        INFO_printf("======================================\n");
        INFO_printf("      SISTEMA TOTALMENTE OPERACIONAL  \n");
        INFO_printf("======================================\n");
        
    } else { // Trata outras falhas de conexão
        ERROR_printf("ERRO: Falha na conexão MQTT (status: %d)\n", status);
        state->connect_done = false; // Sinaliza que a conexão falhou ou foi perdida
        // Não usar panic aqui, pois o loop principal pode tentar reconectar.
        // Se foi uma desconexão após uma conexão bem-sucedida, o loop principal tentará reconectar.
        // Se foi uma falha na conexão inicial, o loop principal também pode tentar.
        if (status == MQTT_CONNECT_DISCONNECTED) {
             INFO_printf("⚠️ Desconectado do broker MQTT.\n");
        } else {
            ERROR_printf("Causas comuns de falha na conexão (status %d):\n", status);
            ERROR_printf("- Endereço do servidor MQTT incorreto ou inacessível.\n");
            ERROR_printf("- Credenciais MQTT (usuário/senha) inválidas.\n");
            ERROR_printf("- Client ID duplicado (se o broker não permitir).\n");
            ERROR_printf("- Problemas de rede ou firewall.\n");
            ERROR_printf("- Configuração TLS incorreta (se estiver usando TLS).\n");
        }
    }
}

// Inicializa e inicia o cliente MQTT
static void start_client(MQTT_CLIENT_DATA_T *state) {
    // Reseta o estado de conexão para permitir reconexões
    state->connect_done = false; 
    state->stop_client = false; // Garante que o cliente não está marcado para parar
    state->subscribe_count = 0; // Reseta a contagem de assinaturas

#if LWIP_ALTCP && LWIP_ALTCP_TLS
    const int port = MQTT_TLS_PORT; // Porta padrão para MQTT sobre TLS
    INFO_printf("🔒 Usando conexão TLS segura (porta %d)\n", port);
#else
    const int port = MQTT_PORT; // Porta padrão para MQTT não seguro
    INFO_printf("⚠️  Aviso: Conexão sem TLS (não segura) (porta %d)\n", port);
#endif

    if (state->mqtt_client_inst == NULL) { // Cria a instância apenas se não existir
        state->mqtt_client_inst = mqtt_client_new();
        if (!state->mqtt_client_inst) {
            ERROR_printf("ERRO: Falha ao criar instância do cliente MQTT\n");
            panic("MQTT client instance creation error");
        }
    }
    
    INFO_printf("📱 IP local: %s\n", ipaddr_ntoa(netif_ip4_addr(netif_default)));
    INFO_printf("🌐 Conectando ao broker MQTT: %s (IP: %s, porta %d)\n", MQTT_SERVER, ipaddr_ntoa(&state->mqtt_server_address), port);

    cyw43_arch_lwip_begin(); // Bloqueia o LwIP core
    err_t err = mqtt_client_connect(state->mqtt_client_inst, &state->mqtt_server_address, port, mqtt_connection_cb, state, &state->mqtt_client_info);
    
    if (err != ERR_OK) {
        ERROR_printf("ERRO: Falha ao iniciar conexão com o broker MQTT (código: %d)\n", err);
        // Não usar panic, o loop principal pode tentar novamente.
    }
#if LWIP_ALTCP && LWIP_ALTCP_TLS
    // Importante para SNI (Server Name Indication) em TLS, se o servidor MQTT usar
    if (state->mqtt_client_inst->conn) { // Verifica se a conexão TCP foi estabelecida
         mbedtls_ssl_set_hostname(altcp_tls_context(state->mqtt_client_inst->conn), MQTT_SERVER);
    }
#endif
    mqtt_set_inpub_callback(state->mqtt_client_inst, mqtt_incoming_publish_cb, mqtt_incoming_data_cb, state);
    cyw43_arch_lwip_end(); // Desbloqueia o LwIP core
}

// Callback para o resultado da resolução DNS
static void dns_found(const char *hostname, const ip_addr_t *ipaddr, void *arg) {
    MQTT_CLIENT_DATA_T *state = (MQTT_CLIENT_DATA_T*)arg;
    if (ipaddr) {
        INFO_printf("✓ DNS resolvido: %s -> %s\n", hostname, ipaddr_ntoa(ipaddr));
        state->mqtt_server_address = *ipaddr;
        start_client(state); // Inicia a conexão MQTT com o IP resolvido
    } else {
        ERROR_printf("ERRO: Falha na resolução DNS para %s\n", hostname);
        // Em um cenário real, poderia tentar novamente após um tempo.
        // panic("dns request failed"); // Evitar panic se possível
    }
}