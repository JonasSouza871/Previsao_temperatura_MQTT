#include <math.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "pico/unique_id.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "lwip/apps/mqtt.h"
#include "lwip/apps/mqtt_priv.h"
#include "lwip/dns.h"
#include "lwip/altcp_tls.h"
#include "ssd1306.h"
#include "ds18b20.h"
#include "matriz_led.h"

//==============================================================================
// Definições e Configurações Iniciais
//==============================================================================

// Definições de QoS e Retain para MQTT
#define MQTT_PUBLISH_QOS 1
#define MQTT_PUBLISH_RETAIN 0

// Definições para funções de log
#define INFO_printf printf
#define ERROR_printf printf

// Configurações WiFi e MQTT
#define WIFI_SSID "Nome_wifi"
#define WIFI_PASSWORD "Senha_wifi"
#define MQTT_SERVER "ip_host"
#define MQTT_USERNAME "Usario_mqtt"
#define MQTT_PASSWORD "Senha_mqtt_"

#define TEMPERATURE_UNITS 'C'
#define TEMP_WORKER_TIME_S 10
#define MQTT_KEEP_ALIVE_S 60
#define MQTT_SUBSCRIBE_QOS 1
#define MQTT_WILL_TOPIC "/online"
#define MQTT_WILL_MSG "0"
#define MQTT_WILL_QOS 1
#define MQTT_DEVICE_NAME "pico"
#define MQTT_UNIQUE_TOPIC 0
#define MQTT_TOPIC_LEN 100
#define MQTT_OUTPUT_RINGBUF_SIZE 256

//==============================================================================
// Estrutura de Dados do Cliente MQTT
//==============================================================================

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

//==============================================================================
// Declarações de Funções
//==============================================================================

static float read_onboard_temperature(const char unit);
static void pub_request_cb(void *arg, err_t err);
static const char *full_topic(MQTT_CLIENT_DATA_T *state, const char *name);
static void control_led(MQTT_CLIENT_DATA_T *state, bool on);
static void publish_temperature(MQTT_CLIENT_DATA_T *state);
static void sub_request_cb(void *arg, err_t err);
static void unsub_request_cb(void *arg, err_t err);
static void sub_unsub_topics(MQTT_CLIENT_DATA_T* state, bool sub);
static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags);
static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len);
static void temperature_worker_fn(async_context_t *context, async_at_time_worker_t *worker);
static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status);
static void start_client(MQTT_CLIENT_DATA_T *state);
static void dns_found(const char *hostname, const ip_addr_t *ipaddr, void *arg);

//==============================================================================
// Função Principal (main)
//==============================================================================

int main(void) {
    // Inicialização da entrada e saída padrão (serial)
    stdio_init_all();
    INFO_printf("======================================\n");
    INFO_printf("      CLIENTE MQTT IoT INICIANDO      \n");
    INFO_printf("======================================\n");

    // Inicialização do ADC para leitura da temperatura interna
    adc_init();
    adc_set_temp_sensor_enabled(true);
    adc_select_input(4);
    INFO_printf("ADC inicializado com sucesso\n");

    // Inicialização do módulo WiFi CYW43
    if (cyw43_arch_init()) {
        ERROR_printf("ERRO: Falha ao inicializar módulo WiFi CYW43\n");
        panic("Failed to initialize CYW43");
    }
    INFO_printf("✓ Módulo WiFi CYW43 inicializado com sucesso\n");

    // Inicialização da estrutura do cliente MQTT
    static MQTT_CLIENT_DATA_T state = {0};
    char unique_id_buf[PICO_UNIQUE_BOARD_ID_SIZE_BYTES * 2 + 1];
    pico_get_unique_board_id_string(unique_id_buf, sizeof(unique_id_buf));

    char client_id_buf[sizeof(MQTT_DEVICE_NAME) + sizeof(unique_id_buf) + 1];
    snprintf(client_id_buf, sizeof(client_id_buf), "%s%s", MQTT_DEVICE_NAME, unique_id_buf);
    INFO_printf("Nome do dispositivo (Client ID): %s\n", client_id_buf);

    // Configuração das informações do cliente MQTT
    state.mqtt_client_info.client_id = client_id_buf;
    state.mqtt_client_info.keep_alive = MQTT_KEEP_ALIVE_S;
    state.mqtt_client_info.client_user = MQTT_USERNAME;
    state.mqtt_client_info.client_pass = MQTT_PASSWORD;
    static char will_topic[MQTT_TOPIC_LEN];
    snprintf(will_topic, sizeof(will_topic), "%s", full_topic(&state, MQTT_WILL_TOPIC));
    state.mqtt_client_info.will_topic = will_topic;
    state.mqtt_client_info.will_msg = MQTT_WILL_MSG;
    state.mqtt_client_info.will_qos = MQTT_WILL_QOS;
    state.mqtt_client_info.will_retain = true;

    // Conexão WiFi no modo estação (STA)
    cyw43_arch_enable_sta_mode();
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        ERROR_printf("ERRO: Falha ao conectar no WiFi\n");
        panic("Failed to connect");
    }
    INFO_printf("✓ CONECTADO AO WiFi COM SUCESSO!\n");
    INFO_printf("✓ Rede: %s\n", WIFI_SSID);
    INFO_printf("✓ Endereço IP obtido: %s\n", ipaddr_ntoa(netif_ip4_addr(netif_default)));

    // Resolução DNS para o broker MQTT
    cyw43_arch_lwip_begin();
    int err = dns_gethostbyname(MQTT_SERVER, &state.mqtt_server_address, dns_found, &state);
    cyw43_arch_lwip_end();
    if (err == ERR_OK) {
        INFO_printf("✓ DNS resolvido diretamente\n");
        start_client(&state);
    } else if (err != ERR_INPROGRESS) {
        ERROR_printf("ERRO: Falha na resolução DNS (código: %d)\n", err);
        panic("DNS request failed");
    } else {
        INFO_printf("Aguardando resolução DNS...\n");
    }

    // Loop principal do programa
    while (!state.stop_client) {
        cyw43_arch_poll();
        busy_wait_ms(1);
        if (state.connect_done && !mqtt_client_is_connected(state.mqtt_client_inst)) {
            INFO_printf("⚠️ Conexão MQTT perdida. Tentando reconectar...\n");
            cyw43_arch_lwip_begin();
            err = dns_gethostbyname(MQTT_SERVER, &state.mqtt_server_address, dns_found, &state);
            cyw43_arch_lwip_end();
            if (err == ERR_OK) {
                start_client(&state);
            } else if (err != ERR_INPROGRESS) {
                ERROR_printf("ERRO: Falha na resolução DNS ao tentar reconectar (código: %d)\n", err);
            }
            busy_wait_ms(5000);
        }
    }

    INFO_printf("Cliente MQTT finalizando...\n");
    if (mqtt_client_is_connected(state.mqtt_client_inst)) {
        mqtt_disconnect(state.mqtt_client_inst);
    }
    return 0;
}

//==============================================================================
// Leitura da Temperatura Interna (ADC)
//==============================================================================

static float read_onboard_temperature(const char unit) {
    const float conversionFactor = 3.3f / (1 << 12);
    float adc = (float)adc_read() * conversionFactor;
    float tempC = 27.0f - (adc - 0.706f) / 0.001721f;
    if (unit == 'C') return tempC;
    if (unit == 'F') return tempC * 9 / 5 + 32;
    return -1.0f;
}

//==============================================================================
// Callbacks MQTT para Publicação, Assinatura e Recepção de Dados
//==============================================================================

static void pub_request_cb(void *arg, err_t err) {
    if (err != ERR_OK) {
        ERROR_printf("ERRO: Falha na publicação MQTT (código: %d)\n", err);
    }
}

static void sub_request_cb(void *arg, err_t err) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
    if (err != ERR_OK) {
        ERROR_printf("ERRO: Falha na assinatura (código: %d)\n", err);
    } else {
        INFO_printf("✓ Assinatura bem-sucedida para um tópico\n");
    }
    state->subscribe_count++;
}

static void unsub_request_cb(void *arg, err_t err) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
    if (err != ERR_OK) {
        ERROR_printf("ERRO: Falha ao cancelar assinatura (código: %d)\n", err);
    } else {
        INFO_printf("✓ Cancelamento de assinatura bem-sucedido para um tópico\n");
    }
    state->subscribe_count--;
    if (state->subscribe_count <= 0 && state->stop_client) {
        mqtt_disconnect(state->mqtt_client_inst);
    }
}

static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
    strncpy(state->topic, topic, sizeof(state->topic) - 1);
    state->topic[sizeof(state->topic) - 1] = '\0';
    state->len = 0;
}

static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
    if (state->len + len <= sizeof(state->data) - 1) {
        memcpy(state->data + state->len, data, len);
        state->len += len;
        state->data[state->len] = '\0';
    } else {
        ERROR_printf("ERRO: Buffer de dados MQTT insuficiente\n");
        state->len = 0;
        state->data[0] = '\0';
        return;
    }
    if (flags & MQTT_DATA_FLAG_LAST) {
        INFO_printf("📩 Mensagem recebida - Tópico: %s, Dados: %s\n", state->topic, state->data);
        const char *basic_topic = state->topic;
#if MQTT_UNIQUE_TOPIC
        if (strncmp(state->topic, "/", 1) == 0 &&
            strncmp(state->topic + 1, state->mqtt_client_info.client_id, strlen(state->mqtt_client_info.client_id)) == 0) {
            basic_topic = state->topic + strlen(state->mqtt_client_info.client_id) + 1;
        }
#endif
        if (strcmp(basic_topic, "/led") == 0) {
            if (lwip_stricmp(state->data, "On") == 0 || strcmp(state->data, "1") == 0)
                control_led(state, true);
            else if (lwip_stricmp(state->data, "Off") == 0 || strcmp(state->data, "0") == 0)
                control_led(state, false);
        } else if (strcmp(basic_topic, "/print") == 0) {
            INFO_printf("💬 Print solicitado: %s\n", state->data);
        } else if (strcmp(basic_topic, "/ping") == 0) {
            char buf[16];
            snprintf(buf, sizeof(buf), "%lu", to_ms_since_boot(get_absolute_time()) / 1000);
            INFO_printf("🏓 Ping recebido - Respondendo com uptime: %s segundos\n", buf);
            mqtt_publish(state->mqtt_client_inst, full_topic(state, "/uptime"), buf, strlen(buf), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
        } else if (strcmp(basic_topic, "/exit") == 0) {
            INFO_printf("🚪 Comando de saída recebido - Finalizando cliente\n");
            state->stop_client = true;
            sub_unsub_topics(state, false);
        }
        state->len = 0;
        state->data[0] = '\0';
    }
}

//==============================================================================
// Funções Auxiliares MQTT
//==============================================================================

static const char *full_topic(MQTT_CLIENT_DATA_T *state, const char *name) {
#if MQTT_UNIQUE_TOPIC
    static char full_topic_buf[MQTT_TOPIC_LEN];
    snprintf(full_topic_buf, sizeof(full_topic_buf), "/%s%s", state->mqtt_client_info.client_id, name);
    return full_topic_buf;
#else
    return name;
#endif
}

static void control_led(MQTT_CLIENT_DATA_T *state, bool on) {
    const char* message = on ? "On" : "Off";
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, on ? 1 : 0);
    INFO_printf("✓ LED %s\n", on ? "ligado" : "desligado");
    mqtt_publish(state->mqtt_client_inst, full_topic(state, "/led/state"), message, strlen(message), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
}

static void publish_temperature(MQTT_CLIENT_DATA_T *state) {
    static float old_temperature = -999.0f;
    const char *temperature_topic_name = full_topic(state, "/temperature");
    float temperature = read_onboard_temperature(TEMPERATURE_UNITS);
    if (fabs(temperature - old_temperature) > 0.1f) {
        old_temperature = temperature;
        char temp_str[16];
        snprintf(temp_str, sizeof(temp_str), "%.2f", temperature);
        INFO_printf("📊 Publicando temperatura: %s°%c no tópico: %s\n", temp_str, TEMPERATURE_UNITS, temperature_topic_name);
        mqtt_publish(state->mqtt_client_inst, temperature_topic_name, temp_str, strlen(temp_str), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
    }
}

static void sub_unsub_topics(MQTT_CLIENT_DATA_T* state, bool sub) {
    mqtt_request_cb_t cb = sub ? sub_request_cb : unsub_request_cb;
    const char* action = sub ? "Assinando" : "Cancelando assinatura de";
    INFO_printf("%s seguintes tópicos MQTT:\n", action);
    const char* topics[] = {"/led", "/print", "/ping", "/exit"};
    for (size_t i = 0; i < sizeof(topics)/sizeof(topics[0]); ++i) {
        const char* topic_full_name = full_topic(state, topics[i]);
        INFO_printf("  - %s\n", topic_full_name);
        mqtt_sub_unsub(state->mqtt_client_inst, topic_full_name, MQTT_SUBSCRIBE_QOS, cb, state, sub);
    }
}

//==============================================================================
// Worker para Publicação Periódica da Temperatura
//==============================================================================

static void temperature_worker_fn(async_context_t *context, async_at_time_worker_t *worker) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)worker->user_data;
    if (mqtt_client_is_connected(state->mqtt_client_inst)) {
        publish_temperature(state);
    }
    async_context_add_at_time_worker_in_ms(context, worker, TEMP_WORKER_TIME_S * 1000);
}

//==============================================================================
// Callbacks de Conexão MQTT
//==============================================================================

static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
    if (status == MQTT_CONNECT_ACCEPTED) {
        INFO_printf("✓ CONECTADO AO BROKER MQTT COM SUCESSO!\n");
        state->connect_done = true;
        sub_unsub_topics(state, true);
        if (state->mqtt_client_info.will_topic) {
            mqtt_publish(state->mqtt_client_inst, state->mqtt_client_info.will_topic, "1", 1, MQTT_WILL_QOS, true, pub_request_cb, state);
            INFO_printf("✓ Status online publicado no tópico: %s\n", state->mqtt_client_info.will_topic);
        }
        static async_at_time_worker_t temperature_worker = { .do_work = temperature_worker_fn };
        temperature_worker.user_data = state;
        async_context_add_at_time_worker_in_ms(cyw43_arch_async_context(), &temperature_worker, TEMP_WORKER_TIME_S * 1000);
        INFO_printf("✓ Monitoramento de temperatura iniciado (intervalo: %d segundos)\n", TEMP_WORKER_TIME_S);
    } else {
        ERROR_printf("ERRO: Falha na conexão MQTT (status: %d)\n", status);
        state->connect_done = false;
    }
}

//==============================================================================
// Inicialização e Conexão do Cliente MQTT
//==============================================================================

static void start_client(MQTT_CLIENT_DATA_T *state) {
    state->connect_done = false;
    state->stop_client = false;
    state->subscribe_count = 0;
    const int port = LWIP_ALTCP_TLS ? MQTT_TLS_PORT : MQTT_PORT;
    if (state->mqtt_client_inst == NULL) {
        state->mqtt_client_inst = mqtt_client_new();
        if (!state->mqtt_client_inst) {
            ERROR_printf("ERRO: Falha ao criar instância do cliente MQTT\n");
            panic("MQTT client instance creation error");
        }
    }
    cyw43_arch_lwip_begin();
    err_t err = mqtt_client_connect(state->mqtt_client_inst, &state->mqtt_server_address, port, mqtt_connection_cb, state, &state->mqtt_client_info);
    if (err != ERR_OK) {
        ERROR_printf("ERRO: Falha ao iniciar conexão com o broker MQTT (código: %d)\n", err);
    }
    mqtt_set_inpub_callback(state->mqtt_client_inst, mqtt_incoming_publish_cb, mqtt_incoming_data_cb, state);
    cyw43_arch_lwip_end();
}

//==============================================================================
// Callback de Resolução DNS
//==============================================================================

static void dns_found(const char *hostname, const ip_addr_t *ipaddr, void *arg) {
    MQTT_CLIENT_DATA_T *state = (MQTT_CLIENT_DATA_T*)arg;
    if (ipaddr) {
        INFO_printf("✓ DNS resolvido: %s -> %s\n", hostname, ipaddr_ntoa(ipaddr));
        state->mqtt_server_address = *ipaddr;
        start_client(state);
    } else {
        ERROR_printf("ERRO: Falha na resolução DNS para %s\n", hostname);
    }
}
