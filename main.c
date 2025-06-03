#include <limits.h>
#include <math.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "pico/time.h"
#include "ssd1306.h"
#include "string.h"
#include "ds18b20.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "matriz_led.h"
#include "pico/cyw43_arch.h"
#include "pico/unique_id.h"
#include "lwip/apps/mqtt.h"
#include "lwip/apps/mqtt_priv.h"
#include "lwip/dns.h"
#include "lwip/altcp_tls.h"

// ================================================================================================
// CONFIGURAÇÕES E DEFINIÇÕES
// ================================================================================================

// Pinos do hardware
#define PINO_SDA_I2C 14 // Pino SDA para comunicação I2C com o display
#define PINO_SCL_I2C 15 // Pino SCL para comunicação I2C com o display
#define PINO_ADC 26 // Pino do conversor analógico-digital para o joystick
#define PINO_BOTAO_A 5 // Botão para avançar de tela
#define PINO_BOTAO_B 6 // Botão para voltar à tela inicial
#define PINO_DS18B20 16 // Pino do sensor de temperatura DS18B20
#define PINO_LED_VERDE 11 // Pino do LED verde
#define PINO_LED_VERMELHO 13 // Pino do LED vermelho
#define PINO_BUZZER 10 // Pino do buzzer

// Configurações do sistema de previsão
#define TAMANHO_HISTORICO_TEMP 30 // Armazena as últimas 30 leituras (5 minutos de histórico)
#define INTERVALO_PREVISAO_SEGUNDOS 300 // Previsão para 5 minutos no futuro
#define INTERVALO_LEITURA_SEGUNDOS 5 // Intervalo entre leituras de temperatura

// Constantes do sistema
#define DEBOUNCE_JOYSTICK_MS 300 // Tempo de debounce para o joystick (ms)
#define DEBOUNCE_BOTAO_MS 50 // Tempo de debounce para os botões (ms)
#define TIMEOUT_ATUALIZACAO_DISPLAY_MS 100 // Timeout para atualização do display (ms)
#define PISCAR_INTERVALO_MS 500 // Intervalo para piscar LEDs/matriz e beeps

// Configurações MQTT
#define WIFI_SSID "Carmelia"
#define WIFI_PASSWORD "25a28b4235"
#define MQTT_SERVER "192.168.0.105"
#define MQTT_USERNAME "Jonas"
#define MQTT_PASSWORD "jonass12"
#define MQTT_TOPIC_LEN 100
#define MQTT_DEVICE_NAME "pico"
#define MQTT_PORT 1883

// Estruturas para filas
typedef struct
{
    float temperatura; // Temperatura lida
    TickType_t marca_tempo; // Timestamp da leitura
} DadosTemperatura_t;

typedef struct
{
    float previsao_linear; // Previsão linear
} ResultadosPrevisao_t;

typedef enum
{
    COMANDO_PROXIMA_TELA,
    COMANDO_TELA_ANTERIOR,
    COMANDO_AJUSTAR_URGENCIA_SUBIR,
    COMANDO_AJUSTAR_URGENCIA_DESCER
} TipoComando_t;

typedef struct
{
    TipoComando_t tipo; // Tipo de comando
    int valor; // Valor associado (ex.: ajuste de urgência)
} ComandoUsuario_t;

// ================================================================================================
// ESTRUTURAS E VARIÁVEIS GLOBAIS
// ================================================================================================

// Estado do sistema
typedef struct
{
    int temperatura_urgencia; // Temperatura limite para alertas
    float temperatura_atual; // Temperatura atual lida pelo sensor
    float temperatura_prevista; // Temperatura prevista para o futuro
    int tela_atual; // Tela atual exibida (0=urgência, 1=resumo)
    bool configuracao_concluida; // Indica se a temperatura de urgência foi configurada
} EstadoSistema_t;

// Dados de previsão de temperatura
typedef struct
{
    float historico_temperatura[TAMANHO_HISTORICO_TEMP]; // Histórico de temperaturas
    float historico_tempo[TAMANHO_HISTORICO_TEMP]; // Histórico de tempos das leituras
    int indice_historico; // Índice atual no histórico
    bool historico_preenchido; // Indica se o histórico está cheio
} PrevisaoTemperatura_t;

// Estrutura para leitura segura do estado
typedef struct
{
    float temperatura_atual;
    float temperatura_prevista;
    int temperatura_urgencia;
    int tela_atual;
    bool configuracao_concluida;
} EstadoSistemaLeitura_t;

// Estrutura para cliente MQTT
typedef struct {
    mqtt_client_t* mqtt_client_inst;
    struct mqtt_connect_client_info_t mqtt_client_info;
    ip_addr_t mqtt_server_address;
    bool connect_done;
} MQTT_CLIENT_DATA_T;

// Variáveis globais
static EstadoSistema_t estado_sistema = {
    .temperatura_urgencia = 30,
    .temperatura_atual = 0.0,
    .temperatura_prevista = 0.0,
    .tela_atual = 0,
    .configuracao_concluida = false
};

static PrevisaoTemperatura_t previsao_temperatura = {0};
static ssd1306_t display;
static MQTT_CLIENT_DATA_T mqtt_state;

// Handles do FreeRTOS
static SemaphoreHandle_t mutex_display;
static SemaphoreHandle_t mutex_estado_sistema;
static TaskHandle_t tarefa_temperatura_handle;
static TaskHandle_t tarefa_display_handle;
static TaskHandle_t tarefa_entrada_handle;

// Handles das filas
static QueueHandle_t fila_dados_temperatura;
static QueueHandle_t fila_resultados_previsao;
static QueueHandle_t fila_comandos_usuario;

// Variáveis para controle de piscar e beeps
static TickType_t ultimo_piscar = 0;
static bool visivel = true;

static uint slice_buzzer;
static uint channel_buzzer;

// Aciona/desliga o PWM do buzzer
static inline void buzzer_on(void)  { pwm_set_enabled(slice_buzzer, true); }
static inline void buzzer_off(void) { pwm_set_enabled(slice_buzzer, false); }

// ================================================================================================
// FUNÇÕES AUXILIARES - GERENCIAMENTO DE ESTADO
// ================================================================================================

static void ler_estado_sistema(EstadoSistemaLeitura_t *leitura)
{
    if (xSemaphoreTake(mutex_estado_sistema, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        leitura->temperatura_atual = estado_sistema.temperatura_atual;
        leitura->temperatura_prevista = estado_sistema.temperatura_prevista;
        leitura->temperatura_urgencia = estado_sistema.temperatura_urgencia;
        leitura->tela_atual = estado_sistema.tela_atual;
        leitura->configuracao_concluida = estado_sistema.configuracao_concluida;
        xSemaphoreGive(mutex_estado_sistema);
    }
    else
    {
        leitura->temperatura_atual = -99.9;
        leitura->temperatura_prevista = -99.9;
        leitura->temperatura_urgencia = 0;
        leitura->tela_atual = -1;
        leitura->configuracao_concluida = false;
    }
}

static const char *determinar_situacao(float temp_atual, float temp_prevista, int temp_urgencia)
{
    float diferenca = temp_urgencia - temp_prevista;
    if (temp_atual > temp_urgencia)
        return "Grave";
    else if (diferenca > 5.0)
        return "Normal";
    else if (diferenca >= 0.0 && diferenca <= 5.0)
        return "atencao";
    else if (diferenca < 0.0)
        return "Alerta";
    return "Desconhecida";
}

static const char* obter_cor(const char* situacao)
{
    if (strcmp(situacao, "Normal") == 0)
        return "Verde";
    else if (strcmp(situacao, "atencao") == 0)
        return "Amarelo";
    else if (strcmp(situacao, "Alerta") == 0 || strcmp(situacao, "Grave") == 0)
        return "Vermelho";
    return "Desconhecida";
}

static void tocar_beep(int duracao_ms, int pausas_restantes, int frequencia_hz)
{
    uint32_t wrap = (1000000 / frequencia_hz) - 1;
    pwm_set_wrap(slice_buzzer, wrap);
    pwm_set_chan_level(slice_buzzer, channel_buzzer, wrap / 2);
    buzzer_on();
    vTaskDelay(pdMS_TO_TICKS(duracao_ms));
    buzzer_off();
    if (pausas_restantes > 0)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
        tocar_beep(duracao_ms, pausas_restantes - 1, frequencia_hz);
    }
}

static void atualizar_indicadores_visuais(const char *situacao, bool config_concluida)
{
    if (!config_concluida)
    {
        gpio_put(PINO_LED_VERDE, 0);
        gpio_put(PINO_LED_VERMELHO, 0);
        matriz_clear();
        buzzer_off();
        return;
    }

    TickType_t tick_atual = xTaskGetTickCount();
    if ((tick_atual - ultimo_piscar) > pdMS_TO_TICKS(PISCAR_INTERVALO_MS))
    {
        visivel = !visivel;
        ultimo_piscar = tick_atual;
    }

    if (strcmp(situacao, "Normal") == 0)
    {
        gpio_put(PINO_LED_VERDE, 1);
        gpio_put(PINO_LED_VERMELHO, 0);
        matriz_draw_pattern(PAD_OK, COR_VERDE);
        buzzer_off();
    }
    else if (strcmp(situacao, "atencao") == 0)
    {
        gpio_put(PINO_LED_VERDE, 1);
        gpio_put(PINO_LED_VERMELHO, 1);
        if (visivel)
        {
            matriz_draw_pattern(PAD_EXC, COR_AMARELO);
            tocar_beep(150, 0, 1500);
        }
        else
            matriz_clear();
    }
    else if (strcmp(situacao, "Alerta") == 0)
    {
        gpio_put(PINO_LED_VERDE, 0);
        gpio_put(PINO_LED_VERMELHO, 1);
        if (visivel)
        {
            matriz_draw_pattern(PAD_X, COR_VERMELHO);
            tocar_beep(100, 1, 2000);
        }
        else
            matriz_clear();
    }
    else if (strcmp(situacao, "Grave") == 0)
    {
        gpio_put(PINO_LED_VERMELHO, visivel ? 1 : 0);
        gpio_put(PINO_LED_VERDE, 0);
        if (visivel)
        {
            matriz_draw_pattern(PAD_X, COR_VERMELHO);
            tocar_beep(80, 2, 2500);
        }
        else
            matriz_clear();
    }
    else
    {
        gpio_put(PINO_LED_VERDE, 0);
        gpio_put(PINO_LED_VERMELHO, 0);
        matriz_clear();
        buzzer_off();
    }
}

// ================================================================================================
// FUNÇÕES AUXILIARES - REGRESSÃO LINEAR
// ================================================================================================

static bool calcular_regressao_linear(float *dados_x, float *dados_y, int n, float *m, float *b)
{
    if (n < 2)
    {
        *m = 0.0;
        *b = (n > 0) ? dados_y[0] : 0.0;
        return false;
    }

    float soma_x = 0.0, soma_y = 0.0, soma_xy = 0.0, soma_x2 = 0.0;
    for (int i = 0; i < n; i++)
    {
        soma_x += dados_x[i];
        soma_y += dados_y[i];
        soma_xy += dados_x[i] * dados_y[i];
        soma_x2 += dados_x[i] * dados_x[i];
    }

    float denominador = (n * soma_x2) - (soma_x * soma_x);
    if (fabs(denominador) < 1e-6)
    {
        *m = 0.0;
        *b = soma_y / n;
        return false;
    }

    *m = ((n * soma_xy) - (soma_x * soma_y)) / denominador;
    *b = (soma_y - (*m * soma_x)) / n;
    return true;
}

static float calcular_previsao_temperatura(float tempo_atual)
{
    float m, b;
    int numero_pontos = previsao_temperatura.historico_preenchido ? TAMANHO_HISTORICO_TEMP : previsao_temperatura.indice_historico;
    if (numero_pontos >= 2 && calcular_regressao_linear(previsao_temperatura.historico_tempo, previsao_temperatura.historico_temperatura, numero_pontos, &m, &b))
    {
        float tempo_futuro = tempo_atual + INTERVALO_PREVISAO_SEGUNDOS;
        return m * tempo_futuro + b;
    }
    return estado_sistema.temperatura_atual;
}

// ================================================================================================
// TASK 1: LEITURA E PROCESSAMENTO DE TEMPERATURA
// ================================================================================================

static void tarefa_temperatura(void *pvParameters)
{
    float leitura_temperatura = 0.0;
    float temperatura_filtrada = 0.0;
    bool primeira_leitura = true;
    TickType_t tempo_inicial_ticks = xTaskGetTickCount();
    DadosTemperatura_t dados_temp;
    ResultadosPrevisao_t resultados_previsao;

    while (1)
    {
        leitura_temperatura = ds18b20_get_temperature();
        if (primeira_leitura)
        {
            temperatura_filtrada = leitura_temperatura;
            primeira_leitura = false;
        }
        else
            temperatura_filtrada = (temperatura_filtrada * 0.8) + (leitura_temperatura * 0.2);

        if (leitura_temperatura > -20.0 && leitura_temperatura < 80.0)
        {
            float tempo_atual = (float)(xTaskGetTickCount() - tempo_inicial_ticks) * portTICK_PERIOD_MS / 1000.0;
            previsao_temperatura.historico_temperatura[previsao_temperatura.indice_historico] = temperatura_filtrada;
            previsao_temperatura.historico_tempo[previsao_temperatura.indice_historico] = tempo_atual;
            previsao_temperatura.indice_historico = (previsao_temperatura.indice_historico + 1) % TAMANHO_HISTORICO_TEMP;
            if (previsao_temperatura.indice_historico == 0 && !previsao_temperatura.historico_preenchido)
                previsao_temperatura.historico_preenchido = true;

            dados_temp.temperatura = temperatura_filtrada;
            dados_temp.marca_tempo = xTaskGetTickCount();
            xQueueSend(fila_dados_temperatura, &dados_temp, portMAX_DELAY);

            resultados_previsao.previsao_linear = calcular_previsao_temperatura(tempo_atual);
            xQueueSend(fila_resultados_previsao, &resultados_previsao, portMAX_DELAY);

            if (xSemaphoreTake(mutex_estado_sistema, portMAX_DELAY) == pdTRUE)
            {
                estado_sistema.temperatura_atual = temperatura_filtrada;
                xSemaphoreGive(mutex_estado_sistema);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(INTERVALO_LEITURA_SEGUNDOS * 1000));
    }
}

// ================================================================================================
// TASK 2: CONTROLE DE ENTRADA DO USUÁRIO
// ================================================================================================

static void tarefa_entrada(void *pvParameters)
{
    uint16_t valor_adc;
    bool botao_a_pressionado = false;
    bool botao_b_pressionado = false;
    TickType_t ultima_mudanca_joystick = 0;
    ComandoUsuario_t comando_usuario;

    while (1)
    {
        valor_adc = adc_read();
        TickType_t tick_atual = xTaskGetTickCount();
        EstadoSistemaLeitura_t leitura_estado;
        ler_estado_sistema(&leitura_estado);

        if (leitura_estado.tela_atual == 0)
        {
            if ((tick_atual - ultima_mudanca_joystick) > pdMS_TO_TICKS(DEBOUNCE_JOYSTICK_MS))
            {
                bool mudou_temp = false;
                if (valor_adc > 3000)
                {
                    comando_usuario.tipo = COMANDO_AJUSTAR_URGENCIA_SUBIR;
                    comando_usuario.valor = 1;
                    mudou_temp = true;
                }
                else if (valor_adc < 1000)
                {
                    comando_usuario.tipo = COMANDO_AJUSTAR_URGENCIA_DESCER;
                    comando_usuario.valor = -1;
                    mudou_temp = true;
                }
                if (mudou_temp)
                {
                    ultima_mudanca_joystick = tick_atual;
                    xQueueSend(fila_comandos_usuario, &comando_usuario, portMAX_DELAY);
                }
            }
        }

        if (!gpio_get(PINO_BOTAO_A) && !botao_a_pressionado)
        {
            botao_a_pressionado = true;
            if (leitura_estado.tela_atual == 0)
            {
                comando_usuario.tipo = COMANDO_PROXIMA_TELA;
                comando_usuario.valor = 0;
                xQueueSend(fila_comandos_usuario, &comando_usuario, portMAX_DELAY);
                tocar_beep(100, 0, 2000);
            }
        }
        else if (gpio_get(PINO_BOTAO_A))
            botao_a_pressionado = false;

        if (!gpio_get(PINO_BOTAO_B) && !botao_b_pressionado)
        {
            botao_b_pressionado = true;
            if (leitura_estado.tela_atual != 0)
            {
                comando_usuario.tipo = COMANDO_TELA_ANTERIOR;
                comando_usuario.valor = 0;
                xQueueSend(fila_comandos_usuario, &comando_usuario, portMAX_DELAY);
                tocar_beep(100, 0, 2000);
            }
        }
        else if (gpio_get(PINO_BOTAO_B))
            botao_b_pressionado = false;

        vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_BOTAO_MS));
    }
}

// ================================================================================================
// TASK 3: CONTROLE DO DISPLAY
// ================================================================================================

static void renderizar_tela_configuracao(int temp_urgencia)
{
    ssd1306_draw_string(&display, "Temperatura", 20, 0, false);
    ssd1306_draw_string(&display, "Urgencia", 32, 16, false);
    char texto_temp[10];
    snprintf(texto_temp, sizeof(texto_temp), "%d C", temp_urgencia);
    int largura_texto = strlen(texto_temp) * 6;
    int centro_x = (128 - largura_texto) / 2;
    ssd1306_draw_string(&display, texto_temp, centro_x, 40, false);
}

static void renderizar_tela_resumo(float temp_atual, float temp_prevista, int temp_urgencia, const char *situacao)
{
    char linha_str[25];
    snprintf(linha_str, sizeof(linha_str), "Temp urg: %d C", temp_urgencia);
    ssd1306_draw_string(&display, linha_str, 0, 0, false);
    snprintf(linha_str, sizeof(linha_str), "Temp atual:%.1fC", temp_atual);
    ssd1306_draw_string(&display, linha_str, 0, 14, false);
    snprintf(linha_str, sizeof(linha_str), "Prev. 5min:%.1fC", temp_prevista);
    ssd1306_draw_string(&display, linha_str, 0, 28, false);
    snprintf(linha_str, sizeof(linha_str), "Situacao:%s", situacao);
    ssd1306_draw_string(&display, linha_str, 0, 42, false);
}

static void tarefa_display(void *pvParameters)
{
    uint32_t valor_notificacao;
    DadosTemperatura_t dados_temp;
    ResultadosPrevisao_t resultados_previsao;
    ComandoUsuario_t comando_usuario;

    while (1)
    {
        xTaskNotifyWait(0, ULONG_MAX, &valor_notificacao, pdMS_TO_TICKS(TIMEOUT_ATUALIZACAO_DISPLAY_MS));
        if (xQueueReceive(fila_dados_temperatura, &dados_temp, 0) == pdTRUE)
        {
            if (xSemaphoreTake(mutex_estado_sistema, portMAX_DELAY) == pdTRUE)
            {
                estado_sistema.temperatura_atual = dados_temp.temperatura;
                xSemaphoreGive(mutex_estado_sistema);
            }
        }
        if (xQueueReceive(fila_resultados_previsao, &resultados_previsao, 0) == pdTRUE)
        {
            if (xSemaphoreTake(mutex_estado_sistema, portMAX_DELAY) == pdTRUE)
            {
                estado_sistema.temperatura_prevista = resultados_previsao.previsao_linear;
                xSemaphoreGive(mutex_estado_sistema);
            }
        }
        if (xQueueReceive(fila_comandos_usuario, &comando_usuario, 0) == pdTRUE)
        {
            if (xSemaphoreTake(mutex_estado_sistema, portMAX_DELAY) == pdTRUE)
            {
                switch (comando_usuario.tipo)
                {
                    case COMANDO_PROXIMA_TELA:
                        estado_sistema.tela_atual = 1;
                        estado_sistema.configuracao_concluida = true;
                        break;
                    case COMANDO_TELA_ANTERIOR:
                        estado_sistema.tela_atual = 0;
                        estado_sistema.configuracao_concluida = false;
                        break;
                    case COMANDO_AJUSTAR_URGENCIA_SUBIR:
                    case COMANDO_AJUSTAR_URGENCIA_DESCER:
                        estado_sistema.temperatura_urgencia += comando_usuario.valor;
                        break;
                }
                xSemaphoreGive(mutex_estado_sistema);
            }
        }

        EstadoSistemaLeitura_t leitura_estado;
        ler_estado_sistema(&leitura_estado);
        const char *situacao = determinar_situacao(leitura_estado.temperatura_atual, leitura_estado.temperatura_prevista, leitura_estado.temperatura_urgencia);
        atualizar_indicadores_visuais(situacao, leitura_estado.configuracao_concluida);

        if (xSemaphoreTake(mutex_display, portMAX_DELAY) == pdTRUE)
        {
            ssd1306_fill(&display, false);
            switch (leitura_estado.tela_atual)
            {
                case 0:
                    renderizar_tela_configuracao(leitura_estado.temperatura_urgencia);
                    break;
                case 1:
                    renderizar_tela_resumo(leitura_estado.temperatura_atual, leitura_estado.temperatura_prevista, leitura_estado.temperatura_urgencia, situacao);
                    break;
                default:
                    ssd1306_draw_string(&display, "Erro: Tela", 0, 0, false);
                    ssd1306_draw_string(&display, "Invalida", 0, 16, false);
                    break;
            }
            ssd1306_send_data(&display);
            xSemaphoreGive(mutex_display);
        }
    }
}

// ================================================================================================
// TASK 4: PUBLICAÇÃO MQTT
// ================================================================================================

static const char *full_topic(MQTT_CLIENT_DATA_T *state, const char *name)
{
    static char full_topic[MQTT_TOPIC_LEN];
    snprintf(full_topic, sizeof(full_topic), "/%s%s", state->mqtt_client_info.client_id, name);
    return full_topic;
}

static void pub_request_cb(void *arg, err_t err)
{
    if (err != 0)
        printf("ERRO: Falha na publicação MQTT (código: %d)\n", err);
}

static void tarefa_mqtt_publisher(void *pvParameters)
{
    MQTT_CLIENT_DATA_T *state = (MQTT_CLIENT_DATA_T *)pvParameters;
    EstadoSistemaLeitura_t leitura_estado;
    char temp_medida_str[16];
    char temp_prevista_str[16];
    char setpoint_str[16];
    const char *estado_str;
    const char *cor_str;

    while (1)
    {
        if (state->connect_done)
        {
            ler_estado_sistema(&leitura_estado);
            if (leitura_estado.configuracao_concluida)
            {
                estado_str = determinar_situacao(leitura_estado.temperatura_atual, leitura_estado.temperatura_prevista, leitura_estado.temperatura_urgencia);
                cor_str = obter_cor(estado_str);

                snprintf(temp_medida_str, sizeof(temp_medida_str), "%.2f", leitura_estado.temperatura_atual);
                snprintf(temp_prevista_str, sizeof(temp_prevista_str), "%.2f", leitura_estado.temperatura_prevista);
                snprintf(setpoint_str, sizeof(setpoint_str), "%d", leitura_estado.temperatura_urgencia);

                cyw43_arch_lwip_begin();
                mqtt_publish(state->mqtt_client_inst, full_topic(state, "/temperatura_medida"), temp_medida_str, strlen(temp_medida_str), 1, 0, pub_request_cb, state);
                mqtt_publish(state->mqtt_client_inst, full_topic(state, "/temperatura_prevista"), temp_prevista_str, strlen(temp_prevista_str), 1, 0, pub_request_cb, state);
                mqtt_publish(state->mqtt_client_inst, full_topic(state, "/estado"), estado_str, strlen(estado_str), 1, 0, pub_request_cb, state);
                mqtt_publish(state->mqtt_client_inst, full_topic(state, "/setpoint"), setpoint_str, strlen(setpoint_str), 1, 0, pub_request_cb, state);
                mqtt_publish(state->mqtt_client_inst, full_topic(state, "/cor"), cor_str, strlen(cor_str), 1, 0, pub_request_cb, state);
                cyw43_arch_lwip_end();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10000)); // Publica a cada 10 segundos
    }
}

// ================================================================================================
// FUNÇÃO PRINCIPAL
// ================================================================================================

static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status)
{
    MQTT_CLIENT_DATA_T *state = (MQTT_CLIENT_DATA_T *)arg;
    if (status == MQTT_CONNECT_ACCEPTED)
    {
        printf("✓ CONECTADO AO BROKER MQTT COM SUCESSO!\n");
        state->connect_done = true;
    }
    else
        printf("ERRO: Falha ao conectar no servidor MQTT, status: %d\n", status);
}

int main(void)
{
    stdio_init_all();
    printf("Iniciando sistema...\n");

    if (cyw43_arch_init())
    {
        printf("ERRO: Falha ao inicializar módulo WiFi CYW43\n");
        return -1;
    }
    cyw43_arch_enable_sta_mode();
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000))
    {
        printf("ERRO: Falha ao conectar no WiFi\n");
        return -1;
    }

    ip_addr_t mqtt_server_address;
    int err = dns_gethostbyname(MQTT_SERVER, &mqtt_server_address, NULL, NULL);
    if (err != ERR_OK)
    {
        printf("ERRO: Falha na resolução DNS\n");
        return -1;
    }
    mqtt_state.mqtt_server_address = mqtt_server_address;

    char client_id_buf[20];
    pico_get_unique_board_id_string(client_id_buf, sizeof(client_id_buf));
    strcat(client_id_buf, "_temp_monitor");
    mqtt_state.mqtt_client_info.client_id = client_id_buf;
    mqtt_state.mqtt_client_info.keep_alive = 60;
    mqtt_state.mqtt_client_info.client_user = MQTT_USERNAME;
    mqtt_state.mqtt_client_info.client_pass = MQTT_PASSWORD;
    mqtt_state.mqtt_client_info.will_topic = NULL;

    mqtt_state.mqtt_client_inst = mqtt_client_new();
    if (!mqtt_state.mqtt_client_inst)
    {
        printf("ERRO: Falha ao criar instância do cliente MQTT\n");
        return -1;
    }

    if (mqtt_client_connect(mqtt_state.mqtt_client_inst, &mqtt_state.mqtt_server_address, MQTT_PORT, mqtt_connection_cb, &mqtt_state, &mqtt_state.mqtt_client_info) != ERR_OK)
    {
        printf("ERRO: Falha na conexão com o broker MQTT\n");
        return -1;
    }

    i2c_init(i2c1, 400 * 1000);
    gpio_set_function(PINO_SDA_I2C, GPIO_FUNC_I2C);
    gpio_set_function(PINO_SCL_I2C, GPIO_FUNC_I2C);
    gpio_pull_up(PINO_SDA_I2C);
    gpio_pull_up(PINO_SCL_I2C);

    ssd1306_init(&display, 128, 64, false, 0x3C, i2c1);
    ssd1306_config(&display);

    adc_init();
    adc_gpio_init(PINO_ADC);
    adc_select_input(0);

    gpio_init(PINO_BOTAO_A);
    gpio_set_dir(PINO_BOTAO_A, GPIO_IN);
    gpio_pull_up(PINO_BOTAO_A);
    gpio_init(PINO_BOTAO_B);
    gpio_set_dir(PINO_BOTAO_B, GPIO_IN);
    gpio_pull_up(PINO_BOTAO_B);

    gpio_init(PINO_LED_VERDE);
    gpio_set_dir(PINO_LED_VERDE, GPIO_OUT);
    gpio_init(PINO_LED_VERMELHO);
    gpio_set_dir(PINO_LED_VERMELHO, GPIO_OUT);

    gpio_set_function(PINO_BUZZER, GPIO_FUNC_PWM);
    slice_buzzer = pwm_gpio_to_slice_num(PINO_BUZZER);
    channel_buzzer = pwm_gpio_to_channel(PINO_BUZZER);
    pwm_set_clkdiv(slice_buzzer, 125.0f);
    uint32_t wrap = (1000000 / 2000) - 1;
    pwm_set_wrap(slice_buzzer, wrap);
    pwm_set_chan_level(slice_buzzer, channel_buzzer, wrap / 2);
    pwm_set_enabled(slice_buzzer, false);
    printf("Buzzer configurado como PWM no GPIO %d, frequencia base 2000 Hz\n", PINO_BUZZER);

    ds18b20_init(PINO_DS18B20);
    inicializar_matriz_led();

    for (int i = 0; i < TAMANHO_HISTORICO_TEMP; i++)
    {
        previsao_temperatura.historico_temperatura[i] = -999.0;
        previsao_temperatura.historico_tempo[i] = -1.0;
    }
    previsao_temperatura.indice_historico = 0;
    previsao_temperatura.historico_preenchido = false;

    mutex_display = xSemaphoreCreateMutex();
    mutex_estado_sistema = xSemaphoreCreateMutex();
    fila_dados_temperatura = xQueueCreate(10, sizeof(DadosTemperatura_t));
    fila_resultados_previsao = xQueueCreate(10, sizeof(ResultadosPrevisao_t));
    fila_comandos_usuario = xQueueCreate(10, sizeof(ComandoUsuario_t));

    xTaskCreate(tarefa_temperatura, "TempTask", configMINIMAL_STACK_SIZE + 512, NULL, 2, &tarefa_temperatura_handle);
    xTaskCreate(tarefa_entrada, "InputTask", configMINIMAL_STACK_SIZE + 128, NULL, 1, &tarefa_entrada_handle);
    xTaskCreate(tarefa_display, "DisplayTask", configMINIMAL_STACK_SIZE + 256, NULL, 1, &tarefa_display_handle);
    xTaskCreate(tarefa_mqtt_publisher, "MqttPubTask", configMINIMAL_STACK_SIZE + 512, &mqtt_state, 1, NULL);

    vTaskStartScheduler();
    while (1) sleep_ms(1000);
    return 0;
}