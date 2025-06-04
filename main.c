#include <stdio.h>
#include <string.h>
#include <math.h>
#include <ctype.h>
#include <limits.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico/unique_id.h"
#include "pico/cyw43_arch.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "lwip/apps/mqtt.h"
#include "lwip/dns.h"
#include "lwip/altcp_tls.h"
#include "ssd1306.h"
#include "ds18b20.h"
#include "matriz_led.h"

/*============================================================================
 * CONFIGURAÇÃO DE REDE
 * Esta seção define as credenciais de rede e MQTT. Ajuste conforme necessário.
 *===========================================================================*/
#define WIFI_SSID       ""
#define WIFI_PASSWORD   ""
#define MQTT_SERVER     ""   // IP do broker ou hostname
#define MQTT_USERNAME   ""         // Deixe "" se broker anônimo
#define MQTT_PASSWORD   ""           // Deixe "" se broker anônimo

/*============================================================================
 * DEFINIÇÕES DE HARDWARE
 * Pinos usados no projeto.
 *===========================================================================*/
#define PINO_SDA_I2C        14        // Pino SDA para I2C
#define PINO_SCL_I2C        15        // Pino SCL para I2C
#define PINO_ADC            26        // Pino para entrada analógica
#define PINO_BOTAO_A        5         // Pino do botão A
#define PINO_BOTAO_B        6         // Pino do botão B
#define PINO_DS18B20        16        // Pino do sensor de temperatura DS18B20
#define PINO_LED_VERDE      11        // Pino do LED verde
#define PINO_LED_VERMELHO   13        // Pino do LED vermelho
#define PINO_BUZZER         10        // Pino do buzzer

/*============================================================================
 * PARÂMETROS DA LÓGICA DE APLICAÇÃO
 * Constantes que controlam o comportamento do sistema.
 *===========================================================================*/
#define TAMANHO_HISTORICO_TEMP      30    // Tamanho do histórico de temperaturas
#define INTERVALO_PREVISAO_SEGUNDOS 300   // Intervalo para previsão (segundos)
#define INTERVALO_LEITURA_SEGUNDOS  5     // Intervalo de leitura da temperatura (segundos)

#define DEBOUNCE_JOYSTICK_MS        300   // Tempo de debounce para joystick (ms)
#define DEBOUNCE_BOTAO_MS           50    // Tempo de debounce para botões (ms)
#define TIMEOUT_ATUALIZACAO_DISPLAY_MS 100 // Timeout para atualização do display (ms)
#define PISCAR_INTERVALO_MS         500   // Intervalo de piscar dos indicadores (ms)

/* Constantes do método Holt para previsão */
#define ALPHA_HOLT 0.3f   // Fator de suavização do nível
#define BETA_HOLT  0.1f   // Fator de suavização da tendência

/*============================================================================
 * CONFIGURAÇÃO MQTT
 * Parâmetros para comunicação com o broker MQTT.
 *===========================================================================*/
#define MQTT_TOPIC_BASE             "/Temperatura_MQTT_Pico" // Tópico base MQTT
#define MQTT_KEEP_ALIVE_S           60    // Tempo de keep-alive (segundos)
#define TEMP_PUBLISH_INTERVAL_S     10    // Intervalo de publicação (segundos)
#define MQTT_SUBSCRIBE_QOS          1     // QoS para subscrição
#define MQTT_PUBLISH_QOS            1     // QoS para publicação
#define MQTT_PUBLISH_RETAIN         0     // Retenção de mensagens (0 = não)
#define MQTT_WILL_TOPIC             "/online" // Tópico de última vontade
#define MQTT_WILL_MSG               "0"   // Mensagem de última vontade
#define MQTT_WILL_QOS               1     // QoS da última vontade
#define MQTT_DEVICE_NAME            "pico" // Nome do dispositivo
#define MQTT_TOPIC_LEN              100   // Tamanho máximo do tópico

/*============================================================================
 * ESTRUTURAS DE DADOS
 * Definições de tipos usados no sistema.
 *===========================================================================*/
typedef struct {
    float temperatura;      // Temperatura registrada
    TickType_t marca_tempo; // Marca de tempo da leitura
} DadosTemperatura_t;

typedef struct {
    float previsao_linear;  // Previsão por regressão linear
    float previsao_holt;    // Previsão por suavização Holt
} ResultadosPrevisao_t;

typedef enum {
    COMANDO_PROXIMA_TELA,       // Avança para a próxima tela
    COMANDO_TELA_ANTERIOR,      // Retorna à tela anterior
    COMANDO_AJUSTAR_URGENCIA_SUBIR,  // Aumenta temperatura de urgência
    COMANDO_AJUSTAR_URGENCIA_DESCER  // Diminui temperatura de urgência
} TipoComando_t;

typedef struct {
    TipoComando_t tipo;     // Tipo do comando
    int valor;              // Valor associado ao comando
} ComandoUsuario_t;

typedef struct {
    int   temperatura_urgencia;      // Limite de temperatura crítica
    float temperatura_atual;         // Temperatura atual
    float temperatura_prevista;      // Previsão por regressão linear
    float temperatura_prevista_holt; // Previsão por suavização Holt
    int   tela_atual;                // Tela exibida no display
    bool  configuracao_concluida;    // Estado da configuração
} EstadoSistema_t;

typedef struct {
    float historico_temperatura[TAMANHO_HISTORICO_TEMP]; // Histórico de temperaturas
    float historico_tempo[TAMANHO_HISTORICO_TEMP];       // Histórico de tempos
    int   indice_historico;       // Índice atual no histórico
    bool  historico_preenchido;   // Indica se o histórico está completo
} PrevisaoTemperatura_t;

typedef struct {
    mqtt_client_t *inst;          // Instância do cliente MQTT
    struct mqtt_connect_client_info_t info; // Informações de conexão
    ip_addr_t server_addr;        // Endereço do servidor MQTT
    char topic[MQTT_TOPIC_LEN];   // Tópico atual
    char data[64];                // Dados recebidos
    bool conectado;               // Estado da conexão
} EstadoMQTT_t;

/*============================================================================
 * VARIÁVEIS GLOBAIS
 * Variáveis compartilhadas entre as tarefas.
 *===========================================================================*/
static EstadoSistema_t       estado_sistema = { .temperatura_urgencia = 30 };
static PrevisaoTemperatura_t previsao       = {0};
static ssd1306_t             display;

static SemaphoreHandle_t mutex_estado;     // Mutex para proteger estado_sistema
static SemaphoreHandle_t mutex_display;    // Mutex para proteger o display
static QueueHandle_t     q_temp;           // Fila para dados de temperatura
static QueueHandle_t     q_prev;           // Fila para previsões
static QueueHandle_t     q_cmd;            // Fila para comandos do usuário

static uint slice_buzzer;      // Slice PWM do buzzer
static uint channel_buzzer;    // Canal PWM do buzzer

static EstadoMQTT_t mqtt_state; // Estado da conexão MQTT

/*============================================================================
 * FUNÇÕES AUXILIARES
 * Funções utilitárias para simplificar a lógica.
 *===========================================================================*/

// Constrói um tópico MQTT completo concatenando o sufixo ao tópico base
static inline const char *topico_completo(const char *sufixo) {
    static char buf[MQTT_TOPIC_LEN];
    snprintf(buf, sizeof(buf), MQTT_TOPIC_BASE "%s", sufixo);
    return buf;
}

// Lê o estado do sistema de forma segura usando mutex
static void ler_estado(EstadoSistema_t *destino) {
    if (xSemaphoreTake(mutex_estado, pdMS_TO_TICKS(5))) {
        *destino = estado_sistema;
        xSemaphoreGive(mutex_estado);
    } else {
        memset(destino, 0, sizeof(*destino));
    }
}

// Determina a situacao com base na temperatura atual, prevista e limite
static const char *determinar_situacao(float temp_atual, float temp_prevista, int urgencia) {
    float diferenca = urgencia - temp_prevista;
    if (temp_atual > urgencia) return "Grave";
    if (diferenca > 5.0f)      return "Normal";
    if (diferenca >= 0.0f)     return "Atenção";
    return "Alerta";
}

/*============================================================================
 * PREVISÃO DE TEMPERATURA
 * Funções para calcular previsões usando regressão linear e suavização Holt.
 *===========================================================================*/

// Calcula regressão linear simples para previsão
static bool calcular_regressao(float *x, float *y, int n, float *m, float *b) {
    if (n < 2) {
        *m = 0;
        *b = (n ? y[0] : 0);
        return false;
    }
    float soma_x = 0, soma_y = 0, soma_xy = 0, soma_x2 = 0;
    for (int i = 0; i < n; i++) {
        soma_x += x[i];
        soma_y += y[i];
        soma_xy += x[i] * y[i];
        soma_x2 += x[i] * x[i];
    }
    float denominador = n * soma_x2 - soma_x * soma_x;
    if (fabsf(denominador) < 1e-6) {
        *m = 0;
        *b = soma_y / n;
        return false;
    }
    *m = (n * soma_xy - soma_x * soma_y) / denominador;
    *b = (soma_y - *m * soma_x) / n;
    return true;
}

// Calcula previsão linear com base no histórico
static float prever_linear(float tempo_atual) {
    float inclinacao, intercepcao;
    int n = previsao.historico_preenchido ? TAMANHO_HISTORICO_TEMP : previsao.indice_historico;
    if (n >= 2 && calcular_regressao(previsao.historico_tempo, previsao.historico_temperatura, n, &inclinacao, &intercepcao)) {
        return inclinacao * (tempo_atual + INTERVALO_PREVISAO_SEGUNDOS) + intercepcao;
    }
    return estado_sistema.temperatura_atual;
}

/*============================================================================
 * CONTROLE DO BUZZER
 * Função para emitir sons com o buzzer.
 *===========================================================================*/

// Emite um beep com duração, repetições e frequência específicas
static void emitir_beep(int duracao_ms, int repeticoes, int frequencia) {
    uint32_t wrap = (1000000 / frequencia) - 1;
    pwm_set_wrap(slice_buzzer, wrap);
    pwm_set_chan_level(slice_buzzer, channel_buzzer, wrap / 2);
    for (int i = 0; i <= repeticoes; i++) {
        pwm_set_enabled(slice_buzzer, true);
        vTaskDelay(pdMS_TO_TICKS(duracao_ms));
        pwm_set_enabled(slice_buzzer, false);
        if (i < repeticoes) vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/*============================================================================
 * INDICADORES VISUAIS
 * Controla LEDs, matriz de LEDs e buzzer com base na situacao.
 *===========================================================================*/
static TickType_t ultimo_piscar = 0; // Última vez que os indicadores piscaram
static bool visivel = true;          // Estado de visibilidade dos indicadores

static void atualizar_indicadores(const char *situacao, bool configurado) {
    if (!configurado) { // Desativa tudo se não configurado
        gpio_put(PINO_LED_VERDE, 0);
        gpio_put(PINO_LED_VERMELHO, 0);
        matriz_clear();
        pwm_set_enabled(slice_buzzer, false);
        return;
    }
    TickType_t agora = xTaskGetTickCount();
    if (agora - ultimo_piscar > pdMS_TO_TICKS(PISCAR_INTERVALO_MS)) {
        visivel = !visivel;
        ultimo_piscar = agora;
    }
    if (!strcmp(situacao, "Normal")) {
        gpio_put(PINO_LED_VERDE, 1);
        gpio_put(PINO_LED_VERMELHO, 0);
        matriz_draw_pattern(PAD_OK, COR_VERDE);
        pwm_set_enabled(slice_buzzer, false);
    } else if (!strcmp(situacao, "Atenção")) {
        gpio_put(PINO_LED_VERDE, 1);
        gpio_put(PINO_LED_VERMELHO, 1);
        if (visivel) {
            matriz_draw_pattern(PAD_EXC, COR_AMARELO);
            emitir_beep(150, 0, 1500);
        } else {
            matriz_clear();
        }
    } else if (!strcmp(situacao, "Alerta")) {
        gpio_put(PINO_LED_VERDE, 0);
        gpio_put(PINO_LED_VERMELHO, 1);
        if (visivel) {
            matriz_draw_pattern(PAD_X, COR_VERMELHO);
            emitir_beep(100, 1, 2000);
        } else {
            matriz_clear();
        }
    } else { // "Grave"
        gpio_put(PINO_LED_VERDE, 0);
        gpio_put(PINO_LED_VERMELHO, visivel);
        if (visivel) {
            matriz_draw_pattern(PAD_X, COR_VERMELHO);
            emitir_beep(80, 2, 2500);
        } else {
            matriz_clear();
        }
    }
}

/*============================================================================
 * EXIBIÇÃO NO DISPLAY OLED
 * Funções para desenhar as telas no display OLED.
 *===========================================================================*/

// Exibe a tela de configuração da temperatura de urgência
static void exibir_tela_configuracao(int urgencia) {
    ssd1306_draw_string(&display, "Temperatura", 20, 0, false);
    ssd1306_draw_string(&display, "Urgência", 32, 16, false);
    char texto[12];
    snprintf(texto, sizeof(texto), "%d C", urgencia);
    ssd1306_draw_string(&display, texto, (128 - strlen(texto) * 6) / 2, 40, false);
}

// Exibe a tela com resultados e situacao atual
static void exibir_tela_resultados(float temp_atual, float temp_prevista, float temp_prevista_holt, int urgencia, const char *situacao) {
    char buffer[30];
    snprintf(buffer, sizeof(buffer), "Temp urg: %d C", urgencia);
    ssd1306_draw_string(&display, buffer, 0, 0, false);
    snprintf(buffer, sizeof(buffer), "Atual: %.1fC", temp_atual);
    ssd1306_draw_string(&display, buffer, 0, 14, false);
    snprintf(buffer, sizeof(buffer), "Prev lin: %.1fC", temp_prevista);
    ssd1306_draw_string(&display, buffer, 0, 28, false);
    snprintf(buffer, sizeof(buffer), "Prev Holt: %.1fC", temp_prevista_holt);
    ssd1306_draw_string(&display, buffer, 0, 42, false);
    snprintf(buffer, sizeof(buffer), "Situação: %s", situacao);
    ssd1306_draw_string(&display, buffer, 0, 56, false);
}

/*============================================================================
 * TAREFA: LEITURA DE TEMPERATURA E PREVISÕES
 * Lê a temperatura do sensor e calcula previsões.
 *===========================================================================*/
static void tarefa_leitura_temperatura(void *param) {
    (void)param;
    float nivel = 0, tendencia = 0; // Variáveis para suavização Holt
    bool primeira_leitura = true, holt_iniciado = false;
    const int passos_adiantados = INTERVALO_PREVISAO_SEGUNDOS / INTERVALO_LEITURA_SEGUNDOS;
    TickType_t inicio = xTaskGetTickCount();
    while (1) {
        float temp = ds18b20_get_temperature();
        if (primeira_leitura) {
            nivel = temp;
            tendencia = 0;
            primeira_leitura = false;
        }
        // Filtro exponencial para atenuar ruído
        static float temp_filtrada = 0;
        static bool primeiro_filtro = true;
        temp_filtrada = primeiro_filtro ? (primeiro_filtro = false, temp) : temp_filtrada * 0.8f + temp * 0.2f;

        if (temp > -20 && temp < 80) { // Validação da temperatura
            // Atualiza histórico para regressão
            float tempo = (xTaskGetTickCount() - inicio) * portTICK_PERIOD_MS / 1000.0f;
            previsao.historico_temperatura[previsao.indice_historico] = temp_filtrada;
            previsao.historico_tempo[previsao.indice_historico] = tempo;
            previsao.indice_historico = (previsao.indice_historico + 1) % TAMANHO_HISTORICO_TEMP;
            if (!previsao.historico_preenchido && previsao.indice_historico == 0) {
                previsao.historico_preenchido = true;
            }

            // Suavização Holt
            if (!holt_iniciado) {
                nivel = temp_filtrada;
                tendencia = 0;
                holt_iniciado = true;
            }
            float nivel_anterior = nivel;
            nivel = ALPHA_HOLT * temp_filtrada + (1 - ALPHA_HOLT) * (nivel + tendencia);
            tendencia = BETA_HOLT * (nivel - nivel_anterior) + (1 - BETA_HOLT) * tendencia;
            float previsao_holt = nivel + tendencia * passos_adiantados;

            // Previsão linear
            float previsao_linear_resultado = prever_linear(tempo);

            // Envia dados para filas
            DadosTemperatura_t dados = { .temperatura = temp_filtrada, .marca_tempo = xTaskGetTickCount() };
            xQueueSend(q_temp, &dados, 0);
            ResultadosPrevisao_t resultados = { .previsao_linear = previsao_linear_resultado, .previsao_holt = previsao_holt };
            xQueueSend(q_prev, &resultados, 0);

            // Atualiza estado global
            if (xSemaphoreTake(mutex_estado, portMAX_DELAY)) {
                estado_sistema.temperatura_atual = temp_filtrada;
                estado_sistema.temperatura_prevista = previsao_linear_resultado;
                estado_sistema.temperatura_prevista_holt = previsao_holt;
                xSemaphoreGive(mutex_estado);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(INTERVALO_LEITURA_SEGUNDOS * 1000));
    }
}

/*============================================================================
 * TAREFA: ENTRADA DO USUÁRIO
 * Processa entradas do joystick e botões.
 *===========================================================================*/
static void tarefa_entrada_usuario(void *param) {
    (void)param;
    uint16_t valor_adc;
    bool botao_a_pressionado = false, botao_b_pressionado = false;
    TickType_t ultimo_joystick = 0;
    ComandoUsuario_t comando;
    while (1) {
        valor_adc = adc_read();
        TickType_t agora = xTaskGetTickCount();
        EstadoSistema_t estado;
        ler_estado(&estado);
        // Processa entrada do joystick na tela de configuração
        if (estado.tela_atual == 0 && (agora - ultimo_joystick) > pdMS_TO_TICKS(DEBOUNCE_JOYSTICK_MS)) {
            if (valor_adc > 3000) {
                comando.tipo = COMANDO_AJUSTAR_URGENCIA_SUBIR;
                comando.valor = 1;
                xQueueSend(q_cmd, &comando, 0);
                ultimo_joystick = agora;
            } else if (valor_adc < 1000) {
                comando.tipo = COMANDO_AJUSTAR_URGENCIA_DESCER;
                comando.valor = -1;
                xQueueSend(q_cmd, &comando, 0);
                ultimo_joystick = agora;
            }
        }
        // Processa botão A (próxima tela)
        if (!gpio_get(PINO_BOTAO_A) && !botao_a_pressionado) {
            botao_a_pressionado = true;
            if (estado.tela_atual == 0) {
                comando.tipo = COMANDO_PROXIMA_TELA;
                xQueueSend(q_cmd, &comando, 0);
                emitir_beep(100, 0, 2000);
            }
        } else if (gpio_get(PINO_BOTAO_A)) {
            botao_a_pressionado = false;
        }
        // Processa botão B (tela anterior)
        if (!gpio_get(PINO_BOTAO_B) && !botao_b_pressionado) {
            botao_b_pressionado = true;
            if (estado.tela_atual != 0) {
                comando.tipo = COMANDO_TELA_ANTERIOR;
                xQueueSend(q_cmd, &comando, 0);
                emitir_beep(100, 0, 2000);
            }
        } else if (gpio_get(PINO_BOTAO_B)) {
            botao_b_pressionado = false;
        }
        vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_BOTAO_MS));
    }
}

/*============================================================================
 * TAREFA: ATUALIZAÇÃO DO DISPLAY
 * Gerencia o display OLED e indicadores visuais.
 *===========================================================================*/
static void tarefa_atualizar_display(void *param) {
    (void)param;
    DadosTemperatura_t dados_temp;
    ResultadosPrevisao_t resultados_prev;
    ComandoUsuario_t comando;
    while (1) {
        xTaskNotifyWait(0, ULONG_MAX, NULL, pdMS_TO_TICKS(TIMEOUT_ATUALIZACAO_DISPLAY_MS));
        // Recebe temperatura atual
        if (xQueueReceive(q_temp, &dados_temp, 0) == pdTRUE) {
            if (xSemaphoreTake(mutex_estado, portMAX_DELAY)) {
                estado_sistema.temperatura_atual = dados_temp.temperatura;
                xSemaphoreGive(mutex_estado);
            }
        }
        // Recebe previsões
        if (xQueueReceive(q_prev, &resultados_prev, 0) == pdTRUE) {
            if (xSemaphoreTake(mutex_estado, portMAX_DELAY)) {
                estado_sistema.temperatura_prevista = resultados_prev.previsao_linear;
                estado_sistema.temperatura_prevista_holt = resultados_prev.previsao_holt;
                xSemaphoreGive(mutex_estado);
            }
        }
        // Processa comandos do usuário
        if (xQueueReceive(q_cmd, &comando, 0) == pdTRUE) {
            if (xSemaphoreTake(mutex_estado, portMAX_DELAY)) {
                switch (comando.tipo) {
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
                        estado_sistema.temperatura_urgencia += comando.valor;
                        break;
                }
                xSemaphoreGive(mutex_estado);
            }
        }
        // Atualiza display e indicadores
        EstadoSistema_t estado;
        ler_estado(&estado);
        const char *situacao = determinar_situacao(estado.temperatura_atual, estado.temperatura_prevista, estado.temperatura_urgencia);
        atualizar_indicadores(situacao, estado.configuracao_concluida);
        if (xSemaphoreTake(mutex_display, portMAX_DELAY)) {
            ssd1306_fill(&display, false);
            if (estado.tela_atual == 0) {
                exibir_tela_configuracao(estado.temperatura_urgencia);
            } else {
                exibir_tela_resultados(estado.temperatura_atual, estado.temperatura_prevista,
                                       estado.temperatura_prevista_holt, estado.temperatura_urgencia, situacao);
            }
            ssd1306_send_data(&display);
            xSemaphoreGive(mutex_display);
        }
    }
}

/*============================================================================
 * TAREFA: PUBLICAÇÃO MQTT
 * Publica dados no broker MQTT periodicamente.
 *===========================================================================*/

// Callback para erros de publicação
static void callback_publicacao(void *arg, err_t erro) {
    if (erro) printf("Erro de publicação MQTT: %d\n", erro);
}

static void tarefa_publicar_mqtt(void *param) {
    (void)param;
    while (1) {
        if (mqtt_state.conectado && mqtt_client_is_connected(mqtt_state.inst)) {
            EstadoSistema_t estado;
            ler_estado(&estado);
            char buffer[16];

            // Publica temperatura atual
            snprintf(buffer, sizeof(buffer), "%.2f", estado.temperatura_atual);
            mqtt_publish(mqtt_state.inst, topico_completo("/temperatura"), buffer, strlen(buffer),
                         MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, callback_publicacao, NULL);

            // Publica previsão por regressão linear
            snprintf(buffer, sizeof(buffer), "%.2f", estado.temperatura_prevista);
            mqtt_publish(mqtt_state.inst, topico_completo("/temperatura_previsao_regressao_linear"), buffer, strlen(buffer),
                         MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, callback_publicacao, NULL);

            // Publica previsão Holt
            snprintf(buffer, sizeof(buffer), "%.2f", estado.temperatura_prevista_holt);
            mqtt_publish(mqtt_state.inst, topico_completo("/temperatura_previsao_holt"), buffer, strlen(buffer),
                         MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, callback_publicacao, NULL);

            // Publica situacao
            const char *situacao = determinar_situacao(estado.temperatura_atual, estado.temperatura_prevista, estado.temperatura_urgencia);
            mqtt_publish(mqtt_state.inst, topico_completo("/estado"), situacao, strlen(situacao),
                         MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, callback_publicacao, NULL);

            // Publica ponto de regulagem
            snprintf(buffer, sizeof(buffer), "%d", estado.temperatura_urgencia);
            mqtt_publish(mqtt_state.inst, topico_completo("/ponto_de_regulagem"), buffer, strlen(buffer),
                         MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, callback_publicacao, NULL);
        }
        vTaskDelay(pdMS_TO_TICKS(TEMP_PUBLISH_INTERVAL_S * 1000));
    }
}

/*============================================================================
 * CALLBACKS MQTT
 * Funções de callback para eventos MQTT.
 *===========================================================================*/

// Processa dados recebidos
static void processar_dados_recebidos(void *arg, const u8_t *dados, u16_t tamanho, u8_t flags) {
    EstadoMQTT_t *estado = (EstadoMQTT_t*)arg;
    strncpy(estado->data, (const char*)dados, MIN(tamanho, sizeof(estado->data) - 1));
    estado->data[tamanho] = '\0';
    if (strcmp(estado->topic, topico_completo("/led")) == 0) {
        bool ligado = (!strcasecmp(estado->data, "on") || !strcmp(estado->data, "1"));
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, ligado);
    } else if (strcmp(estado->topic, topico_completo("/exit")) == 0) {
        mqtt_disconnect(estado->inst);
    }
}

// Registra tópico recebido
static void registrar_topico(void *arg, const char *topico, u32_t tamanho) {
    (void)tamanho;
    EstadoMQTT_t *estado = (EstadoMQTT_t*)arg;
    strncpy(estado->topic, topico, sizeof(estado->topic));
}

// Callback para subscrição
static void callback_subscricao(void *arg, err_t erro) {
    if (erro) printf("Erro de subscrição MQTT: %d\n", erro);
}

// Callback para conexão
static void callback_conexao(mqtt_client_t *cliente, void *arg, mqtt_connection_status_t status) {
    EstadoMQTT_t *estado = (EstadoMQTT_t*)arg;
    if (status == MQTT_CONNECT_ACCEPTED) {
        printf("Conexão MQTT estabelecida\n");
        estado->conectado = true;
        mqtt_sub_unsub(cliente, topico_completo("/led"), MQTT_SUBSCRIBE_QOS, callback_subscricao, estado, true);
        mqtt_sub_unsub(cliente, topico_completo("/print"), MQTT_SUBSCRIBE_QOS, callback_subscricao, estado, true);
        mqtt_sub_unsub(cliente, topico_completo("/ping"), MQTT_SUBSCRIBE_QOS, callback_subscricao, estado, true);
        mqtt_sub_unsub(cliente, topico_completo("/exit"), MQTT_SUBSCRIBE_QOS, callback_subscricao, estado, true);
        mqtt_publish(cliente, topico_completo("/online"), "1", 1, MQTT_WILL_QOS, true, callback_publicacao, NULL);
    } else {
        printf("Conexão MQTT perdida: %d\n", status);
        estado->conectado = false;
    }
}

/*============================================================================
 * TAREFA: CONEXÃO WI-FI E MQTT
 * Estabelece conexão Wi-Fi e MQTT.
 *===========================================================================*/
static void tarefa_conectar_wifi_mqtt(void *param) {
    (void)param;
    if (cyw43_arch_init()) {
        printf("Erro ao inicializar CYW43\n");
        vTaskDelete(NULL);
    }
    cyw43_arch_enable_sta_mode();
    printf("Conectando ao Wi-Fi %s...\n", WIFI_SSID);
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("Falha na conexão Wi-Fi\n");
        vTaskDelete(NULL);
    }
    printf("IP atribuído: %s\n", ipaddr_ntoa(&(netif_list->ip_addr)));
    mqtt_state.inst = mqtt_client_new();
    if (!mqtt_state.inst) {
        vTaskDelete(NULL);
    }
    static char id_cliente[12];
    pico_get_unique_board_id_string(id_cliente, sizeof(id_cliente));
    for (int i = 0; i < strlen(id_cliente); i++) {
        id_cliente[i] = tolower(id_cliente[i]);
    }
    mqtt_state.info.client_id = id_cliente;
    mqtt_state.info.keep_alive = MQTT_KEEP_ALIVE_S;
    mqtt_state.info.client_user = MQTT_USERNAME;
    mqtt_state.info.client_pass = MQTT_PASSWORD;
    mqtt_state.info.will_topic = topico_completo("/online");
    mqtt_state.info.will_msg = MQTT_WILL_MSG;
    mqtt_state.info.will_qos = MQTT_WILL_QOS;
    mqtt_state.info.will_retain = true;
    if (dns_gethostbyname(MQTT_SERVER, &mqtt_state.server_addr, NULL, NULL) != ERR_OK) {
        while (dns_gethostbyname(MQTT_SERVER, &mqtt_state.server_addr, NULL, NULL) == ERR_INPROGRESS) {
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
    printf("Broker MQTT: %s\n", ipaddr_ntoa(&mqtt_state.server_addr));
    mqtt_set_inpub_callback(mqtt_state.inst, registrar_topico, processar_dados_recebidos, &mqtt_state);
    if (mqtt_client_connect(mqtt_state.inst, &mqtt_state.server_addr, MQTT_PORT, callback_conexao, &mqtt_state, &mqtt_state.info) != ERR_OK) {
        printf("Erro ao conectar ao MQTT\n");
        vTaskDelete(NULL);
    }
    while (1) {
        cyw43_arch_poll();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/*============================================================================
 * FUNÇÃO PRINCIPAL
 * Inicializa o sistema e cria as tarefas.
 *===========================================================================*/
int main(void) {
    stdio_init_all();

    // Inicialização do I2C para o display
    i2c_init(i2c1, 400 * 1000);
    gpio_set_function(PINO_SDA_I2C, GPIO_FUNC_I2C);
    gpio_set_function(PINO_SCL_I2C, GPIO_FUNC_I2C);
    gpio_pull_up(PINO_SDA_I2C);
    gpio_pull_up(PINO_SCL_I2C);
    ssd1306_init(&display, 128, 64, false, 0x3C, i2c1);
    ssd1306_config(&display);

    // Inicialização do ADC, botões, LEDs e buzzer
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
    pwm_set_wrap(slice_buzzer, (1000000 / 2000) - 1);
    pwm_set_chan_level(slice_buzzer, channel_buzzer, (1000000 / 2000) / 2);
    pwm_set_enabled(slice_buzzer, false);

    // Inicialização do sensor e matriz de LEDs
    ds18b20_init(PINO_DS18B20);
    inicializar_matriz_led();

    // Infraestrutura do RTOS
    mutex_estado = xSemaphoreCreateMutex();
    mutex_display = xSemaphoreCreateMutex();
    q_temp = xQueueCreate(10, sizeof(DadosTemperatura_t));
    q_prev = xQueueCreate(10, sizeof(ResultadosPrevisao_t));
    q_cmd = xQueueCreate(10, sizeof(ComandoUsuario_t));

    // Criação das tarefas
    xTaskCreate(tarefa_leitura_temperatura, "Temperatura", 1024, NULL, 2, NULL);
    xTaskCreate(tarefa_entrada_usuario, "Entrada", 512, NULL, 1, NULL);
    xTaskCreate(tarefa_atualizar_display, "Display", 1024, NULL, 1, NULL);
    xTaskCreate(tarefa_conectar_wifi_mqtt, "WiFi_MQTT", 2048, NULL, 3, NULL);
    xTaskCreate(tarefa_publicar_mqtt, "Publicacao_MQTT", 768, NULL, 1, NULL);

    vTaskStartScheduler();
    while (1) tight_loop_contents();
    return 0;
}