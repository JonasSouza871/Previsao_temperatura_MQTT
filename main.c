#include <limits.h>
#include <math.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "pico/time.h"
#include "ssd1306.h"
#include "string.h"
#include "ds18b20.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

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

// Configurações do sistema de previsão
#define TAMANHO_HISTORICO_TEMP 30 // Armazena as últimas 30 leituras (5 minutos de histórico)
#define INTERVALO_PREVISAO_SEGUNDOS 300 // Previsão para 5 minutos no futuro
#define INTERVALO_LEITURA_SEGUNDOS 5 // Intervalo entre leituras de temperatura

// Constantes do sistema
#define DEBOUNCE_JOYSTICK_MS 300 // Tempo de debounce para o joystick (ms)
#define DEBOUNCE_BOTAO_MS 50 // Tempo de debounce para os botões (ms)
#define TIMEOUT_ATUALIZACAO_DISPLAY_MS 100 // Timeout para atualização do display (ms)

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
} EstadoSistema_t;

// Dados de previsão de temperatura
typedef struct
{
    float historico_temperatura[TAMANHO_HISTORICO_TEMP]; // Histórico de temperaturas
    float historico_tempo[TAMANHO_HISTORICO_TEMP]; // Histórico de tempos das leituras
    int indice_historico; // Índice atual no histórico
    bool historico_preenchido; // Indica se o histórico está cheio
} PrevisaoTemperatura_t;

// Variáveis globais
static EstadoSistema_t estado_sistema = {
    .temperatura_urgencia = 30, // Valor inicial do limite de urgência
    .temperatura_atual = 0.0, // Valor inicial da temperatura atual
    .temperatura_prevista = 0.0, // Valor inicial da temperatura prevista
    .tela_atual = 0 // Tela inicial (urgência)
};

static PrevisaoTemperatura_t previsao_temperatura = {0}; // Inicializa estrutura de previsão
static ssd1306_t display; // Estrutura para o display OLED

// Handles do FreeRTOS
static SemaphoreHandle_t mutex_display; // Mutex para acesso ao display
static SemaphoreHandle_t mutex_estado_sistema; // Mutex para acesso ao estado do sistema
static TaskHandle_t tarefa_temperatura_handle; // Handle da task de temperatura
static TaskHandle_t tarefa_display_handle; // Handle da task de display
static TaskHandle_t tarefa_entrada_handle; // Handle da task de entrada

// Handles das filas
static QueueHandle_t fila_dados_temperatura; // Fila para dados de temperatura
static QueueHandle_t fila_resultados_previsao; // Fila para resultados de previsão
static QueueHandle_t fila_comandos_usuario; // Fila para comandos do usuário

// ================================================================================================
// FUNÇÕES AUXILIARES - REGRESSÃO LINEAR
// ================================================================================================

// Calcula a regressão linear para previsão de temperatura
static bool calcular_regressao_linear(float *dados_x, float *dados_y, int n, float *m, float *b)
{
    if (n < 2)
    { // Verifica se há pontos suficientes para a regressão
        *m = 0.0;
        *b = (n > 0) ? dados_y[0] : 0.0;
        return false;
    }

    float soma_x = 0.0, soma_y = 0.0, soma_xy = 0.0, soma_x2 = 0.0;

    // Calcula somatórios para a regressão linear
    for (int i = 0; i < n; i++)
    {
        soma_x += dados_x[i];
        soma_y += dados_y[i];
        soma_xy += dados_x[i] * dados_y[i];
        soma_x2 += dados_x[i] * dados_x[i];
    }

    float denominador = (n * soma_x2) - (soma_x * soma_x);

    // Verifica se o denominador é válido para evitar divisão por zero
    if (fabs(denominador) < 1e-6)
    {
        *m = 0.0;
        *b = soma_y / n;
        return false;
    }

    // Calcula inclinação (m) e interceptação (b) da reta
    *m = ((n * soma_xy) - (soma_x * soma_y)) / denominador;
    *b = (soma_y - (*m * soma_x)) / n;

    return true;
}

// Calcula a previsão de temperatura com base no histórico
static float calcular_previsao_temperatura(float tempo_atual)
{
    float m, b;
    int numero_pontos = previsao_temperatura.historico_preenchido ? TAMANHO_HISTORICO_TEMP : previsao_temperatura.indice_historico;

    // Tenta calcular a regressão linear se houver pontos suficientes
    if (numero_pontos >= 2 && calcular_regressao_linear(previsao_temperatura.historico_tempo, previsao_temperatura.historico_temperatura, numero_pontos, &m, &b))
    {
        float tempo_futuro = tempo_atual + INTERVALO_PREVISAO_SEGUNDOS;
        return m * tempo_futuro + b; // Previsão para o tempo futuro
    }

    return estado_sistema.temperatura_atual; // Retorna temperatura atual se não for possível prever
}

// ================================================================================================
// TASK 1: LEITURA E PROCESSAMENTO DE TEMPERATURA
// ================================================================================================

static void tarefa_temperatura(void *pvParameters)
{
    float leitura_temperatura = 0.0; // Armazena a leitura bruta do sensor
    float temperatura_filtrada = 0.0; // Temperatura após filtro
    bool primeira_leitura = true; // Flag para primeira leitura
    TickType_t tempo_inicial_ticks = xTaskGetTickCount(); // Tempo inicial da task
    DadosTemperatura_t dados_temp; // Estrutura para enviar à fila
    ResultadosPrevisao_t resultados_previsao; // Estrutura para enviar previsão

    while (1)
    {
        leitura_temperatura = ds18b20_get_temperature(); // Lê temperatura do sensor

        // Aplica filtro exponencial para suavizar a leitura
        if (primeira_leitura)
        {
            temperatura_filtrada = leitura_temperatura;
            primeira_leitura = false;
        }
        else
        {
            temperatura_filtrada = (temperatura_filtrada * 0.8) + (leitura_temperatura * 0.2);
        }

        // Verifica se a leitura está dentro de um intervalo válido
        if (leitura_temperatura > -20.0 && leitura_temperatura < 80.0)
        {
            // Calcula o tempo atual em segundos
            float tempo_atual = (float)(xTaskGetTickCount() - tempo_inicial_ticks) * portTICK_PERIOD_MS / 1000.0;

            // Armazena a leitura no histórico
            previsao_temperatura.historico_temperatura[previsao_temperatura.indice_historico] = temperatura_filtrada;
            previsao_temperatura.historico_tempo[previsao_temperatura.indice_historico] = tempo_atual;

            // Atualiza o índice do histórico (circular)
            previsao_temperatura.indice_historico = (previsao_temperatura.indice_historico + 1) % TAMANHO_HISTORICO_TEMP;
            if (previsao_temperatura.indice_historico == 0 && !previsao_temperatura.historico_preenchido)
            {
                previsao_temperatura.historico_preenchido = true;
            }

            // Prepara dados para enviar à fila
            dados_temp.temperatura = temperatura_filtrada;
            dados_temp.marca_tempo = xTaskGetTickCount();
            xQueueSend(fila_dados_temperatura, &dados_temp, portMAX_DELAY);

            // Calcula e envia previsão
            resultados_previsao.previsao_linear = calcular_previsao_temperatura(tempo_atual);
            xQueueSend(fila_resultados_previsao, &resultados_previsao, portMAX_DELAY);

            // Atualiza o estado do sistema com a nova temperatura (opcional, dependendo do design)
            if (xSemaphoreTake(mutex_estado_sistema, portMAX_DELAY) == pdTRUE)
            {
                estado_sistema.temperatura_atual = temperatura_filtrada;
                xSemaphoreGive(mutex_estado_sistema);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(INTERVALO_LEITURA_SEGUNDOS * 1000)); // Aguarda próximo ciclo
    }
}

// ================================================================================================
// TASK 2: CONTROLE DE ENTRADA DO USUÁRIO
// ================================================================================================

static void tarefa_entrada(void *pvParameters)
{
    uint16_t valor_adc; // Valor lido do conversor ADC
    bool botao_a_pressionado = false; // Estado do botão A
    bool botao_b_pressionado = false; // Estado do botão B
    TickType_t ultima_mudanca_joystick = 0; // Última mudança do joystick
    ComandoUsuario_t comando_usuario; // Estrutura para enviar comando à fila

    while (1)
    {
        valor_adc = adc_read(); // Lê o valor do joystick
        TickType_t tick_atual = xTaskGetTickCount(); // Tempo atual

        // Ajusta a temperatura de urgência com base no joystick (tela de configuração)
        if (estado_sistema.tela_atual == 0)
        {
            if ((tick_atual - ultima_mudanca_joystick) > pdMS_TO_TICKS(DEBOUNCE_JOYSTICK_MS))
            {
                bool mudou_temp = false;

                if (valor_adc > 3000)
                { // Joystick para cima
                    comando_usuario.tipo = COMANDO_AJUSTAR_URGENCIA_SUBIR;
                    comando_usuario.valor = 1;
                    mudou_temp = true;
                }
                else if (valor_adc < 1000)
                { // Joystick para baixo
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

        // Trata o pressionamento do botão A (avançar tela)
        if (!gpio_get(PINO_BOTAO_A) && !botao_a_pressionado)
        {
            botao_a_pressionado = true;
            if (estado_sistema.tela_atual == 0)
            {
                comando_usuario.tipo = COMANDO_PROXIMA_TELA;
                comando_usuario.valor = 0;
                xQueueSend(fila_comandos_usuario, &comando_usuario, portMAX_DELAY);
            }
        }
        else if (gpio_get(PINO_BOTAO_A))
        {
            botao_a_pressionado = false;
        }

        // Trata o pressionamento do botão B (voltar à tela inicial)
        if (!gpio_get(PINO_BOTAO_B) && !botao_b_pressionado)
        {
            botao_b_pressionado = true;
            if (estado_sistema.tela_atual != 0)
            {
                comando_usuario.tipo = COMANDO_TELA_ANTERIOR;
                comando_usuario.valor = 0;
                xQueueSend(fila_comandos_usuario, &comando_usuario, portMAX_DELAY);
            }
        }
        else if (gpio_get(PINO_BOTAO_B))
        {
            botao_b_pressionado = false;
        }

        vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_BOTAO_MS)); // Aguarda próximo ciclo
    }
}

// ================================================================================================
// TASK 3: CONTROLE DO DISPLAY
// ================================================================================================

static void renderizar_tela_configuracao(void)
{
    // Exibe a tela de configuração da temperatura de urgência
    ssd1306_draw_string(&display, "Temperatura", 20, 0, false);
    ssd1306_draw_string(&display, "Urgencia", 32, 16, false);

    char texto_temp[10];
    snprintf(texto_temp, sizeof(texto_temp), "%d C", estado_sistema.temperatura_urgencia);

    // Centraliza o texto da temperatura no display
    int largura_texto = strlen(texto_temp) * 6;
    int centro_x = (128 - largura_texto) / 2;
    ssd1306_draw_string(&display, texto_temp, centro_x, 40, false);
}

static void renderizar_tela_resumo(void)
{
    char linha_str[25];
    float temp_atual_local, temp_prevista_local;
    float diferenca = 0.0;
    const char *situacao = "Desconhecida";

    // Obtém os valores de temperatura com segurança
    if (xSemaphoreTake(mutex_estado_sistema, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        temp_atual_local = estado_sistema.temperatura_atual;
        temp_prevista_local = estado_sistema.temperatura_prevista;
        diferenca = estado_sistema.temperatura_urgencia - temp_prevista_local;
        xSemaphoreGive(mutex_estado_sistema);
    }
    else
    {
        temp_atual_local = -99.9;
        temp_prevista_local = -99.9;
    }

    // Determina a situação com base na tabela
    if (xSemaphoreTake(mutex_estado_sistema, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        if (estado_sistema.temperatura_atual > estado_sistema.temperatura_urgencia)
        {
            situacao = "Grave";
        }
        else if (diferenca > 5.0)
        {
            situacao = "Normal";
        }
        else if (diferenca >= 0.0 && diferenca <= 5.0)
        {
            situacao = "Atencao";
        }
        else if (diferenca < 0.0)
        {
            situacao = "Alerta";
        }
        xSemaphoreGive(mutex_estado_sistema);
    }

    // Exibe informações na tela de resumo
    snprintf(linha_str, sizeof(linha_str), "Temp urg: %d C", estado_sistema.temperatura_urgencia);
    ssd1306_draw_string(&display, linha_str, 0, 0, false);

    snprintf(linha_str, sizeof(linha_str), "Temp atual:%.1fC", temp_atual_local);
    ssd1306_draw_string(&display, linha_str, 0, 14, false);

    snprintf(linha_str, sizeof(linha_str), "Prev. 5min:%.1fC", temp_prevista_local);
    ssd1306_draw_string(&display, linha_str, 0, 28, false);

    snprintf(linha_str, sizeof(linha_str), "Situacao:%s", situacao);
    ssd1306_draw_string(&display, linha_str, 0, 42, false);

    ssd1306_draw_string(&display, "Cor:valor", 0, 56, false);

    // Controle do LED RGB com base na situação
    if (strcmp(situacao, "Normal") == 0)
    {
        gpio_put(PINO_LED_VERDE, 1);
        gpio_put(PINO_LED_VERMELHO, 0);
    }
    else if (strcmp(situacao, "Atencao") == 0)
    {
        gpio_put(PINO_LED_VERDE, 1);
        gpio_put(PINO_LED_VERMELHO, 1);
    }
    else if (strcmp(situacao, "Alerta") == 0)
    {
        gpio_put(PINO_LED_VERDE, 0);
        gpio_put(PINO_LED_VERMELHO, 1);
    }
    else if (strcmp(situacao, "Grave") == 0)
    {
        // Pisca o LED vermelho a cada 500 ms
        static TickType_t ultimo_piscar = 0;
        TickType_t tick_atual = xTaskGetTickCount();
        if ((tick_atual - ultimo_piscar) > pdMS_TO_TICKS(500))
        {
            static bool led_ligado = false;
            led_ligado = !led_ligado;
            gpio_put(PINO_LED_VERMELHO, led_ligado);
            gpio_put(PINO_LED_VERDE, 0); // Garante que o LED verde está desligado
            ultimo_piscar = tick_atual;
        }
    }
}

static void tarefa_display(void *pvParameters)
{
    uint32_t valor_notificacao;
    DadosTemperatura_t dados_temp;
    ResultadosPrevisao_t resultados_previsao;
    ComandoUsuario_t comando_usuario;

    while (1)
    {
        // Aguarda notificação ou timeout para atualizar o display
        xTaskNotifyWait(0x00, ULONG_MAX, &valor_notificacao, pdMS_TO_TICKS(TIMEOUT_ATUALIZACAO_DISPLAY_MS));

        // Recebe dados de temperatura (se disponíveis)
        if (xQueueReceive(fila_dados_temperatura, &dados_temp, 0) == pdTRUE)
        {
            if (xSemaphoreTake(mutex_estado_sistema, portMAX_DELAY) == pdTRUE)
            {
                estado_sistema.temperatura_atual = dados_temp.temperatura;
                xSemaphoreGive(mutex_estado_sistema);
            }
        }

        // Recebe resultados de previsão (se disponíveis)
        if (xQueueReceive(fila_resultados_previsao, &resultados_previsao, 0) == pdTRUE)
        {
            if (xSemaphoreTake(mutex_estado_sistema, portMAX_DELAY) == pdTRUE)
            {
                estado_sistema.temperatura_prevista = resultados_previsao.previsao_linear;
                xSemaphoreGive(mutex_estado_sistema);
            }
        }

        // Processa comandos do usuário (se disponíveis)
        if (xQueueReceive(fila_comandos_usuario, &comando_usuario, 0) == pdTRUE)
        {
            if (xSemaphoreTake(mutex_estado_sistema, portMAX_DELAY) == pdTRUE)
            {
                switch (comando_usuario.tipo)
                {
                case COMANDO_PROXIMA_TELA:
                    estado_sistema.tela_atual = 1;
                    break;
                case COMANDO_TELA_ANTERIOR:
                    estado_sistema.tela_atual = 0;
                    break;
                case COMANDO_AJUSTAR_URGENCIA_SUBIR:
                    estado_sistema.temperatura_urgencia += comando_usuario.valor;
                    break;
                case COMANDO_AJUSTAR_URGENCIA_DESCER:
                    estado_sistema.temperatura_urgencia += comando_usuario.valor;
                    break;
                }
                xSemaphoreGive(mutex_estado_sistema);
            }
        }

        if (xSemaphoreTake(mutex_display, portMAX_DELAY) == pdTRUE)
        {
            ssd1306_fill(&display, false); // Limpa o display

            // Renderiza a tela apropriada com base no estado
            switch (estado_sistema.tela_atual)
            {
            case 0:
                renderizar_tela_configuracao();
                break;
            case 1:
                renderizar_tela_resumo();
                break;
            default:
                ssd1306_draw_string(&display, "Erro: Tela", 0, 0, false);
                ssd1306_draw_string(&display, "Invalida", 0, 16, false);
                break;
            }

            ssd1306_send_data(&display); // Envia os dados ao display
            xSemaphoreGive(mutex_display);
        }
    }
}

// ================================================================================================
// FUNÇÃO PRINCIPAL
// ================================================================================================

int main(void)
{
    // Inicialização do hardware
    stdio_init_all(); // Inicializa comunicação serial

    // Configura I2C para o display OLED
    i2c_init(i2c1, 100 * 1000);
    gpio_set_function(PINO_SDA_I2C, GPIO_FUNC_I2C);
    gpio_set_function(PINO_SCL_I2C, GPIO_FUNC_I2C);
    gpio_pull_up(PINO_SDA_I2C);
    gpio_pull_up(PINO_SCL_I2C);

    // Inicializa o display OLED
    ssd1306_init(&display, 128, 64, false, 0x3C, i2c1);
    ssd1306_config(&display);

    // Configura o conversor analógico-digital (ADC)
    adc_init();
    adc_gpio_init(PINO_ADC);
    adc_select_input(0);

    // Configura os botões de entrada
    gpio_init(PINO_BOTAO_A);
    gpio_set_dir(PINO_BOTAO_A, GPIO_IN);
    gpio_pull_up(PINO_BOTAO_A);

    gpio_init(PINO_BOTAO_B);
    gpio_set_dir(PINO_BOTAO_B, GPIO_IN);
    gpio_pull_up(PINO_BOTAO_B);

    // Configura os pinos dos LEDs como saída
    gpio_init(PINO_LED_VERDE);
    gpio_set_dir(PINO_LED_VERDE, GPIO_OUT);
    gpio_init(PINO_LED_VERMELHO);
    gpio_set_dir(PINO_LED_VERMELHO, GPIO_OUT);

    // Inicializa o sensor de temperatura
    ds18b20_init(PINO_DS18B20);

    // Inicialização das estruturas de dados
    for (int i = 0; i < TAMANHO_HISTORICO_TEMP; i++)
    {
        previsao_temperatura.historico_temperatura[i] = -999.0; // Valor inicial inválido para temperatura
        previsao_temperatura.historico_tempo[i] = -1.0; // Valor inicial inválido para tempo
    }

    previsao_temperatura.indice_historico = 0; // Inicia o índice do histórico
    previsao_temperatura.historico_preenchido = false; // Histórico começa vazio

    // Inicialização do FreeRTOS
    mutex_display = xSemaphoreCreateMutex(); // Cria mutex para o display
    mutex_estado_sistema = xSemaphoreCreateMutex(); // Cria mutex para o estado do sistema

    // Cria as filas
    fila_dados_temperatura = xQueueCreate(10, sizeof(DadosTemperatura_t)); // Fila para dados de temperatura
    fila_resultados_previsao = xQueueCreate(10, sizeof(ResultadosPrevisao_t)); // Fila para resultados de previsão
    fila_comandos_usuario = xQueueCreate(10, sizeof(ComandoUsuario_t)); // Fila para comandos do usuário

    // Cria as tasks do FreeRTOS
    xTaskCreate(tarefa_temperatura, "TempTask", configMINIMAL_STACK_SIZE + 512, NULL, 2, &tarefa_temperatura_handle);
    xTaskCreate(tarefa_entrada, "InputTask", configMINIMAL_STACK_SIZE + 128, NULL, 1, &tarefa_entrada_handle);
    xTaskCreate(tarefa_display, "DisplayTask", configMINIMAL_STACK_SIZE + 256, NULL, 1, &tarefa_display_handle);

    vTaskStartScheduler(); // Inicia o escalonador do FreeRTOS

    while (1)
    {
        sleep_ms(1000); // Loop infinito com delay
    }

    return 0;
}
