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

// Definições de pinos
#define I2C_SDA_PIN 14
#define I2C_SCL_PIN 15
#define ADC_PIN 26
#define BOTAO_PIN 5
#define DS18B20_PIN 16

// Variáveis Globais e Handles do FreeRTOS
static ssd1306_t display;
static int temperatura_urgencia = 30;
static int temperatura_prevista = 25;
static float temperatura_atual = 0.0;
static int tela_atual = 0; // 0=urgencia, 1=prevista, 2=terceira_tela

// Handles do FreeRTOS
static SemaphoreHandle_t xDisplayMutex;
static SemaphoreHandle_t xTemperaturaMutex;
static TaskHandle_t xTemperaturaTaskHandle;
static TaskHandle_t xDisplayTaskHandle;
static TaskHandle_t xEntradaTaskHandle;

// Queue para comandos de mudança de tela
static QueueHandle_t xScreenQueue;

// Task para leitura de temperatura
static void vTemperaturaTask(void *pvParameters) {
    float leitura_temperatura = 0.0;
    float temperatura_filtrada = 0.0;
    bool primeira_leitura = true;

    while (1) {
        // Lê temperatura do sensor
        leitura_temperatura = ds18b20_get_temperature();

        // Filtro simples para evitar variações bruscas
        if (primeira_leitura) {
            temperatura_filtrada = leitura_temperatura;
            primeira_leitura = false;
        } else {
            // Aplica filtro passa-baixa simples
            temperatura_filtrada = (temperatura_filtrada * 0.8) + (leitura_temperatura * 0.2);
        }

        // Validação da leitura (evita valores absurdos)
        if (leitura_temperatura > -20.0 && leitura_temperatura < 80.0) {
            // Protege a variável global com mutex
            if (xSemaphoreTake(xTemperaturaMutex, portMAX_DELAY) == pdTRUE) {
                temperatura_atual = temperatura_filtrada;
                xSemaphoreGive(xTemperaturaMutex);
            }
        }

        // Aguarda 2 segundos antes da próxima leitura
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

// Task para controle de entrada do usuário
static void vEntradaTask(void *pvParameters) {
    uint16_t valor_adc;
    bool botao_pressionado = false;
    TickType_t ultima_mudanca_joystick = 0;

    while (1) {
        // Lê o ADC (joystick)
        valor_adc = adc_read();
        TickType_t tick_atual = xTaskGetTickCount();

        // Verifica se passou tempo suficiente desde a última mudança (300ms)
        if ((tick_atual - ultima_mudanca_joystick) > pdMS_TO_TICKS(300)) {
            bool mudou = false;

            if (tela_atual < 2) { // Só permite controle nas duas primeiras telas
                if (valor_adc > 3000) {
                    if (tela_atual == 0) {
                        temperatura_urgencia++;
                    } else if (tela_atual == 1) {
                        temperatura_prevista++;
                    }
                    mudou = true;
                } else if (valor_adc < 1000) {
                    if (tela_atual == 0) {
                        temperatura_urgencia--;
                    } else if (tela_atual == 1) {
                        temperatura_prevista--;
                    }
                    mudou = true;
                }

                if (mudou) {
                    ultima_mudanca_joystick = tick_atual;
                }
            }
        }

        // Verifica botão de mudança de tela
        if (!gpio_get(BOTAO_PIN) && !botao_pressionado) {
            botao_pressionado = true;
            tela_atual++;
            if (tela_atual > 2) tela_atual = 2;

            // Força atualização do display
            xTaskNotify(xDisplayTaskHandle, 1, eSetBits);

        } else if (gpio_get(BOTAO_PIN)) {
            botao_pressionado = false;
        }

        vTaskDelay(pdMS_TO_TICKS(50)); // Verifica a cada 50ms
    }
}

// Task para atualização do display
static void vDisplayTask(void *pvParameters) {
    uint32_t ulNotificationValue;

    while (1) {
        // Aguarda notificação ou timeout de 100ms
        xTaskNotifyWait(0x00, ULONG_MAX, &ulNotificationValue, pdMS_TO_TICKS(100));

        // Protege o display com mutex
        if (xSemaphoreTake(xDisplayMutex, portMAX_DELAY) == pdTRUE) {
            ssd1306_fill(&display, false);

            if (tela_atual == 0) {
                // Primeira tela - Temperatura Urgência
                ssd1306_draw_string(&display, "Temperatura", 20, 0, false);
                ssd1306_draw_string(&display, "Urgencia", 32, 16, false);

                char temp_str[10];
                snprintf(temp_str, sizeof(temp_str), "%d C", temperatura_urgencia);

                int largura_texto = strlen(temp_str) * 6;
                int centro_x = (128 - largura_texto) / 2;

                ssd1306_draw_string(&display, temp_str, centro_x, 40, false);

            } else if (tela_atual == 1) {
                // Segunda tela - Temperatura Prevista
                ssd1306_draw_string(&display, "Temperatura", 20, 0, false);
                ssd1306_draw_string(&display, "Prevista", 32, 16, false);
                ssd1306_draw_string(&display, "Urgencia", 32, 32, false);

                char temp_str[10];
                snprintf(temp_str, sizeof(temp_str), "%d C", temperatura_prevista);

                int largura_texto = strlen(temp_str) * 6;
                int centro_x = (128 - largura_texto) / 2;

                ssd1306_draw_string(&display, temp_str, centro_x, 48, false);

            } else if (tela_atual == 2) {
                // Terceira tela - Resumo de Informações
                char linha_str[25];
                float temp_atual_local;

                // Lê temperatura atual de forma protegida
                if (xSemaphoreTake(xTemperaturaMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    temp_atual_local = temperatura_atual;
                    xSemaphoreGive(xTemperaturaMutex);
                } else {
                    temp_atual_local = 0.0; // Valor padrão se não conseguir ler
                }

                // Temperatura de urgência
                snprintf(linha_str, sizeof(linha_str), "Temp urg: %d C", temperatura_urgencia);
                ssd1306_draw_string(&display, linha_str, 0, 0, false);

                // Temperatura prevista de urgência
                snprintf(linha_str, sizeof(linha_str), "Temp prev urg:%dC", temperatura_prevista);
                ssd1306_draw_string(&display, linha_str, 0, 14, false);

                // Temperatura atual
                snprintf(linha_str, sizeof(linha_str), "Temp atual:%.1fC", temp_atual_local);
                ssd1306_draw_string(&display, linha_str, 0, 28, false);

                // Cor (placeholder)
                ssd1306_draw_string(&display, "Cor:valor", 0, 42, false);

                // Situação (placeholder)
                ssd1306_draw_string(&display, "Situacao:valor", 0, 56, false);
            }

            ssd1306_send_data(&display);
            xSemaphoreGive(xDisplayMutex);
        }
    }
}

// Função Principal
int main(void) {
    stdio_init_all();

    // Configuração I2C
    i2c_init(i2c1, 100 * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    // Inicialização do display
    ssd1306_init(&display, 128, 64, false, 0x3C, i2c1);
    ssd1306_config(&display);

    // Configuração ADC
    adc_init();
    adc_gpio_init(ADC_PIN);
    adc_select_input(0);

    // Configuração do botão
    gpio_init(BOTAO_PIN);
    gpio_set_dir(BOTAO_PIN, GPIO_IN);
    gpio_pull_up(BOTAO_PIN);

    // Inicialização do sensor DS18B20
    ds18b20_init(DS18B20_PIN);

    // Criação dos mutexes
    xDisplayMutex = xSemaphoreCreateMutex();
    xTemperaturaMutex = xSemaphoreCreateMutex();

    if (xDisplayMutex == NULL || xTemperaturaMutex == NULL) {
        printf("Erro ao criar mutexes\n");
        while(1);
    }

    // Criação das tasks
    xTaskCreate(vTemperaturaTask, "TempTask", 1024, NULL, 2, &xTemperaturaTaskHandle);
    xTaskCreate(vEntradaTask, "InputTask", 512, NULL, 1, &xEntradaTaskHandle);
    xTaskCreate(vDisplayTask, "DisplayTask", 1024, NULL, 1, &xDisplayTaskHandle);

    // Inicia o scheduler do FreeRTOS
    vTaskStartScheduler();

    // Nunca deve chegar aqui
    while (1) {
        printf("Erro: Scheduler parou\n");
    }

    return 0;
}
