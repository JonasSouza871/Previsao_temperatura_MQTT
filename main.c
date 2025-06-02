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
#define BOTAO_A_PIN 5 // Botão para avançar de tela 0 para 1
#define BOTAO_B_PIN 6 // Botão para resetar para tela 0
#define DS18B20_PIN 16

// Variáveis Globais e Handles do FreeRTOS
static ssd1306_t display;
static int temperatura_urgencia = 30;
static float temperatura_atual = 0.0;
static int tela_atual = 0; // 0=urgencia, 1=resumo

// Handles do FreeRTOS
static SemaphoreHandle_t xDisplayMutex;
static SemaphoreHandle_t xTemperaturaMutex;
static TaskHandle_t xTemperaturaTaskHandle;
static TaskHandle_t xDisplayTaskHandle;
static TaskHandle_t xEntradaTaskHandle;

// Task para leitura de temperatura
static void vTemperaturaTask(void *pvParameters) {
    float leitura_temperatura = 0.0;
    float temperatura_filtrada = 0.0;
    bool primeira_leitura = true;

    while (1) {
        leitura_temperatura = ds18b20_get_temperature();
        if (primeira_leitura) {
            temperatura_filtrada = leitura_temperatura;
            primeira_leitura = false;
        } else {
            temperatura_filtrada = (temperatura_filtrada * 0.8) + (leitura_temperatura * 0.2);
        }

        if (leitura_temperatura > -20.0 && leitura_temperatura < 80.0) {
            if (xSemaphoreTake(xTemperaturaMutex, portMAX_DELAY) == pdTRUE) {
                temperatura_atual = temperatura_filtrada;
                xSemaphoreGive(xTemperaturaMutex);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

// Task para controle de entrada do usuário
static void vEntradaTask(void *pvParameters) {
    uint16_t valor_adc;
    bool botao_a_pressionado = false;
    bool botao_b_pressionado = false;
    TickType_t ultima_mudanca_joystick = 0;

    while (1) {
        valor_adc = adc_read();
        TickType_t tick_atual = xTaskGetTickCount();

        // Controle do Joystick para ajustar temperatura_urgencia (somente na tela 0)
        if (tela_atual == 0) {
            if ((tick_atual - ultima_mudanca_joystick) > pdMS_TO_TICKS(300)) {
                bool mudou_temp = false;
                if (valor_adc > 3000) {
                    temperatura_urgencia++;
                    mudou_temp = true;
                } else if (valor_adc < 1000) {
                    temperatura_urgencia--;
                    mudou_temp = true;
                }

                if (mudou_temp) {
                    ultima_mudanca_joystick = tick_atual;
                    xTaskNotify(xDisplayTaskHandle, 1, eSetBits); // Notifica display para atualizar
                }
            }
        }

        // Verifica Botão A (avançar tela 0 -> 1)
        if (!gpio_get(BOTAO_A_PIN) && !botao_a_pressionado) {
            botao_a_pressionado = true;
            if (tela_atual == 0) { // Só avança se estiver na tela 0
                tela_atual = 1;
                xTaskNotify(xDisplayTaskHandle, 1, eSetBits);
            }
        } else if (gpio_get(BOTAO_A_PIN)) {
            botao_a_pressionado = false;
        }

        // Verifica Botão B (resetar para tela 0)
        if (!gpio_get(BOTAO_B_PIN) && !botao_b_pressionado) {
            botao_b_pressionado = true;
            if (tela_atual != 0) { // Só reseta e notifica se não estiver já na tela 0
                tela_atual = 0;
                xTaskNotify(xDisplayTaskHandle, 1, eSetBits);
            }
        } else if (gpio_get(BOTAO_B_PIN)) {
            botao_b_pressionado = false;
        }

        vTaskDelay(pdMS_TO_TICKS(50)); // Verifica a cada 50ms
    }
}

// Task para atualização do display
static void vDisplayTask(void *pvParameters) {
    uint32_t ulNotificationValue;

    while (1) {
        xTaskNotifyWait(0x00, ULONG_MAX, &ulNotificationValue, pdMS_TO_TICKS(100));

        if (xSemaphoreTake(xDisplayMutex, portMAX_DELAY) == pdTRUE) {
            ssd1306_fill(&display, false);

            if (tela_atual == 0) {
                ssd1306_draw_string(&display, "Temperatura", 20, 0, false);
                ssd1306_draw_string(&display, "Urgencia", 32, 16, false);
                char temp_str[10];
                snprintf(temp_str, sizeof(temp_str), "%d C", temperatura_urgencia);
                int largura_texto = strlen(temp_str) * 6;
                int centro_x = (128 - largura_texto) / 2;
                ssd1306_draw_string(&display, temp_str, centro_x, 40, false);
            } else if (tela_atual == 1) {
                char linha_str[25];
                float temp_atual_local;
                if (xSemaphoreTake(xTemperaturaMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    temp_atual_local = temperatura_atual;
                    xSemaphoreGive(xTemperaturaMutex);
                } else {
                    temp_atual_local = -99.9;
                }
                snprintf(linha_str, sizeof(linha_str), "Temp urg: %d C", temperatura_urgencia);
                ssd1306_draw_string(&display, linha_str, 0, 0, false);
                snprintf(linha_str, sizeof(linha_str), "Temp atual:%.1fC", temp_atual_local);
                ssd1306_draw_string(&display, linha_str, 0, 14, false);
                ssd1306_draw_string(&display, "Cor:valor", 0, 28, false);
                ssd1306_draw_string(&display, "Situacao:valor", 0, 42, false);
            }
            ssd1306_send_data(&display);
            xSemaphoreGive(xDisplayMutex);
        }
    }
}

// Função Principal
int main(void) {
    stdio_init_all();

    i2c_init(i2c1, 100 * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    ssd1306_init(&display, 128, 64, false, 0x3C, i2c1);
    ssd1306_config(&display);

    adc_init();
    adc_gpio_init(ADC_PIN);
    adc_select_input(0);

    // Configuração do botão A
    gpio_init(BOTAO_A_PIN);
    gpio_set_dir(BOTAO_A_PIN, GPIO_IN);
    gpio_pull_up(BOTAO_A_PIN);

    // Configuração do botão B
    gpio_init(BOTAO_B_PIN);
    gpio_set_dir(BOTAO_B_PIN, GPIO_IN);
    gpio_pull_up(BOTAO_B_PIN);

    ds18b20_init(DS18B20_PIN);

    xDisplayMutex = xSemaphoreCreateMutex();
    xTemperaturaMutex = xSemaphoreCreateMutex();

    if (xDisplayMutex == NULL || xTemperaturaMutex == NULL) {
        printf("Erro ao criar mutexes\n");
        while(1);
    }

    xTaskCreate(vTemperaturaTask, "TempTask", configMINIMAL_STACK_SIZE + 256, NULL, 2, &xTemperaturaTaskHandle);
    xTaskCreate(vEntradaTask, "InputTask", configMINIMAL_STACK_SIZE + 128, NULL, 1, &xEntradaTaskHandle);
    xTaskCreate(vDisplayTask, "DisplayTask", configMINIMAL_STACK_SIZE + 256, NULL, 1, &xDisplayTaskHandle);

    vTaskStartScheduler();

    while (1) {
        printf("Erro: Scheduler parou\n");
    }
    return 0;
}