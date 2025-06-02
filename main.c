#include <math.h>
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "pico/time.h"
#include "ssd1306.h"
#include "ds18b20.h"

// FreeRTOS includes
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

//==============================================================================
// Definições de Pinos
//==============================================================================
#define OLED_SDA_PIN    14
#define OLED_SCL_PIN    15
#define ADC_JOYSTICK_PIN    26 // ADC0
#define BUTTON_JOYSTICK_PIN 5
#define DS18B20_PIN     16

//==============================================================================
// Variáveis Globais e Handles do FreeRTOS
//==============================================================================

static ssd1306_t display;

// Variáveis de configuração
static int temperature_urgencia = 30;
static int temperature_prevista = 25;
static volatile float temperature_atual = 0.0; // volatile pois é modificada por uma task e lida por outra
static int current_screen = 0; // 0=urgencia, 1=prevista, 2=terceira_tela

// Handles do FreeRTOS
static SemaphoreHandle_t xDisplayMutex;
static SemaphoreHandle_t xTempMutex; // Para proteger temperature_atual
static TaskHandle_t xDisplayTaskHandle; // Usado para notificar a task de display

//==============================================================================
// Definições de Tasks
//==============================================================================

// Task para leitura de temperatura
static void vTemperatureTask(void *pvParameters) {
    (void)pvParameters; // Evita warning de parâmetro não utilizado
    float temp_reading = 0.0;
    float temp_filtered = 0.0;
    bool first_reading = true;

    while (1) {
        temp_reading = ds18b20_get_temperature();

        // Filtro simples para evitar variações bruscas
        if (first_reading) {
            temp_filtered = temp_reading;
            first_reading = false;
        } else {
            // Aplica filtro passa-baixa simples (coeficientes ajustáveis)
            temp_filtered = (temp_filtered * 0.8f) + (temp_reading * 0.2f);
        }

        // Validação da leitura (evita valores absurdos)
        if (temp_reading > -20.0f && temp_reading < 80.0f) {
            if (xSemaphoreTake(xTempMutex, portMAX_DELAY) == pdTRUE) {
                temperature_atual = temp_filtered;
                xSemaphoreGive(xTempMutex);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(2000)); // Lê a cada 2 segundos
    }
}

// Task para controle de entrada do usuário
static void vInputTask(void *pvParameters) {
    (void)pvParameters;
    uint16_t adc_value;
    bool button_pressed = false;
    TickType_t last_joystick_change = 0;

    while (1) {
        adc_value = adc_read();
        TickType_t current_tick = xTaskGetTickCount();

        // Debounce para o joystick (300ms)
        if ((current_tick - last_joystick_change) > pdMS_TO_TICKS(300)) {
            bool changed = false;

            if (current_screen < 2) { // Só permite controle nas duas primeiras telas
                if (adc_value > 3000) { // Joystick para cima
                    if (current_screen == 0) {
                        temperature_urgencia++;
                    } else if (current_screen == 1) {
                        temperature_prevista++;
                    }
                    changed = true;
                } else if (adc_value < 1000) { // Joystick para baixo
                    if (current_screen == 0) {
                        temperature_urgencia--;
                    } else if (current_screen == 1) {
                        temperature_prevista--;
                    }
                    changed = true;
                }
                if (changed) {
                    last_joystick_change = current_tick;
                }
            }
        }

        // Botão de mudança de tela e debounce
        if (!gpio_get(BUTTON_JOYSTICK_PIN) && !button_pressed) {
            button_pressed = true;
            current_screen++;
            if (current_screen > 2) {
                current_screen = 0; // Volta para a primeira tela
            }
            xTaskNotify(xDisplayTaskHandle, 1, eSetBits); // Força atualização do display
        } else if (gpio_get(BUTTON_JOYSTICK_PIN)) {
            button_pressed = false;
        }

        vTaskDelay(pdMS_TO_TICKS(50)); // Verifica a cada 50ms
    }
}

// Task para atualização do display
static void vDisplayTask(void *pvParameters) {
    (void)pvParameters;
    uint32_t ulNotificationValue;
    char line_str[30];

    for (;;) {
        // Aguarda notificação ou timeout de 100ms para atualizar
        xTaskNotifyWait(0x00, ULONG_MAX, &ulNotificationValue, pdMS_TO_TICKS(100));

        if (xSemaphoreTake(xDisplayMutex, portMAX_DELAY) == pdTRUE) {
            ssd1306_fill(&display, false);

            if (current_screen == 0) {
                ssd1306_draw_string(&display, "Temperatura", 20, 0, false);
                ssd1306_draw_string(&display, "Urgencia", 32, 16, false);

                snprintf(line_str, sizeof(line_str), "%d C", temperature_urgencia);
                int text_width = strlen(line_str) * 6;
                int center_x = (128 - text_width) / 2;
                ssd1306_draw_string(&display, line_str, center_x, 40, false);

            } else if (current_screen == 1) {
                ssd1306_draw_string(&display, "Temperatura", 20, 0, false);
                ssd1306_draw_string(&display, "Prevista", 32, 16, false);
                ssd1306_draw_string(&display, "Urgencia", 32, 32, false); // Mantido conforme seu código original

                snprintf(line_str, sizeof(line_str), "%d C", temperature_prevista);
                int text_width = strlen(line_str) * 6;
                int center_x = (128 - text_width) / 2;
                ssd1306_draw_string(&display, line_str, center_x, 48, false);

            } else if (current_screen == 2) {
                float temp_atual_local;

                // Lê temperatura atual de forma protegida
                if (xSemaphoreTake(xTempMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    temp_atual_local = temperature_atual;
                    xSemaphoreGive(xTempMutex);
                } else {
                    temp_atual_local = 0.0f; // Valor padrão se não conseguir ler
                }

                snprintf(line_str, sizeof(line_str), "Temp urg: %d C", temperature_urgencia);
                ssd1306_draw_string(&display, line_str, 0, 0, false);

                snprintf(line_str, sizeof(line_str), "Temp prev urg:%dC", temperature_prevista);
                ssd1306_draw_string(&display, line_str, 0, 14, false);

                snprintf(line_str, sizeof(line_str), "Temp atual:%.1fC", (double)temp_atual_local); // Cast para double para snprintf com float
                ssd1306_draw_string(&display, line_str, 0, 28, false);

                ssd1306_draw_string(&display, "Cor:valor", 0, 42, false);
                ssd1306_draw_string(&display, "Situacao:valor", 0, 56, false);
            }

            ssd1306_send_data(&display);
            xSemaphoreGive(xDisplayMutex);
        }
    }
}

//==============================================================================
// Função Principal
//==============================================================================

int main(void) {
    stdio_init_all();

    // Configuração I2C para o OLED
    i2c_init(i2c1, 100 * 1000);
    gpio_set_function(OLED_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(OLED_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(OLED_SDA_PIN);
    gpio_pull_up(OLED_SCL_PIN);

    // Inicialização do display OLED
    ssd1306_init(&display, 128, 64, false, 0x3C, i2c1);
    ssd1306_config(&display);

    // Configuração do ADC para o joystick
    adc_init();
    adc_gpio_init(ADC_JOYSTICK_PIN);
    adc_select_input(0); // Assumindo ADC0 está no pino ADC_JOYSTICK_PIN

    // Configuração do pino do botão do joystick
    gpio_init(BUTTON_JOYSTICK_PIN);
    gpio_set_dir(BUTTON_JOYSTICK_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_JOYSTICK_PIN);

    // Inicialização do sensor DS18B20
    ds18b20_init(DS18B20_PIN);

    // Criação dos mutexes
    xDisplayMutex = xSemaphoreCreateMutex();
    xTempMutex = xSemaphoreCreateMutex();

    if (xDisplayMutex == NULL || xTempMutex == NULL) {
        printf("Erro: Falha ao criar mutexes. Verifique a memória heap do FreeRTOS.\n");
        while(1) {
            tight_loop_contents(); // Trava aqui em caso de erro fatal
        }
    }

    // Criação das tasks
    // Prioridades: Quanto maior o número, maior a prioridade.
    // Display e Temperatura com prioridade igual para um bom equilíbrio.
    // Input com prioridade um pouco maior para garantir responsividade rápida.
    xTaskCreate(vTemperatureTask, "TempTask", configMINIMAL_STACK_SIZE * 2, NULL, 2, NULL);
    xTaskCreate(vInputTask, "InputTask", configMINIMAL_STACK_SIZE, NULL, 3, NULL);
    xTaskCreate(vDisplayTask, "DisplayTask", configMINIMAL_STACK_SIZE * 2, NULL, 2, &xDisplayTaskHandle);

    // Inicia o scheduler do FreeRTOS
    vTaskStartScheduler();

    // Este ponto nunca deve ser alcançado se o scheduler iniciar corretamente
    printf("Erro: Scheduler FreeRTOS parou inesperadamente.\n");
    while (1) {
        tight_loop_contents();
    }

    return 0;
}