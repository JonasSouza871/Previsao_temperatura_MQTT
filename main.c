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

//==============================================================================
// Variáveis Globais
//==============================================================================

static ssd1306_t display;
static int temperature_urgencia = 30;
static int temperature_prevista = 25;
static float temperature_atual = 0.0;
static int current_screen = 0; // 0=urgencia, 1=prevista, 2=terceira_tela
static absolute_time_t last_joystick_change = 0;
static absolute_time_t last_temp_reading = 0;

//==============================================================================
// Declarações de Funções
//==============================================================================

static void handle_user_input(void);
static void update_display(void);
static void update_temperature_sensor(void);

//==============================================================================
// Função Principal
//==============================================================================

int main(void) {
    stdio_init_all();

    i2c_init(i2c1, 100 * 1000);
    gpio_set_function(14, GPIO_FUNC_I2C);
    gpio_set_function(15, GPIO_FUNC_I2C);
    gpio_pull_up(14);
    gpio_pull_up(15);

    ssd1306_init(&display, 128, 64, false, 0x3C, i2c1);
    ssd1306_config(&display);

    adc_init();
    adc_gpio_init(26);
    adc_select_input(0);

    gpio_init(5);
    gpio_set_dir(5, GPIO_IN);
    gpio_pull_up(5);

    ds18b20_init(16); // Sensor DS18B20 no GPIO 16

    last_joystick_change = get_absolute_time();
    last_temp_reading = get_absolute_time();

    absolute_time_t next_update = get_absolute_time();
    while (true) {
        if (absolute_time_diff_us(get_absolute_time(), next_update) < 0) {
            update_temperature_sensor();
            handle_user_input();
            update_display();
            next_update = make_timeout_time_ms(100);
        }
        busy_wait_ms(1);
    }

    return 0;
}

//==============================================================================
// Funções de Controle
//==============================================================================

static void update_temperature_sensor(void) {
    absolute_time_t current_time = get_absolute_time();
    
    // Lê temperatura do sensor a cada 2 segundos
    if (absolute_time_diff_us(last_temp_reading, current_time) > 2000000) {
        temperature_atual = ds18b20_get_temperature();
        last_temp_reading = current_time;
    }
}

static void handle_user_input(void) {
    if (current_screen == 2) return; // Terceira tela não tem controles

    uint16_t adc_value = adc_read();
    absolute_time_t current_time = get_absolute_time();

    if (absolute_time_diff_us(last_joystick_change, current_time) > 300000) {
        bool changed = false;

        if (adc_value > 3000) {
            if (current_screen == 0) {
                temperature_urgencia++;
            } else if (current_screen == 1) {
                temperature_prevista++;
            }
            changed = true;
        } else if (adc_value < 1000) {
            if (current_screen == 0) {
                temperature_urgencia--;
            } else if (current_screen == 1) {
                temperature_prevista--;
            }
            changed = true;
        }

        if (changed) {
            last_joystick_change = current_time;
        }
    }

    if (!gpio_get(5)) {
        current_screen++;
        if (current_screen > 2) current_screen = 2;
        sleep_ms(200);
    }
}

static void update_display(void) {
    ssd1306_fill(&display, false);

    if (current_screen == 0) {
        // Primeira tela - Temperatura Urgência
        ssd1306_draw_string(&display, "Temperatura", 20, 0, false);
        ssd1306_draw_string(&display, "Urgencia", 32, 16, false);

        char temp_str[10];
        snprintf(temp_str, sizeof(temp_str), "%d C", temperature_urgencia);

        int text_width = strlen(temp_str) * 6;
        int center_x = (128 - text_width) / 2;

        ssd1306_draw_string(&display, temp_str, center_x, 40, false);

    } else if (current_screen == 1) {
        // Segunda tela - Temperatura Prevista
        ssd1306_draw_string(&display, "Temperatura", 20, 0, false);
        ssd1306_draw_string(&display, "Prevista", 32, 16, false);
        ssd1306_draw_string(&display, "Urgencia", 32, 32, false);

        char temp_str[10];
        snprintf(temp_str, sizeof(temp_str), "%d C", temperature_prevista);

        int text_width = strlen(temp_str) * 6;
        int center_x = (128 - text_width) / 2;

        ssd1306_draw_string(&display, temp_str, center_x, 48, false);

    } else if (current_screen == 2) {
        // Terceira tela - Temperatura Atual do Sensor
        ssd1306_draw_string(&display, "Temperatura", 20, 0, false);
        ssd1306_draw_string(&display, "Atual", 42, 20, false);

        char temp_str[15];
        snprintf(temp_str, sizeof(temp_str), "%.1f C", temperature_atual);

        int text_width = strlen(temp_str) * 6;
        int center_x = (128 - text_width) / 2;

        ssd1306_draw_string(&display, temp_str, center_x, 40, false);
    }

    ssd1306_send_data(&display);
}
