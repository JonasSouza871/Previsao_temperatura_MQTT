#include "ds18b20.h"

static uint ds18b20_pin; //Pino GPIO usado pelo sensor

//Funções internas para comunicação bit a bit
static void write_bit(int bit) {
    gpio_set_dir(ds18b20_pin, GPIO_OUT);
    gpio_put(ds18b20_pin, 0);
    sleep_us(bit ? 6 : 60); //Tempo de espera conforme o bit
    gpio_set_dir(ds18b20_pin, GPIO_IN);
    sleep_us(bit ? 64 : 10);
}

static int read_bit(void) {
    int v;
    gpio_set_dir(ds18b20_pin, GPIO_OUT);
    gpio_put(ds18b20_pin, 0);
    sleep_us(6);
    gpio_set_dir(ds18b20_pin, GPIO_IN);
    sleep_us(9);
    v = gpio_get(ds18b20_pin); //Lê o estado do pino
    sleep_us(55);
    return v;
}

static void write_byte(uint8_t b) {
    for (int i = 0; i < 8; i++)
        write_bit((b >> i) & 1); //Envia cada bit do byte
}

static uint8_t read_byte(void) {
    uint8_t b = 0;
    for (int i = 0; i < 8; i++)
        if (read_bit())
            b |= 1 << i; //Constrói o byte a partir dos bits lidos
    return b;
}

//Inicializa o sensor DS18B20
void ds18b20_init(uint pin) {
    ds18b20_pin = pin;
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    gpio_pull_up(pin); //Habilita resistor de pull-up
}

//Verifica a presença do sensor
bool ds18b20_reset(void) {
    gpio_set_dir(ds18b20_pin, GPIO_OUT);
    gpio_put(ds18b20_pin, 0);
    sleep_us(480); //Pulso de reset
    gpio_set_dir(ds18b20_pin, GPIO_IN);
    sleep_us(70);
    bool present = !gpio_get(ds18b20_pin); //Detecta resposta do sensor
    sleep_us(410);
    return present;
}

//Lê a temperatura do sensor
float ds18b20_get_temperature(void) {
    ds18b20_reset();
    write_byte(0xCC); //Ignora ROM (Skip ROM)
    write_byte(0x44); //Inicia conversão de temperatura
    sleep_ms(750); //Aguarda conversão

    ds18b20_reset();
    write_byte(0xCC);
    write_byte(0xBE); //Lê o scratchpad
    uint8_t lsb = read_byte();
    uint8_t msb = read_byte();
    int16_t raw = (msb << 8) | lsb; //Combina bytes para valor bruto
    return raw * 0.0625f; //Converte para graus Celsius
}