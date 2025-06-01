#ifndef DS18B20_H
#define DS18B20_H

#include <stdbool.h>
#include <stdint.h>
#include "pico/stdlib.h"

//Inicializa o sensor DS18B20 no pino especificado
void ds18b20_init(uint pin); //Configura o barramento 1-Wire
//Verifica a presença do sensor
bool ds18b20_reset(void); //Retorna true se o sensor responder
//Lê a temperatura do sensor
float ds18b20_get_temperature(void); //Retorna temperatura em °C (resolução de 12 bits)

#endif /* DS18B20_H */