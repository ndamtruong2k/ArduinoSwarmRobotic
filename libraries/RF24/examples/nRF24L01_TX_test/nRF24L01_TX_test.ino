#include <SPI.h>
#include "RF24.h"

const uint64_t pipe = 0xE8E8F0F0E1LL; // адрес канала передачи
RF24 radio(9,10);
byte msg[3];

void setup(){ 
  //============================================================Модуль NRF24
  radio.begin();                      // Включение модуля
  radio.setAutoAck(1);                // Установка режима подтверждения приема;
  radio.setRetries(1,1);              // Установка интервала и количества попыток
  radio.setDataRate(RF24_250KBPS);    // Устанавливаем скорость
  radio.setPALevel(RF24_PA_MAX);      // Установка максимальной мощности;
  radio.setChannel(10);               // Устанавливаем канал
  radio.openWritingPipe(pipe);        // Открываем канал передачи
}

void loop(){ 
  msg[0] = 127;
  radio.write(&msg, sizeof(msg));
}

