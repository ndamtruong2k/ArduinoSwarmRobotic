#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

RF24 radio(9,10);
const uint64_t pipe = 0xE8E8F0F0E1LL;

void setup(void){
  Serial.begin(57600);
  printf_begin();
  radio.begin();
  radio.setChannel(0x40);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  //radio.setDataRate(RF24_2MBPS);
 
  radio.enableAckPayload();
  radio.openReadingPipe(1,pipe);
  radio.startListening();
  radio.setAutoAck(1);
 
  radio.printDetails();

  printf("\n\r");
}


void loop(void){
}

int serial_putc( char c, FILE * ) {
  Serial.write( c );
  return c;
}

void printf_begin(void) {
  fdevopen( &serial_putc, 0 );
}

