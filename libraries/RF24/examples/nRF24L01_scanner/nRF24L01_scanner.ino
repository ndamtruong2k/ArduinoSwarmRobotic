#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"

RF24 radio(9,10);

const uint8_t num_channels = 128;
uint8_t values[num_channels];

void setup(void){
  Serial.begin(57600);
  printf_begin();
  printf("\n\rRF24/examples/scanner/\n\r");

  radio.begin();
  radio.setAutoAck(false);

  // Get into standby mode
  radio.startListening();
  radio.stopListening();

  int i = 0;
  while ( i < num_channels ){
    printf("%x",i>>4);
    ++i;
  }
  printf("\n\r");
  i = 0;
  while ( i < num_channels ){
    printf("%x",i&0xf);
    ++i;
  }
  printf("\n\r");
}

const int num_reps = 100;

void loop(void)
{
  // Clear measurement values
  memset(values,0,sizeof(values));

  // Scan all channels num_reps times
  int rep_counter = num_reps;
  while (rep_counter--){
    int i = num_channels;
    while (i--){
      radio.setChannel(i);
      radio.startListening();
      delayMicroseconds(225);
      if ( radio.testCarrier() ){
        ++values[i];
      }
      radio.stopListening();
    }
  }

  int i = 0;
  while ( i < num_channels ){
    printf("%x",min(0xf,values[i]&0xf));
    ++i;
  }
  printf("\n\r");
}

