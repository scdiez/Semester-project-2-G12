#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>

#include "usart.h"

float read_voltage(void);

int main(void) {

  uart_init();
  io_redirect();

  ADMUX = (ADMUX & 0xF8) | 0x07; // Select ADC7 (A7) as the input
  // High reference is AVCC
  // 10-bit result
  ADCSRB = ADCSRB & (0xF8); // 11111000
  ADCSRA = ADCSRA | 0xE7; // 11100111
  // Prescaler divisor set to 128

  while (1) {

    float voltage_read = read_voltage();
    printf("Voltage: %f\n", voltage_read);
    _delay_ms(1000);
  }

}

float read_voltage(void) {
  float voltage;
  unsigned int adclow = 0;
  adclow = ADCL;
  adclow = (adclow + ((ADCH & 0x03) << 8));
  voltage = (float) 3*adclow*5/1024;
  return voltage;
}
