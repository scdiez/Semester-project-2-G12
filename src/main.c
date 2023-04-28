#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include "i2cmaster.h"
#include "lcd.h"

//Vrx to A0
//Vry to A1

#define ADC_PIN0 0 //ADC channels we'll use
#define ADC_PIN1 1 

uint16_t adc_read(uint8_t adc_channel);

int main (void){

  uart_init();
  io_redirect();

  DDRB = 1<<5;
  PORTB = 1<<5;

  int voltagey;
  int voltagex;

  uint16_t adc_result0;
  uint16_t adc_result1;
  ADMUX = (1<<REFS0); //Select vref = avcc
  ADCSRA = (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADEN); //set prescaler to 128 and turn on adc module
  while(1){
    adc_result0 = adc_read(ADC_PIN0); //voltage depends on joystick stage so return voltage read
    adc_result1 = adc_read(ADC_PIN1);
    voltagex = (adc_result1/100);
    printf("Voltage x =%d \n", voltagex);
    voltagey = (adc_result0/100);
    printf("Voltage y =%d \n", voltagey);
  }
}

uint16_t adc_read(uint8_t adc_channel){
  ADMUX &= 0xf0; //clear any previously used channel keeping internal reference
  ADMUX |= adc_channel; //set the desired channel 
  ADCSRA |= (1<<ADSC); //start a conversion
  while ((ADCSRA&(1<<ADSC))); //wait for conversion to complete
  return ADC; //return result as a 16 bit unsigned int
}