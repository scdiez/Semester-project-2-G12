#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>

#include "usart.h"

//Voltage reader in A7
#define VOLTAGE_REGULATOR 7

float read_voltage(void);

//Vrx to A0
//Vry to A1

#define JOYSTICK_Y 0 //ADC channels we'll use A0
#define JOYSTICK_X 1 //A1
#define JOYSTICK_SW 12 //Joystick button D12

//Joystick functions
void joystick ();
uint16_t adc_read(uint8_t);

//Joystick variables
int voltagey;
int voltagex;
uint16_t adc_resulty;
uint16_t adc_resultx;
int joystickflag;
int buttonstate = 1;


int main(void) {

  uart_init();
  io_redirect();

  ADMUX = (ADMUX & 0xF8) | 0x07; // Select ADC7 (A7) as the input
  // High reference is AVCC
  // 10-bit result
  ADCSRB = ADCSRB & (0xF8); // 11111000
  ADCSRA = ADCSRA | 0xE7; // 11100111
  // Prescaler divisor set to 128

  //Joystick configuration
  DDRB = 1<<5;
  PORTB = 1<<5;
  ADMUX = (1<<REFS0); //Select vref = avcc
  ADCSRA = (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADEN); //set prescaler to 128 and turn on adc module
  DDRB &= ~(1 << DDB4); // Set button as an input
  PORTB |= (1 << PORTB4); // Enable pull-up resistor so it reads high when not pressed

  while (1) {

    float voltage_read = read_voltage();
    adc_resultx = adc_read(JOYSTICK_X);
    voltagex = (adc_resultx);
        
    adc_resulty = adc_read(JOYSTICK_Y); //voltage depends on joystick stage so return voltage read
    voltagey = (adc_resulty);
    printf("Voltage x: %d\n", voltagex);
    printf("Voltage y: %d\n", voltagey);
    printf("Voltage read: %f\n", voltage_read);

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

uint16_t adc_read(uint8_t adc_channel){
  ADMUX &= 0xf0; //clear any previously used channel keeping internal reference
  ADMUX |= adc_channel; //set the desired channel 
  ADCSRA |= (1<<ADSC); //start a conversion
  while ((ADCSRA&(1<<ADSC))); //wait for conversion to complete
  return ADC; //return result as a 16 bit unsigned int
}