#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include "i2cmaster.h"
#include "lcd.h"
#include "usart.h"

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

int main (void){

    uart_init();
    io_redirect();


    //Joystick configuration
    DDRB = 1<<5;
    PORTB = 1<<5;
    ADMUX = (1<<REFS0); //Select vref = avcc
    ADCSRA = (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADEN); //set prescaler to 128 and turn on adc module
    DDRB &= ~(1 << DDB4); // Set button as an input
    PORTB |= (1 << PORTB4); // Enable pull-up resistor so it reads high when not pressed

    joystick();

}

uint16_t adc_read(uint8_t adc_channel){
  ADMUX &= 0xf0; //clear any previously used channel keeping internal reference
  ADMUX |= adc_channel; //set the desired channel 
  ADCSRA |= (1<<ADSC); //start a conversion
  while ((ADCSRA&(1<<ADSC))); //wait for conversion to complete
  return ADC; //return result as a 16 bit unsigned int
}

void joystick (void){
    joystickflag = 1;
    while (joystickflag == 1){
        adc_resultx = adc_read(JOYSTICK_X); //voltage depends on joystick stage so return voltage read (0-1024mV)
        voltagex = (adc_resultx);
        
        adc_resulty = adc_read(JOYSTICK_Y); //voltage depends on joystick stage so return voltage read
        voltagey = (adc_resulty);

        if(voltagex >=1000){ //set threshold to start moving
            printf("movemotor right \n");
        }
        if(voltagex <=50){
            printf("movemotor left \n");
        }
        if(voltagey >=1000){
            printf("movemotor up \n");
        }
        if(voltagey <=50){
            printf("movemotor down \n");
        }

        if (!(PINB & (1 << PINB4))) { // Button is active low, so it is pressed when the pin reads low
            printf("Button pressed \n");
        }

        if ((PINB & (1 << PINB4))) { // Button is active high, so it is unpressed when the pin reads high
            printf("Button unpressed \n");
        }
         
    }
    
}


