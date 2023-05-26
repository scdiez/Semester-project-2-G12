#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include "i2cmaster.h"
#include "lcd.h"
#include "usart.h"

//Vrx to A0
//Vry to A1

#define ADC_PIN0 0 //ADC channels we'll use
#define ADC_PIN1 1 

#define stepPin 5
#define dirPin 2
#define enPin 8


int voltagey;
int voltagex;
uint16_t adc_result0;
uint16_t adc_result1;
int joystickflag;
int timerOverflow = 0;

uint16_t adc_read(uint8_t adc_channel);
void joystick();

//functions for motor movement
void move_left(int, int);
void move_right(int, int);
void move_up (int, int);
void move_down(int, int);
void change_position(void);
void zero(void);
  
int main (void){

    uart_init();
    io_redirect();

    //Motor config
    DDRD |= (1 << DDD4) | (1 << DDD6); // Set dirPin1 and stepPin1 as output
	DDRD |= (1 << DDD2) | (1 << DDD5); // Set dirPin2 and stepPin2 as output

    //Joystick config
    DDRB = 1<<5;
    PORTB = 1<<5;
    ADMUX = (1<<REFS0); //Select vref = avcc
    ADCSRA = (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADEN); //set prescaler to 128 and turn on adc module

    while (1){}
    {
        joystick();
    }
    
    

    
}

void move_left(int steps, int delay){
	PORTD |= (1 << PORTD4); // Set dirPin HIGH to move in a particular direction
	PORTD |= (1 << PORTD2); // Set dirPin HIGH to move in a particular direction
        for (int x = 0; x < steps; x++) {
            PORTD |= (1 << PORTD6); // Set stepPin HIGH
			PORTD |= (1 << PORTD5); // Set stepPin HIGH
            _delay_us(delay);
            PORTD &= ~(1 << PORTD6); // Set stepPin LOW
			PORTD &= ~(1 << PORTD5); // Set stepPin LOW
            _delay_us(delay);
			}
}
void move_right(int steps, int delay){
	PORTD &= ~(1 << PORTD4); // Set dirPin LOW to change direction of rotation
	PORTD &= ~(1 << PORTD2);
        for (int x = 0; x < steps; x++) {
            PORTD |= (1 << PORTD6); // Set stepPin HIGH
			PORTD |= (1 << PORTD5); // Set stepPin HIGH
            _delay_us(delay);
            PORTD &= ~(1 << PORTD6); // Set stepPin LOW
			PORTD &= ~(1 << PORTD5); // Set stepPin LOW
            _delay_us(delay);
        }
}

void move_up(int steps, int delay){
	PORTD |= (1 << PORTD4); // Set dirPin HIGH to move in a particular direction
	PORTD &= ~(1 << PORTD2);
        for (int x = 0; x < steps; x++) {
            PORTD |= (1 << PORTD6); // Set stepPin HIGH
			PORTD |= (1 << PORTD5); // Set stepPin HIGH
            _delay_us(delay);
            PORTD &= ~(1 << PORTD6); // Set stepPin LOW
			PORTD &= ~(1 << PORTD5); // Set stepPin LOW
            _delay_us(delay);
        }
}

void move_down(int steps, int delay){
	PORTD |= (1 << PORTD2); // Set dirPin HIGH to move in a particular direction
	PORTD &= ~(1 << PORTD4);
        for (int x = 0; x < steps; x++) {
            PORTD |= (1 << PORTD6); // Set stepPin HIGH
			PORTD |= (1 << PORTD5); // Set stepPin HIGH
            _delay_us(delay);
            PORTD &= ~(1 << PORTD6); // Set stepPin LOW
			PORTD &= ~(1 << PORTD5); // Set stepPin LOW
            _delay_us(delay);
        }
}

uint16_t adc_read(uint8_t adc_channel){
    ADMUX &= 0xf0; //clear any previously used channel keeping internal reference
    ADMUX |= adc_channel; //set the desired channel 
    ADCSRA |= (1<<ADSC); //start a conversion
    while ((ADCSRA&(1<<ADSC))); //wait for conversion to complete
    return ADC; //return result as a 16 bit unsigned int
    }

void joystick(void){
	joystickflag = 1;
	while(joystickflag == 1){
		adc_result0 = adc_read(ADC_PIN0); //voltage depends on joystick stage so return voltage read
		adc_result1 = adc_read(ADC_PIN1);
		voltagex = (adc_result1/100);
		voltagey = (adc_result0/100);
		timerOverflow = 0;

		while(voltagex==5 && voltagey==5 && timerOverflow <= 5000){
				TCCR0A |= (1 << WGM01);
				// Set the compare value for 1 ms 
				OCR0A = 0xF9;
				//Set the prescaler to 64 and start timer
				TCCR0B |= (1 << CS01) | (1 << CS00);
				// Enable the output compare A match interrupt
				while ( (TIFR0 & (1 << OCF0A) ) == 0){  // wait for the overflow event
			}
			timerOverflow++;
			if(timerOverflow >= 4998){
				joystickflag=0;
			}
		}
		// reset the overflow flag
		
		TIFR0 = (1 << OCF0A);
		
		if (voltagex>=6){
			move_right(5, 400);
		}
		if (voltagex<4){
			move_left(5, 400);
		}
		if(voltagey>=6){
			move_up(5, 400);
		}
		if (voltagey<4){
			move_down(5, 400);
		}
	 // Reset the timer overflow variable
    timerOverflow = 0;
        }
    }
    