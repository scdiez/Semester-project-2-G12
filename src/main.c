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

//functions for motor movement
void move_left(int, int);
void move_right(int, int);
void move_up (int, int);
void move_down(int, int);
void change_position(void);
void zero(void);

//Joystick variables
int voltagey;
int voltagex;
uint16_t adc_resulty;
uint16_t adc_resultx;
int joystickflag;
int buttonstate = 1;

//variables for motor movement 
int target_x,target_y, move_x, move_y;
int current_x=0;
int current_y=0;

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

    //motor configuration
    DDRD |= (1 << DDD4) | (1 << DDD6); // Set dirPin1 and stepPin1 as output
	DDRD |= (1 << DDD2) | (1 << DDD5); // Set dirPin2 and stepPin2 as output

    joystick();


}

void joystick (void){
    joystickflag = 1;
    while (joystickflag == 1){
        adc_resultx = adc_read(JOYSTICK_X); //voltage depends on joystick stage so return voltage read (0-1024mV)
        voltagex = (adc_resultx);
        
        adc_resulty = adc_read(JOYSTICK_Y); //voltage depends on joystick stage so return voltage read
        voltagey = (adc_resulty);

        if(voltagex >=1000){ //set threshold to start moving
            move_right(100,500);
            printf("moveright \n");
        }
        if(voltagex <=50){
            move_left(100,500);
            printf("moveleft \n");
        }
        if(voltagey >=1000){
            move_up(100,500);
            printf("moveup \n");
        }
        if(voltagey <=50){
            move_down(100,500);
            printf("movedown \n");
        }

        if (!(PINB & (1 << PINB4))) { // Button is active low, so it is pressed when the pin reads low
            printf("Button pressed \n");
            joystickflag = 0;
        }
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

