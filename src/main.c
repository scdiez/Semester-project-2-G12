#define __DELAY_BACKWARD_COMPATIBLE__ //so that delays can be set with variables

#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include <stdlib.h>
#include "i2cmaster.h"
#include "lcd.h"
#include "usart.h"
#include <time.h>
#include <avr/interrupt.h>

#define F_CPU 16000000UL //DEFINE BAUDRATE AS 9600

#define trigPin 2
#define echopin 4

//Vrx to A0
//Vry to A1
#define ADC_PIN0 0 //ADC channels we'll use
#define ADC_PIN1 1 

//functions for sensor 
void start_pulse();
void end_pulse();

//function for joystick 
uint16_t adc_read(uint8_t adc_channel);

//functions for motor movement
void move_left(int, int);
void move_right(int, int);
void move_up (int, int);
void move_down(int, int);
int change_position(void);
int zero(void);

//variables for sensor
volatile unsigned long pulse_start;
volatile unsigned long pulse_end;
volatile unsigned long pulse_duration;
volatile int distance;
int flag=0;
int score = 0;
int sensor_counter= 0;
int attempt = 0;

//variables for joystick
 int voltagey;
 int voltagex;
volatile uint8_t timerOverflow = 0;



//variables for motor movement 
int target_x,target_y, move_x, move_y;
int current_x=0;
int current_y=0;

int main(void) {
	sei(); // Enable global interrupts
    uart_init();
    io_redirect();
	//variables for joystick
	uint16_t adc_result0; 
  uint16_t adc_result1;

	ADMUX = (1<<REFS0); //Select vref = avcc
	ADCSRA = (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADEN); //set prescaler to 128 and turn on adc module
	
	DDRD |= (1 << DDD4) | (1 << DDD6); // Set dirPin1 and stepPin1 as output
	DDRD |= (1 << DDD2) | (1 << DDD5); // Set dirPin2 and stepPin2 as output

	DDRD |= (1 << DDD2); //Set trig pin to output
    	DDRD &= ~(1 << DDD4); // Set echoPin as an input
    	PORTD |= (1 << PORTD4); // Enable internal pull-up resistor for echoPin


	//SPEAKER "PRESS A BUTTON TO PLAY" 
	zero();
	
	//BUTTON 1 IS PRESSED TWICE { 
	if(score=0){
		// SPEAKER " Your final score is 0. Better luck next time" 
	} else{
		// SPEAKER " Congrats! Your final score is score points. 
	}
	//} END OF BUTTON 1 PRESSED TWICE

	// BUTTON 1 IS PRESSED {
	//SPEAKER "You've selected vision single player"
	//SPEAKER "The basketball hoop will now move. You get 3 attempts to score."
	//SPEAKER " Press Button 1 two times when you want to stop playing. Good Luck!" 
	change_position();
	while (attempt<=2){
		//for motion sensor
		while(flag == 0){
		PORTD &= ~(1 << PORTD2); // Clears the trigPin
		_delay_us(2);
		PORTD |= (1 << PORTD2);
		_delay_us(10); // Sets the trigPin on HIGH state for 10 microseconds
		PORTD &= ~(1 << PORTD2);

		while ((PIND & (1 << PIND4)) == 0) {} // Wait for the falling edge on echoPin
		TCCR1B |= (1 << CS11); // Start Timer/Counter1 and set prescaler to 8

		start_pulse(); // Record the start time of the pulse
		while (PIND & (1 << PIND4)) {} // Wait for the rising edge on echoPin
		end_pulse(); // Record the end time of the pulse
		TCCR1B = 0;  // Stop Timer/Counter1

		distance = pulse_duration * 0.034 / 2; // Calculate the distance

        //Check if an object is dtetected within the desired range
        if (distance < 30) {
			flag =1;
			score++; //Incremement the score when an object is detected
			//SPEAKER "You scored a point"
			//SPEAKER "Current score: score points"
		attempt=0; 
		change_position();
		} else if (distance>30){
		flag = 1; 
		// SPEAKER "You missed, Try again" 
		attempt ++;
		}
		}
	flag = 0;
	}
	//SPEAKER " You have used all your attempts. Press Button 1 if you want to start playing again" 
	zero();
// } END OF BUTTON ONE PRESSED ONCE IF STATEMENT

	// BUTTON 2 IS PRESSED{ 
	//SPEAKER "You've selected vision multi player"
	//SPEAKER "Use the Joystick to control the movement of the hoop and stop moving the hoop once desired location has been reached. You get 3 attempts to score."
	//SPEAKER " Press Button 1 two times when you want to stop playing. Good Luck!" 
	while(timerOverflow>=5){
		adc_result0 = adc_read(ADC_PIN0); //voltage depends on joystick stage so return voltage read
		adc_result1 = adc_read(ADC_PIN1);
		voltagex = (adc_result1/100);
		voltagey = (adc_result0/100);
		
		if(voltagex=5 && voltagey=5){
			 TCCR0A |= (1 << WGM01);
			// Set the compare value for 5 seconds 
			OCR0A = 15624;
			//Set the prescaler to 64 and start timer
			TCCR0B |= (1 << CS01) | (1 << CS00);
			// Enable the output compare A match interrupt
			while ( (TIFR0 & (1 << OCF0A) ) == 0){  // wait for the overflow event
				timerOverflow++;
			}
		}
		// reset the overflow flag
		
		TIFR0 = (1 << OCF0A);
		
		if (voltagex>=5){
			move_right(5, 400);
		}
		if (voltagex<5){
			move_left(5, 400);
		}
		if(voltagey>=5){
			move_up(5, 400);
		}
		if (voltagey<5){
			move_down(5, 400);
		}
	 // Reset the timer overflow variable
    timerOverflow = 0;
}
		
	while (attempt<=2){
		//for motion sensor
		while(flag == 0){
		PORTD &= ~(1 << PORTD2); // Clears the trigPin
		_delay_us(2);
		PORTD |= (1 << PORTD2);
		_delay_us(10); // Sets the trigPin on HIGH state for 10 microseconds
		PORTD &= ~(1 << PORTD2);

		while ((PIND & (1 << PIND4)) == 0) {} // Wait for the falling edge on echoPin
		TCCR1B |= (1 << CS11); // Start Timer/Counter1 and set prescaler to 8

		start_pulse(); // Record the start time of the pulse
		while (PIND & (1 << PIND4)) {} // Wait for the rising edge on echoPin
		end_pulse(); // Record the end time of the pulse
		TCCR1B = 0;  // Stop Timer/Counter1

		distance = pulse_duration * 0.034 / 2; // Calculate the distance

        //Check if an object is dtetected within the desired range
        if (distance < 30) {
			flag =1;
			score++; //Incremement the score when an object is detected
			//SPEAKER "You scored a point"
			//SPEAKER "Current score: score points"
		attempt=0; 
		change_position();
		} else if (distance>30){
		flag = 1; 
		// SPEAKER "You missed, Try again" 
		attempt ++;
		}
		}
	}
		flag = 0;
	}
	//SPEAKER " You have used all your attempts. Press Button 1 if you want to start playing again" 
	zero();
	// END OF BUTTON 2 PRESSED
	// BUTTON 3 IS PRESSED {
	//SPEAKER "You've selected no vision single player"
	//SPEAKER "The basketball hoop will now move and make a sound at its final position, try to put the ball inside it"
	//SPEAKER " Press Button 1 two times when you want to stop playing. Good Luck!" 
	change_position();
	// SPEAKER "beep"
	while (attempt<=2){
		//for motion sensor
		while(flag == 0){
		PORTD &= ~(1 << PORTD2); // Clears the trigPin
		_delay_us(2);
		PORTD |= (1 << PORTD2);
		_delay_us(10); // Sets the trigPin on HIGH state for 10 microseconds
		PORTD &= ~(1 << PORTD2);

		while ((PIND & (1 << PIND4)) == 0) {} // Wait for the falling edge on echoPin
		TCCR1B |= (1 << CS11); // Start Timer/Counter1 and set prescaler to 8

		start_pulse(); // Record the start time of the pulse
		while (PIND & (1 << PIND4)) {} // Wait for the rising edge on echoPin
		end_pulse(); // Record the end time of the pulse
		TCCR1B = 0;  // Stop Timer/Counter1

		distance = pulse_duration * 0.034 / 2; // Calculate the distance

        //Check if an object is dtetected within the desired range
        if (distance < 30) {
			flag =1;
			score++; //Incremement the score when an object is detected
			//SPEAKER "You scored a point"
			//SPEAKER "Current score: score points"
		attempt=0; 
		change_position();
		// SPEAKER "beep"
		} else if (distance>30){
		flag = 1; 
		// SPEAKER "You missed, Try again" 
		//SPEAKER "beep"
		attempt ++;
		}
		}
	flag = 0;
	//SPEAKER " You have used all your attempts. Press Button 3 if you want to start playing again" 
	zero();
// } END OF BUTTON THREE PRESSED ONCE IF STATEMENT

}

void start_pulse() {
    pulse_start = TCNT1; // Record the timer value at the start of the pulse
}

void end_pulse() {
    pulse_end = TCNT1; // Record the timer value at the end of the pulse
    pulse_duration = pulse_end - pulse_start; // Calculate the pulse duration
}
uint16_t adc_read(uint8_t adc_channel){
  ADMUX &= 0xf0; //clear any previously used channel keeping internal reference
  ADMUX |= adc_channel; //set the desired channel 
  ADCSRA |= (1<<ADSC); //start a conversion
  while ((ADCSRA&(1<<ADSC))); //wait for conversion to complete
  return ADC; //return result as a 16 bit unsigned int
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
int change_position(void){
	srand(time(0));
    target_x = rand() % 50;
    target_y = rand() % 50;
    
	move_x = target_x - current_x;
	move_y = target_y - current_y;

	if (move_x>0){
		move_right(move_x, 400);
	}
	if (move_x<0){
		move_left(move_x, 400);
	}
	if (move_y>0){
		move_right(move_y, 400);
	}
	if (move_y<0){
		move_left(move_y, 400);
	}
}

//Using max steps 500,500
int zero(void){
	move_x = 0 - current_x;
	move_y = 0 - current_y;

	if (move_x>0){
		move_right(move_x, 400);
		
	}
	if (move_x<0){
		move_left(move_x, 400);
	}
	if (move_y>0){
		move_right(move_y, 400);
	}
	if (move_y<0){
		move_left(move_y, 400);
	}
}



