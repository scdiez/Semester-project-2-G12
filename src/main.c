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

// USART comunication definitions
#define BAUDRATE 9600
#define BAUD_PRESCALER (((F_CPU/(BAUDRATE*16UL)))-1)

//Motion sensor connections
#define trigPin 8
#define echopin 9

//Joystick connections
#define JOYSTICK_Y 0 //ADC channels we'll use A0
#define JOYSTICK_X 1 //A1
#define JOYSTICK_SW 12 //Joystick button D12 

//functions for sensor 
void start_pulse(void);
void end_pulse(void);
int detect_ball (void);


//function for joystick 
uint16_t adc_read(uint8_t);
void joystick(void);

//functions for motor movement
void move_left(int, int);
void move_right(int, int);
void move_up (int, int);
void move_down(int, int);
void change_position(void);
void zero(void);

//Usart functions
void usart_init(void);
void usart_send(unsigned char);

//Sensor variables
volatile unsigned long pulse_start;
volatile unsigned long pulse_end;
volatile unsigned long pulse_duration;
volatile int distance;
unsigned int trial_time;
int sensorcounter = 0;
int shotin =0;
int sensorflag=0;
int score = 0;
int attempt = 0;

//Joystick variables
int voltagey;
int voltagex;
uint16_t adc_resulty;
uint16_t adc_resultx;
int joystickflag;
int buttonstate = 1;

//Motor movement variables
int target_x,target_y, move_x, move_y;
int current_x=0;
int current_y=0;

int main(void) {
	sei(); // Enable global interrupts
    uart_init();
    io_redirect();
	usart_init();
	
	//Button configuration?
    DDRC = 0xF0;
	PORTC = 0x3F;

	//Joystick configuration
    DDRB = 1<<5;
    PORTB = 1<<5;
    ADMUX = (1<<REFS0); //Select vref = avcc
    ADCSRA = (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADEN); //set prescaler to 128 and turn on adc module
    DDRB &= ~(1 << DDB4); // Set button as an input
    PORTB |= (1 << PORTB4); // Enable pull-up resistor so it reads high when not pressed

    //Motor configuration
    DDRD |= (1 << DDD4) | (1 << DDD6); // Set dirPin1 and stepPin1 as output
	DDRD |= (1 << DDD2) | (1 << DDD5); // Set dirPin2 and stepPin2 as output

	//Motion sensor configuration
    DDRB |= (1 << DDB0);  //Set trigPin as output
    DDRB &= ~(1 << DDB1);// Set echoPin as an input
    PORTB |= (1 << PORTB1);// Enable internal pull-up resistor for echoPin
	
	usart_send(1); //"press any button to start the game"
	_delay_ms(2000);

	zero();
	while (1){
	
	//BUTTON 1 IS PRESSED TWICE { 
	if(score==0){
		usart_send(4); //" You lost Better luck next time" 
		_delay_ms(2000);
	} else{
		usart_send(5); //"You won, congrats your score is x"
		_delay_ms(2000);
	}
	//} END OF BUTTON 1 PRESSED TWICE

	if (!(PINB & (1 << PINB4))) {

	usart_send(16);//"You've selected no vision multi player"
	_delay_ms(2000);
	usart_send(8); // "Use the Joystick to control the movement of the hoop and stop moving the hoop once desired location has been reached. You get 3 attempts to score."
	_delay_ms(9000);
	usart_send(6);//" Press Button 1 two times when you want to stop playing. Good Luck!"
	_delay_ms(4000);
	joystick();
	usart_send(19); // "beep" 
	_delay_ms(1000);
	usart_send(44); //"you have 8 seconds to get the ball in"
	_delay_ms(3000);
	while (attempt<=2){
		shotin =detect_ball();
		if (shotin == 1){
			score++; //Incremement the score when an object is detected
			usart_send(2);//"You scored a point"
			//_delay_ms()
			usart_send(18);//"Current score: score points"
			// _delay_ms()
			attempt=0; 
			change_position();
		}
		//_delay_ms()
		if (shotin == 0){
			usart_send(3);// "Try again to shoot"
			_delay_ms(1000);
			attempt ++;
		}
	}
	attempt = 0;
	}
	usart_send(43)//" You have used all your attempts. Press Button 4 if you want to start playing again" 
	_delay_ms(6000);
	zero();
	}
		
}

void start_pulse() {
    pulse_start = TCNT1; // Record the timer value at the start of the pulse
}

void end_pulse() {
    pulse_end = TCNT1; // Record the timer value at the end of the pulse
    pulse_duration = pulse_end - pulse_start; // Calculate the pulse duration
}

int detect_ball (void){
	sensorflag = 0;
    while (sensorflag == 0) {
        // Clears the trigPin
        PORTB &= ~(1 << PORTB0);
        _delay_us(2);
        PORTB |= (1 << PORTB0);// Sets the trigPin on HIGH state for 10 microseconds
        _delay_us(10);
        PORTB &= ~(1 << PORTB0);
        while ((PINB & (1 << PINB1)) == 0) {} // Wait for the falling edge on echoPin

        // Start Timer/Counter1
        TCCR1B |= (1 << CS11); // Set prescaler to 8
        start_pulse(); // Record the start time of the pulse
        while (PINB & (1 << PINB1)) {} // Wait for the rising edge on echoPin
        end_pulse(); // Record the end time of the pulse
        TCCR1B = 0; // Stop Timer/Counter1
		trial_time = trial_time + pulse_duration;
		if (trial_time >= 32000){
			sensorcounter ++;
			trial_time = 0;
		}

        distance = pulse_duration * 0.017 / 2;// Calculate the distance
        if (distance < 9.5) {
			sensorflag = 1;
			return (2); //Number of audio for "You scored a point"
        }

        if (distance > 9.5 && sensorcounter>= 120) { //edit counter value for a longer time for shooting 
            return (3); //Audio for "Try again to shoot" 
			sensorcounter = 0;
        }
    }
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

void change_position(void){
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
void zero(void){
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

void usart_init(void){
    UBRR0H = (uint8_t)(BAUD_PRESCALER>>8);
    UBRR0L = (uint8_t)(BAUD_PRESCALER);
    UCSR0B = (1<<RXEN0)|(1<<TXEN0);
    UCSR0C = ((1<<UCSZ00)|(1<<UCSZ01));
}

void usart_send (unsigned char data){
    while(!(UCSR0A & (1<<UDRE0))); //wait for new data
    UDR0 = data;
}
