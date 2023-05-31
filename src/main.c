#define __DELAY_BACKWARD_COMPATIBLE__ //so that delays can be set with variables


#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include "i2cmaster.h"
#include "lcd.h"
//#include "usart.h"
#include <avr/interrupt.h>
#include <time.h>

//Vrx to A0
//Vry to A1

#define F_CPU 16000000UL //DEFINE BAUDRATE AS 9600
#define BAUDRATE 9600
#define BAUD_PRESCALER (((F_CPU/(BAUDRATE*16UL)))-1)

#define trigPin 8
#define echopin 9



#define JOYSTICK_Y 0 //ADC channels we'll use A0
#define JOYSTICK_X 1 //A1
#define JOYSTICK_SW 12 //Joystick button D12


//sensor variables
volatile unsigned long pulse_start;
volatile unsigned long pulse_end;
volatile unsigned long pulse_duration;
volatile int distance;
unsigned int trial_time = 0;
int counter = 0;
int sensorflag = 0;
int shotin=0;
int attempt = 0;
int score = 0;

//sensor functions
void start_pulse(void);
void end_pulse(void);
int detect_ball (void);


//Joystick functions
void joystick ();
uint16_t adc_read(uint8_t);

//functions for motor movement
void move_left(int, int);
void move_right(int, int);
void move_up (int, int);
void move_down(int, int);
void zero(void);

//Joystick variables
int voltagey;
int voltagex;
uint16_t adc_resulty;
uint16_t adc_resultx;
int joystickflag;
int buttonstate = 1;

//speaker functions
void usart_init(void);
void usart_send (unsigned char data);

//variables for motor movement 
int target_x,target_y, move_x, move_y;
int current_x=300;
int current_y=300;

int main (void){
  //usart_init();
  uart_init();
  io_redirect();
  sei(); // Enable global interrupts

  //Joystick and sensor  configuration
  DDRB = 0b00100001;
  PORTB = 0b00110010;

  //Joystick configuration
  ADMUX = (1<<REFS0); //Select vref = avcc
  ADCSRA = (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADEN); //set prescaler to 128 and turn on adc module

  //motor configuration
  DDRD |= (1 << DDD4) | (1 << DDD6); // Set dirPin1 and stepPin1 as output
    DDRD |= (1 << DDD2) | (1 << DDD5); // Set dirPin2 and stepPin2 as output

    //Button configuration
    DDRC = 0xF0;
    PORTC = 0x3F;

  //PRESS ANY BUTTON TO START THE GAME

  while (1){
    if (PINC == 0b00110111){ //First button pressed
      printf("You've selected vision single player \n");
      printf("The basketball hoop will now move. You get 3 attempts to score. \n");
      //printf("Press Button 1 if you want to stop playing. Good Luck! \n");
      change_position();
      printf("You have 8 seconds to get the ball in \n");
      while (attempt<=2){
		shotin = detect_ball();
		if (shotin == 1){
			score++; //Incremement the score when an object is detected
            _delay_ms(2000);
			printf("You scored a point \n");
			printf("Current score: ");
			printf("%d \n",score);
			attempt=0; 
            change_position();
		}
		//_delay_ms()
		if (shotin == 0){
            _delay_ms(5000);
			printf("Try again to shoot \n");
			attempt ++;
		}
	}
    attempt = 0;
    printf("You have used all your attempts. Press Button 1 if you want to start playing again");
    }

    if (PINC == 0b00101111){
      printf("Second button pressed");
      zero();
    }

    if (PINC == 0b00011111){
      printf("Third button pressed");
      joystick();
    }
    if (PINC == 0b00111011){
      printf("Fourth button pressed");
      zero();
    }
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
            move_right(600,200);
            printf("moveright \n");
            printf("%d", current_x);
        }
        if(voltagex <=50){
            move_left(600,200);
            printf("moveleft \n");
            printf("%d", current_x);
        }
        if(voltagey >=1000){
            move_up(600,200);
            printf("moveup \n");
            printf("%d", current_y);
        }
        if(voltagey <=50){
            move_down(600,200);
            printf("movedown \n");
            printf("%d", current_y);
        }

        if (!(PINB & (1 << PINB4))) { // Button is active low, so it is pressed when the pin reads low
            printf("Button pressed \n");
            joystickflag = 0;
        }
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
    current_x = current_x+steps;
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
    current_x = current_x-steps;
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
    current_y = current_y-steps;
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
    current_y = current_y+steps;
}

uint16_t adc_read(uint8_t adc_channel){
  ADMUX &= 0xf0; //clear any previously used channel keeping internal reference
  ADMUX |= adc_channel; //set the desired channel 
  ADCSRA |= (1<<ADSC); //start a conversion
  while ((ADCSRA&(1<<ADSC))); //wait for conversion to complete
  return ADC; //return result as a 16 bit unsigned int
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
			counter ++;
			trial_time = 0;
		}

        distance = pulse_duration * 0.017 / 2;// Calculate the distance
        if (distance < 9.5) {
			sensorflag = 1;
			return (1); //Number of audio for "You scored a point"
        }

        if (distance > 9.5 && counter>= 120) { //edit counter value for a longer time for shooting 
            return (0); //Audio for "Try again to shoot" 
			counter = 0;
        }
    }
}

void change_position(void){
	srand(time(0));
    target_x = rand() % 16200;
    target_y = rand() % 15900;
    
	move_x = target_x - current_x;
	move_y = target_y - current_y;

	if (move_x>0){
		move_right(move_x, 400);
	}
	if (move_x<0){
		move_left(move_x, 400);
	}
	if (move_y>0){
		move_up(move_y, 400);
	}
	if (move_y<0){
		move_down(move_y, 400);
	}
}

void zero(void){
	move_x = current_x-300;
	move_y = current_y -300;

	move_left(move_x, 300);
    move_down(move_y, 300);
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