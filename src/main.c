#define __DELAY_BACKWARD_COMPATIBLE__ //so that delays can be set with variables


#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include "i2cmaster.h"
#include "lcd.h"
#include "usart.h"
#include <avr/interrupt.h>
#include <time.h>


#define F_CPU 16000000UL //DEFINE BAUDRATE AS 9600
#define BAUDRATE 9600
#define BAUD_PRESCALER (((F_CPU/(BAUDRATE*16UL)))-1)


//Sensor connections
#define trigPin 8
#define echopin 9


//Joystick connections
#define JOYSTICK_Y 0 //A0 for y movement
#define JOYSTICK_X 1 //A1 for x movement
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
int result = 0;


//sensor functions
void start_pulse(void);
void end_pulse(void);
int detect_ball (void);


//Joystick variables
int voltagey;
int voltagex;
uint16_t adc_resulty;
uint16_t adc_resultx;
int joystickflag;
int buttonstate = 1;


//Joystick functions
void joystick ();
uint16_t adc_read(uint8_t);


//Motor movement variables
int target_x,target_y, move_x, move_y;
int current_x=300;
int current_y=300;


//Motor movement functions
void move_left(int, int);
void move_right(int, int);
void move_up (int, int);
void move_down(int, int);
void zero(void);
void change_position(void);


//Speaker functions, usart communication
void usart_init(void);
void usart_send (unsigned char data);


int main (void){
    srand(time(NULL)); //initializing for random numbers
    usart_init(); //usart communication
    //uart_init();
    //io_redirect();
    sei(); // Enable global interrupts

    //Joystick and sensor  configuration
    DDRB = 0b00000001; //trig pin (B0) output, echo pin input, joystick button (B4) input
    PORTB = 0b00010010; //pullup on joystick button and echo pin


    //Joystick configuration
    ADMUX = (1<<REFS0); //Select vref = avcc
    ADCSRA = (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADEN); //set prescaler to 128 and turn on adc module

    //Motor configuration
    DDRD |= (1 << DDD4) | (1 << DDD6); // Set dirPin1 and stepPin1 as output
    DDRD |= (1 << DDD2) | (1 << DDD5); // Set dirPin2 and stepPin2 as output

    //Button configuration
    DDRC = 0xF0;
    PORTC = 0x3F;

	usart_send(1); //press any button to start the game"
	_delay_ms(2000);
	usart_send(9); //press button 1 for no vision single player mode
	_delay_ms(3000);
	usart_send(10); //press button 2 for no vision multiplayer mode
	_delay_ms(3000);
	usart_send(11); //press button 3 for vision single-player mode
	_delay_ms(3000);
	usart_send(12); //press button 4 for vision multi-player mode
	_delay_ms(3000);
 
	while (1){
		if (PINC == 0b00110111){ //First button pressed
		usart_send(13); //you've selected no vision single player mode
		_delay_ms(3000);
		usart_send(54); //press button 1 when you want to stop playing 
		_delay_ms(---);
		usart_send(7); //The basketball hoop will now move. You get 3 attempts to score.
		_delay_ms(3000);

		change_position();

		usart_send(44); //You have 8 seconds to get the ball in
		_delay_ms(---);

		usart_send(58); //beep
		_delay_ms(---);
		
		while (attempt<=2){ //up to three failed attempts
				detect_ball(); 
				if(PINC == 0b00110111){ //first button pressed
					attempt = 3;
				}
				if (result == 1){ //ball went in
					score++; //Incremement the score when an object is detected
					usart_send(2); //You scored a point 
					_delay_ms(1000);
					usart_send(18); //Your score is 
					_delay_ms(1000);
					usart_send(score+20); //number of points
					_delay_ms(1000);
				
					change_position(); //the user will play again
					usart_send(58); //beep
					_delay_ms(---);
					attempt=0; //attempts are restarted
					result = 0;
				}
				if (result == 2){ //ball didn't go in
					_delay_ms(5000);
					usart_send (45); //shoot again
					_delay_ms(---);
					usart_send(58); //beep
					_delay_ms(---);

					attempt ++;
					result = 0;
				}
			}
			attempt = 0;
			usart_send(46); //You have used all your attempts.
			_delay_ms(---);
			usart_send(18); //Your score is 
			_delay_ms(1000);
			usart_send(score+20); //number of points
			_delay_ms(1000);
			usart_send(48); //Press button 1 if you want to start playing again, or another button for another mode
			_delay_ms(---);
			zero();
		}

		if (PINC == 0b00101111){ //second button pressed
		usart_send(13); //you've selected no vision multi player mode
		_delay_ms(3000);
		usart_send(55); //press button 2 when you want to stop playing 
		_delay_ms(---);
		usart_send(47); //Move the basket with the joystick. Press the button to fix the position. You get 3 attempts to score.
		_delay_ms(---);

		joystick();

		usart_send(58); //beep
		_delay_ms(---);
		
		usart_send(44); //You have 8 seconds to get the ball in
		_delay_ms(---);

		while (attempt<=2){
			detect_ball();
			if(PINC == 0b00101111){
				attempt = 3;
			}
			if (result == 1){
				score++; //Increment the score when an object is detected
				usart_send(2); //You scored a point 
				_delay_ms(1000);
				usart_send(18); //Your score is 
				_delay_ms(1000);
				usart_send(score+20); //Number of points
				_delay_ms(1000);
				usart_send(53); //Play again
				_delay_ms(---);

				joystick();

				usart_send(58); //beep
				_delay_ms(---);
				result = 0;
			}
			if (result == 2){
				usart_send (45); //shoot again
				_delay_ms(---);
				usart_send(58); //beep
				_delay_ms(---);
				attempt ++;
				result = 0;
			}
			}
			attempt = 0;
			usart_send(46); //You have used all your attempts.
			_delay_ms(---);
			usart_send(18); //Your score is 
			_delay_ms(1000);
			usart_send(score+20); //number of points
			_delay_ms(1000);
			usart_send(49); //Press button 2 if you want to start playing again, or another button for another mode
			_delay_ms(---);
			zero();
		}

		if (PINC == 0b00011111){ //Third button pressed
		usart_send(13); //you've selected vision single player mode
		_delay_ms(3000);
		usart_send(56); //press button 3 when you want to stop playing 
		_delay_ms(---);
		usart_send(7); //The basketball hoop will now move. You get 3 attempts to score.
		_delay_ms(3000);

		change_position();
		
		usart_send(44); //You have 8 seconds to get the ball in
		_delay_ms(---);
		
		while (attempt<=2){
			detect_ball();
			if(PINC == 0b00011111){ //third button pressed
				attempt = 3;
			}
			
			if (result == 1){
				score++; //Incremement the score when an object is detected
				usart_send(2); //You scored a point 
				_delay_ms(1000);
				usart_send(18); //Your score is 
				_delay_ms(1000);
				usart_send(score+20); //number of points
				_delay_ms(1000);
			
				change_position();
				
				attempt=0; 
				result = 0;
			}
			if (result == 2){
				_delay_ms(5000);
				usart_send (45); //shoot again
				_delay_ms(---);
				attempt ++;
				result = 0;
			}
			}
			attempt = 0;
			usart_send(46); //You have used all your attempts.
			_delay_ms(---);
			usart_send(18); //Your score is 
			_delay_ms(1000);
			usart_send(score+20); //number of points
			_delay_ms(1000);
			usart_send(50); //Press button 3 if you want to start playing again, or another button for another mode
			_delay_ms(---);
			zero();
		}

		if (PINC == 0b00111011); //fourth button pressed
		usart_send(13); //you've selected vision multi player mode
		_delay_ms(3000);
		usart_send(57); //press button 4 when you want to stop playing 
		_delay_ms(---);
		usart_send(47); //Move the basket with the joystick. Press the button to fix the position. You get 3 attempts to score.
		_delay_ms(---);
		joystick();
		
		usart_send(44); //You have 8 seconds to get the ball in
		_delay_ms(---);
		while (attempt<=2){
			detect_ball();
			if(PINC == 0b00111011){
				attempt = 3;
			}
			
			if (result == 1){
				score++; //Incremement the score when an object is detected
				usart_send(2); //You scored a point 
				_delay_ms(1000);
				usart_send(18); //Your score is 
				_delay_ms(1000);
				usart_send(score+20); //Number of points
				_delay_ms(1000);
				usart_send(53); //Play again
				_delay_ms(---);
				joystick();
				result = 0;
			}
			if (result == 2){
				_delay_ms(5000);
				usart_send (45); //shoot again
				_delay_ms(---);
				attempt ++;
				result = 0;
			}
			}
			attempt = 0;
			usart_send(46); //You have used all your attempts.
			_delay_ms(---);
			usart_send(18); //Your score is 
			_delay_ms(1000);
			usart_send(score+20); //number of points
			_delay_ms(1000);
			usart_send(51); //Press button 4 if you want to start playing again, or another button for another mode.
			_delay_ms(---);
			zero();
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
        }
        if(voltagex <=50){
            move_left(600,200);
        }
        if(voltagey >=1000){
            move_up(600,200);
        }
        if(voltagey <=50){
            move_down(600,200);
        }

        if (!(PINB & (1 << PINB4))) { // Button is active low, so it is pressed when the pin reads low
            printf("Button pressed \n");
            joystickflag = 0;
        }
    }
    
}


void move_left(int stepsleft, int delay){
    PORTD |= (1 << PORTD4); // Set dirPin HIGH to move in a particular direction
	PORTD |= (1 << PORTD2); // Set dirPin HIGH to move in a particular direction
        for (int x = 0; x < stepsleft; x++) {
            PORTD |= (1 << PORTD6); // Set stepPin HIGH
			PORTD |= (1 << PORTD5); // Set stepPin HIGH
            _delay_us(delay);
            PORTD &= ~(1 << PORTD6); // Set stepPin LOW
			PORTD &= ~(1 << PORTD5); // Set stepPin LOW
            _delay_us(delay);
        }
    current_x = current_x-stepsleft;
}

void move_right(int stepsright, int delay){
	PORTD &= ~(1 << PORTD4); // Set dirPin LOW to change direction of rotation
	PORTD &= ~(1 << PORTD2);
        for (int x = 0; x < stepsright; x++) {
            PORTD |= (1 << PORTD6); // Set stepPin HIGH
			PORTD |= (1 << PORTD5); // Set stepPin HIGH
            _delay_us(delay);
            PORTD &= ~(1 << PORTD6); // Set stepPin LOW
			PORTD &= ~(1 << PORTD5); // Set stepPin LOW
            _delay_us(delay);
		}
    current_x = current_x+stepsright;
}

void move_down(int stepsdown, int delay){
	PORTD |= (1 << PORTD2); // Set dirPin HIGH to move in a particular direction
	PORTD &= ~(1 << PORTD4);
        for (int x = 0; x < stepsdown; x++) {
            PORTD |= (1 << PORTD6); // Set stepPin HIGH
			PORTD |= (1 << PORTD5); // Set stepPin HIGH
            _delay_us(delay);
            PORTD &= ~(1 << PORTD6); // Set stepPin LOW
			PORTD &= ~(1 << PORTD5); // Set stepPin LOW
            _delay_us(delay);
        }
    current_y = current_y-stepsdown;
}

void move_up(int stepsup, int delay){
    PORTD |= (1 << PORTD4); // Set dirPin HIGH to move in a particular direction
	PORTD &= ~(1 << PORTD2);
        for (int x = 0; x < stepsup; x++) {
            PORTD |= (1 << PORTD6); // Set stepPin HIGH
			PORTD |= (1 << PORTD5); // Set stepPin HIGH
            _delay_us(delay);
            PORTD &= ~(1 << PORTD6); // Set stepPin LOW
			PORTD &= ~(1 << PORTD5); // Set stepPin LOW
            _delay_us(delay);
        }
    current_y = current_y+stepsup;
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
    while (sensorflag == 0) { //loop runs while two outcomes aren't met
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
			counter ++; //increase range of timer
			trial_time = 0;
		}

        distance = pulse_duration * 0.017 / 2;// Calculate the distance
        if (distance < 9.5) { //ball went in
			sensorflag = 1; 
			result = 1;
        }

        if (distance > 9.5 && counter>= 120) { //edit counter max value for a longer time for shooting 
            result = 2; //time has run out anad ball hasn't gone in 
			counter = 0;
			sensorflag = 1;
        }
    }
	return (result);
}

void change_position(void){
    target_y = rand() % 15900+300;
    target_x = rand() % 16200+300;
    move_y = target_y - current_y;
    move_x = target_x - current_x;
    if (move_y<0)
    {   
        move_y = move_y*(-1);
        move_down(move_y, 200);
    }
	else if (move_y>0){
		move_up(move_y, 200);
	}
	if (move_x<0)
    {
        move_x = move_x*(-1);
        move_left(move_x, 200);
    }
	else if (move_x>0){
		move_right(move_x, 200);
	}
}

void zero(void){
	move_x = current_x-300;
	move_y = current_y -300;

	move_left(move_x, 200);
    move_down(move_y, 200);
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