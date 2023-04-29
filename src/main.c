#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

#define MOTOR_STEP_PIN PB0
#define MOTOR_DIR_PIN PB1

void step_pulse (void);
void move_stepper_motor (uint16_t,uint8_t);

int main(void) {
    //set motor control pins as output
    DDRB |=(1<<MOTOR_STEP_PIN)|(1<<MOTOR_DIR_PIN);

    while(1){
        //Rotate motor clockwise for 200 steps
        move_stepper_motor(200,1);
        _delay_ms(500);

        //Rotate motor counter-clockwise for 200 steps
        move_stepper_motor(200,0);
        _delay_ms(500);
    }
}

void step_pulse(){
    PORTB |= (1<<MOTOR_STEP_PIN);
    _delay_us(2);
    PORTB &= ~(1<<MOTOR_STEP_PIN);
    _delay_us(2);
}

void move_stepper_motor (uint16_t steps,uint8_t direction){
    //Set direction pin to high or low
    if(direction){
        PORTB|=(1<<MOTOR_DIR_PIN);
    }
    else{
        PORTB &= ~(1<<MOTOR_DIR_PIN);
    }

    //Send pulses to the step pin to move the motor
    for (int i =0; i<steps; i++){
        step_pulse();
        _delay_us(200); //adjust this delay to control speed
    }
}