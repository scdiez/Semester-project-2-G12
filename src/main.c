#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include "i2cmaster.h"
#include "lcd.h"
//Change all printfs to audio 

int main (){

    uart_init(); // open the communication to the microcontroller
	io_redirect(); // redirect input and output to the communication

    //setting buttons
    DDRC = 0xF0; //buttons inputs
    PORTC = 0x3F; //Enable internal pullups at PC0..3

    while(1){
        printf("Hello, which mode would you like to play? You'll find the buttons in front of you \n");
        printf("Select button 1 for no vision single player, button 2 for no vision multiplayer, button 3 for vision single player and button 4 for vision multiplayer");


        if (PINC == 0b00111110){ //first button pressed?
            printf("No vision single player mode");
        } 
        if (PINC == 0b00111101){ //second button pressed?
            printf("No vision multiplayer mode");
        }  
            if (PINC == 0b00111011){ //third button pressed?
             printf("Vision single player mode");
        } 
            if (PINC == 0b00110111){ //fourth button pressed?
            printf("Vision multiplayer mode");
        } 
    }
}