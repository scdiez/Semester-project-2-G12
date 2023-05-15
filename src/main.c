#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include "i2cmaster.h"
#include "lcd.h"
#include "usart.h"

int main (){
uart_init(); // open the communication to the microcontroller
io_redirect();

//setting buttons
DDRC = 0xF0; //buttons inputs
PORTC = 0x3F; //Enable internal pullups at PC0..3

while(1){
  if (PINC == 0b00111110){ //first button pressed?
	  printf("DI0 pressed");
  } 
  if (PINC == 0b00111101){ //second button pressed?
	  printf("DI1 pressed");
  }  
    if (PINC == 0b00111011){ //third button pressed?
	  printf("DI2 pressed");
  } 
    if (PINC == 0b00110111){ //fourth button pressed?
	  printf("DI3 pressed");
  } 
}
}