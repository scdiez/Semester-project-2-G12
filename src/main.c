#define F_CPU 16000000ul
#include <avr/io.h>
#include <util/delay.h>
#include "i2cmaster.h"
#define BAUDRATE 57600
#define BAUD_PRESCALER (((F_CPU/(BAUDRATE*16UL)))-1)
#include <stdio.h>
#include "usart.h"


void usart_init(void);
void usart_send (unsigned char data);



int main(void){
  usart_init();
  io_redirect();

  //Button configuration
  DDRC = 0xF0;
	PORTC = 0x3F;

  if (PINC == 0b00111110){
  usart_send(13); //"You've selected vision single player"
  _delay_ms(3000);
  usart_send(7); //"The basketball hoop will now move. You get 3 attempts to score."
  _delay_ms(4000);
  usart_send(6);// " Press Button 1 two times when you want to stop playing. Good Luck!" 
  _delay_ms(4000);
  usart_send(44); // "you have 8 seconds to get the ball in"
  _delay_ms(3000);
  }
    
return 0;

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