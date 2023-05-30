#define F_CPU 16000000ul
#include <avr/io.h>
#include <util/delay.h>
#include "i2cmaster.h"
#define BAUDRATE 9600
#define BAUD_PRESCALER (((F_CPU/(BAUDRATE*16UL)))-1)
#include <stdio.h>
#include "usart.h"


void usart_init(void);
unsigned char usart_receive(void);
void usart_send (unsigned char data);



int main(void){
    usart_init();
    io_redirect();
    while(1){
        usart_send(1);
        _delay_ms(1000);
        usart_send(2);
        _delay_ms(1000);
        usart_send(3);
        _delay_ms(1000);
        usart_send(4);
        _delay_ms(1000);
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