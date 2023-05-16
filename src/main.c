#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include <stdlib.h>
#include "i2cmaster.h"
#include "lcd.h"
#include "usart.h"
#include <time.h>

int target_x,target_y;
int current_x=0;
int current_y=0;;

int main(void) {
    uart_init();
    io_redirect();

    srand(time(0));
    target_x = rand() % 50;
    target_y = rand() % 50;
    
    printf("Target x: %d \n", target_x);
    printf("Target y: %d \n", target_y);
    target_x = rand() % 50;
    target_y = rand() % 50,
    
    printf("Target x: %d \n", target_x);
    printf("Target y: %d \n", target_y);
    target_x = rand() % 50;
    target_y = rand() % 50,
    
    printf("Target x: %d \n", target_x);
    printf("Target y: %d \n", target_y);
}
