#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include <stdlib.h>
#include "i2cmaster.h"
#include "lcd.h"
#include "usart.h"

int randnumx (void);
int randnumy (void);
int target_x,target_y;
int current_x=0;
int current_y=0;
int bla;

int main(void) {
    srand(0);
    printf("%d \n", target_x);
    printf("%d \n", target_y);
}

int randnumx (void){
    target_x = rand()%50; //generate number for position SET LIMITS ONCE TESTED
    return target_x;
}

int randnumy (void){
    target_y = rand()%50; //generate number for position SET LIMITS ONCE TESTED
    return target_y;
}