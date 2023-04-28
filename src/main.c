#include <avr/io.h>
#include <util/delay.h>

#define F_CPU 16000000UL

#define STEP_PIN PB3
#define DIR_PIN PB4

#define STEPS_PER_REV 200

void setup() {
  DDRB |= (1 << DIR_PIN) | (1 << STEP_PIN);
  TCCR1A |= (1 << WGM10) | (1 << WGM11);
  TCCR1B |= (1 << WGM12) | (1 << WGM13) | (1 << CS10);
  DDRB |= (1 << PB1) | (1 << PB2);
  PORTB &= ~(1 << DIR_PIN);
}

void loop() {
  int rpm = 60;
  int delayMicros = (60 * 1000000L) / (STEPS_PER_REV * rpm);

  PORTB &= ~(1 << DIR_PIN);
  for (int i = 0; i < STEPS_PER_REV; i++) {
    PORTB |= (1 << STEP_PIN);
    _delay_us(1);
    PORTB &= ~(1 << STEP_PIN);
    _delay_us(delayMicros - 1);
  }

  _delay_ms(1000);

  PORTB |= (1 << DIR_PIN);
  for (int i = 0; i < STEPS_PER_REV; i++) {
    PORTB |= (1 << STEP_PIN);
    _delay_us(1);
    PORTB &= ~(1 << STEP_PIN);
    _delay_us(delayMicros - 1);
  }

  _delay_ms(1000);
}

int main(void) {
  setup();
  while (1) {
    loop();
  }
  return 0;
}
