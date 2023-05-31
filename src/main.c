#include <avr/io.h>
#include <util/delay.h>
#include <SD.h>
#include <TMRpcm.h>
#include <SPI.h>

#define F_CPU 16000000UL
#define BAUDRATE 9600
#define BAUD_PRESCALER (((F_CPU / (BAUDRATE * 16UL)))-1)

const int SD_CS_PIN = 10;

// Create an object of TMRpcm class
TMRpcm tmrpcm;

void usart_init(void);
unsigned char usart_receive(void);
void processReceivedData(unsigned char data);

void setup() {
  // configuration of the inputs buttons
  DDRC = 0xF0; // I/O BORAD: PC0...3 AS inputs FOR buttons
  PORTC = 0x3F; // Enable internal pull at PC0..3 INPUTS
  DDRD = 0xFF;
  PORTD = 0x00;

  usart_init();
  Serial.begin(9600);

  // Initialize SD card
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD card initialization failed!");
    while (1);
  }
  tmrpcm.speakerPin = 9;  // Connect the speaker to pin 9
  tmrpcm.setVolume(7);    // Set volume level (0 to 7)

}

void loop() {
  // Your main code here
  
  // Check for received data
  if (UCSR0A & (1 << RXC0)) {
    unsigned char received_data = usart_receive();
    
    // Process the received data
    processReceivedData(received_data);
    Serial.print(received_data);
  }
}

void usart_init(void) {
  UBRR0H = (uint8_t)(BAUD_PRESCALER >> 8);
  UBRR0L = (uint8_t)(BAUD_PRESCALER);
  UCSR0B = (1 << RXEN0) | (1 << TXEN0);
  UCSR0C = (1 << UCSZ00) | (1 << UCSZ01);
}

unsigned char usart_receive(void) {
  while (!(UCSR0A & (1 << RXC0))); // wait for new data
  return UDR0; // received data
}

void processReceivedData(unsigned char data) {
  if (data == 1){
      tmrpcm.play("audio1.wav");
    }
   if  (data == 2){
     tmrpcm.play("audio2.wav");
   }
   if  (data == 3){
     tmrpcm.play("audio3.wav");
   }
   if  (data == 4){
     tmrpcm.play("audio4.wav");
   }
   if  (data == 5){ // you won your score is x
     tmrpcm.play("audio5.wav");
     if  (data == 21){ // sounds with numbers , for 1 point number 21
     tmrpcm.play("audio21.wav");
   }
   if  (data == 22){
     tmrpcm.play("audio22.wav");
   }
   if  (data == 23){
     tmrpcm.play("audio23.wav");
   }
   if  (data == 24){
     tmrpcm.play("audio24.wav");
   }
   if  (data == 25){
     tmrpcm.play("audio25.wav");
   }
   if  (data == 26){
     tmrpcm.play("audio26.wav");
   }
   if  (data == 27){
     tmrpcm.play("audio27.wav");
   }
   if  (data == 28){
     tmrpcm.play("audio28.wav");
   }
   if  (data == 29){
     tmrpcm.play("audio29.wav");
   }
   if  (data == 30){
     tmrpcm.play("audio30.wav");
   }
   if  (data == 31){
     tmrpcm.play("audio31.wav");
   }
   if  (data == 32){
     tmrpcm.play("audio32.wav");
   }
   if  (data == 33){
     tmrpcm.play("audio33.wav");
   }
   if  (data == 34){
     tmrpcm.play("audio34.wav");
   }
   if  (data == 35){
     tmrpcm.play("audio35.wav");
   }
   if  (data == 36){
     tmrpcm.play("audio36.wav");
   }
   if  (data == 37){
     tmrpcm.play("audio37.wav");
   }
   if  (data == 38){
     tmrpcm.play("audio38.wav");
   }
   if  (data == 39){
     tmrpcm.play("audio39.wav");
   }
   if  (data == 40){
     tmrpcm.play("audio40.wav");
   }
   }
     if  (data == 6){
     tmrpcm.play("audio6.wav");
   }
   if  (data == 7){
     tmrpcm.play("audio7.wav");
   }
   if  (data == 8){
     tmrpcm.play("audio8.wav");
   }
   if  (data == 9){
     tmrpcm.play("audio9.wav");
   }
   if  (data == 10){
     tmrpcm.play("audio10.wav");
   }
   if  (data == 11){
     tmrpcm.play("audio10.wav");
   }
   if  (data == 12){
     tmrpcm.play("audio12.wav");
   }
   if  (data == 13){
     tmrpcm.play("audio13.wav");
   }
   if  (data == 14){
     tmrpcm.play("audio14.wav");
   }
   if  (data == 15){
     tmrpcm.play("audio15.wav");
   }
   if  (data == 16){
     tmrpcm.play("audio16.wav");
   }
   if  (data == 17){
     tmrpcm.play("audio17.wav");
   }
   if  (data == 18){
     tmrpcm.play("audio18.wav");
     if  (data == 21){ // sounds with numbers , for 1 point number 21
     tmrpcm.play("audio21.wav");
   }
   if  (data == 22){
     tmrpcm.play("audio22.wav");
   }
   if  (data == 23){
     tmrpcm.play("audio23.wav");
   }
   if  (data == 24){
     tmrpcm.play("audio24.wav");
   }
   if  (data == 25){
     tmrpcm.play("audio25.wav");
   }
   if  (data == 26){
     tmrpcm.play("audio26.wav");
   }
   if  (data == 27){
     tmrpcm.play("audio27.wav");
   }
   if  (data == 28){
     tmrpcm.play("audio28.wav");
   }
   if  (data == 29){
     tmrpcm.play("audio29.wav");
   }
   if  (data == 30){
     tmrpcm.play("audio30.wav");
   }
   if  (data == 31){
     tmrpcm.play("audio31.wav");
   }
   if  (data == 32){
     tmrpcm.play("audio32.wav");
   }
   if  (data == 33){
     tmrpcm.play("audio33.wav");
   }
   if  (data == 34){
     tmrpcm.play("audio34.wav");
   }
   if  (data == 35){
     tmrpcm.play("audio35.wav");
   }
   if  (data == 36){
     tmrpcm.play("audio36.wav");
   }
   if  (data == 37){
     tmrpcm.play("audio37.wav");
   }
   if  (data == 38){
     tmrpcm.play("audio38.wav");
   }
   if  (data == 39){
     tmrpcm.play("audio39.wav");
   }
   if  (data == 40){
     tmrpcm.play("audio40.wav");
   }
   }
   if  (data == 19){
     tmrpcm.play("audio19.wav");
   }
   if  (data == 41){
     tmrpcm.play("audio41.wav");
   }
   if  (data == 42){
     tmrpcm.play("audio42.wav");
   }
   if  (data == 43){
     tmrpcm.play("audio43.wav");
   }
    if  (data == 44){
     tmrpcm.play("audio44.wav");
   }

  Serial.println("Playback completed!");
}
