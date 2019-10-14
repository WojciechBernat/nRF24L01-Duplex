/*
   Progam do modułu radiowego nRF24L01+
   Użyta biblioteka TMRh20
   https://tmrh20.github.io/RF24/
   http://tmrh20.github.io/RF24/classRF24.html

   Progam z komunikacją pomiędyz dwoma modułami nRF24L01+
   Płytki z uC - 2x Arduino Uno Rev. 3
*/

/*
   Program dla urządzenia 'Slave' - podrzedne
*/

//////////////////dodac wyrzucanie pomiarów na porcie szeregowym
// w dekodowaniu uzyc lowByte, highByte
// funkcje bitowe butRead, bitWrite, bit(), 

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define LedGreen  5
#define LedRed    4
#define LedYellow 3


RF24 radio(7, 8); // CE, CSN
const byte addresses[][6] = {"00001", "00002"};
boolean buttonState = 0;



void setup() {

  pinMode(LedGreen, OUTPUT);      /*  Definiowania pinów do sterowania LEDs */
  pinMode(LedRed,   OUTPUT);
  pinMode(LedYellow, OUTPUT);
  digitalWrite(LedGreen,  LOW);   /* ustawiamy stan niski */
  digitalWrite(LedRed,    LOW);
  digitalWrite(LedYellow, LOW);
  pinMode(button, INPUT);

  radio.begin();                                /* Radio go on */
  radio.openWritingPipe(addresses[0]);        // Otwarcie strumienia do nadawania - adres 00001
  radio.openReadingPipe(1, addresses[1]);     // Otwarcie strumienia do odbierania - adres 00002
  radio.setPALevel(RF24_PA_MIN);              // Low power of PowerAmp
}

void loop() {
  delay(5);
  radio.startListening();                     /* Zacznij odbierać */
  if ( radio.available()) {
    while (radio.available()) {
      unsigned int axisX = 0;
      unsigned int axisY = 0;
      unsigned int swt   = 0;
      
      radio.read(&angleV, sizeof(angleV));
    }
    delay(5);
    radio.stopListening();


    /* Odpowiedz zwrotna - klikniecie przycisku */
    buttonState = digitalRead(button);
    radio.write(&buttonState, sizeof(buttonState));
  }
}


/* functions */
