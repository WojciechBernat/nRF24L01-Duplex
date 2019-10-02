/*
   Progam do modułu radiowego nRF24L01+
   Użyta biblioteka TMRh20
   https://tmrh20.github.io/RF24/
   http://tmrh20.github.io/RF24/classRF24.html

   Progam z komunikacją pomiędyz dwoma modułami nRF24L01+
   Płytki z uC - 2x Arduino Uno Rev. 3
*/

/*
   Program dla urządzenia 'Master' - nadrzędne
*/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define LED 4

byte timeOut = 255;    //dluzszy tim out -> zmienic typ danych na int
RF24 radio(7, 8); // CE, CSN
const byte addresses[][6] = {"00001", "00002"};
boolean buttonState = 0;

void setup() {
  delay(5);
    pinMode(LED, OUTPUT);
    
  delay(5);
  /* Radio go on */
  radio.begin();
  radio.openWritingPipe(addresses[1]); // 00002
  radio.openReadingPipe(1, addresses[0]); // 00001
  radio.setPALevel(RF24_PA_MIN);

}

void loop() {
  delay(5);

  /* Radio go on */
  radio.stopListening();
  unsigned int axisX = analogRead(A1);  //w zakresie 0 - 1024; 0 -0V; 1024 - 5V
  unsigned int axisY = analogRead(A0);  // - || -
  unsigned int swt  =  analogRead(A2);  // - || -

  /* Sklejanie danych do wysłania */  
  DataAxis = dataMerge(axisX, axisY);
  DataSwitch = dataMerge( 0x00, swt);

  // Wysyłanie danych
  boolean sendState = 0;
  timeOutCounter = 0;
  while(sendState == 0 || timeOutCounter < timeOut )
  {
    sendState = radio.write(&DataAxis, sizeof(DataAxis);      //wysylanie danych oraz zwracanei stanu wyslania 
    timOutCounter++;
  }

  delay(5);
  radio.startListening();
  

  
}

/* User Functions */

// Funkcja sklejania
//Argumenty: x - liczba( 2 bajtowa) ustawiana na dwóch najstarszych bajtach, 
//           y - liczba (2 bajtowa) ustawiana na dwóhc najmłodszych bitach
long int dataMerge( int x, int y) {
  
  long int DataOut = x << 16;    // Przesunięcie wartości dla osi X na starsze bity
  DataOut &= 0x0000;             // Czyszczenie mlodszych bitow - dla pewnosci
  long int tmp = 0x0000 | y;     // tymczasowa zmienan tmp do przetrzymania 4 bajtowej zmienne 'y'     
  DataOut |= tmp;                // Dostawianie zmiennej tmp = 0x00(axisY) do 'dataOut'

  return DataOut;
  
}
