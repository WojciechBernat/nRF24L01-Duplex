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

/*           Libraries           */
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

/*          Declered Variables          */
#define TxLED 4
#define RxLED 6
/*          User Variables              */
byte timeOut = 16;    //dluzszy tim out -> zmienic typ danych na int

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
  radio.stopListening();                // Zastanowic sie czy nie przeniesc przed petle while() 
  unsigned int axisX = analogRead(A1);  //w zakresie 0 - 1024; 0 -0V; 1024 - 5V
  unsigned int axisY = analogRead(A0);  // - || -
  unsigned int swt  =  analogRead(A2);  // - || -

  /* Sklejanie danych do wysłania */  
  long int DataAxis = dataMerge(axisX, axisY);
  long int DataSwitch = dataMerge( 0x00, swt);

  /* Wysyłanie danych */
  boolean sendState = 0;
  byte timeOutCounter = 0;
  byte TxLedCounter = 0;
  
  while((sendStateAxis == 0 && sendStateSwt == 0) || (timeOutCounter < timeOut ))
  {
    sendStateAxis = radio.write(&DataAxis, sizeof(DataAxis));      //wysylanie danych polozenia osi oraz zwracanie stanu wyslania 
    sendStateSwt = radio.write(&DataSwitch, sizeof(DataSwitch));   //wysylanie danych przycisku -||-
    timeOutCounter++;
    /* Miejsce na kod do migania dioda gdy jest nadawanie */
    // TxLED
  }

  /* Powrot do odbierania */
  delay(5);
  radio.startListening();
  while (!radio.available())
  {
  radio.read(&buttonState, sizeof(buttonState));

}

/* User Functions */

// Funkcja sklejania
// Argumenty: x - liczba( 2 bajtowa) ustawiana na dwóch najstarszych bajtach, 
//           y - liczba (2 bajtowa) ustawiana na dwóhc najmłodszych bitach
long int dataMerge( int x, int y) {
  
  long int DataOut = 0x0000;
  DataOut = x << 16;            // Przesunięcie wartości dla osi X na starsze bity
  DataOut |= y;                // Dostawianie zmiennej tmp = 0x00(axisY) do 'dataOut'

  return DataOut;
  
}
