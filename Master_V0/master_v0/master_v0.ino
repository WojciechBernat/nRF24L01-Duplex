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
uint8_t timeOut = 16;              //Dluzszy time out -> zmienic typ danych na uint16_t
uint8_t MeasBuffer[3];            //Bufor pomiarowy
uint8_t TxBuffer[4];               //Bufor nadawczy
uint8_t RxBuffer[4];               //Bufor odbiorczy

const uint8_t addresses[][6] = {"00001", "00002"};            //adresy strumieni przesyłu danych
static const uint8_t analogPins[] = {A0, A1, A2, A3, A4};

/*          User Fucntions Prototypes     */
void pinToggle( uint16_t pin);
void bufferReset( uint8_t *buf);
boolean positionMeasure(uint16_t  *buf, uint8_t analogPinsNum);
void dataLoad( uint8_t *buf, uint8_t *data);  //Zmiana dataMerge na funkcje wsadzajaca dane do TxBuffer


RF24 radio(7, 8); // CE, CSN

void setup() {
  delay(5);
  pinMode(TxLED, OUTPUT);
  pinMode(RxLED, OUTPUT);
  delay(1);

  bufferReset(MeasBuffer);
  bufferReset(TxBuffer);    //zerowanie buforow Tx i Rx
  bufferReset(RxBuffer);

  /* Radio go on */
  radio.begin();
  radio.openWritingPipe(addresses[1]); // 00002
  radio.openReadingPipe(1, addresses[0]); // 00001
  radio.setPALevel(RF24_PA_MIN);

}

void loop() {
  delay(5);
  /* Pomiary */
  positionMeasure(MeasBuffer, 3); //na 'sztynwo' ustawiona wartosc portow analogowych
  /* Sklejanie danych do wysłania */
  dataLoad(TxBuffer, MeasBuffer);
  /* Radio go on */
  radio.stopListening();                // Zastanowic sie czy nie przeniesc przed petle while()

  /* Wysyłanie danych */
  boolean sendState = 0;           //Zmiany! Redukcja do jednej zmiennej stanu wysłania danych - sendState; Usuniecie sendStateSwt
  uint8_t timeOutCounter = 0;     // licznk time out
  while ((sendState == 0 ) || (timeOutCounter < timeOut ))
  {
    //    sendStateAxis = radio.write(&DataAxis, sizeof(DataAxis));      //wysylanie danych polozenia osi oraz zwracanie stanu wyslania
    //    sendStateSwt = radio.write(&DataSwitch, sizeof(DataSwitch));   //wysylanie danych przycisku -||-
    pinToggle(TxLED);                                              // TxLED toggle
  }

  /* Powrot do odbierania */
  int SlaveResponse = 0;
  delay(5);
  radio.startListening();
  while (!radio.available())
  {
    radio.read(&SlaveResponse, sizeof(SlaveResponse));
    pinToggle(RxLED);
  }
}

/* User Functions */

// Funkcja ladowania danych do bufora
// dataLoad wstawia uint8 data do uint buf, w zależnosci od bufCounter
void dataLoad( uint8_t *buf, uint8_t *data) {
  uint8_t bufferSize = sizeof(buf);
  uint8_t dataSize = sizeof(data);
  if (dataSize < bufferSize) {
    for (int i = 0 ; i <= dataSize; i++) {
        buf[i] = data[i];
    }
  }
}

//Funckja przelaczanai diody/pinu
void pinToggle(uint16_t pin) {
  if (digitalRead(pin) == HIGH) {
    digitalWrite(pin, LOW);
  }
  else {
    digitalWrite(pin, HIGH);
  }
}

//Funkcja resetowanai bufora
void bufferReset( uint8_t *buf) {
  uint8_t bufSize = sizeof(buf);
  for (uint8_t i = 0; i < bufSize; i++) {
    buf[i] = 0;
  }
}

//Funkcja resetowanai bufora
void bufferReset_16( uint16_t *buf) {
  uint16_t bufSize = sizeof(buf);
  for (uint8_t i = 0; i < bufSize; i++) {
    buf[i] = 0;
  }
}

//Funkcja wykonujaca pomiar i wsadzajaca dane do bufora measBuffer, przy jednoczesnym remapowaniu na dane 8bitowe
boolean positionMeasure(uint8_t  *buf, uint8_t analogPinsNum)  {
  if (analogPins > 0) {
    for (int i = 0; i < analogPinsNum; i++) {
      buf[i] = map(analogPins[i], 0, 1023, 0, 255);
    }
    return true;
  }
  else {
    return false;

  }
}
