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
uint8_t timeOut = 16;    //dluzszy time out -> zmienic typ danych na uint16_t
uint8_t TxBuffer[8];  //Bufor nadawczy 
uint8_t RxBuffer[8];  //Bufor odbiorczy
const byte addresses[][6] = {"00001", "00002"};   //adresy strumieni przesyłu danych

/*          User Fucntions Prototypes     */
void pinToggle( uint16_t pin);
void bufferReset( uint8_t *buf);
uint16_t dataMerge( uint8_t x, uint8_t y, );  //Zmiana dataMerge na funkcje wsadzajaca dane do TxBuffer


RF24 radio(7, 8); // CE, CSN

void setup() {
  delay(5);
  pinMode(TxLED, OUTPUT);
  pinMode(RxLED, OUTPUT);
  delay(1);

  bufferReset(TxBuffer); //zerowanie buforow Tx i Rx
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
  uint16_t MeasX = analogRead(A1);      //w zakresie 0 - 1024; 0 -0V; 1024 - 5V
  uint16_t MeasY = analogRead(A0);     // - || -
  uint16_t MeasSwt =  analogRead(A2);  // - || -
  uint8_t axisX = map(MeasX, 0, 1023, 0, 255);    // Zmiana wartosci z 10bitowych na 8bitowe - takei beda wykorzystwane jako DC w sterowaniu silnikami
  uint8_t axisY = map(MeasY, 0, 1023, 0, 255);    
  uint8_t swt = map(MeasSwt, 0, 1023, 0, 255);

  /* Wysyłanie danych */
  boolean sendState = 0;      //Zmiany! Redukcja do jednej zmiennej stanu wysłania danych - sendState; Usuniecie sendStateSwt
  uint8_t timeOutCounter = 0;     // licznk time out

  /* Radio go on */
  radio.stopListening();                // Zastanowic sie czy nie przeniesc przed petle while()

  /* Sklejanie danych do wysłania */

  while ((sendState == 0 ) || (timeOutCounter < timeOut ))
  {
    sendStateAxis = radio.write(&DataAxis, sizeof(DataAxis));      //wysylanie danych polozenia osi oraz zwracanie stanu wyslania
    sendStateSwt = radio.write(&DataSwitch, sizeof(DataSwitch));   //wysylanie danych przycisku -||-
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

// Funkcja sklejania
// Argumenty: x - liczba( 2 bajtowa) ustawiana na dwóch najstarszych bajtach,
//           y - liczba (2 bajtowa) ustawiana na dwóhc najmłodszych bitach
uint16_t dataMerge( uint8_t x, uint8_t y) {
  long int DataOut = 0x00;
  DataOut = x << 8;            // Przesunięcie wartości dla osi X na starsze bity
  DataOut |= y;                // Dostawianie zmiennej tmp = 0x00(axisY) do 'dataOut'
  return DataOut;
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
  for(uint8_t i = 0; i < bufSize; i++) {   
    buf[i] = 0;
  }
}
