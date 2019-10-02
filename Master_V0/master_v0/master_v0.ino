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

/*          User Fucntions Prototypes     */
long int dataMerge( int x, int y);
void pinToggle( uint16_t pin);

RF24 radio(7, 8); // CE, CSN
const byte addresses[][6] = {"00001", "00002"};

void setup() {
  delay(5);
  pinMode(TxLED, OUTPUT);
  pinMode(RxLED, OUTPUT);
  delay(5);
  /* Radio go on */
  radio.begin();
  radio.openWritingPipe(addresses[1]); // 00002
  radio.openReadingPipe(1, addresses[0]); // 00001
  radio.setPALevel(RF24_PA_MIN);

}

void loop() {
  delay(5);
  /* Pomiary */
  unsigned int axisX = analogRead(A1);  //w zakresie 0 - 1024; 0 -0V; 1024 - 5V
  unsigned int axisY = analogRead(A0);  // - || -
  unsigned int swt  =  analogRead(A2);  // - || -

  /* Wysyłanie danych */
  boolean sendStateAxis = 0;  //
  boolean sendStateSwt = 0;   //
  byte timeOutCounter = 0;    // licznk time out
  //byte TxLedCounter = 0;    // Do migania dioda

  /* Radio go on */
  radio.stopListening();                // Zastanowic sie czy nie przeniesc przed petle while()

  /* Sklejanie danych do wysłania */
  long int DataAxis = dataMerge(axisX, axisY);
  long int DataSwitch = dataMerge( 0x00, swt);

  while ((sendStateAxis == 0 && sendStateSwt == 0) || (timeOutCounter < timeOut ))
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
long int dataMerge( int x, int y) {
  long int DataOut = 0x0000;
  DataOut = x << 16;            // Przesunięcie wartości dla osi X na starsze bity
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
