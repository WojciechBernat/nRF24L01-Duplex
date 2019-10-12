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
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*           Libraries           */
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

/*          Declered Variables          */
#define TxLED 4
#define RxLED 6

/*          User Variables              */
uint8_t timeOut = 16;              //Dluzszy time out -> zmienic typ danych na uint16_t
uint8_t MeasBuffer[3];             //Bufor pomiarowy
uint8_t TxBuffer[4];               //Bufor nadawczy
uint8_t RxBuffer[8];               //Bufor odbiorczy

const uint8_t TxAddresses[] = {0xAA, 0xAA, 0xAA, 0xAA, 0x01}; //Tx Pipes addresses
const uint8_t RxAddresses[] = {0xBB, 0xBB, 0xBB, 0xBB, 0x01}; //Tx Pipes addresses
const byte addresses[][6] = {"00001", "00002"};            //adresy strumieni przesyłu danych
static const uint8_t analogPins[] = {A0, A1, A2, A3, A4};

/*          User Fucntions Prototypes     */
void pinToggle( uint16_t pin);
void bufferReset( uint8_t *buf);
void dataLoad( uint8_t *buf, uint8_t *data);                                   //Zmiana dataMerge na funkcje wsadzajaca dane do TxBuffer
boolean positionMeasure(uint8_t  *buf, uint8_t bufSize, uint8_t analogPins);    //Pomiar napiec oraz przeskalowanie na 8bitowe zmienne

/*  UART print functions */
void MeasBufferPrint(uint8_t *buf, uint8_t bufSize );
void TxBufferPrint(uint8_t *buf, uint8_t bufSize );
void RxBufferPrint(uint8_t *buf, uint8_t bufSize );
void RadioInitPrint(const uint8_t *TxBufAddress, uint8_t TxSize, const uint8_t *RxBufAddress, uint8_t RxSize);
void TxToSendPrint(uint8_t *buf, uint8_t bufSize);

RF24 radio(7, 8); // CE, CSN
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  delay(5);
  /* UART init */
  Serial.begin(9600);
  delay(5);
  Serial.println("Master V0 application.\nUART correct initialization \nSpeed 9600 baud"); //Start message
  /* Pins init */
  pinMode(TxLED, OUTPUT);
  pinMode(RxLED, OUTPUT);
  Serial.println("Pins correct initialization \nTxLED OUTPUT \nRxLED OUTPUT");
  delay(500);

  /* Buffer cleaning and sending bufers's content*/
  bufferReset(MeasBuffer);
  MeasBufferPrint(MeasBuffer, sizeof(MeasBuffer));
  bufferReset(TxBuffer);
  TxBufferPrint(TxBuffer, sizeof(TxBuffer));
  bufferReset(RxBuffer);
  RxBufferPrint(RxBuffer, sizeof(RxBuffer));
  delay(5);
  Serial.println("Buffers correct RESET");

  /* Radio go on */
  radio.begin();
  radio.openWritingPipe(TxAddresses);     //
  radio.openReadingPipe(1, RxAddresses);  //
  radio.setPALevel(RF24_PA_MIN);
  RadioInitPrint(TxAddresses, sizeof(TxAddresses), RxAddresses, sizeof(RxAddresses));
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  //Start of loop()
  delay(10); //zmniejsze z 5 na 2
  /* Pomiary */
  positionMeasure(MeasBuffer, sizeof(MeasBuffer) , 3); //na 'sztynwo' ustawiona wartosc portow analogowych
  MeasBufferPrint(MeasBuffer, sizeof(MeasBuffer));

  /* Sklejanie danych do wysłania */
  dataLoad(TxBuffer, MeasBuffer);
  TxBufferPrint(TxBuffer, sizeof(TxBuffer));
  /* Radio go on */
  radio.stopListening();                // Zastanowic sie czy nie przeniesc przed petle while()

  /* Zmienne warunku pętli while */
  boolean sendState = false;      //Zmiany! Redukcja do jednej zmiennej stanu wysłania danych - sendState; Usuniecie sendStateSwt
  uint8_t timeOutCounter = 0;     // licznk time out
  /* Wysyłanie danych */
  while ((sendState == false) || (timeOutCounter < timeOut ))
  {
    sendState = radio.write(&TxBuffer, sizeof(TxBuffer));      //Zmiana bufora na TxBuffer
    pinToggle(TxLED);                                          // TxLED toggle
    TxToSendPrint(TxBuffer, sizeof(TxBuffer));                 //Wysyanie na port szeregowy danych które sa wyyłane prze radio link
  }
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /* Powrot do odbierania */
  delay(3);   //zmniejsze na 3
  radio.startListening();
  while (!radio.available())      //Sprawdzenie czy jakieś bajty są do odczytania z bufora RX nRF24 - koniec pętli gdy nic nie ma do odczytu
  {
    radio.read(&RxBuffer, sizeof(RxBuffer));    //Odczytanie z bufora RF
    pinToggle(RxLED);
    Serial.println("Recieved Data");            //wrzucic do funkcji
    for (uint8_t i = 0; i < sizeof(RxBuffer); i++) {
      Serial.println( " Recieved byte: " + RxBuffer[i]);
    }
  }

  //End of loop()
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* User Functions */

//Funkcja wykonujaca pomiar i wsadzajaca dane do bufora measBuffer, przy jednoczesnym remapowaniu na dane 8bitowe
boolean positionMeasure(uint8_t  *buf, uint8_t bufSize, uint8_t analogPins)  {
  if (analogPins <= bufSize) {
    for (int i = 0; i < analogPins; i++) {
      buf[i] = map(analogRead(i), 0, 1023, 0, 255); //Odczyt z i-tego pinu analogowego; remap z 10bitów na 8bitów; wstawienie do itej komórki bufora
      Serial.println(buf[i]);    
    }
    return true;    //Jezeli wykonano trzy iteracje (analogPin0, 1, 2) to zwróc true jako poprawnie wykonana funkcje
  }
  else {
    return false;     //jezeli
  }
} //end of function

/* Zakomentowano do testu innego kodu funkcji */
// if (analogPins > 0) {
//    for (int i = 0; i < analogPinsNum; i++) {
//      buf[i] = map(analogPins[i], 0, 1023, 0, 255);
//      Serial.println(buf[i]);
//    }
//    return true;
//  }
//  else {
//    return false;
//
//  }


// Funkcja ladowania danych do bufora
// dataLoad wstawia uint8 data do uint buf, w zależnosci od bufCounter
void dataLoad( uint8_t *buf, uint8_t *data) {
  uint8_t bufferSize = sizeof(buf);
  uint8_t dataSize = sizeof(data);
  if (dataSize < bufferSize) {
    uint8_t i;
    for (i = 0 ; i < dataSize; i++) {
      buf[i] = data[i];     //przepisywanie bufrow
    }
    if ( i == dataSize) {
      buf[i] = 0;           //gdy i = dataSize => i = 3 - wstaw tam 0
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

void TxBufferPrint(uint8_t *buf, uint8_t bufSize ) {
  Serial.print("TxBuffer size: \t");
  Serial.print(bufSize);
  Serial.print("\n");
  Serial.print("TxBuffer \t");
  for (int i = 0; i < bufSize; i++) {
    Serial.print(buf[i]);
    Serial.print("\t");
    if (i == (bufSize - 1)) {
      Serial.print("\n");
    }
  }

}

void RxBufferPrint(uint8_t *buf, uint8_t bufSize ) {

  Serial.print("\nRxBuffer size: \t");
  Serial.print(bufSize);
  Serial.print("\nRxBuffer \t");
  for (int i = 0; i < bufSize; i++) {
    Serial.print(buf[i]);
    Serial.print("\t");
    if (i == (bufSize - 1)) {
      Serial.print("\n");
    }
  }

}

void MeasBufferPrint(uint8_t *buf, uint8_t bufSize ) {
  Serial.print("\nMeasBuffer size: \t");
  Serial.print(bufSize);
  Serial.print("\nMeasBuffer \t");
  for (int i = 0; i < bufSize; i++) {
    Serial.print(buf[i]);
    Serial.print("\t");
    if (i == (bufSize - 1)) {
      Serial.print("\n");
    }
  }

}

void RadioInitPrint(const uint8_t *TxBufAddress, uint8_t TxSize, const uint8_t *RxBufAddress, uint8_t RxSize) {
  Serial.println("Radio correct initialization");
  Serial.print("\nTx pipe adress: \t");
  for (int i = 0; i < TxSize; i++) {
    Serial.print(TxBufAddress[i], HEX);
    Serial.print("\t");
  }
  Serial.print("\nRx pipe adress: \t");
  for (int i = 0; i < RxSize; i++) {
    Serial.print(RxBufAddress[i], HEX);
    Serial.print("\t");
  }
}

void TxToSendPrint(uint8_t *buf, uint8_t bufSize) {
  Serial.print("\nData to send; X,Y axis position: ");
  Serial.print(TxBuffer[0]);
  Serial.print(" , ");
  Serial.print(TxBuffer[1]);
  Serial.print(" ; switch position: ");
  Serial.print(TxBuffer[2]);
  Serial.print(" , ");
  Serial.print(TxBuffer[3]);
}
