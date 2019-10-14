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

#define BAUD  9600  //Default UART speed
#define TxLED 9     //
#define RxLED 10    //

#define LED_GREEN  5   //Switch LED
#define LED_RED    4   //Y axis LED
#define LED_YELLOW 3   //X axis LED

/* User Variables */
String LD_G = "LED Green";
String LD_R = "LED Red";
String LD_Y = "LED Yellow";

uint8_t RxBuffer[8];
const uint8_t RxAddresses[] = {0xAA, 0xAA, 0xAA, 0xAA, 0x01}; //Rx Pipes addresses
const uint8_t TxAddresses[] = {0xBB, 0xBB, 0xBB, 0xBB, 0x01}; //Tx Pipes addresses

/* User Function Prototypes */ 
void pinsInitPrint(String pinNum, String Name); 

RF24 radio(7, 8); // CE, CSN

void setup() {
  Serial.begin(BAUD);
  delay(5);
  Serial.println("Slave V0 application. /nUART correct initialization \nSpeed 9600 baud");

  /* Pins init */
  pinMode(LED_GREEN, OUTPUT);      /*  Definiowania pinów do sterowania LEDs */
  pinMode(LED_RED,   OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  digitalWrite(LED_GREEN,  LOW);   /* ustawiamy stan niski */
  digitalWrite(LED_RED ,    LOW);
  digitalWrite(LED_YELLOW, LOW);
  
  /* Pins init com. ONLY to DEBUG! */
  pinsInitPrint(String(LED_GREEN), LD_G);
  pinsInitPrint(String(LED_RED), LD_R);
  pinsInitPrint(String(LED_YELLOW), LD_Y);

  radio.begin();                                /* Radio go on */
  radio.openWritingPipe(TxAddresses);        // Otwarcie strumienia do nadawania - adres 00001
  radio.openReadingPipe(1, RxAddresses);     // Otwarcie strumienia do odbierania - adres 00002
  radio.setPALevel(RF24_PA_MIN);             // Low power of PowerAmp
  Serial.println("Radio initialization correct\n");
}

void loop() {
  delay(5);
  radio.startListening();                     /* Zacznij odbierać */
  if (radio.available()) {
    while (radio.available()) {
    
      radio.read(RxBuffer, sizeof(RxBuffer));    //zmienne do zmiany
    }
    delay(5);
    radio.stopListening();


    /* Odpowiedz zwrotna - klikniecie przycisku */
    // DO ZMIANY!
//    buttonState = digitalRead(button);
//    radio.write(&buttonState, sizeof(buttonState));
  }
}



void pinsInitPrint(String pinNum, String Name) {
  Serial.println("Correct initialization of pin " +  pinNum  + ":" + Name);
}
