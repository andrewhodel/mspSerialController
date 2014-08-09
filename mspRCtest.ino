// with arduino 1.0.5 you must remove RobotControl from libraries for
// this to compile
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1351.h"
#include <SPI.h>
#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>

// deadband range
#define DEADBAND 40

// hardware SPI pins for OLED
#define sclk 13
#define mosi 11
#define cs 10
#define rst 9
#define dc 8

// Color definitions
#define BLACK 0x0000
#define BLUE 0x001F
#define RED 0xF800
#define GREEN 0x07E0
#define CYAN 0x07FF
#define MAGENTA 0xF81F
#define YELLOW 0xFFE0
#define WHITE 0xFFFF

// load OLED using hardware SPI interface
Adafruit_SSD1351 tft = Adafruit_SSD1351(cs, dc, rst);

// The indices of the 8 signals in the following arrays
#define ROLL 0
#define PITCH 1
#define YAW 2
#define THROTTLE 3
#define AUX1 4
#define AUX2 5
#define AUX3 6
#define AUX4 7

// buffer for storing rc values
uint16_t rc[8] = { 1500,1500,1500,1000,1000,1000,1000,1000 };

// min max defaults for sticks
//uint16_t rcAnalogMinMax[8] = { 388,639,396,646,410,650,406,657 };
uint16_t rcAnalogMinMax[8] = { 430,630,430,630,430,630,430,630 };

// Buffer for storing the serialized byte form of the RC signals
uint8_t rc_bytes[16];

// buffer for analog rc values
uint16_t analogReads[4];

// rssi
uint8_t rssi = 0;

// time counters
unsigned long previousMillis=0;
unsigned long previousMillis2=0;

void setup() {

  // setup digital pins
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
  pinMode(6, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);

  // setup serial
  //Serial.begin(9600);

  // init the screen
  tft.begin();
  tft.fillScreen(BLACK);
  tft.setTextColor(RED);
  tft.setCursor(0,0);
  tft.print("ROLL");
  tft.setCursor(0,10);
  tft.print("PITCH");
  tft.setCursor(0,20);
  tft.print("YAW");
  tft.setCursor(0,30);
  tft.print("THROTTLE");
  tft.setCursor(85,0);
  tft.print("A1");
  tft.setCursor(85,10);
  tft.print("A2");
  tft.setCursor(85,20);
  tft.print("A3");
  tft.setCursor(85,30);
  tft.print("A4");
  
  tft.setTextColor(WHITE);
  tft.setCursor(0,50);
  tft.print("ALTCM");
  tft.setCursor(0,60);
  tft.print("HOLDCM");
  
  tft.setTextColor(BLUE);
  tft.setCursor(0,80);
  tft.print("RSSI");
  tft.setCursor(50,80);
  tft.print("ARM");
  tft.setTextColor(RED);
  
  tft.setTextColor(GREEN);
  tft.setCursor(0,100);
  tft.print("GPS SAT");
  tft.setCursor(70,100);
  tft.print("HOME");
  tft.setCursor(0,110);
  tft.print("SPEED");
  tft.setCursor(70,110);
  tft.print("MAX");
  tft.setTextColor(RED);
  
  delay(500);

  // initialize timer1 
  cli(); // disable all interrupts
  
  // init serial
  UCSR0A  = (1<<U2X0);
  UBRR0H = ((F_CPU  / 4 / 9600 -1) / 2) >> 8;;
  UBRR0L = ((F_CPU  / 4 / 9600 -1) / 2);
  UCSR0B |= (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0);
  
  // TIMER1
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  OCR1A = 31250;            // compare match register 16MHz/256/2Hz
  TCCR1B |= (1 << WGM12);   // CTC mode
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
  
  sei(); // enable interrupts

}

// inbound data buffer
volatile uint8_t inbuf[200];
volatile uint8_t c = 0;

volatile int32_t gAltitude = 0;
volatile int32_t gHoldAltitude = 0;
unsigned long gArmedTimer = 0;
volatile uint16_t gGPSNumSat = 0;
volatile uint16_t gGPSHome = 0;
volatile uint16_t gGPSSpeed = 0;
volatile uint16_t gGPSMax = 0;

ISR(USART_RX_vect)
{
   uint8_t r = UDR0; // received byte value into r
   if (r == '$' && c != 0) {
    procMsp(inbuf);
    inbuf[0] = r;
    c = 1;
   } else {
    inbuf[c] = r;
    c++;
  }
}

ISR(TIMER1_COMPA_vect) // timer 2hz
{
  
  if (rc[AUX1] == 1999) {
    // inc armed timer
    gArmedTimer += 500;
  }

  rssi = pulseIn(3, LOW, 200);

  updateOled();
}

void loop() {

  // every 30ms
  unsigned long currentMillis = millis();
  if ((unsigned long)(currentMillis - previousMillis) >= 30) {

    // get analog reads
    analogReads[0] = analogRead(0);
    analogReads[1] = analogRead(1);
    analogReads[2] = analogRead(2);
    analogReads[3] = analogRead(3);

    // reset min max if they are different than defaults for analog
    for(int i = 0; i < 4; i++) {
      if (analogReads[i] < rcAnalogMinMax[i*2]) {
        rcAnalogMinMax[i*2] = analogReads[i];
      } else if (analogReads[i] > rcAnalogMinMax[i*2+1]) {
        rcAnalogMinMax[i*2+1] = analogReads[i];
      }
    }

    // set AUX from digital pins
    if (digitalRead(4) == LOW) {
      rc[AUX1] = 1999;
    } else {
      rc[AUX1] = 1000;
      // turn off timer
      gArmedTimer = 0;
    }
    if (digitalRead(5) == LOW) {
      rc[AUX2] = 1999;
    } else {
      rc[AUX2] = 1000;
    }
    if (digitalRead(6) == LOW) {
      rc[AUX3] = 1999;
    } else {
      rc[AUX3] = 1000;
    }
    if (digitalRead(7) == LOW) {
      rc[AUX4] = 1999;
    } else {
      rc[AUX4] = 1000;
    }

    // set analog values
    rc[ROLL] = deadband(aToRc(analogReads[0], rcAnalogMinMax[0], rcAnalogMinMax[1]), 1500);
    rc[PITCH] = deadband(aToRc(analogReads[1], rcAnalogMinMax[2], rcAnalogMinMax[3]), 1500);
    rc[YAW] = deadband(aToRc(analogReads[2], rcAnalogMinMax[4], rcAnalogMinMax[5]), 1500);
    rc[THROTTLE] = deadband(aToRc(analogReads[3], rcAnalogMinMax[6], rcAnalogMinMax[7]), 1000);

    send_msp();

    previousMillis = currentMillis;
  }

}

int16_t ret16(int8_t a, int8_t b) {
  int16_t t = a;
  t+= (int16_t)b<<8;
  return t;
}

int32_t ret32(int8_t a, int8_t b, int8_t c, int8_t d) {
  int32_t t = a;
  t+= (int32_t)b<<8;
  t+= (int32_t)c<<8;
  t+= (int32_t)d<<8;
  return t;
}

uint16_t retu16(uint8_t a, uint8_t b) {
  uint16_t t = a;
  t+= (uint16_t)b<<8;
  return t;
}

uint32_t retu32(uint8_t a, uint8_t b, uint8_t c, uint8_t d) {
  uint32_t t = a;
  t+= (uint32_t)b<<8;
  t+= (uint32_t)c<<8;
  t+= (uint32_t)d<<8;
  return t;
}

// read incoming msp
void procMsp(volatile uint8_t inbuf[]) {

  uint8_t dataLength = 0;
  uint8_t code = 0;
  uint8_t cks = 0;

  if (inbuf[0] == '$' && inbuf[1] == 'M' && inbuf[2] == '>') {
    // valid header
  } else {
    return;
  }

  if (inbuf[3] > 0) {
    // has data length
    cks ^= inbuf[3];
    dataLength = inbuf[3];
  } else {
    return;
  }

  if (inbuf[4] > 0) {
    // has code
    cks ^= inbuf[4];
    code = inbuf[4];
  } else {
    return;
  }

  // allocate local buffer
  uint8_t mybuf[100];

  // loop through data length
  for (int i=0; i<dataLength; i++) {
    mybuf[i] = inbuf[i+5];
    cks ^= mybuf[i];
  }

  // checksum
  if (cks == inbuf[5+dataLength]) {
    // valid checksum
    if (code == 109) {
      gAltitude = ret32(inbuf[5+0], inbuf[5+1], inbuf[5+2], inbuf[5+3]);
      gHoldAltitude = ret32(inbuf[5+6], inbuf[5+7], inbuf[5+8], inbuf[5+9]);
    } else if (code == 106) {
      gGPSNumSat = inbuf[5+1];
      gGPSSpeed = retu16(inbuf[5+12], inbuf[5+13]);
      if (gGPSSpeed > gGPSMax) {
        gGPSMax = gGPSSpeed;
      }
    } else if (code == 107) {
      gGPSHome = retu16(inbuf[5+0], inbuf[5+1]);
    }

  }

}

void USART_send(uint8_t data){
 
 while(!(UCSR0A & (1<<UDRE0)));
 UDR0 = data;
 
}

uint8_t sLoop = 0;

void send_msp() {

  uint8_t checksum = 0;
  
  // send MSP header
  USART_send('$');
  USART_send('M');
  USART_send('<');

  int j = 0;
  for(int i = 0; i < 15; i++) {
    switch (i) {
    case ROLL:
      rc[i] = deadband(rc[i],1500);
      break;
    case PITCH:
      rc[i] = deadband(rc[i],1500);
      break;
    case YAW:
      rc[i] = deadband(rc[i],1500);
      break;
    case THROTTLE:
      rc[i] = deadband(rc[i],1000);
      break;
    }
    rc_bytes[j++] = rc[i] & 0xFF; // LSB first
    rc_bytes[j++] = (rc[i] >> 8) & 0xFF; // MSB second
  }

  // Send message length
  USART_send(16);
  checksum ^= 16;

  // Send the op-code
  USART_send(200);
  checksum ^= 200;

  // Send the data bytes
  for(int i = 0; i < 16; i++) {
    USART_send(rc_bytes[i]);
    checksum ^= rc_bytes[i];
  }

  // Send the checksum
  USART_send(checksum);

  //return;
  if (sLoop == 40) {
    // send status requests
    sLoop = 0;

    // request altitude 109 and gps 106/107
    uint8_t codes[3] = { 109, 106, 107 };

    for (int i=0; i<3; i++) {
      checksum = 0;

      // Send the MSP header
      USART_send('$');
      USART_send('M');
      USART_send('<');

      // Send message length
      USART_send(0);
      checksum ^= 0;

      // Send the op-code
      USART_send(codes[i]);
      checksum ^= codes[i];

      // Send the checksum
      USART_send(checksum);
    }

  }

  sLoop += 1;

}

// apply deadband
uint16_t deadband(uint16_t v, uint16_t center) {
  if (center == 1500) {
    // centered stick
    if (v > 1500-DEADBAND && v < 1500+DEADBAND) {
      v = 1500;
    } else if (v > 1500+DEADBAND && v < 1500+(DEADBAND*2)) {
      // smoothing
      uint16_t vOver = v-(1500+DEADBAND);
      v = (vOver*2)+1500;
    } else if (v < 1500-DEADBAND && v > 1500-(DEADBAND*2)) {
      // smoothing
      uint16_t vUnder = (1500-DEADBAND)-v;
      v = 1500-(vUnder*2);
    }
  } else if (center == 1000) {
    // throttle
    if (v < 1000+DEADBAND) {
      v = 1000;
    }
  }
  return v;
}

// reverse with 1500 midpoint
uint16_t rev(uint16_t x) {
  if (x > 1500) {
    x = x-1500;
    x = 1500-x;
  } else if (x < 1500) {
    x = 1500-x;
    x = 1500+x;
  }
  return x;
}

// return an rc value between 1000-2000 for a pot, scaled with min, max
uint16_t aToRc(uint16_t v, uint16_t min, uint16_t max) {
  uint16_t diff = max - min;
  float scale = 1000.00/diff;
  return dround((v-min)*scale)+1000;
}

// round
uint16_t dround(double d)
{
  return floor(d + 0.5);
}

// to calc screen refresh
uint32_t lastScreen[20];

// update the screen
void updateOled() {
  // RC
  if (rc[ROLL] != lastScreen[ROLL]) {
    tft.fillRect(50,0,30,7,BLACK);
    tft.setCursor(50,0);
    tft.print(rc[ROLL]);
    lastScreen[ROLL] = rc[ROLL];
  }
  if (rc[PITCH] != lastScreen[PITCH]) {
    tft.fillRect(50,10,30,14,BLACK);
    tft.setCursor(50,10);
    tft.print(rc[PITCH]);
    lastScreen[PITCH] = rc[PITCH];
  }
  if (rc[YAW] != lastScreen[YAW]) {
    tft.fillRect(50,20,30,7,BLACK);
    tft.setCursor(50,20);
    tft.print(rc[YAW]);
    lastScreen[YAW] = rc[YAW];
  }
  if (rc[THROTTLE] != lastScreen[THROTTLE]) {
    tft.fillRect(50,30,30,7,BLACK);
    tft.setCursor(50,30);
    tft.print(rc[THROTTLE]);
    lastScreen[THROTTLE] = rc[THROTTLE];
  }
  
  // AUX
  if (rc[AUX1] != lastScreen[AUX1]) {
    tft.fillRect(100,0,30,7,BLACK);
    tft.setCursor(100,0);
    tft.print(rc[AUX1]);
    lastScreen[AUX1] = rc[AUX1];
  }
  if (rc[AUX2] != lastScreen[AUX2]) {
    tft.fillRect(100,10,30,14,BLACK);
    tft.setCursor(100,10);
    tft.print(rc[AUX2]);
    lastScreen[AUX2] = rc[AUX2];
  }
  if (rc[AUX3] != lastScreen[AUX3]) {
    tft.fillRect(100,20,30,7,BLACK);
    tft.setCursor(100,20);
    tft.print(rc[AUX3]);
    lastScreen[AUX3] = rc[AUX3];
  }
  if (rc[AUX4] != lastScreen[AUX4]) {
    tft.fillRect(100,30,30,7,BLACK);
    tft.setCursor(100,30);
    tft.print(rc[AUX4]);
    lastScreen[AUX4] = rc[AUX4];
  }

  // ALTITUDE
  if (gAltitude != lastScreen[8]) {
    tft.fillRect(45,50,65,7,BLACK);
    tft.setTextColor(WHITE);
    tft.setCursor(45,50);
    tft.print(gAltitude);
    tft.setTextColor(RED);
    lastScreen[8] = gAltitude;
  }
  if (gHoldAltitude != lastScreen[9]) {
    tft.fillRect(45,60,65,7,BLACK);
    tft.setTextColor(WHITE);
    tft.setCursor(45,60);
    tft.print(gHoldAltitude);
    tft.setTextColor(RED);
    lastScreen[9] = gHoldAltitude;
  }
  
  // RSSI
  if (rssi != lastScreen[10]) {
    tft.fillRect(30,80,20,7,BLACK);
    tft.setCursor(30,80);
    tft.setTextColor(BLUE);
    tft.print(rssi);
    tft.setTextColor(RED);
    lastScreen[10] = rssi;
  }
  
  // ARM TIMER
  if (gArmedTimer/1000 != lastScreen[11]) {
    tft.fillRect(75,80,50,7,BLACK);
    tft.setCursor(75,80);
    tft.setTextColor(BLUE);
    tft.print(gArmedTimer/1000);
    tft.setTextColor(RED);
    lastScreen[11] = gArmedTimer/1000;
  }
  
  // GPS
  if (gGPSNumSat != lastScreen[12]) {
    tft.fillRect(47,100,20,7,BLACK);
    tft.setCursor(47,100);
    tft.setTextColor(GREEN);
    tft.print(gGPSNumSat);
    tft.setTextColor(RED);
    lastScreen[12] = gGPSNumSat;
  }
  if (gGPSHome != lastScreen[13]) {
    tft.fillRect(100,100,20,7,BLACK);
    tft.setCursor(100,100);
    tft.setTextColor(GREEN);
    tft.print(gGPSHome);
    tft.setTextColor(RED);
    lastScreen[13] = gGPSHome;
  }
  if (gGPSSpeed != lastScreen[14]) {
    tft.fillRect(40,110,30,7,BLACK);
    tft.setCursor(40,110);
    tft.setTextColor(GREEN);
    tft.print(gGPSSpeed);
    tft.setTextColor(RED);
    lastScreen[14] = gGPSSpeed;
  }
  if (gGPSMax != lastScreen[15]) {
    tft.fillRect(90,110,30,7,BLACK);
    tft.setCursor(90,110);
    tft.setTextColor(GREEN);
    tft.print(gGPSMax);
    tft.setTextColor(RED);
    lastScreen[15] = gGPSMax;
  }

}




