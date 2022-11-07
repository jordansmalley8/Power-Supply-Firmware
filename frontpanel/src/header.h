#include <SPI.h>                        
#include <Arduino.h>
#include <Wire.h> 

#include "SparkFun_Qwiic_Twist_Arduino_Library.h"
#include "Adafruit_NeoPixel.h"

extern TWIST twist;
extern TWIST twist2;

extern const byte s7sAddress;
extern const byte s7sAddress2;


extern unsigned long b3lastdebouncetime;
extern unsigned long b2lastdebouncetime;
extern unsigned long b1lastdebouncetime;
extern unsigned long enablelastdebouncetime;

extern unsigned int counter;

extern int buttonMode; 
extern boolean enableButtonState;

extern char tempString[10];

extern int voltage;
extern int current;

extern boolean flag;
extern boolean flag2;


extern byte receivedbuffer [6];
extern byte sendbuffer [6];

extern byte state;

extern volatile byte indx;
extern volatile boolean process;

// This custom function works somewhat like a serial.print.
//  You can send it an array of chars (string) and it'll print
//  the first 4 characters in the array.
void s7sSendStringI2C(String);

void s7sSendStringI2C_2(String);

// Send the clear display command (0x76)
//  This will clear the display and reset the cursor
void clearDisplayI2C();

void clearDisplayI2C_2();

void setCursor(byte);

// Set the displays brightness. Should receive byte with the value
//  to set the brightness to
//  dimmest------------->brightest
//     0--------127--------255
void setBrightnessI2C(byte);

void setBrightnessI2C_2(byte);

// Turn on any, none, or all of the decimals.
//  The six lowest bits in the decimals parameter sets a decimal 
//  (or colon, or apostrophe) on or off. A 1 indicates on, 0 off.
//  [MSB] (X)(X)(Apos)(Colon)(Digit 4)(Digit 3)(Digit2)(Digit1)
void setDecimalsI2C(byte);

void setDecimalsI2C_2(byte);

void button3();

void button2();
void button1();

void enableButton();
