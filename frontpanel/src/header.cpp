#include <header.h>
#include <Arduino.h>
#include <Wire.h> 
#include <SPI.h>    
    
#include "SparkFun_Qwiic_Twist_Arduino_Library.h"
#include "Adafruit_NeoPixel.h"
TWIST twist;
TWIST twist2;

const byte s7sAddress = 0x71;
const byte s7sAddress2 = 0x72;


unsigned long b3lastdebouncetime = 0;
unsigned long b2lastdebouncetime = 0;
unsigned long b1lastdebouncetime = 0;
unsigned long enablelastdebouncetime = 0;

unsigned int counter = 0;

int buttonMode = 0; 
boolean enableButtonState = 0;

char tempString[10];

int voltage = 0;
int current = 0;

boolean flag = false;
boolean flag2 = false;


byte receivedbuffer [6];
byte sendbuffer [6] = {0,0,0,0,0,0};

byte state = 0x00;

volatile byte indx;
volatile boolean process;

// This custom function works somewhat like a serial.print.
//  You can send it an array of chars (string) and it'll print
//  the first 4 characters in the array.
void s7sSendStringI2C(String toSend)
{
  Wire.beginTransmission(s7sAddress);
  for (int i=0; i<4; i++)
  {
    Wire.write(toSend[i]);
  }
  Wire.endTransmission();
}

void s7sSendStringI2C_2(String toSend)
{
  Wire.beginTransmission(s7sAddress2);
  for (int i=0; i<4; i++)
  {
    Wire.write(toSend[i]);
  }
  Wire.endTransmission();
}

// Send the clear display command (0x76)
//  This will clear the display and reset the cursor
void clearDisplayI2C()
{
  Wire.beginTransmission(s7sAddress);
  Wire.write(0x76);  // Clear display command
  Wire.endTransmission();
}

void clearDisplayI2C_2()
{
  Wire.beginTransmission(s7sAddress2);
  Wire.write(0x76);  // Clear display command
  Wire.endTransmission();
}

void setCursor(byte pos)
{
  Wire.beginTransmission(s7sAddress);
  Wire.write(0x79);
  Wire.write(0x03); 
}
// Set the displays brightness. Should receive byte with the value
//  to set the brightness to
//  dimmest------------->brightest
//     0--------127--------255
void setBrightnessI2C(byte value)
{
  Wire.beginTransmission(s7sAddress);
  Wire.write(0x7A);  // Set brightness command byte
  //Wire.write(0x80);
  //Wire.write(s7sAddress2);

  Wire.write(235);  // brightness data byte
  Wire.endTransmission();
}

void setBrightnessI2C_2(byte value)
{
  Wire.beginTransmission(s7sAddress2);
  Wire.write(0x7A);  // Set brightness command byte
  //Wire.write(0x80);
  //Wire.write(s7sAddress2);

  Wire.write(235);  // brightness data byte
  Wire.endTransmission();
}

// Turn on any, none, or all of the decimals.
//  The six lowest bits in the decimals parameter sets a decimal 
//  (or colon, or apostrophe) on or off. A 1 indicates on, 0 off.
//  [MSB] (X)(X)(Apos)(Colon)(Digit 4)(Digit 3)(Digit2)(Digit1)
void setDecimalsI2C(byte decimals)
{
  Wire.beginTransmission(s7sAddress);
  Wire.write(0x77);
  Wire.write(decimals);
  Wire.endTransmission();
}

void setDecimalsI2C_2(byte decimals)
{
  Wire.beginTransmission(s7sAddress2);
  Wire.write(0x77);
  Wire.write(decimals);
  Wire.endTransmission();
}

void button3()
{
  if((millis() - b3lastdebouncetime) > 200)
  {
    buttonMode = 3;
    Serial.println("button 3 pressed");
  }

  b3lastdebouncetime = millis();
}

void button2()
{
  if((millis() - b2lastdebouncetime) > 200)
  {
    buttonMode = 2;
    Serial.println("button 2 pressed");
  }

  b2lastdebouncetime = millis();
}

void button1()
{
  if((millis() - b1lastdebouncetime) > 200)
  {
    buttonMode = 1;
    Serial.println("button 1 pressed");
  }

  b1lastdebouncetime = millis();
}

void enableButton()
{
  if((millis() - enablelastdebouncetime) > 200)
  {
    enableButtonState = !enableButtonState;
    Serial.println("Enable Button Pressed");
  }

  enablelastdebouncetime = millis();
}
