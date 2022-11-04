#include <Arduino.h>
#include <Wire.h> 
#include <SPI.h>                        

#include "SparkFun_Qwiic_Twist_Arduino_Library.h"
#include "Adafruit_NeoPixel.h"

#define LED_PIN 36
#define LED_COUNT 20
#define button1pin 2
#define button2pin 3 
#define button3pin 18
#define enablebuttonpin 19

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_BRWG + NEO_KHZ800);

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

ISR (SPI_STC_vect) // SPI interrupt routine 
{ 
  byte c = SPDR; // read byte from SPI Data Register
  SPDR = sendbuffer[indx];

  receivedbuffer [indx  ] = c; // save data in the next index in the array buff
  
  Serial.println("received byte: ");
  Serial.println(c);
  Serial.println("index: ");
  Serial.println(indx);
  
  if (c == 10) //check for the end of the word
  {
    process = true;
  }
  indx++;
   //Serial.println(c);
}

void setup()
{
  setCursor(0x03);
  indx = 0;
  SPI.setDataMode(SPI_MODE1);
  pinMode(MISO, OUTPUT); // have to send on master in so it set as output
  SPCR |= _BV(SPE); // turn on SPI in slave mode
  SPI.attachInterrupt(); // turn on interrupt


  /*pinMode(MOSI, OUTPUT);
  pinMode(MISO, INPUT);
  pinMode(SS, OUTPUT);  
  pinMode(SCK, OUTPUT);*/
  pinMode(18, INPUT_PULLUP);

  strip.begin();
  Serial.begin(9600);                 

  Serial.println("1");
  Wire.begin();  // Initialize hardware I2C pins
  Serial.println("2");
  // Clear the display, and then turn on all segments and decimals
  clearDisplayI2C();  // Clears display, resets cursor

  // Custom function to send four bytes via I2C
  //  The I2C.write function only allows sending of a single
  //  byte at a time.
  s7sSendStringI2C("-HI-");
  s7sSendStringI2C_2("-HI-");
  setDecimalsI2C(0b111111);  // Turn on all decimals, colon, apos
  setDecimalsI2C_2(0b111111);  // Turn on all decimals, colon, apos

  Serial.println("3");

  // Flash brightness values at the beginning
  setBrightnessI2C(0);  // Lowest brightness
  delay(1500);
  setBrightnessI2C(255);  // High brightness
  delay(1500);

  setBrightnessI2C_2(0);  // Lowest brightness
  delay(1500);
  setBrightnessI2C_2(255);  // High brightness
  delay(1500);

  // Clear the display before jumping into loop
  clearDisplayI2C();  
  clearDisplayI2C_2();

  attachInterrupt(digitalPinToInterrupt(button1pin), button1, RISING);
  attachInterrupt(digitalPinToInterrupt(button2pin), button2, RISING);
  attachInterrupt(digitalPinToInterrupt(button3pin), button3, RISING);
  attachInterrupt(digitalPinToInterrupt(enablebuttonpin), enableButton, RISING);
  //change address of encoder
  //twist.changeAddress(0x1A);


  if (twist.begin(Wire, 0x3E) == false)
  {
    Serial.println("Twist does not appear to be connected. Please check wiring. Freezing...");
    while (1);
  }
  if (twist2.begin(Wire, 0x3F) == false)
  {
    Serial.println("Twist2 does not appear to be connected. Please check wiring. Freezing...");
    while (1);
  }
}

void loop()
{


  if (process) {
      process = false; //reset the process
      Serial.println("delimiter"); //print the array on serial monitor 
      indx= 0; //reset button to zero
      Serial.println(receivedbuffer[0] + (receivedbuffer[1] << 8));
      sendbuffer[0] = voltage;
      sendbuffer[1] = (voltage >> 8); 
      sendbuffer[2] = current;
      sendbuffer[3] = (current >> 8);
      sendbuffer[4] = ((buttonMode) << 3) + (enableButtonState << 6);
      sendbuffer[5] = 0;

      Serial.println();
      Serial.println(sendbuffer[0]);
      Serial.println(sendbuffer[1]);
      Serial.println(sendbuffer[2]);
      Serial.println(sendbuffer[3]);
      Serial.println(sendbuffer[4], BIN);
      Serial.println(sendbuffer[5]);
  }

  for(int i = 0; i < LED_COUNT; i++)
  {
      strip.setPixelColor(i, 0, 69, 0, 0);
      if(enableButtonState)
      {
          strip.setPixelColor(i, 0, 3, 120, 0);
      }
  }
  

  // Magical sprintf creates a string for us to send to the s7s.
  //  The %4d option creates a 4-digit integer.
  strip.show();


  if (buttonMode == 1)
  {
    twist2.setColor(255, 0 , 0);
    twist.setColor(0, 0, 255);
  }
  else if (buttonMode == 2)
  {
    twist.setColor(255, 0, 0);
    twist2.setColor(0, 0, 255);
  }
  else if (buttonMode == 3)
  {
    twist.setColor(0, 0, 255);
    twist2.setColor(0, 0, 255);
  }
  else{
    buttonMode = 3;
  }
  sprintf(tempString, "%4d", current);
  current += twist.getDiff();
  voltage += twist2.getDiff();

  //Serial.print("Count: ");
  //Serial.print(twist.getCount());

  // This will output the tempString to the S7S
  if(!enableButtonState)
  {
    sprintf(tempString, "%4d", current);
    s7sSendStringI2C(tempString);
    sprintf(tempString, "%4d", voltage);
    s7sSendStringI2C_2(tempString);
  }
  else
  {
    sprintf(tempString, "%4d", (receivedbuffer[2] + (receivedbuffer[3] << 8)));
    s7sSendStringI2C(tempString);
    sprintf(tempString, "%4d", (receivedbuffer[0] + (receivedbuffer[1] << 8) ));
    s7sSendStringI2C_2(tempString);
  }
  

  // This will output the tempString to the S7S
  

  /* (twist.isPressed())
  {

    byte red = random(0, 256);
    byte green = random(0, 256);
    byte blue = random(0, 256);

    twist.setColor(red, green, blue); //Randomly set the Red, Green, and Blue LED brightnesses


  }*/


  // Print the decimal at the proper spot
  if (current < 10000)
    setDecimalsI2C(0b00000100);  // Sets digit 3 decimal on
  else
    setDecimalsI2C(0b00001000);

  if (voltage < 10000)
    setDecimalsI2C_2(0b00000100);  // Sets digit 3 decimal on
  else
    setDecimalsI2C_2(0b00001000);

  //delay(100);  //test interval  
}
