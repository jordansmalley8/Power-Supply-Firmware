// Front panel
// This runs on the front panel. Reads from 4 buttons and 2 encoders and controls both 7-segment displays and LED lights
// This also talks to the other microcontroller through SPI and sends a 7 byte packet back and forth 

#include <Arduino.h>
#include <Wire.h> 
#include <SPI.h>                        

#include "SparkFun_Qwiic_Twist_Arduino_Library.h"
#include "Adafruit_NeoPixel.h"
#include <header.h>

#define LED_PIN 36
#define LED_COUNT 20
#define button1pin 2
#define button2pin 3 
#define button3pin 18
#define enablebuttonpin 19

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_BRWG + NEO_KHZ800);

ISR (SPI_STC_vect) // SPI interrupt routine. This triggers every time a SPI byte is received. 
{ 
  // Each SPI message is made up of 7 bytes. The message comes in 1 byte at a time, so this gets triggered 7 times for 1 message
  
  byte c = SPDR; // read byte from SPI Data Register. 

  SPDR = sendbuffer[indx]; // This is the register for the return message. indx keeps track of what part of the message you're in

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

  clearDisplayI2C();  // Clears display, resets cursor
  s7sSendStringI2C("-HI-");
  s7sSendStringI2C_2("-HI-");
  setDecimalsI2C(0b111111);  // Turn on all decimals, colon, apos
  setDecimalsI2C_2(0b111111);  // Turn on all decimals, colon, apos

  Serial.println("3");

  // Flash brightness values at the beginning
  setBrightnessI2C(0);  // Lowest brightness
  delay(500);
  setBrightnessI2C(255);  // High brightness
  delay(500);

  setBrightnessI2C_2(0);  // Lowest brightness
  delay(500);
  setBrightnessI2C_2(255);  // High brightness
  delay(500);

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
  if (process) { // SPI If at the end of a packet
      // This runs at the end of every packet, update the next packet to be sent 
      process = false;
      Serial.println("delimiter");
      indx= 0;
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

  // Set led strip color
  for(int i = 0; i < LED_COUNT; i++) 
  {
      strip.setPixelColor(i, 0, 69, 0, 0);
      if(enableButtonState)
      {
          strip.setPixelColor(i, 0, 3, 120, 0);
      }
  }
  strip.show();


  if (buttonMode == 1) // Voltage Mode
  {
    twist2.setColor(255, 0 , 0);
    twist.setColor(0, 0, 255);
  }
  else if (buttonMode == 2) // Current Mode
  {
    twist.setColor(255, 0, 0);
    twist2.setColor(0, 0, 255);
  }
  else if (buttonMode == 3) // Remote mode
  {
    twist.setColor(0, 0, 255);
    twist2.setColor(0, 0, 255);
  }
  else{ // Set button mode to 3 incase something weird happened
    buttonMode = 3;
  }

  // Update set voltage and current from the encoders
  current += twist.getDiff();
  voltage += twist2.getDiff();

  sprintf(tempString, "%4d", current);
  // Update both 7-segment displays to show voltage/ current either measured voltage current or desired voltage/ current depending on mode
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
