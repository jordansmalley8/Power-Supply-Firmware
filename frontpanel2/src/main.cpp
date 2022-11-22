#include <Arduino.h>
#include <Wire.h> 
#include <SPI.h>                        

#include "SparkFun_Qwiic_Twist_Arduino_Library.h"
#include "Adafruit_NeoPixel.h"
#include <header.h>

#define LED_PIN 36
#define LED_PIN2 38
#define LED_COUNT 20
#define button1pin 2 
#define button2pin 3 
#define button3pin 18
#define enablebuttonpin 19

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_BRWG + NEO_KHZ800); // left LED strip
Adafruit_NeoPixel strip2(LED_COUNT, 38, NEO_BRWG + NEO_KHZ800); // right LED strip

boolean displayUpdate = 1;
unsigned long lastDisplayUpdateTime = 0;

void setup() { // Setup. This method runs once when the arduino gets turned on. This is the 'Start-up sequence'
 
  Serial.begin(19200); // Serial port on the front panel arduino
  Serial3.begin(38400); // Serial between the two arduinos (front panel talking to controller)

  strip.begin(); // Turning on both led strips
  strip2.begin();

  pinMode(18, INPUT_PULLUP); 

  Serial.println("1");
  Wire.begin();  // Initialize hardware I2C pins
  Serial.println("2");
  
  clearDisplayI2C();  // Clears display, resets cursor
  s7sSendStringI2C("-HI-");
  s7sSendStringI2C_2("-HI-");
  setDecimalsI2C(0b111111);  // Turn on all decimals, colon, apos
  setDecimalsI2C_2(0b111111);  // Turn on all decimals, colon, apos


  attachInterrupt(digitalPinToInterrupt(button1pin), button1, RISING); // Attaching interrupts for buttons, the 3 buttons and enable button
  attachInterrupt(digitalPinToInterrupt(button2pin), button2, RISING);
  attachInterrupt(digitalPinToInterrupt(button3pin), button3, RISING);
  attachInterrupt(digitalPinToInterrupt(enablebuttonpin), enableButton, RISING);


  setBrightnessI2C(0);  // Lowest brightness
  setBrightnessI2C_2(0);  // Lowest brightness
  delay(500);
  setBrightnessI2C(255);  // High brightness
  setBrightnessI2C_2(255);  // High brightness
  delay(500);

  // Clear the display before jumping into loop
  clearDisplayI2C();  
  clearDisplayI2C_2();

  if (twist.begin(Wire, 0x3E) == false) // Two encoders
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

void loop() {

  // For loop to set the color of each led in each LED strip
  for(int i = 0; i < LED_COUNT; i++) 
  {
      strip.setPixelColor(i, 0, 49, 0, 0);
      strip2.setPixelColor(i, 0, 169, 0, 0);
      if(enableButtonState)
      {
          strip.setPixelColor(i, 0, 3, 120, 0);
          strip2.setPixelColor(i, 0, 3, 199, 0);
      }
      
  }

  strip.show(); // Show method for both led strips 
  strip2.show();
  

  // This whole block of code with Serial3 is the communication between the two microcontrollers 
  
  
  int state = 3; 
  boolean enable = enableButtonState;
  int voltage;
  int current;

  // Sending the vset and iset values to the other microcontroller
  Serial3.print(vset);
  Serial3.print("\t");
  Serial3.print(iset);
  Serial3.print("\t");
  Serial3.print(state);
  Serial3.print("\t");
  Serial3.print(enable);
  Serial3.print("\n");
  delay(5);

  // Receiving data from the other microcontroller. 
  while(Serial3.available()) 
  {
    voltage = (Serial3.readStringUntil('\t')).toInt(); // vset is the desired or commanded voltage. 'voltage' is the measured voltage. 
    current = (Serial3.readStringUntil('\t')).toInt(); // iset 
    state   = (Serial3.readStringUntil('\t')).toInt();
    int enable1 = (Serial3.readStringUntil('\n')).toInt();
  }
  
  // Print the read values 
  Serial.println(voltage);
  Serial.println(current);
  Serial.println(state);
  Serial.println(enable1);

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
  iset += twist.getDiff();
  vset += twist2.getDiff();

  //Setting up Remote mode
  if(buttonMode == 3)
  {
    Serial2.begin(9600);
   
    while(Serial2.available() == 0)
    {
    }
   
    // Get data from serial monitor
    Serial2.print("Enter in the current value: ");
    int iset = Serial2.parseInt();
    Serial2.println("");
    Serial2.print("Enter in the voltage value: ");
    int vset = Serial2.parseInt();
    Serial2.println("");
    Serial2.print("Enter in the 0 for current mode and 1 for voltage mode: ");
    int mode = Serial2.parseInt();
    Serial2.println("");
   
    // Sending the vset, iset, and mode values to the other microcontroller
    Serial2.print(vset);
    Serial2.print("\t");
    Serial2.print(iset);
    Serial2.print("\t");
    Serial2.print(mode);
    Serial2.print("\n");
    delay(5);   
    
  }
 
  sprintf(tempString, "%4d", current);
  // Update both 7-segment displays to show voltage/ current either measured voltage current or desired voltage/ current depending on mode
  
  if(enable==0)
  {
    if (1) // (millis() - lastDisplayUpdateTime) > 200
    {
      lastDisplayUpdateTime = millis();
      sprintf(tempString, "%4d", iset);
      s7sSendStringI2C(tempString);
      sprintf(tempString, "%4d", vset);
      s7sSendStringI2C_2(tempString);
    }
  }
  else
  {
    if (1) // (millis() - lastDisplayUpdateTime) > 30
    { 
      lastDisplayUpdateTime = millis();
      sprintf(tempString, "%4d", (current));
      s7sSendStringI2C(tempString);
      sprintf(tempString, "%4d", (voltage));
      s7sSendStringI2C_2(tempString);
    }
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
  
}
