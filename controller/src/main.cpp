#include <Arduino.h>
#include <SPI.h>
#include <PID_v2.h>         //PID library

#define DATAOUT 51    //COPI
#define DATAIN  50    //CIPO
#define SPICLOCK  52  //sck

// SPI CS Pins. Pull pin low, SPI.transfer(), and then pull pin high to send a message
#define CHIPSELECT3 8 // DAC 3
#define CHIPSELECT2 7 // DAC 2 
#define CHIPSELECT1 6 // DAC 1
#define CHIPSELECT4 2 // Front Panel

#define pidInPIN1 A5  // PID emulator reading pins (temporary)
#define pidInPIN2 A7
#define pidInPIN3 A6
#define pidOutPIN 23  

unsigned int vset = 0;
unsigned int iset = 0;
unsigned int voltageReading = 0;
unsigned int currentReading = 0;

//PID
double setPoint1;
double PIDinput1, PIDoutput1, writetoPWM1;
int writetoAnalog1;
double kp1 = .7;
double ki1 = .8;
double kd1 = 0;
PID myPID1(&PIDinput1, &PIDoutput1, &setPoint1, kp1, ki1, kd1, DIRECT);


double setPoint2;
double PIDinput2, PIDoutput2, writetoPWM2;
int writetoAnalog2;
double kp2 = kp1;
double ki2 = ki1;
double kd2 = kd1;
PID myPID2(&PIDinput2, &PIDoutput2, &setPoint2, kp2, ki2, kd2, DIRECT);

double setPoint3;
double PIDinput3, PIDoutput3, writetoPWM3;
int writetoAnalog3;
double kp3 = kp1;
double ki3 = ki1;
double kd3 = kd1;
PID myPID3(&PIDinput3, &PIDoutput3, &setPoint3, kp3, ki3, kd3, DIRECT);

unsigned int pidWrite = 0;
byte state = 0;

SPISettings spisettings(500000, MSBFIRST, SPI_MODE1);


// ---------AD5592R----------------------
// Configuration to enable internal 2.5V reference: pg 39 of datasheet
// 0b0101101000000000
// Configuration to set pins as a DAC: pg 30 of datasheet
// 0b0010100000000011
// ---------------------------------------------------------------
// DAC Write CH0
// 0b1000 + [12 bit data]
// DAC write CH1
// 0b1001 + [12 bit data]
// 0 to 4095
// ---------------------------------------------------------------


void setup() {

  pinMode(DATAOUT, OUTPUT);
  pinMode(DATAIN, INPUT);
  pinMode(SPICLOCK, OUTPUT);
  pinMode(CHIPSELECT1, OUTPUT);
  pinMode(CHIPSELECT2, OUTPUT);
  pinMode(CHIPSELECT3, OUTPUT);
  pinMode(CHIPSELECT4, OUTPUT);
  digitalWrite(CHIPSELECT4, HIGH);

  pinMode(pidInPIN1, INPUT);
  pinMode(pidInPIN2, INPUT);
  pinMode(pidInPIN3, INPUT);
  //pinMode(5, OUTPUT);

  //PID 
  PIDinput1 = 0;                     //input to PID (measured voltage)
  setPoint1 = 1.2;                   //variable for setpoint
  myPID1.SetMode(AUTOMATIC);         //sets mode of PID

  PIDinput2 = 0;                     //input to PID (measured voltage)
  setPoint2 = 1.4;                   //variable for setpoint
  myPID2.SetMode(AUTOMATIC);         //sets mode of PID

  PIDinput3 = 0;                     //input to PID (measured voltage)
  setPoint3 = 1.0;                   //variable for setpoint
  myPID3.SetMode(AUTOMATIC);         //sets mode of PID


  Serial.begin(9600);
  delay(1000);
  Serial.println("12345678");
  //SPI.beginTransaction(spisettings); 
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE1);
  SPI.setClockDivider(64);
  //SPI.beginTransaction(spisettings);
  SPI.begin();
  Serial.println("SPI BEGUN transcaation");

  digitalWrite(CHIPSELECT4, HIGH);

  
  // Send messages to configure DACs (enable reference voltage, configure DAC pins)
  
  digitalWrite(CHIPSELECT1, LOW);
  digitalWrite(CHIPSELECT2, LOW);
  digitalWrite(CHIPSELECT3, LOW);
  Serial.println(SPI.transfer16(0b0101101000000000)); // Enable internal reference 
  delay(5);
  digitalWrite(CHIPSELECT1, HIGH);
  digitalWrite(CHIPSELECT2, HIGH);
  digitalWrite(CHIPSELECT3, HIGH);
  

  delay(5);

  digitalWrite(CHIPSELECT1, LOW);
  digitalWrite(CHIPSELECT2, LOW);
  digitalWrite(CHIPSELECT3, LOW);
  Serial.println(SPI.transfer16(0b0010100000000111)); // Configure pins 0, 1, 2 as DAC
  delay(5);
  digitalWrite(CHIPSELECT1, HIGH);
  digitalWrite(CHIPSELECT2, HIGH);
  digitalWrite(CHIPSELECT3, HIGH);
  digitalWrite(CHIPSELECT4, HIGH);
} 

void loop() {
  
  if(state >> 5 == 1)
  {
    setPoint1 = (vset * (5.0 / 1800.0));
    setPoint2 = setPoint1;
    setPoint3 = setPoint1;
  }
  else{
    setPoint1 = 0;
    setPoint2 = setPoint1;
    setPoint3 = setPoint1;
  }


  PIDinput1 = analogRead(5) * (5.0/1023.0);        //converts 0-1023 to 0-5V
  myPID1.Compute();
  writetoAnalog1 = PIDoutput1 * int((4096.0/5.0));    //scale the PIDoutput to range between 0 and 4096

  PIDinput2 = analogRead(7) * (5.0/1023.0);        //converts 0-1023 to 0-5V
  myPID2.Compute();
  writetoAnalog2 = PIDoutput2 * int((4096.0/5.0));    //scale the PIDoutput to range between 0 and 4096
  
  PIDinput3 = analogRead(6) * (5.0/1023.0);        //converts 0-1023 to 0-5V
  myPID3.Compute();
  writetoAnalog3 = PIDoutput3 * int((4096.0/5.0));    //scale the PIDoutput to range between 0 and 4096

  /*
  if(writetoAnalog > 4095)
  {
    writetoAnalog = 4095;
  }
  if(writetoAnalog < 0)
  {
    writetoAnalog = 0;
  }*/

  digitalWrite(5, HIGH);
  digitalWrite(2, HIGH);
  //Serial.println(analogRead(A13));

  // Send voltages to DACs to drive HE Vset pins
  
  digitalWrite(CHIPSELECT1, LOW);
  delay(5);
  Serial.println(SPI.transfer16(((0b1001 << 12) + ((int((((writetoAnalog1))))))))); 
  delay(10);

  digitalWrite(CHIPSELECT1, HIGH);

  digitalWrite(CHIPSELECT2, LOW);
  delay(5);
  Serial.println(SPI.transfer16(((0b1001 << 12) + ((int((((writetoAnalog2))))))))); 
  delay(10);
  digitalWrite(CHIPSELECT2, HIGH);

  digitalWrite(CHIPSELECT3, LOW);
  delay(5);
  Serial.println(SPI.transfer16(((0b1001 << 12) + ((int((((writetoAnalog3))))))))); 
  delay(10);
  digitalWrite(CHIPSELECT3, HIGH);
  
  Serial.println();
  Serial.println(setPoint1);
  Serial.println(writetoAnalog1);
  Serial.println(analogRead(5));
  Serial.println();

  Serial.println(writetoAnalog2);
  Serial.println(analogRead(7));
  Serial.println();

  Serial.println(writetoAnalog3);
  Serial.println(analogRead(6));
  Serial.println();
  Serial.println();
  
  voltageReading = (analogRead(6) + analogRead(7) + analogRead(5)) * (1800 / 3072.0);
  currentReading = 0;
  
  // Send/ receive SPI message:
  // Byte 1-2: voltage reading
  // Byte 3-4: current reading
  // Byte 5: state
  // Byte 6: delimiter
  
  delay(5);
  digitalWrite(CHIPSELECT4, LOW);
  delay(1);
  byte b1 = (SPI.transfer(voltageReading));
  delay(15);
  digitalWrite(CHIPSELECT4, HIGH);
  delay(15);

  digitalWrite(CHIPSELECT4, LOW);
  delay(1);
  byte b2 = (SPI.transfer(voltageReading >> 8));
  delay(15);
  digitalWrite(CHIPSELECT4, HIGH);
  delay(15);

  digitalWrite(CHIPSELECT4, LOW);
  delay(1);
  byte b3 = (SPI.transfer(currentReading));
  delay(15);
  digitalWrite(CHIPSELECT4, HIGH);
  delay(15);

  digitalWrite(CHIPSELECT4, LOW);
  delay(1);
  byte b4 = (SPI.transfer(currentReading >> 8));
  delay(15);
  digitalWrite(CHIPSELECT4, HIGH);
  delay(15);

  digitalWrite(CHIPSELECT4, LOW);
  delay(1);
  byte b5 = SPI.transfer(state);
  delay(15);
  digitalWrite(CHIPSELECT4, HIGH);
  delay(45);

  digitalWrite(CHIPSELECT4, LOW);
  delay(1);
  byte b6 = SPI.transfer('\n');
  delay(15);
  digitalWrite(CHIPSELECT4, HIGH);
  delay(45);
  

  Serial.println();
  Serial.println(b5);
  Serial.println(b6);
  
  //Serial.println(vset);
  //Serial.println(iset);
  //Serial.println(state);
  
  Serial.println();

  // Bitshift entire packet 1 bit to the left
  
  b1 = (b1 << 1) + (b2 >> 7);
  b2 = (b2 << 1) + (b3 >> 7);
  b3 = (b3 << 1) + (b4 >> 7);
  b4 = (b4 << 1) + (b5 >> 7);
  b5 = (b5 << 1) + (b6 >> 7);

  // bitshift to find desired voltage and current (Vset, iset)
  
  vset = b2 + (b3 << 8);
  iset = b4 + (b5 << 8);
  state = b6;

  Serial.println(vset);
  Serial.println(iset);
  Serial.println(state); 
  Serial.println(state >> 5);
  

  delay(15);
  
}
