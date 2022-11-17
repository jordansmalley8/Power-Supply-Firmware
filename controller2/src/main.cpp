#include <Arduino.h>
#include <Smoothed.h>

#include <Arduino.h>
#include <SPI.h>
#include <PID_v2.h>         //PID library

#define DATAOUT 51//COPI
#define DATAIN  50//CIPO
#define SPICLOCK  52//sck
#define CHIPSELECT3 8//cs 
#define CHIPSELECT2 7
#define CHIPSELECT1 6
#define CHIPSELECT4 2

#define PFC1_ENA 10
#define PFC2_ENA 11
#define PFC3_ENA 12

#define PFC1_LOAD 42
#define PFC2_LOAD 40 
#define PFC3_LOAD 39

#define PFC1_ACGOOD 36
#define PFC2_ACGOOD 34 
#define PFC3_ACGOOD 32


#define VD1 A1 //input to PID
#define VD2 A2
#define VD3 A3

Smoothed <int> voltage1filter;
Smoothed <int> voltage2filter;
Smoothed <int> voltage3filter;

int iset = 0; // iset desired current
int vset = 0; // vset desired voltage

int voltage1 = 0; // Measured voltage from HE 1 
int voltage2 = 0; // Measured voltage from HE 2 
int voltage3 = 0; // Measured voltage from HE 3 

double setPoint;

double kp = .1; 
double ki = 1.4;
double kd = 0;

double PIDin1, PIDout1;
PID PID1(&PIDin1, &PIDout1, &setPoint, kp, ki, kd, DIRECT);

double PIDin2, PIDout2;
PID PID2(&PIDin2, &PIDout2, &setPoint, kp, ki, kd, DIRECT);

double PIDin3, PIDout3;
PID PID3(&PIDin3, &PIDout3, &setPoint, kp, ki, kd, DIRECT);

SPISettings spisettings(500000, MSBFIRST, SPI_MODE1);

int SPIDelay = 1;
int SPIDelay2 = 1;
int SPIDelay3 = 10;
int SPIDelayM = 500;
boolean enabled = 0;

int state;
int enable;

void initializeDACs()
{
  digitalWrite(CHIPSELECT4, HIGH);

  digitalWrite(CHIPSELECT1, LOW);
  digitalWrite(CHIPSELECT2, LOW);
  digitalWrite(CHIPSELECT3, LOW);
  Serial.println(SPI.transfer16(0b0101101000000000)); // Enable internal reference 
  delay(SPIDelay);
  digitalWrite(CHIPSELECT1, HIGH);
  digitalWrite(CHIPSELECT2, HIGH);
  digitalWrite(CHIPSELECT3, HIGH);
  


  digitalWrite(CHIPSELECT1, LOW);
  digitalWrite(CHIPSELECT2, LOW);
  digitalWrite(CHIPSELECT3, LOW);
  Serial.println(SPI.transfer16(0b0010100000000111)); // Configure pins 0, 1, 2 as DAC
  delay(SPIDelay);
  digitalWrite(CHIPSELECT1, HIGH);
  digitalWrite(CHIPSELECT2, HIGH);
  digitalWrite(CHIPSELECT3, HIGH);
  digitalWrite(CHIPSELECT4, HIGH);
} 

void writeDAC1(int write)
{
  digitalWrite(CHIPSELECT1, LOW);
  //digitalWrite(CHIPSELECT4, LOW);
  (SPI.transfer16(((0b1001 << 12) + ((int((((write))))))))); 
  delayMicroseconds(SPIDelayM);
  digitalWrite(CHIPSELECT1, HIGH);
}
                          // 4095 => 0V , 0 => 60V
void writeDAC2(int write) // 0 to 4095
{
  digitalWrite(CHIPSELECT2, LOW);
  //digitalWrite(CHIPSELECT4, LOW);
  (SPI.transfer16(((0b1001 << 12) + ((int((((write))))))))); 
  delayMicroseconds(SPIDelayM);
  digitalWrite(CHIPSELECT2, HIGH);
}

void writeDAC3(int write)
{
  digitalWrite(CHIPSELECT3, LOW);
  (SPI.transfer16(((0b1001 << 12) + ((int((((write))))))))); 
  delayMicroseconds(SPIDelayM);
  digitalWrite(CHIPSELECT3, HIGH);
}

void setup() {
  pinMode(DATAOUT, OUTPUT);
  pinMode(DATAIN, INPUT);
  pinMode(SPICLOCK, OUTPUT);
  pinMode(CHIPSELECT1, OUTPUT);
  pinMode(CHIPSELECT2, OUTPUT);
  pinMode(CHIPSELECT3, OUTPUT);
  pinMode(CHIPSELECT4, OUTPUT);
  digitalWrite(CHIPSELECT4, HIGH);

  pinMode(A8, INPUT);
  pinMode(A6, INPUT);
  pinMode(A7, INPUT);
  
  voltage1filter.begin(SMOOTHED_EXPONENTIAL, 8);
  voltage2filter.begin(SMOOTHED_EXPONENTIAL, 8);
  voltage3filter.begin(SMOOTHED_EXPONENTIAL, 8);

  voltage1filter.clear();
  voltage2filter.clear();
  voltage3filter.clear();
  //pinMode(5, OUTPUT);

  //PID stuff
  setPoint = 0;                   //variable for setpoint
  PIDin1 = 0;     //reads the input from output of LPF
  PID1.SetMode(AUTOMATIC);         //sets mode of PID
  PIDin2 = 0;     //reads the input from output of LPF
  PID2.SetMode(AUTOMATIC);         //sets mode of PID
  PIDin3 = 0;     //reads the input from output of LPF
  PID3.SetMode(AUTOMATIC);         //sets mode of PID
  
  PID1.SetSampleTime(70);
  PID2.SetSampleTime(70);
  PID3.SetSampleTime(70);


  // put your setup code here, to run once:
  
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(2);
  SPI.begin();
  initializeDACs();

  Serial.begin(9600);
  Serial3.begin(38400);
}

void loop() {
  //Serial.println(analogRead(VD1));

voltage1filter.add(analogRead(VD1));
voltage2filter.add(analogRead(VD2));
voltage3filter.add(analogRead(VD3));

  //Serial.println(voltage1filter.get());
  /*
  voltage1 = analogRead(VD1);
  voltage2 = (analogRead(VD2) * 2) - (voltage1);
  voltage3 = (analogRead(VD3) * 3) - voltage1 - voltage2;  
  */
 /*
voltage1 = analogRead(VD1);
voltage2 = analogRead(VD2);
voltage3 = analogRead(VD3);
*/ 
voltage1 = analogRead(VD1);
voltage2 = (analogRead(VD2) * 2) - (voltage1);
voltage3 = (analogRead(VD3) * 3) - voltage1 - voltage2;  
voltage1 = voltage1filter.get();
voltage2 = voltage2filter.get();
voltage3 = voltage3filter.get();

  if(enable==1)
  {
    setPoint = double(vset*(255/1800.0));
  }
  else{
    setPoint = 0;
  }

  //setPoint = 127;

  PIDin1 = double(voltage1) / 4.0;
  PIDin2 = double(voltage2) / 4.0;
  PIDin3 = double(voltage3) / 4.0;

  
  // Calculate PID loops
  PID1.Compute();
  PID2.Compute();
  PID3.Compute();
  
 
  writeDAC1(4095 - int(PIDout1 * (4095 / 257)));
  writeDAC2(4095 - int(PIDout2 * (4095 / 257)));
  writeDAC3(4095 - int(PIDout3 * (4095 / 257)));

/*
  writeDAC1(4095 - int(2048));
  writeDAC2(4095 - int(2048));
  writeDAC3(4095 - int(2048));
*/
  /*
  Serial.println();
  Serial.println(analogRead(A1));
  Serial.println(analogRead(A2));


  Serial.println(analogRead(A3));
  Serial.println();
  Serial.println(PIDin1);
  Serial.println(PIDout1);
  Serial.println();
  */

  int current = analogRead(A4);

  double voltage = double(double(voltage1 + voltage2 + voltage3) * (1800.0 / 3072.0));
  //int current = 221;  


  Serial3.print(voltage);
  Serial3.print("\t");
  Serial3.print(current);
  Serial3.print("\t");
  Serial3.print(state);
  Serial3.print("\t");
  Serial3.print(enable);
  Serial3.print("\n");

  while(Serial3.available())
  {
    vset = (Serial3.readStringUntil('\t')).toInt();
    iset = (Serial3.readStringUntil('\t')).toInt();
    state = (Serial3.readStringUntil('\t')).toInt();
    enable = (Serial3.readStringUntil('\n')).toInt();
  }
  /*
  Serial.println("VSET:");
  Serial.println(vset);
  Serial.println(iset);
  Serial.println(state);
  Serial.println(enable);
  */
  
  
  

  // put your main code here, to run repeatedly:
}