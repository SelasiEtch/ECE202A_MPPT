#include "mbed.h"
#include <PID_v1.h>
#include <Wire.h>
#include <Adafruit_AMG88xx.h>

#define PWM_PIN               12
#define DIRECTION_PIN        11
#define PWM_FREQUENCY         31000
#define ADC_RES               1024
#define ADC_REF               3.3
//#define FB_RATIO_VOLT         0.25
#define FB_RATIO_VOLT         0.5
#define FB_RATIO_CURR         0.8
// A0 is current Sensing Pin
// A3 is Output Voltage Sensing
// A2 is input voltage sensing
// A1 is pot sensing pin 
uint8_t sense[4];
float irr_est;
float POT_VOLT = 0;
float SENSE_VOLT_PV = 0;
float SENSE_VOLT_OUT = 0;
float sense_v_loop = 0;
float sense_i_loop = 0;
float DC = 0;
float Imp = 0;
float Vmp = 0;
float Pmp = 0;


double Setpoint, Input, Output;
double Kp=13, Ki=200, Kd=0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
mbed::PwmOut pwmPin(digitalPinToPinName(PWM_PIN));

Adafruit_AMG88xx Temp_Sensor;
float pixels[AMG88xx_PIXEL_ARRAY_SIZE];
float SENSE_TEMP;

//#define CLOSED_LOOP

void setup() 
{
  Temp_Setup();

  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIRECTION_PIN, OUTPUT);
  pinMode(A0,INPUT);
  pinMode(A1,INPUT);
  pinMode(A3, INPUT);

  digitalWrite(DIRECTION_PIN, HIGH);
  pwmPin.period( 1.0 / PWM_FREQUENCY );
  pwmPin.write( 0.1 );
  Serial.begin(115200);
  myPID.SetMode(AUTOMATIC);

  Setpoint = 3;
}

void loop() 
{
  SenseOutput_Voltage();
  SenseOutput_Current();
  Process_Temp_Pixels_Average();
  
  sense[0] = (uint8_t)SENSE_VOLT;
  sense[1] = (uint8_t)SENSE_CURR;
  sense[2] = (uint8_t)SENSE_TEMP;
  sense[3] = (uint8_t)Output;


  Serial.write((uint8_t*)sense,sizeof(sense));
  Serial.write("\r\n");

#ifdef CLOSED_LOOP
  while(Serial.available() == 0);

  if (Serial.available() > 0) 
  {
    irr_est = (8* Serial.read());
    Imp = calculate_Imp(irr_est, SENSE_TEMP);
    Vmp = calculate_Vmp(SENSE_TEMP);
    Pmp = Vmp * Imp;
    Setpoint = Imp;
    UpdatePID();
    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(LED_BUILTIN, LOW);
  }
#endif

#ifndef CLOSED_LOOP
  UpdatePot_DC();
#endif

  delay(100);
}

void Temp_Setup()
{
  bool status;
  status = Temp_Sensor.begin();
  if (!status) 
  {
  while (1); 
  }
}

void Process_Temp_Pixels_Average()
{
  //Update Pixel Array
  SENSE_TEMP = 0;
  Temp_Sensor.readPixels(pixels);
  for(int i=1; i<=AMG88xx_PIXEL_ARRAY_SIZE; i++){
    SENSE_TEMP += pixels[i-1];
  }
  // Average Temp of Pixels in view
  SENSE_TEMP = SENSE_TEMP / (AMG88xx_PIXEL_ARRAY_SIZE);
}

void SenseOutput_Voltage()
{
  SENSE_VOLT_OUT = analogRead(A3);
  SENSE_VOLT_PV = analogRead(A2);
  sense_v_loop = SENSE_VOLT_PV;
  SENSE_VOLT_PV = SENSE_VOLT_PV / 4;
  SENSE_VOLT_OUT = SENVE_VOLT_OUT / 4; 
  sense_v_loop = (sense_v_loop / ADC_RES)*(ADC_REF);
  sense_v_loop = (sense_v_loop / FB_RATIO_VOLT);
}

void SenseOutput_Current()
{
  SENSE_CURR = analogRead(A0);
  sense_i_loop = SENSE_CURR;
  SENSE_CURR = SENSE_CURR / 4;
  sense_i_loop = (sense_i_loop / ADC_RES)*(ADC_REF);
  sense_i_loop = sense_i_loop / FB_RATIO_CURR;
}

void UpdatePot_DC()
{
  POT_VOLT = analogRead(A1);
  DC = (POT_VOLT / 1024);
  pwmPin.write(DC);
}

void UpdatePID()
{
  Input = sense_i_loop; // Loop closed on current measurement
  myPID.Compute();
  DC = (Output / 255);
  pwmPin.write(DC);
}

float calculate_Imp(double irr_est, double Temp)
{
  float Imp_ref = 0.35;
  float Gref = 1000;
  float alpha_T = 0.046;
  float Tref = 25;

  float imp = Imp_ref * (irr_est/Gref)*(1+(alpha_T*(Temp-Tref)));
  return imp;
}

float calculate_Vmp(double Temp)
{
  float beta_T = -0.0118;
  float Tref = 25;
  float Vmp_ref = 6;
  float vmp = Vmp_ref + (beta_T*(Temp-Tref));
  return vmp;
}


