#include "mbed.h"
#include <PID_v1.h>
#include <Wire.h>
#include <Adafruit_AMG88xx.h>

#define PWM_PIN               12
#define DIRECTION_PIN        11
#define PWM_FREQUENCY         31000
#define ADC_RES               1024
#define ADC_REF               3.3
#define FB_RATIO_VOLT         0.16
#define FB_RATIO_CURR         0.8
// A0 is current Sensing Pin
// A3 is Output Voltage Sensing
// A2 is input voltage sensing
// A1 is pot sensing pin 

uint8_t sense[5];
float irr_est;
float POT_VOLT = 0;
float SENSE_VOLT_PV = 0;
float SENSE_VOLT_OUT = 0;
float SENSE_CURR = 0;
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

#define CLOSED_LOOP

void setup() 
{
  //Setup for AMG8833 IR Temp Sensor
  Temp_Setup();

// Setting Sensing Pins as Inputs and Control Pins as Outputs
  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIRECTION_PIN, OUTPUT);
  pinMode(A0,INPUT);
  pinMode(A1,INPUT);
  pinMode(A3, INPUT);


// Direction Pin dictates transistors to operate in Motor Driver
  digitalWrite(DIRECTION_PIN, HIGH);

// Initialize 31kHz PWM with 10% duty cycle ratio
  pwmPin.period( 1.0 / PWM_FREQUENCY );
  pwmPin.write( 0.1 );
  Serial.begin(115200);
  myPID.SetMode(AUTOMATIC);

// Initial PID Setpoint
  Setpoint = 3;
}

void loop() 
{
// Sensing Parameters are updated (PV Voltage, PV Current, and PV Temperature)
  SenseOutput_Voltage();
  SenseOutput_Current();
  Process_Temp_Pixels_Average();

// Packing data to be sent to MATLAB into array
  sense[0] = (uint8_t)SENSE_VOLT_PV;
  sense[1] = (uint8_t)SENSE_CURR;
  sense[2] = (uint8_t)SENSE_TEMP;
  sense[3] = (uint8_t)Output;
  sense[4] = (uint8_t)SENSE_VOLT_OUT;

// Open Loop operation is observed by manually adjusting Buck Converter with potentiometer
#ifndef CLOSED_LOOP
  sense[3] = (uint8_t)(DC * (225));
  UpdatePot_DC();
#endif


// Sending serial data over to NN Simulink Model
  Serial.write((uint8_t*)sense,sizeof(sense));
  Serial.write("\r\n");

// Reading Irradiance Estimate back from NN Simulink Model and adjusting PID Setpoint Accordingly
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

  delay(100);
}

// Setup function for AMG8833 IR Temp Sensor
void Temp_Setup()
{
  bool status;
  status = Temp_Sensor.begin();
  if (!status) 
  {
  while (1); 
  }
}

// Processing function for AMG8833 IR Temp Sensor, averages out temperatures of all pixels in the camera feed
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

// Sensing PV Input Voltage and Buck Converter Output Voltage + Neccesary conditioning for transfer to Simulink
void SenseOutput_Voltage()
{
  SENSE_VOLT_OUT = analogRead(A3);
  SENSE_VOLT_PV = analogRead(A2);
  sense_v_loop = SENSE_VOLT_PV;
  SENSE_VOLT_PV = SENSE_VOLT_PV / 4;
  SENSE_VOLT_OUT = SENSE_VOLT_OUT / 4; 
  sense_v_loop = (sense_v_loop / ADC_RES)*(ADC_REF);
  sense_v_loop = (sense_v_loop / FB_RATIO_VOLT);
}

// Sensing PV Input Current + Neccesary conditioning for transfer to Simulink
void SenseOutput_Current()
{
  SENSE_CURR = analogRead(A0);
  sense_i_loop = SENSE_CURR;
  SENSE_CURR = SENSE_CURR / 4;
  sense_i_loop = (sense_i_loop / ADC_RES)*(ADC_REF);
  sense_i_loop = sense_i_loop / FB_RATIO_CURR;
}

// Open-loop function to adjust Buck Converter with potentiometer
void UpdatePot_DC()
{
  POT_VOLT = analogRead(A1);
  DC = (POT_VOLT / 1024);
  pwmPin.write(DC);
}

// Function to update converter duty cycle based on PID output
void UpdatePID()
{
  Input = sense_i_loop; // Loop closed on current measurement
  myPID.Compute();
  DC = (Output / 255);
  pwmPin.write(DC);
}

// Function to calculate Maximum Power Current with NN irradiance estimate and sensed temperature
float calculate_Imp(double irr_est, double Temp)
{
  float Imp_ref = 0.35;
  float Gref = 1000;
  float alpha_T = 0.046;
  float Tref = 25;

  float imp = Imp_ref * (irr_est/Gref)*(1+(alpha_T*(Temp-Tref)));
  return imp;
}

// Function to calculate Maximum Power Voltage with sensed temperature
float calculate_Vmp(double Temp)
{
  float beta_T = -0.0118;
  float Tref = 25;
  float Vmp_ref = 6;
  float vmp = Vmp_ref + (beta_T*(Temp-Tref));
  return vmp;
}


