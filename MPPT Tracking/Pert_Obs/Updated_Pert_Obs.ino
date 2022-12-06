// Perturb and Observe Function code to be used with PID

#include "mbed.h"
#include <PID_v1.h>
#include <Wire.h>
#include <Adafruit_AMG88xx.h>

#define PWM_PIN               12
#define DIRECTION_PIN        11
#define PWM_FREQUENCY         31000
#define ADC_RES               1024
#define ADC_REF               3.3
#define FB_RATIO_VOLT         0.25
#define FB_RATIO_CURR         1.65
// A0 is current Sensing Pin
// A3 is Output Voltage Sensing
// A1 is pot sensing pin 
uint8_t sense[3];
float irr_est;

float duty = 0;
float duty_2 = 0;

float POT_VOLT = 0;

float SENSE_VOLT = 0;
float sense_v_loop = 0;
float sense_i_loop = 0;
float SENSE_CURR = 0;
float SENSE_POWER = 0;
float DELTA_POWER = 0;
float VOLT_PREV = 0;
float POWER_PREV = 0;
float DELTA_VOLT = 0;

float DC = 0;

double Setpoint, Input, Output;
double Kp=13, Ki=100, Kd=0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
mbed::PwmOut pwmPin(digitalPinToPinName(PWM_PIN));

Adafruit_AMG88xx Temp_Sensor;
float pixels[AMG88xx_PIXEL_ARRAY_SIZE];
float SENSE_TEMP;

// Create thread
Thread t1;

void Pert_Obs() // Thread 1
{
  SenseOutput_Voltage();
  SenseOutput_Current();

  SENSE_POWER = SENSE_VOLT * SENSE_CURR;
  DELTA_POWER = SENSE_POWER - POWER_PREV;

  DELTA_VOLT = SENSE_VOLT - VOLT_PREV;
  if(DELTA_POWER !=0 || DELTA_VOLT != 0)
  {
    if(DELTA_POWER > 0)
    {
      if(DELTA_VOLT < 0)
      {
        duty++;
      }
      else
      {
        duty--;
      }
    }
    else
    {
      if(DELTA_VOLT < 0)
      {
        duty--;
      }
      else
      {
        duty++;
      }
    }
  }
  else
  {
    duty = duty;
  }
  duty_2 = duty / 100;
  
  pwmPin.write(duty_2);

}

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:
     t1.start(callback(Pert_Obs)); // start thread one with callback to Pert_Obs function
}
