#include "mbed.h"
#include <Wire.h>

#define PWM_PIN               12
#define DIRECTION_PIN        11
#define PWM_FREQUENCY         31000
#define ADC_RES               1024
#define ADC_REF               3.3
#define FB_RATIO_VOLT         0.16
#define FB_RATIO_CURR         0.8
#define TEST_PIXELS_AVG
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

mbed::PwmOut pwmPin(digitalPinToPinName(PWM_PIN));

void setup() 
{
  // put your setup code here, to run once:
  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIRECTION_PIN, OUTPUT);
  pinMode(A0,INPUT);
  pinMode(A1,INPUT);
  pinMode(A3, INPUT);
  digitalWrite(DIRECTION_PIN, HIGH);
  pwmPin.period( 1.0 / PWM_FREQUENCY );
  pwmPin.write( 0.5 );
  //pinMode(A0, OUTPUT);
  Serial.begin(115200);
  //myPID.SetMode(AUTOMATIC);
  duty = 50;
}

void loop() // 
{
  Pert_Obs();
  Serial.print("Sense Voltage: ");
  Serial.println(sense_v_loop);
  Serial.print("Sense Current: ");
  Serial.println(sense_i_loop);
  Serial.print("PV Power: ");
  Serial.println(SENSE_POWER);
  Serial.print("Duty Cycle: ");
  Serial.println(duty_2);
  
  
  delay(300);
}

void Pert_Obs()
{
  SenseOutput_Voltage();
  SenseOutput_Current();

  SENSE_POWER = sense_v_loop * sense_i_loop;
  DELTA_POWER = SENSE_POWER - POWER_PREV;

  DELTA_VOLT = SENSE_VOLT - VOLT_PREV;
  if(DELTA_POWER != 0 || DELTA_VOLT != 0)
  {
    if(DELTA_POWER > 0)
    {
      if(DELTA_VOLT < 0)
      {
        duty--;
        if(duty < 0)
        {
          duty = 0;
        }
      }
      else
      {
        duty++;
        if(duty > 100)
        {
          duty = 100;
        }
      }
    }
    else
    {
      if(DELTA_VOLT < 0)
      {
        duty++;
        if(duty > 100)
        {
          duty = 100;
        }
      }
      else
      {
        duty--;
        if(duty < 0)
        {
          duty = 0;
        }
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

void SenseOutput_Voltage()
{
  SENSE_VOLT = analogRead(A2);
  sense_v_loop = SENSE_VOLT;
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
