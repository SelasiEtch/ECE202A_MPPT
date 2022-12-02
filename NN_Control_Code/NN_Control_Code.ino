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

float Imp = 0;
float Vmp = 0;
float Pmp = 0;


double Setpoint, Input, Output;
double Kp=13, Ki=200, Kd=0;
//double Kp=100, Ki=100, Kd=0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
mbed::PwmOut pwmPin(digitalPinToPinName(PWM_PIN));

Adafruit_AMG88xx Temp_Sensor;
float pixels[AMG88xx_PIXEL_ARRAY_SIZE];
float SENSE_TEMP;

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
  pwmPin.write( 0.1 );
  //pinMode(A0, OUTPUT);
  Serial.begin(115200);
  myPID.SetMode(AUTOMATIC);
  Temp_Setup();

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


  /*
  Serial.print("Sense Voltage: ");
  Serial.println(sense_v_loop);
  Serial.print("Sense Current: ");
  Serial.println(sense_i_loop);
  Serial.print("Sense Temp: ");
  Serial.println(SENSE_TEMP);
  Serial.print("Duty Cycle: ");
  Serial.println(DC);
  */

  
  //UpdatePot_DC();
  

  Serial.write((uint8_t*)sense,sizeof(sense));
  Serial.write("\r\n");


  while(Serial.available() == 0);

  if (Serial.available() > 0) 
  {
    irr_est = (8* Serial.read());
    Imp = calculate_Imp(irr_est, SENSE_TEMP);
    Vmp = calculate_Vmp(SENSE_TEMP);
    Pmp = Vmp * Imp;
    Setpoint = Vmp;
    UpdatePID();
    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(LED_BUILTIN, LOW);
  }

  delay(100);
}

void Pert_Obs()
{
  SenseOutput_Voltage();
  SenseOutput_Current();
  POWER_PREV = SENSE_POWER;

  SENSE_POWER = sense_v_loop * sense_i_loop;
  DELTA_POWER = SENSE_POWER - POWER_PREV;

  DELTA_VOLT = sense_v_loop - VOLT_PREV;
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

void Temp_Setup()
{
  bool status;
  status = Temp_Sensor.begin();
  if (!status) 
  {
  //Serial.println("Could not find a valid AMG88xx sensor, check wiring!");
  while (1); 
  }
}

void Process_Temp_Pixels_Average()
{
  //Update Pixel Array
  SENSE_TEMP = 0;
  Temp_Sensor.readPixels(pixels);
  for(int i=1; i<=AMG88xx_PIXEL_ARRAY_SIZE; i++){
    //pixels[i-1] = (pixels[i-1] * 1.8) + 32;
    SENSE_TEMP += pixels[i-1];
  }

  // Average Temp of Pixels in view
  SENSE_TEMP = SENSE_TEMP / (AMG88xx_PIXEL_ARRAY_SIZE);
}

void SenseOutput_Voltage()
{
  VOLT_PREV = SENSE_VOLT;

  SENSE_VOLT = analogRead(A3);
  sense_v_loop = SENSE_VOLT;
  SENSE_VOLT = SENSE_VOLT / 4;
  sense_v_loop = (sense_v_loop / ADC_RES)*(ADC_REF);
  sense_v_loop = (sense_v_loop / FB_RATIO_VOLT);
  //SENSE_VOLT = ( (float)(SENSE_VOLT) / (float)(ADC_RES))*((float)(ADC_REF));
  //SENSE_VOLT = ((float)(SENSE_VOLT) / (float)(FB_RATIO_VOLT));
}

void SenseOutput_Current()
{
  SENSE_CURR = analogRead(A0);
  sense_i_loop = SENSE_CURR;
  SENSE_CURR = SENSE_CURR / 4;
  sense_i_loop = (sense_i_loop / ADC_RES)*(ADC_REF);
  //SENSE_CURR = ((float)(SENSE_CURR) / (float)(ADC_RES))*((float)(ADC_REF));
  sense_i_loop = sense_i_loop / FB_RATIO_CURR;
  //SENSE_CURR = (float)(SENSE_CURR) / (float)(FB_RATIO_CURR);
}

void UpdatePot_DC()
{
  POT_VOLT = analogRead(A1);
  DC = (POT_VOLT / 1024);
  pwmPin.write(DC);
}
void UpdatePID()
{
  //Input = SENSE_VOLT;
  Input = sense_v_loop;
  //Input = sense_i_loop;
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


