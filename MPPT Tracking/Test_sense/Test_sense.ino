#include <AutoPID.h> // Automatic PID library from Arduino
#include <Wire.h>
#include <Adafruit_AMG88xx.h>


#define OUTPUT_MIN 0
#define OUTPUT_MAX 255
#define KP 0 
#define KI 0
#define KD 0

#define V_SENSE_PIN A0
#define I_SENSE_PIN A1
#define PWM_PIN A2

Adafruit_AMG88xx Temp_Sensor;
float pixels[AMG88xx_PIXEL_ARRAY_SIZE];
float Temp_Sense;
uint8_t Current_Sense;
uint8_t Voltage_Sense;

double outputVal;
double Current_Setpoint;

AutoPID myPID(&Current_Sense, &Current_Setpoint, &outputVal, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);

uint8_t sense[3];
float irr_est;
float Vmp;
float Imp;
float Pmp;
float Temp;

int Gref = 1000;
uint8_t Tref = 25;

float alpha_T = 0.046;
float beta_T = -0.0118;

float Imp_ref = 0.35;
float Vmp_ref = 6;

void setup() 
{
  Temp_Setup();
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(V_SENSE_PIN, INPUT);
  pinMode(I_SENSE_PIN, INPUT);
  myPID.setTimeStep(500);

}

void loop() 
{
  UpdateSensorData();
  RunNN();
  runPID();
}

float calculate_Imp(double irr_est, double Temp)
{
  float imp = Imp_ref * (irr_est/Gref)*(1+(alpha_T*(Temp-Tref)));
  return imp;
}

float calculate_Vmp(double Temp)
{
  float vmp = Vmp_ref + (beta_T*(Temp-Tref));
  return vmp;
}

void Temp_Setup()
{
  bool status;
  status = Temp_Sensor.begin();
  if (!status) 
  {
  Serial.println("Could not find a valid AMG88xx sensor, check wiring!");
  while (1); 
  }
}

void Update_Temp_Average()
{
  float Temp_Avg_dummy;
  //Update Pixel Array
  Temp_Sensor.readPixels(pixels);
  for(int i=1; i<=AMG88xx_PIXEL_ARRAY_SIZE; i++){
    pixels[i-1] = (pixels[i-1] * 1.8) + 32;
    Temp_Avg_dummy += pixels[i-1];
  }

  // Average Temp of Pixels in view
  Temp_Sense = Temp_Avg_dummy / (AMG88xx_PIXEL_ARRAY_SIZE);
  Temp_Avg_dummy= 0;
}

void UpdateSensorData()
{
    Voltage_Sense = analogRead(V_SENSE_PIN);
    Current_Sense = analogRead(I_SENSE_PIN);
    Update_Temp_Average();
}

void RunNN()
{
    sense[0] = (uint8_t)Voltage_Sense;
    sense[1] = (uint8_t)Current_Sense;
    sense[2] = (uint8_t)Temp_Sense;
    Serial.write((uint8_t*)sense,sizeof(sense));
    Serial.write("\r\n");

    while(Serial.available() == 0);

      if (Serial.available() > 0) 
    {
      irr_est = (10* Serial.read());
      Imp = calculate_Imp(irr_est, Temp_Sense);
      Vmp = calculate_Vmp(Temp_Sense);
      Pmp = Vmp * Imp;
      digitalWrite(LED_BUILTIN, HIGH);
      delay(500);
      digitalWrite(LED_BUILTIN, LOW);
      delay(500);
    }
}

void runPID()
{
  Current_Setpoint = Imp;
  myPID.run();
  analogWrite(PWM_PIN, outputVal);
}

