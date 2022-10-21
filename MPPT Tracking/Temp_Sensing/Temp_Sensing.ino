#include <Wire.h>
#include <Adafruit_AMG88xx.h>

Adafruit_AMG88xx Temp_Sensor;
float pixels[AMG88xx_PIXEL_ARRAY_SIZE];
float Temp_Avg;

void Temp_Setup()
{
  status = Temp_Sensor.begin();
  if (!status) 
  {
  Serial.println("Could not find a valid AMG88xx sensor, check wiring!");
  while (1); 
  }
}

void Process_Temp()
{
  //Update Pixel Array
  Temp_Sensor.readPixels(pixels)

  //Serial Print Pixel Array
  Serial.print("[");
  for(int i=1; i<=AMG88xx_PIXEL_ARRAY_SIZE; i++){
    Temp_Avg += pixels[i-1]
    Serial.print(pixels[i-1]);
    Serial.print(", ");
    if( i%8 == 0 ) Serial.println();
  }
  Serial.println("]");
  Serial.println();

  // Average Temp of Pixels in view
  Temp_Avg = Temp_Avg / (AMG88xx_PIXEL_ARRAY_SIZE);
  Temp_Avg = 0;
}

float Read_Ambient_Temp()
{ 
  // Updates internal temp sensor reading and returns value
  return Temp_Sensor.readThermistor();
}


// Setting up drivers for Adafruit 3538 AMG8833 IR Thermal Camera

void setup() 
{

  Serial.begin(9600);
  Temp_Setup();
}

void loop() 
{
  Process_Temp();
  delay(1000);
}
