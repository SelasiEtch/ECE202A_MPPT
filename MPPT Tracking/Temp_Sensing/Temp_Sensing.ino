#include <Wire.h>
#include <Adafruit_AMG88xx.h>

Adafruit_AMG88xx Temp_Sensor;
float pixels[AMG88xx_PIXEL_ARRAY_SIZE];
float Temp_Avg;

//#define TEST_AMBIENT // Preproccessor directive 
#define TEST_PIXELS_AVG

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

void Process_Temp_Pixels()
{

  //Update Pixel Array
  Temp_Sensor.readPixels(pixels);

  //Serial Print Pixel Array
  Serial.print("[");
  for(int i=1; i<=AMG88xx_PIXEL_ARRAY_SIZE; i++){
    Temp_Avg += pixels[i-1];
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

void Process_Temp_Pixels_Average()
{
  //Update Pixel Array
  Temp_Sensor.readPixels(pixels);
  for(int i=1; i<=AMG88xx_PIXEL_ARRAY_SIZE; i++){
    Temp_Avg += pixels[i-1];
  }

  // Average Temp of Pixels in view
  Temp_Avg = Temp_Avg / (AMG88xx_PIXEL_ARRAY_SIZE);
  Serial.println(Temp_Avg);
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

#ifndef TEST_AMBIENT
  #ifndef TEST_PIXELS_AVG
    Process_Temp_Pixels();
  #endif
  #ifdef TEST_PIXELS_AVG
    Process_Temp_Pixels_Average();
  #endif
#endif

#ifdef TEST_AMBIENT  
  float temp;
  temp = Read_Ambient_Temp();
  Serial.println(temp);
#endif

  delay(1000);
}
