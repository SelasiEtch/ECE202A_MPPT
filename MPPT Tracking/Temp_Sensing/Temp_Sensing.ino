#include <SPI.h>
#include <Melopero_AMG8833.h>

Melopero_AMG8833 Temp_Sensor;
// Testing Commit to Git
//2nd

void Temp_Setup()
{
  // Register Temp_Sensor Object as slave device for I2C bus
    Temp_Sensor.initI2C();

    // After Initialization, reset sensor and set FPS (Frames per second) to max of the camera (10Hz)
    Serial.print("Resetting sensor ... ");  
    int statusCode = Temp_Sensor.resetFlagsAndSettings();
    Serial.println(Temp_Sensor.getErrorDescription(statusCode));

    Serial.print("Setting FPS ... ");
    statusCode = Temp_Sensor.setFPSMode(FPS_MODE::FPS_10);
    Serial.println(Temp_Sensor.getErrorDescription(statusCode));
}

void I2C_Setup()
{
  //Begin I2C connection
  Wire.begin();
}



// Setting up drivers for Adafruit 3538 AMG8833 IR Thermal Camera

void setup() {
  Serial.begin(9600);

}

void loop() {
  Serial.print("Updating thermistor temperature ... ");
  int statusCode = Temp_Sensor.updateThermistorTemperature();
  Serial.println(Temp_Sensor.getErrorDescription(statusCode));

  Serial.print("Updating pixel matrix ... ");
  statusCode = Temp_Sensor.updatePixelMatrix();
  Serial.println(Temp_Sensor.getErrorDescription(statusCode));

  Serial.print("Thermistor temp: ");
  Serial.print(Temp_Sensor.thermistorTemperature);
  Serial.println("°C");

  Serial.println("Temperature Matrix: ");
  for (int x = 0; x < 8; x++){
    for (int y = 0; y < 8; y++){
      Serial.print(Temp_Sensor.pixelMatrix[y][x]);
      Serial.print(" ");
    }
    Serial.println();
  }

  delay(1000);
}
