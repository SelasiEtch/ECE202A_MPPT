// Define analog input
#define VoltageInPin A1
#define CurrentSensor A0
 
// Floats for ADC voltage & Input voltage
float adc_voltage = 0.0;
float in_voltage = 0.0;
 
// Floats for resistor values in divider (in ohms)
float R1 = 39000.0; // 39K
float R2 = 10000.0; // 10K 
 
// Float for Reference Voltage
float ref_voltage = 3.3;
 
// Integer for ADC value
int adc_value = 0;

float i;
 
void setup()
{
   // Setup Serial Monitor
   Serial.begin(9600);
   Serial.println("DC Voltage and Current Test");
   pinMode(VoltageInPin, INPUT);
   pinMode(CurrentSensor, INPUT);
}
 
void loop(){
   // Read the Analog Input
   adc_value = analogRead(VoltageInPin);
   
   // Determine voltage at ADC input
   adc_voltage  = (adc_value * ref_voltage) / 1024.0; 
   
   // Calculate voltage at divider input
   in_voltage = adc_voltage / (R2/(R1+R2)) ; 

   // Print results to Serial Monitor to 2 decimal places
  Serial.print("Input Voltage = ");
  Serial.println(in_voltage, 2);

  // Current Sensing for ACS712 Sensor 
    i = analogRead(CurrentSensor);
    Serial.print("Current = ");
    Serial.println(i);
  
  // Short delay
  delay(100);
}
