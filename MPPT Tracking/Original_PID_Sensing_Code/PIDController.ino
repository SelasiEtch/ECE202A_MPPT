#include <AutoPID.h> // Automatic PID library from Arduino

// PID settings and gains
#define OUTPUT_MIN 0
#define OUTPUT_MAX 255
#define KP ? // Wasn't sure what these values would be
#define KI ? 
#define KD ?

// Variables
double analogReadValue, setPoint, outputVal;

// Input/output variables passed by reference, so they are updated automatically
AutoPID myPID(&analogReadValue, &setPoint, &outputVal, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);

// Setup timeStep
void setup() {
  myPID.setTimeStep(500);
}


void loop() {
  setPoint = analogRead(A0);          // Set our desired point (read the POT)
  analogReadValue = analogRead(A1);   // Read our output voltage
  myPID.run();                        // Run the PID 
  analogWrite(6, outputVal); // Write the PID result to the output
}