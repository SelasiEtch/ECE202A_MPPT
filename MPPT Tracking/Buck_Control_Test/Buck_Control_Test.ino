#include  <WiFiNINA.h>

void setup() {
  // put your setup code here, to run once:
 // pinMode(A0,INPUT); //
  pinMode(A3,INPUT); // Sensed Voltage
  pinMode(A6,OUTPUT); // PWM Output
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  //int potVal = analogRead(A0);
  //float Duty = ((float)potVal) * (1 / 1023.0);
  //Serial.println(Duty);

  int V_sense_Val = analogRead(A3);
  Serial.println(V_sense_Val);

  //uint8_t PW = Duty * 255;
  //Serial.println(PW);

  //analogWrite(A6,PW);
  analogWrite(A6,255);
  delay(0);
}
