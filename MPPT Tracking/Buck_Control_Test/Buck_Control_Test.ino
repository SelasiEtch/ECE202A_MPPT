#define POT_PIN A0
#define PWM_OUT A7
#define DUMMY_VOLT_SENSE A5

void setup() {
  // put your setup code here, to run once:
  pinMode(A0,INPUT);
  pinMode(A6,OUTPUT);
  pinMode(A5, INPUT);
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  int potVal = analogRead(A0);
  float Duty = ((float)potVal) * (1 / 1023.0);
  //Serial.println(Duty);

  int V_sense_Val = analogRead(A5);
  Serial.println(V_sense_Val);

  uint8_t PW = Duty * 255;
  //Serial.println(PW);

  analogWrite(A6,PW);
  delay(50);
}
