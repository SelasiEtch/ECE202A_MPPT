union BtoF
{
  byte b[16];
  float fval;  
} u;

float SENSE_VOLT = 0;
float SENSE_CURR = 0;
float SENSE_POWER = 0;

float ERROR = 0;
float ERROR_PREV = 0;

float DELTA_VOLT = 0;
float DELTA_POWER = 0;
float DELTA_ERROR = 0;


float VOLT_PREV = 0;

const int buffer_size = 16;
byte buf[buffer_size];

float myVal;

void setup()
{
  Serial.begin(115200);
}


void loop()
{
  Calculate_Fuzzy_Inputs();
  if (Serial.available()>0)
  {
    myVal = readFromMatlab();
    delay(0.1);
    writeToMatlab(ERROR, DELTA_ERROR);
  }
}

float readFromMatlab()
{
  int reln = Serial.readBytesUntil('\r\n', buf, buffer_size);
  for (int i=0; i<buffer_size; i++)
  {
    u.b[i] = buf[i];
  }
  float output = u.fval;
  return output;
}

void writeToMatlab (float error, float change)
{
  byte *b = (byte *) &error;
  Serial.write(b, 4);
  byte *b2 = (byte *) &change;
  Serial.write(b2, 4);
  Serial.write(13); // "\r"
  Serial.write(10); // "\n"
}

void SenseOutput_Current()
{
  SENSE_CURR = analogRead(A0);
  SENSE_CURR = (SENSE_CURR / ADC_RES)*(ADC_REF);
  SENSE_CURR = SENSE_CURR / FB_RATIO_CURR;
}

void SenseOutput_Voltage()
{
  VOLT_PREV = SENSE_VOLT;
  SENSE_VOLT = analogRead(A3);
  SENSE_VOLT = (SENSE_VOLT / ADC_RES)*(ADC_REF);
  SENSE_VOLT = (SENSE_VOLT / FB_RATIO_VOLT);
}

void Calculate_Fuzzy_Inputs()
{
  POWER_PREV = SENSE_POWER;

  SenseOutput_Voltage();
  SenseOutput_Current();

  SENSE_POWER = SENSE_VOLT * SENSE_CURR;

  DELTA_VOLT = SENSE_VOLT - VOLT_PREV;
  DELTA_POWER = SENSE_POWER - POWER_PREV;

  ERROR_PREV = ERROR;

  ERROR = (DELTA_POWER / DELTA_VOLT);

  DELTA_ERROR = ERROR - ERROR_PREV;
}
