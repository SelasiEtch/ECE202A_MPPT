union BtoF
{
  byte b[16];
  float fval;  
} u;

const int buffer_size = 16;
byte buf[buffer_size];

float myVal;

void setup()
{
  Serial.begin(115200);
}


void loop()
{
  if (Serial.available()>0)
  {
    myVal = readFromMatlab();
    delay(0.1);
    writeToMatlab(2, 1);
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
