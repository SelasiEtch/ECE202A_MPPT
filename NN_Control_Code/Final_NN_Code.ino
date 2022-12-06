#include "mbed.h"
#include <PID_v1.h>
#include <Wire.h>
#include <Adafruit_AMG88xx.h>
#include <SPI.h>
#include <WiFi.h>

#define PWM_PIN               12
#define DIRECTION_PIN        11
#define PWM_FREQUENCY         31000
#define ADC_RES               1024
#define ADC_REF               3.3
#define FB_RATIO_VOLT         0.25
#define FB_RATIO_CURR         1.65
#define TEST_PIXELS_AVG
// A0 is current Sensing Pin
// A3 is Output Voltage Sensing
// A1 is pot sensing pin 
uint8_t sense[3];
float irr_est;

float duty = 0;
float duty_2 = 0;

float POT_VOLT = 0;

float SENSE_VOLT = 0;
float sense_v_loop = 0;
float sense_i_loop = 0;
float SENSE_CURR = 0;
float SENSE_POWER = 0;
float DELTA_POWER = 0;
float VOLT_PREV = 0;
float POWER_PREV = 0;
float DELTA_VOLT = 0;

float DC = 0;


double Setpoint, Input, Output;
double Kp=13, Ki=100, Kd=0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
mbed::PwmOut pwmPin(digitalPinToPinName(PWM_PIN));

Adafruit_AMG88xx Temp_Sensor;
float pixels[AMG88xx_PIXEL_ARRAY_SIZE];
float SENSE_TEMP;

char ssid[] = "CXNK003E0507";        // your network SSID (name)
char pass[] = "1aace5f91bba533c";    // your network password (use for WPA, or use as key for WEP)

int status = WL_IDLE_STATUS;
WiFiServer server(80);

// Create three threads for each function that needs to be executed
Thread t1; 
Thread t2;
Thread t3;

void setup() 
{
  // put your setup code here, to run once:
  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIRECTION_PIN, OUTPUT);
  pinMode(A0,INPUT);
  pinMode(A1,INPUT);
  pinMode(A3, INPUT);
  digitalWrite(DIRECTION_PIN, HIGH);
  pwmPin.period( 1.0 / PWM_FREQUENCY );
  pwmPin.write( 0.5 );
  //pinMode(A0, OUTPUT);
  Serial.begin(115200);
  //myPID.SetMode(AUTOMATIC);
  Temp_Setup();

  Setpoint = 1;

  // FROM BASHEER'S WIFI CODE

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  // attempt to connect to WiFi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to Network named: ");
    Serial.println(ssid);                   // print the network name (SSID);

    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);
    // wait 10 seconds for connection:
    delay(10000);
  }

  Serial.println("Connected!");
  server.begin();
  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  // print where to go in a browser:
  Serial.print("To see this page in action, open a browser to http://");
  Serial.println(ip);

}

void loop() // 
{
  t1.start(callback(callFuncs)); // thread 1
  t2.start(callback(wifiTest)); // thread 2
  t3.start(callback(UpdateNN)); // thread 3
}

void callFuncs() // Thread 1
{
  SenseOutput_Voltage();
  SenseOutput_Current();
  Process_Temp_Pixels_Average();

  sense[0] = (uint8_t)SENSE_VOLT;
  sense[1] = (uint8_t)SENSE_CURR;
  sense[2] = (uint8_t)SENSE_TEMP;

 /* 
  Serial.print("Sense Voltage: ");
  Serial.println(sense_v_loop);
  Serial.print("Sense Current: ");
  Serial.println(sense_i_loop);
  Serial.print("Sense Temp: ");
  Serial.println(SENSE_TEMP);
  */
  

  Serial.write((uint8_t*)sense,sizeof(sense));
  Serial.write("\r\n");

  //while(Serial.available() == 0);
/*
  if (Serial.available() > 0) 
  {
    irr_est = (10* Serial.read());
    //Imp = calculate_Imp(irr_est, SENSE_TEMP);
    //Vmp = calculate_Vmp(SENSE_TEMP);
    //Pmp = Vmp * Imp;
    //digitalWrite(LED_BUILTIN, HIGH);
    //delay(500);
    //digitalWrite(LED_BUILTIN, LOW);
    //delay(500);
  }
  */
  
  UpdatePot_DC();
  //Serial.println(DC);
  
  /*
  //UpdatePID();
  Serial.print("Sense Voltage: ");
  Serial.println(sense_v_loop);
  Serial.print("Duty Cycle: ");
  Serial.println(DC);
  Serial.print("Output Current: ");
  Serial.println(SENSE_CURR);
  //Serial.print("Average Temperature: ");
  //Serial.println(SENSE_TEMP);
  delay(500);
  */
  delay(300);
}


void wifiTest() // Thread 2: FROM BASHEER'S WIFI TEST
{
  WiFiClient client = server.available();   // listen for incoming clients

  if (client) {                             // if you get a client,
    Serial.println("new client");           // print a message out the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        if (c == '\n') {                    // if the byte is a newline character

          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();

            client.print("Average Temp (F): ");
            client.print(Process_Temp_Pixels_Average());
            Temp_Avg = 0;
            client.print("  //  Temp per Pixel (F): "); //How to do newlines?
            Process_Temp_Pixels();

            for(int i=1; i<=AMG88xx_PIXEL_ARRAY_SIZE; i++){
              pixels[i-1] = (pixels[i-1] * 1.8) + 32;
              client.print(pixels[i-1]);
              client.print(", ");
              if( i%8 == 0 ) client.println();
            }

            client.println("]");

            /*
            #ifndef TEST_AMBIENT
              #ifndef TEST_PIXELS_AVG
                Process_Temp_Pixels();
              #endif
              #ifdef TEST_PIXELS_AVG
                client.println(Process_Temp_Pixels_Average());
              #endif
            #endif
            */

            #ifdef TEST_AMBIENT  
              float temp;
              temp = Read_Ambient_Temp();
              client.println(temp);
            #endif

            client.print("<meta http-equiv=\"refresh\" content=\"0.5\">");

            // The HTTP response ends with another blank line:
            client.println();
            // break out of the while loop:
            break;
          } else {    // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    // close the connection:
    client.stop();
    Serial.println("client disconnected");
  }
  delay(500);
}

void UpdateNN() // Thread 3
{
    Serial.write((uint8_t*)sense,sizeof(sense));
  Serial.write("\r\n");


  while(Serial.available() == 0);

  if (Serial.available() > 0) 
  {
    irr_est = (8* Serial.read());
    Imp = calculate_Imp(irr_est, SENSE_TEMP);
    Vmp = calculate_Vmp(SENSE_TEMP);
    Pmp = Vmp * Imp;
    Setpoint = Imp;
    UpdatePID();
    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(LED_BUILTIN, LOW);
  }

}

void Pert_Obs()
{
  SenseOutput_Voltage();
  SenseOutput_Current();

  SENSE_POWER = SENSE_VOLT * SENSE_CURR;
  DELTA_POWER = SENSE_POWER - POWER_PREV;

  DELTA_VOLT = SENSE_VOLT - VOLT_PREV;
  if(DELTA_POWER !=0 || DELTA_VOLT != 0)
  {
    if(DELTA_POWER > 0)
    {
      if(DELTA_VOLT < 0)
      {
        duty++;
      }
      else
      {
        duty--;
      }
    }
    else
    {
      if(DELTA_VOLT < 0)
      {
        duty--;
      }
      else
      {
        duty++;
      }
    }
  }
  else
  {
    duty = duty;
  }
  duty_2 = duty / 100;
  
  pwmPin.write(duty_2);

}

void Temp_Setup()
{
  bool status;
  status = Temp_Sensor.begin();
  if (!status) 
  {
  //Serial.println("Could not find a valid AMG88xx sensor, check wiring!");
  while (1); 
  }
}

void Process_Temp_Pixels_Average()
{
  //Update Pixel Array
  SENSE_TEMP = 0;
  Temp_Sensor.readPixels(pixels);
  for(int i=1; i<=AMG88xx_PIXEL_ARRAY_SIZE; i++){
    //pixels[i-1] = (pixels[i-1] * 1.8) + 32;
    SENSE_TEMP += pixels[i-1];
  }

  // Average Temp of Pixels in view
  SENSE_TEMP = SENSE_TEMP / (AMG88xx_PIXEL_ARRAY_SIZE);
}

// FROM BASHEER'S WIFI TEST

float Read_Ambient_Temp()
{ 
  // Updates internal temp sensor reading and returns value
  return ((Temp_Sensor.readThermistor()*1.8) + 32);
}

void SenseOutput_Voltage()
{
  SENSE_VOLT = analogRead(A3);
  sense_v_loop = SENSE_VOLT;
  SENSE_VOLT = SENSE_VOLT / 4;
  sense_v_loop = (sense_v_loop / ADC_RES)*(ADC_REF);
  sense_v_loop = (sense_v_loop / FB_RATIO_VOLT);
  //SENSE_VOLT = ( (float)(SENSE_VOLT) / (float)(ADC_RES))*((float)(ADC_REF));
  //SENSE_VOLT = ((float)(SENSE_VOLT) / (float)(FB_RATIO_VOLT));
}

void SenseOutput_Current()
{
  SENSE_CURR = analogRead(A0);
  sense_i_loop = SENSE_CURR;
  SENSE_CURR = SENSE_CURR / 4;
  sense_i_loop = (sense_i_loop / ADC_RES)*(ADC_REF);
  //SENSE_CURR = ((float)(SENSE_CURR) / (float)(ADC_RES))*((float)(ADC_REF));
  sense_i_loop = sense_i_loop / FB_RATIO_CURR;
  //SENSE_CURR = (float)(SENSE_CURR) / (float)(FB_RATIO_CURR);
}

void UpdatePot_DC()
{
  POT_VOLT = analogRead(A1);
  DC = (POT_VOLT / 1024);
  pwmPin.write(DC);
}
void UpdatePID()
{
  //Input = SENSE_VOLT;
  Input = sense_v_loop;
  myPID.Compute();
  DC = (Output / 255);
  pwmPin.write(DC);
}

float calculate_Imp(double irr_est, double Temp)
{
  float Imp_ref = 0.35;
  float Gref = 1000;
  float alpha_T = 0.046;
  float Tref = 25;

  float imp = Imp_ref * (irr_est/Gref)*(1+(alpha_T*(Temp-Tref)));
  return imp;
}

float calculate_Vmp(double Temp)
{
  float beta_T = -0.0118;
  float Tref = 25;
  float Vmp_ref = 6;
  float vmp = Vmp_ref + (beta_T*(Temp-Tref));
  return vmp;
}
