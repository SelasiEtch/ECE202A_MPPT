#include <Wire.h>
#include <Adafruit_AMG88xx.h>

Adafruit_AMG88xx Temp_Sensor;
float pixels[AMG88xx_PIXEL_ARRAY_SIZE];
float Temp_Avg;

// Max Range of IR Camera is 32°F to 176°F

// To see data from all IR camera pixels, comment out '#define TEST_AMBIENT' and comment '#define TEST_PIXELS_AVG'
// To see the average aggregate temperature of all IR camera pixels, comment out '#define TEST_AMBIENT' and uncomment '#define TEST_PIXELS_AVG'
// To see temperature value from on-board thermistor, uncomment '#define TEST_AMBIENT'

//#define TEST_AMBIENT
#define TEST_PIXELS_AVG

#include <SPI.h>
#include <WiFi.h>

char ssid[] = "";        // your network SSID (name)
char pass[] = "";    // your network password (use for WPA, or use as key for WEP)

int status = WL_IDLE_STATUS;
WiFiServer server(80);

void Temp_Setup()
{
  bool tempStatus;
  tempStatus = Temp_Sensor.begin();
  if (!tempStatus) 
  {
  Serial.println("Could not find a valid AMG88xx sensor, check wiring!");
  while (1); 
  }
}

void Process_Temp_Pixels() 
{

  //Update Pixel Array
  Temp_Sensor.readPixels(pixels);
  //pixels is global so no need to convert the function

  //Serial Print Pixel Array
  /*
  Serial.print("[");
  for(int i=1; i<=AMG88xx_PIXEL_ARRAY_SIZE; i++){ //Serial code for testing
    pixels[i-1] = (pixels[i-1] * 1.8) + 32;
    Temp_Avg += pixels[i-1];
    Serial.print(pixels[i-1]);
    Serial.print(", ");
    if( i%8 == 0 ) Serial.println();
  }
  Serial.println("]");
  Serial.println();
  */
}

float Process_Temp_Pixels_Average()
{
  //Update Pixel Array
  Temp_Sensor.readPixels(pixels);
  for(int i=1; i<=AMG88xx_PIXEL_ARRAY_SIZE; i++){
    pixels[i-1] = (pixels[i-1] * 1.8) + 32;
    Temp_Avg += pixels[i-1];
  }

  // Average Temp of Pixels in view
  Temp_Avg = Temp_Avg / (AMG88xx_PIXEL_ARRAY_SIZE);
  //Serial.println(Temp_Avg); //Serial code for testing
  //Temp_Avg = 0;
  return Temp_Avg;
}

float Read_Ambient_Temp()
{ 
  // Updates internal temp sensor reading and returns value
  return ((Temp_Sensor.readThermistor()*1.8) + 32);
}


// Setting up drivers for Adafruit 3538 AMG8833 IR Thermal Camera

void setup() 
{

  Serial.begin(9600);

  Temp_Setup();

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

void loop() 
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
