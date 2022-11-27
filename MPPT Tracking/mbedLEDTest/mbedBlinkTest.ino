// Testing the scheduling of the RGB LED

#include <rtos.h>
#include<mbed.h>
#include <WiFiNINA.h>

#define MS(x) chrono::milliseconds(x) // millisecond function

using namespace std; // we will be using std::chrono
using namespace rtos; // we will be using rtos::ThisThread
using namespace mbed;

// Declaring the Thread objects
Thread led_red_thread;
Thread led_green_thread;
// Thread led_blue_thread;

void setup_leds() {
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  // pinMode(LEDB, OUTPUT);
}

// function to be attached to led_red_thread

void led_red_function() {
  for (;;) {
  
    ThisThread::sleep_for(MS(512));
    
    for(int i = 255; i >= 0; i--) {
      analogWrite(LEDR, 255 - i);
      ThisThread::sleep_for(MS(1));
    }
    
    ThisThread::sleep_for(MS(512));
  
    for(int i = 05; i < 255; i++) {
      analogWrite(LEDR, 255 - i);
      ThisThread::sleep_for(MS(1));
    }
    
  }
}

// function to be attached to led_green_thread
void led_green_function() {
  for (;;) {
  
    ThisThread::sleep_for(MS(256));
     
    for(int i = 0; i < 255; i++) {
      analogWrite(LEDG, 255 - i);
      ThisThread::sleep_for(MS(1));
    }
    
    ThisThread::sleep_for(MS(512));
    
    for(int i = 255; i>= 0; i--) {
      analogWrite(LEDG, 255 - i);
      ThisThread::sleep_for(MS(1));
    }
    
    ThisThread::sleep_for(MS(256));
  }
}

/*void led_blue_function() {
  for (;;) {
  
    ThisThread::sleep_for(MS(256));
     
    for(int i = 0; i < 255; i++) {
      analogWrite(LEDB, 255 - i);
      ThisThread::sleep_for(MS(1));
    }
    
    ThisThread::sleep_for(MS(512));
    
    for(int i = 255; i>= 0; i--) {
      analogWrite(LEDB, 255 - i);
      ThisThread::sleep_for(MS(1));
    }
    
    ThisThread::sleep_for(MS(512));
  }
}
*/


int main() {
  setup_leds();
  led_red_thread.start(led_red_function); // start red thread
  led_green_thread.start(led_green_function); // start green thread
  //led_blue_thread.start(led_blue_function); // start blue thread
}
