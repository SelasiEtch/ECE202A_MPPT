#define SENSOR_PIN A0

// desired voltage (determined by the NN)
float V1 = ? ;

// proportional controller coefficient
float Kp = 0;

// Initialize Global variables
// reading from the voltage sensor V2
float V2 = 0;
// error between the desired voltage and the voltage we read
float e = V1 - V2;
// output
float u = 0

void setup() {
    Serial.begin(9600);
    pinMode(SENSOR_PIN, INPUT);
}

void loop() {

    // store measured voltage in this variable
    V2 = ?

    // compute the error between the measurement and the desired value
    e = V2 - V1

    // compute the control effort by multiplying the error by Kp
    u = e * Kp

    // make sure the output value is bounded to 0 to 255 using the bound function defined below

    u = bound(u, 0, 255);
    // analogWrite(LED_PIN, u); // then write it to the LED pin to change control voltage to LED

    // plot the measurement
    Serial.print(V2);
    Serial.print('\t');
    // plot the desired output
    Serial.print(V1);
    Serial.print('\t');
    // plot the error
    Serial.println(e);

    delay(50);
}

// Bound the input value between x_min and x_max.
float bound(float x, float x_min, float x_max) {
    if (x < x_min) { x = x_min; }
    if (x > x_max) { x = x_max; }
    return x;
}