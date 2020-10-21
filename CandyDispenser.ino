#include <Servo.h>

#define ON 1
#define OFF 0

/****** Servo Stuff *****/
Servo dispenser;  // create servo object to control a servo
const int servo_pin = 9;

const int spin_cmd = 180; // Full speed clockwise
const int stop_cmd = 90;  // Stop for a CR servo

// Duration to spin the servo in milliseconds. Note, this is
// determined via experimentation. In the future, better to use a 
// limit switch to stop after a single rotation.

const int single_spin_duration = 1450; // in milliseconds


/***** Sonar Sensor Stuff *****/
const int trigPin = 11;
const int echoPin = 10;

float duration = 0;
float distance = 0;

// Take the average of X sonar readings during query
int smoothing_iterations = 15;

// Distance travelled by speed of sound over 10 ms over 2 for round trip, 
// then converted from cm to in = 0.343/2*0.393701
const float RAW_DIST_TO_INCHES = 0.00675197215;


/****** Control Stuff ******/
// You have to get within a foot of the sonar sensor to trigger
// dandy dispensing
const float min_trigger_distance = 1.0; // inches
const float max_trigger_distance = 12.0; // inches

// Amount of time to wait between dispensations
const int time_between_candy_ms = 10000; // 10 seconds

float sonar_distance = 0.0;

int ready_LED_pin = 12; // This LED is lit when ready and waiting to be triggered (use ~220ohm resistor here)
int waiting_LED_pin = 13; // This LED is lit when dispensing/pausing (use ~220ohm resistor here)

// TODO - have servo automatically stop once dispensed instead of being on a timer

void setup()
{
    Serial.begin(9600);                 // open the serial port at 9600 bps
    dispenser.attach(servo_pin);        // attaches the servo

    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);

    pinMode(ready_LED_pin, OUTPUT);     // Declare the LEDs as outputs
    pinMode(waiting_LED_pin, OUTPUT);   // Declare the LEDs as outputs

    stopServo();                        // Make sure servo isn't moving upon startup

    // Turn on the LED that indicates "READY" status and turn off "WAITING" LED
    setLED(ready_LED_pin, ON);
    setLED(waiting_LED_pin, OFF);
}

void setLED(int LED_pin, bool state)
{
    // Set a specific LED either on or off explicitly
    if(state)
    {
        digitalWrite(LED_pin, HIGH); // Turn the LED on
    }
    else
    {
        digitalWrite(LED_pin, LOW); // Turn the LED off
    }
}

bool toggleLED(int LED_pin, bool current_state)
{
    // Quick way to toggle an LED based on its current state and return new state
    bool LED_state = current_state ? OFF : ON;
    setLED(LED_pin, LED_state);
    return LED_state;
}

void startServo()
{
    // Serial.println("About to start spinning the servo");
    dispenser.write(spin_cmd);
}

void stopServo()
{
    // Serial.println("About to stop spinning the servo");
    dispenser.write(stop_cmd);
}

void spinServoOnce()
{
    startServo();
    delay(single_spin_duration);
    stopServo();
}

void dispenseCandy()
{
    Serial.println("!!! --- CANDY TIME --- !!!");
Serial.println("               _      _               "); 
Serial.println("          (_)    | |                  ");
Serial.println(" ___ _ __  _  ___| | _____ _ __ ___   ");
Serial.println("/ __| '_ \\| |/ __| |/ / _ \\ '__/ __|  ");
Serial.println("\\__ \\ | | | | (__|   <  __/ |  \\__ \\  ");
Serial.println("|___/_| |_|_|\\___|_|\\_\\___|_|  |___/  ");

    // Toggle both LEDs
    setLED(ready_LED_pin, OFF);
    setLED(waiting_LED_pin, ON);

    spinServoOnce();
}

void wait_until_ready()
{
    // Kids/user want to know when it is ready to dispense again. This
    // method gives visual indicators to watchers by toggling some LEDs
    Serial.print("Waiting "+String(time_between_candy_ms/1000.0)+" seconds to start potentially dispensing again...");

    setLED(waiting_LED_pin, ON); // Turn wating LED on

    // During delay period, toggle refractory LED on/off as exponential decay until ready
    float ms_elapsed = 0;
    bool LED_state = ON;
    float toggle_time = 750; // start toggle time in ms

    do  
    {
      delay(toggle_time);
      LED_state = toggleLED(waiting_LED_pin, LED_state);
      toggleLED(ready_LED_pin, LED_state);
      ms_elapsed += toggle_time;
      toggle_time = toggle_time*.9;
    }
    while (ms_elapsed < time_between_candy_ms && toggle_time > 25); // keep toggling until only 50 ms left

    // Turn ready LED on as we leave this method
    setLED(ready_LED_pin, ON);
    setLED(waiting_LED_pin, OFF);
}

float getSonarReading()
{
    float total_distance = 0;
  
    for(int i = 0; i < smoothing_iterations; i++)
    {
      // Set trigger pin low. Wait a very brief period to ensure that it is low
      digitalWrite(trigPin, LOW);
      delayMicroseconds(2);
  
      // Set trigger high for exactly 10 ms, then trun off again
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);
  
      // Query to see how long it took for the sound wave to return
      duration = pulseIn(echoPin, HIGH);
      distance = duration*RAW_DIST_TO_INCHES;
      total_distance += distance;
      delay(25);
    }

    Serial.println("Distance: "+String(total_distance/smoothing_iterations)+" inches.");
    return total_distance/smoothing_iterations;
}

void loop() 
{
    sonar_distance = getSonarReading();

    if(sonar_distance >= min_trigger_distance &&
       sonar_distance <= max_trigger_distance)
    {
        dispenseCandy();
        wait_until_ready();
    }
    delay(100);
}
