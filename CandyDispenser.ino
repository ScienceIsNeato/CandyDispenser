#include <Servo.h>

/****** Servo Stuff *****/
Servo dispenser;  // create servo object to control a servo
const int servo_pin = 9;

const int spin_cmd = 180; // Full speed clockwise
const int stop_cmd = 90;  // Stop for a CR servo

// Duration to spin the servo in milliseconds. Note, this is
// determined via experimentation. In the future, better to use a 
// limit switch to stop after a single rotation.

const int single_spin_duration = 1450; 


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
const int time_between_candy = 10000; // 10 seconds

float sonar_distance = 0.0;

// TODO - have a green LED when ready to dispense
// TODO - have a red LED when dispensing or resetting/waiting
// TODO - have servo automatically stop once dispensed instead of being on a timer


void setup()
{
    Serial.begin(9600); // open the serial port at 9600 bps:
    dispenser.attach(servo_pin);  // attaches the servo on pin 9 to the servo object

    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);

    stopServo();
}

void startServo()
{
    Serial.println("About to start spinning the servo");
    dispenser.write(spin_cmd);
}

void stopServo()
{
    Serial.println("About to stop spinning the servo");
    dispenser.write(stop_cmd);
}

void spinServoOnce()
{
    startServo();
    delay(single_spin_duration);
    stopServo();
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

    Serial.print("Distance: ");
    Serial.print(total_distance/smoothing_iterations);
    Serial.println(" inches.");

    return total_distance/smoothing_iterations;
}

void loop() 
{
    sonar_distance = getSonarReading();

    if(sonar_distance >= min_trigger_distance &&
       sonar_distance <= max_trigger_distance)
    {
        Serial.println("!!! --- CANDY TIME --- !!!!");
        spinServoOnce();
        Serial.print("Waiting ");
        Serial.print(time_between_candy/1000.0);
        Serial.println(" to start potentially dispensing again...");
        delay(time_between_candy);
    }
    delay(100);
}
