#include <Wire.h>
#include <PS4Controller.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

// MOTOR DRIVER PINS
// FRONT
const byte F_ENA = 4;
const byte F_IN1 = 16;
const byte F_IN2 = 17;
const byte F_IN3 = 5;
const byte F_IN4 = 18;
const byte F_ENB = 19;
// BACK
const byte B_ENA = 12;
const byte B_IN1 = 14;
const byte B_IN2 = 27;
const byte B_IN3 = 26;
const byte B_IN4 = 25;
const byte B_ENB = 33;

// Scaling factor for joystick sensitivity
// Increase for higher sensitivity
const float joystickScaleFactor = 1.5; 

// Dead zone for joystick inputs
const int joystickDeadZone = 10; 

/* Assign a unique ID to this sensor at the same time */
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

float initHeading = 0.0;

void InitMotorDriverPins()
{
    pinMode(F_ENA, OUTPUT);
    pinMode(F_IN1, OUTPUT);
    pinMode(F_IN2, OUTPUT);
    pinMode(F_IN3, OUTPUT);
    pinMode(F_IN4, OUTPUT);
    pinMode(F_ENB, OUTPUT);

    pinMode(B_ENA, OUTPUT);
    pinMode(B_IN1, OUTPUT);
    pinMode(B_IN2, OUTPUT);
    pinMode(B_IN3, OUTPUT);
    pinMode(B_IN4, OUTPUT);
    pinMode(B_ENB, OUTPUT);
}

void controlMotor(int enPin, int in1Pin, int in2Pin, int speed) 
{
  speed = constrain(speed, -255, 255);

  // Set direction
  if (speed > 0) {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
  } else if (speed < 0) {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
  } else {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
  }

 
  analogWrite(enPin, abs(speed));
}


int applyDeadZone(int value, int deadZone) 
{
  if (abs(value) < deadZone) {return 0;} 
  else if (value > 0)        {return value - deadZone;} 
  else                       {return value + deadZone;}
}

float GetHeadingDegrees()
{
    /* Get a new sensor event */ 
    sensors_event_t event; 
    mag.getEvent(&event);

    float heading = atan2(event.magnetic.y, event.magnetic.x);

    float declinationAngle = (0.0 + (34.0 / 60.0)) / (180 / M_PI);
    heading += declinationAngle; 
    
    // Correct for heading < 0deg and heading > 360deg
    if (heading < 0)      {heading += 2 * PI;}
    if (heading > 2 * PI) {heading -= 2 * PI;}

    return heading * (180/M_PI); // heading in degrees
}

void FieldCentric()
{
  int leftX = PS4.LStickX();
  int leftY = -PS4.LStickY();
  int rightX = -PS4.RStickX();
  leftX = int(applyDeadZone(leftX, joystickDeadZone) * joystickScaleFactor);
  leftY = int(applyDeadZone(leftY, joystickDeadZone) * joystickScaleFactor);
  rightX = int(applyDeadZone(rightX, joystickDeadZone) * joystickScaleFactor);


  float currHeading = GetHeadingDegrees();
  currHeading += initHeading;

  float cosHeading = cos(currHeading * (PI / 180.0));
  float sinHeading = sin(currHeading * (PI / 180.0));

  int rotatedLeftX = int(leftX * cosHeading + leftY * sinHeading);
  int rotatedLeftY = int(-leftX * sinHeading + leftY * cosHeading);

  // Calculate motor speeds
  int forward = rotatedLeftY;  // Positive is forward
  int strafe = rotatedLeftX;   // Positive is right strafe
  int rotation = rightX;       // Positive is clockwise rotation

  // Adjust motor speeds based on orientation
  int speedFR = forward + rotation + strafe;  
  int speedFL = forward - rotation - strafe;  
  int speedBR = forward + rotation - strafe;  
  int speedBL = forward - rotation + strafe;  

  // Control motors
  // FR
  controlMotor(F_ENA, F_IN1, F_IN2, speedFR);
  // FL
  controlMotor(F_ENB, F_IN3, F_IN4, speedFL);
  // BR
  controlMotor(B_ENA, B_IN1, B_IN2, speedBR);
  // BL
  controlMotor(B_ENB, B_IN3, B_IN4, speedBL);
}

void setup() {
    
    InitMotorDriverPins();
    PS4.begin("A8:42:E3:49:29:B4");

    /* Initialise the sensor */
    if(!mag.begin())
    {
        /* There was a problem detecting the HMC5883 ... check your connections */
        Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
        while(1);
    }
    Serial.println(F("HMC5883L Connected!..."));
    initHeading = GetHeadingDegrees();

}

void loop() {
    FieldCentric();
}
