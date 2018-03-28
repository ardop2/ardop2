//********** Libraries **********//
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

//********** Macros **********//
#define NUM_SERVOS  3

#define SERVOMIN0   120
#define SERVOMAX0   560
#define SERVOMIN1   150
#define SERVOMAX1   600
#define SERVOMIN2   120
#define SERVOMAX2   560

#define SERVO_FREQ  60
#define SWEEP_DELAY 30

//********** Globals **********//
uint8_t angles[NUM_SERVOS] = {0}; // Initializes all angles to 0

typedef struct
{
  uint8_t channel;
  uint8_t last_angle;
  uint16_t pulse_min;
  uint16_t pulse_max;
} servo_;

//********** Objects **********//
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Create servo object and calibrate the min and max 
// intervals here for 0-180 degrees
/* NOTE: Additional servos need to be added here */
servo_ servo0 = {0, 0, SERVOMIN0, SERVOMAX0};
servo_ servo1 = {1, 0, SERVOMIN1, SERVOMAX1};
servo_ servo2 = {2, 0, SERVOMIN2, SERVOMAX2};

//********** Functions **********//
void blinker()
{
  digitalWrite(13, HIGH);
  delay(200);
  digitalWrite(13, LOW);
  delay(200);
}

void servoInit()
{
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);

  /* NOTE: Additional servos need to be added here */
  setServoAngle(servo0, 1);
  setServoAngle(servo1, 1);
  setServoAngle(servo2, 1);
}

void setServoAngle(servo_ &servo, uint8_t angle)
{
  /* This function will also handle the rotate-from-zero-degress problem
   * by remembering the previous angle position each servo was in */
  
  Serial.print("\n\n>>> Servo Channel: "); Serial.println(servo.channel);

  if (servo.last_angle < angle)
  {
    for (uint8_t i = servo.last_angle; i <= angle; i++)
    {
      uint16_t pulseLen = map(i, 0, 180, servo.pulse_min, servo.pulse_max);
      if (i % 5 == 0) {
        Serial.print(i);
        Serial.print(" ");
      }
      pwm.setPWM(servo.channel, 0, pulseLen);
      delay(SWEEP_DELAY);
    }
  }

  else if (servo.last_angle > angle)
  {
    for (uint8_t i = servo.last_angle; i >= angle; i--)
    {
      uint16_t pulseLen = map(i, 0, 180, servo.pulse_min, servo.pulse_max);
      if (i % 5 == 0) {
        Serial.print(i);
        Serial.print(" ");
      }
      pwm.setPWM(servo.channel, 0, pulseLen);
      delay(SWEEP_DELAY);
    }
  }

  else Serial.print("No change");

  servo.last_angle = angle;
}

void flushPacket()
{
  for (int i = 0; i < 3; i++)
    angles[i] = 0;
}

void dispAngles()
{
  Serial.println("\n\n>>> Success!");
  Serial.print("Servo 0: "); Serial.println(angles[0]);
  Serial.print("Servo 1: "); Serial.println(angles[1]);
  Serial.print("Servo 2: "); Serial.print(angles[2]);
}

void updateServo()
{
  // Read from python script over serial
  if (Serial.available() > 3)
  {
    // Parse start limiter
    if (Serial.read() == 200)
    {
      // Now read the angles
      for (int i = 0; i < NUM_SERVOS; i++)
      {
        angles[i] = Serial.read();
        angles[i] = constrain(angles[i], 1, 180);
      }

      // Display angles on serial terminal
      dispAngles();

      // Update the servo angle for each servo by passing the servo object
      /* NOTE: Additional servos need to be added here */
      setServoAngle(servo0, angles[0]);
      setServoAngle(servo1, angles[1]);
      setServoAngle(servo2, angles[2]);
    }
  }
}

void serialEcho()
{
  if (Serial.available())
    Serial.write(Serial.read());
}

//********** Main Function **********//
void setup()
{
  pinMode(13, OUTPUT);
  Serial.begin(9600);
  servoInit();
}

void loop()
{
  updateServo();
}
