//********** Libraries **********//

#include <Servo.h>


//********** Macros **********//

#define NUM_JOINTS  6
#define SERVO_FREQ  60
#define SWEEP_DELAY 30


//********** Globals **********//

int servo_pins[NUM_JOINTS] = {44, 45, 46, 6, 7, 8}; // For Mega only
int min_angles[NUM_JOINTS] = {0, 0, 0, 0, 0, 0};
int max_angles[NUM_JOINTS] = {180, 180, 180, 180, 180, 180};
int def_angles[NUM_JOINTS] = {90, 90, 90, 90, 90, 90};
int temp_angles[NUM_JOINTS] = {0, 0, 0, 0, 0, 0};

typedef struct
{
	int pin;
	int min_angle;
	int max_angle;
	int def_angle;
	int last_angle;
} Joint;


//********** Objects **********//

Servo arm_servo[NUM_JOINTS];
Joint arm_joint[NUM_JOINTS];


//********** Functions **********//

void attachServo(int servo_index)
{
	Serial.print("Attaching Servo "); Serial.println(servo_index);
	arm_servo[servo_index].attach(servo_pins[servo_index]);
	arm_servo[servo_index].write(arm_joint[servo_index].last_angle);
}

void attachServos()
{
	for (int i = 0; i < NUM_JOINTS; i++)
	{
		attachServo(i);
	}
}

void detachServo(int servo_index)
{
	if (arm_servo[servo_index].attached())
	{
		Serial.print("Detaching Servo "); Serial.println(servo_index);
		arm_servo[servo_index].detach();
	}
}

void detachServos()
{
	for (int i = 0; i < NUM_JOINTS; i++)
	{
		detachServo(i);
	}
}

void jointInit()
{
	for (int i = 0; i < NUM_JOINTS; i++)
	{
		arm_joint[i].pin = servo_pins[i];
		arm_joint[i].min_angle = min_angles[i];
		arm_joint[i].max_angle = max_angles[i];
		arm_joint[i].def_angle = def_angles[i];
		arm_joint[i].last_angle = def_angles[i];
	}
}

void armAttach()
{
	Serial.println("\n>>> Attach Arm...");
	attachServos();
	Serial.println("Done");
}

void armDetach()
{
	Serial.println("\n>>> Detach Arm...");
	detachServos();
	Serial.println("Done");
}

void armShutdown()
{
	Serial.println("\n>>> Shutdown Arm...");
	resetServoAngles();
	detachServos();
	Serial.println("Done");
}

void servoSweep(int servo_index)
{
	for (int pos = 0; pos <= 180; pos += 1) 
	{
		arm_servo[servo_index].write(pos);
		delay(SWEEP_DELAY);
	}

	for (int pos = 180; pos >= 0; pos -= 1) 
	{
		arm_servo[servo_index].write(pos);              
		delay(SWEEP_DELAY);                       
	}
}

void setServoAngle(int index, int angle)
{
	if (!arm_servo[index].attached())
	{
		Serial.print("\n\n>>> Joint in detached state: "); Serial.println(index);
		Serial.print('*');
		return;
	}
	
	Serial.print("\n\n>>> Actuating Joint: "); Serial.println(index);

	if (angle > arm_joint[index].last_angle)
	{
		for (int i = arm_joint[index].last_angle; i <= angle; i++)
		{
			int write_angle = map(i, 0, 180, arm_joint[index].min_angle, arm_joint[index].max_angle);
			if ((i % 5 == 0) || (i == angle) || (i == arm_joint[index].last_angle)) 
			{
				Serial.print(i); Serial.print(" ");
			}
			arm_servo[index].write(i);
			delay(SWEEP_DELAY);
		}
	}

	else if (angle < arm_joint[index].last_angle)
	{
		Serial.print(arm_joint[index].last_angle); Serial.print(" ");
		for (int i = arm_joint[index].last_angle; i >= angle; i--)
		{
			int write_angle = map(i, 0, 180, arm_joint[index].min_angle, arm_joint[index].max_angle);
			if ((i % 5 == 0) || (i == angle) || (i == arm_joint[index].last_angle))
			{
				Serial.print(i); Serial.print(" ");
			}
			arm_servo[index].write(i);
			delay(SWEEP_DELAY);
		}
	}

	else Serial.print("No change");

	arm_joint[index].last_angle = angle;
}

void setServoAngles()
{
	for (int i = 0; i < NUM_JOINTS; i++)
	{
		setServoAngle(i, temp_angles[i]);
	}
}

void resetServoAngles()
{
	for (int i = 0; i < NUM_JOINTS; i++)
	{
		setServoAngle(i, def_angles[i]);
	}
}

int getServoAngle(int servo_index)
{
	return arm_servo[servo_index].read();
}

void getServoAngles(int *angles)
{
	for (int i = 0; i < NUM_JOINTS; i++)
	{
		angles[i] = getServoAngle(i);
	}
}

void dispAngles()
{
	Serial.println("\n\n>>> Success!");
	for (int i = 0; i < NUM_JOINTS; i++)
	{
		Serial.print("Servo ");
		Serial.print(i);
		Serial.print(": "); 
		Serial.println(temp_angles[i]);
	}
}

void flushSerialBuffer()
{
	while (Serial.available())
	{
		Serial.read();
	}
}

void serialUpdate()
{
	// Wait for serial data from python script
	if (Serial.available() > 0)
	{
		int frameID = Serial.read();
		Serial.print("\n>>> Frame ID: "); Serial.println(frameID);

		// Parse start limiter
		if (frameID == 200)
		{	
			// Wait for complete packet to parse joint angles
			delay(50);
			if (!Serial.available() >= NUM_JOINTS)
			{
				flushSerialBuffer();
				Serial.print('*');
				return;
			}

			// Now read the angles
			for (int i = 0; i < NUM_JOINTS; i++)
			{
				int angle = Serial.read();
				temp_angles[i] = constrain(angle, 1, 180);
			}

			// Display angles on serial terminal
			dispAngles();

			// Update the servo angle for each servo by passing the servo object
			setServoAngles();
		}

		else if (frameID == 202)
		{
			// Attach servos
			armAttach();
		}

		else if (frameID == 204)
		{
			// Detach servos
			armDetach();
		}

		else if (frameID == 206)
		{
			// Shutdown arm
			armShutdown();
		}

		else if (frameID == 210)
		{
			// sendIMUData();
		}

		else
		{
			// Flush buffer
			flushSerialBuffer();
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
	Serial.begin(9600);
	jointInit();
	Serial.println("\n\nSetup Done");
}

void loop() 
{
	serialUpdate();
}
