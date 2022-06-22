#include <FalconRobot.h>
/*Falcon Robot Library Variables*/
FalconRobotLineSensor left(A2);
FalconRobotLineSensor right(A3);

// IR Sensors
int leftSensorValue;  // variable to store the left sensor value
int rightSensorValue; // variable to store the right sensor value

#define LimitLumaDifference 700 // Basis for Comparing the Value Read by the IR Sensor
#define DefaultSpeed 50 // Set to any number from 0 - 100.

int flag; // variable to identify the edges of the black line

FalconRobotMotors motors(5, 7, 6, 8);
int leftMotorSpeed;  // variable used to store the leftMotor speed
int rightMotorSpeed; // variable used to store the rightMotor speed

// Default Variables Values
int initial_motor_speed = DefaultSpeed;

int left_motor_speed;  // variable to store left motor speed after PID correcetion
int right_motor_speed; // variable to store right motor speed after PID correcetion

// PID Constants
float Kp = 40;
float Ki = 20;
float Kd = 32;

float error = 0, P = 0, I = 0, D = 0, PID_value = 0; // defines the variables needed to calculate PID
float previous_error = 0, previous_I = 0;            // defines the variables needed to calculate PID

void setup()
{
  Serial.begin(9600); // Setup the Serial to a speed of 9600bps
  delay(1000);
}

void read_sensor_values()
{
  // this was the trickiest part of this whole code. Especially because there are three situations where the sensors are off the line.
  // To overcome this issue what I came up with a flag variable that differentiates when the robot is moving to the right or to the left.
  // Maybe there's a simpler way of doing this but the way it is now is functinal.

  // this function converts the position of the sensors in relation to the black line to an integer between -2 and 2 - inclusive

  //  The output of each of the "if" statements in this function are related to the position of the sensors in relation to the black line.
  //  Refer to the diagram below to understand the meaning of each of the outputs.
  //
  //  ROBOT MOVING LEFT TO RIGHT
  //
  //  LEFT OF   BLACK  RIGHT OF
  //  THE LINE | LINE | THE LINE
  //       |    |
  //    L R | 0 0 |  0  0     error = -2     flag = 0
  //    0 L | R 0 |  0  0     error = -1     flag = 1
  //    0 L | 0 0 |  R  0     error =  0     flag = 2
  //    0 0 | 0 L |  R  0     error =  1     flag = 3
  //    0 0 | 0 0 |  L  R     error =  2     flag = 4
  //       |    |
  //      DIRECTION OF THE
  //       MOVEMENT
  //   ------------------------- >
  //

  //  ROBOT MOVING RIGHT TO LEFT
  //
  //  LEFT OF   BLACK  RIGHT OF
  //  THE LINE | LINE | THE LINE
  //       |    |
  //    L R | 0 0 |  0  0     error = -2     flag = 0
  //    0 L | R 0 |  0  0     error = -1     flag = -1
  //    0 L | 0 0 |  R  0     error =  0     flag = -2
  //    0 0 | 0 L |  R  0     error =  1     flag = -3
  //    0 0 | 0 0 |  L  R     error =  2     flag = 4
  //       |    |
  //      DIRECTION OF THE
  //       MOVEMENT
  //   <-------------------------
  //

  leftSensorValue = left.read();
  rightSensorValue = right.read();

  /*
   ROBOT MOVING FROM LEFT TO RIGHT
  */

  // both sensors are off the line and to the left of it

  if ((leftSensorValue < LimitLumaDifference) && (rightSensorValue < LimitLumaDifference) && flag == -1)
  {
    error = -2.0;
    flag = 0;
  }

  // left sensor off the line and right sensor on the line

  if ((leftSensorValue < LimitLumaDifference) && (rightSensorValue > LimitLumaDifference) && flag == 0)
  {
    error = -1.0;
    flag = 1;
  }

  // one sensor each side of the line

  if ((leftSensorValue < LimitLumaDifference) && (rightSensorValue < LimitLumaDifference) && flag == 1)
  {
    error = 0;
    flag = 2;
  }

  // left sensor on the line right sensor off the line
  if ((leftSensorValue > LimitLumaDifference) && (rightSensorValue < LimitLumaDifference) && flag == 2)
  {
    error = 1.0;
    flag = 3;
  }

  // both sensors off the line and to the right of it

  if ((leftSensorValue < LimitLumaDifference) && (rightSensorValue < LimitLumaDifference) && flag == 3)
  {
    error = 2.0;
    flag = 4;
  }

  /*
   ROBOT MOVING FROM RIGHT TO LEFT
  */

  // left sensor on the line and right sensor off the line

  // sensor esquerdo em cima da linha e sensor direito fora dela
  if ((leftSensorValue > LimitLumaDifference) && (rightSensorValue < LimitLumaDifference) && flag == 4)
  {
    error = 1.0;
    flag = -3;
  }

  // both sensors off the line. One each side of it

  if ((leftSensorValue < LimitLumaDifference) && (rightSensorValue < LimitLumaDifference) && flag == -3) // turn right vindo da esquerda
  {
    error = 0.0;
    flag = -2;
  }

  // left sensor off the line and right sensor on the line

  if ((leftSensorValue < LimitLumaDifference) && (rightSensorValue > LimitLumaDifference) && flag == -2) // turn right vindo da esquerda
  {
    error = -1.0;
    flag = -1;
  }
}

void calculate_pid()
{
  P = error;                  // given by the position of the sensors as defined in "read_sensor_values()"
  I = I + previous_I;         // calculates the cummulative error
  D = error - previous_error; // detects the variation of the error in time

  PID_value = (Kp * P) + (Ki * I) + (Kd * D);

  previous_I = I;
  previous_error = error; // updates the error value
}

void motor_control()
{
  // Calculating the effective motor speed:
  int left_motor_speed = initial_motor_speed - PID_value;
  int right_motor_speed = initial_motor_speed + PID_value;

  // The motor speed should not exceed 100
  left_motor_speed = constrain(left_motor_speed, 0, 100);
  right_motor_speed = constrain(right_motor_speed, 0, 100);

  leftMotorSpeed = left_motor_speed;   // left motor speed after PID correction
  rightMotorSpeed = right_motor_speed; // right motor speed after PID correction
}

void forward()
{
  /*robot drives forward*/
  motors.leftDrive(leftMotorSpeed, FORWARD);
  motors.rightDrive(rightMotorSpeed, FORWARD);
}
void reverse()
{
  /*robot drives in reverse*/
  motors.leftDrive(leftMotorSpeed, BACKWARD);
  motors.rightDrive(rightMotorSpeed, BACKWARD);
}
void Right()
{
  /*robot makes right turn*/
  motors.leftDrive(leftMotorSpeed, FORWARD);
  motors.rightDrive(rightMotorSpeed, FORWARD);
}
void Left()
{
  /*robot makes left turn*/
  motors.leftDrive(leftMotorSpeed, FORWARD);
  motors.rightDrive(rightMotorSpeed, FORWARD);
}
void sharpRightTurn()
{
  /*robot makes a sharp right turn*/
  motors.leftDrive(leftMotorSpeed, FORWARD);
  motors.rightDrive(rightMotorSpeed, BACKWARD);
}
void sharpLeftTurn()
{
  /*robot makes a sharp left turn*/
  motors.leftDrive(leftMotorSpeed, BACKWARD);
  motors.rightDrive(rightMotorSpeed, FORWARD);
}
void stop_bot()
{
  /*stops the robot */
  motors.stop(); // Stops both motors
}

void loop()
{

  if ((leftSensorValue > LimitLumaDifference) && (rightSensorValue > LimitLumaDifference)) // if both sensors are hovering a black line...
  {
    delay(100);                                                      // wait for 100 ms (depending on the speed of the robot and the width of the line this has to be changed)
    if ((leftSensorValue > LimitLumaDifference) && (rightSensorValue > LimitLumaDifference)) // if both sensors are still hovering a black line...
    {
      motors.stop(); // stop motors
    }
  }

  else
  {

    calculate_pid();      // calls the PID function
    motor_control();      // calls motor control function
    read_sensor_values(); // calls the function that gets the readings from both sensors

    /*  DEBUGGING SECTION

        If need be, uncomment this section to read all relevant outputs of this code
        Serial.print(leftSensorValue);
        Serial.print("\t");
        Serial.print(rightSensorValue);
        Serial.print("\t");
        Serial.print(leftMotorSpeed);
        Serial.print("\t");
        Serial.print(rightMotorSpeed);
        Serial.print("\t");
        Serial.print(PID_value);
        Serial.print("\t");
        Serial.print(error);
        Serial.print("\t");
        Serial.println(flag);
    */
    if (error == 2.0) // if both sensors are off the line and they are to the right of the line...
    {
      sharpLeftTurn(); //... bot must do a sharp left turn.
    }

    if (error == 1.0) // if only the left sensor is hovering the line the robot must...
    {

      Left(); //...bot must turn left.
    }

    if (error == 0.0) // if both sensors are off the line and they are placed either sides of the line...
    {

      forward(); //... bot drives forward
    }

    if (error == -1.0) // if only the right sensor is hovering the line...
    {

      Right(); //...the robot must turn right.
    }

    if (error == -2.0) // if both sensors are off the line and they are to the left of the line
    {

      sharpRightTurn(); //...robot must do a sharp right turn
    }

    delay(0); // to decrease sensitivity add a delay
  }
}