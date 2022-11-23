// functions prototypes
// function to configure all the sensors
void configureAllSensor(int Touchport, int Ultrasonicport, int Colorport, int Gyroport); // configure all the sensors for the robot

// non-trivial functions
void rotate_robot_CW(float angleRadian, int motor_power); // this function will rotate the robot clockwise
void rotate_robot_CCW(float angleRadian, int motor_power); // this function will rotate the robot counter-clockwise
void Drive_Dist_Color(int color_value, int motor_power); // this function will get the x distance and calculate the distance
void Drive_Dist(float distance, int motor_power); // this function will get the distance to drive
void getToStart(int motor_power); // this function will receive the ultrasonic sensor value to get to the staring point
float drive_to_wall(int motor_power); // this function will drive the robot to the wall to get the x value
float drive_to_pick_up(int motor_power); // this function will drive the robot to the pick up spot
void returnDist(int color_value, int motor_power); // this function will return the distance to the pickup point

// Functions for rotating others platforms of the robot
void rotate_arm_robot_CW(float angle, int motor_power);
void rotate_arm_robot_CCW(float angle, int motor_power);
void rotate_arm_robot_downward(float angle, int motor_power);

// declare variables const for sensors and motors
const int Touchport = S1;
const int Ultrasonicport = S2;
const int Colorport = S3;
const int Gyroport = S4;

// global constant numbers declaration
int const cw_direction = 1;
int const ccw_direction = -1;
int const dist_to_door = 0.4;
int const wall_gap = 0.5;
float const DIST_GAP = 0.5;
int const m_to_cm = 100;
float const y_value = 1;
int const MAX_ANGLE = 180;

task main()
{

	// configure all the sensors
	configureAllSensor(S1,S2,S3,S4); /* S1 -> Touch, S2 -> Ultrasonic, S3 -> Color, S4 -> Gyro */

	// declare all the constants
	int const motor_power = 30;
	int const rotate_power = 20;

	// declare the variables
	float to_wall_dist = 0;
	float to_box_dist = 0;

	// begin the program
	getToStart(motor_power);
	to_wall_dist = drive_to_wall(motor_power);
	rotate_robot_CW(PI/2, rotate_power);
	to_box_dist = drive_to_pick_up(motor_power);
}

// configure all the sensors
void configureAllSensor(int Touchport, int Ultrasonicport, int Colorport, int Gyroport)
{
	SensorType[S1] = sensorEV3_Touch; // configure the touch sensor
	wait1Msec(50);
	SensorType[S2] = sensorEV3_Ultrasonic; // configure the ultrasonic sensor
	wait1Msec(50);
	SensorType[S3] = sensorEV3_Color; // configure the color sensor
	wait1MSec(50);
	SensorMode[S3] = modeEV3Color_Color; // configure the mode for the color sensor
	wait1MSec(100);
	SensorType[S4] = sensorEV3_Gyro; // configure the gyro sensor
	wait1MSec(50);
	SensorMode[S4] = modeEV3Gyro_Calibration; // configure the mode for the gyro calibration
	wait1Msec(100);
	SensorMode[S4] = modeEV3Gyro_RateAndAngle; // configure the mode for the rate and angle of the gyro sensor
	wait1Msec(50);
}

// rotate the robot clockwise
void rotate_robot_CW(float angleRadian, int motor_power)
{
	resetGyro(S4); // put the port for the gyrosensor
	motor[motorA] = cw_direction * motor_power; // motor[A] has positive power when rotating clockwise
																																						// motor[A] has neegative power when rotating counter-clockwise
	motor[motorD] = ccw_direction * motor_power; // motor[D] has a negative power when rotating clockwise
																																						// motor[D] has a positive power when rotating counter-clockwise

	wait1MSec(50);

	while (abs(SensorValue[S4]) < angleRadian*MAX_ANGLE/PI) // while the angle did not reach, then keep rotating
	{}

	motor[motorA] = motor[motorD] = 0;
}

// rotate the robot counter-clockwise
void rotate_robot_CCW(float angleRadian, int motor_power)
{
	resetGyro(S4);

	motor[motorA] = ccw_direction * motor_power;

	motor[motorD] = cw_direction * motor_power;

	wait1Msec(50);

	while(abs(SensorValue[S4]) < angleRadian*MAX_ANGLE/PI)
	{}

	motor[motorA] = motor[motorD] = 0;
}

void Drive_Dist_Color(int color_value, int motor_power) // drive the x distance towards the color box
{
	// One a day baggage cart sorter
	const int ENC_LIMIT = DIST_GAP*color_value*MAX_ANGLE/(PI*0.275); // ENC Limit is in meter
	nMotorEncoder[motorA] = 0;
	motor[motorA] = motor[motorD] = motor_power;
	while(nMotorEncoder[motorA] < ENC_LIMIT) // keep driving until the distance
	{}

	motor[motorA] = motor[motorD] = 0;
}

void Drive_Dist(float distance, int motor_power)
{
	const int ENC_LIMIT = distance*MAX_ANGLE/(0.275*PI);
	nMotorEncoder[motorA] = 0;
	motor[motorA] = motor[motorD] = motor_power;
	while(nMotorEncoder[motorA] < ENC_LIMIT)
	{}
	motor[motorA] = motor[motorD] = 0;
}

void getToStart(int motor_power)
{
	// rotate 90 degrees counter-clockwise
	rotate_robot_CCW(PI/2, motor_power); // rotate the robot counter-clockwise direction
	// wait for the door to open
	wait1Msec(3000); // assume the door will open in 3 seconds
	motor[motorA] = motor[motorD] = 0;
	while(abs(SensorValue[S2]) < dist_to_door)
	{}
	motor[motorA] = motor[motorD] = motor_power; // drive forward until reach the starting point
	while(abs(SensorValue[S3]) == int(colorBlue)) // keep driving until not seeing brown color anymore
	{}
	motor[motorA] = motor[motorD] = 0;
	rotate_robot_CCW(PI/2, motor_power); // now at the starting point
}

// drive towards the wall
float drive_to_wall(int motor_power)
{
	motor[motorA] = motor[motorD] = motor_power;
	nMotorEncoder[motorA] = 0;
	while(abs(SensorValue[S2]) > wall_gap)
	{}
	float ENC_VALUE = nMotorEncoder[motorA];
	motor[motorA] = motor[motorD] = 0;
	return ENC_VALUE*(0.275*PI)/MAX_ANGLE;// return the value of the encoder for the x value
	// rotate the robot
}

// drive towards the box
float drive_to_pick_up(int motor_power)
{
	motor[motorA] = motor[motorD] = motor_power;
	nMotorEncoder[motorD] = 0;
	while(abs(SensorValue[S2]) > dist_to_door)
	{}
	motor[motorA] = motor[motorD] = 0;
	float ENC_VALUE = nMotorEncoder[motorD];
	return ENC_VALUE*(0.275*PI)/MAX_ANGLE;
}

// rotate the arm of the robot clockwise
void rotate_arm_robot_CW(float angle, int motor_power)
{
	motor[motorB] = motor_power; // turn on the motor_power to rotate the arm of the robot
	nMotorEncoder[motorB] = 0;
	while(nMotorEncoder[motorB] < angle) // keep looking for fruit until the color sensor
																																															// detects the colors
	{}
	motor[motorB] = 0;
	wait1Msec(2000);
}

// rotate the arm of the robot counter-clockwise
void rotate_arm_robot_CCW(float angle, int motor_power)
{
	motor[motorB] = (-1) * motor_power;
	nMotorEncoder[motorB] = 0;
	while(nMotorEncoder[motorB] > angle)
	{}
	motor[motorB] = 0;
	wait1Msec(2000);
}

// rotate the robot's arm vertically
void rotate_arm_robot_downward(float angle, int motor_power)
{
	motor[motorE] = motor_power;
	nMotorEncoder[motorE] = 0;
	while(nMotorEncoder[motorE] < angle)
	{}
	motor[motorE] = 0;
}
