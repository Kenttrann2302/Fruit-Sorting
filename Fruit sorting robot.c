// Declare the object
Typedef struct
{
	float x,y;
}Point;

//Functions for object-oriented-programming to get the hypothenuse
void driveToBox(Point const &location, int motor_power, bool dir); // pass by constant reference

// Non-Trivial functions
void configureAllSensor(int Touch, int Color, int Ultrasonic, int Gyro); // configure all the sensors for the robot
void rotate_robot(float angle, int motor_power, bool dir); // this function will rotate the robot
float get_x_value(int color_value, int motor_power); // this function will get the x distance of the robot by using the color sensor value
void Drive_Dist_Color(float x_value, int motor_power) // this function will get the x distance and calculate the distance r
void getToStart(int dist_value, int motor_power); // this function will receive the ultrasonic sensor value to get to the staring point
void returnDist(int color_value, int motor_power); // this function will return the distance to the pickup point

// Functions for rotating others platforms of the robot
void rotate_arm_robot(float angleRadian, int motor_power);
void rotate_gripper_robot(int angle, int motor_power);

// global constant numbers declaration
int const cw_direction = 1;
int const ccw_direction = -1;
float const DIST_GAP = 1.5;
int const m_to_cm = 100;
float const y_value = 1;
const MAX_ANGLE = 180;

// object function for driving to the drop-off point
void driveToBox(Point const &location, float x_value, int motor_power, int color_value, bool dir)
{
	float angle;
	angle = atan2(y_value, x_value);
	rotate_robot(angle, motor_power, dir);
	Drive_Dist_Color();
}

task main()
{



}

void configureAllSensor(int Touch, int Color, int Ultrasonic, int Gyro)
{
	SensorType[] = sensorEV3_Touch; // configure the touch sensor
	wait1Msec(50);
	SensorType[] = sensorEV3_Ultrasonic; // configure the ultrasonic sensor
	wait1Msec(50);
	SensorType[] = sensotEV3_Color; // configure thr color sensor
	wait1MSec(50);
	SensorMode[] = modeEV3Color_Color; // configure the mode for the color sensor
	wait1Msec(100);
	SensorType[] = sensorEV3_Gyro; // configure the gyro sensor
	wait1MSec(50);
	SensorMode[] = modeEV3Gyro_Calibration; // configure the mode for the gyro calibration
	wait1Msec(100);
	SensorMode[] = modeEV3Gyro_RateAndAngle; // configure the mode for the rate and angle of the gyro sensor
	wait1Msec(50);
}

void rotate_robot(float angleRadian, int motor_power, bool dir)
{
	resetGyro(); // put the port for the gyrosensor
	motor[MotorA] = (cw_direction * dir + !dir * ccw_direction) * motor_power; // motor[A] has positive power when rotating clockwise
																																						// motor[A] has neegative power when rotating counter-clockwise
	motor[MotorD] = (ccw_direction * dir + !dir * cw_direction) * motor_power; // motor[D] has a negative power when rotating clockwise
																																						// motor[D] has a positive power when rotating counter-clockwise

	wait1MSec(50);

	while (abs(SensorValue[] < angle) // while the angle did not reach, then keep rotating
	{}

	motor[motorA] = motor[MotorD] = 0;
}

float get_x_value(int color_value, int motor_power) // this function will get the x value
{
		float x_dist = color_value * DIST_GAP * m_to_cm * MAX_ANGLE / (2.75 * PI); // getting the distance on the x axis using the color sensor value
		return x_dist;
}

void Drive_Dist_Color(int x_value, int motor_power) // get the encoder limit value
{
	const int ENC_LIMIT = sqrt(pow(x_value, 2) + pow(y_value, 2));
	nMotorEncoder[MotorA] = 0;
	motor[MotorA] = motor[MotorD] = motor_power;
	while(nMotorEncoder[MotorA] < ENC_LIMIT) // keep driving until the distance
	{}

	motor[MotorA] = motor[MotorD] = 0;
}








