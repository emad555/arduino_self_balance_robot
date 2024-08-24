#include <MPU6050.h>         // Library for MPU6050 sensor
#include <Wire.h>            // I2C communication library
#include <MsTimer2.h>        // Library for Timer2 interrupt handling

// Motor driver TB6612 pin definitions
const int motorRightFwd = 8;
const int motorRightRev = 12;
const int motorRightPWM = 10;
const int motorLeftFwd = 7;
const int motorLeftRev = 6;
const int motorLeftPWM = 9;

/////////////////////// Tilt Angle Variables ////////////////////////////
float tiltAngleX;  // Calculated tilt angle around X-axis
float tiltAngleY;  // Calculated tilt angle around Y-axis
float targetAngle = 1;  // Desired angle (ideally 0 degrees)
float angularVelX, angularVelY, angularVelZ;  // Angular velocities from gyroscope
/////////////////////// Tilt Angle Variables ////////////////////////////

/////////////////////// Kalman Filter Variables ////////////////////////
float gyroNoise = 0.001;  // Gyroscope noise covariance
float gyroDriftNoise = 0.003;  // Gyroscope drift noise covariance
float accelNoise = 0.5;  // Accelerometer noise covariance
char kalmanGain = 1;
float deltaTime = 0.005;  // Sampling time for the filter
float kalmanFactor = 0.05;  // Factor for Kalman gain to calculate deviation
float kalmanState_0, kalmanState_1, temp_0, temp_1;
float angleError;
float gyroBias;  // Gyroscope drift bias

float accelZ = 0;  // Renamed to avoid conflict with accelZ_int
float currentAngle;
float filteredAngleY;
float angularSpeed;

float covarianceDot[4] = { 0, 0, 0, 0 };
float covarianceMatrix[2][2] = { { 1, 0 }, { 0, 1 } };
float kalmanTemp_0, kalmanTemp_1, estimationError;
////////////////////// Kalman Filter Variables ////////////////////////

////////////////////// PID Controller Variables ///////////////////////
double angleKp = 28, angleKi = 0.05, angleKd = 0.7;  // PID parameters for angle control
double speedKp = 3.7, speedKi = 0.080, speedKd = 0;  // PID parameters for speed control
double desiredAngle = 0;  // Target angle for balance
int pidOutput;  // PID controller output for angle
float pwmMotor1 = 0, pwmMotor2 = 0;

////////////////// Interrupt-based Speed Measurement //////////////////
#define leftEncoderPin 2  // Pin for left motor encoder (interrupt enabled)
#define rightEncoderPin 3  // Pin for right motor encoder (interrupt enabled)
volatile long rightEncoderCount = 0;  // Count pulses from right motor encoder
volatile long leftEncoderCount = 0;  // Count pulses from left motor encoder
int speedCounter = 0;
////////////////////// Pulse Calculation Variables ////////////////////
int leftPulse = 0;
int rightPulse = 0;
int totalRightPulse = 0;
int totalLeftPulse = 0;
int pulseRight, pulseLeft;
//////////////////////////////// PI Control Variables //////////////////////////
float oldSpeedFilter = 0;
float position = 0;
int piControlFlag;
double piControlOutput;
int cycleCount;
int speedOutput;
float speedFilter;

MPU6050 imuSensor;  // Create an MPU6050 object for IMU sensor
int16_t accelX, accelY, accelZ_int, gyroX, gyroY, gyroZ;  // Variables to hold accelerometer and gyroscope readings


void setup() 
{
  // Configure motor control pins as OUTPUT
  pinMode(motorRightFwd, OUTPUT);
  pinMode(motorRightRev, OUTPUT);
  pinMode(motorLeftFwd, OUTPUT);
  pinMode(motorLeftRev, OUTPUT);
  pinMode(motorRightPWM, OUTPUT);
  pinMode(motorLeftPWM, OUTPUT);

  // Initialize motor state
  digitalWrite(motorRightFwd, HIGH);
  digitalWrite(motorRightRev, LOW);
  digitalWrite(motorLeftFwd, LOW);
  digitalWrite(motorLeftRev, HIGH);
  analogWrite(motorRightPWM, 0);
  analogWrite(motorLeftPWM, 0);

  pinMode(leftEncoderPin, INPUT);  // Configure left encoder pin as INPUT
  pinMode(rightEncoderPin, INPUT);  // Configure right encoder pin as INPUT

  // Attach interrupts to encoder pins
  attachInterrupt(digitalPinToInterrupt(leftEncoderPin), updateLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncoderPin), updateRightEncoder, CHANGE);

  // Initialize I2C communication
  Wire.begin();
  Serial.begin(9600);  // Initialize serial communication at 9600 baud rate
  delay(1500);
  imuSensor.initialize();  // Initialize MPU6050 sensor
  delay(2);

  // Configure Timer2 to trigger an interrupt every 5ms
  MsTimer2::set(5, timerInterruptHandler);
  MsTimer2::start();  // Start Timer2 interrupt
}

void loop() 
{
  Serial.println(currentAngle);
  delay(100);
}

///////////////////// Encoder Update Functions ///////////////////////
// Update left motor encoder count
void updateLeftEncoder() 
{
  leftEncoderCount++;
} 
// Update right motor encoder count
void updateRightEncoder() 
{
  rightEncoderCount++;
} 
//////////////////// Pulse Calculation Function //////////////////////
void calculatePulses()
{
  leftPulse = leftEncoderCount;  // Copy left encoder count
  rightPulse = rightEncoderCount;

  leftEncoderCount = 0;  // Reset encoder counts
  rightEncoderCount = 0;

  totalLeftPulse = leftPulse;
  totalRightPulse = rightPulse;

  if ((pwmMotor1 < 0) && (pwmMotor2 < 0))  // If both motors are running backward, invert pulse counts
  {
    totalRightPulse = -totalRightPulse;
    totalLeftPulse = -totalLeftPulse;
  }
  else if ((pwmMotor1 > 0) && (pwmMotor2 > 0))  // If both motors are running forward, pulse counts remain positive
  {
    totalRightPulse = totalRightPulse;
    totalLeftPulse = totalLeftPulse;
  }
  else if ((pwmMotor1 < 0) && (pwmMotor2 > 0))  // If motors are turning left, invert left pulse count
  {
    totalRightPulse = totalRightPulse;
    totalLeftPulse = -totalLeftPulse;
  }
  else if ((pwmMotor1 > 0) && (pwmMotor2 < 0))  // If motors are turning right, invert right pulse count
  {
    totalRightPulse = -totalRightPulse;
    totalLeftPulse = totalLeftPulse;
  }

  // Accumulate pulse counts every 5ms
  pulseRight += totalRightPulse;
  pulseLeft += totalLeftPulse;
}

///////////////////////////////// Timer Interrupt Handler ////////////////////////////
void timerInterruptHandler()
{
  sei();  // Enable global interrupts
  calculatePulses();  // Calculate pulse counts
  imuSensor.getMotion6(&accelX, &accelY, &accelZ_int, &gyroX, &gyroY, &gyroZ);  // Get IMU sensor data
  computeAngle(accelX, accelY, accelZ_int, gyroX, gyroY, gyroZ, deltaTime, gyroNoise, gyroDriftNoise, accelNoise, kalmanGain, kalmanFactor);  // Compute tilt angle using Kalman filter
  applyPDControl();  // Apply PD control for angle stability
  applyMotorPWM();

  cycleCount++;
  if (cycleCount >= 8)  // Every 40ms, execute speed PI control
  {
    applyPIControl();   
    cycleCount = 0;  // Reset cycle count
  }
}
///////////////////////////////////////////////////////////

///////////////////////////// Angle Calculation Function ///////////////////////
void computeAngle(int16_t accelX, int16_t accelY, int16_t accelZ_int, int16_t gyroX, int16_t gyroY, int16_t gyroZ, float deltaTime, float gyroNoise, float gyroDriftNoise, float accelNoise, float kalmanGain, float kalmanFactor)
{
  float calculatedAngle = -atan2(accelY , accelZ_int) * (180/ PI);  // Calculate angle using arctan and correct sign
  angularVelX = -gyroX / 131;  // Calculate angular velocity from gyroscope data
  Kalman_Filter(calculatedAngle, angularVelX);  // Apply Kalman filter to smooth the angle calculation

  float angleFromAccel = -atan2(accelX, accelZ_int) * (180 / PI);  // Calculate angle from X-axis accelerometer data
  angularVelY = -gyroY / 131.00;  // Calculate angular velocity for Y-axis
  applyFirstOrderFilter(angleFromAccel, angularVelY);  // Apply first-order filter for Y-axis angle
}
////////////////////////////////////////////////////////////////

/////////////////////////////// Kalman Filter Implementation /////////////////////
void Kalman_Filter(double measuredAngle, double measuredGyro)
{
  currentAngle += (measuredGyro - gyroBias) * deltaTime;  // Prior estimate for angle
  angleError = measuredAngle - currentAngle;
  
  covarianceDot[0] = gyroNoise - covarianceMatrix[0][1] - covarianceMatrix[1][0];  // Update covariance matrix
  covarianceDot[1] = -covarianceMatrix[1][1];
  covarianceDot[2] = -covarianceMatrix[1][1];
  covarianceDot[3] = gyroDriftNoise;
  
  covarianceMatrix[0][0] += covarianceDot[0] * deltaTime;  // Integrate covariance matrix
  covarianceMatrix[0][1] += covarianceDot[1] * deltaTime;
  covarianceMatrix[1][0] += covarianceDot[2] * deltaTime;
  covarianceMatrix[1][1] += covarianceDot[3] * deltaTime;
  
  // Compute intermediate variables for matrix operations 
  kalmanTemp_0 = kalmanGain * covarianceMatrix[0][0];
  kalmanTemp_1 = kalmanGain * covarianceMatrix[1][0];
  
  estimationError = accelNoise + kalmanGain * kalmanTemp_0;
  
  kalmanState_0 = kalmanTemp_0 / estimationError;
  kalmanState_1 = kalmanTemp_1 / estimationError;
  
  temp_0 = kalmanTemp_0;  // Store temporary results for matrix operations
  temp_1 = kalmanGain * covarianceMatrix[0][1];
  
  covarianceMatrix[0][0] -= kalmanState_0 * temp_0;  // Update posterior covariance matrix
  covarianceMatrix[0][1] -= kalmanState_0 * temp_1;
  covarianceMatrix[1][0] -= kalmanState_1 * temp_0;
  covarianceMatrix[1][1] -= kalmanState_1 * temp_1;
  
  gyroBias += kalmanState_1 * angleError;  // Update gyro bias
  angularSpeed = measuredGyro - gyroBias;  // Optimal angular speed after Kalman filtering
  currentAngle += kalmanState_0 * angleError;  // Update angle estimate
}

///////////////////// First-Order Filter ///////////////////
void applyFirstOrderFilter(float measuredAngle, float measuredGyro)
{
  filteredAngleY = kalmanFactor * measuredAngle + (1 - kalmanFactor) * (filteredAngleY + measuredGyro * deltaTime);
}

void applyPDControl() {
  // PD control for maintaining balance without movement
  pidOutput = angleKp * (currentAngle + targetAngle) + angleKd * angularSpeed;
  // Balance the PWM output for both motors
  pwmMotor1 = -pidOutput;
  pwmMotor2 = -pidOutput;
}

////////////////// Speed PI Control ////////////////////
void applyPIControl()
{
  float vehicleSpeed = (pulseLeft + pulseRight) * 1.0;  // Calculate speed from pulse counts
  pulseRight = pulseLeft = 0;  // Reset pulse counts
  oldSpeedFilter *= 0.7;  // Apply complementary filter
  speedFilter = oldSpeedFilter + vehicleSpeed * 0.3;
  oldSpeedFilter = speedFilter;
  position += speedFilter;
  position = constrain(position, -3550, 3550);  // Anti-windup for PI control
  piControlOutput = speedKi * (desiredAngle - position) + speedKp * (desiredAngle - speedFilter);  // PI control for speed
}
////////////////// Speed PI Control ////////////////////

//////////////////////////// Apply Motor PWM ///////////////////////////////
void applyMotorPWM()
{
  pwmMotor2 = -pidOutput - piControlOutput;  // Compute final PWM for motor control
  pwmMotor1 = -pidOutput - piControlOutput;
  
  if (pwmMotor1 > 255)  // Limit PWM values to 255
  {
    pwmMotor1 = 255;
  }
  if (pwmMotor1 < -255) 
  {
    pwmMotor1 = -255;
  }
  if (pwmMotor2 > 255)
  {
    pwmMotor2 = 255;
  }
  if (pwmMotor2 < -255)
  {
    pwmMotor2 = -255;
  }

  if (currentAngle > 80 || currentAngle < -80)  // Stop motors if the tilt angle is too steep
  {
    pwmMotor1 = pwmMotor2 = 0;
  }

  if (pwmMotor2 >= 0)  // Set motor direction and speed based on PWM values
  {
    digitalWrite(motorLeftFwd, LOW);
    digitalWrite(motorLeftRev, HIGH);
    analogWrite(motorLeftPWM, pwmMotor2);
  }
  else
  {
    digitalWrite(motorLeftFwd, HIGH);
    digitalWrite(motorLeftRev, LOW);
    analogWrite(motorLeftPWM, -pwmMotor2);
  }

  if (pwmMotor1 >= 0)
  {
    digitalWrite(motorRightFwd, LOW);
    digitalWrite(motorRightRev, HIGH);
    analogWrite(motorRightPWM, pwmMotor1);
  }
  else
  {
    digitalWrite(motorRightFwd, HIGH);
    digitalWrite(motorRightRev, LOW);
    analogWrite(motorRightPWM, -pwmMotor1);
  }
}
