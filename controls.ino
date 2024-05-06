#include <Wire.h>
#include <nRF24L01.h>
#include <printf.h>
#include <RF24.h>
#include <RF24_config.h>
#include <SPI.h>
#include <Servo.h>

// Gyroscope
int timer = 0;
int displayCount = 0;
int gyro_x, gyro_y, gyro_z; // MPU reading in degress/second.
double gyroRot_x, gyroRot_y, gyroRot_z; // Calibrating reading.
double gyroOffset_x, gyroOffset_y, gyroOffset_z; // Calibration offsets.

// Accelerometer
int accel_x, accel_y, accel_z;
double gForce_x, gForce_y, gForce_z;
double accelAngle_x, accelAngle_y;
const double accelOffset_x = 0.00, accelOffset_y = 0.00, accelOffset_z = 0.00; // Precomputed, MPU-specific calibration offsets.

// Motors
const Servo frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
const int minPower = 1100, maxPower = 1500;
int frontLeftMPow, frontRightMPow, backLeftMPow, backRightMPow;
int changeInPower_x, changeInPower_y, changeInPower_z;

// Lighting
const int frontLights = 30, backLights = 31;

// Transiever
const int CE = 7, CSN = 8; // Pin numbers on Arduino Mega.
const RF24 radio = RF24(CE, CSN);
const byte radioBuffer[12];
const byte address[5] = "1chan";
int radioReading = 0;
int throttle = 1100, roll, pitch, yaw, aux1, aux2; // Transmitted inputs.

// Tunable PID gain.
const double PID_gain = 2.0;
const double P_gain = 0.075; // Change in steps of 0.05
const double I_gain = 0.0; // Change in steps of 0.01
const double D_gain = 0.8; // Change in steps of 0.05
double I_roll, I_pitch, I_yaw;

// PID
double xDeg = 0.0, yDeg = 0.0, zDeg = 0.0;
double xGoal = 0.0, yGoal = 0.0, zGoal = 0.0;

// Debug
int printTimer = 0;
int debugTimer = 0;

void setup() 
{
  Serial.begin(9600);
  radio.begin();
  Wire.begin();

  radio.openReadingPipe(1, address);
  radio.startListening();

  pinMode(frontLights, OUTPUT);
  pinMode(backLights, OUTPUT);
  frontLeftMotor.attach(2);
  frontRightMotor.attach(3);
  backLeftMotor.attach(4);
  backRightMotor.attach(5);

  frontLeftMotor.writeMicroseconds(1000);
  frontRightMotor.writeMicroseconds(1000);
  backLeftMotor.writeMicroseconds(1000);
  backRightMotor.writeMicroseconds(1000);
  
  setupMPU();
  calibrateGyro();
  calibrateMPU();

  // Signal setup is complete
  digitalWrite(frontLights, HIGH);
  digitalWrite(backLights, HIGH);

  updateMotorPower();
  
  timer = millis();
}

void loop() 
{
  updateOrientation();
  updateGoalandThrottle();
  doPID();
  powerLoCut();
  powerHiCut();
  updateMotorPower();
  
  /*
  if (printTimer == 100)
  {  
      printPowerToSerial();
      //printAxis(xDeg, yDeg, zDeg);
      //printControls();
      printTimer = -1;
  }
  printTimer++;
  */
  // printExecutionTime();         // Prints the time it took to complete an iteration of the feedback loop.
}

void doPID()
{
  // Reset power levels.
  frontLeftMPow = throttle;
  frontRightMPow = throttle;
  backLeftMPow = throttle;
  backRightMPow = throttle;

  // Compute adjustments.
  changeInPower_x = calculateChangeInPower(xGoal, xDeg, gyroRot_x, I_roll);
  changeInPower_y = calculateChangeInPower(yGoal, yDeg, gyroRot_y, I_pitch);
  changeInPower_z = calculateChangeInPower(zGoal, zDeg, gyroRot_z, I_yaw);

  // Apply adjustments.
  frontLeftMPow += changeInPower_x;
  frontRightMPow += changeInPower_x;
  backLeftMPow -= changeInPower_x;
  backRightMPow -= changeInPower_x;

  frontLeftMPow += changeInPower_y;
  backLeftMPow += changeInPower_y;
  frontRightMPow -= changeInPower_y;
  backRightMPow -= changeInPower_y;

  frontLeftMPow -= changeInPower_z;
  backRightMPow -= changeInPower_z;
  backLeftMPow += changeInPower_z;
  frontRightMPow += changeInPower_z;
}

int calculateChangeInPower(double cur, double goal, double derivative, double integral)
{
  double error = goal - cur;
  double P = error * P_gain;
  double I = integral * I_gain;
  double D = derivative * D_gain;
  return (int)((P + I + D) * PID_gain);
}

void powerLoCut()
{
  if (frontLeftMPow < minPower)
  {
    frontLeftMPow = minPower;
  }

  if (frontRightMPow < minPower)
  {
    frontRightMPow = minPower;
  }

  if (backLeftMPow < minPower)
  {
    backLeftMPow = minPower;
  }

  if (backRightMPow < minPower)
  {
    backRightMPow = minPower;
  }
}

void powerHiCut()
{
  if (frontLeftMPow > maxPower)
  {
    frontLeftMPow = maxPower;
  }

  if (frontRightMPow > maxPower)
  {
    frontRightMPow = maxPower;
  }

  if (backLeftMPow > maxPower)
  {
    backLeftMPow = maxPower;
  }
  
  if (backRightMPow > maxPower)
  {
    backRightMPow = maxPower;
  }
}

void updateMotorPower()
{
  frontLeftMotor.writeMicroseconds(frontLeftMPow);
  frontRightMotor.writeMicroseconds(frontRightMPow);
  backLeftMotor.writeMicroseconds(backLeftMPow);
  backRightMotor.writeMicroseconds(backRightMPow);
}

void updateGoalandThrottle()
{
  if (radio.available())
  {
    radio.read(&radioBuffer, sizeof(radioBuffer));

    throttle = radioBuffer[0] << 8 | radioBuffer[1]; 
    pitch = radioBuffer[2] << 8 | radioBuffer[3];
    roll = radioBuffer[4] << 8 | radioBuffer[5];
    yaw = radioBuffer[6] << 8 | radioBuffer[7];
    aux1 = radioBuffer[8] << 8 | radioBuffer[9];
    aux2 = radioBuffer[10] << 8 | radioBuffer[11];

    xGoal = mapDouble(pitch, 1000, 2000, -90, 90);
    yGoal = -mapDouble(roll, 1000, 2000, -90, 90);
  }
  else
  {
    //Serial.println("NO SIGNAL");
  }
}

double mapDouble(double val, double fromMin, double fromMax, double toMin, double toMax)
{
  double fromRange = fromMax - fromMin;
  double toRange = toMax - toMin;
  return (val - fromMin) / fromRange * toRange + toMin;
}

//Reads data from MPU and refreshes the orientation data. Uses complementary filter.
void updateOrientation()
{
  readMPU();
  calculateAccel();
  calculateGyro();

  int changeInMillis = millis() - timer; //Calculate time elapsed
  timer = millis();                      //Reset timer for the following reading

  double gyroDeg_x = xDeg + (gyroRot_x * changeInMillis / 1000);
  double gyroDeg_y = yDeg + (gyroRot_y * changeInMillis / 1000);

  xDeg = (0.98 * gyroDeg_x) + (0.02 * accelAngle_x);
  yDeg = (0.98 * gyroDeg_y) + (0.02 * accelAngle_y);
  zDeg += (gyroRot_z * changeInMillis / 1000);

  I_pitch += xDeg;
  I_roll += yDeg;
  I_yaw += zDeg;
}

void readMPU()
{
  Wire.beginTransmission(0x68); //Address of the MPU
  Wire.write(0x3B);             //Address of the first accerometer measurement byte
  Wire.requestFrom(0x68, 14);   //Request the 6 bytes of acceleromter readings
  Wire.endTransmission();


  // All data is in little-endian order.
  int byte1 = Wire.read(); //accel x-axis
  int byte2 = Wire.read();
  int byte3 = Wire.read(); //accel y-axis
  int byte4 = Wire.read();
  int byte5 = Wire.read(); //accel z-axis
  int byte6 = Wire.read();

  Wire.read(); // Tempurature, not relevant.
  Wire.read();

  int byte9 = Wire.read(); //gyro x-axis
  int byte10 = Wire.read();
  int byte11 = Wire.read(); //gyro y-axis
  int byte12 = Wire.read();
  int byte13 = Wire.read(); //gyro z-axis
  int byte14 = Wire.read();

  accel_x = (byte1 << 8) | byte2;
  accel_y = (byte3 << 8) | byte4;
  accel_z = (byte5 << 8) | byte6;

  gyro_x = (byte9 << 8) | byte10; 
  gyro_y = (byte11 << 8) | byte12;
  gyro_z = (byte13 << 8) | byte14;
}

void calculateAccel()
{
  gForce_x = (accel_x / 4096.0);
  gForce_y = (accel_y / 4096.0);
  gForce_z = (accel_z / 4096.0);
  
  accelAngle_x = (180 * atan(gForce_y / gForce_z) / 3.1415) - accelOffset_x;
  //Negative added for agreement between acceleromter and gyro angle measurements
  accelAngle_y = -1 * ((180 * atan(gForce_x / gForce_z) / 3.1415) - accelOffset_y);
}

void calculateGyro()
{
  gyroRot_x = (gyro_x / 32.8) - gyroOffset_x;
  gyroRot_y = (gyro_y / 32.8) - gyroOffset_y;
  gyroRot_z = (gyro_z / 32.8) - gyroOffset_z; 
}

void calibrateGyro()
{
  double totalx = 0.0;
  double totaly = 0.0;
  double totalz = 0.0;

  for (int i = 0; i < 10; i++)
  {
    for (int j = 0; j < 100; j++)
    {
      readMPU();
      calculateGyro();
      totalx += gyroRot_x;
      totaly += gyroRot_y;
      totalz += gyroRot_z;
    }
  }

  //Record average reading as the calibration value
  gyroOffset_x = totalx / 1000;
  gyroOffset_y = totaly / 1000;
  gyroOffset_z = totalz / 1000;
} 

void calibrateMPU()
{
  double angleTotalX = 0.0;
  double angleTotalY = 0.0;
  
  for (int i = 0; i < 1000; i++)
  {
      readMPU();
      calculateAccel();
      angleTotalX += accelAngle_x;
      angleTotalY += accelAngle_y;
  }

  xDeg = angleTotalX / 1000;
  yDeg = angleTotalY / 1000;
  zDeg = 0.0;
}

void setupMPU()
{
  //Turn off sleep mode.
  Wire.beginTransmission(0x68); // Address of the MPU 6050. Write bit automatically added.
  Wire.write(0x6b);             // Address of the power management byte.
  Wire.write(0b00001000);       // Turn off sleep mode and set the clock source to the internal 8mz clock
  Wire.endTransmission();

  // Gyroscope
  Wire.beginTransmission(0x68);
  Wire.write(0x1b);             // Address of the gyroscope configuration byte
  Wire.write(0b00010000);       // Sets the full-scale range of the gyroscope to +-500 degrees/second
  Wire.endTransmission();

  // Accelerometer
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);             // Address for acceleromter configuration.
  Wire.write(0b00010000);       // Set full scale range to +-8g
  Wire.endTransmission();
}

void printPowerToSerial()
{
  Serial.print("Front-left: ");
  Serial.print(frontLeftMPow);
  Serial.print("   ");
  Serial.print("Front-right: ");
  Serial.print(frontRightMPow);
  Serial.println();
  Serial.print("Back-left: ");
  Serial.print(backLeftMPow);
  Serial.print("    ");
  Serial.print("Back-right: ");
  Serial.print(backRightMPow);
  Serial.println();
  Serial.print("Throttle: ");
  Serial.println(throttle);
  Serial.print("xDeg: ");
  Serial.println(xDeg);
  Serial.print("yDeg: ");
  Serial.println(yDeg);
  Serial.print("zDeg: ");
  Serial.println(zDeg);
  Serial.println();
}

void printAxis(double x, double y, double z)
{
  Serial.println("Axis");
  Serial.print("x: ");
  Serial.println(x);
  Serial.print("y: ");
  Serial.println(y);
  Serial.print("z: ");
  Serial.println(z);
}

void printExecutionTime()
{
  int deltaT = micros() - debugTimer;
  Serial.println(deltaT);
  debugTimer = micros();
}

void printControls()
{
  printPWM("Throttle: ", throttle);
  printPWM("Pitch: ", pitch);
  printPWM("Roll: ", roll);
  printPWM("Yaw: ", yaw);
  printPWM("Aux_1: ", aux1);
  printPWM("Aux_2: ", aux2);
  Serial.println();
}

void printPWM(String pinLabel, int val)
{
  Serial.print(pinLabel);
  Serial.println(val);
}

