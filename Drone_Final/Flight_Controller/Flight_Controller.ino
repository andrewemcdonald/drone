#include <Wire.h> //Include I2C library
#include <nRF24L01.h> //Include radio driver libraries
#include <printf.h>
#include <RF24.h>
#include <RF24_config.h>
#include <SPI.h>
#include <Servo.h> //Inlclude servo library

/*
 * TODO:
 * 
 * 1. PID Tuning
 * 
 * 2. Solve the inconsitant radio
 * 
 * 3. Build a top covor
 * 
 */

//Variables for the gyro interface
int timer = 0;
int displayCount = 0; //Counts gyro readings made
int gyro_x, gyro_y, gyro_z; //Universal variables that store the gyro readings
double gyroRot_x, gyroRot_y, gyroRot_z; //Stores the final, calibrated rotational velocity values in degrees/second
double gyroOffset_x, gyroOffset_y, gyroOffset_z; //Calibration values

//Accel interface variables
int accel_x, accel_y, accel_z;
double gForce_x, gForce_y, gForce_z;
double accelAngle_x, accelAngle_y;
double accelOffset_x = 0.00, accelOffset_y = 0.00, accelOffset_z = 0.00; //Predetermined constants for this specific MPU

//Motors, LEDs, ect.
Servo frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
int frontLights = 30, backLights = 31;

int frontLeftMPow, frontRightMPow, backLeftMPow, backRightMPow;
int changeInPower_x, changeInPower_y, changeInPower_z;
int minPower = 1100, maxPower = 1500;

//NRF24L01 Radio communication variables
int CE = 7, CSN = 8; //Setup pin numbers MEGA ONLY!!!
RF24 radio = RF24(CE, CSN); //Make radio driver object
byte radioBuffer[12];
byte address[5] = {'1', 'c', 'h', 'a', 'n'};
int radioReading = 0;
int throttle = 1100, roll, pitch, yaw, aux1, aux2; //Represent raw values transmitted from the com. module

//PID gain variables. Can be tuned to adjust microcontroller behavior.
double PID_gain = 2.0;
double P_gain = 0.075; //Change in steps of 0.05
double I_gain = 0.0; //Change in steps of 0.01
double D_gain = 0.8; //Change in steps of 0.05
double I_roll, I_pitch, I_yaw;

//Other Program Variables
double xDeg = 0.0, yDeg = 0.0, zDeg = 0.0;
double xGoal = 0.0, yGoal = 0.0, zGoal = 0.0;
int printTimer = 0;
int debugTimer = 0;

/*
 * ------------------------------------CODE BEGINS HERE!!!!!!!!!----------------------------------------
 */
 
void setup() 
{
  Serial.begin(9600);       //Start serial
  radio.begin();            //Start radio-com
  Wire.begin();             //Start I2C

  //Setup radio
  radio.openReadingPipe(1, address);
  radio.startListening();

  //Setup the ESCs and lights
  pinMode(frontLights, OUTPUT);
  pinMode(backLights, OUTPUT);
  frontLeftMotor.attach(2);
  frontRightMotor.attach(3);
  backLeftMotor.attach(4);
  backRightMotor.attach(5);

  //Starting up the ESCs
  frontLeftMotor.writeMicroseconds(1000);
  frontRightMotor.writeMicroseconds(1000);
  backLeftMotor.writeMicroseconds(1000);
  backRightMotor.writeMicroseconds(1000);
  
  //Setup the motion processor and report initial orientation
  setupMPU();
  calibrateGyro();
  calibrateMPU();

  //Turn on lights to signal setup is complete
  digitalWrite(frontLights, HIGH);
  digitalWrite(backLights, HIGH);

  //Start up motors
  updateMotorPower();
  
  timer = millis(); //Ensures that the gyro value doesn't start really far off on the first reading
}

void loop() 
{
  updateOrientation();          //Uses complementary filter and gyro to recalculate orientation
  updateGoalandThrottle();      //Gets most recent remote control values
  doPID();                      //Uses PID to adjust motor power values
  powerLoCut();                 //Sets any motor power value(s) below minPower to minPower
  powerHiCut();                 //Sets any motor power value(s) above maxPower to maxPower
  updateMotorPower();           //Sends new motor power values to the ESCs
  
  //For debugging
  if (printTimer == 100)
  {  
      printPowerToSerial();
      //printAxis(xDeg, yDeg, zDeg);
      //printControls();
      printTimer = -1;
  }
  printTimer++;
  
  //printExecutionTime();         //Prints the time it took to complete an iteration of the loop
}

void doPID()
{
  //Reset the power levels to start fresh calculations
  frontLeftMPow = throttle;
  frontRightMPow = throttle;
  backLeftMPow = throttle;
  backRightMPow = throttle;

  //Calculate change in power values along each axis
  changeInPower_x = calculateChangeInPower(xGoal, xDeg, gyroRot_x, I_roll);
  changeInPower_y = calculateChangeInPower(yGoal, yDeg, gyroRot_y, I_pitch);
  changeInPower_z = calculateChangeInPower(zGoal, zDeg, gyroRot_z, I_yaw);

  //Changes power levels along x-axis
  frontLeftMPow += changeInPower_x;
  frontRightMPow += changeInPower_x;
  backLeftMPow -= changeInPower_x;
  backRightMPow -= changeInPower_x;

  //Changes power levels along y-axis
  frontLeftMPow += changeInPower_y;
  backLeftMPow += changeInPower_y;
  frontRightMPow -= changeInPower_y;
  backRightMPow -= changeInPower_y;

  //Changes power levels along z-axis
  frontLeftMPow -= changeInPower_z;
  backRightMPow -= changeInPower_z;
  backLeftMPow += changeInPower_z;
  frontRightMPow += changeInPower_z;
}

int calculateChangeInPower(double cur, double goal, double derivative, double integral)
{
  double error = goal - cur;                     //Calculate difference between goal and current orientations
  double P = error * P_gain;                     //Calculate proportion correction
  double I = integral * I_gain;                  //Calculate integral correction
  double D = derivative * D_gain;                //Calculate derivate correction
  return (int)((P + D) * PID_gain);              //Return change in power
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
    radio.read(&radioBuffer, sizeof(radioBuffer)); //Initial read from buffer

    //Reads bytes into raw integer values
    throttle = radioBuffer[0] << 8 | radioBuffer[1]; 
    pitch = radioBuffer[2] << 8 | radioBuffer[3];
    roll = radioBuffer[4] << 8 | radioBuffer[5];
    yaw = radioBuffer[6] << 8 | radioBuffer[7];
    aux1 = radioBuffer[8] << 8 | radioBuffer[9];
    aux2 = radioBuffer[10] << 8 | radioBuffer[11];
    
    xGoal = mapDouble(pitch, 1000, 2000, -90, 90);
    yGoal = -mapDouble(roll, 1000, 2000, -90, 90);
    
    //Safety measure for cases when radio.available accidentaly returns true 
    //as well as a reasonable min throttle cutoff
    if (throttle < 1100)
    {
      throttle = 1100;
    }

    //Max throttle cutoff
    if (throttle > 1800)
    {
      throttle = 1800;
    }

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


  //All data is in little-endian order (MSB first)
  int byte1 = Wire.read(); //accel x-axis
  int byte2 = Wire.read();
  int byte3 = Wire.read(); //accel y-axis
  int byte4 = Wire.read();
  int byte5 = Wire.read(); //accel z-axis
  int byte6 = Wire.read();

  Wire.read(); //Temp, not used in this program
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
  accelAngle_y = -1 * ((180 * atan(gForce_x / gForce_z) / 3.1415) - accelOffset_y); //Negative added for agreement between acceleromter and gyro angle measurements
}

void calculateGyro()
{
  gyroRot_x = (gyro_x / 32.8) - gyroOffset_x;
  gyroRot_y = (gyro_y / 32.8) - gyroOffset_y;
  gyroRot_z = (gyro_z / 32.8) - gyroOffset_z; 
}

void calibrateGyro()
{
  double totalx = 0.0; //Values to record total of measurements over a given period.
  double totaly = 0.0;
  double totalz = 0.0;

  //The calibration process makes a total of 1000 calculations
  for (int i = 0; i < 10; i++)    //Print the "." 10 times
  {
    for (int j = 0; j < 100; j++) //For every print, read 10 values from the gyro
    {
      readMPU();
      calculateGyro();     //Record next consecutive reading
      totalx += gyroRot_x; //Add to each axis total
      totaly += gyroRot_y;
      totalz += gyroRot_z;
    }
  }

  gyroOffset_x = totalx / 1000; //Record average reading as the calibration value
  gyroOffset_y = totaly / 1000;
  gyroOffset_z = totalz / 1000;
} 

void calibrateMPU()
{
  double angleTotalX = 0.0;
  double angleTotalY = 0.0;
  
  //Record 1000 angle reading from the accelerometer.
  for (int i = 0; i < 1000; i++)
  {
      readMPU();
      calculateAccel();
      angleTotalX += accelAngle_x;
      angleTotalY += accelAngle_y;
  }

  //Set calibrated starting orientation values
  xDeg = angleTotalX / 1000;
  yDeg = angleTotalY / 1000;
  zDeg = 0.0;
}

void setupMPU()
{
  //Turn off MPU6050 sleep mode
  Wire.beginTransmission(0x68); //0b1101000 is the address of the MPU 6050. Write bit automatically added.
  Wire.write(0x6b);             //Writes the address of the power management byte
  Wire.write(0b00001000);       //Turn off sleep mode and set the clock source as the internal 8mz clock
  Wire.endTransmission();

  //Setup the gyroscope
  Wire.beginTransmission(0x68); //0b1101000 is the address of the MPU 6050. Write bit automatically added.
  Wire.write(0x1b);             //Writes the address of the gyroscope configuration byte
  Wire.write(0b00010000);       //Sets the full-scale range of the gyroscope to +-500 degrees/second
  Wire.endTransmission();

  //Setup the acceleromter
  Wire.beginTransmission(0x68); //Address of the MPU
  Wire.write(0x1C);             //Address for Accelerometer Configuration
  Wire.write(0b00010000);       //Set full scale range to +-8g
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
  //Serial.print("Throttle: ");
  //Serial.println(throttle);
  //Serial.print("xDeg: ");
  //Serial.println(xDeg);
  //Serial.print("yDeg: ");
  //Serial.println(yDeg);
  //Serial.print("zDeg: ");
  //Serial.println(zDeg);
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

