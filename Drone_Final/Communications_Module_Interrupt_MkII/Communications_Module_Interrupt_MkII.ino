/*
 * Used this website for documentation and troubleshooting: http://arduino-info.wikispaces.com/Nrf24L01-2.4GHz-HowTo
 * Referenced this code: http://forum.arduino.cc/index.php?topic=421081
 */
#include <EnableInterrupt.h>
#include <nRF24L01.h>
#include <printf.h>
#include <RF24.h>
#include <RF24_config.h>
#include <SPI.h>

//NRF24L01 Variables
int CE = 9, CSN = 10;                             //Pin variables
RF24 radio = RF24(CE, CSN);                       //Radio object
byte address[] = {'1', 'c', 'h', 'a', 'n'};       //Represents the communications address
char writeBuffer[12];                             //Stores all of the information to be sent to the drone

//RC Receiver Variables
int receiver_throttlePin = 4, receiver_yawPin = 5, receiver_pitchPin = 3, receiver_rollPin = 2, receiver_aux1Pin = 6, receiver_aux2Pin = 7;

//Interrupt Variables
int throttleTimer, yawTimer, rollTimer, pitchTimer;

//Other program variables
int throttle, yaw, pitch, roll;     //Stores values as read from receiver to be sent to drone

//******************************************CODE STARTS HERE!!!!!!!!!!!***********************************************************

void setup() 
{
  //Setup communcation
  radio.begin();
  radio.openWritingPipe(address);
  Serial.begin(9600);

  pinMode(receiver_throttlePin, INPUT_PULLUP);
  pinMode(receiver_yawPin, INPUT_PULLUP);
  pinMode(receiver_pitchPin, INPUT_PULLUP);
  pinMode(receiver_rollPin, INPUT_PULLUP);

  enableInterrupt(receiver_throttlePin, interrupt, CHANGE); //Roll, pitch, throttle, yaw
  enableInterrupt(receiver_yawPin, interrupt, CHANGE);
  enableInterrupt(receiver_pitchPin, interrupt, CHANGE);
  enableInterrupt(receiver_rollPin, interrupt, CHANGE);
}

void loop() 
{  
  sendToDrone();
  //printControls();
}

void interrupt()
{
  int currentTime = micros();
  
  if (digitalRead(receiver_throttlePin) == 1)           //If throttlePin is receiving signal and the timing mechanism hasn't been started
  {
    if (throttleTimer == 0)
    {
      throttleTimer = currentTime;                      //Start timer
    }                                          
  } 
  else if (throttleTimer != 0)                          //If throttlePin is no longer receiving signal, but the timer is still "running"
  {
     throttle = currentTime - throttleTimer;            //then record measurement and stop "timing"
     throttleTimer = 0;
  }

  if (digitalRead(receiver_pitchPin) == 1)           //If pitchPin is receiving signal and the timing mechanism hasn't been started
  {
    if (pitchTimer == 0)
    {
      pitchTimer = currentTime;                      //Start timer
    }                                          
  } 
  else if (pitchTimer != 0)                          //If pitchPin is no longer receiving signal, but the timer is still "running"
  {
     pitch = currentTime - pitchTimer;               //then record measurement and stop "timing"
     pitchTimer = 0;
  }

  if (digitalRead(receiver_rollPin) == 1)           //If rollPin is receiving signal and the timing mechanism hasn't been started
  {
    if (rollTimer == 0)
    {
      rollTimer = currentTime;                      //Start timer
    }                                          
  } 
  else if (rollTimer != 0)                          //If rollPin is no longer receiving signal, but the timer is still "running"
  {
     roll = currentTime - rollTimer;                //then record measurement and stop "timing"
     rollTimer = 0;
  }

  if (digitalRead(receiver_yawPin) == 1)           //If yawPin is receiving signal and the timing mechanism hasn't been started
  {
    if (yawTimer == 0)
    {
      yawTimer = currentTime;                      //Start timer
    }                                          
  } 
  else if (yawTimer != 0)                          //If yawPin is no longer receiving signal, but the timer is still "running"
  {
     yaw = currentTime - yawTimer;                 //then record measurement and stop "timing"
     yawTimer = 0;
  }
  
}

void sendToDrone()
{
  //Converts throttle into sendable bytes and inserts them into the buffer
  writeBuffer[0] = throttle >> 8;  //Gets most significant byte
  writeBuffer[1] = throttle;       //Gets least significant byte

  //Converts pitch into sendable bytes and inserts them into the buffer
  writeBuffer[2] = pitch >> 8;
  writeBuffer[3] = pitch;

  //Converts roll into sendable bytes and inserts themm into the buffer
  writeBuffer[4] = roll >> 8;
  writeBuffer[5] = roll;

  //Converts yaw into sendable bytes and inserts themm into the buffer
  writeBuffer[6] = yaw >> 8;
  writeBuffer[7] = yaw;
    
  radio.write(&writeBuffer, sizeof(writeBuffer));
}

void printControls()
{
  printPWM("Throttle: ", throttle);
  printPWM("Pitch: ", pitch);
  printPWM("Roll: ", roll);
  printPWM("Yaw: ", yaw);
  Serial.println();
  delay(400);
}

void printPWM(String pinLabel, int val)
{
  Serial.print(pinLabel);
  Serial.println(val);
}



