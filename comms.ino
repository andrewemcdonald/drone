#include <EnableInterrupt.h>
#include <nRF24L01.h>
#include <printf.h>
#include <RF24.h>
#include <RF24_config.h>
#include <SPI.h>

// Transeiver
const int CE = 9, const CSN = 10; // Pin locations
const RF24 radio = RF24(CE, CSN);
const byte address[] = "1chan"; // Communications address
const char writeBuffer[12];

// RC Receiver
const int receiver_throttlePin = 4, receiver_yawPin = 5, receiver_pitchPin = 3, receiver_rollPin = 2, receiver_aux1Pin = 6, receiver_aux2Pin = 7;

// Interrupt
int throttleTimer, yawTimer, rollTimer, pitchTimer;

// Other
int throttle, yaw, pitch, roll;

void setup()
{
  // Setup communcation
  radio.begin();
  radio.openWritingPipe(address);
  Serial.begin(9600);

  pinMode(receiver_throttlePin, INPUT_PULLUP);
  pinMode(receiver_yawPin, INPUT_PULLUP);
  pinMode(receiver_pitchPin, INPUT_PULLUP);
  pinMode(receiver_rollPin, INPUT_PULLUP);

  enableInterrupt(receiver_throttlePin, interrupt, CHANGE); // Roll, pitch, throttle, yaw
  enableInterrupt(receiver_yawPin, interrupt, CHANGE);
  enableInterrupt(receiver_pitchPin, interrupt, CHANGE);
  enableInterrupt(receiver_rollPin, interrupt, CHANGE);
}

void loop()
{
  sendToDrone();
  // printControls();
}

void interrupt()
{
  int currentTime = micros();

  if (digitalRead(receiver_throttlePin) == 1)
  {
    // throttlePin is receiving signal and the timing mechanism hasn't been started.
    if (throttleTimer == 0)
    {
      throttleTimer = currentTime;
    }
  }
  else if (throttleTimer != 0)
  {
    // throttlePin is no longer receiving signal, but the timer is still running.
    throttle = currentTime - throttleTimer;
    throttleTimer = 0;
  }

  if (digitalRead(receiver_pitchPin) == 1)
  {
    // pitchPin is receiving signal and the timing mechanism hasn't been started.
    if (pitchTimer == 0)
    {
      pitchTimer = currentTime;
    }
  }
  else if (pitchTimer != 0)
  {
    // pitchPin is no longer receiving signal, but the timer is still running.
    pitch = currentTime - pitchTimer;
    pitchTimer = 0;
  }

  if (digitalRead(receiver_rollPin) == 1)
  // rollPin is receiving signal and the timing mechanism hasn't been started.
  {
    if (rollTimer == 0)
    {
      rollTimer = currentTime;
    }
  }
  else if (rollTimer != 0)
  {
    // rollPin is no longer receiving signal, but the timer is still running.
    roll = currentTime - rollTimer;
    rollTimer = 0;
  }

  if (digitalRead(receiver_yawPin) == 1)
  {
    // yawPin is receiving signal and the timing mechanism hasn't been started.
    if (yawTimer == 0)
    {
      yawTimer = currentTime;
    }
  }
  else if (yawTimer != 0)
  {
    // yawPin is no longer receiving signal, but the timer is still running.
    yaw = currentTime - yawTimer;
    yawTimer = 0;
  }
}

void sendToDrone()
{
  writeBuffer[0] = throttle >> 8;
  writeBuffer[1] = throttle;

  writeBuffer[2] = pitch >> 8;
  writeBuffer[3] = pitch;

  writeBuffer[4] = roll >> 8;
  writeBuffer[5] = roll;

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
