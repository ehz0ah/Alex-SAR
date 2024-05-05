#include <Servo.h>
#include <Arduino.h>
#include <stdarg.h>
#include <serialize.h>
#include <math.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>


#include "packet.h"
#include "constants.h"

volatile TDirection dir;


/*
   Alex's configuration constants
*/

// Number of ticks per revolution from the
// wheel encoder.

#define COUNTS_PER_REV      4

// Wheel circumference in cm.
// We will use this to calculate forward/backward distance traveled
// by taking revs * WHEEL_CIRC

#define WHEEL_CIRC          20.7     // extra 0.5cm

#define ALEX_LENGTH   11.5
#define ALEX_BREADTH  15.0

#define S0 47
#define S1 45


#define S2 43
#define S3 41
#define sensorOut 39

#define buzzerPin 30

// Define pins by their bit positions in the PORT registers
#define TRIG1_PIN_BIT 0  // Bit 0 of PORTA for the first sensor's trigger (Left)D22
#define ECHO1_PIN_BIT 1  // Bit 1 of PIND for the first sensor's echo (Left)D23
#define TRIG2_PIN_BIT 2  // Bit 2 of PORTA for the second sensor's trigger (Front)D24
#define ECHO2_PIN_BIT 3  // Bit 3 of PIND for the second sensor's echo (Front)D25
#define TRIG3_PIN_BIT 4  // Bit 4 of PORTA for the third sensor's trigger (Right)D26
#define ECHO3_PIN_BIT 5  // Bit 5 of PIND for the third sensor's echo (Right)D27

volatile unsigned long distanceLeft = 0;
volatile unsigned long distanceFront = 0;
volatile unsigned long distanceRight = 0;

// Timer overflow count
volatile unsigned long overflowCount = 0;


float alexDiagonal = 0.0;

float alexCirc = 0.0;

Servo clawServo;  // Create a servo object to control the claw

#define clawServoPin  14  // Servo is attached to digital pin 14
const int openPosition = 20;   // Modify as necessary for your servo's open position
const int closedPosition = 250; // Modify as necessary for your servo's closed position

bool clawIsOpen = false;  // State of the claw


/*
      Alex's State Variables
*/

//volatile unsigned long og_deltaTicks;

// Store the ticks from Alex's left and
// right encoders.
volatile unsigned long leftForwardTicks;
volatile unsigned long rightForwardTicks;
volatile unsigned long leftReverseTicks;
volatile unsigned long rightReverseTicks;

// Store the revolutions on Alex's left
// and right wheels
volatile unsigned long leftRevs;
volatile unsigned long rightRevs;

// Forward and backward distance traveled
volatile unsigned long forwardDist;
volatile unsigned long reverseDist;

volatile unsigned long leftForwardTicksTurns;
volatile unsigned long rightForwardTicksTurns;
volatile unsigned long leftReverseTicksTurns;
volatile unsigned long rightReverseTicksTurns;


//Variables to keep track of whether we've moved
// a commanded distance
unsigned long deltaDist;
unsigned long newDist;
unsigned long deltaTicks;
unsigned long targetTicks;


// Stores frequency read by the photodiodes
int redFreq = 0;
int greenFreq = 0;
int blueFreq = 0;
int color = 0;


// distance front, left, right
volatile unsigned long distanceLeft = 0;
volatile unsigned long distanceFront = 0;
volatile unsigned long distanceRight = 0;


/*

   Alex Communication Routines.

*/

void toggleClaw() {
  if (clawIsOpen) {
    clawServo.write(closedPosition); // Close the claw if it is open
    clawIsOpen = false;              // Update the state to closed
  } else {
    clawServo.write(openPosition);   // Open the claw if it is closed
    clawIsOpen = true;               // Update the state to open
  }
  delay(15); // Delay to give the servo time to reach the position
}

void setupBuzzer() {
  pinMode(buzzerPin, OUTPUT);
  tone(buzzerPin, 1000, 2000);
}

void celebrate() {
  tone(buzzerPin, 190);
  delay(200);
  tone(buzzerPin, 220);
  delay(200);
  tone(buzzerPin, 220);
  delay(200);
  tone(buzzerPin, 587.33);
  delay(200);
  tone(buzzerPin, 587.33);
  delay(200);
  tone(buzzerPin, 739.99);
  delay(200);
  tone(buzzerPin, 739.99);
  delay(200);
  tone(buzzerPin, 880);
  delay(200);
  tone(buzzerPin, 185);
  delay(200);
  tone(buzzerPin, 220);
  delay(200);
  tone(buzzerPin, 220);
  delay(200);
  tone(buzzerPin, 587.33);
  delay(200);
  tone(buzzerPin, 587.33);
  delay(100);
  tone(buzzerPin, 740);
  delay(100);
  tone(buzzerPin, 740);
  delay(100);
  tone(buzzerPin, 880);
  delay(200);
  tone(buzzerPin, 987.77);
  delay(600);
  tone(buzzerPin, 783.99);
  delay(150);
  tone(buzzerPin, 1174.66);
  delay(1600);
  tone(buzzerPin, 185);
  delay(200);
  tone(buzzerPin, 220);
  delay(200);
  tone(buzzerPin, 220);
  delay(200);
  tone(buzzerPin, 587.33);
  delay(200);
  tone(buzzerPin, 587.33);
  delay(200);
  tone(buzzerPin, 739.99);
  delay(200);
  tone(buzzerPin, 739.99);
  delay(200);
  tone(buzzerPin, 880);
  delay(200);
  tone(buzzerPin, 185);
  delay(100);
  tone(buzzerPin, 220);
  delay(200);
  tone(buzzerPin, 220);
  delay(200);
  tone(buzzerPin, 587.33);
  delay(200);
  tone(buzzerPin, 587.33);
  delay(100);
  tone(buzzerPin, 740);
  delay(200);
  tone(buzzerPin, 740);
  delay(200);
  tone(buzzerPin, 880);
  delay(200);
  tone(buzzerPin, 1046.50);
  delay(500);
  tone(buzzerPin, 1046.50);
  delay(200);
  tone(buzzerPin, 1046.50);
  delay(1200);
  tone(buzzerPin, 1046.50);
  delay(200);
  tone(buzzerPin, 987.77);
  delay(500);
  tone(buzzerPin, 1046.50);
  delay(150);
  tone(buzzerPin, 783.99);
  delay(1000);
  noTone(buzzerPin);
}

void greenSiren() {
  tone(buzzerPin, 100);
  delay(300);
  tone(buzzerPin, 200);
  delay(300);
  tone(buzzerPin, 100);
  delay(300);
  tone(buzzerPin, 200);
  delay(300);
  tone(buzzerPin, 100);
  noTone(buzzerPin);
}

void redSiren() {
  tone(buzzerPin, 800);
  delay(300);
  tone(buzzerPin, 700);
  delay(300);
  tone(buzzerPin, 800);
  delay(300);
  tone(buzzerPin, 700);
  delay(300);
  tone(buzzerPin, 800);
  noTone(buzzerPin);
}


void setupUltra() {
  pinMode(TRIG1_PIN, OUTPUT);
  pinMode(ECHO1_PIN, INPUT);
  pinMode(TRIG2_PIN, OUTPUT);
  pinMode(ECHO2_PIN, INPUT);
  pinMode(TRIG3_PIN, OUTPUT);
  pinMode(ECHO3_PIN, INPUT);
}

// Function to trigger ultrasonic pulse
void triggerUltrasonic(int trigPin) {
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);        // 10-microsecond pulse
  digitalWrite(trigPin, LOW);
}

// Function to measure the distance
unsigned long measureDistance(int trigPin, int echoPin) {
  triggerUltrasonic(trigPin);  // Send a pulse

  // Measure the length of the incoming echo pulse
  unsigned long duration = pulseIn(echoPin, HIGH);
  unsigned long distance = (duration / 2) * 0.0346;  // Calculate distance in cm (speed of sound in cm/us)

  return distance;
}

void measureDistances() {
  distanceLeft = measureDistance(TRIG1_PIN, ECHO1_PIN);
  distanceFront = measureDistance(TRIG2_PIN, ECHO2_PIN);
  distanceRight = measureDistance(TRIG3_PIN, ECHO3_PIN);
}

int averageReading() {
  int total = 0;
  for (int i = 0; i < 5; i += 1) {
    int reading = pulseIn(sensorOut, LOW);
    total += reading;
  }
  return total / 5;
}

int getRed() {
  // Setting RED (R) filtered photodiodes to be read
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  redFreq = averageReading();
  return redFreq;
}

int getGreen() {
  // Setting GREEN (G) filtered photodiodes to be read
  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);
  greenFreq = averageReading();
  return greenFreq;
}

int getBlue() {
  // Setting BLUE (B) filtered photodiodes to be read
  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);
  blueFreq = averageReading();
  return blueFreq;
}


void getColour() {
  redFreq = getRed();
  delay(50);
  greenFreq = getGreen();
  delay(50);
  blueFreq = getBlue();
  delay(50);


  if (abs(redFreq - greenFreq) < 20 && greenFreq > blueFreq) {   // blueFreq < 130
    color = 1;
  }
  else if (greenFreq < redFreq) {
    color = 2;
  } else if (redFreq < greenFreq) {
    color = 3;
  } else {
    color = 0;
  }
}

void setupColor() {
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);
}

unsigned long computeDeltaTicks(float ang) {
  unsigned long ticks = (unsigned long) ((ang * alexCirc * COUNTS_PER_REV) / (360.0 * WHEEL_CIRC));
  return ticks;
}

void left(float ang, float speed) {
  if (ang == 0) {
    deltaTicks = 99999999;
  }
  else {
    deltaTicks = computeDeltaTicks(ang);
    //Serial.print(deltaTicks);
  }
  //og_deltaTicks = deltaTicks;
  targetTicks = leftReverseTicksTurns + deltaTicks;
  dir = (TDirection) LEFT;
  cw(ang, speed);
}

void right(float ang, float speed) {
  if (ang == 0) {
    deltaTicks = 99999999;
  }
  else {
    deltaTicks = computeDeltaTicks(ang);
  }
  targetTicks = rightReverseTicksTurns + deltaTicks;
  dir = (TDirection) RIGHT;
  ccw(ang, speed);
}

TResult readPacket(TPacket * packet)
{
  // Reads in data from the serial port and
  // deserializes it.Returns deserialized
  // data in "packet".

  char buffer[PACKET_SIZE];
  int len;

  len = readSerial(buffer);

  if (len == 0)
    return PACKET_INCOMPLETE;
  else
    return deserialize(buffer, len, packet);

}

void sendStatus()
{
  // Implement code to send back a packet containing key
  // information like leftTicks, rightTicks, leftRevs, rightRevs
  // forwardDist and reverseDist
  // Use the params array to store this information, and set the
  // packetType and command files accordingly, then use sendResponse
  // to send out the packet. See sendMessage on how to use sendResponse.
  //
  TPacket statusPacket;
  statusPacket.packetType = PACKET_TYPE_RESPONSE;
  statusPacket.command = RESP_STATUS;
  /*
    statusPacket.params[0] = leftForwardTicks;
    statusPacket.params[1] = rightForwardTicks;
    statusPacket.params[2] = leftReverseTicks;
    statusPacket.params[3] = rightReverseTicks;
    statusPacket.params[4] = leftForwardTicksTurns;
    statusPacket.params[5] = rightForwardTicksTurns;
    statusPacket.params[6] = leftReverseTicksTurns;
    statusPacket.params[7] = rightReverseTicksTurns;
    statusPacket.params[8] = forwardDist;
    statusPacket.params[9] = reverseDist;
  */
  statusPacket.params[0] = color;
  statusPacket.params[1] = distanceLeft;
  statusPacket.params[2] = distanceFront;
  statusPacket.params[3] = distanceRight;
  statusPacket.params[4] = redFreq;
  statusPacket.params[5] = greenFreq;
  statusPacket.params[6] = blueFreq;
  sendResponse(&statusPacket);
}

void sendMessage(const char *message)
{
  // Sends text messages back to the Pi. Useful
  // for debugging.

  TPacket messagePacket;
  messagePacket.packetType = PACKET_TYPE_MESSAGE;
  strncpy(messagePacket.data, message, MAX_STR_LEN);
  sendResponse(&messagePacket);
}

void dbprintf(char *format, ...) {
  va_list args;
  char buffer[128];
  va_start(args, format);
  vsprintf(buffer, format, args);
  sendMessage(buffer);
}

void sendBadPacket()
{
  // Tell the Pi that it sent us a packet with a bad
  // magic number.

  TPacket badPacket;
  badPacket.packetType = PACKET_TYPE_ERROR;
  badPacket.command = RESP_BAD_PACKET;
  sendResponse(&badPacket);

}

void sendBadChecksum()
{
  // Tell the Pi that it sent us a packet with a bad
  // checksum.

  TPacket badChecksum;
  badChecksum.packetType = PACKET_TYPE_ERROR;
  badChecksum.command = RESP_BAD_CHECKSUM;
  sendResponse(&badChecksum);
}

void sendBadCommand()
{
  // Tell the Pi that we don't understand its
  // command sent to us.

  TPacket badCommand;
  badCommand.packetType = PACKET_TYPE_ERROR;
  badCommand.command = RESP_BAD_COMMAND;
  sendResponse(&badCommand);
}

void sendBadResponse()
{
  TPacket badResponse;
  badResponse.packetType = PACKET_TYPE_ERROR;
  badResponse.command = RESP_BAD_RESPONSE;
  sendResponse(&badResponse);
}

void sendOK()
{
  TPacket okPacket;
  okPacket.packetType = PACKET_TYPE_RESPONSE;
  okPacket.command = RESP_OK;
  sendResponse(&okPacket);
}

void sendResponse(TPacket * packet)
{
  // Takes a packet, serializes it then sends it out
  // over the serial port.
  char buffer[PACKET_SIZE];
  int len;

  len = serialize(buffer, packet, sizeof(TPacket));
  writeSerial(buffer, len);
}


/*
   Setup and start codes for external interrupts and
   pullup resistors.

*/
// Enable pull up resistors on pins 18 and 19
void enablePullups()
{
  // Use bare-metal to enable the pull-up resistors on pins
  // 19 and 18. These are pins PD2 and PD3 respectively.
  // We set bits 2 and 3 in DDRD to 0 to make them inputs.
  DDRD &= 0b11110011;
  PORTD |= 0b00001100;
}

// Functions to be called by INT2 and INT3 ISRs.
void leftISR()
{
  if (dir == FORWARD) {
    leftForwardTicks++;
    forwardDist = (unsigned long) ((float) leftForwardTicks / COUNTS_PER_REV * WHEEL_CIRC);
  }
  else if (dir == BACKWARD) {
    leftReverseTicks++;
    reverseDist = (unsigned long) ((float) leftReverseTicks / COUNTS_PER_REV * WHEEL_CIRC);
  }
  else if (dir == LEFT) {
    leftReverseTicksTurns++;
  }
  else if (dir == RIGHT) {
    leftForwardTicksTurns++;
  }
}

void rightISR()
{
  if (dir == FORWARD) {
    rightForwardTicks++;
  }
  else if (dir == BACKWARD) {
    rightReverseTicks++;
  }
  else if (dir == LEFT) {
    rightForwardTicksTurns++;
  }
  else if (dir == RIGHT) {
    rightReverseTicksTurns++;
  }
}

// Set up the external interrupt pins INT2 and INT3
// for falling edge triggered. Use bare-metal.
void setupEINT()
{
  // Use bare-metal to configure pins 18 and 19 to be
  // falling edge triggered. Remember to enable
  // the INT2 and INT3 interrupts.
  // Hint: Check pages 110 and 111 in the ATmega2560 Datasheet.

  cli();
  EICRA = 0b10100000;
  EIMSK = 0b00001100;
  sei();
}

// Implement the external interrupt ISRs below.
// INT3 ISR should call leftISR while INT2 ISR
// should call rightISR.

ISR(INT2_vect) {
  rightISR();
}

ISR(INT3_vect) {
  leftISR();
}



// Implement INT2 and INT3 ISRs above.

/*
   Setup and start codes for serial communications

*/
// Set up the serial connection. For now we are using
// Arduino Wiring, you will replace this later
// with bare-metal code.
void setupSerial()
{
  // To replace later with bare-metal.
  Serial.begin(9600);
  // Change Serial to Serial2/Serial3/Serial4 in later labs when using the other UARTs
}

// Start the serial connection. For now we are using
// Arduino wiring and this function is empty. We will
// replace this later with bare-metal code.

void startSerial()
{
  // Empty for now. To be replaced with bare-metal code
  // later on.

}

// Read the serial port. Returns the read character in
// ch if available. Also returns TRUE if ch is valid.
// This will be replaced later with bare-metal code.

int readSerial(char *buffer)
{

  int count = 0;

  // Change Serial to Serial2/Serial3/Serial4 in later labs when using other UARTs

  while (Serial.available())
    buffer[count++] = Serial.read();

  return count;
}

// Write to the serial port. Replaced later with
// bare-metal code

void writeSerial(const char *buffer, int len)
{
  Serial.write(buffer, len);
  // Change Serial to Serial2/Serial3/Serial4 in later labs when using other UARTs
}

/*
   Alex's setup and run codes

*/



// Clears all our counters
void clearCounters()
{
  leftForwardTicks = 0;
  rightForwardTicks = 0;
  leftReverseTicks = 0;
  rightReverseTicks = 0;

  leftRevs = 0;
  rightRevs = 0;

  forwardDist = 0;
  reverseDist = 0;

  leftForwardTicksTurns = 0;
  rightForwardTicksTurns = 0;
  leftReverseTicksTurns = 0;
  rightReverseTicksTurns = 0;

  distanceFront = 0;
  distanceLeft = 0;
  distanceRight = 0;
}

// Clears one particular counter
void clearOneCounter(int which)
{

  clearCounters();
}

// Intialize Alex's internal states

void initializeState()
{
  clearCounters();
}


void handleCommand(TPacket * command)
{
  switch (command->command)
  {
    // For movement commands, param[0] = distance, param[1] = speed.
    case COMMAND_W:
      sendOK();
      forward((double) command->params[0], (float) command->params[1]);
      break;

    case COMMAND_D:
      sendOK();
      right((double) command->params[0], (float) command->params[1]);
      break;

    case COMMAND_A:
      sendOK();
      left((double) command->params[0], (float) command->params[1]);
      break;

    case COMMAND_FORWARD:
      sendOK();
      forward((double) command->params[0], (float) command->params[1]);
      break;

    case COMMAND_REVERSE:
      sendOK();
      backward((double) command->params[0], (float) command->params[1]);
      break;

    case COMMAND_TURN_LEFT:
      sendOK();
      left((double) command->params[0], (float) command->params[1]);
      break;

    case COMMAND_TURN_RIGHT:
      sendOK();
      right((double) command->params[0], (float) command->params[1]);
      break;

    case COMMAND_CLAW:
      sendOK();
      toggleClaw();
      break;

    case COMMAND_RED:
      sendOK();
      redSiren();
      //policeSiren();
      break;

    case COMMAND_GREEN:
      sendOK();
      greenSiren();
      break;

    case COMMAND_CELEBRATE:
      sendOK();
      celebrate();
      break;

    case COMMAND_STOP:
      sendOK();
      stop();
      break;

    case COMMAND_GET_STATS:
      getColour();
      measureDistances();
      sendStatus();
      break;

    case COMMAND_CLEAR_STATS:
      sendOK();
      clearOneCounter(command->params[0]);
      break;

    default:
      sendBadCommand();
  }
}


void waitForHello()
{
  int exit = 0;

  while (!exit)
  {
    TPacket hello;
    TResult result;

    do
    {
      result = readPacket(&hello);
    } while (result == PACKET_INCOMPLETE);

    if (result == PACKET_OK)
    {
      if (hello.packetType == PACKET_TYPE_HELLO)
      {

        sendOK();
        exit = 1;
      }
      else
        sendBadResponse();
    }
    else if (result == PACKET_BAD)
    {
      sendBadPacket();
    }
    else if (result == PACKET_CHECKSUM_BAD)
      sendBadChecksum();
  } // !exit
}


void setup() {
  // put your setup code here, to run once:
  alexDiagonal = sqrt((ALEX_LENGTH * ALEX_LENGTH) + (ALEX_BREADTH * ALEX_BREADTH));
  alexCirc = PI * alexDiagonal;
  cli();
  setupEINT();
  setupSerial();
  startSerial();
  clawServo.attach(clawServoPin);
  clawServo.write(closedPosition);
  setupColor();
  setupUltra();
  setupColor();
  setupBuzzer();
  enablePullups();
  initializeState();
  waitForHello();
  sei();
}

void handlePacket(TPacket * packet)
{
  switch (packet->packetType)
  {
    case PACKET_TYPE_COMMAND:
      handleCommand(packet);
      break;

    case PACKET_TYPE_RESPONSE:
      break;

    case PACKET_TYPE_ERROR:
      break;

    case PACKET_TYPE_MESSAGE:
      break;

    case PACKET_TYPE_HELLO:
      break;
  }
}

void loop() {

  //getColour();
  //Serial.println(color);


  TPacket recvPacket; // This holds commands from the Pi

  TResult result = readPacket(&recvPacket);

  if (result == PACKET_OK) {
    handlePacket(&recvPacket);
  }
  else if (result == PACKET_BAD) {
    sendBadPacket();
  }
  else if (result == PACKET_CHECKSUM_BAD) {
    sendBadChecksum();
  }


  if (deltaDist > 0) {
    if (dir == FORWARD) {
      if (forwardDist > newDist) {
        deltaDist = 0;
        newDist = 0;
        stop();
      }
    }
    else if (dir == BACKWARD) {
      if (reverseDist > newDist) {
        deltaDist = 0;
        newDist = 0;
        stop();
      }
    }
    else if (dir == (TDirection)STOP) {
      deltaDist = 0;
      newDist = 0;
      stop();
    }
  }
  if (deltaTicks > 0) {
    if (dir == LEFT) {
      if (leftReverseTicksTurns >= targetTicks) {
        deltaTicks = 0;
        targetTicks = 0;
        stop();
      }
    }
    else if (dir == RIGHT) {
      if (rightReverseTicksTurns >= targetTicks) {
        deltaTicks = 0;
        targetTicks = 0;
        stop();
      }
    }
    else if (dir == (TDirection)STOP) {
      deltaTicks = 0;
      targetTicks = 0;
      stop();
    }
  }

}
