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

#define S0_PIN_BIT 2 // pin 47 PL2
#define S1_PIN_BIT 4 // pin 45 PL4
#define S2_PIN_BIT 6 // pin 43 PL6
#define S3_PIN_BIT 0 // pin 41 PG0
#define sensorOut_PIN_BIT 2 // pin 39 PG2


#define buzzerPin 7   // Arduino Mega pin 30 corresponds to PORTC7

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

#define SERVO_PIN 46  // Pin 46 uses OC5A (Timer 5)

// Define the pulse widths in terms of timer ticks
const unsigned int openPositionTicks = 4000;   // 2 ms pulse corresponding to 180 deg
const unsigned int closedPositionTicks = 2000; // 1 ms pulse corresponding to 0 deg

bool clawIsOpen = false;  // State of the claw

// Global serialisation variables
#define BUFFER_LEN 256     // Buffer is initialized to accept up to "size" characters.
// buffers for UART
TBuffer _recvBuffer;
TBuffer _xmitBuffer;


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

/*

   Alex Functionalities.

*/

void setupClaw() {
    // Set SERVO_PIN as output
    DDRL |= 0b00000001;  // PL0 is Pin 46 on Arduino Mega

    // Setup Timer 5 for PWM
    TCCR5A = 0b10000010;  // Non-inverting mode on OC5A, Fast PWM, TOP is ICR5
    TCCR5B = 0b00011010;  // Fast PWM, TOP is ICR5, Prescaler set to 8

    ICR5 = 39999;  // TOP value for 50Hz frequency & 20ms period with a prescaler of 8

    // Set servo to closed position initially
    OCR5A = closedPositionTicks;
}

void toggleClaw() {
    if (clawIsOpen) {
        OCR5A = closedPositionTicks; // Move servo to closed position
        clawIsOpen = false;
    } else {
        OCR5A = openPositionTicks;   // Move servo to open position
        clawIsOpen = true;
    }
    _delay_ms(15); // Delay to allow the servo to reach position
}


void setupBuzzer() {
  // Set the buzzer pin as output
  DDRC |= (1 << DDC7);
}

void tone(unsigned int frequency) {
  // Calculate the period of the wave in microseconds
  unsigned long period = 1000000 / frequency;

  // Set the prescaler to 64
  TCCR4B = (TCCR4B & 0xF8) | 0x03;

  // Set the output compare register
  OCR4A = period - 1;

  // Set timer mode (CTC mode)
  TCCR4A |= (1 << WGM41);

  // Enable timer interrupt
  TIMSK4 |= (1 << OCIE4A);

  // Start the timer
  TCCR4B |= (1 << CS40);
}

void noTone() {
  // Turn off the timer
  TCCR4B = 0;
  // Turn off the buzzer pin
  PORTC &= ~(1 << PORTC7);
}

// Timer 4 output compare A match interrupt service routine
ISR(TIMER4_COMPA_vect) {
  // Toggle the buzzer pin
  PORTC ^= (1 << PORTC7);
}

void celebrate() {
  // Define timer registers
  TCCR4A = 0; // Clear timer control registers
  TCCR4B = 0;
  TCNT4 = 0; // Reset timer counter
  // Set the buzzer pin as output
  DDRC |= (1 << DDC7);

  // Play tones
  tone(190);
  delay(200);
  tone(220);
  delay(200);
  tone(220);
  delay(200);
  tone(587.33);
  delay(200);
  tone(587.33);
  delay(200);
  tone(739.99);
  delay(200);
  tone(739.99);
  delay(200);
  tone(880);
  delay(200);
  tone(185);
  delay(200);
  tone(220);
  delay(200);
  tone(220);
  delay(200);
  tone(587.33);
  delay(200);
  tone(587.33);
  delay(100);
  tone(740);
  delay(200);
  tone(740);
  delay(200);
  tone(880);
  delay(200);
  tone(987.77);
  delay(600);
  tone(783.99);
  delay(150);
  tone(1174.66);
  delay(1600);
  tone(185);
  delay(200);
  tone(220);
  delay(200);
  tone(220);
  delay(200);
  tone(587.33);
  delay(200);
  tone(587.33);
  delay(200);
  tone(739.99);
  delay(200);
  tone(739.99);
  delay(200);
  tone(880);
  delay(200);
  tone(185);
  delay(200);
  tone(220);
  delay(200);
  tone(220);
  delay(200);
  tone(587.33);
  delay(200);
  tone(587.33);
  delay(200);
  tone(740);
  delay(200);
  tone(740);
  delay(200);
  tone(880);
  delay(200);
  tone(1046.50);
  delay(500);
  tone(1046.50);
  delay(200);
  tone(1046.50);
  delay(1200);
  tone(1046.50);
  delay(200);
  tone(987.77);
  delay(500);
  tone(1046.50);
  delay(150);
  tone(783.99);
  delay(1000);
  noTone();
}

void greenSiren() {
  // Define timer registers
  TCCR4A = 0; // Clear timer control registers
  TCCR4B = 0;
  TCNT4 = 0; // Reset timer counter
  // Set the buzzer pin as output
  DDRC |= (1 << DDC7);

  // Play green siren
  tone(100);
  delay(300);
  tone(200);
  delay(300);
  tone(100);
  delay(300);
  tone(200);
  delay(300);
  tone(100);
  noTone();
}

void redSiren() {
  // Define timer registers
  TCCR4A = 0; // Clear timer control registers
  TCCR4B = 0;
  TCNT4 = 0; // Reset timer counter
  // Set the buzzer pin as output
  DDRC |= (1 << DDC7);

  // Play red siren
  tone(800);
  delay(300);
  tone(700);
  delay(300);
  tone(800);
  delay(300);
  tone(700);
  delay(300);
  tone(800);
  noTone();
}


// ISR for timer overflows so that we can keep count how many times it overflows if not max dist is only 70cm
ISR(TIMER5_OVF_vect) {
  overflowCount++;
}

// Timer5 initialization
void timer5_init() {
  cli();
  TCCR5A = 0; // Clear Timer/Counter5 Control Register A
  TCCR5B = 0; // Clear Timer/Counter5 Control Register B
  TCCR5B |= 0b00000001; // Set no prescaler
  TCNT5 = 0; // Clear Timer/Counter5
  TIMSK5 |= 0b00000001; // Enable Timer/Counter5 Overflow Interrupt
  overflowCount = 0; // Reset overflow counter
  sei(); // Enable global interrupts
}
// Ultrasonic sensor initialization
void ultrasonic_init() {
  // Set trigger pins as outputs on PORTA
  DDRA |= (1 << TRIG1_PIN_BIT) | (1 << TRIG2_PIN_BIT) | (1 << TRIG3_PIN_BIT);
  // Set echo pins as inputs on PORTA
  DDRA &= ~((1 << ECHO1_PIN_BIT) | (1 << ECHO2_PIN_BIT) | (1 << ECHO3_PIN_BIT));
}

// Function to trigger ultrasonic pulse
void triggerUltrasonic(uint8_t trigPinBit) {
  PORTA |= (1 << trigPinBit);  // Set the corresponding trigger pin HIGH
  _delay_us(10);               // Wait for 10 microseconds
  PORTA &= ~(1 << trigPinBit); // Set the trigger pin LOW
}

// Function to measure the distance
unsigned long measureDistance(uint8_t trigPinBit, uint8_t echoPinBit) {
  overflowCount = 0; // Reset the overflow counter
  TCNT5 = 0;         // Reset the timer value
  triggerUltrasonic(trigPinBit); // Trigger the ultrasonic pulse

  // Wait for the echo pin to go high (start of the echo signal)
  while (!(PINA & (1 << echoPinBit)));
  TCNT5 = 0; // Reset the timer value again to measure the high pulse width

  // Wait for the echo pin to go low (end of the echo signal)
  while (PINA & (1 << echoPinBit));

  // Calculate the duration in microseconds and distance in cm
  unsigned long totalDuration = TCNT5 + (overflowCount * 65535);
  unsigned long durationInMicroseconds = totalDuration * (1.0 / 16000000.0) * 1000000.0;
  unsigned long distance = (durationInMicroseconds * 0.0346) / 2;

  return distance;
}

void measureDistanceLeft() {
    distanceLeft = measureDistance(TRIG1_PIN_BIT, ECHO1_PIN_BIT);
}

void measureDistanceFront() {
    distanceFront = measureDistance(TRIG2_PIN_BIT, ECHO2_PIN_BIT);
}

void measureDistanceRight() {
    distanceRight = measureDistance(TRIG3_PIN_BIT, ECHO3_PIN_BIT);
}


int getRed() {
  // Setting RED (R) filtered photodiodes to be read
  PORTL &= ~(1 << S2_PIN_BIT);
  PORTG &= ~(1 << S3_PIN_BIT);
  redFreq = averageReading();
  return redFreq;
}

int getGreen() {
  // Setting GREEN (G) filtered photodiodes to be read
  PORTL |= (1 << S2_PIN_BIT);
  PORTG |= (1 << S3_PIN_BIT);
  greenFreq = averageReading();
  return greenFreq;
}

int getBlue() {
  // Setting BLUE (B) filtered photodiodes to be read
  PORTL &= ~(1 << S2_PIN_BIT);
  PORTG |= (1 << S3_PIN_BIT);
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


  if (abs(redFreq - greenFreq) < 20 && greenFreq > blueFreq) {  
    color = 1;  //white colour detected
  }
  else if (greenFreq < redFreq) {
    color = 2;  // green colour detected
  } else if (redFreq < greenFreq) {
    color = 3;  //red colour detected
  } else {
    color = 0;  //invalid colour
  }
}

void setupColor() {
  // Set S0, S1, S2, S3 as output, sensorOut as input
  DDRL |= (1 << S0_PIN_BIT) | (1 << S1_PIN_BIT) | (1 << S2_PIN_BIT);
  DDRG |= (1 << S3_PIN_BIT);
  DDRG &= ~(1 << sensorOut_PIN_BIT);
  // Set S0 to HIGH, S1 to LOW, so output frequency scaling is set to 20%
  PORTL |= (1 << S0_PIN_BIT);
  PORTL &= ~(1 << S1_PIN_BIT);
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
  // Initialize the buffer. We must call this before using writeBuffer or readBuffer.
  // Buffer to use is specified in "buffer", size of buffer in characters is specified in "size"
  //set baud rate to 9600 and initialize our transmit and receive buffers
  initBuffer(&_recvBuffer, BUFFER_LEN);
  initBuffer(&_xmitBuffer, BUFFER_LEN);
  UBRR0L = 103;
  UBRR0H = 0;
  
  //set frame format: 8 bit, no parity, 1 stop bit (8N1)
  UCSR0C = 0b110;
  UCSR0A = 0;
}


// Start the serial connection. For now we are using
// Arduino wiring and this function is empty. We will
// replace this later with bare-metal code.

void startSerial()
{
  // Enable USART transmitter and receiver
  // USART_RX_vect to be triggered when a character is received
  // USART_UDRE_vect intterupt triggered when sending data register is empty
  UCSR0B = 0b10111000;
}


// Read the serial port. Returns the read character in
// ch if available. Also returns TRUE if ch is valid.
// This will be replaced later with bare-metal code.

int readSerial(char *buffer)
{

  int count=0;
  TBufferResult result;

  do{
    result = readBuffer(&_recvBuffer, &buffer[count]);
    if (result == BUFFER_OK){
      count++;
    } 
    if (!dataAvailable(&_recvBuffer)) {
       break;
    }
  } while (result == BUFFER_OK);
  
  return count;
}


// Write to the serial port. Replaced later with
// bare-metal code

void writeSerial(const char *buffer, int len)
{
  TBufferResult result = BUFFER_OK;
  for(int i = 1; i < len; i += 1){
    result = writeBuffer(&_xmitBuffer, buffer[i]);
    if (result != BUFFER_OK) {
       break;
    }
  }
  //read and write data from UDR0
  UDR0 = buffer[0];
  UCSR0B |= 0b00100000;
}

ISR(USART_RX_vect){
  // Data from UDR0 is read and written to data
  unsigned char data = UDR0;
  writeBuffer(&_recvBuffer, data);
}

ISR(USART_UDRE_vect){
  // Data to be send is copied into UDR0
  unsigned char data;
  TBufferResult result = readBuffer(&_xmitBuffer, &data);

  if (result == BUFFER_OK){
    UDR0 = data;
  } 
  else if (result == BUFFER_EMPTY){
      UCSR0B &= 0b11011111;
  }
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
  setupClaw();
  setupColor();
  ultrasonic_init(); // Initialize the ultrasonic sensors
  timer5_init(); // Initialize the timer
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
