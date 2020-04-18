#include <serialize.h>
#include "packet.h"
#include "constants.h"
#include <math.h>
#include <stdarg.h>
#include <avr/sleep.h>
#include "Arduino.h"
#include <avr/io.h>
#include <util/delay.h>

//mask definitions

#define PRR_TWI_MASK 0b10000000
#define PRR_SPI_MASK 0b00000100
#define ADCSRA_ADC_MASK 0b10000000
#define PRR_ADC_MASK 0b00000001
#define PRR_TIMER2_MASK 0b01000000
#define PRR_TIMER0_MASK 0b00100000
#define PRR_TIMER1_MASK 0b00001000
#define SMCR_SLEEP_ENABLE_MASK 0b00000001
#define SMCR_IDLE_MODE_MASK 0b11110001

typedef enum {
  STOP = 0,
  FORWARD = 1,
  BACKWARD = 2,
  LEFT = 3,
  RIGHT = 4,
} TDirection;

volatile TDirection dir = STOP;

// Motor control pins. You need to adjust these till
// Alex moves in the correct direction
#define LF                  5   // Left forward pin 
#define LR                  6   // Left reverse pin
#define RF                  11  // Right forward pin
#define RR                  10  // Right reverse pin

/*
 * 
 * Alex Communication Routines.
 * 
 */
 
TResult readPacket(TPacket *packet)
{
    // Reads in data from the serial port and
    // deserializes it.Returns deserialized
    // data in "packet".
    
    char buffer[PACKET_SIZE];
    int len;

    len = readSerial(buffer);

    if(len == 0)
      return PACKET_INCOMPLETE;
    else
      return deserialize(buffer, len, packet);
    
}

void sendMessage(const char *message)
{
  // Sends text messages back to the Pi. Useful
  // for debugging.
  
  TPacket messagePacket;
  messagePacket.packetType=PACKET_TYPE_MESSAGE;
  strncpy(messagePacket.data, message, MAX_STR_LEN);
  sendResponse(&messagePacket);
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
  badCommand.packetType=PACKET_TYPE_ERROR;
  badCommand.command=RESP_BAD_COMMAND;
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

void sendResponse(TPacket *packet)
{
  // Takes a packet, serializes it then sends it out
  // over the serial port.
  char buffer[PACKET_SIZE];
  int len;

  len = serialize(buffer, packet, sizeof(TPacket));
  writeSerial(buffer, len);
}

/*
 * Setup and start codes for serial communications
 * 
 */
// Set up the serial connection.
void setupSerial()
{
  Serial.begin(9600);
}

// Read the serial port. Returns the read character in
// ch if available. Also returns TRUE if ch is valid. 

int readSerial(char *buffer)
{

  int count=0;

  while(Serial.available())
    buffer[count++] = Serial.read();

  return count;
}

// Write to the serial port.

void writeSerial(const char *buffer, int len)
{
  Serial.write(buffer, len);
}

/*
 * Alex's motor drivers.
 * 
 */

// Set up Alex's motors.
void setupMotors()
{
  //Setting LF, LR, RF, RR to output pins
  DDRD |= 0b01100000;
  DDRB |= 0b00001100;
}

//movement commands move with a fixed delay
// dir = STOP at the end of every movement function
// so that the arduino can go to IDLE if no commands sent from the Pi

void forward()
{  
  dir = FORWARD;
  
  PORTD |= 0b00100000; // output HIGH TO LF
  PORTB |= 0b00001000; // output HIGH TO RF
  delay(250);
  PORTD &= 0b11011111; // output LOW TO LF
  PORTB &= 0b11110111; // output LOW TO RF

  dir = STOP;
}

void reverse()
{ 
  dir = BACKWARD;

  PORTD |= 0b01000000; // output HIGH TO LR
  PORTB |= 0b00000100; // output HIGH TO RR
  delay(250);
  PORTD &= 0b10111111; // output HIGH TO LR
  PORTB &= 0b11111011; // output HIGH TO RR

  dir = STOP;
}

void left()
{  
  dir = LEFT;

  PORTD |= 0b01000000; // output HIGH TO LR
  PORTB |= 0b00001000; // output HIGH TO RF
  delay(150);
  PORTD &= 0b10111111; // output LOW TO LR
  PORTB &= 0b11110111; // output LOW TO RF

  dir = STOP;
}

void right()
{
  dir = RIGHT;

  PORTD |= 0b00100000; // output HIGH TO LF
  PORTB |= 0b00000100; // output HIGH TO RR
  delay(150);
  PORTD &= 0b11011111; // output LOW TO LF
  PORTB &= 0b11111011; // output LOW TO RR

  dir = STOP;
}

void stop()
{
  dir = STOP;
  
  //output LOW to LF, LR, RF, RR
  PORTD &= 0b10011111;
  PORTB &= 0b11110011;
}

/*
 * Alex's setup and run codes
 * 
 */


void handleCommand(TPacket *command)
{
  switch(command->command)
  {
    case COMMAND_FORWARD:
        sendOK();
        forward();
        break;

    case COMMAND_REVERSE:
        sendOK();
        reverse();
        break;

    case COMMAND_TURN_LEFT:
        sendOK();
        left();
        break;

    case COMMAND_TURN_RIGHT:
        sendOK();
        right();
        break;

    case COMMAND_STOP:
        sendOK();
        stop();
        break;
    /*
     * Implement code for other commands here if needed.
     * 
     */
        
    default:
      sendBadCommand();
  }
}

void waitForHello()
{
  int exit=0;

  while(!exit)
  {
    TPacket hello;
    TResult result;
    
    do
    {
      result = readPacket(&hello);
    } while (result == PACKET_INCOMPLETE);

    if(result == PACKET_OK)
    {
      if(hello.packetType == PACKET_TYPE_HELLO)
      {
        sendOK();
        exit=1;
      }
      else
        sendBadResponse();
    }
    else
      if(result == PACKET_BAD)
      {
        sendBadPacket();
      }
      else
        if(result == PACKET_CHECKSUM_BAD)
          sendBadChecksum();
  } // !exit
}

/*
 * Alex's power management codes
 * 
 */

void WDT_off(void){
  /* Global interrupt should be turned OFF here if not already done so */

  // Clear WDRF in MCUSR
  MCUSR &= ~(1<<WDRF);

  // Write logical one to WDCE and WDE
  // Keep old prescaler setting to prevent unintentional time-out
  WDTCSR |= (1<<WDCE) | (1<<WDE);

  //Turn off WDT
  WDTCSR = 0x00;

  //Global interrupt should be turned ON here if subsequent operations after calling this function do not require turning off global interrupt
}

void setupPowerSaving(){
  // Turn off the Watchdog Timer
  WDT_off();
  // Modify PRR to shut down TWI
  PRR |= PRR_TWI_MASK;
  // Modify PRR to shut down SPI
  PRR |= PRR_SPI_MASK;
  // Modify ADCSRA to disable ADC,
  // then modify PRR to shut down ADC
  ADCSRA |= ADCSRA_ADC_MASK;
  PRR |= PRR_ADC_MASK;
  // Set the SMCR to choose the IDLE sleep mode
  // Do not set the Sleep Enable (SE) bit yet
  SMCR &= SMCR_IDLE_MODE_MASK;
  // Set Port B Pin 5 as output pin, then write a logic LOW
  // to it such that the LED tied to Arduino's Pin 13 is OFF.
  // e.g.
  // DDRB |= 0b00100000; // Arduino PIN 13 only
  // PORTB &= 0b11011111;

  // Pins in use are Arduino pins 5,6,10 & 11, the rest will be turned off accordingly like above
  DDRD |= 0b1001111;
  PORTD &= 0b01100000;

  DDRB |= 0b00110011;
  PORTD &= 0b11001100;
}

void putArduinoToIdle(){
  // Modify PRR to shut down TIMER 0, 1 and 2
  PRR |= PRR_TIMER0_MASK;
  PRR |= PRR_TIMER1_MASK;
  PRR |= PRR_TIMER2_MASK;
  // Modify SE bit in SMCR to enable (i.e., allow) sleep
  SMCR |= SMCR_SLEEP_ENABLE_MASK;
  // The following function puts ATmega328P's MCU into sleep;
  // it wakes up from sleep when USART serial data arrives sleep_cpu();
  sleep_cpu();
  // Modify SE bit in SMCR to disable (i.e., disallow) sleep
  SMCR &= (~SMCR_SLEEP_ENABLE_MASK);
  // Modify PRR to power up TIMER 0, 1, and 2
  PRR &= (~PRR_TIMER0_MASK);
  PRR &= (~PRR_TIMER1_MASK);
  PRR &= (~PRR_TIMER2_MASK);
}

void setup() {
  // put your setup code here, to run once:
  cli();
  setupSerial();
  setupMotors();
  setupPowerSaving();
  sei();
}

void handlePacket(TPacket *packet)
{
  switch(packet->packetType)
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

//put your main code here, to run repeatedly:

if (dir == STOP){
  putArduinoToIdle();
}
TPacket recvPacket; // This holds commands from the Pi

TResult result = readPacket(&recvPacket);
  
if(result == PACKET_OK)
  handlePacket(&recvPacket);
else 
  if(result == PACKET_BAD)
  { 
    sendBadPacket();
  }
  else 
    if(result == PACKET_CHECKSUM_BAD)
    {
      sendBadChecksum();
    } 
}
