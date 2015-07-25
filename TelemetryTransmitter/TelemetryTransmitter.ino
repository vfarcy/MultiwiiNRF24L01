/*  
A basic 4 channel transmitter using the nRF24L01 module.
*/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "printf.h"

const uint64_t pipeOut = 0xE8E8F0F0E1LL;

RF24 radio(9, 10);

const byte serialHeader[4] = {255,254,253,252};

// The sizeof this struct should not exceed 32 bytes
struct RadioData {
  byte throttle;
  byte yaw;
  byte pitch;
  byte roll;
  byte dial1;
  byte dial2;
  byte switches; // bitflag
};

RadioData radioData;

struct AckPayload {
  float lat;
  float lon;
  int16_t heading;
  int16_t pitch;
  int16_t roll;  
  int32_t alt;
  byte flags;
};

AckPayload ackPayload;

void resetRadioData() 
{
  radioData.throttle = 127;
  radioData.yaw = 127;
  radioData.pitch = 127;
  radioData.roll = 127;
  radioData.dial1 = 0;
  radioData.dial2 = 0;
  radioData.switches = 0;
}

void setup()
{
  Serial.begin(57600);
  printf_begin();
  
  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.setAutoAck(1);                    // Ensure autoACK is enabled
  radio.enableAckPayload();               // Allow optional ack payloads

  radio.openWritingPipe(pipeOut);

  resetRadioData();
  
  radio.printDetails();
}

/**************************************************/

// Returns a corrected value for a joystick position that takes into account
// the values of the outer extents and the middle of the joystick range.
int mapJoystickValues(int val, int lower, int middle, int upper, bool reverse)
{
  val = constrain(val, lower, upper);
  if ( val < middle )
    val = map(val, lower, middle, 0, 128);
  else
    val = map(val, middle, upper, 128, 255);
  return ( reverse ? 255 - val : val );
}

/**************************************************/

void cleanData()
{
  if ( radioData.throttle == 255 ) radioData.throttle = 254;
  if ( radioData.yaw == 255 ) radioData.yaw = 254;
  if ( radioData.pitch == 255 ) radioData.pitch = 254;
  if ( radioData.roll == 255 ) radioData.roll = 254;
  if ( radioData.switches == 255 ) radioData.switches = 254;

  if ( ackPayload.alt == 255 ) ackPayload.alt = 254; // not quite sufficient, but unlikely that any other bytes of the altitude will be 255
  ackPayload.flags &= 0x2; // ok because we are only interested in bit 2 (ARMED)
}

/**************************************************/

void adjustAckPayloadValues()
{
  ackPayload.pitch *= 0.1;
  ackPayload.roll *= 0.1;
}

/**************************************************/

int pps = 0;

void writeDataToSerial() 
{
  cleanData();
  
  /*
  Serial.print("  Pitch "); Serial.print(ackPayload.pitch);
  Serial.print("  Roll ");  Serial.print(ackPayload.roll);
  Serial.print("  Yaw ");   Serial.print(ackPayload.heading);
  Serial.print("  Alt ");   Serial.print(ackPayload.alt);
  Serial.print("  Flags "); Serial.print(ackPayload.flags);
  Serial.print("  PPS ");   Serial.println(pps);
  */
    
  Serial.write(serialHeader, 4);
  Serial.write((uint8_t*)&radioData, sizeof(RadioData));
  Serial.write((uint8_t*)&ackPayload, sizeof(AckPayload));
  Serial.write((uint8_t*)&pps, 2);
}

/**************************************************/

unsigned long lastPPS = 0;
int ppsCounter = 0;




void loop()
{
  boolean mode1 = !digitalRead(2);
  
  // The calibration numbers used here should be measured 
  // for your joysticks using the TestJoysticks sketch.
  if ( mode1 ) {
    radioData.throttle = mapJoystickValues( analogRead(A0), 180, 497, 807, true );
    radioData.yaw      = mapJoystickValues( analogRead(A3), 165, 564, 906, false );
    radioData.pitch    = mapJoystickValues( analogRead(A2), 136, 462, 779, true );
    radioData.roll     = mapJoystickValues( analogRead(A1), 109, 460, 848, true );
  }
  else {
    radioData.throttle = mapJoystickValues( analogRead(A0), 180, 497, 807, false );
    radioData.yaw      = mapJoystickValues( analogRead(A1), 109, 460, 848, false );
    radioData.pitch    = mapJoystickValues( analogRead(A2), 136, 462, 779, false );
    radioData.roll     = mapJoystickValues( analogRead(A3), 165, 564, 906, true );
  }

  radioData.dial1    = constrain( map( analogRead(A7), 968,   27, 0, 255 ), 0, 255);
  radioData.dial2    = constrain( map( analogRead(A6),   1, 1020, 0, 255 ), 0, 255);
  
  radioData.switches = 0;
  if ( ! digitalRead(4) ) radioData.switches |= 0x1;
  if ( ! digitalRead(2) ) radioData.switches |= 0x2;

  radio.write(&radioData, sizeof(RadioData));
  
  while ( radio.isAckPayloadAvailable()) {
    radio.read(&ackPayload, sizeof(AckPayload));
    ppsCounter++;
  }
  
  unsigned long now = millis();
  if ( now - lastPPS > 1000 ) {
    pps = ppsCounter;
    ppsCounter = 0;
    lastPPS = now;
  }
  
  adjustAckPayloadValues();
  writeDataToSerial();
   
}

