/*
J.Teda 21/04/2013

Exsample of how to run Dynamixel in SERVO mode - tell servo what angle to move to.

Robotis e-Manual ( http://support.robotis.com )
 
*/

#include <Dynamixel_Serial.h>       // Library needed to control Dynamixal servo
#define AVR_ATmega32U4 1
#define SERVO_ID 0x02               // ID of which we will set Dynamixel too 
#define SERVO_ControlPin 0x02       // Control pin of buffer chip, NOTE: this does not matter becasue we are not using a half to full contorl buffer.
#define SERVO_SET_Baudrate 57142    // Baud rate speed which the Dynamixel will be set too (57142)
#define CW_LIMIT_ANGLE 0x001        // lowest clockwise angle is 1, as when set to 0 it set servo to wheel mode
#define CCW_LIMIT_ANGLE 0xFFF       // Highest anit-clockwise angle is 0XFFF, as when set to 0 it set servo to wheel mode
byte cmd[] = {
  0xff, 0xff, 0, 0x07, 0x03, 0x1e, 0, 0, 0, 0, 0};

void setup(){
delay(1000);                                                           // Give time for Dynamixel to start on power-up
Serial.begin(57600);
//Dynamixel.begin(SERVO_SET_Baudrate, SERVO_ControlPin);        			// We now need to set Ardiuno to the new Baudrate speed 
//Dynamixel.setMode(SERVO_ID, SERVO, CW_LIMIT_ANGLE, CCW_LIMIT_ANGLE);    // set mode to SERVO and set angle limits
mx28_init();
}


void loop(){
//  Dynamixel.servo(SERVO_ID,0x001,0x100);   // Move servo to angle 1(0.088 degree) at speed 100
//  delay(4000); 
//  
//  Dynamixel.servo(SERVO_ID,0xFFF,0x3FF);  //  Move servo to max angle at max speed 
//  delay(4000);
// New Code=============================================

 
  if (Serial.available() > 0) {

      
      //interpret 3 values (motor id number, position, and velocity)
      
      int count=0;
      int index=0;
      int temp=0;
      char input=0;
      
      char buffer[] = {' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' '}; // Receive up to 3 bytes
      while (!Serial.available()); // Wait for characters
      Serial.readBytesUntil(32,buffer, 15);
    int motorID = (byte)buffer[0]; // 1st motor ID
    int vel = (byte)buffer[1] * 4;
    int pos = (byte)buffer[2] * 16;
    int motorID1 = (byte)buffer[3]; //2nd motor ID
    int vel1 = (byte)buffer[4] * 4;
    int pos1= (byte)buffer[5] * 16;
    int motorID2 = (byte)buffer[6]; //3rd motor ID
    int vel2 = (byte)buffer[7] * 4;
    int pos2= (byte)buffer[8] * 16;
    int motorID3 = (byte)buffer[9]; //4th motor ID
    int vel3 = (byte)buffer[10] * 4;
    int pos3= (byte)buffer[11] * 16;
    int motorID4 = (byte)buffer[12]; //5th motor ID
    int vel4 = (byte)buffer[13] * 4;
    int pos4= (byte)buffer[14] * 16;
    
//    Dynamixel.servo(motorID,pos,vel);
//    Dynamixel.servo(motorID1,pos1,vel1);
//    Dynamixel.servo(motorID2,pos2,vel2);
//    Dynamixel.servo(motorID3,pos3,vel3);
//    Dynamixel.servo(motorID4,pos4,vel4);
mx28_setPosVel((byte)motorID, pos, vel);
mx28_setPosVel((byte)motorID1, pos1, vel1);
mx28_setPosVel((byte)motorID2, pos2, vel2);
mx28_setPosVel((byte)motorID3, pos3, vel3);
mx28_setPosVel((byte)motorID4, pos4, vel4);
  }
}

void mx28_setPosVel(byte id, int pos, int vel)
{
  byte checksum = 0;
  cmd[2] = id;  checksum+= id;
  checksum += 0x07;
  checksum += 0x03;
  checksum+= 0x1e;
  cmd[6] = lowByte(pos);  checksum += lowByte(pos);
  cmd[7] = highByte(pos);  checksum+= highByte(pos);
  cmd[8] = lowByte(vel);  checksum += lowByte(vel);
  cmd[9] = highByte(vel);  checksum+= highByte(vel);
  cmd[10] = (~checksum);
//  Serial.println("Sent "); 
//  for (int i=0; i<11; i++)
//  {
//    Serial.print(cmd[i],HEX);
//    Serial.print(".");
//  }
  Serial1.write(cmd,11);
}

void mx28_init()
{
  Serial1.begin(250000);  
  bitClear(UCSR1B, RXCIE1);    // disable receive interrupt
  bitClear(UCSR1B, RXEN1);     // disable receive
  bitSet(UCSR1B, TXEN1);       // enable transmission
}
