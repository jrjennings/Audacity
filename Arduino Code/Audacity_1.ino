
/*AUDACITY*/
//updated 3/19/2023 

#include <DynamixelShield.h>
#include "RoboClaw.h"
#define roboClawSerial Serial1
#define bluetoothSerial Serial2
#include <Arduino.h>
RoboClaw roboclaw(&roboClawSerial,10000);

#define address1 0x80 //left side motors
#define address2 0x81 //right side motors

#include <SoftwareSerial.h>
SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
#define DEBUG_SERIAL soft_serial

const uint8_t DXL_ID1 = 1;  //right front wheel
const uint8_t DXL_ID2 = 2;  //left front wheel
const uint8_t DXL_ID3 = 3;  //right rear wheel
const uint8_t DXL_ID4 = 4;  //left rear wheel
const float DXL_PROTOCOL_VERSION = 2.0;

//1 meter is = 1363 encoder counts

DynamixelShield dxl;

//This namespace is required to use Control table item names
using namespace ControlTableItem;

int s =  0; // rover speed encoder counts per second
float r = 0; // turning radius in meters
float thetaInner, thetaOuter = 0;
float speed1, speed2 = 0;

float d1 = 0.2325; // distance in meters
float d2 = 0.209;

int tab = 0;
int steering = 0;
int velocity = 0;
int triggerStateChange = 0;

//Wheel Alignment
//right front,left front, right rear, left rear
int Align[] = { -19,9,0,13};

void setup() {
  //Open Serial and roboclaw serial ports
  roboclaw.begin(38400);
  bluetoothSerial.begin(9600);  // set line end to Newline at 
  bluetoothSerial.setTimeout(50000);

  // For Uno, Nano, Mini, and Mega, use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);
  
  // Set Port baudrate to match with DYNAMIXEL baudrate.
  dxl.begin(1000000);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  dxl.ping(DXL_ID1);
  dxl.ping(DXL_ID2);
  dxl.ping(DXL_ID3);
  dxl.ping(DXL_ID4);

  // Turn off torque when configuring items in EEPROM area
  dxl.torqueOff(DXL_ID1);
  dxl.setOperatingMode(DXL_ID1, OP_POSITION);
  dxl.torqueOn(DXL_ID1);

  dxl.torqueOff(DXL_ID2);
  dxl.setOperatingMode(DXL_ID2, OP_POSITION);
  dxl.torqueOn(DXL_ID2);

  dxl.torqueOff(DXL_ID3);
  dxl.setOperatingMode(DXL_ID3, OP_POSITION);
  dxl.torqueOn(DXL_ID3);

  dxl.torqueOff(DXL_ID4);
  dxl.setOperatingMode(DXL_ID4, OP_POSITION);
  dxl.torqueOn(DXL_ID4);
}

void loop() {

  int tab;   //Tab
  int val;
  int val1;  //Steering
  int val2;  //Velocity
  int val3;  //Rotate
  int alignCorner;
  int alignmentChange;
  int triggerStateChange; 
  
  //added for BLE Versions
  if (bluetoothSerial.available() > 0){
    tab = bluetoothSerial.parseInt();   //tab4,tab3
    val1 = bluetoothSerial.parseInt();  //steering,alignCorner
    val2 = bluetoothSerial.parseInt();  //velocity,alignmentChange
    val3 = bluetoothSerial.parseInt();  //triggerStateChange,
    char r = bluetoothSerial.read();
    if(r == '\n\r'){}
  }
  else {  // if not connected to Bluetooth, rover should stop
    tab = 0;
    val1 = 0; 
    val2 = 0;  
    val3 = 0;
  }

/******************************************************************************************************
Wheel Alignment
******************************************************************************************************/
  if (tab == 3){
    // Align[Right Front, Left Front, Right Rear, Left Rear]
    alignCorner = val1;  
    alignmentChange = val2;
    triggerStateChange = val3;
    
    if ((alignCorner == 1) && (alignmentChange == 1)){  //rf
        val = Align[0] + 1;
        Align[0] = (val);
      }
    if ((alignCorner == 1) && (alignmentChange == 2)){  //rf
        val = Align[0] - 1;
        Align[0] = (val);
      }
    if ((alignCorner == 2) && (alignmentChange == 1)){  //lf
        val = Align[1]+ 1;
        Align[1] = (val);
      }
    if ((alignCorner == 2) && (alignmentChange == 2)){  //lf
        val = Align[1]- 1;
        Align[1] = (val);
      }
    if ((alignCorner == 3) && (alignmentChange == 1)){  //rr
        val = Align[2]+ 1;
        Align[2] = (val);
      }
    if ((alignCorner == 3) && (alignmentChange == 2)){  //rr
        val = Align[2] - 1;
        Align[2] = (val);
      }
    if ((alignCorner == 4) && (alignmentChange == 1)){  //lr
        val = Align[3] + 1;
        Align[3] = (val);
      }
    if ((alignCorner == 4) && (alignmentChange == 2)){  //lr
        val = Align[3] - 1;
        Align[3] = (val);
      }      
    }

/******************************************************************************************************
Remote Drive
******************************************************************************************************/
  if (tab == 4){
    steering = val1;
    velocity = val2;
    triggerStateChange = val3;
  }

/******************************************************************************************************/
  
 // int steering = val1;
  r = abs(d2 * (1/sin(steering * PI / 180))); // get length of turning radius in radians
  s = val2;

  thetaOuter = round((atan((d2 / (r + d1)))) * 180 / PI);
  thetaInner = round((atan((d2 / (r - d1)))) * 180 / PI);

  //turning right
  if (triggerStateChange == 0 && steering > 0){ 
    // Outer wheels  
    speed1 = s;
    // Inner are closer to the turing point and have lower speeds compared to the outer wheels
    speed2 = s * sqrt(pow(d2, 2) + pow((r - d1), 2)) / (r + d1);
    

    //outside left motors 2,4 speed 1
    //inside right motors 1,3 speed 2
    
    roboclaw.SpeedM1(address2,round(speed2)); // Motor 1 RF
    roboclaw.SpeedM1(address1,round(speed1)); // Motor 2 LF
    roboclaw.SpeedM2(address2,round(speed2)); // Motor 3 RR
    roboclaw.SpeedM2(address1,round(speed1)); // Motor 4 LR
    
    dxl.setGoalPosition(DXL_ID1, round(180 + thetaInner + Align[0]), UNIT_DEGREE);  //Right Front
    dxl.setGoalPosition(DXL_ID2,round(180 + thetaOuter + Align[1]), UNIT_DEGREE);   //Left Front
    dxl.setGoalPosition(DXL_ID3, round(180 - thetaInner + Align[2]), UNIT_DEGREE);  //Right Rear
    dxl.setGoalPosition(DXL_ID4,round(180 - thetaOuter + Align[3]), UNIT_DEGREE);   //Left Rear
}

  //turning left
  if (triggerStateChange == 0 && steering < 0){
    // Outer wheels 
    speed1 = s;
    // Inner front and back wheels are closer to the turing point and have lower speeds compared to the outer speeds
    speed2 = s * sqrt(pow(d2, 2) + pow((r - d1), 2)) / (r + d1);

  
    //outside right motors 1,3 speed1
    //inside left motors 2,4 speed2
    
    roboclaw.SpeedM1(address2,round(speed1));  //Motor 1 RF
    roboclaw.SpeedM1(address1,round(speed2));  //Motor 2 LF
    roboclaw.SpeedM2(address2,round(speed1));  //Motor 3 RR
    roboclaw.SpeedM2(address1,round(speed2));  //Motor 4 LR
 
    dxl.setGoalPosition(DXL_ID1,round(180 - thetaOuter + Align[0]), UNIT_DEGREE);
    dxl.setGoalPosition(DXL_ID2,round(180 - thetaInner + Align[1]), UNIT_DEGREE);
    dxl.setGoalPosition(DXL_ID3,round(180 + thetaOuter + Align[2]), UNIT_DEGREE);
    dxl.setGoalPosition(DXL_ID4,round(180 + thetaInner + Align[3]), UNIT_DEGREE);
  }

  // going straight
  if (triggerStateChange == 0 && steering == 0){
    speed1 = speed2 = s;
    
    roboclaw.SpeedM1(address2,round(speed1)); //Motor 1 RF
    roboclaw.SpeedM1(address1,round(speed1)); //Motor 2 LF
    roboclaw.SpeedM2(address2,round(speed1)); //Motor 3 RR
    roboclaw.SpeedM2(address1,round(speed1)); //Motor 4 LR
  
    dxl.setGoalPosition(DXL_ID1,180 + Align[0], UNIT_DEGREE);
    dxl.setGoalPosition(DXL_ID2,180 + Align[1], UNIT_DEGREE);
    dxl.setGoalPosition(DXL_ID3,180 + Align[2], UNIT_DEGREE);
    dxl.setGoalPosition(DXL_ID4,180 + Align[3], UNIT_DEGREE);
  }
  
  // rotate about center
  if (triggerStateChange == 1){
    speed1 = s;
    speed2 = s * 0.9291;


    roboclaw.SpeedM1(address2,round(speed1));     //Motor 1 RF
    roboclaw.SpeedM1(address1,round(speed2));     //Motor 2 LF
    roboclaw.SpeedM2(address2,round(speed1));     //Motor 3 RR
    roboclaw.SpeedM2(address1,round(speed2));     //Motor 4 LR
    

    dxl.setGoalPosition(DXL_ID1,round(129 + Align[0]), UNIT_DEGREE);
    dxl.setGoalPosition(DXL_ID2,round(231 + Align[1]), UNIT_DEGREE);
    dxl.setGoalPosition(DXL_ID3,round(227 + Align[2]), UNIT_DEGREE);
    dxl.setGoalPosition(DXL_ID4,round(137 + Align[3]), UNIT_DEGREE);
  }
}
