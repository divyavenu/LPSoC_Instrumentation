/*
*
* Firmware for the PowerDué (Based on the Arduino Due)
* Carnegie Mellon University Silicon Valley
* Version: 0.0.1
*
*/

// Used for Selecting between Target and Instrument in <PowerDue.h>
#define INSTRUMENT_POWER_DUE
#include <PowerDue.h>

#define SAMPLE_RATE 500

// just make sure 1 ending byte to null terminator
char command1[9] = "*SMPABCD";
char command2[5] = "*STR";
char command3[5] = "*STP";
char command4[9] = "*TRGDBDF";
boolean cmdFlip;

char test[91];
int randomNumber;

void setup(){
  cmdFlip = false;
  
  // Initialize USB Comms
  Serial3.begin(9600);
  randomSeed(analogRead(0));
  
  // Wait for connection?
  while(!Serial3);
}

void loop(){
//  while(1){
//    // wait for the sampling to start
//    while(!PowerDue.bufferReady());
//    PowerDue.writeBuffer(&Serial3);
//  }
  // random number from 1 to 4
  Serial3.write(command1, 8);
  
  while(Serial3.available()) {
    Serial3.readBytes(test, 90);
    SerialUSB.println("POEM");
    SerialUSB.println(test);
    delay(500);
  }
//  
//  randomNumber = random(1, 5);
//  delay(2000);
//
//  switch (randomNumber) {
//    case 1: 
//      SerialUSB.println(command1);
//      Serial3.write(command1, 8);
//      break;
//    case 2:
//      SerialUSB.println(command2);
//      Serial3.write(command2, 4);
//      break;
//    case 3:
//      SerialUSB.println(command3);
//      Serial3.write(command3, 4);
//      break;
//    case 4:
//      SerialUSB.println(command4);
//      Serial3.write(command4, 8);
//      break;            
//  } 
}

