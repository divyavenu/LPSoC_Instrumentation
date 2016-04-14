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

void setup(){

  // Initialize USB Comms
  Serial3.begin(9600);
  // Wait for connection?
  while(!Serial3);


  PowerDue.init(SAMPLE_RATE);
  PowerDue.startSampling();
}

void loop(){
//  while(1){
//    // wait for the sampling to start
//    while(!PowerDue.bufferReady());
//    PowerDue.writeBuffer(&Serial3);
//  }

  if (Serial3.available() > 0) {
      
  }
  
}

