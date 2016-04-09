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

#define SAMPLE_RATE 50000

void setup(){

  // Initialize USB Comms
  SerialUSB.begin(0);
  // Wait for connection?
  while(!SerialUSB);


  PowerDue.init(SAMPLE_RATE);
  PowerDue.startSampling();
}

void loop(){
  while(1){
    // wait for the sampling to start

    //ONly after 1 write does this hold
    while(!PowerDue.bufferReady());
    PowerDue.writeBuffer(&SerialUSB);
  }
}
