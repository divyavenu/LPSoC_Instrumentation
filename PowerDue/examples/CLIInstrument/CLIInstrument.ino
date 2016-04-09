/*
*
* Firmware using a CLI for the PowerDué (Based on the Arduino Due)
* Carnegie Mellon University Silicon Valley
* Version: 0.0.1
*
*/

// Used for Selecting between Target and Instrument in <PowerDue.h>
#define INSTRUMENT_POWER_DUE
#include <PowerDue.h>

#define SAMPLE_RATE 50000

#define CLI_BUFFER_SIZE 100

char commandBuffer[CLI_BUFFER_SIZE] = "";
char command[CLI_BUFFER_SIZE] = "";
char arg[CLI_BUFFER_SIZE] = "";
char c = '\0';


void setup(){
  // Initialize USB Comms
  SerialUSB.begin(0);
  // Wait for connection?
  while(!SerialUSB);

  PowerDue.init(SAMPLE_RATE);
  PowerDue.startSampling();
}

void loop(){
  if(SerialUSB.available()){
      commandBuffer[i] = SerialUSB.read();
      if(commandBuffer[i] == '\n'){
        i = 0;
        executeCommand();
      }else if(i<CLI_BUFFER_SIZE-2){
        i++;
      }else{
        i=0;
      }
  }

  // wait for the sampling to start
  if(PowerDue.bufferReady()){
    PowerDue.writeBuffer(&SerialUSB);
  }

}

void executeCommand(){
  sscanf(commandBuffer, "%s %s", &command, &arg);
  
  commandBuffer
}
