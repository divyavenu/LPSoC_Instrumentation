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

char cmd[5];
int cmdCounter;
boolean cmdWrite;

char parameter[10];
int parCounter;
boolean parWrite;
int parLen;

char test[91] 
  = "This is for the woman who gave me reason to push in my life This is for the woman who gave";

void setup(){
  cmdCounter = 0;
  cmdWrite = false;

  parCounter = 0;
  parWrite = false;
  parLen = 0;
  
  Serial3.begin(9600);
  while(!Serial3);

//  PowerDue.init(SAMPLE_RATE);
//  PowerDue.startSampling();
}

void command_parser(char c) {
  // the lenghth of parameter (in byte)
  // so know how many bytes to read
  if (c == '*') {
    cmd[cmdCounter] = c;
    cmdCounter ++;
    cmdWrite = true;
    parWrite = false;
  }
  else if (cmdWrite) {
    if (cmdCounter < 4) {
      cmd[cmdCounter] = c; 
      cmdCounter ++; 
    }
    if (cmdCounter == 4) {
      SerialUSB.print("[CMD]: ");
      SerialUSB.println(cmd);
      cmdCounter = 0;
      parCounter = 0;
      cmdWrite = false;
      parWrite = true; 
      parLen = command_interpreter(cmd);   
    }
  }
  else if (parWrite) {
    if (parLen == 0) {
      return;  
    }
   
    parameter[parCounter] = c;
    parCounter ++;

    // finished going through parameter (byte by byte)
    if (parCounter == parLen) {
      parameter[parCounter] = '\0';
      SerialUSB.print("[PARAMETER]: ");
      SerialUSB.println(parameter);
      parCounter = 0;
      parWrite = false;
    }
  }
}

int command_interpreter(char* cmd_header) {
  int par_len = 0;
  if (strncmp(cmd_header, "*STR", 4) == 0) {
    par_len = 0;
    SerialUSB.println("[CMD INT]: STR");
    Serial3.write(test, 90);
  }
  else if (strncmp(cmd_header, "*SMP", 4) == 0) {
    par_len = 4;
    SerialUSB.println("[CMD INT]: SMP");
    Serial3.write(test, 90);
  }
  else if (strncmp(cmd_header, "*STP", 4) == 0) {
    par_len = 0;
    SerialUSB.println("[CMD INT]: STP");
    Serial3.write(test, 90);
  }
  else if (strncmp(cmd_header, "*TRG", 4) == 0) {
    par_len = 4;
    SerialUSB.println("[CMD INT]: TRG");
    Serial3.write(test, 90);
  }
  
  return par_len;
}


void loop(){
//  while(1){
//    // wait for the sampling to start
//    while(!PowerDue.bufferReady());
//    PowerDue.writeBuffer(&Serial3);
//  }

  delay(2000);
  while (Serial3.available()) {
    char c = Serial3.read();
//    SerialUSB.println(c, HEX);
    command_parser(c);
  }
}

