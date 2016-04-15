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
#include <FreeRTOS_ARM.h>

#define SAMPLE_RATE 50000
#define SAMPLE_SIZE 9
#define SAMPLE_NUMBER 10

char cmd[5];
int cmdCounter;
boolean cmdWrite;
boolean startFlag;

char parameter[10];
int parCounter;
boolean parWrite;
int parLen;

char sync[5] = "5566";
/* Dammy task for Interpreter */
static void commandInterpreter(void *arg) {
  while(1) {
    if (SerialUSB.available()){
      char c = SerialUSB.read();
      command_parser(c);
    } else {
      taskYIELD();
    }
  }
}

/* PacketSender task stores the averaged samples in the packet and sends packets to the terget */
static void packetSender(void *arg) {  
  char *packet = (char *)malloc(SAMPLE_SIZE*SAMPLE_NUMBER); 

  while(1) {
    for (int i = 0; i < SAMPLE_NUMBER; i++){
      // Wait until it gets a queue which sent by the bufferFullInterrupt 
      PowerDue.queueReceive();
      PowerDue.writeAverage(packet+i*SAMPLE_SIZE);
      for (int j = 1; j < 8; j+=2){
        SerialUSB.print((*(uint16_t *)(packet+i*SAMPLE_SIZE+j))&0x0FFF, DEC);
        SerialUSB.print("  ");
      }  
      SerialUSB.println();  
    }
    
    if (startFlag) {
      SerialUSB.write(sync, 4);
      //SerialUSB.write(packet, SAMPLE_SIZE*SAMPLE_NUMBER);

      //SerialUSB.print((uint8_t *)packet, HEX);
      SerialUSB.println();
    }
    //SerialUSB.write(packet, SAMPLE_SIZE*SAMPLE_NUMBER);
    
  }
}

void setup(){
  cmdCounter = 0;
  cmdWrite = false;
  startFlag = false;

  parCounter = 0;
  parWrite = true;
  parLen = 0;

  
  SerialUSB.begin(0);
  while(!SerialUSB);
  //Serial3.begin(9600);
  //while(!Serial3);
  PowerDue.init(SAMPLE_RATE);
  //PowerDue.startSampling();
  
  xTaskCreate(commandInterpreter, NULL, 8*configMINIMAL_STACK_SIZE, 0, 1, NULL);
  xTaskCreate(packetSender, NULL, 8*configMINIMAL_STACK_SIZE, 0, 1, NULL); 
  SerialUSB.println("start");
  vTaskStartScheduler();
  SerialUSB.println("Insufficient RAM");
  while(1);
}

void loop(){

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
    startFlag = true;
    PowerDue.startSampling();
  }
  else if (strncmp(cmd_header, "*SMP", 4) == 0) {
    par_len = 4;

  }
  else if (strncmp(cmd_header, "*STP", 4) == 0) {
    par_len = 0;
    startFlag = false;
    PowerDue.stopSampling();
  }
  else if (strncmp(cmd_header, "*TRG", 4) == 0) {
    par_len = 4;
  }
  
  return par_len;
}


