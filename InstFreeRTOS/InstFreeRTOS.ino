#define INSTRUMENT_POWER_DUE
#include <PowerDue.h>
#include <FreeRTOS_ARM.h>

#define SAMPLE_RATE 80000
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

/* Command Interpreter task */
static void commandInterpreter(void *arg) {
  while(1) {
    /*if (Serial3.available()){
      char c = Serial3.read();
      command_parser(c);
    } else {
      taskYIELD();
    }*/
    char data;
        if( Serial3.RxQueue != 0 )
    {
        // Receive a message on the created queue.  Block for 10 ticks if a
        // message is not immediately available.
        if( xQueueReceive( Serial3.RxQueue, &( data ), portMAX_DELAY) )
        {
              command_parser(data);
              SerialUSB.println(data);
            // pcRxedMessage now points to the struct AMessage variable posted
            // by vATask.
        }
    }


  }
}

//------------------------------------------------------------------------------
/* PacketSender task stores the averaged samples in the packet and sends packets to the terget */
static void packetSender(void *arg) {
  char sync[5] = "5566";
  char packet[91];
  
  while(1) {
    for (int i = 0; i < SAMPLE_NUMBER; i++){
      // Wait until it gets a queue which sent by the bufferFullInterrupt 
      PowerDue.queueReceive();
      // Skip average if there is no sampling data in the current buffer
      if (!PowerDue.writeAverage(packet+i*SAMPLE_SIZE))
        i--;  
    } 
    if (startFlag) {
      Serial3.write(sync, 4);
      Serial3.write(packet, SAMPLE_SIZE*SAMPLE_NUMBER);
    }    
  }
}

//------------------------------------------------------------------------------
void setup(){
  cmdCounter = 0;
  cmdWrite = false;
  startFlag = false;
  parCounter = 0;
  parWrite = false;
  parLen = 0;
  
  Serial3.begin(9600);
  while(!Serial3);
  PowerDue.init(SAMPLE_RATE);
//  PowerDue.startSampling();
//  startFlag = true;
//  SerialUSB.begin(0);
//  while(!SerialUSB);
  
  xTaskCreate(commandInterpreter, NULL, 8*configMINIMAL_STACK_SIZE, 0, 1, NULL);
  xTaskCreate(packetSender, NULL, 8*configMINIMAL_STACK_SIZE, 0, 1, NULL); 
  
  vTaskStartScheduler();
  SerialUSB.println("Insufficient RAM");
  while(1);
}

void loop(){

}

//------------------------------------------------------------------------------
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


