#define INSTRUMENT_POWER_DUE
#include <PowerDue.h>
#include <FreeRTOS_ARM.h>
#include <stdlib.h>     /* strtol */
#include <cerrno>
#include <climits>

#define SAMPLE_RATE 50000

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
//    if (Serial3.available()){
//      char c = Serial3.read();
//      command_parser(c);
//    } else {
//      taskYIELD();
//    }

    char data;
    if( PowerDue.RxQueue != 0 )
    {
        // Receive a message on the created queue.  Block for 10 ticks if a
        // message is not immediately available.
        if( xQueueReceive( PowerDue.RxQueue, &( data ), portMAX_DELAY) )
        {
              command_parser(data);
//              SerialUSB.println(data);
            // pcRxedMessage now points to the struct AMessage variable posted
            // by vATask.
        }
    } 
  }
}

//------------------------------------------------------------------------------
/* Accumlator Task */
static void accumulator(void *arg) {
  while(1) {
    int bid = PowerDue.bufferReady();
    PowerDue.accumStorage(bid);   
  }
}

//------------------------------------------------------------------------------
void setup(){
  cmdCounter = 0;
  cmdWrite = false;
  parCounter = 0;
  parWrite = false;
  parLen = 0;
  startFlag = false;
  
  Serial3.begin(9600);
  while(!Serial3);
  PowerDue.init(SAMPLE_RATE);
  
  xTaskCreate(commandInterpreter, NULL, 8*configMINIMAL_STACK_SIZE, 0, 1, NULL);
  xTaskCreate(accumulator, NULL, 8*configMINIMAL_STACK_SIZE, 0, 1, NULL); 
  
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
      command_interpreter(cmd);
    }
  }
}

// TODO: make this function de-coupled
int command_interpreter(char* cmd_header) {
  int par_len = 0;
  if (strncmp(cmd_header, "*STR", 4) == 0) {
    par_len = 0;
    PowerDue.startSampling();
    PowerDue.initStorage();
    startFlag = true;
  }
  else if (strncmp(cmd_header, "*SMP", 4) == 0) {
    par_len = 4;

  }
  else if (strncmp(cmd_header, "*STP", 4) == 0) {
    par_len = 0;
    startFlag = false;
    PowerDue.stopSampling();
    PowerDue.initStorage();
  }
  else if (strncmp(cmd_header, "*TRG", 4) == 0) {
    par_len = 4;
  }
  else if (strncmp(cmd_header, "*CSF", 4) == 0) {
    // This is a temp fix.
    if (par_len == 0 && parWrite) {
      par_len = 2;
    }
    else {
      int samplingRate = 0;
      char_to_int(parameter, &samplingRate);
      // kHz is the unit
      samplingRate = samplingRate * 1000;
      SerialUSB.print("sampling rate: ");
      SerialUSB.println(samplingRate);
      PowerDue.changeSamplingRate(samplingRate);
    }
  }
  else if (strncmp(cmd_header, "*RDY", 4) == 0) {
    par_len = 4;
    if (startFlag){
      PowerDue.sendHeader(&Serial3);
      PowerDue.sendPacket(&Serial3);
    }
  }  
  return par_len;
}// end command_interpreter

/** https://www.securecoding.cert.org/confluence/display/cplusplus/INT06-CPP.+Use+strtol()+
  * or+a+related+function+to+convert+a+string+token+to+an+integer
  * Safely converts char array to integer
  */
void char_to_int(char* chararray, int* number) {
  long sl;
  char *end_ptr;
  errno = 0;
   
  sl = strtol(chararray, &end_ptr, 0);
   
  if (ERANGE == errno) {
    SerialUSB.println("number out of range");
  }
  else if (sl > INT_MAX) {
    SerialUSB.println("too large!");
  }
  else if (sl < INT_MIN) {
    SerialUSB.println("too small");
  }
  else if (end_ptr == chararray) {
    SerialUSB.println("not valid numeric input");
  }
  else if ('\0' != *end_ptr) {
    SerialUSB.println("extra characters on input line");
    }
  else {
    *number = static_cast<int>(sl);
  }
}


