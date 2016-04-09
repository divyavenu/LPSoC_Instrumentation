/*
*
* Firmware for the PowerDué (Based on the Arduino Due)
* Example for simple Bash to access the SD Card
* Carnegie Mellon University Silicon Valley
* Version: 0.0.1
*
*/
#include <FreeRTOS_ARM.h>

//#include <PowerDue.h>

#define CAMERA_VERSION 1
#define RESOLUTION640x480 2
#define RESOLUTION320x240 3
#define RESOLUTION160x120 4
#define PICTURE 5
#define GETRESOLUTION 6
#define GETCURRENTFOLDER 7
#define GETVERSION 8



#define INPUT_BUFFER_SIZE 100
#define QUEUE_SIZE 10
#define QUEUE_CLI_BEGIN 0
#define QUEUE_CLI_END 5
#define QUEUE_CAM_BEGIN 5
#define QUEUE_CAM_END 10
#define MAX_PATH_SIZE 100
#define MAX_OUTPUT_SIZE 10000
#define LEFT_KEY_PRESS "\033[D"
#define RIGHT_KEY_PRESS "\033[C"
static char HELP[] = "Available commands:\n\rhelp\n\rcd [<folder>]\n\rls\n\rcat <file>\n\rcam [-r[640|320|160] | -v | -o[<folder>] ]\n\r";
static char PROMPT[] = "root@powerdue:";
static char LSBLOCK[MAX_OUTPUT_SIZE];
uint32_t MAX_PROMPT_SIZE = MAX_PATH_SIZE + strlen(PROMPT) + 2;

static char userInputBuffer[QUEUE_SIZE][INPUT_BUFFER_SIZE];
static char terminalOutputBuffer[QUEUE_SIZE][INPUT_BUFFER_SIZE+50];

QueueHandle_t inputToCli_Queue;
QueueHandle_t ToInput_Queue;
QueueHandle_t cliToCamera_Queue;

static void input_d(void *) {

  uint32_t bufferIndex = 0;
  uint32_t queueIndex_o = 0;
  uint32_t queueIndex_i = 0;
  char c;

  while(1){

    // Peek into the Receive Queue Without Blocking to see if there is something to right on the screen
    if( xQueueReceive( ToInput_Queue, &( queueIndex_i ), ( TickType_t ) 0) == pdTRUE ){
      if(!strcmp(terminalOutputBuffer[queueIndex_i], "--h")){
          SerialUSB.print(HELP);
      }else if(!strcmp(terminalOutputBuffer[queueIndex_i], "--ls")){
          SerialUSB.print(LSBLOCK);
      }else{
          SerialUSB.print(terminalOutputBuffer[queueIndex_i]);
      }
    }
    // Active USB Serial Polling --- TODO Significant Power Improvement by Doing a USB Interrupt
    while(SerialUSB.available()){
      c = SerialUSB.read();
      SerialUSB.print(c);
      userInputBuffer[queueIndex_o][bufferIndex] = c;

      // End of line either by new line or carriage return
      if(userInputBuffer[queueIndex_o][bufferIndex] == '\n' || userInputBuffer[queueIndex_o][bufferIndex] == '\r'){
        if(userInputBuffer[queueIndex_o][bufferIndex] == '\n') {
          SerialUSB.print('\r');
        }else if(userInputBuffer[queueIndex_o][bufferIndex] == '\r') {
          SerialUSB.print('\n');
        }

        // Terminate string
        userInputBuffer[queueIndex_o][bufferIndex]='\0';
        // Send command to CLI
        if( xQueueSend( inputToCli_Queue, ( void * ) &queueIndex_o, ( TickType_t ) 0) != pdPASS){
            // Failed to post the message.
            SerialUSB.println("Error: Failed to queue command.");
        }
        // Increment queueIndex
        queueIndex_o++;
        if(queueIndex_o == QUEUE_SIZE){
          queueIndex_o = 0;
        }
        // Restart bufferIndex
        bufferIndex = 0;
      }else if(bufferIndex >= INPUT_BUFFER_SIZE){
        // Input Buffer Overflow
        bufferIndex = 0;
      }else if(userInputBuffer[queueIndex_o][bufferIndex] == 8){ //Backspace
        if(bufferIndex > 0){
          bufferIndex--;
          SerialUSB.print(" ");
          SerialUSB.print(LEFT_KEY_PRESS);
        }else{
          SerialUSB.print(RIGHT_KEY_PRESS);
        }
      }else{
        bufferIndex++;
      }
    }
  }
}

void parse(char * message, char * command, char * args){
  char * space;
  space=strchr(message,' ');
  if(space!=NULL){
    *space='\0';
    strcpy(args, space+1);
  }else{
    strcpy(args, "");
  }
  strcpy(command, message);
  return;
}
/**/
static void cli(void *){
  char currentPath[MAX_PATH_SIZE] = "/";
  char command[INPUT_BUFFER_SIZE/2];
  char args[INPUT_BUFFER_SIZE/2];
  uint32_t queueIndex_o=QUEUE_CLI_BEGIN;
  uint32_t queueIndex_i=0;

  while(1){
    if( xQueueReceive( inputToCli_Queue, &( queueIndex_i ), portMAX_DELAY) ){
      if(strlen(userInputBuffer[queueIndex_i])){
        parse(userInputBuffer[queueIndex_i], command, args);
        if(!strcmp(command, "ls")){
          ls(args, &queueIndex_o);
        }else if(!strcmp(command, "cat")){
          cat(args, queueIndex_o);
        }else if(!strcmp(command, "cd")){
          cd(args, queueIndex_o);
        }else if(!strcmp(command, "help")){
          strcpy(terminalOutputBuffer[queueIndex_o], "--h");
          sendToQueue(ToInput_Queue, &queueIndex_o);
          queueIndex_o++;
          if(queueIndex_o == QUEUE_CLI_END){
            queueIndex_o = QUEUE_CLI_BEGIN;
          }
        }else if(!strcmp(command, "cam")){
          if(strlen(args)){
            if(!strcmp(args, "-r")){

            }else if(!strcmp(args, "-r640")){

            }else if(!strcmp(args, "-r320")){

            }else if(!strcmp(args, "-r160")){

            }else if(!strcmp(args, "-v")){

            }else if(!strcmp(args, "-o")){

            }else{
              strcpy(terminalOutputBuffer[queueIndex_o], "-cam: ");
              strcat(terminalOutputBuffer[queueIndex_o], args);
              strcat(terminalOutputBuffer[queueIndex_o], ": Invalid arguments\n\r");
              sendToQueue(ToInput_Queue, &queueIndex_o);
              queueIndex_o++;
              if(queueIndex_o == QUEUE_CLI_END){
                queueIndex_o = QUEUE_CLI_BEGIN;
              }
            }

          }
        }else if(!strcmp(command, "mkdir")){
          makedirectory(args, queueIndex_o);
        }else{
          strcpy(terminalOutputBuffer[queueIndex_o], "-USB bash: ");
          strcat(terminalOutputBuffer[queueIndex_o], userInputBuffer[queueIndex_i]);
          strcat(terminalOutputBuffer[queueIndex_o], ": command not found\n\r");
          sendToQueue(ToInput_Queue, &queueIndex_o);
          queueIndex_o++;
          if(queueIndex_o == QUEUE_CLI_END){
            queueIndex_o = QUEUE_CLI_BEGIN;
          }

        }
      }

      // Prompt
      strcpy(terminalOutputBuffer[queueIndex_o], PROMPT);
      strcat(terminalOutputBuffer[queueIndex_o], currentPath);
      strcat(terminalOutputBuffer[queueIndex_o], "$ ");
      sendToQueue(ToInput_Queue, &queueIndex_o);
      queueIndex_o++;
      if(queueIndex_o == QUEUE_CLI_END){
        queueIndex_o = QUEUE_CLI_BEGIN;
      }
    }
  }
}

static void camera_d(void *){
  char savingFolder[MAX_PATH_SIZE] = "";
  char command=0;
  char filename[13];
  uint16_t jpglen = 0;
  uint32_t queueIndex_o=QUEUE_CAM_BEGIN;
  //FileStore * fp;

  //PowerDue.Camera.setMotionDetect(true);

  while(1){
    // Peek into the Receive Queue Without Blocking to see if there is something to right on the screen
    if( xQueueReceive( cliToCamera_Queue, &( command ), ( TickType_t ) 0) == pdTRUE ){
      switch (command){
        case CAMERA_VERSION:
        break;
        case RESOLUTION640x480:
        break;
        case RESOLUTION320x240:
        break;
        case RESOLUTION160x120:
        break;
        case PICTURE:
        break;
        case GETRESOLUTION:
        break;
        case GETCURRENTFOLDER:
        break;
        case GETVERSION:
        break;
      }
    }
    // Active Serial Polling --- TODO Significant Power Improvement by Doing a Serial Interrupt
/*    if(PowerDue.Camera.motionDetected()){
      PowerDue.Camera.setMotionDetect(false);
      if (! PowerDue.Camera.takePicture()){
        //"Failed to snap!"
        strcpy(terminalOutputBuffer[queueIndex_o], "[Camera] Motion detected : Failed to take picture!\n\r");
        sendToQueue(ToInput_Queue, &queueIndex_o);
        queueIndex_o++;
        if(queueIndex_o == QUEUE_CAM_END){
          queueIndex_o = QUEUE_CAM_BEGIN;
        }
      }else{
        //"Picture taken!"
        strcpy(terminalOutputBuffer[queueIndex_o], "[Camera] Motion detected : Picture taken!\n\r");
        sendToQueue(ToInput_Queue, &queueIndex_o);
        queueIndex_o++;
        if(queueIndex_o == QUEUE_CAM_END){
          queueIndex_o = QUEUE_CAM_BEGIN;
        }

        jpglen = PowerDue.Camera.frameLength();

        int32_t time = millis();
        // Check File Name
        strcpy(filename, "IMAGE00.JPG");
        for (int i = 0; i < 100; i++) {
          filename[5] = '0' + i/10;
          filename[6] = '0' + i%10;
          // create if does not exist, do not open existing, write, sync after write
          // TODO put a mutex protecting the SD
          if (! PowerDue.SD.FileExists(PowerDue.SD.CombineName(savingFolder, filename))) {
            break;
          }
        }
        // New File
        fp = new FileStore();
        fp->Init();
        fp->inUse = true;
        fp->Open(savingFolder, filename, FILE_WRITE);

        // Read all the data up to # bytes!
        byte wCount = 0; // For counting # of writes
        while (jpglen > 0) {
          // read 32 bytes at a time;
          uint8_t *buffer;
          uint8_t bytesToRead = min(64, jpglen); // change 32 to 64 for a speedup but may not work with all setups!
          buffer = PowerDue.Camera.readPicture(bytesToRead);

          fp->Write((const char *)buffer, bytesToRead);
          if(wCount > 32){ // Every 1K, blink the LED so it doesn't appear locked up
            PowerDue.LED(PD_BLUE);
          }
          if(++wCount >= 64) {
            PowerDue.LED(PD_OFF);
            wCount = 0;
          }
          jpglen -= bytesToRead;
        }

        time = millis() - time;
        fp->Close();
        sprintf(terminalOutputBuffer[queueIndex_o], "[Camera] File saved (%d ms)\n\r", time);
        sendToQueue(ToInput_Queue, &queueIndex_o);
        queueIndex_o++;
        if(queueIndex_o == QUEUE_CAM_END){
          queueIndex_o = QUEUE_CAM_BEGIN;
        }
      }
    }
  */}
}

void sendToQueue(QueueHandle_t Queue, uint32_t * message){
  // Send
  if( xQueueSend( Queue, ( void * ) message, ( TickType_t ) 0) != pdPASS){
      /* Failed to post the message. */
      SerialUSB.println("Error: Failed to queue command.");
  }
};


void makedirectory(char * args, uint32_t queueIndex_o){
  //sendToQueue(ToInput_Queue, "YEY LS\n\r");
}

void ls(char * args, uint32_t * queueIndex_o){
  strcpy(terminalOutputBuffer[(*queueIndex_o)], "YEY ls\n\r");
  sendToQueue(ToInput_Queue, queueIndex_o);
  (*queueIndex_o)++;
  if((*queueIndex_o) == QUEUE_CLI_END){
    (*queueIndex_o) = QUEUE_CLI_BEGIN;
  }
}
void cat(char * args, uint32_t queueIndex_o){
  //sendToQueue(ToInput_Queue, "YEY CAT\n\r");
}


void cd(char * args, uint32_t queueIndex_o){
  //sendToQueue(ToInput_Queue, "YEY CD\n\r");
}


void setup() {
  // initialize serial:
  SerialUSB.begin(0);
  //while(!SerialUSB);
  PowerDue.init();

  // Create a queue capable of containing QUEUE_SIZE unsigned long values.
  inputToCli_Queue = xQueueCreate( QUEUE_SIZE, sizeof( unsigned long ) );
  if( inputToCli_Queue == 0 ){
        // Queue was not created and must not be used.
        SerialUSB.println("Error Creating Queue.");
        while(1);
  }

  ToInput_Queue = xQueueCreate( QUEUE_SIZE, sizeof( unsigned long ) );
  if( ToInput_Queue == 0 ){
        // Queue was not created and must not be used.
        SerialUSB.println("Error Creating Queue.");
        while(1);
  }

  cliToCamera_Queue = xQueueCreate( QUEUE_SIZE, sizeof( unsigned long ) );
  if( cliToCamera_Queue == 0 ){
        // Queue was not created and must not be used.
        SerialUSB.println("Error Creating Queue.");
        while(1);
  }

  xTaskCreate(input_d, NULL, configMINIMAL_STACK_SIZE, NULL, 1, NULL);
  xTaskCreate(cli, NULL, configMINIMAL_STACK_SIZE, NULL, 1, NULL);
  xTaskCreate(camera_d, NULL, configMINIMAL_STACK_SIZE, NULL, 1, NULL);

  vTaskStartScheduler();

  SerialUSB.println("Insuficient RAM");
  while(1);
}

void loop(){}
