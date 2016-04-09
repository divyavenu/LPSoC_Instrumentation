/*
*
* Firmware for the PowerDué (Based on the Arduino Due)
* Example for simple Bash to access the SD Card
* Carnegie Mellon University Silicon Valley
* Version: 0.0.1
*
*/

#include <PowerDue.h>

#define BUFFER_SIZE 100
char inputBuffer[BUFFER_SIZE] = "";         // a string to hold incoming data
char bufferIndex = 0;

boolean stringComplete = false;  // whether the string is complete

void setup() {
  // initialize serial:
  SerialUSB.begin(0);
  while(!SerialUSB);

  PowerDue.init();
  prompt("/");
}
void prompt(char * directory){
  SerialUSB.print("root@powerdue:");
  SerialUSB.print(directory);
  SerialUSB.print("$");

}

void loop() {
  char c;
  while(SerialUSB.available()){
    c = SerialUSB.read();
    SerialUSB.print(c);
    inputBuffer[bufferIndex] = c;
    if(inputBuffer[bufferIndex] == '\n'){
      inputBuffer[bufferIndex]='\0';
      bufferIndex = 0;
      processCommand();
    }else if(bufferIndex >= BUFFER_SIZE){
      bufferIndex = 0;
    }else{
      bufferIndex++;
    }
  }
}

void processCommand(){
  if(!strncmp(inputBuffer, "ls", 2)){
    ls();
  }else if(!strncmp(inputBuffer, "cat", 3)){
    cat();
  }else if(!strncmp(inputBuffer, "help", 4)){
    help();
  }else if(!strncmp(inputBuffer, "camera", 6)){
    camera();
  }else if(!strncmp(inputBuffer, "mkdir", 3)){
    SerialUSB.println("YEY mkdir");
  }else{
    SerialUSB.print("-USB bash: ");
    SerialUSB.print(inputBuffer);
    SerialUSB.println(": command not found");
  }
  prompt("/");
}

void ls(){
  SerialUSB.println("YEY LS");
}
void cat(){
  SerialUSB.println("YEY CAT");
}
void help(){
  SerialUSB.println("YEY help");
}

void camera(){
  SerialUSB.println("YEY camera");
}

void cd(){
  SerialUSB.println("YEY Cd");
}
