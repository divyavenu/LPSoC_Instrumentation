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

#define SAMPLE_RATE 500
#define SAMPLE_SIZE 9
#define SAMPLE_NUMBER 10

/* Dammy task for Interpreter */
static void commandInterpreter(void *arg) { 
  char c;
  while(1) {
    if(SerialUSB.available() > 0){
      c = SerialUSB.read();
      SerialUSB.println(c);
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
    }
    //Serial3.write(packet, SAMPLE_SIZE*SAMPLE_NUMBER);
    SerialUSB.write(packet, SAMPLE_SIZE*SAMPLE_NUMBER);
    SerialUSB.println();
       
  }
}

void setup(){
  SerialUSB.begin(0);
  while(!SerialUSB);
  Serial3.begin(9600);
  while(!Serial3);
  PowerDue.init(SAMPLE_RATE);
  PowerDue.startSampling();

  xTaskCreate(commandInterpreter, NULL, 8*configMINIMAL_STACK_SIZE, 0, 1, NULL);
  xTaskCreate(packetSender, NULL, 8*configMINIMAL_STACK_SIZE, 0, 1, NULL); 
  
  vTaskStartScheduler();
  SerialUSB.println("Insufficient RAM");
  while(1);
}

void loop(){

}
