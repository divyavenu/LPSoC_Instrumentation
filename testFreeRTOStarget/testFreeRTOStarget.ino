#include <FreeRTOS_ARM.h>

static void Thread1(void* arg) {
  while (1) {
    if (SerialUSB.available()){
      char c = SerialUSB.read();
      Serial3.write(c);
    } else {
      taskYIELD();
    }
  }
}

static void Thread2(void* arg) {
  char sync[5];
  char buf[91];
  
  while (1) {
    if (Serial3.available()){
      Serial3.readBytes(sync, 4);
      SerialUSB.print("SYNC: ");
      for(int i = 0; i < 4; i++){
        SerialUSB.print(sync[i]);
      }
      SerialUSB.println();
      Serial3.readBytes(buf, 90);
      SerialUSB.print("PACKET: ");
      for(int i = 0; i < 10; i++){
        SerialUSB.print(buf[i*9] , HEX);
        SerialUSB.print(" ");
        for(int j = 0; j < 4; j++){
          SerialUSB.print(buf[i*9+j*2+1], HEX);
          SerialUSB.print(buf[i*9+j*2+2], HEX);
          SerialUSB.print(" ");
        }
        SerialUSB.print(" : ");
      }
      SerialUSB.println();
    } else {
      taskYIELD();
    }
  }
}
//------------------------------------------------------------------------------
void setup() {
  SerialUSB.begin(0);
  while(!SerialUSB);
  Serial3.begin(9600);
  while(!Serial3);
  
  xTaskCreate(Thread1, NULL, configMINIMAL_STACK_SIZE, NULL, 1, NULL);
  xTaskCreate(Thread2, NULL, configMINIMAL_STACK_SIZE, NULL, 1, NULL);

  // start scheduler
  vTaskStartScheduler();
  Serial.println("Insufficient RAM");
  while(1);
}

void loop() {
  // Not used.
}
