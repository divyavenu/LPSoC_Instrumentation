#include <FreeRTOS_ARM.h>

// Pin for task hook
#define TASK_ID_PIN_0 45
#define TASK_ID_PIN_1 46
#define TASK_ID_PIN_2 47
#define TASK_ID_PIN_3 48
#define TASK_ID_VALID_PIN 44

static BaseType_t  Thread1TaskHook(void * pvParameter){
  digitalWrite(TASK_ID_VALID_PIN, HIGH);
  digitalWrite(TASK_ID_PIN_0, HIGH);
  digitalWrite(TASK_ID_PIN_1, LOW);
  digitalWrite(TASK_ID_PIN_2, LOW);
  return 0; 
}

static BaseType_t  Thread2TaskHook(void * pvParameter){
  digitalWrite(TASK_ID_VALID_PIN, HIGH);
  digitalWrite(TASK_ID_PIN_0, LOW);
  digitalWrite(TASK_ID_PIN_1, HIGH);
  digitalWrite(TASK_ID_PIN_2, LOW);
  return 0;
}

static BaseType_t  Thread3TaskHook(void * pvParameter){
  digitalWrite(TASK_ID_VALID_PIN, HIGH);
  digitalWrite(TASK_ID_PIN_0, HIGH);
  digitalWrite(TASK_ID_PIN_1, HIGH);
  digitalWrite(TASK_ID_PIN_2, LOW);
  return 0;
}

static void Thread1(void* arg) {
  vTaskSetApplicationTaskTag( NULL, Thread1TaskHook);
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
  vTaskSetApplicationTaskTag( NULL, Thread2TaskHook);
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
        SerialUSB.print(buf[i*9]&0b0111 , HEX);
        for(int j = 0; j < 4; j++){
          SerialUSB.print("/ ");
          SerialUSB.print((buf[i*9+j*2+1]&0xF0)>>4, DEC);
          SerialUSB.print(" ");
          SerialUSB.print(((uint16_t)buf[i*9+j*2+1]&0x000F)<<4|(uint16_t)buf[i*9+j*2+2], DEC);
        }
        SerialUSB.print(", ");
      }
      SerialUSB.println();
    } else {
      taskYIELD();
    }
  }
}

static void Thread3(void* arg) {
  vTaskSetApplicationTaskTag( NULL, Thread3TaskHook);
  while(1);
    taskYIELD();
}

//------------------------------------------------------------------------------
void setup() {
  pinMode(TASK_ID_PIN_0, OUTPUT);
  pinMode(TASK_ID_PIN_1, OUTPUT);
  pinMode(TASK_ID_PIN_2, OUTPUT);
  pinMode(TASK_ID_VALID_PIN, OUTPUT);
  
  SerialUSB.begin(0);
  while(!SerialUSB);
  Serial3.begin(9600);
  while(!Serial3);
  
  xTaskCreate(Thread1, NULL, configMINIMAL_STACK_SIZE, NULL, 1, NULL);
  xTaskCreate(Thread2, NULL, configMINIMAL_STACK_SIZE, NULL, 1, NULL);
  xTaskCreate(Thread3, NULL, configMINIMAL_STACK_SIZE, NULL, 1, NULL);
  
  // start scheduler
  vTaskStartScheduler();
  Serial.println("Insufficient RAM");
  while(1);
}

void loop() {
  digitalWrite(TASK_ID_VALID_PIN, HIGH);
  digitalWrite(TASK_ID_PIN_0, LOW);
  digitalWrite(TASK_ID_PIN_1, LOW);
  digitalWrite(TASK_ID_PIN_2, LOW);
}
