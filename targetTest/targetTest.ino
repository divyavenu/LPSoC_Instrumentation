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

static BaseType_t  Thread4TaskHook(void * pvParameter){
  digitalWrite(TASK_ID_VALID_PIN, HIGH);
  digitalWrite(TASK_ID_PIN_0, LOW);
  digitalWrite(TASK_ID_PIN_1, LOW);
  digitalWrite(TASK_ID_PIN_2, HIGH);
  return 0;
}

static void Thread1(void* arg) {
  vTaskSetApplicationTaskTag( NULL, Thread1TaskHook);
  const TickType_t xPeriod = 2000/portTICK_PERIOD_MS;
  Serial3.write("*STR");
    
  while (1) {  
    if (SerialUSB.available()){
      while(SerialUSB.available() > 0){
        char c = SerialUSB.read();
        Serial3.write(c);
        SerialUSB.print(c);
      }
      SerialUSB.println();
    } else {
      vTaskDelay(xPeriod);
      Serial3.write("*RDY");
      SerialUSB.println("*RDY");
    }
  }
}

static void Thread2(void* arg) {
  vTaskSetApplicationTaskTag( NULL, Thread2TaskHook);
  char sync[5];
  char buf[122];
  uint8_t nTask;
  
  while (1) {
    if (Serial3.available()){
      nTask = 0;
      memset(buf, '0', 122);
//      Serial3.readBytes(sync, 4);
//      SerialUSB.print("SYNC: ");
//      for(int i = 0; i < 4; i++){
//        SerialUSB.print(sync[i]);
//      }
//      SerialUSB.println();
      nTask = Serial3.read();
      SerialUSB.print("#ofTask: ");
      SerialUSB.println(nTask, DEC);
      
      while (!Serial3.available());
      Serial3.readBytes(buf, nTask*11);
      serialFlush();
      
      SerialUSB.println("<PACKET>");
      //SerialUSB.write(buf, (nTask)*11);
      for(int i = 0; i < nTask; i++){
        SerialUSB.print(buf[i*11]&0x07, DEC);
        SerialUSB.print(": [ ");
        for(int j = 0; j < 4; j++){
//          SerialUSB.print(buf[i*11+j*2+1], HEX);
//          SerialUSB.print(buf[i*11+j*2+2], HEX); 
          SerialUSB.print(buf[i*11+j*2+1]>>4, DEC);
          SerialUSB.print("/");      
          SerialUSB.print(((uint16_t)buf[i*11+j*2+1]&0x000F)<<8|(uint16_t)buf[i*11+j*2+2], DEC);
          SerialUSB.print(" ");
        }
        SerialUSB.print("] ");
        SerialUSB.println(((uint16_t)buf[i*11+9])<<8|(uint16_t)buf[i*11+10], DEC);        
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
    if (SerialUSB.available()){
      SerialUSB.print("Thread 3");
    } else {
      //taskYIELD();
    }
}

static void Thread4(void* arg) {
  vTaskSetApplicationTaskTag( NULL, Thread4TaskHook);
  while(1);
    if (SerialUSB.available()){
      SerialUSB.print("Thread 4");
    } else {
      //taskYIELD();
    }
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
  xTaskCreate(Thread4, NULL, configMINIMAL_STACK_SIZE, NULL, 1, NULL);
    
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

void serialFlush(){
  while(Serial3.available() > 0) {
    char c = Serial3.read();
  }
}
