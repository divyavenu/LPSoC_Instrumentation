
#define HDR_SIZE 8 //Not including sync bytes
#define SYNC_BLOCK = 'UUUU' //
#define SYNC_BLOCK_LENGTH = 4
#define TASK_ID_PIN_0 45
#define TASK_ID_PIN_1 46
#define TASK_ID_VALID_PIN 44  //PC19

int counter=-5;

char serialdata[4];   // for incoming serial data
char serialByte;
char syncBytes[4];
char syncCount = 0;
boolean syncWrite = false;
boolean inSync = false;
char syncBlocks[4];

char serial2Bytes[2];
int dataCounter = 0;
int packetLen = 0;

void setup() {
        syncBlocks[0] = 0x66;
        syncBlocks[1] = 0x55;
        syncBlocks[2] = 0x88;
        syncBlocks[3] = 0x77;

        // Test block to test pin toggles and interupts are working
        // should see "0 C" for TASKID
        pinMode(TASK_ID_PIN_0,OUTPUT);
        pinMode(TASK_ID_PIN_1,OUTPUT);
        pinMode(TASK_ID_VALID_PIN,OUTPUT);
        digitalWrite(TASK_ID_VALID_PIN, LOW);
        delay(2000);
        digitalWrite(TASK_ID_PIN_0, LOW);
        digitalWrite(TASK_ID_PIN_1, LOW);
        digitalWrite(TASK_ID_VALID_PIN, HIGH);

        SerialUSB.begin(9600);
        while(!SerialUSB);
        
        Serial3.begin(9600);     // opens serial port, sets data rate to 9600 bps
}

void print2Bytes(char * data, String base) {
  if (base.equals("HEX")) {
    SerialUSB.print(data[1], HEX);
    SerialUSB.print(" ");
    SerialUSB.println(data[0], HEX);
  }
  else if (base.equals("BIN")) {
    SerialUSB.print(serial2Bytes[1], BIN);
    SerialUSB.print(" ");
    SerialUSB.println(serial2Bytes[0], BIN);  
  }
}

void loop() {

        uint16_t test[2];
        test[0] = 0xAA;
        test[1] = 0xBB;
        SerialUSB.print(test[0], HEX);
  
        // send data only when you receive data:
        if (Serial3.available() > 0) {
                if (inSync) {
                  // if inSync unpack data  
                  Serial3.readBytes(serial2Bytes, 2);
                  if (dataCounter == 0) {
                    SerialUSB.print("[TASKID (BINERY)]: ");
                    print2Bytes(serial2Bytes, "BIN");
                    dataCounter ++;
                  }
                  else if (dataCounter == 1) {
                    SerialUSB.print("[TIMESTAMP1]: ");
                    print2Bytes(serial2Bytes, "HEX");
                    dataCounter ++;                                       
                  }
                  else if (dataCounter == 2) {
                    SerialUSB.print("[TIMESTAMP2]: ");
                    print2Bytes(serial2Bytes, "HEX");
                    dataCounter ++;                                       
                  }
                  else if (dataCounter == 3) {
                    // lower byte + higher byte * 2^8
                    packetLen = (int)serial2Bytes[0] + (int)serial2Bytes[1]*256;
                    SerialUSB.print("[PAK SIZE (DEC)]: ");
                    SerialUSB.println(packetLen);
                    
//                    SerialUSB.println((uint8_t *)serial2Bytes);
                    dataCounter ++;                                      
                  }
                  // reset counter
                  if (dataCounter == 4) {
                    dataCounter = 0;
                    inSync = false;
                  }         
                } // end "if inSync"
                else {
                  // read 1 bytes at a time
                  serialByte = Serial3.read();
                  SerialUSB.print(serialByte, HEX);
                  SerialUSB.print(" "); 
                }
                if (serialByte == 0x66) {
                  syncBytes[0] = serialByte; 
                  syncCount ++;
                  syncWrite = true;
                }
                else if (syncWrite){
                  if (syncCount < 3) {
                    syncBytes[syncCount] = serialByte;                            
                    syncCount ++; 
                  }
                  // write last byte
                  else if (syncCount == 3) {
                    syncBytes[syncCount] = serialByte; 
                    syncCount ++;
                    inSync = true;
                    SerialUSB.println("Sync bytes filled");
                    
                    for (int i = 0; i < 4; i++) {
                      SerialUSB.println(syncBytes[i], HEX);
                      if (syncBytes[i] != syncBlocks[i]) {
                        inSync = false;
                        SerialUSB.println("Ooops NVM");
                      }
                    }
                    syncCount = 0;
                    syncWrite = false;
                    for (int j = 0; j < 4; j ++) {
                      syncBytes[j] = (char)0;
                    };
                  }
                }
                             
//                delay(500);
        }
}
