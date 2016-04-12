
#define HDR_SIZE 8 //Not including sync bytes
#define SYNC_BLOCK = 'UUUU' //
#define SYNC_BLOCK_LENGTH = 4

//#define COMMAND_PIN 44  //PC19
int serialdata;


//char serialdata[10];   // for incoming serial data

uint8_t bufferA[2] ="A";



void setup() {
    //pinMode(COMMAND_PIN,INPUT);
     //digitalWrite(COMMAND_PIN,LOW);
        SerialUSB.begin(0);
        while(!SerialUSB);
        
        Serial3.begin(9600);     // opens serial port, sets data rate to 9600 bps

       while(!Serial3);
       SerialUSB.println("Serial 3 is ready sending command");
        Serial3.write(&bufferA[0],2);
        
        //delay(1000);
        //Serial3.write(&bufferA[0],4);
}

void loop() {
        // send data only when you receive data:
       // SerialUSB.println("Yay");
       /* if (Serial3.available() > 0) {
                // read the incoming byte:
                  Serial3.readBytes(serialdata,1020);
                //Or serialdata = Serial3.read();
                // say what you got:
                SerialUSB.print("I received: ");
                SerialUSB.println(serialdata);
        }*/

         if (SerialUSB.available() > 0) {
                // read the incoming byte:
                 //digitalWrite(COMMAND_PIN,LOW);

                  serialdata = SerialUSB.read();
                  
                //Or serialdata = Serial3.read();
                // say what you got:
                SerialUSB.print("I received: ");
                SerialUSB.println(serialdata);
                Serial3.write(serialdata);
               // digitalWrite(COMMAND_PIN,HIGH);
        }
   


}
