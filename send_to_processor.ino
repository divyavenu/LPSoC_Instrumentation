
#define HDR_SIZE 8 //Not including sync bytes
#define SYNC_BLOCK = 'UUUU' //
#define SYNC_BLOCK_LENGTH = 4

int counter=-5;


char serialdata[10];   // for incoming serial data

//uint8_t bufferA[4] ="*ST" ;



void setup() {
        SerialUSB.begin(0);
        while(!SerialUSB);
        
        Serial3.begin(9600);     // opens serial port, sets data rate to 9600 bps
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
                  SerialUSB.readBytes(serialdata,10);
                //Or serialdata = Serial3.read();
                // say what you got:
               // SerialUSB.print("I received: ");
                //SerialUSB.println(serialdata);
                Serial3.write(serialdata);
        }
       // SerialUSB.print(bufferA);


}
