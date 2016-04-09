

//char serialdata[10];   // for incoming serial data




void setup() {
    
        SerialUSB.begin(0);
        while(!SerialUSB);
        
        Serial3.begin(9600);     // opens serial port, sets data rate to 9600 bps
}

void loop() {
        
        if (Serial3.available() > 0) {
                // read the incoming byte:
                Serial3.readBytes(serialdata,1020);
                SerialUSB.print("I received: ");
                SerialUSB.println(serialdata);
        }
}
