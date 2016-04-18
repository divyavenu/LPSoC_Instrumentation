

char command1[5]="A";

char command2[5]="B";

char command3[5]="C";


void setup() {

        SerialUSB.begin(0);
        while(!SerialUSB);
        
        Serial3.begin(9600);     // opens serial port, sets data rate to 9600 bps

       while(!Serial3);
        SerialUSB.println("Delay:");
        delay(1000);
        SerialUSB.println("Serial 3 is ready sending command");
        Serial3.write(command1,1);
        delay(1000);     //Without delay, there are too many interrupts
        SerialUSB.println("Serial 3 is ready sending command");
        Serial3.write(command2,1);
        delay(1000);
        SerialUSB.println("Serial 3 is ready sending command");
        Serial3.write(command3,1);
}

void loop() {
   

}
