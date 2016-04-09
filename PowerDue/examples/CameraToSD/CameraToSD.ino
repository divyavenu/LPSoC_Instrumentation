// This is a basic snapshot sketch using the VC0706 library.
// On start, the Arduino will find the camera and USB connection and
// then snap a photo and send it over USB.

#include <PowerDue.h>

FileStore * filePtr;

void setup() {

  // SerialUSB has no Baudrate Settings
  SerialUSB.begin(0);
  // Wait for connection
  while(!SerialUSB);

  PowerDue.init();

  PowerDue.SD.MakeDirectory("JDF");

  filePtr = new FileStore();
  filePtr->Init();

  // Print out the camera version information (optional)
  PowerDue.Camera.getVersion();
  char *reply = PowerDue.Camera.getVersion();
  if (reply == 0) {
    //Failed to get version
    PowerDue.LED(PD_RED);
    return;
  }

  // Set the picture size - you can choose one of 640x480, 320x240 or 160x120
  // Remember that bigger pictures take longer to transmit!
  PowerDue.Camera.setImageSize(VC0706_640x480);        // biggest
  //PowerDue.Camera.setImageSize(VC0706_320x240);        // medium
  //PowerDue.Camera.setImageSize(VC0706_160x120);          // small

  // You can read the size back from the camera (optional, but maybe useful?)
  uint8_t imgsize = PowerDue.Camera.getImageSize();
  //Image size:
  //if (imgsize == VC0706_640x480) //"640x480"
  //if (imgsize == VC0706_320x240) //"320x240"
  //if (imgsize == VC0706_160x120) //"160x120"

  //"Snap in 6 secs..."
  PowerDue.LED(PD_WHITE);
  delay(1000);
  PowerDue.LED(PD_OFF);
  delay(1000);
  PowerDue.LED(PD_WHITE);
  delay(1000);
  PowerDue.LED(PD_OFF);
  delay(1000);
  PowerDue.LED(PD_WHITE);
  delay(1000);
  PowerDue.LED(PD_OFF);
  delay(1000);

  if (! PowerDue.Camera.takePicture()){
    //"Failed to snap!"
    PowerDue.LED(PD_RED);
  }else{
    //"Picture taken!"
    PowerDue.LED(PD_GREEN);
  }

  uint16_t jpglen = PowerDue.Camera.frameLength();

  int32_t time = millis();

  // Create an image with the name IMAGExx.JPG
  char filename[13];
  strcpy(filename, "IMAGE00.JPG");
  for (int i = 0; i < 100; i++) {
    filename[5] = '0' + i/10;
    filename[6] = '0' + i%10;
    // create if does not exist, do not open existing, write, sync after write
    if (! PowerDue.SD.FileExists(PowerDue.SD.CombineName("JDF", filename))) {
      break;
    }
  }

  // Open the file for writing
  filePtr->inUse = true;
  filePtr->Open("JDF", filename, FILE_WRITE);

  // Read all the data up to # bytes!
  byte wCount = 0; // For counting # of writes
  while (jpglen > 0) {
    // read 32 bytes at a time;
    uint8_t *buffer;
    uint8_t bytesToRead = min(32, jpglen); // change 32 to 64 for a speedup but may not work with all setups!
    buffer = PowerDue.Camera.readPicture(bytesToRead);
    // Send over USB
    //SerialUSB.write(buffer, bytesToRead);
    filePtr->Write((const char *)buffer, bytesToRead);
    if(wCount > 32){ // Every 1K, blink the LED so it doesn't appear locked up
      PowerDue.LED(PD_BLUE);
    }
    if(++wCount >= 64) {
      PowerDue.LED(PD_OFF);
      wCount = 0;
    }
    jpglen -= bytesToRead;
  }

  time = millis() - time;

  PowerDue.LED(PD_GREEN);
  filePtr->Close();
}

void loop() {
}
