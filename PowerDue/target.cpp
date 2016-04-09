/*
  Main Processar Power Due ATSAM3X8E
  Library for accessing the peripherals of the Main Processor on the PowerDue
  Created by Joao Diogo Falcao, February 2016
  Carnegie Mellon University - Silicon Valley
*/

#include "target.h"


// PowerDué connects to the Camera via Serial2
TargetPowerDue::TargetPowerDue(){}

bool TargetPowerDue::init(){

  SD = MassStorage();
  SD.Init();
  Camera = Adafruit_VC0706(&Serial2);
  // Try to locate the camera
  if (Camera.begin()) {
    Debug("Camera initialized.");
  } else {
    // Camera not found
    Debug("Camera not found.");
    Debug("---- Hit Reset -----");
    return false;
  }

  // Init and Turn Off Bright RGD LED
  LED();

  return true;
}

void TargetPowerDue::LED(){
  pd_rgb_led_init();
  return;
}

void TargetPowerDue::LED(int color){
  pd_rgb_led(color);
  return;
}

void TargetPowerDue::Debug(String msg){
  if(SerialUSB){
    SerialUSB.print("DEBUG: ");
    SerialUSB.println(msg);
  }
  return;
}
