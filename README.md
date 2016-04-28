# LPSoC_Instrumentation
Please copy instrument.cpp and instrument.h to the Arduino libary files before running .ino code.

Arduino Extention 
Replace UARTClass.h and UARTClass.cpp with the respective files in arduino_library folder
This exposes a method in UARTClass that can be overriden by any program.

Processor code and Test Code
The InstFreeRTOS contains code that needs to be uploaded to the instrumentation side.
targetTest code that needs to be uploaded in the target processor for testing instrumentation functionality.
