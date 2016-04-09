/*
  Instrument Power Due ATSAM3X8E
  Library for accessing Analog and peripherals of the Instrument Processor on the PowerDue
  Created by Joao Diogo Falcao, February 2016
  Carnegie Mellon University - Silicon Valley
*/

#ifndef _INSTRUMENT_PD_H_
#define _INSTRUMENT_PD_H_

#define INSTRUMENT_CLOCK_RATE 84000000

#define OFFSET_DISABLE HIGH
#define OFFSET_ENABLE LOW

#define CH0GS0PIN 40
#define CH0GS1PIN 39
#define CH0OFFSET_SHUTDOWN 28

#define CH1GS0PIN 38
#define CH1GS1PIN 37
#define CH1OFFSET_SHUTDOWN 27

#define CH2GS0PIN 36
#define CH2GS1PIN 35
#define CH2OFFSET_SHUTDOWN 26

#define CH3GS0PIN 34
#define CH3GS1PIN 33
#define CH3OFFSET_SHUTDOWN 25

#define GAIN_25 0
#define GAIN_50 1
#define GAIN_100 2
#define GAIN_200 3

#define TASK_ID_PIN_0 45
#define TASK_ID_PIN_1 46
#define TASK_ID_PIN_2 47
#define TASK_ID_PIN_3 48
#define TASK_ID_VALID_PIN 44  //PC19

//packet = timestamp 4 bytes, task ID 2 bytes to maintain even symmetry????
#define SYNC_BLOCK 0x5555

#define HEADER_SIZE 12
// 4 bytes of sync
// Task ID 2 bytes
// Timestamp 4 bytes
// packet length 2 bytes
// Actual data
// Total of 1024 bytes always

// Detect valid rising ID for task ID


#define NUM_SAMPLES_PER_BUFFER 126
// Each sample is 2bytes and there are 4 channels
#define BUFFER_SIZE_FOR_USB (NUM_SAMPLES_PER_BUFFER*2*4)+HEADER_SIZE // in bytes
#define MIN_SAMPLES_PER_TASK 60
#define MIN_BUFFER_SIZE_PER_TASK (MIN_SAMPLES_PER_TASK*2*4) // in bytes
#define PADDING 10
// Has to be a power of 2
#define NUM_BUFFERS 4

#include "Arduino.h"
#include <Wire.h>


class InstrumentPowerDue
{
  public:
    InstrumentPowerDue();
    void init(int sample_rate);
    void setGain(int channel, int gain);
    void setDCOffset(int channel, int level);
    void changeSamplingRate(int sample_rate);
    void startSampling();
    void stopSampling();
    void startADC();
    bool bufferReady();
    void writeBuffer(Serial_ * port);
    void bufferFullInterrupt();
    void taskIdValidTrigger();
    uint16_t readTaskID();


  private:
    volatile bool isSampling, isInterrupted;
    uint16_t buffer[NUM_BUFFERS][BUFFER_SIZE_FOR_USB+PADDING];
    volatile int currentBuffer, nextBuffer;
    volatile uint16_t currentTask;
    uint32_t timeReference, currentTime;

};

extern InstrumentPowerDue PowerDue;

#endif
