/*
  Instrument Power Due ATSAM3X8E
  Library for accessing Analog and peripherals of the Instrument Processor on the PowerDue
  Created by Joao Diogo Falcao, February 2016
  Carnegie Mellon University - Silicon Valley
*/

#include "instrument.h"
int tempCharacter;

void ValidTaskIDChange_Handler() {
  PowerDue.taskIdValidTrigger();
}


InstrumentPowerDue::InstrumentPowerDue():currentBuffer(0), nextBuffer(0), currentTime(0), timeReference(0), currentTask(0), isSampling(false), isInterrupted(false){

}

void InstrumentPowerDue::init(int sample_rate){
  // Start I2C
  Wire.begin();

  // Task ID Pin as Inputs
  pinMode(TASK_ID_PIN_0,INPUT);
  pinMode(TASK_ID_PIN_1,INPUT);
  pinMode(TASK_ID_PIN_2,INPUT);
  pinMode(TASK_ID_PIN_3,INPUT);
  //pinMode(TASK_ID_VALID_PIN,INPUT);

  pinMode(RX_PIN,INPUT);


   attachInterrupt(digitalPinToInterrupt(TASK_ID_VALID_PIN), ValidTaskIDChange_Handler, RISING);

  // Start Channel 0 No Gain and No Offset
  pinMode(CH0GS0PIN, OUTPUT);
  pinMode(CH0GS1PIN, OUTPUT);
  digitalWrite(CH0GS0PIN, LOW);
  digitalWrite(CH0GS1PIN, LOW);
  digitalWrite(CH0OFFSET_SHUTDOWN, OFFSET_DISABLE);

  // Start Channel 1 No Gain and No Offset
  pinMode(CH1GS0PIN, OUTPUT);
  pinMode(CH1GS1PIN, OUTPUT);
  digitalWrite(CH1GS0PIN, LOW);
  digitalWrite(CH1GS1PIN, LOW);
  digitalWrite(CH1OFFSET_SHUTDOWN, OFFSET_DISABLE);

  // Start Channel 2 No Gain and No Offset
  pinMode(CH2GS0PIN, OUTPUT);
  pinMode(CH2GS1PIN, OUTPUT);
  digitalWrite(CH2GS0PIN, LOW);
  digitalWrite(CH2GS1PIN, LOW);
  digitalWrite(CH2OFFSET_SHUTDOWN, OFFSET_DISABLE);

  // Start Channel 3 No Gain and No Offset
  pinMode(CH3GS0PIN, OUTPUT);
  pinMode(CH3GS1PIN, OUTPUT);
  digitalWrite(CH3GS0PIN, LOW);
  digitalWrite(CH3GS1PIN, LOW);
  digitalWrite(CH3OFFSET_SHUTDOWN, OFFSET_DISABLE);

  //changeSamplingRate(sample_rate);
 // startADC();
}


uint16_t InstrumentPowerDue::readTaskID(){
  return  ((digitalRead(TASK_ID_PIN_3)<<3)|
      (digitalRead(TASK_ID_PIN_2)<<2)|
      (digitalRead(TASK_ID_PIN_1)<<1)|
      (digitalRead(TASK_ID_PIN_0)));
}

void InstrumentPowerDue::startADC(){


  timeReference = micros();
  // Power measurement Controller Enable Peripheral ADC
  pmc_enable_periph_clk(ID_ADC);

  //Initialize the given ADC with the SystemCoreClock and startup time
  adc_init(ADC, SystemCoreClock, ADC_FREQ_MAX, ADC_STARTUP_FAST);
  // trigger on Timer Counter 0
  ADC->ADC_MR |=0x3;

  //enable 4 specific channels
  ADC->ADC_CHER=0x8E;

  //enable adc interrupt
  NVIC_EnableIRQ(ADC_IRQn);

  //set highest priority
  NVIC_SetPriority(ADC_IRQn, 0);

  //disable all interrupts except RXEND
  ADC->ADC_IDR=~(1<<27);

  //enable DMA-driven ADC RXEND interrupt
  ADC->ADC_IER=1<<27;

  //Initialize packet for DMA's buffer pointer
  currentTime = micros()-timeReference;

  buffer[currentBuffer][0] = SYNC_BLOCK;
  buffer[currentBuffer][1] = SYNC_BLOCK;
  buffer[currentBuffer][2] = readTaskID();
  buffer[currentBuffer][3] = (uint16_t)(currentTime >> 16);
  buffer[currentBuffer][4] = (uint16_t)(0x0000FFFF & currentTime);
  buffer[currentBuffer][5] = (BUFFER_SIZE_FOR_USB-HEADER_SIZE);
  buffer[currentBuffer][6] |= (uint16_t)0x1000;

  // ADC writes to the DMA buffer
  ADC->ADC_RPR=(uint32_t)(&(buffer[currentBuffer][(HEADER_SIZE/2)-1]));

  ADC->ADC_RCR=(BUFFER_SIZE_FOR_USB-HEADER_SIZE);

  // Initialize packet for DMA's next buffer pointer
  // ---------------
  nextBuffer = currentBuffer+1;

  buffer[nextBuffer][0] = SYNC_BLOCK;
  buffer[nextBuffer][1] = SYNC_BLOCK;
  buffer[nextBuffer][2] = readTaskID();
  buffer[nextBuffer][5] = (BUFFER_SIZE_FOR_USB-HEADER_SIZE);
  buffer[nextBuffer][6] |= (uint16_t)0x1000;

  // next DMA buffer
  ADC->ADC_RNPR=(uint32_t)(&(buffer[nextBuffer][(HEADER_SIZE/2)-1]));

  //next dma receive counter
  ADC->ADC_RNCR=(BUFFER_SIZE_FOR_USB-HEADER_SIZE);

  //enable dma pdc receiver channel
  ADC->ADC_PTCR=1;

  // TAG measurements with channel number
  ADC->ADC_EMR|=0x01000000;

  //start adc
  ADC->ADC_CR=2;
}

// Changes the gain of a specific channel. Valid gain values are
// 25, 50, 100, 200. Valid channel values are 0, 1, 2, 3
// Created by Ervin Teng
void InstrumentPowerDue::setGain(int channel, int gain)
{
  switch(channel){
  	case 0:
  		if(gain == GAIN_25){
  			digitalWrite(CH0GS0PIN, LOW);
  			digitalWrite(CH0GS1PIN, LOW);
  		}
  		else if(gain == GAIN_50){
  			digitalWrite(CH0GS0PIN, LOW);
  			digitalWrite(CH0GS1PIN, HIGH);
  		}
  		else if(gain == GAIN_100){
  			digitalWrite(CH0GS0PIN, HIGH);
  			digitalWrite(CH0GS1PIN, LOW);
  		}
  		else if(gain == GAIN_200){
  			digitalWrite(CH0GS0PIN, HIGH);
  			digitalWrite(CH0GS1PIN, HIGH);
  		}
  		break;
    case 1:
      if(gain == GAIN_25){
        digitalWrite(CH1GS0PIN, LOW);
        digitalWrite(CH1GS1PIN, LOW);
      }
      else if(gain == GAIN_50){
        digitalWrite(CH1GS0PIN, LOW);
        digitalWrite(CH1GS1PIN, HIGH);
      }
      else if(gain == GAIN_100){
        digitalWrite(CH1GS0PIN, HIGH);
        digitalWrite(CH1GS1PIN, LOW);
      }
      else if(gain == GAIN_200){
        digitalWrite(CH1GS0PIN, HIGH);
        digitalWrite(CH1GS1PIN, HIGH);
      }
      break;
    case 2:
      if(gain == GAIN_25){
        digitalWrite(CH2GS0PIN, LOW);
        digitalWrite(CH0GS1PIN, LOW);
      }
      else if(gain == GAIN_50){
        digitalWrite(CH2GS0PIN, LOW);
        digitalWrite(CH0GS1PIN, HIGH);
      }
      else if(gain == GAIN_100){
        digitalWrite(CH2GS0PIN, HIGH);
        digitalWrite(CH2GS1PIN, LOW);
      }
      else if(gain == GAIN_200){
        digitalWrite(CH2GS0PIN, HIGH);
        digitalWrite(CH2GS1PIN, HIGH);
      }
      break;
    case 3:
      if(gain == GAIN_25){
        digitalWrite(CH3GS0PIN, LOW);
        digitalWrite(CH3GS1PIN, LOW);
      }
      else if(gain == GAIN_50){
        digitalWrite(CH3GS0PIN, LOW);
        digitalWrite(CH3GS1PIN, HIGH);
      }
      else if(gain == GAIN_100){
        digitalWrite(CH3GS0PIN, HIGH);
        digitalWrite(CH3GS1PIN, LOW);
      }
      else if(gain == GAIN_200){
        digitalWrite(CH3GS0PIN, HIGH);
        digitalWrite(CH3GS1PIN, HIGH);
      }
      break;
  }
}

void InstrumentPowerDue::startSampling(){
  isSampling = true;
  TC_Start(TC0, 0);
}

void InstrumentPowerDue::stopSampling(){
  isSampling = false;
  TC_Stop(TC0, 0);
}

bool InstrumentPowerDue::bufferReady(){
  return currentBuffer!=nextBuffer;
}

void InstrumentPowerDue::writeBuffer(HardwareSerial * port){
  int i=6;
  // send it
  // ADC takes to start
  while(((buffer[currentBuffer][i])>>12)!=1){
    i++;
  };
  i-=6;
  if(i){
    buffer[currentBuffer][5+i] = (buffer[currentBuffer][5]-2*i);
    buffer[currentBuffer][4+i] = buffer[currentBuffer][4];
    buffer[currentBuffer][3+i] = buffer[currentBuffer][3];
    buffer[currentBuffer][2+i] = buffer[currentBuffer][2];
    buffer[currentBuffer][1+i] = buffer[currentBuffer][1];
    buffer[currentBuffer][0+i] = buffer[currentBuffer][0];
  }

  port->write((uint8_t *)&(buffer[currentBuffer][i]),BUFFER_SIZE_FOR_USB);
  currentBuffer=(currentBuffer+1)&(NUM_BUFFERS-1);
  return;
}

// TODO: Does this work for different channels and levels?
void InstrumentPowerDue::setDCOffset(int channel, int level){
  Wire.beginTransmission(76); // transmit to device #76 (0x4c)
  // device address is specified in datasheet
  uint8_t ctrl_byte = (0b0001 << 4) | (0b000 << 1) | 0;
  Wire.write(byte(ctrl_byte));            // sends instruction byte
  Wire.write(byte(0xFF));
  Wire.write(byte(0xFF));
  Wire.endTransmission();     // stop transmitting
}

void InstrumentPowerDue::changeSamplingRate(int sample_rate){
  // I/O line PB25 on Peripheral B is TIOA0

  // PB25 = 2 Arduino Due
  pinMode(2,OUTPUT);

  // Start the pin HIGH
  analogWrite(2,255);

  // Disables the PIO from controlling the pin 25 (enables peripheral control of pin 25).
  REG_PIOB_PDR = 1<<25;

  // Assigns the Pin 25 I/O line to the Peripheral B function.
  REG_PIOB_ABSR= 1<<25;

  // Timer Control Channel Mode register
  // ABETRG = 1 -- TIOA is used as an external trigger
  // WAVE = 1 -- Capture mode is disabled (Waveform mode is enabled): PWM with idependant duty cycles
  // CPCTRG = 1 --  RC Compare resets the counter and restarts the counter (Reset on Comparison with TC0_RC0)
  // LDRA = 01 -- Selected Edge for PIOA Rising
  // LDRB = 10 -- Selected Edge for PIOB Falling
  REG_TC0_CMR0=0b00000000000010011100010000000000; //CLock selection

  // Compare register C for the TC
  // * 4 for the 4 channels per sample
  REG_TC0_RC0=INSTRUMENT_CLOCK_RATE/(sample_rate*4);
  // Compare register A for the TC
  REG_TC0_RA0=1;

  // Stop the Timer
  // TC_STOP uses REG_TC0_CCR0 to enable and disable the clock
  TC_Stop(TC0, 0);
}

void InstrumentPowerDue::bufferFullInterrupt(){
  // Write time for the current buffer
  currentTime = micros()-timeReference;
  buffer[currentBuffer][3] = (uint16_t)(currentTime >> 16);
  buffer[currentBuffer][4] = (uint16_t)(0x0000FFFF & currentTime);

  // Increment Buffers
  nextBuffer=(currentBuffer+1)&(NUM_BUFFERS-1);


  buffer[nextBuffer][0] = SYNC_BLOCK;
  buffer[nextBuffer][1] = SYNC_BLOCK;
  buffer[nextBuffer][2] = currentTask;
  buffer[nextBuffer][5] = (BUFFER_SIZE_FOR_USB-HEADER_SIZE);
  buffer[nextBuffer][6] |= (uint16_t)0x1000;

  // next DMA buffer
  ADC->ADC_RNPR=(uint32_t)(&(buffer[nextBuffer][(HEADER_SIZE/2)]));

  //next dma receive counter
  ADC->ADC_RNCR=(BUFFER_SIZE_FOR_USB-HEADER_SIZE);
}

void InstrumentPowerDue::taskIdValidTrigger(){
  stopSampling();
  currentTask = readTaskID();
  // Set packet length
  buffer[currentBuffer][5] = ((BUFFER_SIZE_FOR_USB-HEADER_SIZE)-(ADC->ADC_RCR));
  (ADC->ADC_RCR)=0;

  startSampling();
}

void ADC_Handler()
{
  //Read reason for interrupt
  int f=ADC->ADC_ISR;

  // The Receive Counter Register has reached 0 since the last write in ADC_RCR or ADC_RNCR.
  // Check if the Buffer is full
  if (f&(1<<27)){
    PowerDue.bufferFullInterrupt();
    return;
  }

}

InstrumentPowerDue PowerDue;
