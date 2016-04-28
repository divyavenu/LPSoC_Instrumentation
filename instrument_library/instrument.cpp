/*
  Instrument Power Due ATSAM3X8E
  Library for accessing Analog and peripherals of the Instrument Processor on the PowerDue
  Created by Joao Diogo Falcao, February 2016
  Carnegie Mellon University - Silicon Valley
*/

#include "instrument.h"

void ValidTaskIDChange_Handler(){
  PowerDue.taskIdValidTrigger();
}

InstrumentPowerDue::InstrumentPowerDue():currentBuffer(0), nextBuffer(0), currentTime(0), timeReference(0), currentTask(0), isSampling(false), isInterrupted(false){

}

void InstrumentPowerDue::init(int sample_rate){
  // Start I2C
  Wire.begin();

  // queue init
  xQueue = xQueueCreate(NUM_BUFFERS, sizeof(currentBuffer));
  RxQueue = xQueueCreate(RECIEVE_QUEUE_SIZE, sizeof(char));

  xSemaphore = xSemaphoreCreateMutex();
  initStorage();

  // Task ID Pin as Inputs
  pinMode(TASK_ID_PIN_0,INPUT);
  pinMode(TASK_ID_PIN_1,INPUT);
  pinMode(TASK_ID_PIN_2,INPUT);
  pinMode(TASK_ID_PIN_3,INPUT);
  pinMode(TASK_ID_VALID_PIN,INPUT);


  //When the task valid pin goes from low to high, ISR is called
  attachInterrupt(digitalPinToInterrupt(TASK_ID_VALID_PIN), ValidTaskIDChange_Handler, RISING);

  // Start Channel 0 No Gain and No Offset
  // For setting the offset - gain 25 , 0 amp
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

// 
  changeSamplingRate(sample_rate);
  startADC();
}


uint16_t InstrumentPowerDue::readTaskID(){
  return  ((digitalRead(TASK_ID_PIN_3)<<3)|
      (digitalRead(TASK_ID_PIN_2)<<2)|
      (digitalRead(TASK_ID_PIN_1)<<1)|
      (digitalRead(TASK_ID_PIN_0)));
}

void InstrumentPowerDue::startADC(){

  //Returns the number of microseconds since the Arduino board began running the current program
  timeReference = micros();
  // Power measurement Controller Enable Peripheral ADC
  pmc_enable_periph_clk(ID_ADC);

  //Initialize the given ADC with the SystemCoreClock and startup time
  adc_init(ADC, SystemCoreClock, ADC_FREQ_MAX, ADC_STARTUP_FAST);
  // trigger on Timer Counter 0
  ADC->ADC_MR |=0x3;

  //enable 4 specific channels
  //ch7, ch3, ch2, ch0
  ADC->ADC_CHER=0x8E;

  //enable adc interrupt
  NVIC_EnableIRQ(ADC_IRQn);

  //set highest priority
  NVIC_SetPriority(ADC_IRQn, 0);

  //disable all interrupts except RXEND
  //Only accept end of receive buffer interrupt
  ADC->ADC_IDR=~(1<<27);

  //enable DMA-driven ADC RXEND interrupt
  ADC->ADC_IER=1<<27;

  //Initialize packet for DMA's buffer pointer
  currentTime = micros()-timeReference;

  buffer[currentBuffer][0] = SYNC_BLOCK; //2bytes 5555
  buffer[currentBuffer][1] = SYNC_BLOCK; //2bytes 5555
  buffer[currentBuffer][2] = readTaskID(); 
  buffer[currentBuffer][3] = (uint16_t)(currentTime >> 16); //upper half 16bits
  buffer[currentBuffer][4] = (uint16_t)(0x0000FFFF & currentTime); //lower half 16bits
  buffer[currentBuffer][5] = (BUFFER_SIZE_FOR_USB-HEADER_SIZE);
  buffer[currentBuffer][6] |= (uint16_t)0x1000; //2bytes making the 13th bit 1

  // ADC writes to the DMA buffer
  ADC->ADC_RPR=(uint32_t)(&(buffer[currentBuffer][(HEADER_SIZE/2)-1])); //write 126 samples each 2 bytes, for 4 channels

  ADC->ADC_RCR=(BUFFER_SIZE_FOR_USB-HEADER_SIZE); //How many bytes to write

  // Initialize packet for DMA's next buffer pointer
  // ---------------
  nextBuffer = currentBuffer+1;

  buffer[nextBuffer][0] = SYNC_BLOCK;
  buffer[nextBuffer][1] = SYNC_BLOCK;
  buffer[nextBuffer][2] = readTaskID();
  buffer[nextBuffer][5] = (BUFFER_SIZE_FOR_USB-HEADER_SIZE);
  buffer[nextBuffer][6] |= (uint16_t)0x1000;

  // next DMA buffer
  ADC->ADC_RNPR=(uint32_t)(&(buffer[nextBuffer][(HEADER_SIZE/2)]));

  //next dma receive counter
  ADC->ADC_RNCR=(BUFFER_SIZE_FOR_USB-HEADER_SIZE);

  //enable dma pdc receiver channel
  ADC->ADC_PTCR=1;

  // TAG measurements with channel number
  ADC->ADC_EMR|=0x01000000;

  //start adc
  ADC->ADC_CR=2;
  // ADC->ADC_CR=0;
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

// bool InstrumentPowerDue::bufferReady(){
//   return currentBuffer!=nextBuffer;
// }

int InstrumentPowerDue::bufferReady(){
  int bid;
  portBASE_TYPE xStatus = xQueueReceive(xQueue, &bid, portMAX_DELAY);
  return bid;
}

void InstrumentPowerDue::writeBuffer(Serial_ * port){
  int i=6;
  // send it
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

  // Send Queue
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xQueueSendToBackFromISR( xQueue, (void *)&currentBuffer, &xHigherPriorityTaskWoken );
  currentBuffer = (currentBuffer+1)&(NUM_BUFFERS-1);
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
  	//noInterrupts();
    PowerDue.bufferFullInterrupt();
    //interrupts();
    return;
  }
}

bool InstrumentPowerDue::writeAverage(void *packet){
  int m = 6;
  int j = 0;
  uint16_t count1 = 0;
  uint16_t count2 = 0;
  uint16_t count3 = 0;
  uint16_t count4 = 0;
  uint32_t total1 = 0;
  uint32_t total2 = 0;
  uint32_t total3 = 0;
  uint32_t total4 = 0;
  uint16_t buffer_size;

  while(((buffer[currentBuffer][m])>>12)!=1){
    m++;
  };
  m-=6;
  if(m){
    buffer[currentBuffer][5+m] = (buffer[currentBuffer][5]-2*m);
    //buffer[currentBuffer][4+m] = buffer[currentBuffer][4];
    //buffer[currentBuffer][3+m] = buffer[currentBuffer][3];
    buffer[currentBuffer][2+m] = buffer[currentBuffer][2];
    //buffer[currentBuffer][1+m] = buffer[currentBuffer][1];
    //buffer[currentBuffer][0+m] = buffer[currentBuffer][0];
  }
   
  buffer_size = ((buffer[currentBuffer][5]-2*m)/2);

	while ( buffer_size > 0){ 
    
	    if (buffer_size > 0){
	      total1 = total1 + (buffer[currentBuffer][(6+m) + j] & 0b0000111111111111) ;
	      buffer_size = buffer_size - 1;
	      count1++; 	 
	    }
	    if (buffer_size > 0){
	      total2 = total2 + (buffer[currentBuffer][(7+m)+ j] & 0b0000111111111111);
	      buffer_size = buffer_size - 1;
	      count2++;
	    }
	    if (buffer_size > 0){
	      total3 = total3 + (buffer[currentBuffer][(8+m) + j]& 0b0000111111111111);
	      buffer_size = buffer_size - 1;
	      count3++;
	    }
	    if (buffer_size > 0 ){
	      total4 = total4 + (buffer[currentBuffer][(9+m) + j]& 0b0000111111111111);
	      buffer_size = buffer_size - 1;
	      count4++;
	    }
	    j = j+4;
	}
    
  //buffer[currentBuffer][5+m] =  ((BUFFER_SIZE_FOR_AVERAGE-HEADER_SIZE)-2*m);
 	if (count1 && count2 && count3 && count4){ 
		total1 /= count1;
		total2 /= count2;
		total3 /= count3;
		total4 /= count4;
	  
		*(uint8_t *)packet = (uint8_t)buffer[currentBuffer][2+m]; 
		*(uint8_t *)(packet+1) = (uint8_t)(((buffer[currentBuffer][6+m] & 0xF000) |((uint16_t)(total1))) >> 8 );
		*(uint8_t *)(packet+2) = (uint8_t)(((uint16_t)total1) & 0x00FF);
		*(uint8_t *)(packet+3) = (uint8_t)(((buffer[currentBuffer][7+m] & 0xF000) |((uint16_t)(total2))) >> 8 );
		*(uint8_t *)(packet+4) = (uint8_t)(((uint16_t)total2) & 0x00FF);
		*(uint8_t *)(packet+5) = (uint8_t)(((buffer[currentBuffer][8+m] & 0xF000) |((uint16_t)(total3))) >> 8 );
		*(uint8_t *)(packet+6) = (uint8_t)(((uint16_t)total3) & 0x00FF);
		*(uint8_t *)(packet+7) = (uint8_t)(((buffer[currentBuffer][9+m] & 0xF000) |((uint16_t)(total4))) >> 8 );
		*(uint8_t *)(packet+8) = (uint8_t)(((uint16_t)total4) & 0x00FF);
	}

	currentBuffer=(currentBuffer+1)&(NUM_BUFFERS-1);
	return (count1 && count2 && count3 && count4);
}

void InstrumentPowerDue::accumStorage(int bid){
	int m = 6;
	int j = 0;
	int taskIndex;
	uint16_t count = 0;
	uint32_t total[4];
	uint16_t buffer_size;
	uint8_t taskId = (uint8_t)buffer[bid][2];

	while(((buffer[bid][m])>>12)!=1)
		m++;
	m-=6;   
 	buffer_size = (buffer[bid][5]-2*m)/2;

	for (int i = 0; i < 4; i++)
		total[i] = 0;	

	while (buffer_size >= 4){     
		for (int i = 0; i < 4; i++)
			total[i] += (buffer[bid][(6+m+i)+j] & 0x0FFF);     			
		count++;
		buffer_size -= 4;
	    j += 4;
	}

	if (count > 0){
		xSemaphoreTake(xSemaphore, portMAX_DELAY);
		for (taskIndex = 0; taskIndex < numberOfTasks[currentStorage]; taskIndex++){
			if(accumTaskId[currentStorage][taskIndex] == taskId)
				break;
		}
		if (taskIndex == numberOfTasks[currentStorage]){
			accumTaskId[currentStorage][taskIndex] = taskId;
			numberOfTasks[currentStorage]++;
		}

		for (int i = 0; i < 4; i++)
			accumTotal[currentStorage][taskIndex][i] += total[i];
		accumCount[currentStorage][taskIndex] += count;
		xSemaphoreGive(xSemaphore);
	}

	//currentBuffer=(currentBuffer+1)&(NUM_BUFFERS-1);
	return;
}

void InstrumentPowerDue::sendHeader(USARTClass *port){
	uint8_t sid = currentStorage;

	xSemaphoreTake(xSemaphore, portMAX_DELAY);
	currentStorage = (currentStorage+1)&(NUM_STORAGE-1);
	xSemaphoreGive(xSemaphore);

  // SYNC bytes comes before the header
  char sync[5] = "5566"; 
  port->write(sync, 4);
	port->write(numberOfTasks[sid]);

	// Creating Packet
	for (int i = 0; i < numberOfTasks[sid]; i++){
    // first byte: TaskID (4 bits) + lower half of the first byte of accumCount (4 bits)
		packet[i][0] = (accumTaskId[sid][i] << 4) || (0x0F && (uint8_t)(accumCount[sid][i] >> 24));
    // byte index 1,2,3: lower 3 bytes of accumCount
    packet[i][1] = (uint8_t)(accumCount[sid][i] >> 16);
    packet[i][2] = (uint8_t)(accumCount[sid][i] >> 8);
    packet[i][3] = (uint8_t)(accumCount[sid][i]);

    // 8 bytes of data, excluding channel ID
    // byte index 5,7, 9, 11
    int offset = 3;
		for (int j = 0; j < 4; j++){
      // calculate
			accumTotal[sid][i][j] /= accumCount[sid][i];
			if (accumTotal[sid][i][j] > 4095){
				packet[i][j*2+1+offset] = 0xFF;
				packet[i][j*2+2+offset] = 0xFF;			
			} else {
				packet[i][j*2+1+offset] = (uint8_t)(accumTotal[sid][i][j] >> 8);
				packet[i][j*2+2+offset] = (uint8_t)(accumTotal[sid][i][j]);
			}
		}
    // channel Id for all channels
		packet[i][offset+1] |= 0x10;
		packet[i][offset+3] |= 0x20;
		packet[i][offset+5] |= 0x30;
		packet[i][offset+7] |= 0x70;

		// packet[i][9] = (uint8_t)(accumCount[sid][i] >> 8);
		// packet[i][10] = (uint8_t)(accumCount[sid][i]);
	}
	// packetSize = numberOfTasks[sid] * 11;
  packetSize = numberOfTasks[sid] * NUM_BYTES;

	// Initialize Storage
	for (int i = 0; i < MAX_TASKS; i++){
		for (int j = 0; j < 4; j++)
			accumTotal[sid][i][j] = 0;
		accumCount[sid][i] = 0;
		accumTaskId[sid][i] = 0xFF;
	}
	numberOfTasks[sid] = 0;
	return;
}

void InstrumentPowerDue::sendPacket(USARTClass *port){
	port->write((uint8_t *)&packet, packetSize);
	packetSize = 0;
}

void InstrumentPowerDue::initStorage(){
	xSemaphoreTake(xSemaphore, portMAX_DELAY);	
	for (int sid = 0; sid < NUM_STORAGE; sid++){
		for (int i = 0; i < MAX_TASKS; i++){
			for (int j = 0; j < 4; j++) {
				accumTotal[sid][i][j] = 0;
      }
			accumCount[sid][i] = 0;
			accumTaskId[sid][i] = 0xFF;
		}
		numberOfTasks[sid] = 0;
	}
	packetSize = 0;
	xSemaphoreGive(xSemaphore);
	return;	
}


void UARTClass::callback(Uart *pUart) {
  BaseType_t xHigherPriorityTaskWoken;
  xQueueSendFromISR( PowerDue.RxQueue, (void *)&_pUart->UART_RHR, &xHigherPriorityTaskWoken );
 }

InstrumentPowerDue PowerDue;
