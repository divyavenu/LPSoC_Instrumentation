/*
*
* Firmware for the PowerDué (Based on the Arduino Due)
* Example for computing and FFT on A0 and sending the results via WiFi
* Carnegie Mellon University Silicon Valley
* Version: 0.0.1
*
*/

#define LOG_OUT 1 // use the log output function
#define FFT_N 256 // set to 256 point fft

#include <PowerDue.h>
#include <FFT.h> // include the library


#define NUM_SAMPLES_PER_BUFFER 256
#define ZERO_LEVEL 2048
// Each sample is 2bytes and there are 4 channels
#define BUFFER_SIZE (NUM_SAMPLES_PER_BUFFER*2) // in bytes

// Has to be a power of 2
#define NUM_BUFFERS 4
#define SAMPLE_RATE 50000

#define LOG_OUT 1 // use the log output function
#define FFT_N 256 // set to 256 point fft

uint16_t buffer[NUM_BUFFERS][BUFFER_SIZE];
volatile int currentBuffer=0, nextBuffer=0;

void startADC(){

  // Power measurement Controller Enable Peripheral ADC
  pmc_enable_periph_clk(ID_ADC);

  //Initialize the given ADC with the SystemCoreClock and startup time
  adc_init(ADC, SystemCoreClock, ADC_FREQ_MAX, ADC_STARTUP_FAST);

  // trigger on Timer Counter 0
  ADC->ADC_MR |=0x3;

  //enable CH3 - A1 of PowerDue
  ADC->ADC_CHER=0x8;

  //enable adc interrupt
  NVIC_EnableIRQ(ADC_IRQn);

  //set highest priority
  NVIC_SetPriority(ADC_IRQn, 0);

  //disable all interrupts except RXEND
  ADC->ADC_IDR=~(1<<27);

  //enable DMA-driven ADC RXEND interrupt
  ADC->ADC_IER=1<<27;

  // ADC writes to the DMA buffer
  ADC->ADC_RPR=(uint32_t)(buffer[currentBuffer]);

  ADC->ADC_RCR=BUFFER_SIZE;

  // Initialize packet for DMA's next buffer pointer
  // ---------------
  nextBuffer = currentBuffer+1;

  // next DMA buffer
  ADC->ADC_RNPR=(uint32_t)(buffer[nextBuffer]);

  //next dma receive counter
  ADC->ADC_RNCR=BUFFER_SIZE;

  //enable dma pdc receiver channel
  ADC->ADC_PTCR=1;

  //start adc
  ADC->ADC_CR=2;
}


void startSampling(){
  TC_Start(TC0, 0);
}

void stopSampling(){
  TC_Stop(TC0, 0);
}

bool bufferReady(){
  return currentBuffer!=nextBuffer;
}

void writeBuffer(Serial_ * port){
  port->write(fft_log_out,NUM_SAMPLES_PER_BUFFER/2);
  currentBuffer=(currentBuffer+1)&(NUM_BUFFERS-1);
  return;
}

void changeSamplingRate(int sample_rate){
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
  REG_TC0_RC0=INSTRUMENT_CLOCK_RATE/(sample_rate);
  // Compare register A for the TC
  REG_TC0_RA0=1;

  // Stop the Timer
  // TC_STOP uses REG_TC0_CCR0 to enable and disable the clock
  TC_Stop(TC0, 0);
}

void bufferFullInterrupt(){
  // Increment Buffers
  nextBuffer=(currentBuffer+1)&(NUM_BUFFERS-1);

  // next DMA buffer
  ADC->ADC_RNPR=(uint32_t)(buffer[nextBuffer]);

  //next dma receive counter
  ADC->ADC_RNCR=BUFFER_SIZE;
}

void ADC_Handler()
{
  //Read reason for interrupt
  int f=ADC->ADC_ISR;

  // The Receive Counter Register has reached 0 since the last write in ADC_RCR or ADC_RNCR.
  // Check if the Buffer is full
  if (f&(1<<27)){
    bufferFullInterrupt();
    return;
  }

}


void setup(){
  // initialize serial:
  SerialUSB.begin(0);
  while(!SerialUSB);

  PowerDue.init();
  changeSamplingRate(SAMPLE_RATE);
  startADC();
}

void loop(){

}


/*
fft_adc.pde
guest openmusiclabs.com 8.18.12
example sketch for testing the fft library.
it takes in data on ADC0 (Analog0) and processes them
with the fft. the data is sent out over the serial
port at 115.2kb.  there is a pure data patch for
visualizing the data.
*/
void fft_prepare_data_input(){
  for (int i = 0, int k = 0 ; i < NUM_SAMPLES_PER_BUFFER/2 ; i += 2, k++) {
    fft_input[i] = (int16_t)buffer[currentBuffer][k] - (int16_t)ZERO_LEVEL;
    fft_input[i+1] = 0; // set odd bins to 0
  }
}

while(1){
  // wait for the sampling to start
  while(!bufferReady());
  fft_window(); // window the data for better frequency response
  fft_reorder(); // reorder the data before doing the fft
  fft_run(); // process the data in the fft
  fft_mag_log(); // take the output of the fft
  SerialUSB.write(255);
  writeBuffer(&SerialUSB);
}
void loop() {
  while(1) { // reduces jitter
    cli();  // UDRE interrupt slows this way down on arduino1.0

  }
}
