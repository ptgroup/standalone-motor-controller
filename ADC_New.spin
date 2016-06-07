{{
File: ADC_Old.spin
Description: Gives a method to read from the old ADC
}}
CON


  '16 bit ADC Pins (as configured in the controller box)
  ADC_SCLK = 25
  ADC_CS = 27
  ADC_DATA = 26

OBJ
  spi: "SPI_Asm"

PUB Init
  'Initialize ADC
  dira[ADC_SCLK]~~
  dira[ADC_CS]~~
  dira[ADC_DATA]~
   
  outa[ADC_SCLK]~~
  outa[ADC_CS]~~

  spi.start(1,1) 'Start ADC data collection~

PUB ReadData : ADCbits     'Reads a single measurement from the ADC and returns it
  outa[ADC_CS]~
  outa[ADC_SCLK]~
  ADCBits := spi.SHIFTIN(ADC_DATA,ADC_SCLK,spi#MSBPOST,19)
  outa[ADC_CS]~~