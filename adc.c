#include<htc.h>
#include <xc.h>
#include <stdint.h>
#include <pic16F877A.h>
#include "adc.h"

void ADCInit()
{
  ADCON1bits.ADFM=1;

  //All 8 pins as analog inputs
  //Vref+ = VDD
  //Vref- = VSS
  ADCON1bits.PCFG=0x0;

  //ADC Clock = Fosc/64
  ADCON0bits.ADCS=0b10;
  ADCON1bits.ADCS2=1;

  //Turn on ADC Module
  ADCON0bits.ADON=1;

}

int ReadADC(int ch)
{
  if(ch>7) return 0;

  ADCON0bits.CHS=ch;

  //Wait for aquisition
  _delay(100);

  //Start Conversion
  ADCON0bits.GO=1;

  //Wait for the conversion to complete
  while(ADCON0bits.GO);

  return ((ADRESH<<8)|ADRESL);

}

float ReadVoltage(int ch)
{

    //Read ADC
    float adc_value=ReadADC(ch);
    _delay(1000);

    adc_value+=ReadADC(ch);
    _delay(1000);

     adc_value+=ReadADC(ch);
    _delay(1000);

    adc_value/=3;

    adc_value*=VDD;

    //Convert to voltage
    adc_value /= 1024;

    return adc_value;
}


