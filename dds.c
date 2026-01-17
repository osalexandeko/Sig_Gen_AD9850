#include <htc.h>
#include <string.h>
#include <xc.h>
//#include <stdint.h>
#include <pic16F877A.h>

#include "dds.h"



/******************************************************************************/
/* DDS init                                                                   */
/******************************************************************************/
void DDS_init()
{
  TRISB0 = 0;
  TRISB1 = 0;
  TRISB2 = 0;

  //this way makes signal look better in scope
  dataUpdate(0);
  //_delay(DELAY_TIME);
  dataUpdate(0);
 //_delay(DELAY_TIME);
  dataUpdate(0);
  //_delay(DELAY_TIME);
  dataUpdate(0);
 // _delay(DELAY_TIME);
  dataUpdate(0);
 // _delay(DELAY_TIME);
  dataUpdate(0);
 // _delay(DELAY_TIME);
  dataUpdate(1);
 // _delay(DELAY_TIME);
  dataUpdate(1);
 // _delay(DELAY_TIME);
}

/******************************************************************************/
/* Updates bit signalling to AD9850 that frequency is being loaded,           */
/* or that frequency load is stopped                                          */
/******************************************************************************/
void frequencyUpdate()
{
  FREQ_UPDATE = 1;
  //_delay(DELAY_TIME);
  //CLK = 1;
  //_delay(DELAY_TIME);
  //CLK = 0;
  FREQ_UPDATE = 0;
  //_delay(DELAY_TIME);
}

/******************************************************************************/
/* loads data a data bit and updates the clock                                */
/* @param sig the value of the data                                           */
/******************************************************************************/
void dataUpdate(int sig)
{
  DATA = sig;
  _delay(DELAY_TIME);
  CLK = 1;
  //_delay(DELAY_TIME);
  CLK = 0;
  DATA = 0;
  _delay(DELAY_TIME);
}


/******************************************************************************/
/* Loads the frequency.                                                       */
/* @param freq - the frequency in format of AD9850                            */
/******************************************************************************/
void load_freq(unsigned long freq)
{
  int i;
  for(i = 0; i < INT_LENGTH; i++)
    {
      dataUpdate(freq & 1);
      freq = freq >> 1;
    }
}

/******************************************************************************/
/* Calculation for frequency.It converts decimal freq. in HZ to AD9850 format */
/* @param the frequency                                                       */
/* @return calculated freq                                                    */
/******************************************************************************/
long calculate_frequency(unsigned long frequency)
{
    
    double tmp = FREQ_FACTOR*frequency;

    //return   (frequency * ((long) tmp) );
    return (long) tmp ;
}

/******************************************************************************/
/* Updates the DDS  */
/******************************************************************************/
void update_dds(unsigned long frequency)
{
  frequencyUpdate();

  load_freq(calculate_frequency(frequency));

 

  //this way makes signal look better in scope
  dataUpdate(0);
  dataUpdate(0);
  dataUpdate(0);
  dataUpdate(0);
  dataUpdate(0);
  dataUpdate(0);
  dataUpdate(0);
  dataUpdate(0);
  frequencyUpdate();
}
