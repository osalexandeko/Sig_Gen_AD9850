/* 
 * File:   dds.h
 * Author: os
 *
 * Created on April 29, 2015, 8:22 AM
 */

#ifndef DDS_H
#define	DDS_H

#ifdef	__cplusplus
extern "C" {
#endif

#define  CLK         RB0             //pin #
#define  FREQ_UPDATE RB1             //pin #
#define  DATA        RB2             //pin #
#define  INT_LENGTH  32            //length of integer
//#define  FREQ_CONST  4294967296    //AD9850 calculation constant
//#define  XTAL_MHZ    125000000     //cristal freq. on AD9850's pcb
#define  FREQ_FACTOR 34.36            //FREQ_CONST/XTAL_MHZ
#define  INTERMED    4000000       //intermediate frequency
#define  DELAY_TIME  1000           //delay time


/******************************************************************************/
/* DDS init                                                                   */
/******************************************************************************/
void DDS_init();


/******************************************************************************/
/* Updates bit signalling to AD9850 that frequency is being loaded,           */
/* or that frequency load is stopped                                          */
/******************************************************************************/
void frequencyUpdate();

/******************************************************************************/
/* loads data a data bit and updates the clock                                */
/* @param sig the value of the data                                           */
/******************************************************************************/
void dataUpdate(int sig);

/******************************************************************************/
/* Loads the frequency.                                                       */
/* @param freq - the frequency in format of AD9850                            */
/******************************************************************************/
void load_freq(unsigned long freq);


/******************************************************************************/
/* Calculation for frequency.It converts decimal freq. in HZ to AD9850 format */
/* @param the frequency                                                       */
/* @return calculated freq                                                    */
/******************************************************************************/
long calculate_frequency(unsigned long frequency);

/******************************************************************************/
/* Updates the DDS  */
/******************************************************************************/
void update_dds(unsigned long frequency);



#ifdef	__cplusplus
}
#endif

#endif	/* DDS_H */

