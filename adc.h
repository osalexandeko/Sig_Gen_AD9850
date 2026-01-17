/* 
 * File:   adc.h
 * Author: os
 *
 * Created on April 26, 2015, 4:10 AM
 */

#ifndef ADC_H
#define	ADC_H

#ifdef	__cplusplus
extern "C" {
#endif

//define vdd
#define VDD 5

/*Init adc*/
void ADCInit(void);

/*read channel*/
int ReadADC(int ch);

/*rasd valtage*/
float ReadVoltage(int ch);
   
#ifdef	__cplusplus
}
#endif

#endif	/* ADC_H */

