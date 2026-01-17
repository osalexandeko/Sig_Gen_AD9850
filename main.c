#include <htc.h>
#include <string.h>
#include <xc.h>
#include <stdint.h>
#include <stdlib.h>
#include <pic16F877A.h>
#include "lcd.h"
#include "adc.h"
#include "dds.h"


// BEGIN CONFIG
#pragma config FOSC = HS // Oscillator Selection bits (HS oscillator)
#pragma config WDTE = OFF // Watchdog Timer Enable bit (WDT enabled)
#pragma config PWRTE = ON // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = ON // Brown-out Reset Enable bit (BOR enabled)
#pragma config LVP = OFF // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3 is digital I/O, HV on MCLR must be used for programming)
#pragma config CPD = ON // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
#pragma config WRT = OFF // Flash Program Memory Write Enable bits (Write protection off; all program memory may be written to by EECON control)
#pragma config CP = OFF // Flash Program Memory Code Protection bit (Code protection off)

/*infinite loop*/
#define FOREVER for(;;);

/*Buttons*/
#define BUTTON_1  RB5
#define BUTTON_2  RB4
#define BUTTON_3  RB3







/*xtal*/
#define _XTAL_FREQ 20000000

//globals

//frequency
unsigned long freq = 14000000;

//step of frequency
unsigned long step = 1 ;

//digit at coursor
unsigned current_digit = 9;

/*move integer to string*/
/*char* itoa1(unsigned long i, char b[])
{
    char const digit[] = "0123456789";
    char* p = b;
    /*if(i<0)
    {
        *p++ = '-';
        i *= -1;
    }*/
   /* int shifter = i;
    do{ //Move to where representation ends
        ++p;
        shifter = shifter/10;
    }while(shifter);
    *p = '\0';
    do{ //Move back, inserting digits as u go
        *--p = digit[i%10];
        i = i/10;
    }while(i);
    return b;
}*/

// convert float to string one decimal digit at a time
 // assumes float is < 65536 and ARRAYSIZE is big enough
 // problem: it truncates numbers at size without rounding
 // str is a char array to hold the result, float is the number to convert
 // size is the number of decimal digits you want
 void FloatToString(char str[], float f, char size)
 {
 	char pos;  // position in string
 	char len;  // length of decimal part of result
 	char curr;  // temp holder for next digit
 	unsigned int value;  // decimal digit(s) to convert
 	pos = 0;  // initialize pos, just to be sure

 	value = (int)f;  // truncate the floating point number
 	utoa(str, value, 10);  // this is kinda dangerous depending on the length of str
 	// now str array has the digits before the decimal

 	if (f < 0 )  // handle negative numbers
 	{
 		f *= -1;
 		value *= -1;
 	}

        len = strlen(str);  // find out how big the integer part was
 	pos = len;  // position the pointer to the end of the integer part
 	str[pos++] = '.';  // add decimal point to string

 	while(pos < (size + len + 1) )  // process remaining digits
 	{
 		f = f - (float)value;  // hack off the whole part of the number
 		f *= 10;  // move next digit over
 		value = (int)f;  // get next digit
 		curr = '0' + value ; // convert digit to string
 		str[pos++] = curr; // add digit to result string and increment pointer
 	}
 }

 /*
  * Transfer freq number to gui string
  */
void freq_to_string(char * buf, unsigned long val)
{
    int i = 9;
    const char digit[10] = "0123456789";
    while(i >= 0)
    {
        if(i == 2 || i == 6)
        {
            buf[i] = '.';
            
        }else{
            buf[i] = digit[val%10];
            val /= 10;
        }
        i--;

    }
    
}

/*Interrupt service routine*/
void interrupt ISR()
{
     
    char buffer [10];
     
    if(INTCONbits.TMR0IF == 1)                        // Timer 0 Interrupt
    {
            //set_cur_l_2();
            set_cur(0xC0 + 11 );
            FloatToString(buffer, ReadVoltage(0), 6);
            write_str(buffer,4);
            write_str("V", 2);

            set_underline(0x80 + current_digit);

            INTCONbits.TMR0IF = 0;  // Bit T0IF is cleared so that the interrupt could reoccur
    }


    if(BUTTON_1 == 0 && freq <= 44000000)
    {

         freq+= step;
         update_dds(freq);

         set_cur_l_1();
         
         freq_to_string(buffer, freq);
         write_str(buffer, 10);
         _delay(1000000 );
         


    }

    if(BUTTON_2 == 0 && step <= freq)
    {
        freq-= step;
        update_dds(freq);

        set_cur_l_1();
         
        freq_to_string(buffer, freq);

        write_str(buffer, 10);
        _delay(1000000 );
        
         

    }

    if(BUTTON_3 == 0)
    {
       
       
       switch(step)
       {
           case 1:
               step = 10;
               current_digit = 8;
               break;
           case 10:
               step = 100;
               current_digit = 7;
               break;
           case 100:
               step = 1000;
               current_digit = 5;
               break;
           case 1000:
               step = 10000;
               current_digit = 4;
               break;
           case 10000:
               step = 100000;
               current_digit = 3;
               break;
           case 100000:
               step = 1000000;
               current_digit = 1;
               break;
           case 1000000:
               step = 1;
               current_digit = 9;
               break;
           default:
               step = 1;
               current_digit = 9;
                
       }

       
       
       _delay(5000000);

    }

    
}


/*Inits the tomer #0*/
void TMR0_init()
{
    OPTION_REG = 0x07; // Prescaler (1:256) is assigned to the timer TMR0
    TMR0 = 0;          // Timer T0 counts from 39 to 255
}

void main(void)
{
        			
        ADCInit();
        init_lcd();
        TMR0_init();
        set_cur_l_1();
        DDS_init();
        

        INTCON = 0xA8;     // Enable interrupts 

        
        /*Buttons*/
        TRISB5 = 1;
        TRISB4 = 1;
        TRISB3 = 1;


       //update_dds(freq);

        
	FOREVER
}