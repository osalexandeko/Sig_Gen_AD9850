/* 
 * File:   lcd.h
 * Author: os
 *
 * Created on April 26, 2015, 3:30 AM
 */

#ifndef LCD_H
#define	LCD_H

#ifdef	__cplusplus
extern "C" {
#endif

#define RS RB6
#define EN RB7
#define D0 RC0
#define D1 RC1
#define D2 RC2
#define D3 RC3
#define D4 RC4
#define D5 RC5
#define D6 RC6
#define D7 RC7


/*toggles lcd*/
void toggle();

/*init lcd*/
void init_lcd();

//Set cursor position to 1st line,1st column
void set_cur_l_1();

//Set cursor position to 2st line,1st column
void set_cur_l_2();

//Set cursor position  to offset
void set_cur(char offset);

/*writes a string*/
void write_str(const char  str[], int length);

//Set cursor underline
void set_underline(char offset);


#ifdef	__cplusplus
}
#endif

#endif	/* LCD_H */

