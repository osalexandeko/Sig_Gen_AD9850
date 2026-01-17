#include<htc.h>
#include <xc.h>
//#include <stdint.h>
#include <pic16F877A.h>
#include "lcd.h"
/*toggles lcd*/
void toggle()
{
    EN = 1;
    _delay(2000);
    EN = 0;
}

/*init lcd*/
void init_lcd()
{
    TRISC  = 0;
    TRISB6 = 0;
    TRISB7 = 0;

    RS = 0;
    //in 8-bit mode
    PORTC = 0x38;
    toggle();

    //ENAB_DISPLAY_CURSOR
    PORTC = 0X0C; // 0x0F for blinking
    toggle();

    //CLEAR_DISPLAY
    PORTC = 0x01;
    toggle();

    //INC_CURS
    PORTC = 0X06;
    toggle();

    //SRT_CURS_1_1
    PORTC = 0X80;
    toggle();
}


/*writes a char*/
void write_char(char c)
{
    RS = 1;
    PORTC = c;
    toggle();
}

//Set cursor position to 1st line,1st column
void set_cur_l_1()
{
    RS = 0;
    PORTC = 0X80;
    toggle();
}

//Set cursor position to 2st line,1st column
void set_cur_l_2()
{
    RS = 0;
    PORTC = 0XC0;
    toggle();
}

//Set cursor position  to offset
void set_cur(char offset)
{
    RS = 0;
    PORTC = offset;
    toggle();
}

//Set cursor underline
void set_underline(char offset)
{
    RS = 0;
    PORTC = offset;
    toggle();
    PORTC = 0X0F;
    toggle();
    _delay(100000);
    PORTC = 0X0C;
    toggle();
}


/*writes a string*/
void write_str(const char  str[], int length)
{
    int i;
    for(i=0;i<length && str[i]!='\0';write_char(str[i]), i++);
    for(;i < length;i++, write_char(' '));

}