/*
 * gateway.cpp
 *
 * Created: 4/17/2017 11:43:37 AM
 * Author : Zulkar Nayem
 * code is written in Atmel Studio 7.0
 */ 
#define F_CPU 8000000

#include <avr/io.h>
#include <stdlib.h>
#include <util/delay.h>

#include "RFM69.h"
#include "RFM69registers.h"
#include "hd44780.c"
#include "hd44780.h"
#include "hd44780_settings.h"

#define NETWORKID 33
#define NODEID    4

int main(void)
{
	// initialize RFM69
	rfm69_init(NODEID,NETWORKID);
	setHighPower(1); // if model number rfm69hw
	setPowerLevel(30); // 0-31; 5dBm to 20 dBm 
	encrypt(NULL); // if set has to be 16 bytes. example: "1234567890123456"
	
	// initialize 16x2 LCD
	lcd_init();
	lcd_clrscr();
	  
    while (1) 
    {
		if(receiveDone())
		{
			if(ACKRequested())
				sendACK();
			char stringData[16];
			for(uint8_t i=0;i<16;i++) // max 16 digit can be shown in this case
			{
				stringData[i]=DATA[i];
			}
			lcd_clrscr();
			lcd_puts(stringData);
			
		}
    }
}

