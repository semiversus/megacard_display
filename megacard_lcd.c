/*****************************************************************************
 
 megacard_lcd.c - LCD over I2C library for HTL Rankweil Megacard 
		Designed for HD44870 based LCDs with I2C expander PCF8574X
		on Atmels AVR MCUs
 
 Copyright (C) 2006 Nico Eichelmann and Thomas Eichelmann
               2014 clean up by Falk Brunner 
			   2021 adaptions for HTL Rankweil Megacard by Günther Jena

               Includes I2C library by Peter Fleury <pfleury@gmx.ch>  http://jump.to/fleury
 
 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.
 
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.
 
 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 
 You can contact the authors at info@computerheld.de
 
*****************************************************************************/

/*
	Version 0.11
	Requires I2C-Library from Peter Fleury http://jump.to/fleury
	
	See i2clcd.h for description and example.
*/
#define F_CPU 12000000UL       // CPU clock for Megacard is 12MHz

#include <avr/pgmspace.h>
#include <stdbool.h>
#include <stdint.h> 
#include <inttypes.h>
#include <util/delay.h>
#include <compat/twi.h>
#include "megacard_lcd.h"

#define SCL_CLOCK  100000L

/*************************************************************************
 Initialization of the I2C bus interface. Need to be called only once
*************************************************************************/
void i2c_init(void)
{
  /* initialize TWI clock: 100 kHz clock, TWPS = 0 => prescaler = 1 */
  
  TWSR = 0;                         /* no prescaler */
  TWBR = ((F_CPU/SCL_CLOCK)-16)/2;  /* must be > 10 for stable operation */

}/* i2c_init */


/*************************************************************************	
  Issues a start condition and sends address and transfer direction.
  return 0 = device accessible, 1= failed to access device
*************************************************************************/
unsigned char i2c_start(unsigned char address)
{
    uint8_t   twst;

	// send START condition
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);

	// wait until transmission completed
	while(!(TWCR & (1<<TWINT)));

	// check value of TWI Status Register. Mask prescaler bits.
	twst = TW_STATUS & 0xF8;
	if ( (twst != TW_START) && (twst != TW_REP_START)) return 1;

	// send device address
	TWDR = address;
	TWCR = (1<<TWINT) | (1<<TWEN);

	// wail until transmission completed and ACK/NACK has been received
	while(!(TWCR & (1<<TWINT)));

	// check value of TWI Status Register. Mask prescaler bits.
	twst = TW_STATUS & 0xF8;
	if ( (twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK) ) return 1;

	return 0;

}/* i2c_start */


/*************************************************************************
 Issues a start condition and sends address and transfer direction.
 If device is busy, use ack polling to wait until device is ready
 
 Input:   address and transfer direction of I2C device
*************************************************************************/
void i2c_start_wait(unsigned char address)
{
    uint8_t   twst;


    while ( 1 )
    {
	    // send START condition
	    TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
    
    	// wait until transmission completed
    	while(!(TWCR & (1<<TWINT)));
    
    	// check value of TWI Status Register. Mask prescaler bits.
    	twst = TW_STATUS & 0xF8;
    	if ( (twst != TW_START) && (twst != TW_REP_START)) continue;
    
    	// send device address
    	TWDR = address;
    	TWCR = (1<<TWINT) | (1<<TWEN);
    
    	// wail until transmission completed
    	while(!(TWCR & (1<<TWINT)));
    
    	// check value of TWI Status Register. Mask prescaler bits.
    	twst = TW_STATUS & 0xF8;
    	if ( (twst == TW_MT_SLA_NACK )||(twst ==TW_MR_DATA_NACK) ) 
    	{    	    
    	    /* device busy, send stop condition to terminate write operation */
	        TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
	        
	        // wait until stop condition is executed and bus released
	        while(TWCR & (1<<TWSTO));
	        
    	    continue;
    	}
    	//if( twst != TW_MT_SLA_ACK) return 1;
    	break;
     }

}/* i2c_start_wait */


/*************************************************************************
 Issues a repeated start condition and sends address and transfer direction 

 Input:   address and transfer direction of I2C device
 
 Return:  0 device accessible
          1 failed to access device
*************************************************************************/
unsigned char i2c_rep_start(unsigned char address)
{
    return i2c_start( address );

}/* i2c_rep_start */


/*************************************************************************
 Terminates the data transfer and releases the I2C bus
*************************************************************************/
void i2c_stop(void)
{
    /* send stop condition */
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
	
	// wait until stop condition is executed and bus released
	while(TWCR & (1<<TWSTO));

}/* i2c_stop */


/*************************************************************************
  Send one byte to I2C device
  
  Input:    byte to be transfered
  Return:   0 write successful 
            1 write failed
*************************************************************************/
unsigned char i2c_write( unsigned char data )
{	
    uint8_t   twst;
    
	// send data to the previously addressed device
	TWDR = data;
	TWCR = (1<<TWINT) | (1<<TWEN);

	// wait until transmission completed
	while(!(TWCR & (1<<TWINT)));

	// check value of TWI Status Register. Mask prescaler bits
	twst = TW_STATUS & 0xF8;
	if( twst != TW_MT_DATA_ACK) return 1;
	return 0;

}/* i2c_write */


/*************************************************************************
 Read one byte from the I2C device, request more data from device 
 
 Return:  byte read from I2C device
*************************************************************************/
unsigned char i2c_readAck(void)
{
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
	while(!(TWCR & (1<<TWINT)));    

    return TWDR;

}/* i2c_readAck */


/*************************************************************************
 Read one byte from the I2C device, read is followed by a stop condition 
 
 Return:  byte read from I2C device
*************************************************************************/
unsigned char i2c_readNak(void)
{
	TWCR = (1<<TWINT) | (1<<TWEN);
	while(!(TWCR & (1<<TWINT)));
	
    return TWDR;

}/* i2c_readNak */


// global variable f�r light control

static uint8_t lightOn=0;

//-	Display initialization sequence

void lcd_init(void) {
    i2c_init();
	
    lcd_light(false);
    	
	_delay_ms(15);		        //-	Wait for more than 15ms after VDD rises to 4.5V
	lcd_write(CMD_D1 | CMD_D0);	//-	Set interface to 8-bit
	_delay_ms(5);			    //-	Wait for more than 4.1ms
	lcd_write(CMD_D1 | CMD_D0);	//-	Set interface to 8-bit
	_delay_ms(0.1);		        //-	Wait for more than 100us	
	lcd_write(CMD_D1 | CMD_D0);	//-	Set interface to 8-bit
	lcd_write(CMD_D1);		    //-	Set interface to 4-bit

	//- From now on in 4-bit-Mode
	lcd_command(LCD_LINE_MODE | LCD_5X7);
	lcd_command(LCD_DISPLAYON | LCD_CURSORON | LCD_BLINKINGOFF);
	lcd_command(LCD_CLEAR);
    _delay_ms(2);
	lcd_command(LCD_INCREASE | LCD_DISPLAYSHIFTOFF);
}

//-	Write data to i2c

void lcd_write_i2c(uint8_t value) {
	i2c_start_wait(LCD_I2C_DEVICE+I2C_WRITE);
	i2c_write(value);
	i2c_stop();
}

//-	Write nibble to display with pulse of enable bit
// map pinout between PCF8574 and LCD

void lcd_write(uint8_t value) {
    uint8_t data_out=0;
    
    // map data to LCD pinout
    if (value & CMD_D0) data_out |= LCD_D4;
    if (value & CMD_D1) data_out |= LCD_D5;
    if (value & CMD_D2) data_out |= LCD_D6;
    if (value & CMD_D3) data_out |= LCD_D7;
    if (value & CMD_RS) data_out |= LCD_RS;
    if (value & CMD_RW) data_out |= LCD_RW;
    if (!lightOn) data_out |= LCD_LIGHT_N;

	lcd_write_i2c(data_out | LCD_E);		//-	Set new data and enable to high
	lcd_write_i2c(data_out);	            //-	Set enable to low
}

//-	Read data from i2c

uint8_t lcd_read_i2c(void) {
	uint8_t lcddata = 0x00;

	i2c_start_wait(LCD_I2C_DEVICE+I2C_READ);
	lcddata = i2c_readNak();
	i2c_stop();
	return lcddata;
}

//-	Read data from display over i2c (lower nibble contains LCD data)

uint8_t lcd_read(bool mode) {
	uint8_t lcddata, data;

	if(mode == LCD_DATA) {
		lcddata = (LCD_E | LCD_RS | LCD_RW | LCD_D4 | LCD_D5 | LCD_D6 | LCD_D7);
	} else {
		lcddata = (LCD_E |          LCD_RW | LCD_D4 | LCD_D5 | LCD_D6 | LCD_D7);
	}

    if (!lightOn) lcddata |= LCD_LIGHT_N;
    lcd_write_i2c(lcddata);
	lcddata = lcd_read_i2c();

    data=0;
    // map data from LCD pinout to internal positions
    if (lcddata & LCD_D4) data |= CMD_D0;
    if (lcddata & LCD_D5) data |= CMD_D1;
    if (lcddata & LCD_D6) data |= CMD_D2;
    if (lcddata & LCD_D7) data |= CMD_D3;
 
    lcddata=0;   
    if (!lightOn) lcddata |= LCD_LIGHT_N;
	lcd_write_i2c(lcddata);

	return data;
}

//-	Read one complete byte via i2c from display

uint8_t lcd_getbyte(bool mode)  {
	uint8_t hi, lo;

	hi = lcd_read(mode);
	lo = lcd_read(mode);
	return (hi << 4) + (lo & 0x0F);
}

//-	Issue a command to the display (use the defined commands above)

void lcd_command(uint8_t command) {

    lcd_write((command >> 4));
	lcd_write((command & 0x0F));	
}

//-	Print string to cursor position

void lcd_print(char *string) {

	while(*string)	{
		lcd_putchar(*string++);
	}
}

//-	Print string from flash to cursor position

void lcd_print_P(PGM_P string) {
    uint8_t c;

	while((c=pgm_read_byte(string++)))	{
		lcd_putchar(c);
	}
}

//-	Put char to atctual cursor position

void lcd_putchar(char lcddata) {

	lcd_write((lcddata >> 4) | CMD_RS);
	lcd_write((lcddata & 0x0F) | CMD_RS);
}

//-	Put char to position

bool lcd_putcharlc(uint8_t line, uint8_t col, char value) {

	if(!lcd_gotolc(line, col)) return false;
	lcd_putchar(value);
	
	return true;
}

//-	Print string to position (If string is longer than LCD_COLS overwrite first chars)(line, row, string)

bool lcd_printlc(uint8_t line, uint8_t col, char *string) {

    if (!lcd_gotolc(line, col)) return false;
    	
	while(*string) {
		lcd_putchar(*string++);
		col++;
		if(col > LCD_COLS) {
			col = 1;
            lcd_gotolc(line, col);
		}
	}
	return true;
}

//-	Print string from flash to position
//  (If string is longer than LCD_COLS overwrite first chars)(line, row, string)

bool lcd_printlc_P(uint8_t line, uint8_t col, char *string) {
    char c;
    
    if (!lcd_gotolc(line, col)) return false;
    	
	while((c=pgm_read_byte(string++))) {
		lcd_putchar(c);
		col++;
		if(col > LCD_COLS) {
			col = 1;
            lcd_gotolc(line, col);
		}
	}
	return true;
}

//-	Print string to position (If string is longer than LCD_COLS continue in next line)

bool lcd_printlcc(uint8_t line, uint8_t col, char *string) {	
    
    if (!lcd_gotolc(line, col)) return false;
    	
	while(*string) {
		lcd_putchar(*string++);
		col++;
		if(col > LCD_COLS) {
			line++;
			col = 1; 
    		if(line > LCD_LINES) {
			    line = 1;
		    }
            lcd_gotolc(line, col);
		}
	}
	return true;
}

//-	Print string from flash to position
// (If string is longer than LCD_COLS continue in next line)

bool lcd_printlcc_P(uint8_t line, uint8_t col, char *string) {	
    char c;
      
    if (!lcd_gotolc(line, col)) return false;
    	
	while((c=pgm_read_byte(string++))) {
		lcd_putchar(c);
		col++;
		if(col > LCD_COLS) {
			line++;
			col = 1; 
    		if(line > LCD_LINES) {
			    line = 1;
		    }
            lcd_gotolc(line, col);
		}
	}
	return true;
}

//-	Go to position (line, column)

bool lcd_gotolc(uint8_t line, uint8_t col ) {
	uint8_t lcddata=0;
	
	if( (line > LCD_LINES) ||
	    (col > LCD_COLS) ||
	    ((line == 0) || (col == 0)) ) return false;
	
    switch (line) {
        case 1: lcddata = LCD_LINE1; break;
        case 2: lcddata = LCD_LINE2; break;
        //case 3: lcddata = LCD_LINE3; break;
       // case 4: lcddata = LCD_LINE4; break;
    }
    lcddata |= 0x80;
    lcddata += (col-1);
	lcd_command(lcddata);
	return true;	
}

//-	Go to nextline (if next line > (LCD_LINES-1) return false)

bool lcd_nextline(void) {
	uint8_t line, col;

	lcd_getlc(&line, &col);
	if (!lcd_gotolc(line + 1, 1)) {
        return false;
    } else {
        return true;
    }
}

//-	Get line and row (target byte for line, target byte for row)

bool lcd_getlc(uint8_t *line, uint8_t *col) {
	uint8_t lcddata;

	lcddata = lcd_getbyte(LCD_ADDRESS);
	if (lcddata & (1 << 7)) return false;       // LCD busy

    if (lcddata >= LCD_LINE1 && lcddata < (LCD_LINE1+LCD_COLS)) {
        *line = 1;
        *col = lcddata - LCD_LINE1 + 1;
        return true;
    } else if (lcddata >= LCD_LINE2 && lcddata < (LCD_LINE2+LCD_COLS)) {
        *line = 2;
        *col = lcddata - LCD_LINE2 + 1;
        return true;
    } /*else if (lcddata >= LCD_LINE3 && lcddata < (LCD_LINE3+LCD_COLS)) {
        *line = 3;
        *col = lcddata - LCD_LINE3 + 1;
        return true;
    } else if (lcddata >= LCD_LINE4 && lcddata < (LCD_LINE4+LCD_COLS)) {
        *line = 4;
        *col = lcddata - LCD_LINE4 + 1;
        return true;
    }*/

	return false;
}

// turn light on/off

void lcd_light(bool light) {

    if (!light) {
        lightOn = 1;
	    lcd_write_i2c(LCD_LIGHT_ON);
    } else {
        lightOn = 0;
    	lcd_write_i2c(LCD_LIGHT_OFF);
    }
}

//-	Check if busy

bool lcd_busy(void) {
	uint8_t state;
    
    state = lcd_getbyte(LCD_ADDRESS);
	if (state & (1 << 7)) {
        return true;
    } else {
        return false;
    }
}
