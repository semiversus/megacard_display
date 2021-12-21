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
 
*/

#define F_CPU 12000000UL       // CPU clock for Megacard is 12MHz

#include <stdio.h>
#include <util/delay.h>
#include <compat/twi.h>
#include "megacard_lcd.h"

static uint8_t backlight_on=0;
static uint8_t initialized=0;

void lcd_write_i2c(uint8_t value) {
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
        TWDR = 0x4E;
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
        break;
     }

    // send data to the previously addressed device
    TWDR = value;
    TWCR = (1<<TWINT) | (1<<TWEN);

    // wait until transmission completed
    while(!(TWCR & (1<<TWINT)));

    /* send stop condition */
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
    
    // wait until stop condition is executed and bus released
    while(TWCR & (1<<TWSTO));
}

void lcd_write(uint8_t value) {
    uint8_t data_out=(value << 4) | ((value & 0x30) >> 4);
    if (!backlight_on) data_out |= 1 << 3;
    lcd_write_i2c(data_out | (1<<2));       //-    Set new data and enable to high
    lcd_write_i2c(data_out);                //-    Set enable to low
}

void lcd_command(uint8_t command) {
    lcd_write((command >> 4));
    lcd_write((command & 0x0F));    
}

void lcd_init(void) {
    // I2C init
    TWSR = 0;
    TWBR = 52;
    
    lcd_light(1);
        
    _delay_ms(15);           //-    Wait for more than 15ms after VDD rises to 4.5V
    lcd_write(3);            //-    Set interface to 8-bit
    _delay_ms(5);            //-    Wait for more than 4.1ms
    lcd_write(3);            //-    Set interface to 8-bit
    _delay_ms(0.1);          //-    Wait for more than 100us    
    lcd_write(3);            //-    Set interface to 8-bit
    lcd_write(2);            //-    Set interface to 4-bit

    //- From now on in 4-bit-Mode
    lcd_command(0x28);  // line mode and 5x7
    lcd_command(0x0C);  // display on, cursor off, blinking off
    lcd_command(0x01);  // clear
    _delay_ms(2);
    lcd_command(0x06);  // set move cursor direction, display not shifted
}

void lcd_clear(void) {
    lcd_command(0x01);
	_delay_ms(2);
}

void lcd_light(uint8_t light) {
    if (!light) {
        backlight_on = 1;
        lcd_write_i2c(0x00);
    } else {
        backlight_on = 0;
        lcd_write_i2c(1<<3);
    }
}

void lcd_printf(uint8_t line, uint8_t pos, char *msg, ...) {
    char buffer[17];
    va_list args;

      va_start (args, msg);
    vsnprintf(buffer, 17, msg, args);
    va_end(args);

    if (line > 1 || pos > 15) {
        return;
    }

    uint8_t lcddata = 0x80;
    
    if (line == 1) {
        lcddata |= 0x40;
    }

    lcddata |= 0x80;
    lcddata += pos;
    lcd_command(lcddata);

    uint8_t index = 0;

    while (pos < 16 && buffer[index]) {
        lcd_write((buffer[index] >> 4) | (1<<4));
        lcd_write((buffer[index] & 0x0F) | (1<<4));
        pos++;
        index++;
    }
}
