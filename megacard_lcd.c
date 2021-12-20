#define F_CPU 12000000UL       // CPU clock for Megacard is 12MHz

#include <avr/pgmspace.h>
#include <stdbool.h>
#include <stdint.h> 
#include <inttypes.h>
#include <util/delay.h>
#include <compat/twi.h>
#include "megacard_lcd.h"

static uint8_t backlight_on=0;

#define LCD_D4_PIN            4    /**< LCD-Pin D4 is connected to P4 on the PCF8574 */
#define LCD_D5_PIN            5    /**< LCD-Pin D5 is connected to P5 on the PCF8574 */
#define LCD_D6_PIN            6    /**< LCD-Pin D6 is connected to P6 on the PCF8574 */
#define LCD_D7_PIN            7    /**< LCD-Pin D7 is connected to P7 on the PCF8574 */
#define LCD_RS_PIN            0    /**< LCD-Pin RS is connected to P0 on the PCF8574 */
#define LCD_RW_PIN            1    /**< LCD-Pin RW is connected to P1 on the PCF8574 */
#define LCD_E_PIN              2    /**< LCD-Pin E is connected to P2 on the PCF8574 */
#define LCD_LIGHT_PIN        3    /**< LCD backlight is connected to P3 on the PCF8574, low active */

#define LCD_D4                (1 << LCD_D4_PIN)    /**< bit 4 in 2nd lower nibble */
#define LCD_D5                (1 << LCD_D5_PIN)    /**< bit 5 in 2nd lower nibble */
#define LCD_D6                (1 << LCD_D6_PIN)    /**< bit 6 in 2nd lower nibble */
#define LCD_D7                (1 << LCD_D7_PIN)    /**< bit 7 in 2nd lower nibble */

#define LCD_RS                (1 << LCD_RS_PIN)    /**< RS-bit in 1st and 2nd higher nibble */
#define LCD_RW                (1 << LCD_RW_PIN)    /**< RW-bit in 1st and 2nd higher nibble */
#define LCD_LIGHT_N            (1 << LCD_LIGHT_PIN)/**< LCD backlight control, low active */
#define LCD_E                (1 << LCD_E_PIN)    /**< E-bit in 1st and 2nd higher nibble */

/*@}*/

// data & control bits for internal use, do not change!

#define CMD_D0                (1 << 0)    /**< bit 0 in lower nibble */
#define CMD_D1                (1 << 1)    /**< bit 1 in lower nibble */
#define CMD_D2                (1 << 2)    /**< bit 2 in lower nibble */
#define CMD_D3                (1 << 3)    /**< bit 3 in lower nibble */
#define CMD_RS                (1 << 4)    /**< RS-bit */
#define CMD_RW                (1 << 5)    /**< RW-bit */

/** \defgroup DEFINED_READ_MODES DEFINED READ MODES
*/
/*@{*/
#define LCD_ADDRESS            0    /**< Used for reading the address-counter and busy-flag */
#define LCD_DATA            1    /**< Used for reading data */
/*@}*/

//-LCD-COMMANDS------------------------------------------------------------------------------------------------------
/** \defgroup DEFINED_COMMANDS DEFINED COMMANDS
 These defined commands should be used to configure the display. \n
 Don't use commands from different categories together. \n
 
 Configuration commands from one category should get combined to one command.
 \par Example: 
 \code lcd_command(LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKINGON); \endcode
 
 The category modes like LCD_SHIFTMODE and LCD_CONFIGURATION can be omitted.
*/
/*@{*/ 

/** @name GENERAL COMMANDS */
/*@{*/ 
#define LCD_CLEAR            0x01    /**< Clear screen */
#define LCD_HOME            0x02    /**< Cursor move to first digit */
/*@}*/ 

/** @name ENTRYMODES */
/*@{*/ 
#define LCD_ENTRYMODE            0x04            /**< Set entrymode */
    #define LCD_INCREASE        LCD_ENTRYMODE | 0x02    /**<    Set cursor move direction -- Increase */
    #define LCD_DECREASE        LCD_ENTRYMODE | 0x00    /**<    Set cursor move direction -- Decrease */
    #define LCD_DISPLAYSHIFTON    LCD_ENTRYMODE | 0x01    /**<    Display is shifted */
    #define LCD_DISPLAYSHIFTOFF    LCD_ENTRYMODE | 0x00    /**<    Display is not shifted */
/*@}*/ 

/** @name DISPLAYMODES */
/*@{*/ 
#define LCD_DISPLAYMODE            0x08            /**< Set displaymode */
    #define LCD_DISPLAYON        LCD_DISPLAYMODE | 0x04    /**<    Display on */
    #define LCD_DISPLAYOFF        LCD_DISPLAYMODE | 0x00    /**<    Display off */
    #define LCD_CURSORON        LCD_DISPLAYMODE | 0x02    /**<    Cursor on */
    #define LCD_CURSOROFF        LCD_DISPLAYMODE | 0x00    /**<    Cursor off */
    #define LCD_BLINKINGON        LCD_DISPLAYMODE | 0x01    /**<    Blinking on */
    #define LCD_BLINKINGOFF        LCD_DISPLAYMODE | 0x00    /**<    Blinking off */
/*@}*/ 

/** @name SHIFTMODES */
/*@{*/ 
#define LCD_SHIFTMODE            0x10            /**< Set shiftmode */
    #define LCD_DISPLAYSHIFT    LCD_SHIFTMODE | 0x08    /**<    Display shift */
    #define LCD_CURSORMOVE        LCD_SHIFTMODE | 0x00    /**<    Cursor move */
    #define LCD_RIGHT            LCD_SHIFTMODE | 0x04    /**<    Right shift */
    #define LCD_LEFT            LCD_SHIFTMODE | 0x00    /**<    Left shift */
/*@}*/ 

/** @name DISPLAY_CONFIGURATION */
/*@{*/ 
#define LCD_CONFIGURATION        0x20                /**< Set function */
    #define LCD_8BIT        LCD_CONFIGURATION | 0x10    /**<    8 bits interface */
    #define LCD_4BIT        LCD_CONFIGURATION | 0x00    /**<    4 bits interface */
    #define LCD_4LINE                            0x09    /**<    4 line display */
    #define LCD_2LINE        LCD_CONFIGURATION | 0x08    /**<    2 line display */
    #define LCD_1LINE        LCD_CONFIGURATION | 0x00    /**<    1 line display */
    #define LCD_5X10        LCD_CONFIGURATION | 0x04    /**<    5 X 10 dots */
    #define LCD_5X7            LCD_CONFIGURATION | 0x00    /**<    5 X 7 dots */

#define LCD_LIGHT_OFF        LCD_LIGHT_N       // low active
#define LCD_LIGHT_ON        0x00


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
        break;
     }

}

void i2c_stop(void)
{
    /* send stop condition */
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
    
    // wait until stop condition is executed and bus released
    while(TWCR & (1<<TWSTO));

}

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

}

void lcd_write_i2c(uint8_t value) {
    i2c_start_wait(0x4E);
    i2c_write(value);
    i2c_stop();
}

void lcd_write(uint8_t value) {
    uint8_t data_out=(value << 4) | ((value & 0x30) >> 4);
    if (!backlight_on) data_out |= 1 << 3;
    lcd_write_i2c(data_out | (1<<2));        //-    Set new data and enable to high
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
    
    lcd_light(false);
        
    _delay_ms(15);           //-    Wait for more than 15ms after VDD rises to 4.5V
    lcd_write(3);            //-    Set interface to 8-bit
    _delay_ms(5);            //-    Wait for more than 4.1ms
    lcd_write(3);            //-    Set interface to 8-bit
    _delay_ms(0.1);          //-    Wait for more than 100us    
    lcd_write(3);            //-    Set interface to 8-bit
    lcd_write(2);            //-    Set interface to 4-bit

    //- From now on in 4-bit-Mode
    lcd_command(0x28);
    lcd_command(0x0C);
    lcd_command(0x01);
    _delay_ms(2);
    lcd_command(0x06);
}



void lcd_light(uint8_t light) {
    if (!light) {
        backlight_on = 1;
        lcd_write_i2c(LCD_LIGHT_ON);
    } else {
        backlight_on = 0;
        lcd_write_i2c(LCD_LIGHT_OFF);
    }
}


#include <stdio.h>

void lcd_printf(uint8_t line, uint8_t pos, char *msg, ...) {
    char buffer[16];
    va_list args;

      va_start (args, msg);
    vsnprintf(buffer, 16, msg, args);
    va_end(args);

    if (line > 1 || pos > 15) {
        return;
    }

    uint8_t lcddata = 0x80;
    
    if (line == 1) {
        lcddata |= 0x40;
    }

    lcddata |= 0x80;
    lcddata += pos - 1;
    lcd_command(lcddata);

    uint8_t index = 0;

    while (pos < 16 && buffer[index]) {
        lcd_write((buffer[index] >> 4) | CMD_RS);
        lcd_write((buffer[index] & 0x0F) | CMD_RS);
        pos++;
        index++;
    }
}
