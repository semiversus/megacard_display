# Megacard-I2C-Display
Library for use with I2C Display based on PCF8574 and HD44780 (2 lines, 16 characters).

How to start:
* Add `megacard_lcd.c` and `megacard_lcd.h` to your project
* Connect display (see "Connection")
* See "Example" for a usage example

## Connection
* Display GND to Megacard GND
* Display VCC to Megacard VCC
* Display SDA to Megacard PORT C Bit 1
* Display SCL to Megacard PORT C Bit 0

** Remove LED Jumper (X9) ** - otherwise it will not work

## Available commands
``void lcd_init()`` - initialise I2C and the display
``void lcd_clear()`` - clear content of display
``void lcd_printf(uint8_t line, uint8_t position, char *message, ...)`` - use like `printf` to output a string to the given line and position
``void lcd_light(uint8_t on)`` - turn on backlight if `on` is other than 0

## Example
```c
#include "megacard_lcd.h"

int main(void)
{
  lcd_init();
  
  lcd_printf(0, 4, "Megacard");
  lcd_printf(1, 2, "HTL Rankweil");
  
  while (1);
}
```