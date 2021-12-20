#ifndef MEGACARD_LCD
#define MEGACARD_LCD

#include <stdint.h> 

void lcd_init(void);
void lcd_clear(void);
void lcd_light(uint8_t on);
void lcd_printf(uint8_t line, uint8_t pos, char *fmt, ...);

#endif
