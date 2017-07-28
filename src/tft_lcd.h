//tft_lcd.h

#ifndef __TFT_LCD_H
#define __TFT_LCD_H

#include "Adafruit_GFX.h"
#include "glcdfont.c"

void tft_WriteCmdData(uint16_t cmd, uint16_t dat);
void tft_init(void);

#endif
