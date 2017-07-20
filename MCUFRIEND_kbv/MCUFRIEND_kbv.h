#ifndef MCUFRIEND_KBV_H_
#define MCUFRIEND_KBV_H_

#include <stdbool.h>
#include "stm32f10x.h"

//#define USE_SERIAL

//#if ARDUINO < 165
//#define USE_GFX_KBV
//#include "ADA_GFX_kbv.h"
//#else
//#include "Adafruit_GFX.h"
//#endif

// from ADAFRUIT_GFX
typedef enum
{ False = 0,
  True = !False
} boolean;

typedef struct
{
	int32_t WIDTH, HEIGHT;			/* This is the 'raw' display w/h - never changes */
	int32_t _width, _height;		/* Display w/h as modified by current rotation */
	int32_t cursor_x, cursor_y;
	uint16_t textcolor, textbgcolor;
	uint8_t textsize, rotation;
	boolean wrap;					/* If set, 'wrap' text at right edge of display */

	void (*drawPixel)(int32_t, int32_t, uint16_t);	/* This MUST be set by the creator of a GFX_Object */
} GFX_Object;



/*class MCUFRIEND_kbv : public Adafruit_GFX {*/

	/*public:*/
#if defined USE_GFX_KBV
	MCUFRIEND_kbv();
#elif defined(ARDUINO_GENERIC_STM32F103C) || defined(ARDUINO_GENERIC_STM32F103V) || defined(ARDUINO_MAPLE_REV3)
	MCUFRIEND_kbv(int CS=0, int RS=0, int WR=0, int RD=0, int RST=0);  //dummy arguments 
#else
//	MCUFRIEND_kbv(int CS=A3, int RS=A2, int WR=A1, int RD=A0, int RST=A4);
void	MCUFRIEND_init(int CS, int RS, int WR, int RD, int RST);
#endif
	
	void     MCUFRIEND_reset(void);                                       // you only need the constructor
	void     MCUFRIEND_begin(uint16_t ID/* = 0x9341*/);                       // you only need the constructor
	/*virtual*/ void     MCUFRIEND_drawPixel(int16_t x, int16_t y, uint16_t color);  // and these three
	void     MCUFRIEND_WriteCmdData(uint16_t cmd, uint16_t dat);                 // ?public methods !!!
  void     MCUFRIEND_pushCommand(uint16_t cmd, uint8_t * block, int8_t N);
	uint16_t MCUFRIEND_color565(uint8_t r, uint8_t g, uint8_t b) { return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | ((b & 0xF8) >> 3); }
	uint16_t MCUFRIEND_readID(void);
	/*virtual*/ void     MCUFRIEND_fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
	//virtual void     MCUFRIEND_drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color) { MCUFRIEND_fillRect(x, y, 1, h, color); }
	//virtual void     MCUFRIEND_drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color) { MCUFRIEND_fillRect(x, y, w, 1, color); }
	/*virtual*/ void     MCUFRIEND_fillScreen(uint16_t color);//                                     { MCUFRIEND_fillRect(0, 0, GFX_Object._width, GFX_Object._height, color); }
	/*virtual*/ void     MCUFRIEND_setRotation(uint8_t r);
  /*virtual*/ void     MCUFRIEND_invertDisplay(boolean i);

	uint16_t MCUFRIEND_readReg(uint16_t reg, int8_t index);
	int16_t  MCUFRIEND_readGRAM(int16_t x, int16_t y, uint16_t *block, int16_t w, int16_t h);
	uint16_t MCUFRIEND_readPixel(int16_t x, int16_t y) { uint16_t color; MCUFRIEND_readGRAM(x, y, &color, 1, 1); return color; }
	void     MCUFRIEND_setAddrWindow(int16_t x, int16_t y, int16_t x1, int16_t y1);
	void     MCUFRIEND_pushColors16(uint16_t *block, int16_t n, boolean first);
	void     MCUFRIEND_pushColors8(uint8_t *block, int16_t n, boolean first);
	void     MCUFRIEND_pushColors(const uint8_t *block, int16_t n, boolean first, boolean bigend);
  void     MCUFRIEND_vertScroll(int16_t top, int16_t scrollines, int16_t offset);

    
//	protected:
	uint32_t MCUFRIEND_readReg32(uint16_t reg);
	uint32_t MCUFRIEND_readReg40(uint16_t reg);
  uint16_t  _lcd_xor, _lcd_capable;

	//	private:	
	uint16_t _lcd_ID, _lcd_rev, _lcd_madctl, _lcd_drivOut, _MC, _MP, _MW, _SC, _EC, _SP, _EP;


#endif
