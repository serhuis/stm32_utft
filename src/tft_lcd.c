
#include "tft_lcd.h"
#include "mcufriend_keil.h"

// control pins as used in MCUFRIEND shields 
#define TFT_PORT 				GPIOB
#define TFT_DATA_MASK		0x00FF

#define RD_PORT 				GPIOB
#define RD_PIN  				GPIO_Pin_2

#define WR_PORT 				GPIOB
#define WR_PIN  				GPIO_Pin_3

#define CD_PORT 				GPIOB
#define CD_PIN  				GPIO_Pin_0

#define CS_PORT 				GPIOB
#define CS_PIN  				GPIO_Pin_1

#define RESET_PORT 			GPIOC
#define RESET_PIN  			GPIO_Pin_13

#define PIN_LOW(port, pin)    (port)->BRR = (1<<(pin))
#define PIN_HIGH(port, pin)   (port)->BSRR = (1<<(pin))
#define PIN_READ(port, pin)   (port)->IDR & (1<<(pin))
#define PIN_MODE4(reg, pin, mode) reg=(reg&~(0xF<<((pin)<<2)))|(mode<<((pin)<<2))
#define PIN_OUTPUT(port, pin) {if (pin > 7) PIN_MODE4((port)->CRH, (pin&7), 0x3); else  PIN_MODE4((port)->CRL, pin, 0x3); } //50MHz push-pull only 0-7
#define PIN_INPUT(port, pin) {if (pin > 7) PIN_MODE4((port)->CRH, (pin&7), 0x4); else  PIN_MODE4((port)->CRL, pin, 0x4); }  //input


// general purpose pin macros
#define RD_ACTIVE  		PIN_LOW(RD_PORT, RD_PIN)
#define RD_IDLE    		PIN_HIGH(RD_PORT, RD_PIN)
#define RD_OUTPUT  		PIN_OUTPUT(RD_PORT, RD_PIN)

#define WR_ACTIVE  		PIN_LOW(WR_PORT, WR_PIN)
#define WR_IDLE    		PIN_HIGH(WR_PORT, WR_PIN)
#define WR_OUTPUT  		PIN_OUTPUT(WR_PORT, WR_PIN)
#define CD_COMMAND		PIN_LOW(CD_PORT, CD_PIN)
#define CD_DATA    		PIN_HIGH(CD_PORT, CD_PIN)
#define CD_OUTPUT  		PIN_OUTPUT(CD_PORT, CD_PIN)
 
#define CS_ACTIVE  		PIN_LOW(CS_PORT, CS_PIN)
#define CS_IDLE    		PIN_HIGH(CS_PORT, CS_PIN)
#define CS_OUTPUT  		PIN_OUTPUT(CS_PORT, CS_PIN)
#define RESET_ACTIVE  PIN_LOW(RESET_PORT, RESET_PIN)
#define RESET_IDLE    PIN_HIGH(RESET_PORT, RESET_PIN)
#define RESET_OUTPUT  PIN_OUTPUT(RESET_PORT, RESET_PIN)

// General macros.   IOCLR registers are 1 cycle when optimised.
#define WR_STROBE { WR_ACTIVE; WR_IDLE; }         //PWLW=TWRL=50ns
#define RD_STROBE RD_IDLE, RD_ACTIVE, RD_ACTIVE, RD_ACTIVE   //PWLR=TRDL=150ns

#define write8(d) { write_8(d); WR_ACTIVE; WR_ACTIVE; WR_STROBE; WR_IDLE; } // STROBEs are defined later
// read 250ns after RD_ACTIVE goes low
#define read8() ( RD_STROBE, RD_ACTIVE, RD_ACTIVE, RD_ACTIVE, RD_ACTIVE, RD_ACTIVE, RD_ACTIVE, read_8() )


#define write16(x)    { uint8_t h = (x)>>8, l = x; write8(h); write8(l); }
#define READ_8(dst)   { dst = read8(); RD_IDLE; }
#define READ_16(dst)  { dst = read8(); dst = (dst<<8) | read8(); RD_IDLE; }

#define CTL_INIT()   { RD_OUTPUT; WR_OUTPUT; CD_OUTPUT; CS_OUTPUT; RESET_OUTPUT; }
#define WriteCmd(x)  { CD_COMMAND; write16(x); }
#define WriteData(x) { CD_DATA; write16(x); }






GFX_Object tft;
void tft_init()
{
	/*
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_DeInit(KEYS_PORT);

	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOB, ENABLE );
	
	// Configure LCD Back Light (PA8) as output push-pull 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_2|GPIO_Pin_4|GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init( KEYS_PORT, &GPIO_InitStructure );

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_10|GPIO_Pin_12|GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init( KEYS_PORT, &GPIO_InitStructure );
	
	GPIO_AFConfigure(AFIO_SWJ_JTAG_NO_SW);
	
	KEYS_PORT->BRR |= GPIO_Pin_0|GPIO_Pin_2|GPIO_Pin_4|GPIO_Pin_6;
*/	
	
	
	Adafruit_GFX_Init(&tft);
}
void tft_reset(void)
{
    done_reset = 1;
    setWriteDir();
    CTL_INIT();
    CS_IDLE;
    RD_IDLE;
    WR_IDLE;
    RESET_IDLE;
    delay(50);
    RESET_ACTIVE;
    delay(100);
    RESET_IDLE;
    delay(100);
	WriteCmdData(0xB0, 0x0000);   //R61520 needs this to read ID
}

void tft_WriteCmdData(uint16_t cmd, uint16_t dat)
{
    CS_ACTIVE;
    WriteCmd(cmd);
    WriteData(dat);
    CS_IDLE;
}

static void WriteCmdParamN(uint16_t cmd, int8_t N, uint8_t * block)
{
    CS_ACTIVE;
    WriteCmd(cmd);
    while (N-- > 0) {
        uint8_t u8 = *block++;
        CD_DATA;
        write8(u8);
/*
			if (N && is8347) {
            cmd++;
            WriteCmd(cmd);
        }
*/
    }
    CS_IDLE;
}

static void WriteCmdParam4(uint8_t cmd, uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4)
{
    uint8_t d[4];
    d[0] = d1, d[1] = d2, d[2] = d3, d[3] = d4;
    WriteCmdParamN(cmd, 4, d);
}

static uint16_t read16bits(void)
{
    uint16_t ret;
    uint8_t lo;
#if USING_16BIT_BUS
    READ_16(ret);               //single strobe to read whole bus
    if (ret > 255)              //ID might say 0x00D3
        return ret;
#else
    READ_8(ret);
#endif
    //all MIPI_DCS_REV1 style params are 8-bit
    READ_8(lo);
    return (ret << 8) | lo;
}

uint16_t tft_readReg(uint16_t reg, int8_t index)
{
    uint16_t ret;
    uint8_t lo;
    if (!done_reset)
        reset();
    CS_ACTIVE;
    WriteCmd(reg);
    setReadDir();
    CD_DATA;
    delay(1);    //1us should be adequate
    //    READ_16(ret);
    do { ret = read16bits(); }while (--index >= 0);  //need to test with SSD1963
    RD_IDLE;
    CS_IDLE;
    setWriteDir();
    return ret;
}

uint32_t tft_readReg32(uint16_t reg)
{
    uint16_t h = readReg(reg, 0);
    uint16_t l = readReg(reg, 1);
    return ((uint32_t) h << 16) | (l);
}

uint32_t tft_readReg40(uint16_t reg)
{
    uint16_t h = readReg(reg, 0);
    uint16_t m = readReg(reg, 1);
    uint16_t l = readReg(reg, 2);
    return ((uint32_t) h << 24) | (m << 8) | (l >> 8);
}



void tft_drawPixel(int16_t x, int16_t y, uint16_t color)
{
    // MCUFRIEND just plots at edge if you try to write outside of the box:
    if (x < 0 || y < 0 || x >= width() || y >= height())
        return;
#if defined(OFFSET_9327)
	if (_lcd_ID == 0x9327) {
	    if (rotation == 2) y += OFFSET_9327;
	    if (rotation == 3) x += OFFSET_9327;
    }
#endif
    if (_lcd_capable & MIPI_DCS_REV1) {
        WriteCmdParam4(_MC, x >> 8, x, x >> 8, x);
        WriteCmdParam4(_MP, y >> 8, y, y >> 8, y);
    } else {
        WriteCmdData(_MC, x);
        WriteCmdData(_MP, y);
    }
#if defined(SUPPORT_9488_555)
    if (is555) color = color565_to_555(color);
#endif
    WriteCmdData(_MW, color);
}