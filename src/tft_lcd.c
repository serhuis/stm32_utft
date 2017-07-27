
#include "tft_lcd.h"
#include "mcufriend_keil.h"

// control pins as used in MCUFRIEND shields 
#define TFT_PORT 				GPIOB
#define TFT_PORT_MASK		0x0FFF			//RB12-RB15 are for SPI
#define TFT_DATA_MASK		0x0FF0


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
#define CD_COMMAND		{PIN_LOW(CD_PORT, CD_PIN);}
#define CD_DATA    		{PIN_HIGH(CD_PORT, CD_PIN);}
#define CD_OUTPUT  		PIN_OUTPUT(CD_PORT, CD_PIN)
 
#define CS_ACTIVE  		PIN_LOW(CS_PORT, CS_PIN)
#define CS_IDLE    		PIN_HIGH(CS_PORT, CS_PIN)
#define CS_OUTPUT  		PIN_OUTPUT(CS_PORT, CS_PIN)
#define RESET_ACTIVE  PIN_LOW(RESET_PORT, RESET_PIN)
#define RESET_IDLE    PIN_HIGH(RESET_PORT, RESET_PIN)
#define RESET_OUTPUT  PIN_OUTPUT(RESET_PORT, RESET_PIN)

// General macros.   IOCLR registers are 1 cycle when optimised.
#define WR_STROBE { WR_ACTIVE(); WR_IDLE(); }         //PWLW=TWRL=50ns
#define RD_STROBE {RD_IDLE(), RD_ACTIVE(), RD_ACTIVE(), RD_ACTIVE();}   //PWLR=TRDL=150ns

//#define write_8(x)    { TFT_PORT = (TFT_PORT & ~TFT_DATA_MASK) | ((x) & TFT_DATA_MASK);}
//#define write8(d) { WR_ACTIVE; TFT_PORT = (TFT_PORT & ~TFT_DATA_MASK) | ((x) & TFT_DATA_MASK); WR_IDLE;} // STROBEs are defined later
// read 250ns after RD_ACTIVE goes low
//#define read8 ( RD_ACTIVE(), RD_ACTIVE(), RD_ACTIVE(), RD_ACTIVE(), RD_ACTIVE(), RD_ACTIVE(), read_8() )


//#define write16(x)    { uint8_t h = (x)>>8, l = x; write8(h); write8(l); }
//#define write16(x)    {write8((x)>>8); write8(x); }
//#define READ_8(dst)   { dst = read8(); RD_IDLE; }
#define READ_16(dst)  { dst = read8(); dst = (dst<<8) | read8(); RD_IDLE; }

#define CTL_INIT   { RD_OUTPUT; WR_OUTPUT; CD_OUTPUT; CS_OUTPUT; RESET_OUTPUT; }
//#define WriteCmd(x)  { CD_COMMAND; write16(x);}
//#define WriteData(x) { CD_DATA(); write16(x);}

#define PORT_SET_WRITE {TFT_PORT->CRH = (TFT_PORT->CRH & 0xFFFF0000) | 0x00003333; TFT_PORT->CRL = (TFT_PORT->CRL & 0x0000FFFF) | 0x33330000;}
#define PORT_SET_READ  {TFT_PORT->CRH = (TFT_PORT->CRH & 0xFFFF0000) | 0x00004444; TFT_PORT->CRL = (TFT_PORT->CRL & 0x0000FFFF) | 0x44440000;}


void tft_delay(u32 delay)
{
	while(--delay);
}


GFX_Object tft;
u8 done_reset=0;


void write8(u8 data)
{
	WR_ACTIVE;
	PORT_SET_WRITE;
	TFT_PORT->ODR = ((TFT_PORT->ODR & ~TFT_DATA_MASK) | (data<<4)); 
	WR_IDLE;

}

u8 read8(void)
{
	u8 result;
	
	RD_ACTIVE;
	PORT_SET_READ;
	result = ((TFT_PORT->IDR & TFT_DATA_MASK)>>4);
	RD_IDLE;
	return result;
}

u16 read16(void)
{
	u16 result=0;
	
	RD_ACTIVE;
	PORT_SET_READ;
	result = ((TFT_PORT->IDR & TFT_DATA_MASK)<<4);
	RD_IDLE;

	RD_ACTIVE;
	result |= ((TFT_PORT->IDR & TFT_DATA_MASK)>>4);
	RD_IDLE;

	return result;
}
void write16(u16 x)
{
	write8(((x&0xFF00)>>8)&0xFF);
	write8(x&0xFF);
}
void WriteCmd(u8 x)
{ 
	CD_COMMAND; 
	write16(x);
}
void WriteData(u8 x) 
{ 
	CD_DATA; 
	write16(x);
}



void tft_init()
{

	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_DeInit(TFT_PORT);

	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC, ENABLE );
	
	// Configure LCD Back Light (PA8) as output push-pull 
	GPIO_InitStructure.GPIO_Pin = TFT_PORT_MASK;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	
	GPIO_Init( TFT_PORT, &GPIO_InitStructure );

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init( RESET_PORT, &GPIO_InitStructure );
	
	
	
	done_reset = 1;
	PORT_SET_WRITE;
	CTL_INIT;
	CS_IDLE;
	RD_IDLE;
	WR_IDLE;
	RESET_IDLE;
	tft_delay(50);
	RESET_ACTIVE;
	tft_delay(100);
	RESET_IDLE;
	tft_delay(100);
	tft_WriteCmdData(0xB0, 0x0000);   //R61520 needs this to read ID
	
	Adafruit_GFX_Init(&tft);
}


void tft_reset(void)
{
    done_reset = 1;
    PORT_SET_WRITE;
    CTL_INIT;
    CS_IDLE;
    RD_IDLE;
    WR_IDLE;
    RESET_IDLE;
    tft_delay(50);
    RESET_ACTIVE;
    tft_delay(100);
    RESET_IDLE;
    tft_delay(100);
		tft_WriteCmdData(0xB0, 0x0000);   //R61520 needs this to read ID
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
/*
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
*/

uint16_t tft_readReg(uint16_t reg, int8_t index)
{
    uint16_t ret;
    uint8_t lo;

	if (!done_reset)
        tft_reset();
    CS_ACTIVE;
    WriteCmd(reg);
    PORT_SET_READ;
    CD_DATA;
    tft_delay(1);    //1us should be adequate
    //    READ_16(ret);
    do { ret = read16(); }while (--index >= 0);  //need to test with SSD1963
    RD_IDLE;
    CS_IDLE;
    PORT_SET_WRITE;
		
    return ret;
}

uint32_t tft_readReg32(uint16_t reg)
{
    uint16_t h = tft_readReg(reg, 0);
    uint16_t l = tft_readReg(reg, 1);
    return ((uint32_t) h << 16) | (l);
}

uint32_t tft_readReg40(uint16_t reg)
{
    uint16_t h = tft_readReg(reg, 0);
    uint16_t m = tft_readReg(reg, 1);
    uint16_t l = tft_readReg(reg, 2);
    return ((uint32_t) h << 24) | (m << 8) | (l >> 8);
}


void MCUFRIEND_kbv::setRotation(uint8_t r)
{
    uint16_t GS, SS, ORG, REV = _lcd_rev;
    uint8_t val, d[3];
    rotation = r & 3;           // just perform the operation ourselves on the protected variables
    _width = (rotation & 1) ? HEIGHT : WIDTH;
    _height = (rotation & 1) ? WIDTH : HEIGHT;
    switch (rotation) {
    case 0:                    //PORTRAIT:
        val = 0x48;             //MY=0, MX=1, MV=0, ML=0, BGR=1
        break;
    case 1:                    //LANDSCAPE: 90 degrees
        val = 0x28;             //MY=0, MX=0, MV=1, ML=0, BGR=1
        break;
    case 2:                    //PORTRAIT_REV: 180 degrees
        val = 0x98;             //MY=1, MX=0, MV=0, ML=1, BGR=1
        break;
    case 3:                    //LANDSCAPE_REV: 270 degrees
        val = 0xF8;             //MY=1, MX=1, MV=1, ML=1, BGR=1
        break;
    }
    if (_lcd_capable & INVERT_GS)
        val ^= 0x80;
    if (_lcd_capable & INVERT_SS)
        val ^= 0x40;
    if (_lcd_capable & INVERT_RGB)
        val ^= 0x08;
    if (_lcd_capable & MIPI_DCS_REV1) {
        if (_lcd_ID == 0x6814) {  //.kbv my weird 0x9486 might be 68140
            GS = (val & 0x80) ? (1 << 6) : 0;   //MY
            SS = (val & 0x40) ? (1 << 5) : 0;   //MX
            val &= 0x28;        //keep MV, BGR, MY=0, MX=0, ML=0
            d[0] = 0;
            d[1] = GS | SS | 0x02;      //MY, MX
            d[2] = 0x3B;
            WriteCmdParamN(0xB6, 3, d);
            goto common_MC;
        } else if (_lcd_ID == 0x1963 || _lcd_ID == 0x9481 || _lcd_ID == 0x1511 || _lcd_ID == 0x1581) {
            if (val & 0x80)
                val |= 0x01;    //GS
            if ((val & 0x40))
                val |= 0x02;    //SS
            if (_lcd_ID == 0x1581) { // no Horizontal Flip
                d[0] = (val & 0x40) ? 0x13 : 0x12; //REV | BGR | SS
                WriteCmdParamN(0xC0, 1, d);
            }
            if (_lcd_ID == 0x1963) val &= ~0xC0;
            if (_lcd_ID == 0x9481 || _lcd_ID == 0x1581) val &= ~0xD0;
            if (_lcd_ID == 0x1511) {
                val &= ~0x10;   //remove ML
                val |= 0xC0;    //force penguin 180 rotation
            }
//            val &= (_lcd_ID == 0x1963) ? ~0xC0 : ~0xD0; //MY=0, MX=0 with ML=0 for ILI9481
            goto common_MC;
        } else if (is8347) {
            _MC = 0x02, _MP = 0x06, _MW = 0x22, _SC = 0x02, _EC = 0x04, _SP = 0x06, _EP = 0x08;
			if (_lcd_ID == 0x5252) {
			    val |= 0x02;   //VERT_SCROLLON
				if (val & 0x10) val |= 0x04;   //if (ML) SS=1 kludge mirror in XXX_REV modes
            }
			goto common_BGR;
        }
      common_MC:
        _MC = 0x2A, _MP = 0x2B, _MW = 0x2C, _SC = 0x2A, _EC = 0x2A, _SP = 0x2B, _EP = 0x2B;
      common_BGR:
        WriteCmdParamN(is8347 ? 0x16 : 0x36, 1, &val);
        _lcd_madctl = val;
//	    if (_lcd_ID	== 0x1963) WriteCmdParamN(0x13, 0, NULL);   //NORMAL mode
    }
    // cope with 9320 variants
    else {
        switch (_lcd_ID) {
#if defined(SUPPORT_0139) || defined(SUPPORT_0154)
#ifdef SUPPORT_0139
        case 0x0139:
            _SC = 0x46, _EC = 0x46, _SP = 0x48, _EP = 0x47;
            goto common_S6D;
#endif
#ifdef SUPPORT_0154
        case 0x0154:
            _SC = 0x37, _EC = 0x36, _SP = 0x39, _EP = 0x38;
            goto common_S6D;
#endif
          common_S6D:
            _MC = 0x20, _MP = 0x21, _MW = 0x22;
            GS = (val & 0x80) ? (1 << 9) : 0;
            SS = (val & 0x40) ? (1 << 8) : 0;
            WriteCmdData(0x01, GS | SS | 0x0028);       // set Driver Output Control
            goto common_ORG;
#endif
        case 0x5420:
        case 0x7793:
        case 0x9326:
		case 0xB509:
            _MC = 0x200, _MP = 0x201, _MW = 0x202, _SC = 0x210, _EC = 0x211, _SP = 0x212, _EP = 0x213;
            GS = (val & 0x80) ? (1 << 15) : 0;
			uint16_t NL;
			NL = ((432 / 8) - 1) << 9;
            if (_lcd_ID == 0x9326 || _lcd_ID == 0x5420) NL >>= 1;
            WriteCmdData(0x400, GS | NL);
            goto common_SS;
        default:
            _MC = 0x20, _MP = 0x21, _MW = 0x22, _SC = 0x50, _EC = 0x51, _SP = 0x52, _EP = 0x53;
            GS = (val & 0x80) ? (1 << 15) : 0;
            WriteCmdData(0x60, GS | 0x2700);    // Gate Scan Line (0xA700)
          common_SS:
            SS = (val & 0x40) ? (1 << 8) : 0;
            WriteCmdData(0x01, SS);     // set Driver Output Control
          common_ORG:
            ORG = (val & 0x20) ? (1 << 3) : 0;
#ifdef SUPPORT_8230
            if (_lcd_ID == 0x8230) {    // UC8230 has strange BGR and READ_BGR behaviour
                if (rotation == 1 || rotation == 2) {
                    val ^= 0x08;        // change BGR bit for LANDSCAPE and PORTRAIT_REV
                }
            }               
#endif
            if (val & 0x08)
                ORG |= 0x1000;  //BGR
            _lcd_madctl = ORG | 0x0030;
            WriteCmdData(0x03, _lcd_madctl);    // set GRAM write direction and BGR=1.
            break;
#ifdef SUPPORT_1289
        case 0x1289:
            _MC = 0x4E, _MP = 0x4F, _MW = 0x22, _SC = 0x44, _EC = 0x44, _SP = 0x45, _EP = 0x46;
            if (rotation & 1)
                val ^= 0xD0;    // exchange Landscape modes
            GS = (val & 0x80) ? (1 << 14) | (1 << 12) : 0;      //called TB (top-bottom)
            SS = (val & 0x40) ? (1 << 9) : 0;   //called RL (right-left)
            ORG = (val & 0x20) ? (1 << 3) : 0;  //called AM
            _lcd_drivOut = GS | SS | (REV << 13) | 0x013F;      //REV=0, BGR=0, MUX=319
            if (val & 0x08)
                _lcd_drivOut |= 0x0800; //BGR
            WriteCmdData(0x01, _lcd_drivOut);   // set Driver Output Control
            WriteCmdData(0x11, ORG | 0x6070);   // set GRAM write direction.
            break;
#endif
		}
    }
    if ((rotation & 1) && ((_lcd_capable & MV_AXIS) == 0)) {
        uint16_t x;
        x = _MC, _MC = _MP, _MP = x;
        x = _SC, _SC = _SP, _SP = x;    //.kbv check 0139
        x = _EC, _EC = _EP, _EP = x;    //.kbv check 0139
    }
    setAddrWindow(0, 0, width() - 1, height() - 1);
    vertScroll(0, HEIGHT, 0);   //reset scrolling after a rotation
}


void tft_drawPixel(int16_t x, int16_t y, uint16_t color)
{
  // MCUFRIEND just plots at edge if you try to write outside of the box:

  if (x < 0 || y < 0 || x >= width() || y >= height())
		return;
	tft_WriteCmdData(_MC, x);
	tft_WriteCmdData(_MP, y);
	tft_WriteCmdData(_MW, color);

}


/*
#ifdef SUPPORT_0154
    case 0x0154:
        _lcd_capable = AUTO_READINC | REV_SCREEN;
        static const uint16_t S6D0154_regValues[] PROGMEM = {
            0x0011, 0x001A,
            0x0012, 0x3121,     //BT=3, DC1=1, DC2=2, DC3=1
            0x0013, 0x006C,     //GVD=108
            0x0014, 0x4249,     //VCM=66, VML=73

            0x0010, 0x0800,     //SAP=8
            TFTLCD_DELAY, 10,
            0x0011, 0x011A,     //APON=0, PON=1, AON=0, VCI1_EN=1, VC=10
            TFTLCD_DELAY, 10,
            0x0011, 0x031A,     //APON=0, PON=3, AON=0, VCI1_EN=1, VC=10
            TFTLCD_DELAY, 10,
            0x0011, 0x071A,     //APON=0, PON=7, AON=0, VCI1_EN=1, VC=10
            TFTLCD_DELAY, 10,
            0x0011, 0x0F1A,     //APON=0, PON=15, AON=0, VCI1_EN=1, VC=10
            TFTLCD_DELAY, 10,
            0x0011, 0x0F3A,     //APON=0, PON=15, AON=1, VCI1_EN=1, VC=10 
            TFTLCD_DELAY, 30,

            0x0001, 0x0128,
            0x0002, 0x0100,
            0x0003, 0x1030,
            0x0007, 0x1012,
            0x0008, 0x0303,
            0x000B, 0x1100,
            0x000C, 0x0000,
            0x000F, 0x1801,
            0x0015, 0x0020,
            
               0x0050,0x0101,
               0x0051,0x0603,
               0x0052,0x0408,
               0x0053,0x0000,
               0x0054,0x0605,
               0x0055,0x0406,
               0x0056,0x0303,
               0x0057,0x0303,
               0x0058,0x0010,
               0x0059,0x1000,
            
            0x0007, 0x0012,     //GON=1, REV=0, D=2
            TFTLCD_DELAY, 40,
            0x0007, 0x0013,     //GON=1, REV=0, D=3
            0x0007, 0x0017,     //GON=1, REV=1, D=3 DISPLAY ON 
        };
        init_table16(S6D0154_regValues, sizeof(S6D0154_regValues));

        break;
#endif

*/