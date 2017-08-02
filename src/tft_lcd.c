
#include "tft_lcd.h"
//#include "mcufriend_keil.h"

// control pins as used in MCUFRIEND shields 
#define myTFT_PORT 				GPIOB
#define myTFT_PORT_MASK		0x0FFF			//RB12-RB15 are for SPI
#define myTFT_DATA_MASK		0x0FF0


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

#define PIN_LOW(port, pin)    (port)->BRR |= (pin)
#define PIN_HIGH(port, pin)   (port)->BSRR |= (pin)
#define PIN_READ(port, pin)   (port)->IDR & (pin)


// general purpose pin macros
#define RD_ACTIVE  		PIN_LOW(RD_PORT, RD_PIN)
#define RD_IDLE    		PIN_HIGH(RD_PORT, RD_PIN)
//#define RD_OUTPUT  		PIN_OUTPUT(RD_PORT, RD_PIN)

#define WR_ACTIVE  		PIN_LOW(WR_PORT, WR_PIN)
#define WR_IDLE    		PIN_HIGH(WR_PORT, WR_PIN)

#define CD_COMMAND		{PIN_LOW(CD_PORT, CD_PIN);}
#define CD_DATA    		{PIN_HIGH(CD_PORT, CD_PIN);}

 
#define CS_ACTIVE  		PIN_LOW(CS_PORT, CS_PIN)
#define CS_IDLE    		PIN_HIGH(CS_PORT, CS_PIN)

#define RESET_ACTIVE  PIN_LOW(RESET_PORT, RESET_PIN)
#define RESET_IDLE    PIN_HIGH(RESET_PORT, RESET_PIN)


// General macros.   IOCLR registers are 1 cycle when optimised.
#define WR_STROBE { WR_ACTIVE; WR_IDLE; }         //t~200ns
#define RD_STROBE {RD_ACTIVE, RD_ACTIVE, RD_ACTIVE,RD_IDLE;}   //PWLR=TRDL=150ns


#define PORT_SET_WRITE 	{myTFT_PORT->ODR &= ~myTFT_DATA_MASK;\
												myTFT_PORT->CRH = (myTFT_PORT->CRH & 0xFFFF0000) | 0x00003333; myTFT_PORT->CRL = (myTFT_PORT->CRL & 0x0000FFFF) | 0x33330000;}
#define PORT_SET_READ  {myTFT_PORT->ODR &= ~myTFT_DATA_MASK;\
												myTFT_PORT->CRH = (myTFT_PORT->CRH & 0xFFFF0000) | 0x00004444; myTFT_PORT->CRL = (myTFT_PORT->CRL & 0x0000FFFF) | 0x44440000;}

void myTFT_drawPixel(s32 x, s32 y, u16 color);




//initialising the GFX TFT object

GFX_Object myTFT ={
	.WIDTH = 240,
	.HEIGHT = 320,			/* This is the 'raw' display w/h - never changes */
	.cursor_x = 0,
	.cursor_y = 0,
	.textcolor = 0,
	.textbgcolor = 255,
	.rotation = 0,
	.drawPixel = myTFT_drawPixel	/* This MUST be set by the creator of a GFX_Object */
};

void myTFT_delay(u32 delay)
{
	if(0 == delay)
		return;
	while(--delay)
	{
		__nop;
		__nop;
		__nop;
		__nop;
		__nop;
	};
}

u8 done_reset=0;


void write8(u8 data)
{
	PORT_SET_WRITE;
	WR_ACTIVE; 
	myTFT_delay(1);
	myTFT_PORT->ODR = ((myTFT_PORT->ODR & ~myTFT_DATA_MASK) | (data<<4)); 
	myTFT_delay(1);
	WR_IDLE;
	myTFT_delay(10);
}

u16 read16(void)
{
	u16 result=0;
	
	PORT_SET_READ;
	RD_ACTIVE;
	myTFT_delay(10);
	result = ((myTFT_PORT->IDR & myTFT_DATA_MASK)<<4);
	myTFT_delay(10);
	RD_IDLE;
	myTFT_delay(10);
	RD_ACTIVE;
	myTFT_delay(10);
	result |= ((myTFT_PORT->IDR & myTFT_DATA_MASK)>>4);
	myTFT_delay(10);
	RD_IDLE;
	
	return result;
}
void write16(u16 x)
{
	write8(((x&0xFF00)>>8)&0xFF);
	write8(x&0xFF);
}
void myTFT_WriteCmd(u16 x)
{ 
	CD_COMMAND; 
	CS_ACTIVE;
	write16(x);
	CS_IDLE;
}
void myTFT_WriteData(u16 x) 
{ 
	CD_DATA; 
	CS_ACTIVE;
	write16(x);
	CS_IDLE;
}


void myTFT_reset(void)
{

    CS_IDLE;
    RD_IDLE;
    WR_IDLE;
    RESET_IDLE;
    myTFT_delay(50);
    RESET_ACTIVE;
    done_reset = 1;
		myTFT_delay(100);
    RESET_IDLE;
    myTFT_delay(100);
}

void myTFT_WriteCmdData(uint16_t cmd, uint16_t dat)
{
    myTFT_WriteCmd(cmd);
    myTFT_WriteData(dat);
}
/*
static void myTFT_WriteCmdParamN(uint16_t cmd, int8_t N, uint8_t * block)
{
    CS_ACTIVE;
    myTFT_WriteCmd(cmd);
    while (N-- > 0) {
        uint8_t u8 = *block++;
        CD_DATA;
        write8(u8);
    }
    CS_IDLE;
}
*/
/*
static void myTFT_WriteCmdParam4(uint8_t cmd, uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4)
{
    uint8_t d[4];
    d[0] = d1, d[1] = d2, d[2] = d3, d[3] = d4;
    myTFT_WriteCmdParamN(cmd, 4, d);
}
*/
uint16_t myTFT_readReg(uint16_t reg, u8 index)
{
    uint16_t ret;

	if (!done_reset)
        myTFT_reset();
  myTFT_WriteCmd(reg);

	CS_ACTIVE;
	CD_DATA;
  myTFT_delay(1);    //1us should be adequate
	ret = read16();

	CS_IDLE;
  return ret;
}
uint32_t myTFT_readReg32(uint16_t reg)
{
    uint16_t h = myTFT_readReg(reg, 0);
    uint16_t l = myTFT_readReg(reg, 1);
    return ((uint32_t) h << 16) | (l);
}

uint32_t myTFT_readReg40(uint16_t reg)
{
    uint16_t h = myTFT_readReg(reg, 0);
    uint16_t m = myTFT_readReg(reg, 1);
    uint16_t l = myTFT_readReg(reg, 2);
    return ((uint32_t) h << 24) | (m << 8) | (l >> 8);
}



void myTFT_setAddrWindow(int16_t x, int16_t y, int16_t x1, int16_t y1)
{
//  myTFT_WriteCmdData(0x20, x);
//  myTFT_WriteCmdData(0x21, y);

  myTFT_WriteCmdData(0x37, x);
  myTFT_WriteCmdData(0x39, y);
  myTFT_WriteCmdData(0x36, x1);
  myTFT_WriteCmdData(0x38, y1);
}

void myTFT_vertScroll(int16_t top, int16_t scrollines, int16_t offset)
{
  int16_t vsp;
  int16_t sea = top;
	
  if (offset <= -scrollines || offset >= scrollines) offset = 0; //valid scroll
	vsp = top + offset; // vertical start position
    if (offset < 0)
			
  vsp += scrollines;          //keep in unsigned range
  myTFT_WriteCmdData(0x31, sea);        //SEA
  myTFT_WriteCmdData(0x32, top);        //SSA
  myTFT_WriteCmdData(0x33, vsp - top);  //SST

}


void myTFT_setRotation(uint8_t r)
{
 
		uint16_t GS=0, SS=0;// = _lcd_rev;
	u8 val;
	
    myTFT._width = ((r&3) & 1) ? myTFT.HEIGHT : myTFT.WIDTH;
    myTFT._height = ((r&3) & 1) ? myTFT.WIDTH : myTFT.HEIGHT;
/*    
	switch (r) {
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
*/
//  _SC = 0x37, _EC = 0x36, _SP = 0x39, _EP = 0x38;
//  _MC = 0x20, _MP = 0x21, _MW = 0x22;
  GS = (val & 0x80) ? (1 << 9) : 0;
  SS = (val & 0x40) ? (1 << 8) : 0;
  myTFT_WriteCmdData(0x01, 0x0428);       // set Driver Output Control

//  ORG = (val & 0x20) ? (1 << 3) : 0;


  myTFT_setAddrWindow(0, 0, myTFT._width - 1, myTFT._height - 1);
  myTFT_vertScroll(0, myTFT.HEIGHT, 0);   //reset scrolling after a rotation
}



void myTFT_drawPixel(s32 x, s32 y, u16 color)
{
	u16 reg;
  // MCUFRIEND just plots at edge if you try to write outside of the box:

//  if (x < 0 || y < 0 || x >= width() || y >= height())
//		return;
	
	myTFT_WriteCmdData(0x20, x);
	myTFT_WriteCmdData(0x21, y);
	myTFT_WriteCmdData(0x22, color);

//	reg = myTFT_readReg(0x22,0);
//	reg = myTFT_readReg(0x22,0);
//	reg = myTFT_readReg(0x22,0);
	
	__nop;

}



static void init_table16(const void *table, int16_t size);
#define TFTLCD_DELAY 0xFFFF
static void init_table16(const void *table, int16_t size)
{
    uint16_t *p = (uint16_t *) table;
    while (size > 0) {
        uint16_t cmd = *p++;
        uint16_t d = *p++;
        if (cmd == TFTLCD_DELAY)
            myTFT_delay(d);
        else {
            myTFT_WriteCmd(cmd);
            myTFT_WriteData(d);
        }
        size -= 2 * sizeof(int16_t);
    }
}

static const uint16_t S6D0154_regValues[] = {




            0x0001, 0x0128,
            0x0002, 0x0100,
            0x0003, 0x1030,			//ID[1.0]="11", AM=”0”
//            0x0007, 0x0012,     //GON=1, REV=0, D=2
//            TFTLCD_DELAY, 40,
            0x0007, 0x0013,     //GON=1, REV=0, D=3
//            0x0007, 0x0017,     //GON=1, REV=1, D=3 DISPLAY ON 
						0x0008, 0x0303,
						0x000C, 0x0000, 
            0x0010, 0x0800,     //SAP=8
            TFTLCD_DELAY, 10,
/*
					  0x0011, 0x011A,     //APON=0, PON=1, AON=0, VCI1_EN=1, VC=10
            TFTLCD_DELAY, 10,
            0x0011, 0x031A,     //APON=0, PON=3, AON=0, VCI1_EN=1, VC=10
            TFTLCD_DELAY, 10,
            0x0011, 0x071A,     //APON=0, PON=7, AON=0, VCI1_EN=1, VC=10
            TFTLCD_DELAY, 10,
            0x0011, 0x0F1A,     //APON=0, PON=15, AON=0, VCI1_EN=1, VC=10
            TFTLCD_DELAY, 10,
*/
						0x0011, 0x0F3A,     //APON=0, PON=15, AON=1, VCI1_EN=1, VC=10 
            TFTLCD_DELAY, 30,						
//            0x0011, 0x001A,
            0x0012, 0x3121,     //BT=3, DC1=1, DC2=2, DC3=1
            0x0013, 0x006C,     //GVD=108
            0x0014, 0x4249,     //VCM=66, VML=73						

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
        };
#define DELAY_50us	300
#define DELAY_100us	600

#define TFT_CONFIG_SIZE	0x83
u16 tftConfig[TFT_CONFIG_SIZE];
void myTFT_getConfig(u16 *ptftConfig)
{
	u8 cnt;
	for (cnt=0; cnt<TFT_CONFIG_SIZE; cnt++){
		*ptftConfig++ = myTFT_readReg(cnt, 0);
	
	}	
}
void myTFT_init(void)
{
	u16 id;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_DeInit(myTFT_PORT);

	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC, ENABLE );
	
	// Configure LCD Back Light (PA8) as output push-pull 
	GPIO_InitStructure.GPIO_Pin = myTFT_PORT_MASK;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	
	GPIO_Init( myTFT_PORT, &GPIO_InitStructure );

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(RESET_PORT, &GPIO_InitStructure );
	
	
//	PORT_SET_READ;
	done_reset = 1;
	PORT_SET_WRITE;
//	CTL_INIT;
	CS_IDLE;
	RD_IDLE;
	WR_IDLE;
	RESET_IDLE;
	myTFT_delay(DELAY_50us);
	RESET_ACTIVE;
	myTFT_delay(DELAY_100us);
	RESET_IDLE;
	myTFT_delay(DELAY_100us);

	id = myTFT_readReg(0x00, 0);
	init_table16(S6D0154_regValues, sizeof(S6D0154_regValues));
	
	Adafruit_GFX_Init(&myTFT);
	
	myTFT_setRotation(0);

	myTFT_getConfig(tftConfig);
	myTFT_getConfig(tftConfig);
	myTFT_drawPixel(100,100,0);
	myTFT_drawPixel(101,100,0);
	myTFT_drawPixel(100,101,0);
	myTFT_drawPixel(101,101,0);
	
}
