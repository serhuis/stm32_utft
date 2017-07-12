//ui.c
#include "stm32f10x.h"
#include "GPIO_STM32F10x.h"
#include "FreeRTOS.h"
#include "ui.h"
#include "timers.h"


#define LED_PORT		GPIOA
#define LED_PIN			GPIO_Pin_1
#define LedOn()			LED_PORT->BRR = GPIO_Pin_1
#define LedOff()		LED_PORT->BSRR = GPIO_Pin_1
#define LedToggle()	LED_PORT->ODR ^= GPIO_Pin_1


void KeyScan( TimerHandle_t xTimer );
	
static void prvConfigLed(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOA, ENABLE );
	
	// Configure LCD Back Light (PA8) as output push-pull 
	GPIO_InitStructure.GPIO_Pin = LED_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init( LED_PORT, &GPIO_InitStructure );

	/* Set the Backlight Pin */
	GPIO_WriteBit(LED_PORT, LED_PIN, Bit_RESET);
	
}


#define KEYS_PORT		GPIOB
static void prvConfigKeyboard(void)
{
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
		
	
	
	/* Set the Backlight Pin */

}


/*-----------------------------------------------------------*/

static void prvConfigureLCD( void )
{
/*	
GPIO_InitTypeDef GPIO_InitStructure;

	// Configure LCD Back Light (PA8) as output push-pull 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init( GPIOA, &GPIO_InitStructure );

	// Set the Backlight Pin 
	GPIO_WriteBit(GPIOA, GPIO_Pin_8, Bit_SET);

	// Initialize the LCD 
	LCD_Init();

	// Set the Back Color 
	LCD_SetBackColor( White );

	// Set the Text Color 
	LCD_SetTextColor( 0x051F );

	LCD_Clear();
	*/
}



void vKeysScankTask( void *pvParameters )
{
	TimerHandle_t tmrKeyScan;

//	xLastExecutionTime = xTaskGetTickCount();
	prvConfigKeyboard();
	
	tmrKeyScan = xTimerCreate("timerKeyScan", pdMS_TO_TICKS(100), pdTRUE, ( void * ) 0, KeyScan);
	xTimerStart(tmrKeyScan, pdMS_TO_TICKS(100));
	for( ;; )
	{

	}

}
/*-----------------------------------------------------------*/

void vLCDTask( void *pvParameters )
{
	
//	xLCDMessage xMessage;

	/* Initialise the LCD and display a startup message. */
	prvConfigLed();
	prvConfigureLCD();
//	LCD_DrawMonoPict( ( unsigned long * ) pcBitmap );


	for( ;; )
	{


	}
}
#define KEY_ROWS						4
#define KEY_COLUMNS					4
#define	KEY_OUT_MASK				GPIO_Pin_0|GPIO_Pin_2|GPIO_Pin_4|GPIO_Pin_6
#define	KEY_IN_MASK					(GPIO_Pin_8|GPIO_Pin_10|GPIO_Pin_12|GPIO_Pin_14) >> 8

#define KEY_CODE_1		  0x01
#define KEY_CODE_2		  0x04
#define KEY_CODE_3		  0x10
#define KEY_CODE_4		  0x01
#define KEY_CODE_5		  0x04
#define KEY_CODE_6		  0x10
#define KEY_CODE_7		  0x01
#define KEY_CODE_8		  0x04
#define KEY_CODE_9		  0x10
#define KEY_CODE_0		  0x04
#define KEY_CODE_A		  0x40
#define KEY_CODE_B		  0x40
#define KEY_CODE_C			0x40
#define KEY_CODE_D			0x40
#define KEY_CODE_STAR		0x01
#define KEY_CODE_HASH		0x10



u16 KeyOutPins[] = {GPIO_Pin_0, GPIO_Pin_2, GPIO_Pin_4, GPIO_Pin_6};
u8 KeysPressed[KEY_ROWS] = {0};

void KeyScan( TimerHandle_t xTimer )
{
	u8 r;
	LedToggle();
	for(r = 0; r < KEY_ROWS; ++r)
	{
		KEYS_PORT->BRR  |= GPIO_Pin_0|GPIO_Pin_2|GPIO_Pin_4|GPIO_Pin_6;
		KEYS_PORT->BSRR |= KeyOutPins[r];
		
		KeysPressed[r] = (KEYS_PORT->IDR >> 8) & 0xFF;// & 0xFF00;
		KeysPressed[r] &= KEY_IN_MASK;
	}
	
	
	
}

