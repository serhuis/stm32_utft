//ui.c
#include "stm32f10x.h"
#include "GPIO_STM32F10x.h"
#include "FreeRTOS.h"
#include "ui.h"
#include "timers.h"
#include "queue.h"
#include "tft_lcd.h"

#define LED_PORT		GPIOA
#define LED_PIN			GPIO_Pin_1
//#define LedOn()			LED_PORT->BRR = GPIO_Pin_1
//#define LedOff()		LED_PORT->BSRR = GPIO_Pin_1
//#define LedToggle()	LED_PORT->ODR ^= GPIO_Pin_1

//functions for key actions
void LedOn(void* param)
{
	LED_PORT->BRR = GPIO_Pin_1;
}
void LedOff(void* pram)
{
	LED_PORT->BSRR = GPIO_Pin_1;
}
void LedToggle(void* param)
{
	LED_PORT->ODR ^= GPIO_Pin_1;
}
void NopFunc(void* param)
{
	;
}

extern TaskHandle_t xKeyTaskHandle;
static void KeyScan( TimerHandle_t xTimer );

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
	LedOff(NULL);
	
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
}


/*-----------------------------------------------------------*/

void prvConfigureLCD( void )
{
	tft_init();
}




#define KEY_ROWS						4
#define KEY_COLUMNS					4
#define KEY_NUM							KEY_ROWS*KEY_COLUMNS
#define	KEY_OUT_MASK				GPIO_Pin_0|GPIO_Pin_2|GPIO_Pin_4|GPIO_Pin_6
#define	KEY_IN_MASK					(GPIO_Pin_8|GPIO_Pin_10|GPIO_Pin_12|GPIO_Pin_14) >> 8
#define KEY_SCAN_PERIOD			10			//ms
#define KEY_SHORT_PRESSED		2				// * KEY_SCAN_PERIOD
#define KEY_LONG_PRESSED		500				// * KEY_SCAN_PERIOD

u16 KeyOutPins[] = {GPIO_Pin_0, GPIO_Pin_2, GPIO_Pin_4, GPIO_Pin_6};
u8 KeysPressed[KEY_ROWS] = {0};
key_t Key[KEY_NUM] = {KeyUndefined};
KeyFunc_t		keyShortAction[KEY_NUM] = {LedOn, LedOff, LedOff, NopFunc,NopFunc,NopFunc,NopFunc,NopFunc,NopFunc,NopFunc,NopFunc,NopFunc,NopFunc,NopFunc,NopFunc,NopFunc};
KeyFunc_t		keyLongAction[KEY_NUM] = 	{LedOff, LedOff, LedOff, NopFunc,NopFunc,NopFunc,NopFunc,NopFunc,NopFunc,NopFunc,NopFunc,NopFunc,NopFunc,NopFunc,NopFunc,NopFunc};
KeyFunc_t		keyUpAction[KEY_NUM] = 		{LedToggle, LedOff, LedToggle, NopFunc,NopFunc,NopFunc,NopFunc,NopFunc,NopFunc,NopFunc,NopFunc,NopFunc,NopFunc,NopFunc,NopFunc,NopFunc};


static void KeyScan( TimerHandle_t xTimer )
{
	u8 r, c;

	for(r = 0; r < KEY_ROWS; ++r)
	{
		KEYS_PORT->BRR  |= GPIO_Pin_0|GPIO_Pin_2|GPIO_Pin_4|GPIO_Pin_6;
		KEYS_PORT->BSRR |= KeyOutPins[r];
		
		KeysPressed[r] = (KEYS_PORT->IDR >> 8) & 0xFF;// & 0xFF00;
		KeysPressed[r] &= KEY_IN_MASK;
		
		for(c=0; c < KEY_COLUMNS; ++c)
		{
			if((1<<(c<<1)) & KeysPressed[r])
			{
				if(KeyDown == Key[(r<<2) + (c)].Status )
					Key[(r<<2) + (c)].Timer++;
				else
				{
					Key[(r<<2) + (c)].Status = KeyDown;
					Key[(r<<2) + (c)].Timer = 0;
				}
			}
			else
				if(KeyDown == Key[(r<<2) + (c)].Status)
					Key[(r<<2) + (c)].Status = KeyUp;
		}
	}
	vTaskResume(xKeyTaskHandle);
}

volatile QueueHandle_t xKeyQueue;

/*----------------- TASKS -----------------------------------*/
void vKeysTask( void *pvParameters )
{
	u8 k;
	TimerHandle_t tmrKeyScan;

	prvConfigKeyboard();

	tmrKeyScan = xTimerCreate("timerKeyScan", pdMS_TO_TICKS(KEY_SCAN_PERIOD), pdTRUE, ( void * ) 0, KeyScan);
	xTimerStart(tmrKeyScan, pdMS_TO_TICKS(KEY_SCAN_PERIOD));

	for( ;; )
	{
		for(k=0; k < KEY_NUM ; ++k)
		{
			if((KeyDown == Key[k].Status) && (KEY_SHORT_PRESSED < Key[k].Timer)){
				keyShortAction[k](NULL);
			}				
			
			if((KeyDown == Key[k].Status) && (KEY_LONG_PRESSED < Key[k].Timer)){
				keyLongAction[k](NULL);
			}					
			
			if((KeyUp == Key[k].Status) && (0 != Key[k].Timer)){
				keyUpAction[k](NULL);
				Key[k].Status = KeyUndefined;
			}
		}
		vTaskSuspend( NULL );
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
