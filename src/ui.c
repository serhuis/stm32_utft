//ui.c
#include "stm32f10x.h"
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
	
	KEYS_PORT->BRR |= GPIO_Pin_8|GPIO_Pin_10|GPIO_Pin_12|GPIO_Pin_14;
	
	
	
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

/*
		// Perform this check every mainCHECK_DELAY milliseconds. 
		vTaskDelayUntil( &xLastExecutionTime, mainCHECK_DELAY );

		// Has an error been found in any task? 

    if( xAreBlockingQueuesStillRunning() != pdTRUE )
		{
			xMessage.pcMessage = "ERROR IN BLOCK Q\n";
		}
		else if( xAreBlockTimeTestTasksStillRunning() != pdTRUE )
		{
			xMessage.pcMessage = "ERROR IN BLOCK TIME\n";
		}
    else if( xAreSemaphoreTasksStillRunning() != pdTRUE )
    {
        xMessage.pcMessage = "ERROR IN SEMAPHORE\n";
    }
    else if( xArePollingQueuesStillRunning() != pdTRUE )
    {
        xMessage.pcMessage = "ERROR IN POLL Q\n";
    }
    else if( xIsCreateTaskStillRunning() != pdTRUE )
    {
        xMessage.pcMessage = "ERROR IN CREATE\n";
    }
    else if( xAreIntegerMathsTaskStillRunning() != pdTRUE )
    {
        xMessage.pcMessage = "ERROR IN MATH\n";
    }
		else if( xAreComTestTasksStillRunning() != pdTRUE )
		{
			xMessage.pcMessage = "ERROR IN COM TEST\n";
		}
		else
		{
			sprintf( ( char * ) cPassMessage, "PASS [%uns]\n", ( ( unsigned long ) usMaxJitter ) * mainNS_PER_CLOCK );
		}

		// Send the message to the LCD gatekeeper for display. 
		xQueueSend( xLCDQueue, &xMessage, portMAX_DELAY );
*/
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

/*
		// Perform this check every mainCHECK_DELAY milliseconds. 
		vTaskDelayUntil( &xLastExecutionTime, mainCHECK_DELAY );

		// Has an error been found in any task? 

    if( xAreBlockingQueuesStillRunning() != pdTRUE )
		{
			xMessage.pcMessage = "ERROR IN BLOCK Q\n";
		}
		else if( xAreBlockTimeTestTasksStillRunning() != pdTRUE )
		{
			xMessage.pcMessage = "ERROR IN BLOCK TIME\n";
		}
    else if( xAreSemaphoreTasksStillRunning() != pdTRUE )
    {
        xMessage.pcMessage = "ERROR IN SEMAPHORE\n";
    }
    else if( xArePollingQueuesStillRunning() != pdTRUE )
    {
        xMessage.pcMessage = "ERROR IN POLL Q\n";
    }
    else if( xIsCreateTaskStillRunning() != pdTRUE )
    {
        xMessage.pcMessage = "ERROR IN CREATE\n";
    }
    else if( xAreIntegerMathsTaskStillRunning() != pdTRUE )
    {
        xMessage.pcMessage = "ERROR IN MATH\n";
    }
		else if( xAreComTestTasksStillRunning() != pdTRUE )
		{
			xMessage.pcMessage = "ERROR IN COM TEST\n";
		}
		else
		{
			sprintf( ( char * ) cPassMessage, "PASS [%uns]\n", ( ( unsigned long ) usMaxJitter ) * mainNS_PER_CLOCK );
		}

		// Send the message to the LCD gatekeeper for display. 
		xQueueSend( xLCDQueue, &xMessage, portMAX_DELAY );
*/
	}
}
#define KEY_ROWS						4
#define KEY_COLUMNS					4
#define	KEY_OUT_MASK				GPIO_Pin_0|GPIO_Pin_2|GPIO_Pin_4|GPIO_Pin_6
#define	KEY_IN_MASK					GPIO_Pin_8|GPIO_Pin_10|GPIO_Pin_12|GPIO_Pin_14

u16 KeyOutPins[] = {GPIO_Pin_0, GPIO_Pin_2, GPIO_Pin_4, GPIO_Pin_6};
u16 KeysPressed[KEY_ROWS] = {0};

void KeyScan( TimerHandle_t xTimer )
{
	u8 r;
	LedToggle();
	for(r = 0; r < KEY_ROWS; ++r)
	{
		KEYS_PORT->BRR  |= GPIO_Pin_0|GPIO_Pin_2|GPIO_Pin_4|GPIO_Pin_6;
		KEYS_PORT->BSRR |= KeyOutPins[r];
		
		KeysPressed[r] = KEYS_PORT->IDR & KEY_IN_MASK;
	}
}
