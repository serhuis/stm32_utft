//ui.c
#include "stm32f10x.h"
#include "ui.h"


static void prvConfigKeyboard(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |RCC_APB2Periph_GPIOC
							| RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO, ENABLE );
	
	// Configure LCD Back Light (PA8) as output push-pull 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init( GPIOA, &GPIO_InitStructure );

	/* Set the Backlight Pin 
	GPIO_WriteBit(GPIOA, GPIO_Pin_8, Bit_SET);
	*/
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

	/* Set the Backlight Pin 
	GPIO_WriteBit(GPIOA, GPIO_Pin_8, Bit_SET);

	/* Initialize the LCD 
	LCD_Init();

	/* Set the Back Color 
	LCD_SetBackColor( White );

	/* Set the Text Color 
	LCD_SetTextColor( 0x051F );

	LCD_Clear();
	*/
}



void vKeysScankTask( void *pvParameters )
{
//	xLastExecutionTime = xTaskGetTickCount();
	prvConfigKeyboard();
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
