//ui.h

#ifndef _UI_H_
#define _UI_H_


// key statuses
typedef enum KEY_STAT_T
{
	KeyUndefined = 0,
	KeyDown = 1,
	KeyUp,
	KeyPressed,
	KeyReleased,
}key_stat_t;

//call-back function prototype for key action
typedef void (*KeyFunc_t)(void* param);		

// key type
typedef struct
{
	key_stat_t 	Status;
	u16	Timer;
	KeyFunc_t		CallBackFunc;
}key_t;

static void prvConfigureLCD( void );
void vLCDTask( void *pvParameters );
void vKeysScankTask( void *pvParameters);

#endif
