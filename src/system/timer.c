#include "timer.h"
#include "FreeRTOS.h"
#include "radio.h"
#include "rtc.h"
#include "task.h"
#include <stdint.h>

// clang-format off
// sub-second number of bits
#define N_PREDIV_S 									10

// Synchronous prediv
#define PREDIV_S 									( ( 1 << N_PREDIV_S ) - 1 )

// Asynchronous prediv
#define PREDIV_A 									( 1 << ( 15 - N_PREDIV_S ) ) - 1

// RTC Time base in us
#define USEC_NUMBER                                 1000000
#define MSEC_NUMBER                                 ( USEC_NUMBER / 1000 )

#define COMMON_FACTOR 3
#define CONV_NUMER 									( MSEC_NUMBER >> COMMON_FACTOR )
#define CONV_DENOM                                  ( 1 << ( N_PREDIV_S - COMMON_FACTOR ) )
// clang-format on

/* Function Declarations ------------------------------------*/
void vTimerTaskFunction( void *pvParams );
void vTimerDone( TimerEvent_t *obj );

/* Private Variables ----------------------------------------*/

static TimerEvent_t *xGlobalHandle;

STATIC_TASK_STRUCTURES( TimerTask, configMINIMAL_STACK_SIZE, tskIDLE_PRIORITY );
STATIC_SEMAPHORE_STRUCTURES( xRadioInteruptHandle );

/* Functions ----------------------------------------------- */
void vTimerTaskInit( void )
{
	STATIC_TASK_CREATE( TimerTask, vTimerTaskFunction, "Timer Task", NULL );
}

void TimerInit( TimerEvent_t *obj, void ( *fnCallback )( void *pvContext ) )
{
	obj->xTimerCallback = fnCallback;
	obj->pvContext		= NULL;
	obj->ulTicksExpiry  = 0;
	STATIC_SEMAPHORE_CREATE_BINARY( xRadioInteruptHandle );
}

void TimerSetContext( TimerEvent_t *obj, void *context )
{
	obj->pvContext = context;
}

void TimerSetValue( TimerEvent_t *obj, uint32_t value )
{

	obj->ulTicksExpiry = ulRtcMsToTick( value );
}

void TimerStart( TimerEvent_t *obj )
{
	obj->pxAlarmCallback = (void *) vTimerDone;
	obj->xHandle		 = xRtcAlarmSetup( obj->ulTicksExpiry, obj );
}

void TimerStop( TimerEvent_t *obj )
{
	UNUSED( obj );
	//vRtxAlarmStop( obj->xHandle );
}

TimerTime_t TimerGetCurrentTime( void )
{
	uint64_t now   = ullRtcTickCount();
	uint32_t nowMs = ulRtcTicksToMs( now );
	return nowMs;
}

TimerTime_t TimerGetElapsedTime( TimerTime_t past )
{
	if ( past == 0 ) {
		return 0;
	}
	uint32_t nowInTicks  = ullRtcTickCount();
	uint32_t pastInTicks = ulRtcMsToTick( past );

	// Intentional wrap around. Works Ok if tick duration below 1ms
	return ulRtcTicksToMs( nowInTicks - pastInTicks );
}

void vTimerDone( TimerEvent_t *obj )
{
	xGlobalHandle						= obj;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR( xRadioInteruptHandle, &xHigherPriorityTaskWoken );
}

void vTimerTaskFunction( void *pvParams )
{
	UNUSED( pvParams );

	while ( 1 ) {
		xSemaphoreTake( xRadioInteruptHandle, portMAX_DELAY );
		if ( xGlobalHandle->xTimerCallback != NULL ) {
			xGlobalHandle->xTimerCallback( xGlobalHandle->pvContext );
		}
	}
}
