#include "FreeRTOS.h"
#include "task.h"
#include "cyhal.h"
#include "cybsp.h"

static inline uint32_t msToTicks(uint32_t ms)
{
	uint64_t val = (CY_SYSCLK_WCO_FREQ*(uint64_t)ms/1000);
	val = (val>UINT32_MAX)?UINT32_MAX:val;
	return (uint32_t)val;
}

static inline uint32_t ticksToMs(uint32_t ticks)
{
	return (ticks * 1000) / CY_SYSCLK_WCO_FREQ;
}

/* Define the function that is called by portSUPPRESS_TICKS_AND_SLEEP(). */
void vApplicationSleep( TickType_t xExpectedIdleTime )
{
	static cyhal_lptimer_t myTimer={0};
	unsigned long ulLowPowerTimeBeforeSleep, ulLowPowerTimeAfterSleep;

	if(myTimer.base == 0)
		cyhal_lptimer_init(&myTimer);

    Cy_SysTick_Disable();
    uint8_t interruptState = Cy_SysLib_EnterCriticalSection();

    /* Ensure it is still ok to enter the sleep mode. */
    eSleepModeStatus eSleepStatus = eTaskConfirmSleepModeStatus();
	cyhal_lptimer_reload(&myTimer);

    if( eSleepStatus != eAbortSleep )
    {
    	if( eSleepStatus != eNoTasksWaitingTimeout )
    	{
    		cyhal_lptimer_set_delay	(&myTimer,msToTicks(xExpectedIdleTime));
    	    cyhal_lptimer_enable_event (&myTimer, CYHAL_LPTIMER_COMPARE_MATCH, 7, true);
    	}
    	/* Enter the low power state. */
        ulLowPowerTimeBeforeSleep = cyhal_lptimer_read(&myTimer);
#if CY_CFG_PWR_SYS_IDLE_MODE == CY_CFG_PWR_MODE_DEEPSLEEP
    	cyhal_system_deepsleep();
#elif CY_CFG_PWR_SYS_IDLE_MODE == CY_CFG_PWR_MODE_SLEEP
    	cyhal_system_sleep();
#else
    	goto exitPoint;
#endif
    	// How long did it sleep
    	ulLowPowerTimeAfterSleep = cyhal_lptimer_read(&myTimer);

    	/* Correct the kernels tick count to account for the time the microcontroller spent in its low power state. */
    	vTaskStepTick( ticksToMs(ulLowPowerTimeAfterSleep - ulLowPowerTimeBeforeSleep));
    }
    cyhal_lptimer_enable_event (&myTimer, CYHAL_LPTIMER_COMPARE_MATCH, 4, false);

#if !(CY_CFG_PWR_SYS_IDLE_MODE == CY_CFG_PWR_MODE_DEEPSLEEP || CY_CFG_PWR_SYS_IDLE_MODE == CY_CFG_PWR_MODE_SLEEP)
exitPoint:
#endif
    Cy_SysLib_ExitCriticalSection(interruptState);
    Cy_SysTick_Enable();
}
