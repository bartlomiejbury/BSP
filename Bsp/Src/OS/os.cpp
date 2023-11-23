#include <stddef.h>
#include <stdlib.h>
#include "main.h"
#include "os.h"
#include "trace.h"

void OS_Init() {
    LOG_INFO("OS init");
}

ITimer *timer;


void TIMER_Start(ITimer *timer) {
	timer->started = true;
	timer->startTime = OS_GetTime();
}

static bool isTimeout(ITimer *timer, uint32_t currentTime) {
    uint32_t elapsed = 0;
    if (currentTime >= timer->startTime) {
        elapsed = currentTime - timer->startTime;
    } else {
        elapsed = UINT32_MAX - timer->startTime + currentTime;
    }
    return elapsed >= timer->period;
}


static void TIMER_Process(ITimer *timer) {
	uint32_t currentTime = OS_GetTime();
    if (timer->started && isTimeout(timer, currentTime)) {
        if (timer->oneShot) {
            timer->started = false;
        } else {
            timer->startTime = currentTime;
        }
        if (timer->callback) {
            timer->callback(timer->data);
        }
    }
}

void OS_Process() {
	if (timer) {
		TIMER_Process(timer);
	}
}

uint32_t OS_GetTime() {
    return HAL_GetTick();
}

ITimer *OS_CreateOneShotTimer(uint32_t periodMs, void (*callback)(void *data), void *data) {
	timer = (ITimer *)malloc(sizeof(ITimer));

	timer->callback = callback;
	timer->data = data;
	timer->oneShot = true;
	timer->started = false;
	timer->period = periodMs;
	timer->startTime = 0;
	return timer;
}
