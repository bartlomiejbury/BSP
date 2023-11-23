#pragma once

#include <stdint.h>

typedef struct ITimer ITimer;
typedef struct MainLoopTask MainLoopTask;

struct ITimer {

	bool started;
	bool oneShot;
	uint32_t startTime;
	uint32_t period;
	void (*callback)(void *data);
	void *data;
};

void TIMER_Start(ITimer *timer);

void OS_Init();
ITimer *OS_CreateOneShotTimer(uint32_t periodMs, void (*callback)(void *data), void *data);
uint32_t OS_GetTime();

void OS_Process();
void OS_ProcessTimers();
void OS_ProcessTasks();
