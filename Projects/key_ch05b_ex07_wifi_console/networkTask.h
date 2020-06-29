#pragma once

#include "FreeRTOS.h"
#include "queue.h"

extern QueueHandle_t networkQueue;

typedef enum {
	net_scan,
	net_connect,
	net_disconnect,
	net_printip,
	net_printmac,
} networkCmd_t;

typedef struct {
	networkCmd_t cmd;
	uint32_t val0;
	uint32_t val1;
} networkQueueMsg_t;

void networkTask(void *arg);
