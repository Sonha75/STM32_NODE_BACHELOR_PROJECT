#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include <time.h>

#include "main.h"
#include "sx1278.h"
#include "log.h"


extern UART_HandleTypeDef huart2;


void LOG(const char *TAG, char *data)
{
	char data_log[100] = {0};
	sprintf(data_log, "%s: %s\n", TAG, data);
	HAL_UART_Transmit(&huart2, (uint8_t*)data_log, strlen(data_log), 100);
}
