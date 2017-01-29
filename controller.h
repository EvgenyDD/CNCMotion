/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _CONTROLLER_H
#define _CONTROLLER_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "croutine.h"
#include "portmacro.h"
#include "task.h"
#include "timers.h"
#include "queue.h"

/* Exported types ------------------------------------------------------------*/
typedef struct{
	uint8_t code;

	int32_t X;
	int32_t Y;
	int32_t Z;
	int32_t A;

	uint16_t feedRate;

	int32_t I;
	int32_t J;
	int32_t K;

	int32_t R;

	uint8_t dontStop;
}GCodeTypeDef;


/* Exported constants --------------------------------------------------------*/
#define GCODE_QUEUE_SIZE	(10)


/* Exported macro ------------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void vControl(void *pvParameters);

#endif //_CONTROLLER_H
