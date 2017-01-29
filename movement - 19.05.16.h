/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _MOVEMENT_H
#define _MOVEMENT_H


/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"

#include "stm32f4xx_adc.h"
#include "stm32f4xx_flash.h"
#include "stm32f4xx_fsmc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_iwdg.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_pwr.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_rng.h"
#include "stm32f4xx_sdio.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_usart.h"
#include "misc.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "croutine.h"
#include "portmacro.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"


/* Exported constants --------------------------------------------------------*/
#define AXIS_NUM		3

#if AXIS_NUM > 8
#error "Axis number must be lower than 8"
#endif

#define COORD_BUFFER_S	/*(RAMP_STEPS+20)*/2000
#define GMOVE_BUFFER_S	4 //10 command - buffer

#define MICROSTEP_SIZE 	125

#define RAMP_MIN		100
//#define RAMP_MAX		180000
#define RAMP_MAX		64000 //200k max for drivers
#define RAMP_ACCEL		8
#define RAMP_STEPS		(((RAMP_MAX)-(RAMP_MIN))/(RAMP_ACCEL))

#define STEPS_REV		(200*MICROSTEP_SIZE)		//steps/rev
#define SCREW_PITCH_MM	(5) 						//mm/rev
#define STEPS_MM		(STEPS_REV/SCREW_PITCH_MM)
#define MM_STEPS		(SCREW_PITCH_MM/STEPS_REV)

#define SCREW_PITCH_uM	(SCREW_PITCH_MM*1000) 		//um/rev
#define STEPS_uM		(STEPS_REV/SCREW_PITCH_uM)
#define uM_STEPS		(SCREW_PITCH_uM/STEPS_REV)

#define XAxis			0
#define YAxis 			1
#define ZAxis			2
#define AAxis 			3


/* Exported types ------------------------------------------------------------*/
typedef struct{
	int32_t step[AXIS_NUM];
}CoordTypeDef;


typedef struct{
	uint32_t square[AXIS_NUM];
}CoordSquareTypeDef;


typedef struct{
	uint64_t comTotalStep;			//total number of Global steps
	CoordTypeDef GCommandStartPos;	//relative axes start position in steps
	CoordTypeDef GCommandEndPos;	//relative axes end position in steps
	uint64_t SqComTotalSteps;		//(total number of Global steps)^2
	uint64_t SqAxisSteps[AXIS_NUM];	//(Delta)^2 ??? really???

	uint64_t comCurrentStep;		//current Global step
	CoordTypeDef GCommandCurrPos;	//current axes position

//	uint64_t rollupCalcValues;
//	uint64_t rollupFinishedValues;

	uint64_t comTotalStep2;			//total number of Global steps
	uint64_t comCurrentStep2;		//current Global step

	uint64_t calcAvail[AXIS_NUM];

	CoordTypeDef Delta;				//interval in steps of axes movement
	int32_t incDecAxis[AXIS_NUM];	//select axes direction
	uint16_t flagAxisNotCompleted;	//set of completion flags
	int16_t Axes;					//enabled axes
	uint8_t doNotStop;				//don't deaccelerate at end of treactory;

	uint64_t numOfCalcs[AXIS_NUM];

	uint64_t SqKCoef[AXIS_NUM];
	uint64_t SqRCoef[AXIS_NUM];
	uint64_t SqBCoef[AXIS_NUM];

	uint64_t SyncCounter;
}GCommandMoveTypeDef;


typedef struct{
	float pos[AXIS_NUM];
}PositionTypeDef;


typedef uint64_t (*CalcPtr)(uint8_t);
typedef void (*preCalcPtr)(uint8_t, GCommandMoveTypeDef*);


/* Exported macro ------------------------------------------------------------*/
#define BitSet(p,m) ((p) |= (1<<(m)))
#define BitReset(p,m) ((p) &= ~(1<<(m)))
#define BitFlip(p,m) ((p) ^= (m))
#define BitWrite(c,p,m) ((c) ? BitSet(p,m) : BitReset(p,m))
#define BitIsSet(reg, bit) (((reg) & (1<<(bit))) != 0)
#define BitIsReset(reg, bit) (((reg) & (1<<(bit))) == 0)


/* Exported define -----------------------------------------------------------*/
#define PORT_DIR 	GPIOB

#define PIN_DIR_X	GPIO_Pin_3
#define PIN_DIR_Y	GPIO_Pin_4
#define PIN_DIR_Z	GPIO_Pin_5
#define PIN_DIR_A	GPIO_Pin_7


/* Exported functions ------------------------------------------------------- */
void vMoveTask(void *pvParameters);

PositionTypeDef GetCurrentPositionMM();
PositionTypeDef GetCurrentPositionUM();


#endif //_MOVEMENT_H
