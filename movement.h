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
#include "controller.h"


/* Exported constants --------------------------------------------------------*/
#define AXIS_NUM		3

#if AXIS_NUM > 8
#error "Axis number must be lower than 8"
#endif

#define COORD_BUFFER_S	/*(RAMP_STEPS+20)*/2000
#define GMOVE_BUFFER_S	20 //10 command - buffer

#define MICROSTEP_SIZE 	125

#define RAMP_MIN		700
//#define RAMP_MAX		180000
#define RAMP_MAX		100000//64000 //200k max for drivers
#define RAMP_ACCEL		20
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

#define DIR_X_INV		0
#define DIR_Y_INV		0
#define DIR_Z_INV		0
#define DIR_A_INV		0

#define X_1MM			2560
#define Y_1MM			2560//12800

#define X_AXIS_MULT		1
#define Y_AXIS_MULT		5
#define Z_AXIS_MULT		1
#define A_AXIS_MULT		1


/* Exported types ------------------------------------------------------------*/
typedef struct{
	int32_t step[AXIS_NUM];
}CoordTypeDef;


typedef struct{
	uint32_t step[AXIS_NUM];
}CoordMoveTypeDef;


typedef struct{
	uint64_t comTotalSteps;			//total number of Global steps
	uint64_t comRealTotalSteps;		//total number of Global steps + repetition steps

	//uint64_t SqComTotalSteps;		//(total number of Global steps)^2
	///uint64_t AxisSteps[AXIS_NUM];	//(Delta)^2 ??? really???

	uint64_t comCurrentStep;		//current Global step
	uint64_t comRealCurrentStep;	//current Global step + repetition steps
	CoordTypeDef GCommandCurrPos;	//current axes position

	CoordTypeDef GCommandEndPos;	//relative axes end position in steps

//	uint64_t rollupCalcValues;
//	uint64_t rollupFinishedValues;
//	uint64_t calcAvail[AXIS_NUM];

	CoordTypeDef Delta;				//interval in steps of axes movement

	int32_t incDecAxis[AXIS_NUM];	//select axes direction
	uint16_t flagAxisNotCompleted;	//set of completion flags
	int16_t Axes;					//enabled axes
	uint8_t doNotStop;				//don't deaccelerate at end of treactory;

	//uint64_t numOfCalcs[AXIS_NUM];

	//uint64_t SqKCoef[AXIS_NUM];
	uint64_t SqRCoef[AXIS_NUM];
	//uint64_t SqBCoef[AXIS_NUM];

	uint64_t SyncCounter;
}GCommandMoveTypeDef;


typedef struct{
	float pos[AXIS_NUM];
}PositionTypeDef;


typedef uint64_t (*CalcPtr)(uint8_t, GCodeTypeDef *);
typedef void (*preCalcPtr)(uint8_t, GCodeTypeDef *);


/* Exported macro ------------------------------------------------------------*/
#define BitSet(p,m) ((p) |= (1<<(m)))
#define BitReset(p,m) ((p) &= ~(1<<(m)))
#define BitFlip(p,m) ((p) ^= (m))
#define BitWrite(c,p,m) ((c) ? BitSet(p,m) : BitReset(p,m))
#define BitIsSet(reg, bit) (((reg) & (1<<(bit))) != 0)
#define BitIsReset(reg, bit) (((reg) & (1<<(bit))) == 0)


/* Exported define -----------------------------------------------------------*/
#define PORT_DIR 	GPIOB



#define PIN_X_DIR	GPIO_Pin_6
#define PORT_X_DIR	GPIOB
#define PIN_X_STP	GPIO_Pin_6
#define PORT_X_STP	GPIOB
#define PIN_X_ENA	GPIO_Pin_6
#define PORT_X_ENA	GPIOB

#define PIN_Y_DIR	GPIO_Pin_6
#define PORT_Y_DIR	GPIOB
#define PIN_Y_STP	GPIO_Pin_6
#define PORT_Y_STP	GPIOB
#define PIN_Y_ENA	GPIO_Pin_6
#define PORT_Y_ENA	GPIOB

#define PIN_Z_DIR	GPIO_Pin_6
#define PORT_Z_DIR	GPIOB
#define PIN_Z_STP	GPIO_Pin_6
#define PORT_Z_STP	GPIOB
#define PIN_Z_ENA	GPIO_Pin_6
#define PORT_Z_ENA	GPIOB

#define PIN_A_DIR	GPIO_Pin_6
#define PORT_A_DIR	GPIOB
#define PIN_A_STP	GPIO_Pin_6
#define PORT_A_STP	GPIOB
#define PIN_A_ENA	GPIO_Pin_6
#define PORT_A_ENA	GPIOB




/* Exported functions ------------------------------------------------------- */
void vMoveTask(void *pvParameters);

PositionTypeDef GetCurrentPositionMM();
PositionTypeDef GetCurrentPositionUM();


#endif //_MOVEMENT_H
