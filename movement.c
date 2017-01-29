/* Includes ------------------------------------------------------------------*/
#include "movement.h"
#include "controller.h"
#include "plasma.h"
#include "math.h"

#include "ILI9327.h"

extern xSemaphoreHandle xLEDMutex;

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define abs(x) ((x) >= 0 ? (x) : -(x))

#define isSqBuf(xx) 	((xx)<COORD_BUFFER_S)
#define isCoordBuf(xx) 	((xx)<GMOVE_BUFFER_S)
#define isAxis(xx) 		((xx)<AXIS_NUM)


/* Global variables ----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
CoordTypeDef CNCCoordinates;

// Circular buffer steps
volatile uint32_t ptrMoveCurrent[AXIS_NUM], ptrMoveEnd[AXIS_NUM];
CoordMoveTypeDef coordinates[COORD_BUFFER_S]; //buffer containing squares of the axis coordinates

// Circular buffer command params
volatile uint32_t ptrGMoveCurrentCom, ptrGMoveEndCom;
GCommandMoveTypeDef GMoveBuffer[GMOVE_BUFFER_S]; //buffer for variables for each g-command

volatile int32_t MoveSpeedMultiplier = RAMP_MIN;

volatile uint32_t multSpeed = 0;
volatile uint32_t prevSync = 0;


/* Extern variables ----------------------------------------------------------*/
extern xQueueHandle GCodeQueue;

		extern uint8_t fuckBuffer[110];


/* Private function prototypes -----------------------------------------------*/
static void Move_stepGenRising(uint8_t axes);
static void Move_stepGenFalling(uint8_t axes);
static void Move_setDirectionAxes(int32_t axes);
static void Move_enableAxes(uint8_t axes);
static void Move_disableAxes(uint8_t axes);


/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
 * Function Name  : ZeroPosition
 *******************************************************************************/
static void ZeroPosition()
{
	for(uint8_t i = 0; i < AXIS_NUM; i++)
		CNCCoordinates.step[i] = 0;
}

/*******************************************************************************
 * Function Name  : ZeroPosition
 *******************************************************************************/
static void InitMovement()
{
	// Reset pointers of the circular buffer
	for(uint8_t i = 0; i < AXIS_NUM; i++)
		ptrMoveEnd[i] = ptrMoveCurrent[i] = 0;

	ptrGMoveCurrentCom = ptrGMoveEndCom = 0;

#define Target_khz RAMP_MIN

	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOE, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_7 | GPIO_Pin_11 | GPIO_Pin_13
		| GPIO_Pin_14 | GPIO_Pin_15 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	Move_disableAxes(0xFF);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

	TIM_TimeBaseInitTypeDef timerInitStructure;
	timerInitStructure.TIM_Prescaler = 9;
	timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	timerInitStructure.TIM_Period = 4000000 / Target_khz /** 2*/;
	timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	timerInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &timerInitStructure);

	timerInitStructure.TIM_Period = 4000000 / Target_khz / 2;
	TIM_TimeBaseInit(TIM5, &timerInitStructure);
	TIM_SelectOnePulseMode(TIM5, TIM_OPMode_Single);

	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

	NVIC_InitTypeDef nvicStructure;
	nvicStructure.NVIC_IRQChannel = TIM2_IRQn;
	nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
	nvicStructure.NVIC_IRQChannelSubPriority = 1;
	nvicStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvicStructure);

	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
	NVIC_EnableIRQ(TIM5_IRQn);

	TIM_ARRPreloadConfig(TIM2, ENABLE);
	TIM_ARRPreloadConfig(TIM5, ENABLE);
}


/////////////////////////// CALCULATING FUNCTIONS /////////////////////////////
static void preCalcLinear(uint8_t axis, GCodeTypeDef *X){}
static void preCalcEllipse(uint8_t axis, GCodeTypeDef *X){}

/*******************************************************************************
 * Function Name  : preCalcRound
 *******************************************************************************/
static void preCalcRound(uint8_t axis, GCodeTypeDef *X)
{
	GMoveBuffer[ptrGMoveEndCom].SqRCoef[axis] =
		(uint64_t) GMoveBuffer[ptrGMoveEndCom].comTotalSteps *
		(uint64_t) GMoveBuffer[ptrGMoveEndCom].comTotalSteps;

	GMoveBuffer[ptrGMoveEndCom].SqRCoef[axis] =
		(uint64_t) X->R * (uint64_t) X->R;

	if(X->I != 0 || X->J != 0)
	{
		GMoveBuffer[ptrGMoveEndCom].SqRCoef[axis] =
			(uint64_t) X->I * (uint64_t) X->I +
			(uint64_t) X->J * (uint64_t) X->J;

		X->R = (uint64_t) sqrtl(GMoveBuffer[ptrGMoveEndCom].SqRCoef[axis]);
	}
}

/*******************************************************************************
 * Function Name  : CalcLinear
 *******************************************************************************/
static uint64_t CalcLinear(uint8_t axis, GCodeTypeDef *X)
{
	GMoveBuffer[ptrGMoveEndCom].GCommandCurrPos.step[axis]++;

	uint32_t val =
		GMoveBuffer[ptrGMoveEndCom].GCommandCurrPos.step[axis] *
		GMoveBuffer[ptrGMoveEndCom].comTotalSteps
		/ GMoveBuffer[ptrGMoveEndCom].Delta.step[axis];

	return val;
}

/*******************************************************************************
 * Function Name  : CalcRound
 *******************************************************************************/
static uint64_t CalcRound(uint8_t axis, GCodeTypeDef *X)
{
	GMoveBuffer[ptrGMoveEndCom].GCommandCurrPos.step[axis]++;

	uint64_t xx = X->R - GMoveBuffer[ptrGMoveEndCom].GCommandCurrPos.step[axis];
	uint64_t SqCoordDiff = GMoveBuffer[ptrGMoveEndCom].SqRCoef[axis] - (xx * xx);

assert_param(GMoveBuffer[ptrGMoveEndCom].SqRCoef[axis] >= (xx * xx));

	uint64_t val = (uint32_t) sqrtl(SqCoordDiff);

	if(X->code == 2)
	{
		if(axis == XAxis)
			return val;
	}
	else if(X->code == 3)
	{
		if(axis == YAxis)
			return val;
	}

	return GMoveBuffer[ptrGMoveEndCom].GCommandCurrPos.step[axis];
}

///*******************************************************************************
// * Function Name  : CalcEllipse
// *******************************************************************************/
//static uint64_t CalcEllipse(uint8_t axis, GCodeTypeDef *X)
//{
//	return 0;
//}

/*******************************************************************************
 * Function Name  : GetNextEndPtr_Inc
 *******************************************************************************/
static uint32_t GetNextEndPtr_Inc(uint8_t axis)
{
assert_param(isAxis(axis));

	uint32_t value = ptrMoveEnd[axis];

	if(++ptrMoveEnd[axis] >= COORD_BUFFER_S)
		ptrMoveEnd[axis] = 0;

assert_param(isSqBuf(ptrMoveEnd[axis]));

	return value;
}

/*******************************************************************************
 * Function Name  : GetPrevEndPtr
 *******************************************************************************/
static uint32_t GetPrevEndPtr(uint8_t axis)
{
	if(ptrMoveEnd[axis] == 0)
		return COORD_BUFFER_S - 1;
	else
		return ptrMoveEnd[axis] - 1;
}

/*******************************************************************************
 * Function Name  : IncCurrMovePtr
 *******************************************************************************/
static uint32_t IncCurrMovePtr(uint8_t axis)
{
	uint32_t value = ptrMoveCurrent[axis];

assert_param(ptrMoveCurrent[axis] != ptrMoveEnd[axis]);

	if(++ptrMoveCurrent[axis] >= COORD_BUFFER_S)
		ptrMoveCurrent[axis] = 0;

assert_param(isSqBuf(ptrMoveCurrent[axis]));
	//assert_param(ptrMoveCurrent[axis]!=ptrMoveEnd[axis]);

	return value;
}


/*******************************************************************************
 * Function Name  : CalcBufferRemain
 *******************************************************************************/
uint32_t CalcBufferRemain(uint8_t axis)
{
assert_param(isCoordBuf(ptrGMoveEndCom));

	if(BitIsReset(GMoveBuffer[ptrGMoveEndCom].flagAxisNotCompleted, axis))
		return 0xFFFF;

	if(ptrMoveEnd[axis] >= ptrMoveCurrent[axis])
		return ptrMoveEnd[axis] - ptrMoveCurrent[axis];
	else
		return (ptrMoveEnd[axis] + COORD_BUFFER_S - ptrMoveCurrent[axis]);
}


/*******************************************************************************
 * Function Name  : SelectAbsBiggestVal
 *******************************************************************************/
static uint32_t SelectAbsBiggestVal(CoordTypeDef *data)
{
	uint32_t result = 0;

	for(uint8_t i = 0; i < AXIS_NUM; i++)
	{
		if(abs(data->step[i]) > result)
			result = abs(data->step[i]);
	}

	return result;
}


/*******************************************************************************
 * Function Name  : SelectAbsBiggestVal
 *******************************************************************************/
static uint32_t SelectAbsMinValNotZero(CoordTypeDef *data)
{
	uint32_t result = 0xFFFFFFFF;

	for(uint8_t i = 0; i < AXIS_NUM; i++)
	{
		if(abs(data->step[i]) < result && abs(data->step[i]) != 0)
			result = abs(data->step[i]);
	}

	return result;
}





/*******************************************************************************
 * Function Name  : LinearMove
 *******************************************************************************/
static void Move(GCodeTypeDef *GCommand)
{
	static int32_t* ptrGCommand[AXIS_NUM] = { &(GCommand->X), &(GCommand->Y), &(GCommand->Z) /*, &(GCommand->A) */};

	preCalcPtr PreCalcFunc[] = { preCalcLinear, preCalcLinear, //G00 G01
								 preCalcRound, preCalcRound,  //G02 G03
								 preCalcEllipse }; //???

	CalcPtr CalcFunc[] = { CalcLinear, CalcLinear, //G00 G01
						   CalcRound, CalcRound/*,  //G02 G03
						   CalcEllipse*/ }; //???

	//Debug via display
	xSemaphoreTake(xLEDMutex, portMAX_DELAY);
	{
		ILI9327_NumWDesc(0, 275, "X:", GCommand->X);
		ILI9327_NumWDesc(80, 275, "Y:", GCommand->Y);

		ILI9327_NumWDesc(150, 275, "p:", ptrGMoveEndCom);

		static uint32_t stepNum = 0;
		ILI9327_NumWDesc(190, 275, "n:", stepNum++);
	}
	xSemaphoreGive(xLEDMutex);


	//pre-calculate constants for each axis
	for(uint8_t i = 0; i < AXIS_NUM; i++)
	{
assert_param(GCommand->code < 5);

		PreCalcFunc[GCommand->code](i, GCommand);
	}

	//Get target axis counters values
	for(uint8_t i = 0; i < AXIS_NUM; i++)
	{
assert_param(isCoordBuf(ptrGMoveEndCom));

		GMoveBuffer[ptrGMoveEndCom].GCommandCurrPos.step[i] = 0;
		GMoveBuffer[ptrGMoveEndCom].GCommandEndPos.step[i] = ((*ptrGCommand[i]) /** STEPS_REV / SCREW_PITCH_uM*/);

		// If it's first command -> delta of movement is reference by current position
		if(ptrGMoveEndCom == ptrGMoveCurrentCom)
		{
			GMoveBuffer[ptrGMoveEndCom].Delta.step[i] =
				GMoveBuffer[ptrGMoveEndCom].GCommandEndPos.step[i] -
				CNCCoordinates.step[i];
		}
		else
		{
			GMoveBuffer[ptrGMoveEndCom].Delta.step[i] =
				GMoveBuffer[ptrGMoveEndCom].GCommandEndPos.step[i] -
				GMoveBuffer[ptrGMoveEndCom == 0 ? GMOVE_BUFFER_S - 1 : ptrGMoveEndCom - 1].GCommandEndPos.step[i];
		}

		if(GCommand->I != 0 && i == XAxis)
		{
			GMoveBuffer[ptrGMoveEndCom].GCommandCurrPos.step[i] =
				GCommand->R - abs(GCommand->I);
		}

		if(GCommand->J != 0 && i == YAxis)
		{
			GMoveBuffer[ptrGMoveEndCom].GCommandCurrPos.step[i] =
				GCommand->R - abs(GCommand->J);
		}
	}

assert_param(isCoordBuf(ptrGMoveEndCom));

	GMoveBuffer[ptrGMoveEndCom].flagAxisNotCompleted = 0; 	//reset uncompleted flags
	GMoveBuffer[ptrGMoveEndCom].Axes = 0; 					// reset GCode Config

	//Normalize GMoveBuffer[ptrGMoveEndCom].Delta steps
	for(uint8_t i = 0; i < AXIS_NUM; i++)
	{
		if(GMoveBuffer[ptrGMoveEndCom].Delta.step[i] != 0)
		{
assert_param(isCoordBuf(ptrGMoveEndCom));

			BitSet(GMoveBuffer[ptrGMoveEndCom].flagAxisNotCompleted, i);
			BitSet(GMoveBuffer[ptrGMoveEndCom].Axes, i*2);
		}

		if(GMoveBuffer[ptrGMoveEndCom].Delta.step[i] <= 0)
		{
assert_param(isCoordBuf(ptrGMoveEndCom));

			//make Delta positive
			GMoveBuffer[ptrGMoveEndCom].Delta.step[i] = -1 * GMoveBuffer[ptrGMoveEndCom].Delta.step[i];
			GMoveBuffer[ptrGMoveEndCom].incDecAxis[i] = -1;
		}
		else
		{
assert_param(isCoordBuf(ptrGMoveEndCom));

			GMoveBuffer[ptrGMoveEndCom].incDecAxis[i] = 1;
			BitSet(GMoveBuffer[ptrGMoveEndCom].Axes, i*2 + 1);
		}
	}


assert_param(isCoordBuf(ptrGMoveEndCom));

	uint8_t cmdEndCondition = GMoveBuffer[ptrGMoveEndCom].flagAxisNotCompleted;

assert_param(isCoordBuf(ptrGMoveEndCom));

	GMoveBuffer[ptrGMoveEndCom].comTotalSteps =
					GMoveBuffer[ptrGMoveEndCom].comRealTotalSteps =
							SelectAbsBiggestVal(&(GMoveBuffer[ptrGMoveEndCom].Delta));
	GMoveBuffer[ptrGMoveEndCom].SyncCounter = 0;
	GMoveBuffer[ptrGMoveEndCom].doNotStop = 0;

//	if(GMoveBuffer[ptrGMoveEndCom].Axes
//		== GMoveBuffer[ptrGMoveEndCom == 0 ? GMOVE_BUFFER_S - 1 : ptrGMoveEndCom - 1].Axes)
//		GMoveBuffer[ptrGMoveEndCom].doNotStop = 1;

//	if(GCommand->dontStop!=0)
//		GMoveBuffer[ptrGMoveEndCom].doNotStop = 1;
//
//	if(GCommand->code == 0)
//		GMoveBuffer[ptrGMoveEndCom].doNotStop = 1;


	GMoveBuffer[ptrGMoveEndCom].comCurrentStep = GMoveBuffer[ptrGMoveEndCom].comRealCurrentStep = 1; //Initialize step counter

//	if(GCommand->I != 0 || GCommand->J != 0)
//		GMoveBuffer[ptrGMoveEndCom].comCurrentStep
//		= GMoveBuffer[ptrGMoveEndCom].comRealCurrentStep
//		= SelectAbsMinValNotZero(&(GMoveBuffer[ptrGMoveEndCom].GCommandCurrPos));

	TIM_Cmd(TIM2, ENABLE); //enable counter

	// main processor
	while(cmdEndCondition)
	{
		uint16_t penalty = 0;

		for(uint8_t i = 0; i < AXIS_NUM; i++)
		{
assert_param(isCoordBuf(ptrGMoveEndCom));

			if(CalcBufferRemain(i) < 1800 &&
				GMoveBuffer[ptrGMoveEndCom].GCommandCurrPos.step[i] != GMoveBuffer[ptrGMoveEndCom].Delta.step[i]
				/*BitIsSet(flagAxisNotCompleted, i)*/)
			{
				GPIO_SetBits(GPIOB, GPIO_Pin_1);

				uint64_t curr, prev = coordinates[GetPrevEndPtr(i)].step[i];

				coordinates[GetNextEndPtr_Inc(i)].step[i] =
					curr =
						CalcFunc[GCommand->code](i, GCommand);

				uint64_t diff = curr - prev;

				if(diff == 0)
					BitSet(penalty, i);
			}

			if(GMoveBuffer[ptrGMoveEndCom].GCommandCurrPos.step[i] == GMoveBuffer[ptrGMoveEndCom].Delta.step[i])
			{
				BitReset(cmdEndCondition, i);
			}
		}

		if(penalty)
			GMoveBuffer[ptrGMoveEndCom].comRealTotalSteps++;

		GMoveBuffer[ptrGMoveEndCom].SyncCounter++;

GPIO_ResetBits(GPIOB, GPIO_Pin_1);
	}

	GMoveBuffer[ptrGMoveEndCom].SyncCounter = 0;

	// increment HEAD of GCodes Ring buffer
	if(++ptrGMoveEndCom >= GMOVE_BUFFER_S)
	{
		ptrGMoveEndCom = 0;
	}

	// lock if we have too much unprocessed GCodes
	if(ptrGMoveEndCom == ptrGMoveCurrentCom)
	{
		while(ptrGMoveEndCom != ptrGMoveCurrentCom)
			taskYIELD();
	}
}

/*******************************************************************************
 * Function Name  : ProcessGCode
 *******************************************************************************/
static void ProcessGCode(GCodeTypeDef *GCommand)
{
//	switch(GCommand->code)
//	{
//	case 0x00:
//
//		break;
//
//	case 0x01:
//		PlasmaOn();
//		LinearMove(GCommand);
//		PlasmaOff();
//		break;
//
//	case 0x02:
//		break;
//
//	case 0x03:
//		break;
//	}

	Move(GCommand);
}



/*******************************************************************************
 * Function Name  : vMoveTask
 *******************************************************************************/
void vMoveTask(void *pvParameters)
{
	GCodeTypeDef GCommand;
	UBaseType_t uxHighWaterMark;

	/* Inspect our own high water mark on entering the task. */
	uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);

	vTaskDelay(100);

	InitMovement();

//	GPIO_ResetBits(GPIOB, GPIO_Pin_4 | GPIO_Pin_3 | GPIO_Pin_5 | GPIO_Pin_7 | GPIO_Pin_1 | GPIO_Pin_11 | GPIO_Pin_13);

	ZeroPosition();

	vTaskDelay(100);
	extern TaskHandle_t xMoveHandle;
	vTaskPrioritySet(xMoveHandle, 8);
	while(1)
	{

		if(xQueueReceive(GCodeQueue, &GCommand, (TickType_t) 10))
		{
			//vTaskDelay(1000);
			fuckBuffer[10]++;
			ProcessGCode(&GCommand);
			//vTaskPrioritySet(xMoveHandle, 0);

//			xSemaphoreTake(xLEDMutex, portMAX_DELAY);
//			{
//				static uint8_t ddd = 255;
//				if(ddd<200)
//				{
//				ILI9327_Num(220, 50 + (ddd) * 20, ddd);
//				//ILI9327_NumWDesc(0, 50 + (ddd) * 20, "C", GMoveBuffer[ddd].rollupCalcValues);
//				//ILI9327_NumWDesc(80, 50 + (ddd) * 20, "F", GMoveBuffer[ddd].rollupFinishedValues);
////				ILI9327_NumWDesc(0, 60 + (ddd) * 20, "c1 ", GMoveBuffer[ddd].calcAvail[0]);
////				ILI9327_NumWDesc(80, 60 + (ddd) * 20, "c2 ", GMoveBuffer[ddd].calcAvail[1]);
//				ILI9327_NumWDesc(160, 60 + (ddd) * 20, "co ", GMoveBuffer[ddd].comTotalStep);
//
//				}
//ddd++;
//			}
//			xSemaphoreGive(xLEDMutex);

			/* Calling the function will have used some stack space, we would
			 therefore now expect uxTaskGetStackHighWaterMark() to return a
			 value lower than when it was called on entering the task. */
			uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
//			xSemaphoreTake(xLEDMutex, portMAX_DELAY);
//			{
//				static uint8_t rrr = 200;
//				ILI9327_NumWDesc(150, rrr, "S:", uxHighWaterMark);
//				rrr += 10;
//
//			}
//			xSemaphoreGive(xLEDMutex);
		}
		taskYIELD();

		vTaskDelay(10);
	}

	vTaskDelete(NULL);
}










/*******************************************************************************
 * Function Name  : TIM2_IRQHandler
 * Description    : Handle setting clock pins of CNC axis control
 *******************************************************************************/
void TIM2_IRQHandler()
{
	portENTER_CRITICAL();

	static uint32_t overrunDecSpeed = 0;

	if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
	{
		TIM2->CNT = 65000;
		TIM2->SR = (uint16_t) ~TIM_IT_Update; // Clear IRQ pending bit

		uint16_t axisToStepOut = 0;

		uint8_t dontIncrementCounter = 0;

//		uint8_t getAwayFlag = 0;
		for(uint8_t i = 0; i < AXIS_NUM; i++)
		{
//			if(repeatCounter[i])
			{
//				getAwayFlag = 1;
//				repeatCounter[i]--;

				BitSet(axisToStepOut, i);
			}
		}
//		if(getAwayFlag)
//			goto SetStepsAndExit;

assert_param(isCoordBuf(ptrGMoveEndCom));

		if(GMoveBuffer[ptrGMoveCurrentCom].comCurrentStep == 1)
		{
assert_param(multSpeed<3);

			Move_setDirectionAxes(GMoveBuffer[ptrGMoveCurrentCom].incDecAxis);
			Move_enableAxes(0xFF);
			prevSync = 0;
			multSpeed = 0;
		}

//#################################################################################################
//############################ EMERG STOP - NO CALC VALUES ########################################
//#################################################################################################
//this is shit. RAMP is very important
		for(uint8_t i = 0; i < AXIS_NUM; i++) // if no calculated values available - stop execution
			if(CalcBufferRemain(i) == 0)
			{
				TIM2->ARR = 4000000 / MoveSpeedMultiplier * 2;
				TIM2->CNT = 0;
				TIM5->ARR = 4000000 / MoveSpeedMultiplier / 2;
				TIM5->CNT = 0;

				//assert_param(0); // this is critical;

				GPIOB->BSRRH = GPIO_Pin_15;

				portEXIT_CRITICAL();
				return;
			}

		uint64_t remainStepsInGBuf = 0xFFFFFFFF;

assert_param(isCoordBuf(ptrGMoveCurrentCom));

//#################################################################################################
//################################## Command end ##################################################
//#################################################################################################
		if(GMoveBuffer[ptrGMoveCurrentCom].comCurrentStep > GMoveBuffer[ptrGMoveCurrentCom].comTotalSteps &&
			GMoveBuffer[ptrGMoveCurrentCom].flagAxisNotCompleted == 0)
		{
			if(ptrGMoveCurrentCom == ptrGMoveEndCom)
			{
				Move_disableAxes(0xFF);

				TIM2->CNT = 1000;
				TIM_Cmd(TIM2, DISABLE);

assert_param(multSpeed<700);

				portEXIT_CRITICAL();
				return;
			}

assert_param(isCoordBuf(ptrGMoveCurrentCom));

			if(++ptrGMoveCurrentCom >= GMOVE_BUFFER_S)
			{
				ptrGMoveCurrentCom = 0;
			}

//#################################################################################################
//################################## Command start ################################################
//#################################################################################################
			if(GMoveBuffer[ptrGMoveCurrentCom].comCurrentStep == 1)
			{
//				if(GMoveBuffer[(ptrGMoveCurrentCom == 0) ? GMOVE_BUFFER_S-1 : ptrGMoveCurrentCom - 1].doNotStop==0)
//					assert_param(multSpeed<2);

				Move_setDirectionAxes(GMoveBuffer[ptrGMoveCurrentCom].incDecAxis);
				Move_enableAxes(0xFF);
				prevSync = 0; //GMoveBuffer[ptrGMoveCurrentCom].SyncCounter;
				multSpeed = (MoveSpeedMultiplier - RAMP_MIN) / RAMP_ACCEL;
			}
		}

//#################################################################################################
//############################### Coordinates processing ##########################################
//#################################################################################################
		for(uint8_t i = 0; i < AXIS_NUM; i++)
		{
assert_param(isCoordBuf(ptrGMoveCurrentCom));

			if(BitIsReset(GMoveBuffer[ptrGMoveCurrentCom].flagAxisNotCompleted, i))
			{
				continue;
			}

assert_param(isSqBuf(ptrMoveCurrent[i]));

			if(coordinates[ptrMoveCurrent[i]].step[i] <= GMoveBuffer[ptrGMoveCurrentCom].comCurrentStep)
			{
				BitSet(axisToStepOut, i);
				IncCurrMovePtr(i);

assert_param(isCoordBuf(ptrGMoveCurrentCom));

				// refresh real coordinates
				//if(coordinates[ptrMoveCurrent[i]].step[i] > GMoveBuffer[ptrGMoveCurrentCom].comCurrentStep)
					CNCCoordinates.step[i] += GMoveBuffer[ptrGMoveCurrentCom].incDecAxis[i];

				if(CNCCoordinates.step[i] == GMoveBuffer[ptrGMoveCurrentCom].GCommandEndPos.step[i])
				{
					//delete axis till end of GCommand
					BitReset(GMoveBuffer[ptrGMoveCurrentCom].flagAxisNotCompleted, i);
				}
				else if(coordinates[ptrMoveCurrent[i]].step[i] <= GMoveBuffer[ptrGMoveCurrentCom].comCurrentStep)
				{
					dontIncrementCounter = 1; //we have 1more same coordinates to process
				}
			}
		}

//#################################################################################################
//################################### Calculate speed #############################################
//#################################################################################################

		uint64_t leftTemp = 0;

		if((ptrGMoveCurrentCom != ptrGMoveEndCom) && GMoveBuffer[ptrGMoveCurrentCom].doNotStop)
		{
			leftTemp =
				GMoveBuffer[(ptrGMoveCurrentCom + 1 == GMOVE_BUFFER_S) ? 0 : ptrGMoveCurrentCom + 1].comTotalSteps +
				GMoveBuffer[ptrGMoveCurrentCom].comRealTotalSteps -
				GMoveBuffer[ptrGMoveCurrentCom].comRealCurrentStep;
		}
		else
		{
			leftTemp = GMoveBuffer[ptrGMoveCurrentCom].comRealTotalSteps -
				GMoveBuffer[ptrGMoveCurrentCom].comRealCurrentStep;
		}

assert_param(GMoveBuffer[ptrGMoveCurrentCom].comRealTotalSteps+1/*1*/ >= GMoveBuffer[ptrGMoveCurrentCom].comRealCurrentStep);

		if(remainStepsInGBuf > leftTemp)
		{
			remainStepsInGBuf = leftTemp;
		}


		if(prevSync == GMoveBuffer[ptrGMoveCurrentCom].SyncCounter)
			overrunDecSpeed++;

		if(GMoveBuffer[ptrGMoveCurrentCom].SyncCounter != prevSync)
		{
			uint32_t accelPtrVal = GMoveBuffer[ptrGMoveCurrentCom].SyncCounter - prevSync - 1;

			if(overrunDecSpeed >= accelPtrVal)
				overrunDecSpeed -= accelPtrVal;
			else
				overrunDecSpeed = 0;
		}

		if(GMoveBuffer[ptrGMoveCurrentCom].SyncCounter == 0)
			overrunDecSpeed = 0;

		if((multSpeed <= remainStepsInGBuf/**RAMP_SUB_ACCEL*/) &&
		 /*(GMoveBuffer[ptrGMoveCurrentCom].SyncCounter == 0
		 || GMoveBuffer[ptrGMoveCurrentCom].SyncCounter != prevSync) &&*/ (overrunDecSpeed == 0))
		{
			if(MoveSpeedMultiplier < RAMP_MAX)
			{
				multSpeed++;
			}
		}
		else
		{
			if(multSpeed > 0)
			{
				multSpeed--;
			}
		}

assert_param(multSpeed<49950);

		MoveSpeedMultiplier = RAMP_MIN + multSpeed * RAMP_ACCEL;

		// increment step counter if it's need
		if(dontIncrementCounter == 0)
		{
			//assert_param(isCoordBuf(ptrGMoveCurrentCom));
			GMoveBuffer[ptrGMoveCurrentCom].comCurrentStep++;
		}

//assert_param(axisToStepOut);
		if(axisToStepOut)
		{
			GMoveBuffer[ptrGMoveCurrentCom].comRealCurrentStep++;
		}

		prevSync = GMoveBuffer[ptrGMoveCurrentCom].SyncCounter;

SetStepsAndExit:
		//Write Step GPIOs
		Move_stepGenRising(axisToStepOut);

assert_param((4200000 / MoveSpeedMultiplier / 2) > 0);

		TIM2->ARR = 4200000 / MoveSpeedMultiplier/**2*/;
		TIM2->CNT = 0;
		TIM5->ARR = 4200000 / MoveSpeedMultiplier / 2;
		TIM5->CNT = 0;

		TIM5->CR1 |= TIM_CR1_CEN;
	}

	portEXIT_CRITICAL();
	portYIELD_FROM_ISR(0);
}


/*******************************************************************************
 * Function Name  : TIM5_IRQHandler
 * Description    : Handle resetting clock pins of CNC axis control
 *******************************************************************************/
void TIM5_IRQHandler()
{
	portENTER_CRITICAL();

	if(TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET)
	{
		TIM5->SR = (uint16_t) ~TIM_IT_Update;

		Move_stepGenFalling(0xFF);

		multSpeed = (MoveSpeedMultiplier - RAMP_MIN) / RAMP_ACCEL;
	}

	TIM5->CR1 &= ~(TIM_CR1_CEN);

	portEXIT_CRITICAL();
}



/* ========================================================================= */
/* ========================================================================= */
/* ========================================================================= */
/* ================================ H A L ================================== */
/* ========================================================================= */
/* ========================================================================= */
/* ========================================================================= */
static void Move_enableAxes(uint8_t axes)
{
	GPIO_SetBits(GPIOE, GPIO_Pin_4); //EN
}

static void Move_disableAxes(uint8_t axes)
{
	GPIO_ResetBits(GPIOE, GPIO_Pin_4); //EN
}

/*******************************************************************************
 * Function Name  : Move_setDirectionAxes
 *******************************************************************************/
static void Move_setDirectionAxes(int32_t axes)
{
}


static void Move_stepGenRising(uint8_t axes)
{

}

static void Move_stepGenFalling(uint8_t axes)
{

}

/* ========================================================================= */
/* ========================================================================= */
/* ========================================================================= */
/* ============================ I N T E R F A C E ========================== */
/* ========================================================================= */
/* ========================================================================= */
/* ========================================================================= */

/*******************************************************************************
 * Function Name  : GetCurrentPositionMM (in mm)
 *******************************************************************************/
PositionTypeDef GetCurrentPositionMM()
{
	PositionTypeDef pos;

	pos.pos[XAxis] = CNCCoordinates.step[XAxis] * MM_STEPS;
	pos.pos[YAxis] = CNCCoordinates.step[YAxis] * MM_STEPS;
	pos.pos[ZAxis] = CNCCoordinates.step[ZAxis] * MM_STEPS;

	return pos;
}

/*******************************************************************************
 * Function Name  : GetCurrentPositionUM (in um)
 *******************************************************************************/
PositionTypeDef GetCurrentPositionUM()
{
	PositionTypeDef pos;

	pos.pos[XAxis] = CNCCoordinates.step[XAxis] * uM_STEPS;
	pos.pos[YAxis] = CNCCoordinates.step[YAxis] * uM_STEPS;
	pos.pos[ZAxis] = CNCCoordinates.step[ZAxis] * uM_STEPS;

	return pos;
}
