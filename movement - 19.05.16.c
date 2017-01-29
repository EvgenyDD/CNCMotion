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

#define isSqBuf(xx) ((xx)<COORD_BUFFER_S)
#define isCoordBuf(xx) ((xx)<GMOVE_BUFFER_S)
#define isAxis(xx) ((xx)<AXIS_NUM)

/* Global variables ----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
CoordTypeDef CNCCoordinates;
static const uint16_t DirPin[4] = { PIN_DIR_X, PIN_DIR_Y, PIN_DIR_Z, PIN_DIR_A };
static const uint16_t DirPin2[4] = { GPIO_Pin_5, GPIO_Pin_2, 0, 0 };

static const uint16_t StepPin[4] = { PIN_DIR_X, PIN_DIR_Y, PIN_DIR_Z, PIN_DIR_A };
static const uint16_t StepPin2[4] = { GPIO_Pin_1, GPIO_Pin_0, 0, 0 };


// Circular buffer steps
volatile uint32_t ptrMoveCurrent[AXIS_NUM], ptrMoveEnd[AXIS_NUM];
CoordSquareTypeDef squares[COORD_BUFFER_S]; //buffer containing squares of the axis coordinates

// Circular buffer command params
volatile uint32_t ptrGMoveCurrentCom, ptrGMoveEndCom;
GCommandMoveTypeDef GMoveBuffer[GMOVE_BUFFER_S]; //buffer for variables per each g-command

volatile uint32_t MoveSpeedMultiplier = RAMP_MIN;

volatile uint32_t multSpeed = 0;
volatile uint32_t prevSync = 0;


/* Extern variables ----------------------------------------------------------*/
extern xQueueHandle GCodeQueue;

	extern int32_t fuckfuck, fuckin2;
		extern uint8_t fuckBuffer[110];


/* Private function prototypes -----------------------------------------------*/
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

	GPIO_ResetBits(GPIOE, GPIO_Pin_4); //EN

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
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4; //?
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
	NVIC_EnableIRQ(TIM5_IRQn);

	TIM_ARRPreloadConfig(TIM2, ENABLE);
	TIM_ARRPreloadConfig(TIM5, ENABLE);
}


/////////////////////////// CALCULATING FUNCTIONS /////////////////////////////
static void preCalcLinear(uint8_t axis, GCodeTypeDef *GCommand)
{
//	if(BitIsReset(GMoveBuffer[ptrGMoveEndCom].flagAxisNotCompleted, axis))
//		return;

assert_param(isCoordBuf(ptrGMoveEndCom));
assert_param(isAxis(axis));

	GMoveBuffer[ptrGMoveEndCom].SqAxisSteps[axis] =
		GMoveBuffer[ptrGMoveEndCom].Delta.step[axis]/* * GMoveBuffer[ptrGMoveEndCom].Delta.step[axis]*/;
}


static void preCalcRound(uint8_t axis, GCodeTypeDef *GCommand)
{
//	if(BitIsReset(GMoveBuffer[ptrGMoveEndCom].flagAxisNotCompleted, axis))
//		return;

assert_param(isCoordBuf(ptrGMoveEndCom));
assert_param(isAxis(axis));

	GMoveBuffer[ptrGMoveEndCom].SqAxisSteps[axis] = 	GMoveBuffer[ptrGMoveEndCom].Delta.step[axis]/* * GMoveBuffer[ptrGMoveEndCom].Delta.step[axis]*/;

	if(GCommand->R != 0)
	{
		GMoveBuffer[ptrGMoveEndCom].SqRCoef[axis] = (uint64_t)GMoveBuffer[ptrGMoveEndCom].comTotalStep * (uint64_t)GMoveBuffer[ptrGMoveEndCom].comTotalStep;
	}
	else //I J K
	{
		//xd = x2-x1
		//yd = y2-y1
		//SquareRoot(xd*xd + yd*yd)
	
		GMoveBuffer[ptrGMoveEndCom].SqRCoef[axis] = (uint32_t) sqrtl(GCommand->I * GCommand->I + GCommand->J * GCommand->J); 
	}
	
	if(GMoveBuffer[ptrGMoveEndCom].SqRCoef[axis] == 0) 
		GMoveBuffer[ptrGMoveEndCom].SqRCoef[axis] = 1;
		
	//GMoveBuffer[ptrGMoveEndCom].GCommandStartPos.step[i]

	//GMoveBuffer[ptrGMoveEndCom].SqRCoef = ;
}


static void preCalcEllipse(uint8_t axis, GCodeTypeDef *GCommand)
{
//	if(BitIsReset(GMoveBuffer[ptrGMoveEndCom].flagAxisNotCompleted, axis))
//		return;

assert_param(isCoordBuf(ptrGMoveEndCom));
assert_param(isAxis(axis));

	GMoveBuffer[ptrGMoveEndCom].SqAxisSteps[axis] = 	GMoveBuffer[ptrGMoveEndCom].Delta.step[axis]/* * GMoveBuffer[ptrGMoveEndCom].Delta.step[axis]*/;
	GMoveBuffer[ptrGMoveEndCom].SqRCoef[axis] = (uint64_t)GMoveBuffer[ptrGMoveEndCom].comTotalStep * (uint64_t)GMoveBuffer[ptrGMoveEndCom].comTotalStep;
	
	

//	GMoveBuffer[ptrGMoveEndCom].SqKCoef = ;
}


static uint64_t CalcLinear(uint8_t axis)
{
	// check future value
//	if(GCommandRelPos.step[axis] + 1 == GMoveBuffer[ptrGMoveEndCom].Delta.step[axis])
//		BitReset(flagAxisNotCompleted, axis);

assert_param(isCoordBuf(ptrGMoveEndCom));
assert_param(isAxis(axis));

	GMoveBuffer[ptrGMoveEndCom].GCommandCurrPos.step[axis]++;
	uint32_t SqCoord = (GMoveBuffer[ptrGMoveEndCom].GCommandCurrPos.step[axis])/* * (GCommandRelPos.step[axis])*/;
	uint32_t val = SqCoord * GMoveBuffer[ptrGMoveEndCom].SqComTotalSteps
		/ GMoveBuffer[ptrGMoveEndCom].SqAxisSteps[axis];

	return val;
}


static uint64_t CalcRound(uint8_t axis)
{
	GMoveBuffer[ptrGMoveEndCom].GCommandCurrPos.step[axis]++;

	uint64_t xx = GMoveBuffer[ptrGMoveEndCom].Delta.step[axis] - GMoveBuffer[ptrGMoveEndCom].GCommandCurrPos.step[axis];
	uint64_t SqCoordDiff = GMoveBuffer[ptrGMoveEndCom].SqRCoef[axis] - (xx * xx);
assert_param(GMoveBuffer[ptrGMoveEndCom].SqRCoef[axis]>=(xx * xx));
	uint64_t val = (uint32_t)sqrtl(SqCoordDiff); // * SqComTotalSteps / GMoveBuffer[ptrGMoveEndCom].SqAxisSteps[axis];

	if(axis == YAxis)
	{

		return val;
	}

//	xx = GCommandRelPos.step[axis];
//	SqCoordDiff = GMoveBuffer[ptrGMoveEndCom].SqRCoef[axis] - (xx*xx);

	uint64_t val2 = GMoveBuffer[ptrGMoveEndCom].GCommandCurrPos.step[axis]; // * SqComTotalSteps / GMoveBuffer[ptrGMoveEndCom].SqAxisSteps[axis];

	return val2;
}


static uint64_t CalcEllipse(uint8_t axis)
{
	return 0;
}


static uint32_t GetNextEndPtr_Inc(uint8_t axis)
{
assert_param(isAxis(axis));

	uint32_t value = ptrMoveEnd[axis];

	if(++ptrMoveEnd[axis] >= COORD_BUFFER_S)
		ptrMoveEnd[axis] = 0;

assert_param(isSqBuf(ptrMoveEnd[axis]));

	return value;
}

static uint32_t GetPrevEndPtr(uint8_t axis)
{
assert_param(isAxis(axis));

	uint32_t value = ptrMoveEnd[axis];

	if(ptrMoveEnd[axis] == 0)
		return COORD_BUFFER_S-1;
	else
		return ptrMoveEnd[axis]-1;

assert_param(isSqBuf(ptrMoveEnd[axis]));

	return value;
}


static uint32_t IncCurrMovePtr(uint8_t axis)
{
assert_param(isAxis(axis));

	uint32_t value = ptrMoveCurrent[axis];

assert_param(ptrMoveCurrent[axis]!=ptrMoveEnd[axis]);

	if(++ptrMoveCurrent[axis] >= COORD_BUFFER_S)
		ptrMoveCurrent[axis] = 0;

assert_param(isSqBuf(ptrMoveCurrent[axis]));
	//assert_param(ptrMoveCurrent[axis]!=ptrMoveEnd[axis]);

	return value;
}


/*static */uint32_t CalcBufferRemain(uint8_t axis)
{

assert_param(isCoordBuf(ptrGMoveEndCom));
assert_param(isAxis(axis));

	if(BitIsReset(GMoveBuffer[ptrGMoveEndCom].flagAxisNotCompleted, axis))
		return 0xFFFF;

//	if(axis == 0)
//		return 10;

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
		if(abs(data->step[i]) > result)
			result = abs(data->step[i]);

	return result;
}


/*******************************************************************************
 * Function Name  : SetMotorDirection
 *******************************************************************************/
static void SetMotorDirection(int32_t *data)
{
	//const GPIO_TypeDef* DirPort[4] = {GPIO_DIR_X, GPIO_DIR_Y, GPIO_DIR_Z, GPIO_DIR_A};

	for(uint8_t i = 0; i < AXIS_NUM; i++)
	{
		if(data[i] >= 0)
			GPIOE->BSRRL = DirPin2[i];
		else
			GPIOE->BSRRH = DirPin2[i];
	}
	//GPIO_WriteBit(/*(GPIO_TypeDef*)DirPort[i]*/PORT_DIR, DirPin[i], data->step[i] >= 0 ? Bit_SET : Bit_RESET);
}


/*******************************************************************************
 * Function Name  : LinearMove
 *******************************************************************************/
static void Move(GCodeTypeDef *GCommand)
{
	fuckBuffer[0] ++;
	int32_t* ptrGCommand[AXIS_NUM] = { &(GCommand->X), &(GCommand->Y), &(GCommand->Z) /*, &(GCommand->A) */};

	preCalcPtr PreCalcFunc[] = { preCalcLinear, preCalcLinear, //G00 G01
								 preCalcRound, preCalcRound,  //G02 G03
								 preCalcEllipse }; //???

	CalcPtr CalcFunc[] = { CalcLinear, CalcLinear, //G00 G01
						   CalcRound, CalcRound,  //G02 G03
						   CalcEllipse }; //???

	uint16_t endComCondition = 0;

	xSemaphoreTake(xLEDMutex, portMAX_DELAY);
	{

		ILI9327_NumWDesc(0, 300 + ptrGMoveEndCom * 10, "X:", GCommand->X);
		ILI9327_NumWDesc(80, 300 + ptrGMoveEndCom * 10, "Y:", GCommand->Y);
		ILI9327_NumWDesc(160, 300 + ptrGMoveEndCom * 10, "Z:", GCommand->Z);
		ILI9327_NumWDesc(220, 300 + ptrGMoveEndCom * 10, "p:", ptrGMoveEndCom);

	}
	xSemaphoreGive(xLEDMutex);

	//Get target axis counters values
	for(uint8_t i = 0; i < AXIS_NUM; i++)
	{
assert_param(isCoordBuf(ptrGMoveEndCom));

		GMoveBuffer[ptrGMoveEndCom].GCommandCurrPos.step[i] = 0;
		GMoveBuffer[ptrGMoveEndCom].GCommandEndPos.step[i] = ((*ptrGCommand[i]) * STEPS_REV / SCREW_PITCH_uM);
		
		if(ptrGMoveEndCom == ptrGMoveCurrentCom)
			GMoveBuffer[ptrGMoveEndCom].GCommandStartPos.step[i] = CNCCoordinates.step[i]; //in um
		else
			GMoveBuffer[ptrGMoveEndCom].GCommandStartPos.step[i] = 
				GMoveBuffer[ptrGMoveEndCom == 0 ? GMOVE_BUFFER_S - 1 : ptrGMoveEndCom - 1].GCommandEndPos.step[i]; //in um
				
		GMoveBuffer[ptrGMoveEndCom].Delta.step[i] = GMoveBuffer[ptrGMoveEndCom].GCommandEndPos.step[i] - GMoveBuffer[ptrGMoveEndCom].GCommandStartPos.step[i];
	}

assert_param(isCoordBuf(ptrGMoveEndCom));

	GMoveBuffer[ptrGMoveEndCom].flagAxisNotCompleted = 0; //reset uncompleted flags
	GMoveBuffer[ptrGMoveEndCom].Axes = 0; // reset GCode Config

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

	endComCondition = GMoveBuffer[ptrGMoveEndCom].flagAxisNotCompleted;

	// calculate constants
assert_param(isCoordBuf(ptrGMoveEndCom));

	GMoveBuffer[ptrGMoveEndCom].comTotalStep = GMoveBuffer[ptrGMoveEndCom].comTotalStep2 = SelectAbsBiggestVal(&(GMoveBuffer[ptrGMoveEndCom].Delta));
	GMoveBuffer[ptrGMoveEndCom].SyncCounter = 0;
	GMoveBuffer[ptrGMoveEndCom].doNotStop = 0;

//	if(ptrGMoveEndCom != ptrGMoveCurrentCom)
//	{
//		uint32_t prevCom = (ptrGMoveEndCom == 0) ? GMOVE_BUFFER_S - 1 : ptrGMoveEndCom - 1;
//		if(GMoveBuffer[prevCom].Axes == GMoveBuffer[ptrGMoveEndCom].Axes)
//			GMoveBuffer[prevCom].doNotStop = 1;
//	}

	GMoveBuffer[ptrGMoveEndCom].SqComTotalSteps =
		GMoveBuffer[ptrGMoveEndCom].comTotalStep/**GMoveBuffer[ptrGMoveEndCom].comTotalStep*/;

	for(uint8_t i = 0; i < AXIS_NUM; i++)
	{
assert_param(GCommand->code < 5);

		PreCalcFunc[GCommand->code](i, GCommand); //pre-calculate constants for each axis
//		GMoveBuffer[ptrGMoveEndCom].calcAvail[i] = 0;
	}

	GMoveBuffer[ptrGMoveEndCom].comCurrentStep = GMoveBuffer[ptrGMoveEndCom].comCurrentStep2 = 1; //Initialize step counter
//	GMoveBuffer[ptrGMoveEndCom].rollupCalcValues = GMoveBuffer[ptrGMoveEndCom].rollupFinishedValues = 0;

//	if(GCommand->code == 1)
//		GMoveBuffer[ptrGMoveEndCom].rollupCalcValues = GMoveBuffer[ptrGMoveEndCom].comTotalStep;
//	else
//	{
//		GMoveBuffer[ptrGMoveEndCom].rollupCalcValues = GMoveBuffer[ptrGMoveEndCom].comTotalStep*1.4142135623730950488016887242097;
//	}

//	GMoveBuffer[ptrGMoveEndCom].rollupCalcValues = 0;

	TIM_Cmd(TIM2, ENABLE); //enable counter


	// main processor
	while(/*GMoveBuffer[ptrGMoveEndCom].flagAxisNotCompleted*/endComCondition)
	{
		uint16_t penalty = 0;

		for(uint8_t i = 0; i < AXIS_NUM; i++)
		{
assert_param(isCoordBuf(ptrGMoveEndCom));

			if(	CalcBufferRemain(i) < 1800 &&
				GMoveBuffer[ptrGMoveEndCom].GCommandCurrPos.step[i] != GMoveBuffer[ptrGMoveEndCom].Delta.step[i]
				/*BitIsSet(flagAxisNotCompleted, i)*/)
			{
				GPIO_SetBits(GPIOB, GPIO_Pin_1);

				uint64_t curr, prev = squares[GetPrevEndPtr(i)].square[i];

				squares[GetNextEndPtr_Inc(i)].square[i] = curr = CalcFunc[GCommand->code](i);

				uint64_t diff = curr - prev;

				//assert_param(curr>=prev);

//				if(diff == 0)
//					GMoveBuffer[ptrGMoveEndCom].calcAvail[i] += 1;
//				else
//				{
//					GMoveBuffer[ptrGMoveEndCom].calcAvail[i] += diff;
//				}

				if(diff==0)
					BitSet(penalty, i);
//				if(i==1)
//				{
//					GPIO_SetBits(GPIOB, GPIO_Pin_14);
//					DebugSendNumWDesc("i=",	prev);
//					DebugSendNumWDesc("j=",	curr);
//					DebugSendNumWDesc(">>>>>>>>=",	diff);
//				}
//				if(i==0)
//				{
//					GPIO_SetBits(GPIOB, GPIO_Pin_14);
//					DebugSendNumWDesc("---------i=",	prev);
//					DebugSendNumWDesc("---------j=",	curr);
//					DebugSendNumWDesc("--------->>>>>>>>=",	diff);
//				}
			}

			if(GMoveBuffer[ptrGMoveEndCom].GCommandCurrPos.step[i] == GMoveBuffer[ptrGMoveEndCom].Delta.step[i])
				BitReset(endComCondition, i);
		}

		if(penalty)
		{
			GMoveBuffer[ptrGMoveEndCom].comTotalStep2++;

//			for(uint8_t i = 0; i < AXIS_NUM; i++)
//			{
//				if(BitIsSet(endComCondition, i) && BitIsReset(penalty, i))
//				{
//					GMoveBuffer[ptrGMoveEndCom].calcAvail[i]++;
//				}
//			}
		}


		GMoveBuffer[ptrGMoveEndCom].SyncCounter++;

//		for(uint32_t i=0; i<100000; i++)
//			asm("nop");
		GPIO_ResetBits(GPIOB, GPIO_Pin_1);
	}

	GMoveBuffer[ptrGMoveEndCom].SyncCounter = 0;

	// increment HEAD of GCodes Ring buffer
	if(++ptrGMoveEndCom >= GMOVE_BUFFER_S)
		ptrGMoveEndCom = 0;

	// lock if we have too much unprocessed GCodes
	if(ptrGMoveEndCom == ptrGMoveCurrentCom)
		while(ptrGMoveEndCom != ptrGMoveCurrentCom)
			taskYIELD();
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

	fuckfuck = 1;
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
			fuckfuck = 0;


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
			xSemaphoreTake(xLEDMutex, portMAX_DELAY);
			{
				static uint8_t rrr = 200;
				ILI9327_NumWDesc(150, rrr, "S:", uxHighWaterMark);
				rrr += 10;

			}
			xSemaphoreGive(xLEDMutex);
		}
		taskYIELD();

		vTaskDelay(10);
	}

	vTaskDelete(NULL);
}


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

static uint8_t lockfuck = 0;
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
		//TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		TIM2->SR = (uint16_t) ~TIM_IT_Update; // Clear IRQ pending bit

		GPIOB->BSRRL = GPIO_Pin_15;

		uint16_t writeBits = 0, writeBits2 = 0;

		uint64_t SqComCurrentStep; //square of relative position counter
		uint8_t dontIncrementCounter = 0; //!!!

assert_param(isCoordBuf(ptrGMoveEndCom));

		if(GMoveBuffer[ptrGMoveCurrentCom].comCurrentStep == 1)
		{
assert_param(multSpeed<3);

			SetMotorDirection(GMoveBuffer[ptrGMoveCurrentCom].incDecAxis);
			GPIO_SetBits(GPIOE, GPIO_Pin_4); //EN
			prevSync = 0; //GMoveBuffer[ptrGMoveCurrentCom].SyncCounter;
			multSpeed = 0;//(MoveSpeedMultiplier - RAMP_MIN) / RAMP_ACCEL;
			lockfuck = 1;
			fuckBuffer[4]++;


			//GMoveBuffer[ptrGMoveCurrentCom].rollupFinishedValues = 0;
		}

//TODO: remake. RAMP is very important
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

		if(GMoveBuffer[ptrGMoveCurrentCom].comCurrentStep > GMoveBuffer[ptrGMoveCurrentCom].comTotalStep)
		{
			if(ptrGMoveCurrentCom == ptrGMoveEndCom)
			{
				GPIO_ResetBits(GPIOE, GPIO_Pin_4); //EN

				TIM2->CNT = 1000;
				TIM_Cmd(TIM2, DISABLE);
				lockfuck = 2;
				fuckBuffer[3] = ptrGMoveCurrentCom;

assert_param(multSpeed<700);

				portEXIT_CRITICAL();
				return;
			}

assert_param(isCoordBuf(ptrGMoveCurrentCom));

			if(++ptrGMoveCurrentCom >= GMOVE_BUFFER_S)
				ptrGMoveCurrentCom = 0;

			if(GMoveBuffer[ptrGMoveCurrentCom].comCurrentStep == 1)
			{
assert_param(multSpeed<2);

				SetMotorDirection(GMoveBuffer[ptrGMoveCurrentCom].incDecAxis);
				GPIO_SetBits(GPIOE, GPIO_Pin_4); //EN
				prevSync = 0; //GMoveBuffer[ptrGMoveCurrentCom].SyncCounter;
				multSpeed = (MoveSpeedMultiplier - RAMP_MIN) / RAMP_ACCEL;



				//GMoveBuffer[ptrGMoveCurrentCom].rollupFinishedValues=0;
				//GPIOB->BSRRL = GPIO_Pin_4;
				//	GPIOB->BSRRL = GPIO_Pin_4;
			}
		}

		SqComCurrentStep =
			GMoveBuffer[ptrGMoveCurrentCom].comCurrentStep/* * GMoveBuffer[ptrGMoveCurrentCom].comCurrentStep*/; //count square of relative position counter

		for(uint8_t i = 0; i < AXIS_NUM; i++)
		{
assert_param(isCoordBuf(ptrGMoveCurrentCom));

			if(BitIsReset(GMoveBuffer[ptrGMoveCurrentCom].flagAxisNotCompleted, i))
				continue;

			//******************** LIMIT BY CALCULATED VALUES *************************/
			//Calculate remain values for axis in Coord. buffer
			//uint32_t leftTemp = squares[ptrMoveEnd[i]].square[i] - SqComCurrentStep; @ or vv
//			uint32_t leftTemp = GMoveBuffer[ptrGMoveCurrentCom].rollupCalcValues
//					- GMoveBuffer[ptrGMoveCurrentCom].rollupFinishedValues;
//
//				assert_param(GMoveBuffer[ptrGMoveCurrentCom].rollupFinishedValues<=GMoveBuffer[ptrGMoveCurrentCom].rollupCalcValues);
//			uint64_t leftTemp = GMoveBuffer[ptrGMoveCurrentCom].calcAvail[i] - GMoveBuffer[i].comCurrentStep;
//
//			fuckfuck = i;
//			assert_param(GMoveBuffer[ptrGMoveCurrentCom].calcAvail[i] >= GMoveBuffer[i].comCurrentStep);
//
//				if(remainStepsInGBuf > leftTemp)
//					remainStepsInGBuf = leftTemp;

assert_param(isSqBuf(ptrMoveCurrent[i]));

			if(squares[ptrMoveCurrent[i]].square[i] <= SqComCurrentStep)
			{
				writeBits |= StepPin[i];
				writeBits2 |= StepPin2[i];
				IncCurrMovePtr(i);

assert_param(isCoordBuf(ptrGMoveCurrentCom));

				// refresh real coordinates
				CNCCoordinates.step[i] += GMoveBuffer[ptrGMoveCurrentCom].incDecAxis[i];

				if(CNCCoordinates.step[i] == GMoveBuffer[ptrGMoveCurrentCom].GCommandEndPos.step[i])
				{
					//delete axis till end of GCommand
					BitReset(GMoveBuffer[ptrGMoveCurrentCom].flagAxisNotCompleted, i);
				}
				else if(squares[ptrMoveCurrent[i]].square[i] <= SqComCurrentStep)
				{
					dontIncrementCounter = 1; //we have 1more same coordinates to process
				}
			}
		}


		uint64_t leftTemp = GMoveBuffer[ptrGMoveCurrentCom].comTotalStep2 - GMoveBuffer[ptrGMoveCurrentCom].comCurrentStep2;

assert_param(GMoveBuffer[ptrGMoveCurrentCom].comTotalStep2+1 >= GMoveBuffer[ptrGMoveCurrentCom].comCurrentStep2);

		if(remainStepsInGBuf > leftTemp)
			remainStepsInGBuf = leftTemp;





//assert_param(isCoordBuf(nextCom));*

		//******************** LIMIT BY END OF COMMAND *************************/
		// if next GCommand not equal in config - stop ramp
	/*	if(ptrGMoveCurrentCom != ptrGMoveEndCom
			&& GMoveBuffer[ptrGMoveCurrentCom].doNotStopGMoveBuffer[nextCom].Axes != GMoveBuffer[ptrGMoveCurrentCom].Axes)
		{
			//GPIOB->BSRRL = GPIO_Pin_4;
			//assert_param(isCoordBuf(ptrGMoveCurrentCom));
			uint32_t tmp = GMoveBuffer[ptrGMoveCurrentCom].comTotalStep
				- GMoveBuffer[ptrGMoveCurrentCom].comCurrentStep;
			tmp += GMoveBuffer[(ptrGMoveCurrentCom + 1 == GMOVE_BUFFER_S) ? 0 : ptrGMoveCurrentCom + 1].comTotalStep;

			if(remainStepsInGBuf > tmp)
			{
				//GPIOB->BSRRL = GPIO_Pin_4;
				//remainStepsInGBuf = tmp;
			}
		}
		else
		{
			//assert_param(isCoordBuf(ptrGMoveCurrentCom));
			uint32_t tmp = (GMoveBuffer[ptrGMoveCurrentCom].comTotalStep
				- GMoveBuffer[ptrGMoveCurrentCom].comCurrentStep);
			if(remainStepsInGBuf > tmp)
			{
				//GPIOB->BSRRL = GPIO_Pin_14;
			//	remainStepsInGBuf = tmp;

			}
		}*/

		if(prevSync == GMoveBuffer[ptrGMoveCurrentCom].SyncCounter)
			overrunDecSpeed++;

		if(GMoveBuffer[ptrGMoveCurrentCom].SyncCounter != prevSync)
		{
			uint32_t accelPtrVal = GMoveBuffer[ptrGMoveCurrentCom].SyncCounter - prevSync - 1;

			if(overrunDecSpeed >= accelPtrVal)
				overrunDecSpeed -= accelPtrVal;
			else
				overrunDecSpeed = 0;
			//GPIOB->BSRRL = GPIO_Pin_4;
		}

		if(GMoveBuffer[ptrGMoveCurrentCom].SyncCounter == 0)
			overrunDecSpeed = 0;

		if(overrunDecSpeed)
			GPIOB->BSRRL = GPIO_Pin_5;

		if(multSpeed <= remainStepsInGBuf/**RAMP_SUB_ACCEL*/ && /*(GMoveBuffer[ptrGMoveCurrentCom].SyncCounter == 0
		 || GMoveBuffer[ptrGMoveCurrentCom].SyncCounter != prevSync) &&*/overrunDecSpeed == 0)
		{
			if(MoveSpeedMultiplier < RAMP_MAX)
			{
				multSpeed++;
				//MoveSpeedMultiplier = RAMP_MIN + multSpeed * RAMP_ACCEL;
				GPIOB->BSRRL = GPIO_Pin_13;
			}
//			else
//			{
//				GPIOB->BSRRL = GPIO_Pin_4;
//			}
		}
		else
		{
			//if(MoveSpeedMultiplier >= RAMP_ACCEL)
			if(multSpeed > 0)
			{
				//MoveSpeedMultiplier -= RAMP_ACCEL;
				multSpeed--;

				GPIOB->BSRRL = GPIO_Pin_11/*|GPIO_Pin_13*/;
			}
		}

//		if(writeBits)
//		{
//			ILI9327_setColor(VGA_BLACK);
//			ILI9327_Circle(CNCCoordinates.step[0]+100, CNCCoordinates.step[1]+100, 1);
//		}

assert_param(multSpeed<49950);

		MoveSpeedMultiplier = RAMP_MIN + multSpeed * RAMP_ACCEL;

		// increment step counter if it's need
		if(dontIncrementCounter == 0)
		{
			//assert_param(isCoordBuf(ptrGMoveCurrentCom));
			GMoveBuffer[ptrGMoveCurrentCom].comCurrentStep++;
		}

//assert_param(writeBits);
		if(writeBits)
			GMoveBuffer[ptrGMoveCurrentCom].comCurrentStep2++;

		//Write Step GPIOs
		PORT_DIR->BSRRL = writeBits/* & GPIO_Pin_3*/;


//		if(writeBits != 0)
//			GMoveBuffer[ptrGMoveCurrentCom].rollupFinishedValues++;

		GPIOE->BSRRL = writeBits2;

		prevSync = GMoveBuffer[ptrGMoveCurrentCom].SyncCounter;

assert_param((4200000 / MoveSpeedMultiplier / 2) > 0);

		TIM2->ARR = 4200000 / MoveSpeedMultiplier/**2*/;
		TIM2->CNT = 0;
		TIM5->ARR = 4200000 / MoveSpeedMultiplier / 2;
		TIM5->CNT = 0;

		TIM5->CR1 |= TIM_CR1_CEN;

		GPIOB->BSRRH = GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_15;
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
	GPIOB->BSRRL = GPIO_Pin_14;

	//if(TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET)
	{
		//TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
		TIM5->SR = (uint16_t) ~TIM_IT_Update;

		for(uint8_t i = 0; i < AXIS_NUM; i++)
			PORT_DIR->BSRRH = StepPin[i]; //reset pins

		GPIOE->BSRRH = GPIO_Pin_1 | GPIO_Pin_0;

		multSpeed = (MoveSpeedMultiplier - RAMP_MIN) / RAMP_ACCEL;
	}

	TIM5->CR1 &= ~(TIM_CR1_CEN);

	GPIOB->BSRRH = GPIO_Pin_14;
	portEXIT_CRITICAL();
}
