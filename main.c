/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "system_stm32f4xx.h"

#include <math.h>

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
#include "stm32f4xx_syscfg.h"
#include "misc.h"
#include "core_cm4.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "croutine.h"
#include "portmacro.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "portmacro.h"

#include "pcd8544.h"
#include "ILI9327.h"

#include "touch.h"


#include "string.h"


#include "movement.h"
#include "controller.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
xTimerHandle tFlashLine;
xSemaphoreHandle xLEDMutex;
xQueueHandle xUARTQueue;

char textBuf[31*(30+1)];

uint32_t backtimer = 0;

static uint8_t weAreInAss = 0;


/* Extern variables ----------------------------------------------------------*/
extern uint8_t SmallFont[];
extern uint8_t BigFont[];
extern uint8_t SevenSegNumFont[];
extern uint8_t font5x8[];

extern PointType TouchPoint;

extern void _delay_ms(uint16_t val);

extern xQueueHandle GCodeQueue;

extern CoordTypeDef CNCCoordinates;


			uint8_t fuckBuffer[110] = {0};

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
* Function Name  : vApplicationIdleHook
* Description    :
*******************************************************************************/
void vApplicationIdleHook(void)
{
}


/*******************************************************************************
* Function Name  : vApplicationMallocFailedHook
* Description    :
*******************************************************************************/
void vApplicationMallocFailedHook(void)
{
    for( ;; );
}


/*******************************************************************************
* Function Name  : vApplicationStackOverflowHook
* Description    :
*******************************************************************************/
void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName )
{
//    ( void ) pcTaskName;
//    ( void ) pxTask;

	GPIO_SetBits(GPIOE, GPIO_Pin_4); //EN

	for(;;);
}


/*******************************************************************************
* Function Name  : vApplicationTickHook
* Description    :
*******************************************************************************/
void vApplicationTickHook( void )
{
	backtimer++;
}


/*******************************************************************************
* Function Name  : InitPeriph
* Description    :
*******************************************************************************/
void InitPeriph()
{
   /** DISCOVERY LEDS */
//ORANGE - LD3 - PD13
//GREEN  - LD4 - PD12
//RED 	 - LD5 - PD14
	//BLUE	 - LD6 - PD15
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource13);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    /* DISCOVERY LEDS **/


    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOC, &GPIO_InitStructure);



    RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_RNG, ENABLE);
    /* RNG Peripheral enable */
    RNG_Cmd(ENABLE);
}


// stop everything on emergency button
void EXTI15_10_IRQHandler()
{
    /* Make sure that interrupt flag is set */
    if(EXTI_GetITStatus(EXTI_Line13) != RESET) {
        /* Do your stuff when PB12 is changed */

    	if(weAreInAss)
    		NVIC_SystemReset();

    	EXTI_ClearITPendingBit(EXTI_Line13);

    	assert_param(0);
        /* Clear interrupt flag */

        weAreInAss = 1;
    }
}

/*******************************************************************************
* Function Name  :
* Description    :
*******************************************************************************/
void vLedTask (void *pvParameters)
{
    while(1)
    {
		vTaskDelay(500);
    }
    vTaskDelete(NULL);
}

void vTouchDispatcher(void *pvParameters)
{

	ILI9327_Clear();
	ILI9327_Fill(VGA_WHITE);
	ILI9327_setColor(VGA_BLACK);

//	extern const unsigned int BITMAP1[0x11760];
//	ILI9327_Bitmap(0,0,240,298,(unsigned int*)BITMAP1,1);
//	vTaskDelay(2000);


uint8_t ccc = 0;

//rrr();

	NVIC_InitTypeDef NVIC_InitStruct;

	/* PD0 is connected to EXTI_Line0, which has EXTI0_IRQn vector */
	NVIC_InitStruct.NVIC_IRQChannel = EXTI15_10_IRQn;
	/* Set priority */
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
	/* Set sub priority */
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
	/* Enable interrupt */
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	/* Add to NVIC */
	NVIC_Init(&NVIC_InitStruct);


	//vTaskDelay(2000);

	GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    uint8_t progPtr = 0;

	while(1)
	{
		static uint16_t TscX, TscY;

		if(!GPIO_ReadInputDataBit(XPT2046_IRQ_PORT, XPT2046_IRQ_PAD))
		{
			if(ccc == 0)
			{
				XPT2046_GetCursor(&TscX, &TscY);

//				xSemaphoreTake(xLEDMutex, portMAX_DELAY);
//				{
//
//					ILI9327_setColor(VGA_BLACK);
//					ILI9327_setBackColor(VGA_WHITE);
//					ILI9327_setFont(SmallFont);
//					ILI9327_NumWDesc(0,220,"X: ", TscX);
//					ILI9327_NumWDesc(0,232,"Y: ", TscY);
//
//					ILI9327_Circle(TscX, TscY, 1);
//
//				}xSemaphoreGive(xLEDMutex);

				if(uxQueueMessagesWaiting(GCodeQueue) == 0)
				{
					GCodeTypeDef TargetMove;
					uint8_t givecom = 0;

					TargetMove.code = 0;
					TargetMove.A = 0;
					TargetMove.I = 0;
					TargetMove.J = 0;
					TargetMove.K = 0;
					TargetMove.R = 0;
					TargetMove.feedRate = 0;
					TargetMove.X = 0;
					TargetMove.Y = 0;
					TargetMove.Z = 0;


					if(TscY<50)
					{
						TargetMove.Y = CNCCoordinates.step[YAxis] + 10 * X_1MM;
						givecom = 1;
					}
					else if(TscY > 100 && TscY < 150)
					{
						TargetMove.Y = CNCCoordinates.step[YAxis] - 10 * X_1MM;
						givecom = 1;
					}

					if(TscX < 80 && TscY < 150)
					{
						TargetMove.X = CNCCoordinates.step[XAxis] - 10 * X_1MM;
						givecom = 1;
					}
					else if(TscX > 160 && TscY < 150)
					{
						TargetMove.X = CNCCoordinates.step[XAxis] + 10 * X_1MM;
						givecom = 1;
					}

					if(TscY>=50 && TscY <= 50 * 2 && TscX >= 80 && TscX <= 160)
					{
						if(givecom == 0)
							SendHomePos();
					}

					if(givecom == 1)
					{
						SendGCodeCommand(&TargetMove);
						vTaskDelay(100);
					}


					if(TscY>=160 && TscY <= 210)
					{
						if(TscX <= 80)
							progPtr = 1;
						else if(TscX <= 160)
							progPtr = 2;
						else
							progPtr = 3;
					}

					if(TscY>=220 && TscY <= 270)
					{
						if(TscX <= 80)
							progPtr = 4;
						else if(TscX <= 160)
							progPtr = 5;
						else
							progPtr = 6;
					}

					if(TscY>=280 && TscY <= 350 && TscX >= 120)
					{
						switch(progPtr)
						{
						case 1:
							RoundTest();
							break;
						case 2:
							Parser(0);
							break;
						case 3:
							draw_decart_space(40,40);
							break;
						case 4:
							TestCircle(40);
							break;
						case 5:
							TestCircle(5);
							break;
						case 6:
							Parser2(0);
							break;
						}
					}
				}

				vTaskDelay(10);
			}
			ccc = 1;
		}
		else
		{
			ccc = 0;
			xSemaphoreTake(xLEDMutex, portMAX_DELAY);
			{
				static uint8_t yyy=0;

				ILI9327_setFont(SmallFont);
				ILI9327_setBackColor(VGA_WHITE);

				ILI9327_setColor(VGA_BLUE);
				ILI9327_Line(80,0,80,150);
				ILI9327_Line(160,0,160,150);
				ILI9327_Line(0,50,240,50);
				ILI9327_Line(0,100,240,100);
				ILI9327_Line(0,150,240,150);

				ILI9327_Rect(0, 160, 80, 210);
				ILI9327_Rect(80, 160, 160, 210);
				ILI9327_Rect(160, 160, 239, 210);

				ILI9327_Rect(0, 220, 80, 270);
				ILI9327_Rect(80, 220, 160, 270);
				ILI9327_Rect(160, 220, 239, 270);

				ILI9327_Rect(120, 295, 239, 350);


				ILI9327_setColor(VGA_BLACK);
				ILI9327_String(20,20,"X-Y+");
				ILI9327_String(100,20,"Y+");
				ILI9327_String(180,20,"X+Y+");

				ILI9327_String(20,70,"X-");
				ILI9327_String(100,70,"X0Y0");
				ILI9327_String(180,70,"X+");

				ILI9327_String(20,120,"X-Y-");
				ILI9327_String(100,120,"Y-");
				ILI9327_String(180,120,"X+Y-");

				ILI9327_String(130,320,"START");

				ILI9327_String(5,180,"P1(CT)");
				ILI9327_String(85,180,"P2(PRG)");
				ILI9327_String(165,180,"P3(D55)");

				ILI9327_String(5,240,"P4(C40)");
				ILI9327_String(85,240,"P5(C5)");
				ILI9327_String(165,240,"P6(SPR)");


				ILI9327_NumWDesc(20,320,"PROGRAM: ", progPtr);



//extern volatile uint32_t ptrGMoveCurrentCom, ptrGMoveEndCom;
//extern volatile GCommandMoveTypeDef GMoveBuffer[GMOVE_BUFFER_S]; //buffer for variables per each g-command

//extern volatile uint32_t ptrMoveCurrent[AXIS_NUM], ptrMoveEnd[AXIS_NUM];
//extern volatile CoordSquareTypeDef squares[COORD_BUFFER_S];

//extern CoordTypeDef CNCCoordinates;


				for(uint8_t i=0; i<AXIS_NUM; i++)
				{
//					ILI9327_setColor(VGA_RED);
//									ILI9327_setBackColor(VGA_WHITE);
//					ILI9327_NumWDesc(i*60,85,"S: ", ptrMoveCurrent[i]);
//					ILI9327_NumWDesc(i*60,95,"E: ", ptrMoveEnd[i]);
//
//					ILI9327_setColor(VGA_GREEN);
//					ILI9327_setBackColor(VGA_BLACK);
//					ILI9327_NumWDesc(i * 60, 105, ">:",
//GMoveBuffer[ptrGMoveCurrentCom].GCommandEndPos.step[i] - GMoveBuffer[ptrGMoveCurrentCom].Delta.step[i]);
//
//					ILI9327_NumWDesc(i * 60, 115, "^:", CNCCoordinates.step[i]);
//					ILI9327_NumWDesc(i * 60, 125, "<:", GMoveBuffer[ptrGMoveCurrentCom].GCommandEndPos.step[i]);
//
//					ILI9327_NumWDesc(i * 60, 140, "D:", GMoveBuffer[ptrGMoveCurrentCom].Delta.step[i]);
//
//					ILI9327_setColor(VGA_BLUE);
//					ILI9327_NumWDesc(i*60,70,"R:", CalcBufferRemain(i));
				}

extern CoordMoveTypeDef coordinates[COORD_BUFFER_S];
extern volatile uint32_t ptrMoveCurrent[AXIS_NUM], ptrMoveEnd[AXIS_NUM];

//				for(uint8_t i=0; i<ptrMoveEnd[0]; i++)
//				{
//					ILI9327_NumWDesc(100,160+i*9,"X:",coordinates[i].step[0]);
//				}
//				for(uint8_t i=0; i<ptrMoveEnd[1]; i++)
//				{
//					ILI9327_NumWDesc(150,160+i*9,"Y:",coordinates[i].step[1]);
//				}
////
//				for(uint8_t i=0; i<ptrMoveEnd[1]; i++)
//				{
//
//					ILI9327_NumWDesc(180,160+i*9,"Y:",squares[i].square[1]);
//				}

//				ILI9327_NumWDesc(0,200,"R2:",GMoveBuffer[ptrGMoveCurrentCom].SqRCoef[0]);
//				ILI9327_NumWDesc(0,190,":",fuckfuck);
//				ILI9327_NumWDesc(0,180,":",fuckin2);
//				ILI9327_NumWDesc(0,220,"S2:",GMoveBuffer[ptrGMoveCurrentCom].GCommandCurrPos.step[0]);

				//heartbeat
ILI9327_setFont(SmallFont);
ILI9327_setBackColor(VGA_WHITE);

				ILI9327_setColor(VGA_BLUE);
				ILI9327_NumWDesc(0, 360, "X: ", CNCCoordinates.step[0]);
				ILI9327_FNumWDesc(0, 375, "X> ", (float)CNCCoordinates.step[0]/X_1MM, 2);
				ILI9327_setColor(VGA_BLACK);
				ILI9327_NumWDesc(100, 360, "Y: ", CNCCoordinates.step[1]);
				ILI9327_FNumWDesc(100, 375, "Y> ", (float)CNCCoordinates.step[1]/X_1MM, 2);

				ILI9327_setColor(VGA_BLUE);
				ILI9327_NumWDesc(100,388,"Wait: ", uxQueueMessagesWaiting(GCodeQueue));
				ILI9327_Num(210,388, yyy++);
			}
			xSemaphoreGive(xLEDMutex);

			taskYIELD();

			vTaskDelay(10);
		}


		if(uxQueueMessagesWaiting(GCodeQueue) == 0)
		{
			vTaskDelay(20);
			fuckBuffer[1] ++;

			static uint8_t eeeee = 0;

//			if(eeeee % 4 == 0)
//				ILI9327_Fill(VGA_WHITE);
//			else if(eeeee % 4 == 1)
//				ILI9327_Fill(VGA_GREEN);
//			else if(eeeee % 4 == 2)
//				ILI9327_Fill(VGA_BLUE);
//			else if(eeeee % 4 == 3)
//			{
//				ILI9327_Fill(VGA_PURPLE);
//				eeeee = 255;
//			}

			eeeee++;

			//rrr();
		}

backtimer = 0;

	}
}

void Flashing(xTimerHandle xTimer)
{
	//static bool flag = FALSE;

/*	xSemaphoreTake(xLEDMutex, portMAX_DELAY);

	if(flag)
	{
		ILI9327_setColor(VGA_BLACK);
		flag = FALSE;
	}
	else
	{
		ILI9327_setColor(VGA_WHITE);
		flag = TRUE;
	}
	ILI9327_Char('_', x*8, y*13);

	 xSemaphoreGive(xLEDMutex);*/
}


TaskHandle_t xMoveHandle;
/*******************************************************************************
* Function Name  : main
*******************************************************************************/
int main()
{
	for(uint8_t i=0; i<110; i++)
		fuckBuffer[i] = 0;

	InitPeriph();
	SystemInit();

	PWM_BL_Init();
	ILI9327_Init();
	ILI9327_Clear();
	ILI9327_Fill(VGA_BLACK);

	XPT2046_Init();

	EXTI_InitTypeDef EXTI_InitStruct;
	EXTI_InitStruct.EXTI_Line = EXTI_Line13;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_Init(&EXTI_InitStruct);

#if 0
		GPIO_InitTypeDef GPIO_InitStruct; // this is for the GPIO pins used as TX and RX
		USART_InitTypeDef USART_InitStruct; // this is for the USART1 initilization
		//NVIC_InitTypeDef NVIC_InitStructure; // this is used to configure the NVIC (nested vector interrupt controller)

		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

		GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init(GPIOA, &GPIO_InitStruct);

		GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

		USART_InitStruct.USART_BaudRate = 115200*3;
		USART_InitStruct.USART_WordLength = USART_WordLength_8b;
		USART_InitStruct.USART_StopBits = USART_StopBits_1;
		USART_InitStruct.USART_Parity = USART_Parity_No;
		USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_InitStruct.USART_Mode = USART_Mode_Tx/* | USART_Mode_Rx*/;
		USART_Init(USART2, &USART_InitStruct);
//		USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
//
//		NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
//		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//		NVIC_Init(&NVIC_InitStructure);

		// finally this enables the complete USART1 peripheral
		USART_Cmd(USART2, ENABLE);
#endif

	//tFlashLine = xTimerCreate("FlashLine", (100/portTICK_RATE_MS), pdTRUE, 0, Flashing);
	xTimerReset(tFlashLine, 0);

	xLEDMutex = xSemaphoreCreateMutex();


	GCodeQueue = xQueueCreate(/*GCODE_QUEUE_SIZE*/20, sizeof(GCodeTypeDef));

    //xTaskCreate(vLedTask, "LedTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
    //xTaskCreate(vDisplayTask, "DisplayTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);

    xTaskCreate(vTouchDispatcher, "vTouch", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(vMoveTask, "vMove", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 4, &xMoveHandle);
    xTaskCreate(vControl, "vControl", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 3, NULL);

	/* Add IRQ vector to NVIC */
	EXTI_ClearITPendingBit(EXTI_Line13);



    vTaskStartScheduler();

	/*static portBASE_TYPE xHigherPriorityTaskWoken;
		xHigherPriorityTaskWoken = pdFALSE;

		xSemaphoreGiveFromISR(xSemaphore_Wait, &xHigherPriorityTaskWoken );
		if(xHigherPriorityTaskWoken == pdTRUE)
		{
			taskYIELD();
		}*/

    return 0;
}


extern volatile GCommandMoveTypeDef GMoveBuffer[GMOVE_BUFFER_S];
extern volatile uint32_t ptrGMoveCurrentCom, ptrGMoveEndCom;
extern uint32_t multSpeed;

void assert_failed(uint8_t* file, uint32_t line)
{
	vTaskEndScheduler();

	GPIO_ResetBits(GPIOE, GPIO_Pin_4); //EN

	//goto rre;

	ILI9327_Fill(VGA_RED);
	ILI9327_setColor(VGA_WHITE);
	ILI9327_setBackColor(VGA_RED);
	ILI9327_setFont(SmallFont);


	ILI9327_NumWDesc(0, 0, "RTS: ", GMoveBuffer[ptrGMoveCurrentCom].comRealTotalSteps);
	ILI9327_NumWDesc(0, 10, "RCS: ", GMoveBuffer[ptrGMoveCurrentCom].comRealCurrentStep);

	ILI9327_setColor(VGA_SILVER);
	ILI9327_NumWDesc(0, 25, "TS: ", GMoveBuffer[ptrGMoveCurrentCom].comTotalSteps);
	ILI9327_NumWDesc(0, 35, "CS: ", GMoveBuffer[ptrGMoveCurrentCom].comCurrentStep);

	ILI9327_setColor(VGA_GREEN);
	ILI9327_NumWDesc(0, 380, "X: ", CNCCoordinates.step[0]);
	ILI9327_setColor(VGA_WHITE);
	ILI9327_NumWDesc(0, 390, "Y: ", CNCCoordinates.step[1]);

	ILI9327_NumWDesc(0, 50, "curGCom: ", ptrGMoveCurrentCom);
	ILI9327_NumWDesc(0, 60, "endGCom: ", ptrGMoveEndCom);
	ILI9327_NumWDesc(0, 75, "mult: ", multSpeed);

	ILI9327_NumWDesc(0, 85, "Ncom: ", fuckBuffer[18]);

	//ILI9327_String(0,200,(char*)file);
	ILI9327_setColor(VGA_YELLOW);
	ILI9327_NumWDesc(80,200,"Line:", line);

	weAreInAss = 0;
//rre:
	while(1)
	{
		if(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13) == 1)
		{
			if(weAreInAss == 1)
				NVIC_SystemReset();
		}
		else
		{
			weAreInAss = 1;
		}
	}
}


///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#if 0

#define PWM_TIMER	TIM5

#define DMA_STREAM	DMA1_Stream2
#define DMA_TCIF	DMA_FLAG_TCIF2
#define DMA_CHANNEL	DMA_Channel_5
#define DMA_SOURCE	TIM_DMA_Update


#define TIM_PERIOD			100
//#define TIM_COMPARE_HIGH	10
//#define TIM_COMPARE_LOW		20
#define TIM_COMPARE_HIGH	100
#define TIM_COMPARE_LOW		4

#include "stm32f4xx_dma.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"


uint16_t LED_BYTE_Buffer[50];
uint16_t LED_BYTE_Buffer2[50];




void xxxxxx()
{
	//uint16_t PrescalerValue;

	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	/* GPIOB Configuration: PWM_TIMER Channel 1 as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_TIM3);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);


	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	/* Compute the prescaler value */
	RCC_ClocksTypeDef clocks;
	RCC_GetClocksFreq(&clocks);
	//PrescalerValue = (uint16_t)(/*SystemCoreClock/10000*/10) - 1;
	/* Time base configuration */
	TIM_TimeBaseInitTypeDef 	TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Period = /*TIM_PERIOD*/60000; // 800kHz
	TIM_TimeBaseStructure.TIM_Prescaler = /*PrescalerValue*/4;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(PWM_TIMER, &TIM_TimeBaseStructure);

	/* PWM1 Mode configuration: Channel1 */
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Toggle;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 2;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(PWM_TIMER, &TIM_OCInitStructure);

	/***
	 * Must enable reload for PWM (STMicroelectronicd RM0090 section 18.3.9
	 * "PWM mode":
	 *
	 *   You must enable the corresponding preload register by setting the
	 *   OCxPE bit in the TIMx_CCMRx register.
	 *
	 * This is part of the fix for the pulse corruption (the runt pulse at
	 * the start and the extra pulse at the end).
	 */
	//TIM_OC1PreloadConfig(PWM_TIMER, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(PWM_TIMER, ENABLE);

	/* configure DMA */
	/* DMA clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

	/* DMA1 Channel6 Config */
	DMA_DeInit(DMA_STREAM);
	DMA_InitTypeDef DMA_InitStructure;

	DMA_InitStructure.DMA_BufferSize = 50;
	DMA_InitStructure.DMA_Channel = DMA_CHANNEL;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;					// data shifted from memory to peripheral
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;

	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) &LED_BYTE_Buffer;		// this is the buffer memory
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;					// automatically increase buffer index
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;

	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;/*DMA_Mode_Normal;*/						// stop DMA feed after buffer size is reached

	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &/*PWM_TIMER->CCR1*/PWM_TIMER->ARR;	// physical address of Timer 3 CCR1
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

	DMA_InitStructure.DMA_Priority = DMA_Priority_High;

	/* DoubleBufferMode enable*/


	DMA_Init(DMA_STREAM, &DMA_InitStructure);

	DMA_DoubleBufferModeConfig(DMA_STREAM, (uint32_t) &LED_BYTE_Buffer, (uint32_t) &LED_BYTE_Buffer2);
	DMA_DoubleBufferModeCmd(DMA_STREAM, ENABLE);


//	 TIM_DMACmd(TIM2, TIM_DMA_Update, ENABLE);
//
//
//	/* PWM_TIMER CC1 DMA Request enable */
	TIM_DMACmd(PWM_TIMER, DMA_SOURCE, ENABLE);


	for(uint16_t i=0; i<50; i++)
	{
		LED_BYTE_Buffer2[i] = TIM_COMPARE_HIGH;
		LED_BYTE_Buffer[i] = TIM_COMPARE_LOW;
	}

	LED_BYTE_Buffer[0] = 1;
	LED_BYTE_Buffer[10] = 35;

//XX:
taskENTER_CRITICAL();
TIM_SetCounter(PWM_TIMER, 0);
	DMA_SetCurrDataCounter(DMA_STREAM, 10); 	// load number of bytes to be transferred

	// PAP: Clear the timer's counter and set the compare value to 0. This
	// sets the output low on start and gives us a full cycle to set up DMA.
	//
	TIM_SetCompare1(PWM_TIMER, 0);
	TIM_Cmd(PWM_TIMER, ENABLE); 						// enable Timer 3

	// PAP: Start DMA transfer after starting the timer. This prevents the
	// DMA/PWM from dropping the first bit.
	DMA_Cmd(DMA_STREAM, ENABLE); 			// enable DMA channel 6
//	while(!DMA_GetFlagStatus(DMA_STREAM, DMA_TCIF)); 	// wait until transfer complete
//	TIM_Cmd(PWM_TIMER, DISABLE); 					// disable Timer 3
//	DMA_Cmd(DMA_STREAM, DISABLE); 			// disable DMA channel 6
//	DMA_ClearFlag(DMA_STREAM, DMA_TCIF); 				// clear DMA1 Channel 6 transfer complete flag


	//vTaskDelay(2);
	GPIO_SetBits(GPIOB, GPIO_Pin_5);

	while(!DMA_GetFlagStatus(DMA_STREAM, DMA_TCIF)); 	// wait until transfer complete
	DMA_Cmd(DMA_STREAM, DISABLE); 			// disable DMA channel 6
	GPIO_SetBits(GPIOB, GPIO_Pin_7);

	DMA_ClearFlag(DMA_STREAM, DMA_TCIF); 				// clear DMA1 Channel 6 transfer complete flag
	TIM_Cmd(PWM_TIMER, DISABLE); 						// enable Timer 3

	taskEXIT_CRITICAL();

	vTaskDelay(2);
	GPIO_ResetBits(GPIOB, GPIO_Pin_5|GPIO_Pin_7);
	//goto XX;
	while(1);

	//goto XX;
}

#endif
