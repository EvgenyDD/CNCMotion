/* Includes ------------------------------------------------------------------*/
#include "ILI9327.h"
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "system_stm32f4xx.h"

#include "stm32f4xx_dma.h"
#include "stm32f4xx_flash.h"
#include "stm32f4xx_fsmc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_iwdg.h"
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
#include "semphr.h"
#include "task.h"
#include "queue.h"

#include "string.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Note: LCD /CS is NE1 - Bank 1 of NOR/SRAM Bank 1~4 */
#define LCD_BASE           ((uint32_t)(0x60000000 | 0x0001fffE))
#define LCD                ((LCD_TypeDef *) LCD_BASE)
#define LCD2                ((LCD_TypeDef2 *) LCD_BASE)
#define MAX_POLY_CORNERS   200
#define POLY_Y(Z)          ((int32_t)((Points + Z)->X))
#define POLY_X(Z)          ((int32_t)((Points + Z)->Y))


/* Private macro -------------------------------------------------------------*/
#define swap(type, i, j) {type t = i; i = j; j = t;}


/* Private variables ---------------------------------------------------------*/
uint16_t fillColor, backColor;
struct _current_font
{
	uint8_t* font;
	uint8_t x_size;
	uint8_t y_size;
	uint8_t offset;
	uint8_t numchars;
}cfont;

uint8_t orient;
uint8_t _transparent;

uint16_t disp_x_size = 239;
uint16_t disp_y_size = 399;

xQueueHandle  QueueHandle;
xSemaphoreHandle xSemaphore_Wait;


/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

void _delay_ms(uint16_t val){
	for(uint32_t i=val*100000; i!=0; i--);
}


/*******************************************************************************
* Function Name  :
* Description    :
* Input		 :
*******************************************************************************/

/*******************************************************************************
* Function Name  :
* Description    :
*******************************************************************************/
void PWM_BL_Init()
{
	/* PWM Backlight */
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //Enable port E clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); // Configure PE9 as output for PWM

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// Connect PB0 to Timer 3 channel 3
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3);

	/* Compute the prescaler value
	* Timer 1,8,9,10 and 11 are connected to APB2 which runs at SystemCoreClock
	* Other timers are connected to APB1 and runs at SystemCoreClock/2
	Prescaler = (TIM3CLK / TIM1 counter clock) - 1
	Prescaler = ((SystemCoreClock) /Frequency) - 1
	*/

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 100;
	TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t)((SystemCoreClock) / 100000) - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	/* PWM1 Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 100;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);

	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM3, ENABLE);

	TIM_Cmd(TIM3, ENABLE);
	//TIM_CtrlPWMOutputs(TIM3, ENABLE);
}



void FSMC_Init()
{
	/* FSMC */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE, ENABLE);
	RCC_AHB3PeriphClockCmd(RCC_AHB3Periph_FSMC, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5 |
			GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	/* Configure GPIO D pins as FSMC alternate functions */
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_FSMC);  // D2
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_FSMC);  // D3
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource4, GPIO_AF_FSMC);  // nOE - RD
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_FSMC);  // nWE - WR
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource7, GPIO_AF_FSMC);  // nE1 - CS
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_FSMC);  // D13
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_FSMC);  // D14
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource10, GPIO_AF_FSMC); // D10
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource11, GPIO_AF_FSMC); // DC
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_FSMC); // D0
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_FSMC); // D1

	/* Create GPIO E Init structure for used pin */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 |
			GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	/* Configure GPIO E pins as FSMC alternate functions */
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource7, GPIO_AF_FSMC);  // D4
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource8, GPIO_AF_FSMC);  // D5
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_FSMC);  // D6
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource10, GPIO_AF_FSMC); // D7
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_FSMC); // D8
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource12, GPIO_AF_FSMC); // D9
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_FSMC); // D10
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_FSMC); // D11
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource15, GPIO_AF_FSMC); // D12

	/* Configure Reset Pin */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	FSMC_NORSRAMInitTypeDef FSMC_NORSRAMInitStructure;
	FSMC_NORSRAMTimingInitTypeDef FSMC_NORSRAMTimingInitStructureRead;
	FSMC_NORSRAMTimingInitTypeDef FSMC_NORSRAMTimingInitStructureWrite;

	/* Define Read timing parameters */
	FSMC_NORSRAMTimingInitStructureRead.FSMC_AddressSetupTime = 3;//3
	FSMC_NORSRAMTimingInitStructureRead.FSMC_AddressHoldTime = 0;//0
	FSMC_NORSRAMTimingInitStructureRead.FSMC_DataSetupTime = 15;//15
	FSMC_NORSRAMTimingInitStructureRead.FSMC_BusTurnAroundDuration = 0;
	FSMC_NORSRAMTimingInitStructureRead.FSMC_CLKDivision = 0;
	FSMC_NORSRAMTimingInitStructureRead.FSMC_DataLatency = 3;
	FSMC_NORSRAMTimingInitStructureRead.FSMC_AccessMode = FSMC_AccessMode_A;

	/* Define Write Timing parameters */
	FSMC_NORSRAMTimingInitStructureWrite.FSMC_AddressSetupTime = 4;//4
	FSMC_NORSRAMTimingInitStructureWrite.FSMC_AddressHoldTime = 0;//0
	FSMC_NORSRAMTimingInitStructureWrite.FSMC_DataSetupTime = 5;//5
	FSMC_NORSRAMTimingInitStructureWrite.FSMC_BusTurnAroundDuration = 0;
	FSMC_NORSRAMTimingInitStructureWrite.FSMC_CLKDivision = 0;
	FSMC_NORSRAMTimingInitStructureWrite.FSMC_DataLatency = 3;
	FSMC_NORSRAMTimingInitStructureWrite.FSMC_AccessMode = FSMC_AccessMode_A;

	/* Define protocol type */
	FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM1; //Bank1
	FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable; //No mux
	FSMC_NORSRAMInitStructure.FSMC_MemoryType = FSMC_MemoryType_SRAM; //SRAM type
	FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b; //16 bits wide
	FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Disable; //No Burst
	FSMC_NORSRAMInitStructure.FSMC_AsynchronousWait = FSMC_AsynchronousWait_Disable; // No wait
	FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low; //Don'tcare
	FSMC_NORSRAMInitStructure.FSMC_WrapMode = FSMC_WrapMode_Disable; //No wrap mode
	FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState; //Don't care
	FSMC_NORSRAMInitStructure.FSMC_WriteOperation = FSMC_WriteOperation_Enable;
	FSMC_NORSRAMInitStructure.FSMC_WaitSignal = FSMC_WaitSignal_Disable; //Don't care
	FSMC_NORSRAMInitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Enable; //Allow distinct Read/Write parameters
	FSMC_NORSRAMInitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Disable; //Don't care

	// Set read timing structure
	FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &FSMC_NORSRAMTimingInitStructureRead;

	// Set write timing structure
	FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &FSMC_NORSRAMTimingInitStructureWrite;

	// Initialize FSMC for read and write
	FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure);

	// Enable FSMC
	FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM1, ENABLE);
}


void ILI9327_Init()
{
	portENTER_CRITICAL();
	FSMC_Init();

	GPIOE->BSRRL = GPIO_Pin_3;
    _delay_ms(5);
  	GPIOE->BSRRH = GPIO_Pin_3;
    _delay_ms(30);
  	GPIOE->BSRRL = GPIO_Pin_3;
    _delay_ms(5);


	/* Initialize LCD */
	ILI9327_WriteCom(0x11); 	//Exit Sleep
	_delay_ms(100);

	ILI9327_WriteCom(0xD1);		//VCOM Control
	ILI9327_WriteParam(0x00);
	ILI9327_WriteParam(0x71);
	ILI9327_WriteParam(0x19);

	ILI9327_WriteCom(0xD0);		//Power_Setting
	ILI9327_WriteParam(0x07);
	ILI9327_WriteParam(0x01);
	ILI9327_WriteParam(0x08);

	ILI9327_WriteCom(0x36);		//set_address_mode
	ILI9327_WriteParam(0x48);

	ILI9327_WriteCom(0x3A);		//set_pixel_format
	ILI9327_WriteParam(0x05);

	ILI9327_WriteCom(0xC1);		//Display_Timing_Setting for Normal/Partial Mode
	ILI9327_WriteParam(0x10);
	ILI9327_WriteParam(0x10);
	ILI9327_WriteParam(0x02);
	ILI9327_WriteParam(0x02);

	ILI9327_WriteCom(0xC5); 		//Set frame rate
	ILI9327_WriteParam(0x04);

	ILI9327_WriteCom(0xD2); 		//Power_Setting for Normal Mode
	ILI9327_WriteParam(0x01);
	ILI9327_WriteParam(0x44);

	ILI9327_WriteCom(0xC0); 		//Set Default Gamma
	ILI9327_WriteParam(0x00);
	ILI9327_WriteParam(0x35);
	ILI9327_WriteParam(0x00);
	ILI9327_WriteParam(0x00);
	ILI9327_WriteParam(0x01);
	ILI9327_WriteParam(0x02);

	ILI9327_WriteCom(0xC8); 		//Set Gamma
	ILI9327_WriteParam(0x04);
	ILI9327_WriteParam(0x67);
	ILI9327_WriteParam(0x35);
	ILI9327_WriteParam(0x04);
	ILI9327_WriteParam(0x08);
	ILI9327_WriteParam(0x06);
	ILI9327_WriteParam(0x24);
	ILI9327_WriteParam(0x01);
	ILI9327_WriteParam(0x37);
	ILI9327_WriteParam(0x40);
	ILI9327_WriteParam(0x03);
	ILI9327_WriteParam(0x10);
	ILI9327_WriteParam(0x08);
	ILI9327_WriteParam(0x80);
	ILI9327_WriteParam(0x00);

	ILI9327_WriteCom(0x29); 		//display on

	orient=PORTRAIT;
	//orient=LANDSCAPE;//

	vSemaphoreCreateBinary(xSemaphore_Wait);

	portEXIT_CRITICAL();
}

void ILI9327_WriteCom(uint16_t reg)
{
	LCD->LCD_REG = reg;
}

void ILI9327_WriteParam(uint16_t paramValue)
{
	LCD->LCD_RAM = paramValue;
}


void setXY(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
	if(orient==LANDSCAPE)
	{
		swap(uint16_t, x1, y1);
		swap(uint16_t, x2, y2);
		y1=disp_y_size-y1;
		y2=disp_y_size-y2;
		swap(uint16_t, y1, y2);
	}

	ILI9327_WriteCom(0x2a);		//set_column_address
	ILI9327_WriteParam(x1>>8);
	ILI9327_WriteParam(x1);
	ILI9327_WriteParam(x2>>8);
	ILI9327_WriteParam(x2);

	ILI9327_WriteCom(0x2b);		//set_page_address
	ILI9327_WriteParam(y1>>8);
	ILI9327_WriteParam(y1);
	ILI9327_WriteParam(y2>>8);
	ILI9327_WriteParam(y2);

	ILI9327_WriteCom(0x2c);		//write_memory_start
}

void clrXY()
{
	if(orient==PORTRAIT)
		setXY(0, 0, disp_x_size, disp_y_size);
	else
		setXY(0, 0, disp_y_size, disp_x_size);
}


void _fast_fill_16(uint16_t color, long pix)
{
	for(uint32_t i=0; i<pix; i++)
		LCD->LCD_RAM = color;
}
//w/ DMA = 14.3 ms
//w/o DMA = 33.9 ms
void _fast_fill_32(uint16_t color, long pix)
{
	for(uint32_t i=0; i<pix/2; i++)
		LCD2->LCD_RAM = (color<<16)|color;
}




/**
  * @brief  Writes to the selected LCD register.
  * @param  LCD_Reg: address of the selected register.
  * @param  LCD_RegValue: value to write to the selected register.
  * @retval None
  */
void LCD_WriteReg(uint8_t LCD_Reg, uint16_t LCD_RegValue)
{
  /* Write 16-bit Index, then Write Reg */
  LCD->LCD_REG = LCD_Reg;
  /* Write 16-bit Reg */
  LCD->LCD_RAM = LCD_RegValue;
}

/**
  * @brief  Reads the selected LCD Register.
  * @param  LCD_Reg: address of the selected register.
  * @retval LCD Register Value.
  */
uint16_t LCD_ReadReg(uint8_t LCD_Reg)
{
	/* Write 16-bit Index (then Read Reg) */
	LCD->LCD_REG = LCD_Reg;
	/* Read 16-bit Reg */
	return (LCD->LCD_RAM);
}


/**
  * @brief  Writes to the LCD RAM.
  * @param  RGB_Code: the pixel color in RGB mode (5-6-5).
  * @retval None
  */
void LCD_WriteRAM(uint16_t RGB_Code)
{
	/* Write 16-bit GRAM Reg */
	LCD->LCD_RAM = RGB_Code;
}

/**
  * @brief  Reads the LCD RAM.
  * @param  None
  * @retval LCD RAM Value.
  */
uint16_t LCD_ReadRAM(void)
{
	/* Write 16-bit Index (then Read Reg) */
	LCD->LCD_REG = 0;//LCD_REG_34; /* Select GRAM Reg */
	/* Read 16-bit Reg */
	return LCD->LCD_RAM;
}








void ILI9327_Fill(uint16_t color)
{
	clrXY();
//	_fast_fill_16(color, (disp_x_size+1)*(disp_y_size+1));
	_fast_fill_32(color, (disp_x_size+1)*(disp_y_size+1));
}


void ILI9327_FillRGB(uint8_t r, uint8_t g, uint8_t b)
{
	uint16_t color = ASSEMBLE_RGB(r, g, b);
	ILI9327_Fill(color);
}

void ILI9327_Clear()
{
	clrXY();
	//_fast_fill_16(0, (disp_x_size+1)*(disp_y_size+1));
	_fast_fill_32(0, (disp_x_size+1)*(disp_y_size+1));
}

void ILI9327_Line(int x1, int y1, int x2, int y2)
{
	if (y1==y2)
		ILI9327_HLine(x1, y1, x2-x1);
	else if (x1==x2)
		ILI9327_VLine(x1, y1, y2-y1);
	else
	{
		unsigned int	dx = (x2 > x1 ? x2 - x1 : x1 - x2);
		short			xstep =  x2 > x1 ? 1 : -1;
		unsigned int	dy = (y2 > y1 ? y2 - y1 : y1 - y2);
		short			ystep =  y2 > y1 ? 1 : -1;
		int				color = x1, row = y1;

		if (dx < dy)
		{
			int t = - (dy >> 1);
			while (ENABLE)
			{
				setXY (color, row, color, row);
				LCD->LCD_RAM = fillColor;
				if (row == y2)
					return;
				row += ystep;
				t += dx;
				if (t >= 0)
				{
					color += xstep;
					t   -= dy;
				}
			}
		}
		else
		{
			int t = - (dx >> 1);
			while (ENABLE)
			{
				setXY (color, row, color, row);
				LCD->LCD_RAM = fillColor;
				if (color == x2)
					return;
				color += xstep;
				t += dy;
				if (t >= 0)
				{
					row += ystep;
					t   -= dx;
				}
			}
		}
	}
	clrXY();
}

void ILI9327_HLine(int x, int y, int l)
{
	if (l<0)
	{
		l = -l;
		x -= l;
	}

	setXY(x, y, x+l, y);
	_fast_fill_16(fillColor, l);
	//clrXY();
}

void ILI9327_VLine(int x, int y, int l)
{
	if (l<0)
	{
		l = -l;
		y -= l;
	}

	setXY(x, y, x, y+l);
	_fast_fill_16(fillColor,l);
	//clrXY();
}

void ILI9327_Rect(int x1, int y1, int x2, int y2)
{
	if(x1>x2)
		swap(int, x1, x2);
	if(y1>y2)
		swap(int, y1, y2);

	ILI9327_HLine(x1, y1, x2-x1);
	ILI9327_HLine(x1, y2, x2-x1);
	ILI9327_VLine(x1, y1, y2-y1);
	ILI9327_VLine(x2, y1, y2-y1);
}

void drawRoundRect(int x1, int y1, int x2, int y2)
{
	if(x1>x2)
		swap(int, x1, x2);
	if(y1>y2)
		swap(int, y1, y2);

	if((x2-x1)>4 && (y2-y1)>4)
	{
		//CS_L;
			setXY(x1+1, y1+1, x1+1, y1+1);
			ILI9327_Pixel(fillColor);
		//CS_H;
		//clrXY();
		//CS_L;
			setXY(x2-1, y1+1, x2-1, y1+1);
			ILI9327_Pixel(fillColor);
		//CS_H;
		//clrXY();
		//CS_L;
			setXY(x1+1, y2-1, x1+1, y2-1);
			ILI9327_Pixel(fillColor);
		//CS_H;
		//clrXY();
		//CS_L;
			setXY(x2-1, y2-1, x2-1, y2-1);
			ILI9327_Pixel(fillColor);
		//CS_H;
		//clrXY();

		ILI9327_HLine(x1+2, y1, x2-x1-4);
		ILI9327_HLine(x1+2, y2, x2-x1-4);
		ILI9327_VLine(x1, y1+2, y2-y1-4);
		ILI9327_VLine(x2, y1+2, y2-y1-4);
	}
}

void ILI9327_RectFill(int x1, int y1, int x2, int y2)
{
	if(x1>x2)
		swap(int, x1, x2);

	if(y1>y2)
		swap(int, y1, y2);

	setXY(x1, y1, x2, y2);
	_fast_fill_16(fillColor, (x2-x1+1)*(y2-y1+1));
}

void fillRoundRect(int x1, int y1, int x2, int y2)
{
	if(x1>x2)
		swap(int, x1, x2);

	if(y1>y2)
		swap(int, y1, y2);

	if((x2-x1)>4 && (y2-y1)>4)
	{
		for(int i=0; i<((y2-y1)/2)+1; i++)
		{
			switch(i)
			{
			case 0:
				ILI9327_HLine(x1+2, y1+i, x2-x1-4);
				ILI9327_HLine(x1+2, y2-i, x2-x1-4);
				break;
			case 1:
				ILI9327_HLine(x1+1, y1+i, x2-x1-2);
				ILI9327_HLine(x1+1, y2-i, x2-x1-2);
				break;
			default:
				ILI9327_HLine(x1, y1+i, x2-x1);
				ILI9327_HLine(x1, y2-i, x2-x1);
			}
		}
	}
}

void ILI9327_Circle(int x, int y, int radius)
{
	int f = 1 - radius;
	int ddF_x = 1;
	int ddF_y = -2 * radius;
	int x1 = 0;
	int y1 = radius;

	setXY(x, y + radius, x, y + radius);
	LCD->LCD_RAM = fillColor;
	setXY(x, y - radius, x, y - radius);
	LCD->LCD_RAM = fillColor;
	setXY(x + radius, y, x + radius, y);
	LCD->LCD_RAM = fillColor;
	setXY(x - radius, y, x - radius, y);
	LCD->LCD_RAM = fillColor;

	while(x1 < y1)
	{
		if(f >= 0)
		{
			y1--;
			ddF_y += 2;
			f += ddF_y;
		}

		x1++;
		ddF_x += 2;
		f += ddF_x;

		setXY(x + x1, y + y1, x + x1, y + y1);
		LCD->LCD_RAM = fillColor;
		setXY(x - x1, y + y1, x - x1, y + y1);
		LCD->LCD_RAM = fillColor;
		setXY(x + x1, y - y1, x + x1, y - y1);
		LCD->LCD_RAM = fillColor;
		setXY(x - x1, y - y1, x - x1, y - y1);
		LCD->LCD_RAM = fillColor;
		setXY(x + y1, y + x1, x + y1, y + x1);
		LCD->LCD_RAM = fillColor;
		setXY(x - y1, y + x1, x - y1, y + x1);
		LCD->LCD_RAM = fillColor;
		setXY(x + y1, y - x1, x + y1, y - x1);
		LCD->LCD_RAM = fillColor;
		setXY(x - y1, y - x1, x - y1, y - x1);
		LCD->LCD_RAM = fillColor;
	}
	//clrXY();
}

void ILI9327_CircleFill(int x, int y, int radius)
{
	for(int y1=-radius; y1<=0; y1++)
		for(int x1=-radius; x1<=0; x1++)
			if(x1*x1+y1*y1 <= radius*radius)
			{
				ILI9327_HLine(x+x1, y+y1, 2*(-x1));
				ILI9327_HLine(x+x1, y-y1, 2*(-x1));
				break;
			}
}


void ILI9327_setColorRGB(uint8_t r, uint8_t g, uint8_t b){
	fillColor = ASSEMBLE_RGB(r, g, b);
}

void ILI9327_setColor(uint16_t color){
	fillColor = color;
}

void ILI9327_setBackColorRGB(uint8_t r, uint8_t g, uint8_t b)
{
	backColor = ASSEMBLE_RGB(r, g, b);
	_transparent = DISABLE;
}

void ILI9327_setBackColor(uint32_t color)
{
	if(color == VGA_TRANSPARENT)
		_transparent = ENABLE;
	else
	{
		backColor = color;
		_transparent = DISABLE;
	}
}

void ILI9327_Pixel(uint16_t color){
	LCD->LCD_RAM = color;
}

void ILI9327_Char(char c, int x, int y)
{
	char colorH;
	uint16_t temp;

	if(!_transparent)
	{
		if(orient==PORTRAIT)
		{
			setXY(x,y,x+cfont.x_size-1,y+cfont.y_size-1);

			temp = ((c-cfont.offset)*((cfont.x_size/8)*cfont.y_size))+4;

			for(uint16_t j=0; j<((cfont.x_size/8)*cfont.y_size); j++)
			{
				colorH = cfont.font[temp];
				for(uint8_t i=0; i<8; i++)
				{
					if((colorH & (1<<(7-i))) != 0)
						ILI9327_Pixel(fillColor);
					else
						ILI9327_Pixel(backColor);
				}
				temp++;
			}
		}
		else
		{
			temp = ((c-cfont.offset)*((cfont.x_size/8)*cfont.y_size))+4;

			for(uint16_t j=0; j<((cfont.x_size/8)*cfont.y_size); j+=(cfont.x_size/8))
			{
				setXY(x,y+(j/(cfont.x_size/8)),x+cfont.x_size-1,y+(j/(cfont.x_size/8)));

				for(int zz=(cfont.x_size/8)-1; zz>=0; zz--)
				{
					colorH = cfont.font[temp+zz];
					for(uint8_t i=0; i<8; i++)
					{
						if((colorH & (1<<i)) != 0)
							ILI9327_Pixel(fillColor);
						else
							ILI9327_Pixel(backColor);
					}
				}
				temp+=(cfont.x_size/8);
			}
		}
	}
	else
	{
		temp = ((c-cfont.offset)*((cfont.x_size/8)*cfont.y_size))+4;
		for(uint16_t j=0; j<cfont.y_size; j++)
		{
			for(int zz=0; zz<(cfont.x_size/8); zz++)
			{
				colorH = cfont.font[temp+zz];
				for(uint8_t i=0; i<8; i++)
				{
					setXY(x+i+(zz*8), y+j, x+i+(zz*8)+1, y+j+1);

					if((colorH & (1<<(7-i))) != 0)
						ILI9327_Pixel(fillColor);
				}
			}
			temp += (cfont.x_size/8);
		}
	}

	clrXY();
}

void ILI9327_setFont(uint8_t* font)
{
	cfont.font = font;
	cfont.x_size = cfont.font[0];
	cfont.y_size = cfont.font[1];
	cfont.offset = cfont.font[2];
	cfont.numchars = cfont.font[3];
}

uint8_t* getFont(){
	return cfont.font;
}

uint8_t getFontXsize(){
	return cfont.x_size;
}

uint8_t getFontYsize(){
	return cfont.y_size;
}

int getDisplayXSize(){
	return (orient==PORTRAIT)?(disp_x_size+1):(disp_y_size+1);
}

int getDisplayYSize(){
	return (orient==PORTRAIT)?(disp_y_size+1):(disp_x_size+1);
}

void ILI9327_Bitmap(int x, int y, int sx, int sy, unsigned int* data, int scale)
{
	uint16_t color;

	if(scale==1) //default
	{
		if (orient==PORTRAIT)
		{
			setXY(x, y, x+sx-1, y+sy-1);
			for(int tc=0; tc<(sx*sy); tc++)
			{
				color = data[tc];
				LCD->LCD_RAM = color;
			}
		}
		else
		{
			for(int ty=0; ty<sy; ty++)
			{
				setXY(x, y+ty, x+sx-1, y+ty);
				for(int tx=sx; tx>=0; tx--)
				{
					color = data[(ty*sx)+tx];
					LCD->LCD_RAM = color;
				}
			}
		}
	}
	else
	{
		if(orient==PORTRAIT)
		{
			for(int ty=0; ty<sy; ty++)
			{
				setXY(x, y+(ty*scale), x+((sx*scale)-1), y+(ty*scale)+scale);
				for(int tsy=0; tsy<scale; tsy++)
				{
					for(int tx=0; tx<sx; tx++)
					{
						color = data[(ty*sx)+tx];
						for(int tsx=0; tsx<scale; tsx++)
							LCD->LCD_RAM = color;
					}
				}
			}
		}
		else
		{
			for(int ty=0; ty<sy; ty++)
			{
				for(int tsy=0; tsy<scale; tsy++)
				{
					setXY(x, y+(ty*scale)+tsy, x+((sx*scale)-1), y+(ty*scale)+tsy);

					for(int tx=sx; tx>=0; tx--)
					{
						color = data[(ty*sx)+tx];
						for(int tsx=0; tsx<scale; tsx++)
							LCD->LCD_RAM = color;
					}
				}
			}
		}
	}
	clrXY();
}


void ILI9327_String(int x, int y, char *s)
{
	int firstX = x;
	while(*s != '\0')
	{
		ILI9327_Char(*s, x, y);
		x = x /*+ 2*/ + cfont.x_size;
		s++;
	}
	uint16_t memClr = fillColor;
	ILI9327_setColor(backColor);
	ILI9327_RectFill(x, y, x+cfont.x_size*2, y+cfont.y_size);
	ILI9327_setColor(memClr);
}


void ILI9327_Num(int x, int y, int32_t num)
{
	char d[25];
	itoa_(num, d);
	ILI9327_String(x, y, d);
}

void ILI9327_NumWDesc(int x, int y, char *s, int32_t num)
{
	char o[strlen(s)+15], d[25];
	o[0] = '\0';
	strcat_(o, s);
	itoa_(num, d);
	strcat_(o, d);
	ILI9327_String(x, y, o);
}

void ILI9327_FNumWDesc(int x, int y, char *s, float num, uint8_t precision)
{
	char o[strlen(s)+15], d[25];
	o[0] = '\0';
	strcat_(o, s);
	ftoa_(num, d, precision);
	strcat_(o, d);
	ILI9327_String(x, y, o);
}

