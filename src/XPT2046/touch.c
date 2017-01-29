/* Includes ------------------------------------------------------------------*/
#include "touch.h"

#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "system_stm32f4xx.h"

#include "stm32f4xx_adc.h"
#include "stm32f4xx_flash.h"
#include "stm32f4xx_fsmc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_iwdg.h"
#include "stm32f4xx_pwr.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_rng.h"
#include "stm32f4xx_sdio.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_usart.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "croutine.h"
#include "portmacro.h"
#include "task.h"
#include "queue.h"

#include "ILI9327.h"
#include "string.h"

extern uint8_t SmallFont[];
extern uint8_t BigFont[];
extern uint8_t SevenSegNumFont[];
extern uint8_t font5x8[];
#define abs(x) ((x) >= 0 ? (x) : -(x))

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define X1	GPIO_Pin_2
#define X2 	GPIO_Pin_4
#define Y1	GPIO_Pin_1
#define Y2	GPIO_Pin_5


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
int32_t xd[3], yd[3], xt[3], yt[3], xc[3], yc[3];

PointType TouchPoint;
struct{
	float A;
	float B;
	float C;
	float D;
	float E;
	float F;
}Calibr;




/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

unsigned char SPI_RW_Byte(u8 num)
{
	//unsigned char Data = 0;
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)==RESET);
	SPI_I2S_SendData(SPI1, num);
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE)==RESET);
	return SPI_I2S_ReceiveData(SPI1);
}

void spiExchange(u8 num, u8 *tx, u8 *rx)
{
	for(uint8_t i=0; i<num; i++)
		*(rx+i) = SPI_RW_Byte(*(tx+i));
}


u16 XPT2046_GetData( u8 control )
{
	u8 tData[3] = { control , 0 , 0 };
	u8 rData[3] = { 0 , 0 , 0 };

#if SPI_USE_MUTUAL_EXCLUSION
	spiAcquireBus( &( XPT2046_SPI_DRIVER ) );
#endif

	//palClearPad( XPT2046_NSS_PORT , XPT2046_NSS_PAD );
	GPIO_ResetBits(XPT2046_NSS_PORT, XPT2046_NSS_PAD);
	spiExchange(3 , tData , rData);
	GPIO_SetBits(XPT2046_NSS_PORT, XPT2046_NSS_PAD);
	//palSetPad( XPT2046_NSS_PORT , XPT2046_NSS_PAD );

#if SPI_USE_MUTUAL_EXCLUSION
	spiReleaseBus( &( XPT2046_SPI_DRIVER ) );
#endif

	if ( ( control & 0x08 ) == 0 ) {
		return ( rData[1] << 5 ) | ( rData[2] >> 3 );
	}
	return ( rData[1] << 4 ) | ( rData[2] >> 4 );
}



void XPT2046_Init()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	// Initialise SPI
	SPI_InitTypeDef SPIInitStructure;
	SPIInitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPIInitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPIInitStructure.SPI_Mode = SPI_Mode_Master;
	SPIInitStructure.SPI_NSS = SPI_NSS_Soft;
	SPIInitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPIInitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
	SPIInitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPIInitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPIInitStructure.SPI_CRCPolynomial = 0x1;
	SPI_Init(SPI1, &SPIInitStructure);

	GPIO_PinAFConfig(GPIOA,GPIO_PinSource5,GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_SPI1);

	// NSS signal
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin = XPT2046_NSS_PAD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(XPT2046_NSS_PORT, &GPIO_InitStructure);
	//palSetPadMode( XPT2046_NSS_PORT , XPT2046_NSS_PAD ,	PAL_MODE_OUTPUT_PUSHPULL );
	//palSetPad( XPT2046_NSS_PORT , XPT2046_NSS_PAD );

	// Main SPI signals
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Pin = XPT2046_CLK_PAD | XPT2046_DIN_PAD | XPT2046_DOUT_PAD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(XPT2046_CLK_PORT, &GPIO_InitStructure);
	/*palSetPadMode( XPT2046_CLK_PORT , XPT2046_CLK_PAD ,	PAL_MODE_ALTERNATE( 5 ) );
	palSetPadMode( XPT2046_DIN_PORT , XPT2046_DIN_PAD ,	PAL_MODE_ALTERNATE( 5 ) );
	palSetPadMode( XPT2046_DOUT_PORT, XPT2046_DOUT_PAD, PAL_MODE_ALTERNATE( 5 ) );*/

	// PENIRQ signal
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin = XPT2046_IRQ_PAD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(XPT2046_IRQ_PORT, &GPIO_InitStructure);
	//palSetPadMode( XPT2046_IRQ_PORT , XPT2046_IRQ_PAD , PAL_MODE_INPUT );

	SPI_Cmd(SPI1, ENABLE);

	// Read a sample, leaving PENIRQ active
	XPT2046_GetData( 0x90 );

	/* calibr points
	 *
	ILI9327_Circle(25,25, 2);
	ILI9327_Circle(215,25, 2);
	ILI9327_Circle(120,200, 2);
	ILI9327_Circle(25,375, 2);
	ILI9327_Circle(215,375, 2);

	ILI9327_Circle(25,200, 2);
	ILI9327_Circle(215,200, 2);
	 */
	Calibr.A = 0.080486;
	Calibr.B = -0.001254;
	Calibr.C = -32.1536;
	Calibr.D = 0.00076;
	Calibr.E = 0.111572;
	Calibr.F = -32.6017;

}


int XPT2046_GetCoord( int * pX , int * pY )
{
	int i;
	int allX[ 7 ] , allY[ 7 ];
	XPT2046_GetData( 0xd1 );
	XPT2046_GetData( 0x91 );
	for ( i = 0 ; i < 7 ; i ++ ) {
		allX[ i ] = XPT2046_GetData( 0xd1 );
		allY[ i ] = XPT2046_GetData( 0x91 );
	}

	int j;
	for ( i = 0 ; i < 4 ; i ++ ) {
		for ( j = i ; j < 7 ; j ++ ) {
			int temp = allX[ i ];
			if ( temp > allX[ j ] ) {
				allX[ i ] = allX[ j ];
				allX[ j ] = temp;
			}
			temp = allY[ i ];
			if ( temp > allY[ j ] ) {
				allY[ i ] = allY[ j ];
				allY[ j ] = temp;
			}
		}
	}
	XPT2046_GetData( 0x90 );

	if(GPIO_ReadInputDataBit(XPT2046_IRQ_PORT, XPT2046_IRQ_PAD))
		return 0;

	*pX = allX[ 3 ];
	*pY = allY[ 3 ];

	return 1;
}

int XPT2046_GetAvgCoord( int * pX , int * pY , int nSamples )
{
	int nRead = 0;
	int xAcc = 0 , yAcc = 0;
	int x , y;

	while ( nRead < nSamples ) {
		if ( !XPT2046_GetCoord( &x , &y ) ) {
			break;
		}
		xAcc += x;
		yAcc += y;
		nRead ++;
	}

	if ( nRead == 0 ) {
		return 0;
	}
	*pX = 4096 - xAcc / nRead;
	*pY = yAcc / nRead;
	return 1;
}


void XPT2046_GetCursor(uint16_t * pX , uint16_t * pY)
{
	uint16_t xx, yy;

	XPT2046_GetAvgCoord((int*)&yy, (int*)&xx, 25);

	float x=xx, y=yy;

	*pX = (int)(Calibr.A*x + Calibr.B*y + Calibr.C);
	*pY = (int)(Calibr.D*x + Calibr.E*y + Calibr.F);

	if((*pY) > 500) *pY = 0;
	if((*pX) > 500) *pX = 0;
}















#if 0
/*******************************************************************************
* Function Name  : vApplicationIdleHook
* Description    :
*******************************************************************************/

void Touch_Init()
{
	ADC_Enable(ADC2);

	/*xd[0]=120; yd[0]=40;
	xd[1]=220; yd[1]=200;
	xd[2]=20;  yd[2]=360;

	xc[0]=115; yc[0]=70;
	xc[1]=177; yc[1]=200;
	xc[2]=50;  yc[2]=333;

	ILI9327_setColor(VGA_GREEN);
	for(uint8_t i=0; i<3; i++)
		ILI9327_Circle(xd[i], yd[i], 3);
*/
#if 0
	Calibr.A = Calibr.E = 1;
	Calibr.B = Calibr.C = Calibr.D = Calibr.F = 0;
#else
	Calibr.A = 1.55728;
	Calibr.B = -0.016047;
	Calibr.C = -58.3726;
	Calibr.D = 0.005397;
	Calibr.E = 1.20892;
	Calibr.F = -42.1718;
#endif
}


void Touch_Calibrate(uint8_t point)
{
	float a, b;

	switch(point)
	{
	case 0:
	case 1:
	case 2:
		while(abs(xt[point]-xc[point])>15 || abs(yt[point]-yc[point])>15)
		{
			if(Touch_GetPos())
			{
				xt[point] = TouchPoint.x;
				yt[point] = TouchPoint.y;
			}
		}
		break;

	case 3:
		a = (xd[0]*(yt[1]-yt[2])+xd[1]*(yt[2]-yt[0])+xd[2]*(yt[0]-yt[1]));
		b = (xt[0]*(yt[1]-yt[2])+xt[1]*(yt[2]-yt[0])+xt[2]*(yt[0]-yt[1]));
		Calibr.A = a/b;

		Calibr.B = (Calibr.A*(xt[2]-xt[1])+xd[1]-xd[2])/(yt[1]-yt[2]);
		Calibr.C = xd[2]-Calibr.A*xt[2]-Calibr.B*yt[2];

		a = (yd[0]*(yt[1]-yt[2])+yd[1]*(yt[2]-yt[0])+yd[2]*(yt[0]-yt[1]));
		b = (xt[0]*(yt[1]-yt[2])+xt[1]*(yt[2]-yt[0])+xt[2]*(yt[0]-yt[1]));
		Calibr.D =  a/b;

		Calibr.E = (Calibr.D*(xt[2]-xt[1])+yd[1]-yd[2])/(yt[1]-yt[2]);
		Calibr.F = yd[2]-Calibr.D*xt[2]-Calibr.E*yt[2];
		break;
	}

}

static bool firstPress = FALSE;
	static uint8_t pressCnt = 0;
	static PointType last;
int Touch_GetPos()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Pin = Y2 | Y1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIOC->BSRRH = Y1 | Y2;

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Pin = X1 | X2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	vTaskDelay(1);



	if((GPIOC->IDR & X1) == 0)
	{
		vTaskDelay(1);
		PointType curTch;
		TouchPoint=curTch = Touch_Read();
		//return 1;//

		//if(!firstPress/* || abs(TouchPoint.x-lastX)>=2 || abs(TouchPoint.y-lastY)>=2*/)
		//{
			last.x += curTch.x;
			last.y += curTch.y;

			if(++pressCnt >= 1)
			{
				TouchPoint.x = last.x/1;
				TouchPoint.y = last.y/1;
				last.x = last.y = 0;
				pressCnt = 0;
				//firstPress = TRUE;
				return 1;
			}
			else
				return 0;



		/*}
		else
			return 0;*/
	}
	else
	{
		last.x = last.y = 0;
		pressCnt = 0;

		firstPress = FALSE;
		return 0;
	}
}

PointType Touch_Read()
{
	//extern uint8_t SmallFont[];
	PointType point;

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Pin = X1 | X2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIOC->BSRRH = X1;
	GPIOC->BSRRL = X2;

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_Pin = Y1 | Y2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	vTaskDelay(3);

	uint16_t y = (4096-readADC(15))*400/4096;

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Pin = Y1 | Y2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIOC->BSRRH = Y1;
	GPIOC->BSRRL = Y2;

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_Pin = X1 | X2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	vTaskDelay(3);

	uint16_t x = readADC(14)*240/4096;

	point.x = Calibr.A*x + Calibr.B*y + Calibr.C;
	point.y = Calibr.D*x + Calibr.E*y + Calibr.F;

	return point;
}


void ADC_Enable()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);

	ADC_InitTypeDef ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;

	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div8;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);

	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = 1;

	ADC_Init(ADC2, &ADC_InitStructure);

	/* Enable the specified ADC*/
	ADC_Cmd(ADC2, ENABLE);
}


u16 readADC(u8 channel)
{
	ADC_RegularChannelConfig(ADC2, channel, 1, ADC_SampleTime_56Cycles);
	// Start the conversion
	ADC_SoftwareStartConv(ADC2);
	// Wait until conversion completion
	while (ADC_GetFlagStatus(ADC2, ADC_FLAG_EOC) == RESET);
	// Get the conversion value
	return 	(ADC_GetConversionValue(ADC2));
}
#endif
