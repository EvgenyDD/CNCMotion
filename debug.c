/* Includes ------------------------------------------------------------------*/
#include "debug.h"
#include "string.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint32_t DebugDecrease;

/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
* Function Name  : DebugInit
* Description    : Initialize debug (via USART2)
*******************************************************************************/
void DebugInit()
{


}


/*******************************************************************************
* Function Name  : DebugSendString
* Description    : Sends debug information (via USART2)
* Input          : pointer to text massive
*******************************************************************************/
void DebugSendString(char *pText)
{
	for(; *pText != '\0'; pText++)
	{
		while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET){}
		USART_SendData(USART2, *pText);
	}
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET){}
	USART_SendData(USART2, '\n');

}


/*******************************************************************************
* Function Name  : DebugSendChar
* Description    : Sends debug information (via USART2)
* Input          : char to send
*******************************************************************************/
void DebugSendChar(char charTx)
{
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
	USART_SendData(USART2, charTx);
}

void DebugSysTimeStart()
{
	DebugDecrease = 0;
}
/*******************************************************************************
* Function Name  : DebugSysTimeEnd
* Description    : send time from DebugSysTimeStart()
*******************************************************************************/
/*void DebugSysTimeEnd()
{
	char f[15];
	//if(Debug)
	ftoa_(DebugDecrease, f);
	DebugSendString(f);
}*/



void DebugSendNum(uint16_t Num)
{
	char str[50];
	itoa_(Num, str);
	DebugSendString(str);
}

void DebugSendNumWDesc(char *string, uint64_t Num)
{
	char str[50]={'\0'}, number[20];
	strcat_(str, string);
	itoa_(Num, number);
	strcat_(str, number);
	DebugSendString(str);
}

void DebugSendPID(int16_t pos, int16_t speed)
{
	char out[10] = "A";
	char conv[5];
	itoa_(pos, conv);
	strcat_(out, conv);
	strcat_(out, "B");
	itoa_(speed, conv);
	strcat_(out, conv);
	strcat_(out, "C");
	DebugSendString(out);
}
