/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DEBUG_H
#define DEBUG_H


/* Includes ------------------------------------------------------------------*/
#include <stm32f4xx.h>
#include <stm32f4xx_usart.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>


/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void DebugInit();
void DebugSendString(char*);
void DebugSendChar(char);
void DebugSysTimeEnd();
void DebugSysTimeStart();
void DebugSendNum(uint16_t Num);
void DebugSendNumWDesc(char *string, uint64_t Num);
void DebugSendPID(int16_t pos, int16_t speed);

#endif //DEBUG_H
