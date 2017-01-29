/* Includes ------------------------------------------------------------------*/
#include "string.h"
#include "stm32f4xx.h"
//#include "math_.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


/*******************************************************************************
* Function Name  : strlen
* Description    : calculating length of the string
* Input          : pointer to text string
* Return         : string length
*******************************************************************************/
int strlen(char *pText)
{
	int len = 0;
	for(; *pText != '\0'; pText++, len++);
	return len;
}


/*******************************************************************************
* Function Name  : strlenNum
* Description    : calculating length of the string
* Input          : pointer to text string
* Return         : string length
*******************************************************************************/
int strlenNum(char *pText, int begin)
{
	int len = 0;
	pText += begin;
	for(; *pText != '\0'; pText++, len++);
	return len;
}



/*******************************************************************************
* Function Name  : itoa
* Description    : Convert int to char
* Input          : int number (signed/unsigned)
* Return         : pointer to text string
*******************************************************************************/
void itoa_(int64_t n, char s[])
{
	int sign;

	if ((sign = n) < 0)
		n = -n;

	int i = 0;

	do {
		s[i++] = n % 10 + '0';
	}
	while ((n /= 10) > 0);

	if (sign < 0) s[i++] = '-';

	s[i] = '\0';
	reverse(s);
}

/*******************************************************************************
* Function Name  : itoa
* Description    : Convert int to char
* Input          : int number (signed/unsigned)
* Return         : pointer to text string
*******************************************************************************/
void dtoa_(uint32_t n, char s[])
{
	int sign;

	if ((sign = n) < 0)
		n = -n;

	int i = 0;

	do {
		s[i++] = n % 10 + '0';
	}
	while ((n /= 10) > 0);

	if (sign < 0) s[i++] = '-';

	s[i] = '\0';
	reverse(s);
}


/*******************************************************************************
* Function Name  : ftoa_
* Description    : Convert float to char
* Input          : float number, char, output precision
* Return         : pointer to text string
*******************************************************************************/
void ftoa_(float num, char str[], char precision)
{
	unsigned char zeroFlag=0;
	int digit=0, reminder=0;
	long wt=0;

	if(num < 0)
	{
		num = -num;
		zeroFlag = 1;
	}

	int whole_part = num;
	int log_value=log10_(num), index=log_value;

	if(zeroFlag) str[0]='-';

	//Extract the whole part from float num
	for(int i=1; i<log_value+2; i++)
	{
		wt = pow_(10.0, i);
		reminder = whole_part % wt;
		digit = (reminder - digit) / (wt/10);

		//Store digit in string
		str[index-- + zeroFlag] = digit + 48;              // ASCII value of digit  = digit + 48
		if (index == -1)
			break;
	}

	index = log_value + 1;
	str[index+zeroFlag] = '.';

	float fraction_part = num - whole_part;
	float tmp1 = fraction_part, tmp=0;

	//Extract the fraction part from  number
	for( int i=1; i<=precision; i++)
	{
		wt = 10;
		tmp  = tmp1 * wt;
		digit = tmp;

		//Store digit in string
		str[++index + zeroFlag] = digit + 48;           // ASCII value of digit  = digit + 48
		tmp1 = tmp - digit;
	}
	str[++index + zeroFlag] = '\0';
}


/*******************************************************************************
* Function Name  : reverse
* Description    : Reverses string
* Input          : pointer to string
*******************************************************************************/
void reverse(char s[])
{
	int c, i, j;
	for (i = 0, j = strlen(s)-1; i < j; i++, j--)
	{
		c = s[i];
		s[i] = s[j];
		s[j] = c;
	}
}


/*******************************************************************************
* Function Name  : strcat
* Description    : add
* Input          : pointer to string
*******************************************************************************/
void strcat_(char first[], char second[])
{
	int i=0, j=0;

	while (first[i] != '\0') i++;
	while ((first[i++] = second[j++]) != '\0');
}


/*******************************************************************************
* Function Name  : strcat
* Description    : add
* Input          : pointer to string
*******************************************************************************/
void strcatNum(char first[], char second[], int begin, int end)
{
	if(begin >= end) return;

	int i=0, j=begin;

	while(j != end)
	{
		first[i++] = second[j++];
	}
	first[i]='\0';
}


/*******************************************************************************
* Function Name  : pow_
* Description    : Power x in y
*******************************************************************************/
float pow_(float x, float y)
{
    double result = 1;

    for (int i=0; i<y; i++)
        result *= x;

    return result;
}


/*******************************************************************************
* Function Name  : log10_
*******************************************************************************/
float log10_(int v)
{
    return (v >= 1000000000u) ? 9 : (v >= 100000000u) ? 8 :
        (v >= 10000000u) ? 7 : (v >= 1000000u) ? 6 :
        (v >= 100000u) ? 5 : (v >= 10000u) ? 4 :
        (v >= 1000u) ? 3 : (v >= 100u) ? 2 : (v >= 10u) ? 1u : 0u;
}
