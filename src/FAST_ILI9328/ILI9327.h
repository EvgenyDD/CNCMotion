/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef ILI9327_H
#define ILI9327_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"


/* Exported types ------------------------------------------------------------*/
 typedef struct
{
  __IO uint16_t LCD_REG;
  __IO uint16_t LCD_RAM;
} LCD_TypeDef;

typedef struct
{
 __IO uint16_t LCD_REG;
 __IO uint32_t LCD_RAM;
} LCD_TypeDef2;


/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
#define BitSet(p,m) ((p) |= (1<<(m)))
#define BitReset(p,m) ((p) &= ~(1<<(m)))
#define BitFlip(p,m) ((p) ^= (m))
#define BitWrite(c,p,m) ((c) ? BitSet(p,m) : BitReset(p,m))
#define BitIsSet(reg, bit) (((reg) & (1<<(bit))) != 0)
#define BitIsReset(reg, bit) (((reg) & (1<<(bit))) == 0)

#define ASSEMBLE_RGB(R ,G, B)    ((((R)& 0xF8) << 8) | (((G) & 0xFC) << 3) | (((B) & 0xF8) >> 3))

/* Exported define -----------------------------------------------------------*/
//*********************************
// COLORS
//*********************************
// VGA color palette
#define VGA_BLACK		0x0000
#define VGA_WHITE		0xFFFF
#define VGA_RED			0xF800
#define VGA_GREEN		0x07E0
#define VGA_BLUE		0x001F
#define VGA_SILVER		0xC618
#define VGA_GRAY		0x8410
#define VGA_MAROON		0x8000
#define VGA_YELLOW		0xFFE0
#define VGA_OLIVE		0x8400
#define VGA_LIME		0x07E0
#define VGA_AQUA		0x07FF
#define VGA_TEAL		0x0410
#define VGA_NAVY		0x0010
#define VGA_FUCHSIA		0xF81F
#define VGA_PURPLE		0x8010
#define VGA_TRANSPARENT	0xFFFFFFFF

#define PORTRAIT 0
#define LANDSCAPE 1


/* Exported functions ------------------------------------------------------- */
void PWM_BL_Init();
void FSMC_Init();
void ILI9327_Init();

void ILI9327_WriteCom(uint16_t reg);
void ILI9327_WriteParam(uint16_t paramValue);
void setXY(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void clrXY();

void _fast_fill_16(uint16_t color, long pix);
void _fast_fill_32(uint16_t color, long pix);

void ILI9327_Pixel(uint16_t color);

void ILI9327_FillRGB(uint8_t r, uint8_t g, uint8_t b);
void ILI9327_Fill(uint16_t color);
void ILI9327_Clear();

void ILI9327_Line(int x1, int y1, int x2, int y2);
void ILI9327_VLine(int x, int y, int l);
void ILI9327_HLine(int x, int y, int l);

void ILI9327_Rect(int x1, int y1, int x2, int y2);
void drawRoundRect(int x1, int y1, int x2, int y2);
void ILI9327_RectFill(int x1, int y1, int x2, int y2);
void fillRoundRect(int x1, int y1, int x2, int y2);

void ILI9327_Circle(int x, int y, int radius);
void ILI9327_CircleFill(int x, int y, int radius);

void ILI9327_setColorRGB(uint8_t r, uint8_t g, uint8_t b);
void ILI9327_setColor(uint16_t color);
void ILI9327_setBackColorRGB(uint8_t r, uint8_t g, uint8_t b);
void ILI9327_setBackColor(uint32_t color);

void ILI9327_Char(char c, int x, int y);

void ILI9327_setFont(uint8_t* font);
uint8_t* getFont();
uint8_t getFontXsize();
uint8_t getFontYsize();

void ILI9327_Bitmap(int x, int y, int sx, int sy, unsigned int* data, int scale);

int  getDisplayXSize();
int	 getDisplayYSize();


void ILI9327_String(int x, int y, char *s);
void ILI9327_Num(int x, int y, int32_t num);
void ILI9327_NumWDesc(int x, int y, char *s, int32_t num);
void ILI9327_FNumWDesc(int x, int y, char *s, float num, uint8_t precision);


		void rotateChar(uint8_t c, int x, int y, int pos, int deg);

#endif //ILI9327_H
