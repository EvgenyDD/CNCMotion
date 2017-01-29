#ifndef LCD_PCD8544
#define LCD_PCD8544

// если следующее определение закомментировано то работает програмный SPI
// если раскомментировано - то аппаратный SPI2
// по скорости у меня получилось что аппаратный SPI в 5 раза быстрее заполнял экран (SPI2 = 18 мс, против SOFT-SPI=97 мс)
#define LCD_SPI_HARD

// ИСПОЛЬЗОВАНИЕ DMA ПРИ ОБМЕНЕ С ДИСПЛЕЕМ
//#define LCD_SPI_DMA_HARD


// для простоты драйвера при программном SPI все линии управления и данных дисплея должны быть подключены к одному порту ввода\вывода !
// если же задан режим аппаратного SPI2 то, на один порт должны быть подключены линии RST, A0, CSE
// а остальные линии (SDA, SCK) должны быть подключены к GPIOB (пины 15 и 13 соответственно)
#define 	LCD_GPIO			GPIOB
#define     LCD_AHB1_GPIO		RCC_AHB1Periph_GPIOB
// пины дисплея (управление)
#define 	LCD_RST_PIN 		14
#define     LCD_A0_PIN			12
#define		LCD_CSE_PIN			10
//
// в случае если используется SPI2 подключение то вывод SDA дисплея должен быть подключен к GPIOB_Pin_15, и SCK к GPIOB_Pin_13
// остальные линии при SPI2 подключении могут быть подключены к любому порту (не только к GPIOB) см. выше
// пины дисплея (данные)

#define		LCD_SDA_PIN			15
#define		LCD_SCK_PIN			13




// определения для битбанда
#define IO_BB_ADDR(io_reg_addr,bit_number) ((uint32_t*)(PERIPH_BB_BASE + (((uint32_t)io_reg_addr - PERIPH_BASE) << 5) + (bit_number << 2)))

#define LCD_RST_BB_ADDR  IO_BB_ADDR(&LCD_GPIO->ODR, LCD_RST_PIN)
#define LCD_A0_BB_ADDR   IO_BB_ADDR(&LCD_GPIO->ODR, LCD_A0_PIN)
#define LCD_CSE_BB_ADDR  IO_BB_ADDR(&LCD_GPIO->ODR, LCD_CSE_PIN)
#define LCD_SDA_BB_ADDR  IO_BB_ADDR(&LCD_GPIO->ODR, LCD_SDA_PIN)
#define LCD_SCK_BB_ADDR  IO_BB_ADDR(&LCD_GPIO->ODR, LCD_SCK_PIN)

// Управление линией LCD_RST
#define LCD_RST1  *LCD_RST_BB_ADDR=0x00000001
#define LCD_RST0  *LCD_RST_BB_ADDR=0x00000000
// Управление линией LCD_DC
#define LCD_DC1   *LCD_A0_BB_ADDR=0x00000001
#define LCD_DC0   *LCD_A0_BB_ADDR=0x00000000
// Управление линией LCD_CS
#define LCD_CS1   *LCD_CSE_BB_ADDR=0x00000001
#define LCD_CS0   *LCD_CSE_BB_ADDR=0x00000000

#ifndef LCD_SPI_HARD
// Управление линией LCD_SCK
#define LCD_SCK1  *LCD_SCK_BB_ADDR=0x00000001
#define LCD_SCK0  *LCD_SCK_BB_ADDR=0x00000000
// Управление линией LCD_MOSI
#define LCD_MOSI1 *LCD_SDA_BB_ADDR=0x00000001
#define LCD_MOSI0 *LCD_SDA_BB_ADDR=0x00000000
#endif



void LCD_Init();
void LCD_data(uint8_t);
void LCD_Refresh();
void LCD_Clear();

void LCD_Pixel(uint8_t, uint8_t, uint8_t);
void LCD_Line(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t);
void LCD_Rectangle(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t);

void LCD_Char(uint8_t, uint8_t, uint8_t, uint8_t);
void LCD_String(uint8_t, uint8_t y, char*, uint8_t);
void LCD_Int(uint8_t, uint8_t, int, uint8_t);
void LCD_IntWString(uint8_t, uint8_t, int, char*, uint8_t);


#endif

