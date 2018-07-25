/*
 * ili9341_drv.c
 *
 *  Created on: 23. 7. 2018
 *      Author: yurovv
 */


#include "ili9341_drv.h"
#include "stm32f4xx_ll_conf.h"
#include "ili9341_port.h"
#include "sdram.h"


//=================================================================================================
//    LTDC SECTION
//=================================================================================================

#define PLLSAI_N     224
#define PLLSAI_R     LL_RCC_PLLSAIR_DIV_5
#define PLLSAI_DIVR  LL_RCC_PLLSAIDIVR_DIV_8

#define PIXEL_SIZE        ((uint16_t)2)

typedef enum
{
  FALSE = 0,
  TRUE = 1,
}lcd_bool_t;


/**************************************** Timing display ***************************************/

#define  LCD_HSYNC            ((uint16_t)9)      // Horizontal synchronization
#define  LCD_HBP              ((uint16_t)29)      // Horizontal back porch
#define  LCD_HFP              ((uint16_t)2)      // Horizontal front porch

#define  LCD_VSYNC            ((uint16_t)1)      // Vertical synchronization
#define  LCD_VBP              ((uint16_t)3)       // Vertical back porch
#define  LCD_VFP              ((uint16_t)2)       // Vertical front porch

#define  LCD_LTDC_W 240
#define  LCD_LTDC_H 320

#define LCD_BUFF_ADDR_1  (SDRAM_BASE)

#if (LCD_ENABLE_VSYNC == 1)
#define LCD_BUFF_ADDR_2  (SDRAM_BASE + ( (LCD_LTDC_W * LCD_LTDC_H) * PIXEL_SIZE) )
#else
#define LCD_BUFF_ADDR_2  LCD_BUFF_ADDR_1
#endif

volatile uint32_t nBuffAddrToView = LCD_BUFF_ADDR_1;
volatile uint32_t nBuffAddrToDraw = LCD_BUFF_ADDR_2;
volatile lcd_bool_t  nAreBuffersSwapped = FALSE;

static void ili9341_fillLayerColour(uint16_t colour);

static void ili9341_InitPins();
static void ili9341_InitRGBInterface();
static void ili9341_InitLTDC();

static void ili9341_send8Bit(uint16_t data);
static void ili9341_SendCommand(uint8_t data);
static void ili9341_SendData(uint8_t data);

void ili9341_initLcd(void)
{

  ili9341_InitPins();
  ili9341_InitRGBInterface();
  ili9341_InitLTDC();

  ili9341_fillLayerColour(0x07E0);

}


static void ili9341_InitPins()
{
  LL_GPIO_InitTypeDef gpioInitStruct;

  gpioInitStruct.Speed      = LL_GPIO_SPEED_FREQ_MEDIUM;
  gpioInitStruct.Mode       = LL_GPIO_MODE_OUTPUT;
  gpioInitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  gpioInitStruct.Pull       = LL_GPIO_PULL_NO;

  RCC->AHB1ENR |= ILI9341_MOSI_PCLK;
  gpioInitStruct.Pin = ILI9341_MOSI_PIN;
  LL_GPIO_Init(ILI9341_MOSI_PORT, &gpioInitStruct);

  RCC->AHB1ENR |= ILI9341_SCL_PCLK;
  gpioInitStruct.Pin = ILI9341_SCL_PIN;
  LL_GPIO_Init(ILI9341_SCL_PORT, &gpioInitStruct);

  RCC->AHB1ENR |= ILI9341_WRX_PCLK;
  gpioInitStruct.Pin = ILI9341_WRX_PIN;
  LL_GPIO_Init(ILI9341_WRX_PORT, &gpioInitStruct);

  RCC->AHB1ENR |= ILI9341_CS_PCLK;
  gpioInitStruct.Pin = ILI9341_CS_PIN;
  LL_GPIO_Init(ILI9341_CS_PORT, &gpioInitStruct);

  gpioInitStruct.Speed      = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  gpioInitStruct.Mode       = LL_GPIO_MODE_ALTERNATE;

  /* GPIOA                     Blue5        VSYNC        Green2       Red4          Red5 */
  RCC->AHB1ENR             |= RCC_AHB1ENR_GPIOAEN;
  gpioInitStruct.Pin        = LCD_B5_PIN | LCD_VSYNC_PIN | LCD_G2_PIN | LCD_R4_PIN | LCD_R5_PIN;
  gpioInitStruct.Alternate  = LCD_GPIO_AF_LTDC1;
  LL_GPIO_Init(GPIOA, &gpioInitStruct);

  /* GPIOB                     Red3         Red6 */
  RCC->AHB1ENR             |= RCC_AHB1ENR_GPIOBEN;
  gpioInitStruct.Pin        = LCD_R3_PIN | LCD_R6_PIN;
  gpioInitStruct.Alternate  = LCD_GPIO_AF_LTDC2;
  LL_GPIO_Init(GPIOB, &gpioInitStruct);

  /* GPIOB                     Blue6      Blue7        Green4        Green5 */
  RCC->AHB1ENR             |= RCC_AHB1ENR_GPIOBEN;
  gpioInitStruct.Pin        = LCD_B6_PIN | LCD_B7_PIN | LCD_G4_PIN | LCD_G5_PIN;
  gpioInitStruct.Alternate  = LCD_GPIO_AF_LTDC1;
  LL_GPIO_Init(GPIOB, &gpioInitStruct);

  /* GPIOC                     HSYNC        Green6       Red2 */
  RCC->AHB1ENR             |= RCC_AHB1ENR_GPIOCEN;
  gpioInitStruct.Pin        = LCD_HSYNC_PIN | LCD_G6_PIN | LCD_R2_PIN;
  gpioInitStruct.Alternate  = LCD_GPIO_AF_LTDC1;
  LL_GPIO_Init(GPIOC, &gpioInitStruct);

  /* GPIOD                     Green7       Blue2 */
  RCC->AHB1ENR             |= RCC_AHB1ENR_GPIODEN;
  gpioInitStruct.Pin        = LCD_G7_PIN | LCD_B2_PIN;
  gpioInitStruct.Alternate  = LCD_GPIO_AF_LTDC1;
  LL_GPIO_Init(GPIOD, &gpioInitStruct);

  /* GPIOF                     Enable */
  RCC->AHB1ENR             |= RCC_AHB1ENR_GPIOFEN;
  gpioInitStruct.Pin        = LCD_DE_PIN;
  gpioInitStruct.Alternate  = LCD_GPIO_AF_LTDC1;
  LL_GPIO_Init(GPIOF, &gpioInitStruct);

  /* GPIOG                     Red7         DOTCLK       Blue3 */
  RCC->AHB1ENR             |= RCC_AHB1ENR_GPIOGEN;
  gpioInitStruct.Pin        = LCD_R7_PIN | LCD_CLK_PIN | LCD_B3_PIN;
  gpioInitStruct.Alternate  = LCD_GPIO_AF_LTDC1;
  LL_GPIO_Init(GPIOG, &gpioInitStruct);

  /* GPIOG                     Green3        Blue4 */
  RCC->AHB1ENR             |= RCC_AHB1ENR_GPIOGEN;
  gpioInitStruct.Pin        = LCD_G3_PIN | LCD_B4_PIN;
  gpioInitStruct.Alternate  = LCD_GPIO_AF_LTDC2;
  LL_GPIO_Init(GPIOG, &gpioInitStruct);

}

static void ili9341_InitRGBInterface()
{
  ILI9341_CS_SET;
  ili9341_SendCommand(0xCA);
  ili9341_SendData(0xC3);
  ili9341_SendData(0x08);
  ili9341_SendData(0x50);
  ili9341_SendCommand(ILI9341_POWERB);
  ili9341_SendData(0x00);
  ili9341_SendData(0xC1);
  ili9341_SendData(0x30);
  ili9341_SendCommand(ILI9341_POWER_SEQ);
  ili9341_SendData(0x64);
  ili9341_SendData(0x03);
  ili9341_SendData(0x12);
  ili9341_SendData(0x81);
  ili9341_SendCommand(ILI9341_DTCA);
  ili9341_SendData(0x85);
  ili9341_SendData(0x00);
  ili9341_SendData(0x78);
  ili9341_SendCommand(ILI9341_POWERA);
  ili9341_SendData(0x39);
  ili9341_SendData(0x2C);
  ili9341_SendData(0x00);
  ili9341_SendData(0x34);
  ili9341_SendData(0x02);
  ili9341_SendCommand(ILI9341_PRC);
  ili9341_SendData(0x20);
  ili9341_SendCommand(ILI9341_DTCB);
  ili9341_SendData(0x00);
  ili9341_SendData(0x00);
  ili9341_SendCommand(ILI9341_FRC);
  ili9341_SendData(0x00);
  ili9341_SendData(0x1B);
  ili9341_SendCommand(ILI9341_DFC);
  ili9341_SendData(0x0A);
  ili9341_SendData(0xA2);
  ili9341_SendCommand(ILI9341_POWER1);
  ili9341_SendData(0x10);
  ili9341_SendCommand(ILI9341_POWER2);
  ili9341_SendData(0x10);
  ili9341_SendCommand(ILI9341_VCOM1);
  ili9341_SendData(0x45);
  ili9341_SendData(0x15);
  ili9341_SendCommand(ILI9341_VCOM2);
  ili9341_SendData(0x90);
  ili9341_SendCommand(ILI9341_MAC);
  ili9341_SendData(0xC8);
  ili9341_SendCommand(ILI9341_3GAMMA_EN);
  ili9341_SendData(0x00);
  ili9341_SendCommand(ILI9341_RGB_INTERFACE);
  ili9341_SendData(0xC2);
  ili9341_SendCommand(ILI9341_DFC);
  ili9341_SendData(0x0A);
  ili9341_SendData(0xA7);
  ili9341_SendData(0x27);
  ili9341_SendData(0x04);

  ili9341_SendCommand(ILI9341_COLUMN_ADDR);
  ili9341_SendData(0x00);
  ili9341_SendData(0x00);
  ili9341_SendData(0x00);
  ili9341_SendData(0xEF);

  ili9341_SendCommand(ILI9341_PAGE_ADDR);
  ili9341_SendData(0x00);
  ili9341_SendData(0x00);
  ili9341_SendData(0x01);
  ili9341_SendData(0x3F);
  ili9341_SendCommand(ILI9341_INTERFACE);
  ili9341_SendData(0x01);
  ili9341_SendData(0x00);
  ili9341_SendData(0x06);

  ili9341_SendCommand(ILI9341_GRAM);
  LL_mDelay(20);

  ili9341_SendCommand(ILI9341_GAMMA);
  ili9341_SendData(0x01);

  ili9341_SendCommand(ILI9341_PGAMMA);
  ili9341_SendData(0x0F);
  ili9341_SendData(0x29);
  ili9341_SendData(0x24);
  ili9341_SendData(0x0C);
  ili9341_SendData(0x0E);
  ili9341_SendData(0x09);
  ili9341_SendData(0x4E);
  ili9341_SendData(0x78);
  ili9341_SendData(0x3C);
  ili9341_SendData(0x09);
  ili9341_SendData(0x13);
  ili9341_SendData(0x05);
  ili9341_SendData(0x17);
  ili9341_SendData(0x11);
  ili9341_SendData(0x00);
  ili9341_SendCommand(ILI9341_NGAMMA);
  ili9341_SendData(0x00);
  ili9341_SendData(0x16);
  ili9341_SendData(0x1B);
  ili9341_SendData(0x04);
  ili9341_SendData(0x11);
  ili9341_SendData(0x07);
  ili9341_SendData(0x31);
  ili9341_SendData(0x33);
  ili9341_SendData(0x42);
  ili9341_SendData(0x05);
  ili9341_SendData(0x0C);
  ili9341_SendData(0x0A);
  ili9341_SendData(0x28);
  ili9341_SendData(0x2F);
  ili9341_SendData(0x0F);

  ili9341_SendCommand(ILI9341_SLEEP_OUT);
  LL_mDelay(120);
  ili9341_SendCommand(ILI9341_DISPLAY_ON);

  ili9341_SendCommand(ILI9341_GRAM);
}

static void ili9341_SclDelay(uint16_t nCount)
{
  uint16_t nIndex = 0;
  for (nIndex = nCount; nIndex; nIndex--)
  {
    __NOP();
  }
}
static void ili9341_send8Bit(uint16_t data)
{

  uint8_t count = 0;
  // first bit MSB
  for (count = 0; count < 8; count++)
  {
    if (data & 0x80)
    {
      ILI9341_MOSI_SET;
    }
    else
    {
      ILI9341_MOSI_RESET;
    }
    data <<= 1;
    ILI9341_SCL_RESET;
    ili9341_SclDelay(1);
    ILI9341_SCL_SET;
    ili9341_SclDelay(1);
  }
}

static void ili9341_SendCommand(uint8_t data)
{
  ILI9341_WRX_RESET;
  ILI9341_CS_RESET;
  ili9341_send8Bit(data);

  ILI9341_CS_SET;
}

static void ili9341_SendData(uint8_t data)
{
  ILI9341_WRX_SET;
  ILI9341_CS_RESET;
  ili9341_send8Bit(data);
  ILI9341_CS_SET;
}



static void ili9341_InitLTDC()
{
  //*** PLLSAI clock configuration  **************************************************************
    LL_RCC_PLLSAI_ConfigDomain_LTDC(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLSAIM_DIV_8, PLLSAI_N , PLLSAI_R, PLLSAI_DIVR);
    LL_RCC_PLLSAI_Enable();
    while(LL_RCC_PLLSAI_IsReady() != 1);


  //**** Timings for TFT display  *****************************************************************
  #define HSW     (LCD_HSYNC  - 1)
  #define AHBP    (LCD_HSYNC  + LCD_HBP   - 1)
  #define AAH     (LCD_LTDC_W  + LCD_HSYNC + LCD_HBP   - 1)
  #define TOTALH  (LCD_LTDC_W  + LCD_HSYNC + LCD_HBP   + LCD_HFP - 1)
  #define WWSSTP  (LCD_LTDC_W  + LCD_HBP   + LCD_HSYNC - 1)
  #define WWSSRT  (LCD_HBP    + LCD_HSYNC)

  #define VSH     (LCD_VSYNC  - 1)
  #define AVBP    (LCD_VSYNC  + LCD_VBP   - 1)
  #define AAW     (LCD_LTDC_H + LCD_VSYNC + LCD_VBP - 1)
  #define TOTALW  (LCD_LTDC_H + LCD_VSYNC + LCD_VBP + LCD_VFP - 1)
  #define WHSSTP  (LCD_LTDC_H + LCD_VSYNC + LCD_VBP - 1)
  #define WHSSRT  (LCD_VSYNC  + LCD_VBP)

  #define CFBP    (PIXEL_SIZE * LCD_LTDC_W)  // Color frame buffer pitch in bytes

  RCC->APB2ENR |= RCC_APB2ENR_LTDCEN;

  LTDC->SSCR |= ((HSW << 16) | VSH);
  LTDC->BPCR |= ((AHBP << 16) | AVBP);
  LTDC->AWCR |= ((AAH << 16) | AAW);
  LTDC->TWCR |= ((TOTALH << 16) | TOTALW);
  LTDC->BCCR = 0x000000;                         // Background color

  LTDC_Layer1->WHPCR |= ((WWSSTP << 16) | WWSSRT);      // Window width size (stop and start)
  LTDC_Layer1->WVPCR |= ((WHSSTP << 16) | WHSSRT);      // Window height size (stop and start)
  LTDC_Layer1->CFBLR |= ((CFBP << 16) | (CFBP + 3));   // Color frame buffer pitch and length+3 in bytes
  LTDC_Layer1->BFCR |= ((4 << 8) | 5);                  // Blending factors
  LTDC_Layer1->CFBLNR |= LCD_LTDC_H;                      // Frame buffer line number
  LTDC_Layer1->PFCR = LTDC_PIXEL_FORMAT_RGB565;        // Format RGB565
  LTDC_Layer1->CFBAR = ili9341_GetBufferToView();           // Address layer buffer display
  LTDC_Layer1->CACR = 255;                              // Alpha constant

  LTDC_Layer1->CR |= LTDC_LxCR_LEN;                   // Enable layer #1

  LTDC->LIPCR = (uint32_t) LCD_LTDC_H;            // Set line nubmer for line interrupt
  LTDC->SRCR |= LTDC_SRCR_VBR;                   // Reload set up configuration on vertical blank
  LTDC->GCR |= LTDC_GCR_LTDCEN;                 // LTDC enable

  NVIC_EnableIRQ(LTDC_IRQn);
  NVIC_SetPriority(LTDC_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
}




/*****************************************************************************************
@brief:  Fill entire LCD with colour
@note:   None
@param:  uint16_t colour: value of colour for pixel
@retval: None
******************************************************************************************/
static void ili9341_fillLayerColour(uint16_t colour)
{
  uint32_t i;
  uint32_t pixels = LCD_PIXELS;
  for (i = 0; i < pixels; i++)
  {
    *((uint16_t *)SDRAM_BASE + i) = colour;
  }
}

uint32_t ili9341_GetBufferToDraw()
{
  return nBuffAddrToDraw;
}

uint32_t ili9341_GetBufferToView()
{
  return nBuffAddrToView;
}

void ili9341_SwitchBuffers()
{
#if (LCD_ENABLE_VSYNC == 0)
    return;
#endif

  uint32_t pTempBuff = nBuffAddrToView;
  nBuffAddrToView = nBuffAddrToDraw;
  nBuffAddrToDraw = pTempBuff;

  nAreBuffersSwapped = FALSE;
  // enable line interrupt
  LTDC->IER |= LTDC_IER_LIE;
  // wait till buffers are switched
  while (nAreBuffersSwapped == FALSE)
  {
    // TODO: implement method to catch timeout error event here
  };
}


// LTDC IRQ Handler
void LCD_TFT_IRQHandler(void)
{
 if (LTDC->ISR & LTDC_ISR_LIF)
 {
   // disable interrupt, we need to enable it only when screen had been refreshed
   LTDC->IER &= ~(LTDC_IER_LIE);
   // clear flag
   LTDC->ICR = (uint32_t)LTDC_ICR_CLIF;

   // set buffer to view
   LTDC_Layer1->CFBAR  = ili9341_GetBufferToView();
   // Reload set up configuration on vertical blank
   LTDC->SRCR  |= LTDC_SRCR_VBR;
   nAreBuffersSwapped = TRUE;
 }
}













