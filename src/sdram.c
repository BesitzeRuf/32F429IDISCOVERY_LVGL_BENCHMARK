/**
 *******************************************************************************
 * @file    sdram.cpp
 * @version
 * @date    18. 6. 2018
 * @brief
 *******************************************************************************
   copied from original source http://en.radzio.dxp.pl/stm32f429idiscovery/sdram.html
 *******************************************************************************
 */

#include "sdram.h"
#include "stm32f4xx.h"
#include "stm32f4xx_ll_conf.h"

#define MODE_INPUT  0
#define MODE_GPIO   1
#define MODE_AF     2
#define MODE_ANALOG 3

#define TYPE_PUSHPULL   0
#define TYPE_OPENDRAIN  1

#define SPEED_2MHz    0
#define SPEED_25MHz   1
#define SPEED_50MHz   2
#define SPEED_100MHz  3

#define PULLUP_NONE   0
#define PULLUP_UP     1
#define PULLUP_DOWN   2




static __attribute((optimize("-O0"))) void SDRAM_InitPins();
static __attribute((optimize("-O0"))) void SDRAM_InitSequence();

static void gpio_conf(GPIO_TypeDef * GPIO, uint8_t pin, uint8_t mode, uint8_t type, uint8_t speed, uint8_t pullup, uint8_t af);


//=================================================================================================
// SDRAM_Init function
//=================================================================================================
void   SDRAM_Init(void)
{
  SDRAM_InitPins();
  SDRAM_InitSequence();

  // Clear SDRAM
  uint16_t* ptr = 0;
  for(ptr = ( uint16_t*)SDRAM_BASE; ptr <  (uint16_t*)(SDRAM_BASE + SDRAM_SIZE/2); ptr++)
  {
    *ptr = 0xF81F;
  }
}

static void SDRAM_InitPins()
{
  RCC->AHB1ENR |= 0
      | RCC_AHB1ENR_GPIOAEN
      | RCC_AHB1ENR_GPIOBEN
      | RCC_AHB1ENR_GPIOCEN
      | RCC_AHB1ENR_GPIODEN
      | RCC_AHB1ENR_GPIOEEN
      | RCC_AHB1ENR_GPIOFEN
      | RCC_AHB1ENR_GPIOGEN
      ;

  static GPIO_TypeDef * const GPIOInitTable[] =
  {
  GPIOF, GPIOF, GPIOF, GPIOF, GPIOF, GPIOF, GPIOF, GPIOF, GPIOF, GPIOF, GPIOG, GPIOG,
  GPIOD, GPIOD, GPIOD, GPIOD, GPIOE, GPIOE, GPIOE, GPIOE, GPIOE, GPIOE, GPIOE, GPIOE, GPIOE, GPIOD, GPIOD, GPIOD,
  GPIOB, GPIOB, GPIOC, GPIOE, GPIOE, GPIOF, GPIOG, GPIOG, GPIOG, GPIOG, 0 };

  static uint8_t const PINInitTable[] =
  { 0, 1, 2, 3, 4, 5, 12, 13, 14, 15, 0, 1, 14, 15, 0, 1, 7, 8, 9, 10, 11, 12, 13, 14, 15, 8, 9, 10, 5, 6, 0, 0, 1, 11, 4, 5, 8, 15, 0 };

  uint8_t i = 0;
  while (GPIOInitTable[i] != 0)
  {
    gpio_conf(GPIOInitTable[i], PINInitTable[i], MODE_AF, TYPE_PUSHPULL, SPEED_100MHz, PULLUP_NONE, 12);
    i++;
  }

}

static void SDRAM_InitSequence()
{

#define TMRD(x) (x << 0)  /* Load Mode Register to Active */
#define TXSR(x) (x << 4)  /* Exit Self-refresh delay */
#define TRAS(x) (x << 8)  /* Self refresh time */
#define TRC(x)  (x << 12) /* Row cycle delay */
#define TWR(x)  (x << 16) /* Recovery delay */
#define TRP(x)  (x << 20) /* Row precharge delay */
#define TRCD(x) (x << 24) /* Row to column delay */

  uint32_t tmp;

  // Enable clock for FMC
  RCC->AHB3ENR |= RCC_AHB3ENR_FMCEN;
  // Initialization step 1
  FMC_Bank5_6->SDCR[0] = FMC_SDCR1_SDCLK_1 | FMC_SDCR1_RBURST | FMC_SDCR1_RPIPE_1;
  FMC_Bank5_6->SDCR[1] = FMC_SDCR1_NR_0 | FMC_SDCR1_MWID_0 | FMC_SDCR1_NB | FMC_SDCR1_CAS;
  // Initialization step 2
  FMC_Bank5_6->SDTR[0] = TRC(7) | TRP(2);
  FMC_Bank5_6->SDTR[1] = TMRD(2) | TXSR(7) | TRAS(4) | TWR(2) | TRCD(2);
  // Initialization step 3
  while(FMC_Bank5_6->SDSR & FMC_SDSR_BUSY);
  FMC_Bank5_6->SDCMR = 1 | FMC_SDCMR_CTB2 | (1 << 5);
  // Initialization step 4
  for(tmp = 0; tmp < 1000000; tmp++);
  // Initialization step 5
  while(FMC_Bank5_6->SDSR & FMC_SDSR_BUSY);
  FMC_Bank5_6->SDCMR = 2 | FMC_SDCMR_CTB2 | (1 << 5);
  // Initialization step 6
  while(FMC_Bank5_6->SDSR & FMC_SDSR_BUSY);
  FMC_Bank5_6->SDCMR = 3 | FMC_SDCMR_CTB2 | (4 << 5);
  // Initialization step 7
  while(FMC_Bank5_6->SDSR & FMC_SDSR_BUSY);
  FMC_Bank5_6->SDCMR = 4 | FMC_SDCMR_CTB2 | (1 << 5) | (0x231 << 9);
  // Initialization step 8
  while(FMC_Bank5_6->SDSR & FMC_SDSR_BUSY);
  FMC_Bank5_6->SDRTR |= (683 << 1);
  while(FMC_Bank5_6->SDSR & FMC_SDSR_BUSY);
}



//=================================================================================================
// STM32F4 GPIO Configuration
// Author : Radoslaw Kwiecien
// e-mail : radek@dxp.pl
// http://en.radzio.dxp.pl/
//=================================================================================================
static void gpio_conf(GPIO_TypeDef * GPIO, uint8_t pin, uint8_t mode, uint8_t type, uint8_t speed, uint8_t pullup, uint8_t af)
{
#define MASK1BIT(pin) ((uint32_t)~(1 << (pin * 1)))
#define MASK2BIT(pin) ((uint32_t)~(3 << (pin * 2)))
#define MASK4BIT(pin) ((uint32_t)~(15 << (pin * 4)))
#define AFMASKL(pin)  ((uint32_t)~(15 << (pin * 4)))
#define AFMASKH(pin)  ((uint32_t)~(15 << ((pin - 8) * 4)))

    GPIO->MODER   = (GPIO->MODER   & MASK2BIT(pin))   | (mode << (pin * 2));
    GPIO->OTYPER  = (GPIO->OTYPER  & MASK1BIT(pin))   | (type << pin);
    GPIO->OSPEEDR = (GPIO->OSPEEDR & MASK2BIT(pin))   | (speed << (pin * 2));
    GPIO->PUPDR   = (GPIO->PUPDR   & MASK2BIT(pin))   | (pullup << (pin * 2));
    if(pin > 7)
      {
        GPIO->AFR[1] = (GPIO->AFR[1] & AFMASKH(pin)) | (af << ((pin - 8) * 4));
      }
    else
      {
        GPIO->AFR[0] = (GPIO->AFR[0] & AFMASKL(pin)) | (af << ((pin) * 4));
      }
}
