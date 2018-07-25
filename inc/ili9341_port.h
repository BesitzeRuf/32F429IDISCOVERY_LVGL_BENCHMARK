/*
 * ili9341_port.h
 *
 *  Created on: 23. 7. 2018
 *      Author: yurovv
 */
#ifndef ILI9341_PORT_H_
#define ILI9341_PORT_H_

#include "stm32f4xx.h"
#include "stm32f4xx_ll_conf.h"

#include "tm_stm32f4_gpio.h"

/***************************************************************
LDTC PIN
*********************************************************************/

// rgb interface
#define LCD_GPIO_AF_LTDC1       LL_GPIO_AF_14   // Alternative Function as LDTC
#define LCD_GPIO_AF_LTDC2       LL_GPIO_AF_9    // Alternative Function as LDTC

#define LCD_R2_PCLK             RCC_AHB1ENR_GPIOCEN
#define LCD_R2_PORT             GPIOC
#define LCD_R2_PIN              LL_GPIO_PIN_10
#define LCD_R2_PIN_SRC          LL_GPIO_PIN_10

#define LCD_R3_PCLK             RCC_AHB1ENR_GPIOBEN
#define LCD_R3_PORT             GPIOB
#define LCD_R3_PIN              LL_GPIO_PIN_0
#define LCD_R3_PIN_SRC          LL_GPIO_PIN_0

#define LCD_R4_PCLK             RCC_AHB1ENR_GPIOAEN
#define LCD_R4_PORT             GPIOA
#define LCD_R4_PIN              LL_GPIO_PIN_11
#define LCD_R4_PIN_SRC          LL_GPIO_PIN_11

#define LCD_R5_PCLK             RCC_AHB1ENR_GPIOAEN
#define LCD_R5_PORT             GPIOA
#define LCD_R5_PIN              LL_GPIO_PIN_12
#define LCD_R5_PIN_SRC          LL_GPIO_PIN_12

#define LCD_R6_PCLK             RCC_AHB1ENR_GPIOBEN
#define LCD_R6_PORT             GPIOB
#define LCD_R6_PIN              LL_GPIO_PIN_1
#define LCD_R6_PIN_SRC          LL_GPIO_PIN_1

#define LCD_R7_PCLK             RCC_AHB1ENR_GPIOGEN
#define LCD_R7_PORT             GPIOG
#define LCD_R7_PIN              LL_GPIO_PIN_6
#define LCD_R7_PIN_SRC          LL_GPIO_PIN_6


#define LCD_G2_PCLK             RCC_AHB1ENR_GPIOAEN
#define LCD_G2_PORT             GPIOA
#define LCD_G2_PIN              LL_GPIO_PIN_6
#define LCD_G2_PIN_SRC          LL_GPIO_PIN_6

#define LCD_G3_PCLK             RCC_AHB1ENR_GPIOGEN
#define LCD_G3_PORT             GPIOG
#define LCD_G3_PIN              LL_GPIO_PIN_10
#define LCD_G3_PIN_SRC          LL_GPIO_PIN_10

#define LCD_G4_PCLK             RCC_AHB1ENR_GPIOBEN
#define LCD_G4_PORT             GPIOB
#define LCD_G4_PIN              LL_GPIO_PIN_10
#define LCD_G4_PIN_SRC          LL_GPIO_PIN_10

#define LCD_G5_PCLK             RCC_AHB1ENR_GPIOBEN
#define LCD_G5_PORT             GPIOB
#define LCD_G5_PIN              LL_GPIO_PIN_11
#define LCD_G5_PIN_SRC          LL_GPIO_PIN_11

#define LCD_G6_PCLK             RCC_AHB1ENR_GPIOCEN
#define LCD_G6_PORT             GPIOC
#define LCD_G6_PIN              LL_GPIO_PIN_7
#define LCD_G6_PIN_SRC          LL_GPIO_PIN_7

#define LCD_G7_PCLK             RCC_AHB1ENR_GPIODEN
#define LCD_G7_PORT             GPIOD
#define LCD_G7_PIN              LL_GPIO_PIN_3
#define LCD_G7_PIN_SRC          LL_GPIO_PIN_3

#define LCD_B2_PCLK             RCC_AHB1ENR_GPIODEN
#define LCD_B2_PORT             GPIOD
#define LCD_B2_PIN              LL_GPIO_PIN_6
#define LCD_B2_PIN_SRC          LL_GPIO_PIN_6

#define LCD_B3_PCLK             RCC_AHB1ENR_GPIOGEN
#define LCD_B3_PORT             GPIOG
#define LCD_B3_PIN              LL_GPIO_PIN_11
#define LCD_B3_PIN_SRC          LL_GPIO_PIN_11

#define LCD_B4_PCLK             RCC_AHB1ENR_GPIOGEN
#define LCD_B4_PORT             GPIOG
#define LCD_B4_PIN              LL_GPIO_PIN_12
#define LCD_B4_PIN_SRC          LL_GPIO_PIN_12

#define LCD_B5_PCLK             RCC_AHB1ENR_GPIOAEN
#define LCD_B5_PORT             GPIOA
#define LCD_B5_PIN              LL_GPIO_PIN_3
#define LCD_B5_PIN_SRC          LL_GPIO_PIN_3

#define LCD_B6_PCLK             RCC_AHB1ENR_GPIOBEN
#define LCD_B6_PORT             GPIOB
#define LCD_B6_PIN              LL_GPIO_PIN_8
#define LCD_B6_PIN_SRC          LL_GPIO_PIN_8

#define LCD_B7_PCLK             RCC_AHB1ENR_GPIOBEN
#define LCD_B7_PORT             GPIOB
#define LCD_B7_PIN              LL_GPIO_PIN_9
#define LCD_B7_PIN_SRC          LL_GPIO_PIN_9

#define LCD_CLK_PCLK            RCC_AHB1ENR_GPIOGEN
#define LCD_CLK_PORT            GPIOG
#define LCD_CLK_PIN             LL_GPIO_PIN_7
#define LCD_CLK_PIN_SRC         LL_GPIO_PIN_7

#define LCD_VSYNC_PCLK          RCC_AHB1ENR_GPIOAEN
#define LCD_VSYNC_PORT          GPIOA
#define LCD_VSYNC_PIN           LL_GPIO_PIN_4
#define LCD_VSYNC_PIN_SRC       LL_GPIO_PIN_4

#define LCD_HSYNC_PCLK          RCC_AHB1ENR_GPIOCEN
#define LCD_HSYNC_PORT          GPIOC
#define LCD_HSYNC_PIN           LL_GPIO_PIN_6
#define LCD_HSYNC_PIN_SRC       LL_GPIO_PIN_6

#define LCD_DE_PCLK             RCC_AHB1ENR_GPIOFEN
#define LCD_DE_PORT             GPIOF
#define LCD_DE_PIN              LL_GPIO_PIN_10
#define LCD_DE_PIN_SRC          LL_GPIO_PIN_10

#define LCD_WRX_PCLK             RCC_AHB1ENR_GPIODEN
#define LCD_WRX_PORT             GPIOD
#define LCD_WRX_PIN              LL_GPIO_PIN_13
#define LCD_WRX_PIN_SRC          LL_GPIO_PIN_13




#define ILI9341_MOSI_PCLK         RCC_AHB1ENR_GPIOFEN
#define ILI9341_MOSI_PORT         GPIOF
#define ILI9341_MOSI_PIN          LL_GPIO_PIN_9

#define ILI9341_SCL_PCLK          RCC_AHB1ENR_GPIOFEN
#define ILI9341_SCL_PORT          GPIOF
#define ILI9341_SCL_PIN           LL_GPIO_PIN_7




/* Change custom CS, DC and RESET pins */
/* CS pin */
#define ILI9341_CS_PCLK       RCC_AHB1ENR_GPIOCEN
#define ILI9341_CS_PORT       GPIOC
#define ILI9341_CS_PIN        GPIO_PIN_2
/* WRX or DC pin */

#define ILI9341_WRX_PCLK      RCC_AHB1ENR_GPIODEN
#define ILI9341_WRX_PORT      GPIOD
#define ILI9341_WRX_PIN       GPIO_PIN_13



#define ILI9341_CS_SET        LL_GPIO_SetOutputPin(ILI9341_CS_PORT, ILI9341_CS_PIN)
#define ILI9341_CS_RESET      LL_GPIO_ResetOutputPin(ILI9341_CS_PORT, ILI9341_CS_PIN)
#define ILI9341_WRX_SET       LL_GPIO_SetOutputPin(ILI9341_WRX_PORT, ILI9341_WRX_PIN)
#define ILI9341_WRX_RESET     LL_GPIO_ResetOutputPin(ILI9341_WRX_PORT, ILI9341_WRX_PIN)

#define ILI9341_SCL_SET        LL_GPIO_SetOutputPin(ILI9341_SCL_PORT, ILI9341_SCL_PIN)
#define ILI9341_SCL_RESET      LL_GPIO_ResetOutputPin(ILI9341_SCL_PORT, ILI9341_SCL_PIN)
#define ILI9341_MOSI_SET       LL_GPIO_SetOutputPin(ILI9341_MOSI_PORT, ILI9341_MOSI_PIN)
#define ILI9341_MOSI_RESET     LL_GPIO_ResetOutputPin(ILI9341_MOSI_PORT, ILI9341_MOSI_PIN)


/* Commands */
#define ILI9341_RESET       0x01
#define ILI9341_SLEEP_OUT     0x11
#define ILI9341_GAMMA       0x26
#define ILI9341_DISPLAY_OFF     0x28
#define ILI9341_DISPLAY_ON      0x29
#define ILI9341_COLUMN_ADDR     0x2A
#define ILI9341_PAGE_ADDR     0x2B
#define ILI9341_GRAM        0x2C
#define ILI9341_MAC         0x36
#define ILI9341_PIXEL_FORMAT    0x3A
#define ILI9341_WDB         0x51
#define ILI9341_WCD         0x53
#define ILI9341_RGB_INTERFACE   0xB0
#define ILI9341_FRC         0xB1
#define ILI9341_BPC         0xB5
#define ILI9341_DFC         0xB6
#define ILI9341_POWER1        0xC0
#define ILI9341_POWER2        0xC1
#define ILI9341_VCOM1       0xC5
#define ILI9341_VCOM2       0xC7
#define ILI9341_POWERA        0xCB
#define ILI9341_POWERB        0xCF
#define ILI9341_PGAMMA        0xE0
#define ILI9341_NGAMMA        0xE1
#define ILI9341_DTCA        0xE8
#define ILI9341_DTCB        0xEA
#define ILI9341_POWER_SEQ     0xED
#define ILI9341_3GAMMA_EN     0xF2
#define ILI9341_INTERFACE     0xF6
#define ILI9341_PRC         0xF7


#endif /* ILI9341_PORT_H_ */
