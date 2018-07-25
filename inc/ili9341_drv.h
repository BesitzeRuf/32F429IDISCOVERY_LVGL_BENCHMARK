/*
 * ili9341_drv.h
 *
 *  Created on: 23. 7. 2018
 *      Author: yurovv
 */

#ifndef ILI9341_DRV_H_
#define ILI9341_DRV_H_

/**** Includes ****/
#include "stm32f4xx.h"
#include "ili9341_port.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_bus.h"

#define LCD_ENABLE_VSYNC            0
//#define LCD_ENABLE_VSYNC            1


//Colors
#define ILI9341_COLOR_LIGHTRED      0x80FF8000
#define ILI9341_COLOR_LIGHTBLUE     0xFF800000
#define ILI9341_COLOR_LIGHTGREEN    0x0080FF00
#define ILI9341_COLOR_LIGHTYELLOW   0x80FFFF00
#define ILI9341_COLOR_LIGHTCYAN     0xFF80FF00
#define ILI9341_COLOR_LIGHTMAGENTA  0xFFFF8000
#define ILI9341_COLOR_LIGHTGRAY     0xD3D3D300

#define ILI9341_COLOR_DARKRED       0x00800000
#define ILI9341_COLOR_DARKBLUE      0x80000000
#define ILI9341_COLOR_DARKGREEN     0x00008000
#define ILI9341_COLOR_DARKYELLOW    0x00808000
#define ILI9341_COLOR_DARKCYAN      0x80008000
#define ILI9341_COLOR_DARKMAGENTA   0x80800000
#define ILI9341_COLOR_DARKGRAY      0x40404000





#define LCD_WIDTH         320 //320 //240
#define LCD_HEIGHT        240 //320
#define LCD_PIXELS        (LCD_WIDTH*LCD_HEIGHT)

#define LTDC_PIXEL_FORMAT_ARGB8888        0x00000000U   /*!< ARGB8888 LTDC pixel format */
#define LTDC_PIXEL_FORMAT_RGB888          0x00000001U   /*!< RGB888 LTDC pixel format   */
#define LTDC_PIXEL_FORMAT_RGB565          0x00000002U   /*!< RGB565 LTDC pixel format   */
#define LTDC_PIXEL_FORMAT_ARGB1555        0x00000003U   /*!< ARGB1555 LTDC pixel format */
#define LTDC_PIXEL_FORMAT_ARGB4444        0x00000004U   /*!< ARGB4444 LTDC pixel format */
#define LTDC_PIXEL_FORMAT_L8              0x00000005U   /*!< L8 LTDC pixel format       */
#define LTDC_PIXEL_FORMAT_AL44            0x00000006U   /*!< AL44 LTDC pixel format     */
#define LTDC_PIXEL_FORMAT_AL88            0x00000007U   /*!< AL88 LTDC pixel format     */

/*** Orientation types ***/
typedef enum {
  Orientation_Portrait_1 = 1,
  Orientation_Portrait_2,
  Orientation_Landscape_1,
  Orientation_Landscape_2,
  Orientation_Landscape_3,
  Orientation_Landscape_4
}OrientationType_e;

/**
 * LCD param for manual drawing/plotting graph
 */
typedef struct {
  uint16_t width;
  uint16_t height;
  uint16_t currentLayer;
  uint16_t currentLayerOffset;
  OrientationType_e orientType;
} LcdParam_st;


void ili9341_initLcd(void);
void ili9341_SwitchBuffers();
uint32_t ili9341_GetBufferToDraw();
uint32_t ili9341_GetBufferToView();












#endif /* ILI9341_DRV_H_ */
