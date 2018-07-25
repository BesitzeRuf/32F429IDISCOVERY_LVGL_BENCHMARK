/*
 * lcd.c
 *
 *  Created on: 23. 7. 2018
 *      Author: yurovv
 */

#ifdef __cplusplus
extern "C" {
#endif


/*********************
 *      INCLUDES
 *********************/
#include "lcd.h"
#include "ili9341_drv.h"
#include "stmpe811_drv.h"


#include "sdram.h"
#include "lvgl.h"
#include "lv_core/lv_refr.h"

#include "stm32f4xx.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_dma2d.h"


/*********************
 *      DEFINES
 *********************/
#define DMA                      DMA2
#define DMA_STREAM               LL_DMA_STREAM_0
#define DMA_CHANNEL              LL_DMA_CHANNEL_0
#define DMA_STREAM_IRQ           DMA2_Stream0_IRQn
#define DMA_STREAM_IRQHANDLER    DMA2_Stream0_IRQHandler

#define GH LV_VER_RES
#define GW LV_HOR_RES

#define LH  320
#define LW  240

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void lv_hal_lcd_flush(int32_t x1, int32_t y1, int32_t x2, int32_t y2, const lv_color_t * color_p);
static void lv_hal_lcd_map(int32_t x1, int32_t y1, int32_t x2, int32_t y2, const lv_color_t * color_p);
static void lv_hal_lcd_fill(int32_t x1, int32_t y1, int32_t x2, int32_t y2,  lv_color_t color);
#if USE_LV_GPU
static void ex_mem_blend(lv_color_t * dest, const lv_color_t * src, uint32_t length, lv_opa_t opa);
static void ex_mem_fill(lv_color_t * dest, uint32_t length, lv_color_t color);
#endif


static bool lv_hal_lcd_tp_read(lv_indev_data_t *data);
static void lv_hal_lcd_gpu_init(void);

typedef struct
{
  lv_area_t   pAreas[LV_INV_FIFO_SIZE];
  uint32_t    nCount;
}lv_areas_to_refresh;

lv_areas_to_refresh  oAreasToRefresh;

static void lv_hal_lcd_on_refresh_end(void* pData, uint32_t nCount);


static void ProcessFlushingDma2d();
static void ProcessBlendingDma2d();
static void ProcessFillingDma2d();
static void ProcessSwappingBuffersDma2D();

static touch_info_t touchInfo;
static int16_t last_x = 0;
static int16_t last_y = 0;

static volatile lv_color_t * pBuffToDraw = NULL;
static volatile lv_color_t * pBuffToView = NULL;


uint32_t nScreenWasRefreshed = 0;

typedef enum
{
  E_DMA2D_IS_READY            = 0,
  E_DMA2D_IS_FLUSHING         = 1,
  E_DMA2D_IS_BLENDING         = 2,
  E_DMA2D_IS_FILLING          = 3,
  E_DMA2D_IS_SWAPPING_BUFFERS = 4,

}T_DMA2D_TRANSFER_STATE;


typedef struct
{
  T_DMA2D_TRANSFER_STATE  eState;

  int16_t    nLinesToProcess;
  uint32_t   nBytesToProcess;
  uint32_t   nAddressOut;
  uint32_t   nAddressFG;
  uint32_t   nAddressBG;
}T_DMA2D_EVENT_HANDLE;

volatile T_DMA2D_EVENT_HANDLE  oDma2dHandle = { .eState = E_DMA2D_IS_READY };



void lv_hal_lcd_init(void)
{
  /***********************
   * Display interface
   ***********************/
  SDRAM_Init();
  ili9341_initLcd();
  stmpe811_Init(LW, LH);

  lv_hal_lcd_gpu_init();

  lv_disp_drv_t disp_drv; /*Descriptor of a display driver*/
  lv_disp_drv_init(&disp_drv); /*Basic initialization*/

  /*Set up the functions to access to your display*/
  disp_drv.disp_flush = lv_hal_lcd_flush; /*Used in buffered mode (LV_VDB_SIZE != 0  in lv_conf.h)*/

  disp_drv.disp_fill = lv_hal_lcd_fill; /*Used in unbuffered mode (LV_VDB_SIZE == 0  in lv_conf.h)*/
  disp_drv.disp_map = lv_hal_lcd_map; /*Used in unbuffered mode (LV_VDB_SIZE == 0  in lv_conf.h)*/

#if USE_LV_GPU
  /*Optionally add functions to access the GPU. (Only in buffered mode, LV_VDB_SIZE != 0)*/
  disp_drv.mem_blend = ex_mem_blend; /*Blend two color array using opacity*/
  disp_drv.mem_fill = ex_mem_fill; /*Fill a memory array with a color*/
#endif

#if (LCD_ENABLE_VSYNC == 1)
  lv_refr_set_onScreenRefreshed_cb(lv_hal_lcd_on_refresh_end);
#endif

  /*Finally register the driver*/
  lv_disp_drv_register(&disp_drv);

  /*************************
   * Input device interface
   *************************/
  /*Add a touchpad in the example*/
//    lcd_touch_panel_init();
  lv_indev_drv_t indev_drv; /*Descriptor of an input device driver*/
  lv_indev_drv_init(&indev_drv); /*Basic initialization*/
  indev_drv.type = LV_INDEV_TYPE_POINTER; /*The touchpad is pointer type device*/
  indev_drv.read = lv_hal_lcd_tp_read; /*Library ready your touchpad via this function*/
  lv_indev_drv_register(&indev_drv); /*Finally register the driver*/

  pBuffToView = (lv_color_t*)ili9341_GetBufferToView();
  pBuffToDraw = (lv_color_t*)ili9341_GetBufferToDraw();
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

/* Flush the content of the internal buffer the specific area on the display
 * You can use DMA or any hardware acceleration to do this operation in the background but
 * 'lv_flush_ready()' has to be called when finished
 * This function is required only when LV_VDB_SIZE != 0 in lv_conf.h*/
static void lv_hal_lcd_flush(int32_t x1, int32_t y1, int32_t x2, int32_t y2, const lv_color_t * color_p)
{
  // Return if the area is out the screen
  if(x2 < 0) return;
  if(y2 < 0) return;
  if(x1 > LV_HOR_RES - 1) return;
  if(y1 > LV_VER_RES - 1) return;

  /*Truncate the area to the screen*/
  int32_t act_x1 = x1 < 0 ? 0 : x1;
  int32_t act_y1 = y1 < 0 ? 0 : y1;
  int32_t act_x2 = x2 > LV_HOR_RES - 1 ? LV_HOR_RES - 1 : x2;
  int32_t act_y2 = y2 > LV_VER_RES - 1 ? LV_VER_RES - 1 : y2;


  while (DMA2D->CR & DMA2D_CR_START);
  while (oDma2dHandle.eState != E_DMA2D_IS_READY){ }
  oDma2dHandle.eState = E_DMA2D_IS_FLUSHING;
  DMA2D->CR = 0;

  oDma2dHandle.nAddressFG      = (uint32_t)color_p;
  oDma2dHandle.nAddressOut     = (uint32_t)&pBuffToDraw[ (LW - 1) - act_y1 + act_x1*LW];
  oDma2dHandle.nBytesToProcess = (act_x2 - act_x1 + 1) * 2;  // multiplied by count of bytes per pixel
  oDma2dHandle.nLinesToProcess = (act_y2 - act_y1 + 1 - 1);

  LL_DMA2D_FGND_SetColorMode(DMA2D, LL_DMA2D_INPUT_MODE_RGB565);
  LL_DMA2D_FGND_SetLineOffset(DMA2D, 0);
  //LL_DMA2D_FGND_SetMemAddr(DMA2D,   (uint32_t)color_p);

  LL_DMA2D_SetOutputMemAddr(DMA2D, oDma2dHandle.nAddressOut);
  LL_DMA2D_FGND_SetMemAddr(DMA2D, oDma2dHandle.nAddressFG );



  LL_DMA2D_SetLineOffset(DMA2D, (LW - 1));
  LL_DMA2D_SetNbrOfLines(DMA2D, oDma2dHandle.nBytesToProcess / 2);
  LL_DMA2D_SetNbrOfPixelsPerLines(DMA2D, 1);
  //LL_DMA2D_SetOutputMemAddr(DMA2D, (uint32_t)&my_fb[ (LW - 1) - act_y1 + act_x1*LW]);


  DMA2D->CR |= DMA2D_CR_TCIE | DMA2D_CR_START;

}


/* Write a pixel array (called 'map') to the a specific area on the display
 * This function is required only when LV_VDB_SIZE == 0 in lv_conf.h*/
static void lv_hal_lcd_map(int32_t x1, int32_t y1, int32_t x2, int32_t y2, const lv_color_t * color_p)
{
   // Return if the area is out the screen
   if(x2 < 0) return;
   if(y2 < 0) return;
   if(x1 > LV_HOR_RES - 1) return;
   if(y1 > LV_VER_RES - 1) return;

   /*Truncate the area to the screen*/
   int32_t act_x1 = x1 < 0 ? 0 : x1;
   int32_t act_y1 = y1 < 0 ? 0 : y1;
   int32_t act_x2 = x2 > LV_HOR_RES - 1 ? LV_HOR_RES - 1 : x2;
   int32_t act_y2 = y2 > LV_VER_RES - 1 ? LV_VER_RES - 1 : y2;


   while (DMA2D->CR & DMA2D_CR_START);
   while (oDma2dHandle.eState != E_DMA2D_IS_READY){ }
   oDma2dHandle.eState = E_DMA2D_IS_FLUSHING;
   DMA2D->CR = 0;

   oDma2dHandle.nAddressFG      = (uint32_t)color_p;
   oDma2dHandle.nAddressOut     = (uint32_t)&pBuffToDraw[ (LW - 1) - act_y1 + act_x1*LW];
   oDma2dHandle.nBytesToProcess = (act_x2 - act_x1 + 1) * 2;  // multiplied by count of bytes per pixel
   oDma2dHandle.nLinesToProcess = (act_y2 - act_y1 + 1 - 1);

   LL_DMA2D_FGND_SetColorMode(DMA2D, LL_DMA2D_INPUT_MODE_RGB565);
   LL_DMA2D_FGND_SetLineOffset(DMA2D, 0);
   //LL_DMA2D_FGND_SetMemAddr(DMA2D,   (uint32_t)color_p);

   LL_DMA2D_SetOutputMemAddr(DMA2D, oDma2dHandle.nAddressOut);
   LL_DMA2D_FGND_SetMemAddr(DMA2D, oDma2dHandle.nAddressFG );

   LL_DMA2D_SetLineOffset(DMA2D, (LW - 1));
   LL_DMA2D_SetNbrOfLines(DMA2D, oDma2dHandle.nBytesToProcess / 2);
   LL_DMA2D_SetNbrOfPixelsPerLines(DMA2D, 1);
   //LL_DMA2D_SetOutputMemAddr(DMA2D, (uint32_t)&my_fb[ (LW - 1) - act_y1 + act_x1*LW]);


   DMA2D->CR |= DMA2D_CR_TCIE | DMA2D_CR_START;

}


/* Write a pixel array (called 'map') to the a specific area on the display
 * This function is required only when LV_VDB_SIZE == 0 in lv_conf.h*/
static void lv_hal_lcd_fill(int32_t x1, int32_t y1, int32_t x2, int32_t y2,  lv_color_t color)
{

  // Return if the area is out the screen
    if(x2 < 0) return;
    if(y2 < 0) return;
    if(x1 > LV_HOR_RES - 1) return;
    if(y1 > LV_VER_RES - 1) return;

    /*Truncate the area to the screen*/
    int32_t act_x1 = x1 < 0 ? 0 : x1;
    int32_t act_y1 = y1 < 0 ? 0 : y1;
    int32_t act_x2 = x2 > LV_HOR_RES - 1 ? LV_HOR_RES - 1 : x2;
    int32_t act_y2 = y2 > LV_VER_RES - 1 ? LV_VER_RES - 1 : y2;

    while (DMA2D->CR & DMA2D_CR_START);
    while (oDma2dHandle.eState != E_DMA2D_IS_READY){ }
    oDma2dHandle.eState = E_DMA2D_IS_FILLING;
    DMA2D->CR = 0;

  oDma2dHandle.nAddressOut = (uint32_t) &pBuffToDraw[(LW - 1) - act_y1 + act_x1 * LW];
  oDma2dHandle.nLinesToProcess = (act_y2 - act_y1 + 1 - 1);


    LL_DMA2D_SetOutputColor(DMA2D, color.full);
  LL_DMA2D_SetOutputMemAddr(DMA2D, oDma2dHandle.nAddressOut);
  LL_DMA2D_SetLineOffset(DMA2D, (LW - (act_y2 - act_y1 + 1)));
  LL_DMA2D_SetNbrOfLines(DMA2D, (act_x2 - act_x1 + 1));
  LL_DMA2D_SetNbrOfPixelsPerLines(DMA2D, act_y2 - act_y1 + 1);

  DMA2D->CR |= LL_DMA2D_MODE_R2M | DMA2D_CR_TCIE | DMA2D_CR_START;
}

#if USE_LV_GPU

/* If your MCU has hardware accelerator (GPU) then you can use it to blend to memories using opacity
 * It can be used only in buffered mode (LV_VDB_SIZE != 0 in lv_conf.h)*/
static void ex_mem_blend(lv_color_t * dest, const lv_color_t * src, uint32_t length, lv_opa_t opa)
{
  while (oDma2dHandle.eState != E_DMA2D_IS_READY);
  while (LL_DMA2D_IsTransferOngoing(DMA2D));
  oDma2dHandle.eState = E_DMA2D_IS_BLENDING;

  DMA2D->CR = 0;
  // Output
  LL_DMA2D_SetOutputMemAddr(DMA2D, (uint32_t) dest);
  LL_DMA2D_SetLineOffset(DMA2D, 0);
  LL_DMA2D_SetNbrOfLines(DMA2D, 1);
  LL_DMA2D_SetNbrOfPixelsPerLines(DMA2D, length);

  // foreground

  LL_DMA2D_FGND_SetMemAddr(DMA2D, (uint32_t) src);
  LL_DMA2D_FGND_SetAlpha(DMA2D, opa);

  // background
  LL_DMA2D_BGND_SetMemAddr(DMA2D, (uint32_t) dest);


  oDma2dHandle.nLinesToProcess = 0; // here we need to process only 1 line
  DMA2D->CR = LL_DMA2D_MODE_M2M_BLEND | DMA2D_CR_TCIE | DMA2D_CR_START;

}

/* If your MCU has hardware accelerator (GPU) then you can use it to fill a memory with a color
 * It can be used only in buffered mode (LV_VDB_SIZE != 0 in lv_conf.h)*/
static void ex_mem_fill(lv_color_t * dest, uint32_t length, lv_color_t color)
{
  while (oDma2dHandle.eState != E_DMA2D_IS_READY);
  while (LL_DMA2D_IsTransferOngoing(DMA2D)){;}
  oDma2dHandle.eState = E_DMA2D_IS_FILLING;
  DMA2D->CR = 0;

  LL_DMA2D_SetOutputColor(DMA2D, color.full);

  LL_DMA2D_SetLineOffset(DMA2D, 0);
  LL_DMA2D_SetNbrOfPixelsPerLines(DMA2D, length);
  LL_DMA2D_SetNbrOfLines(DMA2D, 1);

  LL_DMA2D_SetOutputMemAddr(DMA2D, (uint32_t)dest);


  DMA2D->CR = LL_DMA2D_MODE_R2M | DMA2D_CR_TCIE | DMA2D_CR_START;

}

#endif

static void lv_hal_lcd_flush_area(lv_area_t* pArea)
{
   /*Truncate the area to the screen*/
   int32_t lcd_x1 = LW - pArea->y2 - 1;
   int32_t lcd_y1 = pArea->x1;  // done
   int32_t lcd_x2 = LW - pArea->y1 - 1;
   int32_t lcd_y2 = pArea->x2;  // done

   DMA2D->CR = 0;

   LL_DMA2D_SetOutputMemAddr(DMA2D, (uint32_t)&pBuffToDraw[ lcd_y1*LW + lcd_x1 ]); // done ?

   LL_DMA2D_FGND_SetMemAddr(DMA2D, (uint32_t)&pBuffToView[lcd_y1*LW + lcd_x1 ]); // done ?
   LL_DMA2D_FGND_SetLineOffset(DMA2D, (LW - (lcd_x2 - lcd_x1 + 1) ));


   LL_DMA2D_SetLineOffset(DMA2D, (LW - (lcd_x2 - lcd_x1 + 1) ));
   LL_DMA2D_SetNbrOfLines(DMA2D, lcd_y2 - lcd_y1 + 1);    // done ?
   LL_DMA2D_SetNbrOfPixelsPerLines(DMA2D, lcd_x2 - lcd_x1 + 1 );

   DMA2D->CR |= LL_DMA2D_MODE_M2M | DMA2D_CR_TCIE | DMA2D_CR_START;

}

static void lv_hal_lcd_on_refresh_end(void* pData, uint32_t nCount)
{
  if (nCount == 0) return;

  while (oDma2dHandle.eState != E_DMA2D_IS_READY);
  while (LL_DMA2D_IsTransferOngoing(DMA2D)){;}
  oDma2dHandle.eState = E_DMA2D_IS_SWAPPING_BUFFERS;


  ili9341_SwitchBuffers();
  pBuffToDraw = (lv_color_t*) ili9341_GetBufferToDraw();
  pBuffToView = (lv_color_t*) ili9341_GetBufferToView();

  //oDma2dHandle.nAreaToRefreshIndex = 0;
  //lv_hal_lcd_flush_area(&oAreasToRefresh.pAreas[oDma2dHandle.nAreaToRefreshIndex]);

  lv_area_t oArea;
  if (nCount == 1000)
  {
    lv_join_t* pAreaToCopy = (lv_join_t*) pData;
    oArea.x1 = pAreaToCopy->area.x1;
    oArea.y1 = pAreaToCopy->area.y1;
    oArea.x2 = pAreaToCopy->area.x2;
    oArea.y2 = pAreaToCopy->area.y2;
  }
  else
  {
    oArea.x1 = 0;
    oArea.y1 = 0;
    oArea.x2 = GW - 1;
    oArea.y2 = GH - 1;
  }

  lv_hal_lcd_flush_area(&oArea);
}



/* Read the touchpad and store it in 'data'
 * REaturn false if no more data read; true for ready again */
static bool lv_hal_lcd_tp_read(lv_indev_data_t *data)
{
  stmpe811_Scan(&touchInfo);

  if (touchInfo.bTouchDetected == 1)
  {
    data->point.x = touchInfo.y;      //touchInfo.x;
    data->point.y = LW - touchInfo.x; //touchInfo.y;
    last_x = data->point.x;
    last_y = data->point.y;
    data->state = LV_INDEV_STATE_PR;
  }
  else
  {
    data->point.x = last_x;
    data->point.y = last_y;
    data->state = LV_INDEV_STATE_REL;
  }

  return false;
}


void lv_hal_lcd_gpu_init()
{
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA2DEN;

  LL_DMA2D_SetOutputColorMode(DMA2D, LL_DMA2D_OUTPUT_MODE_RGB565);

  LL_DMA2D_FGND_SetColorMode(DMA2D, LL_DMA2D_INPUT_MODE_RGB565);
  LL_DMA2D_FGND_SetAlphaMode(DMA2D, LL_DMA2D_ALPHA_MODE_REPLACE);

  LL_DMA2D_BGND_SetColorMode(DMA2D, LL_DMA2D_INPUT_MODE_RGB565);
  LL_DMA2D_BGND_SetAlphaMode(DMA2D, LL_DMA2D_ALPHA_MODE_REPLACE);
  LL_DMA2D_BGND_SetAlpha(DMA2D, 0xFF);

  NVIC_SetPriority(DMA2D_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 1, 0));
  NVIC_EnableIRQ(DMA2D_IRQn);
}

#if USE_LV_GPU != 0




void DMA2D_IRQHandler(void)
{
  // Transfer error interrupt management
  if (LL_DMA2D_IsActiveFlag_TE(DMA2D))
  {
    // Clear the transfer Error flag
    LL_DMA2D_ClearFlag_TE(DMA2D);
    if (LL_DMA2D_IsEnabledIT_TE(DMA2D))
    {
      // Disable the transfer error interrupt
      LL_DMA2D_DisableIT_TE(DMA2D);
    }
    // TODO: Process the transfer error event
  }

  // Configuration error interrupt management
  if (LL_DMA2D_IsActiveFlag_CE(DMA2D))
  {
    // Clear the configuration error flag
    LL_DMA2D_ClearFlag_CE(DMA2D);
    if (LL_DMA2D_IsEnabledIT_CE(DMA2D))
    {
      // Disable the configuration error interrupt
      LL_DMA2D_DisableIT_CE(DMA2D);
    }
    // TODO: Process the configuration error events
  }

  // CLUT access Error Interrupt management
  if (LL_DMA2D_IsActiveFlag_CAE(DMA2D))
  {
    // Clear the CLUT access error flag
    LL_DMA2D_ClearFlag_CAE(DMA2D);
    if (LL_DMA2D_IsEnabledIT_CAE(DMA2D))
    {
      // Disable the CLUT access error interrupt
      LL_DMA2D_DisableIT_CAE(DMA2D);
    }
    // TODO: Process the configuration error event
  }

  // Transfer watermark interrupt management
  if (LL_DMA2D_IsActiveFlag_TW(DMA2D))
  {
    // Clear the transfer watermark flag
    LL_DMA2D_ClearFlag_TW(DMA2D);
    if (LL_DMA2D_IsEnabledIT_TW(DMA2D))
    {
      // Disable the transfer watermark interrupt
      LL_DMA2D_DisableIT_TW(DMA2D);
    }
    // TODO: Process the transfer watermark event
  }

  // CLUT transfer complete interrupt management
  if (LL_DMA2D_IsActiveFlag_CTC(DMA2D))
  {
    // Clear the CLUT transfer complete flag
    LL_DMA2D_ClearFlag_CTC(DMA2D);
    if (LL_DMA2D_IsEnabledIT_CTC(DMA2D))
    {
      // Disable the CLUT transfer complete interrupt
      LL_DMA2D_DisableIT_CTC(DMA2D);
      // TODO: Process the CLUT transfer complete event
    }
  }

  if (LL_DMA2D_IsActiveFlag_TC(DMA2D))
  {
    // Clear the transfer complete flag
    LL_DMA2D_ClearFlag_TC(DMA2D);
    if (LL_DMA2D_IsEnabledIT_TC(DMA2D))
    {
      //LL_DMA2D_DisableIT_TC(DMA2D);
    }

    switch (oDma2dHandle.eState)
    {
    case E_DMA2D_IS_FLUSHING          : ProcessFlushingDma2d(); return;
    case E_DMA2D_IS_BLENDING          : ProcessBlendingDma2d(); return;
    case E_DMA2D_IS_FILLING           : ProcessFillingDma2d(); return;
    case E_DMA2D_IS_SWAPPING_BUFFERS  : ProcessSwappingBuffersDma2D(); return;
    case E_DMA2D_IS_READY             : break;

    }
  }
}


static void ProcessFlushingDma2d()
{
  if (oDma2dHandle.nLinesToProcess > 0)
  {

    oDma2dHandle.nAddressOut -= 2;
    oDma2dHandle.nAddressFG  += oDma2dHandle.nBytesToProcess;
    oDma2dHandle.nLinesToProcess--;

    LL_DMA2D_SetOutputMemAddr(DMA2D, oDma2dHandle.nAddressOut);
    LL_DMA2D_FGND_SetMemAddr(DMA2D, oDma2dHandle.nAddressFG );

    DMA2D->CR |= DMA2D_CR_START;
  }
  else
  {
    LL_DMA2D_DisableIT_TC(DMA2D);
    oDma2dHandle.eState = E_DMA2D_IS_READY;
    lv_flush_ready();
  }
}

static void ProcessBlendingDma2d()
{
  if (oDma2dHandle.nLinesToProcess > 0)
  {
    oDma2dHandle.nLinesToProcess--;
    DMA2D->CR |= LL_DMA2D_MODE_M2M_BLEND | DMA2D_CR_START;
  }
  else
  {
    oDma2dHandle.eState = E_DMA2D_IS_READY;
    LL_DMA2D_DisableIT_TC(DMA2D);
  }
}


static void ProcessFillingDma2d()
{
  LL_DMA2D_DisableIT_TC(DMA2D);
  oDma2dHandle.eState = E_DMA2D_IS_READY;
}

static void ProcessSwappingBuffersDma2D()
{
  if (nScreenWasRefreshed == 0)
  {
    nScreenWasRefreshed = 1;
  }
  oDma2dHandle.eState = E_DMA2D_IS_READY;
}

#endif

#ifdef __cplusplus
}
#endif
