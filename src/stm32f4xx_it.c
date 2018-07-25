/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @author  Ac6
  * @version V1.0
  * @date    02-Feb-2015
  * @brief   Default Interrupt Service Routines.
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#ifdef USE_RTOS_SYSTICK
#include <cmsis_os.h>
#endif
#include "stm32f4xx_it.h"

#include "lvgl.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            	  	    Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles SysTick Handler, but only if no RTOS defines it.
  * @param  None
  * @retval None
  */

volatile uint32_t nFpsCounter = 0;
volatile uint32_t nMFpsResult = 0;
static volatile uint32_t nTimeFromLastFPS_ms = 0;


void SysTick_Handler(void)
{
  lv_tick_inc(1);

  nTimeFromLastFPS_ms++;
  if (nTimeFromLastFPS_ms % 1000 == 0)
  {
    nMFpsResult = nFpsCounter;
    nFpsCounter = 0;
  }

}
