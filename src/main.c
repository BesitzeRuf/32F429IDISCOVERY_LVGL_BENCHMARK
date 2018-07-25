
#include "stm32f4xx.h"
#include "stm32f4xx_ll_conf.h"

#include "lvgl.h"
#include "benchmark.h"

#include "lcd.h"
#include "sdram.h"

void SystemClock_Config(void);
void Experimental();

int main()
{
  SystemInit();
  SystemClock_Config();

  lv_init();

  lv_hal_lcd_init();

  benchmark_create();
//  Experimental();

  while (1)
  {
    lv_task_handler();
  }

  return 0;
}

static lv_style_t oStyleScreenHome;
static lv_style_t oStyleScreenSmart;
void Experimental()
{
  lv_obj_t* oParent = lv_scr_act();
  lv_obj_t* oScreenHome;
  lv_obj_t* oScreenSmart;

  lv_style_copy(&oStyleScreenHome, lv_obj_get_style(oParent));
  lv_style_copy(&oStyleScreenSmart, lv_obj_get_style(oParent));

  oScreenHome = lv_obj_create(oParent, NULL);
  oScreenSmart = lv_obj_create(oScreenHome, NULL);

  oStyleScreenHome.body.main_color = LV_COLOR_WHITE;
  oStyleScreenHome.body.grad_color = LV_COLOR_WHITE;

  oStyleScreenSmart.body.main_color = LV_COLOR_YELLOW;
  oStyleScreenSmart.body.grad_color = LV_COLOR_YELLOW;

  lv_obj_set_size(oScreenHome, 200,200);
  lv_obj_set_pos(oScreenHome, 10,10);


  lv_obj_set_size(oScreenSmart, 50,100);
  lv_obj_set_pos(oScreenSmart, 50,20);


  lv_obj_set_style(oScreenHome, &oStyleScreenHome);
  lv_obj_set_style(oScreenSmart, &oStyleScreenSmart);
}

void SystemClock_Config(void)
{
  LL_FLASH_EnableInstCache();
  LL_FLASH_EnableDataCache();
  LL_FLASH_EnablePrefetch();

  LL_RCC_HSE_Enable();
  while(LL_RCC_HSE_IsReady() != 1);

  LL_FLASH_SetLatency(LL_FLASH_LATENCY_5);

  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_8, 360, LL_RCC_PLLP_DIV_2);
  LL_RCC_PLL_Enable();
  while(LL_RCC_PLL_IsReady() != 1);

  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_PWR_EnableOverDriveMode();

  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_4);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL);

  LL_RCC_SetTIMPrescaler(LL_RCC_TIM_PRESCALER_TWICE);

  LL_SYSTICK_SetClkSource(LL_SYSTICK_CLKSOURCE_HCLK);
  LL_SetSystemCoreClock(180000000);
  LL_InitTick(180000000, 1000U);

  // SysTick_IRQn interrupt configuration
  NVIC_EnableIRQ(SysTick_IRQn);
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
}
