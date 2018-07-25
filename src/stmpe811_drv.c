/*
 * stmpe811_drv.c
 *
 *  Created on: 24. 7. 2018
 *      Author: yurovv
 */


#include "stmpe811_drv.h"
#include "stm32f4xx.h"
#include "stm32f4xx_ll_conf.h"

#include "tm_stm32f4_i2c.h"

#define IO_I2C_ADDRESS                      0x82
#define TS_I2C_ADDRESS                      0x82

#define STMPE811_PERIPH                          I2C3
#define STMPE811_PERIPH_SPEED                   100000

static uint16_t TsXBoundary = 0;
static uint16_t TsYBoundary = 0;

static void     stmpe811_TS_Start(uint16_t DeviceAddr);
static void     stmpe811_Reset(uint16_t DeviceAddr);
static void     stmpe811_IO_DisableAF(uint16_t DeviceAddr, uint32_t IO_Pin);
static void     stmpe811_IO_EnableAF(uint16_t DeviceAddr, uint32_t IO_Pin);
static uint8_t  stmpe811_TS_DetectTouch(uint16_t DeviceAddr);
static void     stmpe811_TS_GetXY(uint16_t DeviceAddr, uint16_t *X, uint16_t *Y);

void stmpe811_Init(uint16_t nResolutionX, uint16_t nResolutionY)
{
  TsXBoundary = nResolutionX;
  TsYBoundary = nResolutionY;

  TM_I2C_Init(STMPE811_PERIPH, TM_I2C_PinsPack_1, STMPE811_PERIPH_SPEED );
  LL_mDelay(5);
  stmpe811_Reset(TS_I2C_ADDRESS);
  LL_mDelay(5);
  stmpe811_TS_Start(TS_I2C_ADDRESS);
}


static void stmpe811_TS_Start(uint16_t DeviceAddr)
{
  uint8_t mode;

  /* Get the current register value */
  mode = TM_I2C_Read(STMPE811_PERIPH, DeviceAddr, STMPE811_REG_SYS_CTRL2);

  /* Set the Functionalities to be Enabled */
  mode &= ~(STMPE811_IO_FCT);

  /* Write the new register value */
  TM_I2C_Write(STMPE811_PERIPH,DeviceAddr, STMPE811_REG_SYS_CTRL2, mode);

  /* Select TSC pins in TSC alternate mode */
  stmpe811_IO_EnableAF(DeviceAddr, STMPE811_TOUCH_IO_ALL);

  /* Set the Functionalities to be Enabled */
  mode &= ~(STMPE811_TS_FCT | STMPE811_ADC_FCT);

  /* Set the new register value */
  TM_I2C_Write(STMPE811_PERIPH,DeviceAddr, STMPE811_REG_SYS_CTRL2, mode);

  /* Select Sample Time, bit number and ADC Reference */
  TM_I2C_Write(STMPE811_PERIPH,DeviceAddr, STMPE811_REG_ADC_CTRL1, 0x49);

  /* Wait for 2 ms */
  LL_mDelay(2);

  /* Select the ADC clock speed: 3.25 MHz */
  TM_I2C_Write(STMPE811_PERIPH,DeviceAddr, STMPE811_REG_ADC_CTRL2, 0x01);

  /* Select 2 nF filter capacitor */
  /* Configuration:
     - Touch average control    : 4 samples
     - Touch delay time         : 500 uS
     - Panel driver setting time: 500 uS
  */
  TM_I2C_Write(STMPE811_PERIPH,DeviceAddr, STMPE811_REG_TSC_CFG, 0x9A);

  /* Configure the Touch FIFO threshold: single point reading */
  TM_I2C_Write(STMPE811_PERIPH,DeviceAddr, STMPE811_REG_FIFO_TH, 0x01);

  /* Clear the FIFO memory content. */
  TM_I2C_Write(STMPE811_PERIPH,DeviceAddr, STMPE811_REG_FIFO_STA, 0x01);

  /* Put the FIFO back into operation mode  */
  TM_I2C_Write(STMPE811_PERIPH,DeviceAddr, STMPE811_REG_FIFO_STA, 0x00);

  /* Set the range and accuracy pf the pressure measurement (Z) :
     - Fractional part :7
     - Whole part      :1
  */
  TM_I2C_Write(STMPE811_PERIPH,DeviceAddr, STMPE811_REG_TSC_FRACT_XYZ, 0x01);

  /* Set the driving capability (limit) of the device for TSC pins: 50mA */
  TM_I2C_Write(STMPE811_PERIPH,DeviceAddr, STMPE811_REG_TSC_I_DRIVE, 0x01);

  /* Touch screen control configuration (enable TSC):
     - No window tracking index
     - XYZ acquisition mode
   */
  TM_I2C_Write(STMPE811_PERIPH,DeviceAddr, STMPE811_REG_TSC_CTRL, 0x01);

  /*  Clear all the status pending bits if any */
  TM_I2C_Write(STMPE811_PERIPH,DeviceAddr, STMPE811_REG_INT_STA, 0xFF);

  /* Wait for 2 ms delay */
  LL_mDelay(2);
}


static void stmpe811_Reset(uint16_t DeviceAddr)
{
  /* Power Down the stmpe811 */
  TM_I2C_Write(STMPE811_PERIPH, DeviceAddr, STMPE811_REG_SYS_CTRL1, 2);

  /* Wait for a delay to ensure registers erasing */
  LL_mDelay(10);

  /* Power On the Codec after the power off => all registers are reinitialized */
  TM_I2C_Write(STMPE811_PERIPH, DeviceAddr, STMPE811_REG_SYS_CTRL1, 0);

  /* Wait for a delay to ensure registers erasing */
  LL_mDelay(2);
}


/**
  * @brief  Disable the AF for the selected IO pin(s).
  * @param  DeviceAddr: Device address on communication Bus.
  * @param  IO_Pin: The IO pin to be configured. This parameter could be any
  *         combination of the following values:
  *   @arg  STMPE811_PIN_x: Where x can be from 0 to 7.
  * @retval None
  */
static void stmpe811_IO_DisableAF(uint16_t DeviceAddr, uint32_t IO_Pin)
{
  uint8_t tmp = 0;

  /* Get the current state of the IO_AF register */
  tmp = TM_I2C_Read(STMPE811_PERIPH, DeviceAddr, STMPE811_REG_IO_AF);

  /* Enable the selected pins alternate function */
  tmp |= (uint8_t)IO_Pin;

  /* Write back the new value in IO AF register */
  TM_I2C_Write(STMPE811_PERIPH, DeviceAddr, STMPE811_REG_IO_AF, tmp);

}

/**
  * @brief  Enable the AF for the selected IO pin(s).
  * @param  DeviceAddr: Device address on communication Bus.
  * @param  IO_Pin: The IO pin to be configured. This parameter could be any
  *         combination of the following values:
  *   @arg  STMPE811_PIN_x: Where x can be from 0 to 7.
  * @retval None
  */
static void stmpe811_IO_EnableAF(uint16_t DeviceAddr, uint32_t IO_Pin)
{
  uint8_t tmp = 0;

  /* Get the current register value */
  tmp = TM_I2C_Read(STMPE811_PERIPH ,DeviceAddr, STMPE811_REG_IO_AF);

  /* Enable the selected pins alternate function */
  tmp &= ~(uint8_t)IO_Pin;

  /* Write back the new register value */
  TM_I2C_Write(STMPE811_PERIPH, DeviceAddr, STMPE811_REG_IO_AF, tmp);
}

/**
  * @brief  Return if there is touch detected or not.
  * @param  DeviceAddr: Device address on communication Bus.
  * @retval Touch detected state.
  */
static uint8_t stmpe811_TS_DetectTouch(uint16_t DeviceAddr)
{
  uint8_t state;
  uint8_t ret = 0;

  state = ((TM_I2C_Read(STMPE811_PERIPH, DeviceAddr, STMPE811_REG_TSC_CTRL) & (uint8_t)STMPE811_TS_CTRL_STATUS) == (uint8_t)0x80);

  if(state > 0)
  {
    if(TM_I2C_Read(STMPE811_PERIPH, DeviceAddr, STMPE811_REG_FIFO_SIZE) > 0)
    {
      ret = 1;
    }
  }
  else
  {
    /* Reset FIFO */
    TM_I2C_Write(STMPE811_PERIPH,DeviceAddr, STMPE811_REG_FIFO_STA, 0x01);
    /* Enable the FIFO again */
    TM_I2C_Write(STMPE811_PERIPH,DeviceAddr, STMPE811_REG_FIFO_STA, 0x00);
  }

  return ret;
}

/**
  * @brief  Get the touch screen X and Y positions values
  * @param  DeviceAddr: Device address on communication Bus.
  * @param  X: Pointer to X position value
  * @param  Y: Pointer to Y position value
  * @retval None.
  */
static void stmpe811_TS_GetXY(uint16_t DeviceAddr, uint16_t *X, uint16_t *Y)
{
  uint8_t  dataXYZ[4];
  uint32_t uldataXYZ;

  TM_I2C_ReadMulti(STMPE811_PERIPH, DeviceAddr, STMPE811_REG_TSC_DATA_NON_INC, dataXYZ, sizeof(dataXYZ)) ;

  /* Calculate positions values */
  uldataXYZ = (dataXYZ[0] << 24)|(dataXYZ[1] << 16)|(dataXYZ[2] << 8)|(dataXYZ[3] << 0);
  *X = (uldataXYZ >> 20) & 0x00000FFF;
  *Y = (uldataXYZ >>  8) & 0x00000FFF;

  /* Reset FIFO */
  TM_I2C_Write(STMPE811_PERIPH,DeviceAddr, STMPE811_REG_FIFO_STA, 0x01);
  /* Enable the FIFO again */
  TM_I2C_Write(STMPE811_PERIPH,DeviceAddr, STMPE811_REG_FIFO_STA, 0x00);
}


void stmpe811_Scan(touch_info_t* pTouchInfo)
{
  static uint32_t _x = 0, _y = 0;
  uint16_t xDiff, yDiff , x , y, xr, yr;

  pTouchInfo->bTouchDetected = stmpe811_TS_DetectTouch(TS_I2C_ADDRESS);

  if(pTouchInfo->bTouchDetected)
  {
    stmpe811_TS_GetXY(TS_I2C_ADDRESS, &x, &y);

    y -= 360;       // Y value first correction
    yr = y / 11;    //Y value second correction

    // Return y position value
    if(yr <= 0)
    {
      yr = 0;
    }
    else if (yr > TsYBoundary)
    {
      yr = TsYBoundary - 1;
    }
    else
    {}
    y = yr;

    /* X value first correction */
    if(x <= 3000)
    {
      x = 3870 - x;
    }
    else
    {
      x = 3800 - x;
    }

    /* X value second correction */
    xr = x / 15;

    /* Return X position value */
    if(xr <= 0)
    {
      xr = 0;
    }
    else if (xr > TsXBoundary)
    {
      xr = TsXBoundary - 1;
    }
    else
    {}

    x = xr;
    xDiff = x > _x? (x - _x): (_x - x);
    yDiff = y > _y? (y - _y): (_y - y);

    if (xDiff + yDiff > 5)
    {
      _x = x;
      _y = y;
    }

    /* Update the X position */
    pTouchInfo->x = _x;

    /* Update the Y position */
    pTouchInfo->y = _y;
  }
}

