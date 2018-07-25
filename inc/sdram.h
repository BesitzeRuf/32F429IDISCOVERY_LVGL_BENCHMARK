/**
 *******************************************************************************
 * @file    sdram.h
 * @author  V. Yurov
 * @version
 * @date    18. 6. 2018
 * @brief
 *******************************************************************************
 * @copyright
 * Copyright (C) 2016 Medical Technologies CZ a.s.. All rights reserved.
 * Contact: http://www.medictech.com
 *
 * Without prior written permission from Medical Technologies CZ a.s.,
 * the file must not be modified, distributed and/or otherwise used.
 *******************************************************************************
 */

#ifndef SDRAM_INC_SDRAM_H_
#define SDRAM_INC_SDRAM_H_


#ifdef __cplusplus
 extern "C" {
#endif

#define SDRAM_BASE ((uint32_t)0xD0000000)
#define SDRAM_SIZE ((uint32_t)0x800000)


void __attribute((optimize("-O0"))) SDRAM_Init(void);


#ifdef __cplusplus
 }
#endif

#endif /* SDRAM_INC_SDRAM_H_ */
