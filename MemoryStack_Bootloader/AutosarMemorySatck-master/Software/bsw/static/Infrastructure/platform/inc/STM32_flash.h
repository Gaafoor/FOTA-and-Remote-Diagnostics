/*
 * STM32_flash.h
 *
 *  Created on: Mar 6, 2024
 *      Author: pc
 */

#include "Platform_Types.h"

#ifndef SOFTWARE_BSW_STATIC_INFRASTRUCTURE_PLATFORM_INC_STM32_FLASH_H_
#define SOFTWARE_BSW_STATIC_INFRASTRUCTURE_PLATFORM_INC_STM32_FLASH_H_

#define FLASH_BASE_ADDRESS 0x40022000
//#define FLASH ((STM32_FLASH_TypeDef *)FLASH_BASE_ADDRESS)



/*typedef struct {
    volatile uint32 FLASH_ACR;      // Flash Access Control Register
    volatile uint32 FLASH_KEYR;     // Flash Key Register
    volatile uint32 FLASH_OPTKEYR;  // Flash Option Key Register
    volatile uint32 FLASH_SR;       // Flash Status Register
    volatile uint32 FLASH_CR;       // Flash Control Register
    volatile uint32 FLASH_AR;       // Flash Address Register
	volatile uint32 RESERVED;
    volatile uint32 FLASH_OBR;      // Flash Option Byte Register
    volatile uint32 FLASH_WRPR;     // Flash Write Protection Register
} STM32_FLASH_TypeDef;*/



#endif /* SOFTWARE_BSW_STATIC_INFRASTRUCTURE_PLATFORM_INC_STM32_FLASH_H_ */
