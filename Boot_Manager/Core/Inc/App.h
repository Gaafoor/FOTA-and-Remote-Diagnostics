/*
 * App.h
 *
 *  Created on: Jun 24, 2024
 *      Author: radwa
 */

#ifndef INC_APP_H_
#define INC_APP_H_

/*******************************************************************************
 *                                Includes                                  *
 *******************************************************************************/
#include "stm32f4xx_hal.h"

/*******************************************************************************
 *                                Definitions                                  *
 *******************************************************************************/
#define FAILED 1
#define SUCCEEDED 0

#define N_ENTER 0
#define ENTER 1

#define APPLICATION_ENTER_FLAG_ADDRESS 0x00

// BootLoader Address
#define BOOTLOADER_BINARY_START_ADDRESS 0x000000

// Image Address
#define APP_BINARY_START_ADDRESS 0x08008000U
#define APP_NO_OF_BYTES_START_ADDRESS APP_BINARY_START_ADDRESS
#define APP_CRC_START_ADDRESS 0x000000000


/*******************************************************************************
 *                              Functions Prototypes                           *
 *******************************************************************************/
void App_Logic();


#endif /* INC_APP_H_ */
