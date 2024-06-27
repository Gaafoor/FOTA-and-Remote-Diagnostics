/*
 * App.c
 *
 *  Created on: Jun 24, 2024
 *      Author: radwa
 */


/*******************************************************************************
 *                              Includes                       					*
 *******************************************************************************/
#include "App.h"
#include "rtc.h"
#include <stdlib.h> // Included to declare atoi function

/*******************************************************************************
 *                      Static Functions Definitions                           *
 *******************************************************************************/

/* Jump to specific Address */
static void jump_to_Image_Address(uint32_t start_addr){

	/* Set the main stack pointer to the application start address */
	__set_MSP(*(uint32_t *)start_addr);
	//__set_PSP(*(uint32_t *)start_addr);

	/* Get the main application start address */
	uint32_t jump_address = *(uint32_t *)(start_addr + 4);

	// Create function pointer for the main application
	void (*app_ptr)(void);
	app_ptr = (void *)(jump_address);

	// Now jump to the main application
	app_ptr();
}

/* Calculate CRC of the given data */
static uint32_t CalculateCRC(const uint8_t* image_start_address, uint32_t image_size) {
	CRC_HandleTypeDef hcrc;
	hcrc.Instance = CRC;
	HAL_CRC_Init(&hcrc);
	uint32_t crc = HAL_CRC_Calculate(&hcrc, (uint32_t *)image_start_address, image_size / 4);
	HAL_CRC_DeInit(&hcrc);
	return crc;
}

/* Compare two CRC values */
static uint8_t CRCCompare(uint32_t calculatedCRC, uint32_t storedCRC) {
	return (calculatedCRC == storedCRC) ? SUCCEEDED : FAILED;
}

static uint8_t Read_RTC_backup_reg(uint8_t reg) {
    return (uint8_t)HAL_RTCEx_BKUPRead(&hrtc, reg);
}

static void Write_RTC_backup_reg(uint32_t reg, uint32_t data) {
    HAL_PWR_EnableBkUpAccess();
    HAL_RTCEx_BKUPWrite(&hrtc, reg, data);
    HAL_PWR_DisableBkUpAccess();
}

/*******************************************************************************
 *                      Global Functions Definitions                           *
 *******************************************************************************/

/***************************************************************************************************
 * [Function Name]: App_Logic
 *
 * [Description]:  App logic and behaviour
 *
 * [Args]:         void
 *
 * [Returns]:      void
 *
 ***************************************************************************************************/
void App_Logic(){

	// Toggling LED
	for (uint8_t var = 0; var < 3; ++var) {
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
		HAL_Delay(200);
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
		HAL_Delay(200);
	}

	/*
	 * Calculate Application Integrity
	 */
	uint32_t app_no_bytes = atoi((const char *)(APP_NO_OF_BYTES_START_ADDRESS));
	uint32_t stored_crc = *((uint32_t *)(APP_CRC_START_ADDRESS));
	uint32_t calculated_crc = CalculateCRC((const uint8_t *)APP_BINARY_START_ADDRESS, app_no_bytes);
	uint8_t Application_Integrity_result = CRCCompare(calculated_crc, stored_crc);

	/*
	 * Reading Control Flags
	 */
	uint8_t Application_Enter_flag = Read_RTC_backup_reg(APPLICATION_ENTER_FLAG_ADDRESS);

	/*
	 * Branching Conditions
	 */
	if ((Application_Integrity_result == SUCCEEDED) && (Application_Enter_flag == ENTER)) {
		// Jump to Application
		jump_to_Image_Address(APP_BINARY_START_ADDRESS);
	} else {
		// Resetting flags as for example(application flag = ENTER but its integrity is NOK)
		Write_RTC_backup_reg(APPLICATION_ENTER_FLAG_ADDRESS, N_ENTER);
		// Jump to Bootloader
		jump_to_Image_Address(BOOTLOADER_BINARY_START_ADDRESS);
	}
}
