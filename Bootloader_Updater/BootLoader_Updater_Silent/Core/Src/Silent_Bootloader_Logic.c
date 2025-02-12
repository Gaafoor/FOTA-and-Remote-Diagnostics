/******************************************************************************
 *
 * File Name: Silent_Bootloader_Logic.c
 *
 * Description: Source file for helpful functions and global variables used by application
 *
 * Author: Radwa
 *
 *******************************************************************************/


/*******************************************************************************
 *                              Includes                       					*
 *******************************************************************************/
#include <Silent_Bootloader_Logic.h>
#include "stm32f4xx_hal.h"
#include "rtc.h"
#include "string.h"

/*******************************************************************************
 *                      Static global Variables		                            *
 *******************************************************************************/
/*
 * bootloader image
 */
static uint8_t bootloader_as_array[] = {
0x00, 0x00, 0x03, 0x20, 0xa5, 0x1c, 0x04};


/*******************************************************************************
 *                      Static Functions Definitions                            *
 *******************************************************************************/
/*
 * get flash sector number based on passed Address
 */
static uint32_t GetSector(uint32_t Address)
{
    uint32_t sector = 0;

    /* BANK 1 */
    if ((Address >= 0x08000000) && (Address < 0x08003FFF))
    {
        sector = FLASH_SECTOR_0;
    }
    else if ((Address >= 0x08004000) && (Address < 0x08007FFF))
    {
        sector = FLASH_SECTOR_1;
    }
    else if ((Address >= 0x08008000) && (Address < 0x0800BFFF))
    {
        sector = FLASH_SECTOR_2;
    }
    else if ((Address >= 0x0800C000) && (Address < 0x0800FFFF))
    {
        sector = FLASH_SECTOR_3;
    }
    else if ((Address >= 0x08010000) && (Address < 0x0801FFFF))
    {
        sector = FLASH_SECTOR_4;
    }
    else if ((Address >= 0x08020000) && (Address < 0x0803FFFF))
    {
        sector = FLASH_SECTOR_5;
    }
    else if ((Address >= 0x08040000) && (Address < 0x0805FFFF))
    {
        sector = FLASH_SECTOR_6;
    }
    else if ((Address >= 0x08060000) && (Address < 0x0807FFFF))
    {
        sector = FLASH_SECTOR_7;
    }
    return sector;
}

/*
 * erase flash
 */
static uint8_t Flash_Memory_Erase(uint32_t StartSectorAddress , uint32_t dataSizeInBytes){
	static FLASH_EraseInitTypeDef EraseInitStruct;   /* Structure to erase the flash area */
	uint32_t SECTORError;

	/* Getting the number of sector to erase from the first sector */
	uint32_t StartSector = GetSector(StartSectorAddress);                /*getting the start sector number*/
	uint32_t EndSectorAddress = StartSectorAddress + dataSizeInBytes;    /*getting the end sector address*/
	uint32_t EndSector = GetSector(EndSectorAddress);                    /*getting the end sector number*/

	/* Filling the erasing structure */
	EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
	EraseInitStruct.Sector        = StartSector;
	EraseInitStruct.NbSectors     = (EndSector - StartSector) + 1;

	/* Unlocking the Flash control register */
	HAL_FLASH_Unlock();

	/* check if the erasing process is done correctly */
	if (HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError) != HAL_OK)
	{
		/*Error occurred while page erase*/
		return ERROR;
	}

	/* Locking the Flash control register */
	HAL_FLASH_Lock();

	return SUCCESS;
}


/*
 * write on flash
 */
static uint8_t Flash_Memory_Write(uint32_t StartSectorAddress, uint8_t *data, uint32_t dataSizeInBytes) {
    uint32_t numofWords = dataSizeInBytes / 4;     // Number of complete 32-bit words
    uint32_t numofWordsWritten = 0;
    uint32_t remainingBytes = dataSizeInBytes % 4; // Remaining bytes that don't form a complete word
    uint32_t tempWord = 0;

    // Unlocking the Flash control register
    HAL_FLASH_Unlock();

    // Looping on the data word by word to write it in the flash
    while (numofWordsWritten < numofWords) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, StartSectorAddress, *(uint32_t *)&data[numofWordsWritten * 4]) == HAL_OK) {
            StartSectorAddress += 4;
            numofWordsWritten++;
        } else {
            // Error occurred while writing data in Flash memory
            HAL_FLASH_Lock();
            return ERROR;
        }
    }

    // Write any remaining bytes
    if (remainingBytes > 0) {
        tempWord = 0xFFFFFFFF; // Initialize to all 1s (erased state)
        memcpy(&tempWord, &data[numofWords * 4], remainingBytes); // Copy remaining bytes into tempWord
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, StartSectorAddress, tempWord) != HAL_OK) {
            // Error occurred while writing remaining bytes in Flash memory
            HAL_FLASH_Lock();
            return ERROR;
        }
    }

    // Locking the Flash control register
    HAL_FLASH_Lock();

    return SUCCESS;
}

static void Write_RTC_backup_reg(uint32_t reg ,uint32_t data){
    HAL_PWR_EnableBkUpAccess();
    HAL_RTCEx_BKUPWrite(&hrtc, reg, data);
    HAL_PWR_DisableBkUpAccess();

}

/*
 * leaving bootloader updater
 */
static 	void Leaving_Handler(){
	/*
	 * updating Control Flags to make bootManager enter bootloader
	 */
	Write_RTC_backup_reg(APPLICATION_ENTER_FLAG_ADDRESS,N_ENTER);
	Write_RTC_backup_reg(BOOTLOADER_UPDATER_ENTER_FLAG_ADDRESS, N_ENTER);
	//sw reset
	NVIC_SystemReset();
}

/*******************************************************************************
 *                      Global Functions Definitions                            *
 *******************************************************************************/

/***************************************************************************************************
 * [Function Name]: Update_Logic
 *
 * [Description]:  Update logic and behaviour
 *
 * [Args]:         void
 *
 * [Returns]:      void
 *
 ***************************************************************************************************/
void Update_Logic(){
	uint32_t size = sizeof(bootloader_as_array)/sizeof(bootloader_as_array[0]);
	//erase the old bootloader
	Flash_Memory_Erase(BOOTLOADER_BINARY_START_ADDRESS,size);
	//write new bootloader
	Flash_Memory_Write(BOOTLOADER_BINARY_START_ADDRESS,bootloader_as_array, size);

	Leaving_Handler();
}

