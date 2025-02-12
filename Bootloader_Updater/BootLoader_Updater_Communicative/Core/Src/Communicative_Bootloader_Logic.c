/******************************************************************************
 *
 * File Name: Communicative_Bootloader_Logic.c
 *
 * Description: Source file for helpful functions and global variables used by application
 *
 * Author: Radwa
 *
 *******************************************************************************/


/*******************************************************************************
 *                              Includes                       					*
 *******************************************************************************/
#include <Communicative_Bootloader_Logic.h>
#include "stm32f4xx_hal.h"
#include "rtc.h"
#include "can.h"
#include "stdlib.h"
#include "string.h"

/*******************************************************************************
 *                      Static global Variables		                            *
 *******************************************************************************/

//1: Bootloader valid
//0: Bootloader not valid
static uint8_t Bootloader_Validation = 1 ;

/*
 * Buffer used for Communication with BCM
 */
static uint8_t Bootloader_Updater_Rx_Buffer[BOOTLOADER_UPDATER_RX_BUFFER_LENGTH];


/*******************************************************************************
 *                      Static user define types		                        *
 *******************************************************************************/
enum Bootloader_Supported_Commands {
    BOOTLOADER_UPDATER_GET_VERION_COMMAND,
    BOOTLOADER_UPDATER_MEM_WRITE_BOOTLOADER_COMMAND,
    BOOTLOADER_UPDATER_MEM_ERASE_BOOTLOADER_COMMAND,
    BOOTLOADER_UPDATER_LEAVING_TO_BOOT_MANAGER_COMMAND = 5
};


/*******************************************************************************
 *                      Static Functions Definitions                            *
 *******************************************************************************/
/*
 * get flash sector number based on passed address
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
 * write on flash
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
 * erase flash region
 */
static uint8_t Flash_Memory_Write(uint32_t StartSectorAddress ,uint32_t *data, uint32_t dataSizeInBytes){
	uint32_t numofWords=dataSizeInBytes/4;     /*getting number of words to write*/
	uint32_t numofWordsWritten=0;

	/* Unlocking the Flash control register */
	HAL_FLASH_Unlock();

	/* looping on the data word by word to write it in the flash */
	while(numofWordsWritten < numofWords){

		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, StartSectorAddress, data[numofWordsWritten]) == HAL_OK)
		{
			StartSectorAddress += 4;
			numofWordsWritten++;
		}
		else
		{
			/* Error occurred while writing data in Flash memory*/
			return ERROR;
		}

	}

	/* Locking the Flash control register */
	HAL_FLASH_Lock();

	return SUCCESS;

}


static void Write_RTC_backup_reg(uint32_t reg ,uint32_t data){
    HAL_PWR_EnableBkUpAccess();
    HAL_RTCEx_BKUPWrite(&hrtc, reg, data);
    HAL_PWR_DisableBkUpAccess();

}


/*
 * handle Bootloader updater version
 */
static void Get_Version_Command_Handler() {
    uint8_t bootloader_updater_version[3] = {
        BOOTLOADER_UPDATER_MAJOR_VERSION,
        BOOTLOADER_UPDATER_MINOR_VERSION,
        BOOTLOADER_UPDATER_PATCH_VERSION
    };

    // Define the CAN Tx header
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;

    // Configure the CAN Tx header
    TxHeader.DLC = 3; // Data length code: 3 bytes
    TxHeader.StdId = 0x321; // Standard Identifier (you can use a specific ID for your application)
    TxHeader.IDE = CAN_ID_STD; // Standard ID
    TxHeader.RTR = CAN_RTR_DATA; // Data frame

    // Transmit the CAN message
    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, bootloader_updater_version, &TxMailbox) != HAL_OK) {
        // Transmission request Error
        Error_Handler(); // Handle the error accordingly
    }
}

/*
 * erase bootloader flash region
 */
static void Mem_Erase_BOOTLOADER_Command_Handler() {
    uint32_t Bootloader_size_length = atoi((char*)&Bootloader_Updater_Rx_Buffer[2]);

    uint8_t result = Flash_Memory_Erase(BOOTLOADER_BINARY_START_ADDRESS, Bootloader_size_length);

    // Define the CAN Tx header
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;

    // Configure the CAN Tx header
    TxHeader.DLC = 1; // Data length code: 1 byte (result)
    TxHeader.StdId = 0x322; // Standard Identifier for response
    TxHeader.IDE = CAN_ID_STD; // Standard ID
    TxHeader.RTR = CAN_RTR_DATA; // Data frame

    // Transmit the result via CAN
    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, &result, &TxMailbox) != HAL_OK) {
        // Transmission request Error
        Error_Handler(); // Handle the error accordingly
    }

    // Update the Bootloader_Validation flag based on the result
    if (result == SUCCESS) {
        Bootloader_Validation = 0; // Desired state
    } else {
        Bootloader_Validation = 0; // Not desired but for safety
    }
}

/*
 * receive bootloader and write it on its flash region
 */
static void Mem_Write_BOOTLOADER_Command_Handler() {
    uint32_t Bootloader_size_length = atoi((char*)&Bootloader_Updater_Rx_Buffer[2]);

    // Declare CAN Rx header
    CAN_RxHeaderTypeDef RxHeader;

    // Receive the bootloader binary data via CAN
    for (uint32_t i = 0; i < Bootloader_size_length; i += 8) {
        HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, &Bootloader_Updater_Rx_Buffer[10 + i]);
    }

    // Write the bootloader binary to flash memory
    uint8_t result = Flash_Memory_Write(BOOTLOADER_BINARY_START_ADDRESS, (uint32_t*)&Bootloader_Updater_Rx_Buffer[10], Bootloader_size_length);

    // Define the CAN Tx header
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;

    // Configure the CAN Tx header
    TxHeader.DLC = 1; // Data length code: 1 byte (result)
    TxHeader.StdId = 0x323; // Standard Identifier for response
    TxHeader.IDE = CAN_ID_STD; // Standard ID
    TxHeader.RTR = CAN_RTR_DATA; // Data frame

    // Transmit the result via CAN
    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, &result, &TxMailbox) != HAL_OK) {
        // Transmission request Error
        Error_Handler(); // Handle the error accordingly
    }

    // Update the Bootloader_Validation flag based on the result
    if (result == SUCCESS) {
        Bootloader_Validation = 1;
    } else {
        Bootloader_Validation = 0;
    }
}

/*
 * handle leaving bootloader updater
 */
static void Leaving_To_Boot_Manager_Command_Handler() {
    // Update control flags
    if (Bootloader_Validation == 1) {
        Write_RTC_backup_reg(APPLICATION_ENTER_FLAG_ADDRESS, N_ENTER);
        Write_RTC_backup_reg(BOOTLOADER_UPDATER_ENTER_FLAG_ADDRESS, N_ENTER);
    } else {
        Write_RTC_backup_reg(APPLICATION_ENTER_FLAG_ADDRESS, N_ENTER);
        Write_RTC_backup_reg(BOOTLOADER_UPDATER_ENTER_FLAG_ADDRESS, ENTER);
    }

    // Perform a system reset to apply the changes
    NVIC_SystemReset();
}

/*
 * Receiving Commands from BCM and handle it
 */
static void Bootloader_Updater_Receive_Command(void) {
    CAN_RxHeaderTypeDef RxHeader;

    // Clear receiving buffer
    memset(Bootloader_Updater_Rx_Buffer, 0, BOOTLOADER_UPDATER_RX_BUFFER_LENGTH);

    // Receive the length of the command via CAN
    HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, Bootloader_Updater_Rx_Buffer);

    // Receive the actual command based on the length
    HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, &Bootloader_Updater_Rx_Buffer[1]);

    switch (Bootloader_Updater_Rx_Buffer[1]) {
        case BOOTLOADER_UPDATER_GET_VERION_COMMAND:
            Get_Version_Command_Handler();
            break;
        case BOOTLOADER_UPDATER_MEM_WRITE_BOOTLOADER_COMMAND:
            Mem_Write_BOOTLOADER_Command_Handler();
            break;
        case BOOTLOADER_UPDATER_MEM_ERASE_BOOTLOADER_COMMAND:
            Mem_Erase_BOOTLOADER_Command_Handler();
            break;
        case BOOTLOADER_UPDATER_LEAVING_TO_BOOT_MANAGER_COMMAND:
            Leaving_To_Boot_Manager_Command_Handler();
            break;
        default:
            // Do nothing for unsupported commands
            break;
    }
}

/*******************************************************************************
 *                      Global Functions Definitions                            *
 *******************************************************************************/

/***************************************************************************************************
 * [Function Name]: App_Logic
 *
 * [Description]:  App logic and behaviour
 *
 * [Args]:         void
 *
 * [Returns]:      void
 ***************************************************************************************************/
void App_Logic() {
    Bootloader_Updater_Receive_Command();
}
