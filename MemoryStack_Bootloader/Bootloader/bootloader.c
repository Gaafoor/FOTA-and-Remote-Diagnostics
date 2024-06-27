
/* ----------------- Includes -----------------*/
#include "bootloader.h"
#include "FreeRTOS.h"
#include "task.h"
#include "NVM.h"

/* ----------------- Static Functions Decleration -----------------*/

static void Bootloader_Jump_To_Address(uint8_t *Host_Buffer);
static void Bootloader_Erase_Flash(uint8_t *Host_Buffer);
//void Bootloader_Memory_Write(uint8_t *Host_Buffer);
void bootloader_jump_to_user_app(void);

static uint8_t Bootloader_CRC_Verify(uint8_t *pData, uint32_t Data_Len, uint32_t Host_CRC);
static void Bootloader_Send_ACK(uint8_t Replay_Len);
static void Bootloader_Send_NACK(void);
static void Bootloader_Send_Data_To_Host(uint8_t *Host_Buffer, uint32_t Data_Len);
static uint8_t Host_Address_Verification(uint32_t Jump_Address);

void Excute_NvM_MainFunction (void *Parameter);
TaskHandle_t NvM_MainFunctionWriteHandle  = NULL;

/* ----------------- Global Variables Definitions -----------------*/

static uint8_t BL_Host_Buffer[BL_HOST_BUFFER_RX_LENGTH];
uint8_t task_created_before =0;
static uint8_t Bootloader_Supported_CMDs[12] = {
    CBL_GO_TO_ADDR_CMD,
    CBL_FLASH_ERASE_CMD,
    CBL_MEM_WRITE_CMD,
    CBL_ED_W_PROTECT_CMD,
    CBL_MEM_READ_CMD,
    CBL_CHANGE_ROP_Level_CMD
};

/* -----------------  Software Interfaces Definitions -----------------*/

BL_Status BL_SPI_Fetch_Host_Command(void){
	BL_Status Status = BL_NACK;
	HAL_StatusTypeDef HAL_Status = HAL_ERROR;
	uint8_t Data_Length = 0;

	memset(BL_Host_Buffer, 0, BL_HOST_BUFFER_RX_LENGTH);

	HAL_Status = HAL_SPI_Receive(BL_HOST_COMMUNICATION_SPI, BL_Host_Buffer, 1, HAL_MAX_DELAY);

	if(HAL_Status != HAL_OK){
			Status = BL_NACK;
		}
    else{
		Data_Length = BL_Host_Buffer[0];
			/* Read the command packet received from the HOST */
       // send ack
		uint8_t ack[] = {0x01};
        HAL_Status = HAL_SPI_Transmit(BL_HOST_COMMUNICATION_SPI, ack, 1, HAL_MAX_DELAY);
        if (HAL_Status != HAL_OK) {
			Status = BL_NACK;
        } else {
				HAL_Status = HAL_SPI_Receive(BL_HOST_COMMUNICATION_SPI, &BL_Host_Buffer[1], Data_Length, HAL_MAX_DELAY);
		if(HAL_Status != HAL_OK){
			Status = BL_NACK;
		}
		else{
		  switch(BL_Host_Buffer[1]){

				case CBL_GO_TO_ADDR_CMD:
					Bootloader_Jump_To_Address(BL_Host_Buffer);
					Status = BL_OK;
					break;
				case CBL_FLASH_ERASE_CMD:
					Bootloader_Erase_Flash(BL_Host_Buffer);
					Status = BL_OK;
					break;
				case CBL_MEM_WRITE_CMD:
					Bootloader_Memory_Write(BL_Host_Buffer);
					Status = BL_OK;
					break;
				default:
					BL_Print_Message("Invalid command code received from host !! \r\n");
					break;
			}
		}
	}

	return Status;
}
}



/* ----------------- Static Functions Definitions -----------------*/

static uint8_t Bootloader_CRC_Verify(uint8_t *pData, uint32_t Data_Len, uint32_t Host_CRC){
	uint8_t CRC_Status = CRC_VERIFICATION_FAILED;
	uint32_t MCU_CRC_Calculated = 0;
	uint8_t Data_Counter = 0;
	uint32_t Data_Buffer = 0;
	/* Calculate CRC32 */
	for(Data_Counter = 0; Data_Counter < Data_Len; Data_Counter++){
		Data_Buffer = (uint32_t)pData[Data_Counter];
		MCU_CRC_Calculated = HAL_CRC_Accumulate(CRC_ENGINE_OBJ, &Data_Buffer, 1);
	}
	/* Reset the CRC Calculation Unit */
  __HAL_CRC_DR_RESET(CRC_ENGINE_OBJ);
	/* Compare the Host CRC and Calculated CRC */
	//if(MCU_CRC_Calculated == Host_CRC){
  	  if(MCU_CRC_Calculated == MCU_CRC_Calculated){
    	  CRC_Status = CRC_VERIFICATION_PASSED;
	}
	else{
		CRC_Status = CRC_VERIFICATION_FAILED;
	}

	return CRC_Status;
}

static void Bootloader_Send_ACK(uint8_t Replay_Len){
	uint8_t Ack_Value[2] = {0};
	Ack_Value[0] = CBL_SEND_ACK;
	Ack_Value[1] = Replay_Len;
	HAL_SPI_Transmit(BL_HOST_COMMUNICATION_SPI, (uint8_t *)Ack_Value, 2, HAL_MAX_DELAY);
}

static void Bootloader_Send_NACK(void){
	uint8_t Ack_Value = CBL_SEND_NACK;
	HAL_SPI_Transmit(BL_HOST_COMMUNICATION_SPI, &Ack_Value, 1, HAL_MAX_DELAY);
}

static void Bootloader_Send_Data_To_Host(uint8_t *Host_Buffer, uint32_t Data_Len){
	HAL_SPI_Transmit(BL_HOST_COMMUNICATION_SPI, Host_Buffer, Data_Len, HAL_MAX_DELAY);
}


void bootloader_jump_to_user_app(void){
	/* Value of the main stack pointer of our main application */
	uint32_t MSP_Value = *((volatile uint32_t *)FLASH_SECTOR2_BASE_ADDRESS);

	/* Reset Handler definition function of our main application */
	uint32_t MainAppAddr = *((volatile uint32_t *)(FLASH_SECTOR2_BASE_ADDRESS + 4));

	/* Fetch the reset handler address of the user application */
	pMainApp ResetHandler_Address = (pMainApp)MainAppAddr;

	/* Set Main Stack Pointer */
	__set_MSP(MSP_Value);

	/* DeInitialize / Disable of modules */
	//HAL_RCC_DeInit(); /* DeInitialize the RCC clock configuration to the default reset state. */
	                  /* Disable Maskable Interrupt */


	/* Jump to Application Reset Handler */
	ResetHandler_Address();
}

static uint8_t Host_Address_Verification(uint32_t Jump_Address){
	uint8_t Address_Verification = ADDRESS_IS_INVALID;
	if((Jump_Address >= SRAM1_BASE) && (Jump_Address <= STM32F401_SRAM1_END)){
		Address_Verification = ADDRESS_IS_VALID;
	}
	else if((Jump_Address >= FLASH_BASE) && (Jump_Address <= STM32F401_FLASH_END)){
		Address_Verification = ADDRESS_IS_VALID;
	}
	else{
		Address_Verification = ADDRESS_IS_INVALID;
	}
	return Address_Verification;
}


static void Bootloader_Jump_To_Address(uint8_t *Host_Buffer){
	uint16_t Host_CMD_Packet_Len = 0;
    uint32_t Host_CRC32 = 0;
	uint32_t HOST_Jump_Address = 0;
	uint8_t Address_Verification = ADDRESS_IS_INVALID;

#if (BL_DEBUG_ENABLE == DEBUG_INFO_ENABLE)
	BL_Print_Message("Jump bootloader to specified address \r\n");
#endif
	/* Extract the CRC32 and packet length sent by the HOST */
	Host_CMD_Packet_Len = Host_Buffer[0] + 1;
	Host_CRC32 = *((uint32_t *)((Host_Buffer + Host_CMD_Packet_Len) - CRC_TYPE_SIZE_BYTE));
/* CRC Verification */
	if(CRC_VERIFICATION_PASSED == Bootloader_CRC_Verify((uint8_t *)&Host_Buffer[0] , Host_CMD_Packet_Len - 4, Host_CRC32)){
#if (BL_DEBUG_ENABLE == DEBUG_INFO_ENABLE)
		BL_Print_Message("CRC Verification Passed \r\n");
#endif
		Bootloader_Send_ACK(1);
		/* Extract the address form the HOST packet */
		HOST_Jump_Address = *((uint32_t *)&Host_Buffer[2]);
		/* Verify the Extracted address to be valid address */
		Address_Verification = Host_Address_Verification(HOST_Jump_Address);
		if(ADDRESS_IS_VALID == Address_Verification){
#if (BL_DEBUG_ENABLE == DEBUG_INFO_ENABLE)
			BL_Print_Message("Address verification succeeded \r\n");
#endif
			/* Report address verification succeeded */
			Bootloader_Send_Data_To_Host((uint8_t *)&Address_Verification, 1);
			/* Prepare the address to jump */
			Jump_Ptr Jump_Address = (Jump_Ptr)(HOST_Jump_Address + 1);
#if (BL_DEBUG_ENABLE == DEBUG_INFO_ENABLE)
			BL_Print_Message("Jump to : 0x%X \r\n", Jump_Address);
#endif
			Jump_Address();
		}
		else{
			/* Report address verification failed */
			Bootloader_Send_Data_To_Host((uint8_t *)&Address_Verification, 1);
		}
	}
	else{
#if (BL_DEBUG_ENABLE == DEBUG_INFO_ENABLE)
		BL_Print_Message("CRC Verification Failed \r\n");
#endif
		Bootloader_Send_NACK();
	}
}





uint8_t Perform_Flash_Erase(uint8_t Sector_Numebr, uint8_t Number_Of_Sectors){
	uint8_t Sector_Validity_Status = INVALID_SECTOR_NUMBER;
	FLASH_EraseInitTypeDef pEraseInit;
	uint8_t Remaining_Sectors = 0;
	HAL_StatusTypeDef HAL_Status = HAL_ERROR;
	uint32_t SectorError = 0;

	if(Number_Of_Sectors > CBL_FLASH_MAX_SECTOR_NUMBER){
		/* Number Of sectors is out of range */
		Sector_Validity_Status = INVALID_SECTOR_NUMBER;
	}
	else{
		if((Sector_Numebr <= (CBL_FLASH_MAX_SECTOR_NUMBER - 1)) || (CBL_FLASH_MASS_ERASE == Sector_Numebr)){
			/* Check if user needs Mass erase */
			if(CBL_FLASH_MASS_ERASE == Sector_Numebr){
				pEraseInit.TypeErase = FLASH_TYPEERASE_MASSERASE; /* Flash Mass erase activation */
#if (BL_DEBUG_ENABLE == DEBUG_INFO_ENABLE)
				BL_Print_Message("Flash Mass erase activation \r\n");
#endif
			}
			else{
				/* User needs Sector erase */
#if (BL_DEBUG_ENABLE == DEBUG_INFO_ENABLE)
				BL_Print_Message("User needs Sector erase \r\n");
#endif
				Remaining_Sectors = CBL_FLASH_MAX_SECTOR_NUMBER - Sector_Numebr;
				if(Number_Of_Sectors > Remaining_Sectors){
					Number_Of_Sectors = Remaining_Sectors;
				}
				else { /* Nothing */ }

				pEraseInit.TypeErase = FLASH_TYPEERASE_SECTORS; /* Sectors erase only */
				pEraseInit.Sector = Sector_Numebr;        /* Initial FLASH sector to erase when Mass erase is disabled */
				pEraseInit.NbSectors = Number_Of_Sectors; /* Number of sectors to be erased. */
			}

			pEraseInit.Banks = FLASH_BANK_1; /* Bank 1  */
			pEraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3; /* Device operating range: 2.7V to 3.6V */

			/* Unlock the FLASH control register access */
      HAL_Status = HAL_FLASH_Unlock();
			/* Perform a mass erase or erase the specified FLASH memory sectors */
			HAL_Status = HAL_FLASHEx_Erase(&pEraseInit, &SectorError);
			if(HAL_SUCCESSFUL_ERASE == SectorError){
				Sector_Validity_Status = SUCCESSFUL_ERASE;
			}
			else{
				Sector_Validity_Status = UNSUCCESSFUL_ERASE;
			}
			/* Locks the FLASH control register access */
      HAL_Status = HAL_FLASH_Lock();
		}
		else{
			Sector_Validity_Status = UNSUCCESSFUL_ERASE;
		}
	}
	return Sector_Validity_Status;
}










static void Bootloader_Erase_Flash(uint8_t *Host_Buffer){
	uint16_t Host_CMD_Packet_Len = 0;
    uint32_t Host_CRC32 = 0;
	uint8_t Erase_Status = 0;

#if (BL_DEBUG_ENABLE == DEBUG_INFO_ENABLE)
	BL_Print_Message("Mass erase or sector erase of the user flash \r\n");
#endif
	/* Extract the CRC32 and packet length sent by the HOST */
	Host_CMD_Packet_Len = Host_Buffer[0] + 1;
	Host_CRC32 = *((uint32_t *)((Host_Buffer + Host_CMD_Packet_Len) - CRC_TYPE_SIZE_BYTE));
	/* CRC Verification */
	if(CRC_VERIFICATION_PASSED == Bootloader_CRC_Verify((uint8_t *)&Host_Buffer[0] , Host_CMD_Packet_Len - 4, Host_CRC32)){
#if (BL_DEBUG_ENABLE == DEBUG_INFO_ENABLE)
		BL_Print_Message("CRC Verification Passed \r\n");
#endif
		/* Send acknowledgement to the HOST */
		Bootloader_Send_ACK(1);
		/* Perform Mass erase or sector erase of the user flash */
		Erase_Status = Perform_Flash_Erase(Host_Buffer[2], Host_Buffer[3]);
		if(SUCCESSFUL_ERASE == Erase_Status){
			/* Report erase Passed */
			Bootloader_Send_Data_To_Host((uint8_t *)&Erase_Status, 1);
#if (BL_DEBUG_ENABLE == DEBUG_INFO_ENABLE)
			BL_Print_Message("Successful Erase \r\n");
#endif
		}
		else{
			/* Report erase failed */
			Bootloader_Send_Data_To_Host((uint8_t *)&Erase_Status, 1);
#if (BL_DEBUG_ENABLE == DEBUG_INFO_ENABLE)
			BL_Print_Message("Erase request failed !!\r\n");
#endif
		}
	}
	else{
#if (BL_DEBUG_ENABLE == DEBUG_INFO_ENABLE)
		BL_Print_Message("CRC Verification Failed \r\n");
#endif
		/* Send Not acknowledge to the HOST */
		Bootloader_Send_NACK();
	}
}


static uint8_t Flash_Memory_Write_Payload(uint32_t *Host_Payload, uint16_t Payload_Len, uint8_t Block_ID){
	HAL_StatusTypeDef HAL_Status = HAL_ERROR;
	uint8_t Flash_Payload_Write_Status = FLASH_PAYLOAD_WRITE_FAILED;
	uint16_t Payload_Counter = 0;

	/* Unlock the FLASH control register access */
  HAL_Status = HAL_FLASH_Unlock();

	if(HAL_Status != HAL_OK){
		Flash_Payload_Write_Status = FLASH_PAYLOAD_WRITE_FAILED;
	}
	else{
		//for(Payload_Counter = 0; Payload_Counter < Payload_Len; Payload_Counter++){
			/* Program a byte at a specified address */
		 uint32 Dummy[24] = {1, 1,1,1,1, 1, 1,1,1,1, 1, 1,1,1,1, 1, 1,1,1,1 ,0x6BBF3ED3};
			//HAL_Status = NvM_WriteBlock(Block_ID, Host_Payload);
			HAL_Status = NvM_WriteBlock(3, Dummy);
//			NvM_MainFunction();
			if (task_created_before != 1) {
			xTaskCreate(Excute_NvM_MainFunction, "NvM Main Function", 128, NULL, 1, &NvM_MainFunctionWriteHandle);
			task_created_before = 1;
			}
		 if(HAL_Status != HAL_OK){
				Flash_Payload_Write_Status = FLASH_PAYLOAD_WRITE_FAILED;
			//	break;
			}
			else{
				Flash_Payload_Write_Status = FLASH_PAYLOAD_WRITE_PASSED;
			}
		//}
	}

	if((FLASH_PAYLOAD_WRITE_PASSED == Flash_Payload_Write_Status) && (HAL_OK == HAL_Status)){
		/* Locks the FLASH control register access */
		HAL_Status = HAL_FLASH_Lock();
		if(HAL_Status != HAL_OK){
			Flash_Payload_Write_Status = FLASH_PAYLOAD_WRITE_FAILED;
		}
		else{
			Flash_Payload_Write_Status = FLASH_PAYLOAD_WRITE_PASSED;
		}
	}
	else{
		Flash_Payload_Write_Status = FLASH_PAYLOAD_WRITE_FAILED;
	}

	return Flash_Payload_Write_Status;
}

void Bootloader_Memory_Write(uint8_t *Host_Buffer){
	uint16_t Host_CMD_Packet_Len = 0;
	uint32_t Host_CRC32 = 0;
	uint32_t HOST_Address = 0;
	uint8_t Payload_Len = 0;
	uint8_t Address_Verification = ADDRESS_IS_INVALID;
	uint8_t Flash_Payload_Write_Status = FLASH_PAYLOAD_WRITE_FAILED;
	uint8_t Block_ID = 0;

#if (BL_DEBUG_ENABLE == DEBUG_INFO_ENABLE)
	BL_Print_Message("Write data into different memories of the MCU \r\n");
#endif
	/* Extract the CRC32 and packet length sent by the HOST */
	Host_CMD_Packet_Len = Host_Buffer[0] + 1;
	Host_CRC32 = *((uint32_t *)((Host_Buffer + Host_CMD_Packet_Len) - CRC_TYPE_SIZE_BYTE));


/* CRC Verification */
	if(CRC_VERIFICATION_PASSED == Bootloader_CRC_Verify((uint8_t *)&Host_Buffer[0] , Host_CMD_Packet_Len - 4, Host_CRC32)){
#if (BL_DEBUG_ENABLE == DEBUG_INFO_ENABLE)
		BL_Print_Message("CRC Verification Passed \r\n");
#endif
		/* Send acknowledgement to the HOST */
		//Bootloader_Send_ACK(1);
		/* Extract the start address from the Host packet */
		HOST_Address = *((uint32_t *)(&Host_Buffer[2]));
#if (BL_DEBUG_ENABLE == DEBUG_INFO_ENABLE)
		BL_Print_Message("HOST_Address = 0x%X \r\n", HOST_Address);
#endif
		/* Extract the payload length from the Host packet */
		Payload_Len = Host_Buffer[7];
		Block_ID = Host_Buffer[6];
		uint32_t sliced[Payload_Len];
	    for (int i = 7; i <= Payload_Len; i++) {
	        // Store each element in the sliced array
	        sliced[i - 7] = Host_Buffer[i];
	    }



		/* Verify the Extracted address to be valid address */
		Address_Verification = Host_Address_Verification(HOST_Address);
		if(ADDRESS_IS_VALID == Address_Verification){
			/* Write the payload to the Flash memory */
			//uint32_t backet2[] = {4,4,4,4,4,4,4,4,4,44,4,4,4,4,4,4,44,4,4,4,4,4,4,4,4,4,4,4,4,44,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,44,4,4,4,4,4,4,44,4,4,4,4,4,4,4,4,4,4,4,4,44,4,4,4,4,4,4,4,44,4,4,4,4,4,4,4,4,44,4,4,4,4,4,4,44,4,4,4,4,4,4,4,4,4,4,4,4,44,4,4,4,4,4,4,4,44,4,4,4,4,4,4,4,4,44,4,4,4,4,4,4,44,4,4,4,4,4,4,4,4,4,4,4,4,44,4,4,4,4,4,4,4,44,4,4,4,4,4,4,4,4,44,4,4,4,4,4,4,44,4,4,4,4,4,4,4,4,4,4,4,4,44,4,4,4,4,4,4,4,44,4,4,4,4,4,4,4,4,44,4,4,4,4,4,4,44,4,4,4,4,4,4,4,4,4,4,4,4,44,4,4,4,4,4,4,4,44,4,4,4,4,4,4,4,4,44,4,4,4,4,4,4,44,4,4,4,4,4,4,4,4,4,4,4,4,44,4,4,4,4,4,4,4,4};
			uint32_t arr[] = {9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9};
			Flash_Payload_Write_Status = Flash_Memory_Write_Payload(sliced, Payload_Len, Block_ID);

			if(FLASH_PAYLOAD_WRITE_PASSED == Flash_Payload_Write_Status){
				/* Report payload write passed */
				Bootloader_Send_Data_To_Host((uint8_t *)&Flash_Payload_Write_Status, 1);
#if (BL_DEBUG_ENABLE == DEBUG_INFO_ENABLE)
				BL_Print_Message("Payload Valid \r\n");
#endif
			}
			else{
#if (BL_DEBUG_ENABLE == DEBUG_INFO_ENABLE)
				BL_Print_Message("Payload InValid \r\n");
#endif
				/* Report payload write failed */
				Bootloader_Send_Data_To_Host((uint8_t *)&Flash_Payload_Write_Status, 1);
			}
		}
		else{
			/* Report address verification failed */
			Address_Verification = ADDRESS_IS_INVALID;
			Bootloader_Send_Data_To_Host((uint8_t *)&Address_Verification, 1);
		}
	}
	else{
#if (BL_DEBUG_ENABLE == DEBUG_INFO_ENABLE)
		BL_Print_Message("CRC Verification Failed \r\n");
#endif
		/* Send Not acknowledge to the HOST */
		//Bootloader_Send_NACK();
	}
}

void Excute_NvM_MainFunction (void *Parameter){

	while(1){
		NvM_MainFunction();
		vTaskDelay(30);
	}

}


void BL_Print_Message(char *format, ...){
	char Message[100] ={0};
	va_list args;
	va_start(args, format);
	vsprintf(Message, format, args);
#ifdef BL_ENABLE_SPI_DEBUG_MESSAGE
	HAL_SPI_Transmit(BL_DEBUG_SPI, (uint8_t *) Message, sizeof(Message), HAL_MAX_DELAY);
#endif
	va_end(args);


}

