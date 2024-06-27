
#ifndef INC_BOOTLOADER_H_
#define INC_BOOTLOADER_H_

/* ----------------- Includes -----------------*/

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "spi.h"
#include "crc.h"
#include "NVM.h"



/* ----------------- Macro Declarations -----------------*/
#define BL_DEBUG_SPI                    &hspi1
#define BL_HOST_COMMUNICATION_SPI       &hspi1
#define CRC_ENGINE_OBJ                  &hcrc

#define DEBUG_INFO_DISABLE           0
#define DEBUG_INFO_ENABLE            1
#define BL_DEBUG_ENABLE              DEBUG_INFO_DISABLE

#define BL_HOST_BUFFER_RX_LENGTH     200


#define BL_ENABLE_SPI_DEBUG_MESSAGE     0x00



#define CBL_GET_VER_CMD              0x10
#define CBL_GET_HELP_CMD             0x11
#define CBL_GET_CID_CMD              0x12
/* Get Read Protection Status */
#define CBL_GET_RDP_STATUS_CMD       0x13
#define CBL_GO_TO_ADDR_CMD           0x14
#define CBL_FLASH_ERASE_CMD          0x15
#define CBL_MEM_WRITE_CMD            0x16
/* Enable/Disable Write Protection */
#define CBL_ED_W_PROTECT_CMD         0x17
#define CBL_MEM_READ_CMD             0x18
/* Get Sector Read/Write Protection Status */
#define CBL_READ_SECTOR_STATUS_CMD   0x19
#define CBL_OTP_READ_CMD             0x20
/* Change Read Out Protection Level */
#define CBL_CHANGE_ROP_Level_CMD     0x21

#define CBL_VENDOR_ID                100
#define CBL_SW_MAJOR_VERSION         1
#define CBL_SW_MINOR_VERSION         1
#define CBL_SW_PATCH_VERSION         0

/* CRC_VERIFICATION */
#define CRC_TYPE_SIZE_BYTE           4

#define CRC_VERIFICATION_FAILED      0x00
#define CRC_VERIFICATION_PASSED      0x01

#define CBL_SEND_NACK                0xAB
#define CBL_SEND_ACK                 0xCD

/* Start address of sector 2 */
#define FLASH_SECTOR2_BASE_ADDRESS   0x08008000U
#define FLASH_SECTOR3_BASE_ADDRESS   0x0800C000U

#define ADDRESS_IS_INVALID           0x00
#define ADDRESS_IS_VALID             0x01

#define STM32F401_SRAM1_SIZE          (96 * 1024)
#define STM32F401_FLASH_SIZE          (512 * 1024)

#define STM32F401_SRAM1_END           (SRAM1_BASE + STM32F401_SRAM1_SIZE)
#define STM32F401_FLASH_END           (FLASH_BASE + STM32F401_FLASH_SIZE)

/* CBL_FLASH_ERASE_CMD */
#define CBL_FLASH_MAX_SECTOR_NUMBER  12
#define CBL_FLASH_MASS_ERASE         0xFF

#define INVALID_SECTOR_NUMBER        0x00
#define VALID_SECTOR_NUMBER          0x01
#define UNSUCCESSFUL_ERASE           0x02
#define SUCCESSFUL_ERASE             0x03

#define HAL_SUCCESSFUL_ERASE         0xFFFFFFFFU

/* CBL_MEM_WRITE_CMD */
#define FLASH_PAYLOAD_WRITE_FAILED   0x00
#define FLASH_PAYLOAD_WRITE_PASSED   0x01

#define FLASH_LOCK_WRITE_FAILED      0x00
#define FLASH_LOCK_WRITE_PASSED      0x01

/* CBL_GET_RDP_STATUS_CMD */
#define ROP_LEVEL_READ_INVALID       0x00
#define ROP_LEVE_READL_VALID         0X01

/* CBL_CHANGE_ROP_Level_CMD */
#define ROP_LEVEL_CHANGE_INVALID     0x00
#define ROP_LEVEL_CHANGE_VALID       0X01

#define CBL_ROP_LEVEL_0              0x00
#define CBL_ROP_LEVEL_1              0x01
#define CBL_ROP_LEVEL_2              0x02

/* ----------------- Macro Functions Declarations -----------------*/


/* ----------------- Data Type Declarations -----------------*/

typedef enum{
	BL_NACK = 0,
	BL_OK
}BL_Status;

typedef void (*pMainApp)(void);
typedef void (*Jump_Ptr)(void);

/* ----------------- Software Interfaces Declarations -----------------*/
void BL_Print_Message(char *format, ...);
BL_Status BL_SPI_Fetch_Host_Command(void);
void Bootloader_Memory_Write(uint8_t *Host_Buffer);
uint8_t Perform_Flash_Erase(uint8_t Sector_Numebr, uint8_t Number_Of_Sectors);
#endif /* INC_BOOTLOADER_H_ */
