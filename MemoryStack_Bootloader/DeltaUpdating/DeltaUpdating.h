/*
 * DeltaUpdating.h
 *
 *  Created on: May 15, 2024
 *      Author: Mohamed Khaled
 */

#ifndef DELTAUPDATING_H_
#define DELTAUPDATING_H_

#include "stm32f4xx_hal.h"

// Flash memory parameters (example values, adjust as needed)
#define FLASH_START_ADDRESS 0x08000000
#define FLASH_PAGE_SIZE 1024
#define FLASH_SIZE 0x80000
#define CHUNK_SIZE 1024 // 1 KB chunk size

// Function prototypes
void generate_delta(const uint8_t *new_firmware, uint32_t new_size, uint32_t flash_address, uint8_t *delta, uint32_t *delta_size);
void apply_delta(const uint8_t *delta, uint32_t delta_size, uint32_t flash_address);
void read_flash_chunk(uint32_t address, uint8_t *buffer, uint32_t size);
void write_flash_chunk(uint32_t address, const uint8_t *data, uint32_t size);
uint32_t calculate_crc(const uint8_t *data, uint32_t size);

#endif /* DELTAUPDATING_H_ */
