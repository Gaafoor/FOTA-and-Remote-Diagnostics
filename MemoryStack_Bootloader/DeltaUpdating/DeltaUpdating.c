/*
 * DeltaUpdating.c
 *
 *  Created on: May 15, 2024
 *      Author: Mohamed Khaled
 */


#include "DeltaUpdating.h"


// Function to generate the delta update from new firmware and existing firmware
// Parameters:
// - new_firmware: Pointer to the new firmware data
// - new_size: Size of the new firmware data
// - flash_address: Starting address of the existing firmware in flash memory
// - delta: Pointer to the buffer to store the generated delta update
// - delta_size: Pointer to store the size of the generated delta update
void generate_delta(const uint8_t *new_firmware, uint32_t new_size, uint32_t flash_address, uint8_t *delta, uint32_t *delta_size) {
    uint32_t delta_index = 0; // Initialize delta array index
    uint8_t flash_chunk[CHUNK_SIZE]; // Buffer to hold a chunk of flash memory

    // Loop through the new firmware in chunks of 1 KB
    for (uint32_t i = 0; i < new_size; i += CHUNK_SIZE) {
        // Calculate the actual chunk size, which might be less than 1 KB for the last chunk
        uint32_t chunk_size = (new_size - i) < CHUNK_SIZE ? (new_size - i) : CHUNK_SIZE;

        // Read a chunk from the existing firmware in flash memory
        read_flash_chunk(flash_address + i, flash_chunk, chunk_size);

        // Calculate CRCs for the new firmware chunk and the flash memory chunk
        uint32_t crc_new = calculate_crc(&new_firmware[i], chunk_size);
        uint32_t crc_flash = calculate_crc(flash_chunk, chunk_size);

        // If CRCs are different, record the chunk in the delta
        if (crc_new != crc_flash) {
            // Store the offset and size of the differing chunk in the delta array
            delta[delta_index++] = (i & 0xFF);
            delta[delta_index++] = ((i >> 8) & 0xFF);
            delta[delta_index++] = ((i >> 16) & 0xFF);
            delta[delta_index++] = ((i >> 24) & 0xFF);
            delta[delta_index++] = (chunk_size & 0xFF);
            delta[delta_index++] = ((chunk_size >> 8) & 0xFF);

            // Store the new chunk data in the delta array
            for (uint32_t j = 0; j < chunk_size; ++j) {
                delta[delta_index++] = new_firmware[i + j];
            }
        }
    }
    *delta_size = delta_index; // Set the size of the delta array
}




// Function to apply the delta update to the firmware in flash memory
// Parameters:
// - delta: Pointer to the delta update data
// - delta_size: Size of the delta update data
// - flash_address: Starting address of the firmware in flash memory
void apply_delta(const uint8_t *delta, uint32_t delta_size, uint32_t flash_address) {
    uint32_t i = 0; // Initialize delta array index
    while (i < delta_size) {
        // Read the offset and chunk size from the delta array
        uint32_t offset = delta[i] | (delta[i + 1] << 8) | (delta[i + 2] << 16) | (delta[i + 3] << 24);
        uint32_t chunk_size = delta[i + 4] | (delta[i + 5] << 8);
        i += 6; // Increment index to point to the chunk data

        // Write the new chunk data to flash memory
        write_flash_chunk(flash_address + offset, &delta[i], chunk_size);
        i += chunk_size; // Increment index to point to the next delta entry
    }
}





// Function to read a chunk of data from flash memory into a buffer
// Parameters:
// - address: Address of the flash memory to read from
// - buffer: Pointer to the buffer to store the read data
// - size: Size of the data to read
void read_flash_chunk(uint32_t address, uint8_t *buffer, uint32_t size) {
    for (uint32_t i = 0; i < size; ++i) {
        buffer[i] = *(volatile uint8_t *)(address + i);
    }
}







// Function to write a chunk of data to flash memory
// Parameters:
// - address: Address of the flash memory to write to
// - data: Pointer to the data to write
// - size: Size of the data to write
void write_flash_chunk(uint32_t address, const uint8_t *data, uint32_t size) {
    HAL_FLASH_Unlock(); // Unlock flash memory for writing

    // Write each byte of the chunk to flash memory
    for (uint32_t i = 0; i < size; ++i) {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, address + i, data[i]);
    }

    HAL_FLASH_Lock(); // Lock flash memory after writing
}






// Function to calculate the CRC32 checksum of a data array
// Parameters:
// - data: Pointer to the data array
// - size: Size of the data array
// Returns: CRC32 checksum of the data array
uint32_t calculate_crc(const uint8_t *data, uint32_t size) {
    uint32_t crc = 0xFFFFFFFF; // Initial CRC value
    for (uint32_t i = 0; i < size; ++i) {
        crc ^= data[i]; // XOR byte with CRC
        for (uint8_t j = 0; j < 8; ++j) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xEDB88320; // Apply polynomial if LSB is set
            } else {
                crc >>= 1; // Shift right if LSB is not set
            }
        }
    }
    return ~crc; // Return the complement of the CRC value
}
