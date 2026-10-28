#ifndef UTK_COMPAT_HARDWARE_FLASH_H
#define UTK_COMPAT_HARDWARE_FLASH_H
#include <stddef.h>
#include <stdint.h>
#ifndef PICO_FLASH_SIZE_BYTES
#define PICO_FLASH_SIZE_BYTES (2u * 1024u * 1024u)
#endif
#define FLASH_PAGE_SIZE 256u
#define FLASH_SECTOR_SIZE 4096u
void flash_range_erase(uint32_t flash_offs, size_t count);
void flash_range_program(uint32_t flash_offs, const uint8_t *data, size_t count);
#endif
