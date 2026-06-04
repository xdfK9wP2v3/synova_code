#ifndef IMU_FLASH_H
#define IMU_FLASH_H

#define CONFIG_VERSION (~1)
#define CONFIG_BANK FLASH_BANK_2
#define CONFIG_SECTOR FLASH_SECTOR_7
#define CONFIG_BASE_ADDR ((uint32_t) 0x081E0000)
#define FLASH_LINE_WIDTH (256 / 8) // flash is 256bits / group = 32bytes / group

HAL_StatusTypeDef erase_flash_config_sector();

HAL_StatusTypeDef save_config_to_flash();

void find_flash_config();

void load_config_from_flash();

void load_eth_addr_from_flash();

#endif //IMU_FLASH_H
