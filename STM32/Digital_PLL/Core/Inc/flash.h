#ifndef DIGITAL_PLL_FLASH_H
#define DIGITAL_PLL_FLASH_H

HAL_StatusTypeDef erase_flash_config_sector();

HAL_StatusTypeDef save_config_to_flash();

void find_flash_config();

void load_config_from_flash();

void load_eth_addr_from_flash();

void load_system_timer_from_flash();

void load_clks_per_pulse_from_flash();

void load_rtc_from_flash();

#endif //DIGITAL_PLL_FLASH_H
