#include "main.h"
#include "flash.h"

#define FLASH_LINE_IN_UINT32 (FLASH_LINE_WIDTH / sizeof(uint32_t))

extern RTC_HandleTypeDef hrtc;

uint32_t flash_curr_base_addr = CONFIG_BASE_ADDR;
uint32_t flash_next_base_addr = CONFIG_BASE_ADDR;

inline uint32_t size_align_flash(const uint32_t size) {
  return (size / FLASH_LINE_WIDTH) * FLASH_LINE_WIDTH + (size % FLASH_LINE_WIDTH != 0 ? FLASH_LINE_WIDTH : 0);
}

HAL_StatusTypeDef erase_flash_config_sector() {
  FLASH_EraseInitTypeDef FlashEraseInit = {.TypeErase=FLASH_TYPEERASE_SECTORS, .VoltageRange=FLASH_VOLTAGE_RANGE_3, .Banks=CONFIG_BANK, .Sector=CONFIG_SECTOR, .NbSectors=1};
  uint32_t SectorError = 0;
  HAL_FLASH_Unlock();
  if (HAL_FLASHEx_Erase(&FlashEraseInit, &SectorError) != HAL_OK)
    return HAL_ERROR;
  if (SectorError != 0xFFFFFFFF)
    return HAL_ERROR;
  if (FLASH_WaitForLastOperation(HAL_MAX_DELAY, CONFIG_BANK) != HAL_OK)
    return HAL_ERROR;
  HAL_FLASH_Lock();
  return HAL_OK;
}

HAL_StatusTypeDef save_config_to_flash() {

  HAL_FLASH_Unlock(); // unlock flash to start flash operations

  /** get the addr section of new flash section **/
  flash_curr_base_addr = flash_next_base_addr;
  const uint32_t config_data_size = FLASH_LINE_WIDTH                                                                   // config header
                                    + FLASH_LINE_WIDTH                                                                 // eth config
                                    + size_align_flash(sizeof(RTKConfig))            // timer capture config
                                    + size_align_flash(sizeof(IMUFreqConfig));    // output trigger config
  flash_next_base_addr = flash_curr_base_addr + config_data_size;

  /** erase the config sector if there is no enough space **/
  int erase = flash_next_base_addr > CONFIG_BASE_ADDR + FLASH_SECTOR_SIZE; // if flash has enough space to save a new config section
  for (uint8_t *p = (uint8_t *) flash_curr_base_addr, *q = (uint8_t *) flash_next_base_addr; !erase && p < q; ++p) // check if the writing area has been initialed
    erase = *p != 0xFF; // equal to (erase || ~(*p) != 0), but as !erase checked in loop condition, so erase always be 0
  if (erase) {
    if (erase_flash_config_sector() != HAL_OK)
      goto flash_error;
    flash_curr_base_addr = CONFIG_BASE_ADDR; // after erase, write the data from begin
  }

  /** Leave the Config Header to the last write (avoid unfinished writing) **/
  uint32_t flash_writing_addr = flash_curr_base_addr + FLASH_LINE_WIDTH;

  /** Save system variable (frequency data & other calibration data) **/
  __attribute__((aligned(FLASH_LINE_WIDTH))) uint32_t eth_data[FLASH_LINE_IN_UINT32] = {
          eth_addr_config.ip_addr, eth_addr_config.ip_mask, eth_addr_config.ip_gate, eth_addr_config.mac_prefix,
          ~0, ~0, ~0, ~0
  };
  if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, flash_writing_addr, (uint32_t) eth_data) != HAL_OK)
    goto flash_error;
  flash_writing_addr += FLASH_LINE_WIDTH;

  /** save config **/
  // the two part of config need to save
  const uint8_t *const config_addr[] = {(const uint8_t *) &rtk_config, (const uint8_t *) &imu_freq_config};
  const uint16_t config_size[] = {sizeof(rtk_config), sizeof(imu_freq_config)};
  // saving the configs
  for (uint32_t config_idx = 0; config_idx < sizeof(config_size) / sizeof(*config_size); ++config_idx) {
    __attribute__((aligned(FLASH_LINE_WIDTH))) uint8_t data_buf[FLASH_LINE_WIDTH] = {0};
    for (uint32_t flash_bias = 0; flash_bias < config_size[config_idx]; flash_bias += FLASH_LINE_WIDTH, flash_writing_addr += FLASH_LINE_WIDTH) {
      for (uint32_t data_bias = 0; flash_bias + data_bias < config_size[config_idx] && data_bias < FLASH_LINE_WIDTH; ++data_bias)
        data_buf[data_bias] = config_addr[config_idx][flash_bias + data_bias];
      if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, flash_writing_addr, (uint32_t) data_buf) != HAL_OK)
        goto flash_error;
    }
  }

  /** save config header at least (indicate all config saved successful) **/
#define NEXT_FLASH_ADDR_OFFSET 1 // the next config section address is on position 1 of the header
  __attribute__((aligned(FLASH_LINE_WIDTH))) uint32_t header[FLASH_LINE_IN_UINT32] = {
          CONFIG_VERSION, flash_writing_addr,
          (sizeof(rtk_config) << 8) + sizeof(imu_freq_config),
          1, 1,
          ~0, ~0, ~0
  };
  /** add check sum to config header **/
  uint32_t checksum = 0xAA;
  for (int i = 0; i < FLASH_LINE_IN_UINT32 - 1; ++i)
    checksum ^= header[i];
  header[FLASH_LINE_IN_UINT32 - 1] = checksum;
  if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, flash_curr_base_addr, (uint32_t) header) != HAL_OK)
    goto flash_error;

  /** normal return **/
  HAL_FLASH_Lock();
  return HAL_OK;

  /** ERROR return **/
  flash_error:
  HAL_FLASH_Lock();
  return HAL_ERROR;
}

void find_flash_config() {
  /** find last config section in flash **/
  flash_curr_base_addr = CONFIG_BASE_ADDR;
  flash_next_base_addr = CONFIG_BASE_ADDR;
  int has_flash_config = 0;
  while (1) {
    /** safe check on the potential config section **/
    if (flash_next_base_addr < CONFIG_BASE_ADDR || CONFIG_BASE_ADDR + FLASH_SECTOR_SIZE <= flash_next_base_addr + FLASH_LINE_WIDTH)
      break;
    /** check if version field has data (or it's an unused section) **/
    const uint32_t *header_line = (uint32_t *) flash_next_base_addr;
    if (~header_line[0] == 0)
      break;

    /** config config_header checksum verify **/
    uint32_t checksum = 0xAA;
    for (int i = 0; i < FLASH_LINE_IN_UINT32 - 1; ++i)
      checksum ^= header_line[i];
    if (header_line[FLASH_LINE_IN_UINT32 - 1] != checksum)
      break;

    /** safe check on the next section pointer **/
    if (header_line[NEXT_FLASH_ADDR_OFFSET] <= CONFIG_BASE_ADDR || CONFIG_BASE_ADDR + FLASH_SECTOR_SIZE < header_line[NEXT_FLASH_ADDR_OFFSET])
      break;

    /** find a config section **/
    flash_curr_base_addr = flash_next_base_addr;
    flash_next_base_addr = header_line[NEXT_FLASH_ADDR_OFFSET];
    has_flash_config = 1;
  }
  /** if no config section found, skip **/
  if (!has_flash_config)
    return;

  /** check if the last config section has same config version with current setup **/
  const uint32_t *config_header = (uint32_t *) flash_curr_base_addr;
  if (config_header[0] == CONFIG_VERSION && config_header[2] == (sizeof(rtk_config) << 8) + sizeof(imu_freq_config) &&
      config_header[3] == 1 && config_header[4] == 1)
    flash_data_available = 1;
}

void load_eth_addr_from_flash() {
  if (!flash_data_available)
    return;
  const uint32_t *eth_addr_line = (uint32_t *) (flash_curr_base_addr + 1 * FLASH_LINE_WIDTH);

  eth_addr_config.ip_addr = eth_addr_line[0];
  eth_addr_config.ip_mask = eth_addr_line[1];
  eth_addr_config.ip_gate = eth_addr_line[2];
  eth_addr_config.mac_prefix = eth_addr_line[3];
}

void load_config_from_flash() {
  if (!flash_data_available)
    return;
  const RTKConfig *const flash_rtk_config = (const RTKConfig *) (flash_curr_base_addr + 2 * FLASH_LINE_WIDTH);
  rtk_config = *flash_rtk_config;

  const IMUFreqConfig *const flash_imu_freq_config = (const IMUFreqConfig *) (
          flash_curr_base_addr + 2 * FLASH_LINE_WIDTH + size_align_flash(sizeof(RTKConfig)));
  imu_freq_config = *flash_imu_freq_config;
}