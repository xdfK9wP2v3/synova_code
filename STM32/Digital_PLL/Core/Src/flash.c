#include <memory.h>
#include "main.h"
#include "flash.h"

#define FLASH_LINE_IN_UINT32 (FLASH_LINE_WIDTH / sizeof(uint32_t))

extern RTC_HandleTypeDef hrtc;

uint32_t flash_curr_base_addr = CONFIG_BASE_ADDR;
uint32_t flash_next_base_addr = CONFIG_BASE_ADDR;

inline static uint32_t size_align_flash(const uint32_t size) {
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
                                      + FLASH_LINE_WIDTH                                                                 // frequency calibration data
                                      + FLASH_LINE_WIDTH                                                                 // eth config
                                      + size_align_flash(sizeof(TimerCaptureConfig) * TIMER_CAPTURE_NUM)            // timer capture config
                                      + size_align_flash(sizeof(OutputTriggerPulseConfig) * OUTPUT_TRIGGER_NUM);    // output trigger config
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
    __attribute__((aligned(FLASH_LINE_WIDTH))) uint32_t system_data[FLASH_LINE_IN_UINT32] = {
            timing_scale.pulse_per_second, timing_scale.clks_per_pulse.integer, timing_scale.clks_per_pulse.frac.numer, timing_scale.clks_per_pulse.frac.denom,
            timing_scale.ticks_per_second, second_clock_jitter_period, hrtc.Instance->PRER, hrtc.Instance->CALR
    };
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, flash_writing_addr, (uint32_t) system_data) != HAL_OK)
        goto flash_error;
    flash_writing_addr += FLASH_LINE_WIDTH;

    __attribute__((aligned(FLASH_LINE_WIDTH))) uint32_t eth_data[FLASH_LINE_IN_UINT32] = {
            eth_addr_config.ip_addr, eth_addr_config.ip_mask, eth_addr_config.ip_gate, eth_addr_config.mac_prefix,
            ~0, ~0, ~0, ~0
    };
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, flash_writing_addr, (uint32_t) eth_data) != HAL_OK)
        goto flash_error;
    flash_writing_addr += FLASH_LINE_WIDTH;

    /** save config **/
    CaptureEdge input_capture_edge_config[INPUT_CAPTURE_NUM];
    for (int i = 0; i < INPUT_CAPTURE_NUM; ++i)
        input_capture_edge_config[i] = input_capture_configs[i].capture_edge;
    OutputTriggerPulseConfig output_trigger_pulse_config[OUTPUT_TRIGGER_NUM]; // make array that only contain pulse config (without physics interface)
    for (int i = 0; i < OUTPUT_TRIGGER_NUM; ++i) {
        output_trigger_pulse_config[i] = output_trigger_configs[i].pulse;
    }

    // the three part of config need to save
    const uint8_t *const config_addr[] = {
            (const uint8_t *) input_capture_edge_config,
            (const uint8_t *) timer_capture_configs,
            (const uint8_t *) output_trigger_pulse_config
    };
    const uint16_t config_size[] = {
            sizeof(input_capture_edge_config),
            sizeof(timer_capture_configs),
            sizeof(output_trigger_pulse_config)
    };

    // saving the configs
    for (uint32_t config_idx = 0; config_idx < sizeof(config_size) / sizeof(*config_size); ++config_idx) {
        for (uint32_t flash_bias = 0; flash_bias < config_size[config_idx]; flash_bias += FLASH_LINE_WIDTH, flash_writing_addr += FLASH_LINE_WIDTH) {
            __attribute__((aligned(FLASH_LINE_WIDTH))) uint8_t data_buf[FLASH_LINE_WIDTH];
            memset(data_buf, ~0, sizeof(data_buf));
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
            (sizeof(TimerCaptureConfig) << 8) + sizeof(OutputTriggerPulseConfig),
            TIMER_CAPTURE_NUM, OUTPUT_TRIGGER_NUM,
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
    if (config_header[0] == CONFIG_VERSION && config_header[2] == (sizeof(TimerCaptureConfig) << 8) + sizeof(OutputTriggerPulseConfig) &&
        config_header[3] == TIMER_CAPTURE_NUM && config_header[4] == OUTPUT_TRIGGER_NUM)
        ATOMIC_FLAG_SET(sys_state, SYS_STATE_FLASH_DATA_AVAILABLE);
}

void load_eth_addr_from_flash() {
    assert_param(FLAG_CHECK(sys_state, SYS_STATE_FLASH_DATA_AVAILABLE));
    const uint32_t *eth_addr_line = (uint32_t *) (flash_curr_base_addr + 2 * FLASH_LINE_WIDTH);

    eth_addr_config.ip_addr = eth_addr_line[0];
    eth_addr_config.ip_mask = eth_addr_line[1];
    eth_addr_config.ip_gate = eth_addr_line[2];
    eth_addr_config.mac_prefix = eth_addr_line[3];
}

void load_config_from_flash() {
    assert_param(FLAG_CHECK(sys_state, SYS_STATE_FLASH_DATA_AVAILABLE));
    uint32_t config_base_addr = flash_curr_base_addr + 3 * FLASH_LINE_WIDTH;

    const CaptureEdge *const flash_input_capture_config = (const CaptureEdge *) config_base_addr;
    for(int i = 0; i < INPUT_CAPTURE_NUM; ++i)
        input_capture_configs[i].capture_edge = flash_input_capture_config[i];
    config_base_addr += size_align_flash(sizeof(CaptureEdge) * INPUT_CAPTURE_NUM);

    const TimerCaptureConfig *const flash_timer_capture_config = (const TimerCaptureConfig *) config_base_addr;
    for (int i = 0; i < TIMER_CAPTURE_NUM; ++i)
        timer_capture_configs[i] = flash_timer_capture_config[i];
    config_base_addr += size_align_flash(sizeof(TimerCaptureConfig) * TIMER_CAPTURE_NUM);

    const OutputTriggerPulseConfig *const flash_output_trigger_config = (const OutputTriggerPulseConfig *) config_base_addr;
    for (int i = 0; i < OUTPUT_TRIGGER_NUM; ++i)
        output_trigger_configs[i].pulse = flash_output_trigger_config[i];
    config_base_addr += size_align_flash(sizeof(OutputTriggerPulseConfig) * OUTPUT_TRIGGER_NUM);
}

void load_system_timer_from_flash() {
    assert_param(FLAG_CHECK(sys_state, SYS_STATE_FLASH_DATA_AVAILABLE));
    const uint32_t *freq_calib_line = (uint32_t *) (flash_curr_base_addr + FLASH_LINE_WIDTH);

    timing_scale.pulse_per_second = freq_calib_line[0];
    timing_scale.ticks_per_second = freq_calib_line[4];
    second_clock_jitter_period = freq_calib_line[5];
}

void load_clks_per_pulse_from_flash() {
    assert_param(FLAG_CHECK(sys_state, SYS_STATE_FLASH_DATA_AVAILABLE));
    const uint32_t *freq_calib_line = (uint32_t *) (flash_curr_base_addr + FLASH_LINE_WIDTH);

    timing_scale.clks_per_pulse.integer = freq_calib_line[1];
    timing_scale.clks_per_pulse.frac.numer = freq_calib_line[2];
    timing_scale.clks_per_pulse.frac.denom = freq_calib_line[3];
}

void load_rtc_from_flash() {
    assert_param(FLAG_CHECK(sys_state, SYS_STATE_FLASH_DATA_AVAILABLE));
    const uint32_t *freq_calib_line = (uint32_t *) (flash_curr_base_addr + FLASH_LINE_WIDTH);

    __HAL_RTC_WRITEPROTECTION_DISABLE(&hrtc);
    hrtc.Instance->PRER = freq_calib_line[6];
    hrtc.Instance->CALR = freq_calib_line[7];
    __HAL_RTC_WRITEPROTECTION_ENABLE(&hrtc);
}