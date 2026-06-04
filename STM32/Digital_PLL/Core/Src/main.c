/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "lptim.h"
#include "lwip.h"
#include "memorymap.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "flash.h"
#include "rtc.h"
#include "wwdg.h"
#include "iwdg.h"
#include "tcp_server.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern uint8_t NMEA_DMA_buffer[UART_DMA_BUFFER_SIZE];
extern uint8_t BKP_DMA_buffer[UART_DMA_BUFFER_SIZE];
extern volatile uint32_t msg_wdg;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

static void MPU_Config(void);

/* USER CODE BEGIN PFP */
uint32_t find_magic_number(uint32_t period);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {

    /* USER CODE BEGIN 1 */
    init_memory();
    /* USER CODE END 1 */

    /* MPU Configuration--------------------------------------------------------*/
    MPU_Config();

    /* Enable the CPU Cache */

    /* Enable I-Cache---------------------------------------------------------*/
    SCB_EnableICache();

    /* Enable D-Cache---------------------------------------------------------*/
    SCB_EnableDCache();

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    // init configs as zero
    for (uint32_t i = 0; i < INPUT_CAPTURE_NUM; ++i)
        init_input_capture_config_zero(&input_capture_configs[i]);
    for (uint32_t i = 0; i < TIMER_CAPTURE_NUM; ++i)
        init_timer_capture_config_zero(&timer_capture_configs[i]);
    for (uint32_t i = 0; i < OUTPUT_TRIGGER_NUM; ++i)
        init_output_trigger_config_zero(&output_trigger_configs[i]);

    // list of all slave timers
    static TIM_HandleTypeDef slave_timers[] = {{.Instance = TIM8},
                                               {.Instance = TIM2},
                                               {.Instance = TIM3},
                                               {.Instance = TIM4},
                                               {.Instance = TIM5},
                                               {.Instance = TIM12},
                                               {.Instance = TIM15}
    };

    /** define physical interface of input & output **/
    set_physical_interface(&input_capture_configs[0].interface, &slave_timers[6], 2, GPIOE, GPIO_PIN_6, GPIO_AF4_TIM15);
    set_physical_interface(&input_capture_configs[1].interface, &slave_timers[1], 2, GPIOB, GPIO_PIN_3, GPIO_AF1_TIM2);
    set_physical_interface(&input_capture_configs[2].interface, &slave_timers[2], 2, GPIOB, GPIO_PIN_5, GPIO_AF2_TIM3);
    set_physical_interface(&input_capture_configs[3].interface, &slave_timers[0], 4, GPIOC, GPIO_PIN_9, GPIO_AF3_TIM8);
    set_physical_interface(&input_capture_configs[4].interface, &slave_timers[2], 4, GPIOB, GPIO_PIN_1, GPIO_AF2_TIM3);
    set_physical_interface(&input_capture_configs[5].interface, &slave_timers[4], 4, GPIOI, GPIO_PIN_0, GPIO_AF2_TIM5);
    set_physical_interface(&input_capture_configs[6].interface, &slave_timers[4], 2, GPIOH, GPIO_PIN_11, GPIO_AF2_TIM5);
    set_physical_interface(&input_capture_configs[7].interface, &MAIN_TIM, 4, GPIOE, GPIO_PIN_14, GPIO_AF1_TIM1);
    set_physical_interface(&input_capture_configs[8].interface, &slave_timers[5], 2, GPIOH, GPIO_PIN_9, GPIO_AF2_TIM12);
    set_physical_interface(&input_capture_configs[9].interface, &slave_timers[3], 2, GPIOD, GPIO_PIN_13, GPIO_AF2_TIM4);
    set_physical_interface(&input_capture_configs[10].interface, &slave_timers[0], 2, GPIOC, GPIO_PIN_7, GPIO_AF3_TIM8);
    set_physical_interface(&input_capture_configs[11].interface, &slave_timers[3], 4, GPIOD, GPIO_PIN_15, GPIO_AF2_TIM4);

    set_physical_interface(&output_trigger_configs[0].interface, &slave_timers[6], 1, GPIOE, GPIO_PIN_5, GPIO_AF4_TIM15);
    set_physical_interface(&output_trigger_configs[1].interface, &slave_timers[1], 1, GPIOA, GPIO_PIN_15, GPIO_AF1_TIM2);
    set_physical_interface(&output_trigger_configs[2].interface, &slave_timers[2], 1, GPIOB, GPIO_PIN_4, GPIO_AF2_TIM3);
    set_physical_interface(&output_trigger_configs[3].interface, &slave_timers[0], 3, GPIOC, GPIO_PIN_8, GPIO_AF3_TIM8);
    set_physical_interface(&output_trigger_configs[4].interface, &slave_timers[2], 3, GPIOB, GPIO_PIN_0, GPIO_AF2_TIM3);
    set_physical_interface(&output_trigger_configs[5].interface, &slave_timers[4], 3, GPIOH, GPIO_PIN_12, GPIO_AF2_TIM5);
    set_physical_interface(&output_trigger_configs[6].interface, &slave_timers[4], 1, GPIOH, GPIO_PIN_10, GPIO_AF2_TIM5);
    set_physical_interface(&output_trigger_configs[7].interface, &MAIN_TIM, 3, GPIOE, GPIO_PIN_13, GPIO_AF1_TIM1);
    set_physical_interface(&output_trigger_configs[8].interface, &slave_timers[5], 1, GPIOH, GPIO_PIN_6, GPIO_AF2_TIM12);
    set_physical_interface(&output_trigger_configs[9].interface, &slave_timers[3], 1, GPIOD, GPIO_PIN_12, GPIO_AF2_TIM4);
    set_physical_interface(&output_trigger_configs[10].interface, &slave_timers[0], 1, GPIOC, GPIO_PIN_6, GPIO_AF3_TIM8);
    set_physical_interface(&output_trigger_configs[11].interface, &slave_timers[3], 3, GPIOD, GPIO_PIN_14, GPIO_AF2_TIM4);

    /** Check if there is a config section in the Flash **/
    find_flash_config();

    /** get ip address and the mac address prefix (first 32 bits, the last 16 bits given by hash of UID of chip) [MUST BEFORE MX_LWIP_Init] **/
    if (FLAG_CHECK(sys_state, SYS_STATE_FLASH_DATA_AVAILABLE))
        load_eth_addr_from_flash();
    else {
        const uint8_t default_ip_addr[] = {192, 168, 11, 12};
        const uint8_t default_ip_mask[] = {255, 255, 255, 0};
        const uint8_t default_ip_gate[] = {192, 168, 11, 1};
        // OUI of STMicroelectronics: 00:80:E1; Code for Sync board: 01
        const uint8_t mac_prefix[] = {0x00, 0x80, 0xE1, 0x01};

        memcpy(&eth_addr_config.ip_addr, default_ip_addr, 4);
        memcpy(&eth_addr_config.ip_mask, default_ip_mask, 4);
        memcpy(&eth_addr_config.ip_gate, default_ip_gate, 4);
        memcpy(&eth_addr_config.mac_prefix, mac_prefix, 4);
    }

    /** start clock for TIM & GPIO **/
    __HAL_RCC_TIM8_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_TIM5_CLK_ENABLE();
    __HAL_RCC_TIM12_CLK_ENABLE();
    __HAL_RCC_TIM15_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOI_CLK_ENABLE();

    /** initial GPIO of input captures **/
    for (uint32_t i = 0; i < INPUT_CAPTURE_NUM; ++i)
        init_input_capture_gpio(&input_capture_configs[i]);
    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_TIM1_Init();
    MX_LPTIM1_Init();
    MX_USB_DEVICE_Init();
    MX_USART3_UART_Init();
    MX_SPI5_Init();
    MX_LWIP_Init();
    MX_USART1_UART_Init();
    /* USER CODE BEGIN 2 */
#ifdef ENABLE_WDG
    MX_IWDG1_Init();
#endif
    RTC_Init();
    check_error();
    TCP_Server_Init();

    /** get clock speed and calculate constants **/
    const uint32_t APB1_CLKFreq = HAL_RCC_GetPCLK1Freq();
    const uint32_t APB1_TIMER_CLKFreq = ((uint32_t) (RCC->D2CFGR & RCC_D2CFGR_D2PPRE1) != RCC_HCLK_DIV1) ? (2 * APB1_CLKFreq) : APB1_CLKFreq;
    const uint32_t crystal_oscillator_tolerance_in_clks = 1 + APB1_TIMER_CLKFreq / CRYSTAL_OSCILLATOR_ERROR_RATE;
    clks_per_pulse_range.max = APB1_TIMER_CLKFreq + crystal_oscillator_tolerance_in_clks;
    clks_per_pulse_range.min = APB1_TIMER_CLKFreq - crystal_oscillator_tolerance_in_clks;

    /** setup system main timer config **/
    if (FLAG_CHECK(sys_state, SYS_STATE_FLASH_DATA_AVAILABLE))
        load_system_timer_from_flash();
    else {
        timing_scale.pulse_per_second = 1ul;
        timing_scale.ticks_per_second = 12000ul;
        // TODO: second_clock_jitter_period need choice carefully to avoid jitter happen in same second
        second_clock_jitter_period = 363ul;
    }
    tick_per_wwdg = 4096ull * (3 * WWDG_WIN_SIZE - 1) * (uint64_t) timing_scale.ticks_per_second / APB1_CLKFreq;

    /** initial the clocks per pulse calibration data [MUST AFTER RTC_Init] **/
    if (FLAG_CHECK(sys_state, SYS_STATE_HOT_START_FLAG) && HAL_RTCEx_BKUPRead(&hrtc, FREQ_INITIALIZED_ADDR) == INITIALIZED_MAGIC_NUMBER
        && HAL_RTCEx_BKUPRead(&hrtc, FREQ_PPS_ADDR) == timing_scale.pulse_per_second) {
        // hot start mode, read the data from backup domain
        timing_scale.clks_per_pulse.integer = HAL_RTCEx_BKUPRead(&hrtc, FREQ_INTEGER_ADDR);
        timing_scale.clks_per_pulse.frac.numer = HAL_RTCEx_BKUPRead(&hrtc, FREQ_NUMER_ADDR);
        timing_scale.clks_per_pulse.frac.denom = HAL_RTCEx_BKUPRead(&hrtc, FREQ_DENOM_ADDR);
    }
    else if (FLAG_CHECK(sys_state, SYS_STATE_FLASH_DATA_AVAILABLE))
        load_clks_per_pulse_from_flash();
    else {
        timing_scale.clks_per_pulse.integer = APB1_TIMER_CLKFreq / timing_scale.pulse_per_second;
        timing_scale.clks_per_pulse.frac.denom = timing_scale.ticks_per_second * 10 + 1;
        timing_scale.clks_per_pulse.frac.numer = (APB1_TIMER_CLKFreq % timing_scale.pulse_per_second) * timing_scale.clks_per_pulse.frac.denom / timing_scale.pulse_per_second;
    }
    HAL_RTCEx_BKUPWrite(&hrtc, FREQ_PPS_ADDR, timing_scale.pulse_per_second); // save the pulse per second to backup domain

    /** load the timer capture and output trigger config **/
    if (FLAG_CHECK(sys_state, SYS_STATE_FLASH_DATA_AVAILABLE))
        load_config_from_flash();

    /** get magic number for clock jitter **/
    tick_pseudo_magic = find_magic_number(timing_scale.ticks_per_second);
    second_pseudo_magic = find_magic_number(timing_scale.ticks_per_second * second_clock_jitter_period);

    /** initial slave TIMs **/
    for (uint32_t i = 0; i < sizeof(slave_timers) / sizeof(*slave_timers); ++i)
        init_slave_tim(&slave_timers[i], TIM_TS_ITR0, slave_timers[i].Instance == TIM4);
    /** initial CC channel of input captures **/
    for (uint32_t i = 0; i < INPUT_CAPTURE_NUM; ++i)
        init_tim_input_capture(&input_capture_configs[i]);
    /** initial CC channel and GPIO of output triggers **/
    for (uint32_t i = 0; i < OUTPUT_TRIGGER_NUM; ++i) {
        init_tim_output_trigger(&output_trigger_configs[i]);
        init_output_trigger_gpio(&output_trigger_configs[i]);
    }

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */

    /** start SWDG to monitor other **/
    if (HAL_LPTIM_Counter_Start_IT(&hlptim1, APB1_CLKFreq / 128 / SOFTWARE_WDG_RATE - 1) != HAL_OK)
        error_handler(ERROR_FLAG_HAL_ERROR);

    /** start back up UART to receive command from host **/
    if (HAL_UARTEx_ReceiveToIdle_DMA(&BKP_UART, BKP_DMA_buffer, UART_DMA_BUFFER_SIZE) != HAL_OK)
        error_handler(ERROR_FLAG_UART);

    /** start UART to receive NMEA from GNSS **/
    if (HAL_UARTEx_ReceiveToIdle_DMA(&NMEA_UART, NMEA_DMA_buffer, UART_DMA_BUFFER_SIZE) != HAL_OK)
        error_handler(ERROR_FLAG_UART);

    /** initial input capture & output trigger state **/
    for (uint32_t i = 0; i < INPUT_CAPTURE_NUM; ++i)
        init_input_capture_state(&input_capture_states[i]);
    for (uint32_t i = 0; i < OUTPUT_TRIGGER_NUM; ++i)
        init_trigger_state(&output_trigger_states[i]);

    /** start CC channel of the timers **/
    for (uint32_t i = 0; i < INPUT_CAPTURE_NUM; ++i)
        if (TIM_IC_Setup(input_capture_configs[i].interface.htim, get_TIM_CHANNEL_x(input_capture_configs[i].interface.channel)) != HAL_OK)
            error_handler(ERROR_FLAG_HAL_ERROR);
    for (uint32_t i = 0; i < OUTPUT_TRIGGER_NUM; ++i)
        if (TIM_OC_Setup(output_trigger_configs[i].interface.htim, get_TIM_CHANNEL_x(output_trigger_configs[i].interface.channel)) != HAL_OK)
            error_handler(ERROR_FLAG_HAL_ERROR);

    /** start all slave timers **/
    for (uint32_t i = 0; i < sizeof(slave_timers) / sizeof(*slave_timers); ++i)
        if (HAL_TIM_Base_Start(&slave_timers[i]) != HAL_OK)
            error_handler(ERROR_FLAG_HAL_ERROR);

    // TIM 1 (Main timer) SETUP
    __HAL_TIM_SET_COMPARE(&MAIN_TIM, TIM_CHANNEL_2, timing_scale.pulse_per_second * timing_scale.clks_per_pulse.integer / timing_scale.ticks_per_second / 2 + 1);
    if (TIM_IC_Setup_IT(&MAIN_TIM, TIM_CHANNEL_1) != HAL_OK)     // TIM1 CC event
        error_handler(ERROR_FLAG_HAL_ERROR);
    if (TIM_PWM_Setup_IT(&MAIN_TIM, TIM_CHANNEL_2) != HAL_OK)    // TIM1 calibration and half-check
        error_handler(ERROR_FLAG_HAL_ERROR);

    // wait RTC second happen: sync TIM1 with RTC
    __HAL_TIM_SET_COUNTER(&MAIN_TIM, 0);
    wait_RTC_second_sync(&hrtc);
    if (HAL_TIM_Base_Start_IT(&MAIN_TIM) != HAL_OK)                      // Main clock start (last timer to start)
        error_handler(ERROR_FLAG_HAL_ERROR);

    // start WWDG for tick checking
#ifdef ENABLE_WDG
    if (tick_per_wwdg > 0)
        MX_WWDG1_Init();
#endif

    static __DMA_BUFFER char usb_msg_buffer[EVENT_MSG_STR_MAX_LEN];
    static __DMA_BUFFER char tcp_msg_buffer[EVENT_MSG_STR_MAX_LEN];
    static __DMA_BUFFER char bkp_msg_buffer[EVENT_MSG_STR_MAX_LEN];
#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
    while (1) {
        static uint32_t usb_curr_event_msg_idx = 0;
        static uint32_t tcp_curr_event_msg_idx = 0;
        static uint32_t bkp_curr_event_msg_idx = 0;

        if (usb_curr_event_msg_idx != next_event_msg_idx) {
            const uint32_t len = get_event_msg_str(usb_msg_buffer, &event_msg_buffer[usb_curr_event_msg_idx], usb_curr_event_msg_idx);
            if (CDC_Transmit_FS((uint8_t *) usb_msg_buffer, len) == USBD_OK)
                usb_curr_event_msg_idx = (usb_curr_event_msg_idx + 1) % EVENT_MSG_BUF_SIZE;
        }

        if (tcp_curr_event_msg_idx != next_event_msg_idx) {
            const uint32_t len = get_event_msg_str(tcp_msg_buffer, &event_msg_buffer[tcp_curr_event_msg_idx], tcp_curr_event_msg_idx);
            if (tcp_transmit(tcp_msg_buffer, len) == HAL_OK)
                tcp_curr_event_msg_idx = (tcp_curr_event_msg_idx + 1) % EVENT_MSG_BUF_SIZE;
        }

#ifndef ENABLE_DEBUG_MSG
        if (bkp_curr_event_msg_idx != next_event_msg_idx) {
            // the backup uart only output system message
            const int output_filter[] = {PULSE_EVENT, RTC_UPDATE_EVENT, COMMAND_RESPONSE_EVENT, PENDING_ERROR_EVENT, RETURN_CONFIG_EVENT};
            int output = 0;
            for (int i = 0; i < sizeof(output_filter) / sizeof(*output_filter); ++i)
                output = output || (event_msg_buffer[bkp_curr_event_msg_idx].type == output_filter[i]);

            if (output) {
                const uint32_t uart_state = HAL_UART_GetState(&BKP_UART);
                if (uart_state == HAL_UART_STATE_READY || uart_state == HAL_UART_STATE_BUSY_RX) { // if the previous sending didn't finish, skip this time
                    const uint32_t len = get_event_msg_str(bkp_msg_buffer, &event_msg_buffer[bkp_curr_event_msg_idx], bkp_curr_event_msg_idx);
                    if (HAL_UART_Transmit_DMA(&BKP_UART, (uint8_t *) bkp_msg_buffer, len) == HAL_OK)
                        bkp_curr_event_msg_idx = (bkp_curr_event_msg_idx + 1) % EVENT_MSG_BUF_SIZE;
                }
            }
            else
                bkp_curr_event_msg_idx = (bkp_curr_event_msg_idx + 1) % EVENT_MSG_BUF_SIZE;
        }
#endif

        msg_wdg = SOFTWARE_WDG_RATE;
        HAL_GPIO_TogglePin(ALIVE_GPIO_Port, ALIVE_Pin);

        MX_LWIP_Process();

        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
#pragma clang diagnostic pop
    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Supply configuration update enable
    */
    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

    /** Configure the main internal regulator output voltage
    */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

    /** Configure LSE Drive Capability
    */
    HAL_PWR_EnableBkUpAccess();
    __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_HIGH);

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSE
                                       | RCC_OSCILLATORTYPE_LSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;
    RCC_OscInitStruct.LSIState = RCC_LSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 3;
    RCC_OscInitStruct.PLL.PLLN = 72;
    RCC_OscInitStruct.PLL.PLLP = 2;
    RCC_OscInitStruct.PLL.PLLQ = 20;
    RCC_OscInitStruct.PLL.PLLR = 2;
    RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
    RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
    RCC_OscInitStruct.PLL.PLLFRACN = 0;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2
                                  | RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
        Error_Handler();
    }

    /** Enables the Clock Security System
    */
    HAL_RCC_EnableCSS();
}

/* USER CODE BEGIN 4 */
uint32_t gcd(uint32_t a, uint32_t b) {
    while (b != 0) {
        uint32_t t = b;
        b = a % b;
        a = t;
    }
    return a;
}

uint32_t find_magic_number(uint32_t period) {
    if (period <= 2)
        return 1;
    uint64_t min_err = period, best_magic = 1;
    for (uint64_t magic = 2; magic < period; ++magic)
        if (gcd(period, magic) == 1) {
            const uint64_t magic2 = magic * magic;
            const uint64_t error = (magic2 < period) ? (period - magic2) : (magic2 - period);
            if (min_err > error)
                min_err = error, best_magic = magic;
            else
                break;
        }
    return (uint32_t) best_magic;
}

#ifdef ENABLE_DEBUG_MSG

void debug_printf(const char *format, ...) {
    static __DMA_BUFFER char debug_buffer[DEBUG_MSG_BUFFER_SIZE];

    /** wait previous transmit finished **/
    uint32_t uart_state;
    do {
        uart_state = HAL_UART_GetState(&BKP_UART);
    } while (uart_state != HAL_UART_STATE_READY && uart_state != HAL_UART_STATE_BUSY_RX);

    /** format the string **/
    va_list args;
    va_start(args, format);
    int len = vsnprintf(debug_buffer, DEBUG_MSG_BUFFER_SIZE, format, args);
    va_end(args);

    /** transmit the message **/
    HAL_UART_Transmit_DMA(&BKP_UART, (uint8_t *) debug_buffer, len);
}

#endif

/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void) {
    MPU_Region_InitTypeDef MPU_InitStruct = {0};

    /* Disables the MPU */
    HAL_MPU_Disable();

    /** Initializes and configures the Region and the memory to be protected
    */
    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.Number = MPU_REGION_NUMBER0;
    MPU_InitStruct.BaseAddress = 0x0;
    MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
    MPU_InitStruct.SubRegionDisable = 0x87;
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
    MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
    MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
    MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    /** Initializes and configures the Region and the memory to be protected
    */
    MPU_InitStruct.Number = MPU_REGION_NUMBER1;
    MPU_InitStruct.BaseAddress = 0x24000000;
    MPU_InitStruct.Size = MPU_REGION_SIZE_512KB;
    MPU_InitStruct.SubRegionDisable = 0x0;
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;

    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    /** Initializes and configures the Region and the memory to be protected
    */
    MPU_InitStruct.Number = MPU_REGION_NUMBER2;
    MPU_InitStruct.BaseAddress = 0x30000000;
    MPU_InitStruct.Size = MPU_REGION_SIZE_256KB;
    MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    /** Initializes and configures the Region and the memory to be protected
    */
    MPU_InitStruct.Number = MPU_REGION_NUMBER3;
    MPU_InitStruct.BaseAddress = 0x30040000;
    MPU_InitStruct.Size = MPU_REGION_SIZE_32KB;

    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    /** Initializes and configures the Region and the memory to be protected
    */
    MPU_InitStruct.Number = MPU_REGION_NUMBER4;
    MPU_InitStruct.BaseAddress = 0x38000000;
    MPU_InitStruct.Size = MPU_REGION_SIZE_64KB;
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;

    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    /** Initializes and configures the Region and the memory to be protected
    */
    MPU_InitStruct.Number = MPU_REGION_NUMBER5;
    MPU_InitStruct.BaseAddress = 0x38800000;
    MPU_InitStruct.Size = MPU_REGION_SIZE_4KB;
    MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
    MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;

    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    /** Initializes and configures the Region and the memory to be protected
    */
    MPU_InitStruct.Number = MPU_REGION_NUMBER6;
    MPU_InitStruct.BaseAddress = 0x30040000;
    MPU_InitStruct.Size = MPU_REGION_SIZE_512B;
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;

    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    /** Initializes and configures the Region and the memory to be protected
    */
    MPU_InitStruct.Number = MPU_REGION_NUMBER7;
    MPU_InitStruct.BaseAddress = 0x30020000;
    MPU_InitStruct.Size = MPU_REGION_SIZE_128KB;
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
    MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

    HAL_MPU_ConfigRegion(&MPU_InitStruct);
    /* Enables the MPU */
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    error_handler(ERROR_FLAG_HAL_ERROR);
    /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line) {
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    shutdown_mode();
    static __DMA_BUFFER char msg_buffer[256];
    uint16_t len = sprintf(msg_buffer, "ASSERT FAILED @ file: %s, line: %ld\r\n", file, line);
    while (1)
        HAL_UART_Transmit_DMA(&BKP_UART, (uint8_t *) msg_buffer, len);
    /* USER CODE END 6 */
}

#endif /* USE_FULL_ASSERT */
