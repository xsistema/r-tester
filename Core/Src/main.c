/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SAMPLES 100

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

DAC_HandleTypeDef hdac;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

//Atramine itampa.
uint16_t dac_value = 4095;

//Matavimo riba.
uint8_t diap = 0;

//Maksimali verte - kalibruota.
uint16_t maks[] = {
    3959,//200
    3988,//2K
    3997,//20K
    3990//100K
};

//Matavimo ribu koeficientai.
uint16_t koef[] = {
    200,
    2,
    20,
    100
};

/*
Matavimo ribos indikavimo komandos.
*/
unsigned char cmd_200_Ohm[] = {0x74, 0x34, 0x2E, 0x74, 0x78, 0x74, 0x3D, 0x22, 0x32, 0x30, 0x30, 0x20, 0xE2, 0x84, 0xA6, 0x22};//t4.txt="200 Ohm" - 16 baitu
unsigned char cmd_2_kOhm[] = {0x74, 0x34, 0x2E, 0x74, 0x78, 0x74, 0x3D, 0x22, 0x32, 0x20, 0x6B, 0xE2, 0x84, 0xA6, 0x22};//t4.txt="2 kOhm" - 15 baitu
unsigned char cmd_20_kOhm[] = {0x74, 0x34, 0x2E, 0x74, 0x78, 0x74, 0x3D, 0x22, 0x32, 0x30, 0x20, 0x6B, 0xE2, 0x84, 0xA6, 0x22};//t4.txt="20 kOhm" - 16 baitu
unsigned char cmd_100_kOhm[] = {0x74, 0x34, 0x2E, 0x74, 0x78, 0x74, 0x3D, 0x22, 0x31, 0x30, 0x30, 0x20, 0x6B, 0xE2, 0x84, 0xA6, 0x22};//t4.txt="100 kOhm" - 17 baitu

/*
Matavimo proceso indikatoriaus kontroles komandos.
*/
unsigned char cmd_tick_on[] = {0x74, 0x35, 0x2E, 0x61, 0x70, 0x68, 0x3D, 0x31, 0x32, 0x37};//t5.aph=127 - 10 baitu
unsigned char cmd_tick_off[] = {0x74, 0x35, 0x2E, 0x61, 0x70, 0x68, 0x3D, 0x30};//t5.aph=127 - 8 baitai

/*
Nextion komandos terminavimo seka.
*/
static uint8_t LF_pattern[] = {0xFF, 0xFF, 0xFF};

/*
Matavimo vienetai.
*/
unsigned char s_ohm[] = {0xE2, 0x84, 0xA6};//Ohm
unsigned char s_kohm[] = {0x6B, 0xE2, 0x84, 0xA6};//kOhm

/*
Struktura naudojama priimant UART duomenis.
*/
typedef struct {
    uint8_t buffer[16];
    volatile uint16_t head;
    volatile uint16_t tail;
} uart_rx_buffer_t;
uart_rx_buffer_t uart_rx = { .head = 0, .tail = 0 };

uint8_t sample_count = 0;
uint16_t ADC_samples[SAMPLES];
float R;
uint8_t rx_data;

unsigned char ADC_Ready = 1;
char buffr[25];
char rToNextion[25];
size_t txt_len = 0;
unsigned char mat_vnt[4];
bool tick_on = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*
Isciuncia komandos terminavimo seka i Nextion.
*/
void sendTerm() {
    HAL_UART_Transmit(&huart1, (uint8_t*)LF_pattern, 3, 10);
}

/*
Atlieka matavimo ribos perjungima.
*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        HAL_UART_Receive_IT(&huart1, &rx_data, 1);

        uart_rx.buffer[uart_rx.head] = rx_data;

        if (uart_rx.head > 1
        && uart_rx.buffer[uart_rx.head - 2] == 0xFF
        && uart_rx.buffer[uart_rx.head - 1] == 0xFF
        && uart_rx.buffer[uart_rx.head] == 0xFF) { //Visi duomenys priimti (atpazinta komandos tarminavimo seka 0xFF, 0xFF, 0xFF.

            switch (uart_rx.buffer[uart_rx.head - 4]) {
            case 0x04://Diapozono sumazinimo mygtuko ID
                if (diap > 0) {
                    diap = diap - 1;
                }
                break;
            case 0x03://Diapozono padidinimo mygtuko ID
                if (diap < 3) {
                    diap = diap + 1;
                }
                break;
            }

            //Diapozono perjungimas

            HAL_GPIO_WritePin(R_200_GPIO_Port, R_200_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(R_2K_GPIO_Port, R_2K_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(R_20K_GPIO_Port, R_20K_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(R_100K_GPIO_Port, R_100K_Pin, GPIO_PIN_RESET);

            switch (diap) {
            case 0:
                HAL_GPIO_WritePin(R_200_GPIO_Port, R_200_Pin, GPIO_PIN_SET);
                HAL_UART_Transmit(&huart1, cmd_200_Ohm, 16, 10);
                sendTerm();
                break;
            case 1:
                HAL_GPIO_WritePin(R_2K_GPIO_Port, R_2K_Pin, GPIO_PIN_SET);
                HAL_UART_Transmit(&huart1, cmd_2_kOhm, 15, 10);
                sendTerm();
                break;
            case 2:
                HAL_GPIO_WritePin(R_20K_GPIO_Port, R_20K_Pin, GPIO_PIN_SET);
                HAL_UART_Transmit(&huart1, cmd_20_kOhm, 16, 10);
                sendTerm();
                break;
            case 3:
                HAL_GPIO_WritePin(R_100K_GPIO_Port, R_100K_Pin, GPIO_PIN_SET);
                HAL_UART_Transmit(&huart1, cmd_100_kOhm, 17, 10);
                sendTerm();
                break;
            }

            uart_rx.head = 0;
            memset(uart_rx.buffer, 0x00, sizeof(uart_rx.buffer));
        }

        uart_rx.head = uart_rx.head + 1;
    }
}

/*
Atlieka duomenu nsukaityma, ju issiuntima i PC bei Nextion. Taip pat kontroliuoja duomenu nuskaitymo indikatoriu.
*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim) {
    if (htim == &htim7 && sample_count == SAMPLES) {

        float total = 0;
        for (int a = 0; a < sample_count; a++) {
            total += (float) ADC_samples[a] / maks[diap];
        }

        R = (total / SAMPLES) * koef[diap];

        if (R > koef[diap] + (float_t)koef[diap]/200) {//Indikuojama kai virsijama matavimo riba, papildomai pridedant 0.5 proc. 
            strcpy(buffr, "0,L\r\n");
            txt_len = 5;

        } else {
				uint8_t i = 3;//Indeksas pakeisti taska kableliu.
            char fmt[] = "%007.3f %.4s\r\n";
            switch (diap) {
            case 0:
                memcpy(mat_vnt, s_ohm, 3);//Ohm
                strcpy(fmt, "%007.3f %.3s\r\n");
								i = 3;
                break;
            case 1:
                memcpy(mat_vnt, s_kohm, 4);//kOhm
                strcpy(fmt, "%5.3f %.4s\r\n");
								i = 1;
                break;
            case 2:
                memcpy(mat_vnt, s_kohm, 4);//kOhm
                strcpy(fmt, "%06.3f %.4s\r\n");
								i = 2;
                break;
            case 3:
                memcpy(mat_vnt, s_kohm, 4);//kOhm
                strcpy(fmt, "%007.3f %.4s\r\n");
								i = 3;
                break;
            };

						R = roundf(R*1000.0f)/1000.0f;

            txt_len = snprintf(NULL, 0, fmt, R, mat_vnt);
            sprintf(buffr, fmt, R, mat_vnt);
						memset(buffr + i, ',', 1);//Pakeiciamas taskas kableliu.
        }

        sample_count = 0;
        size_t buf_len = sizeof buffr;

        //Isiunciami duomenys i personalini kompiuteri.
        HAL_UART_Transmit(&huart2, (uint8_t*)buffr, buf_len, 10);

        //Isiunciami duomenys i Nextion.
        memset(rToNextion, 0x00, sizeof(rToNextion));

        char cmd[] = "t3.txt=\"";
        size_t c_len = sizeof cmd - 1;

        memcpy(rToNextion, cmd, c_len);
        txt_len -= 2;
        memcpy(rToNextion + c_len, buffr,  txt_len);
        memcpy(rToNextion + c_len + txt_len, "\"", 1);

        HAL_UART_Transmit(&huart1, (uint8_t*)rToNextion, c_len + txt_len + 1, 10);
        sendTerm();

    } else if (htim == &htim3) {
        if (sample_count < SAMPLES) {

            //Ijungiamas matavimo indikatorius
            if (!tick_on) {
                HAL_UART_Transmit(&huart1, cmd_tick_on, 10, 10);
                sendTerm();

                tick_on = true;
            }

            //Nuskaitomi ADC duomenys
            if (ADC_Ready == 1) {
                if (HAL_ADC_Start(&hadc) != HAL_OK) {
                    Error_Handler();
                } else {
                    ADC_Ready = 0;
                }

                if (HAL_ADC_PollForConversion(&hadc, 10) == HAL_OK) {
                    if ((HAL_ADC_GetState(&hadc) & HAL_ADC_STATE_REG_EOC) == HAL_ADC_STATE_REG_EOC) {
                        ADC_samples[sample_count] = HAL_ADC_GetValue(&hadc);

                        sample_count ++;
                        ADC_Ready = 1;
                    }
                }
            }
        } else {
            //Isjungiamas matavimo indikatorius
            if (tick_on) {
                HAL_UART_Transmit(&huart1, cmd_tick_off, 8, 10);
                sendTerm();

                tick_on = false;
            }
        }
    } else if (htim == &htim2) {//Nustatomi matuoklio pradiniai parametrai

        diap = 0;
        HAL_GPIO_WritePin(R_200_GPIO_Port, R_200_Pin, GPIO_PIN_SET);
        HAL_UART_Transmit(&huart1, cmd_200_Ohm, 16, 10);
        sendTerm();

        //Taimeris isjungiamas.
        if (HAL_TIM_Base_Stop_IT(&htim2) != HAL_OK) {
            Error_Handler();
        }
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {

    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DAC_Init();
    MX_TIM7_Init();
    MX_TIM3_Init();
    MX_ADC_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
    MX_TIM2_Init();
    /* USER CODE BEGIN 2 */

// Ref. itampos nustatymas
    HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, dac_value);

//ADC kalibravimas
    if (HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED) != HAL_OK) {
        Error_Handler();
    }

//Taimeriu paleidimas
    if (HAL_TIM_Base_Start_IT(&htim7) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK) {
        Error_Handler();
    }

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    /** Configure the main internal regulator output voltage
    */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
        Error_Handler();
    }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1 | RCC_PERIPHCLK_USART2;
    PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
    PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        Error_Handler();
    }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void) {

    /* USER CODE BEGIN ADC_Init 0 */

    /* USER CODE END ADC_Init 0 */

    ADC_ChannelConfTypeDef sConfig = {0};

    /* USER CODE BEGIN ADC_Init 1 */

    /* USER CODE END ADC_Init 1 */

    /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
    */
    hadc.Instance = ADC1;
    hadc.Init.OversamplingMode = DISABLE;
    hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
    hadc.Init.Resolution = ADC_RESOLUTION_12B;
    hadc.Init.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
    hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
    hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc.Init.ContinuousConvMode = DISABLE;
    hadc.Init.DiscontinuousConvMode = DISABLE;
    hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc.Init.DMAContinuousRequests = DISABLE;
    hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
    hadc.Init.LowPowerAutoWait = DISABLE;
    hadc.Init.LowPowerFrequencyMode = DISABLE;
    hadc.Init.LowPowerAutoPowerOff = DISABLE;
    if (HAL_ADC_Init(&hadc) != HAL_OK) {
        Error_Handler();
    }

    /** Configure for the selected ADC regular channel to be converted.
    */
    sConfig.Channel = ADC_CHANNEL_6;
    sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
    if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN ADC_Init 2 */

    /* USER CODE END ADC_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void) {

    /* USER CODE BEGIN DAC_Init 0 */

    /* USER CODE END DAC_Init 0 */

    DAC_ChannelConfTypeDef sConfig = {0};

    /* USER CODE BEGIN DAC_Init 1 */

    /* USER CODE END DAC_Init 1 */

    /** DAC Initialization
    */
    hdac.Instance = DAC;
    if (HAL_DAC_Init(&hdac) != HAL_OK) {
        Error_Handler();
    }

    /** DAC channel OUT1 config
    */
    sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
    sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
    if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }

    /** DAC channel OUT2 config
    */
    if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN DAC_Init 2 */

    /* USER CODE END DAC_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void) {

    /* USER CODE BEGIN TIM2_Init 0 */

    /* USER CODE END TIM2_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* USER CODE BEGIN TIM2_Init 1 */

    /* USER CODE END TIM2_Init 1 */
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 7999;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 999;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM2_Init 2 */

    /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void) {

    /* USER CODE BEGIN TIM3_Init 0 */

    /* USER CODE END TIM3_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* USER CODE BEGIN TIM3_Init 1 */

    /* USER CODE END TIM3_Init 1 */
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 7999;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 9;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM3_Init 2 */

    /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void) {

    /* USER CODE BEGIN TIM7_Init 0 */

    /* USER CODE END TIM7_Init 0 */

    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* USER CODE BEGIN TIM7_Init 1 */

    /* USER CODE END TIM7_Init 1 */
    htim7.Instance = TIM7;
    htim7.Init.Prescaler = 7999;
    htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim7.Init.Period = 999;
    htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim7) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM7_Init 2 */

    /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void) {

    /* USER CODE BEGIN USART1_Init 0 */

    /* USER CODE END USART1_Init 0 */

    /* USER CODE BEGIN USART1_Init 1 */

    /* USER CODE END USART1_Init 1 */
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 9600;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart1) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN USART1_Init 2 */
// Enable RX interrupt
    HAL_UART_Receive_IT(&huart1, &rx_data, 10);
    /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void) {

    /* USER CODE BEGIN USART2_Init 0 */

    /* USER CODE END USART2_Init 0 */

    /* USER CODE BEGIN USART2_Init 1 */

    /* USER CODE END USART2_Init 1 */
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart2) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN USART2_Init 2 */

    /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /* USER CODE BEGIN MX_GPIO_Init_1 */
    /* USER CODE END MX_GPIO_Init_1 */

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, R_200_Pin | R_2K_Pin | R_20K_Pin | R_100K_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin : B1_Pin */
    GPIO_InitStruct.Pin = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : R_200_Pin R_2K_Pin R_20K_Pin R_100K_Pin */
    GPIO_InitStruct.Pin = R_200_Pin | R_2K_Pin | R_20K_Pin | R_100K_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USER CODE BEGIN MX_GPIO_Init_2 */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);

    /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
    }
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
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
