/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include <stdio.h>
#include "cli.h"
#include "utils.h"
#include "mcp41010.h"
#include "usbd_cdc_if.h"
#include "ee.h"
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
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void Read_ADC(
		Measurement_t *result,
		ADC_HandleTypeDef *hadc,
		uint8_t samples,
		float multiplier,
		float divider
		);

void status_print();
void voltage_regulation();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile uint8_t voltage_regulation_enabled = 0;
volatile uint8_t current_regulation_enabled = 0;
volatile uint8_t debug_enabled = 0;
volatile uint8_t status_output_enabled = 0;
volatile uint8_t get_status_enabled = 0;
volatile uint8_t measure_enabled = 0;

volatile float target_voltage = 12.5;
volatile float target_current = 1.0f;
static float integral_err_curr = 0.0f;
static float integral_err_vol= 0.0f;
static float base_pot_value = 128.0f;
static float base_voltage_feedforward = 0.0f;
void voltage_regulation_pi(int16_t p_feedforward, float t_target);
void current_regulation_pi(float v_feedforward, float t_target);


volatile uint16_t system_ms_ticks = 0;
volatile uint16_t debug_interval = 1000;

float k_voltage = 1;
float b_voltage = 0;

float k_current = 1;
float b_current = 0;

Measurement_t voltage = { .type = 0};
Measurement_t current = { .type = 1};
calibration_settings_t settings;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

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
  MX_ADC1_Init();
  MX_USB_DEVICE_Init();
  MX_ADC2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADCEx_Calibration_Start(&hadc2);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_GPIO_WritePin(CONV_EN_GPIO_Port, CONV_EN_Pin, GPIO_PIN_SET);
  MCP41010_SetValue(255);

  EE_Init(&settings, sizeof(calibration_settings_t));
  EE_Read();

  if (settings.magic_number != 0xCAFEBABE) {
      settings.k_volt = 1.0f;
      settings.b_volt = 0.0f;
      settings.k_curr = 1.0f;
      settings.b_curr = 0.0f;
      settings.magic_number = 0xCAFEBABE;
      EE_Write();
  }

  k_voltage = settings.k_volt;
  b_voltage = settings.b_volt;
  k_current = settings.k_curr;
  b_current = settings.b_curr;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  	if (measure_enabled)
	  	{
	  		measure_enabled = 0;
	  		Read_ADC(&voltage, &hadc1, 32, (180.0f + 15.0f), 15.0f);
	  		Read_ADC(&current, &hadc2, 32, 1.0f, 200.0f * 0.005f);
	  		if (voltage_regulation_enabled) {
	  			voltage_regulation_pi(-1, -1.0f);
			}
	  		else if (current_regulation_enabled) {
	  			current_regulation_pi(-1.0f, -1.0f);
	  		}
	  	}

		if (status_output_enabled && debug_enabled)
		{
			status_output_enabled = 0;
			status_print();
		}

		if (get_status_enabled)
		{
			get_status_enabled = 0;
			status_print();
		}


		if (cmd_ready) {
			cmd_ready = 0;
			process_command(cmd_buffer);
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 47999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, POT_SCK_Pin|POT_MOSI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(POT_CS_GPIO_Port, POT_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CONV_EN_Pin|GPIO_PIN_15, GPIO_PIN_SET);

  /*Configure GPIO pins : POT_SCK_Pin POT_CS_Pin POT_MOSI_Pin */
  GPIO_InitStruct.Pin = POT_SCK_Pin|POT_CS_Pin|POT_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : CONV_EN_Pin PA15 */
  GPIO_InitStruct.Pin = CONV_EN_Pin|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}



/* USER CODE BEGIN 4 */
/**
 * @brief  Обработчик прерывания по переполнению таймера TIM3.
 * @details Вызывается аппаратно с базовым интервалом в 1 мс.
 * @param[in] htim Указатель на базовую структуру таймера HAL.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM3)
	{
		system_ms_ticks++;

		if ((system_ms_ticks % 50) == 0)
		{
			measure_enabled = 1;
		}

		if (system_ms_ticks >= debug_interval)
		{
			status_output_enabled = 1;

			system_ms_ticks = 0;
		}
	}
}

/**
 * @brief  Чтение, усреднение, и калибровка данных с АЦП.
 * @param[out] result     Указатель на структуру Measurement_t для записи отформатированного результата.
 * @param[in]  hadc       Указатель на дескриптор используемого АЦП HAL.
 * @param[in]  samples    Количество выборок для усреднения (размер окна оверсемплинга).
 * @param[in]  multiplier Множитель входного аппаратного делителя/усилителя.
 * @param[in]  divider    Делитель входного аппаратного делителя/усилителя.
 */
void Read_ADC(
		Measurement_t *result,
		ADC_HandleTypeDef *hadc,
		uint8_t samples,
		float multiplier,
		float divider
		)
{
    uint32_t sum = 0;
    for (uint8_t i = 0; i < samples; i++)
    {
        HAL_ADC_Start(hadc);
        HAL_ADC_PollForConversion(hadc, 10);
        sum += HAL_ADC_GetValue(hadc);
        HAL_ADC_Stop(hadc);
    }

    float raw_avg = (float)sum / samples;
    float v_adc = (raw_avg * 3.3f) / 4095.0f;
    float value = v_adc * multiplier / divider;

    if (result->type == 0) {
    	value = k_voltage * value + b_voltage;
    } else {
    	value = k_current * value + b_current;
    }
    result->whole_num = value;

    float value_abs = value;
    if (value < 0.0f) {
    	result->sign = 1;
    	value_abs = -value;
    } else {
    	result->sign = 0;
    }

    result->integer = (uint32_t)value_abs;
	result->decimal = (uint32_t)((value_abs - result->integer) * PRECISION);
}

/**
 * @brief  Форматирование и отправка текущего системного времени и параметров.
 * @param  None
 */
void status_print()
{
	uint32_t uptime_ms = HAL_GetTick();
	uint32_t minutes = (uptime_ms / 1000) / 60 % 60;
	uint32_t seconds = (uptime_ms / 1000) % 60;
	uint32_t ms = uptime_ms % 1000;

	char tx_buffer[64];
	int p = 0;

	tx_buffer[p++] = '[';
	p += custom_utoa(minutes, &tx_buffer[p], 2, '0');
	tx_buffer[p++] = ':';
	p += custom_utoa(seconds, &tx_buffer[p], 2, '0');
	tx_buffer[p++] = '.';
	p += custom_utoa(ms,      &tx_buffer[p], 3, '0');
	tx_buffer[p++] = ']';
	tx_buffer[p++] = '\n';
	tx_buffer[p++] = '\r';

	p = append_sensor_data(tx_buffer, p, "Voltage", voltage.sign, voltage.integer, voltage.decimal, 'V');

	p = append_sensor_data(tx_buffer, p, "Current", current.sign, current.integer, current.decimal, 'A');

	CDC_Transmit_FS((uint8_t*)tx_buffer, p);
}

/**
 * @brief  Выполнение одного шага PI-регулирования напряжения.
 * @details Функция опрашивается в главном цикле по флагу готовности измерений (каждые 50 мс).
 * Реализует формулу Feedforward + PI с защитой интегратора от насыщения (Anti-windup clamping).
 * * @param[in] p_feedforward Расчетное стартовое положение потенциометра (feedforward от модели).
 * Если передано < 0, используется текущее базовое значение.
 * @param[in] t_target      Целевое напряжение. Если < 0, используется текущее целевое.
 */
void voltage_regulation_pi(int16_t p_feedforward, float t_target) {
    if (p_feedforward >= 0) {
        base_pot_value = (float)p_feedforward;
    }
    if (t_target >= 0.0f) {
    	float diff = t_target - target_voltage;
    	if (diff < 0.0f) diff = -diff;
        if (diff > 0.5f) integral_err_vol= 0.0f;
        target_voltage = t_target;
    }

    float error = target_voltage - voltage.whole_num;

    float p_term = 12 * error;

    float next_integral = integral_err_vol+ error * 0.05;

    float i_term = 5 * next_integral;
    float control_action = p_term + i_term;

    // Результирующее значение потенциометра (обратная зависимость: меньше код -> больше вольт)
    float raw_pot = base_pot_value - control_action;

    if (raw_pot >= 0.0f && raw_pot <= 255.0f) {
        integral_err_vol= next_integral;
    } else {
        if (raw_pot < 0.0f) raw_pot = 0.0f;
        if (raw_pot > 255.0f) raw_pot = 255.0f;
    }

    uint8_t final_pot = (uint8_t)raw_pot;
    MCP41010_SetValue(final_pot);
}
/**
 * @brief  Установка целевого напряжения.
 * @param[in] target_voltage Желаемое выходное напряжение в Вольтах.
 * @retval 1 Целевое напряжение валидно и установлено.
 * @retval 0 Заданное значение выходит за аппаратные границы схемы регулирования.
 */
uint8_t Set_Target_Voltage(float trg_voltage) {
	float load = 1.25f * 180000.0f / (trg_voltage-1.25f) - 10000.0f;

	if (load < 0.0f) {
		return 0;
	} else if (load > 10000.0f) {
		return 0;
	} else {
		int16_t pot = (int16_t)((load) * 256.0f / 10000.0f);
		pot = pot - 2;
		if (pot < 0) pot = 0;
		if (pot > 255) pot = 255;

		voltage_regulation_pi(pot, trg_voltage);
		voltage_regulation_enabled = 1;
		current_regulation_enabled = 0;
		return 1;
	}
}

/**
 * @brief  Выполнение одного шага PI-регулирования тока.
 * @details Рассчитывает необходимую коррекцию напряжения на основе ошибки по току,
 * объединяет её с расчетным напряжением модели (Feedforward) и устанавливает
 * новое целевое напряжение для аппаратного регулятора.
 * * @param[in] v_feedforward Расчетное напряжение модели (V_out2). Если < 0, берется старое значение.
 * @param[in] t_target      Целевой ток (А). Если < 0, используется текущий целевой ток.
 */
void current_regulation_pi(float v_feedforward, float t_target) {
    if (v_feedforward >= 0.0f) {
        base_voltage_feedforward = v_feedforward;
    }

    if (t_target >= 0.0f) {
        float diff = t_target - target_current;
        if (diff < 0.0f) diff = -diff;
        if (diff > 0.05f) integral_err_curr = 0.0f;

        target_current = t_target;
    }

    float error = target_current - current.whole_num;

    float p_term = 4.5 * error;

    float next_integral = integral_err_curr + error * 0.05;
    float i_term = 6 * next_integral;

    float voltage_correction = p_term + i_term;

    float v_result = base_voltage_feedforward + voltage_correction;

    if (v_result >= 12.25f && v_result <= 24.0f) {
        integral_err_curr = next_integral;
    } else {
        if (v_result < 12.25f) v_result = 12.25f;
        if (v_result > 24.0f) v_result = 24.0f;
    }

    float load = 1.25f * 180000.0f / (v_result - 1.25f) - 10000.0f;
	int16_t pot;
	if (load < 0.0f) {
		pot = 0;
	} else if (load > 10000.0f) {
		pot = 255;
	} else {
		pot = (int16_t)((load) * 256.0f / 10000.0f) - 2;
		if (pot < 0) pot = 0;
		if (pot > 255) pot = 255;
	}

	MCP41010_SetValue((uint8_t)pot);
}

/**
 * @brief  Установка целевого значения тока.
 * @param[in] target_current Желаемый выходной ток в Амперах.
 * @retval 1 Расчет успешен.
 * @retval 0 Ошибка расчета.
 */
uint8_t Set_Target_Current(float trg_current) {
	float I_out1 = current.whole_num;
	float V_out1 = voltage.whole_num;

	if (I_out1 <= 0.001f) {
		return 0;
	}

	float R_load = V_out1 / I_out1;
	float V_out2 = trg_current * R_load;

    float load = 1.25f * 180000.0f / (V_out2 - 1.25f) - 10000.0f;
	if (load < 0.0f) return 0;
	else if (load > 10000.0f) return 0;

	current_regulation_pi(V_out2, trg_current);
	voltage_regulation_enabled = 0;
	current_regulation_enabled = 1;
	return 1;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
