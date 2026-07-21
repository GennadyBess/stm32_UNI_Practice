/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */


extern uint8_t Set_Target_Voltage(float target_voltage);
extern uint8_t Set_Target_Current(float target_current);

typedef struct {
    uint32_t integer;
    uint32_t decimal;
    float whole_num;
    uint8_t sign;
    uint8_t type;
} Measurement_t;
extern Measurement_t voltage;
extern Measurement_t current;

typedef struct {
	uint32_t magic_number;
    float k_volt;
    float b_volt;
    float k_curr;
    float b_curr;

    float volt_Kp;
    float volt_Ki;
    float volt_Kd;

    float curr_Kp;
    float curr_Ki;
    float curr_Kd;
} calibration_settings_t;
extern calibration_settings_t settings;

extern volatile uint16_t debug_interval;
extern volatile uint8_t debug_enabled;
extern volatile uint8_t voltage_regulation_enabled;
extern volatile uint8_t current_regulation_enabled;
extern volatile uint8_t get_status_enabled;

extern float k_voltage;
extern float b_voltage;
extern float k_current;
extern float b_current;

extern float v_Kp;
extern float v_Ki;
extern float v_Kd;

extern float c_Kp;
extern float c_Ki;
extern float c_Kd;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define POT_SCK_Pin GPIO_PIN_13
#define POT_SCK_GPIO_Port GPIOB
#define POT_CS_Pin GPIO_PIN_14
#define POT_CS_GPIO_Port GPIOB
#define POT_MOSI_Pin GPIO_PIN_15
#define POT_MOSI_GPIO_Port GPIOB
#define CONV_EN_Pin GPIO_PIN_8
#define CONV_EN_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
