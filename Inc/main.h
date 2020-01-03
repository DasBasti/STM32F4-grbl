/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

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
#define DATA_2_Pin GPIO_PIN_3
#define DATA_2_GPIO_Port GPIOE
#define DATA_1_Pin GPIO_PIN_4
#define DATA_1_GPIO_Port GPIOE
#define STROBE_Pin GPIO_PIN_5
#define STROBE_GPIO_Port GPIOE
#define DATA_4_Pin GPIO_PIN_6
#define DATA_4_GPIO_Port GPIOE
#define SPOT_Pin GPIO_PIN_13
#define SPOT_GPIO_Port GPIOC
#define PAT_L_Pin GPIO_PIN_14
#define PAT_L_GPIO_Port GPIOC
#define HEAD_0_8_Pin GPIO_PIN_15
#define HEAD_0_8_GPIO_Port GPIOC
#define VAC_0_8_Pin GPIO_PIN_0
#define VAC_0_8_GPIO_Port GPIOF
#define STOPPER_Pin GPIO_PIN_1
#define STOPPER_GPIO_Port GPIOF
#define CVY_M_Pin GPIO_PIN_2
#define CVY_M_GPIO_Port GPIOF
#define SUPPORTER_Pin GPIO_PIN_3
#define SUPPORTER_GPIO_Port GPIOF
#define LOCATOR_Pin GPIO_PIN_4
#define LOCATOR_GPIO_Port GPIOF
#define SP_A_Pin GPIO_PIN_5
#define SP_A_GPIO_Port GPIOF
#define SP_B_Pin GPIO_PIN_6
#define SP_B_GPIO_Port GPIOF
#define SP_C_Pin GPIO_PIN_7
#define SP_C_GPIO_Port GPIOF
#define SP_D_Pin GPIO_PIN_8
#define SP_D_GPIO_Port GPIOF
#define SP_1_Pin GPIO_PIN_9
#define SP_1_GPIO_Port GPIOF
#define SP_2_Pin GPIO_PIN_10
#define SP_2_GPIO_Port GPIOF
#define SP_3_Pin GPIO_PIN_0
#define SP_3_GPIO_Port GPIOC
#define SP_4_Pin GPIO_PIN_5
#define SP_4_GPIO_Port GPIOA
#define SP_5_Pin GPIO_PIN_6
#define SP_5_GPIO_Port GPIOA
#define BAD_MARK_Pin GPIO_PIN_2
#define BAD_MARK_GPIO_Port GPIOB
#define SP_6_Pin GPIO_PIN_11
#define SP_6_GPIO_Port GPIOF
#define SP_7_Pin GPIO_PIN_12
#define SP_7_GPIO_Port GPIOF
#define SP_8_Pin GPIO_PIN_13
#define SP_8_GPIO_Port GPIOF
#define PIN_UP_Pin GPIO_PIN_14
#define PIN_UP_GPIO_Port GPIOF
#define PCB_OUT_Pin GPIO_PIN_15
#define PCB_OUT_GPIO_Port GPIOF
#define PCB_IN_Pin GPIO_PIN_0
#define PCB_IN_GPIO_Port GPIOG
#define PCB_DET_Pin GPIO_PIN_1
#define PCB_DET_GPIO_Port GPIOG
#define CCW_SW_Pin GPIO_PIN_7
#define CCW_SW_GPIO_Port GPIOE
#define CW_SW_Pin GPIO_PIN_8
#define CW_SW_GPIO_Port GPIOE
#define M_CENTER_Pin GPIO_PIN_9
#define M_CENTER_GPIO_Port GPIOE
#define Y_HM_Pin GPIO_PIN_10
#define Y_HM_GPIO_Port GPIOE
#define Y_L4_Pin GPIO_PIN_11
#define Y_L4_GPIO_Port GPIOE
#define Y_L3_Pin GPIO_PIN_12
#define Y_L3_GPIO_Port GPIOE
#define Y_L2_Pin GPIO_PIN_13
#define Y_L2_GPIO_Port GPIOE
#define Y_L1_Pin GPIO_PIN_14
#define Y_L1_GPIO_Port GPIOE
#define Y_CW_Pin GPIO_PIN_15
#define Y_CW_GPIO_Port GPIOE
#define OUTP_nEN_Pin GPIO_PIN_10
#define OUTP_nEN_GPIO_Port GPIOB
#define Y_CCW_Pin GPIO_PIN_14
#define Y_CCW_GPIO_Port GPIOB
#define X_CW_Pin GPIO_PIN_15
#define X_CW_GPIO_Port GPIOB
#define X_CCW_Pin GPIO_PIN_8
#define X_CCW_GPIO_Port GPIOD
#define X_L1_Pin GPIO_PIN_9
#define X_L1_GPIO_Port GPIOD
#define X_L2_Pin GPIO_PIN_10
#define X_L2_GPIO_Port GPIOD
#define X_L3_Pin GPIO_PIN_11
#define X_L3_GPIO_Port GPIOD
#define X_L4_Pin GPIO_PIN_12
#define X_L4_GPIO_Port GPIOD
#define X_HM_Pin GPIO_PIN_13
#define X_HM_GPIO_Port GPIOD
#define T_HEAD_Pin GPIO_PIN_14
#define T_HEAD_GPIO_Port GPIOD
#define T_VAC_Pin GPIO_PIN_15
#define T_VAC_GPIO_Port GPIOD
#define TEACH_Pin GPIO_PIN_2
#define TEACH_GPIO_Port GPIOG
#define FAST_Pin GPIO_PIN_3
#define FAST_GPIO_Port GPIOG
#define Y_MINUS_Pin GPIO_PIN_4
#define Y_MINUS_GPIO_Port GPIOG
#define Y_PLUS_Pin GPIO_PIN_5
#define Y_PLUS_GPIO_Port GPIOG
#define X_MINUS_Pin GPIO_PIN_6
#define X_MINUS_GPIO_Port GPIOG
#define X_PLUS_Pin GPIO_PIN_7
#define X_PLUS_GPIO_Port GPIOG
#define DEG_90_Pin GPIO_PIN_8
#define DEG_90_GPIO_Port GPIOG
#define VCC_SENS_Pin GPIO_PIN_6
#define VCC_SENS_GPIO_Port GPIOC
#define HEAD_IN_Pin GPIO_PIN_7
#define HEAD_IN_GPIO_Port GPIOC
#define D_END_Pin GPIO_PIN_8
#define D_END_GPIO_Port GPIOC
#define READY_IN_Pin GPIO_PIN_9
#define READY_IN_GPIO_Port GPIOC
#define SP_10_Pin GPIO_PIN_8
#define SP_10_GPIO_Port GPIOA
#define SP_9_Pin GPIO_PIN_9
#define SP_9_GPIO_Port GPIOA
#define M_ORG_Pin GPIO_PIN_10
#define M_ORG_GPIO_Port GPIOA
#define D_HEAD_Pin GPIO_PIN_11
#define D_HEAD_GPIO_Port GPIOA
#define T_KNOCK_Pin GPIO_PIN_12
#define T_KNOCK_GPIO_Port GPIOA
#define SPI3_CS_Pin GPIO_PIN_15
#define SPI3_CS_GPIO_Port GPIOA
#define READY_OUT_Pin GPIO_PIN_0
#define READY_OUT_GPIO_Port GPIOD
#define D_START_Pin GPIO_PIN_1
#define D_START_GPIO_Port GPIOD
#define VAC_Pin GPIO_PIN_2
#define VAC_GPIO_Port GPIOD
#define HEAD_Pin GPIO_PIN_3
#define HEAD_GPIO_Port GPIOD
#define ROT_Pin GPIO_PIN_4
#define ROT_GPIO_Port GPIOD
#define CENTERING_Pin GPIO_PIN_7
#define CENTERING_GPIO_Port GPIOD
#define SP_G_Pin GPIO_PIN_9
#define SP_G_GPIO_Port GPIOG
#define SP_F_Pin GPIO_PIN_10
#define SP_F_GPIO_Port GPIOG
#define SP_E_Pin GPIO_PIN_11
#define SP_E_GPIO_Port GPIOG
#define SENSORDRV_Pin GPIO_PIN_12
#define SENSORDRV_GPIO_Port GPIOG
#define V_CNG_Pin GPIO_PIN_13
#define V_CNG_GPIO_Port GPIOG
#define A_CENT_Pin GPIO_PIN_14
#define A_CENT_GPIO_Port GPIOG
#define PULSE_Pin GPIO_PIN_15
#define PULSE_GPIO_Port GPIOG
#define CW_CCW_Pin GPIO_PIN_3
#define CW_CCW_GPIO_Port GPIOB
#define STP2_nEN_Pin GPIO_PIN_4
#define STP2_nEN_GPIO_Port GPIOB
#define STP2_nSLP_Pin GPIO_PIN_5
#define STP2_nSLP_GPIO_Port GPIOB
#define STP2_DIR_Pin GPIO_PIN_6
#define STP2_DIR_GPIO_Port GPIOB
#define STP1_DIR_Pin GPIO_PIN_7
#define STP1_DIR_GPIO_Port GPIOB
#define STP2_STP_Pin GPIO_PIN_8
#define STP2_STP_GPIO_Port GPIOB
#define STP1_STP_Pin GPIO_PIN_9
#define STP1_STP_GPIO_Port GPIOB
#define STP1_nEN_Pin GPIO_PIN_0
#define STP1_nEN_GPIO_Port GPIOE
#define STP1_nSLP_Pin GPIO_PIN_1
#define STP1_nSLP_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
