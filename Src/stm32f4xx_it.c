/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define CPU_MAP_STM32F407 // Load pin configuration for STM32F407
#include "cpu_map_stm32.h"
#include "grbl.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint32_t prevLIMITPORT;
static uint32_t led;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
extern void grbl_TIM2_IRQHandler();
extern void grbl_TIM3_IRQHandler();
extern void grbl_EXTI15_10_IRQHandler();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ETH_HandleTypeDef heth;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim14;
extern UART_HandleTypeDef huart2;
/* USER CODE BEGIN EV */
extern uint8_t rx_byte;
extern void processUARTByte (void);
extern volatile uint32_t ioPort;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */
  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */
  grbl_TIM2_IRQHandler();
  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */
  grbl_TIM3_IRQHandler();
  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */
  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */
  processUARTByte();
  HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles TIM8 trigger and commutation interrupts and TIM14 global interrupt.
  */
void TIM8_TRG_COM_TIM14_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_TRG_COM_TIM14_IRQn 0 */
  /* USER CODE END TIM8_TRG_COM_TIM14_IRQn 0 */
  HAL_TIM_IRQHandler(&htim14);
  /* USER CODE BEGIN TIM8_TRG_COM_TIM14_IRQn 1 */
// We patch the inputs to the correct ports for grbl and trigger interrups here
  if(HAL_GPIO_ReadPin(X_L1_GPIO_Port, X_L1_Pin)/* |
	 !HAL_GPIO_ReadPin(X_L2_GPIO_Port, X_L2_Pin) |
	 !HAL_GPIO_ReadPin(X_L3_GPIO_Port, X_L3_Pin) |
	 !HAL_GPIO_ReadPin(X_L4_GPIO_Port, X_L4_Pin)*/){
	  ioPort |= (1 << X_LIMIT_BIT);
  } else {
	  ioPort &= ~(1 << X_LIMIT_BIT);
  }
  if(HAL_GPIO_ReadPin(Y_L1_GPIO_Port, Y_L1_Pin)/* |
	 !HAL_GPIO_ReadPin(Y_L2_GPIO_Port, Y_L2_Pin) |
	 !HAL_GPIO_ReadPin(Y_L3_GPIO_Port, Y_L3_Pin) |
	 !HAL_GPIO_ReadPin(Y_L4_GPIO_Port, Y_L4_Pin)*/){
	  ioPort |= (1 << Y_LIMIT_BIT);
  } else {
	  ioPort &= ~(1 << Y_LIMIT_BIT);
  }
  if (prevLIMITPORT != ioPort){
	  prevLIMITPORT = ioPort;
	  if(((ioPort & IGNORE_LIMITS_BIT) != 0)){ // only call if irq is activated
		  grbl_EXTI15_10_IRQHandler();
	  }
  }

  // read HOME button and execute homing cycle if press is detected
  if(!HAL_GPIO_ReadPin(M_ORG_GPIO_Port, M_ORG_Pin)){
	  system_execute_line("$H");
  }

  // read Jogging commands
  if(sys.state != STATE_JOG){
	  if(!HAL_GPIO_ReadPin(X_PLUS_GPIO_Port, X_PLUS_Pin)){
		  if(HAL_GPIO_ReadPin(FAST_GPIO_Port, FAST_Pin)){
			  // we go slow
			  system_execute_line("$J=G0 X0.5 F500"); // jog 0.5mm
		  } else {
			  system_execute_line("$J=G0 X1 F500"); // jog 1mm
		  }
	  }
	  if(!HAL_GPIO_ReadPin(X_MINUS_GPIO_Port, X_MINUS_Pin)){
		  if(HAL_GPIO_ReadPin(FAST_GPIO_Port, FAST_Pin)){
			  // we go slow
			  system_execute_line("$J=G0 X-0.5 F500"); // jog 0.5mm
		  } else {
			  system_execute_line("$J=G0 X-1 F500"); // jog 1mm
		  }
	  }
	  if(!HAL_GPIO_ReadPin(Y_PLUS_GPIO_Port, Y_PLUS_Pin)){
		  if(HAL_GPIO_ReadPin(FAST_GPIO_Port, FAST_Pin)){
			  // we go slow
			  system_execute_line("$J=G0 Y0.5 F500"); // jog 0.5mm
		  } else {
			  system_execute_line("$J=G0 Y1 F500"); // jog 1mm
		  }
	  }
	  if(!HAL_GPIO_ReadPin(Y_MINUS_GPIO_Port, Y_MINUS_Pin)){
		  if(HAL_GPIO_ReadPin(FAST_GPIO_Port, FAST_Pin)){
			  // we go slow
			  system_execute_line("$J=G0 Y-0.5 F500"); // jog 0.5mm
		  } else {
			  system_execute_line("$J=G0 Y-1 F500"); // jog 1mm
		  }
	  }
  }
  // Run LED
  if(led++ == 500){
	  HAL_GPIO_TogglePin(RUN_GPIO_Port, RUN_Pin);
	  led = 0;
	  system_set_exec_state_flag(EXEC_STATUS_REPORT);
  }
  /* USER CODE END TIM8_TRG_COM_TIM14_IRQn 1 */
}

/**
  * @brief This function handles Ethernet wake-up interrupt through EXTI line 19.
  */
void ETH_WKUP_IRQHandler(void)
{
  /* USER CODE BEGIN ETH_WKUP_IRQn 0 */

  /* USER CODE END ETH_WKUP_IRQn 0 */
  HAL_ETH_IRQHandler(&heth);
  /* USER CODE BEGIN ETH_WKUP_IRQn 1 */

  /* USER CODE END ETH_WKUP_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
