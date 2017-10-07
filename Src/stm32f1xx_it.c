/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"
#include "stm32f1xx_it.h"

/* USER CODE BEGIN 0 */
#include "stdint.h"

extern Status_HandleTypeDef IO_Status;
extern timCNT_HandleTypeDef CNT;
extern Temp_HandleTypeDef Temp;
extern CNT_Sta_HandleTypeDef	
									fan_CNT_Sta,
									tube_CNT_Sta,
									
									defrost_cyc_CNT_Sta,
									defrost_CNT_Sta,
									
									drip_CNT_Sta,
									
									house_temp_H_CNT_Sta,
									house_temp_L_CNT_Sta,
									
									refri_temp_H_CNT_Sta,						
									refri_temp_L_CNT_Sta,
									
									door_open_del_CNT_Sta,
                  
                  Alarm_Blind_CNT_Sta,
                  
                  SRP_Routine_Sta,
                  Boot_CNT_Sta;
extern Alarm_Blind_HandleTypeDef CMP;                  
extern NTC_HandleTypeDef NTC;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim2;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern UART_HandleTypeDef huart1;

/******************************************************************************/
/*            Cortex-M3 Processor Interruption and Exception Handlers         */ 
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
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
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
  }
  /* USER CODE BEGIN MemoryManagement_IRQn 1 */

  /* USER CODE END MemoryManagement_IRQn 1 */
}

/**
* @brief This function handles Prefetch fault, memory access fault.
*/
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN BusFault_IRQn 1 */

  /* USER CODE END BusFault_IRQn 1 */
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
  }
  /* USER CODE BEGIN UsageFault_IRQn 1 */

  /* USER CODE END UsageFault_IRQn 1 */
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
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles DMA1 channel1 global interrupt.
*/
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
* @brief This function handles DMA1 channel5 global interrupt.
*/
void DMA1_Channel5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel5_IRQn 0 */

  /* USER CODE END DMA1_Channel5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA1_Channel5_IRQn 1 */

  /* USER CODE END DMA1_Channel5_IRQn 1 */
}

/**
* @brief This function handles ADC1 and ADC2 global interrupts.
*/
void ADC1_2_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_2_IRQn 0 */

  /* USER CODE END ADC1_2_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  /* USER CODE BEGIN ADC1_2_IRQn 1 */

  /* USER CODE END ADC1_2_IRQn 1 */
}

/**
* @brief This function handles TIM2 global interrupt.
*/
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  //**********************************
  //**********************************
  //**********************************
  //*****USER FUNCTION START HERE*****

  if(fan_CNT_Sta == running)
  {
    CNT.fan_CNT++;
    if(CNT.fan_CNT==CNT.fan_CNT_TRIG)
    {
      
      A4_L;
      CNT.fan_CNT=0;
      fan_CNT_Sta=End_Once; 
    }
  }
  
  if(defrost_cyc_CNT_Sta==running)
  {
    CNT.defrost_cyc_CNT++;
    if(CNT.defrost_cyc_CNT==CNT.defrost_cyc_CNT_TRIG)
    {
      CNT.defrost_cyc_CNT=0;
      defrost_cyc_CNT_Sta=End_Once;
      
      CNT.fan_CNT=0;
      fan_CNT_Sta=End_Once; 

      A11_H;
      defrost_CNT_Sta=running;
    }
  }
  
  if(defrost_CNT_Sta==running)
  {
    CNT.defrost_CNT++;
    if(CNT.defrost_CNT==CNT.defrost_CNT_TRIG)
    {
      CNT.defrost_CNT=0;
      defrost_CNT_Sta=End_Once;
    }
  }  

  if(drip_CNT_Sta==running)
  {
    CNT.drip_CNT++;
    if(CNT.drip_CNT==CNT.drip_CNT_TRIG)
    {
      CNT.drip_CNT=0;
      drip_CNT_Sta=End_Once;
      SRP_Routine_Sta=idle;
    }
  } 

  if(tube_CNT_Sta==running)
  {
    CNT.tube_CNT++;
    if(CNT.tube_CNT==CNT.tube_CNT_TRIG)
    {
      CNT.tube_CNT=0;
      tube_CNT_Sta=End_Once;
      A3_L;
      SRP_Routine_Sta=idle;
    }
  }  

  if(door_open_del_CNT_Sta==running)
  {
    CNT.door_open_del_CNT++;
    if(CNT.door_open_del_CNT==CNT.door_open_del_CNT_TRIG)
    {
      CNT.door_open_del_CNT=0;
      door_open_del_CNT_Sta=idle;
    }    
  } 
  
  if(Alarm_Blind_CNT_Sta==running)
  {
    CNT.cmp_alarm_blind_CNT++;
    if(CNT.cmp_alarm_blind_CNT==CNT.cmp_alarm_blind_CNT_TRIG)
    {
      CNT.cmp_alarm_blind_CNT=0;
      Alarm_Blind_CNT_Sta=idle;
    }    
  }
  
  if(Boot_CNT_Sta==running)
  {
    CNT.boot_del_CNT++;
    if(CNT.boot_del_CNT==CNT.boot_del_CNT_TRIG)
    {
      CNT.boot_del_CNT=0;
      Boot_CNT_Sta=idle;
    }    
  }
  
  if(house_temp_H_CNT_Sta==running)
  {
    CNT.house_temp_H_CNT++;
    if(CNT.house_temp_H_CNT==CNT.house_temp_H_CNT_TRIG)
    {
      CNT.house_temp_H_CNT=0;
      house_temp_H_CNT_Sta=idle;
    }    
  }

  if(house_temp_L_CNT_Sta==running)
  {
    CNT.house_temp_L_CNT++;
    if(CNT.house_temp_L_CNT==CNT.house_temp_L_CNT_TRIG)
    {
      CNT.house_temp_L_CNT=0;
      house_temp_L_CNT_Sta=idle;
    }    
  }

  if(refri_temp_H_CNT_Sta==running)
  {
    CNT.refri_temp_H_CNT++;
    if(CNT.refri_temp_H_CNT==CNT.refri_temp_H_CNT_TRIG)
    {
      CNT.refri_temp_H_CNT=0;
      refri_temp_H_CNT_Sta=idle;
    }    
  }

  if(refri_temp_L_CNT_Sta==running)
  {
    CNT.refri_temp_L_CNT++;
    if(CNT.refri_temp_L_CNT==CNT.refri_temp_L_CNT_TRIG)
    {
      CNT.refri_temp_L_CNT=0;
      refri_temp_L_CNT_Sta=idle;
    }    
  }
  //*****USER FUNCTION END HERE*******
  //**********************************
  //**********************************
  //**********************************
      
  /* USER CODE END TIM2_IRQn 1 */
}

/**
* @brief This function handles USART1 global interrupt.
*/
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
