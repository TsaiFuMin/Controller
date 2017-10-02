/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
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
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint32_t sec_TICK;

Status_HandleTypeDef IO_Status;
timCNT_HandleTypeDef CNT;
Temp_HandleTypeDef Temp;
CNT_Sta_HandleTypeDef	
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

                  SRP_Routine_Sta;
Alarm_Blind_HandleTypeDef CMP;	
unsigned char _rept[5]="rept ";
unsigned char _null[3]={255,255,255};
unsigned char _leng[2]=",5";

uint32_t ADC_Buf[6];

#define cmd_rept HAL_UART_Transmit(&huart1, _rept, 5, 0xff)
#define cmd_leng HAL_UART_Transmit(&huart1, _leng, 2, 0xff)
#define cmd_end HAL_UART_Transmit(&huart1, _null, 3, 0xff)
                                                    
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void IO_CHK_Routine(void);
void CMP_CHK_Routine(void); //Always run after "IO_CHK_Routine()"
void Alarm_CHK_Routine(void);
void Refri_CHK_Routine(void);
void SRP_CHK_Routine(void);
void Get_Data(uint16_t addr);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();

  /* USER CODE BEGIN 2 */
  door_open_del_CNT_Sta=idle;

  SRP_Routine_Sta=idle; //Reset SRP switch
	
	fan_CNT_Sta=idle;
  defrost_cyc_CNT_Sta=idle;
  defrost_CNT_Sta=idle;
  drip_CNT_Sta=idle;
	
	tube_CNT_Sta=idle;
	
  Alarm_Blind_CNT_Sta=idle;
  /****************Reserved for boot delay**********************/

  HAL_TIM_Base_Start_IT(&htim2);
	HAL_ADC_Start_DMA(&hadc1, ADC_Buf, 6);
	HAL_ADC_Start(&hadc1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

    
		
    IO_CHK_Routine();       //DO NOT EDIT

    CMP_CHK_Routine();

    Refri_CHK_Routine();

    SRP_CHK_Routine();

	
  /* USER CODE END 3 */
	}
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 2828;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2828;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6 
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA3 PA4 PA5 PA6 
                           PA7 PA8 PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6 
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB5 PB6 PB7 
                           PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7 
                          |GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void IO_CHK_Routine(void){

  IO_Status.SRP_Swt=GPIO_B(GPIO_PIN_5);     //SRP ?��?��
  IO_Status.Comp_OVP=GPIO_B(GPIO_PIN_6);    //??��?? ?��?�� 
  IO_Status.LV=GPIO_B(GPIO_PIN_7);          //低�?? ?��?��
  IO_Status.HV=GPIO_B(GPIO_PIN_8);          //高�?? ?��?��
  IO_Status.Oil=GPIO_B(GPIO_PIN_9);         //油�?? ?��?��
  IO_Status.Door=GPIO_B(GPIO_PIN_10);       //?? ?��?��****

  if(IO_Status.Door==1){
    if(IO_Status._Door==opened){
      //Before counter start, read the data from Panel.
      //*****Reserve for read the data*****
      //
      Get_Data(door_delay);
      CNT.door_open_del_CNT=0;
      door_open_del_CNT_Sta=running;
    }
    IO_Status._Door=closed;
  }
  else if(IO_Status.Door==0){
    IO_Status._Door=opened;
  }

}

void CMP_CHK_Routine(void){
  Get_Data(house_temp);
  Get_Data(house_temp_error);

  uint8_t alarm_sta;
  alarm_sta=IO_Status.Comp_OVP&&IO_Status.LV&&IO_Status.HV&&IO_Status.Oil;

  //Temporary
  Temp.Read_House_Temp_int=0;
  static uint8_t house_temp_cmp_sta;
  if(Temp.Read_House_Temp_int<Temp.house_temp_int){
    house_temp_cmp_sta=0; //0 = turn off comp
  }else if(Temp.Read_House_Temp_int>=(Temp.house_temp_int+Temp.house_temp_error_int)){
    house_temp_cmp_sta=1; //1 = turn on comp
  }

	//SRP sta CHK
  uint8_t defrost_drip_cmp_sta;
  if((defrost_CNT_Sta==running)||(defrost_CNT_Sta==End_Once)||(drip_CNT_Sta==running)||(drip_CNT_Sta==End_Once)){
    defrost_drip_cmp_sta=0; //0 = turn off comp
  }else{
    defrost_drip_cmp_sta=1; //1 = turn on comp
  }
	
	
  if(   (alarm_sta||(Alarm_Blind_CNT_Sta==running))	&& \
        house_temp_cmp_sta                       		&& \
        door_open_del_CNT_Sta==idle              		&& \
        defrost_drip_cmp_sta  ){
    if(CMP._cmpsta==off){
      CNT.cmp_alarm_blind_CNT_TRIG=5; //5 seconds
      Alarm_Blind_CNT_Sta=running;
      A8_H;
      CMP._cmpsta=on;
    }

  }else{
    A8_L;
    CMP._cmpsta=off;
  }

}



void Refri_CHK_Routine(void){

  Get_Data(refri_temp);
  Get_Data(refri_temp_error);
  if(Temp.Read_Refri_Temp_int >= Temp.refri_temp_int){
    A6_H;
  }else if(Temp.Read_Refri_Temp_int < Temp.refri_temp_error_int){
    A6_L;
  }

}

void SRP_CHK_Routine(void){

  if(SRP_Routine_Sta==idle){

    if(GPIO_B(GPIO_PIN_5)){        //Y
      CNT.fan_CNT=0;
      CNT.defrost_cyc_CNT=0;
      CNT.defrost_CNT=0;
      CNT.drip_CNT=0;

      fan_CNT_Sta=idle;
      defrost_cyc_CNT_Sta=idle;
      defrost_CNT_Sta=idle;
      drip_CNT_Sta=idle;

      SRP_Routine_Sta=running_Fan;

      Get_Data(refrez_fan_temp);    //Get refrez fan temp
      Get_Data(defrost_temp);       //Get defrost temp

      Get_Data(fan_cooling_time);  //Read trigger data from the panel
      Get_Data(defrost_cyc);
      Get_Data(defrost_timer);
      Get_Data(drip_time);


			HAL_Delay(100);
			
      defrost_cyc_CNT_Sta=running;  //Start timer counter
      fan_CNT_Sta=running;

    }else if(!GPIO_B(GPIO_PIN_5)){ //N
      CNT.tube_CNT=0;

      tube_CNT_Sta=idle;

      SRP_Routine_Sta=running_Tube;
      
      Get_Data(tube_cooling_time);
      
      A3_H;
      tube_CNT_Sta=running; 
    }
  }

  if(SRP_Routine_Sta==running_Fan){
    if((fan_CNT_Sta==running||defrost_cyc_CNT_Sta!=End_Once)&&(!(Temp.Read_EVAP_Temp_int>=Temp.refrez_temp_int))){
      A4_H; //風冷??��??
            //If fan timer counter running
            //Or defrost cycle counter did not complete once.
    }else{
      A4_L;
    } 

    if(defrost_CNT_Sta==End_Once||Temp.Read_EVAP_Temp_int>=Temp.defrost_temp_int){
      A11_L;
      
      drip_CNT_Sta=running;
    }    
  }
  
}

void Get_Data(uint16_t addr){
  
  uint8_t len;
  if(addr<10){
    len=1;
  }else if(addr>10&&addr<100){
    len=2;
  }else if(addr>=100){
    len=3;
  }

  uint8_t addr_buf[5];
	sprintf((char*)addr_buf,"%d",addr);
  
	cmd_rept;
  HAL_UART_Transmit(&huart1, addr_buf, len, 0xff);
	cmd_leng;
  cmd_end;
	
  uint8_t receive_buf[5];
  HAL_UART_Receive(&huart1, receive_buf,15, 0xff);
  
  if(addr==house_temp){
    Temp.house_temp_flt=atof((char*)receive_buf);
    Temp.house_temp_int=(int16_t)(Temp.house_temp_flt*(float)10.0);
  }else if(addr==house_temp_error){
    Temp.house_temp_error_flt=atof((char*)receive_buf);
    Temp.house_temp_error_int=(int16_t)(Temp.house_temp_error_flt*(float)10.0);
  }else if(addr==house_ALARM_OT_temp){
    Temp.house_alarm_OT_flt=atof((char*)receive_buf);
    Temp.house_alarm_OT_int=(int16_t)(Temp.house_alarm_OT_flt*(float)10.0);
  }else if(addr==house_ALARM_LT_temp){
    Temp.house_alarm_LT_flt=atof((char*)receive_buf);
    Temp.house_alarm_LT_int=(int16_t)(Temp.house_alarm_LT_flt*(float)10.0);
  }else if(addr==house_ALARM_OT_temp_delay){
    CNT.house_temp_H_CNT_TRIG=atol((char*)receive_buf)*60;
  }else if(addr==house_ALARM_LT_temp_delay){
    CNT.house_temp_L_CNT_TRIG=atol((char*)receive_buf)*60;
  }else if(addr==refri_temp){
    Temp.refri_temp_flt=atof((char*)receive_buf);
    Temp.refri_temp_int=(int16_t)(Temp.refri_temp_flt*(float)10.0);
  }else if(addr==refri_temp_error){
    Temp.refri_temp_error_flt=atof((char*)receive_buf);
    Temp.refri_temp_error_int=(int16_t)(Temp.refri_temp_error_flt*(float)10.0);
  }else if(addr==refri_ALARM_OT_temp){
    Temp.refri_alarm_OT_flt=atof((char*)receive_buf);
    Temp.refri_alarm_OT_int=(int16_t)(Temp.refri_alarm_OT_flt*(float)10.0);
  }else if(addr==refri_ALARM_LT_temp){
    Temp.refri_alarm_LT_flt=atof((char*)receive_buf);
    Temp.refri_alarm_LT_int=(int16_t)(Temp.refri_alarm_LT_flt*(float)10.0);
  }else if(addr==refri_ALARM_OT_temp_delay){
    CNT.refri_temp_H_CNT_TRIG=atol((char*)receive_buf)*60;
  }else if(addr==refri_ALARM_LT_temp_delay){
    CNT.refri_temp_L_CNT_TRIG=atol((char*)receive_buf)*60;
  }else if(addr==defrost_cyc){
    CNT.defrost_cyc_CNT_TRIG=atol((char*)receive_buf)*60;
  }else if(addr==defrost_timer){
    CNT.defrost_CNT_TRIG=atol((char*)receive_buf)*60;
  }else if(addr==defrost_temp){
    Temp.defrost_temp_flt=atof((char*)receive_buf);
    Temp.defrost_temp_int=(int16_t)(Temp.defrost_temp_flt*(float)10.0);
  }else if(addr==refrez_fan_temp){
    Temp.refrez_temp_flt=atof((char*)receive_buf);
    Temp.refrez_temp_int=(int16_t)(Temp.refrez_temp_flt*(float)10.0);
  }else if(addr==fan_cooling_time){
    CNT.fan_CNT_TRIG=atol((char*)receive_buf)*60;
  }else if(addr==drip_time){
    CNT.drip_CNT_TRIG=atol((char*)receive_buf)*60;
  }else if(addr==tube_cooling_time){
    CNT.tube_CNT_TRIG=atol((char*)receive_buf)*60;
  }else if(addr==boot_delay){
    
  }else if(addr==door_delay){
    CNT.door_open_del_CNT_TRIG=atol((char*)receive_buf)*60;
  }

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hadc);
  /* NOTE : This function should not be modified. When the callback is needed,
            function HAL_ADC_ConvCpltCallback must be implemented in the user file.
   */
  if(hadc->Instance==hadc1.Instance){
    uint32_t ch[6];
    ch[0]=ADC_Buf[0];
    ch[1]=ADC_Buf[1];
    ch[2]=ADC_Buf[2];
		ch[3]=ADC_Buf[3];
		ch[4]=ADC_Buf[4];
		ch[5]=ADC_Buf[5];
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
