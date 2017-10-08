/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include <stdint.h>
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

/* USER CODE BEGIN Private defines */
#define A3_H	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET)		//Tube_Cooling.....
#define A3_L	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET)	//Tube_Cooling.....
#define A4_H	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)		//Fan_Cooling.....
#define A4_L	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)	//Fan_Cooling.....
#define A5_H	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET)		//Alarm
#define A5_L	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET)	//Alarm
#define A6_H	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET)		//Refri_Fan.....
#define A6_L	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET)	//Refri_Fan.....
#define A7_H	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET)		//Refrz_Fan
#define A7_L	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET)	//Refrz_Fan
#define A8_H	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET)		//Comp.....
#define A8_L	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET)	//Comp.....
#define A11_H	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET)		//Defrost.....
#define A11_L	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET)	//Defrost.....
#define GPIO_B(num) HAL_GPIO_ReadPin(GPIOB, num)

#define closed 0
#define opened 1

#define off 0
#define on  1

#define over  0
#define lower 1
#define norm  2

#define Not_Trig 0
#define Trig 1
//Only for House_Temp_Alarm_Sta, Refri_Temp_Alarm_Sta;

#define house_temp                  0
#define house_temp_error            15
#define house_ALARM_OT_temp         30
#define house_ALARM_LT_temp         45  
#define house_ALARM_OT_temp_delay   60
#define house_ALARM_LT_temp_delay   75
#define refri_temp                  90
#define refri_temp_error            105
#define refri_ALARM_OT_temp         120
#define refri_ALARM_LT_temp         135
#define refri_ALARM_OT_temp_delay   150
#define refri_ALARM_LT_temp_delay   165
#define defrost_cyc                 180
#define defrost_timer               195
#define defrost_temp                210
#define refrez_fan_temp             225
#define fan_cooling_time            240
#define drip_time                   255
#define tube_cooling_time           270
#define boot_delay                  285
#define door_delay                  300

#define Beta_of_NTC                 315
#define EVP_Temp_offset             330
#define House_Temp_offset           345
#define Refri_Temp_offset           360
#define EVP_Percent_offset          375
#define House_Percent_offset        390
#define Refri_Percent_offset        405

typedef struct _status  // 1 = Wrong, 0 = Normal.
{
  uint8_t Door;     //??
  uint8_t _Door;    //Last status
  uint8_t HV;       //????
  uint8_t LV;       //????
  uint8_t Oil;      //????
  uint8_t Comp_OVP; //?????????
  uint8_t SRP_Swt;  //SRP
}Status_HandleTypeDef;

typedef struct _timCNT      //Unit: Minute
{
  unsigned long fan_CNT;         //??????
  unsigned long fan_CNT_TRIG;

  unsigned long tube_CNT;        //??????
  unsigned long tube_CNT_TRIG;

  unsigned long defrost_cyc_CNT; //??????????????????
  unsigned long defrost_cyc_CNT_TRIG;

  unsigned long defrost_CNT;     //????????????
  unsigned long defrost_CNT_TRIG;

  unsigned long drip_CNT;        //??????
  unsigned long drip_CNT_TRIG;

  unsigned long house_temp_H_CNT;  //?????????????
  unsigned long house_temp_H_CNT_TRIG;

  unsigned long house_temp_L_CNT;  //?????????????
  unsigned long house_temp_L_CNT_TRIG;

  unsigned long refri_temp_H_CNT;  //?????????????????????
  unsigned long refri_temp_H_CNT_TRIG;

  unsigned long refri_temp_L_CNT;  //?????????????????????
  unsigned long refri_temp_L_CNT_TRIG;
  
  unsigned long door_open_del_CNT; //???????????????????
  unsigned long door_open_del_CNT_TRIG;

  unsigned long cmp_alarm_blind_CNT;
  unsigned long cmp_alarm_blind_CNT_TRIG;

  unsigned long boot_del_CNT;
  unsigned long boot_del_CNT_TRIG;
}timCNT_HandleTypeDef;

typedef struct _temp
{
  float Read_House_Temp_flt; 
  float Read_Refri_Temp_flt;
  float Read_EVAP_Temp_flt;
  
  float house_temp_flt;
  float house_temp_error_flt;
  float house_alarm_OT_flt;
  float house_alarm_LT_flt;
  float refri_temp_flt;
  float refri_temp_error_flt;
  float refri_alarm_OT_flt;
  float refri_alarm_LT_flt;
  float defrost_temp_flt;
  float refrez_temp_flt;

  int16_t Read_House_Temp_int;
  int16_t Read_Refri_Temp_int;
  int16_t Read_EVAP_Temp_int;

  int16_t house_temp_int;
  int16_t house_temp_error_int;
  int16_t house_alarm_OT_int;
  int16_t house_alarm_LT_int;
  int16_t refri_temp_int;
  int16_t refri_temp_error_int;
  int16_t refri_alarm_OT_int;
  int16_t refri_alarm_LT_int;
  int16_t defrost_temp_int;
  int16_t refrez_temp_int;
}Temp_HandleTypeDef;

//Counter condition
typedef enum _com_CNT_status
{
  running,
  idle,

  //For fan_CNT
  End_Once,
  ending,

  //For SRP
  running_Fan,
  running_Tube,
}CNT_Sta_HandleTypeDef;

typedef struct _cmp_last_sta
{
  uint8_t _cmpsta;
}Alarm_Blind_HandleTypeDef;

typedef struct _NTC_info
{
  uint32_t Beta;                  //used
  float EVP_Temp_offset_flt;      //used
  float House_Temp_offset_flt;    //used
  float Refri_Temp_offset_flt;    //used
  float EVP_Percent_offset_flt;   //used
  float House_Percent_offset_flt; //used
  float Refri_Percent_offset_flt; //used

  int16_t EVP_Temp_offset_int;
  int16_t House_Temp_offset_int;
  int16_t Refri_Temp_offset_int;

}NTC_HandleTypeDef;

/* USER CODE END Private defines */

void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
