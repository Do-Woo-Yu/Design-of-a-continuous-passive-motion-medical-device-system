/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "extern.h"
#include "user_define.h"
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "stdio.h"
#include <stdbool.h>
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
byte Rcv2_cnt = 0;
byte test_index = 0, LoRa_Rxbuf[255], test_rxbuf[255];
byte  LoRa_test_buf[255] = {"Motor_Driver_tx_Success"};
byte Tx2_buf[100] = {0};
byte Tx2_index=0, Tx2_send_number = 0;

byte lora_tx_EN = 0, LoRa_Equipment_Num = 0;
/*****************************************추가한 부분(도우)*************************************************/
byte ble_test_rxbuf[64];
byte  ble_test_index = 0,Ble_Rcv_ok= 0,Ble_CheckSum_EN = 0,TX_EMS_CNT = 0;
/*****************************************추가한 부분(도우)*************************************************/

/* USER CODE END PV */
//24.08.05
// Slave LoRa 국번을 정하는 변수( 변경전 로라 국번 )
byte Bf_Equipment_Num = 0;
byte ENQ = 0x00; // 로라 모듈 국번

//calibration
word angle_ad_point1 = 0, angle_ad_point2 = 0, angle_ad_point3 = 0, angle_ad_point4 = 0, angle_ad_point5 = 0, Succed_Tx_Cnt = 0;
float real_angle_point1 = 0, real_angle_point2 = 0, real_angle_point3 = 0, real_angle_point4 = 0, real_angle_point5 = 0;
float real_current_500mA = 0, real_current_2500mA = 0, cal_500mA_val = 0, cal_2500mA_val = 0;
//calibration

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void LoRa_Tx(byte lora_tx_num);
void Ble_Data_Tx(byte ble_tx_num);	//UART3
void Ble_UART3_exe(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern uint8_t T10ms_flag, gbTimer10ms_cnt;
extern uint8_t T100ms_flag, gbTimer100ms_cnt;
extern byte Record_F;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern ADC_HandleTypeDef hadc1;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
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
  * @brief This function handles Prefetch fault, memory access fault.
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

void SysTick_Handler(void)// 시프트 부분
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
	
  /* USER CODE END SysTick_IRQn 0 */
	HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */
	
  /* USER CODE END SysTick_IRQn 1 */
}

/*****************************************추가한 부분(도우)*************************************************/

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM2 global interrupt.
  */
word adc_value;
dword AVR_CNT = 1000;

//각도센서(Potentiometer)
dword P_M_sum = 0;
float P_M_avr = 0.0;
float P_M_slop = 0;
float P_M_offset = 0;
float P_M_actual = 0;
float PM_avr_value = 0;
float Current_angle = 0.0;
float bf_pm_actual = 0;

//전류센서(CT)
dword CT_ad_sum = 0;
float CT_ad_avr = 0.0;
float CT_slop = 0;
float CT_offset = 0;
float Current_actual = 0;


//1ms cnt(모터 하한, 상한각에서의 정지시간 측정용)
dword timer2_ms_cnt = 0;
byte timer2_ms_cnt_start = 0;

//1min cnt(운동 시간 측정용)
word t2_min_cnt = 0;
byte exerc_time_start = 0;
word min_cnt = 0;
//LED1 토글
byte led1_toggle_flag = 0;

//에러체크
byte Self_check_end = 0;
byte check_step = 0;
//모터 방향 관련 변수
byte CW_pm_check_ok = 0, CCW_pm_check_ok = 0;
byte CW_ct_check_ok = 0, CCW_ct_check_ok = 0;
byte CW_encoder_check_ok = 0, CCW_encoder_check_ok = 0;

//측정모드 테스트용(23.03.06)
byte measure_EN = 0;
byte cur_index = 0;
float cur_measurement1[250], cur_measurement2[250];
//캘리브레이션 관련 변수
byte Cal_rate_of_change_f = 0,plus_cnt = 100;
extern byte Measurement_mode, Measure_up, Measure_down;
extern byte skip_cnt_start;
//각도, 전류 측정확인용 변수
word p_m_avr_cnt = 0, ct_avr_cnt = 0;
byte save_angle_end = 0, default_save_angle = 0;
byte save_ct_end = 0, default_ct_angle = 0, default_save_ct;
//음성모듈
byte Voice_err_cnt=0, Voice_error_cnt = 0;//음성모듈에서 에러가 발생하면 증가하는 카운트
//타이머 정상 작동 확인 변수
void Disable_TIM1_Update_Interrupt(void)
{
    __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_UPDATE);
}
float previous_count = 0, current_count = 0, delta_count = 0, final_count = 0;
word one_degree_1counter = 0, one_degree_2counter = 0, angle_arr[33]= {0};
float lower_ad = 0, upper_ad = 0, lower_angle = 0, upper_angle = 0;
byte angle_measure_1start = 0, angle_measure_2start = 0, angle_count = 0, Tim1_inttrupt = 0;

void TIM1_UP_IRQHandler(void)
{//Encoder
  /* USER CODE BEGIN TIM1_UP_IRQn 0 */
  /* USER CODE END TIM1_UP_IRQn 0 */
  	//HAL_TIM_IRQHandler(&htim1);
	TIM1->SR &= ~TIM_IT_UPDATE;	//TIM1의 업데이트 인터럽트 플래그를 클리어를 수행하며 이는 인터럽트가 발생했음을 알리는 플래그를 지우는 작업으로, 인터럽트를 재발생시킬 수 있도록 함. HAL_TIM_IRQHandler(&htim2);
	{
		Tim1_inttrupt = 1;
		TIM1->SR &= ~TIM_SR_UIF; // 인터럽트 플래그 클리어
		/* USER CODE BEGIN TIM1_UP_IRQn 1 */
		//TIM1->ARR = (65535 - purse1_up_1angle * 5) - 1; //ARR 값 설정 첫 번째 하강
		if(angle_measure_1start)
		{
			one_degree_1counter++;
			angle_count = ANGLE_RANGE - one_degree_1counter;//one_degree_1counter ->  1 = 5도
		}
	} /* USER CODE END TIM1_UP_IRQn 1 */
}
byte Not_save_arr = 0;
byte T2_sec=0, save_170_angle = 1;
word T2_100us_cnt=0;
word Exe_time_ns = 0;
word check_en_counter = 0;//Only use checking
void TIM2_IRQHandler(void)//0.1mS Task
{	
    /*USER CODE BEGIN TIM2_IRQn 0 */
	TIM5->CNT = 0;//프로그램 소요시간 측정 시작	
	//VOICE_CS_HIGH;
	TIM2->SR &= ~TIM_IT_UPDATE;//TIM2의 업데이트 인터럽트 플래그를 클리어를 수행하며 이는 인터럽트가 발생했음을 알리는 플래그를 지우는 작업으로, 인터럽트를 재발생시킬 수 있도록 함. HAL_TIM_IRQHandler(&htim2);		
	
	check_en_counter = TIM1 -> CNT;
	if(anlge_cal_flag && Tim1_inttrupt)//anlge_cal_flag : 첫 번쨰 하강 구간에서 ad저장 Not_save_arr두 번째 상승 구간에서 AD저장, Not_save_arr지워도 됨
	{//첫 번째 상승 종료 -> 첫 하강에서 각도 배열에 AD값 저장하는 구간  
		Tim1_inttrupt = 0; //엔코더 5도 인터럽트가 발생한 순간에만 진입하도록 하였음
		if(save_170_angle)
		{
			save_170_angle = 0;
			angle_count = ANGLE_RANGE;//각도 측정이 시작 될 때 31부터 진입하기 때문에 32로 설정해줘야 32배열에 170도의 ad값이 들어감	
		}
		if(angle_count >= 1 && angle_count <= (ANGLE_RANGE - 1))
			angle_arr[angle_count] = (word)P_M_avr; //P_M_avr: 200ms AD평균값(1000번 평균값)  Store the value of P_M_avr in the array
		else if(angle_count == ANGLE_RANGE || angle_count == 0)
		{
			angle_arr[ANGLE_RANGE] = angle_ad_point2;//210도일 경우에는 직접 설정한 AD를 넣어준다.	
			angle_arr[0] = angle_ad_point4;
		}
		//if(angle_count >= (high_angle / 5))
			//Disable_TIM1_Update_Interrupt();//인터럽트 비활성화
		//Disable_TIM1_Update_Interrupt();//인터럽트 비활성화, 측정 후 TIM1은 사용 x
	}
	P_M_avr = ADC1_data[1];//테스트용으로 밖으로 꺼내 놓음.
	if(low_to_high)
	{//펄스당 카운트를 계산하는 코드이며 이는 첫 번째 상승일 경우에만 필요하다.
		current_count = __HAL_TIM_GET_COUNTER(&htim1);
		delta_count = current_count - previous_count;
		if(delta_count < -32768)
			delta_count += 65536;
		else if (delta_count > 32768)
			delta_count -= 65536;
		// 총 펄스 수 누적
		final_count += delta_count;
		previous_count = current_count;
		//960 to 3940            210.9		
	}
	
	
	static byte save_one = 0;
	if(low_to_high)//-30도에서 210도로 이동명령 반드시 -30도에서 시작해야함.
	{
		if(save_one)
		{
			angle_measure_2start = 1; // 두 번째 상승시 각도/AD 측정 시작 첫 측정 시작 상승시에는 진입하면 안 됨.
			//angle_arr[0] = angle_ad_point4;//-30도일 경우에는 직접 설정한 AD를 넣어준다.	
		}
		PWM_NOT_SLEEP;
		if((P_M_avr >= (angle_ad_point4 - 30)) && (P_M_avr < (angle_ad_point2 - ad_count_down)))//- 1은 P_M_avr이 최대 1이라도 틀어져서 hgih로 올라가지 못하는 현상을 방지해줌
		{
			static byte pwm_cnt = 0;
			if(++pwm_cnt >= 10)
			{
				pwm_cnt = 0;
				low_to_high_pwm += 1;
			}				
			if(low_to_high_pwm >= 2300)//2200
				low_to_high_pwm = 2300;//2200
		}
		if((P_M_avr >= (angle_ad_point2 - ad_count_down)) && (P_M_avr <= angle_ad_point2 + ad_count_down))//감속하는 ad 구간 감속은 ad 가속은 타이머로 설정하였음.
		{//이 구간의 범위는 ad_count_down 값
			remain_ad = angle_ad_point2 - (word)P_M_avr;//현재 남아있는 ad값이며 점점 줄어듦.
			low_to_high_pwm = low_to_high_pwm - (ad_count_down - remain_ad);
			if(low_to_high_pwm <= 2010 && P_M_avr < angle_ad_point2)
				low_to_high_pwm = 2010;
			//if(P_M_avr >= angle_ad_point2 - 7)//여기서 5은 실제 측정 했을 때 목표 ad값에 도달하기 전 에 -10을 해준다. 여기서 이 -값 개념이 매우 중요함. 각도당 펄스의 정확도에 상당한 영향을 미친다.
			if(P_M_avr >= angle_ad_point2 + 4)//여기서 5은 실제 측정 했을 때 목표 ad값에 도달하기 전 에 -10을 해준다. 여기서 이 -값 개념이 매우 중요함. 각도당 펄스의 정확도에 상당한 영향을 미친다.
			{//210도에 도달       11
				low_to_high = 0;
				low_to_high_pwm = 1800;
				//첫 번째 상승에서 각도 배열에 AD값 저장
				//purse1_up_1angle = (dword)final_count / (dword)(real_angle_point2-real_angle_point4);
				purse1_up_1angle = 8333;
				TIM1->ARR = ((word)purse1_up_1angle * 5);//(65535 - ((word)purse1_up_1angle * 5)); // ARR 값 설정 펄스 8322기준으로 ARR은 23925가 정상
				__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
				//purse1_up_1angle : 8315 ~ 8340 정상범위
				//카운트 관련 변수 모두 초기화
				save_one = 1;
				angle_measure_1start = 1;//angle_count계산 시작
				final_count = 0;//누적 펄스 카운트 횟수 초기화
				delta_count = 0;
				current_count = 0;
				previous_count = 0;
				//카운트 관련 변수 모두 초기화		
				TIM5 -> CNT = 0;
				anlge_cal_flag = 1;//AD값 저장
				if(angle_measure_2start)
				{// 두 번 쨰 상승 구간 종료
					angle_measure_2start = 0;
					Not_save_arr = 1;// ad저장 되어 있는 배열에 기록 종료
				}	
				wait_high_to_low = 1;
			}
		}
		TIM8 -> CCR1 = low_to_high_pwm;
	}
	if(start_high_to_low)
	{
		start_high_to_low = 0;
		high_to_low = 1;
	}	
	if(high_to_low)//1차 측정 후 -30도에 해당하는 ad값으로 다시 복귀 -> 1도마다 측정되는 ad값을 기록하기 위함
	{
		static byte enter_only_one = 0;
		if(enter_only_one == 0)
		{
			enter_only_one = 1;
			TIM1->CR1 &= ~TIM_CR1_CEN; // 타이머 카운터 비활성화// 타이머 비활성화				
			TIM1->CNT = ((word)purse1_up_1angle * 5);//설정한 숫자부터 감소 시작, 타이머를 비활성화 한 상태에서 변경해야 cnt가 바뀜
			TIM1->DIER |= TIM_DIER_UIE; // 업데이트 인터럽트를 활성화
			TIM1->CR1 |= TIM_CR1_CEN;
			TIM1->CR1 |= TIM_CR1_DIR;//카운트 방향을 감소로 설정			
		}
		PWM_NOT_SLEEP;
		if((P_M_avr <= (angle_ad_point2 + 30)) && (P_M_avr > (angle_ad_point4 + ad_count_down)))
		{
			static byte pwm_cnt = 0;
			if(++pwm_cnt >= 10)
			{
				pwm_cnt = 0;
				high_to_low_pwm -= 1;
			}
			if(high_to_low_pwm <= 1300)
				high_to_low_pwm = 1300;
		}
		if((P_M_avr <= (angle_ad_point4 + ad_count_down)) && (P_M_avr >= angle_ad_point4 - ad_count_down))
		{
			remain_ad = (word)P_M_avr - angle_ad_point4;
			high_to_low_pwm = high_to_low_pwm + (ad_count_down - remain_ad);
			if(high_to_low_pwm >= 1590 && P_M_avr > angle_ad_point4)
				high_to_low_pwm = 1590;
			//if(P_M_avr <= angle_ad_point4 + 19)//여기서 이 +값 개념이 매우 중요함. 각도당 펄스의 정확도에 상당한 영향을 미친다.
			if(P_M_avr <= angle_ad_point4 + 2)//여기서 이 +값 개념이 매우 중요함. 각도당 펄스의 정확도에 상당한 영향을 미친다.
			{//19
				high_to_low = 0;
				high_to_low_pwm = 1800;
				//purse1_down_1angle = (dword)final_count / (ANGLE_RANGE); // 1도당 펄스의 수
				purse1_down_1angle = (dword)final_count / ((real_angle_point2-real_angle_point4)); // 1도당 펄스의 수
				//카운트 관련 변수 모두 초기화
				final_count = 0;//누적 펄스 카운트 횟수 초기화
				delta_count = 0;
				current_count = 0;
				previous_count = 0;
				//카운트 관련 변수 모두 초기화		
				anlge_cal_flag = 0;//stop recording		
				eeprom_page_state = 6;//eeprom에 ad값 저장 
			}
		}
		TIM8->CCR1 = high_to_low_pwm;			
	}
	
	if(low_to_high || high_to_low)
	{
		P_M_slop = (real_angle_point2 - real_angle_point4) / (angle_ad_point2 - angle_ad_point4); 
		P_M_offset = real_angle_point4 - (P_M_slop * angle_ad_point4);		
		P_M_actual = (P_M_slop * P_M_avr) + P_M_offset;
		PM_avr_value = (P_M_actual + bf_pm_actual) / 2;	//과거값과 현재값을 평균냄
		bf_pm_actual = P_M_actual;//현재값을 과거값에 저장		
	}
	else 
	{
		if(P_M_avr >= angle_ad_point3 - 100 && P_M_avr <= angle_ad_point4)//ad구간의 여유를 두기 위해 100으로 설정 정확도에는 영향을 주지 않음.
		{//Point3 ~ Point4
			P_M_slop = (real_angle_point4 - real_angle_point3) / (angle_ad_point4 - angle_ad_point3); 
			P_M_offset = real_angle_point3 - (P_M_slop * angle_ad_point3);					
		}
		else if(P_M_avr <= angle_ad_point1 + 100 && P_M_avr >= angle_ad_point2)//ad구간의 여유를 두기 위해 100으로 설정 정확도에는 영향을 주지 않음.
		{//Point1 ~ Point2
			P_M_slop = (real_angle_point1 - real_angle_point2) / (angle_ad_point1 - angle_ad_point2); 
			P_M_offset = real_angle_point2 - (P_M_slop * angle_ad_point2);					
		}	
		if(((P_M_avr >= angle_ad_point3 - 100) && (P_M_avr <= angle_ad_point4)) || ((P_M_avr <= angle_ad_point1 + 100) && (P_M_avr >= angle_ad_point2)))
		{
			P_M_actual = (P_M_slop * P_M_avr) + P_M_offset;
			PM_avr_value = (P_M_actual + bf_pm_actual) / 2;	//과거값과 현재값을 평균냄
			bf_pm_actual = P_M_actual;//현재값을 과거값에 저장
		}	
		else
		{//각도 계산
			if(angle_recording_end)// 0으로 만들면 측정이 불가능해진다. 
			{/*
				//byte ANGLE_RANGE = 49;  // 48*5=240
				int lower_index = 0;
				int upper_index = 0;
				// 배열을 순회하여 P_M_avr 값이 포함되는 구간을 찾음
				//P_M_avr = 3800;//1000
				for(int i = 0; i < ANGLE_RANGE; i++)  
				{//5도씩 ad저장
					if(P_M_avr >= angle_arr[i] && P_M_avr <= angle_arr[i + 1])
					{
						lower_index = i;
						upper_index = i + 1;
						break;
					}
				}
				//선형 보간법을 사용하여 정확한 각도 계산
				lower_ad = angle_arr[lower_index];
				upper_ad = angle_arr[upper_index];
				lower_angle = real_angle_point4 + lower_index * 5;
				upper_angle = real_angle_point4 + upper_index * 5;
				//보간 계산
				if(P_M_avr > angle_ad_point4 && P_M_avr < angle_ad_point2)
				{
					if(upper_ad != lower_ad)
						P_M_actual = lower_angle + (P_M_avr - lower_ad) * (upper_angle - lower_angle) / (upper_ad - lower_ad);
					else
					{
						if(P_M_avr > (angle_ad_point2 + angle_ad_point4)/2) //10도 170도 AD값 평균
							P_M_actual = real_angle_point2;
						else
							P_M_actual = real_angle_point4;
					}
					PM_avr_value = (P_M_actual + bf_pm_actual) / 2;	//과거값과 현재값을 평균냄
					bf_pm_actual = P_M_actual;//현재값을 과거값에 저장			
				}
				*/
				int size = sizeof(angle_arr) / sizeof(angle_arr[0]); // 배열 크기 계산 // 실제 각도를 계산하기 위해 interpolate 함수 호출
				PM_avr_value = interpolate(angle_arr, size, P_M_avr, real_angle_point4, real_angle_point2, angle_ad_point4, angle_ad_point2, &bf_pm_actual);
			}
		}		
	}
	if(++T2_100us_cnt >= 10000)//10000*0.1ms=1sec
	{//1000msec = 1sec task
		led1_toggle_flag = !led1_toggle_flag;
		//plus_cnt++;
		if(led1_toggle_flag)
		{
			LED1_ON;
		}
		else
			LED1_OFF;
		T2_100us_cnt = 0;
		if(++T2_sec >= 60)
			T2_sec = 0;
	}	
	////voice_output(); 1ms카운터에 넣으면 작동 불가
	if(voice_out)//0.1ms task
	{
		voice_out = 0;
		voice_output();
	}
	//Calibration모드 모터 수동조작시 Delay
	static unsigned int motor_delay_cnt = 0;
	if(Motor_Run_F)
	{
		if(++motor_delay_cnt >= 5000)	//500ms
		{
			motor_delay_cnt = 0;
			Motor_Run_F = 0;
		}
	}
	else	
		motor_delay_cnt = 0;
	//motor_delay_ms 함수용
	static unsigned int t2_cnt = 0;
	if(timer2_ms_cnt_start == 1)
	{
		if(++t2_cnt >= 10)
		{
			t2_cnt = 0;
			timer2_ms_cnt++;	//1ms마다 1씩증가
		}
	}
	else
	{
		t2_cnt = 0;
		timer2_ms_cnt = 0;	
	}
	//motor_exerc_time 함수용
	static unsigned int Timer2_cnt = 0;
	if(exerc_time_start == 1)
	{
		if(!PAUSE_F)	//일시정지 아닐때
		{
			if(++Timer2_cnt >= 600000)
			{
				Timer2_cnt = 0;
				t2_min_cnt++;
				min_cnt++;
			}
		}
	}
	else
	{
		Timer2_cnt = 0;
		t2_min_cnt = 0;
	}
	/*********************ADC1(전류,각도)*********************/
	//전류센서, 각도센서 AD값 받아옴
	adc_value = ADC1->DR;
	ADC1_data[ADC1_index] = adc_value;
	//어깨 기구용 P_M_avr 값으로 설정하였음
	if(_USER_RUN_SETTING.calibration == 0x01)	//Calibration 모드
	{
		PWM_NOT_SLEEP;
		AVR_CNT = 10000;	//Calibration중에는 10000번 평균(10초에 한번씩 평균값 계산)
	}
	else
		AVR_CNT = 1000;		//평상시에는 1000번 평균(100ms에 한번씩 평균값 계산)
	static byte ad_save = 0;
	switch(ADC1_index)
	{
		case 0:		//ADC1 CH15 (전류센서)
			//ADC1_data[0] = adc_value;
			if((_USER_RUN_SETTING.ad_start == 0x05)||(_USER_RUN_SETTING.ad_start == 0x06))
			{
				ad_save = _USER_RUN_SETTING.ad_start;
				CT_ad_sum = 0;
				ct_avr_cnt = 0;
				_USER_RUN_SETTING.ad_start = 0x00;
			}
			CT_ad_sum += ADC1_data[0];
			if(++ct_avr_cnt >= AVR_CNT)
			{
				ct_avr_cnt = 0;
				CT_ad_avr  = (float)CT_ad_sum / AVR_CNT;// 전류값 평균
				CT_ad_sum = 0;
				switch(ad_save)
				{
					case 5:
						_CALIBRATION_SETTING.current_500mA_AD = CT_ad_avr;	//500mA일때 AD평균값 저장
						_CALIBRATION_SETTING.AD_ok = 1;
						ad_save = 0;
						ct_cal_ad_1 = 1;
						break;
					case 6:
						_CALIBRATION_SETTING.current_2500mA_AD = CT_ad_avr;		//2500mA일때 AD평균값 저장
						_CALIBRATION_SETTING.AD_ok = 1;
						save_ct_flag = 1;
						ad_save = 0;
						ct_cal_ad_2 = 1;//최종저장
						break;
					default:
						break;
				}
				CT_slop = (real_current_2500mA - real_current_500mA)/(cal_2500mA_val - cal_500mA_val);//0.2A, 2A  two point calibration
				CT_offset = real_current_500mA - (CT_slop*cal_500mA_val);
				//CT_slop = (2500 - 500)/(2580 - 434);		//0.2A, 2A  two point calibration
				//CT_offset = 500 - (CT_slop*434);
				Current_actual = (CT_slop * CT_ad_avr) + CT_offset;					
				if((Current_actual < 0.1) && (Current_actual > -0.1))	//-0.1A초과 +0.1A 미만은 0A로 처리
					Current_actual = 0.0;
			}
			break;
		case 1:	//ADC1 CH8	(각도센서)Potentiometer
			//ADC1_data[ADC1_index] = adc_value;
			if((_USER_RUN_SETTING.ad_start == 0x01) || (_USER_RUN_SETTING.ad_start == 0x02) || (_USER_RUN_SETTING.ad_start == 0x03) || (_USER_RUN_SETTING.ad_start == 0x04))
			{
				ad_save = _USER_RUN_SETTING.ad_start;
				P_M_sum = 0;
				p_m_avr_cnt = 0;
				_USER_RUN_SETTING.ad_start = 0x00;
			}
			P_M_sum += ADC1_data[1];
			
			if(++p_m_avr_cnt >= AVR_CNT)
			{
				p_m_avr_cnt = 0;
				P_M_avr = (float)P_M_sum / AVR_CNT;
				P_M_sum = 0;
				P_M_avr = ADC1_data[1];
				switch(ad_save)
				{
					case 1:							
						_CALIBRATION_SETTING.angle_ad_point_1 = (word)P_M_avr;	//210도일때 AD평균값 저장
						_CALIBRATION_SETTING.AD_ok = 1;
						ad_save = 0;	
						angle_cal_ad_1 = 1;
						break;
					case 2:
						_CALIBRATION_SETTING.angle_ad_point_2 = (word)P_M_avr;	//170도일때 AD평균값 저장
						_CALIBRATION_SETTING.AD_ok = 1;
						ad_save = 0;
						angle_cal_ad_2 = 1;
						break;
					case 3:
						_CALIBRATION_SETTING.angle_ad_point_3 = (word)P_M_avr;	//-30도일때 AD평균값 저장
						_CALIBRATION_SETTING.AD_ok = 1;
						ad_save = 0;
						angle_cal_ad_3 = 1;
						break;		
					case 4:
						_CALIBRATION_SETTING.angle_ad_point_4 = (word)P_M_avr;	//10도일때 AD평균값 저장
						_CALIBRATION_SETTING.AD_ok = 1;
						ad_save = 0;
						angle_cal_ad_4 = 1;
						calibration_state = 1;//calibration ing state
						break;
						/*
					case 5:
						_CALIBRATION_SETTING.angle_ad_point_5 = (word)P_M_avr;	//xx도일때 AD평균값 저장
						_CALIBRATION_SETTING.AD_ok = 1;
						ad_save = 0;
						angle_cal_ad_5 = 1;
						break;	
						*/						
					default:
						break;
				}
				switch(_USER_RUN_SETTING.motion)
				{
					case Elbow:		//팔꿈치
						_Ble_Data.motion = 0x00;
						_LoRa_Data.motion = 0x00;
						Current_angle = (round(PM_avr_value*10)/10) - 60;	    //소수점 둘째자리에서 반올림
						break;
					case Shoulder:	//어깨
						_Ble_Data.motion = 0x01;
						_LoRa_Data.motion = 0x01;
						Current_angle = round(PM_avr_value*10)/10;		        //소수점 둘째자리에서 반올림	
						break;
					case Knee:		//무릎
						_Ble_Data.motion = 0x02;
						_LoRa_Data.motion = 0x02;
						//Current_angle = -((round(PM_avr_value*10)/10) - 90);	//소수점 둘째자리에서 반올림
						Current_angle = -((round(PM_avr_value*10)/10) - 110);	//소수점 둘째자리에서 반올림
						//Current_angle = 110 - PM_avr_value;
						break;
					case wrist:		//손목
						_Ble_Data.motion = 0x04;
						_LoRa_Data.motion = 0x03;
						Current_angle = (round(PM_avr_value*10)/10) - 90;	    //소수점 둘째자리에서 반올림
						break;
					case ankle:		//발목
						_Ble_Data.motion = 0x08;
						_LoRa_Data.motion = 0x04;
						Current_angle = (round(PM_avr_value*10)/10) - 90;	    //소수점 둘째자리에서 반올림
						break;						
					default:
						break;
				}
				if(Priority_dir_set == 1)	
					Priority_dir_set = 'S';	//셀프체크 방향 설정시작(Start)
			}
			break;
		default:
			break;
	}
	ADC1_index++;
	if(ADC1_index >= 2)
		ADC1_index = 0;
	//HAL_ADC_Start(&hadc1);
	ADC1->CR2 |= (uint32_t)ADC_CR2_SWSTART;	 //selected ADC1 software conversion
	static byte gbTimer1ms_cnt = 0;
	if(++gbTimer1ms_cnt >= 10)
	{
		gbTimer1ms_cnt = 0;
		if(LoRa_Rcv_ok)
		{
			if(LoRa_RS485_dead_time)
				LoRa_RS485_dead_time--;	//1msec 마다 1씩 감소
			else
			{
				LoRa_Rcv_ok = (LoRa_Rcv_ok<<4) | LoRa_Rcv_ok;	//0x10:110, 0x04:44, 0x0E:EE
			}
		}	
		if(Rcv2_ok)//UART4 data received?
		{
			if(RS485_dead_time)
				RS485_dead_time--;	//1msec 마다 1씩 감소
			else
			{
				Rcv2_ok = (Rcv2_ok<<4) | Rcv2_ok;//10:110, 4:44, E:EE
			}
		}
	}
	if(++gbTimer10ms_cnt>=100)
	{
		gbTimer10ms_cnt=0;
		T10ms_flag=1;
		if(++gbTimer100ms_cnt>=10)
		{
			gbTimer100ms_cnt=0;
			T100ms_flag=1;
		}
	}		
	
	
	
	
  /* USER CODE END TIM2_IRQn 0 */
//	HAL_TIM_IRQHandler(&htim2);
//	if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE) != RESET)
//	{
//		if (__HAL_TIM_GET_IT_SOURCE(&htim2, TIM_IT_UPDATE) != RESET)
//			__HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
//	}
	/* USER CODE BEGIN TIM2_IRQn 1 */
  /* USER CODE END TIM2_IRQn 1 */
	Exe_time_ns = (word)(TIM5->CNT*1000.0/72.0);//프로그램 소요 시간 측정 0.1ms, 100ns 초과하면 비정상  
}
//CPM기기 높낮이 전용 상수 정의
#define MAX_PWM 1800
#define MIN_PWM 0
#define RAMP_STEPS 3000//pwm 증가 or 감소하는 구간
#define LONG_PRESS_TIME_MS 1500
#define HOLD_TIME_MS 10000 //상승 or 하강하는 구간
#define HOLD_16SEC 16000
//CPM기기 높낮이 전용 변수 정의
byte button_up_pressed = 0;
volatile byte button_down_pressed = 0;
float current_pwm = 0.0f;
volatile bool moving_up = false;
volatile bool moving_down = false;
volatile dword button_up_duration = 0;
byte up_long_press = 0, down_long_press = 0; 
byte down_stop_flag = 0, up_stop_flag = 0;
word step_count = 0;
dword hold_count = 0;

//스위치 채터링 방지
byte T10ms_cnt_debouncing = 0;
byte EMS_STATE = 0, cant_read_id = 0;
byte BF_START_SW = 0, BF_STOP_SW = 0, BF_EMS_SW = 0;
word T3_1000ms_cnt=0;
byte T3_sec=0, T3_min=0;

//EEROM변수
word eeprom_timer_cnt = 0;

//음성모듈 변수
byte t3_10ms_cnt2 = 0, voice_en = 0, state_cnt2 = 0;

byte record_en = 0;
byte RS485_dead_time=0;			//1msec down counter
byte LoRa_RS485_dead_time=0;	//1msec down counter
//셀프체크 관련 변수
byte Start_self_check = 0, Restart_self_check = 0, Opposition = 0;// 처음 값이 1이 돼야 처음 감지 가능함
float arr_test[255] = {0};
byte ddsfsdf = 0, T3_5000ms_cnt_flag = 0, wait_measurement = 0, over_angle__start_flag = 0;
word T3_5000ms_cnt = 0,t3_500ms_cnt1 = 0, t3_500ms_cnt2 = 0;
void TIM3_IRQHandler(void)	//1mS Task
{
	////HAL_TIM_IRQHandler(&htim3);
	//#define __HAL_TIM_CLEAR_IT(__HANDLE__, __INTERRUPT__)      ((__HANDLE__)->Instance->SR = ~(__INTERRUPT__))
	TIM3->SR &= ~TIM_IT_UPDATE;	//TIM3의 업데이트 인터럽트 플래그를 클리어를 수행하며 이는 인터럽트가 발생했음을 알리는 플래그를 지우는 작업으로, 인터럽트를 재발생시킬 수 있도록 함. HAL_TIM_IRQHandler(&htim2);
	/*
	if (__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_UPDATE) != RESET)
	{
		if (__HAL_TIM_GET_IT_SOURCE(&htim3, TIM_IT_UPDATE) != RESET)
		{
			__HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
		}
	}	
*/
	if(over_angle__start_flag && wait_measurement == 0)
	{	
		over_angle__start_flag = 0;
		wait_measurement = 1;
		T3_5000ms_cnt_flag = 1;
	}
	if(T3_5000ms_cnt_flag)
	{
		if(++T3_5000ms_cnt >= 2200)
		{
			wait_measurement = 0;
			T3_5000ms_cnt_flag = 0;
			Change_dir = 1; //3초 뒤에 Cal_rate_of_change_f = 1수행
			T3_5000ms_cnt = 0;
			start_over_angle = 1;//상, 하한각 도달 시 연속으로 측정되는 현상을 방지하기 위함
		}
	}
	if(++T3_1000ms_cnt >= 1000)
	{//타이머 정상작동 테스트용
		T3_1000ms_cnt = 0;
		if(++T3_sec >= 60)
		{
			T3_sec=0;
			if(++T3_min >= 60)
				T3_min=0;
		}
	}	
	static byte Voice_NG = 0;
	if((Voice_err_cnt == 0) && (Voice_NG == 1))//처음으로 Voice Error다 나오고 5ms 경과시
	{
		VOICE_RESET_LOW;	
		state_cnt2 = 1;		//Voce IC Power Down Sequence
	}
	else
		Voice_err_cnt--;	//5mS동안 Voice Reset을 High로 만들기 위함
	// End od Reset sequence
	if(++t3_10ms_cnt2 >= 10) //10msec
	{	
		t3_10ms_cnt2 = 0;
		/*
		static byte cnt_ttt = 0;
		if(ddsfsdf && cnt_ttt <=253)
		{
			arr_test[cnt_ttt++] = Current_actual;
		}
		*/
		if(voice_error_check)
		{
			SPI_tx_buf[0] = 0x48;// 0x48 : Read ID -> (0x04 EF 40 15)를 읽어야 에러체크 OK			
			voice_error_check = 0;
			VOICE_CS_LOW;//음성모듈 CS LOW
			HAL_SPI_TransmitReceive(&hspi1, SPI_tx_buf, SPI_rx_buf, 5 ,10);
			VOICE_CS_HIGH;//음성모듈 CS HIGH
			if(SPI_rx_buf[1] == 0x04 & SPI_rx_buf[2] == 0xEF & SPI_rx_buf[3] == 0x40 & SPI_rx_buf[4] == 0x15)
				cant_read_id = 1;//정상
			else 
				cant_read_id = 0;//비정상
		}	
		if(voice_en)
		{		
			SPI_tx_buf[0]= 0x40;// 0x40 : Read Status ->  (0x60 0x04)를 읽어야 OK
			VOICE_CS_LOW;// 음성모듈 CS LOW
			HAL_SPI_TransmitReceive(&hspi1, SPI_tx_buf, SPI_rx_buf,2, 10);
			VOICE_CS_HIGH;// 음성모듈 CS HIGH
			if(SPI_rx_buf[0] == 0x60)
			{
				if(SPI_rx_buf[1] == 0x04)
				{
					voice_en = 0;				
					SPI_voice_set();//Voice값 출력
					state_cnt2 = 0;
					Voice_NG = 0;
					_USER_RUN_SETTING.x06_1B = 0;
					_VOICE_DATA.x06_1B = 0;					
				}
				else
				{
					Voice_NG = 1;
					Voice_error_cnt++;
					if(state_cnt2 == 0)	//처음으로 Voice Error 만난경우
					{
						Voice_err_cnt = 5;	//5mS동안 Voice Reset을 High로 만들기 위함
						VOICE_RESET_HIGH;	//리셋(High Active)
						state_cnt2 = 1;
					}
					else if(state_cnt2 == 1) 	//power down Sequence
					{
						SPI_tx_buf[0]= 0x12;	//power down
						VOICE_CS_LOW;			// 음성모듈 CS LOW
						HAL_SPI_Transmit(&hspi1, SPI_tx_buf, 1, 10); 
						VOICE_CS_HIGH;			// 음성모듈 CS HIGH		
						state_cnt2 = 2; 		//Voice IC Power Up
					}
					else if(state_cnt2 == 2)	//power up Sequence
					{
						SPI_tx_buf[0]= 0x10; //power up
						VOICE_CS_LOW;//음성모듈 CS LOW
						HAL_SPI_Transmit(&hspi1, SPI_tx_buf, 1, 10); 
						VOICE_CS_HIGH;//음성모듈 CS HIGH	
						voice_en = 0;
						state_cnt2 = 0;
					}					
				}
			}
		}		
	}
	if(read_sensitivity || save_angle_flag || save_ct_flag || anlge_ad_err_flag || anlge_real_err_flag || ct_err_flag || page1_end_flag || save_angle_end_flag || page2_end_flag || wait_1400m_flag || Last_page ||wait_high_to_low)
	{//EERPOM 각도, 전류 캘리브레이션 저장 클릭 후 1.4초 후에 저장
		eeprom_timer_cnt++;
		if(eeprom_timer_cnt >= 1700) // 1.7초
		{
			if(read_sensitivity)
			{
				read_sensitivity = 0;
				start_read_sense = 1;
			}
		    if(wait_high_to_low)
		    {
		  		wait_high_to_low = 0;
				start_high_to_low = 1;
		    }
			if(wait_1400m_flag)
			{
				wait_1400m_flag = 0;
				wait_1400m_start = 1;
			}
			if(page1_end_flag)
			{
				page1_end_flag = 0;
				page1_1400msec_wait = 1;
			}		
			if(page2_end_flag)
			{
				page2_end_flag = 0;
				page2_1400msec_wait = 1;
			}				
			if(save_angle_flag)
			{
				save_angle_flag = 0; // 타이머를 리셋하는 조건 추가 (선택 사항)
				save_angle_end = 1;
			}			
			if(save_angle_end_flag)
			{
				save_angle_end_flag = 0;
				real_angle_value_page2_end = 1;
			}
			if(Last_page)
			{
				Last_page = 0;
				Last_page_end = 1;
			}	
			if(save_ct_flag)
			{
				save_ct_flag = 0; // 타이머를 리셋하는 조건 추가 (선택 사항)
				save_ct_end = 1;
			}

			if(anlge_ad_err_flag || anlge_real_err_flag)
			{
				default_save_angle = 1;
				anlge_real_err_flag = 0;
				anlge_ad_err_flag = 0; // 타이머를 리셋하는 조건 추가 (선택 사항)
			}
			if(ct_err_flag)
			{
				default_save_ct = 1;
				ct_err_flag = 0; // 타이머를 리셋하는 조건 추가 (선택 사항)
			}
			eeprom_timer_cnt = 0;
		}
	}	
	static byte self_checking = 0;	
	if(++T10ms_cnt_debouncing >= 10)	//10ms
	{		
		T10ms_cnt_debouncing = 0;
		//START 스위치 디바운싱
		
		if(Succed_Tx_Cnt)
		{
		  Succed_Tx_Cnt--;
		}
		
		//09.20 도우가 함
		if(TX_EMS_CNT)
		{
		  TX_EMS_CNT--;
		}
		
		static byte BFBF_START_SW = 0;
		//도우가 변경함
		if(self_checking == 0)
		{//셀프체크중이면 start stop 기능 x
			if(START_SW == BF_START_SW)//초기값이 1 -->  아래의 if문을 수행하지 않고 다음 수행문을 수행         
			{//cattering ok
				 if(START_SW == 1)//sw1 push 상태                           
				 {//여기 부분에 스위치를 누르고 있을 때 원하는 동작 작성하기
					 //도우가 변경함
					 //if(START_SW != BFBF_START_SW && (_SENSOR_DATA.state == 1 || _SENSOR_DATA.state == 4))//누르고 나서 한 번만 동작 수행
					 if(START_SW != BFBF_START_SW && (_SENSOR_DATA.state == 1 || _SENSOR_DATA.state == 4) && STOP_SW != 1)//누르고 나서 한 번만 동작 수행( 정지 버튼이 동시에 눌리지 않았을때 )
					 {
						START_STATE = 1;
						_SENSOR_DATA.start_sw = START_STATE;
						Succed_Tx_Cnt = 50;
					 }
					 else if(START_SW != BFBF_START_SW && STOP_SW == 1)//누르고 나서 한 번만 동작 수행( 정지 버튼이 동시에 눌렸을때 5초후 리셋및 LCD 모니터링 화면으로 진입 )
					 {
					   	START_STATE = 1;
						_SENSOR_DATA.start_sw = START_STATE;
						Succed_Tx_Cnt = 500;
					 }
				 }
				 else 
				 {
					 if(BFBF_START_SW)  //sw1이 과거이전에 안눌린 상태, 누르고 원상태로 복귀함 
					 {//뗏을 때
					   Succed_Tx_Cnt = 50;
					 }
				 }
				 BFBF_START_SW = BF_START_SW;
			}
			BF_START_SW = START_SW;
			//STOP 스위치 디바운싱
			static byte BFBF_STOP_SW = 0;
			if(STOP_SW == BF_STOP_SW)                    //초기값이 1 -->  아래의 if문을 수행하지 않고 다음 수행문을 수행         
			{//cattering ok
				 if(STOP_SW == 1)//sw1 push 상태                           
				 {// 여기 부분에 스위치를 누르고 있을 때 원하는 동작 작성하기
					 //도우가 변경함
					 //if(STOP_SW != BFBF_STOP_SW && (_SENSOR_DATA.state == 0 || _SENSOR_DATA.state == 4))//누르고 나서 한 번만 동작 수행
				   	 if(STOP_SW != BFBF_STOP_SW && (_SENSOR_DATA.state == 0 || _SENSOR_DATA.state == 4) && START_SW != 1)//누르고 나서 한 번만 동작 수행( 시작 버튼이 동시에 눌리지 않았을때 )
					 {
						STOP_STATE = 1;
						_SENSOR_DATA.stop_sw = STOP_STATE;
						Succed_Tx_Cnt = 50;
					 }		
					 else if(STOP_SW != BFBF_STOP_SW && START_SW == 1)//누르고 나서 한 번만 동작 수행( 시작 버튼이 동시에 눌렸을때 5초후 리셋및 LCD 모니터링 화면으로 진입 )
					 {
						STOP_STATE = 1;
						_SENSOR_DATA.stop_sw = STOP_STATE;
						Succed_Tx_Cnt = 500;
					 }
				 }
				 else 
				 {
					 if(BFBF_STOP_SW)  //sw1이 과거이전에 안눌린 상태, 누르고 원상태로 복귀함 
					 {//뗏을 때
					   Succed_Tx_Cnt = 50;
					 }
				 }
				 BFBF_STOP_SW = BF_STOP_SW;
			}
			BF_STOP_SW = STOP_SW;	
		}
		/*
		if(EMS_SW == BF_EMS_SW)	//Debounce Check OK(10ms 동안 동일 상태)
		{
			if(EMS_SW)
			{
				EMS_STATE = 1; // EMS 스위치가 눌렸을 때의 동작
				if(Measurement_mode == 1)
					measure_restart = 1;
			}
			else 
				EMS_STATE = 0; // EMS 스위치가 눌리지 않았을 때의 동작
		}
		BF_EMS_SW = EMS_SW;
		*/
	}	
	if(Settings) 
	{
		if(_USER_RUN_SETTING.equip_up_down)
			PWM_NOT_SLEEP;
		if(_USER_RUN_SETTING.equip_up_down == 1)// _USER_RUN_SETTING.equip_up_down 값에 따라 동작 설정
		{
			_USER_RUN_SETTING.equip_up_down = 0;
			if(up_long_press == 0 && down_long_press == 0 && moving_down == false)//up길게 누르면 실행하는 동작을 완료 해야 진입가능
				button_up_pressed = 1; //버튼이 위로 눌림
			if(down_long_press)//아래로 10초동안 운동 중인 상태에서 반대방향인 down을 눌러서 정지시킴
				down_stop_flag = 1;
		} 
		else if(_USER_RUN_SETTING.equip_up_down == 2) 
		{
			_USER_RUN_SETTING.equip_up_down = 0;
			up_long_press = 1;// 위로 길게 눌림
			moving_up = true;
			moving_down = false;
		} 
		else if(_USER_RUN_SETTING.equip_up_down == 3) 
		{
			_USER_RUN_SETTING.equip_up_down = 0;
			if(up_long_press == 0 && down_long_press == 0 && moving_up == false)//down길게 누르면 실행하는 동작을 완료 해야 진입가능
				button_down_pressed = 1;// 버튼이 아래로 눌림
			if(up_long_press)//위로 10초동안 운동 중인 상태에서 반대방향인 up을 눌러서 정지시킴
				up_stop_flag = 1;
		} 
		else if(_USER_RUN_SETTING.equip_up_down == 4) 
		{
			_USER_RUN_SETTING.equip_up_down = 0;
			down_long_press = 1;// 아래로 길게 눌림
			moving_down = true;
			moving_up = false;
		}
			
		// 위로 버튼이 눌렸을 경우
		if(button_up_pressed) 
		{
			MOTOR_UP_DOWN;//모터 높낮이 조절
			moving_up = true;
			moving_down = false;
			button_up_pressed = 0;
			step_count = 0; // 스텝 카운터 초기화
			hold_count = 0; // 홀드 카운터 초기화
		}
		// 아래로 버튼이 눌렸을 경우
		if(button_down_pressed) 
		{
			MOTOR_UP_DOWN; // 모터 높낮이 조절
			moving_down = true;
			moving_up = false;
			button_down_pressed = 0;
			step_count = 0; // 스텝 카운터 초기화
			hold_count = 0; // 홀드 카운터 초기화
		}
		if(moving_up || moving_down) // 위로 움직이는 경우
		{//길게 눌렸을 경우
			if(up_long_press || down_long_press) 
			{
				hold_count++;
				if(down_stop_flag)//up ing작업을 실행중에 dwon 버튼을 누를 경우 진입
				{
					down_stop_flag = 0;
					hold_count = RAMP_STEPS + HOLD_TIME_MS;
				}
				else if(up_stop_flag)//down ing작업을 실행중에 up 버튼을 누를 경우 진입
				{
					up_stop_flag = 0;
					hold_count = RAMP_STEPS + HOLD_TIME_MS;					
				}							
				if(hold_count < RAMP_STEPS) 
				{ // PWM 증가 구간 (0 ~ 3초)
					current_pwm += (float)MAX_PWM / (RAMP_STEPS / 2);
					if(current_pwm > MAX_PWM) 
						current_pwm = MAX_PWM;
				} 
				else if((hold_count >= RAMP_STEPS) && (hold_count < RAMP_STEPS + HOLD_TIME_MS)) // 일정 PWM 유지 (3 ~ 13초)
					current_pwm = MAX_PWM;
				else if((hold_count >= RAMP_STEPS + HOLD_TIME_MS) && (hold_count <= RAMP_STEPS + HOLD_TIME_MS + RAMP_STEPS)) 
				{ // PWM 감소 구간 (13 ~ 16초)
					current_pwm -= (float)MAX_PWM / (RAMP_STEPS / 2);
					if(current_pwm < MIN_PWM) 
						current_pwm = MIN_PWM;
				} 
				else if(hold_count > HOLD_16SEC) 
				{ // 일정 시간 이상 경과 시 초기화
					up_long_press = 0;
					down_long_press = 0;
					moving_up = false;
					moving_down = false;
					current_pwm = 0;  // PWM 초기화
				}
			}
			else 
			{//짧게 눌렸을 경우
				if(step_count < RAMP_STEPS / 2) 
				{ // PWM 증가 구간
					current_pwm += (float)MAX_PWM / (RAMP_STEPS / 2);
					if(current_pwm > MAX_PWM) 
						current_pwm = MAX_PWM;
					step_count++;
				} 
				else if(step_count >= (RAMP_STEPS / 2) && step_count < RAMP_STEPS) 
				{ // PWM 감소 구간
					current_pwm -= (float)MAX_PWM / (RAMP_STEPS / 2);
					if(current_pwm < MIN_PWM) 
						current_pwm = MIN_PWM;
					step_count++;
				} 
				else 
				{
					moving_up = false;
					moving_down = false;
					current_pwm = 0;  // PWM 초기화
				}
			}
			if(moving_up) 
			{
				Motor_PWM_CW = (unsigned int)(0.5 * current_pwm + 1800);
				TIM8 -> CCR1 = Motor_PWM_CW;
			} 
			else if(moving_down)
			{
				Motor_PWM_CCW = (unsigned int)(-0.5 * current_pwm + 1800);
				TIM8 -> CCR1 = Motor_PWM_CCW;
			}
			_SENSOR_DATA.state = 0;//운동중				
		} 
		else//환경설정에서 기본적으로 여기 진입해야함.
		{//운동종료
			Motor_PWM_CW = 1800; // 모터 정지 (PWM 0)
			Motor_PWM_CCW = 1800; // 모터 정지 (PWM 0)
			if(_SENSOR_DATA.state == 0)
				_SENSOR_DATA.state = 1;//운동종료
			PWM_SLEEP;
		}		
	}


	
	if(Measurement_mode && Change_dir)//부하가 감지 되면 들어오게 구현해야함.
	{//변화율 계산 flag ON	
		if(Measure_up == 1 || Measure_down == 1)//상한측정 또는 하한측정 중일때만 인터럽트(상한측정후 1초 정지하는 동안에는 변화율 계산하지않음)
		{
			if(++t3_500ms_cnt1 >= 2200)		
			{//0.5sec task
				Change_dir = 0;
				t3_500ms_cnt1 = 0;
				Cal_rate_of_change_f = 1;	//변화율 계산 flag ON	
			}
		}
	}
	else
	{
		t3_500ms_cnt1 = 0;
	}
	if(++t3_500ms_cnt2 >= 500)
	{
		//0.5sec task
		t3_500ms_cnt2 = 0;
		if(Record_F == 1)
		{
			static float record_data1 = 0;
			static float record_data2 = 0;
			record_data1 = (word)(Current_angle);
			record_data2 = Current_actual;
			if(measure_EN && (cur_index <= 250))
			{
				cur_measurement1[cur_index++] = record_data1;
				cur_measurement2[cur_index] = record_data2;
			}
		}
	}	
	/******************************셀프체크*************************************/
	static unsigned int error_check_cnt = 0, t100us_cnt = 0, time_cnt = 0;
	static byte retry_f = 0;
	static byte k = 0, save_f = 0;

	if((Error_Check && angle_recording_end) || (Restart_self_check && angle_recording_end))//셀프체크 시작 && 각도 ad 값 읽어오기 끝 //1.7초 뒤에 시작
	{
		_SENSOR_DATA.state = 0x00;//셀프체크중에는 운동중으로 표시
		self_checking = 1;
		Self_check_end = 0;
		//check_step 마다 카운팅 시간이 다름
		if(check_step % 2 == 1)	
			time_cnt = 2000;	//20000*0.1ms => 2s
		else if(check_step % 2 == 0)
			time_cnt = 1000;	//10000*0.1ms => 1s
		if(++error_check_cnt >= time_cnt)
		{
			error_check_cnt = 0;
			check_step++;
		}	
		static byte check_dir;
		switch(check_step)
		{
			case 0:	//CW 셀프체크 시작(셀프체크전 값 저장)
				_LoRa_Data.check_state |= 0x02;//Para.CT_error = 0;		//에러체크 중
				_LoRa_Data.check_state |= 0x04;//Para.encoder_error = 0;	//에러체크 중
				_LoRa_Data.check_state |= 0x08;//Para.PM_error = 0;		//에러체크 중		
				Bf_pm_avr = P_M_actual;		//에러체크하기 전에 각도센서 값 저장
				Bf_ct_avr = Current_actual;	//에러체크하기 전에 전류센서 값 저장
				Bf_encoder_avr = diff;		//에러체크하기 전에 엔코더 값 저장	
				check_step = 1;//가속 시작
			case 1:	//CW 셀프체크 시작(가속 시작)
				save_f = 1;	//테스트용 변수 지워도됨
				if(++t100us_cnt >= 100)
				{
					t100us_cnt = 0;
					shift_step++;
					if(shift_step >= 21)
						shift_step = 20;
				}
				break;
			case 2://CW셀프체크 중(가속 끝)
				t100us_cnt = 0;
				shift_step = 20;	//최대 속도 유지
				Af_ct_avr = Current_actual;	//에러체크 후, 전류센서 값 저장(전류센서 값은 모터가 구동중일 때 저장함)
				break;
			case 3:	//CW 셀프체크 중(감속 시작)
				if(++t100us_cnt >= 100)
				{
					t100us_cnt = 0;
					shift_step--;
					if(shift_step <= 0)
						shift_step = 0;
				}
				break;
			case 4:	//CW 셀프체크 끝 and 방향 전환(1초 일시정지)
				Af_pm_avr = P_M_actual;//에러체크 후, 각도센서 값 저장
				Af_encoder_avr = diff;//에러체크 후, 엔코더 값 저장
				Change_pm_avr = fabs(Af_pm_avr - Bf_pm_avr);			//에러체크 전,후 각도센서 값 변화량
				Change_ct_avr = fabs(Af_ct_avr - Bf_ct_avr);			//에러체크 전,후 전류센서 값 변화량
				Change_encoder_avr = Af_encoder_avr - Bf_encoder_avr;	//에러체크 전,후 엔코더 값 변화량
				//각도센서 자가진단
				if((Change_pm_avr >= 10.0) && (Change_pm_avr <= 80.0))// 대략 20 ~50도 내의 범위로 셀프체크 동작
					CW_pm_check_ok = 1;	//각도센서 정상 
				else
					CW_pm_check_ok = 0;	//각도센서 비정상
				//전류센서 자가진단
				if((Change_ct_avr >= 150) && (Change_ct_avr <= 800))	//20 ~ 650mA 정상판단
					CW_ct_check_ok = 1;	//전류센서 정상 
				else
					CW_ct_check_ok = 0;	//전류센서 비정상
				//엔코더 자가진단				
				//check_dir = (CCW_priority_mode == 0) ? 'U' : 'D';
				check_dir = (CCW_priority_mode == 0) ? 'D' : 'U';//CCW_priority_mode가 0이면 check_dir에 U할당 아니면 D할당 
				//if((Change_encoder_avr >= 2000) && (Change_encoder_avr <= 4000) && (Encoder_dir == check_dir))	//diff값 2000~4000 정상판단
				
				// 실험결과 51.23도 -> 74.2도 Change_encoder_avr = 679, 실험결과 74.2도 -> 53.4도 Change_encoder_avr = 1020 
				if((Change_encoder_avr >= 300) && (Change_encoder_avr <= 1500) && (Encoder_dir == check_dir))	//22도 움직였을 때 diff값 500~1500 정상판단 24.05.24 수정하였음.
					CW_encoder_check_ok = 1;	//엔코더 정상							//회전방향 확인
				else
					CW_encoder_check_ok = 0;	//엔코더 비정상
				//에러체크 실험용
				if(k <= 39 && save_f)
				{
					save_f = 0;
					//angle_change[k++] = Change_load_avr;
				}
				t100us_cnt = 0;	//0.1ms 카운터 초기화
				shift_step = 0;	//shift_step 초기화
				check_step = 6;	//변화량 값을 한번만 계산하기 위해서
				break;
			case 6:	//셀프체크전 값 저장
				Bf_pm_avr = P_M_actual;		//에러체크하기 전에 각도센서 값 저장
				Bf_ct_avr = Current_actual;	//에러체크하기 전에 전류센서 값 저장
				Bf_encoder_avr = diff;		//에러체크하기 전에 엔코더 값 저장
				break;	
			case 7:	//CCW 셀프체크 시작(가속 시작)
				if(++t100us_cnt >= 100)
				{
					t100us_cnt = 0;
					shift_step++;
					if(shift_step >= 21)
						shift_step = 20;
				}
				break;
			case 8:	//CCW 셀프체크 중(가속 끝)
				t100us_cnt = 0;
				shift_step = 20;	//최대 속도 유지
				Af_ct_avr = Current_actual;	//에러체크 후, 전류센서 값 저장	(전류센서 값은 모터가 구동중일 때 저장함)
				break;
			case 9:	//CCW 셀프체크 중(감속 시작)
				if(++t100us_cnt >= 100)
				{
					t100us_cnt = 0;
					shift_step--;
					if(shift_step <= 0)
						shift_step = 0;
				}
				break;
			case 10://CCW 셀프체크 끝
				Af_pm_avr = P_M_actual;		//에러체크 후, 각도센서 값 저장
				Af_encoder_avr = diff;		//에러체크 후, 엔코더 값 저장
				Change_pm_avr = fabs(Bf_pm_avr - Af_pm_avr);			//에러체크 전,후 각도센서 값 변화량
				Change_ct_avr =	fabs(Af_ct_avr - Bf_ct_avr);			//에러체크 전,후 전류센서 값 변화량 fbas:절댓값
				Change_encoder_avr = Af_encoder_avr - Bf_encoder_avr;	//에러체크 전,후 엔코더 값 변화량
				//각도센서 자가진단
				if((Change_pm_avr >= 10.0) && (Change_pm_avr <= 80.0))// 대략 20 ~35도 내의 범위로 셀프체크 동작
					CCW_pm_check_ok = 1;	//각도센서 정상
				else
					CCW_pm_check_ok = 0;	//각도센서 비정상
				//전류센서 자가진단
				if((Change_ct_avr >= 50) && (Change_ct_avr <= 800))	//20 ~ 300mA 정상판단
					CCW_ct_check_ok = 1;//전류센서 정상
				else
					CCW_ct_check_ok = 0;//전류센서 비정상
				//엔코더 자가진단		
				//check_dir = (CCW_priority_mode == 0) ? 'D' : 'U';
				check_dir = (CCW_priority_mode == 0) ? 'U' : 'D';
				// 실험결과 51.23도 -> 74.2도 Change_encoder_avr = 679, 실험결과 74.2도 -> 53.4도 Change_encoder_avr = 1020 
				//if((Change_encoder_avr >= 2000) && (Change_encoder_avr <= 4000) && (Encoder_dir == check_dir))	//diff값 2000~4000 정상판단
				if((Change_encoder_avr >= 300) && (Change_encoder_avr <= 1500) && (Encoder_dir == check_dir))	//22도 움직였을 때 diff값 500~1500 정상판단 24.05.24 수정하였음.	
					CCW_encoder_check_ok = 1;	//엔코더 정상					//회전방향 확인
				else
					CCW_encoder_check_ok = 0;	//엔코더 비정상
				t100us_cnt = 0;	//1ms 카운터 초기화
				shift_step = 0;	//shift_step 초기화
				Restart_self_check = 0;
				Error_Check = 0;
				Start_self_check = 0;
				Self_check_end = 1;
				PWM_SLEEP;
				self_checking = 0;//셀프체크 움직임 동작 끝
				_SENSOR_DATA.state = 0x01;// 셀프체크가 끝나면 대기중 상태로 변경
				break;
			default:
				break;
		}
		//check_step에 따른 모터 PWM 제어
		if((check_step == 1) || (check_step == 3))	//CW 가속 또는 감속
		{
			if(CCW_priority_mode)
			{
				//반시계방향
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);           
				//htim4.Instance->CCR1 = 540 + 54*shift_step;  			//duty 30% ~ 60% (가속 또는 감속)//민수 수정
				Motor_PWM = 300 + 108*shift_step;  	//duty 30% ~ 60% (가속 또는 감속) // pwm초기값을 줄이면 그만큼 이동하는 각도의 범위가 줄어들어서 shift_step*x에서 x값을 늘림
				//Motor_PWM = 1080 + 54*shift_step;  			//duty 30% ~ 60% (가속 또는 감속) 
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);       
				htim4.Instance->CCR2 = 0;
				Motor_PWM_CCW = (unsigned int)(-0.5*Motor_PWM + 1800);
				TIM8 -> CCR1 = Motor_PWM_CCW; 				
			}
			else
			{
				//시계방향
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);              
				htim4.Instance->CCR1 = 	0; 								
				//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);       
				//htim4.Instance->CCR2 = 540 + 54*shift_step;  			//duty 30% ~ 60% (가속 또는 감속)
				Motor_PWM = 300 + 108*shift_step;	//duty 30% ~ 60% (가속 또는 감속) // pwm초기값을 줄이면 그만큼 이동하는 각도의 범위가 줄어들어서 shift_step*x에서 x값을 늘림
				//Motor_PWM = 1080 + 54*shift_step;  			//duty 30% ~ 60% (가속 또는 감속)
				Motor_PWM_CW = (unsigned int)(0.5*Motor_PWM + 1800);
				TIM8 -> CCR1 = Motor_PWM_CW; 				
			}
		}
		else if((check_step == 7) || (check_step == 9))	//CCW 가속 또는 감속
		{
			if(CCW_priority_mode)
			{
				//시계방향
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);              
				//htim4.Instance->CCR1 = 0;
				TIM4_CCR1_buffer = 0;
				//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);       
				//htim4.Instance->CCR2 = 540 + 54*shift_step;  			//duty 30% ~ 60% (가속 또는 감속)
				Motor_PWM = 300 + 108*shift_step;  	//duty 30% ~ 60% (가속 또는 감속) // pwm초기값을 줄이면 그만큼 이동하는 각도의 범위가 줄어들어서 shift_step*x에서 x값을 늘림
				//Motor_PWM = 1080 + 54*shift_step;  			//duty 30% ~ 60% (가속 또는 감속) 
				Motor_PWM_CW = (unsigned int)(0.5*Motor_PWM + 1800);
				TIM8 -> CCR1 = Motor_PWM_CW; 				
			}
			else
			{
				//반시계방향
				TIM4_CCR2_buffer = 0;
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);           
				//htim4.Instance->CCR1 = 540 + 54*shift_step;  			//duty 30% ~ 60% (가속 또는 감속)
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);       
				//htim4.Instance->CCR2 = 0;
				Motor_PWM = 300 + 108*shift_step;  //duty 30% ~ 60% (가속 또는 감속) // pwm초기값을 줄이면 그만큼 이동하는 각도의 범위가 줄어들어서 shift_step*x에서 x값을 늘림
				//Motor_PWM = 1080 + 54*shift_step;  		//duty 30% ~ 60% (가속 또는 감속)
				Motor_PWM_CCW = (unsigned int)(-0.5*Motor_PWM + 1800);
				TIM8 -> CCR1 = Motor_PWM_CCW; 			
			} 
		}
		else if((check_step == 6) || (check_step == 10))	//방향 전환(1초 일시정지) 또는 정지
		{
			//일시정지
			Motor_PWM_CCW = 1800;
			Motor_PWM_CW = 1800;
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);           
			htim4.Instance->CCR1 = 0;  		
			//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);       
			htim4.Instance->CCR2 = 0;
			TIM8 -> CCR1 = 1800; 	
			if(check_step == 10)
			{
				check_step = 0;
				PWM_SLEEP;
			}	
		}
	}	
	/*
	else
	{
		error_check_cnt = 0;
		//에러체크 실험용
		if(record_en && k <= 40)
			retry_f = 1;	//지워야함
	}
	//에러체크 실험용
	if(retry_f == 1)
	{
		Error_Check = 1;
		retry_f = 0;
	}
*/
	/*****************************************추가한 부분(도우)*************************************************/

	if(Ble_Rcv_ok)
		Ble_Rcv_ok = (Ble_Rcv_ok<<4) | Ble_Rcv_ok; // 0x03 : 0x33	
	
	/*****************************************추가한 부분(도우)*************************************************/
}//타이머 3인터럽트문 끝
void TIM5_IRQHandler(void)
{//0.01ms타이머 시간 측정용
  /* USER CODE BEGIN TIM5_IRQn 0 */
	TIM5->SR &= ~TIM_IT_UPDATE;
  /* USER CODE END TIM5_IRQn 0 */
	if(TIM5->SR & TIM_IT_UPDATE) 
		TIM5->SR &= ~TIM_IT_UPDATE; // 인터럽트 플래그 클리어
  /* USER CODE BEGIN TIM5_IRQn 1 */

  /* USER CODE END TIM5_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
/*****************************************추가한 부분(도우)*************************************************/
byte Rx2_index = 0, Rx2_next_data_no = 0;
word LoRa_CheckSum_data;
word Error2_cnt = 0;
byte Rx2_step = 0;

byte Rx2_CRC_H = 0, Rx2_CRC_L = 0;
byte LoRa_CheckSum_ing = 0;
byte LoRa_CheckSum_EN = 0;

word LoRa_Rcv_ok = 0;

word LoRa_test_crc1 = 0, LoRa_test_crc2 = 0;
byte LoRa_Tx_buf[255] ={0x01, 0x04, 0x02, 0x00, 0x0A, 0xF8, 0xF4};
byte LoRa_Tx_index=0;
byte LoRa_Tx_send_number=0;
byte Rx2_RS_L = 0, Rx2_RS_H = 0;


byte lora_test_index[256];
uint8_t rx2_index;
void USART2_IRQHandler(void)//로라
{
  /* USER CODE BEGIN USART2_IRQn 0 */
	byte rx2_data;
	if((USART2->CR1 & USART_CR1_RXNEIE) && (USART2-> SR & USART_SR_RXNE))//수신버퍼가 채워졌을 때
    {
        Rcv2_cnt++;
        rx2_data = USART2->DR;	// 수신값 저장

		lora_test_index[rx2_index++] = rx2_data;
		
		if(LoRa_CheckSum_ing == 0)
		{
		  switch(Rx2_step)
		  {
		   case 0 :
			//24.08.05
			if(rx2_data == ENQ && rx2_data != 0x00)// 0번 인덱스 = 국번이 같고 국번이 0이 아니라면
			{
			  LoRa_Rxbuf[0] = rx2_data;
			  Rx2_step = 1;
			  Rx2_index = 1;
			}
			else
			{
			  Rx2_step = 0;
			  Rx2_index = 0;
			  // 1ms 카운터
			}
			break;
		   case 1:
			if((rx2_data == 0x04 || rx2_data == 0x10))//function 0x04 : Read Request
			{										  //function 0x10 : Write Request 
			  LoRa_Rxbuf[Rx2_index++] = rx2_data;
			  Rx2_step++;
			}
			else
			{
			  Rx2_step = 0;
			  Rx2_index = 0;
			}
			break;
			
		   case 2: // Data Address[High]
			LoRa_Rxbuf[Rx2_index++] = rx2_data;
			Rx2_step++;
			break;
			
		   case 3: // Data Address[Low]
		  	LoRa_Rxbuf[Rx2_index++] = rx2_data;
		  	Rx2_step++;
			break;
			
		   case 4: // Quantity of input registers[High]
			LoRa_Rxbuf[Rx2_index++] = rx2_data;
			Rx2_step++;
			break;
			
		   case 5: // Quantity of input registers[Low]
			LoRa_Rxbuf[Rx2_index++] = rx2_data;
			if(LoRa_Rxbuf[1] == 0x10)
			{
			  Rx2_step = 10; // Wtrite n개 data 수신 mode
			  Rx2_next_data_no = LoRa_Rxbuf[5]*2; // Write n개 data number
			}
			else
			{
			  Rx2_step++; //CRC16_Low 수신
			  Rx2_next_data_no = 0;
			}
			break;
			
		   case 6: // CRC16 Low
			LoRa_Rxbuf[Rx2_index++] = rx2_data;
			Rx2_step++;
			break;
			
		   case 7: // CRC16 High
			LoRa_Rxbuf[Rx2_index] = rx2_data;
			Rx2_CRC_H = LoRa_Rxbuf[Rx2_index];
			Rx2_CRC_L = LoRa_Rxbuf[Rx2_index-1];
			Rx2_step++;
			break;
			
		   case 8 : // 수신 감도를 Slave LoRa로 부터 받아옴(Low)
			Rx2_RS_L = rx2_data;
			Rx2_step++;
			break;
			
		   case 9: // 수신 감도를 Slave LoRa로 부터 받아옴(High)
			Rx2_RS_H = rx2_data;
			_LoRa_Data.reception_sensitivity = (Rx2_RS_L<<8) | Rx2_RS_H;	
			LoRa_CheckSum_EN = 1; // main.c 에서 CRC16 계산
			LoRa_CheckSum_ing = 1; // UART2 수신 불가능
			Rx2_step = 0;
			Rx2_index = 0;
			break;
			
		   default:
			Rx2_step = 0;
			Rx2_index = 0;
			break;
			
		   case 10: // Write data 수신
			LoRa_Rxbuf[Rx2_index++] = rx2_data;
			if(Rx2_next_data_no <= 0)
			{
			  Rx2_step = 6; // CRC16_Low 수신
			}
			else
			{
			  Rx2_next_data_no--;
			}
			break;
		  }
		}
	}
	else if((USART2->CR1 & USART_CR1_TCIE) && (USART2-> SR & USART_CR1_TCIE))	//송신버퍼가 채워졌을 때 /// 송신 되는지 확인할것!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	{
		static byte Tx2_cnt=0;
		Tx2_cnt++;
		if(Tx2_send_number)
		{
			Tx2_send_number--;  
			USART2->DR = LoRa_Tx_buf[Tx2_index++];
			//USART2->DR = test_rxbuf[test_index++];
		    //USART2->DR = LoRa_test_buf[Tx2_index++];
		}
		else	//모든 Data 전송
		{
			USART2->CR1 |= USART_CR1_RXNEIE; 	// UART2's RXE Interrupt Enable		
			USART2->CR1 &= ~USART_CR1_TCIE;		// UART2's TXE Interrupt Disable
			/*for(int reset_lora_tx_buf = 0; reset_lora_tx_buf < sizeof(LoRa_Tx_buf); reset_lora_tx_buf++)
			{
			  LoRa_Tx_buf[reset_lora_tx_buf] = '\0';
			}*/
		}
		return;	
	}
  /* USER CODE END USART2_IRQn 0 */
  //HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */
  /* USER CODE END USART2_IRQn 1 */
}

/*****************************************추가한 부분(도우)*************************************************/
word BCC_error_cnt = 0,LoRa_BCC_error_cnt = 0; byte Tx_lora_data = 0;
void LoRa_UART2_exe(void)		//Call by main.c (UART2)
{
	byte i, k, no;
	word wtemp;
	byte *bptr_lora;
	if(LoRa_Rcv_ok == 0x110)	//쓰기 ACK
	{
		LoRa_Rcv_ok = 0;  
		bptr_lora = (byte *)(0x20000000);
		bptr_lora = (byte *)(0x20000000 + (LoRa_Rxbuf[2]<<8) + LoRa_Rxbuf[3]);		 	
		//bptr_lora = (byte *)(LORA_MEMORY);
		if(!Motor_BUSY)		//모터가 일시정지 Soft stop중일 때는 구조체에 데이터 쓰지 않음
		{
			no = LoRa_Rxbuf[5]*2;	//Write number
			for(i = 0; i < no; i++)
			{
				*bptr_lora = LoRa_Rxbuf[7+i];	//n개 write data...
				bptr_lora++;
			}
			
			Tx_lora_data = 1;
			/*if(!Priority_dir_set)	
				Priority_dir_set = 1;	//셀프체크 방향 설정시작
			/*****쓰기 완료*****/
		}

		if(_LoRa_Data.state == 1 && _LoRa_Data.measure_check == 0) // 모터 드라이버가 대기중일때만 PC를 이용해 데이터를 쓸수 있음.
		{
			//ACK Data Sending
			LoRa_Tx_buf[0] = LoRa_Rxbuf[0];	//Station(국번 1byte) 0x01
			LoRa_Tx_buf[1] = LoRa_Rxbuf[1];	//Function(함수 1byte) 쓰기 : 0x10			
			LoRa_Tx_buf[2] = LoRa_Rxbuf[2];	//High Address(1byte) 쓰기 시작할 번지
			LoRa_Tx_buf[3] = LoRa_Rxbuf[3];	//Low Adddress(1byte)
			LoRa_Tx_buf[4] = LoRa_Rxbuf[4];	//Length High(1byte) 쓰기 갯수
			LoRa_Tx_buf[5] = LoRa_Rxbuf[5];	//Length Low(1byte)
			
			LoRa_CheckSum_data = CRC16(LoRa_Tx_buf, 6);	//6개 데이터의 CRC16 계산 
					
			LoRa_Tx_buf[6] = LoRa_CheckSum_data & 0xFF;	    //CRC's Low byte	
			LoRa_Tx_buf[7] = (LoRa_CheckSum_data >> 8);		//CRC's High byte	
			
			LoRa_RS232_putchar(8);		//Total 8 byte sending	
			copyLoRaDataToSensorData();//로라 데이터 -> 센서 데이터
		}
		else if(_LoRa_Data.state == 0 || _LoRa_Data.state == 2) // 모터 드라이버가 운동중이거나 고장이면 PC를 이용해 데이터를 쓸수 없으며 예외 코드를 PC로 전송.
		{
		  LoRa_Tx_buf[0] = LoRa_Rxbuf[0];	//Station(국번 1byte) 0x01
		  LoRa_Tx_buf[1] = LoRa_Rxbuf[1] | 0x80;	//The Function Code ( 0x10 + 0x80 = 0x90 )
		  LoRa_Tx_buf[2] = 0x01;	//The Exception Code( 예외 코드 0x01 : 슬레이브가 이 유형의 요청을 처리할 수 없는 잘못된 상태에 있음을 나타냄. )
		  
		  LoRa_CheckSum_data = CRC16(LoRa_Tx_buf, 3);	//3개 데이터의 CRC16 계산 
		  
		  LoRa_Tx_buf[3] = LoRa_CheckSum_data & 0xFF;	    //CRC's Low byte	
		  LoRa_Tx_buf[4] = (LoRa_CheckSum_data >> 8);		//CRC's High byte	
		  
		  LoRa_RS232_putchar(5);		//Total 5 byte sending	
		}
		
		if(_LoRa_Data.measure_check == 1)
		{
		  _LoRa_Data.measure_check = 0;
		  _LoRa_Data.use_state = 0x01;
		  _LoRa_Data.upper_limit_angle = 0;
		  _LoRa_Data.lower_limit_angle = 0;
		  //ACK Data Sending
		  LoRa_Tx_buf[0] = LoRa_Rxbuf[0];	//Station(국번 1byte) 0x01
		  LoRa_Tx_buf[1] = LoRa_Rxbuf[1];	//Function(함수 1byte) 쓰기 : 0x10			
		  LoRa_Tx_buf[2] = LoRa_Rxbuf[2];	//High Address(1byte) 쓰기 시작할 번지
		  LoRa_Tx_buf[3] = LoRa_Rxbuf[3];	//Low Adddress(1byte)
		  LoRa_Tx_buf[4] = LoRa_Rxbuf[4];	//Length High(1byte) 쓰기 갯수
		  LoRa_Tx_buf[5] = LoRa_Rxbuf[5];	//Length Low(1byte)

		  LoRa_CheckSum_data = CRC16(LoRa_Tx_buf, 6);	//6개 데이터의 CRC16 계산 

		  LoRa_Tx_buf[6] = LoRa_CheckSum_data & 0xFF;	    //CRC's Low byte	
		  LoRa_Tx_buf[7] = (LoRa_CheckSum_data >> 8);		//CRC's High byte	

		  LoRa_RS232_putchar(8);		//Total 8 byte sending	
		}		
	}
	else if(LoRa_Rcv_ok == 0x44) // 읽기 ACK
	{
		LoRa_Rcv_ok = 0;
		LoRa_Tx_buf[0] = LoRa_Rxbuf[0];	//Station(국번 1byte) 0x01
		LoRa_Tx_buf[1] = LoRa_Rxbuf[1];	//Function(함수 1byte) 읽기 : 0x04	
		
		if(LoRa_Rxbuf[1] == 0x04)	//읽기 요청 함수코드(0x04)
		{
			LoRa_Rcv_ok = 0;
			LoRa_Tx_buf[0] = LoRa_Rxbuf[0];	//Station(국번 1byte) 0x01
			LoRa_Tx_buf[1] = LoRa_Rxbuf[1];	//Function(함수 1byte) 읽기 : 0x04	
			
			LoRa_Tx_buf[2] = LoRa_Rxbuf[5]*2;    // The number of data bytes
				
			wtemp = (LoRa_Rxbuf[2]<<8) + LoRa_Rxbuf[3];	//Read Address	 
			bptr_lora = (byte *)(LORA_MEMORY + (LoRa_Rxbuf[2]<<8) + LoRa_Rxbuf[3]);	
			
			i = 3;
			for(k = 0; k < LoRa_Tx_buf[2]; k++)		//LoRa_Rxbuf[5] = Read number
			{
				LoRa_Tx_buf[i++] = *bptr_lora++;
			}
			//CRC(Hex)
			LoRa_CheckSum_data = CRC16(LoRa_Tx_buf, i);	//송신할 데이터의 CheckSum계산 
			
			LoRa_Tx_buf[i++] = LoRa_CheckSum_data & 0xFF;	    //CRC's Low byte	
			LoRa_Tx_buf[i++] = (LoRa_CheckSum_data >> 8);		//CRC's High byte	
			
			LoRa_RS232_putchar(i);		//Total i byte sending	
		}
	}
	else if(LoRa_Rcv_ok==0xEE)	//BCC Error
	{
		LoRa_Rcv_ok = 0;
		BCC_error_cnt++;
		LoRa_Tx_buf[0] = LoRa_Rxbuf[0];	//국번	
		LoRa_Tx_buf[1] = LoRa_Rxbuf[1];	//함수
		LoRa_Tx_buf[2] = LoRa_Rxbuf[5];	//data length(low_length)
		LoRa_Tx_buf[3] = 0xEE;			//Error Cmd	(0xEE:CRC Error)
		LoRa_Tx_buf[4] = 0xEE;			//Error Cmd	(0xEE:CRC Error)
		LoRa_Tx_buf[5] = 0xEE;			//Error Cmd	(0xEE:CRC Error)

		wtemp = CRC16(LoRa_Tx_buf, 6);	//총 6byte [0]~[5]
		LoRa_Tx_buf[6] = wtemp & 0xFF;		//CRC's Low byte	
		LoRa_Tx_buf[7] = (wtemp>>8);		//CRC's High byte	
		LoRa_RS232_putchar(8);				//Total 8 byte sendding	[0]~[7] Total 8 byte sending.
	}	
}
void LoRa_RS232_putchar(byte lora_tx_num)	//USART2
{
	/*Send the character*/
	USART2->DR = LoRa_Tx_buf[0];
    //USART2->DR = LoRa_test_buf[0];
	Tx2_index = 1;
	Tx2_send_number = lora_tx_num-1;
	USART2->CR1 &= ~USART_CR1_RXNEIE; 	//USART2's RXE Interrupt Disable	
	USART2->CR1 |= USART_CR1_TCIE; 		//USART2's TXE Interrupt Enable	
}
/*****************************************추가한 부분(도우)*************************************************/
byte Rx3_buf[100];
byte Rx3_index = 0, Rcv3_cnt = 0, Rx4_next_data_no;
word CheckSum_data;
byte Tx3_send_number=0;
byte Tx3_index=0;
byte Rx3_step = 0;

void USART3_IRQHandler(void)
{/* USER CODE BEGIN UART3_IRQn 0 */
	byte rx3_data;
    if((USART3->CR1 & USART_CR1_RXNEIE) && (USART3-> SR & USART_SR_RXNE))	// 수신버퍼가 채워졌다면?	
    {
		Rcv3_cnt++;
        rx3_data = USART3->DR;	// 수신값 저장
        ble_test_rxbuf[ble_test_index++] = rx3_data;
		switch(Rx3_step) 
		{
			case 0:
				if(rx3_data == 'R') 
				{
					Rx3_buf[0] = rx3_data;	//Rx_Buf[0] = 국번(0x01)
					Rx3_step = 1;
					Rx3_index = 1;
				}
				else
				{
					Rx3_step = 0;
					Rx3_index = 0;
				}
				break;
			case 1: //함수 
				if(rx3_data == 'D') 
				{															 	   
					Rx3_buf[Rx3_index] = rx3_data;
					Ble_CheckSum_EN = 1;
					Rx3_step = 0;
					Rx3_index = 0;
					ble_test_index = 0;
				}
				else                                                    
				{
					Rx3_step = 0;
					Rx3_index = 0;
					ble_test_index = 0;
				}
				break;
				
			default:
				Rx3_step = 0;
				Rx3_index = 0;
				break;
		}
    }
    // UART in mode Transmitter (transmission end) -----------------------------
	else if((USART3->CR1 & USART_CR1_TCIE) && (USART3-> SR & USART_CR1_TCIE))
	{
		static byte Tx3_cnt=0;
		Tx3_cnt++;
		if(Tx3_send_number)
		{
			Tx3_send_number--;  
			USART3->DR = Ble_buffer[Tx3_index++];
		}
		else	//모든 Data 전송 하였음.
		{
			USART3->CR1 |= USART_CR1_RXNEIE; 	// UART2's RXE Interrupt Enable		
			USART3->CR1 &= ~USART_CR1_TCIE;		// UART2's TXE Interrupt Disable
		}
		return;	
	}	

  /* USER CODE END UART4_IRQn 0 */
  //HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN UART4_IRQn 1 */

  /* USER CODE END UART4_IRQn 1 */
}
void Ble_Data_Tx(byte ble_tx_num)	//UART3
{
	/*Send the character*/
	USART3->DR = Ble_buffer[0];
	Tx3_index = 1;
	Tx3_send_number = ble_tx_num-1;
	USART3->CR1 &= ~USART_CR1_RXNEIE; 	//USART3's RXE Interrupt Disable	
	USART3->CR1 |= USART_CR1_TCIE; 		//USART3's TXE Interrupt Enable	
}
typedef union {//float 값을 byte 배열로 변환
    float float_data;
    byte byte_data[4];
} FloatUnion;
byte byte_buffer[4] = {0};
void Ble_UART3_exe(void)//Call by main.c (UART3)
{
	if(Ble_Rcv_ok == 0x33)	
	{
		Ble_Rcv_ok = 0; 

		Ble_buffer[0] = _Ble_Data.current_angle;	
		
		Ble_buffer[1] = _Ble_Data.upper_angle;
		Ble_buffer[2] = _Ble_Data.lower_angle;
		Ble_buffer[3] = _Ble_Data.stop_time;
		Ble_buffer[4] = _Ble_Data.motion;
		
		Ble_buffer[5] = _Ble_Data.mode;
		Ble_buffer[6] = _Ble_Data.velocity_mode;
		Ble_buffer[7] = _Ble_Data.exerc_time_and_exercnum;
		Ble_buffer[8] = _Ble_Data.upper_limit_angle;
		
		Ble_buffer[9] =  _Ble_Data.lower_limit_angle;
		Ble_buffer[10] = _Ble_Data.remain_time_and_cnt;
		Ble_buffer[11] = _Ble_Data.special_angle;
		Ble_buffer[12] = _Ble_Data.repeat_num;
		
		Ble_buffer[13] = _Ble_Data.exerc_time;
		Ble_buffer[14] = _Ble_Data.exerc_num;
		Ble_buffer[15] = _Ble_Data.exer_state;
		
		Ble_Data_Tx(16);		//블루투스 데이터 전송(Total 16 byte sending)	
		
		if(_Ble_Data.exer_state == 1 || _Ble_Data.exer_state == 2) // 운동 종료이거나 측정이 완료되었을때, 신호를 한번 보내고 다시 0으로 만든다.
		{
		  _Ble_Data.exer_state = 0;// 평상시
		}
	}
}
/**
  * @brief This function handles UART4 global interrupt.
  */
byte Rx4_buf[100];
byte Rx4_index = 0, Rcv4_cnt = 0, Rx4_next_data_no;
word CheckSum_data;
byte Tx4_buf[100] ={0x01, 0x04, 0x00, 0x0A, 0x00, 0x01, 0x11, 0xC8};
byte Tx4_send_number=0;
byte Tx4_index=0;
word Error4_cnt = 0;
byte Rx4_step = 0;
dword uart4_interrupt_scan = 0;	//UART4 인터럽트 카운터
byte Rx4_CRC_H = 0, Rx4_CRC_L = 0;
byte CheckSum_ing = 0;
//word Rx4_data_sum = 0; //24.04.11 제거함
void UART4_IRQHandler(void)
{
  /* USER CODE BEGIN UART4_IRQn 0 */
	byte rx4_data;
    uart4_interrupt_scan++;
    if((UART4->CR1 & USART_CR1_RXNEIE) && (UART4-> SR & USART_SR_RXNE))	// 수신버퍼가 채워졌다면?		원본
    {
        Rcv4_cnt++;
        rx4_data = UART4->DR;	// 수신값 저장
        test_rxbuf[test_index++] = rx4_data;
		
		if(CheckSum_ing == 0)
		{
			switch(Rx4_step) 
			{
				case 0:
					if(rx4_data == UART_STX) //국번(0x01)이 일치?
					{
						//Rx4_data_sum = 0; //24.04.11 제거함
						Rx4_buf[0] = rx4_data;	//Rx_Buf[0] = 국번(0x01)
						Rx4_step = 1;
						Rx4_index = 1;
						//Rx4_data_sum += rx4_data; //24.04.11 제거함
					}
					else
					{
						Rx4_step = 0;
						Rx4_index = 0;
					}
					break;
				case 1: //함수 
					if((rx4_data == 0x04) || (rx4_data == 0x10) || (rx4_data == 0x06) || (rx4_data == 0x08)) //function 0x04: Read Request
					{															 	   						//function 0x10: Write Request
															 						  						//function 0x06: Calibration mode, function 0x08: voice mode 
						Rx4_buf[Rx4_index++] = rx4_data;
						Rx4_step++;
						//Rx4_data_sum += rx4_data; //24.04.11 제거함
					}
					else                                                    
					{
						static word ng_cnt = 0;
						ng_cnt++;
						Rx4_step = 0;     //Waiting for  국번
						Rx4_index = 0;
					}
					break;
				case 2: //Waiting Address (High Address byte)
					Rx4_buf[Rx4_index++] = rx4_data;
					Rx4_step++;
					//Rx4_data_sum += rx4_data; //24.04.11 제거함
					break;
				
				case 3: //Waiting Address  (Low Address byte)
					if(rx4_data == 0xEE)
					{						
						Rx4_step = 0;     //Waiting for  국번
						Rx4_index = 0;
						//Rx4_data_sum += rx4_data; //24.04.11 제거함
					}
					else                                                    
					{
						Rx4_buf[Rx4_index++] = rx4_data;
						Rx4_step++;
					}
					Error4_cnt++;
					break;	
				
				case 4: //Waiting 길이 (High Length byte)
					Rx4_buf[Rx4_index++] = rx4_data;
					Rx4_step++;
				
					//Rx4_data_sum += rx4_data; //24.04.11 제거함
					break;
					
				case 5://Waiting 길이 (Low Length byte)
					Rx4_buf[Rx4_index++] = rx4_data;
					Rx4_step++;
					if(Rx4_buf[1]==0x04)
						Rx4_step=7;
					//Rx4_data_sum += rx4_data; //24.04.11 제거함
					break;
					
				case 6: //쓰기 요청할 데이터 수 (The number of data bytes)
					Rx4_buf[Rx4_index++] = rx4_data;
					//Rx4_data_sum += rx4_data;//24.04.11 제거함
					if((Rx4_buf[1] == 0x10) || (Rx4_buf[1] == 0x06) ||(Rx4_buf[1] == 0x08))	//Write or Calibration command
					{

						Rx4_step = 10;		//Write n개 data 수신 mode
						Rx4_next_data_no = Rx4_buf[5]*2;//Write n개 data number	
					}
					else
					{
						Rx4_step++;		
						Rx4_next_data_no = 0;
					}
					break;	
					
				case 7: //Waiting for CRC16 (Low byte)	
					Rx4_buf[Rx4_index++] = rx4_data;	
					Rx4_step++;
					//Rx4_data_sum += rx4_data; //24.04.11 제거함
					break;

				case 8: //Waiting for CRC16 (High byte)	
					Rx4_buf[Rx4_index] = rx4_data;	
					CheckSum_EN = 1;
					CheckSum_ing = 1;
					Rx4_CRC_L = Rx4_buf[Rx4_index-1];
					Rx4_CRC_H = Rx4_buf[Rx4_index];

					
				default:
					Rx4_step = 0;
					Rx4_index = 0;
					//Rx4_data_sum = 0;
					break;

				case 10:	//Write data 수신
					Rx4_buf[Rx4_index++] = rx4_data; //데이터 뿌림
					//Rx4_data_sum += rx4_data; //24.04.11 제거함
					if(Rx4_next_data_no <= 1)
					{
						Rx4_step = 7;	//CRC16_Low 수신
					}
					else
						Rx4_next_data_no--;
					break;
			}
		}
    }	
        // UART in mode Transmitter (transmission end) -----------------------------
	else if((UART4->CR1 & USART_CR1_TCIE) && (UART4-> SR & USART_CR1_TCIE))
	{
		static byte Tx4_cnt=0;
		Tx4_cnt++;
		if(Tx4_send_number)
		{
			Tx4_send_number--;  
			UART4->DR = Tx4_buf[Tx4_index++];
		}
		else	//모든 Data 전송 하였음.
		{
			UART4->CR1 |= USART_CR1_RXNEIE; 	// UART2's RXE Interrupt Enable		
			UART4->CR1 &= ~USART_CR1_TCIE;		// UART2's TXE Interrupt Disable
		}
		return;	
	}	
	else
		Error4_cnt++;

  /* USER CODE END UART4_IRQn 0 */
  //HAL_UART_IRQHandler(&huart4);
  /* USER CODE BEGIN UART4_IRQn 1 */

  /* USER CODE END UART4_IRQn 1 */
}
/* USER CODE BEGIN 1 */
word Rcv2_ok = 0;
// 도우가 추가함
word Sw_Reset_Fg = 0;
void UART4_exe(void)		//Call by main.c (UART4)
{
	byte i, j, k, no;
	word wtemp;
	byte *bptr;
	if(Rcv2_ok == 0x110)	//쓰기 ACK
	{
		Rcv2_ok = 0;  
		bptr = (byte *)(USER_RUN_ADDRESS + (Rx4_buf[2]<<8) + Rx4_buf[3]);	//주소설정	
		if(!Motor_BUSY)//모터가 일시정지 Soft stop중일 때는 구조체에 데이터 쓰지 않음
		{
			no = Rx4_buf[5]*2;	//Write number
			for(i = 0; i < no; i++)
			{
				*bptr = Rx4_buf[7+i];	//n개 write data...
				bptr++;
			}
			/*if(!Priority_dir_set)	
				Priority_dir_set = 1;	//셀프체크 방향 설정시작*/
			
			//24.09.13 부하 민감도 설정시 셀프체크 방지용 
			if(!Priority_dir_set && _USER_RUN_SETTING.calibration != 2)
			{
				Priority_dir_set = 1;	//셀프체크 방향 설정시작
			}
 
			/*****쓰기 완료*****/
		}
		_SENSOR_DATA.cnt = 0;			//운동횟수 초기화
		_SENSOR_DATA.remain_time = 0;	//남은 운동시간 초기화
		//ACK Data Sending
		Tx4_buf[0] = Rx4_buf[0];	//Station(국번 1byte) 0x01
		Tx4_buf[1] = Rx4_buf[1];	//Function(함수 1byte) 쓰기 : 0x05			
		Tx4_buf[2] = Rx4_buf[2];	//High Address(1byte) 쓰기 시작할 번지
		Tx4_buf[3] = Rx4_buf[3];	//Low Adddress(1byte)
		Tx4_buf[4] = Rx4_buf[4];	//Length High(1byte) 쓰기 갯수
		Tx4_buf[5] = Rx4_buf[5];	//Length Low(1byte)
		
		CheckSum_data = CRC16(Tx4_buf, 6);	//6개 데이터의 CheckSum계산 
		Tx4_buf[6] = CheckSum_data & 0xFF;	    //CRC's Low byte	
		Tx4_buf[7] = (CheckSum_data >> 8);		//CRC's High byte	
		
		RS232_putchar(8);		//Total 8 byte sending 데이터 전송		
		
		if(LoRa_Equipment_Num)// 모터드라이버 최초 전원 On시 LoRa_Equipment_Num이 1 딱 한번만 초기 국번 값을 모터드라이버에서 Slave_LoRa로 보낸다.
		{
		  LoRa_Equipment_Num = 0;
		  ENQ = _USER_RUN_SETTING.equipment_num; // 국번 변수에 LCD의 EEPROM에 저장되어있는 장비번호를 대입
		  LoRa_Tx_buf[0] = _USER_RUN_SETTING.equipment_num;	//최초 전원 On시 Slave LoRa 국번을 보냄
			
		  LoRa_RS232_putchar(1);		//Total i byte sending	
		  Bf_Equipment_Num = _USER_RUN_SETTING.equipment_num;
		  
		  if(ENQ < 0x64) // 100보다 작으면
		  {
		  	sprintf((char*)AT_BTNAME, "AT+NAMEARTUS-840-%02d", ENQ);
		  }
		  else if(ENQ >= 0x64) // 100보다 크거나 같으면
		  {
			sprintf((char*)AT_BTNAME, "AT+NAMEARTUS-840-%03d", ENQ);
		  }
		  ble_name_change_fg = 1; // 블루투스 이름 변경
		}
	}
	else  if(Rcv2_ok == 0x44) // 읽기 ACK
	{
		Rcv2_ok = 0;
		bptr = (byte *)(SENSOR_ADDRESS + Rx4_buf[3]); //주소설정	 
		Tx4_buf[0] = Rx4_buf[0];//Station(국번 1byte) 0x01
		Tx4_buf[1] = Rx4_buf[1];//Function(함수 1byte) 읽기 : 0x04	
		if(Rx4_buf[1] == 0x04)	//읽기 요청 함수코드(0x04)
		{
			Rcv2_ok = 0;
			Tx4_buf[0] = Rx4_buf[0];	//Station(국번 1byte) 0x01
			Tx4_buf[1] = Rx4_buf[1];	//Function(함수 1byte) 읽기 : 0x04	
			
			Tx4_buf[2] = Rx4_buf[5] * 2;    //Master에서 읽기 요청한 데이터 수(단위 : word)

			wtemp = (Rx4_buf[2]<<8) + Rx4_buf[3];	//Read Address	 
			bptr = (byte *)(SENSOR_ADDRESS + wtemp);
			
			i = 3;
			for(k = 0; k < Tx4_buf[2]; k++)		//Rx4_buf[5] = Read number
			{
				Tx4_buf[i++] = *bptr++;
			}
			//CRC(Hex)
			CheckSum_data = CRC16(Tx4_buf, i);	//송신할 데이터의 CheckSum계산 
			Tx4_buf[i++] = CheckSum_data & 0xFF;	    //CRC's Low byte	
			Tx4_buf[i++] = (CheckSum_data >> 8);		//CRC's High byte	
			
			RS232_putchar(i);		//Total i byte sending	
							
			if(_SENSOR_DATA.upper_limit_angle || _SENSOR_DATA.lower_limit_angle)
			{
				_SENSOR_DATA.upper_limit_angle = 0;
				_SENSOR_DATA.lower_limit_angle = 0;
			}
			/**07.02 _SENSOR_DATA.lora_wr_fg이 0이면 LCD로 PC데이터를 보내지 않음.**/
			if(_SENSOR_DATA.lora_wr_fg)
			{
			  	_SENSOR_DATA.lora_wr_fg = 0;
			}
			
			//도우가 추가함
			static byte succed_tx = 0;
			succed_tx = 1;//stop, switch 정보를 lcd로 전송 완료  하면 스위치 관련 데이터를 초기화 시켜줌
			if(!Succed_Tx_Cnt) // Sw 정보를 500ms후에 삭제 함. 
			{
				if(succed_tx)//lcd로 start, stop 스위치 신호 성공적으로 보냄
				{
					succed_tx = 0;
					_SENSOR_DATA.stop_sw = 0;
					_SENSOR_DATA.start_sw = 0;
				}
			}
			
			//09.20 도우가 함
			static byte succed_ems_tx = 0;
			succed_ems_tx = 1;//비상 정지 스위치 정보를 lcd로 전송 완료 하면 스위치 관련 데이터를 초기화 시켜줌
			if(!TX_EMS_CNT) // EMS_Sw 정보를 500ms후에 삭제 함. 
			{
				if(succed_ems_tx)//lcd로 EMS_Sw 스위치 신호를 성공적으로 보냄
				{
					succed_ems_tx = 0;
					EMS_STATE = 0; // EMS 스위치 상태를 안눌림 상태로 변경
				}
			}
		}
	}
	else if(Rcv2_ok==0x66)	//Calibration ACK
	{
		Rcv2_ok = 0;
		//쓰기부분
		bptr = (byte *)(USER_RUN_ADDRESS + (Rx4_buf[2]<<8) + Rx4_buf[3]);		
		no =  Rx4_buf[5] * 2;	//Write number
		for(i = 0; i < no; i++)
		{
			*bptr = Rx4_buf[7+i];	//n개 write data...
			bptr++;
		}
		_USER_RUN_SETTING.exerc_start = 0x00;
		
		
		Tx4_buf[0] = Rx4_buf[0];	//Station(국번 1byte) 0x01
		Tx4_buf[1] = Rx4_buf[1];	//Function(함수 1byte) Calibration : 0x06
		Tx4_buf[2] = Rx4_buf[2]; 	//High Address(1byte)
		Tx4_buf[3] = Rx4_buf[3];	//Low Adddress(1byte)
		
		//읽기부분
		wtemp = (Rx4_buf[2]<<8) + Rx4_buf[3];	//Read Address	 
		bptr = (byte *)(_CALIBRATION_ADDRESS + wtemp);	
		
		//AD_ok 0으로 초기화하는 부분
		static byte bf_ad_start = 0;
		if(bf_ad_start != _USER_RUN_SETTING.ad_start)
		{
			if(_CALIBRATION_SETTING.AD_ok == 1)
				_CALIBRATION_SETTING.AD_ok = 0;
			bf_ad_start = _USER_RUN_SETTING.ad_start;
		}
		
		j = 4;
		for(k = 0; k < sizeof(_CALIBRATION_SETTING); k++)	//Read number(28byte)
		{
			Tx4_buf[j++] = *bptr++;
		}
		//CRC(Hex)
		CheckSum_data = CRC16(Tx4_buf, j);	//송신할 데이터의 CheckSum계산 
		
		Tx4_buf[j++] = CheckSum_data & 0xFF;	    //CRC's Low byte	
		Tx4_buf[j++] = (CheckSum_data >> 8);		//CRC's High byte	
		
		RS232_putchar(j);		//Total j byte sending	
	}

	else if(Rcv2_ok==0xEE)	//BCC Error
	{
		Rcv2_ok = 0;

		BCC_error_cnt++;
		Tx4_buf[0] = Rx4_buf[0];	//국번	
		Tx4_buf[1] = Rx4_buf[1];	//함수
		Tx4_buf[2] = Rx4_buf[5];	//data length(low_length)
		Tx4_buf[3] = 0xEE;			//Error Cmd	(0xEE:CRC Error)
		Tx4_buf[4] = 0xEE;			//Error Cmd	(0xEE:CRC Error)
		Tx4_buf[5] = 0xEE;			//Error Cmd	(0xEE:CRC Error)

		wtemp = CRC16(Tx4_buf, 6);	//총 6byte [0]~[5]
		Tx4_buf[6] = wtemp & 0xFF;		//CRC's Low byte	
		Tx4_buf[7] = (wtemp>>8);		//CRC's High byte	
		RS232_putchar(8);				//Total 8 byte sendding	[0]~[7] Total 8 byte sending.
	}	
	else if(Rcv2_ok == 0x88)	//음성모듈용 받기만 수행하기에 쓰기작업 하지 않았음. 
	{	
		Rcv2_ok = 0;  
		bptr = (byte *)(VOICE_DATA_ADDRESS + (Rx4_buf[2]<<8) + Rx4_buf[3]);	//주소설정			
		no = Rx4_buf[5]*2;	//Write number
		for(i = 0; i < no; i++)
		{
			*bptr = Rx4_buf[7+i];	//n개 write data...
			bptr++;
		}	
		//ACK Data Sending
		Tx4_buf[0] = Rx4_buf[0];	//Station(국번 1byte) 0x01
		Tx4_buf[1] = Rx4_buf[1];	//Function(함수 1byte) 쓰기 : 0x08			
		Tx4_buf[2] = Rx4_buf[2];	//High Address(1byte) 쓰기 시작할 번지
		Tx4_buf[3] = Rx4_buf[3];	//Low Adddress(1byte)
		Tx4_buf[4] = Rx4_buf[4];	//Length High(1byte) 쓰기 갯수
		Tx4_buf[5] = Rx4_buf[5];	//Length Low(1byte)
		CheckSum_data = CRC16(Tx4_buf, 6);	//6개 데이터의 CheckSum계산 
		Tx4_buf[6] = CheckSum_data & 0xFF;	    //CRC's Low byte	
		Tx4_buf[7] = (CheckSum_data >> 8);		//CRC's High byte	
		RS232_putchar(8);		//Total 8 byte sending 데이터 전송		
				
		/*if(Bf_Equipment_Num != _VOICE_DATA.equipment_num && _VOICE_DATA.equipment_num != 0x00)
		{
		  ENQ = _VOICE_DATA.equipment_num; // 국번 변수에 LCD의 환경설정으로 변경한 장비번호 값을 대입
		  LoRa_Tx_buf[0] = _VOICE_DATA.equipment_num;	//LCD의 환경설정으로 장비번호를 바꾸었을때 이 장비번호를 Slave LoRa로 보내서 LoRa의 국번으로 사용함.
		  
		  LoRa_RS232_putchar(1);		//Total i byte sending	
		  Bf_Equipment_Num = _VOICE_DATA.equipment_num;
		  _USER_RUN_SETTING.equipment_num = _VOICE_DATA.equipment_num;
		}*/
		
		if(Bf_Equipment_Num != _VOICE_DATA.equipment_num && _VOICE_DATA.equipment_num != 0x00)
		{
		  LoRa_Tx_buf[0] = ENQ;		//최초 전원 On시 Slave LoRa 국번을 보냄
		  LoRa_Tx_buf[1] = 0x07;	//장비 번호 바꾸는 함수
		  LoRa_Tx_buf[2] = _VOICE_DATA.equipment_num;	//최초 전원 On시 Slave LoRa 국번을 보냄
		  
		  LoRa_CheckSum_data = CRC16(LoRa_Tx_buf, 3);	//3개 데이터의 CheckSum계산 
		  
		  LoRa_Tx_buf[3] = LoRa_CheckSum_data & 0xFF;	    //CRC's Low byte	
		  LoRa_Tx_buf[4] = (LoRa_CheckSum_data >> 8);		//CRC's High byte	
			
		  LoRa_RS232_putchar(5);		//Total i byte sending	
		  ENQ = _VOICE_DATA.equipment_num; // 국번 변수에 LCD의 환경설정으로 변경한 장비번호 값을 대입
		  Bf_Equipment_Num = _VOICE_DATA.equipment_num; // 국번 변수에 LCD의 EEPROM에 저장되어있는 장비번호를 대입
		  
		  if(ENQ < 0x64) // 100보다 작으면
		  {
		  	sprintf((char*)AT_BTNAME, "AT+NAMEARTUS-840-%02d", ENQ);
		  }
		  else if(ENQ >= 0x64) // 100보다 크거나 같으면
		  {
			sprintf((char*)AT_BTNAME, "AT+NAMEARTUS-840-%03d", ENQ);
		  }
		  ble_name_change_fg = 1; // 블루투스 이름 변경
		}
	}	
}

void RS232_putchar(byte rs232_tx_num)	//UART4
{
	/*Send the character*/
	UART4->DR = Tx4_buf[0];
	Tx4_index = 1;
	Tx4_send_number = rs232_tx_num-1;
	UART4->CR1 &= ~USART_CR1_RXNEIE; 	//UART4's RXE Interrupt Disable	
	UART4->CR1 |= USART_CR1_TCIE; 		//UART4's TXE Interrupt Enable	
}


// CRC16 is based on the polynomial x^16+x^15+x^2+1
word CRC16_Table[256] = 
{
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
}; 


#if 1
word CRC16(byte *buf, word size)
{
	word crc=0xFFFF;
	byte   i;	
	unsigned char *p = buf;

	while (size--)
	{
		i=(crc ^ (*p++));
		crc = CRC16_Table[i & 0xFF] ^ (crc >> 8);
	}
	return crc;
}
#endif
#if 0	
word CRC16(byte *Data, byte Len)
{
	//CRC 버그로 인해 Check Sum으로 대체
	word check_sum = 0;
	while(Len--)
	{
		check_sum += *Data++;
	}
	return check_sum;
}
#endif
/* USER CODE END 1 */
