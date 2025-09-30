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
/*****************************************�߰��� �κ�(����)*************************************************/
byte ble_test_rxbuf[64];
byte  ble_test_index = 0,Ble_Rcv_ok= 0,Ble_CheckSum_EN = 0,TX_EMS_CNT = 0;
/*****************************************�߰��� �κ�(����)*************************************************/

/* USER CODE END PV */
//24.08.05
// Slave LoRa ������ ���ϴ� ����( ������ �ζ� ���� )
byte Bf_Equipment_Num = 0;
byte ENQ = 0x00; // �ζ� ��� ����

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

void SysTick_Handler(void)// ����Ʈ �κ�
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
	
  /* USER CODE END SysTick_IRQn 0 */
	HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */
	
  /* USER CODE END SysTick_IRQn 1 */
}

/*****************************************�߰��� �κ�(����)*************************************************/

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

//��������(Potentiometer)
dword P_M_sum = 0;
float P_M_avr = 0.0;
float P_M_slop = 0;
float P_M_offset = 0;
float P_M_actual = 0;
float PM_avr_value = 0;
float Current_angle = 0.0;
float bf_pm_actual = 0;

//��������(CT)
dword CT_ad_sum = 0;
float CT_ad_avr = 0.0;
float CT_slop = 0;
float CT_offset = 0;
float Current_actual = 0;


//1ms cnt(���� ����, ���Ѱ������� �����ð� ������)
dword timer2_ms_cnt = 0;
byte timer2_ms_cnt_start = 0;

//1min cnt(� �ð� ������)
word t2_min_cnt = 0;
byte exerc_time_start = 0;
word min_cnt = 0;
//LED1 ���
byte led1_toggle_flag = 0;

//����üũ
byte Self_check_end = 0;
byte check_step = 0;
//���� ���� ���� ����
byte CW_pm_check_ok = 0, CCW_pm_check_ok = 0;
byte CW_ct_check_ok = 0, CCW_ct_check_ok = 0;
byte CW_encoder_check_ok = 0, CCW_encoder_check_ok = 0;

//������� �׽�Ʈ��(23.03.06)
byte measure_EN = 0;
byte cur_index = 0;
float cur_measurement1[250], cur_measurement2[250];
//Ķ���극�̼� ���� ����
byte Cal_rate_of_change_f = 0,plus_cnt = 100;
extern byte Measurement_mode, Measure_up, Measure_down;
extern byte skip_cnt_start;
//����, ���� ����Ȯ�ο� ����
word p_m_avr_cnt = 0, ct_avr_cnt = 0;
byte save_angle_end = 0, default_save_angle = 0;
byte save_ct_end = 0, default_ct_angle = 0, default_save_ct;
//�������
byte Voice_err_cnt=0, Voice_error_cnt = 0;//������⿡�� ������ �߻��ϸ� �����ϴ� ī��Ʈ
//Ÿ�̸� ���� �۵� Ȯ�� ����
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
	TIM1->SR &= ~TIM_IT_UPDATE;	//TIM1�� ������Ʈ ���ͷ�Ʈ �÷��׸� Ŭ��� �����ϸ� �̴� ���ͷ�Ʈ�� �߻������� �˸��� �÷��׸� ����� �۾�����, ���ͷ�Ʈ�� ��߻���ų �� �ֵ��� ��. HAL_TIM_IRQHandler(&htim2);
	{
		Tim1_inttrupt = 1;
		TIM1->SR &= ~TIM_SR_UIF; // ���ͷ�Ʈ �÷��� Ŭ����
		/* USER CODE BEGIN TIM1_UP_IRQn 1 */
		//TIM1->ARR = (65535 - purse1_up_1angle * 5) - 1; //ARR �� ���� ù ��° �ϰ�
		if(angle_measure_1start)
		{
			one_degree_1counter++;
			angle_count = ANGLE_RANGE - one_degree_1counter;//one_degree_1counter ->  1 = 5��
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
	TIM5->CNT = 0;//���α׷� �ҿ�ð� ���� ����	
	//VOICE_CS_HIGH;
	TIM2->SR &= ~TIM_IT_UPDATE;//TIM2�� ������Ʈ ���ͷ�Ʈ �÷��׸� Ŭ��� �����ϸ� �̴� ���ͷ�Ʈ�� �߻������� �˸��� �÷��׸� ����� �۾�����, ���ͷ�Ʈ�� ��߻���ų �� �ֵ��� ��. HAL_TIM_IRQHandler(&htim2);		
	
	check_en_counter = TIM1 -> CNT;
	if(anlge_cal_flag && Tim1_inttrupt)//anlge_cal_flag : ù ���� �ϰ� �������� ad���� Not_save_arr�� ��° ��� �������� AD����, Not_save_arr������ ��
	{//ù ��° ��� ���� -> ù �ϰ����� ���� �迭�� AD�� �����ϴ� ����  
		Tim1_inttrupt = 0; //���ڴ� 5�� ���ͷ�Ʈ�� �߻��� �������� �����ϵ��� �Ͽ���
		if(save_170_angle)
		{
			save_170_angle = 0;
			angle_count = ANGLE_RANGE;//���� ������ ���� �� �� 31���� �����ϱ� ������ 32�� ��������� 32�迭�� 170���� ad���� ��	
		}
		if(angle_count >= 1 && angle_count <= (ANGLE_RANGE - 1))
			angle_arr[angle_count] = (word)P_M_avr; //P_M_avr: 200ms AD��հ�(1000�� ��հ�)  Store the value of P_M_avr in the array
		else if(angle_count == ANGLE_RANGE || angle_count == 0)
		{
			angle_arr[ANGLE_RANGE] = angle_ad_point2;//210���� ��쿡�� ���� ������ AD�� �־��ش�.	
			angle_arr[0] = angle_ad_point4;
		}
		//if(angle_count >= (high_angle / 5))
			//Disable_TIM1_Update_Interrupt();//���ͷ�Ʈ ��Ȱ��ȭ
		//Disable_TIM1_Update_Interrupt();//���ͷ�Ʈ ��Ȱ��ȭ, ���� �� TIM1�� ��� x
	}
	P_M_avr = ADC1_data[1];//�׽�Ʈ������ ������ ���� ����.
	if(low_to_high)
	{//�޽��� ī��Ʈ�� ����ϴ� �ڵ��̸� �̴� ù ��° ����� ��쿡�� �ʿ��ϴ�.
		current_count = __HAL_TIM_GET_COUNTER(&htim1);
		delta_count = current_count - previous_count;
		if(delta_count < -32768)
			delta_count += 65536;
		else if (delta_count > 32768)
			delta_count -= 65536;
		// �� �޽� �� ����
		final_count += delta_count;
		previous_count = current_count;
		//960 to 3940            210.9		
	}
	
	
	static byte save_one = 0;
	if(low_to_high)//-30������ 210���� �̵���� �ݵ�� -30������ �����ؾ���.
	{
		if(save_one)
		{
			angle_measure_2start = 1; // �� ��° ��½� ����/AD ���� ���� ù ���� ���� ��½ÿ��� �����ϸ� �� ��.
			//angle_arr[0] = angle_ad_point4;//-30���� ��쿡�� ���� ������ AD�� �־��ش�.	
		}
		PWM_NOT_SLEEP;
		if((P_M_avr >= (angle_ad_point4 - 30)) && (P_M_avr < (angle_ad_point2 - ad_count_down)))//- 1�� P_M_avr�� �ִ� 1�̶� Ʋ������ hgih�� �ö��� ���ϴ� ������ ��������
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
		if((P_M_avr >= (angle_ad_point2 - ad_count_down)) && (P_M_avr <= angle_ad_point2 + ad_count_down))//�����ϴ� ad ���� ������ ad ������ Ÿ�̸ӷ� �����Ͽ���.
		{//�� ������ ������ ad_count_down ��
			remain_ad = angle_ad_point2 - (word)P_M_avr;//���� �����ִ� ad���̸� ���� �پ��.
			low_to_high_pwm = low_to_high_pwm - (ad_count_down - remain_ad);
			if(low_to_high_pwm <= 2010 && P_M_avr < angle_ad_point2)
				low_to_high_pwm = 2010;
			//if(P_M_avr >= angle_ad_point2 - 7)//���⼭ 5�� ���� ���� ���� �� ��ǥ ad���� �����ϱ� �� �� -10�� ���ش�. ���⼭ �� -�� ������ �ſ� �߿���. ������ �޽��� ��Ȯ���� ����� ������ ��ģ��.
			if(P_M_avr >= angle_ad_point2 + 4)//���⼭ 5�� ���� ���� ���� �� ��ǥ ad���� �����ϱ� �� �� -10�� ���ش�. ���⼭ �� -�� ������ �ſ� �߿���. ������ �޽��� ��Ȯ���� ����� ������ ��ģ��.
			{//210���� ����       11
				low_to_high = 0;
				low_to_high_pwm = 1800;
				//ù ��° ��¿��� ���� �迭�� AD�� ����
				//purse1_up_1angle = (dword)final_count / (dword)(real_angle_point2-real_angle_point4);
				purse1_up_1angle = 8333;
				TIM1->ARR = ((word)purse1_up_1angle * 5);//(65535 - ((word)purse1_up_1angle * 5)); // ARR �� ���� �޽� 8322�������� ARR�� 23925�� ����
				__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
				//purse1_up_1angle : 8315 ~ 8340 �������
				//ī��Ʈ ���� ���� ��� �ʱ�ȭ
				save_one = 1;
				angle_measure_1start = 1;//angle_count��� ����
				final_count = 0;//���� �޽� ī��Ʈ Ƚ�� �ʱ�ȭ
				delta_count = 0;
				current_count = 0;
				previous_count = 0;
				//ī��Ʈ ���� ���� ��� �ʱ�ȭ		
				TIM5 -> CNT = 0;
				anlge_cal_flag = 1;//AD�� ����
				if(angle_measure_2start)
				{// �� �� �� ��� ���� ����
					angle_measure_2start = 0;
					Not_save_arr = 1;// ad���� �Ǿ� �ִ� �迭�� ��� ����
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
	if(high_to_low)//1�� ���� �� -30���� �ش��ϴ� ad������ �ٽ� ���� -> 1������ �����Ǵ� ad���� ����ϱ� ����
	{
		static byte enter_only_one = 0;
		if(enter_only_one == 0)
		{
			enter_only_one = 1;
			TIM1->CR1 &= ~TIM_CR1_CEN; // Ÿ�̸� ī���� ��Ȱ��ȭ// Ÿ�̸� ��Ȱ��ȭ				
			TIM1->CNT = ((word)purse1_up_1angle * 5);//������ ���ں��� ���� ����, Ÿ�̸Ӹ� ��Ȱ��ȭ �� ���¿��� �����ؾ� cnt�� �ٲ�
			TIM1->DIER |= TIM_DIER_UIE; // ������Ʈ ���ͷ�Ʈ�� Ȱ��ȭ
			TIM1->CR1 |= TIM_CR1_CEN;
			TIM1->CR1 |= TIM_CR1_DIR;//ī��Ʈ ������ ���ҷ� ����			
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
			//if(P_M_avr <= angle_ad_point4 + 19)//���⼭ �� +�� ������ �ſ� �߿���. ������ �޽��� ��Ȯ���� ����� ������ ��ģ��.
			if(P_M_avr <= angle_ad_point4 + 2)//���⼭ �� +�� ������ �ſ� �߿���. ������ �޽��� ��Ȯ���� ����� ������ ��ģ��.
			{//19
				high_to_low = 0;
				high_to_low_pwm = 1800;
				//purse1_down_1angle = (dword)final_count / (ANGLE_RANGE); // 1���� �޽��� ��
				purse1_down_1angle = (dword)final_count / ((real_angle_point2-real_angle_point4)); // 1���� �޽��� ��
				//ī��Ʈ ���� ���� ��� �ʱ�ȭ
				final_count = 0;//���� �޽� ī��Ʈ Ƚ�� �ʱ�ȭ
				delta_count = 0;
				current_count = 0;
				previous_count = 0;
				//ī��Ʈ ���� ���� ��� �ʱ�ȭ		
				anlge_cal_flag = 0;//stop recording		
				eeprom_page_state = 6;//eeprom�� ad�� ���� 
			}
		}
		TIM8->CCR1 = high_to_low_pwm;			
	}
	
	if(low_to_high || high_to_low)
	{
		P_M_slop = (real_angle_point2 - real_angle_point4) / (angle_ad_point2 - angle_ad_point4); 
		P_M_offset = real_angle_point4 - (P_M_slop * angle_ad_point4);		
		P_M_actual = (P_M_slop * P_M_avr) + P_M_offset;
		PM_avr_value = (P_M_actual + bf_pm_actual) / 2;	//���Ű��� ���簪�� ��ճ�
		bf_pm_actual = P_M_actual;//���簪�� ���Ű��� ����		
	}
	else 
	{
		if(P_M_avr >= angle_ad_point3 - 100 && P_M_avr <= angle_ad_point4)//ad������ ������ �α� ���� 100���� ���� ��Ȯ������ ������ ���� ����.
		{//Point3 ~ Point4
			P_M_slop = (real_angle_point4 - real_angle_point3) / (angle_ad_point4 - angle_ad_point3); 
			P_M_offset = real_angle_point3 - (P_M_slop * angle_ad_point3);					
		}
		else if(P_M_avr <= angle_ad_point1 + 100 && P_M_avr >= angle_ad_point2)//ad������ ������ �α� ���� 100���� ���� ��Ȯ������ ������ ���� ����.
		{//Point1 ~ Point2
			P_M_slop = (real_angle_point1 - real_angle_point2) / (angle_ad_point1 - angle_ad_point2); 
			P_M_offset = real_angle_point2 - (P_M_slop * angle_ad_point2);					
		}	
		if(((P_M_avr >= angle_ad_point3 - 100) && (P_M_avr <= angle_ad_point4)) || ((P_M_avr <= angle_ad_point1 + 100) && (P_M_avr >= angle_ad_point2)))
		{
			P_M_actual = (P_M_slop * P_M_avr) + P_M_offset;
			PM_avr_value = (P_M_actual + bf_pm_actual) / 2;	//���Ű��� ���簪�� ��ճ�
			bf_pm_actual = P_M_actual;//���簪�� ���Ű��� ����
		}	
		else
		{//���� ���
			if(angle_recording_end)// 0���� ����� ������ �Ұ���������. 
			{/*
				//byte ANGLE_RANGE = 49;  // 48*5=240
				int lower_index = 0;
				int upper_index = 0;
				// �迭�� ��ȸ�Ͽ� P_M_avr ���� ���ԵǴ� ������ ã��
				//P_M_avr = 3800;//1000
				for(int i = 0; i < ANGLE_RANGE; i++)  
				{//5���� ad����
					if(P_M_avr >= angle_arr[i] && P_M_avr <= angle_arr[i + 1])
					{
						lower_index = i;
						upper_index = i + 1;
						break;
					}
				}
				//���� �������� ����Ͽ� ��Ȯ�� ���� ���
				lower_ad = angle_arr[lower_index];
				upper_ad = angle_arr[upper_index];
				lower_angle = real_angle_point4 + lower_index * 5;
				upper_angle = real_angle_point4 + upper_index * 5;
				//���� ���
				if(P_M_avr > angle_ad_point4 && P_M_avr < angle_ad_point2)
				{
					if(upper_ad != lower_ad)
						P_M_actual = lower_angle + (P_M_avr - lower_ad) * (upper_angle - lower_angle) / (upper_ad - lower_ad);
					else
					{
						if(P_M_avr > (angle_ad_point2 + angle_ad_point4)/2) //10�� 170�� AD�� ���
							P_M_actual = real_angle_point2;
						else
							P_M_actual = real_angle_point4;
					}
					PM_avr_value = (P_M_actual + bf_pm_actual) / 2;	//���Ű��� ���簪�� ��ճ�
					bf_pm_actual = P_M_actual;//���簪�� ���Ű��� ����			
				}
				*/
				int size = sizeof(angle_arr) / sizeof(angle_arr[0]); // �迭 ũ�� ��� // ���� ������ ����ϱ� ���� interpolate �Լ� ȣ��
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
	////voice_output(); 1msī���Ϳ� ������ �۵� �Ұ�
	if(voice_out)//0.1ms task
	{
		voice_out = 0;
		voice_output();
	}
	//Calibration��� ���� �������۽� Delay
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
	//motor_delay_ms �Լ���
	static unsigned int t2_cnt = 0;
	if(timer2_ms_cnt_start == 1)
	{
		if(++t2_cnt >= 10)
		{
			t2_cnt = 0;
			timer2_ms_cnt++;	//1ms���� 1������
		}
	}
	else
	{
		t2_cnt = 0;
		timer2_ms_cnt = 0;	
	}
	//motor_exerc_time �Լ���
	static unsigned int Timer2_cnt = 0;
	if(exerc_time_start == 1)
	{
		if(!PAUSE_F)	//�Ͻ����� �ƴҶ�
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
	/*********************ADC1(����,����)*********************/
	//��������, �������� AD�� �޾ƿ�
	adc_value = ADC1->DR;
	ADC1_data[ADC1_index] = adc_value;
	//��� �ⱸ�� P_M_avr ������ �����Ͽ���
	if(_USER_RUN_SETTING.calibration == 0x01)	//Calibration ���
	{
		PWM_NOT_SLEEP;
		AVR_CNT = 10000;	//Calibration�߿��� 10000�� ���(10�ʿ� �ѹ��� ��հ� ���)
	}
	else
		AVR_CNT = 1000;		//���ÿ��� 1000�� ���(100ms�� �ѹ��� ��հ� ���)
	static byte ad_save = 0;
	switch(ADC1_index)
	{
		case 0:		//ADC1 CH15 (��������)
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
				CT_ad_avr  = (float)CT_ad_sum / AVR_CNT;// ������ ���
				CT_ad_sum = 0;
				switch(ad_save)
				{
					case 5:
						_CALIBRATION_SETTING.current_500mA_AD = CT_ad_avr;	//500mA�϶� AD��հ� ����
						_CALIBRATION_SETTING.AD_ok = 1;
						ad_save = 0;
						ct_cal_ad_1 = 1;
						break;
					case 6:
						_CALIBRATION_SETTING.current_2500mA_AD = CT_ad_avr;		//2500mA�϶� AD��հ� ����
						_CALIBRATION_SETTING.AD_ok = 1;
						save_ct_flag = 1;
						ad_save = 0;
						ct_cal_ad_2 = 1;//��������
						break;
					default:
						break;
				}
				CT_slop = (real_current_2500mA - real_current_500mA)/(cal_2500mA_val - cal_500mA_val);//0.2A, 2A  two point calibration
				CT_offset = real_current_500mA - (CT_slop*cal_500mA_val);
				//CT_slop = (2500 - 500)/(2580 - 434);		//0.2A, 2A  two point calibration
				//CT_offset = 500 - (CT_slop*434);
				Current_actual = (CT_slop * CT_ad_avr) + CT_offset;					
				if((Current_actual < 0.1) && (Current_actual > -0.1))	//-0.1A�ʰ� +0.1A �̸��� 0A�� ó��
					Current_actual = 0.0;
			}
			break;
		case 1:	//ADC1 CH8	(��������)Potentiometer
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
						_CALIBRATION_SETTING.angle_ad_point_1 = (word)P_M_avr;	//210���϶� AD��հ� ����
						_CALIBRATION_SETTING.AD_ok = 1;
						ad_save = 0;	
						angle_cal_ad_1 = 1;
						break;
					case 2:
						_CALIBRATION_SETTING.angle_ad_point_2 = (word)P_M_avr;	//170���϶� AD��հ� ����
						_CALIBRATION_SETTING.AD_ok = 1;
						ad_save = 0;
						angle_cal_ad_2 = 1;
						break;
					case 3:
						_CALIBRATION_SETTING.angle_ad_point_3 = (word)P_M_avr;	//-30���϶� AD��հ� ����
						_CALIBRATION_SETTING.AD_ok = 1;
						ad_save = 0;
						angle_cal_ad_3 = 1;
						break;		
					case 4:
						_CALIBRATION_SETTING.angle_ad_point_4 = (word)P_M_avr;	//10���϶� AD��հ� ����
						_CALIBRATION_SETTING.AD_ok = 1;
						ad_save = 0;
						angle_cal_ad_4 = 1;
						calibration_state = 1;//calibration ing state
						break;
						/*
					case 5:
						_CALIBRATION_SETTING.angle_ad_point_5 = (word)P_M_avr;	//xx���϶� AD��հ� ����
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
					case Elbow:		//�Ȳ�ġ
						_Ble_Data.motion = 0x00;
						_LoRa_Data.motion = 0x00;
						Current_angle = (round(PM_avr_value*10)/10) - 60;	    //�Ҽ��� ��°�ڸ����� �ݿø�
						break;
					case Shoulder:	//���
						_Ble_Data.motion = 0x01;
						_LoRa_Data.motion = 0x01;
						Current_angle = round(PM_avr_value*10)/10;		        //�Ҽ��� ��°�ڸ����� �ݿø�	
						break;
					case Knee:		//����
						_Ble_Data.motion = 0x02;
						_LoRa_Data.motion = 0x02;
						//Current_angle = -((round(PM_avr_value*10)/10) - 90);	//�Ҽ��� ��°�ڸ����� �ݿø�
						Current_angle = -((round(PM_avr_value*10)/10) - 110);	//�Ҽ��� ��°�ڸ����� �ݿø�
						//Current_angle = 110 - PM_avr_value;
						break;
					case wrist:		//�ո�
						_Ble_Data.motion = 0x04;
						_LoRa_Data.motion = 0x03;
						Current_angle = (round(PM_avr_value*10)/10) - 90;	    //�Ҽ��� ��°�ڸ����� �ݿø�
						break;
					case ankle:		//�߸�
						_Ble_Data.motion = 0x08;
						_LoRa_Data.motion = 0x04;
						Current_angle = (round(PM_avr_value*10)/10) - 90;	    //�Ҽ��� ��°�ڸ����� �ݿø�
						break;						
					default:
						break;
				}
				if(Priority_dir_set == 1)	
					Priority_dir_set = 'S';	//����üũ ���� ��������(Start)
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
				LoRa_RS485_dead_time--;	//1msec ���� 1�� ����
			else
			{
				LoRa_Rcv_ok = (LoRa_Rcv_ok<<4) | LoRa_Rcv_ok;	//0x10:110, 0x04:44, 0x0E:EE
			}
		}	
		if(Rcv2_ok)//UART4 data received?
		{
			if(RS485_dead_time)
				RS485_dead_time--;	//1msec ���� 1�� ����
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
	Exe_time_ns = (word)(TIM5->CNT*1000.0/72.0);//���α׷� �ҿ� �ð� ���� 0.1ms, 100ns �ʰ��ϸ� ������  
}
//CPM��� ������ ���� ��� ����
#define MAX_PWM 1800
#define MIN_PWM 0
#define RAMP_STEPS 3000//pwm ���� or �����ϴ� ����
#define LONG_PRESS_TIME_MS 1500
#define HOLD_TIME_MS 10000 //��� or �ϰ��ϴ� ����
#define HOLD_16SEC 16000
//CPM��� ������ ���� ���� ����
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

//����ġ ä�͸� ����
byte T10ms_cnt_debouncing = 0;
byte EMS_STATE = 0, cant_read_id = 0;
byte BF_START_SW = 0, BF_STOP_SW = 0, BF_EMS_SW = 0;
word T3_1000ms_cnt=0;
byte T3_sec=0, T3_min=0;

//EEROM����
word eeprom_timer_cnt = 0;

//������� ����
byte t3_10ms_cnt2 = 0, voice_en = 0, state_cnt2 = 0;

byte record_en = 0;
byte RS485_dead_time=0;			//1msec down counter
byte LoRa_RS485_dead_time=0;	//1msec down counter
//����üũ ���� ����
byte Start_self_check = 0, Restart_self_check = 0, Opposition = 0;// ó�� ���� 1�� �ž� ó�� ���� ������
float arr_test[255] = {0};
byte ddsfsdf = 0, T3_5000ms_cnt_flag = 0, wait_measurement = 0, over_angle__start_flag = 0;
word T3_5000ms_cnt = 0,t3_500ms_cnt1 = 0, t3_500ms_cnt2 = 0;
void TIM3_IRQHandler(void)	//1mS Task
{
	////HAL_TIM_IRQHandler(&htim3);
	//#define __HAL_TIM_CLEAR_IT(__HANDLE__, __INTERRUPT__)      ((__HANDLE__)->Instance->SR = ~(__INTERRUPT__))
	TIM3->SR &= ~TIM_IT_UPDATE;	//TIM3�� ������Ʈ ���ͷ�Ʈ �÷��׸� Ŭ��� �����ϸ� �̴� ���ͷ�Ʈ�� �߻������� �˸��� �÷��׸� ����� �۾�����, ���ͷ�Ʈ�� ��߻���ų �� �ֵ��� ��. HAL_TIM_IRQHandler(&htim2);
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
			Change_dir = 1; //3�� �ڿ� Cal_rate_of_change_f = 1����
			T3_5000ms_cnt = 0;
			start_over_angle = 1;//��, ���Ѱ� ���� �� �������� �����Ǵ� ������ �����ϱ� ����
		}
	}
	if(++T3_1000ms_cnt >= 1000)
	{//Ÿ�̸� �����۵� �׽�Ʈ��
		T3_1000ms_cnt = 0;
		if(++T3_sec >= 60)
		{
			T3_sec=0;
			if(++T3_min >= 60)
				T3_min=0;
		}
	}	
	static byte Voice_NG = 0;
	if((Voice_err_cnt == 0) && (Voice_NG == 1))//ó������ Voice Error�� ������ 5ms �����
	{
		VOICE_RESET_LOW;	
		state_cnt2 = 1;		//Voce IC Power Down Sequence
	}
	else
		Voice_err_cnt--;	//5mS���� Voice Reset�� High�� ����� ����
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
			SPI_tx_buf[0] = 0x48;// 0x48 : Read ID -> (0x04 EF 40 15)�� �о�� ����üũ OK			
			voice_error_check = 0;
			VOICE_CS_LOW;//������� CS LOW
			HAL_SPI_TransmitReceive(&hspi1, SPI_tx_buf, SPI_rx_buf, 5 ,10);
			VOICE_CS_HIGH;//������� CS HIGH
			if(SPI_rx_buf[1] == 0x04 & SPI_rx_buf[2] == 0xEF & SPI_rx_buf[3] == 0x40 & SPI_rx_buf[4] == 0x15)
				cant_read_id = 1;//����
			else 
				cant_read_id = 0;//������
		}	
		if(voice_en)
		{		
			SPI_tx_buf[0]= 0x40;// 0x40 : Read Status ->  (0x60 0x04)�� �о�� OK
			VOICE_CS_LOW;// ������� CS LOW
			HAL_SPI_TransmitReceive(&hspi1, SPI_tx_buf, SPI_rx_buf,2, 10);
			VOICE_CS_HIGH;// ������� CS HIGH
			if(SPI_rx_buf[0] == 0x60)
			{
				if(SPI_rx_buf[1] == 0x04)
				{
					voice_en = 0;				
					SPI_voice_set();//Voice�� ���
					state_cnt2 = 0;
					Voice_NG = 0;
					_USER_RUN_SETTING.x06_1B = 0;
					_VOICE_DATA.x06_1B = 0;					
				}
				else
				{
					Voice_NG = 1;
					Voice_error_cnt++;
					if(state_cnt2 == 0)	//ó������ Voice Error �������
					{
						Voice_err_cnt = 5;	//5mS���� Voice Reset�� High�� ����� ����
						VOICE_RESET_HIGH;	//����(High Active)
						state_cnt2 = 1;
					}
					else if(state_cnt2 == 1) 	//power down Sequence
					{
						SPI_tx_buf[0]= 0x12;	//power down
						VOICE_CS_LOW;			// ������� CS LOW
						HAL_SPI_Transmit(&hspi1, SPI_tx_buf, 1, 10); 
						VOICE_CS_HIGH;			// ������� CS HIGH		
						state_cnt2 = 2; 		//Voice IC Power Up
					}
					else if(state_cnt2 == 2)	//power up Sequence
					{
						SPI_tx_buf[0]= 0x10; //power up
						VOICE_CS_LOW;//������� CS LOW
						HAL_SPI_Transmit(&hspi1, SPI_tx_buf, 1, 10); 
						VOICE_CS_HIGH;//������� CS HIGH	
						voice_en = 0;
						state_cnt2 = 0;
					}					
				}
			}
		}		
	}
	if(read_sensitivity || save_angle_flag || save_ct_flag || anlge_ad_err_flag || anlge_real_err_flag || ct_err_flag || page1_end_flag || save_angle_end_flag || page2_end_flag || wait_1400m_flag || Last_page ||wait_high_to_low)
	{//EERPOM ����, ���� Ķ���극�̼� ���� Ŭ�� �� 1.4�� �Ŀ� ����
		eeprom_timer_cnt++;
		if(eeprom_timer_cnt >= 1700) // 1.7��
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
				save_angle_flag = 0; // Ÿ�̸Ӹ� �����ϴ� ���� �߰� (���� ����)
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
				save_ct_flag = 0; // Ÿ�̸Ӹ� �����ϴ� ���� �߰� (���� ����)
				save_ct_end = 1;
			}

			if(anlge_ad_err_flag || anlge_real_err_flag)
			{
				default_save_angle = 1;
				anlge_real_err_flag = 0;
				anlge_ad_err_flag = 0; // Ÿ�̸Ӹ� �����ϴ� ���� �߰� (���� ����)
			}
			if(ct_err_flag)
			{
				default_save_ct = 1;
				ct_err_flag = 0; // Ÿ�̸Ӹ� �����ϴ� ���� �߰� (���� ����)
			}
			eeprom_timer_cnt = 0;
		}
	}	
	static byte self_checking = 0;	
	if(++T10ms_cnt_debouncing >= 10)	//10ms
	{		
		T10ms_cnt_debouncing = 0;
		//START ����ġ ��ٿ��
		
		if(Succed_Tx_Cnt)
		{
		  Succed_Tx_Cnt--;
		}
		
		//09.20 ���찡 ��
		if(TX_EMS_CNT)
		{
		  TX_EMS_CNT--;
		}
		
		static byte BFBF_START_SW = 0;
		//���찡 ������
		if(self_checking == 0)
		{//����üũ���̸� start stop ��� x
			if(START_SW == BF_START_SW)//�ʱⰪ�� 1 -->  �Ʒ��� if���� �������� �ʰ� ���� ���๮�� ����         
			{//cattering ok
				 if(START_SW == 1)//sw1 push ����                           
				 {//���� �κп� ����ġ�� ������ ���� �� ���ϴ� ���� �ۼ��ϱ�
					 //���찡 ������
					 //if(START_SW != BFBF_START_SW && (_SENSOR_DATA.state == 1 || _SENSOR_DATA.state == 4))//������ ���� �� ���� ���� ����
					 if(START_SW != BFBF_START_SW && (_SENSOR_DATA.state == 1 || _SENSOR_DATA.state == 4) && STOP_SW != 1)//������ ���� �� ���� ���� ����( ���� ��ư�� ���ÿ� ������ �ʾ����� )
					 {
						START_STATE = 1;
						_SENSOR_DATA.start_sw = START_STATE;
						Succed_Tx_Cnt = 50;
					 }
					 else if(START_SW != BFBF_START_SW && STOP_SW == 1)//������ ���� �� ���� ���� ����( ���� ��ư�� ���ÿ� �������� 5���� ���¹� LCD ����͸� ȭ������ ���� )
					 {
					   	START_STATE = 1;
						_SENSOR_DATA.start_sw = START_STATE;
						Succed_Tx_Cnt = 500;
					 }
				 }
				 else 
				 {
					 if(BFBF_START_SW)  //sw1�� ���������� �ȴ��� ����, ������ �����·� ������ 
					 {//���� ��
					   Succed_Tx_Cnt = 50;
					 }
				 }
				 BFBF_START_SW = BF_START_SW;
			}
			BF_START_SW = START_SW;
			//STOP ����ġ ��ٿ��
			static byte BFBF_STOP_SW = 0;
			if(STOP_SW == BF_STOP_SW)                    //�ʱⰪ�� 1 -->  �Ʒ��� if���� �������� �ʰ� ���� ���๮�� ����         
			{//cattering ok
				 if(STOP_SW == 1)//sw1 push ����                           
				 {// ���� �κп� ����ġ�� ������ ���� �� ���ϴ� ���� �ۼ��ϱ�
					 //���찡 ������
					 //if(STOP_SW != BFBF_STOP_SW && (_SENSOR_DATA.state == 0 || _SENSOR_DATA.state == 4))//������ ���� �� ���� ���� ����
				   	 if(STOP_SW != BFBF_STOP_SW && (_SENSOR_DATA.state == 0 || _SENSOR_DATA.state == 4) && START_SW != 1)//������ ���� �� ���� ���� ����( ���� ��ư�� ���ÿ� ������ �ʾ����� )
					 {
						STOP_STATE = 1;
						_SENSOR_DATA.stop_sw = STOP_STATE;
						Succed_Tx_Cnt = 50;
					 }		
					 else if(STOP_SW != BFBF_STOP_SW && START_SW == 1)//������ ���� �� ���� ���� ����( ���� ��ư�� ���ÿ� �������� 5���� ���¹� LCD ����͸� ȭ������ ���� )
					 {
						STOP_STATE = 1;
						_SENSOR_DATA.stop_sw = STOP_STATE;
						Succed_Tx_Cnt = 500;
					 }
				 }
				 else 
				 {
					 if(BFBF_STOP_SW)  //sw1�� ���������� �ȴ��� ����, ������ �����·� ������ 
					 {//���� ��
					   Succed_Tx_Cnt = 50;
					 }
				 }
				 BFBF_STOP_SW = BF_STOP_SW;
			}
			BF_STOP_SW = STOP_SW;	
		}
		/*
		if(EMS_SW == BF_EMS_SW)	//Debounce Check OK(10ms ���� ���� ����)
		{
			if(EMS_SW)
			{
				EMS_STATE = 1; // EMS ����ġ�� ������ ���� ����
				if(Measurement_mode == 1)
					measure_restart = 1;
			}
			else 
				EMS_STATE = 0; // EMS ����ġ�� ������ �ʾ��� ���� ����
		}
		BF_EMS_SW = EMS_SW;
		*/
	}	
	if(Settings) 
	{
		if(_USER_RUN_SETTING.equip_up_down)
			PWM_NOT_SLEEP;
		if(_USER_RUN_SETTING.equip_up_down == 1)// _USER_RUN_SETTING.equip_up_down ���� ���� ���� ����
		{
			_USER_RUN_SETTING.equip_up_down = 0;
			if(up_long_press == 0 && down_long_press == 0 && moving_down == false)//up��� ������ �����ϴ� ������ �Ϸ� �ؾ� ���԰���
				button_up_pressed = 1; //��ư�� ���� ����
			if(down_long_press)//�Ʒ��� 10�ʵ��� � ���� ���¿��� �ݴ������ down�� ������ ������Ŵ
				down_stop_flag = 1;
		} 
		else if(_USER_RUN_SETTING.equip_up_down == 2) 
		{
			_USER_RUN_SETTING.equip_up_down = 0;
			up_long_press = 1;// ���� ��� ����
			moving_up = true;
			moving_down = false;
		} 
		else if(_USER_RUN_SETTING.equip_up_down == 3) 
		{
			_USER_RUN_SETTING.equip_up_down = 0;
			if(up_long_press == 0 && down_long_press == 0 && moving_up == false)//down��� ������ �����ϴ� ������ �Ϸ� �ؾ� ���԰���
				button_down_pressed = 1;// ��ư�� �Ʒ��� ����
			if(up_long_press)//���� 10�ʵ��� � ���� ���¿��� �ݴ������ up�� ������ ������Ŵ
				up_stop_flag = 1;
		} 
		else if(_USER_RUN_SETTING.equip_up_down == 4) 
		{
			_USER_RUN_SETTING.equip_up_down = 0;
			down_long_press = 1;// �Ʒ��� ��� ����
			moving_down = true;
			moving_up = false;
		}
			
		// ���� ��ư�� ������ ���
		if(button_up_pressed) 
		{
			MOTOR_UP_DOWN;//���� ������ ����
			moving_up = true;
			moving_down = false;
			button_up_pressed = 0;
			step_count = 0; // ���� ī���� �ʱ�ȭ
			hold_count = 0; // Ȧ�� ī���� �ʱ�ȭ
		}
		// �Ʒ��� ��ư�� ������ ���
		if(button_down_pressed) 
		{
			MOTOR_UP_DOWN; // ���� ������ ����
			moving_down = true;
			moving_up = false;
			button_down_pressed = 0;
			step_count = 0; // ���� ī���� �ʱ�ȭ
			hold_count = 0; // Ȧ�� ī���� �ʱ�ȭ
		}
		if(moving_up || moving_down) // ���� �����̴� ���
		{//��� ������ ���
			if(up_long_press || down_long_press) 
			{
				hold_count++;
				if(down_stop_flag)//up ing�۾��� �����߿� dwon ��ư�� ���� ��� ����
				{
					down_stop_flag = 0;
					hold_count = RAMP_STEPS + HOLD_TIME_MS;
				}
				else if(up_stop_flag)//down ing�۾��� �����߿� up ��ư�� ���� ��� ����
				{
					up_stop_flag = 0;
					hold_count = RAMP_STEPS + HOLD_TIME_MS;					
				}							
				if(hold_count < RAMP_STEPS) 
				{ // PWM ���� ���� (0 ~ 3��)
					current_pwm += (float)MAX_PWM / (RAMP_STEPS / 2);
					if(current_pwm > MAX_PWM) 
						current_pwm = MAX_PWM;
				} 
				else if((hold_count >= RAMP_STEPS) && (hold_count < RAMP_STEPS + HOLD_TIME_MS)) // ���� PWM ���� (3 ~ 13��)
					current_pwm = MAX_PWM;
				else if((hold_count >= RAMP_STEPS + HOLD_TIME_MS) && (hold_count <= RAMP_STEPS + HOLD_TIME_MS + RAMP_STEPS)) 
				{ // PWM ���� ���� (13 ~ 16��)
					current_pwm -= (float)MAX_PWM / (RAMP_STEPS / 2);
					if(current_pwm < MIN_PWM) 
						current_pwm = MIN_PWM;
				} 
				else if(hold_count > HOLD_16SEC) 
				{ // ���� �ð� �̻� ��� �� �ʱ�ȭ
					up_long_press = 0;
					down_long_press = 0;
					moving_up = false;
					moving_down = false;
					current_pwm = 0;  // PWM �ʱ�ȭ
				}
			}
			else 
			{//ª�� ������ ���
				if(step_count < RAMP_STEPS / 2) 
				{ // PWM ���� ����
					current_pwm += (float)MAX_PWM / (RAMP_STEPS / 2);
					if(current_pwm > MAX_PWM) 
						current_pwm = MAX_PWM;
					step_count++;
				} 
				else if(step_count >= (RAMP_STEPS / 2) && step_count < RAMP_STEPS) 
				{ // PWM ���� ����
					current_pwm -= (float)MAX_PWM / (RAMP_STEPS / 2);
					if(current_pwm < MIN_PWM) 
						current_pwm = MIN_PWM;
					step_count++;
				} 
				else 
				{
					moving_up = false;
					moving_down = false;
					current_pwm = 0;  // PWM �ʱ�ȭ
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
			_SENSOR_DATA.state = 0;//���				
		} 
		else//ȯ�漳������ �⺻������ ���� �����ؾ���.
		{//�����
			Motor_PWM_CW = 1800; // ���� ���� (PWM 0)
			Motor_PWM_CCW = 1800; // ���� ���� (PWM 0)
			if(_SENSOR_DATA.state == 0)
				_SENSOR_DATA.state = 1;//�����
			PWM_SLEEP;
		}		
	}


	
	if(Measurement_mode && Change_dir)//���ϰ� ���� �Ǹ� ������ �����ؾ���.
	{//��ȭ�� ��� flag ON	
		if(Measure_up == 1 || Measure_down == 1)//�������� �Ǵ� �������� ���϶��� ���ͷ�Ʈ(���������� 1�� �����ϴ� ���ȿ��� ��ȭ�� �����������)
		{
			if(++t3_500ms_cnt1 >= 2200)		
			{//0.5sec task
				Change_dir = 0;
				t3_500ms_cnt1 = 0;
				Cal_rate_of_change_f = 1;	//��ȭ�� ��� flag ON	
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
	/******************************����üũ*************************************/
	static unsigned int error_check_cnt = 0, t100us_cnt = 0, time_cnt = 0;
	static byte retry_f = 0;
	static byte k = 0, save_f = 0;

	if((Error_Check && angle_recording_end) || (Restart_self_check && angle_recording_end))//����üũ ���� && ���� ad �� �о���� �� //1.7�� �ڿ� ����
	{
		_SENSOR_DATA.state = 0x00;//����üũ�߿��� ������� ǥ��
		self_checking = 1;
		Self_check_end = 0;
		//check_step ���� ī���� �ð��� �ٸ�
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
			case 0:	//CW ����üũ ����(����üũ�� �� ����)
				_LoRa_Data.check_state |= 0x02;//Para.CT_error = 0;		//����üũ ��
				_LoRa_Data.check_state |= 0x04;//Para.encoder_error = 0;	//����üũ ��
				_LoRa_Data.check_state |= 0x08;//Para.PM_error = 0;		//����üũ ��		
				Bf_pm_avr = P_M_actual;		//����üũ�ϱ� ���� �������� �� ����
				Bf_ct_avr = Current_actual;	//����üũ�ϱ� ���� �������� �� ����
				Bf_encoder_avr = diff;		//����üũ�ϱ� ���� ���ڴ� �� ����	
				check_step = 1;//���� ����
			case 1:	//CW ����üũ ����(���� ����)
				save_f = 1;	//�׽�Ʈ�� ���� ��������
				if(++t100us_cnt >= 100)
				{
					t100us_cnt = 0;
					shift_step++;
					if(shift_step >= 21)
						shift_step = 20;
				}
				break;
			case 2://CW����üũ ��(���� ��)
				t100us_cnt = 0;
				shift_step = 20;	//�ִ� �ӵ� ����
				Af_ct_avr = Current_actual;	//����üũ ��, �������� �� ����(�������� ���� ���Ͱ� �������� �� ������)
				break;
			case 3:	//CW ����üũ ��(���� ����)
				if(++t100us_cnt >= 100)
				{
					t100us_cnt = 0;
					shift_step--;
					if(shift_step <= 0)
						shift_step = 0;
				}
				break;
			case 4:	//CW ����üũ �� and ���� ��ȯ(1�� �Ͻ�����)
				Af_pm_avr = P_M_actual;//����üũ ��, �������� �� ����
				Af_encoder_avr = diff;//����üũ ��, ���ڴ� �� ����
				Change_pm_avr = fabs(Af_pm_avr - Bf_pm_avr);			//����üũ ��,�� �������� �� ��ȭ��
				Change_ct_avr = fabs(Af_ct_avr - Bf_ct_avr);			//����üũ ��,�� �������� �� ��ȭ��
				Change_encoder_avr = Af_encoder_avr - Bf_encoder_avr;	//����üũ ��,�� ���ڴ� �� ��ȭ��
				//�������� �ڰ�����
				if((Change_pm_avr >= 10.0) && (Change_pm_avr <= 80.0))// �뷫 20 ~50�� ���� ������ ����üũ ����
					CW_pm_check_ok = 1;	//�������� ���� 
				else
					CW_pm_check_ok = 0;	//�������� ������
				//�������� �ڰ�����
				if((Change_ct_avr >= 150) && (Change_ct_avr <= 800))	//20 ~ 650mA �����Ǵ�
					CW_ct_check_ok = 1;	//�������� ���� 
				else
					CW_ct_check_ok = 0;	//�������� ������
				//���ڴ� �ڰ�����				
				//check_dir = (CCW_priority_mode == 0) ? 'U' : 'D';
				check_dir = (CCW_priority_mode == 0) ? 'D' : 'U';//CCW_priority_mode�� 0�̸� check_dir�� U�Ҵ� �ƴϸ� D�Ҵ� 
				//if((Change_encoder_avr >= 2000) && (Change_encoder_avr <= 4000) && (Encoder_dir == check_dir))	//diff�� 2000~4000 �����Ǵ�
				
				// ������ 51.23�� -> 74.2�� Change_encoder_avr = 679, ������ 74.2�� -> 53.4�� Change_encoder_avr = 1020 
				if((Change_encoder_avr >= 300) && (Change_encoder_avr <= 1500) && (Encoder_dir == check_dir))	//22�� �������� �� diff�� 500~1500 �����Ǵ� 24.05.24 �����Ͽ���.
					CW_encoder_check_ok = 1;	//���ڴ� ����							//ȸ������ Ȯ��
				else
					CW_encoder_check_ok = 0;	//���ڴ� ������
				//����üũ �����
				if(k <= 39 && save_f)
				{
					save_f = 0;
					//angle_change[k++] = Change_load_avr;
				}
				t100us_cnt = 0;	//0.1ms ī���� �ʱ�ȭ
				shift_step = 0;	//shift_step �ʱ�ȭ
				check_step = 6;	//��ȭ�� ���� �ѹ��� ����ϱ� ���ؼ�
				break;
			case 6:	//����üũ�� �� ����
				Bf_pm_avr = P_M_actual;		//����üũ�ϱ� ���� �������� �� ����
				Bf_ct_avr = Current_actual;	//����üũ�ϱ� ���� �������� �� ����
				Bf_encoder_avr = diff;		//����üũ�ϱ� ���� ���ڴ� �� ����
				break;	
			case 7:	//CCW ����üũ ����(���� ����)
				if(++t100us_cnt >= 100)
				{
					t100us_cnt = 0;
					shift_step++;
					if(shift_step >= 21)
						shift_step = 20;
				}
				break;
			case 8:	//CCW ����üũ ��(���� ��)
				t100us_cnt = 0;
				shift_step = 20;	//�ִ� �ӵ� ����
				Af_ct_avr = Current_actual;	//����üũ ��, �������� �� ����	(�������� ���� ���Ͱ� �������� �� ������)
				break;
			case 9:	//CCW ����üũ ��(���� ����)
				if(++t100us_cnt >= 100)
				{
					t100us_cnt = 0;
					shift_step--;
					if(shift_step <= 0)
						shift_step = 0;
				}
				break;
			case 10://CCW ����üũ ��
				Af_pm_avr = P_M_actual;		//����üũ ��, �������� �� ����
				Af_encoder_avr = diff;		//����üũ ��, ���ڴ� �� ����
				Change_pm_avr = fabs(Bf_pm_avr - Af_pm_avr);			//����üũ ��,�� �������� �� ��ȭ��
				Change_ct_avr =	fabs(Af_ct_avr - Bf_ct_avr);			//����üũ ��,�� �������� �� ��ȭ�� fbas:����
				Change_encoder_avr = Af_encoder_avr - Bf_encoder_avr;	//����üũ ��,�� ���ڴ� �� ��ȭ��
				//�������� �ڰ�����
				if((Change_pm_avr >= 10.0) && (Change_pm_avr <= 80.0))// �뷫 20 ~35�� ���� ������ ����üũ ����
					CCW_pm_check_ok = 1;	//�������� ����
				else
					CCW_pm_check_ok = 0;	//�������� ������
				//�������� �ڰ�����
				if((Change_ct_avr >= 50) && (Change_ct_avr <= 800))	//20 ~ 300mA �����Ǵ�
					CCW_ct_check_ok = 1;//�������� ����
				else
					CCW_ct_check_ok = 0;//�������� ������
				//���ڴ� �ڰ�����		
				//check_dir = (CCW_priority_mode == 0) ? 'D' : 'U';
				check_dir = (CCW_priority_mode == 0) ? 'U' : 'D';
				// ������ 51.23�� -> 74.2�� Change_encoder_avr = 679, ������ 74.2�� -> 53.4�� Change_encoder_avr = 1020 
				//if((Change_encoder_avr >= 2000) && (Change_encoder_avr <= 4000) && (Encoder_dir == check_dir))	//diff�� 2000~4000 �����Ǵ�
				if((Change_encoder_avr >= 300) && (Change_encoder_avr <= 1500) && (Encoder_dir == check_dir))	//22�� �������� �� diff�� 500~1500 �����Ǵ� 24.05.24 �����Ͽ���.	
					CCW_encoder_check_ok = 1;	//���ڴ� ����					//ȸ������ Ȯ��
				else
					CCW_encoder_check_ok = 0;	//���ڴ� ������
				t100us_cnt = 0;	//1ms ī���� �ʱ�ȭ
				shift_step = 0;	//shift_step �ʱ�ȭ
				Restart_self_check = 0;
				Error_Check = 0;
				Start_self_check = 0;
				Self_check_end = 1;
				PWM_SLEEP;
				self_checking = 0;//����üũ ������ ���� ��
				_SENSOR_DATA.state = 0x01;// ����üũ�� ������ ����� ���·� ����
				break;
			default:
				break;
		}
		//check_step�� ���� ���� PWM ����
		if((check_step == 1) || (check_step == 3))	//CW ���� �Ǵ� ����
		{
			if(CCW_priority_mode)
			{
				//�ݽð����
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);           
				//htim4.Instance->CCR1 = 540 + 54*shift_step;  			//duty 30% ~ 60% (���� �Ǵ� ����)//�μ� ����
				Motor_PWM = 300 + 108*shift_step;  	//duty 30% ~ 60% (���� �Ǵ� ����) // pwm�ʱⰪ�� ���̸� �׸�ŭ �̵��ϴ� ������ ������ �پ�� shift_step*x���� x���� �ø�
				//Motor_PWM = 1080 + 54*shift_step;  			//duty 30% ~ 60% (���� �Ǵ� ����) 
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);       
				htim4.Instance->CCR2 = 0;
				Motor_PWM_CCW = (unsigned int)(-0.5*Motor_PWM + 1800);
				TIM8 -> CCR1 = Motor_PWM_CCW; 				
			}
			else
			{
				//�ð����
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);              
				htim4.Instance->CCR1 = 	0; 								
				//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);       
				//htim4.Instance->CCR2 = 540 + 54*shift_step;  			//duty 30% ~ 60% (���� �Ǵ� ����)
				Motor_PWM = 300 + 108*shift_step;	//duty 30% ~ 60% (���� �Ǵ� ����) // pwm�ʱⰪ�� ���̸� �׸�ŭ �̵��ϴ� ������ ������ �پ�� shift_step*x���� x���� �ø�
				//Motor_PWM = 1080 + 54*shift_step;  			//duty 30% ~ 60% (���� �Ǵ� ����)
				Motor_PWM_CW = (unsigned int)(0.5*Motor_PWM + 1800);
				TIM8 -> CCR1 = Motor_PWM_CW; 				
			}
		}
		else if((check_step == 7) || (check_step == 9))	//CCW ���� �Ǵ� ����
		{
			if(CCW_priority_mode)
			{
				//�ð����
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);              
				//htim4.Instance->CCR1 = 0;
				TIM4_CCR1_buffer = 0;
				//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);       
				//htim4.Instance->CCR2 = 540 + 54*shift_step;  			//duty 30% ~ 60% (���� �Ǵ� ����)
				Motor_PWM = 300 + 108*shift_step;  	//duty 30% ~ 60% (���� �Ǵ� ����) // pwm�ʱⰪ�� ���̸� �׸�ŭ �̵��ϴ� ������ ������ �پ�� shift_step*x���� x���� �ø�
				//Motor_PWM = 1080 + 54*shift_step;  			//duty 30% ~ 60% (���� �Ǵ� ����) 
				Motor_PWM_CW = (unsigned int)(0.5*Motor_PWM + 1800);
				TIM8 -> CCR1 = Motor_PWM_CW; 				
			}
			else
			{
				//�ݽð����
				TIM4_CCR2_buffer = 0;
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);           
				//htim4.Instance->CCR1 = 540 + 54*shift_step;  			//duty 30% ~ 60% (���� �Ǵ� ����)
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);       
				//htim4.Instance->CCR2 = 0;
				Motor_PWM = 300 + 108*shift_step;  //duty 30% ~ 60% (���� �Ǵ� ����) // pwm�ʱⰪ�� ���̸� �׸�ŭ �̵��ϴ� ������ ������ �پ�� shift_step*x���� x���� �ø�
				//Motor_PWM = 1080 + 54*shift_step;  		//duty 30% ~ 60% (���� �Ǵ� ����)
				Motor_PWM_CCW = (unsigned int)(-0.5*Motor_PWM + 1800);
				TIM8 -> CCR1 = Motor_PWM_CCW; 			
			} 
		}
		else if((check_step == 6) || (check_step == 10))	//���� ��ȯ(1�� �Ͻ�����) �Ǵ� ����
		{
			//�Ͻ�����
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
		//����üũ �����
		if(record_en && k <= 40)
			retry_f = 1;	//��������
	}
	//����üũ �����
	if(retry_f == 1)
	{
		Error_Check = 1;
		retry_f = 0;
	}
*/
	/*****************************************�߰��� �κ�(����)*************************************************/

	if(Ble_Rcv_ok)
		Ble_Rcv_ok = (Ble_Rcv_ok<<4) | Ble_Rcv_ok; // 0x03 : 0x33	
	
	/*****************************************�߰��� �κ�(����)*************************************************/
}//Ÿ�̸� 3���ͷ�Ʈ�� ��
void TIM5_IRQHandler(void)
{//0.01msŸ�̸� �ð� ������
  /* USER CODE BEGIN TIM5_IRQn 0 */
	TIM5->SR &= ~TIM_IT_UPDATE;
  /* USER CODE END TIM5_IRQn 0 */
	if(TIM5->SR & TIM_IT_UPDATE) 
		TIM5->SR &= ~TIM_IT_UPDATE; // ���ͷ�Ʈ �÷��� Ŭ����
  /* USER CODE BEGIN TIM5_IRQn 1 */

  /* USER CODE END TIM5_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
/*****************************************�߰��� �κ�(����)*************************************************/
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
void USART2_IRQHandler(void)//�ζ�
{
  /* USER CODE BEGIN USART2_IRQn 0 */
	byte rx2_data;
	if((USART2->CR1 & USART_CR1_RXNEIE) && (USART2-> SR & USART_SR_RXNE))//���Ź��۰� ä������ ��
    {
        Rcv2_cnt++;
        rx2_data = USART2->DR;	// ���Ű� ����

		lora_test_index[rx2_index++] = rx2_data;
		
		if(LoRa_CheckSum_ing == 0)
		{
		  switch(Rx2_step)
		  {
		   case 0 :
			//24.08.05
			if(rx2_data == ENQ && rx2_data != 0x00)// 0�� �ε��� = ������ ���� ������ 0�� �ƴ϶��
			{
			  LoRa_Rxbuf[0] = rx2_data;
			  Rx2_step = 1;
			  Rx2_index = 1;
			}
			else
			{
			  Rx2_step = 0;
			  Rx2_index = 0;
			  // 1ms ī����
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
			  Rx2_step = 10; // Wtrite n�� data ���� mode
			  Rx2_next_data_no = LoRa_Rxbuf[5]*2; // Write n�� data number
			}
			else
			{
			  Rx2_step++; //CRC16_Low ����
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
			
		   case 8 : // ���� ������ Slave LoRa�� ���� �޾ƿ�(Low)
			Rx2_RS_L = rx2_data;
			Rx2_step++;
			break;
			
		   case 9: // ���� ������ Slave LoRa�� ���� �޾ƿ�(High)
			Rx2_RS_H = rx2_data;
			_LoRa_Data.reception_sensitivity = (Rx2_RS_L<<8) | Rx2_RS_H;	
			LoRa_CheckSum_EN = 1; // main.c ���� CRC16 ���
			LoRa_CheckSum_ing = 1; // UART2 ���� �Ұ���
			Rx2_step = 0;
			Rx2_index = 0;
			break;
			
		   default:
			Rx2_step = 0;
			Rx2_index = 0;
			break;
			
		   case 10: // Write data ����
			LoRa_Rxbuf[Rx2_index++] = rx2_data;
			if(Rx2_next_data_no <= 0)
			{
			  Rx2_step = 6; // CRC16_Low ����
			}
			else
			{
			  Rx2_next_data_no--;
			}
			break;
		  }
		}
	}
	else if((USART2->CR1 & USART_CR1_TCIE) && (USART2-> SR & USART_CR1_TCIE))	//�۽Ź��۰� ä������ �� /// �۽� �Ǵ��� Ȯ���Ұ�!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
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
		else	//��� Data ����
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

/*****************************************�߰��� �κ�(����)*************************************************/
word BCC_error_cnt = 0,LoRa_BCC_error_cnt = 0; byte Tx_lora_data = 0;
void LoRa_UART2_exe(void)		//Call by main.c (UART2)
{
	byte i, k, no;
	word wtemp;
	byte *bptr_lora;
	if(LoRa_Rcv_ok == 0x110)	//���� ACK
	{
		LoRa_Rcv_ok = 0;  
		bptr_lora = (byte *)(0x20000000);
		bptr_lora = (byte *)(0x20000000 + (LoRa_Rxbuf[2]<<8) + LoRa_Rxbuf[3]);		 	
		//bptr_lora = (byte *)(LORA_MEMORY);
		if(!Motor_BUSY)		//���Ͱ� �Ͻ����� Soft stop���� ���� ����ü�� ������ ���� ����
		{
			no = LoRa_Rxbuf[5]*2;	//Write number
			for(i = 0; i < no; i++)
			{
				*bptr_lora = LoRa_Rxbuf[7+i];	//n�� write data...
				bptr_lora++;
			}
			
			Tx_lora_data = 1;
			/*if(!Priority_dir_set)	
				Priority_dir_set = 1;	//����üũ ���� ��������
			/*****���� �Ϸ�*****/
		}

		if(_LoRa_Data.state == 1 && _LoRa_Data.measure_check == 0) // ���� ����̹��� ������϶��� PC�� �̿��� �����͸� ���� ����.
		{
			//ACK Data Sending
			LoRa_Tx_buf[0] = LoRa_Rxbuf[0];	//Station(���� 1byte) 0x01
			LoRa_Tx_buf[1] = LoRa_Rxbuf[1];	//Function(�Լ� 1byte) ���� : 0x10			
			LoRa_Tx_buf[2] = LoRa_Rxbuf[2];	//High Address(1byte) ���� ������ ����
			LoRa_Tx_buf[3] = LoRa_Rxbuf[3];	//Low Adddress(1byte)
			LoRa_Tx_buf[4] = LoRa_Rxbuf[4];	//Length High(1byte) ���� ����
			LoRa_Tx_buf[5] = LoRa_Rxbuf[5];	//Length Low(1byte)
			
			LoRa_CheckSum_data = CRC16(LoRa_Tx_buf, 6);	//6�� �������� CRC16 ��� 
					
			LoRa_Tx_buf[6] = LoRa_CheckSum_data & 0xFF;	    //CRC's Low byte	
			LoRa_Tx_buf[7] = (LoRa_CheckSum_data >> 8);		//CRC's High byte	
			
			LoRa_RS232_putchar(8);		//Total 8 byte sending	
			copyLoRaDataToSensorData();//�ζ� ������ -> ���� ������
		}
		else if(_LoRa_Data.state == 0 || _LoRa_Data.state == 2) // ���� ����̹��� ����̰ų� �����̸� PC�� �̿��� �����͸� ���� ������ ���� �ڵ带 PC�� ����.
		{
		  LoRa_Tx_buf[0] = LoRa_Rxbuf[0];	//Station(���� 1byte) 0x01
		  LoRa_Tx_buf[1] = LoRa_Rxbuf[1] | 0x80;	//The Function Code ( 0x10 + 0x80 = 0x90 )
		  LoRa_Tx_buf[2] = 0x01;	//The Exception Code( ���� �ڵ� 0x01 : �����̺갡 �� ������ ��û�� ó���� �� ���� �߸��� ���¿� ������ ��Ÿ��. )
		  
		  LoRa_CheckSum_data = CRC16(LoRa_Tx_buf, 3);	//3�� �������� CRC16 ��� 
		  
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
		  LoRa_Tx_buf[0] = LoRa_Rxbuf[0];	//Station(���� 1byte) 0x01
		  LoRa_Tx_buf[1] = LoRa_Rxbuf[1];	//Function(�Լ� 1byte) ���� : 0x10			
		  LoRa_Tx_buf[2] = LoRa_Rxbuf[2];	//High Address(1byte) ���� ������ ����
		  LoRa_Tx_buf[3] = LoRa_Rxbuf[3];	//Low Adddress(1byte)
		  LoRa_Tx_buf[4] = LoRa_Rxbuf[4];	//Length High(1byte) ���� ����
		  LoRa_Tx_buf[5] = LoRa_Rxbuf[5];	//Length Low(1byte)

		  LoRa_CheckSum_data = CRC16(LoRa_Tx_buf, 6);	//6�� �������� CRC16 ��� 

		  LoRa_Tx_buf[6] = LoRa_CheckSum_data & 0xFF;	    //CRC's Low byte	
		  LoRa_Tx_buf[7] = (LoRa_CheckSum_data >> 8);		//CRC's High byte	

		  LoRa_RS232_putchar(8);		//Total 8 byte sending	
		}		
	}
	else if(LoRa_Rcv_ok == 0x44) // �б� ACK
	{
		LoRa_Rcv_ok = 0;
		LoRa_Tx_buf[0] = LoRa_Rxbuf[0];	//Station(���� 1byte) 0x01
		LoRa_Tx_buf[1] = LoRa_Rxbuf[1];	//Function(�Լ� 1byte) �б� : 0x04	
		
		if(LoRa_Rxbuf[1] == 0x04)	//�б� ��û �Լ��ڵ�(0x04)
		{
			LoRa_Rcv_ok = 0;
			LoRa_Tx_buf[0] = LoRa_Rxbuf[0];	//Station(���� 1byte) 0x01
			LoRa_Tx_buf[1] = LoRa_Rxbuf[1];	//Function(�Լ� 1byte) �б� : 0x04	
			
			LoRa_Tx_buf[2] = LoRa_Rxbuf[5]*2;    // The number of data bytes
				
			wtemp = (LoRa_Rxbuf[2]<<8) + LoRa_Rxbuf[3];	//Read Address	 
			bptr_lora = (byte *)(LORA_MEMORY + (LoRa_Rxbuf[2]<<8) + LoRa_Rxbuf[3]);	
			
			i = 3;
			for(k = 0; k < LoRa_Tx_buf[2]; k++)		//LoRa_Rxbuf[5] = Read number
			{
				LoRa_Tx_buf[i++] = *bptr_lora++;
			}
			//CRC(Hex)
			LoRa_CheckSum_data = CRC16(LoRa_Tx_buf, i);	//�۽��� �������� CheckSum��� 
			
			LoRa_Tx_buf[i++] = LoRa_CheckSum_data & 0xFF;	    //CRC's Low byte	
			LoRa_Tx_buf[i++] = (LoRa_CheckSum_data >> 8);		//CRC's High byte	
			
			LoRa_RS232_putchar(i);		//Total i byte sending	
		}
	}
	else if(LoRa_Rcv_ok==0xEE)	//BCC Error
	{
		LoRa_Rcv_ok = 0;
		BCC_error_cnt++;
		LoRa_Tx_buf[0] = LoRa_Rxbuf[0];	//����	
		LoRa_Tx_buf[1] = LoRa_Rxbuf[1];	//�Լ�
		LoRa_Tx_buf[2] = LoRa_Rxbuf[5];	//data length(low_length)
		LoRa_Tx_buf[3] = 0xEE;			//Error Cmd	(0xEE:CRC Error)
		LoRa_Tx_buf[4] = 0xEE;			//Error Cmd	(0xEE:CRC Error)
		LoRa_Tx_buf[5] = 0xEE;			//Error Cmd	(0xEE:CRC Error)

		wtemp = CRC16(LoRa_Tx_buf, 6);	//�� 6byte [0]~[5]
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
/*****************************************�߰��� �κ�(����)*************************************************/
byte Rx3_buf[100];
byte Rx3_index = 0, Rcv3_cnt = 0, Rx4_next_data_no;
word CheckSum_data;
byte Tx3_send_number=0;
byte Tx3_index=0;
byte Rx3_step = 0;

void USART3_IRQHandler(void)
{/* USER CODE BEGIN UART3_IRQn 0 */
	byte rx3_data;
    if((USART3->CR1 & USART_CR1_RXNEIE) && (USART3-> SR & USART_SR_RXNE))	// ���Ź��۰� ä�����ٸ�?	
    {
		Rcv3_cnt++;
        rx3_data = USART3->DR;	// ���Ű� ����
        ble_test_rxbuf[ble_test_index++] = rx3_data;
		switch(Rx3_step) 
		{
			case 0:
				if(rx3_data == 'R') 
				{
					Rx3_buf[0] = rx3_data;	//Rx_Buf[0] = ����(0x01)
					Rx3_step = 1;
					Rx3_index = 1;
				}
				else
				{
					Rx3_step = 0;
					Rx3_index = 0;
				}
				break;
			case 1: //�Լ� 
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
		else	//��� Data ���� �Ͽ���.
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
typedef union {//float ���� byte �迭�� ��ȯ
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
		
		Ble_Data_Tx(16);		//������� ������ ����(Total 16 byte sending)	
		
		if(_Ble_Data.exer_state == 1 || _Ble_Data.exer_state == 2) // � �����̰ų� ������ �Ϸ�Ǿ�����, ��ȣ�� �ѹ� ������ �ٽ� 0���� �����.
		{
		  _Ble_Data.exer_state = 0;// ����
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
dword uart4_interrupt_scan = 0;	//UART4 ���ͷ�Ʈ ī����
byte Rx4_CRC_H = 0, Rx4_CRC_L = 0;
byte CheckSum_ing = 0;
//word Rx4_data_sum = 0; //24.04.11 ������
void UART4_IRQHandler(void)
{
  /* USER CODE BEGIN UART4_IRQn 0 */
	byte rx4_data;
    uart4_interrupt_scan++;
    if((UART4->CR1 & USART_CR1_RXNEIE) && (UART4-> SR & USART_SR_RXNE))	// ���Ź��۰� ä�����ٸ�?		����
    {
        Rcv4_cnt++;
        rx4_data = UART4->DR;	// ���Ű� ����
        test_rxbuf[test_index++] = rx4_data;
		
		if(CheckSum_ing == 0)
		{
			switch(Rx4_step) 
			{
				case 0:
					if(rx4_data == UART_STX) //����(0x01)�� ��ġ?
					{
						//Rx4_data_sum = 0; //24.04.11 ������
						Rx4_buf[0] = rx4_data;	//Rx_Buf[0] = ����(0x01)
						Rx4_step = 1;
						Rx4_index = 1;
						//Rx4_data_sum += rx4_data; //24.04.11 ������
					}
					else
					{
						Rx4_step = 0;
						Rx4_index = 0;
					}
					break;
				case 1: //�Լ� 
					if((rx4_data == 0x04) || (rx4_data == 0x10) || (rx4_data == 0x06) || (rx4_data == 0x08)) //function 0x04: Read Request
					{															 	   						//function 0x10: Write Request
															 						  						//function 0x06: Calibration mode, function 0x08: voice mode 
						Rx4_buf[Rx4_index++] = rx4_data;
						Rx4_step++;
						//Rx4_data_sum += rx4_data; //24.04.11 ������
					}
					else                                                    
					{
						static word ng_cnt = 0;
						ng_cnt++;
						Rx4_step = 0;     //Waiting for  ����
						Rx4_index = 0;
					}
					break;
				case 2: //Waiting Address (High Address byte)
					Rx4_buf[Rx4_index++] = rx4_data;
					Rx4_step++;
					//Rx4_data_sum += rx4_data; //24.04.11 ������
					break;
				
				case 3: //Waiting Address  (Low Address byte)
					if(rx4_data == 0xEE)
					{						
						Rx4_step = 0;     //Waiting for  ����
						Rx4_index = 0;
						//Rx4_data_sum += rx4_data; //24.04.11 ������
					}
					else                                                    
					{
						Rx4_buf[Rx4_index++] = rx4_data;
						Rx4_step++;
					}
					Error4_cnt++;
					break;	
				
				case 4: //Waiting ���� (High Length byte)
					Rx4_buf[Rx4_index++] = rx4_data;
					Rx4_step++;
				
					//Rx4_data_sum += rx4_data; //24.04.11 ������
					break;
					
				case 5://Waiting ���� (Low Length byte)
					Rx4_buf[Rx4_index++] = rx4_data;
					Rx4_step++;
					if(Rx4_buf[1]==0x04)
						Rx4_step=7;
					//Rx4_data_sum += rx4_data; //24.04.11 ������
					break;
					
				case 6: //���� ��û�� ������ �� (The number of data bytes)
					Rx4_buf[Rx4_index++] = rx4_data;
					//Rx4_data_sum += rx4_data;//24.04.11 ������
					if((Rx4_buf[1] == 0x10) || (Rx4_buf[1] == 0x06) ||(Rx4_buf[1] == 0x08))	//Write or Calibration command
					{

						Rx4_step = 10;		//Write n�� data ���� mode
						Rx4_next_data_no = Rx4_buf[5]*2;//Write n�� data number	
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
					//Rx4_data_sum += rx4_data; //24.04.11 ������
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

				case 10:	//Write data ����
					Rx4_buf[Rx4_index++] = rx4_data; //������ �Ѹ�
					//Rx4_data_sum += rx4_data; //24.04.11 ������
					if(Rx4_next_data_no <= 1)
					{
						Rx4_step = 7;	//CRC16_Low ����
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
		else	//��� Data ���� �Ͽ���.
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
// ���찡 �߰���
word Sw_Reset_Fg = 0;
void UART4_exe(void)		//Call by main.c (UART4)
{
	byte i, j, k, no;
	word wtemp;
	byte *bptr;
	if(Rcv2_ok == 0x110)	//���� ACK
	{
		Rcv2_ok = 0;  
		bptr = (byte *)(USER_RUN_ADDRESS + (Rx4_buf[2]<<8) + Rx4_buf[3]);	//�ּҼ���	
		if(!Motor_BUSY)//���Ͱ� �Ͻ����� Soft stop���� ���� ����ü�� ������ ���� ����
		{
			no = Rx4_buf[5]*2;	//Write number
			for(i = 0; i < no; i++)
			{
				*bptr = Rx4_buf[7+i];	//n�� write data...
				bptr++;
			}
			/*if(!Priority_dir_set)	
				Priority_dir_set = 1;	//����üũ ���� ��������*/
			
			//24.09.13 ���� �ΰ��� ������ ����üũ ������ 
			if(!Priority_dir_set && _USER_RUN_SETTING.calibration != 2)
			{
				Priority_dir_set = 1;	//����üũ ���� ��������
			}
 
			/*****���� �Ϸ�*****/
		}
		_SENSOR_DATA.cnt = 0;			//�Ƚ�� �ʱ�ȭ
		_SENSOR_DATA.remain_time = 0;	//���� ��ð� �ʱ�ȭ
		//ACK Data Sending
		Tx4_buf[0] = Rx4_buf[0];	//Station(���� 1byte) 0x01
		Tx4_buf[1] = Rx4_buf[1];	//Function(�Լ� 1byte) ���� : 0x05			
		Tx4_buf[2] = Rx4_buf[2];	//High Address(1byte) ���� ������ ����
		Tx4_buf[3] = Rx4_buf[3];	//Low Adddress(1byte)
		Tx4_buf[4] = Rx4_buf[4];	//Length High(1byte) ���� ����
		Tx4_buf[5] = Rx4_buf[5];	//Length Low(1byte)
		
		CheckSum_data = CRC16(Tx4_buf, 6);	//6�� �������� CheckSum��� 
		Tx4_buf[6] = CheckSum_data & 0xFF;	    //CRC's Low byte	
		Tx4_buf[7] = (CheckSum_data >> 8);		//CRC's High byte	
		
		RS232_putchar(8);		//Total 8 byte sending ������ ����		
		
		if(LoRa_Equipment_Num)// ���͵���̹� ���� ���� On�� LoRa_Equipment_Num�� 1 �� �ѹ��� �ʱ� ���� ���� ���͵���̹����� Slave_LoRa�� ������.
		{
		  LoRa_Equipment_Num = 0;
		  ENQ = _USER_RUN_SETTING.equipment_num; // ���� ������ LCD�� EEPROM�� ����Ǿ��ִ� ����ȣ�� ����
		  LoRa_Tx_buf[0] = _USER_RUN_SETTING.equipment_num;	//���� ���� On�� Slave LoRa ������ ����
			
		  LoRa_RS232_putchar(1);		//Total i byte sending	
		  Bf_Equipment_Num = _USER_RUN_SETTING.equipment_num;
		  
		  if(ENQ < 0x64) // 100���� ������
		  {
		  	sprintf((char*)AT_BTNAME, "AT+NAMEARTUS-840-%02d", ENQ);
		  }
		  else if(ENQ >= 0x64) // 100���� ũ�ų� ������
		  {
			sprintf((char*)AT_BTNAME, "AT+NAMEARTUS-840-%03d", ENQ);
		  }
		  ble_name_change_fg = 1; // ������� �̸� ����
		}
	}
	else  if(Rcv2_ok == 0x44) // �б� ACK
	{
		Rcv2_ok = 0;
		bptr = (byte *)(SENSOR_ADDRESS + Rx4_buf[3]); //�ּҼ���	 
		Tx4_buf[0] = Rx4_buf[0];//Station(���� 1byte) 0x01
		Tx4_buf[1] = Rx4_buf[1];//Function(�Լ� 1byte) �б� : 0x04	
		if(Rx4_buf[1] == 0x04)	//�б� ��û �Լ��ڵ�(0x04)
		{
			Rcv2_ok = 0;
			Tx4_buf[0] = Rx4_buf[0];	//Station(���� 1byte) 0x01
			Tx4_buf[1] = Rx4_buf[1];	//Function(�Լ� 1byte) �б� : 0x04	
			
			Tx4_buf[2] = Rx4_buf[5] * 2;    //Master���� �б� ��û�� ������ ��(���� : word)

			wtemp = (Rx4_buf[2]<<8) + Rx4_buf[3];	//Read Address	 
			bptr = (byte *)(SENSOR_ADDRESS + wtemp);
			
			i = 3;
			for(k = 0; k < Tx4_buf[2]; k++)		//Rx4_buf[5] = Read number
			{
				Tx4_buf[i++] = *bptr++;
			}
			//CRC(Hex)
			CheckSum_data = CRC16(Tx4_buf, i);	//�۽��� �������� CheckSum��� 
			Tx4_buf[i++] = CheckSum_data & 0xFF;	    //CRC's Low byte	
			Tx4_buf[i++] = (CheckSum_data >> 8);		//CRC's High byte	
			
			RS232_putchar(i);		//Total i byte sending	
							
			if(_SENSOR_DATA.upper_limit_angle || _SENSOR_DATA.lower_limit_angle)
			{
				_SENSOR_DATA.upper_limit_angle = 0;
				_SENSOR_DATA.lower_limit_angle = 0;
			}
			/**07.02 _SENSOR_DATA.lora_wr_fg�� 0�̸� LCD�� PC�����͸� ������ ����.**/
			if(_SENSOR_DATA.lora_wr_fg)
			{
			  	_SENSOR_DATA.lora_wr_fg = 0;
			}
			
			//���찡 �߰���
			static byte succed_tx = 0;
			succed_tx = 1;//stop, switch ������ lcd�� ���� �Ϸ�  �ϸ� ����ġ ���� �����͸� �ʱ�ȭ ������
			if(!Succed_Tx_Cnt) // Sw ������ 500ms�Ŀ� ���� ��. 
			{
				if(succed_tx)//lcd�� start, stop ����ġ ��ȣ ���������� ����
				{
					succed_tx = 0;
					_SENSOR_DATA.stop_sw = 0;
					_SENSOR_DATA.start_sw = 0;
				}
			}
			
			//09.20 ���찡 ��
			static byte succed_ems_tx = 0;
			succed_ems_tx = 1;//��� ���� ����ġ ������ lcd�� ���� �Ϸ� �ϸ� ����ġ ���� �����͸� �ʱ�ȭ ������
			if(!TX_EMS_CNT) // EMS_Sw ������ 500ms�Ŀ� ���� ��. 
			{
				if(succed_ems_tx)//lcd�� EMS_Sw ����ġ ��ȣ�� ���������� ����
				{
					succed_ems_tx = 0;
					EMS_STATE = 0; // EMS ����ġ ���¸� �ȴ��� ���·� ����
				}
			}
		}
	}
	else if(Rcv2_ok==0x66)	//Calibration ACK
	{
		Rcv2_ok = 0;
		//����κ�
		bptr = (byte *)(USER_RUN_ADDRESS + (Rx4_buf[2]<<8) + Rx4_buf[3]);		
		no =  Rx4_buf[5] * 2;	//Write number
		for(i = 0; i < no; i++)
		{
			*bptr = Rx4_buf[7+i];	//n�� write data...
			bptr++;
		}
		_USER_RUN_SETTING.exerc_start = 0x00;
		
		
		Tx4_buf[0] = Rx4_buf[0];	//Station(���� 1byte) 0x01
		Tx4_buf[1] = Rx4_buf[1];	//Function(�Լ� 1byte) Calibration : 0x06
		Tx4_buf[2] = Rx4_buf[2]; 	//High Address(1byte)
		Tx4_buf[3] = Rx4_buf[3];	//Low Adddress(1byte)
		
		//�б�κ�
		wtemp = (Rx4_buf[2]<<8) + Rx4_buf[3];	//Read Address	 
		bptr = (byte *)(_CALIBRATION_ADDRESS + wtemp);	
		
		//AD_ok 0���� �ʱ�ȭ�ϴ� �κ�
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
		CheckSum_data = CRC16(Tx4_buf, j);	//�۽��� �������� CheckSum��� 
		
		Tx4_buf[j++] = CheckSum_data & 0xFF;	    //CRC's Low byte	
		Tx4_buf[j++] = (CheckSum_data >> 8);		//CRC's High byte	
		
		RS232_putchar(j);		//Total j byte sending	
	}

	else if(Rcv2_ok==0xEE)	//BCC Error
	{
		Rcv2_ok = 0;

		BCC_error_cnt++;
		Tx4_buf[0] = Rx4_buf[0];	//����	
		Tx4_buf[1] = Rx4_buf[1];	//�Լ�
		Tx4_buf[2] = Rx4_buf[5];	//data length(low_length)
		Tx4_buf[3] = 0xEE;			//Error Cmd	(0xEE:CRC Error)
		Tx4_buf[4] = 0xEE;			//Error Cmd	(0xEE:CRC Error)
		Tx4_buf[5] = 0xEE;			//Error Cmd	(0xEE:CRC Error)

		wtemp = CRC16(Tx4_buf, 6);	//�� 6byte [0]~[5]
		Tx4_buf[6] = wtemp & 0xFF;		//CRC's Low byte	
		Tx4_buf[7] = (wtemp>>8);		//CRC's High byte	
		RS232_putchar(8);				//Total 8 byte sendding	[0]~[7] Total 8 byte sending.
	}	
	else if(Rcv2_ok == 0x88)	//�������� �ޱ⸸ �����ϱ⿡ �����۾� ���� �ʾ���. 
	{	
		Rcv2_ok = 0;  
		bptr = (byte *)(VOICE_DATA_ADDRESS + (Rx4_buf[2]<<8) + Rx4_buf[3]);	//�ּҼ���			
		no = Rx4_buf[5]*2;	//Write number
		for(i = 0; i < no; i++)
		{
			*bptr = Rx4_buf[7+i];	//n�� write data...
			bptr++;
		}	
		//ACK Data Sending
		Tx4_buf[0] = Rx4_buf[0];	//Station(���� 1byte) 0x01
		Tx4_buf[1] = Rx4_buf[1];	//Function(�Լ� 1byte) ���� : 0x08			
		Tx4_buf[2] = Rx4_buf[2];	//High Address(1byte) ���� ������ ����
		Tx4_buf[3] = Rx4_buf[3];	//Low Adddress(1byte)
		Tx4_buf[4] = Rx4_buf[4];	//Length High(1byte) ���� ����
		Tx4_buf[5] = Rx4_buf[5];	//Length Low(1byte)
		CheckSum_data = CRC16(Tx4_buf, 6);	//6�� �������� CheckSum��� 
		Tx4_buf[6] = CheckSum_data & 0xFF;	    //CRC's Low byte	
		Tx4_buf[7] = (CheckSum_data >> 8);		//CRC's High byte	
		RS232_putchar(8);		//Total 8 byte sending ������ ����		
				
		/*if(Bf_Equipment_Num != _VOICE_DATA.equipment_num && _VOICE_DATA.equipment_num != 0x00)
		{
		  ENQ = _VOICE_DATA.equipment_num; // ���� ������ LCD�� ȯ�漳������ ������ ����ȣ ���� ����
		  LoRa_Tx_buf[0] = _VOICE_DATA.equipment_num;	//LCD�� ȯ�漳������ ����ȣ�� �ٲپ����� �� ����ȣ�� Slave LoRa�� ������ LoRa�� �������� �����.
		  
		  LoRa_RS232_putchar(1);		//Total i byte sending	
		  Bf_Equipment_Num = _VOICE_DATA.equipment_num;
		  _USER_RUN_SETTING.equipment_num = _VOICE_DATA.equipment_num;
		}*/
		
		if(Bf_Equipment_Num != _VOICE_DATA.equipment_num && _VOICE_DATA.equipment_num != 0x00)
		{
		  LoRa_Tx_buf[0] = ENQ;		//���� ���� On�� Slave LoRa ������ ����
		  LoRa_Tx_buf[1] = 0x07;	//��� ��ȣ �ٲٴ� �Լ�
		  LoRa_Tx_buf[2] = _VOICE_DATA.equipment_num;	//���� ���� On�� Slave LoRa ������ ����
		  
		  LoRa_CheckSum_data = CRC16(LoRa_Tx_buf, 3);	//3�� �������� CheckSum��� 
		  
		  LoRa_Tx_buf[3] = LoRa_CheckSum_data & 0xFF;	    //CRC's Low byte	
		  LoRa_Tx_buf[4] = (LoRa_CheckSum_data >> 8);		//CRC's High byte	
			
		  LoRa_RS232_putchar(5);		//Total i byte sending	
		  ENQ = _VOICE_DATA.equipment_num; // ���� ������ LCD�� ȯ�漳������ ������ ����ȣ ���� ����
		  Bf_Equipment_Num = _VOICE_DATA.equipment_num; // ���� ������ LCD�� EEPROM�� ����Ǿ��ִ� ����ȣ�� ����
		  
		  if(ENQ < 0x64) // 100���� ������
		  {
		  	sprintf((char*)AT_BTNAME, "AT+NAMEARTUS-840-%02d", ENQ);
		  }
		  else if(ENQ >= 0x64) // 100���� ũ�ų� ������
		  {
			sprintf((char*)AT_BTNAME, "AT+NAMEARTUS-840-%03d", ENQ);
		  }
		  ble_name_change_fg = 1; // ������� �̸� ����
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
	//CRC ���׷� ���� Check Sum���� ��ü
	word check_sum = 0;
	while(Len--)
	{
		check_sum += *Data++;
	}
	return check_sum;
}
#endif
/* USER CODE END 1 */
