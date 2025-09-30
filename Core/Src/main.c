/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "extern.h"
#include "user_define.h"
#include <stdint.h>
#include "stdlib.h"
#include <stdio.h>
#include <string.h>
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
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
PCD_HandleTypeDef hpcd_USB_FS;
/* USER CODE BEGIN PV */
uint8_t T10ms_flag=0,gbTimer10ms_cnt=0;
uint8_t T100ms_flag=0,gbTimer100ms_cnt=0;
/* USER CODE END PV */
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM8_Init(void);
static void MX_UART4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USB_PCD_Init(void);
/* USER CODE BEGIN PFP */
void Exercise_ctrl(void);
void Motor_ctrl(void); 
byte motor_delay_ms(dword delay_time);
extern void LoRa_Tx(byte lora_tx_num);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t TIM8_CCR1 = 0;
float ROM = 0.0;

#pragma location = 0x20000000 // LoRa Data�� �����ϴ� �޸� �ּ� ��
struct LoRa_Data   _LoRa_Data;

#pragma location = 0x20000100 
struct SENSOR_DATA _SENSOR_DATA;//Sensor���� Moterdriver -> LCD

#pragma location = 0x20000200
struct USER_RUN_SETTING        _USER_RUN_SETTING; //LCD -> Moterdriver

//#pragma location = 0x20000100 
//struct LCD_MD_DATA  _LCD_MD_DATA;//LCD and M

#pragma location = 0x20000400
struct CALIBRATION_SETTING		_CALIBRATION_SETTING;

#pragma location = 0x20000560
struct Voice_value _VOICE_DATA;

#pragma location = 0x20000600
struct MD_EEPROM_DATA _MD_EEPROM_DATA;


struct Ble_Data      _Ble_Data;
byte Not_into_cal = 0;
////////////////////////////////////////////

unsigned int Motor_PWM = 0;
int toggle_cnt = 0;
float set_min_angle = 0, set_max_angle = 0;
float set_conc_angle = 0;
float set_PA = 0;
byte exercise_mode = 0;
byte set_exerc_num = 2, set_speed_step = 7, set_exerc_time = 0;
byte set_repeat_num = 0;
byte set_location = 0;

extern byte test_rxbuf[255];



//PID ���� �Լ� ���� ���� ����
float P_GAIN = 0.025;//0.03//0.09//0.2//0.5//1
float I_GAIN = 0.00021;//0.01//0
float D_GAIN = 0.018;//0.1//;//0
float TIME = 1;//4;

float pControl = 0;
float iControl = 0;
float dControl = 0;
float pidControl = 0;


float error = 0, pre_error = 0;
float accError = 0;
float errorGap = 0;
//////////////////////////////

float pid_val = 0;
int speed_RPM = 0;
float target_speed = 0;
float DISTANCE = 0;
float angle_distance = 0;
byte mode_flag = 0;
word pwm_step = 0;
byte deaccel_flag = 0;
float accel_angle = 0;
float f_m_speed;
//
word acceltime = 300, deacceltime = 300;//ó�� ����, ���� �ð��� 300m������ ���ӵǴ� �ð��� �÷��� ������ȯ�� ���� ������ ������ ����, acceltime�� �ٿ��� ���ӵ��� �ð��� ����
float upper_correction_angle  = 0.0, lower_correction_angle = 0.0;//�� ����� �ٸ� ���� �����ϴ� ������
dword set_upper_limit_time = 300, set_lower_limit_time = 300; //����, ���� �����ð� 300

byte set_velocity_mode = 0;
byte move_to_lower = 0;
byte exerc_end = 0;	

//PID
#define Kp		0.025	//0.0001//0.025//0.7//0.3//0.7//0.025		//0.002//30//0.7//0.7//0.07//2///3/0.7//0.4//8//0.8//0.4//0.2//0.3//0.7//0.5             //0.7
#define Ki		20		//50//50//50//20 							//30//0.7//30//60//15//18//27//6.5//50//70//7//20//30//40//60//70//50//15//20.0//30
#define Kd		0
#define DT		0.01
#define PIconst	0.9



//EEPROM 1�������� 16byte
#define Byte_num_16 16
double PIconstant = Kp+Ki*DT;
/////////////////////MOTOR1//////////////////
unsigned int speed = 0;//signed int speed=0;
uint8_t dir=0;
uint16_t cnt1=0;
uint16_t cnt2=0;
signed int diff=0;
 
byte ADC1_index = 0;
word ADC1_data[4];

unsigned int M_speed = 0;
float set_angle_value = 0;

byte num_of_workouts = 0;
byte Motor_Run_F = 0;
byte Dir_of_rotation = 0;

//������� ����
float test_rate;
byte skip_cnt_start = 0;
byte measurement_stop = 0, gen_ada_con_back = 0;//������� �������� ����
byte CheckSum_EN = 0;
word LCD_crc = 0, MT_Driver_crc = 0;

word check_sum = 0;
byte PAUSE_F = 0;
byte Motor_BUSY = 0;

//����üũ
byte Priority_dir_set = 0;
byte Error_Check = 0;	//���͵���̹� ������ ������, �ٷ� ���� üũ ����
byte shift_step = 0;	//����üũ�� PWM ���� ����
byte CCW_priority_mode = 0;	//�ݽð� ���� �켱 üũ flag

float Bf_pm_avr = 0, Af_pm_avr = 0, Change_pm_avr = 0;	//�������� üũ
float Bf_ct_avr = 0, Af_ct_avr = 0, Change_ct_avr = 0;	//�������� üũ
int Bf_encoder_avr = 0, Af_encoder_avr = 0, Change_encoder_avr = 0;	//���ڴ� üũ
byte Encoder_dir = 0;

byte Passive_ing = 0;
//Ķ���극�̼ǰ���

float real_angle = 0;
byte except_f = 0;	//�������� (���߸���� ���Ѱ���� ����ó��)
//������� �׽�Ʈ
byte Record_F = 0;

word bf_CNT = 0, Over_cnt = 0, Under_cnt = 0;
byte voice_error_check = 0;
byte Record_Index = 0, TEST_NUM = 0, Record_data_f = 0;
float Start_angle_1[100], Start_angle_2[100];
float End_angle_1[100], End_angle_2[100];
word Over_cnt_arr[100], Under_cnt_arr[100], Current_CNT_1[100], Current_CNT_2[100];

float Angle_by_encoder_up = 0,Angle_by_encoder_down = 0 ,OFFSET_angle = 0, OFFSET_angle_2 = 0;

/**������� ���� ����**/
// ��� ���� Ȯ�� ����
unsigned int bluetooth_AT = 0,cnt_flag = 0;; 

// HOST�� BT�� ���������� ����Ǿ� �ִ��� ���θ� Ȯ��. 
unsigned char AT_OK[10] = "AT\r\n"; 

unsigned int ble_name_change_fg = 0;

// HC-06 ������� ��� �̸� �ٲٱ�_BT
unsigned char AT_BTNAME[21] = "AT+NAMEARTUS_840_00"; 

unsigned char Ble_buffer[50];

// RX ���� �迭
unsigned char RE_re_1[100]; 

// ���÷� ������ ������
unsigned char RE_data_tr_1[5]; 

// ���÷� ������ ������, 1�ʰ������� 1�� ������ ������ �����͸� ����
extern unsigned int time_count, at_tx_flag; 
unsigned int test_flag_4 =  0;
byte Not_Send_Ble = 0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval 
  */
#define DEVICE_ADDRESS 0x50 //1 ~ 128 7bit 0x50 -> 0xA0
//EEPROM
byte data_Length = 0, anlge_ad_err_flag = 0, anlge_real_err_flag, ct_err_flag = 0;
byte EEPROM_tx_buf[16], EEPROM_angle_rx_buf_1[16], EEPROM_angle_rx_buf_2[16], EEPROM_rx_buf[16];
void delayMicroseconds(uint32_t us)
{
    // ����� ���ǵ� ����ũ���� ���� �Լ�
    // �ش� �Լ��� �ణ�� ������ ���� �� �ֽ��ϴ�.
    volatile uint32_t counter = us * (SystemCoreClock / 1000000U) / 12U;
    while (counter--) {
        // ������ ���鼭 ���
        asm("nop");
    }
}
word EEPROM_crc_rx_data = 0, compare_tx_rx = 0;
byte Mute_state = 1, eeprom_crc_not_ok = 0, Arr_eeprom1[16], Arr_eeprom2[16], Arr_eeprom3[16],Arr_eeprom4[16],Arr_eeprom5[16], bytes_data[4];
//Mute_state 1 : ��� ���ɻ���
byte EEPOM_Read(byte device_rd_address, byte Size, byte start_page)
{//EEPROM �б��û
	byte *bdata;
	if(start_page == 1)
	{//������� ������ �����̸� ������ ������������ ���� x
		HAL_I2C_Mem_Read(&hi2c1, (device_rd_address << 1), (start_page - 1) * 16, I2C_MEMADD_SIZE_8BIT, EEPROM_rx_buf, Size, 10);// 7bit ������ ������ 1 ~ 128 7bit�� 8��Ʈ�� ������༭ ����ؾ���
		Mute_state = EEPROM_rx_buf[2];//���Ұ� ���� 0 : ���Ұ� ����, 1 : ��� ����
	}
	else//������ ��������
		HAL_I2C_Mem_Read(&hi2c1, (device_rd_address << 1), (start_page - 1) * 16, I2C_MEMADD_SIZE_8BIT, EEPROM_rx_buf, Size, 10);// 7bit ������ ������ 1 ~ 128 7bit�� 8��Ʈ�� ������༭ ����ؾ���
	EEPROM_crc_rx_data = EEPROM_rx_buf[14];	    //CRC's Low byte
	EEPROM_crc_rx_data = EEPROM_crc_rx_data | (EEPROM_rx_buf[15] << 8);//CRC's High byte
	compare_tx_rx = CRC16(EEPROM_rx_buf, 14);	//���� 14�� �������� CRC���
	if(EEPROM_crc_rx_data == compare_tx_rx) //���� crc�� ��
	{
		eeprom_crc_not_ok = 0;//crc�� ������ OK
		for (int i = 0; i < 16; i++) 
		{
			bdata = (byte *)MD_EEPROM_ADDRESS;
		    bdata[i] = EEPROM_rx_buf[i];
   		}		
	}
	else
	{
		eeprom_crc_not_ok = 1;
	}
	save_angle_flag = 0;
	save_ct_flag = 0;
	return eeprom_crc_not_ok;
}	
byte crc_write = 0;
void crc_err_check(byte start_page)
{
	if(start_page ==  2)//����
	{//�������� �̻��� ����� default�� �ִ� �κ�
		int_to_bytes(word_to_uint32(3891), bytes_data);//angle_point1 ad
		for(byte an = 0; an <= 1; an++)//2byte
			Arr_eeprom2[an] = bytes_data[an - 0];			
		int_to_bytes(word_to_uint32(3245), bytes_data);//angle_point2 ad
		for(byte an = 2; an <= 3; an++)//2byte
			Arr_eeprom2[an] = bytes_data[an - 2];	
		int_to_bytes(word_to_uint32(947), bytes_data);//angle_point3 ad
		for(byte an = 4; an <= 5; an++)//2byte
			Arr_eeprom2[an] = bytes_data[an - 4];
		int_to_bytes(word_to_uint32(1349), bytes_data);//angle_point4 ad
		for(byte an = 6; an <= 7; an++)//2byte
			Arr_eeprom2[an] = bytes_data[an - 6];
		/*
		int_to_bytes(word_to_uint32(3940), bytes_data);//angle_point5 ad
		for(byte an = 8; an <= 9; an++)//2byte
			Arr_eeprom2[an] = bytes_data[an - 8];	
		*/	
		int_to_bytes(float_to_uint32(210.00), bytes_data);//angle_point1 real
		for(byte an = 0; an <= 3; an++)//4byte
			Arr_eeprom3[an] = bytes_data[an - 0];			
		int_to_bytes(float_to_uint32(170.00), bytes_data);//angle_point2 real
		for(byte an = 4; an <= 7; an++)//4byte
			Arr_eeprom3[an] = bytes_data[an - 4];	
		int_to_bytes(float_to_uint32(-30), bytes_data);//angle_point3 real
		for(byte an = 8; an <= 11; an++)//4byte
			Arr_eeprom3[an] = bytes_data[an - 8];		
		int_to_bytes(float_to_uint32(10), bytes_data);//angle_point4 real
		for(byte an = 0; an <= 3; an++)//4byte
			Arr_eeprom4[an] = bytes_data[an - 0];	
		/*
		int_to_bytes(float_to_uint32(211.2), bytes_data);//angle_point5 real
		for(byte an = 4; an <= 7; an++)//4byte
			Arr_eeprom4[an] = bytes_data[an - 4];
		*/	
		crc_write = 1;// ���⼭ write
	}
	else if(start_page == 5)//����
	{//�������� �̻��� ����� default�� �ִ� �κ�
		int_to_bytes(word_to_uint32(434), bytes_data);//500ma ad
		for(byte an = 0; an <= 1; an++)//2byte
			Arr_eeprom5[an] = bytes_data[an];			
		int_to_bytes(word_to_uint32(2581), bytes_data);//2500ma ad
		for(byte an = 2; an <= 3; an++)//2byte
			Arr_eeprom5[an] = bytes_data[an - 2];				
		int_to_bytes(float_to_uint32(500), bytes_data);//500ma real
		for(byte an = 4; an <= 7; an++)//2byte
			Arr_eeprom5[an] = bytes_data[an - 4];			
		int_to_bytes(float_to_uint32(2500), bytes_data);//2500ma real	
		for(byte an = 8; an <= 11; an++)//2byte
			Arr_eeprom5[an] = bytes_data[an - 8];	
		EEPROM_Write(DEVICE_ADDRESS, start_page, Arr_eeprom5);	//(I2C�ּ�, EEPROM������, Tx�� �迭)	
	}
}
byte Eeprom_ok = 0;
word EEPROM_crc_tx_data = 0;
void EEPROM_Write(byte device_wr_address, byte start_page, byte *Wr_Data)
{//EEPROM �����û
	for(byte i = 0; i < 16; i++) //�� �������� 16Byte
	{
		EEPROM_tx_buf[i] = Wr_Data[i]; 		
	}
	EEPROM_crc_tx_data = CRC16(EEPROM_tx_buf, 14);	//14�� �������� CRC��� 
	EEPROM_tx_buf[14] = EEPROM_crc_tx_data & 0xFF;	//CRC's Low byte	
	EEPROM_tx_buf[15] = (EEPROM_crc_tx_data >> 8);  //CRC's High byte	
	HAL_I2C_Mem_Write(&hi2c1, (device_wr_address << 1) + 1, (start_page - 1) * 16, I2C_MEMADD_SIZE_8BIT, EEPROM_tx_buf, Byte_num_16, 10);//I2C ����� ����Ͽ� EEPROM�� �����͸� ����.
}
//word��ȯ
typedef union 
{
    word word; // 32��Ʈ ������ �����ϴ� ���
    word i;    // 32��Ʈ ������ �����ϴ� �� �ٸ� ���
} WordToUint32Converter;
word word_to_uint32(word value) 
{
    WordToUint32Converter converter; // ��ȯ�� ���� ���Ͽ� ���� ����
    converter.word = value;          // ���Ͽ��� word ����� �Է� ���� ����
    return converter.i;              // ���Ͽ��� i ����� ��ȯ
}
word bytes_to_word(byte *bytes_data) 
{
    union 
    {
        word w;        // word Ÿ���� ���
        uint32_t i;    // 32��Ʈ ���� ���
    } converter;
    // �� ����Ʈ�� 32��Ʈ ������ ��ġ�� �۾�
    converter.i = ((uint32_t)bytes_data[1] << 8) | (uint32_t)bytes_data[0];
    return converter.w; // word Ÿ���� ����� ��ȯ
}
///word��ȯ
///float��ȯ
union 
{
	float f;
	uint32_t i;
} converter;
// �ε� �Ҽ��� ���� 4����Ʈ ������ ��ȯ�ϴ� �Լ�
uint32_t float_to_uint32(float value) 
{
    converter.f = value;
    return converter.i;
}
///float��ȯ
void int_to_bytes(uint32_t value, byte *bytes_data) 
{
    if (value <= 0xFFFF) 
	{ // value�� 16��Ʈ ���� ���� ���
        bytes_data[1] = (value >> 8) & 0xFF;
        bytes_data[0] = value & 0xFF;
    } 
	else 
	{ // value�� 32��Ʈ�� ���
        bytes_data[3] = (value >> 24) & 0xFF;
        bytes_data[2] = (value >> 16) & 0xFF;
        bytes_data[1] = (value >> 8) & 0xFF;
        bytes_data[0] = value & 0xFF;
    }
}
/*
void int_to_bytes(uint32_t value, byte *bytes_data) 
{
    bytes_data[0] = (value >> 24) & 0xFF;
    bytes_data[1] = (value >> 16) & 0xFF;
    bytes_data[2] = (value >> 8) & 0xFF;
    bytes_data[3] = value & 0xFF;
}*/
float bytes_to_float(byte *bytes_data)
{
    union {
        float f;
        uint32_t i;
    } converter;
    converter.i = ((uint32_t)bytes_data[3] << 24) | ((uint32_t)bytes_data[2] << 16) | ((uint32_t)bytes_data[1] << 8) | (uint32_t)bytes_data[0];
    return converter.f;
}
//�������
byte  SPI_tx_buf[10], SPI_rx_buf[10]; 
byte Voice_error = 0;
byte Volume_step = 0; // ���� ����� �ʱ� ���� 3�ܰ�
byte Settings=0,Settings_cancel=0,Settings_save=0 ,Voice_data,Setting_voice=0;//Setting_voice = ���� 1�� ��¿뵵
byte  Voice_working=0, bf_Setting_voice=0, no_repeat = 0;  	
//�������
byte voice_cancel=0;
byte volum_count = 0;
byte voice_error=0, Volume_save=0;
byte volume_state = 0;
void voice_status(void)
{
	Mute_state = EEPROM_rx_buf[2];
	volume_state = EEPROM_rx_buf[1];
	if(voice_error == 0)
	{
		SPI_tx_buf[0] = 0xB0;
		SPI_tx_buf[1] = 0x00;
		if(Volume_step == 0 || voice_cancel == 5 || Volume_save == 0)//���� ����Ǿ��ִ� ���������� ���� ����.
			SPI_tx_buf[2] = volume_state + 0x05;
		if(Volume_step)
		{
			SPI_tx_buf[2] = Volume_step + 0x05;//���� �� �����Ͽ��� ��� ����
		}
		if(Settings_save == 4 && Volume_save)
			SPI_tx_buf[2] = Volume_save + 0x05;
		
		if(volume_state < 0x1C && volume_state > 0x20)//���� �ܰ� ���� ������ ���� ��� 5�ܰ�� �����Ǿ� ���
			volume_state = 0x20;
		if((volume_state >= 0x1C && volume_state <= 0x20) || volume_state == 0x26)
		{				
			VOICE_CS_LOW;// ������� CS LOW
			HAL_SPI_Transmit(&hspi1, SPI_tx_buf, 3, 10);//3byte ���
			VOICE_CS_HIGH;// ������� CS HIGH	
		}
	}
}
byte bf_voice_back = 0 , Setting_in = 0, Load_sensitivity_stage = 0;
void SPI_voice_set(void)
{
	EEPOM_Read(DEVICE_ADDRESS, Byte_num_16, 1);//(I2C�� �ּ�, �� �������� 16bytye, �б� �������ּ�)// ������� ���� ��ȭ��, voice_step, ���Ұ� ���� ������ ����Ǿ�����.
	voice_status(); //���� �ܰ� �ʱ⼳���ϴ� ��	
	EEPOM_Read(DEVICE_ADDRESS, Byte_num_16, 1);//(I2C�� �ּ�, �� �������� 16bytye, �б� �������ּ�)// ������� ���� ��ȭ��, voice_step, ���Ұ� ���� ������ ����Ǿ�����.
	if(Settings == 0x03)
	{			
		Arr_eeprom1[0] = Load_sensitivity_stage;//���� �ܰ�
		Mute_state = _VOICE_DATA.Mute_Flag;//���Ұ� ��������		
		if((_VOICE_DATA.x06_1B >= 0x1C) && (_VOICE_DATA.x06_1B <= 0x20)) 
		   Volume_step = _VOICE_DATA.x06_1B;
		Setting_voice = 1;
		Voice_data = Settings;
		if(Volume_step)//���� ����
		{//1�ܰ� : 0x1C, 2�ܰ� : 0x1D, 3�ܰ� : 0x1E, 4�ܰ� : 0x1F, 5�ܰ� : 0x20
			bf_Setting_voice = 0;
			Voice_data = Volume_step;
			Volume_save = Volume_step;
			Arr_eeprom1[1] = Volume_step;//eeprom 0�� �ε��� ���������� �� �ٲ���
			voice_status();//���� ���� ���⼭ ������ �ܰ踦 �������ش�.
		}
		Setting_in = 1;
		if(Settings_save == 0x04)//������ �����մϴ�.
		{
			if(Arr_eeprom1[1] != 0 || (_VOICE_DATA.Mute_Flag != EEPROM_rx_buf[2]))//���� �� �������� �ʰ� �ٷ� ���� ��ư�� ������ ��� EERPOM�� 00�� Wirte�ϴ� �۾��� ����
			{
				Arr_eeprom1[2] = _VOICE_DATA.Mute_Flag;
				if(Arr_eeprom1[1] <= 0x20 && Arr_eeprom1[1] >= 0x1C)
					EEPROM_Write(DEVICE_ADDRESS, 1, Arr_eeprom1);//(I2C�ּ�, EEPROM������, Tx�� �迭)
			}
			Voice_data = Settings_save;//0x04�־���	
			voice_status();//���� ����
			Mute_state = _VOICE_DATA.Mute_Flag;
			Volume_save = 0;
			Settings_save = 0;
			_VOICE_DATA.x06_1B = 0;
			_USER_RUN_SETTING.x06_1B = 0;
			Settings = 0;	
			bf_Setting_voice = 0;	
		}			
		if(Settings_cancel == 0x05)//������ ����մϴ�.
		{//���� ��ҿ� ���� ������ ���� ���·� �ǵ����� �۾��� �����ؾ��� EEPRM���� ��.	
			Voice_data = Settings_cancel;//0x05�־���	
			_USER_RUN_SETTING.x06_1B = 0;
			_VOICE_DATA.x06_1B = 0;
			bf_Setting_voice = 0;
			voice_cancel = 1;//���� ����� ���̽� �ܰ�� ����
			Settings_cancel = 0;
			_VOICE_DATA.Mute_Flag = 0;
			Settings = 0;	
		}
	}				
	switch(_VOICE_DATA.x06_1B)//0x06(����) ~ 0x1B(�����Ϸ� ���� �����Ǿ����ϴ�.)voice_en
	{
		case 0x06://� ���
			Voice_data = _VOICE_DATA.x06_1B;
			break;
		case 0x07://��� �����մϴ�.
			Voice_data = _VOICE_DATA.x06_1B;
			break;
		case 0x08://�Ϲ� �
			Voice_data = _VOICE_DATA.x06_1B;
			break;
		case 0x09://���� �
			Voice_data = _VOICE_DATA.x06_1B;
			break;
		case 0x0A://���� �
			Voice_data = _VOICE_DATA.x06_1B;
			break;
		case 0x0B://������ġ ���Ѱ�
			Voice_data = _VOICE_DATA.x06_1B;
			break;
		case 0x0C://������ġ ���Ѱ�
			Voice_data = _VOICE_DATA.x06_1B;			
			break;
		case 0x0D://������ġ �����Ѱ�
			Voice_data = _VOICE_DATA.x06_1B;
			break;
		case 0x0E://��� ���
			Voice_data = _VOICE_DATA.x06_1B;			
			break;
		case 0x0F://���� ���
			Voice_data = _VOICE_DATA.x06_1B;
			break;
		case 0x10://� �ð�
			Voice_data = _VOICE_DATA.x06_1B;
			break;
		case 0x11://� Ƚ��
			Voice_data = _VOICE_DATA.x06_1B;	
			break;
		case 0x12://�������
			Voice_data = _VOICE_DATA.x06_1B;			
			break;
		case 0x13://������ �����մϴ�
			Voice_data = _VOICE_DATA.x06_1B;	
			break;
		case 0x14://������ �����մϴ�
			Voice_data = _VOICE_DATA.x06_1B;
			break;
		case 0x15://������ �Ϸ�Ǿ����ϴ�
			Voice_data = _VOICE_DATA.x06_1B;	
			break;
		case 0x16://���� ���
			Voice_data = _VOICE_DATA.x06_1B;	
			break;
		case 0x17://������� �Ǿ����ϴ�
			Voice_data = _VOICE_DATA.x06_1B;	
			break;
		case 0x18://���簢���� ���������� ������ϴ�
			Voice_data = _VOICE_DATA.x06_1B;
			break;
		case 0x19://���Ͱ� �Ͻ����� �Ǿ����ϴ�.
			Voice_data = _VOICE_DATA.x06_1B;
			break;
		case 0x1A://���ڴ� ������ Ȯ�����ּ���
			Voice_data = _VOICE_DATA.x06_1B;
			break;
		case 0x1B://����ȭ�� ���� �����Ǿ����ϴ�.
			Voice_data = _VOICE_DATA.x06_1B;	
			break;	
		case 0x27://������� �����մϴ�.
			Voice_data = _VOICE_DATA.x06_1B;	
			break;	
		case 0x28://��� �����մϴ�.
			Voice_data = _VOICE_DATA.x06_1B;	
			break;
		case 0x29://������� ����մϴ�.
			Voice_data = _VOICE_DATA.x06_1B;	
			break;			
		case 0x2A://���ϰ� �����Ǿ����ϴ�.
			Voice_data = _VOICE_DATA.x06_1B;	
			break;	
		case 0x2B://������� ��ư�� �����ϼ���.
			Voice_data = _VOICE_DATA.x06_1B;	
			break;	
		case 0x2C://������� ��ư�� �����Ǿ����ϴ�.
			Voice_data = _VOICE_DATA.x06_1B;	
			break;					
	}
	if((Setting_voice == 1 && bf_Setting_voice == 0 && Voice_working == 0) || _USER_RUN_SETTING.x06_1B || Volume_step ||  _VOICE_DATA.x06_1B)
	{//��º�
		voice_out = 1;
	}
}
byte voice_out = 0;
void voice_output(void)
{//������� ��º�
	SPI_tx_buf[0] = 0xB0;
	SPI_tx_buf[1] = 0x00;
	SPI_tx_buf[2] = Voice_data;	
	VOICE_CS_LOW;// ������� CS LOW
	if(Mute_state == 1 || _VOICE_DATA.Mute_Flag == 1)	//���Ұ� ���°� �ƴϸ� ���			
		HAL_SPI_Transmit(&hspi1, SPI_tx_buf, 3, 10);//3byte ���
	VOICE_CS_HIGH;// ������� CS HIGH
	if(Settings == 0x03)
		bf_Setting_voice = Setting_voice;
	Setting_voice = 0;
	Volume_step = 0;
	Voice_data = 0;
	no_repeat = 0;
}
		
uint32_t integer_angle_Value_1 = 0, integer_angle_Value_2 = 0, integer_angle_Value_3 = 0, integer_angle_Value_4 = 0, integer_angle_Value_5 = 0, integer_ct_Value_1 = 0, integer_ct_Value_2 = 0;
void angle_1(void)
{
	integer_angle_Value_1 = word_to_uint32(_CALIBRATION_SETTING.angle_ad_point_1);
	real_angle_point1 = _USER_RUN_SETTING.real_value;//210��		
	integer_angle_Value_2 = word_to_uint32(_CALIBRATION_SETTING.angle_ad_point_2);
	real_angle_point2 = _USER_RUN_SETTING.real_value;//170��		
	integer_angle_Value_3 = word_to_uint32(_CALIBRATION_SETTING.angle_ad_point_3);
	real_angle_point3 = _USER_RUN_SETTING.real_value;//-30��			
	integer_angle_Value_4 = word_to_uint32(_CALIBRATION_SETTING.angle_ad_point_4);
	real_angle_point4 = _USER_RUN_SETTING.real_value;//10��
}
void ct_1(void)
{
	integer_ct_Value_1 = word_to_uint32((word)_CALIBRATION_SETTING.current_500mA_AD);	
	real_current_500mA = _USER_RUN_SETTING.real_value;
	integer_ct_Value_2 = word_to_uint32((word)_CALIBRATION_SETTING.current_2500mA_AD);
	real_current_2500mA = _USER_RUN_SETTING.real_value;						
}
void copyLoRaDataToSensorData()
{
 	_SENSOR_DATA.lora_wr_fg = 1; // LCD�� PC������(�Ʒ� LoRa ����ü ������) ����, _SENSOR_DATA.lora_wr_fg�� 0�̸� LCD�� �����͸� ����X
	
    _SENSOR_DATA.user_name_1 = (_LoRa_Data.user_name_1<<8) | (_LoRa_Data.user_name_1>>8);
    _SENSOR_DATA.user_name_2 = (_LoRa_Data.user_name_2<<8) | (_LoRa_Data.user_name_2>>8);
	
    _SENSOR_DATA.user_name_3 = (_LoRa_Data.user_name_3<<8) | (_LoRa_Data.user_name_3>>8);
    _SENSOR_DATA.user_name_4 = (_LoRa_Data.user_name_4<<8) | (_LoRa_Data.user_name_4>>8);
	
    _SENSOR_DATA.user_name_5 = (_LoRa_Data.user_name_5<<8) | (_LoRa_Data.user_name_5>>8);
    _SENSOR_DATA.user_name_6 = (_LoRa_Data.user_name_6<<8) | (_LoRa_Data.user_name_6>>8);
	
	_SENSOR_DATA.patient_sex_age = (_LoRa_Data.user_sex << 7) | (_LoRa_Data.user_age & 0x7F); //MSB�� ���� �� : 0, �� : 1 Ex) ���� 65�� -> 1100 0001 / ���� 65�� -> 0100 0001
	_SENSOR_DATA.upper_angle = _LoRa_Data.upper_angle;
	_SENSOR_DATA.lower_angle = _LoRa_Data.lower_angle;
	_SENSOR_DATA.upper_stop_time = (_LoRa_Data.stop_time >> 4) & 0x0F;  //   ���������ð� : ���� 4bit, ���������ð� : ���� 4bit
	
	_SENSOR_DATA.lower_stop_time = _LoRa_Data.stop_time & 0x0F;	
	_SENSOR_DATA.mode = _LoRa_Data.mode;//� ���(0:�Ϲݵ�� , 1:����, 2:���� 3: �Ϲݰ���)
	_SENSOR_DATA.speed = _LoRa_Data.velocity_mode;//( � �ӵ� : 1 ~ 9�ܰ� )	
	if((_LoRa_Data.exerc_time_and_exercnum & 0x80) != 0x80) // ��ð�
	{
	    _SENSOR_DATA.exerc_num = 0;
		_SENSOR_DATA.exerc_time = _LoRa_Data.exerc_time_and_exercnum & 0x7F;//(LSB : 1 ~ 99 �����ð� �Ǵ� �Ƚ�� �����̸�,  MSB�� 0�̸� ��ð�, 1�̸� �Ƚ�� )
	}
	else if((_LoRa_Data.exerc_time_and_exercnum & 0x80) == 0x80) // �Ƚ��
	{
	  	_SENSOR_DATA.exerc_time = 0;
		_SENSOR_DATA.exerc_num = _LoRa_Data.exerc_time_and_exercnum & 0x7F;//(LSB : 1 ~ 99 �����ð� �Ǵ� �Ƚ�� �����̸�,  MSB�� 0�̸� ��ð�, 1�̸� �Ƚ�� )
	}
	_SENSOR_DATA.special_angle = _LoRa_Data.special_angle;
	_SENSOR_DATA.repeat_num = _LoRa_Data.repeat_num;
	_SENSOR_DATA.special_location = _LoRa_Data.special_location;
}
//EEPROM
byte wait_1400m_flag = 0, wait_1400m_start = 0;
byte page1_1400msec_wait = 0, page2_1400msec_wait = 0, page1_end_flag = 0,page2_end_flag = 0, ct_cal_ad_1 = 0, ct_cal_ad_2 = 0, angle_cal_ad_1 = 0, angle_cal_ad_2 = 0, angle_cal_ad_3 = 0, angle_cal_ad_4 = 0, angle_cal_ad_5 = 0;
byte save_angle_flag = 0, save_ct_flag = 0, real_angle_value_page2_end = 0, Last_page_end = 0, save_angle_end_flag = 0, decide_angle_flag = 0, Last_page = 0;
//BlueTooth

byte START_STATE = 0, STOP_STATE = 0; 
dword TIM4_CCR1_buffer = 0,  TIM4_CCR2_buffer = 0; 

// ���� ���� ����
volatile uint32_t start_time = 0;
volatile uint32_t end_time = 0;
volatile uint32_t elapsed_time = 0;

void start_timer() 
{ // Ÿ�̸� ī���� �ʱ�ȭ
  TIM5->CNT = 0;
  // TIM5 ���ͷ�Ʈ Ȱ��ȭ
//  NVIC_EnableIRQ(TIM5_IRQn);
//  TIM5->DIER |= TIM_IT_UPDATE;
}

float stop_timer() 
{// TIM5 ���ͷ�Ʈ ��Ȱ��ȭ
//	NVIC_DisableIRQ(TIM5_IRQn);
//	TIM5->DIER &= ~TIM_IT_UPDATE;
	// Ÿ�̸� ī���� �� �б�
	end_time = TIM5->CNT;
	return end_time;
}

// ���� ������ �����ϴ� �Լ�
float interpolate(uint16_t* angle_arr, int size, float P_M_avr, float real_angle_point4, float real_angle_point2, uint16_t angle_ad_point4, uint16_t angle_ad_point2, float* bf_pm_actual) 
{
    int lower_index = 0;
    int upper_index = size - 1;
    // ���� Ž���� ���� ���Ѱ� ���� �ε����� ã��
    for (int i = 1; i < size; ++i) 
	{
        if (angle_arr[i] > P_M_avr) 
		{
            upper_index = i;
            lower_index = i - 1;
            break;
        }
    }
    lower_ad = angle_arr[lower_index];
    upper_ad = angle_arr[upper_index];
    lower_angle = real_angle_point4 + lower_index * 5;
    upper_angle = real_angle_point4 + upper_index * 5;
    P_M_actual = (upper_ad != lower_ad) 
        ? lower_angle + (P_M_avr - lower_ad) * (upper_angle - lower_angle) / (upper_ad - lower_ad)
        : ((P_M_avr > (angle_ad_point2 + angle_ad_point4) / 2) ? real_angle_point2 : real_angle_point4);
    *bf_pm_actual = P_M_actual;
    return (P_M_actual + *bf_pm_actual) / 2;
}


#define NUM_SAMPLES 5 // ������ ���� �迭 ũ��
float current_history[NUM_SAMPLES] = {0}; // ������ ���� �迭
byte current_index = 0; // ���� ������ �ε���
float baseline_current = 0; // ������ �� ��� ���� ��
float Load_sensitivity = 0, relative_change = 0; // ���� ���� �ΰ��� (���ذ� ��� 40% ���)

float calculate_slope(float values[], byte size)
{
    // ������ 1�� ȸ�� ���� ���
    float sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0;
    for (int i = 0; i < size; i++) 
	{
        sum_x += i;
        sum_y += values[i];
        sum_xy += i * values[i];
        sum_x2 += i * i;
    }
    float slope = (size * sum_xy - sum_x * sum_y) / (size * sum_x2 - sum_x * sum_x);
    return slope;
}

float calculate_average(float values[], byte size) 
{
    float sum = 0;
    for(byte i = 0; i < size; i++) 
	{
        sum += values[i];
    }
    return sum / size;
}

float purse1_down_1angle = 0.0;//��Ȯ���� ���� float���� �ؾ���
//Ķ���극�̼� ����Ʈ +10��, 170�� ����
word ad_count_down = 100, remain_ad = 0;
byte low_to_high = 0, anlge_cal_flag = 0, angle_record_flag = 0;
word low_to_high_pwm = 1800, high_to_low_pwm = 1800;//240�� ������ �̵�
dword purse1_up_1angle = 0;//��Ȯ���� ���� float���� �ؾ���
byte angle_recording_end = 0, calibration_state = 0, enable_low_to_high = 0;
float angle_real = 0;
byte high_to_low = 0, Save_final_ad = 0, eeprom_page_state = 0, read_ad_angle_flag = 0,Read_ad_angle_state = 0;
byte wait_high_to_low = 0, start_high_to_low = 0;
byte SYSTEM_INITIALIZATION_VALUE_ANGLE = 0;//�ý��� ó�� �����ϸ� 1 �� �������� ����� �� ����. 0���� ����.
byte SYSTEM_INITIALIZATION_VALUE_CT = 0;
//������� ��������.
byte measure_restart = 0, restart_f = 0;
byte Benchmark = 0, Detect_load = 0;
byte Bf_Dir_of_rotation = 0, Change_dir = 0;
word EMS_start_cnt = 0;
byte read_sensitivity = 0,start_read_sense = 0;
//������� ��������.
byte pause_state = 0;
byte start_sensing = 0;
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
	MX_TIM1_Init();         //encoder
	MX_TIM2_Init();			//0.1ms Timer
	MX_TIM3_Init();			//1ms Timer
	MX_TIM4_Init();			//Not used
	MX_TIM5_Init();
	MX_TIM8_Init();			//Motor PWM(50us=20KHz)
	MX_UART4_Init();		//For Display Board Communication
	MX_USART2_UART_Init();	//For LoRa Communication
	MX_USART3_UART_Init();	//For Bluetooth Communication
	MX_ADC1_Init();
	MX_I2C1_Init();
	MX_SPI1_Init();
	MX_USB_PCD_Init();
	/* USER CODE BEGIN 2 */
	__HAL_AFIO_REMAP_SWJ_NOJTAG();//JTAG�� ��Ȱ��ȭ �ؾ� clk�� Ȱ��ȭ ��	
	//delayMicroseconds(10); // 100 ����ũ������ ����		
	MOTOR_ROTATE; 
	HAL_TIM_Base_Start_IT(&htim1);		//TIM1 interrupt start	
	HAL_TIM_Base_Start_IT(&htim2);		//TIM2 interrupt start		0.1m
	HAL_TIM_Base_Start_IT(&htim3);		//TIM3 interrupt start		1.0m
	HAL_TIM_Base_Start_IT(&htim5);		//TIM5 interrupt start		0.01m
	USART2->CR1 |= USART_CR1_RXNEIE;	//USART2 �������ͷ�Ʈ ���
	UART4->CR1 |= USART_CR1_RXNEIE;		//UART4 �������ͷ�Ʈ ���
	USART3->CR1 |= USART_CR1_RXNEIE;	//UART3 �������ͷ�Ʈ ���
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1);// ���� ä�� �ѱ�		
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_1 | TIM_CHANNEL_2);//���ڴ� Ÿ�̸� start 
	//���� üũ	
	//ADC1�� AD��ȯ ����.
	HAL_Delay(1400);
	Read_ad_angle_state = 6;//���� ad�����͸� eeprom���κ��� �б� ���� �ڵ� 6������ ���� 10������ ���� ����	
	ADC1_index = 0;
	HAL_ADC_Start(&hadc1);
	//���� ���� output, PWM�� 0���� �ʱ�ȭ
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);//GPIO �⺻�� ����		   
	htim4.Instance->CCR1 = 0;  // duty ����       
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);//GPIO �⺻�� ����		
	htim4.Instance->CCR2 = 0;  // duty����  
	_LoRa_Data.state = 1;//���� ����(�������)
	_SENSOR_DATA.state = 1;//���� ����(�������)
	Dir_of_rotation = STOP;		//����� �ƴҶ� �ʱⰪ=>Stop
	GPIOC -> ODR |= 0x80;	// BUZZER_ON
	HAL_Delay(50);
	GPIOC -> ODR &= ~0x80;	// BUZZER_OFF		
	HAL_Delay(1000);		//ó���� ����üũ�ϱ����� ����,���� AD���� �޾ƿ��� ���� 1�� ��ٸ�.
	bluetooth_AT = 1;
	SPI_tx_buf[0] = 0xB0;
	SPI_tx_buf[1] = 0x00;
	SPI_tx_buf[2] = 0x01;
	VOICE_CS_LOW;// ������� CS LOW
	HAL_SPI_Transmit(&hspi1, SPI_tx_buf, 3, 10);//3byte ���
	VOICE_CS_HIGH;// ������� CS HIGH	
	VOICE_RESET_HIGH;
	HAL_Delay(5);
	VOICE_RESET_LOW;
	//EEPOM_Read(DEVICE_ADDRESS, Byte_num_16, 1);//(I2C�� �ּ�, �� �������� 16bytye, �б� �������ּ�)// ������� ���� ��ȭ��, voice_step, ���Ұ� ���� ������ ����Ǿ�����.
	//Load_sensitivity = EEPROM_rx_buf[0];		
	PWM_NOT_SLEEP;//ON
	save_ct_end = 1;//�������� ���� eeprom���� �ҷ�����
	HAL_Delay(1400);
	save_angle_end = 1; //�������� ���� eeprom���� �ҷ�����
	HAL_Delay(1400);
	TIM5->DIER &= ~TIM_IT_UPDATE;//���α׷� �ҿ�ð��� �����ϱ� ���� ��Ȱ��ȭ �ڵ�
	FA_RESET_L; // ������� ��� FA_RESET LOW
	HAL_Delay(100);
	FA_RESET_H; // ������� ��� FA_RESET HIGH
	LoRa_Equipment_Num = 1; // ���͵���̹� ���� ���� On�� LoRa_Equipment_Num�� 1 �� �ѹ��� �ʱ� ���� ���� ���͵���̹����� Slave_LoRa�� ������.
	while (1)
	{
		/*
		if(SYSTEM_INITIALIZATION_VALUE_CT)//EEPOM�� �����Ͱ� ���� ��쿡 ����
		{
			SYSTEM_INITIALIZATION_VALUE_CT = 0;
			real_current_500mA = 500, real_current_2500mA = 2500, cal_500mA_val = 434, cal_2500mA_val = 2580;
		}
		*/
		//EEPOM_Read(DEVICE_ADDRESS, Byte_num_16, 1);//(I2C�� �ּ�, �� �������� 16bytye, �б� �������ּ�)// ��ġ��ȣ, voice_step, ���Ұ� ���� ������ ����Ǿ�����.
		
		_SENSOR_DATA.load_sensitivity = Load_sensitivity_stage;
		
		if(EMS_SW)
		{			
			PWM_SLEEP;	
			if((exercise_mode >= 5) && (exercise_mode <=8))//�������
				_USER_RUN_SETTING.exerc_start = 0x03;//�����
			//09.20 ���찡 ��
			TX_EMS_CNT = 50; // EMS_STATE = 1 ���¸� 500ms ���� ����
			if((_SENSOR_DATA.state == 0) && ((exercise_mode == 1) || (exercise_mode == 2) || (exercise_mode == 3)))//�Ϲ� ���� ���߿�� ��쿡 ����� ���¿��� ������� ������ ����
			    pause_state = 2;	//�Ͻ�����	
			if(_SENSOR_DATA.state == 2)//���Ѱ� �̵����� ���
				_SENSOR_DATA.state = 0x01; //����� ����
			Motor_BUSY = 0;
			if(Measurement_mode == 1)
				Detect_load = 1;//������� �� ���Ƿ� ���ϰ����ؼ�	
			EMS_STATE = 1; // EMS ����ġ�� ������ ���� ����
			if(_SENSOR_DATA.state == 2)//���Ѱ����� �̵����� ��쿡 ��������� ������ ����� ����� �ӽ÷� ������. 
				_USER_RUN_SETTING.exerc_start = 0x03;
		}
		if(EMS_STATE)
			_LoRa_Data.state = 0x02;//����
		else if(_SENSOR_DATA.state == 1 && _LoRa_Data.measure_check != 1)
			_LoRa_Data.state = 1;	
		if(Load_sensitivity == 0 && angle_recording_end)
			read_sensitivity = 1;
		if(start_read_sense)
		{
			start_read_sense = 0;
			EEPOM_Read(DEVICE_ADDRESS, Byte_num_16, 1);//(I2C�� �ּ�, �� �������� 16bytye, �б� �������ּ�)// ������� ���� ��ȭ��, voice_step, ���Ұ� ���� ������ ����Ǿ�����.
			Load_sensitivity_stage = EEPROM_rx_buf[0];
			switch(Load_sensitivity_stage)
			{
				case 1:
					Load_sensitivity = 0.2;//20%
					break;
				case 2:
					Load_sensitivity = 0.4;//40%
					break;
				case 3:
					Load_sensitivity = 0.7;	//70%				
					break;
				case 4:
					Load_sensitivity = 1.0;//100%
					break;										
				case 5:
					Load_sensitivity = 1.3;//130%
					break;					
				case 6:
					Load_sensitivity = 1.6;//160%
					break;					
				case 7:
					Load_sensitivity = 1.9;//190%
					break;										
				case 8:
					Load_sensitivity = 2.2;//220%
					break;					
				case 9:
					Load_sensitivity = 2.5;//250%
					break;
				case 10:
					Load_sensitivity = 2.8;//280%
					break;		
			}
		}
		if(_USER_RUN_SETTING.calibration == 0x02)//�ΰ��� ���� Ķ���극�̼� ����� ��쿡�� ���� �ϵ��� ����
			start_sensing = 1;
		if(start_sensing && angle_recording_end)
		{
			start_sensing = 0;
			switch((byte)_USER_RUN_SETTING.real_value)
			{//������� ���� ���� �ΰ��� ����
				case 1:
					Load_sensitivity = 0.2;//20%
					break;
				case 2:
					Load_sensitivity = 0.4;//40%
					break;
				case 3:
					Load_sensitivity = 0.7;	//70%				
					break;
				case 4:
					Load_sensitivity = 1.0;//100%
					break;										
				case 5:
					Load_sensitivity = 1.3;//130%
					break;					
				case 6:
					Load_sensitivity = 1.6;//160%
					break;					
				case 7:
					Load_sensitivity = 1.9;//190%
					break;										
				case 8:
					Load_sensitivity = 2.2;//220%
					break;					
				case 9:
					Load_sensitivity = 2.5;//250%
					break;
				case 10:
					Load_sensitivity = 2.8;//280%
					break;					
			}
			Load_sensitivity_stage = (byte)_USER_RUN_SETTING.real_value;
			Arr_eeprom1[0] = Load_sensitivity_stage;//(byte)_USER_RUN_SETTING.real_value;
			Arr_eeprom1[1] = volume_state;
			Arr_eeprom1[2] = Mute_state;			
			EEPROM_Write(DEVICE_ADDRESS, 1, Arr_eeprom1);//(I2C�ּ�, EEPROM������, Tx�� �迭)
			//read_sensitivity = 1;
		}
		if(_USER_RUN_SETTING.exerc_start == 0x0A)//������� ȭ�� ���� ��쿡 0x0A ����
		{
			_USER_RUN_SETTING.exerc_start = 0x00;
			Measurement_mode = 0;
		}
		static byte minsooo = 0;
		if(minsooo)
		{
			PWM_NOT_SLEEP;
			TIM8 -> CCR1 = Motor_PWM_CCW;
		}
		static byte save_ad_arr[16] = {0}, arr_cnt = 0;// 1�������� 14Byte + crc(2Byte)������ ���� ����
		switch(eeprom_page_state)//write
		{
			case 6://6page
				wait_1400m_flag = 1;
				if(wait_1400m_start)
				{
					wait_1400m_start = 0;
					for(byte i = 0; i < (14/2); i++)//1Page = 14Byte = 7word
					{
						int_to_bytes(angle_arr[i + 0], bytes_data);
						for(byte an = 0; an <= 1; an++)//2byte
							save_ad_arr[i + an + arr_cnt] = bytes_data[an];
						arr_cnt++;
					}
					arr_cnt = 0;
					EEPROM_Write(DEVICE_ADDRESS, 6, save_ad_arr);	//(I2C�ּ�, EEPROM������, Tx�� �迭), real���� ��					
					eeprom_page_state = 7;
				}
				break;
			case 7://7page
				wait_1400m_flag = 1;
				if(wait_1400m_start)
				{
					wait_1400m_start = 0;
					for(byte i = 0; i < (14/2); i++)//1Page = 14Byte = 7word
					{
						int_to_bytes(angle_arr[i + 7], bytes_data);
						for(byte an = 0; an <= 1; an++)//2byte
							save_ad_arr[i + an + arr_cnt] = bytes_data[an];
						arr_cnt++;
					}
					arr_cnt = 0;
					EEPROM_Write(DEVICE_ADDRESS, 7, save_ad_arr);	//(I2C�ּ�, EEPROM������, Tx�� �迭), real���� ��						
					eeprom_page_state = 8;
				}
				break;	
			case 8://8page
				wait_1400m_flag = 1;			
				if(wait_1400m_start)
				{
					wait_1400m_start = 0;
					for(byte i = 0; i < (14/2); i++)//1Page = 14Byte = 7word
					{
						int_to_bytes(angle_arr[i + 14], bytes_data);
						for(byte an = 0; an <= 1; an++)//2byte
							save_ad_arr[i + an + arr_cnt] = bytes_data[an];
						arr_cnt++;
					}
					arr_cnt = 0;
					EEPROM_Write(DEVICE_ADDRESS, 8, save_ad_arr);	//(I2C�ּ�, EEPROM������, Tx�� �迭), real���� ��						
					eeprom_page_state = 9;
				}			
				break;		
			case 9://9page
				wait_1400m_flag = 1;			
				if(wait_1400m_start)
				{
					wait_1400m_start = 0;
					for(byte i = 0; i < (14/2); i++)//1Page = 14Byte = 7word
					{
						int_to_bytes(angle_arr[i + 21], bytes_data);
						for(byte an = 0; an <= 1; an++)//2byte
							save_ad_arr[i + an + arr_cnt] = bytes_data[an];
						arr_cnt++;
					}
					arr_cnt = 0;
					EEPROM_Write(DEVICE_ADDRESS, 9, save_ad_arr);	//(I2C�ּ�, EEPROM������, Tx�� �迭), real���� ��						
					eeprom_page_state = 10;
				}		
				break;		
			case 10://10page
				wait_1400m_flag = 1;			
				if(wait_1400m_start)
				{
					wait_1400m_start = 0;
					for(byte i = 0; i < (14/2); i++)//1Page = 14Byte = 7word
					{
						int_to_bytes(angle_arr[i + 28], bytes_data);
						for(byte an = 0; an <= 1; an++)//2byte
							save_ad_arr[i + an + arr_cnt] = bytes_data[an];
						arr_cnt++;
					}
					arr_cnt = 0;
					EEPROM_Write(DEVICE_ADDRESS, 10, save_ad_arr);	//(I2C�ּ�, EEPROM������, Tx�� �迭), real���� ��						
					eeprom_page_state = 0;//page state initialization
					Read_ad_angle_state = 6;//finish write -> read only to check ad, angle data
				}					
				break;					
		}
		static byte read_ad_err = 0;
		switch(Read_ad_angle_state)
		{
			case 6://6page	
				wait_1400m_flag = 1;
				if(wait_1400m_start)
				{	
					wait_1400m_start = 0;
					for(byte i = 0; i <= ANGLE_RANGE; i++) 
					{//erase arr data for checking read data
						angle_arr[i] = 0;
					}							
					read_ad_err = EEPOM_Read(DEVICE_ADDRESS, Byte_num_16, 6);//(I2C�� �ּ�, �� �������� 16bytye, �б� �������ּ�)

					for(int i = 0; i < 7; i++) 
						angle_arr[i + 0] = (word)(EEPROM_rx_buf[i * 2] | (EEPROM_rx_buf[i * 2 + 1] << 8));
					Read_ad_angle_state = 7;
					if(read_ad_err)
					{
						//SYSTEM_INITIALIZATION_VALUE_ANGLE = 1;//�б� ������ ����ٸ� ������ ���� ����
						Read_ad_angle_state = 6;//�ٽ�����
						read_ad_err = 0;
						//eeprom_page_state = 6;// 6���������� �ٽ� ��
						break;
					}					
				}
				break;
			case 7://7page	
				wait_1400m_flag = 1;
				if(wait_1400m_start)
				{	
					wait_1400m_start = 0;					
					read_ad_err = EEPOM_Read(DEVICE_ADDRESS, Byte_num_16, 7);//(I2C�� �ּ�, �� �������� 16bytye, �б� �������ּ�)
		
					for(int i = 0; i < 7; i++) 
						angle_arr[i + 7] = (word)(EEPROM_rx_buf[i * 2] | (EEPROM_rx_buf[i * 2 + 1] << 8));
					Read_ad_angle_state = 8;
					if(read_ad_err)
					{
						//SYSTEM_INITIALIZATION_VALUE_ANGLE = 1;//�б� ������ ����ٸ� ������ ���� ����
						Read_ad_angle_state = 6;//�ٽ�����
						read_ad_err = 0;
						break;
					}							
				}
				break;	
			case 8://8page	
				wait_1400m_flag = 1;
				if(wait_1400m_start)
				{	
					wait_1400m_start = 0;					
					read_ad_err = EEPOM_Read(DEVICE_ADDRESS, Byte_num_16, 8);//(I2C�� �ּ�, �� �������� 16bytye, �б� �������ּ�)					
					for(int i = 0; i < 7; i++) 
						angle_arr[i + 14] = (word)(EEPROM_rx_buf[i * 2] | (EEPROM_rx_buf[i * 2 + 1] << 8));
					Read_ad_angle_state = 9;
					if(read_ad_err)
					{
						//SYSTEM_INITIALIZATION_VALUE_ANGLE = 1;//�б� ������ ����ٸ� ������ ���� ����
						Read_ad_angle_state = 6;//�ٽ�����
						read_ad_err = 0;
						break;
					}						
				}
				break;
			case 9://9page	
				wait_1400m_flag = 1;
				if(wait_1400m_start)
				{	
					wait_1400m_start = 0;					
					read_ad_err = EEPOM_Read(DEVICE_ADDRESS, Byte_num_16, 9);//(I2C�� �ּ�, �� �������� 16bytye, �б� �������ּ�)
				
					for(int i = 0; i < 7; i++) 
						angle_arr[i + 21] = (word)(EEPROM_rx_buf[i * 2] | (EEPROM_rx_buf[i * 2 + 1] << 8));
					Read_ad_angle_state = 10;
					if(read_ad_err)
					{
						//SYSTEM_INITIALIZATION_VALUE_ANGLE = 1;//�б� ������ ����ٸ� ������ ���� ����
						Read_ad_angle_state = 6;//�ٽ�����
						read_ad_err = 0;
						break;
					}						
				}
				break;		
			case 10://10page	
				wait_1400m_flag = 1;
				if(wait_1400m_start)
				{	
					wait_1400m_start = 0;					
					read_ad_err = EEPOM_Read(DEVICE_ADDRESS, Byte_num_16, 10);//(I2C�� �ּ�, �� �������� 16bytye, �б� �������ּ�)
					
					for(int i = 0; i < 7; i++) 
						angle_arr[i + 28] = (word)(EEPROM_rx_buf[i * 2] | (EEPROM_rx_buf[i * 2 + 1] << 8));
					Read_ad_angle_state = 0;
					if(read_ad_err)
					{
						//SYSTEM_INITIALIZATION_VALUE_ANGLE = 1;//�б� ������ ����ٸ� ������ ���� ����
						Read_ad_angle_state = 6;//�ٽ�����
						read_ad_err = 0;
						break;
					}						
					angle_recording_end = 1;//���� ��� ����
				}
				break;						
		}
		static word main_routine = 0;
		if(_VOICE_DATA.x06_1B == 0x06)
			main_routine = 0;
		main_routine++;		
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
		if(angle_cal_ad_1)//Ķ��
		{//Ķ���극�̼Ǹ��ȭ�� ù ��° üũ 
			angle_recording_end = 0; 
			angle_1();
			angle_cal_ad_1 = 0;			
			int_to_bytes(integer_angle_Value_1, bytes_data);
			for(byte an = 0; an <= 1; an++)//2byte
				Arr_eeprom2[an] = bytes_data[an];
			integer_angle_Value_1 = float_to_uint32(real_angle_point1);
			int_to_bytes(integer_angle_Value_1, bytes_data);			
			for(byte an = 0; an <= 3; an++)//4byte
				Arr_eeprom3[an] = bytes_data[an];
		}	
		if(angle_cal_ad_2)
		{//Ķ���극�̼Ǹ��ȭ�� �� ��° üũ 
			angle_1();
			angle_cal_ad_2 = 0;			
			int_to_bytes(integer_angle_Value_2, bytes_data);
			for(byte an = 2; an <= 3; an++)//2byte
				Arr_eeprom2[an] = bytes_data[an - 2];
			integer_angle_Value_2 = float_to_uint32(real_angle_point2);
			int_to_bytes(integer_angle_Value_2, bytes_data);			
			for(byte an = 4; an <= 7; an++)//4byte
				Arr_eeprom3[an] = bytes_data[an - 4];
		}
		if(angle_cal_ad_3)
		{//Ķ���극�̼Ǹ��ȭ�� �� ��° üũ 
			angle_1();
			angle_cal_ad_3 = 0;			
			int_to_bytes(integer_angle_Value_3, bytes_data);
			for(byte an = 4; an <= 5; an++)//2byte
				Arr_eeprom2[an] = bytes_data[an - 4];
			integer_angle_Value_3 = float_to_uint32(real_angle_point3);
			int_to_bytes(integer_angle_Value_3, bytes_data);			
			for(byte an = 8; an <= 11; an++)//4byte
				Arr_eeprom3[an] = bytes_data[an - 8];
		}
		if(angle_cal_ad_4)
		{//Ķ���극�̼Ǹ��ȭ�� �� ��° üũ 
			angle_1();
			angle_cal_ad_4 = 0;			
			int_to_bytes(integer_angle_Value_4, bytes_data);
			for(byte an = 6; an <= 7; an++)//2byte
				Arr_eeprom2[an] = bytes_data[an - 6];
			integer_angle_Value_4 = float_to_uint32(real_angle_point4);
			int_to_bytes(integer_angle_Value_4, bytes_data);			
			for(byte an = 0; an <= 3; an++)//4byte
				Arr_eeprom4[an] = bytes_data[an];
			EEPROM_Write(DEVICE_ADDRESS, 2, Arr_eeprom2);	//(I2C�ּ�, EEPROM������, Tx�� �迭), ���� ad ��
			page1_end_flag = 1;//�� ��° ȭ����� ������ ������ �Ѵٸ� 1������ write �۾� �Ŀ� 1.4�ʵ��� ��ٸ��� 2������ write�ϱ� ���� �÷���			
		}				
		if(page1_1400msec_wait)
		{//3������ write
			page1_1400msec_wait = 0;
			EEPROM_Write(DEVICE_ADDRESS, 3, Arr_eeprom3);	//(I2C�ּ�, EEPROM������, Tx�� �迭), real���� ��		
			page2_end_flag = 1;
		}
		if(page2_1400msec_wait)
		{//4������ write
			page2_1400msec_wait = 0;
			Arr_eeprom4[11] = 0x77;//EEPROM�� �����͸� ���ٴ� �ǹ�
			EEPROM_Write(DEVICE_ADDRESS, 4, Arr_eeprom4);	//(I2C�ּ�, EEPROM������, Tx�� �迭), real���� ��
			save_angle_flag = 1;
			
		}	
		if(save_angle_end)//save_angle_flag ������ 1�� �ǰ� 1.4�� �ڿ� save_angle_end ������ 1
		{//Ķ���극�̼Ǹ��ȭ�� ������ üũ	
			save_angle_end = 0;
			anlge_ad_err_flag = EEPOM_Read(DEVICE_ADDRESS, Byte_num_16, 2);//(I2C�� �ּ�, �� �������� 16bytye, �б� �������ּ�)			
			angle_ad_point1 = 0, angle_ad_point2 = 0, angle_ad_point3 = 0, angle_ad_point4 = 0, angle_ad_point5 = 0;	
			angle_ad_point1 = bytes_to_word(EEPROM_rx_buf + 0); //1point AD��
			angle_ad_point2 = bytes_to_word(EEPROM_rx_buf + 2); //2point AD��
			angle_ad_point3 = bytes_to_word(EEPROM_rx_buf + 4); //3point AD��	
			angle_ad_point4 = bytes_to_word(EEPROM_rx_buf + 6); //4point AD��
			//angle_ad_point5 = bytes_to_word(EEPROM_rx_buf + 8); //5point AD��
			save_angle_end_flag = 1;
		}
		if(real_angle_value_page2_end)
		{//save_angle_end_flag ������ 1�� �ǰ� 1.4�� �ڿ� ����
			real_angle_value_page2_end = 0;
			anlge_real_err_flag = EEPOM_Read(DEVICE_ADDRESS, Byte_num_16, 3);//(I2C�� �ּ�, �� �������� 16bytye, �б� �������ּ�)	
			real_angle_point1= 0, real_angle_point2 = 0, real_angle_point3 = 0;
			real_angle_point1 = bytes_to_float(EEPROM_rx_buf + 0);//1point ���� ���� �� LCD�κ��� �о�� ����
			real_angle_point2 = bytes_to_float(EEPROM_rx_buf + 4);//2point ���� ���� �� LCD�κ��� �о�� ����
			real_angle_point3 = bytes_to_float(EEPROM_rx_buf + 8);//3point ���� ���� �� LCD�κ��� �о�� ����	
			Last_page = 1;
		}
		if(Last_page_end)	
		{//Last_page ������ 1�� �ǰ� 1.4�� �ڿ� ����
			Last_page_end = 0;
			anlge_real_err_flag = EEPOM_Read(DEVICE_ADDRESS, Byte_num_16, 4);//(I2C�� �ּ�, �� �������� 16bytye, �б� �������ּ�)	
			if(EEPROM_rx_buf[11] == 0x77)//EEPROM�� �������� ���� �Ǿ� �ֳ� Ȯ��
				SYSTEM_INITIALIZATION_VALUE_ANGLE = 0;
			/*
			else 
				SYSTEM_INITIALIZATION_VALUE_ANGLE = 1;//�̰��� ������ �ٴ� ���� EEPROM�� ������ ������ or �����Ͱ� ���� �Ǿ����� �ʴٴ� ���� �ǹ��Ѵ�.	
			*/			
			real_angle_point4 = 0, real_angle_point5 = 0;
			real_angle_point4 = bytes_to_float(EEPROM_rx_buf + 0);//4point ���� ���� �� LCD�κ��� �о�� ����	
			//real_angle_point5 = bytes_to_float(EEPROM_rx_buf + 4);//5point ���� ���� �� LCD�κ��� �о�� ����	
			enable_low_to_high = 1;
		}
		if(default_save_angle)//anlge_ad_err_flag, anlge_real_err_flag �� �� ������ ��� ����
		{//�̰��� �����Ѵٴ� ���� crc�� ������ ����ٴ� �ǹ�
			default_save_angle = 0;
			crc_err_check(2);//�������� �̻��� ����� default�� �ִ� �κ�
		}
		if((enable_low_to_high == 1) && (default_save_angle == 0) && (calibration_state == 1)) //calibration ing state
		{
			enable_low_to_high = 0;
			calibration_state = 0;//Ķ���극�̼� ��Ȳ�� ���� low to high �ϴ� ����
			low_to_high = 1;
		}
		if(crc_write)
		{
			crc_write = 0;
			static byte crc_step = 0;
			switch(crc_step)
			{
				case 0:
					EEPROM_Write(DEVICE_ADDRESS, 2, Arr_eeprom2);	//(I2C�ּ�, EEPROM������, Tx�� �迭)	
					wait_1400m_flag = 1;
					if(wait_1400m_start)
					{
						wait_1400m_start = 0;
						crc_step = 1;
					}
					break;
				case 1:
					EEPROM_Write(DEVICE_ADDRESS, 3, Arr_eeprom3);	//(I2C�ּ�, EEPROM������, Tx�� �迭)	
					wait_1400m_flag = 1;
					if(wait_1400m_start)
					{
						wait_1400m_start = 0;
						crc_step = 2;
					}
					break;
				case 2:
					EEPROM_Write(DEVICE_ADDRESS, 4, Arr_eeprom4);	//(I2C�ּ�, EEPROM������, Tx�� �迭)	
					wait_1400m_flag = 1;
					if(wait_1400m_start)
					{
						wait_1400m_start = 0;
						crc_step = 0;
					}
					break;				
			}
		}	
		if(ct_cal_ad_1)
		{
			ct_1();
			ct_cal_ad_1 = 0;
			int_to_bytes(integer_ct_Value_1, bytes_data);
			for(byte an = 0; an <= 1; an++)//2byte
				Arr_eeprom1[an] = bytes_data[an];
			integer_ct_Value_1 = float_to_uint32(real_current_500mA);
			int_to_bytes(integer_ct_Value_1, bytes_data);			
			for(byte an = 4; an <= 7; an++)//4byte
				Arr_eeprom1[an] = bytes_data[an - 4];
		}
		if(ct_cal_ad_2)
		{		
			ct_1();
			ct_cal_ad_2 = 0;
			int_to_bytes(integer_ct_Value_2, bytes_data);
			for(byte ct = 2; ct <= 3; ct++)//2byte
				Arr_eeprom1[ct] = bytes_data[ct - 2];
			integer_ct_Value_2 = float_to_uint32(real_current_2500mA);
			int_to_bytes(integer_ct_Value_2, bytes_data);			
			for(byte ct = 8; ct <= 11; ct++)//4byte
				Arr_eeprom1[ct] = bytes_data[ct - 8];
			Arr_eeprom1[13] = 0x77; //EEPROM�� ������ ����
			EEPROM_Write(DEVICE_ADDRESS, 5, Arr_eeprom1);	//(I2C�ּ�, EEPROM������, Tx�� �迭)		
			save_ct_flag = 1;
		}
		if(save_ct_end)//save_ct_flag ������ 1�� �ǰ� 1.4�� �ڿ� save_ct_end�� 1
		{
			save_ct_end = 0;
			ct_err_flag = EEPOM_Read(DEVICE_ADDRESS, Byte_num_16, 5);
			if(ct_err_flag)
			{
				ct_err_flag = 0;
				SYSTEM_INITIALIZATION_VALUE_CT = 1;
			}
			if(EEPROM_rx_buf[13] == 0x77)
				SYSTEM_INITIALIZATION_VALUE_CT = 0;
			else 
				SYSTEM_INITIALIZATION_VALUE_CT = 1;
			cal_500mA_val = 0, cal_2500mA_val = 0;			
			cal_500mA_val = bytes_to_word(EEPROM_rx_buf + 0);     	//0.2A AD��
			cal_2500mA_val = bytes_to_word(EEPROM_rx_buf + 2);   		//2.0A AD��
			real_current_500mA= 0, real_current_2500mA = 0;
			real_current_500mA = bytes_to_float(EEPROM_rx_buf + 4);   //0.2A ���� �� LCD�κ��� �о�� ���� 
			real_current_2500mA = bytes_to_float(EEPROM_rx_buf + 8);   //2.0A�� ���� �� LCD�κ��� �о�� ����	
		}		
		if(default_save_ct)
		{
			ct_err_flag = 0;
			default_save_ct = 0;
			crc_err_check(5);//�������� �̻��� ����� default�� �ִ� �κ�
		}	
		if(_VOICE_DATA.x06_1B || _USER_RUN_SETTING.x06_1B)
			voice_en = 1;
		if(_VOICE_DATA.x06_1B == 3)
		{
			_VOICE_DATA.x06_1B = 0;
			Settings = 3;
			//voice_en = 1;
		}				
		if(_VOICE_DATA.x06_1B == 5 && Settings == 3)
		{
			MOTOR_ROTATE;//������ -> ����
			_VOICE_DATA.x06_1B = 0;
			Settings_cancel = 5;
			if(cant_read_id == 0)//������� ������
				Settings = 0;	
		}		
		if(_VOICE_DATA.x06_1B == 4 && Settings == 3)
		{
			Settings_save = 4;	
			MOTOR_ROTATE;//������ -> ����
			//voice_en = 1;
		}
		if((bf_voice_back != _VOICE_DATA.Volum_up_down) && _VOICE_DATA.Volum_up_down)
		{
			if(Setting_in == 0)//�ý����� ó�� ������ �� ó�� ȯ�漳�� ���� �ú���x�ܰ� ����� ����
				_VOICE_DATA.Volum_up_down = 0;			
			volum_count = _VOICE_DATA.Volum_up_down; // 05.08 �ּ� ����
			switch(volum_count)
			{
				case 0:
					//_VOICE_DATA.x06_1B = 0x26; //���Ұ�
					break;							
				case 1:
					_VOICE_DATA.x06_1B = 0x1C; //1�ܰ�
					break;
				case 2:			
					_VOICE_DATA.x06_1B = 0x1D; //2�ܰ�
					break;
				case 3:			
					_VOICE_DATA.x06_1B = 0x1E; //3�ܰ�
					break;
				case 4:				
					_VOICE_DATA.x06_1B = 0x1F; //4�ܰ�
					break;
				case 5:
					_VOICE_DATA.x06_1B = 0x20; //5�ܰ�
					break;
			}
			bf_voice_back = _VOICE_DATA.Volum_up_down;
			_VOICE_DATA.Volum_up_down = 0;
		}
 
		/**������� �̸� ���� Ű**/
		
		if(ble_name_change_fg) // ������� �̸� ���� Ű
		{
		  ble_name_change_fg = 0;
		  //sprintf(Ble_buffer,"%s",AT_BTNAME_H);
		  for (int i = 0; AT_BTNAME[i] != '\0'; i++) 
		  {
			  Ble_buffer[i] = AT_BTNAME[i];//sprintf��� ����Ͽ���.
		  }
		  if(ENQ < 0x64) // 100���� ������
		  {
		  	Ble_Data_Tx(19);
		  }
		  else if(ENQ >= 0x64) // 100���� ũ�ų� ������
		  {
			Ble_Data_Tx(20);
		  }
		}
		
		if(Ble_CheckSum_EN)
		{
		  Ble_CheckSum_EN = 0;
			if(Rx3_buf[0] == 'R' && Rx3_buf[1] == 'D') //�б� �Լ�
			{
				Ble_Rcv_ok = 0x03;
			}
		}
		Ble_UART3_exe();
		//HAL_UART_Receive(&huart3,RE_re_1,100,1000);
		
		/*for(int ble_cnt = 0; ble_cnt <= sizeof(RE_re_1); ble_cnt++)
		{
			if(RE_re_1[ble_cnt] == 'R' && RE_re_1[ble_cnt+1] == 'D')
			{
				HAL_UART_Transmit(&huart3,Ble_test_buffer,strlen(Ble_test_buffer),1000);
				for(int reset_flag = 0; reset_flag <= sizeof(RE_re_1); reset_flag++)
				{
					RE_re_1[reset_flag] = '\0';
				}
			}
		}*/

		/*if(lora_tx_EN)
		{
			lora_tx_EN = 0;
			byte lora_tx_num = 0;
			for(byte i = 0; Tx2_buf[i] != 0x00; i++)
			{
				lora_tx_num++;
			}
			LoRa_Tx(lora_tx_num+1);
			
			LoRa_RS232_putchar(15);
		}*/
		/** Read / Write **/
		/*_LoRa_Data.user_name_1 = plus_cnt;					// 0x2000_0000 : 0x0000 ~ 0xFFFF ( ȯ�� �̸�(��) ) ǥ�ذ� : 0xD64D( ȫ )
		_LoRa_Data.user_name_2 = plus_cnt - 1;				// 0x2000_0002 : 0x0000 ~ 0xFFFF ( ȯ�� �̸�(�̸�1) ) ǥ�ذ� : 0xAE38( �� )
		_LoRa_Data.user_name_3 = plus_cnt - 2;				// 0x2000_0004 : 0x0000 ~ 0xFFFF ( ȯ�� �̸�(�̸�2) ) ǥ�ذ� : 0xB3D9( �� )
		_LoRa_Data.user_name_4 = plus_cnt - 3;				// 0x2000_0006 : 0x0000 ~ 0xFFFF ( ȯ�� �̸�(�̸�3) )
		_LoRa_Data.user_name_5 = plus_cnt - 4;				// 0x2000_0008 : 0x0000 ~ 0xFFFF ( ȯ�� �̸�(�̸�4) )
		_LoRa_Data.user_name_6 = plus_cnt - 5;				// 0x2000_000A : 0x0000 ~ 0xFFFF ( ȯ�� �̸�(�̸�5) )		
		_LoRa_Data.user_age = plus_cnt - 6;					// 0x2000_000C : 1 ~ 120�� 
		_LoRa_Data.user_sex = plus_cnt -7;					// 0x2000_000D	   : �� : 0, �� : 1
		_LoRa_Data.user_num_high = plus_cnt - 8;			// 0x2000_000E : 0 ( ȯ�� ��� ��ȣ 1 ~ 99999999 ) ( ���� 2byte ) 
		_LoRa_Data.user_num_low = plus_cnt - 9;				// 0x2000_0010 : 0 ( ȯ�� ��� ��ȣ 1 ~ 99999999 ) ( ���� 2byte ) 
		_LoRa_Data.upper_angle = plus_cnt - 10;				// 0x2000_0012 : 0 ���Ѱ� ( 0 ~ 230 ����� ���� ������ ���� ���� -50 ó���� ������ ) ( -50 ~ 180 )
		_LoRa_Data.lower_angle = plus_cnt - 11;				// 0x2000_0013 : 0 ���Ѱ� ( 0 ~ 230 ����� ���� ������ ���� ����  -70 ó���� ������ ) ( -70 ~ 160 )
		_LoRa_Data.stop_time = plus_cnt -12;				// 0x2000_0014 : ( ���������ð� : ���� 4bit, ���������ð� : ���� 4bit )
		_LoRa_Data.mode = plus_cnt - 13;					// 0x2000_0015 : ǥ�ذ��� : 0x00( �Ϲ� ��� ), [ 0x00 : �Ϲ�(���), 0x01 : �Ϲ�(����), 0x02 : ����, 0x03 : ���� ]
		_LoRa_Data.velocity_mode = plus_cnt -14;			// 0x2000_0016 : 0 ( � �ӵ� : 1 ~ 9�ܰ� )
		_LoRa_Data.exerc_time_and_exercnum = plus_cnt - 15;	// 0x2000_0017 : 0 ( 1 ~ 99 �����ð� �Ǵ� �Ƚ�� �����̹Ƿ� MSB�� 0�̸� ��ð�, 1�̸� �Ƚ�� )
		_LoRa_Data.special_angle = plus_cnt - 16;			// 0x2000_0018 : 5 ~ 15 ( ����/���� � ���� �ּ� 5��, �ִ� 15�� )
		_LoRa_Data.repeat_num = plus_cnt - 17;				// 0x2000_0019 : 3 ~ 10 ( ����/���� � �ݺ� Ƚ�� �ּ� 3ȸ, �ִ� 10ȸ )
		_LoRa_Data.special_location = plus_cnt - 18;		// 0x2000_001A : 0 ~ 2( ���Ѱ� : 0x00, ���Ѱ� : 0x01, �����Ѱ� : 0x02 )
		_LoRa_Data.motion = plus_cnt -19;					// 0x2000_001B : 0 ~ 4 ( ǥ�ذ��� : 0x00 ( ��� ), [ 0x00 : ���, 0x01 : �Ȳ�ġ, 0x02 : ���� , 0x03 : �ո�, 0x04 : �߸� ] )*/
	
		/** Read Only **/
		/*_LoRa_Data.check_state = plus_cnt - 20;             // 0x2000 001C : 0 ����üũ �Ϸ� : 0x00, ���� ����üũ ������ : 0x02, ���ڴ� ����üũ ������ : 0x04, ���� ����üũ ������: 0x08
		_LoRa_Data.state = plus_cnt - 21;					// 0x2000_001D : �ֱ������� ���(��� �ֱ� ����) / ����� : 0x00, ����� : 0x01, ���� : 0x02, ������� : 0x03
		_LoRa_Data.use_state = plus_cnt - 22;				// 0x2000_001E : ��� : 0x00, ���� : 0x01, ������ : 0x02, ���� : 0x03
		_LoRa_Data.error_state = plus_cnt - 23;				// 0x2000_001F : 0 ( 0 ~ 5, ������� : 0x01, ������ : 0x02, ���ڴ� : 0x03, ������Ż : 0x04, ��ſ��� : 0x05 )
		_LoRa_Data.remain_time_and_cnt = plus_cnt -24; 		// 0x2000_0020 : 0 ( 0 ~ 99 ���� ��ð� �Ǵ� ���� �Ƚ�� �����̹Ƿ� MSB�� 0�̸� ���� ��ð�, 1�̸� ���� �Ƚ�� )
		_LoRa_Data.upper_limit_angle = plus_cnt -25;		// 0x2000_0021 : 0 ���� ���Ѱ� ( 0 ~ 230 ����� ���� ������ ���� ���� -50 ó���� ������ ) ( -50 ~ 180 )
		_LoRa_Data.lower_limit_angle = plus_cnt -26;		// 0x2000_0022 : 0 ���� ���Ѱ� ( 0 ~ 230 ����� ���� ������ ���� ����  -70 ó���� ������ ) ( -70 ~ 160 )
		_LoRa_Data.rsv1 = 0;								// 0x2000_0023 : 0( Dummy_Data1 )*/
	
		/**PC�� ��� �ϱ� �� ����ü�� �����͸� �ִ� �κ�( �׽�Ʈ �� ) **/
		
		//������ ������, CRC16 �� ��// LoRa�� Motor Driver������ ���
		if(LoRa_CheckSum_EN) // UART2 Interrupt root���� �������� �������� ������ ���Ž� 1
		{
		  word LoRa_w_check;
		  static byte LoRa_cal_number;
		  LoRa_CheckSum_EN = 0;
		  
		  if(LoRa_Rxbuf[1] == 0x04) //�б� �Լ�
		  {
			LoRa_cal_number = 6; // LoRa_Rxbuf[0] ~ LoRa_Rxbuf[5] ������ 6byte CRC16 ���
		  }
		  else if(LoRa_Rxbuf[1] == 0x10) // ���� �Լ�
		  {
			LoRa_cal_number = LoRa_Rxbuf[5]*2 + 7; // // LoRa_Rxbuf[5]*2 + 7 ���� CRC16 ���
		  }
		  
		  LoRa_CheckSum_data = CRC16(LoRa_Rxbuf, LoRa_cal_number); // ������ �������� CRC16 ���
		  
		  LoRa_CheckSum_ing = 0; // UART2 ���Ű���
		  LoRa_w_check = (Rx2_CRC_H<<8) | Rx2_CRC_L;	
		  LoRa_test_crc1 = LoRa_w_check;
		  LoRa_test_crc2 = LoRa_CheckSum_data;
		  
		  if(LoRa_CheckSum_data == LoRa_w_check)	//CRC16 OK!
		  {
			  if(LoRa_Rxbuf[1] == 0x10)	//���� ACK
			  {
				  LoRa_Rcv_ok = 0x10;
				  LoRa_RS485_dead_time = LORA_RS485_DEAD_TIME;	//15ms delay�� ����ó��, 1msec���� 1�� �����Ͽ� 0�϶� Rcv5_ok = 0x01;
			  }
			  else if(LoRa_Rxbuf[1] == 0x04)	//�б� ACK
			  {
				  LoRa_Rcv_ok = 0x04;
				  LoRa_RS485_dead_time = LORA_RS485_DEAD_TIME;	//15ms delay�� ����ó��, 1msec���� 1�� �����Ͽ� 0�϶� Rcv5_ok = 0x44;
			  }
		  }
		  else
		  {
			  LoRa_Rcv_ok = 0x0E;
			  LoRa_CheckSum_data = CRC16(LoRa_Rxbuf, LoRa_cal_number);	//������ �������� CRC���	
			  LoRa_RS485_dead_time = LORA_RS485_DEAD_TIME;	//15ms delay�� ����ó��, 1msec���� 1�� �����Ͽ� 0�϶� Rcv5_ok = 0x01;
		  }
		}
		LoRa_UART2_exe(); 
		/*****************************************�߰��� �κ�(����)*************************************************/
		//������ ������, Check Sum üũ(23.01.11, CRC���׷� ���� Check Sum���� ��ü) // Motor Driver�� LCD ������ ���
		if(_SENSOR_DATA.CT_error == 2 || _SENSOR_DATA.PM_error == 2 || _SENSOR_DATA.encoder_error == 2)
			PWM_NOT_SLEEP;
		if(CheckSum_EN)
		{
			static byte cal_number;
			CheckSum_EN = 0;
			MT_Driver_crc = 0;
			LCD_crc = 0;
			if(Rx4_buf[1] == 0x04)
				cal_number = 6;
			else if((Rx4_buf[1] == 0x10) || (Rx4_buf[1] == 0x08) || Rx4_buf[1] == 0x06)
				cal_number = (Rx4_buf[5]*2)+7;
			MT_Driver_crc = CRC16(Rx4_buf, cal_number);//������ �������� CheckSum���	
			CheckSum_ing = 0;
			LCD_crc = (Rx4_CRC_H<<8) | Rx4_CRC_L;	
			if(MT_Driver_crc == LCD_crc)//CheckSum OK!
			{
				if(Rx4_buf[1] == 0x10)	//���� ACK
				{
					Rcv2_ok = 0x10;
					//RS485_dead_time = 100;	//100ms delay�� ����ó��, 1msec���� 1�� �����Ͽ� 0�϶� Rcv5_ok = 0x110;
				}
				else if(Rx4_buf[1] == 0x04)	//�б� ACK
				{
					Rcv2_ok = 0x04;
					//RS485_dead_time = 100;	//100ms delay�� ����ó��, 1msec���� 1�� �����Ͽ� 0�϶� Rcv5_ok = 0x44;
				}
				else if(Rx4_buf[1] == 0x06)	//Calibration ACK
				{
					Rcv2_ok = 0x06;
					//RS485_dead_time = 100;	//100ms delay�� ����ó��, 1msec���� 1�� �����Ͽ� 0�϶� Rcv5_ok = 0x66;
				}
				else if(Rx4_buf[1] == 0x08)	//������� ACK
				{
					Rcv2_ok = 0x08;
					//RS485_dead_time = 100;	//100ms delay�� ����ó��, 1msec���� 1�� �����Ͽ� 0�϶� Rcv5_ok = 0x66;
				}				
			}
			else
			{
				Rcv2_ok = 0x0E;
				CheckSum_data = CRC16(Rx4_buf, cal_number);	//������ �������� CRC���	
				//RS485_dead_time = 100;	//100ms delay�� ����ó��, 1msec���� 1�� �����Ͽ� 0�϶� Rcv5_ok = 0x55;
			}
		}
		UART4_exe(); 	//UART2 RS232
		static byte start_selfcheck_flag = 0;
		static byte ems_start = 0;
		//����üũ ���� ����(���� ���� �ִ�, �ּ� ������ ����� �ʰ� ����üũ�ϱ� ����)


		if(Priority_dir_set == 'S')	//'S' => Start
		{
			ems_start = 1;
			if(EMS_STATE)
			{
				_VOICE_DATA.x06_1B = 0x2B;//���������ư�� �����ϼ���. �������
			}
			else if(EMS_STATE == 0)
			{
				Priority_dir_set = 'E';	//�ٽ� ����üũ �����ϴ� ���� ����.	//'E' => End
				switch(_USER_RUN_SETTING.motion)
				{		
					case Elbow:	    //�Ȳ�ġ
						if(Current_angle >= 115) 	//115�� �̻��϶�
							CCW_priority_mode = 1;	//CCW ���� üũ
						else
							CCW_priority_mode = 0;	//CW ���� üũ
						break;
					case Shoulder:	//���
						if(Current_angle >= 160) 	//160�� �̻��϶�
							CCW_priority_mode = 1;	//CCW ���� üũ
						else
							CCW_priority_mode = 0;	//CW ���� üũ
						break;							
					case Knee:	    //����
						if(Current_angle >= 120) 	//120�� �̻��϶�
							CCW_priority_mode = 0;	//CW ���� üũ
						else
							CCW_priority_mode = 1;	//CCW ���� üũ
						break;
					case wrist:	    //�ո�
						if(Current_angle >= 50) 	//�� �̻��϶�
							CCW_priority_mode = 1;	//CCW ���� üũ
						else
							CCW_priority_mode = 0;	//CW ���� üũ
						break;
					case ankle:	    //�߸�
						if(Current_angle >= 30) 	//�� �̻��϶�
							CCW_priority_mode = 1;	//CCW ���� üũ
						else
							CCW_priority_mode = 0;	//CW ���� üũ
						break;					
				}
				start_selfcheck_flag = 1;
			}
		}
		
		if(start_selfcheck_flag && _USER_RUN_SETTING.calibration == 0)
		{
			PWM_NOT_SLEEP;
			start_selfcheck_flag = 0;
			Error_Check = 1;//����üũ ����
		}
		// �μ����� ������ �ϽŰ� ������� ��ư ���� ������ �̰� �ּ� Ǯ�� �Ʒ� ���찡 �� �����!!!!
		/*if(EMS_STATE && Priority_dir_set != 'S' && ems_start)//Emergency Stop SW ON!
		{
	  		_LoRa_Data.error_state |= 0x01;// ������� ���� ȭ�� ǥ�� 24.04.01 ���� 	
			_SENSOR_DATA.ESB_state |= 0x01;// ������� ���� ȭ�� ǥ�� 24.02.28 �߰� 
			_VOICE_DATA.x06_1B = 0x17; //������� �Ǿ����ϴ�. �������
		}
		else
		{
	  		_LoRa_Data.error_state &= ~0x01;// ������� ���� ȭ�� ���� 24.04.01 ���� 	
			_SENSOR_DATA.ESB_state &= ~0x01;// ������� ���� ȭ�� ���� 24.02.28 �߰� 
		}*/
		//���찡 �Ѱ�
		static int ems_sound = 0;
		if(EMS_STATE)//Emergency Stop SW ON!
		{
			total_low_angle = 0, total_high_angle = 0;
	  		_LoRa_Data.error_state |= 0x01;// ������� ���� ȭ�� ǥ�� 24.04.01 ���� 	
			_SENSOR_DATA.ESB_state |= 0x01;// ������� ���� ȭ�� ǥ�� 24.02.28 �߰� 
			//���찡��.
			if(++ems_sound >= 50000)
			{
			  ems_sound = 50000;
			  _VOICE_DATA.x06_1B = 0x17; //������� �Ǿ����ϴ�. �������
			}
		}
		else
		{
	  		_LoRa_Data.error_state &= ~0x01;// ������� ���� ȭ�� ���� 24.04.01 ���� 	
			_SENSOR_DATA.ESB_state &= ~0x01;// ������� ���� ȭ�� ���� 24.02.28 �߰� 
			if(ems_sound == 50000)
			{
				ems_sound = 0;
				_VOICE_DATA.x06_1B = 0x2C;//"������� ��ư�� �����Ǿ����ϴ�." �������
			}
		}
		
	  	/***** TIMER *****/
		if(T10ms_flag == 1)	
		{//10ms task
			T10ms_flag = 0;
			Exercise_ctrl();	//� ��� ���� �κ�(�Ϲ�, ����, ����)
			/******************************���� üũ******************************/
			if(!Self_check_end)	//����üũ��
			{
				if(Self_Check == 0)	//Self Check ����� OFF�Ǿ� ������ ���Ƿ� ���� ó��
				{
					//���Ƿ� ���� ó��
					_SENSOR_DATA.PM_error = 1;		//����
					_SENSOR_DATA.CT_error = 1;		//����
					_SENSOR_DATA.encoder_error = 1;	//����
				}
				else
				{
					_LoRa_Data.check_state |= 0x02;//�������� ����üũ �� 24.02.28 ���� Para.CT_error = 0;	
					_SENSOR_DATA.CT_error = 0x00;//�������� ����üũ ��
					_LoRa_Data.check_state |= 0x04;//���ڴ� ����üũ �� 24.02.28 ���� Para.encoder_error = 0;	
					_SENSOR_DATA.encoder_error = 0x00;//���ڴ� ����üũ �� 
					_LoRa_Data.check_state |= 0x08;//�������� ����üũ �� 24.02.28 ���� Para.PM_error = 0;
					_SENSOR_DATA.PM_error = 0x00;//�������� ����üũ ��
				}
			}
			else if(Self_check_end == 1 && Self_check_end)//����üũ �Ϸ�
			{
				_LoRa_Data.check_state = 0;//����üũ ������ �ʱ�ȭ ������
				if(Self_Check == 0)	//Self_Check�� 0:����üũ ��� OFF, 1:����üũ ��� ON
				{
					//���Ƿ� ���� ó��
					_SENSOR_DATA.PM_error = 1;		//����
					_SENSOR_DATA.CT_error = 1;		//����
					_SENSOR_DATA.encoder_error = 1;	//����
				}
				else
				{			
					if(_SENSOR_DATA.ESB_state == 0)
						_SENSOR_DATA.EMS_error = 1;//������� ����
					else 
					{
						if(++EMS_start_cnt >= 800)
						{
							EMS_start_cnt = 0;
							_SENSOR_DATA.EMS_error = 2;//������� 
						}
					}
					if(cant_read_id == 0)
					{
						voice_error_check = 1;//������� ����üũ
					}
					if(ADC1_data[1] == 0)	//���������� ������ ��
					{
						_LoRa_Data.error_state |= 0x08;//�������� ����
						_SENSOR_DATA.PM_error = 2;//�������� ����
					}
					else
					{
						if((CW_pm_check_ok + CCW_pm_check_ok) == 2)
						{
							_LoRa_Data.error_state &= ~0x08;//�������� ����
							_SENSOR_DATA.PM_error = 1;//�������� ����
						}
						else
						{
							_LoRa_Data.error_state |= 0x08;	//�������� ����
							_SENSOR_DATA.PM_error = 2;//�������� ����
						}
					}
					if((CW_ct_check_ok + CCW_ct_check_ok) == 2)
					{
						_LoRa_Data.error_state &= ~0x02;//�������� ����
						_SENSOR_DATA.CT_error = 1;//�������� ����
					}
					else
					{
						_LoRa_Data.error_state |= 0x02;	//�������� ������
						_SENSOR_DATA.CT_error = 2;//�������� ������
					}
					if((CW_encoder_check_ok + CCW_encoder_check_ok) == 2)
					{
						_LoRa_Data.error_state &= ~0x04;//���ڴ� ����
						_SENSOR_DATA.encoder_error = 1;
					}
					else
					{
						_LoRa_Data.error_state |= 0x04;//���ڴ� ����
						_SENSOR_DATA.encoder_error = 2;
					}
				}
			}
			/*******************************************************************/
		}
		if(T100ms_flag==1)
		{
			//100ms task
			T100ms_flag=0;
			static byte t200ms_cnt = 0;
			if(++t200ms_cnt >= 2)
				t200ms_cnt = 0;
    	}
		static byte error_touch = 0;
		error_touch = _USER_RUN_SETTING.re_self_check;
		_USER_RUN_SETTING.re_self_check = 0;
		if(error_touch)//���ڴ� or ���ټ� or ���� �ϳ��� ������ ���� ��� ���� ȭ�� ���� -> ȭ�� ��ġ�� ����
		{
			error_touch = 0;
			CW_pm_check_ok = 0;CCW_pm_check_ok = 0;CW_ct_check_ok = 0;CCW_ct_check_ok = 0;
			CW_encoder_check_ok = 0;CCW_encoder_check_ok = 0;			
			PWM_NOT_SLEEP;
			_SENSOR_DATA.EMS_error = 0;
			_SENSOR_DATA.CT_error = 0;
			_SENSOR_DATA.PM_error = 0;
			_SENSOR_DATA.encoder_error = 0;
			Restart_self_check = 1;//����üũ �ٽý���
		}
	}//while�� ����
  /* USER CODE END 3 */
}//main�� ����
/*********************************************************************************/
/*******************************���� ���� ���� �Լ�************************************/

//PID ���� �Լ� ���� ���� ����
//PID ��Ʈ��				//PID����� PI����� ���� �� I���� �ڵ� ����(22.11.18) 
float PID_control_sys(float target, float current) 
{
	//����(����) ���
	error = target - current;				//����		p
    accError += error;// + pre_error;		//������ ����	i
	//errorGap = pre_error - error;         //���� ������ ���� ������ ��	d
	
	
	if(accError > 3600)
		accError = 3600;
	else if(accError < -3600)
		accError = -3600;
	
	pControl = P_GAIN * error;				//P(�����) �����
	iControl = I_GAIN * (accError * TIME);	//I(������) �����
	//dControl = D_GAIN * (errorGap / TIME);	//D(�̺���) �����
	
	//pre_error = error;		//������ ������ ���� ���Ű��� ����
	pidControl = pControl + iControl;// + dControl;	//PID �����
	return pidControl;
}

// MPC ���� ���� ����
float A = 1.0;    // �ý����� ���� ���� ��� (���⼭�� ������ 1�� ����)
float B = 1.0;    // �Է¿� ���� �ý����� ���� ���
float Q = 1.0;    // ��� ������ ���� ����ġ
float R = 0.1;    // ���� �Է¿� ���� ����ġ

float u = 0;      // ���� �Է�
float x = 0;      // �ý��� ���� (���)

float predictHorizon = 10; // ���� �ð� ����
float controlHorizon = 1;  // ���� �ð� ����

// MPC ���� �Լ�
float MPC_control_sys(float target, float current) 
{
    float error = target - current;   // ���� ���� ���
    float futureX = current;          // �̷� ���� ���� �ʱ�ȭ
    float optimalControl = 0;
    float minCost = 1000000;          // �ּ� ��� �ʱ�ȭ (ū ������ ����)

    // ���� ���� �Է��� �õ��Ͽ� ������ ���� ��ȣ�� ã��
    for (float candidateU = -1; candidateU <= 1; candidateU += 0.1) 
    {
        float cost = 0;
        futureX = current;

        // ���� ���� ������ ��� ���
        for (int t = 0; t < predictHorizon; t++) 
        {
            futureX = A * futureX + B * candidateU;  // �̷� ���� ����
            float futureError = target - futureX;    // �̷� ���� ���
            cost += Q * futureError * futureError + R * candidateU * candidateU; // ��� ���
        }

        // �ּ� ����� ���� ���� �Է� ����
        if (cost < minCost) 
        {
            minCost = cost;
            optimalControl = candidateU;
        }
    }

    u = optimalControl;  // ������ ���� �Է� ����
    x = A * x + B * u;   // ���� ���� ������Ʈ

    return u;
}






//���� PWM ����(PID)
float set_motor_pwm(float PID_Value)
{
	if(Dir_of_rotation == CW)				//�ð����
		PID_Value = TIM4_CCR2_buffer + PID_Value;
	else if(Dir_of_rotation == CCW)	//�ݽð� ����
		PID_Value = TIM4_CCR1_buffer + PID_Value;
		//PID_Value = (htim4.Instance->CCR1) + PID_Value;
	else
		return 0;
	//����ó��, PWM �ּ�,�ִ� �� ������ ������ ��� ���
	if(PID_Value > 3600)
		PID_Value = 3600;
	else if(PID_Value < 0)
		PID_Value = 0;
	
	return PID_Value;
}
					    //   ���� �ð�,		   ���� �ð�, 		  �ִ�ӵ�(�ܰ�)	   	�ӵ����
void Soft_start_stop_PI(word accel_time, word deaccel_time, byte speed_step, byte velocity_mode)	//accel_time = 100*10ms�� 1��, Call by 10ms Task
{	
	static int set_rpm, accel_rpm;
	//22.12.19. ���������� ���� �ӵ�����, �̵����� 15�� �̸��� ��� ���� ��Ż�ϴ� ���� �ذ��� ����
	if((DISTANCE < 15) && (speed_step >= 5))//23.02.10 ��Ż���� ���� �ӵ��ܰ踦 6�ܰ��̻󿡼� 5�ܰ��̻����� ����
		speed_step = speed_step - 2;
	if(_USER_RUN_SETTING.motion == Knee)//�����
		upper_correction_angle  = 7.0, lower_correction_angle = 6.0;	
	else
		upper_correction_angle  = 9.0, lower_correction_angle = 7.0;
	switch(speed_step)//�ӵ��ܰ迡 ���� ���� ���,���� rpm ����(9�ܰ���� ��������)
	{				  //DC���Ϳ� �ⱸ���� ���� = 1500:1
		case 0:	//������� �Ͻ�������(23.06.23)
			set_rpm = 0;
			break;
		case 1:
			set_rpm = 460;	//�ⱸ�� RPM = 0.277
			accel_rpm = 700;
			break;
		case 2:
			set_rpm = 665; //�ⱸ�� RPM = 0.37
			accel_rpm = 850;
			break;
		case 3:
			set_rpm = 830; //�ⱸ�� RPM = 0.463
			accel_rpm = 1050;
			break;
		case 4:
			set_rpm = 1015; //�ⱸ�� RPM = 0.556
			accel_rpm = 1300;
			break;
		case 5:
			set_rpm = 1200; //�ⱸ�� RPM = 0.639//105
			accel_rpm = 1400;
			break;
		case 6:
			set_rpm = 1340; //�ⱸ�� RPM = 0.789
			accel_rpm = 1580;
			break;
		case 7:
			set_rpm = 1460; //�ⱸ�� RPM = 0.833//140
			accel_rpm = 1850;
			break;
		case 8:
			set_rpm = 1615; //�ⱸ�� RPM = 0.917
			accel_rpm = 2100;
			break;
		case 9:
			set_rpm = 1760; //�ⱸ�� RPM = 1.0
			accel_rpm = 2300;
			break;
		default:
			break;
	}
	static byte cal_accel_angle_f;
	if(deaccel_flag == 0)
	{ 
		if(speed_step == 0)		//speed_step�� 0�϶��� ���ͼӵ��� 0 ����ó��
		{
			f_m_speed = 0;
			exercise_mode = 0;
			Motor_PWM = (dword)f_m_speed;
		}
		/*
		else if((speed_step >= 1) && (target_speed < set_rpm))
		{
			target_speed = ((set_rpm-720)/(float)accel_time)*pwm_step + 720;  //Soft Start
			pid_val = PID_control_sys(target_speed, speed_RPM);
			Motor_PWM = (unsigned int)set_motor_pwm(pid_val);	//���� PWM PID ����
			

			static word test_kcnt2 = 0;
			test_kcnt2++;
		}
*/		

		
		target_speed = ((set_rpm-240)/(float)accel_time)*pwm_step + 240;  //Soft Start Soft Stop[soft stop�� �ּҼӵ��� 240(duty=15%)]
		pid_val = PID_control_sys(target_speed, speed_RPM);
		Motor_PWM = (unsigned int)set_motor_pwm(pid_val);	//���� PWM PID ����
		if(pwm_step < accel_time)	//���� step�� ������ ���ӽð����� ���� ��� ���� pwm_step 1������
		{
			pwm_step++;
			if(pwm_step == accel_time-1)
				cal_accel_angle_f = 1;
		}
		static byte direction_1, direction_2;
		//�����ⱸ ���� �� ���� ���� �׽�Ʈ ����(23.02.02)
		if(_USER_RUN_SETTING.motion == Knee)	//����
		{
			direction_1 = CCW;
			direction_2 = CW;
		}
		else	//���, ����
		{
			direction_1 = CW;
			direction_2 = CCW;
		}
		if(target_speed >= set_rpm) //�����ӵ��� �� ��
		{
			if(cal_accel_angle_f == 1)	//�����ӵ��� �Ǹ� ������ ���� ���
			{
				if(Dir_of_rotation == direction_1)	  //�ð����
					accel_angle = DISTANCE - angle_distance + upper_correction_angle;	//�ְ�ӵ����� �����ϴ� ���� �̵��� ���� ���
				else if(Dir_of_rotation == direction_2) //�ݽð����
					accel_angle = DISTANCE - angle_distance + lower_correction_angle;	//�ְ�ӵ����� �����ϴ� ���� �̵��� ���� ���
				cal_accel_angle_f = 0;
			}
			if(velocity_mode == CONSTANT_MODE)	//��Ӹ��
			{	
				if(exercise_mode == 1)
				   _Ble_Data.mode =  0x00;//�Ϲ� ��ӿ
				pid_val = PID_control_sys(target_speed, speed_RPM);
				Motor_PWM = (unsigned int)set_motor_pwm(pid_val);	//���� PWM PID ����
				if(angle_distance <= accel_angle)	//�������� �޼�
				{
					deaccel_flag = 1;		 		//���� flag  ON
				}
			}
			else if(velocity_mode == ACCEL_MODE)  //���Ӹ��
			{			
				if(exercise_mode == 1)
				   _Ble_Data.mode = 3;//�Ϲݰ���				
				if(angle_distance <= accel_angle)	//�������� �޼�
					deaccel_flag = 1;		 		//���� flag  ON
				else
				{
					if((Current_angle >= set_min_angle) && (Current_angle < set_min_angle+(ROM*0.25)))	//�� �����(ROM)�� ���� 25% ����
					{
						target_speed = set_rpm;	//�����ӵ� ����
						_LoRa_Data.mode = 0;//Para.velocity_mode = 0; ����ü �򰥸� �� ���� mode�� ����. 24.04.08 ����
						_SENSOR_DATA.velocity_mode = 0;//��� 24.04.08 �߰��Ͽ���.
					}
					else if( (Current_angle >= set_min_angle+(ROM*0.25)) && (Current_angle < set_max_angle-(ROM*0.25)) ) //�� �����(ROM)�� ���� 25%, ���� 25% ���� ����
					{	
						target_speed = accel_rpm;	//�����ӵ� + 2 �ܰ� => ���ӿ����
						_LoRa_Data.mode = 1;//Para.velocity_mode = 1; ����ü �򰥸� �� ���� mode�� ����. 24.04.08 ����
						_SENSOR_DATA.velocity_mode = 1;//���� 24.04.08 �߰��Ͽ���.
					}
					else if( (Current_angle >= set_max_angle-(ROM*0.25)) && (Current_angle <= set_max_angle) )	//�� �����(ROM)�� ���� 25% ����
					{
						target_speed = set_rpm;	//�����ӵ� ����
						_LoRa_Data.mode = 0;//Para.velocity_mode = 0; ����ü �򰥸� �� ���� mode�� ����. 24.04.08 ����
						_SENSOR_DATA.velocity_mode = 0;//��� 24.04.08 �߰��Ͽ���.
					}
					pid_val = PID_control_sys(target_speed, speed_RPM);
					Motor_PWM = (unsigned int)set_motor_pwm(pid_val);	//���� PWM PID ����
				}
			}
		}
	}
	else//deaccel_flag == 1
	{
		if(speed_step == 1 || speed_step == 2)
		{
			if((pwm_step -= 2) > deaccel_time)	//step�� 0������ ������ 65535 �Ǵ°� ����, ���� step�� ������ ���ӽð����� ���� ��� ���� pwm_step 2������
				pwm_step = 0;	
		}
		else if(--pwm_step > deaccel_time)	//step�� 0������ ������ 65535 �Ǵ°� ����, ���� step�� ������ ���ӽð����� ���� ��� ���� pwm_step 1������
			pwm_step = 0;		
		target_speed = ((set_rpm - 240)/(float)deaccel_time)*pwm_step + 240; 	 //Soft Stop[soft stop�� �ּҼӵ��� 240(duty=15%)]
		pid_val = PID_control_sys(target_speed, speed_RPM);
		Motor_PWM = (unsigned int)set_motor_pwm(pid_val);	//���� PWM PID ����
	}
}

//������� ���� ����
float High_angle = 0, Lower_angle = 0, High_angle_correction = 0, Lower_angle_correction = 0;
byte Measurement_mode = 0, Measure_up = 0, Measure_down = 0, Measurement_CNT = 0;//Measure_up =1?
float Bf_value = 0, BfBf_value = 0, Bf_avr = 0, Now_avr = 0;
float Rate = 0, Rate_2 = 0;
extern byte Cal_rate_of_change_f;
//��ȭ�� ��� �Լ�
float Rate_of_change(float value)
{
	float rate_of_change = 0;
	/*
	if(Bf_value != 0 && BfBf_value != 0)	//0���� ������ ���� �����ϱ� ����(NaN ���� ����), ó�� �λ���Ŭ�� ���Ű��� �������Ű��� �����ϱ����� ��հ� �� ������ ��ŵ(23.05.19)
	{
		//rate_of_change = (value - Bf_value)/Bf_value * 100;	//�������� ���簪�� ��ȭ�� ���
		Now_avr = (value + Bf_value)/2;	
		Bf_avr = (Bf_value + BfBf_value)/2;	
		if(Bf_avr != 0)
			rate_of_change = (Now_avr - Bf_avr)/Now_avr * 100;	//������հ��� ������հ��� ��ȭ�� ���
	}

	BfBf_value = Bf_value;		//�������Ű��� ���Ű��� ����
	
	if(Current_actual >= 0.1)	//0.1A�̻��� ���� ���Ű� ����		(�ε弿�� �����߿��� ����23.06.23)
		Bf_value = value;		//���簪�� ���Ű��� ����
	*/
	if(Bf_value != 0)	//0���� ������ ���� �����ϱ� ����(NaN ���� ����), ó�� �λ���Ŭ�� ���Ű��� �������Ű��� �����ϱ����� ��հ� �� ������ ��ŵ(23.05.19)
	{
		rate_of_change = fabs(value - Bf_value)/Bf_value * 100;	//�������� ���簪�� ��ȭ�� ��� 24.05.02 ���� �Լ��� �߰��Ͽ� ���Ѱ��̴� ���Ѱ��̴� ���� ������ �Ǹ� �ݴ�������� ȸ���ϵ��� ������
	}
	if(Bf_value != value)
		Bf_value = value;	
	return rate_of_change;
}

float Current_2 = 0,avr_total_high_angle = 0, avr_total_low_angle = 0;
byte start_over_angle = 1, over_angle = 0, Detect_ok = 1, Over_angle_flag = 0;
void angle_adjustment_PI(float target_angle, byte speed_step, byte velocity_mode)
{
	static byte direction_1, direction_2;
	//�����ⱸ ���� �� ���� ���� �׽�Ʈ ����(23.02.02)
	//�߰��Ѱ�
	if(_USER_RUN_SETTING.motion == Knee)//����
	{
		direction_1 = CCW;//1
		direction_2 = CW;//0
	}
	else//���, �Ȳ�ġ, �ո�, �߸�
	{
		direction_1 = CW;//0
		direction_2 = CCW;//1
	}

	if(Dir_of_rotation == direction_1)	//�ð����(���, �Ȳ�ġ ����)
	{
		angle_distance = target_angle - Current_angle; //���� ������ ���� ������ ��(�ǽð�, 10ms���� ���)
		Soft_start_stop_PI(acceltime, deacceltime, speed_step, velocity_mode);
		//int direction = (_USER_RUN_SETTING.motion == Knee) ? -1 : 1;		
		//if(direction * target_angle <= direction * Current_angle)
		
		if(target_angle <= Current_angle)
		{
			Change_dir = 0;
			Cal_rate_of_change_f = 0;	
			if(_USER_RUN_SETTING.calibration == 0)//Ķ���극�̼� ����� ��쿡�� ���� ������ Ǯ������ �̷��� �ڵ带 �߰��Ͽ���.
			{
				if(Measurement_mode && start_over_angle)
				{
					start_over_angle = 0;					
					over_angle = 1;	//���� �ʰ��� �޼��ϸ� ���ʵ����� �ٽ� 1�� ������ ���ϰ� ���� �������� ��������� �Ǵ� ������ �����Ͽ���.	
					over_angle__start_flag = 1;
				}
				else//4,5,6,7 ���������� ���
				{
					if((exercise_mode <= 7) && (exercise_mode >= 4))//�Ϲ�, ����, ����
					{
						PWM_SLEEP;
					}
					over_angle = 1;		
					_VOICE_DATA.x06_1B = 0x00;
				}
			}
		}
		if(over_angle)//���簢���� ���Ѱ��� �Ѿ ��
		{
			if(Measurement_mode)
				Over_angle_flag = 1;
			over_angle = 0;
			Motor_PWM = 0;				//���� ������ �Ǹ� ����
			pwm_step = 0;					
			deaccel_flag = 0;
			move_to_lower = 0;			//�Ϲݿ ó�� ����� ���Ѱ����� ���� �̵��� move_to_lower = 0
			
			if((exercise_mode == 1) || (exercise_mode == 3) || (exercise_mode == 2))	//�Ϲ�, ����, ���� �
				timer2_ms_cnt_start = 1;	//��ǥ ������ �Ǹ� ���� ī���� up ����
			else if((exercise_mode >= 4) && (exercise_mode <= 8)) 		//�������, �������
			{
				if(exercise_mode != 4)
					Dir_of_rotation = !direction_1;
				exercise_mode = 0;			
				_LoRa_Data.state = 1;//� ���� 24.04.08 �߰� �Ͽ���.
				_SENSOR_DATA.state = 1;	//� ����
				//_LoRa_Data.use_state = 0x08;
				Passive_ing = 0;//������������ �̵�����
				_USER_RUN_SETTING.exerc_start = 0x00;
		
				//////////////////////////////////////////////////////
				Record_data_f = 1;	//���ڴ� �������� �׽�Ʈ(�� ���� ���� flag)
				OFFSET_angle_2 = P_M_actual;
				
				//////////////�����////////////////
				//timer2_ms_cnt_start = 1;	//���ڴ� ���������׽�Ʈ��(�׽�Ʈ������ ��!!! �����!!!!!) ������������ ������� ������ ����!!!
				//////////////////////////////
				//������� �׽�Ʈ(������ ��Ͽ�)
				Record_F = 0;	//��ǥ�� ���޽� ��� �ߴ�
			}
			/*
			if(Measurement_mode)
			{
				//���Ѱ� �����߿� �ִ밢���� �������� �� ���� ���� ����
				if(Measure_up)
					Measure_up = 0;
				Dir_of_rotation = !direction_1;	//�̵� ���� ��ȯ
				High_angle = Current_angle;	//���� ������ ����
				_LoRa_Data.upper_limit_angle = (byte)High_angle;	//���� ������ ����(PC�� ���� ������)
				_SENSOR_DATA.upper_limit_angle = (byte)High_angle;	//���� ������ ����(LCD���忡 ���� ������)
				Motor_PWM = 0;				//���ϰ� �����Ǹ� ����
				pwm_step = 0;					
				deaccel_flag = 0;
				Measurement_CNT++;	//���� Ƚ�� ����
				timer2_ms_cnt_start = 1;	//���Ѱ� �����Ϸ� ��, ���� ī���� up ����
			}
			*/
		}	
	}
	else if(Dir_of_rotation == direction_2)	//�ݽð����(���, �Ȳ�ġ ����)
	{
		angle_distance = Current_angle - target_angle; //���� ������ ���� ������ ��(�ǽð�, 10ms���� ���)
		Soft_start_stop_PI(acceltime, deacceltime, speed_step, velocity_mode);
		//int direction = (_USER_RUN_SETTING.motion == Knee) ? -1 : 1;		
		//if(direction * target_angle >= direction * Current_angle)
		if(target_angle /*+ 0.5*/ >= Current_angle)//������� lcd���� 19���� ��ϵǴ� ���� ����
		{
			Change_dir = 0;
			Cal_rate_of_change_f = 0;
			if(_USER_RUN_SETTING.calibration == 0)//Ķ���극�̼� ����� ��쿡�� ���� ������ Ǯ������ �̷��� �ڵ带 �߰��Ͽ���.
			{	
				if(Measurement_mode && start_over_angle)
				{
					start_over_angle = 0;
					over_angle = 1;	//���� �ʰ��� �޼��ϸ� ���ʵ����� �ٽ� 1�� ������ ���ϰ� ���� �������� ��������� �Ǵ� ������ �����Ͽ���.	
					over_angle__start_flag = 1;
				}
				else//4,5,6,7 ���������� ���
				{
					if((exercise_mode <= 7) && (exercise_mode >= 4))//�Ϲ�, ����, ����
					{
						PWM_SLEEP;
					}
					over_angle = 1;		
					_VOICE_DATA.x06_1B = 0x00;
				}
			}
		}
		if(over_angle)//���簢���� ���Ѱ��� �Ѿ ��
		{
			if(Measurement_mode)
				Over_angle_flag = 1;
			over_angle = 0;
			Motor_PWM = 0;				//���� ������ �Ǹ� ����
			pwm_step = 0;
			deaccel_flag = 0;
			move_to_lower = 0;			//�Ϲݿ ó�� ����� ���Ѱ����� ���� �̵��� move_to_lower = 0
			
			if((exercise_mode == 1) || (exercise_mode == 3) || (exercise_mode == 2))//�Ϲ�, ����, ���� �
				timer2_ms_cnt_start = 1;	//��ǥ ������ �Ǹ� ���� ī���� up ����
			else if((exercise_mode >= 4) && (exercise_mode <= 8))//�������, �������
			{
				if(exercise_mode != 4)
					Dir_of_rotation = !direction_2;
				exercise_mode = 0;			
				_LoRa_Data.state = 1;		//� ����
				_SENSOR_DATA.state = 1;		//� ����
				Passive_ing = 0;	//������������ �̵� ����
				_USER_RUN_SETTING.exerc_start = 0x00;
				Record_data_f = 2;	//���ڴ� �������� �׽�Ʈ(�� ���� ���� flag)
				OFFSET_angle_2 = P_M_actual;
				//������� �׽�Ʈ(������ ��Ͽ�)
				Record_F = 0;	//��ǥ�� ���޽� ��� �ߴ�
			}
			/*
			if(Measurement_mode)
			{	
				//���Ѱ� �����߿� �ּҰ����� �������� �� ���� ���� ����
				if(Measure_down)
					Measure_down = 0;
				Dir_of_rotation = !direction_2;	//�̵� ���� ��ȯ
				Motor_PWM = 0;				//���ϰ� �����Ǹ� ����
				pwm_step = 0;					
				deaccel_flag = 0;
				Measurement_CNT++;//���� Ƚ�� ����
				timer2_ms_cnt_start = 1;	//���Ѱ� �����Ϸ� ��, ���� ī���� up ����
			}
			*/
		}
	}

	if(Detect_load || Over_angle_flag)	//��������̰�, ���ϰ� �����Ǿ��� ��
	{
		Over_angle_flag = 0;
		Detect_load = 0;		//���ϰ��� 0���� �ʱ�ȭ
		Cal_rate_of_change_f = 0;
		if(!EMS_SW)// ��������� �ƴҶ��� ���ϰ� �����Ǿ��ٴ� ���� ���
		{
			_VOICE_DATA.x06_1B = 0x2A;//���ϰ� �����Ǿ����ϴ� �������	
		}
		Motor_PWM = 0;				//���ϰ� �����Ǹ� ����
		pwm_step = 0;					
		deaccel_flag = 0;	
		Bf_Dir_of_rotation = Dir_of_rotation;
		timer2_ms_cnt_start = 1;	//���Ѱ� �����Ϸ� ��, ���� ī���� up ����, ���Ѱ� �����Ϸ� ��, ���� ī���� up ����	
		if(Measure_up)		//���Ѱ� ��������� ��
		{
			Measure_up = 0;	//���Ѱ� �������� ����
			Dir_of_rotation = !direction_1;	//�̵� ���� ��ȯ
			
			// 24.10.07 ���찡 ��
			if(_USER_RUN_SETTING.motion == 0) // � ������ �Ȳ�ġ�϶� ( ���Ѱ� 0 ~ ���Ѱ� 150 )
			{
				if(Current_angle >= 150.0) // ���Ѱ��� 150 �̻��� ���� �ʰԲ� ����
				{
				  High_angle_correction = 200.0; // ( 0 ~ 230 ) ����� -50 ó��
				  High_angle = High_angle_correction; //���� ������(����) ����
				  total_high_angle += 150.0;
				}
				else
				{
				  High_angle = Current_angle + 50.0;
				  total_high_angle += Current_angle;
				}
			}
			else if(_USER_RUN_SETTING.motion == 1) // � ������ ����϶� ( ���Ѱ� 20 ~ ���Ѱ� 180 )
			{
				if(Current_angle >= 180.0) // ���Ѱ��� 180 �̻��� ���� �ʰԲ� ����
				{
				  High_angle_correction = 230.0; // ( 0 ~ 230 ) ����� -50 ó��
				  High_angle = High_angle_correction; //���� ������(����) ����
				  total_high_angle += 180.0;
				}
				else
				{
				  High_angle = Current_angle + 50.0;
				  total_high_angle += Current_angle;
				}
			}
			else if(_USER_RUN_SETTING.motion == 2) // � ������ �����϶� ( ���Ѱ� -10 ~ ���Ѱ� 140 )
			{
				if(Current_angle >= 140.0) // ���Ѱ��� 140 �̻��� ���� �ʰԲ� ����
				{
				  High_angle_correction = 190.0; // ( 0 ~ 230 ) ����� -50 ó��
				  High_angle = High_angle_correction; //���� ������(����) ����
				  total_high_angle += 140.0;
				}
				else
				{
				  High_angle = Current_angle + 50.0;
				  total_high_angle += Current_angle;
				}
			}
			else if(_USER_RUN_SETTING.motion == 3) // � ������ �ո��϶� ( ���Ѱ� -70 ~ ���Ѱ� 70 )
			{
				if(Current_angle >= 70.0) // ���Ѱ��� 70 �̻��� ���� �ʰԲ� ����
				{
				  High_angle_correction = 120.0; // ( 0 ~ 230 ) ����� -50 ó��
				  High_angle = High_angle_correction; //���� ������(����) ����
				  total_high_angle += 70.0;
				}
				else
				{
				  High_angle = Current_angle + 50.0;
				  total_high_angle += Current_angle;
				}
			}
			else if(_USER_RUN_SETTING.motion == 4) // � ������ �߸��϶� ( ���Ѱ� -50 ~ ���Ѱ� 50 )
			{
				if(Current_angle >= 50.0) // ���Ѱ��� 50 �̻��� ���� �ʰԲ� ����
				{
				  High_angle_correction = 100.0; // ( 0 ~ 230 ) ����� -50 ó��
				  High_angle = High_angle_correction; //���� ������(����) ����
				  total_high_angle += 50.0;
				}
				else
				{
				  High_angle = Current_angle + 50.0;
				  total_high_angle += Current_angle;
				}
			}
			_SENSOR_DATA.upper_limit_angle = (byte)High_angle;//���� ������ ����(LCD���忡 ���� ������)
			if(_SENSOR_DATA.upper_limit_angle == 0)
			{
			  _SENSOR_DATA.upper_limit_angle = 1;
			}
			else if(_SENSOR_DATA.lower_limit_angle == 0x32)
			{
			  _SENSOR_DATA.lower_limit_angle = 0x33;
			}
		}
		else if(Measure_down)	//���Ѱ� ��������� ��
		{
			Measure_down = 0;	//���Ѱ� �������� ����
			Dir_of_rotation = !direction_2;	//�̵� ���� ��ȯ
			
			if(_USER_RUN_SETTING.motion == 0) // � ������ �Ȳ�ġ�϶� ( ���Ѱ� 0 ~ ���Ѱ� 150 )
			{
				if(Current_angle <= 0) // ���Ѱ��� 0 ���ϰ� ���� �ʰԲ� ����
				{
				  Lower_angle_correction = 70.0; // ( 0 ~ 230 ) ����� -70 ó��
				  Lower_angle = Lower_angle_correction; //���� ������(����) ����
				  total_low_angle += 0;
				}
				else
				{
				  Lower_angle = Current_angle + 70.0;
				  total_low_angle += Current_angle;
				}
			}
			else if(_USER_RUN_SETTING.motion == 1) // � ������ ����϶� ( ���Ѱ� 20 ~ ���Ѱ� 180 )
			{
				if(Current_angle <= 20.0) // ���Ѱ��� 20 ���ϰ� ���� �ʰԲ� ����
				{
				  Lower_angle_correction = 90.0; // ( 0 ~ 230 ) ����� -70 ó��
				  Lower_angle = Lower_angle_correction; //���� ������(����) ����
				  total_low_angle += 20.0;
				}
				else
				{
				  Lower_angle = Current_angle + 70.0;
				  total_low_angle += Current_angle;
				}
			}
			else if(_USER_RUN_SETTING.motion == 2) // � ������ �����϶� ( ���Ѱ� -10 ~ ���Ѱ� 140 )
			{
				if(Current_angle <= -10.0) // ���Ѱ��� -10 ���� ���� �ʰԲ� ����
				{
				  Lower_angle_correction = 60.0; // ( 0 ~ 230 ) ����� -70 ó��
				  Lower_angle = Lower_angle_correction; //���� ������(����) ����
				  total_low_angle += -10.0;
				}
				else
				{
				  Lower_angle = Current_angle + 70.0;
				  total_low_angle += Current_angle;
				}
			}
			else if(_USER_RUN_SETTING.motion == 3) // � ������ �ո��϶� ( ���Ѱ� -70 ~ ���Ѱ� 70 )
			{
				if(Current_angle <= -70.0) // ���Ѱ��� -70 ���ϰ� ���� �ʰԲ� ����
				{
				  Lower_angle_correction = 0; // ( 0 ~ 230 ) ����� -70 ó��
				  Lower_angle = Lower_angle_correction; //���� ������(����) ����
				  total_low_angle += -70.0;
				}
				else
				{
				  Lower_angle = Current_angle + 70.0;
				  total_low_angle += Current_angle;
				}
			}
			else if(_USER_RUN_SETTING.motion == 4) // � ������ �߸��϶� ( ���Ѱ� -50 ~ ���Ѱ� 50 )
			{
				if(Current_angle <= -50.0) // ���Ѱ��� -50 ���ϰ� ���� �ʰԲ� ����
				{
				  Lower_angle_correction = 20.0; // ( 0 ~ 230 ) ����� -70 ó��
				  Lower_angle = Lower_angle_correction; //���� ������(����) ����
				  total_low_angle += -50.0;
				}
				else
				{
				  Lower_angle = Current_angle + 70.0;
				  total_low_angle += Current_angle;
				}
			}
			_SENSOR_DATA.lower_limit_angle = (byte)Lower_angle;//���� ������ ����(LCD���忡 ���� ������)
			if(_SENSOR_DATA.lower_limit_angle == 0)
			{
			  _SENSOR_DATA.lower_limit_angle = 1;
			}
			else if(_SENSOR_DATA.lower_limit_angle == 0x46)
			{
			  _SENSOR_DATA.lower_limit_angle = 0x47;
			}
		}
		if(Dir_of_rotation != Bf_Dir_of_rotation) //������ ��ȯ �Ǹ�
		{
			Change_dir = 1;		//3�� �ڿ� Cal_rate_of_change_f = 1����	
		}
		Measurement_CNT++;//���� Ƚ�� ����
		Rate_2 = Rate;
		Current_2 = Current_actual;
	}	
	if(Measurement_CNT >= 6)
	{	
		Measurement_CNT = 0;		//���� Ƚ�� �ʱ�ȭ		
		set_min_angle = total_low_angle/3;
		
		avr_total_high_angle = total_high_angle/3;
		avr_total_low_angle = total_low_angle/3;
		// 24.10.07 ���찡 ��
		if(_USER_RUN_SETTING.motion == 0) // � ������ �Ȳ�ġ�϶� ( ���Ѱ� 0 ~ ���Ѱ� 150 )
		{
			if(avr_total_high_angle >= 150.0) // ���Ѱ��� 150 �̻��� ���� �ʰԲ� ����
			{
			  avr_total_high_angle = 200.0; // ( 0 ~ 230 ) ����� -50 ó��
			}
			else
			{
			  avr_total_high_angle = avr_total_high_angle + 50.0;
			}
		}
		else if(_USER_RUN_SETTING.motion == 1) // � ������ ����϶� ( ���Ѱ� 20 ~ ���Ѱ� 180 )
		{
			if(avr_total_high_angle >= 180.0) // ���Ѱ��� 180 �̻��� ���� �ʰԲ� ����
			{
			  avr_total_high_angle = 230.0; // ( 0 ~ 230 ) ����� -50 ó��
			}
			else
			{
			  avr_total_high_angle = avr_total_high_angle + 50.0;
			}
		}
		else if(_USER_RUN_SETTING.motion == 2) // � ������ �����϶� ( ���Ѱ� -10 ~ ���Ѱ� 140 )
		{
			if(avr_total_high_angle >= 140.0) // ���Ѱ��� 140 �̻��� ���� �ʰԲ� ����
			{
			  avr_total_high_angle = 190.0; // ( 0 ~ 230 ) ����� -50 ó��
			}
			else
			{
			  avr_total_high_angle = avr_total_high_angle + 50.0;
			}
		}
		else if(_USER_RUN_SETTING.motion == 3) // � ������ �ո��϶� ( ���Ѱ� -70 ~ ���Ѱ� 70 )
		{
			if(avr_total_high_angle >= 70.0) // ���Ѱ��� 70 �̻��� ���� �ʰԲ� ����
			{
			  avr_total_high_angle = 120.0; // ( 0 ~ 230 ) ����� -50 ó��
			}
			else
			{
			  avr_total_high_angle = avr_total_high_angle + 50.0;
			}
		}
		else if(_USER_RUN_SETTING.motion == 4) // � ������ �߸��϶� ( ���Ѱ� -50 ~ ���Ѱ� 50 )
		{
			if(avr_total_high_angle >= 50.0) // ���Ѱ��� 50 �̻��� ���� �ʰԲ� ����
			{
			  avr_total_high_angle = 100.0; // ( 0 ~ 230 ) ����� -50 ó��
			}
			else
			{
			  avr_total_high_angle = avr_total_high_angle + 50.0;
			}
		}
		
		if(_USER_RUN_SETTING.motion == 0) // � ������ �Ȳ�ġ�϶� ( ���Ѱ� 0 ~ ���Ѱ� 150 )
		{
			if(avr_total_low_angle <= 0) // ���Ѱ��� 0 ���ϰ� ���� �ʰԲ� ����
			{
			  avr_total_low_angle = 70.0; // ( 0 ~ 230 ) ����� -70 ó��
			}
			else
			{
			  avr_total_low_angle = avr_total_low_angle + 70.0;
			}
		}
		else if(_USER_RUN_SETTING.motion == 1) // � ������ ����϶� ( ���Ѱ� 20 ~ ���Ѱ� 180 )
		{
			if(avr_total_low_angle <= 20.0) // ���Ѱ��� 20 ���ϰ� ���� �ʰԲ� ����
			{
			  avr_total_low_angle = 90.0; // ( 0 ~ 230 ) ����� -70 ó��
			}
			else
			{
			  avr_total_low_angle = avr_total_low_angle + 70.0;
			}
		}
		else if(_USER_RUN_SETTING.motion == 2) // � ������ �����϶� ( ���Ѱ� -10 ~ ���Ѱ� 140 )
		{
			if(avr_total_low_angle <= -10.0) // ���Ѱ��� 140 �̻��� ���� �ʰԲ� ����
			{
			  avr_total_low_angle = 60.0; // ( 0 ~ 230 ) ����� -70 ó��
			}
			else
			{
			  avr_total_low_angle = avr_total_low_angle + 70.0;
			}
		}
		else if(_USER_RUN_SETTING.motion == 3) // � ������ �ո��϶� ( ���Ѱ� -70 ~ ���Ѱ� 70 )
		{
			if(avr_total_low_angle <= -70.0) // ���Ѱ��� -70 ���ϰ� ���� �ʰԲ� ����
			{
			  avr_total_low_angle = 0; // ( 0 ~ 230 ) ����� -70 ó��
			}
			else
			{
			  avr_total_low_angle = avr_total_low_angle + 70.0;
			}
		}
		else if(_USER_RUN_SETTING.motion == 4) // � ������ �߸��϶� ( ���Ѱ� -50 ~ ���Ѱ� 50 )
		{
			if(avr_total_low_angle <= -50.0) // ���Ѱ��� -50 ���ϰ� ���� �ʰԲ� ����
			{
			  avr_total_low_angle = 20.0; // ( 0 ~ 230 ) ����� -70 ó��
			}
			else
			{
			  avr_total_low_angle = avr_total_low_angle + 70.0;
			}
		}
		
		total_low_angle = 0, total_high_angle = 0; //��� �� �ʱ�ȭ ��Ŵ
		/*_VOICE_DATA.x06_1B = 0x15; //������ �Ϸ� �Ǿ����ϴ� ���� ���
		_LoRa_Data.use_state = 0x08;//�����
		//_SENSOR_DATA.state = 1;		//� ����(LCD�� ���� ������)
		//_SENSOR_DATA.state = 1;		//� ����(LCD���忡 ���� ������)
		//_USER_RUN_SETTING.exerc_start = 0x0A;//6ȸ ����
		timer2_ms_cnt_start = 0;	//���� ī���� flag ����
		_Ble_Data.exer_state = 2; // ���÷� ���� �Ϸ� ��ȣ�� ���� // �߰��Ѱ�
		Not_Send_Ble = 1; // ��� ������ ����Ǿ������� ��������� ���� �����͸� ������ �ʵ��� ��( ��� ���� ��� ��ȣ�� ���� ).*/
		timer2_ms_cnt_start = 0;	//���� ī���� flag ����
		exerc_end = 1;//+10
		mode_flag = 10;	
		Move_to_lower = 1;
		Measurement_mode = 0;
	}
}
//����, ���� ���� �ð� counter
byte motor_delay_ms(dword delay_time)
{
	if(timer2_ms_cnt < delay_time)
		return 0;
	else
	{
		timer2_ms_cnt_start = 0;
		return 1;
	}
}
byte Time_mode_end = 0;
//��ð� ����� ��ð� ������ �Լ�
byte motor_exerc_time(word exerc_time_min)
{
	if(t2_min_cnt < exerc_time_min)
		return 1;
	else if(t2_min_cnt >= exerc_time_min)
	{
		Time_mode_end = 1;		//��ð� �������� �˷��ֱ� ���� flag(22.10.31 �Ƚ����尡 ������ ��ð���带 �������� �� �ð�ī������ ����� ���� �ʴ� ���� ����)
		exerc_time_start = 0;	//1ms timer counting end
		return 2;
	}
	return 0;
}
word exerc_cnt = 0;	//�Ƚ��(�պ�)
byte Reach_high_angle_flag = 0; //���Ѱ��� �����ؾ߸� ī��Ʈ �ǰԲ� �����Ͽ���. 2024.09.25
//�Ϲݿ ���					 ���� ����					���� ����					�ӵ����			   �ӵ��ܰ�			�Ƚ��			��ð�(min)			  ���� �����ð�				���� �����ð�	
void Normal_exercise(float lower_limit_angle, float upper_limit_angle, byte velocity_mode, byte speed_step, byte num_of_exerc, word exerc_time, dword lower_limit_time, dword upper_limit_time)
{
	ROM = upper_limit_angle - lower_limit_angle; 	//�� �����(ROM)
	static byte operating;
	//operating = num_of_exerc ? num_of_exerc : motor_exerc_time(exerc_time);
	if((num_of_exerc > 0) && (exerc_time == 0))			//�Ƚ�� ����� ���
	{
		operating = num_of_exerc;	
	}
	else if((exerc_time > 0) && (num_of_exerc == 0))	//��ð� ����� ���
	{
		operating = motor_exerc_time(exerc_time);	
	}
	static byte direction_1, direction_2;
	//���� ������ ���� ���� ����
	if(_USER_RUN_SETTING.motion == Knee)	//����
	{
		direction_1 = CCW;
		direction_2 = CW;
	}
	else	//���, �Ȳ�ġ, �ո�, �߸�
	{
		direction_1 = CW;
		direction_2 = CCW;
	}
	if(operating)
	{
		static byte bf = 0;
		if(bf != Dir_of_rotation)    //Dir_of_rotation�� �ٲ���� �� �ѹ��� �����
		{
			if(Dir_of_rotation == direction_1)		
			{
				DISTANCE = upper_limit_angle - Current_angle;	//� ������ �ٲ�� �̵��� �Ÿ�(����) ���
			}
			else if(Dir_of_rotation == direction_2)
			{
				DISTANCE = Current_angle - lower_limit_angle;	//� ������ �ٲ�� �̵��� �Ÿ�(����) ���
			}
			bf = Dir_of_rotation;
		}
		
		if(Dir_of_rotation == direction_1)		//�ð����, ���Ѱ����� �̵�
		{	
 			angle_adjustment_PI(upper_limit_angle, speed_step, velocity_mode);
			if(motor_delay_ms(upper_limit_time))	//������ ���������ð��� �Ǹ� 1
			{
				if(upper_limit_angle <= Current_angle)		//���� ������ ���� ���� �̻��� ��
				{
					Dir_of_rotation = direction_2;	//�ݽð�������� ��ȯ
					toggle_cnt++;
				}
			}
		}
		else if(Dir_of_rotation == direction_2)	//�ݽð����, ���Ѱ����� �̵�
		{
			angle_adjustment_PI(lower_limit_angle, speed_step, velocity_mode);
			if(motor_delay_ms(lower_limit_time))	//������ ���������ð��� �Ǹ� 1
			{
				if(lower_limit_angle >= Current_angle)		//���� ������ ���� ���� ������ ��
				{				
					Dir_of_rotation = direction_1;			//�ð�������� ��ȯ
					toggle_cnt++;
				}
			}
		}
		
		
		if((toggle_cnt >= 2) && (num_of_exerc > 0) && (exerc_time == 0))	//�Ƚ�� ����϶��� ī���� (23.01.16)//�ѹ� �պ��Ҷ� ����//���Ѱ��� �����ؾ߸� ī��Ʈ �ǰԲ� �����Ͽ���. 2024.09.25
		{
			toggle_cnt = 0;
			exerc_cnt++;	//�Ƚ�� ī����
			num_of_workouts++;	//�Ƚ�� ī����(��ſ�)
		}
			
		
		if((num_of_exerc > 0) && (exerc_cnt >= operating))		//�Ƚ�� ������
		{
			exercise_mode = 0;	//�Ϲݿ ��� ����
			deaccel_flag = 0;	
			pwm_step = 0;
			exerc_cnt = 0;
			exerc_end = 1;		//� ���� ��, ���Ѱ����� �̵��� ���� flag
			mode_flag = 10;		//� ���� ��, ���Ѱ� �̵��� DISTANCE�� ����� ���� flag
			
		}
		else if((exerc_time > 0) && (Time_mode_end == 1)) //��ð� ������   //(exerc_time_start == 0)) //(motor_exerc_time(exerc_time) == 2))	
		{
			mode_flag = 10;		//� ���� ��, ���Ѱ� �̵��� DISTANCE�� ����� ���� flag
			if(Dir_of_rotation == direction_1)
			{
				deaccel_flag = 1;	//��ð� ������ ���ӽ���
				if((upper_limit_angle <= Current_angle) || (pwm_step <= 0))	//���ӽ��� ��, ���簢���� ���Ѱ� �̻��� �ǰų� ������ ������ �Ϲݿ��� ����
				{
					exercise_mode = 0;
					deaccel_flag = 0;
					pwm_step = 0;
					exerc_cnt = 0;
					exerc_end = 1;		//� ���� ��, ���Ѱ����� �̵��� ���� flag
					Time_mode_end = 0;
				}
			}
			else if(Dir_of_rotation == direction_2)
			{
				deaccel_flag = 1;	//��ð� ������ ���ӽ���
				if((lower_limit_angle >= Current_angle) || (pwm_step <= 0)) //���ӽ��� ��, ���簢���� ���Ѱ� ���ϰ� �ǰų� ������ ������ �Ϲݿ��� ����
				{
					exercise_mode = 0;
					deaccel_flag = 0;
					pwm_step = 0;
					exerc_cnt = 0;
					exerc_end = 1;		//� ���� ��, ���Ѱ����� �̵��� ���� flag
					Time_mode_end = 0;
				}
			}
		}
	}
}
byte Move_to_lower = 1;
//�Ϲݿ��� ���� ��, ���Ѱ� �̵� �� ���Ѱ�+10������ ����
void exerc_stop_mode(float target_angle, byte speed_step, byte Dir)
{
	static int rpm;
	switch(speed_step)		//�ӵ��ܰ迡 ���� ���� ���,���� rpm ����(7�ܰ���� ��������)
	{						//DC���Ϳ� �ⱸ���� ���� = 1500:1
		case 1:
			rpm = 460;	//�ⱸ�� RPM = 0.277  
			break;
		case 2:
			rpm = 665; //�ⱸ�� RPM = 0.37  
			break;
		case 3:
			rpm = 830; //�ⱸ�� RPM = 0.463 
			break;
		case 4:
			rpm = 1015; //�ⱸ�� RPM = 0.556 
			break;
		case 5:
			rpm = 1200; //�ⱸ�� RPM = 0.639 
			break;
		case 6:
			rpm = 1340; //�ⱸ�� RPM = 0.789 
			break;
		case 7:
			rpm = 1460; //�ⱸ�� RPM = 0.833 
			break;
		case 8:
			rpm = 1615; //�ⱸ�� RPM = 0.917 
			break;
		case 9:
			rpm = 1800; //�ⱸ�� RPM = 1.000 
			break;			
		default:
			break;
	}
	static byte cal_accel_angle;
	if(Move_to_lower == 1)
		angle_distance = Current_angle - target_angle; //���� ������ ���� ������ ��(�ǽð�)
	else if(Move_to_lower == 0)
		angle_distance = target_angle - Current_angle; //���� ����+10�� ���� ������ ��(�ǽð�)
	if(!deaccel_flag)
	{
		/*
		if((speed_step >= 1) && (target_speed < rpm))
		{
			target_speed = ((rpm-720)/(float)acceltime)*pwm_step + 720;	//Soft Start
			pid_val = PID_control_sys(target_speed, speed_RPM);
			Motor_PWM = (unsigned int)set_motor_pwm(pid_val);	//���� PWM PID ����
			
			
			static int target_save[300];
			static float pid_save[300]; 
			static dword pwm_save[300];
			static word save_index = 0, save_en = 0;
			
			if(save_en)
			{
				target_save[save_index] = target_speed;
				pid_save[save_index] = pid_val;
				pwm_save[save_index] = Motor_PWM;
				
				if(save_index++ >= 299)
					save_index = 0;
			}
		}
		*/
		if((speed_step >= 1) && (target_speed < rpm))
		{
			static word test_kcnt = 0;
			test_kcnt++;
		}
		target_speed = ((rpm-240)/(float)acceltime)*pwm_step + 240;	//Soft Start 240���� 480���� �����Ͽ���.
		pid_val = PID_control_sys(target_speed, speed_RPM);
		Motor_PWM = (unsigned int)set_motor_pwm(pid_val);	//���� PWM PID ����
		if(pwm_step < acceltime)
		{
			pwm_step++;
			if(pwm_step == acceltime-1)
			{
				cal_accel_angle = 1;
			}
		}
		
		static byte direction_1, direction_2;
		//�����ⱸ ���� �� ���� ���� �׽�Ʈ ����(23.02.02)
		if(_USER_RUN_SETTING.motion == Knee)	//����
		{
			direction_1 = CCW;
			direction_2 = CW;
		}
		else	//���, ����
		{
			direction_1 = CW;
			direction_2 = CCW;
		}
		
		if(target_speed >= rpm) //�����ӵ��� �� ��
		{
			if(cal_accel_angle == 1)
			{
				if(Dir == direction_1)	  //�ð����
					accel_angle = DISTANCE - angle_distance + upper_correction_angle;	//�ְ�ӵ����� �����ϴ� ���� �̵��� ���� ���
				else if(Dir == direction_2) //�ݽð����
					accel_angle = DISTANCE - angle_distance + lower_correction_angle;	//�ְ�ӵ����� �����ϴ� ���� �̵��� ���� ���
				
				cal_accel_angle = 0;
			}
			if(angle_distance <= accel_angle)	//�������� �޼� 
				deaccel_flag = 1;	//���� flag  ON
		}
		
	}
	else	//deaccel_flag == 1
	{
		if(--pwm_step > deacceltime)	//step�� 0������ ������ 65535 �Ǵ°� ����
			pwm_step = 0;
		target_speed = ((rpm-300)/(float)deacceltime)*pwm_step + 300;		//Soft Stop  //Soft Start 240
		pid_val = PID_control_sys(target_speed, speed_RPM);
		Motor_PWM = (unsigned int)set_motor_pwm(pid_val);	//���� PWM PID ����
	}
	

	if((Move_to_lower == 1) && (target_angle >= Current_angle))		//���Ѱ� ���޽�
	{
		Motor_PWM = 0;				//���� ������ �Ǹ� ����
		pwm_step = 0;
		deaccel_flag = 0;
		mode_flag = 10;
		//���Ѱ� ���� ��, 200ms ���� �Ͻ�����(delay time�� ������ ���� ����̹��� ������ ��) => �����, ���� ǥ�� ������ Ǯ���� ���װ� �־ ����(23.02.07)
		//timer2_ms_cnt_start = 1;
		//if(motor_delay_ms(200))
		Move_to_lower = !Move_to_lower;		//Move_to_lower : 1 => 0
	}
	else if((Move_to_lower == 0) && (target_angle <= Current_angle))
	{
		Motor_PWM = 0;//���� ������ �Ǹ� ����
		pwm_step = 0;
		deaccel_flag = 0;
		exerc_end = 0;
		Move_to_lower = !Move_to_lower;//Move_to_lower : 0 => 1
		//Dir_of_rotation = STOP;//��� ������ ������ STOP���� �ʱ�ȭ
		toggle_cnt = 0;
		num_of_workouts = 0;
		_LoRa_Data.state = 1;//�����(PC���忡 ���� ������)
		_SENSOR_DATA.state = 1;//�����(LCD���忡 ���� ������)
		voice_en = 1;
		
		if(exercise_mode == 0x09)
		{
		  	exercise_mode = 0;
		  	_VOICE_DATA.x06_1B = 0x15; //������ �Ϸ� �Ǿ����ϴ� ���� ���
			_LoRa_Data.use_state = 0x81;//�����
			_Ble_Data.exer_state = 2; // ���÷� ���� �Ϸ� ��ȣ�� ���� // �߰��Ѱ�
			if((byte)avr_total_high_angle == 0)
			{
			  avr_total_high_angle = 1;
			}
			else if((byte)avr_total_high_angle == 0x32)
			{
			  avr_total_high_angle = 0x33;
			}
			
			if((byte)avr_total_low_angle == 0)
			{
			  avr_total_low_angle = 1;
			}
			else if((byte)avr_total_low_angle == 0x46)
			{
			  avr_total_low_angle = 0x47;
			}
			_Ble_Data.lower_limit_angle = (byte)avr_total_low_angle;	//���� ������ ����(���ÿ� ���� ������)//��հ����� �����ؾ���
			_LoRa_Data.lower_limit_angle = (byte)avr_total_low_angle;	//���� ������ ����(�ζ󺸵忡 ���� ������)//��հ����� �����ؾ���		
			_Ble_Data.upper_limit_angle = (byte)avr_total_high_angle;	//���� ������ ����(���ÿ� ���� ������)//��հ����� �����ؾ���
			_LoRa_Data.upper_limit_angle = (byte)avr_total_high_angle;	//���� ������ ����(�ζ󺸵忡 ���� ������)//��հ����� �����ؾ���	
			Not_Send_Ble = 1; // ��� ������ ����Ǿ������� ��������� ���� �����͸� ������ �ʵ��� ��( ��� ���� ��� ��ȣ�� ���� ).
		}
		else
		{
			_VOICE_DATA.x06_1B = 0x28;  //"��� �����մϴ�" ���� ���	
		}
		
		if(Not_Send_Ble != 1) // ����� ��ư���� ��� ������������ � ����� ���� ����
		{
			_Ble_Data.exer_state = 1; // �߰��Ѱ�
		}
		else
		{
		  Not_Send_Ble = 0;
		}
		
		gen_ada_con_back = 0;
		PWM_SLEEP;
	}
}
byte angle_step = 0;
//����� ���						 ���� ���� 				  ���� ����			��������		 �ݺ�Ƚ��			 ������ġ	 		�ӵ��ܰ�		       ���� �����ð�				���� �����ð�
void Adaptive_exercise(float lower_limit_angle, float upper_limit_angle, float PA, byte repeat_num, byte location, byte speed_step, dword lower_limit_time, dword upper_limit_time)
{
	static float lower_initial_angle, upper_initial_angle;
	static word exerc_num;
	_LoRa_Data.mode = 1;	//�����
	_Ble_Data.mode = 1;		//�����
	//�����ʱⰢ��, �����ʱⰢ�� ���
	lower_initial_angle = lower_limit_angle + PA;
	upper_initial_angle	= upper_limit_angle - PA;
	
	//�Ƚ�� ���
	exerc_num = (word)(PA+1)*repeat_num;	//�Ƚ�� = (��������+1)*�ݺ�Ƚ��
	
	static byte direction_1, direction_2;
	//�����ⱸ ���� �� ���� ���� �׽�Ʈ ����(23.02.02)
	if(_USER_RUN_SETTING.motion == Knee)	//����
	{
		direction_1 = CCW;
		direction_2 = CW;
	}
	else	//���, ����
	{
		direction_1 = CW;
		direction_2 = CCW;
	}
	
	
	if(exerc_num)	//������� �Ƚ���� ����
	{
		static byte bf = 0;
		if(bf != Dir_of_rotation)		//Dir_of_rotation�� �ٲ���� �� �ѹ��� �����
		{
			switch(location)	//������ġ�� ���� DISTANCE�� ���
			{
				case UPPER:	//���� ����
					if(Dir_of_rotation == direction_1)		
						DISTANCE = (upper_initial_angle + angle_step) - Current_angle;	//� ������ �ٲ�� �̵��� �Ÿ�(����) ���
					else if(Dir_of_rotation == direction_2)
						DISTANCE = Current_angle - lower_limit_angle;					//� ������ �ٲ�� �̵��� �Ÿ�(����) ���
					break;
				case LOWER:	//���� ����
					if(Dir_of_rotation == direction_1)		
						DISTANCE = upper_limit_angle - Current_angle;					//� ������ �ٲ�� �̵��� �Ÿ�(����) ���
					else if(Dir_of_rotation == direction_2)
						DISTANCE = Current_angle - (lower_initial_angle - angle_step);	//� ������ �ٲ�� �̵��� �Ÿ�(����) ���
					break;
				case UP_LOW: //������ ����
					if(Dir_of_rotation == direction_1)		
						DISTANCE = (upper_initial_angle + angle_step) - Current_angle;	//� ������ �ٲ�� �̵��� �Ÿ�(����) ���
					else if(Dir_of_rotation == direction_2)
						DISTANCE = Current_angle - (lower_initial_angle - angle_step);	//� ������ �ٲ�� �̵��� �Ÿ�(����) ���
					break;
				default:
					break;
			}
			bf = Dir_of_rotation;
		}			
		static float target_angle;
		if(Dir_of_rotation == direction_1)		//�ð����, ���Ѱ����� �̵�
		{
			//������ġ�� ���� ��ǥ ���� ����
			if(location == UPPER)			//������ġ : ���Ѱ�
				target_angle = upper_initial_angle + angle_step;
			else if(location == LOWER)		//������ġ : ���Ѱ�
				target_angle = upper_limit_angle;
			else if(location == UP_LOW)		//������ġ : �����Ѱ�
				target_angle = upper_initial_angle + angle_step;
			angle_adjustment_PI(target_angle, speed_step, CONSTANT_MODE);	//������� ��Ӹ�常 ���� 
			if(motor_delay_ms(upper_limit_time))	//������ ���������ð��� �Ǹ� 1
			{
				if(target_angle <= Current_angle)
				{
					Dir_of_rotation = direction_2;	//�ݽð�������� ��ȯ
					toggle_cnt++;	
				}
			}
		}
		else if(Dir_of_rotation == direction_2)	//�ݽð����, ���Ѱ����� �̵�
		{
			//������ġ�� ���� ��ǥ ���� ����
			if(location == UPPER)			//������ġ : ���Ѱ�
				target_angle = lower_limit_angle;
			else if(location == LOWER)		//������ġ : ���Ѱ�
				target_angle = lower_initial_angle - angle_step;
			else if(location == UP_LOW)		//������ġ : �����Ѱ�
				target_angle = lower_initial_angle - angle_step;
			angle_adjustment_PI(target_angle, speed_step, CONSTANT_MODE);		//������� ��Ӹ�常 ����  
			if(motor_delay_ms(lower_limit_time))	//������ ���������ð��� �Ǹ� 1
			{
				if(target_angle >= Current_angle)		//���� ������ ���� ���� ������ ��
				{
					Dir_of_rotation = direction_1;			//�ð�������� ��ȯ
					toggle_cnt++;
				}
			}
		}
		
		if(toggle_cnt >= 2)	
		{
			toggle_cnt = 0;
			num_of_workouts++;
			exerc_cnt++;		//1ȸ �պ��Ҷ����� exer_cnt 1�� ����
			if(exerc_cnt >= repeat_num)
			{
				exerc_cnt = 0;	//22.12.20. exerc_cnt�� 0���� �ʱ�ȭ ���� �ʾƼ� angle_step�� �ݺ�Ƚ���� ������� �պ��ҋ����� �����ϴ� ���� ����
				angle_step += 1; 
			}
		}
							//22.12.20. ���׼���
		//if((exerc_num > 0) && (exerc_cnt >= exerc_num))	//�Ƚ�� ������
		if((exerc_num > 0) && (num_of_workouts >= exerc_num))	//�Ƚ�� ������
		{
			exercise_mode = 0;	//���߿ ��� ����
			deaccel_flag = 0;	
			pwm_step = 0;
			exerc_end = 1;		//� ���� ��, ���Ѱ����� �̵��� ���� flag
			mode_flag = 10;		//� ���� ��, ���Ѱ� �̵��� DISTANCE�� ����� ���� flag
			exerc_cnt = 0;
		}
	}
}

//���߿ �������(23.01.17)
//������ġ�� ���Ѱ� �Ǵ� ���Ѱ��� ��� �Ѱ����� ��� �ݺ���ϴ� ���� �ƴ�, ���߿�� �ݺ�Ƚ����ŭ ��ä���, �ݴ밢(���� �Ǵ� ����)�� �ѹ� ��� ���ƿͼ� �ٽ� �ݺ����(�̶� �Ƚ�� 1ī��Ʈ)

byte repeat_ok = 0, fake_cnt = 0;	
byte Focus_f = 0;	//����ǥ�� ������ ���� flag(0:���� �Ǵ� �������� �̵���, 1:��������, 2:��������)
//���߿ ���						 ���� ���� 				  ���� ����					���߰���				   �ݺ�Ƚ��			������ġ	 		�ӵ��ܰ�			�Ƚ��				��ð�		      ���� �����ð�				���� �����ð�
void Intensive_exercise(float lower_limit_angle, float upper_limit_angle, float concentration_angle, byte repeat_num, byte location, byte speed_step, byte num_of_exerc, word exerc_time, dword lower_limit_time, dword upper_limit_time)
{
	static float lower_conc_anlge, upper_conc_angle;
	static byte operating;
	_LoRa_Data.mode = 2;	//���߿
	_Ble_Data.mode = 2;		//���߿	
	//�������߰���, �������߰��� ���, 	22.12.19. �������� 1��
	if(speed_step >= 6)	//6�ܰ� �̻��� ���� ��������
	{
		lower_conc_anlge = (lower_limit_angle) + (concentration_angle);	
		upper_conc_angle = (upper_limit_angle) - (concentration_angle);
	}
	else	//if(speed_step < 6)
	{
		lower_conc_anlge = lower_limit_angle + concentration_angle;	
		upper_conc_angle = upper_limit_angle - concentration_angle;
	}
	if((num_of_exerc > 0) && (exerc_time == 0))			//�Ƚ�� ����� ���
		operating = num_of_exerc;	
	else if((exerc_time > 0) && (num_of_exerc == 0))	//��ð� ����� ���
		operating = motor_exerc_time(exerc_time);	
	
	
	static byte direction_1, direction_2;
	//�����ⱸ ���� �� ���� ���� �׽�Ʈ ����(23.02.02)
	if(_USER_RUN_SETTING.motion == Knee)	//����
	{
		direction_1 = CCW;
		direction_2 = CW;
	}
	else	//���, ����
	{
		direction_1 = CW;
		direction_2 = CCW;
	}
	if(operating)
	{
		static byte bf = 2;
		if(bf != Dir_of_rotation)	//Dir_of_rotation�� �ٲ���� �� �ѹ��� �����
		{	//������ġ�� ���� DISTANCE�� ���
			switch(location)
			{
				case UPPER:	//���� ����
					if(Dir_of_rotation == direction_1)		
						DISTANCE = upper_limit_angle - Current_angle;	//� ������ �ٲ�� �̵��� �Ÿ�(����) ���
					else if(Dir_of_rotation == direction_2)
					{
						if(repeat_ok)//���Ѱ����� �ݺ�Ƚ����ŭ ���߿�� �ϰ�, ���Ѱ����� �̵�
							DISTANCE = Current_angle - lower_limit_angle;	
						else
							DISTANCE = Current_angle - upper_conc_angle;	
					}
					break;
				case LOWER:	//���� ����
					if(Dir_of_rotation == direction_1)		
					{
						if(repeat_ok)		//���Ѱ����� �ݺ�Ƚ����ŭ ���߿�� �ϰ�, ���Ѱ����� �̵�
						{
							DISTANCE = upper_limit_angle - Current_angle;	
							//Focus_f = 0;	//�������� �̵���
						}
						else
						{
							DISTANCE = lower_conc_anlge - Current_angle;
							//Focus_f = 1;	//��������
						}
					}
					else if(Dir_of_rotation == direction_2)
					{
						DISTANCE = Current_angle - lower_limit_angle;	//� ������ �ٲ�� �̵��� �Ÿ�(����) ���
						//Focus_f = 1;	//��������
					}
					break;
				case UP_LOW: //������ ����
					if((exerc_cnt+1)%2 == 1)	
					{
						if(Dir_of_rotation == direction_1)		
							DISTANCE = upper_limit_angle - Current_angle;	//� ������ �ٲ�� �̵��� �Ÿ�(����) ���
						else if(Dir_of_rotation == direction_2)
							DISTANCE = Current_angle - upper_conc_angle;	//� ������ �ٲ�� �̵��� �Ÿ�(����) ���
					}
					else if((exerc_cnt+1)%2 == 0)	
					{
						if(Dir_of_rotation == direction_1)		
							DISTANCE = lower_conc_anlge - Current_angle;	//� ������ �ٲ�� �̵��� �Ÿ�(����) ���
						else if(Dir_of_rotation == direction_2)
							DISTANCE = Current_angle - lower_limit_angle;	//� ������ �ٲ�� �̵��� �Ÿ�(����) ���
					}
					break;
				default:
					break;
			}
			bf = Dir_of_rotation;
		}
		
		if((location == UP_LOW) && (DISTANCE > (upper_limit_angle-upper_conc_angle)*1.7))
			except_f = 1;	//�������� ����(������ġ ���Ѱ�), 23.02.14(�����Ѱ� ��� ���������� ����)
		
		
		static float target_angle;
		if(Dir_of_rotation == direction_1)		//�ð����
		{
			//������ġ�� ���� ��ǥ ���� ����
			if(location == UPPER)			//������ġ : ���Ѱ�
			{
				target_angle = upper_limit_angle;
				Focus_f = 1;	//���Ѱ��̵�
			}
			else if(location == LOWER)		//������ġ : ���Ѱ�
			{
				if(repeat_ok)		//���Ѱ����� �ݺ�Ƚ����ŭ ���߿�� �ϰ�, ���Ѱ����� �̵�
				{
					target_angle = upper_limit_angle;
					Focus_f = 1;	//���Ѱ��̵�
				}
				else
				{
					target_angle = lower_conc_anlge;
					Focus_f = 2;	//���� ���߰� �̵�
				}
			}
			else if(location == UP_LOW)		//������ġ : �����Ѱ�
			{
				if((exerc_cnt+1)%2 == 1)		//���� ����
				{
					target_angle = upper_limit_angle;
					Focus_f = 1;	//���Ѱ��̵�
				}
				else if((exerc_cnt+1)%2 == 0)	//���� ����
				{
					target_angle = lower_conc_anlge;
					Focus_f = 2;	//���� ���߰� �̵�
				}
			}
			
			angle_adjustment_PI(target_angle, speed_step, CONSTANT_MODE);	//���߿�� ��Ӹ�常 ���� 
			if(motor_delay_ms(upper_limit_time))	//������ ���������ð��� �Ǹ� 1
			{
				if(target_angle <= Current_angle)
				{
					Dir_of_rotation = direction_2;	//�ݽð�������� ��ȯ
					if((location != LOWER) && (exerc_time > 0))	//������ġ�� ���� �Ǵ� ������ �϶�
					{
						if((DISTANCE > 0) && (DISTANCE <= (upper_limit_angle-upper_conc_angle)*1.7))	//���ʿ��� ī������ ���ֱ� ����
							toggle_cnt++;
						else if(exerc_time_start == 0)
						{
							toggle_cnt++;
							if(exerc_cnt == 0 && toggle_cnt == 1)
							{
								toggle_cnt = 0;
								exerc_time_start = 1;
							}
						}			
					}										//���߰��� 5������ ī���õ����ʾƼ� ������ 1.2�迡�� 1.7��� �ø�(23.01.17)
					else if((DISTANCE > 0) && (DISTANCE <= (upper_limit_angle-upper_conc_angle)*1.7))	//���ʿ��� ī������ ���ֱ� ����
						toggle_cnt++;														//ex) �������߿ ������ ���Ѱ� �̵��� ī���� ���� ����
					
					if(repeat_ok)
					{
						if(++fake_cnt >= 2)
						{
							repeat_ok = 0;
							fake_cnt = 0;
						}
					}
				}
				
			}
		}
		else if(Dir_of_rotation == direction_2)	//�ݽð����
		{
			//������ġ�� ���� ��ǥ ���� ����
			if(location == UPPER)			//������ġ : ���Ѱ�
			{
				if(repeat_ok)		//���Ѱ����� �ݺ�Ƚ����ŭ ���߿�� �ϰ�, ���Ѱ����� �̵�
				{
					target_angle = lower_limit_angle;
					Focus_f = 0;	//���Ѱ��̵�
				}
				else
				{
					target_angle = upper_conc_angle;
					Focus_f = 3;	//���� ���߰� �̵�
				}
			}
			else if(location == LOWER)		//������ġ : ���Ѱ�
			{
				target_angle = lower_limit_angle;
				Focus_f = 0;	//���Ѱ��̵�
			}
			else if(location == UP_LOW)		//������ġ : �����Ѱ�
			{
				if((exerc_cnt+1)%2 == 1)		//���� ����
				{
					target_angle = upper_conc_angle;
					Focus_f = 3;	//���� ���߰� �̵�
				}
				else if((exerc_cnt+1)%2 == 0)	//���� ����
				{
					target_angle = lower_limit_angle;
					Focus_f = 0;	//���Ѱ��̵�
				}
			}
			
			angle_adjustment_PI(target_angle, speed_step, CONSTANT_MODE);	//���߿�� ��Ӹ�常 ����
			if(motor_delay_ms(lower_limit_time))	//������ ���������ð��� �Ǹ� 1 
			{
				if(target_angle >= Current_angle)
				{
					Dir_of_rotation = direction_1;	//�ð�������� ��ȯ
					if((location == LOWER) && (exerc_time > 0))		//���Ѱ�, ��ð� ����� ���
					{
						if((DISTANCE > 0) && (DISTANCE <= (lower_conc_anlge-lower_limit_angle)*1.7) && (DISTANCE >= (lower_conc_anlge-lower_limit_angle)*0.7))	//���ʿ��� ī������ ���ֱ� ����
							toggle_cnt++;	
						else if(exerc_time_start == 0)
						{
							toggle_cnt++;
							if(exerc_cnt == 0 && toggle_cnt == 1)
							{
								toggle_cnt = 0;
								exerc_time_start = 1;
							}
						}
					}																
					else if((DISTANCE > 0) && (DISTANCE <= (lower_conc_anlge-lower_limit_angle)*1.7) && (DISTANCE >= (lower_conc_anlge-lower_limit_angle)*0.7))	//���ʿ��� ī������ ���ֱ� ����
						toggle_cnt++;																										//ex) �������߿ ������ ���Ѱ� �̵��� ī���� ���� ����
					
					if(repeat_ok)
					{
						if(++fake_cnt >= 2)
						{
							repeat_ok = 0;
							fake_cnt = 0;
						}
					}
				}		
			}
		}
		if(toggle_cnt >= repeat_num*2)	
		{
			toggle_cnt = 0;
			exerc_cnt++;	//���Ѱ� �Ǵ� ���Ѱ����� �ݺ�Ƚ���� ��� ä�ﶧ ���� exerc_cnt 1�� ����(���Ƚ��)
			if(location != UP_LOW)	//������ġ�� ���Ѱ� �Ǵ� ���Ѱ��� ���
				repeat_ok = 1;		//������ġ�� ���� �Ǵ� ���Ѱ��� ���, �ݺ�Ƚ���� ��ä��� ���� �Ǵ� ���Ѱ����� �̵��ϱ� ���� flag(DISTANCE�� ���) 23.01.17			
			if(location == UP_LOW)	//������ġ�� �����Ѱ�
			{
				if(exerc_cnt%2 == 0)
					num_of_workouts++;	//LCD����� ���� ���� �Ƚ�� ������
			}
			else
				num_of_workouts++;
		}
		

		static word i;
		i = (location == UP_LOW) ? operating*2 : operating;
		if((num_of_exerc > 0) && exerc_cnt >= i)	//�Ƚ�� ������
		{
			exercise_mode = 0;	//���߿ ��� ����
			deaccel_flag = 0;	
			pwm_step = 0;
			exerc_end = 1;		//� ���� ��, ���Ѱ����� �̵��� ���� flag
			mode_flag = 10;		//� ���� ��, ���Ѱ� �̵��� DISTANCE�� ����� ���� flag
			exerc_cnt = 0;
			
			repeat_ok = 0;
			fake_cnt = 0;
		}
		else if((exerc_time > 0) && (Time_mode_end == 1)) //��ð� ������ 
		{
			_SENSOR_DATA.state = 2;	//������(���Ѱ��̵���)
			mode_flag = 10;		//� ���� ��, ���Ѱ� �̵��� DISTANCE�� ����� ���� flag
			if(Dir_of_rotation == direction_1)
			{
				deaccel_flag = 1;	//�����Ѱ� ���߿ ����� ���, ����(�Ǵ� ����)���߿��� ����(�Ǵ� ����)�������� �̵��ϰ� ������ ��ð��� ������ �Ǹ� ���ӽ���
				if((upper_limit_angle <= Current_angle) || (pwm_step <= 0))	//���ӽ��� ��, ���簢���� ���Ѱ� �̻��� �ǰų� ������ ������ ���߿��� ����
				{
					exercise_mode = 0;
					deaccel_flag = 0;
					pwm_step = 0;
					exerc_end = 1;		//� ���� ��, ���Ѱ����� �̵��� ���� flag
					Time_mode_end = 0;
					exerc_cnt = 0;
					repeat_ok = 0;
					fake_cnt = 0;
				}
			}
			else if(Dir_of_rotation == direction_2)
			{
				deaccel_flag = 1;	//�����Ѱ� ���߿ ����� ���, ����(�Ǵ� ����)���߿��� ����(�Ǵ� ����)�������� �̵��ϰ� ������ ��ð��� ������ �Ǹ� ���ӽ���
				if((lower_limit_angle >= Current_angle) || (pwm_step <= 0)) //���ӽ��� ��, ���簢���� ���Ѱ� ���ϰ� �ǰų� ������ ������ ���߿��� ����
				{
					exercise_mode = 0;
					deaccel_flag = 0;
					pwm_step = 0;
					exerc_end = 1;		//� ���� ��, ���Ѱ����� �̵��� ���� flag
					Time_mode_end = 0;
					exerc_cnt = 0;
					repeat_ok = 0;
					fake_cnt = 0;
				}
			}
		}
	}
}


/*******************���� ���� ���*******************/
float up_corr = 0.2, down_corr = 0.2;
float total_high_angle= 0, total_low_angle= 0;
byte toggle_cnt_exception = 0, init_skip_flag = 0;
void Exercise_ctrl()//10ms task
{//LCD ��Ʈ�ѷ� ���� �κ�(_USER_RUN_SETTING ����ü�� ������ ���)	
	if(_USER_RUN_SETTING.exerc_start == 0x01 && (_SENSOR_DATA.state == 1 || _SENSOR_DATA.state == 4))//LCD������ ���� ����� ��ȣ�� ���� ��
	{//����� or �Ͻ����� �Ϸ� ���¿����� ������ ����           //�����                 //�Ͻ����� �Ϸ�
		PWM_NOT_SLEEP;
		if(_SENSOR_DATA.state != 4)
			init_skip_flag = 0;
		if(exercise_mode == 0x09)	//������� �Ͻ����� ���¿��� ��� �����Ҷ�
		{
			voice_en = 1;
			exercise_mode = 0x00; 		//0���� �ʱ�ȭ
			Measurement_CNT = 0;		//���� Ƚ�� �ʱ�ȭ
			_LoRa_Data.state = 1;		//� ����(PC���忡 ���� ������)
			_SENSOR_DATA.state = 1;		//� ����(LCD���忡 ���� ������)
			timer2_ms_cnt_start = 0;	//���� ī���� flag ����
		}
		if(exercise_mode == 0) //�Ϲݿ
		{
			min_cnt = 0x00;		//� ���۽� �����ð� ����� ���� min_cnt�� 0���� �ʱ�ȭ
			num_of_workouts = 0x00;	//� ���۽� ���� � Ƚ�� ����� ���� num_of_workouts�� 0���� �ʱ�ȭ]
			_USER_RUN_SETTING.exerc_start = 0x00;
			_LoRa_Data.state = 0;		//���(PC���忡 ���� ������)
			_SENSOR_DATA.state = 0;		//���(LCD���忡 ���� ������)
		}
		//���Ѱ�, ���Ѱ� ����
		set_min_angle = _USER_RUN_SETTING.lower_angle;				//���Ѱ�
		set_max_angle = _USER_RUN_SETTING.upper_angle;				//���Ѱ�
		_Ble_Data.upper_angle = (byte)(set_max_angle + 50.0);//���Ѱ�;
		_Ble_Data.lower_angle = (byte)(set_min_angle + 70.0);//���Ѱ�;
		_LoRa_Data.upper_angle = (byte)(set_max_angle + 50.0);//���Ѱ�;
		_LoRa_Data.lower_angle = (byte)(set_min_angle + 70.0);//���Ѱ�;	
		if(_USER_RUN_SETTING.mode == 1 || _USER_RUN_SETTING.mode == 2)//���� ������ ���
		{
			/*if(bf_lcd_location != set_location)
			{		
				set_location = _USER_RUN_SETTING.application_location;//������ġ
				bf_lcd_location = set_location;	
			}			*/	
			if(Tx_lora_data)
			{
				Tx_lora_data = 0;
				set_location = _LoRa_Data.special_location;//������ġ
				_SENSOR_DATA.special_location = _LoRa_Data.special_location;
			}
			else 
				set_location = _USER_RUN_SETTING.application_location;//������ġ	
		}		
		switch(_USER_RUN_SETTING.mode)
		{
			case 0:	//�Ϲ� � ���	
				if(exercise_mode == 0)	//��� ó���������� ���� ���带 NORMAL�� �����ؾ���. �Ͻ����� ���¿��� �ٽ� �����Ҷ��� �����Ǹ� �ȵ�(22.02.07)
					exercise_mode = NORMAL;
				set_velocity_mode = _USER_RUN_SETTING.speed_mode;			//�ӵ����
				set_speed_step = _USER_RUN_SETTING.speed;					//��ӵ�
				_Ble_Data.velocity_mode = _USER_RUN_SETTING.speed;//� �ӵ� �ܰ�
				_LoRa_Data.velocity_mode = _USER_RUN_SETTING.speed;//� �ӵ� �ܰ�						
				set_lower_limit_time = _USER_RUN_SETTING.lower_stop_time * 1000;	//���������ð�(s->ms ȯ��)
				set_upper_limit_time = _USER_RUN_SETTING.upper_stop_time * 1000;	//���������ð�(s->ms ȯ��)
				_Ble_Data.stop_time = 0x0F & _USER_RUN_SETTING.lower_stop_time;	//���������ð�(s->ms ȯ��) ���� 4bit
				_Ble_Data.stop_time |= 0xF0 & (_USER_RUN_SETTING.upper_stop_time<<4);	//���������ð�(s->ms ȯ��) ���� 4bit
				_LoRa_Data.stop_time = 0x0F & _USER_RUN_SETTING.lower_stop_time;	//���������ð�(s->ms ȯ��) ���� 4bit
				_LoRa_Data.stop_time |= 0xF0 & (_USER_RUN_SETTING.upper_stop_time<<4);	//���������ð�(s->ms ȯ��) ���� 4bit			
				if(_USER_RUN_SETTING.cnt_mode == 0x01)	//�Ƚ�� ����϶�
				{
					set_exerc_num = _USER_RUN_SETTING.exerc_num;			//�Ƚ��
					_Ble_Data.exerc_time_and_exercnum = (0x80 | _USER_RUN_SETTING.exerc_num);//�Ƚ��//MSB : 1
					_LoRa_Data.exerc_time_and_exercnum = (0x80 | _USER_RUN_SETTING.exerc_num);//�Ƚ��//MSB : 1				
					set_exerc_time = 0;
				}
				else if(_USER_RUN_SETTING.cnt_mode == 0x00)	//��ð� ����϶�
				{
					set_exerc_time = _USER_RUN_SETTING.exerc_time;			//��ð�
					_Ble_Data.exerc_time_and_exercnum = (0x7F & _USER_RUN_SETTING.exerc_time);//�Ƚ��//MSB : 0
					_LoRa_Data.exerc_time_and_exercnum = (0x7F & _USER_RUN_SETTING.exerc_time);//�Ƚ��//MSB : 0					
					set_exerc_num = 0;
				}			
				break;
				
			case 1:	//���� � ���				
				if(exercise_mode == 0)	//��� ó���������� ���� ���带 NORMAL�� �����ؾ���. �Ͻ����� ���¿��� �ٽ� �����Ҷ��� �����Ǹ� �ȵ�(22.02.07)
					exercise_mode = ADAPTIVE;
				
				set_PA = _USER_RUN_SETTING.adaptation_angle;				//��������
				_Ble_Data.special_angle =  _USER_RUN_SETTING.adaptation_angle;//��������
				_LoRa_Data.special_angle =  _USER_RUN_SETTING.adaptation_angle;//��������
				set_speed_step = _USER_RUN_SETTING.speed;					//��ӵ�
				_Ble_Data.velocity_mode = _USER_RUN_SETTING.speed;//� �ӵ� �ܰ�
				_LoRa_Data.velocity_mode = _USER_RUN_SETTING.speed;//� �ӵ� �ܰ�				

				set_lower_limit_time = _USER_RUN_SETTING.lower_stop_time * 1000;	//���������ð�(s->ms ȯ��)
				set_upper_limit_time = _USER_RUN_SETTING.upper_stop_time * 1000;	//���������ð�(s->ms ȯ��)
				_Ble_Data.stop_time = 0x0F & _USER_RUN_SETTING.lower_stop_time;	//���������ð�(s->ms ȯ��) ���� 4bit
				_Ble_Data.stop_time |= 0xF0 & (_USER_RUN_SETTING.upper_stop_time<<4);	//���������ð�(s->ms ȯ��) ���� 4bit
				_LoRa_Data.stop_time = 0x0F & _USER_RUN_SETTING.lower_stop_time;	//���������ð�(s->ms ȯ��) ���� 4bit
				_LoRa_Data.stop_time |= 0xF0 & (_USER_RUN_SETTING.upper_stop_time<<4);	//���������ð�(s->ms ȯ��) ���� 4bit						
				set_repeat_num = _USER_RUN_SETTING.repeat_num;				//�ݺ�Ƚ��
				_Ble_Data.repeat_num = _USER_RUN_SETTING.repeat_num;//�ݺ�Ƚ��
				_LoRa_Data.repeat_num = _USER_RUN_SETTING.repeat_num;//�ݺ�Ƚ��
				if(_USER_RUN_SETTING.cnt_mode == 0x01)	//�Ƚ�� ����϶�
				{
					set_exerc_num = _USER_RUN_SETTING.exerc_num;			//�Ƚ��
					_Ble_Data.exerc_time_and_exercnum = (0x80 | _USER_RUN_SETTING.exerc_num);//�Ƚ��//MSB : 1
					_LoRa_Data.exerc_time_and_exercnum = (0x80 | _USER_RUN_SETTING.exerc_num);//�Ƚ��//MSB : 1				
					set_exerc_time = 0;
				}
				else if(_USER_RUN_SETTING.cnt_mode == 0x00)	//��ð� ����϶�
				{
					set_exerc_time = _USER_RUN_SETTING.exerc_time;			//��ð�
					_Ble_Data.exerc_time_and_exercnum = (0x7F & _USER_RUN_SETTING.exerc_time);//�Ƚ��//MSB : 0
					_LoRa_Data.exerc_time_and_exercnum = (0x7F & _USER_RUN_SETTING.exerc_time);//�Ƚ��//MSB : 0					
					set_exerc_num = 0;
				}				
				set_exerc_time = 0;
				break;
				
			case 2:	//���� � ���				
				if(exercise_mode == 0)	//��� ó���������� ���� ���带 NORMAL�� �����ؾ���. �Ͻ����� ���¿��� �ٽ� �����Ҷ��� �����Ǹ� �ȵ�(22.02.07)
					exercise_mode = INTENSIVE;
				
				set_conc_angle = _USER_RUN_SETTING.concentration_angle;		//���߰���
				_Ble_Data.special_angle =  _USER_RUN_SETTING.concentration_angle;//���߰���
				_LoRa_Data.special_angle =  _USER_RUN_SETTING.concentration_angle;//���߰���
				set_speed_step = _USER_RUN_SETTING.speed;					//��ӵ�
				
				_Ble_Data.velocity_mode = _USER_RUN_SETTING.speed;//� �ӵ� �ܰ�
				_LoRa_Data.velocity_mode = _USER_RUN_SETTING.speed;//� �ӵ� �ܰ�			
				set_lower_limit_time = _USER_RUN_SETTING.lower_stop_time * 1000;	//���������ð�(s->ms ȯ��)
				set_upper_limit_time = _USER_RUN_SETTING.upper_stop_time * 1000;	//���������ð�(s->ms ȯ��)
				_Ble_Data.stop_time = 0x0F & _USER_RUN_SETTING.lower_stop_time;	//���������ð�(s->ms ȯ��) ���� 4bit
				_Ble_Data.stop_time |= 0xF0 & (_USER_RUN_SETTING.upper_stop_time<<4);	//���������ð�(s->ms ȯ��) ���� 4bit
				_LoRa_Data.stop_time = 0x0F & _USER_RUN_SETTING.lower_stop_time;	//���������ð�(s->ms ȯ��) ���� 4bit
				_LoRa_Data.stop_time |= 0xF0 & (_USER_RUN_SETTING.upper_stop_time<<4);	//���������ð�(s->ms ȯ��) ���� 4bit					
				set_repeat_num = _USER_RUN_SETTING.repeat_num;				//�ݺ�Ƚ��
				_Ble_Data.repeat_num = _USER_RUN_SETTING.repeat_num;//�ݺ�Ƚ��
				_LoRa_Data.repeat_num = _USER_RUN_SETTING.repeat_num;//�ݺ�Ƚ��				
				if(_USER_RUN_SETTING.cnt_mode == 0x01)	//�Ƚ�� ����϶�
				{
					set_exerc_num = _USER_RUN_SETTING.exerc_num;			//�Ƚ��
					_Ble_Data.exerc_time_and_exercnum = (0x80 | _USER_RUN_SETTING.exerc_num);//�Ƚ��//MSB : 1
					_LoRa_Data.exerc_time_and_exercnum = (0x80 | _USER_RUN_SETTING.exerc_num);//�Ƚ��//MSB : 1				
					set_exerc_time = 0;
				}
				else if(_USER_RUN_SETTING.cnt_mode == 0x00)	//��ð� ����϶�
				{
					set_exerc_time = _USER_RUN_SETTING.exerc_time;			//��ð�
					_Ble_Data.exerc_time_and_exercnum = (0x7F & _USER_RUN_SETTING.exerc_time);//�Ƚ��//MSB : 0
					_LoRa_Data.exerc_time_and_exercnum = (0x7F & _USER_RUN_SETTING.exerc_time);//�Ƚ��//MSB : 0					
					set_exerc_num = 0;
				}
				break;
			default:
				break;
		}
	}
	else if(_USER_RUN_SETTING.exerc_start == 0x04)	//������� UP(1��)
	{
		PWM_NOT_SLEEP;
		_USER_RUN_SETTING.exerc_start = 0x00;
		exercise_mode = 0x05;		//������� UP(1��)
		_LoRa_Data.use_state = 0x04;
		_LoRa_Data.state = 0; // ���
		_SENSOR_DATA.state = 0; // ���
	}
	else if(_USER_RUN_SETTING.exerc_start == 0x05)	//������� DOWN(1��)
	{
		PWM_NOT_SLEEP;
		_USER_RUN_SETTING.exerc_start = 0x00;
		exercise_mode = 0x06;		//������� DOWN(1��)
		_LoRa_Data.use_state = 0x04;
		_LoRa_Data.state = 0; // ���
		_SENSOR_DATA.state = 0; // ���		
	}
	else if(_USER_RUN_SETTING.exerc_start == 0x06)	//������� Ű�ٿ� UP ing(������ ������ ������ �ӵ���� ��� �ö�)
	{
		PWM_NOT_SLEEP;
		_USER_RUN_SETTING.exerc_start = 0x00;
		exercise_mode = 0x07;		//������� Ű�ٿ� UP
		_LoRa_Data.state = 0; // ���
		_LoRa_Data.use_state = 0x04;
		_SENSOR_DATA.state = 0; // ���		
	}
	else if(_USER_RUN_SETTING.exerc_start == 0x07)	//������� Ű�ٿ� DOWN ing(������ ������ ������ �ӵ���� ��� ������)
	{
		PWM_NOT_SLEEP;
		_USER_RUN_SETTING.exerc_start = 0x00;
		_LoRa_Data.use_state = 0x04;
		exercise_mode = 0x08;		//������� Ű�ٿ� DOWN
		_LoRa_Data.state = 0; // ���
		_SENSOR_DATA.state = 0; // ���		
	}
	else if(_USER_RUN_SETTING.exerc_start == 0x08)	//ROM(Range of Motion) �������
	{
		PWM_NOT_SLEEP;
		_VOICE_DATA.x06_1B = 0x13; //"������ �����մϴ�" �������	
		Measurement_mode = 1;
		_USER_RUN_SETTING.exerc_start = 0x00;
		_LoRa_Data.state = 0; // ���
		_SENSOR_DATA.state = 0; // ���
		exercise_mode = 0x09;	//�������
	}
//	else 
//	{
//		static byte no_action = 0;
//	}

	static byte bf_mode = 0;
	if(bf_mode != exercise_mode)	//exercis_mode�� �ٲ���� �� �ѹ��� �����
	{	//23.01.12 �յ����۽� target_speed = 0 ���� �ʱ�ȭ �߰�
		if(init_skip_flag)	//�Ͻ����� ��, �ٽ� ���۹�ư�� ������ �� �������¸� �ʱ�ȭ���� ����
		{
			init_skip_flag = 0;
		}
		else
		{
			switch(exercise_mode)
			{
				case NORMAL:	//�Ϲݿ
					pwm_step = 0;
					deaccel_flag = 0;
					mode_flag = 1;
					toggle_cnt = 0;
					exerc_cnt = 0;
					exerc_end = 0;
					target_speed = 0;
					break;
				case ADAPTIVE:	//�����
					pwm_step = 0;
					deaccel_flag = 0;
					mode_flag = 2;
					toggle_cnt = 0;
					exerc_cnt = 0;
					exerc_end = 0;
					angle_step = 0;
					target_speed = 0;
					break;
				case INTENSIVE:	//���߿
					pwm_step = 0;
					deaccel_flag = 0;
					mode_flag = 3;
					toggle_cnt = 0;
					exerc_cnt = 0;
					exerc_end = 0;
					target_speed = 0;
					break;
				case ADJUSTMENT: //������ �̵�
					pwm_step = 0;
					deaccel_flag = 0;
					mode_flag = 4;
					target_speed = 0;
					
					cur_index = 0;
					for(byte k = 0; k < 250; k++)
					{
						cur_measurement1[k] = 0;
						cur_measurement2[k] = 0;
					}
					break;
				case 0x05:	//������� UP(1��)
					if(!Passive_ing)	//������������ �̵����� �ƴҶ� 0���� �ʱ�ȭ(23.02.08)
						pwm_step = 0;
					deaccel_flag = 0;
					mode_flag = 5;
					target_speed = 0;
					break;
				case 0x06:	//������� DOWN(1��)
					if(!Passive_ing)	//������������ �̵����� �ƴҶ� 0���� �ʱ�ȭ(23.02.08)
						pwm_step = 0;
					deaccel_flag = 0;
					mode_flag = 6;
					target_speed = 0;
					break;
				case 0x07: //������� Ű�ٿ� UP ING
					pwm_step = 0;
					deaccel_flag = 0;
					mode_flag = 7;
					target_speed = 0;
					break;
				case 0x08: //������� Ű�ٿ� DOWN ING
					pwm_step = 0;
					deaccel_flag = 0;
					mode_flag = 8;
					target_speed = 0;
					break;
				case 0x09:	//�������
					pwm_step = 0;
					deaccel_flag = 0;
					mode_flag = 9;
					target_speed = 0;
					break;
				case 0x10:	//�Ͻ�������
					break;
				case 0x20:	//�Ͻ�������
					break;
				case 0x30:	//�Ͻ�������
					break;
				default:	//� ����
					deaccel_flag = 0;
					Motor_PWM = 0;
					target_speed = 0;
					break;
			}
		}
		bf_mode = exercise_mode;
	}
	
	
	
	//�������� Ű�ٿ�, Soft Stop��
	static word RPM_num;
	switch(_USER_RUN_SETTING.speed)		//�ӵ��ܰ迡 ���� ���� rpm ����(7�ܰ���� ��������)
	{									//DC���Ϳ� �ⱸ���� ���� = 1500:1
		case 1:
			RPM_num = 460;	//�ⱸ�� RPM = 0.277
			break;
		case 2:
			RPM_num = 665; //�ⱸ�� RPM = 0.37
			break;
		case 3:
			RPM_num = 830; //�ⱸ�� RPM = 0.463
			break;
		case 4:
			RPM_num = 1015; //�ⱸ�� RPM = 0.556
			break;
		case 5:
			RPM_num = 1200; //�ⱸ�� RPM = 0.639
			break; 
		case 6:
			RPM_num = 1340; //�ⱸ�� RPM = 0.789
			break;
		case 7:
			RPM_num = 1460;
			break;
		case 8:
			RPM_num = 1615;
			break;
		case 9:
			RPM_num = 1800;
			break;			
		default:
			break;
	}
	static float target_angle_3 = 0.0, target_angle_4 = 0.0, target_angle_5 = 0.0;
	static float correction_angle;
	static byte distance_cal_f = 0;
	switch(exercise_mode)
	{

		case NORMAL:	//�Ϲݿ ���
			if(mode_flag == 1)		//�Ϲ� � ���۽� �ѹ��� ����
			{
				if(set_min_angle > Current_angle) 		//���簢���� ���Ѱ������� ���� ���
				{
					//�����ⱸ ���� �� ���� ���� �׽�Ʈ ����(23.02.02)
					if(_USER_RUN_SETTING.motion == Knee)	//����
						Dir_of_rotation = CCW;				//�ݽð��������
					else	//���, �Ȳ�ġ
						Dir_of_rotation = CW;				//�ð��������
					
					
					//22.12.19. 1�� ��������
					DISTANCE = (set_min_angle-1) - Current_angle;	//���� ������ ���
					//correction_angle = (set_min_angle);
					
					distance_cal_f = 1;		//DISTANCE���� ������ȯ�ÿ��� ����ϵ��� �Ǿ��־ ����۽� ���簢���� ���Ѱ����� ���� ��� DISTANCE�� ����ϱ����� flag(23.01.17)
				}
				else if(set_min_angle <= Current_angle)	//���簢���� ���Ѱ� �̻��� ���
				{
					//�����ⱸ ���� �� ���� ���� �׽�Ʈ ����(23.02.02)
					if(_USER_RUN_SETTING.motion == Knee)	//����
						Dir_of_rotation = CW;				//�ð��������
					else	//���, �Ȳ�ġ, �ո�, �߸�
						Dir_of_rotation = CCW;				//�ݽð��������
					
					
					//22.12.19. 1�� ��������
					DISTANCE = Current_angle - (set_min_angle+1);	//���� ������ ���
					toggle_cnt_exception = 1;	//toggle_cnt ����ó���� ���� flag
				}
				move_to_lower = 1;	//�Ϲݿ ó�� ����� ���Ѱ����� ���� �̵��ϱ� ���� flag
				mode_flag = 0;
			}
			
			static byte bf_move = 1;
			if(bf_move != move_to_lower)	//move_to_lower�� 1���� 0�� �ɶ� �ѹ��� ����
			{
				if((move_to_lower == 0) && (set_exerc_num == 0))	//�Ƚ�� ��尡 �ƴҶ�
					exerc_time_start = 1;	//��ð� ī���� ���� flag ON
				bf_move = move_to_lower;		//���� <- ����
			}
			if(move_to_lower)	//ó�� �Ϲݿ ����� ���Ѱ����� ���� �̵�
			{					//1�� ��������, ���Ѱ� �̵��ϴ� �ӵ��� �����ϴ� ��ġ
				if(set_speed_step < 8)//1,2,3,4,5,6,7
					angle_adjustment_PI(set_min_angle, set_speed_step, CONSTANT_MODE);
				else if(set_speed_step >= 8)	//8,9�ܰ� �̻��� ���� �ӵ� 7�ܰ�� ����
					angle_adjustment_PI(set_min_angle, 7, CONSTANT_MODE);				
			}
			else if((set_min_angle < set_max_angle) && (move_to_lower == 0))//���Ѱ����� �Ϲݿ ����
			{	
				if(gen_ada_con_back == 0)
				{
					PWM_NOT_SLEEP;
					gen_ada_con_back = 1;
					_VOICE_DATA.x06_1B = 0x07;  //"��� �����մϴ�" �������	
					voice_en = 1;
				}
				if(distance_cal_f)
				{
					distance_cal_f = 0;
					DISTANCE = set_max_angle - Current_angle;	//� ������ �ٲ�� �̵��� �Ÿ�(����) ���
				}
					//22.12.19. 1�� ��������
				if(set_speed_step >= 6)	//6�ܰ� �̻��� ���� ��������(���Ѱ�, ���Ѱ� 1�� ����)
					Normal_exercise(set_min_angle, set_max_angle, set_velocity_mode, set_speed_step, set_exerc_num, set_exerc_time, set_lower_limit_time, set_upper_limit_time);				
				else if(set_speed_step < 6)	//6�ܰ� �̸��� ���, ���Ѱ� 0.5�� ����
					Normal_exercise(set_min_angle, set_max_angle, set_velocity_mode, set_speed_step, set_exerc_num, set_exerc_time, set_lower_limit_time, set_upper_limit_time);
			}				
			//����ó�� : �Ϲݿ ��� ó�� ����� ���簢������ ���Ѱ��� Ŭ ���, toggle_cnt(Ƚ��)ī������ �ѹ��� �Ǳ� ������ -1 ����
			if(toggle_cnt_exception == 1)
			{
				toggle_cnt = toggle_cnt - 1;	//�Ϲݿ ��忡���� toggle_cnt�� �Ƚ���� �ǹ���
				toggle_cnt_exception = 0;
			}
			break;
		case ADAPTIVE:	//����� ���
			if(mode_flag == 2)		//�Ϲ� � ���۽� �ѹ��� ����
			{
				if(set_min_angle > Current_angle) 		//���簢���� ���Ѱ������� ���� ���
				{
					if(_USER_RUN_SETTING.motion == Knee)	//����
						Dir_of_rotation = CCW;				//�ݽð��������
					else	//���, �Ȳ�ġ
						Dir_of_rotation = CW;				//�ð��������
					
					//22.12.19. 1�� ��������
					DISTANCE = (set_min_angle-1) - Current_angle;	//���� ������ ���
					//correction_angle = (set_min_angle-1);
					
					distance_cal_f = 1;	//DISTANCE���� ������ȯ�ÿ��� ����ϵ��� �Ǿ��־ ����۽� ���簢���� ���Ѱ����� ���� ��� DISTANCE�� ����ϱ����� flag(23.01.17)
				}
				else if(set_min_angle <= Current_angle)	//���簢���� ���Ѱ� �̻��� ���
				{
					if(_USER_RUN_SETTING.motion == Knee)	//����
						Dir_of_rotation = CW;				//�ð��������
					else	//���, �Ȳ�ġ, �ո�, �߸�
						Dir_of_rotation = CCW;		//�ݽð��������
					
					//22.12.19. 1�� ��������
					DISTANCE = Current_angle - (set_min_angle+1);	//���� ������ ���
					//correction_angle = (set_min_angle+1);
					
					toggle_cnt_exception = 1;	//toggle_cnt ����ó���� ���� flag
				}
				move_to_lower = 1;	//�Ϲݿ ó�� ����� ���Ѱ����� ���� �̵��ϱ� ���� flag
				mode_flag = 0;
			}
			
			
			if(move_to_lower)	//ó�� �Ϲݿ ����� �����ʱⰢ��(���Ѱ���+��������)�� �̵�
			{			//22.12.19. 1�� ��������
				if(set_location == UPPER)	//������ġ�� ���Ѱ��϶��� ó���� ���Ѱ����� �̵�
				{//���Ѱ� �̵��ϴ� �ӵ��� �����ϴ� ��ġ
					if(set_speed_step < 8)//1~7�ܰ�
						angle_adjustment_PI(set_min_angle, set_speed_step, CONSTANT_MODE);	
					else if(set_speed_step >= 8)	//8,9�ܰ� �̻��� ���� ��������
						angle_adjustment_PI(set_min_angle, 7, CONSTANT_MODE);					
				}
				else
				{
					if(set_speed_step >= 8)	//8, 9�ܰ� �̻��� ���� ��������
						angle_adjustment_PI(set_min_angle+set_PA ,7 , CONSTANT_MODE);	
					else if(set_speed_step < 8)	//1 ~ 7�ܰ� �̻��� ���� ��������
						angle_adjustment_PI(set_min_angle+set_PA, set_speed_step, CONSTANT_MODE);	
				}
			}
			else if((set_min_angle < set_max_angle) && (move_to_lower == 0))	//���Ѱ����� ����� ����
			{
				if(gen_ada_con_back == 0)
				{
					gen_ada_con_back = 1;
					_VOICE_DATA.x06_1B = 0x07;  //"��� �����մϴ�" �������	
					voice_en = 1;
				}				
				if(distance_cal_f)
				{
					distance_cal_f = 0;
					if(set_location == LOWER)	//������ġ�� ���Ѱ��� ���
						DISTANCE = set_max_angle - Current_angle;	//DISTANCE ���
					else	//������ġ�� ���Ѱ�, �����Ѱ��� ���
						DISTANCE = (set_max_angle - set_PA) - Current_angle;
				}
				//22.12.19. 1�� ��������
				if(set_speed_step >= 6)	//6�ܰ� �̻��� ���� ��������
					Adaptive_exercise(set_min_angle, set_max_angle, set_PA, set_repeat_num, set_location, set_speed_step, set_lower_limit_time, set_upper_limit_time);	//������� ��ð� ���� �Ұ���
				else if(set_speed_step < 6)
					Adaptive_exercise(set_min_angle, set_max_angle, set_PA, set_repeat_num, set_location, set_speed_step, set_lower_limit_time, set_upper_limit_time);
			}
			//����ó�� : ����� ��� ó�� ����� ���簢������ ���Ѱ��� Ŭ ���, toggle_cnt(Ƚ��)ī������ �ѹ��� �Ǳ� ������ -1 ����
			if(toggle_cnt_exception == 1)
			{
				toggle_cnt = toggle_cnt - 1;
				toggle_cnt_exception = 0;
			}
			break;
			
		case INTENSIVE:	//���߿ ���
			if(mode_flag == 3)		//���� � ���۽� �ѹ��� ����
			{
				if(set_min_angle > Current_angle) 		//���簢���� ���Ѱ������� ���� ���
				{
					if(_USER_RUN_SETTING.motion == Knee)	//����
						Dir_of_rotation = CCW;				//�ݽð��������
					else	//���, �Ȳ�ġ, �ո�, �߸�
						Dir_of_rotation = CW;				//�ð��������
					
					//22.12.19. 1�� ��������
					DISTANCE = (set_min_angle-1) - Current_angle;	//���� ������ ���
					//correction_angle = (set_min_angle-1);
					
					distance_cal_f = 1;		//DISTANCE���� ������ȯ�ÿ��� ����ϵ��� �Ǿ��־ ����۽� ���簢���� ���Ѱ����� ���� ��� DISTANCE�� ����ϱ����� flag(23.01.17)
					
					Focus_f = 'U';
				}
				else if(set_min_angle <= Current_angle)	//���簢���� ���Ѱ� �̻��� ���
				{
					if(_USER_RUN_SETTING.motion == Knee)	//����
						Dir_of_rotation = CW;		//�ð��������
					else
						Dir_of_rotation = CCW;		//�ݽð��������
					
					//22.12.19. 1�� ��������
					DISTANCE = Current_angle - (set_min_angle+1);	//���� ������ ���
					//correction_angle = (set_min_angle+1);
					
					distance_cal_f = 1;		//���Ѱ� ���� ��, ���߿ ó�� ���۽�, DISTANCE �� ����� �ȵǴ� ���װ� �־ flag on(23.02.07)
					
					Focus_f = 'D';
				}
				move_to_lower = 1;	//���߿ ó�� ����� ���Ѱ����� ���� �̵��ϱ� ���� flag
				mode_flag = 0;
				if(set_location == LOWER)	//������ġ�� ���Ѱ��� ��� ���Ѱ����� ���� �̵��ϱ� ����
					repeat_ok = 1;
			}
			
			static byte dir_f = 0;
			if(move_to_lower)	//ó�� ���߿ ����� ���Ѱ����� ���� �̵�
			{// 1�� ��������, ���Ѱ� �̵��ϴ� �ӵ��� �����ϴ� ��ġ
				//22.12.19. 1�� ��������
				if(set_speed_step < 8)//1 ~ 7�ܰ�
					angle_adjustment_PI(set_min_angle, set_speed_step, CONSTANT_MODE);	
				else if(set_speed_step >= 8)//8,9�ܰ�
					angle_adjustment_PI(set_min_angle, 7, CONSTANT_MODE);
			
				
				//���Ѱ� ������, ���߿ �����Ҷ� ��ǥ ������ �߸������Ǵ� ���׸� ��������(22.02.06)
				dir_f = 1;
			}
			else if((set_min_angle < set_max_angle) && (move_to_lower == 0))	//���Ѱ����� ���߿ ����
			{
				if(gen_ada_con_back == 0)
				{
					gen_ada_con_back = 1;
					_VOICE_DATA.x06_1B = 0x07;  //"��� �����մϴ�" �������	
					voice_en = 1;
				}				
				if(distance_cal_f)
				{
					distance_cal_f = 0;
					DISTANCE = set_max_angle - Current_angle;	//DISTANCE ���
					if((set_location == LOWER) && (_USER_RUN_SETTING.cnt_mode == 0))
						exerc_time_start = 1;
				}
				
				if(dir_f)
				{
					dir_f = 0;
					//ó���� Target_Angle ���� ���� ���׶����� ���߿ �Լ� ���� ���� ����� ��������(������ �ݽð� �����̰�  //���, �Ȳ�ġ, �ո�, �߸�� �ð� �������� �̸� ����) 22.02.06
					if(_USER_RUN_SETTING.motion == Knee)
						Dir_of_rotation = CCW;
					else
						Dir_of_rotation = CW;
					
					except_f = 1;	//�������� ����(������ġ ���Ѱ�)
				}

				//22.12.19. 1�� ��������
				if(set_speed_step >= 6)	//6�ܰ� �̻��� ���� ��������
					Intensive_exercise(set_min_angle, set_max_angle, set_conc_angle, set_repeat_num, set_location, set_speed_step, set_exerc_num, set_exerc_time, set_lower_limit_time, set_upper_limit_time);
				else if(set_speed_step < 6)
					Intensive_exercise(set_min_angle, set_max_angle, set_conc_angle, set_repeat_num, set_location, set_speed_step, set_exerc_num, set_exerc_time, set_lower_limit_time, set_upper_limit_time);
			}
			break;
		case ADJUSTMENT: //���� ����
			if(mode_flag == 4)	//�ѹ��� ����
			{
				if(set_angle_value > Current_angle)	//���� ������ ���� �������� Ŭ ��
				{	
					if(_USER_RUN_SETTING.motion == Knee)	//����
						Dir_of_rotation = CCW;		//�ݽð��������
					else	//���, �Ȳ�ġ, �ո�, �߸�
						Dir_of_rotation = CW;				//�ð��������
					TIM1->CNT = 0xffff;	//���ڴ� �������� �׽�Ʈ��(���� ������Ҷ� CNT�� 0xffff���� �ʱ�ȭ)
					bf_CNT = 0xffff;	//���ڴ� �������� �׽�Ʈ
					Start_angle_1[Record_Index] = Current_angle;	//���� ���� ����(���ڴ� ���������׽�Ʈ)
					
					DISTANCE = set_angle_value - Current_angle;	//���� ������ ���
				}
				else if(set_angle_value < Current_angle)
				{
					if(_USER_RUN_SETTING.motion == Knee)	//����
						Dir_of_rotation = CW;		//�ð��������
					else	//���, �Ȳ�ġ, �ո�, �߸�
						Dir_of_rotation = CCW;		//�ݽð��������
					
					TIM1->CNT = 0x0000;	//���ڴ� �������� �׽�Ʈ��(���� ������Ҷ� CNT�� 0���� �ʱ�ȭ)
					bf_CNT = 0x0000;	//���ڴ� �������� �׽�Ʈ
					Start_angle_2[Record_Index] = Current_angle;	//���� ���� ����(���ڴ� ���������׽�Ʈ)
					
					DISTANCE = Current_angle - set_angle_value;	//���� ������ ���
				}
				
				mode_flag = 0;
				Under_cnt = 0;	//���ڴ� �������� �׽�Ʈ
				Over_cnt = 0;	//���ڴ� �������� �׽�Ʈ
				
				OFFSET_angle = P_M_actual;	//���ڴ� ���������� �ʿ�
			}
			Record_F = 1;
			
			angle_adjustment_PI(set_angle_value, set_speed_step, set_velocity_mode);
			
			break;
		static float target_angle_1 = 0.0;	
		case 0x05: 	//������� UP(1��)
			if(Passive_ing)		//������������ �̵����� ��
			{
				if (GPIOD->ODR & GPIO_PIN_10) // PD10�� ���°� 1�� �� ������ �ڵ�
				{//���Ͱ� ���ư� �߿��� �������� ����
					_SENSOR_DATA.current_angle = Current_angle;
				}		
				if(--pwm_step > 300)	//step�� 0������ ������ 65535 �Ǵ°� ���� (���ӽð� => 10x100ms)
					pwm_step = 0;
						
				target_speed = ((RPM_num-240)/(float)300)*pwm_step + 240; 	 //Soft Stop[soft stop�� �ּҼӵ��� 240(duty=15%)]
				pid_val = PID_control_sys(target_speed, speed_RPM);
				Motor_PWM = (unsigned int)set_motor_pwm(pid_val);	//���� PWM PID ����
				
				if(pwm_step <= 0)
				{
					pwm_step = 0;
					Motor_PWM = 0;
					exercise_mode = 0;			
					_SENSOR_DATA.state = 1;		//� ���� and �����
					_LoRa_Data.state = 1;		//� ���� and �����
					Passive_ing = 0;	//������������ �̵� ����
				}
			}
			else
			{
				if(mode_flag == 5)	//�ѹ��� ����
				{
					if(_USER_RUN_SETTING.motion == Knee)	//����
					{
						if(Current_angle <= 0)
							target_angle_1 = (signed long)(Current_angle-0.5) - 1;
						else
							target_angle_1 = (signed long)(Current_angle+0.5) - 1;
					}
					else	//���, �Ȳ�ġ, �ո�, �߸�
						target_angle_1 = (dword)(Current_angle+0.5) + 1;
					Dir_of_rotation = CW;	//�ð��������
					DISTANCE = 1.0;			//���� ������ ���
					mode_flag = 0;
					if(_USER_RUN_SETTING.calibration == 0x01)	//Calibration ������ �϶� 
						Motor_Run_F = 1; 
				}
				
				if(_USER_RUN_SETTING.calibration == 0x01)	//Calibration ������ �϶�
				{
					if(Motor_Run_F == 1)
						Motor_PWM = 416;
					else	//500ms�� ����.
					{
						PWM_SLEEP;
						Motor_PWM = 0;
						exercise_mode = 0;
						_SENSOR_DATA.state = 1;
					}
				}
				else	//����
				{
					switch(_USER_RUN_SETTING.motion)
					{
						case Elbow:	//�Ȳ�ġ
							if(Current_angle >= 134.5)	//�Ȳ�ġ ���Ѱ��̻��� ��, ���۾���
							{
								exercise_mode = 0;			
								_LoRa_Data.state = 1;		//� ���� and �����
								_SENSOR_DATA.state = 1;		//� ���� and �����
							}
							else
								angle_adjustment_PI(target_angle_1-up_corr, 1, CONSTANT_MODE);
							break;
						case Shoulder:	//���
							if(Current_angle >= 179.5)	//��� ���Ѱ��̻��� ��, ���۾���
							{
								exercise_mode = 0;			
								_LoRa_Data.state = 1;		//� ���� and �����
								_SENSOR_DATA.state = 1;		//� ���� and �����
							}
							else
								angle_adjustment_PI(target_angle_1-up_corr, 1, CONSTANT_MODE);
							break;
						case Knee:	//����
							if(Current_angle <= (-9.5))	//���� ���Ѱ������� ��, ���۾���
							{
								exercise_mode = 0;			
								_LoRa_Data.state = 1;		//� ���� and �����
								_SENSOR_DATA.state = 1;		//� ���� and �����
							}
							else
								angle_adjustment_PI(target_angle_1/*+up_corr*/, 1, CONSTANT_MODE);
							break;
						case wrist:	//�ո�
							if(Current_angle >= 69.5)	//�ո� ���Ѱ� �̻��� ��, ���۾���
							{
								exercise_mode = 0;			
								_LoRa_Data.state = 1;		//� ���� and �����
								_SENSOR_DATA.state = 1;		//� ���� and �����
							}
							else
								angle_adjustment_PI(target_angle_1-up_corr, 1, CONSTANT_MODE);
							break;
						case ankle:	//�߸�
							if(Current_angle >= 49.5)	//�߸� ���Ѱ� �̻��� ��, ���۾���
							{
								exercise_mode = 0;			
								_LoRa_Data.state = 1;		//� ���� and �����
								_SENSOR_DATA.state = 1;		//� ���� and �����
							}
							else
								angle_adjustment_PI(target_angle_1-up_corr, 1, CONSTANT_MODE);
							break;							
						default:
							break;
					}
				}
			}
			break;
		static float target_angle_2 = 0.0;	
		case 0x06:	//������� DOWN(1��)
			if(Passive_ing)		//������������ �̵����� ��
			{
				_SENSOR_DATA.current_angle = Current_angle;	
				if(--pwm_step > 300)	//step�� 0������ ������ 65535 �Ǵ°� ���� (���ӽð� => 10x100ms)
					pwm_step = 0;	
				target_speed = ((RPM_num-240)/(float)300)*pwm_step + 240; 	 //Soft Stop[soft stop�� �ּҼӵ��� 240(duty=15%)]
				pid_val = PID_control_sys(target_speed, speed_RPM);
				Motor_PWM = (unsigned int)set_motor_pwm(pid_val);	//���� PWM PID ����
				if(pwm_step <= 0)
				{
					pwm_step = 0;
					Motor_PWM = 0;
					exercise_mode = 0;			
					_LoRa_Data.state = 1;		//� ���� and �����
					_SENSOR_DATA.state = 1;		//� ���� and �����
					Passive_ing = 0;	//������������ �̵� ����
				}
			}
			else
			{
				if(mode_flag == 6)	//�ѹ��� ����
				{
					if(_USER_RUN_SETTING.motion == Knee)	//����
					{
						if(Current_angle <= 0)
							target_angle_2 = (signed long)(Current_angle-0.5) + 1;
						else
							target_angle_2 = (signed long)(Current_angle+0.5) + 1;
					}
					else	//���, �Ȳ�ġ, �ո�, �߸�
						target_angle_2 = (dword)(Current_angle+0.5) - 1;
					
					Dir_of_rotation = CCW;	//�ݽð��������
					DISTANCE = 1.0;			//���� ������ ���
					mode_flag = 0;
					
					if(_USER_RUN_SETTING.calibration == 0x01)	//Calibration ����� ����
						Motor_Run_F = 1;
				}
				if(_USER_RUN_SETTING.calibration == 0x01)	//Calibration ������ �϶�
				{
					if(Motor_Run_F == 1)
						Motor_PWM = 416;
					else// ����.                  //�� ���� �������ϰ� ���� ��
					{
						PWM_SLEEP;
						Motor_PWM = 0;
						exercise_mode = 0;
						_SENSOR_DATA.state = 1;//� ���� and ����� 
					}
				}
				else	//����
				{
					switch(_USER_RUN_SETTING.motion)
					{
						case Elbow:	//�Ȳ�ġ
							if(Current_angle <= 0.4)	//�Ȳ�ġ ���Ѱ��� ��, ���۾���
							{
								exercise_mode = 0;			
								_LoRa_Data.state = 1;		//� ���� and �����
								_SENSOR_DATA.state = 1;		//� ���� and �����
							}
							else
								angle_adjustment_PI(target_angle_2/*+down_corr*/, 1, CONSTANT_MODE);
							break;
						case Shoulder://���
							if(Current_angle <= 20.4)	//��� ���Ѱ������� ��, ���۾���
							{
								exercise_mode = 0;			
								_LoRa_Data.state = 1;		//� ���� and �����
								_SENSOR_DATA.state = 1;		//� ���� and �����
							}
							else
								angle_adjustment_PI(target_angle_2/*+down_corr*/, 1, CONSTANT_MODE);
							break;
						case Knee:	//����
							if(Current_angle >= 139.5)	//���� ���Ѱ��̻��� ��, ���۾���
							{
								exercise_mode = 0;			
								_LoRa_Data.state = 1;		//� ���� and �����
								_SENSOR_DATA.state = 1;		//� ���� and �����
							}
							else
								angle_adjustment_PI(target_angle_2-down_corr, 1, CONSTANT_MODE);
							break;
						case wrist:	//�ո�
							if(Current_angle <= (-69.4))	//�ո� ���Ѱ������� ��, ���۾���
							{
								exercise_mode = 0;			
								_LoRa_Data.state = 1;		//� ���� and �����
								_SENSOR_DATA.state = 1;		//� ���� and �����
							}
							else
								angle_adjustment_PI(target_angle_2/*+down_corr*/, 1, CONSTANT_MODE);
							break;
						case ankle:	//�߸�
							if(Current_angle <= (-49.4))	//�߸� ���Ѱ������� ��, ���۾���
							{
								exercise_mode = 0;			
								_LoRa_Data.state = 1;		//� ���� and �����
								_SENSOR_DATA.state = 1;		//� ���� and �����
							}
							else
								angle_adjustment_PI(target_angle_2/*+down_corr*/, 1, CONSTANT_MODE);
							break;							
						default:
							break;
					}
				}
			}
			break;
		case 0x07:	//������� Ű�ٿ� UP ��� ��� ����
			if(mode_flag == 7)
			{			
				switch(_USER_RUN_SETTING.motion)
				{
					case Elbow:	//�Ȳ�ġ
						target_angle_3 = 135 - 0.5;	//0.5�� ��������
						DISTANCE = 135 - Current_angle;		//���� ������ ���
						break;
					case Shoulder:	//���
						target_angle_3 = 180 - 0.5;	//0.5�� ��������
						DISTANCE = 180 - Current_angle;		//���� ������ ���
						break;
					case Knee:	//����
						target_angle_3 = -10 + 0.5;	//0.5�� ��������
						DISTANCE = Current_angle - (-10);	//���� ������ ���
						break;
					case wrist:	//�ո�
						target_angle_3 = 70 - 0.5;	//0.5�� ��������
						DISTANCE = 70 - Current_angle;		//���� ������ ���
						break;
					case ankle:	//�߸�
						target_angle_3 = 50 - 0.5;	//0.5�� ��������
						DISTANCE = 50 - Current_angle;		//���� ������ ���
						break;						
					default:
						break;
				}
				Dir_of_rotation = CW;			//�ð��������
				mode_flag = 0;
				Passive_ing = 1;	//������������ �̵���
			}
			angle_adjustment_PI(target_angle_3, _USER_RUN_SETTING.speed, CONSTANT_MODE);
			break;
		case 0x08:	//������� Ű�ٿ� DOWN ��� ��� ����
			if(mode_flag == 8)
			{
				switch(_USER_RUN_SETTING.motion)
				{
					case Elbow:	//�Ȳ�ġ
						target_angle_4 = 0 + 0.5;	//0.5�� ��������
						DISTANCE = Current_angle - 0;		//���� ������ ���
						break;
					case Shoulder:	//���
						target_angle_4 = 20 + 0.5;	//0.5�� ��������
						DISTANCE = Current_angle - 20;		//���� ������ ���
						break;
					case Knee:	//����
						target_angle_4 = 140 - 0.5;	//0.5�� ��������
						DISTANCE = 140 - Current_angle;		//���� ������ ���
						break;
					case wrist:	//�ո�
						target_angle_4 = (-70)+ 0.5;	//0.5�� ��������
						DISTANCE = Current_angle - 20;		//���� ������ ���
						break;
					case ankle:	//�߸�
						target_angle_4 = (-50)+ 0.5;	//0.5�� ��������
						DISTANCE = Current_angle - 20;		//���� ������ ���
						break;								
					default:
						break;
				}			
				Dir_of_rotation = CCW;			//�ݽð��������
				mode_flag = 0;
				Passive_ing = 1;	//������������ �̵���
			}
			angle_adjustment_PI(target_angle_4, _USER_RUN_SETTING.speed, CONSTANT_MODE);
			break;
		case 0x09:	//������� �׽�Ʈ(23.06.22), ��� �������� �ۼ���(20~180�����̿��� ����)
			if(mode_flag == 9)	//���������Ҷ� �ѹ��� ����
			{
				DISTANCE = 180 - Current_angle;		//���� ������ ���
				
				if(_USER_RUN_SETTING.motion == Knee)
					Dir_of_rotation = CCW;
				else //���, �Ȳ�ġ, �߸�, �ո�
					Dir_of_rotation = CW;
				mode_flag = 0;
				_LoRa_Data.use_state = 0x02;
				Measure_up = 1;			//���Ѱ� ���� ����
				Change_dir = 1;//3�� �ڿ� Cal_rate_of_change_f = 1����
				Measurement_CNT = 0;
				Bf_value = 0;
				Bf_avr = 0;
				Detect_load = 0;
				Cal_rate_of_change_f = 0;
			}
			//Exercise_ctrl()�� 10ms �� ����
			static word measure_10mcnt = 0;
			if(measure_10mcnt++ >= 10)//�����丮 �迭�� ���� ���� ������ 10���� ������ ��� 100ms���� �����
			{
				measure_10mcnt = 0;
				current_history[current_index] = Current_actual;
			}
			current_index = (current_index + 1) % NUM_SAMPLES; // ��ȯ ����
			test_rate = calculate_slope(current_history, NUM_SAMPLES);
			baseline_current = calculate_average(current_history, NUM_SAMPLES);// ���ذ� ������Ʈ (��� ���� ���)
			relative_change = (Current_actual - baseline_current) / baseline_current;	//���ذ� ��� ������� ��ȭ���� ���� ����		
			if(Cal_rate_of_change_f)
			{
				if(relative_change > Load_sensitivity) //40%, 0.4
				{
					Cal_rate_of_change_f = 0;
					Detect_load = 1; // ���� ����
				} 				
			}
			Rate = test_rate;		//��ȭ�� ����͸���(�׽�Ʈ�� ������ �����ص� ��)
			test_rate = 0;
			if(motor_delay_ms(1000))	//1�� �ڿ� ���Ѱ� ��������
			{
				if((Measurement_CNT % 2) == 1)
					Measure_down = 1;		//���Ѱ� �������� flag ON
				else if((Measurement_CNT % 2) == 0)
					Measure_up = 1;	//���Ѱ� �������� flag ON
			}			
			if(_USER_RUN_SETTING.exerc_start == 0x09)	//������� �Ͻ������� or ���������
			{//���⼭ ������� ���� ���� ��� �ʱ�ȭ ���Ѿ���.
				total_high_angle= 0, total_low_angle= 0;				
				Measurement_CNT = 0;		//���� Ƚ�� �ʱ�ȭ
				exercise_mode = 0;			//� ����
				_VOICE_DATA.x06_1B = 0x14;//"������ �����մϴ�"���� ���
				//_LoRa_Data.use_state = 0x08;//�����
				_LoRa_Data.state = 1;		//� ����(LCD���忡 ���� ������)
				_SENSOR_DATA.state = 1;		//� ����(PC�� ���� ������)
				timer2_ms_cnt_start = 0;	//���� ī���� flag ����
				PWM_SLEEP;				
				if(measurement_stop == 0)
				{
					measurement_stop = 1; 
					if(EMS_STATE != 1)
						_VOICE_DATA.x06_1B = 0x14;//"������ �����մϴ�"���� ���
				}				
				pwm_step = 0;
			}
			else
			{//������� �����۵��� ����
				PWM_NOT_SLEEP;
				if(Measure_up)			//���Ѱ� ����
				{
					switch(_USER_RUN_SETTING.motion)
					{
						case Elbow:	//�Ȳ�ġ
							target_angle_5 = 135;
							DISTANCE = 135 - 0;		//���� ������ ���
							break;
						case Shoulder:	//���
							target_angle_5 = 180;
							DISTANCE = 180 - 20;		//���� ������ ���
							break;
						case Knee:	//����
							target_angle_5 = 140;
							DISTANCE = 140 - (-10);		//���� ������ ���
							break;
						case wrist:	//�ո�
							target_angle_5 = 70;
							DISTANCE = 70 - (-70);		//���� ������ ���
							break;
						case ankle:	//�߸�
							target_angle_5 = 50;
							DISTANCE = 50 - (-50);		//���� ������ ���
							break;							
						default:
							break;
					}
					angle_adjustment_PI(target_angle_5, 1, CONSTANT_MODE);	//���Ѱ����� �̵�
					
				}
				else if(Measure_down)	//���Ѱ� ����
				{
					switch(_USER_RUN_SETTING.motion)
					{
						case Elbow:	//�Ȳ�ġ
							target_angle_5 = 0;
							DISTANCE = 135 - 0;		//���� ������ ���
							break;
						case Shoulder:	//���
							target_angle_5 = 20;
							DISTANCE = 180 - 20;		//���� ������ ���
							break;
						case Knee:	//����
							target_angle_5 = -10;
							DISTANCE = 140 - (-10);		//���� ������ ���
							break;
						case wrist:	//�ո�
							target_angle_5 = -70;
							DISTANCE = 70 - (-70);		//���� ������ ���
							break;
						case ankle:	//�߸�
							target_angle_5 = -50;
							DISTANCE = 50 - (-50);		//���� ������ ���
							break;							
						default:
							break;
					}
					angle_adjustment_PI(target_angle_5, 1, CONSTANT_MODE);		//���Ѱ����� �̵�
				}
			}
			break;
		default:
			break;
	}
	/****************************������� ��ư ������ ��******************************/

	switch(_USER_RUN_SETTING.exerc_start)//�����߿� lcd �Ǵ� �����ġ�� �����ϴ� �κ��̸� ó�� �������δ� ���Ը���.
	{
		case 0x01:	//���۹�ư ������ ��
			if(!Motor_BUSY)
				pause_state = 1;
			break;
		case 0x02:	//�Ͻ����� ��ư ������ ��
			pause_state = 2;
			break;
		case 0x03:	//�����ư ������ ��
			if(!Motor_BUSY)
				pause_state = 3;
			break;			
		default:
			break;
	}
	static byte direction_1, direction_2;
	//�����ⱸ ���� �� ���� ���� �׽�Ʈ ����(23.02.02)
	if(_USER_RUN_SETTING.motion == Knee)	//����
	{
		direction_1 = CCW;
		direction_2 = CW;
	}
	else	//���, ����, �ո�, �߸� 24.03.07
	{
		direction_1 = CW;
		direction_2 = CCW;
	}
	if(measure_restart)
	{
		measure_restart = 0;
		//restart_f = 1;//������� ����ġ ������ Soft Start�� �ٽ� ��� ������ 2024/09/13 â������ �䱸�� ������� ���� �� ���� ���� �����ϴ� ��� ����.
	}
	/*
	if(Measurement_mode && EMS_STATE == 0)
		_SENSOR_DATA.state = 0x01;//����� or �����
	
	else if(restart_f)
	{	
	    //���찡 ��� �����մϴ� ���� ��� ������
		restart_f = 0;
		if(_SENSOR_DATA.state == 0 || exerc_end || _USER_RUN_SETTING.exerc_start == 0x09)//����� ���� and ���Ѱ����� �̵����� ��츸
		{
			PWM_NOT_SLEEP;
			init_skip_flag = 1;	//�Ͻ����� ��, �ٽ� ���۹�ư�� ������ �� �������¸� �ʱ�ȭ���� �ʱ� ���� flag
		}
		if(_USER_RUN_SETTING.exerc_start == 0x09)//�Ͻ����������̸�
		{
			_USER_RUN_SETTING.exerc_start = 0x08;//��������	
			//Measurement_mode = 1;
		}
		_LoRa_Data.state = 0;	//���
		//_SENSOR_DATA.state = 0;	//���		
		pause_state = 0;
		deaccel_flag = 0;
		PAUSE_F = 0;	//Ÿ�̸� �Ͻ����� ����
		exercise_mode = exercise_mode>>4;		
		if(exercise_mode == 0x01)//�Ϲݿ ����� ��, �ٽ� �����ϸ� �ݴ�������� �����
		{				
			if(Dir_of_rotation == direction_1)
			{
				toggle_cnt++;
				Dir_of_rotation = direction_2;
				DISTANCE = Current_angle - (set_min_angle+1);	//���� ������ ���	
			}
			else if(Dir_of_rotation == direction_2)
			{
				toggle_cnt--;
				Dir_of_rotation = direction_1;
				DISTANCE = (set_max_angle-1) - Current_angle;	//���� ������ ���
			}	
		}
		else	//����, ���߿ ���� �ٽ� �����ϸ� ���� ���� �״�� ����
		{		
			if(Dir_of_rotation == direction_1)
				DISTANCE = (set_max_angle-1) - Current_angle;	//���� ������ ���
			else if(Dir_of_rotation == direction_2)
				DISTANCE = Current_angle - (set_min_angle+1);	//���� ������ ���
		}
	}
	*/
	/***************************************************************************/
	static byte Pause_back = 0;
	static byte	stop_puase = 0;
	/****************************�Ͻ����� ��ư ������ ��******************************/
	if(pause_state == 2)	//�Ͻ����� ��ư ������ ��
	{
		stop_puase = 1;
		_SENSOR_DATA.state = 3;	//�Ͻ����� ����
		Motor_BUSY = 1;		//Soft stop ��
		deaccel_flag = 1;
		PAUSE_F = 1;	//Ÿ�̸� �Ͻ�����
		if(Pause_back == 0)
		{	
			toggle_cnt = toggle_cnt - 2;			
			Pause_back = 1;
			_VOICE_DATA.x06_1B = 0x19;//���Ͱ� �Ͻ����� �Ǿ����ϴ�. ���� ���
			voice_en = 1;
		}
		if(Dir_of_rotation == direction_1)
		{
			if((set_max_angle <= Current_angle) || (pwm_step <= 0))	//���ӽ��� ��, ���簢���� ���Ѱ� �̻��� �ǰų� ������ ������ �Ϲݿ��� ����
			{
				pwm_step = 0;
				Motor_PWM = 0;
				pause_state = 0;	//_USER_RUN_SETTING.exerc_start = 0x00;
				if(exercise_mode <= 0x03)	//�Ϲ�,����,���� ��߿�����
				{
					exercise_mode = exercise_mode<<4; //�Ͻ�������
				}
				
				Motor_BUSY = 0;
				_SENSOR_DATA.state = 0x04;	//�Ͻ����� �Ϸ�
			}
		}
		else if(Dir_of_rotation == direction_2)
		{
			if((set_min_angle >= Current_angle) || (pwm_step <= 0)) //���ӽ��� ��, ���簢���� ���Ѱ� ���ϰ� �ǰų� ������ ������ �Ϲݿ��� ����
			{
				pwm_step = 0;
				Motor_PWM = 0;
				pause_state = 0;	//_USER_RUN_SETTING.exerc_start = 0x00;
				if(exercise_mode <= 0x03)	//�Ϲ�,����,���� ��߿�����
				{
					exercise_mode = exercise_mode<<4; //�Ͻ�������
				}
				Motor_BUSY = 0;
				_SENSOR_DATA.state = 0x04;	//�Ͻ����� �Ϸ�
			}
		}
		
	}
	else if(((pause_state == 1) && (Motor_BUSY == 0)))	//�ٽ� ���۹�ư ������ ��
	{
		gen_ada_con_back = 0; //��� �����մϴ� �������(�Ϲ�,����,����)
		measurement_stop = 0;  //������� ���� ����
		//voice_en = 1;
		Pause_back = 0;
		init_skip_flag = 1;	//�Ͻ����� ��, �ٽ� ���۹�ư�� ������ �� �������¸� �ʱ�ȭ���� �ʱ� ���� flag
		_USER_RUN_SETTING.exerc_start = 0x00;
		pause_state = 0;
		deaccel_flag = 0;
		_LoRa_Data.state = 0;	//���
		_SENSOR_DATA.state = 0;	//���		
		PAUSE_F = 0;	//Ÿ�̸� �Ͻ����� ����
		exercise_mode = exercise_mode>>4;		//�ϴ� �Ϲݿ �Ͻ������� �׽�Ʈ��.23.01.12
		if(exercise_mode == 0x01)	//�Ϲݿ ����� ��, �ٽ� �����ϸ� �ݴ�������� �����
		{
			if(Dir_of_rotation == direction_1)
			{
				toggle_cnt++;
				Dir_of_rotation = direction_2;
				DISTANCE = Current_angle - (set_min_angle+1);	//���� ������ ���	
			}
			else if(Dir_of_rotation == direction_2)
			{
				toggle_cnt--;
				Dir_of_rotation = direction_1;
				DISTANCE = (set_max_angle-1) - Current_angle;	//���� ������ ���
			}	
		}
		else	//����, ���߿ ���� �ٽ� �����ϸ� ���� ���� �״�� ����
		{
			if(Dir_of_rotation == direction_1)
				DISTANCE = (set_max_angle-1) - Current_angle;	//���� ������ ���
			else if(Dir_of_rotation == direction_2)
				DISTANCE = Current_angle - (set_min_angle+1);	//���� ������ ���
		}
	}
	/***************************************************************************/
	
	
	/****************************���� ��ư ������ ��********************************/
	if(pause_state == 3 && (Motor_BUSY == 0))	//�����ư ������ ��
	{
		exerc_time_start = 0;	//� �ð������ ��� �ð� ī���� ����
		deaccel_flag = 0;
		pwm_step = 0;
		exerc_cnt = 0;
		if (exercise_mode != 5 && exercise_mode != 6 && exercise_mode != 7 && exercise_mode != 8 && exercise_mode != 9) //�������, ������尡 �ƴϸ� ����
			exerc_end = 1;
		exercise_mode = 0;
		Pause_back = 0;
		Move_to_lower = 1;
		mode_flag = 10;
		
		Time_mode_end = 0;
		_USER_RUN_SETTING.exerc_start = 0x00;
		pause_state = 0;
		PAUSE_F = 0;	//Ÿ�̸� �Ͻ����� ����
		
		//���߿������ ���
		repeat_ok = 0;	
		fake_cnt = 0;
		
		Not_Send_Ble = 1; // ���� ��ư�� ���������� ��������� ���� �����͸� ������ �ʵ��� ��.
	}
	/**************************************************************************/
	/*****************************� ������ ��������******************************/
	if(exerc_end)	//��� ���� �� ���Ѱ� �̵�, ���Ѱ�+10���� ����
	{	
		PWM_NOT_SLEEP;
		_SENSOR_DATA.state = 2;	//������(���Ѱ��̵���) 
		if(stop_puase)
		{
			stop_puase = 0;
			//restart_f = 1;	//������� ����ġ ������ Soft Start�� �ٽ� ��� ������
		}
		_LoRa_Data.state = 0;		//���Ѱ����� �̵��� ��߻���
		_SENSOR_DATA.velocity_mode = 0x00;	//���
		if(Move_to_lower)//���� ������ �̵�(�ݽð����)
		{
			if(mode_flag == 10)
			{		//22.12.19. 1�� ��������
				if(set_speed_step >= 6)	//6�ܰ� �̻��� ���� ����
					DISTANCE = Current_angle - (set_min_angle);
					//DISTANCE = Current_angle - (set_min_angle+0.5);
				else if(set_speed_step < 6)
					DISTANCE = Current_angle - set_min_angle;
				mode_flag = 0;
			}
			Dir_of_rotation = direction_2;	//���Ѱ����� �̵��ϱ� ���� �ݽð� �������� ��ȯ
			if(set_speed_step < 8)//1 ~ 7 �ܰ�
				exerc_stop_mode(set_min_angle, set_speed_step, direction_2);
			else if(set_speed_step >= 8)
				exerc_stop_mode(set_min_angle, 6, direction_2);				

		}
		else//���� ����+10���� �̵�(�ð����)
		{
			if(mode_flag == 10)
			{			//22.12.19. 1�� ��������
				DISTANCE = (set_min_angle+10) - Current_angle;//�μ�
				mode_flag = 0;
			}
			Dir_of_rotation = direction_1;
			exerc_stop_mode(set_min_angle + 10, 3, direction_1);	//���Ѱ�+10���� �̵�(�ӵ� 3�ܰ�� ���Ƿ� ����)
		}
	}
	/**************************************************************************/
	
	
	
	/*********************************�ӵ� ����***********************************/
	static byte tcnt_10ms = 0;
	if(++tcnt_10ms >= 10)
	{//100ms task
		tcnt_10ms = 0;
		cnt2 = TIM1->CNT;//���ڴ��Է�
		//TIM1�� CNT���� �ð�������� ȸ���� �� ����, �ݽð�������� ȸ���� �� ������
		/* ȸ�� ���� Ȯ�� */
		if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim1))
		{
			dir = 1;	//�ݽð� ����
			
			if((TIM1->CNT) > bf_CNT)	//CNT�� under�Ǿ� ���� CNT���� ���� CNT������ Ŭ ��
				Under_cnt++;
			
			bf_CNT = TIM1->CNT;

			
			/* __HAL_TIM_IS_TIM_COUNTING_DOWN ��ũ�ΰ� ���⼺�� �߸� �˷��ִ� ��� ����ó�� */
			if((cnt2 > cnt1) && (cnt2 - cnt1 < 100)) 
				dir = 0;
		}
		else
		{
			dir = 0;	//�ð� ����
			
			if((TIM1->CNT) < bf_CNT)	//CNT�� over�Ǿ� ���� CNT���� ���� CNT������ ���� ��
				Over_cnt++;
			
			bf_CNT = TIM1->CNT;
			
			/* __HAL_TIM_IS_TIM_COUNTING_DOWN ��ũ�ΰ� ���⼺�� �߸� �˷��ִ� ��� ����ó�� */
			if((cnt1 > cnt2) && (cnt1 - cnt2 < 100)) 
				dir = 1;
		}
		if(dir)    //�ð� ����(����)
		{
			/* Down Counting �������� ȸ���� �� */
			if(cnt1 >= cnt2) 
			{
				diff = cnt1 - cnt2;
			}
			//if(cnt1 >= cnt2 && dir==1) 
			//	diff = cnt1 - cnt2;
			//else if(cnt2 > cnt1 && dir==0) 
			//	diff = cnt2 - cnt1;
			else 
				diff = (TIM1 -> ARR + cnt1) - cnt2; 
			Encoder_dir = 'U';		//���ڴ� ����üũ��(���⼺ üũ)	//UP
		}
		else       //�ݽð� ����(����)
		{
			/* Up Counting �������� ȸ���� �� */
			if(cnt2 >= cnt1) 
			{
				diff = cnt2 - cnt1;
			}
			//if(cnt2 >= cnt1 && dir==0) 
			//	diff = cnt2 - cnt1;
			//else if(cnt1 > cnt2 && dir==1) 
			//	diff = cnt1 - cnt2;
			else 
				diff = (TIM1->ARR + cnt2) - cnt1;
			Encoder_dir = 'D';		//���ڴ� ����üũ��(���⼺ üũ)	//DOWN
		}
		//SMCR : Slave Mode Control Registe
		if((TIM1->SMCR & 0x03) == 0x03)	//Encoder mode 3 : X4 mode �϶� (SMCR���������� SMS�� 011�϶� ���ڴ� 4ü����)
		{
			/* X4 Mode �϶��� ī���Ͱ� 4 ������ ��, 1���� Pulse */
			speed_RPM = diff * 600 / 4 / 500;  //ONE_ROTATION_PULSES/15; //RPM ����� ���ؼ��� x 60
									//1����Ŭ�� 500�޽� : 500cpr
		}	
		cnt1 = TIM1->CNT;// ���� ���� ���� ������ ������Ʈ �Ʒ��� ���� �ǹ�
		//cnt1 = cnt2;// ���� ���� ���� ������ ������Ʈ
	}
	/******************************�ӵ� ���� END******************************** **/

	
	/**************************LCD ��Ʈ�ѷ� ����(�б�)******************************/
	_SENSOR_DATA.current = Current_actual;					//����(A)
	_SENSOR_DATA.current_speed = speed_RPM;					//���� �ӵ�(RPM)
	_SENSOR_DATA.dir = Dir_of_rotation;						//���� � ����
	if(_USER_RUN_SETTING.cnt_mode == 0)// || (_LoRa_Data.remain_time_and_cnt >> 7)) == 0)	//��ð� ����϶�
	{
		_SENSOR_DATA.remain_time = set_exerc_time - min_cnt;	//���� ��ð�(��)
		_LoRa_Data.remain_time_and_cnt = set_exerc_time - min_cnt;	//���� ��ð�(��) MSB : 0
		_Ble_Data.remain_time_and_cnt = set_exerc_time - min_cnt;	//���� ��ð�(��) MSB : 0
		_SENSOR_DATA.cnt = 0;//� ����Ƚ�� 0���� ó��, Lora������ �Ǿ �����൵ �� 
	}
	else	//�Ƚ�� ����϶�
	{
		_SENSOR_DATA.cnt = num_of_workouts;//� ����Ƚ��
		_LoRa_Data.remain_time_and_cnt = 0x80 | num_of_workouts;	//� ���� Ƚ�� MSB : 1
		_Ble_Data.remain_time_and_cnt =  0x80 | num_of_workouts;	//� ���� Ƚ�� MSB : 1
		_SENSOR_DATA.remain_time = 0;//���� ��ð� 0���� ó��, Lora������ �Ǿ �����൵ �� 
	}
	/**************************************************************************/
	
	/*****************************LCD ǥ�ð��� ����********************************/
	static float limit_angle;
	if((_USER_RUN_SETTING.exerc_start == 0x06) || (_USER_RUN_SETTING.exerc_start == 0x07) || (Passive_ing == 1))	//������� 
	{
		switch(_USER_RUN_SETTING.motion)
		{
			case Elbow:	//�Ȳ�ġ
				if(Current_angle >= 135)
				{
					_SENSOR_DATA.current_angle = 135;
					_Ble_Data.current_angle = 135;
				}
				else if(Current_angle <= 0)
				{
					_SENSOR_DATA.current_angle = 0;
					_Ble_Data.current_angle = 0;
				}
				else
				{
					if (GPIOD->ODR & GPIO_PIN_10) // PD10�� ���°� 1�� �� ������ �ڵ�
					{//���Ͱ� ���ư� �߿��� �������� ����
						_SENSOR_DATA.current_angle = Current_angle;
						_Ble_Data.current_angle = Current_angle;	//���簢�� ǥ��
					}							
				}
				break;
			case Shoulder:	//���
				if(Current_angle >= 180)
				{
					_SENSOR_DATA.current_angle = 180;
					_Ble_Data.current_angle = 180;
				}
				else if(Current_angle <= 20)
				{
					_SENSOR_DATA.current_angle = 20;
					_Ble_Data.current_angle = 20;
				}
				break;
			case Knee:	//����
				if(Current_angle >= 140)
				{
					_SENSOR_DATA.current_angle = 140;
					_Ble_Data.current_angle = 140;
				}
				else if(Current_angle <= (-10))
				{
					_SENSOR_DATA.current_angle = -10;
					_Ble_Data.current_angle = -10;
				}
				else
				{
					if (GPIOD->ODR & GPIO_PIN_10) // PD10�� ���°� 1�� �� ������ �ڵ�
					{//���Ͱ� ���ư� �߿��� �������� ����
						_SENSOR_DATA.current_angle = Current_angle;
						_Ble_Data.current_angle = Current_angle;	//���簢�� ǥ��
					}							
				}				
				break;
			case wrist:	//�ո�
				if(Current_angle >= 70)
				{
					_SENSOR_DATA.current_angle = 70;
					_Ble_Data.current_angle = 70;
				}
				else if(Current_angle <= (-70))
				{
					_SENSOR_DATA.current_angle = -70;
					_Ble_Data.current_angle = -70;
				}
				else
				{
					if (GPIOD->ODR & GPIO_PIN_10) // PD10�� ���°� 1�� �� ������ �ڵ�
					{//���Ͱ� ���ư� �߿��� �������� ����
						_SENSOR_DATA.current_angle = Current_angle;
						_Ble_Data.current_angle = Current_angle;	//���簢�� ǥ��
					}							
				}				
				break;	
			case ankle:	//�߸�
				if(Current_angle >= 50)
				{
					_SENSOR_DATA.current_angle = 50;
					_Ble_Data.current_angle = 50;
				}
				else if(Current_angle <= (-50))
				{
					_SENSOR_DATA.current_angle = -50;
					_Ble_Data.current_angle = -50;
				}
				else
				{
					if (GPIOD->ODR & GPIO_PIN_10) // PD10�� ���°� 1�� �� ������ �ڵ�
					{//���Ͱ� ���ư� �߿��� �������� ����
						_SENSOR_DATA.current_angle = Current_angle;
						_Ble_Data.current_angle = Current_angle;	//���簢�� ǥ��
					}							
				}				
				break;				
			default:
				break;
		}
	}
	else
	{
		switch(_USER_RUN_SETTING.mode)
		{
			case 0:	//�Ϲݿ �����Ǿ� ���� ��
				if((exercise_mode == NORMAL) || (exerc_end))	//�Ϲݿ ������ �Ǵ� ������
				{
					if(Current_angle >= set_max_angle)	//���Ѱ� �Ѿ�� ���Ѱ����� ǥ��
					{
						_SENSOR_DATA.current_angle = set_max_angle;
						_Ble_Data.current_angle = set_max_angle;
					}
					else if(Current_angle <= set_min_angle)	//���Ѱ� �Ѿ�� ���Ѱ����� ǥ��
					{
						if(move_to_lower)
						{
							_SENSOR_DATA.current_angle = Current_angle;	//ó���� ���Ѱ� �̵��߿��� ���簢 ǥ��
							_Ble_Data.current_angle = Current_angle;	//ó���� ���Ѱ� �̵��߿��� ���簢 ǥ��
						}
						else
						{
							_SENSOR_DATA.current_angle = set_min_angle;
							_Ble_Data.current_angle = set_min_angle;	
						}
					}
					else//���Ѱ�, ���Ѱ� ������ �϶�
					{
						_SENSOR_DATA.current_angle = Current_angle;	//���簢�� ǥ��
						_Ble_Data.current_angle = Current_angle;	//���簢�� ǥ��
					}
					if(!Move_to_lower)	//���Ѱ�+10���� �̵����� ��
					{
						_SENSOR_DATA.current_angle = set_min_angle + 10;
						_Ble_Data.current_angle = set_min_angle + 10;
					}
				}
				else if(!exercise_mode)		//����
				{
					if(GPIOD->ODR & GPIO_PIN_10) // PD10�� ���°� 1�� �� ������ �ڵ�
					{
						_SENSOR_DATA.current_angle = Current_angle;	//���簢�� ǥ��
						_Ble_Data.current_angle = Current_angle;	//���簢�� ǥ��
					}
				}
				break;
			case 1:	//����� �����Ǿ� ���� ��
				if(exercise_mode == ADAPTIVE)	//����� ������
				{
					switch(set_location)	//������ġ
					{
						case 0:	//LOWER
							if(Current_angle >= set_max_angle)	//���Ѱ� �Ѿ�� ���Ѱ����� ǥ��
							{
								_SENSOR_DATA.current_angle = set_max_angle;
								_Ble_Data.current_angle = set_max_angle;	
							}
							else if(Current_angle <= (set_min_angle+set_PA-angle_step))
							{
								if(move_to_lower)
								{
									_SENSOR_DATA.current_angle = Current_angle; //���簢�� ǥ��
									_Ble_Data.current_angle = Current_angle;	//���簢�� ǥ��
								}
								else
								{
									_SENSOR_DATA.current_angle = (set_min_angle+set_PA-angle_step);
									_Ble_Data.current_angle =(set_min_angle+set_PA-angle_step);
								}
							}
							else	//���Ѱ�, ���Ѱ� ������ �϶�
							{
								_SENSOR_DATA.current_angle = Current_angle;	//���簢�� ǥ��
								_Ble_Data.current_angle = Current_angle;	//���簢�� ǥ��
							}
							break;
						case 1:	//UP_LOW
							if(Current_angle >= (set_max_angle-set_PA+angle_step))	//���Ѱ� �Ѿ�� ���Ѱ����� ǥ��
								_SENSOR_DATA.current_angle = (set_max_angle-set_PA+angle_step);
							else if(Current_angle <= (set_min_angle+set_PA-angle_step))	//���Ѱ� �Ѿ�� ���Ѱ����� ǥ��
							{
								if(move_to_lower)
								{
									_SENSOR_DATA.current_angle = Current_angle;
									_Ble_Data.current_angle = (set_min_angle+set_PA-angle_step);
								}
								else
								{
									_SENSOR_DATA.current_angle = (set_min_angle+set_PA-angle_step);
									_Ble_Data.current_angle = (set_min_angle+set_PA-angle_step);
								}
							}
							else	//���Ѱ�, ���Ѱ� ������ �϶�
							{
								_SENSOR_DATA.current_angle = Current_angle;	//���簢�� ǥ��
								_Ble_Data.current_angle = Current_angle;	//���簢�� ǥ��
							}
							break;
						case 2:	//UPPER
							if(Current_angle >= (set_max_angle-set_PA+angle_step))	
							{
								_SENSOR_DATA.current_angle = (set_max_angle-set_PA+angle_step);
								_Ble_Data.current_angle = (set_max_angle-set_PA+angle_step);
							}
							else if(Current_angle <= set_min_angle)	//���Ѱ� �Ѿ�� ���Ѱ����� ǥ��
							{
								if(move_to_lower)
								{
									_SENSOR_DATA.current_angle = Current_angle;
									_Ble_Data.current_angle = Current_angle; 
								}
								else
								{
									_SENSOR_DATA.current_angle = set_min_angle;
									_Ble_Data.current_angle = set_min_angle; 
								}
							}
							else	//���Ѱ�, ���Ѱ� ������ �϶�
							{
								_SENSOR_DATA.current_angle = Current_angle;	//���簢�� ǥ��
								_Ble_Data.current_angle = Current_angle; //���簢�� ǥ��
							}
								
							break;
						default:
							break;
					}	
				}
				else if(!exercise_mode)		//���� �Ǵ� �������� ��
				{
					if(exerc_end)	//����� ���� ���϶�
					{
						if(Move_to_lower)	//���Ѱ����� �̵����� ��
						{
							if(Current_angle <= set_min_angle)	//���Ѱ� �Ѿ �� ���Ѱ� ǥ��
							{
								_SENSOR_DATA.current_angle = set_min_angle;
								_Ble_Data.current_angle = set_min_angle;	
							}
							else
							{
								_SENSOR_DATA.current_angle = Current_angle;	//���簢�� ǥ��
								_Ble_Data.current_angle = Current_angle;	//���簢�� ǥ��
							}
						}
						else	//���Ѱ�+10���� �̵����� ��
						{
							if(Current_angle >= (set_min_angle+10))	//���Ѱ�+10 �Ѿ�� ���Ѱ�+10���� ǥ��
							{
								_SENSOR_DATA.current_angle = set_min_angle + 10;
								_Ble_Data.current_angle = set_min_angle + 10;
							}
							else
							{
								_SENSOR_DATA.current_angle = Current_angle;	//���簢�� ǥ��
								_Ble_Data.current_angle = Current_angle;	//���簢�� ǥ��
							}
						}
					}
					else	//���� �Ǵ� ����ᰡ ������ ������ ��
					{
						if(GPIOD->ODR & GPIO_PIN_10) // PD10�� ���°� 1�� �� ������ �ڵ�
							_SENSOR_DATA.current_angle = Current_angle;	//���簢�� ǥ��
					}
				}
			
				break;
			case 2:	//���߿ �����Ǿ� ���� ��
				if(exercise_mode == INTENSIVE)	//���߿ ������
				{
					switch(set_location)	//������ġ
					{
						case 0:	//LOWER
							switch(Focus_f)
							{
								case 0:	//���Ѱ��̵�
									limit_angle = (repeat_ok == 1) ? set_max_angle : (set_min_angle+set_conc_angle);
									if(Current_angle >= limit_angle)
									{
										_SENSOR_DATA.current_angle = limit_angle;
										_Ble_Data.current_angle = limit_angle;
									}
									else if(Current_angle <= set_min_angle)
									{
										_SENSOR_DATA.current_angle = set_min_angle;
										_Ble_Data.current_angle = set_min_angle;
									}
									else
									{
										_SENSOR_DATA.current_angle = Current_angle;
										_Ble_Data.current_angle = Current_angle;
									}
									break;
								case 1:	//���Ѱ��̵�
									if(Current_angle >= set_max_angle)
									{
										_SENSOR_DATA.current_angle = set_max_angle;
										_Ble_Data.current_angle = set_max_angle;
									}
									else if(Current_angle <= set_min_angle)
									{
										_SENSOR_DATA.current_angle = set_min_angle;
										_Ble_Data.current_angle = set_min_angle;
									}
									else
									{
										_SENSOR_DATA.current_angle = Current_angle;
										_Ble_Data.current_angle = Current_angle;
									}
									break;
								case 2:	//���� ���߰� �̵�
									if(Current_angle >= (set_min_angle+set_conc_angle))
										_SENSOR_DATA.current_angle = (set_min_angle+set_conc_angle);
									else if(Current_angle <= set_min_angle)
									{
										_SENSOR_DATA.current_angle = set_min_angle;
										_Ble_Data.current_angle = set_min_angle;
									}
									else
									{
										_SENSOR_DATA.current_angle = Current_angle;
										_Ble_Data.current_angle = Current_angle;
									}
									break;
								default:
									break;
							}
							break;
						case 1:	//UP_LOW
							switch(Focus_f)
							{
								case 0:	//���Ѱ��̵�
									limit_angle = (except_f == 1) ? set_max_angle : (set_min_angle+set_conc_angle);
									
									if(Current_angle >= limit_angle)
									{
										_SENSOR_DATA.current_angle = limit_angle;
										_Ble_Data.current_angle = limit_angle;
									}
									else if(Current_angle <= set_min_angle)
									{
										_SENSOR_DATA.current_angle = set_min_angle;
										_Ble_Data.current_angle = set_min_angle;
									}
									else
									{
										_SENSOR_DATA.current_angle = Current_angle;
										_Ble_Data.current_angle = Current_angle;
									}
									break;
								case 1:	//���Ѱ��̵�
									limit_angle = (except_f == 1) ? set_min_angle : (set_max_angle-set_conc_angle);
									
									if(Current_angle >= set_max_angle)
									{
										_SENSOR_DATA.current_angle = set_max_angle;
										_Ble_Data.current_angle = set_max_angle;
									}
									else if(Current_angle <= limit_angle)
									{
										_SENSOR_DATA.current_angle = limit_angle;
										_Ble_Data.current_angle = limit_angle;
									}
									else
									{
										_SENSOR_DATA.current_angle = Current_angle;
										_Ble_Data.current_angle = Current_angle;
									}
									break;
								case 2:	//���� ���߰� �̵�
									except_f = 0;	//�������� ����(������ġ ���Ѱ�)
									if(Current_angle >= (set_min_angle+set_conc_angle))
									{
										_SENSOR_DATA.current_angle = (set_min_angle+set_conc_angle);
										_Ble_Data.current_angle =(set_min_angle+set_conc_angle);
									}
									else if(Current_angle <= set_min_angle)
									{
										_SENSOR_DATA.current_angle = set_min_angle;
										_Ble_Data.current_angle = set_min_angle;
									}
									else
									{
										_SENSOR_DATA.current_angle = Current_angle;
										_Ble_Data.current_angle = Current_angle;
									}
									break;
								case 3:	//���� ���߰� �̵�
									except_f = 0;	//�������� ����(������ġ ���Ѱ�)
									if(Current_angle >= set_max_angle)
									{
										_SENSOR_DATA.current_angle = set_max_angle;
										_Ble_Data.current_angle = set_max_angle;
									}
									else if(Current_angle <= (set_max_angle-set_conc_angle))
									{
										_SENSOR_DATA.current_angle = (set_max_angle-set_conc_angle);
										_Ble_Data.current_angle = (set_max_angle-set_conc_angle);
									}
									else
									{
										_SENSOR_DATA.current_angle = Current_angle;
										_Ble_Data.current_angle = Current_angle;
									}
									break;
								default:
									break;
							}
							break;
						case 2:	//UPPER
							switch(Focus_f)
							{
								case 0:	//���Ѱ��̵�
									if(Current_angle >= set_max_angle)
									{
										_SENSOR_DATA.current_angle = set_max_angle;
									}
									else if(Current_angle <= set_min_angle)
									{
										_SENSOR_DATA.current_angle = set_min_angle;
										_Ble_Data.current_angle = set_min_angle;
									}
									else
									{
										_SENSOR_DATA.current_angle = Current_angle;
										_Ble_Data.current_angle = Current_angle;
									}
									break;
								case 1:	//���Ѱ��̵�
									limit_angle = (repeat_ok == 1) ? set_min_angle : (set_max_angle-set_conc_angle);
									if(except_f)
										limit_angle = set_min_angle;	//ó�� ���Ѱ��̵��� ����ó��
									if(Current_angle >= set_max_angle)
									{
										_SENSOR_DATA.current_angle = set_max_angle;
										_Ble_Data.current_angle = set_max_angle;
									}
									else if(Current_angle <= limit_angle)
									{
										_SENSOR_DATA.current_angle = limit_angle;;
										_Ble_Data.current_angle = limit_angle;
									}
									else
									{
										_SENSOR_DATA.current_angle = Current_angle;
										_Ble_Data.current_angle = Current_angle;
									}
									break;
								case 3:	//���� ���߰� �̵�
									except_f = 0;	//�������� ����(������ġ ���Ѱ�)
									if(Current_angle >= set_max_angle)
									{
										_SENSOR_DATA.current_angle = set_max_angle;
										_Ble_Data.current_angle = (set_max_angle-set_conc_angle);
									}
									else if(Current_angle <= (set_max_angle-set_conc_angle))
									{
										_SENSOR_DATA.current_angle = (set_max_angle-set_conc_angle);
										_Ble_Data.current_angle = (set_max_angle-set_conc_angle);
									}
									else
									{
										_SENSOR_DATA.current_angle = Current_angle;//���簢�� ǥ��
										_Ble_Data.current_angle = Current_angle;//���簢�� ǥ��
									}
									break;
								default:
									break;
							}
							break;
						default:
							break;
					}	
					if(Focus_f == 'U')		//������� ���Ѱ� �̵���(���簢�� ���Ѱ����� ������)
					{
						_SENSOR_DATA.current_angle = Current_angle;
						_Ble_Data.current_angle = Current_angle;//���簢�� ǥ��
					}
					else if(Focus_f == 'D')	//������� ���Ѱ� �̵���(���簢�� ���Ѱ����� Ŭ��)
					{
						if(Current_angle <= set_min_angle)
						{
							_SENSOR_DATA.current_angle = set_min_angle;
							_Ble_Data.current_angle = Current_angle;//���簢�� ǥ��
						}
						else
						{
							_SENSOR_DATA.current_angle = Current_angle;
							_Ble_Data.current_angle = Current_angle;//���簢�� ǥ��
						}
					}	
				}
				else if(!exercise_mode)//���� �Ǵ� �������� �� �μ�
				{
					if(exerc_end)//����� ���� ���϶�
					{
						if(Move_to_lower)	//���Ѱ����� �̵����� ��
						{
							if(Current_angle >= set_max_angle)
								_SENSOR_DATA.current_angle = set_max_angle;
							else if(Current_angle <= set_min_angle)	//���Ѱ� �Ѿ �� ���Ѱ� ǥ��
								_SENSOR_DATA.current_angle = set_min_angle;
							else
							{
								_SENSOR_DATA.current_angle = Current_angle;	//���簢�� ǥ��
								_Ble_Data.current_angle = Current_angle;//���簢�� ǥ��
							}
						}
						else//���Ѱ�+10���� �̵����� ��
						{
							if(Current_angle <= set_min_angle)
								_SENSOR_DATA.current_angle = set_min_angle;
							else if(Current_angle >= (set_min_angle + 10))	//���Ѱ�+10 �Ѿ�� ���Ѱ�+10���� ǥ��
								_SENSOR_DATA.current_angle = set_min_angle + 10;
							else
							{
								_SENSOR_DATA.current_angle = Current_angle;	//���簢�� ǥ��
								_Ble_Data.current_angle = Current_angle;//���簢�� ǥ��
							}
						}
					}
					else	//���� �Ǵ� ����ᰡ ������ ������ ��
					{
						if(GPIOD->ODR & GPIO_PIN_10) // PD10�� ���°� 1�� �� ������ �ڵ�
							_SENSOR_DATA.current_angle = Current_angle;	//���簢�� ǥ��
					}
				}
				break;
			default:
				break;
		}
	}
	/***************************LCD ǥ�ð��� ���� END*****************************/
	Motor_ctrl();	//���� PWM ����  
}//������ Exercise_ctrl �Լ��� ��!
float Chang_angle = 0;
unsigned int Motor_PWM_CCW = 1800, Motor_PWM_CW = 1800; 
word Log_buf[1024], Log_index=0, Log_enable=0;
void Motor_ctrl()	//Call by 10msec
{
	if(!Error_Check)
	{
		if(Dir_of_rotation == CCW)	//������(�ݽð� ����),1
		{
			Motor_PWM_CCW = (unsigned int)(-0.5*Motor_PWM + 1800);	//Motor_PWM:0~3600
			if(Motor_PWM_CCW >= 3600 * 0.9)			//PWM's Duty : 90%(3240)	
				Motor_PWM_CCW = 3600 * 0.9;			//3240
			else if(Motor_PWM_CCW <= 3600 * 0.1)	//PWM's Duty : 10%(360)	
				Motor_PWM_CCW = 3600 * 0.1;			//360
			htim8.Instance->CCR1 = Motor_PWM_CCW; 
		}
		else if(Dir_of_rotation == CW)	//������(�ð�	����),0
		{	
			Motor_PWM_CW = (unsigned int)(0.5*Motor_PWM + 1800);	//Motor_PWM:0~3600
			if(Motor_PWM_CW >= 3600 * 0.9)			//PWM's Duty : 90%(3240)	
				Motor_PWM_CW = 3600 * 0.9;			//3240
			else if(Motor_PWM_CW <= 3600 * 0.1)	//PWM's Duty : 10%(360)	
				Motor_PWM_CW = 3600 * 0.1;			//360
			htim8.Instance->CCR1 = Motor_PWM_CW; 
		}
		

		if(Log_enable)//Motor_PWM ���� ���� �ǽð����� ���ϴ� ���� Ȯ���ϴ� �׽�Ʈ�� �����̸� ���α׷� ���ۿ� �������� �ʾ� �����Ͽ��� ���ۿ��� ���� x
		{
			Log_buf[Log_index++] = Motor_PWM;
			if(Log_index>1024)
			{
				Log_index=0;
				Log_enable=0;
			}
		}	
		
//		static word test_ccr1 = 0, test_ccr2 = 0;
//		test_ccr1 = htim4.Instance->CCR1;
//		test_ccr2 = htim4.Instance->CCR2;
		if(Dir_of_rotation == CCW)	//������(�ݽð� ����)
		{
		//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);    //PWM2_H   (OFF) 
		//	htim4.Instance->CCR2 = 0;  								 //PWM1_L	(OFF)
			TIM4_CCR2_buffer = 0;  								 //PWM1_L	(OFF)
		//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);      //PWM1_H	(ON)
		//	htim4.Instance->CCR1 = 	Motor_PWM; 						 //PWM2_L	(ON) : ������ ���� �����ϱ� ���ؼ� �����.
			TIM4_CCR1_buffer = Motor_PWM; 						 //PWM2_L	(ON) : ������ ���� �����ϱ� ���ؼ� �����.
		}
		else if(Dir_of_rotation == CW)	//������(�ð�	����)
		{
			//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);     //PWM1_H	(OFF)
		//	htim4.Instance->CCR1 = 	0; 						 		  //PWM2_L	(OFF)
			TIM4_CCR2_buffer = Motor_PWM;
		//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);       //PWM2_H	(ON) 
		//	htim4.Instance->CCR2 = Motor_PWM;  						  //PWM1_L	(ON)
			TIM4_CCR1_buffer = 0;
		}
		//���ڴ� �������� �׽�Ʈ(������ ���ؼ� ������� ����)
		/*
		if(Record_data_f == 1)
		{
			Chang_angle = OFFSET_angle_2 - OFFSET_angle;
			Record_data_f = 0;
		}
		else if(Record_data_f == 2)
		{
			Chang_angle = OFFSET_angle - OFFSET_angle_2;
			Record_data_f = 0;
		}
		
		if(TEST_NUM < 50)
		{
			if(motor_delay_ms(2000))	//2���� ���� ��������
			{
				if(Record_data_f == 1)
				{
					set_angle_value = Current_angle - 2;
					Record_data_f = 0;
					if(Record_Index < 100)
					{
						End_angle_1[Record_Index] = Current_angle;	//�� ���� ����(���ڴ� ��������)
						Under_cnt_arr[Record_Index] = Under_cnt;	//���� ī��Ʈ ����(���ڴ� ��������)
						Current_CNT_1[Record_Index++] = TIM1->CNT;	//���� CNT�� ����
					}
					
				}
				else if(Record_data_f == 2)
				{
					set_angle_value = Current_angle + 2;
					Record_data_f = 0;
					if(Record_Index < 100)
					{
						End_angle_2[Record_Index] = Current_angle;	//�� ���� ����(���ڴ� ��������)
						Over_cnt_arr[Record_Index] = Over_cnt;	//���� ī��Ʈ ����(���ڴ� ��������)
						Current_CNT_2[Record_Index++] = TIM1->CNT;	//���� CNT�� ����
					}
				}
				exercise_mode = 4;	//���� ����
				TEST_NUM++;
			}
		}*/
	}
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
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
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
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = ENABLE;
  hadc1.Init.NbrOfDiscConversion = 1;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  /* USER CODE END ADC1_Init 2 */
}
/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;//eeprom�� 400khz���� ��� , i2c�� 100khz ~ 400khz ���� ����
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 12;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}
/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{
  /* USER CODE BEGIN SPI1_Init 0 */
  /* USER CODE END SPI1_Init 0 */
  /* USER CODE BEGIN SPI1_Init 1 */
  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;//SPI_DIRECTION_1LINE 
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;// SPI_NSS_SOFT
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;//APB2 72MHz�� 8���� �Ͽ� ������� ���� ���ļ��� 9MHz�� ���־���. ��������� �ִ� ���� ���ļ��� 16MHz�� ����Ͽ���.
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */
  /* USER CODE END SPI1_Init 2 */
}
/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{//���ڴ� ���� Ÿ�̸�
  /* USER CODE BEGIN TIM1_Init 0 */
  /* USER CODE END TIM1_Init 0 */
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  /* USER CODE BEGIN TIM1_Init 1 */
  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;//72MHz APB2 
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;   //���ڴ� ���� Ÿ�̸�
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
  /* USER CODE END TIM1_Init 2 */
}
/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{//0.1msecŸ�̸�
//APB1 TIMER 72MHz
  /* USER CODE BEGIN TIM2_Init 0 */
  /* USER CODE END TIM2_Init 0 */
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  /* USER CODE BEGIN TIM2_Init 1 */
  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 72e6/10e3-1;// = 7200 -> 10000  0.1msec ī����      
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  /* USER CODE END TIM2_Init 2 */
}
static void MX_TIM3_Init(void)
{
	/* USER CODE BEGIN TIM3_Init 0 */
	/* USER CODE END TIM3_Init 0 */
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	/* USER CODE BEGIN TIM3_Init 1 */
	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 2 - 1;	//2����(72MHz/2=32MHz : TIM3's Clock)
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = (36e6 / 1e3 - 1);// = 36000 -> 1msec ī����     
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{//���͵���̹� ���� ���� ���� Ÿ�̸��̸� ����� TIM8�� �����
  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;//72MHz, 20KHz
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 3600-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}
static void MX_TIM5_Init(void)//APB1 72MHz
{//0.01ms Ÿ�̸�
  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;//0����(72MHz/2=36MHz : TIM3's Clock)
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 65535;//   0.91ms ī����
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}
/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)	//Display Board Communication
{//lcd and ���͵���̹� ��ſ�
  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 19200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)	//Blue tooth or LoRa Communication
{
//������� or �ζ� ���
  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

}


/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, FA_RESET_Pin|PWM2_H_Pin|PWM1_H_Pin|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, RLY_M_SEL_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET);
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RUN_LED_Pin|GPIO_PIN_6, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET); //������� RDY ���� HIGH
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BUZ_OUT_GPIO_Port, BUZ_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : FA_RESET_Pin PWM2_H_Pin PWM1_H_Pin PA15 */
  GPIO_InitStruct.Pin = FA_RESET_Pin|PWM2_H_Pin|PWM1_H_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  


  /*Configure GPIO pin : dummy5_Pin */
  GPIO_InitStruct.Pin = dummy5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(dummy5_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RLY_M_SEL_Pin */
  GPIO_InitStruct.Pin = RLY_M_SEL_Pin|GPIO_PIN_2;//PE2 : ������� CS
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : START_SIG_Pin STOP_SIG_Pin EMS1_SIG_Pin */
  GPIO_InitStruct.Pin = START_SIG_Pin|STOP_SIG_Pin|EMS1_SIG_Pin;//PE12 : START_SIG_Pin, PE13 :STOP_SIG_Pin, PE14 :EMS1_SIG_Pin
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : RUN_LED_Pin PB6 PB7 */
  GPIO_InitStruct.Pin = RUN_LED_Pin|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = GPIO_PIN_7; //PB7 : //VOICE RDY
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;      
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

   GPIO_InitStruct.Pin = GPIO_PIN_6;		//PB6 : //VOICE RESET
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;      
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  /*Configure GPIO pins : dummy1_Pin dummy2_Pin */
  GPIO_InitStruct.Pin = dummy1_Pin|dummy2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : dummy3_Pin dummy4_Pin */
  GPIO_InitStruct.Pin = dummy3_Pin|dummy4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_6;//VOICE CS
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);  
  
  GPIO_InitStruct.Pin = GPIO_PIN_10;//SLEEP
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);  
  
  /*Configure GPIO pin : BUZ_OUT_Pin */
  GPIO_InitStruct.Pin = BUZ_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUZ_OUT_GPIO_Port, &GPIO_InitStruct);

}


/* USER CODE BEGIN 4 */

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

#ifdef  USE_FULL_ASSERT
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

static void MX_TIM8_Init(void)
{//pwm���� Ÿ�̸�
  /* USER CODE BEGIN TIM8_Init 0 */
	//TIM8's CLK : 72MHz
  /* USER CODE END TIM8_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 3600-1;	//72MHz/20KHz = 3600
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1800-1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 36;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}
