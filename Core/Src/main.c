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

#pragma location = 0x20000000 // LoRa Data를 저장하는 메모리 주소 값
struct LoRa_Data   _LoRa_Data;

#pragma location = 0x20000100 
struct SENSOR_DATA _SENSOR_DATA;//Sensor값들 Moterdriver -> LCD

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



//PID 제어 함수 관련 변수 선언
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
word acceltime = 300, deacceltime = 300;//처음 가속, 감속 시간은 300m였지만 감속되는 시간을 늘려줘 방향전환시 전류 노이즈 현상을 방지, acceltime을 줄여서 가속도달 시간을 단축
float upper_correction_angle  = 0.0, lower_correction_angle = 0.0;//각 운동마다 다름 감속 시작하는 구간이
dword set_upper_limit_time = 300, set_lower_limit_time = 300; //상한, 하한 정지시간 300

byte set_velocity_mode = 0;
byte move_to_lower = 0;
byte exerc_end = 0;	

//PID
#define Kp		0.025	//0.0001//0.025//0.7//0.3//0.7//0.025		//0.002//30//0.7//0.7//0.07//2///3/0.7//0.4//8//0.8//0.4//0.2//0.3//0.7//0.5             //0.7
#define Ki		20		//50//50//50//20 							//30//0.7//30//60//15//18//27//6.5//50//70//7//20//30//40//60//70//50//15//20.0//30
#define Kd		0
#define DT		0.01
#define PIconst	0.9



//EEPROM 1페이지당 16byte
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

//측정모드 관련
float test_rate;
byte skip_cnt_start = 0;
byte measurement_stop = 0, gen_ada_con_back = 0;//측정모드 음성관련 변수
byte CheckSum_EN = 0;
word LCD_crc = 0, MT_Driver_crc = 0;

word check_sum = 0;
byte PAUSE_F = 0;
byte Motor_BUSY = 0;

//셀프체크
byte Priority_dir_set = 0;
byte Error_Check = 0;	//모터드라이버 전원이 켜지면, 바로 셀프 체크 시작
byte shift_step = 0;	//에러체크시 PWM 가속 스텝
byte CCW_priority_mode = 0;	//반시계 방향 우선 체크 flag

float Bf_pm_avr = 0, Af_pm_avr = 0, Change_pm_avr = 0;	//각도센서 체크
float Bf_ct_avr = 0, Af_ct_avr = 0, Change_ct_avr = 0;	//전류센서 체크
int Bf_encoder_avr = 0, Af_encoder_avr = 0, Change_encoder_avr = 0;	//엔코더 체크
byte Encoder_dir = 0;

byte Passive_ing = 0;
//캘리브레이션각도

float real_angle = 0;
byte except_f = 0;	//각도제한 (집중모드의 상한각모드 예외처리)
//측정모드 테스트
byte Record_F = 0;

word bf_CNT = 0, Over_cnt = 0, Under_cnt = 0;
byte voice_error_check = 0;
byte Record_Index = 0, TEST_NUM = 0, Record_data_f = 0;
float Start_angle_1[100], Start_angle_2[100];
float End_angle_1[100], End_angle_2[100];
word Over_cnt_arr[100], Under_cnt_arr[100], Current_CNT_1[100], Current_CNT_2[100];

float Angle_by_encoder_up = 0,Angle_by_encoder_down = 0 ,OFFSET_angle = 0, OFFSET_angle_2 = 0;

/**블루투스 관련 변수**/
// 통신 상태 확인 변수
unsigned int bluetooth_AT = 0,cnt_flag = 0;; 

// HOST와 BT가 정상적으로 연결되어 있는지 여부를 확인. 
unsigned char AT_OK[10] = "AT\r\n"; 

unsigned int ble_name_change_fg = 0;

// HC-06 블루투스 모듈 이름 바꾸기_BT
unsigned char AT_BTNAME[21] = "AT+NAMEARTUS_840_00"; 

unsigned char Ble_buffer[50];

// RX 버퍼 배열
unsigned char RE_re_1[100]; 

// 어플로 보내는 데이터
unsigned char RE_data_tr_1[5]; 

// 어플로 보내는 데이터, 1초간격으로 1씩 증가한 값으로 데이터를 보냄
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
    // 사용자 정의된 마이크로초 지연 함수
    // 해당 함수는 약간의 오차가 있을 수 있습니다.
    volatile uint32_t counter = us * (SystemCoreClock / 1000000U) / 12U;
    while (counter--) {
        // 루프를 돌면서 대기
        asm("nop");
    }
}
word EEPROM_crc_rx_data = 0, compare_tx_rx = 0;
byte Mute_state = 1, eeprom_crc_not_ok = 0, Arr_eeprom1[16], Arr_eeprom2[16], Arr_eeprom3[16],Arr_eeprom4[16],Arr_eeprom5[16], bytes_data[4];
//Mute_state 1 : 출력 가능상태
byte EEPOM_Read(byte device_rd_address, byte Size, byte start_page)
{//EEPROM 읽기요청
	byte *bdata;
	if(start_page == 1)
	{//음성모듈 페이지 전용이며 나머지 페이지에서는 적용 x
		HAL_I2C_Mem_Read(&hi2c1, (device_rd_address << 1), (start_page - 1) * 16, I2C_MEMADD_SIZE_8BIT, EEPROM_rx_buf, Size, 10);// 7bit 범위를 가지며 1 ~ 128 7bit를 8비트로 만들어줘서 사용해야함
		Mute_state = EEPROM_rx_buf[2];//음소거 상태 0 : 음소거 상태, 1 : 출력 가능
	}
	else//나머지 페이지들
		HAL_I2C_Mem_Read(&hi2c1, (device_rd_address << 1), (start_page - 1) * 16, I2C_MEMADD_SIZE_8BIT, EEPROM_rx_buf, Size, 10);// 7bit 범위를 가지며 1 ~ 128 7bit를 8비트로 만들어줘서 사용해야함
	EEPROM_crc_rx_data = EEPROM_rx_buf[14];	    //CRC's Low byte
	EEPROM_crc_rx_data = EEPROM_crc_rx_data | (EEPROM_rx_buf[15] << 8);//CRC's High byte
	compare_tx_rx = CRC16(EEPROM_rx_buf, 14);	//읽은 14개 데이터의 CRC계산
	if(EEPROM_crc_rx_data == compare_tx_rx) //읽은 crc값 비교
	{
		eeprom_crc_not_ok = 0;//crc값 맞으면 OK
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
	if(start_page ==  2)//각도
	{//각도값에 이상이 생기면 default값 넣는 부분
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
		crc_write = 1;// 여기서 write
	}
	else if(start_page == 5)//전류
	{//전류값에 이상이 생기면 default값 넣는 부분
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
		EEPROM_Write(DEVICE_ADDRESS, start_page, Arr_eeprom5);	//(I2C주소, EEPROM페이지, Tx의 배열)	
	}
}
byte Eeprom_ok = 0;
word EEPROM_crc_tx_data = 0;
void EEPROM_Write(byte device_wr_address, byte start_page, byte *Wr_Data)
{//EEPROM 쓰기요청
	for(byte i = 0; i < 16; i++) //한 페이지당 16Byte
	{
		EEPROM_tx_buf[i] = Wr_Data[i]; 		
	}
	EEPROM_crc_tx_data = CRC16(EEPROM_tx_buf, 14);	//14개 데이터의 CRC계산 
	EEPROM_tx_buf[14] = EEPROM_crc_tx_data & 0xFF;	//CRC's Low byte	
	EEPROM_tx_buf[15] = (EEPROM_crc_tx_data >> 8);  //CRC's High byte	
	HAL_I2C_Mem_Write(&hi2c1, (device_wr_address << 1) + 1, (start_page - 1) * 16, I2C_MEMADD_SIZE_8BIT, EEPROM_tx_buf, Byte_num_16, 10);//I2C 통신을 사용하여 EEPROM에 데이터를 쓴다.
}
//word변환
typedef union 
{
    word word; // 32비트 정수를 저장하는 멤버
    word i;    // 32비트 정수를 저장하는 또 다른 멤버
} WordToUint32Converter;
word word_to_uint32(word value) 
{
    WordToUint32Converter converter; // 변환을 위한 유니온 변수 선언
    converter.word = value;          // 유니온의 word 멤버에 입력 값을 저장
    return converter.i;              // 유니온의 i 멤버를 반환
}
word bytes_to_word(byte *bytes_data) 
{
    union 
    {
        word w;        // word 타입의 멤버
        uint32_t i;    // 32비트 정수 멤버
    } converter;
    // 두 바이트를 32비트 정수로 합치는 작업
    converter.i = ((uint32_t)bytes_data[1] << 8) | (uint32_t)bytes_data[0];
    return converter.w; // word 타입의 멤버를 반환
}
///word변환
///float변환
union 
{
	float f;
	uint32_t i;
} converter;
// 부동 소수점 값을 4바이트 정수로 변환하는 함수
uint32_t float_to_uint32(float value) 
{
    converter.f = value;
    return converter.i;
}
///float변환
void int_to_bytes(uint32_t value, byte *bytes_data) 
{
    if (value <= 0xFFFF) 
	{ // value가 16비트 범위 내인 경우
        bytes_data[1] = (value >> 8) & 0xFF;
        bytes_data[0] = value & 0xFF;
    } 
	else 
	{ // value가 32비트인 경우
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
//음성모듈
byte  SPI_tx_buf[10], SPI_rx_buf[10]; 
byte Voice_error = 0;
byte Volume_step = 0; // 음성 모듈의 초기 값은 3단계
byte Settings=0,Settings_cancel=0,Settings_save=0 ,Voice_data,Setting_voice=0;//Setting_voice = 음성 1번 출력용도
byte  Voice_working=0, bf_Setting_voice=0, no_repeat = 0;  	
//음성모듈
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
		if(Volume_step == 0 || voice_cancel == 5 || Volume_save == 0)//기존 저장되어있는 볼륨값으로 돌려 놓음.
			SPI_tx_buf[2] = volume_state + 0x05;
		if(Volume_step)
		{
			SPI_tx_buf[2] = Volume_step + 0x05;//볼륨 값 변경하였을 경우 진입
		}
		if(Settings_save == 4 && Volume_save)
			SPI_tx_buf[2] = Volume_save + 0x05;
		
		if(volume_state < 0x1C && volume_state > 0x20)//볼륨 단계 설정 범위에 없을 경우 5단계로 고정되어 출력
			volume_state = 0x20;
		if((volume_state >= 0x1C && volume_state <= 0x20) || volume_state == 0x26)
		{				
			VOICE_CS_LOW;// 음성모듈 CS LOW
			HAL_SPI_Transmit(&hspi1, SPI_tx_buf, 3, 10);//3byte 출력
			VOICE_CS_HIGH;// 음성모듈 CS HIGH	
		}
	}
}
byte bf_voice_back = 0 , Setting_in = 0, Load_sensitivity_stage = 0;
void SPI_voice_set(void)
{
	EEPOM_Read(DEVICE_ADDRESS, Byte_num_16, 1);//(I2C의 주소, 한 페이지당 16bytye, 읽기 페이지주소)// 측정모드 감지 변화량, voice_step, 음소거 상태 순으로 저장되어있음.
	voice_status(); //볼륨 단계 초기설정하는 곳	
	EEPOM_Read(DEVICE_ADDRESS, Byte_num_16, 1);//(I2C의 주소, 한 페이지당 16bytye, 읽기 페이지주소)// 측정모드 감지 변화량, voice_step, 음소거 상태 순으로 저장되어있음.
	if(Settings == 0x03)
	{			
		Arr_eeprom1[0] = Load_sensitivity_stage;//감지 단계
		Mute_state = _VOICE_DATA.Mute_Flag;//음소거 상태저장		
		if((_VOICE_DATA.x06_1B >= 0x1C) && (_VOICE_DATA.x06_1B <= 0x20)) 
		   Volume_step = _VOICE_DATA.x06_1B;
		Setting_voice = 1;
		Voice_data = Settings;
		if(Volume_step)//볼륨 설정
		{//1단계 : 0x1C, 2단계 : 0x1D, 3단계 : 0x1E, 4단계 : 0x1F, 5단계 : 0x20
			bf_Setting_voice = 0;
			Voice_data = Volume_step;
			Volume_save = Volume_step;
			Arr_eeprom1[1] = Volume_step;//eeprom 0번 인덱스 볼륨데이터 값 바꿔줌
			voice_status();//볼륨 설정 여기서 볼륨의 단계를 설정해준다.
		}
		Setting_in = 1;
		if(Settings_save == 0x04)//설정을 저장합니다.
		{
			if(Arr_eeprom1[1] != 0 || (_VOICE_DATA.Mute_Flag != EEPROM_rx_buf[2]))//볼륨 값 설정하지 않고 바로 저장 버튼을 눌렀을 경우 EERPOM에 00을 Wirte하는 작업을 방지
			{
				Arr_eeprom1[2] = _VOICE_DATA.Mute_Flag;
				if(Arr_eeprom1[1] <= 0x20 && Arr_eeprom1[1] >= 0x1C)
					EEPROM_Write(DEVICE_ADDRESS, 1, Arr_eeprom1);//(I2C주소, EEPROM페이지, Tx의 배열)
			}
			Voice_data = Settings_save;//0x04넣어줌	
			voice_status();//볼륨 설정
			Mute_state = _VOICE_DATA.Mute_Flag;
			Volume_save = 0;
			Settings_save = 0;
			_VOICE_DATA.x06_1B = 0;
			_USER_RUN_SETTING.x06_1B = 0;
			Settings = 0;	
			bf_Setting_voice = 0;	
		}			
		if(Settings_cancel == 0x05)//설정을 취소합니다.
		{//설정 취소에 음성 볼륨을 원래 상태로 되돌리는 작업을 수행해야함 EEPRM에서 옴.	
			Voice_data = Settings_cancel;//0x05넣어줌	
			_USER_RUN_SETTING.x06_1B = 0;
			_VOICE_DATA.x06_1B = 0;
			bf_Setting_voice = 0;
			voice_cancel = 1;//기존 저장된 보이스 단계로 복구
			Settings_cancel = 0;
			_VOICE_DATA.Mute_Flag = 0;
			Settings = 0;	
		}
	}				
	switch(_VOICE_DATA.x06_1B)//0x06(운동모드) ~ 0x1B(과부하로 인해 정지되었습니다.)voice_en
	{
		case 0x06://운동 모드
			Voice_data = _VOICE_DATA.x06_1B;
			break;
		case 0x07://운동을 시작합니다.
			Voice_data = _VOICE_DATA.x06_1B;
			break;
		case 0x08://일반 운동
			Voice_data = _VOICE_DATA.x06_1B;
			break;
		case 0x09://적응 운동
			Voice_data = _VOICE_DATA.x06_1B;
			break;
		case 0x0A://집중 운동
			Voice_data = _VOICE_DATA.x06_1B;
			break;
		case 0x0B://적용위치 상한각
			Voice_data = _VOICE_DATA.x06_1B;
			break;
		case 0x0C://적용위치 하한각
			Voice_data = _VOICE_DATA.x06_1B;			
			break;
		case 0x0D://적용위치 상하한각
			Voice_data = _VOICE_DATA.x06_1B;
			break;
		case 0x0E://등속 모드
			Voice_data = _VOICE_DATA.x06_1B;			
			break;
		case 0x0F://가속 모드
			Voice_data = _VOICE_DATA.x06_1B;
			break;
		case 0x10://운동 시간
			Voice_data = _VOICE_DATA.x06_1B;
			break;
		case 0x11://운동 횟수
			Voice_data = _VOICE_DATA.x06_1B;	
			break;
		case 0x12://측정모드
			Voice_data = _VOICE_DATA.x06_1B;			
			break;
		case 0x13://측정을 시작합니다
			Voice_data = _VOICE_DATA.x06_1B;	
			break;
		case 0x14://측정을 정지합니다
			Voice_data = _VOICE_DATA.x06_1B;
			break;
		case 0x15://측정이 완료되었습니다
			Voice_data = _VOICE_DATA.x06_1B;	
			break;
		case 0x16://수동 모드
			Voice_data = _VOICE_DATA.x06_1B;	
			break;
		case 0x17://비상정지 되었습니다
			Voice_data = _VOICE_DATA.x06_1B;	
			break;
		case 0x18://현재각도가 설정범위를 벗어났습니다
			Voice_data = _VOICE_DATA.x06_1B;
			break;
		case 0x19://모터가 일시정지 되었습니다.
			Voice_data = _VOICE_DATA.x06_1B;
			break;
		case 0x1A://엔코더 연결을 확인해주세요
			Voice_data = _VOICE_DATA.x06_1B;
			break;
		case 0x1B://과부화로 인해 정지되었습니다.
			Voice_data = _VOICE_DATA.x06_1B;	
			break;	
		case 0x27://운동설정을 저장합니다.
			Voice_data = _VOICE_DATA.x06_1B;	
			break;	
		case 0x28://운동을 종료합니다.
			Voice_data = _VOICE_DATA.x06_1B;	
			break;
		case 0x29://운동설정을 취소합니다.
			Voice_data = _VOICE_DATA.x06_1B;	
			break;			
		case 0x2A://부하가 감지되었습니다.
			Voice_data = _VOICE_DATA.x06_1B;	
			break;	
		case 0x2B://비상정지 버튼을 해제하세요.
			Voice_data = _VOICE_DATA.x06_1B;	
			break;	
		case 0x2C://비상정지 버튼이 해제되었습니다.
			Voice_data = _VOICE_DATA.x06_1B;	
			break;					
	}
	if((Setting_voice == 1 && bf_Setting_voice == 0 && Voice_working == 0) || _USER_RUN_SETTING.x06_1B || Volume_step ||  _VOICE_DATA.x06_1B)
	{//출력부
		voice_out = 1;
	}
}
byte voice_out = 0;
void voice_output(void)
{//음성모듈 출력부
	SPI_tx_buf[0] = 0xB0;
	SPI_tx_buf[1] = 0x00;
	SPI_tx_buf[2] = Voice_data;	
	VOICE_CS_LOW;// 음성모듈 CS LOW
	if(Mute_state == 1 || _VOICE_DATA.Mute_Flag == 1)	//음소거 상태가 아니면 출력			
		HAL_SPI_Transmit(&hspi1, SPI_tx_buf, 3, 10);//3byte 출력
	VOICE_CS_HIGH;// 음성모듈 CS HIGH
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
	real_angle_point1 = _USER_RUN_SETTING.real_value;//210도		
	integer_angle_Value_2 = word_to_uint32(_CALIBRATION_SETTING.angle_ad_point_2);
	real_angle_point2 = _USER_RUN_SETTING.real_value;//170도		
	integer_angle_Value_3 = word_to_uint32(_CALIBRATION_SETTING.angle_ad_point_3);
	real_angle_point3 = _USER_RUN_SETTING.real_value;//-30도			
	integer_angle_Value_4 = word_to_uint32(_CALIBRATION_SETTING.angle_ad_point_4);
	real_angle_point4 = _USER_RUN_SETTING.real_value;//10도
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
 	_SENSOR_DATA.lora_wr_fg = 1; // LCD로 PC데이터(아래 LoRa 구조체 데이터) 전송, _SENSOR_DATA.lora_wr_fg가 0이면 LCD로 데이터를 전송X
	
    _SENSOR_DATA.user_name_1 = (_LoRa_Data.user_name_1<<8) | (_LoRa_Data.user_name_1>>8);
    _SENSOR_DATA.user_name_2 = (_LoRa_Data.user_name_2<<8) | (_LoRa_Data.user_name_2>>8);
	
    _SENSOR_DATA.user_name_3 = (_LoRa_Data.user_name_3<<8) | (_LoRa_Data.user_name_3>>8);
    _SENSOR_DATA.user_name_4 = (_LoRa_Data.user_name_4<<8) | (_LoRa_Data.user_name_4>>8);
	
    _SENSOR_DATA.user_name_5 = (_LoRa_Data.user_name_5<<8) | (_LoRa_Data.user_name_5>>8);
    _SENSOR_DATA.user_name_6 = (_LoRa_Data.user_name_6<<8) | (_LoRa_Data.user_name_6>>8);
	
	_SENSOR_DATA.patient_sex_age = (_LoRa_Data.user_sex << 7) | (_LoRa_Data.user_age & 0x7F); //MSB로 구분 남 : 0, 여 : 1 Ex) 여성 65세 -> 1100 0001 / 남성 65세 -> 0100 0001
	_SENSOR_DATA.upper_angle = _LoRa_Data.upper_angle;
	_SENSOR_DATA.lower_angle = _LoRa_Data.lower_angle;
	_SENSOR_DATA.upper_stop_time = (_LoRa_Data.stop_time >> 4) & 0x0F;  //   상한정지시간 : 상위 4bit, 하한정지시간 : 하위 4bit
	
	_SENSOR_DATA.lower_stop_time = _LoRa_Data.stop_time & 0x0F;	
	_SENSOR_DATA.mode = _LoRa_Data.mode;//운동 모드(0:일반등속 , 1:적응, 2:집중 3: 일반가속)
	_SENSOR_DATA.speed = _LoRa_Data.velocity_mode;//( 운동 속도 : 1 ~ 9단계 )	
	if((_LoRa_Data.exerc_time_and_exercnum & 0x80) != 0x80) // 운동시간
	{
	    _SENSOR_DATA.exerc_num = 0;
		_SENSOR_DATA.exerc_time = _LoRa_Data.exerc_time_and_exercnum & 0x7F;//(LSB : 1 ~ 99 윤동시간 또는 운동횟수 선택이며,  MSB가 0이면 운동시간, 1이면 운동횟수 )
	}
	else if((_LoRa_Data.exerc_time_and_exercnum & 0x80) == 0x80) // 운동횟수
	{
	  	_SENSOR_DATA.exerc_time = 0;
		_SENSOR_DATA.exerc_num = _LoRa_Data.exerc_time_and_exercnum & 0x7F;//(LSB : 1 ~ 99 윤동시간 또는 운동횟수 선택이며,  MSB가 0이면 운동시간, 1이면 운동횟수 )
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

// 전역 변수 선언
volatile uint32_t start_time = 0;
volatile uint32_t end_time = 0;
volatile uint32_t elapsed_time = 0;

void start_timer() 
{ // 타이머 카운터 초기화
  TIM5->CNT = 0;
  // TIM5 인터럽트 활성화
//  NVIC_EnableIRQ(TIM5_IRQn);
//  TIM5->DIER |= TIM_IT_UPDATE;
}

float stop_timer() 
{// TIM5 인터럽트 비활성화
//	NVIC_DisableIRQ(TIM5_IRQn);
//	TIM5->DIER &= ~TIM_IT_UPDATE;
	// 타이머 카운터 값 읽기
	end_time = TIM5->CNT;
	return end_time;
}

// 선형 보간을 수행하는 함수
float interpolate(uint16_t* angle_arr, int size, float P_M_avr, float real_angle_point4, float real_angle_point2, uint16_t angle_ad_point4, uint16_t angle_ad_point2, float* bf_pm_actual) 
{
    int lower_index = 0;
    int upper_index = size - 1;
    // 선형 탐색을 통해 하한과 상한 인덱스를 찾음
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


#define NUM_SAMPLES 5 // 전류값 저장 배열 크기
float current_history[NUM_SAMPLES] = {0}; // 전류값 저장 배열
byte current_index = 0; // 현재 저장할 인덱스
float baseline_current = 0; // 기준이 될 평균 전류 값
float Load_sensitivity = 0, relative_change = 0; // 부하 감지 민감도 (기준값 대비 40% 상승)

float calculate_slope(float values[], byte size)
{
    // 간단한 1차 회귀 기울기 계산
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

float purse1_down_1angle = 0.0;//정확도를 위해 float으로 해야함
//캘리브레이션 포인트 +10도, 170도 기준
word ad_count_down = 100, remain_ad = 0;
byte low_to_high = 0, anlge_cal_flag = 0, angle_record_flag = 0;
word low_to_high_pwm = 1800, high_to_low_pwm = 1800;//240도 범위를 이동
dword purse1_up_1angle = 0;//정확도를 위해 float으로 해야함
byte angle_recording_end = 0, calibration_state = 0, enable_low_to_high = 0;
float angle_real = 0;
byte high_to_low = 0, Save_final_ad = 0, eeprom_page_state = 0, read_ad_angle_flag = 0,Read_ad_angle_state = 0;
byte wait_high_to_low = 0, start_high_to_low = 0;
byte SYSTEM_INITIALIZATION_VALUE_ANGLE = 0;//시스템 처음 시작하면 1 그 다음부턴 사용할 일 없음. 0으로 만듦.
byte SYSTEM_INITIALIZATION_VALUE_CT = 0;
//측정모드 변수모음.
byte measure_restart = 0, restart_f = 0;
byte Benchmark = 0, Detect_load = 0;
byte Bf_Dir_of_rotation = 0, Change_dir = 0;
word EMS_start_cnt = 0;
byte read_sensitivity = 0,start_read_sense = 0;
//측정모드 변수모음.
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
	__HAL_AFIO_REMAP_SWJ_NOJTAG();//JTAG핀 비활성화 해야 clk핀 활성화 됨	
	//delayMicroseconds(10); // 100 마이크로초의 지연		
	MOTOR_ROTATE; 
	HAL_TIM_Base_Start_IT(&htim1);		//TIM1 interrupt start	
	HAL_TIM_Base_Start_IT(&htim2);		//TIM2 interrupt start		0.1m
	HAL_TIM_Base_Start_IT(&htim3);		//TIM3 interrupt start		1.0m
	HAL_TIM_Base_Start_IT(&htim5);		//TIM5 interrupt start		0.01m
	USART2->CR1 |= USART_CR1_RXNEIE;	//USART2 수신인터럽트 허용
	UART4->CR1 |= USART_CR1_RXNEIE;		//UART4 수신인터럽트 허용
	USART3->CR1 |= USART_CR1_RXNEIE;	//UART3 수신인터럽트 허용
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1);// 보완 채널 켜기		
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_1 | TIM_CHANNEL_2);//엔코더 타이머 start 
	//각도 체크	
	//ADC1의 AD변환 시작.
	HAL_Delay(1400);
	Read_ad_angle_state = 6;//각도 ad데이터를 eeprom으로부터 읽기 위한 코드 6페이지 부터 10페이지 까지 읽음	
	ADC1_index = 0;
	HAL_ADC_Start(&hadc1);
	//모터 구동 output, PWM핀 0으로 초기화
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);//GPIO 기본값 설정		   
	htim4.Instance->CCR1 = 0;  // duty 변경       
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);//GPIO 기본값 설정		
	htim4.Instance->CCR2 = 0;  // duty변경  
	_LoRa_Data.state = 1;//현재 상태(종료상태)
	_SENSOR_DATA.state = 1;//현재 상태(종료상태)
	Dir_of_rotation = STOP;		//운동중이 아닐때 초기값=>Stop
	GPIOC -> ODR |= 0x80;	// BUZZER_ON
	HAL_Delay(50);
	GPIOC -> ODR &= ~0x80;	// BUZZER_OFF		
	HAL_Delay(1000);		//처음에 셀프체크하기전에 각도,전류 AD값을 받아오기 위해 1초 기다림.
	bluetooth_AT = 1;
	SPI_tx_buf[0] = 0xB0;
	SPI_tx_buf[1] = 0x00;
	SPI_tx_buf[2] = 0x01;
	VOICE_CS_LOW;// 음성모듈 CS LOW
	HAL_SPI_Transmit(&hspi1, SPI_tx_buf, 3, 10);//3byte 출력
	VOICE_CS_HIGH;// 음성모듈 CS HIGH	
	VOICE_RESET_HIGH;
	HAL_Delay(5);
	VOICE_RESET_LOW;
	//EEPOM_Read(DEVICE_ADDRESS, Byte_num_16, 1);//(I2C의 주소, 한 페이지당 16bytye, 읽기 페이지주소)// 측정모드 감지 변화량, voice_step, 음소거 상태 순으로 저장되어있음.
	//Load_sensitivity = EEPROM_rx_buf[0];		
	PWM_NOT_SLEEP;//ON
	save_ct_end = 1;//전류센서 값을 eeprom에서 불러들임
	HAL_Delay(1400);
	save_angle_end = 1; //각도센서 값을 eeprom에서 불러들임
	HAL_Delay(1400);
	TIM5->DIER &= ~TIM_IT_UPDATE;//프로그램 소요시간을 측정하기 위한 비활성화 코드
	FA_RESET_L; // 블루투스 모듈 FA_RESET LOW
	HAL_Delay(100);
	FA_RESET_H; // 블루투스 모듈 FA_RESET HIGH
	LoRa_Equipment_Num = 1; // 모터드라이버 최초 전원 On시 LoRa_Equipment_Num이 1 딱 한번만 초기 국번 값을 모터드라이버에서 Slave_LoRa로 보낸다.
	while (1)
	{
		/*
		if(SYSTEM_INITIALIZATION_VALUE_CT)//EEPOM에 데이터가 없을 경우에 진입
		{
			SYSTEM_INITIALIZATION_VALUE_CT = 0;
			real_current_500mA = 500, real_current_2500mA = 2500, cal_500mA_val = 434, cal_2500mA_val = 2580;
		}
		*/
		//EEPOM_Read(DEVICE_ADDRESS, Byte_num_16, 1);//(I2C의 주소, 한 페이지당 16bytye, 읽기 페이지주소)// 장치번호, voice_step, 음소거 상태 순으로 저장되어있음.
		
		_SENSOR_DATA.load_sensitivity = Load_sensitivity_stage;
		
		if(EMS_SW)
		{			
			PWM_SLEEP;	
			if((exercise_mode >= 5) && (exercise_mode <=8))//수동모드
				_USER_RUN_SETTING.exerc_start = 0x03;//운동종료
			//09.20 도우가 함
			TX_EMS_CNT = 50; // EMS_STATE = 1 상태를 500ms 동안 유지
			if((_SENSOR_DATA.state == 0) && ((exercise_mode == 1) || (exercise_mode == 2) || (exercise_mode == 3)))//일반 적응 집중운동일 경우에 운동중인 상태에서 비상정지 눌르면 진입
			    pause_state = 2;	//일시정지	
			if(_SENSOR_DATA.state == 2)//하한각 이동중일 경우
				_SENSOR_DATA.state = 0x01; //대기중 상태
			Motor_BUSY = 0;
			if(Measurement_mode == 1)
				Detect_load = 1;//비상정지 시 임의로 부하감지해서	
			EMS_STATE = 1; // EMS 스위치가 눌렸을 때의 동작
			if(_SENSOR_DATA.state == 2)//하한각으로 이동중일 경우에 비상정지가 눌리면 운동종료 명령을 임시로 보낸다. 
				_USER_RUN_SETTING.exerc_start = 0x03;
		}
		if(EMS_STATE)
			_LoRa_Data.state = 0x02;//고장
		else if(_SENSOR_DATA.state == 1 && _LoRa_Data.measure_check != 1)
			_LoRa_Data.state = 1;	
		if(Load_sensitivity == 0 && angle_recording_end)
			read_sensitivity = 1;
		if(start_read_sense)
		{
			start_read_sense = 0;
			EEPOM_Read(DEVICE_ADDRESS, Byte_num_16, 1);//(I2C의 주소, 한 페이지당 16bytye, 읽기 페이지주소)// 측정모드 감지 변화량, voice_step, 음소거 상태 순으로 저장되어있음.
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
		if(_USER_RUN_SETTING.calibration == 0x02)//민감도 설정 캘리브레이션 모드일 경우에만 진입 하도록 설정
			start_sensing = 1;
		if(start_sensing && angle_recording_end)
		{
			start_sensing = 0;
			switch((byte)_USER_RUN_SETTING.real_value)
			{//측정모드 전류 부하 민감도 설정
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
			EEPROM_Write(DEVICE_ADDRESS, 1, Arr_eeprom1);//(I2C주소, EEPROM페이지, Tx의 배열)
			//read_sensitivity = 1;
		}
		if(_USER_RUN_SETTING.exerc_start == 0x0A)//측정모드 화면 나갈 경우에 0x0A 보냄
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
		static byte save_ad_arr[16] = {0}, arr_cnt = 0;// 1페이지당 14Byte + crc(2Byte)데이터 쓰기 가능
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
					EEPROM_Write(DEVICE_ADDRESS, 6, save_ad_arr);	//(I2C주소, EEPROM페이지, Tx의 배열), real각도 값					
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
					EEPROM_Write(DEVICE_ADDRESS, 7, save_ad_arr);	//(I2C주소, EEPROM페이지, Tx의 배열), real각도 값						
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
					EEPROM_Write(DEVICE_ADDRESS, 8, save_ad_arr);	//(I2C주소, EEPROM페이지, Tx의 배열), real각도 값						
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
					EEPROM_Write(DEVICE_ADDRESS, 9, save_ad_arr);	//(I2C주소, EEPROM페이지, Tx의 배열), real각도 값						
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
					EEPROM_Write(DEVICE_ADDRESS, 10, save_ad_arr);	//(I2C주소, EEPROM페이지, Tx의 배열), real각도 값						
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
					read_ad_err = EEPOM_Read(DEVICE_ADDRESS, Byte_num_16, 6);//(I2C의 주소, 한 페이지당 16bytye, 읽기 페이지주소)

					for(int i = 0; i < 7; i++) 
						angle_arr[i + 0] = (word)(EEPROM_rx_buf[i * 2] | (EEPROM_rx_buf[i * 2 + 1] << 8));
					Read_ad_angle_state = 7;
					if(read_ad_err)
					{
						//SYSTEM_INITIALIZATION_VALUE_ANGLE = 1;//읽기 오류가 생긴다면 임의의 값을 저장
						Read_ad_angle_state = 6;//다시읽음
						read_ad_err = 0;
						//eeprom_page_state = 6;// 6페이지부터 다시 씀
						break;
					}					
				}
				break;
			case 7://7page	
				wait_1400m_flag = 1;
				if(wait_1400m_start)
				{	
					wait_1400m_start = 0;					
					read_ad_err = EEPOM_Read(DEVICE_ADDRESS, Byte_num_16, 7);//(I2C의 주소, 한 페이지당 16bytye, 읽기 페이지주소)
		
					for(int i = 0; i < 7; i++) 
						angle_arr[i + 7] = (word)(EEPROM_rx_buf[i * 2] | (EEPROM_rx_buf[i * 2 + 1] << 8));
					Read_ad_angle_state = 8;
					if(read_ad_err)
					{
						//SYSTEM_INITIALIZATION_VALUE_ANGLE = 1;//읽기 오류가 생긴다면 임의의 값을 저장
						Read_ad_angle_state = 6;//다시읽음
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
					read_ad_err = EEPOM_Read(DEVICE_ADDRESS, Byte_num_16, 8);//(I2C의 주소, 한 페이지당 16bytye, 읽기 페이지주소)					
					for(int i = 0; i < 7; i++) 
						angle_arr[i + 14] = (word)(EEPROM_rx_buf[i * 2] | (EEPROM_rx_buf[i * 2 + 1] << 8));
					Read_ad_angle_state = 9;
					if(read_ad_err)
					{
						//SYSTEM_INITIALIZATION_VALUE_ANGLE = 1;//읽기 오류가 생긴다면 임의의 값을 저장
						Read_ad_angle_state = 6;//다시읽음
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
					read_ad_err = EEPOM_Read(DEVICE_ADDRESS, Byte_num_16, 9);//(I2C의 주소, 한 페이지당 16bytye, 읽기 페이지주소)
				
					for(int i = 0; i < 7; i++) 
						angle_arr[i + 21] = (word)(EEPROM_rx_buf[i * 2] | (EEPROM_rx_buf[i * 2 + 1] << 8));
					Read_ad_angle_state = 10;
					if(read_ad_err)
					{
						//SYSTEM_INITIALIZATION_VALUE_ANGLE = 1;//읽기 오류가 생긴다면 임의의 값을 저장
						Read_ad_angle_state = 6;//다시읽음
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
					read_ad_err = EEPOM_Read(DEVICE_ADDRESS, Byte_num_16, 10);//(I2C의 주소, 한 페이지당 16bytye, 읽기 페이지주소)
					
					for(int i = 0; i < 7; i++) 
						angle_arr[i + 28] = (word)(EEPROM_rx_buf[i * 2] | (EEPROM_rx_buf[i * 2 + 1] << 8));
					Read_ad_angle_state = 0;
					if(read_ad_err)
					{
						//SYSTEM_INITIALIZATION_VALUE_ANGLE = 1;//읽기 오류가 생긴다면 임의의 값을 저장
						Read_ad_angle_state = 6;//다시읽음
						read_ad_err = 0;
						break;
					}						
					angle_recording_end = 1;//각도 계산 시작
				}
				break;						
		}
		static word main_routine = 0;
		if(_VOICE_DATA.x06_1B == 0x06)
			main_routine = 0;
		main_routine++;		
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
		if(angle_cal_ad_1)//캘리
		{//캘리브레이션모드화면 첫 번째 체크 
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
		{//캘리브레이션모드화면 두 번째 체크 
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
		{//캘리브레이션모드화면 세 번째 체크 
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
		{//캘리브레이션모드화면 네 번째 체크 
			angle_1();
			angle_cal_ad_4 = 0;			
			int_to_bytes(integer_angle_Value_4, bytes_data);
			for(byte an = 6; an <= 7; an++)//2byte
				Arr_eeprom2[an] = bytes_data[an - 6];
			integer_angle_Value_4 = float_to_uint32(real_angle_point4);
			int_to_bytes(integer_angle_Value_4, bytes_data);			
			for(byte an = 0; an <= 3; an++)//4byte
				Arr_eeprom4[an] = bytes_data[an];
			EEPROM_Write(DEVICE_ADDRESS, 2, Arr_eeprom2);	//(I2C주소, EEPROM페이지, Tx의 배열), 각도 ad 값
			page1_end_flag = 1;//네 번째 화면까지 각도가 저장을 한다면 1페이지 write 작업 후에 1.4초동안 기다리고 2페이지 write하기 위한 플레그			
		}				
		if(page1_1400msec_wait)
		{//3페이지 write
			page1_1400msec_wait = 0;
			EEPROM_Write(DEVICE_ADDRESS, 3, Arr_eeprom3);	//(I2C주소, EEPROM페이지, Tx의 배열), real각도 값		
			page2_end_flag = 1;
		}
		if(page2_1400msec_wait)
		{//4페이지 write
			page2_1400msec_wait = 0;
			Arr_eeprom4[11] = 0x77;//EEPROM에 데이터를 썻다는 의미
			EEPROM_Write(DEVICE_ADDRESS, 4, Arr_eeprom4);	//(I2C주소, EEPROM페이지, Tx의 배열), real각도 값
			save_angle_flag = 1;
			
		}	
		if(save_angle_end)//save_angle_flag 변수가 1이 되고 1.4초 뒤에 save_angle_end 변수가 1
		{//캘리브레이션모드화면 마지막 체크	
			save_angle_end = 0;
			anlge_ad_err_flag = EEPOM_Read(DEVICE_ADDRESS, Byte_num_16, 2);//(I2C의 주소, 한 페이지당 16bytye, 읽기 페이지주소)			
			angle_ad_point1 = 0, angle_ad_point2 = 0, angle_ad_point3 = 0, angle_ad_point4 = 0, angle_ad_point5 = 0;	
			angle_ad_point1 = bytes_to_word(EEPROM_rx_buf + 0); //1point AD값
			angle_ad_point2 = bytes_to_word(EEPROM_rx_buf + 2); //2point AD값
			angle_ad_point3 = bytes_to_word(EEPROM_rx_buf + 4); //3point AD값	
			angle_ad_point4 = bytes_to_word(EEPROM_rx_buf + 6); //4point AD값
			//angle_ad_point5 = bytes_to_word(EEPROM_rx_buf + 8); //5point AD값
			save_angle_end_flag = 1;
		}
		if(real_angle_value_page2_end)
		{//save_angle_end_flag 변수가 1이 되고 1.4초 뒤에 진입
			real_angle_value_page2_end = 0;
			anlge_real_err_flag = EEPOM_Read(DEVICE_ADDRESS, Byte_num_16, 3);//(I2C의 주소, 한 페이지당 16bytye, 읽기 페이지주소)	
			real_angle_point1= 0, real_angle_point2 = 0, real_angle_point3 = 0;
			real_angle_point1 = bytes_to_float(EEPROM_rx_buf + 0);//1point 실제 각도 값 LCD로부터 읽어온 값임
			real_angle_point2 = bytes_to_float(EEPROM_rx_buf + 4);//2point 실제 각도 값 LCD로부터 읽어온 값임
			real_angle_point3 = bytes_to_float(EEPROM_rx_buf + 8);//3point 실제 각도 값 LCD로부터 읽어온 값임	
			Last_page = 1;
		}
		if(Last_page_end)	
		{//Last_page 변수가 1이 되고 1.4초 뒤에 진입
			Last_page_end = 0;
			anlge_real_err_flag = EEPOM_Read(DEVICE_ADDRESS, Byte_num_16, 4);//(I2C의 주소, 한 페이지당 16bytye, 읽기 페이지주소)	
			if(EEPROM_rx_buf[11] == 0x77)//EEPROM에 각도값이 저장 되어 있나 확인
				SYSTEM_INITIALIZATION_VALUE_ANGLE = 0;
			/*
			else 
				SYSTEM_INITIALIZATION_VALUE_ANGLE = 1;//이곳에 진입한 다는 것은 EEPROM에 비정상 데이터 or 데이터가 저장 되어있지 않다는 것을 의미한다.	
			*/			
			real_angle_point4 = 0, real_angle_point5 = 0;
			real_angle_point4 = bytes_to_float(EEPROM_rx_buf + 0);//4point 실제 각도 값 LCD로부터 읽어온 값임	
			//real_angle_point5 = bytes_to_float(EEPROM_rx_buf + 4);//5point 실제 각도 값 LCD로부터 읽어온 값임	
			enable_low_to_high = 1;
		}
		if(default_save_angle)//anlge_ad_err_flag, anlge_real_err_flag 둘 다 오류일 경우 진입
		{//이곳에 진입한다는 것은 crc에 오류가 생겼다는 의미
			default_save_angle = 0;
			crc_err_check(2);//각도값에 이상이 생기면 default값 넣는 부분
		}
		if((enable_low_to_high == 1) && (default_save_angle == 0) && (calibration_state == 1)) //calibration ing state
		{
			enable_low_to_high = 0;
			calibration_state = 0;//캘리브레이션 상황일 때만 low to high 하는 변수
			low_to_high = 1;
		}
		if(crc_write)
		{
			crc_write = 0;
			static byte crc_step = 0;
			switch(crc_step)
			{
				case 0:
					EEPROM_Write(DEVICE_ADDRESS, 2, Arr_eeprom2);	//(I2C주소, EEPROM페이지, Tx의 배열)	
					wait_1400m_flag = 1;
					if(wait_1400m_start)
					{
						wait_1400m_start = 0;
						crc_step = 1;
					}
					break;
				case 1:
					EEPROM_Write(DEVICE_ADDRESS, 3, Arr_eeprom3);	//(I2C주소, EEPROM페이지, Tx의 배열)	
					wait_1400m_flag = 1;
					if(wait_1400m_start)
					{
						wait_1400m_start = 0;
						crc_step = 2;
					}
					break;
				case 2:
					EEPROM_Write(DEVICE_ADDRESS, 4, Arr_eeprom4);	//(I2C주소, EEPROM페이지, Tx의 배열)	
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
			Arr_eeprom1[13] = 0x77; //EEPROM에 데이터 유무
			EEPROM_Write(DEVICE_ADDRESS, 5, Arr_eeprom1);	//(I2C주소, EEPROM페이지, Tx의 배열)		
			save_ct_flag = 1;
		}
		if(save_ct_end)//save_ct_flag 변수가 1이 되고 1.4초 뒤에 save_ct_end가 1
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
			cal_500mA_val = bytes_to_word(EEPROM_rx_buf + 0);     	//0.2A AD값
			cal_2500mA_val = bytes_to_word(EEPROM_rx_buf + 2);   		//2.0A AD값
			real_current_500mA= 0, real_current_2500mA = 0;
			real_current_500mA = bytes_to_float(EEPROM_rx_buf + 4);   //0.2A 실제 값 LCD로부터 읽어온 값임 
			real_current_2500mA = bytes_to_float(EEPROM_rx_buf + 8);   //2.0A도 실제 값 LCD로부터 읽어온 값임	
		}		
		if(default_save_ct)
		{
			ct_err_flag = 0;
			default_save_ct = 0;
			crc_err_check(5);//전류값에 이상이 생기면 default값 넣는 부분
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
			MOTOR_ROTATE;//높낮이 -> 모터
			_VOICE_DATA.x06_1B = 0;
			Settings_cancel = 5;
			if(cant_read_id == 0)//음성모듈 비정상
				Settings = 0;	
		}		
		if(_VOICE_DATA.x06_1B == 4 && Settings == 3)
		{
			Settings_save = 4;	
			MOTOR_ROTATE;//높낮이 -> 모터
			//voice_en = 1;
		}
		if((bf_voice_back != _VOICE_DATA.Volum_up_down) && _VOICE_DATA.Volum_up_down)
		{
			if(Setting_in == 0)//시스템을 처음 시작할 때 처음 환경설정 진입 시볼륨x단계 출력을 방지
				_VOICE_DATA.Volum_up_down = 0;			
			volum_count = _VOICE_DATA.Volum_up_down; // 05.08 주석 해제
			switch(volum_count)
			{
				case 0:
					//_VOICE_DATA.x06_1B = 0x26; //음소거
					break;							
				case 1:
					_VOICE_DATA.x06_1B = 0x1C; //1단계
					break;
				case 2:			
					_VOICE_DATA.x06_1B = 0x1D; //2단계
					break;
				case 3:			
					_VOICE_DATA.x06_1B = 0x1E; //3단계
					break;
				case 4:				
					_VOICE_DATA.x06_1B = 0x1F; //4단계
					break;
				case 5:
					_VOICE_DATA.x06_1B = 0x20; //5단계
					break;
			}
			bf_voice_back = _VOICE_DATA.Volum_up_down;
			_VOICE_DATA.Volum_up_down = 0;
		}
 
		/**블루투스 이름 변경 키**/
		
		if(ble_name_change_fg) // 블루투스 이름 변경 키
		{
		  ble_name_change_fg = 0;
		  //sprintf(Ble_buffer,"%s",AT_BTNAME_H);
		  for (int i = 0; AT_BTNAME[i] != '\0'; i++) 
		  {
			  Ble_buffer[i] = AT_BTNAME[i];//sprintf대신 사용하였음.
		  }
		  if(ENQ < 0x64) // 100보다 작으면
		  {
		  	Ble_Data_Tx(19);
		  }
		  else if(ENQ >= 0x64) // 100보다 크거나 같으면
		  {
			Ble_Data_Tx(20);
		  }
		}
		
		if(Ble_CheckSum_EN)
		{
		  Ble_CheckSum_EN = 0;
			if(Rx3_buf[0] == 'R' && Rx3_buf[1] == 'D') //읽기 함수
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
		/*_LoRa_Data.user_name_1 = plus_cnt;					// 0x2000_0000 : 0x0000 ~ 0xFFFF ( 환자 이름(성) ) 표준값 : 0xD64D( 홍 )
		_LoRa_Data.user_name_2 = plus_cnt - 1;				// 0x2000_0002 : 0x0000 ~ 0xFFFF ( 환자 이름(이름1) ) 표준값 : 0xAE38( 길 )
		_LoRa_Data.user_name_3 = plus_cnt - 2;				// 0x2000_0004 : 0x0000 ~ 0xFFFF ( 환자 이름(이름2) ) 표준값 : 0xB3D9( 동 )
		_LoRa_Data.user_name_4 = plus_cnt - 3;				// 0x2000_0006 : 0x0000 ~ 0xFFFF ( 환자 이름(이름3) )
		_LoRa_Data.user_name_5 = plus_cnt - 4;				// 0x2000_0008 : 0x0000 ~ 0xFFFF ( 환자 이름(이름4) )
		_LoRa_Data.user_name_6 = plus_cnt - 5;				// 0x2000_000A : 0x0000 ~ 0xFFFF ( 환자 이름(이름5) )		
		_LoRa_Data.user_age = plus_cnt - 6;					// 0x2000_000C : 1 ~ 120세 
		_LoRa_Data.user_sex = plus_cnt -7;					// 0x2000_000D	   : 남 : 0, 여 : 1
		_LoRa_Data.user_num_high = plus_cnt - 8;			// 0x2000_000E : 0 ( 환자 등록 번호 1 ~ 99999999 ) ( 상위 2byte ) 
		_LoRa_Data.user_num_low = plus_cnt - 9;				// 0x2000_0010 : 0 ( 환자 등록 번호 1 ~ 99999999 ) ( 하위 2byte ) 
		_LoRa_Data.upper_angle = plus_cnt - 10;				// 0x2000_0012 : 0 상한각 ( 0 ~ 230 통신후 실제 변수에 담기는 값은 -50 처리후 넣을것 ) ( -50 ~ 180 )
		_LoRa_Data.lower_angle = plus_cnt - 11;				// 0x2000_0013 : 0 하한각 ( 0 ~ 230 통신후 실제 변수에 담기는 값은  -70 처리후 넣을것 ) ( -70 ~ 160 )
		_LoRa_Data.stop_time = plus_cnt -12;				// 0x2000_0014 : ( 상한정지시간 : 상위 4bit, 하한정지시간 : 하위 4bit )
		_LoRa_Data.mode = plus_cnt - 13;					// 0x2000_0015 : 표준값은 : 0x00( 일반 등속 ), [ 0x00 : 일반(등속), 0x01 : 일반(가속), 0x02 : 적응, 0x03 : 집중 ]
		_LoRa_Data.velocity_mode = plus_cnt -14;			// 0x2000_0016 : 0 ( 운동 속도 : 1 ~ 9단계 )
		_LoRa_Data.exerc_time_and_exercnum = plus_cnt - 15;	// 0x2000_0017 : 0 ( 1 ~ 99 윤동시간 또는 운동횟수 선택이므로 MSB가 0이면 운동시간, 1이면 운동횟수 )
		_LoRa_Data.special_angle = plus_cnt - 16;			// 0x2000_0018 : 5 ~ 15 ( 집중/적응 운동 각도 최소 5°, 최대 15° )
		_LoRa_Data.repeat_num = plus_cnt - 17;				// 0x2000_0019 : 3 ~ 10 ( 집중/적응 운동 반복 횟수 최소 3회, 최대 10회 )
		_LoRa_Data.special_location = plus_cnt - 18;		// 0x2000_001A : 0 ~ 2( 상한각 : 0x00, 하한각 : 0x01, 상하한각 : 0x02 )
		_LoRa_Data.motion = plus_cnt -19;					// 0x2000_001B : 0 ~ 4 ( 표준값은 : 0x00 ( 어깨 ), [ 0x00 : 어깨, 0x01 : 팔꿈치, 0x02 : 무릎 , 0x03 : 손목, 0x04 : 발목 ] )*/
	
		/** Read Only **/
		/*_LoRa_Data.check_state = plus_cnt - 20;             // 0x2000 001C : 0 에러체크 완료 : 0x00, 전류 에러체크 진행중 : 0x02, 엔코더 에러체크 진행중 : 0x04, 각도 에러체크 진행중: 0x08
		_LoRa_Data.state = plus_cnt - 21;					// 0x2000_001D : 주기적으로 통신(통신 주기 미정) / 대기중 : 0x00, 사용중 : 0x01, 고장 : 0x02, 연결실패 : 0x03
		_LoRa_Data.use_state = plus_cnt - 22;				// 0x2000_001E : 운동중 : 0x00, 정지 : 0x01, 측정중 : 0x02, 수동 : 0x03
		_LoRa_Data.error_state = plus_cnt - 23;				// 0x2000_001F : 0 ( 0 ~ 5, 비상정지 : 0x01, 과전류 : 0x02, 엔코더 : 0x03, 각도이탈 : 0x04, 통신에러 : 0x05 )
		_LoRa_Data.remain_time_and_cnt = plus_cnt -24; 		// 0x2000_0020 : 0 ( 0 ~ 99 남은 운동시간 또는 남은 운동횟수 선택이므로 MSB가 0이면 남은 운동시간, 1이면 남은 운동횟수 )
		_LoRa_Data.upper_limit_angle = plus_cnt -25;		// 0x2000_0021 : 0 측정 상한각 ( 0 ~ 230 통신후 실제 변수에 담기는 값은 -50 처리후 넣을것 ) ( -50 ~ 180 )
		_LoRa_Data.lower_limit_angle = plus_cnt -26;		// 0x2000_0022 : 0 측정 하한각 ( 0 ~ 230 통신후 실제 변수에 담기는 값은  -70 처리후 넣을것 ) ( -70 ~ 160 )
		_LoRa_Data.rsv1 = 0;								// 0x2000_0023 : 0( Dummy_Data1 )*/
	
		/**PC와 통신 하기 전 구조체에 데이터를 넣는 부분( 테스트 용 ) **/
		
		//데이터 수신후, CRC16 값 비교// LoRa와 Motor Driver사이의 통신
		if(LoRa_CheckSum_EN) // UART2 Interrupt root에서 정상적인 프로토콜 데이터 수신시 1
		{
		  word LoRa_w_check;
		  static byte LoRa_cal_number;
		  LoRa_CheckSum_EN = 0;
		  
		  if(LoRa_Rxbuf[1] == 0x04) //읽기 함수
		  {
			LoRa_cal_number = 6; // LoRa_Rxbuf[0] ~ LoRa_Rxbuf[5] 까지의 6byte CRC16 계산
		  }
		  else if(LoRa_Rxbuf[1] == 0x10) // 쓰기 함수
		  {
			LoRa_cal_number = LoRa_Rxbuf[5]*2 + 7; // // LoRa_Rxbuf[5]*2 + 7 개의 CRC16 계산
		  }
		  
		  LoRa_CheckSum_data = CRC16(LoRa_Rxbuf, LoRa_cal_number); // 수신한 데이터의 CRC16 계산
		  
		  LoRa_CheckSum_ing = 0; // UART2 수신가능
		  LoRa_w_check = (Rx2_CRC_H<<8) | Rx2_CRC_L;	
		  LoRa_test_crc1 = LoRa_w_check;
		  LoRa_test_crc2 = LoRa_CheckSum_data;
		  
		  if(LoRa_CheckSum_data == LoRa_w_check)	//CRC16 OK!
		  {
			  if(LoRa_Rxbuf[1] == 0x10)	//쓰기 ACK
			  {
				  LoRa_Rcv_ok = 0x10;
				  LoRa_RS485_dead_time = LORA_RS485_DEAD_TIME;	//15ms delay후 응답처리, 1msec마다 1씩 감소하여 0일때 Rcv5_ok = 0x01;
			  }
			  else if(LoRa_Rxbuf[1] == 0x04)	//읽기 ACK
			  {
				  LoRa_Rcv_ok = 0x04;
				  LoRa_RS485_dead_time = LORA_RS485_DEAD_TIME;	//15ms delay후 응답처리, 1msec마다 1씩 감소하여 0일때 Rcv5_ok = 0x44;
			  }
		  }
		  else
		  {
			  LoRa_Rcv_ok = 0x0E;
			  LoRa_CheckSum_data = CRC16(LoRa_Rxbuf, LoRa_cal_number);	//수신한 데이터의 CRC계산	
			  LoRa_RS485_dead_time = LORA_RS485_DEAD_TIME;	//15ms delay후 응답처리, 1msec마다 1씩 감소하여 0일때 Rcv5_ok = 0x01;
		  }
		}
		LoRa_UART2_exe(); 
		/*****************************************추가한 부분(도우)*************************************************/
		//데이터 수신후, Check Sum 체크(23.01.11, CRC버그로 인해 Check Sum으로 대체) // Motor Driver랑 LCD 사이의 통신
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
			MT_Driver_crc = CRC16(Rx4_buf, cal_number);//수신한 데이터의 CheckSum계산	
			CheckSum_ing = 0;
			LCD_crc = (Rx4_CRC_H<<8) | Rx4_CRC_L;	
			if(MT_Driver_crc == LCD_crc)//CheckSum OK!
			{
				if(Rx4_buf[1] == 0x10)	//쓰기 ACK
				{
					Rcv2_ok = 0x10;
					//RS485_dead_time = 100;	//100ms delay후 응답처리, 1msec마다 1씩 감소하여 0일때 Rcv5_ok = 0x110;
				}
				else if(Rx4_buf[1] == 0x04)	//읽기 ACK
				{
					Rcv2_ok = 0x04;
					//RS485_dead_time = 100;	//100ms delay후 응답처리, 1msec마다 1씩 감소하여 0일때 Rcv5_ok = 0x44;
				}
				else if(Rx4_buf[1] == 0x06)	//Calibration ACK
				{
					Rcv2_ok = 0x06;
					//RS485_dead_time = 100;	//100ms delay후 응답처리, 1msec마다 1씩 감소하여 0일때 Rcv5_ok = 0x66;
				}
				else if(Rx4_buf[1] == 0x08)	//음성모듈 ACK
				{
					Rcv2_ok = 0x08;
					//RS485_dead_time = 100;	//100ms delay후 응답처리, 1msec마다 1씩 감소하여 0일때 Rcv5_ok = 0x66;
				}				
			}
			else
			{
				Rcv2_ok = 0x0E;
				CheckSum_data = CRC16(Rx4_buf, cal_number);	//수신한 데이터의 CRC계산	
				//RS485_dead_time = 100;	//100ms delay후 응답처리, 1msec마다 1씩 감소하여 0일때 Rcv5_ok = 0x55;
			}
		}
		UART4_exe(); 	//UART2 RS232
		static byte start_selfcheck_flag = 0;
		static byte ems_start = 0;
		//셀프체크 방향 결정(설정 가능 최대, 최소 각도를 벗어나지 않고 셀프체크하기 위함)


		if(Priority_dir_set == 'S')	//'S' => Start
		{
			ems_start = 1;
			if(EMS_STATE)
			{
				_VOICE_DATA.x06_1B = 0x2B;//비상정지버튼을 해제하세요. 음성출력
			}
			else if(EMS_STATE == 0)
			{
				Priority_dir_set = 'E';	//다시 셀프체크 시작하는 것을 방지.	//'E' => End
				switch(_USER_RUN_SETTING.motion)
				{		
					case Elbow:	    //팔꿈치
						if(Current_angle >= 115) 	//115도 이상일때
							CCW_priority_mode = 1;	//CCW 먼저 체크
						else
							CCW_priority_mode = 0;	//CW 먼저 체크
						break;
					case Shoulder:	//어깨
						if(Current_angle >= 160) 	//160도 이상일때
							CCW_priority_mode = 1;	//CCW 먼저 체크
						else
							CCW_priority_mode = 0;	//CW 먼저 체크
						break;							
					case Knee:	    //무릎
						if(Current_angle >= 120) 	//120도 이상일때
							CCW_priority_mode = 0;	//CW 먼저 체크
						else
							CCW_priority_mode = 1;	//CCW 먼저 체크
						break;
					case wrist:	    //손목
						if(Current_angle >= 50) 	//도 이상일때
							CCW_priority_mode = 1;	//CCW 먼저 체크
						else
							CCW_priority_mode = 0;	//CW 먼저 체크
						break;
					case ankle:	    //발목
						if(Current_angle >= 30) 	//도 이상일때
							CCW_priority_mode = 1;	//CCW 먼저 체크
						else
							CCW_priority_mode = 0;	//CW 먼저 체크
						break;					
				}
				start_selfcheck_flag = 1;
			}
		}
		
		if(start_selfcheck_flag && _USER_RUN_SETTING.calibration == 0)
		{
			PWM_NOT_SLEEP;
			start_selfcheck_flag = 0;
			Error_Check = 1;//셀프체크 시작
		}
		// 민수형이 예전에 하신거 비상정지 버튼 버그 있으면 이거 주석 풀것 아래 도우가 함 지우고!!!!
		/*if(EMS_STATE && Priority_dir_set != 'S' && ems_start)//Emergency Stop SW ON!
		{
	  		_LoRa_Data.error_state |= 0x01;// 비상정지 에러 화면 표시 24.04.01 수정 	
			_SENSOR_DATA.ESB_state |= 0x01;// 비상정지 에러 화면 표시 24.02.28 추가 
			_VOICE_DATA.x06_1B = 0x17; //비상정지 되었습니다. 음성출력
		}
		else
		{
	  		_LoRa_Data.error_state &= ~0x01;// 비상정지 에러 화면 해제 24.04.01 수정 	
			_SENSOR_DATA.ESB_state &= ~0x01;// 비상정지 에러 화면 해제 24.02.28 추가 
		}*/
		//도우가 한것
		static int ems_sound = 0;
		if(EMS_STATE)//Emergency Stop SW ON!
		{
			total_low_angle = 0, total_high_angle = 0;
	  		_LoRa_Data.error_state |= 0x01;// 비상정지 에러 화면 표시 24.04.01 수정 	
			_SENSOR_DATA.ESB_state |= 0x01;// 비상정지 에러 화면 표시 24.02.28 추가 
			//도우가함.
			if(++ems_sound >= 50000)
			{
			  ems_sound = 50000;
			  _VOICE_DATA.x06_1B = 0x17; //비상정지 되었습니다. 음성출력
			}
		}
		else
		{
	  		_LoRa_Data.error_state &= ~0x01;// 비상정지 에러 화면 해제 24.04.01 수정 	
			_SENSOR_DATA.ESB_state &= ~0x01;// 비상정지 에러 화면 해제 24.02.28 추가 
			if(ems_sound == 50000)
			{
				ems_sound = 0;
				_VOICE_DATA.x06_1B = 0x2C;//"비상정지 버튼이 해제되었습니다." 음성출력
			}
		}
		
	  	/***** TIMER *****/
		if(T10ms_flag == 1)	
		{//10ms task
			T10ms_flag = 0;
			Exercise_ctrl();	//운동 모드 실행 부분(일반, 적응, 집중)
			/******************************셀프 체크******************************/
			if(!Self_check_end)	//셀프체크중
			{
				if(Self_Check == 0)	//Self Check 기능이 OFF되어 있으면 임의로 정상 처리
				{
					//임의로 정상 처리
					_SENSOR_DATA.PM_error = 1;		//정상
					_SENSOR_DATA.CT_error = 1;		//정상
					_SENSOR_DATA.encoder_error = 1;	//정상
				}
				else
				{
					_LoRa_Data.check_state |= 0x02;//전류센서 에러체크 중 24.02.28 수정 Para.CT_error = 0;	
					_SENSOR_DATA.CT_error = 0x00;//전류센서 에러체크 중
					_LoRa_Data.check_state |= 0x04;//엔코더 에러체크 중 24.02.28 수정 Para.encoder_error = 0;	
					_SENSOR_DATA.encoder_error = 0x00;//엔코더 에러체크 중 
					_LoRa_Data.check_state |= 0x08;//각도센서 에러체크 중 24.02.28 수정 Para.PM_error = 0;
					_SENSOR_DATA.PM_error = 0x00;//각도센서 에러체크 중
				}
			}
			else if(Self_check_end == 1 && Self_check_end)//셀프체크 완료
			{
				_LoRa_Data.check_state = 0;//셀프체크 끝나서 초기화 시켜줌
				if(Self_Check == 0)	//Self_Check가 0:셀프체크 기능 OFF, 1:셀프체크 기능 ON
				{
					//임의로 정상 처리
					_SENSOR_DATA.PM_error = 1;		//정상
					_SENSOR_DATA.CT_error = 1;		//정상
					_SENSOR_DATA.encoder_error = 1;	//정상
				}
				else
				{			
					if(_SENSOR_DATA.ESB_state == 0)
						_SENSOR_DATA.EMS_error = 1;//비상정지 해제
					else 
					{
						if(++EMS_start_cnt >= 800)
						{
							EMS_start_cnt = 0;
							_SENSOR_DATA.EMS_error = 2;//비상정지 
						}
					}
					if(cant_read_id == 0)
					{
						voice_error_check = 1;//음성모듈 에러체크
					}
					if(ADC1_data[1] == 0)	//각도센서가 뽑혔을 때
					{
						_LoRa_Data.error_state |= 0x08;//각도센서 에러
						_SENSOR_DATA.PM_error = 2;//각도센서 에러
					}
					else
					{
						if((CW_pm_check_ok + CCW_pm_check_ok) == 2)
						{
							_LoRa_Data.error_state &= ~0x08;//각도센서 정상
							_SENSOR_DATA.PM_error = 1;//각도센서 정상
						}
						else
						{
							_LoRa_Data.error_state |= 0x08;	//각도센서 에러
							_SENSOR_DATA.PM_error = 2;//각도센서 에러
						}
					}
					if((CW_ct_check_ok + CCW_ct_check_ok) == 2)
					{
						_LoRa_Data.error_state &= ~0x02;//전류센서 정상
						_SENSOR_DATA.CT_error = 1;//전류센서 정상
					}
					else
					{
						_LoRa_Data.error_state |= 0x02;	//전류센서 비정상
						_SENSOR_DATA.CT_error = 2;//전류센서 비정상
					}
					if((CW_encoder_check_ok + CCW_encoder_check_ok) == 2)
					{
						_LoRa_Data.error_state &= ~0x04;//엔코더 정상
						_SENSOR_DATA.encoder_error = 1;
					}
					else
					{
						_LoRa_Data.error_state |= 0x04;//엔코더 에러
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
		if(error_touch)//엔코더 or 포텐셜 or 전류 하나라도 오류가 생길 경우 에러 화면 생성 -> 화면 터치시 진입
		{
			error_touch = 0;
			CW_pm_check_ok = 0;CCW_pm_check_ok = 0;CW_ct_check_ok = 0;CCW_ct_check_ok = 0;
			CW_encoder_check_ok = 0;CCW_encoder_check_ok = 0;			
			PWM_NOT_SLEEP;
			_SENSOR_DATA.EMS_error = 0;
			_SENSOR_DATA.CT_error = 0;
			_SENSOR_DATA.PM_error = 0;
			_SENSOR_DATA.encoder_error = 0;
			Restart_self_check = 1;//셀프체크 다시시작
		}
	}//while문 닫힘
  /* USER CODE END 3 */
}//main문 닫힘
/*********************************************************************************/
/*******************************모터 구동 관련 함수************************************/

//PID 제어 함수 관련 변수 선언
//PID 컨트롤				//PID제어에서 PI제어로 변경 및 I제어 코드 번경(22.11.18) 
float PID_control_sys(float target, float current) 
{
	//편차(에러) 계산
	error = target - current;				//편차		p
    accError += error;// + pre_error;		//편차의 누적	i
	//errorGap = pre_error - error;         //이전 편차와 현재 편차의 차	d
	
	
	if(accError > 3600)
		accError = 3600;
	else if(accError < -3600)
		accError = -3600;
	
	pControl = P_GAIN * error;				//P(비례항) 제어기
	iControl = I_GAIN * (accError * TIME);	//I(적분항) 제어기
	//dControl = D_GAIN * (errorGap / TIME);	//D(미분항) 제어기
	
	//pre_error = error;		//편차의 누적을 위해 과거값을 저장
	pidControl = pControl + iControl;// + dControl;	//PID 제어기
	return pidControl;
}

// MPC 관련 변수 선언
float A = 1.0;    // 시스템의 상태 전이 행렬 (여기서는 간단히 1로 가정)
float B = 1.0;    // 입력에 대한 시스템의 반응 계수
float Q = 1.0;    // 출력 오차에 대한 가중치
float R = 0.1;    // 제어 입력에 대한 가중치

float u = 0;      // 제어 입력
float x = 0;      // 시스템 상태 (출력)

float predictHorizon = 10; // 예측 시간 구간
float controlHorizon = 1;  // 제어 시간 구간

// MPC 제어 함수
float MPC_control_sys(float target, float current) 
{
    float error = target - current;   // 현재 오차 계산
    float futureX = current;          // 미래 상태 예측 초기화
    float optimalControl = 0;
    float minCost = 1000000;          // 최소 비용 초기화 (큰 값으로 시작)

    // 여러 제어 입력을 시도하여 최적의 제어 신호를 찾음
    for (float candidateU = -1; candidateU <= 1; candidateU += 0.1) 
    {
        float cost = 0;
        futureX = current;

        // 예측 구간 동안의 비용 계산
        for (int t = 0; t < predictHorizon; t++) 
        {
            futureX = A * futureX + B * candidateU;  // 미래 상태 예측
            float futureError = target - futureX;    // 미래 오차 계산
            cost += Q * futureError * futureError + R * candidateU * candidateU; // 비용 계산
        }

        // 최소 비용을 갖는 제어 입력 선택
        if (cost < minCost) 
        {
            minCost = cost;
            optimalControl = candidateU;
        }
    }

    u = optimalControl;  // 최적의 제어 입력 적용
    x = A * x + B * u;   // 현재 상태 업데이트

    return u;
}






//모터 PWM 설정(PID)
float set_motor_pwm(float PID_Value)
{
	if(Dir_of_rotation == CW)				//시계방향
		PID_Value = TIM4_CCR2_buffer + PID_Value;
	else if(Dir_of_rotation == CCW)	//반시계 방향
		PID_Value = TIM4_CCR1_buffer + PID_Value;
		//PID_Value = (htim4.Instance->CCR1) + PID_Value;
	else
		return 0;
	//예외처리, PWM 최소,최대 값 사이의 범위를 벗어날 경우
	if(PID_Value > 3600)
		PID_Value = 3600;
	else if(PID_Value < 0)
		PID_Value = 0;
	
	return PID_Value;
}
					    //   가속 시간,		   감속 시간, 		  최대속도(단계)	   	속도모드
void Soft_start_stop_PI(word accel_time, word deaccel_time, byte speed_step, byte velocity_mode)	//accel_time = 100*10ms은 1초, Call by 10ms Task
{	
	static int set_rpm, accel_rpm;
	//22.12.19. 각도보정을 위한 속도조절, 이동각이 15도 미만일 경우 각도 이탈하는 문제 해결을 위함
	if((DISTANCE < 15) && (speed_step >= 5))//23.02.10 이탈방지 적용 속도단계를 6단계이상에서 5단계이상으로 수정
		speed_step = speed_step - 2;
	if(_USER_RUN_SETTING.motion == Knee)//무릎운동
		upper_correction_angle  = 7.0, lower_correction_angle = 6.0;	
	else
		upper_correction_angle  = 9.0, lower_correction_angle = 7.0;
	switch(speed_step)//속도단계에 따른 모터 등속,가속 rpm 설정(9단계까지 설정가능)
	{				  //DC모터와 기구물의 기어비 = 1500:1
		case 0:	//측정모드 일시정지용(23.06.23)
			set_rpm = 0;
			break;
		case 1:
			set_rpm = 460;	//기구물 RPM = 0.277
			accel_rpm = 700;
			break;
		case 2:
			set_rpm = 665; //기구물 RPM = 0.37
			accel_rpm = 850;
			break;
		case 3:
			set_rpm = 830; //기구물 RPM = 0.463
			accel_rpm = 1050;
			break;
		case 4:
			set_rpm = 1015; //기구물 RPM = 0.556
			accel_rpm = 1300;
			break;
		case 5:
			set_rpm = 1200; //기구물 RPM = 0.639//105
			accel_rpm = 1400;
			break;
		case 6:
			set_rpm = 1340; //기구물 RPM = 0.789
			accel_rpm = 1580;
			break;
		case 7:
			set_rpm = 1460; //기구물 RPM = 0.833//140
			accel_rpm = 1850;
			break;
		case 8:
			set_rpm = 1615; //기구물 RPM = 0.917
			accel_rpm = 2100;
			break;
		case 9:
			set_rpm = 1760; //기구물 RPM = 1.0
			accel_rpm = 2300;
			break;
		default:
			break;
	}
	static byte cal_accel_angle_f;
	if(deaccel_flag == 0)
	{ 
		if(speed_step == 0)		//speed_step이 0일때는 모터속도는 0 예외처리
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
			Motor_PWM = (unsigned int)set_motor_pwm(pid_val);	//모터 PWM PID 제어
			

			static word test_kcnt2 = 0;
			test_kcnt2++;
		}
*/		

		
		target_speed = ((set_rpm-240)/(float)accel_time)*pwm_step + 240;  //Soft Start Soft Stop[soft stop의 최소속도는 240(duty=15%)]
		pid_val = PID_control_sys(target_speed, speed_RPM);
		Motor_PWM = (unsigned int)set_motor_pwm(pid_val);	//모터 PWM PID 제어
		if(pwm_step < accel_time)	//가속 step이 설정한 가속시간보다 낮을 경우 가속 pwm_step 1씩증가
		{
			pwm_step++;
			if(pwm_step == accel_time-1)
				cal_accel_angle_f = 1;
		}
		static byte direction_1, direction_2;
		//무릎기구 방향 및 각도 조정 테스트 시작(23.02.02)
		if(_USER_RUN_SETTING.motion == Knee)	//무릎
		{
			direction_1 = CCW;
			direction_2 = CW;
		}
		else	//어깨, 무릎
		{
			direction_1 = CW;
			direction_2 = CCW;
		}
		if(target_speed >= set_rpm) //설정속도가 될 때
		{
			if(cal_accel_angle_f == 1)	//설정속도가 되면 가속한 구간 계산
			{
				if(Dir_of_rotation == direction_1)	  //시계방향
					accel_angle = DISTANCE - angle_distance + upper_correction_angle;	//최고속도까지 도달하는 동안 이동한 각도 계산
				else if(Dir_of_rotation == direction_2) //반시계방향
					accel_angle = DISTANCE - angle_distance + lower_correction_angle;	//최고속도까지 도달하는 동안 이동한 각도 계산
				cal_accel_angle_f = 0;
			}
			if(velocity_mode == CONSTANT_MODE)	//등속모드
			{	
				if(exercise_mode == 1)
				   _Ble_Data.mode =  0x00;//일반 등속운동
				pid_val = PID_control_sys(target_speed, speed_RPM);
				Motor_PWM = (unsigned int)set_motor_pwm(pid_val);	//모터 PWM PID 제어
				if(angle_distance <= accel_angle)	//감속조건 달성
				{
					deaccel_flag = 1;		 		//감속 flag  ON
				}
			}
			else if(velocity_mode == ACCEL_MODE)  //가속모드
			{			
				if(exercise_mode == 1)
				   _Ble_Data.mode = 3;//일반가속				
				if(angle_distance <= accel_angle)	//감속조건 달성
					deaccel_flag = 1;		 		//감속 flag  ON
				else
				{
					if((Current_angle >= set_min_angle) && (Current_angle < set_min_angle+(ROM*0.25)))	//총 운동범위(ROM)의 하위 25% 구간
					{
						target_speed = set_rpm;	//설정속도 유지
						_LoRa_Data.mode = 0;//Para.velocity_mode = 0; 구조체 헷갈릴 것 주의 mode가 맞음. 24.04.08 수정
						_SENSOR_DATA.velocity_mode = 0;//등속 24.04.08 추가하였음.
					}
					else if( (Current_angle >= set_min_angle+(ROM*0.25)) && (Current_angle < set_max_angle-(ROM*0.25)) ) //총 운동범위(ROM)의 하위 25%, 상위 25% 사이 구간
					{	
						target_speed = accel_rpm;	//설정속도 + 2 단계 => 가속운동구간
						_LoRa_Data.mode = 1;//Para.velocity_mode = 1; 구조체 헷갈릴 것 주의 mode가 맞음. 24.04.08 수정
						_SENSOR_DATA.velocity_mode = 1;//가속 24.04.08 추가하였음.
					}
					else if( (Current_angle >= set_max_angle-(ROM*0.25)) && (Current_angle <= set_max_angle) )	//총 운동범위(ROM)의 상위 25% 구간
					{
						target_speed = set_rpm;	//설정속도 유지
						_LoRa_Data.mode = 0;//Para.velocity_mode = 0; 구조체 헷갈릴 것 주의 mode가 맞음. 24.04.08 수정
						_SENSOR_DATA.velocity_mode = 0;//등속 24.04.08 추가하였음.
					}
					pid_val = PID_control_sys(target_speed, speed_RPM);
					Motor_PWM = (unsigned int)set_motor_pwm(pid_val);	//모터 PWM PID 제어
				}
			}
		}
	}
	else//deaccel_flag == 1
	{
		if(speed_step == 1 || speed_step == 2)
		{
			if((pwm_step -= 2) > deaccel_time)	//step이 0밑으로 떨어져 65535 되는걸 방지, 가속 step이 설정한 가속시간보다 높을 경우 가속 pwm_step 2씩감소
				pwm_step = 0;	
		}
		else if(--pwm_step > deaccel_time)	//step이 0밑으로 떨어져 65535 되는걸 방지, 가속 step이 설정한 가속시간보다 높을 경우 가속 pwm_step 1씩감소
			pwm_step = 0;		
		target_speed = ((set_rpm - 240)/(float)deaccel_time)*pwm_step + 240; 	 //Soft Stop[soft stop의 최소속도는 240(duty=15%)]
		pid_val = PID_control_sys(target_speed, speed_RPM);
		Motor_PWM = (unsigned int)set_motor_pwm(pid_val);	//모터 PWM PID 제어
	}
}

//측정모드 관련 변수
float High_angle = 0, Lower_angle = 0, High_angle_correction = 0, Lower_angle_correction = 0;
byte Measurement_mode = 0, Measure_up = 0, Measure_down = 0, Measurement_CNT = 0;//Measure_up =1?
float Bf_value = 0, BfBf_value = 0, Bf_avr = 0, Now_avr = 0;
float Rate = 0, Rate_2 = 0;
extern byte Cal_rate_of_change_f;
//변화율 계산 함수
float Rate_of_change(float value)
{
	float rate_of_change = 0;
	/*
	if(Bf_value != 0 && BfBf_value != 0)	//0으로 나누는 것을 방지하기 위함(NaN 오류 방지), 처음 두사이클은 과거값과 이전과거값을 저장하기위해 평균값 비교 과정을 스킵(23.05.19)
	{
		//rate_of_change = (value - Bf_value)/Bf_value * 100;	//이전값과 현재값의 변화율 계산
		Now_avr = (value + Bf_value)/2;	
		Bf_avr = (Bf_value + BfBf_value)/2;	
		if(Bf_avr != 0)
			rate_of_change = (Now_avr - Bf_avr)/Now_avr * 100;	//이전평균값과 현재평균값의 변화율 계산
	}

	BfBf_value = Bf_value;		//이전과거값에 과거값을 저장
	
	if(Current_actual >= 0.1)	//0.1A이상일 때만 과거값 저장		(로드셀로 실험중에는 지움23.06.23)
		Bf_value = value;		//현재값을 과거값에 저장
	*/
	if(Bf_value != 0)	//0으로 나누는 것을 방지하기 위함(NaN 오류 방지), 처음 두사이클은 과거값과 이전과거값을 저장하기위해 평균값 비교 과정을 스킵(23.05.19)
	{
		rate_of_change = fabs(value - Bf_value)/Bf_value * 100;	//이전값과 현재값의 변화율 계산 24.05.02 절댓값 함수를 추가하여 상한각이던 하한각이던 부하 감지가 되면 반대방향으로 회던하도록 유도함
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
	//무릎기구 방향 및 각도 조정 테스트 시작(23.02.02)
	//추가한것
	if(_USER_RUN_SETTING.motion == Knee)//무릎
	{
		direction_1 = CCW;//1
		direction_2 = CW;//0
	}
	else//어깨, 팔꿈치, 손목, 발목
	{
		direction_1 = CW;//0
		direction_2 = CCW;//1
	}

	if(Dir_of_rotation == direction_1)	//시계방향(어깨, 팔꿈치 기준)
	{
		angle_distance = target_angle - Current_angle; //설정 각도와 현재 각도의 차(실시간, 10ms마다 계산)
		Soft_start_stop_PI(acceltime, deacceltime, speed_step, velocity_mode);
		//int direction = (_USER_RUN_SETTING.motion == Knee) ? -1 : 1;		
		//if(direction * target_angle <= direction * Current_angle)
		
		if(target_angle <= Current_angle)
		{
			Change_dir = 0;
			Cal_rate_of_change_f = 0;	
			if(_USER_RUN_SETTING.calibration == 0)//캘리브레이션 모드일 경우에는 각도 제한을 풀기위해 이러한 코드를 추가하였음.
			{
				if(Measurement_mode && start_over_angle)
				{
					start_over_angle = 0;					
					over_angle = 1;	//각도 초과가 달성하면 몇초동안은 다시 1로 만들지 못하게 만들어서 연속으로 측정기록이 되는 현상을 방지하였음.	
					over_angle__start_flag = 1;
				}
				else//4,5,6,7 수동운동모드일 경우
				{
					if((exercise_mode <= 7) && (exercise_mode >= 4))//일반, 적응, 집중
					{
						PWM_SLEEP;
					}
					over_angle = 1;		
					_VOICE_DATA.x06_1B = 0x00;
				}
			}
		}
		if(over_angle)//현재각도가 상한각을 넘어갈 때
		{
			if(Measurement_mode)
				Over_angle_flag = 1;
			over_angle = 0;
			Motor_PWM = 0;				//설정 각도가 되면 정지
			pwm_step = 0;					
			deaccel_flag = 0;
			move_to_lower = 0;			//일반운동 처음 실행시 하한각으로 먼저 이동후 move_to_lower = 0
			
			if((exercise_mode == 1) || (exercise_mode == 3) || (exercise_mode == 2))	//일반, 적응, 집중 운동
				timer2_ms_cnt_start = 1;	//목표 각도가 되면 정지 카운터 up 시작
			else if((exercise_mode >= 4) && (exercise_mode <= 8)) 		//수동모드, 측정모드
			{
				if(exercise_mode != 4)
					Dir_of_rotation = !direction_1;
				exercise_mode = 0;			
				_LoRa_Data.state = 1;//운동 종료 24.04.08 추가 하였음.
				_SENSOR_DATA.state = 1;	//운동 종료
				//_LoRa_Data.use_state = 0x08;
				Passive_ing = 0;//수동조작으로 이동종료
				_USER_RUN_SETTING.exerc_start = 0x00;
		
				//////////////////////////////////////////////////////
				Record_data_f = 1;	//엔코더 각도측정 테스트(끝 각도 저장 flag)
				OFFSET_angle_2 = P_M_actual;
				
				//////////////지울것////////////////
				//timer2_ms_cnt_start = 1;	//엔코더 각도측정테스트용(테스트끝나면 꼭!!! 지울것!!!!!) 지우지않으면 수동모드 정상동작 안함!!!
				//////////////////////////////
				//측정모드 테스트(데이터 기록용)
				Record_F = 0;	//목표각 도달시 기록 중단
			}
			/*
			if(Measurement_mode)
			{
				//상한각 측정중에 최대각도에 도달했을 시 측정 강제 종료
				if(Measure_up)
					Measure_up = 0;
				Dir_of_rotation = !direction_1;	//이동 방향 전환
				High_angle = Current_angle;	//상한 측정각 저장
				_LoRa_Data.upper_limit_angle = (byte)High_angle;	//상한 측정각 저장(PC에 보낼 데이터)
				_SENSOR_DATA.upper_limit_angle = (byte)High_angle;	//상한 측정각 저장(LCD보드에 보낼 데이터)
				Motor_PWM = 0;				//부하가 감지되면 정지
				pwm_step = 0;					
				deaccel_flag = 0;
				Measurement_CNT++;	//측정 횟수 증가
				timer2_ms_cnt_start = 1;	//상한각 측정완료 후, 정지 카운터 up 시작
			}
			*/
		}	
	}
	else if(Dir_of_rotation == direction_2)	//반시계방향(어깨, 팔꿈치 기준)
	{
		angle_distance = Current_angle - target_angle; //설정 각도와 현재 각도의 차(실시간, 10ms마다 계산)
		Soft_start_stop_PI(acceltime, deacceltime, speed_step, velocity_mode);
		//int direction = (_USER_RUN_SETTING.motion == Knee) ? -1 : 1;		
		//if(direction * target_angle >= direction * Current_angle)
		if(target_angle /*+ 0.5*/ >= Current_angle)//측정모드 lcd에서 19도로 기록되는 현상 방지
		{
			Change_dir = 0;
			Cal_rate_of_change_f = 0;
			if(_USER_RUN_SETTING.calibration == 0)//캘리브레이션 모드일 경우에는 각도 제한을 풀기위해 이러한 코드를 추가하였음.
			{	
				if(Measurement_mode && start_over_angle)
				{
					start_over_angle = 0;
					over_angle = 1;	//각도 초과가 달성하면 몇초동안은 다시 1로 만들지 못하게 만들어서 연속으로 측정기록이 되는 현상을 방지하였음.	
					over_angle__start_flag = 1;
				}
				else//4,5,6,7 수동운동모드일 경우
				{
					if((exercise_mode <= 7) && (exercise_mode >= 4))//일반, 적응, 집중
					{
						PWM_SLEEP;
					}
					over_angle = 1;		
					_VOICE_DATA.x06_1B = 0x00;
				}
			}
		}
		if(over_angle)//현재각도가 하한각을 넘어갈 때
		{
			if(Measurement_mode)
				Over_angle_flag = 1;
			over_angle = 0;
			Motor_PWM = 0;				//설정 각도가 되면 정지
			pwm_step = 0;
			deaccel_flag = 0;
			move_to_lower = 0;			//일반운동 처음 실행시 하한각으로 먼저 이동후 move_to_lower = 0
			
			if((exercise_mode == 1) || (exercise_mode == 3) || (exercise_mode == 2))//일반, 적응, 집중 운동
				timer2_ms_cnt_start = 1;	//목표 각도가 되면 정지 카운터 up 시작
			else if((exercise_mode >= 4) && (exercise_mode <= 8))//수동모드, 측정모드
			{
				if(exercise_mode != 4)
					Dir_of_rotation = !direction_2;
				exercise_mode = 0;			
				_LoRa_Data.state = 1;		//운동 종료
				_SENSOR_DATA.state = 1;		//운동 종료
				Passive_ing = 0;	//수동조작으로 이동 종료
				_USER_RUN_SETTING.exerc_start = 0x00;
				Record_data_f = 2;	//엔코더 각도측정 테스트(끝 각도 저장 flag)
				OFFSET_angle_2 = P_M_actual;
				//측정모드 테스트(데이터 기록용)
				Record_F = 0;	//목표각 도달시 기록 중단
			}
			/*
			if(Measurement_mode)
			{	
				//하한각 측정중에 최소각도에 도달했을 시 측정 강제 종료
				if(Measure_down)
					Measure_down = 0;
				Dir_of_rotation = !direction_2;	//이동 방향 전환
				Motor_PWM = 0;				//부하가 감지되면 정지
				pwm_step = 0;					
				deaccel_flag = 0;
				Measurement_CNT++;//측정 횟수 증가
				timer2_ms_cnt_start = 1;	//상한각 측정완료 후, 정지 카운터 up 시작
			}
			*/
		}
	}

	if(Detect_load || Over_angle_flag)	//측정모드이고, 부하가 감지되었을 때
	{
		Over_angle_flag = 0;
		Detect_load = 0;		//부하감지 0으로 초기화
		Cal_rate_of_change_f = 0;
		if(!EMS_SW)// 비상정지가 아닐때만 부하가 감지되었다는 음성 출력
		{
			_VOICE_DATA.x06_1B = 0x2A;//부하가 감지되었습니다 음성출력	
		}
		Motor_PWM = 0;				//부하가 감지되면 정지
		pwm_step = 0;					
		deaccel_flag = 0;	
		Bf_Dir_of_rotation = Dir_of_rotation;
		timer2_ms_cnt_start = 1;	//상한각 측정완료 후, 정지 카운터 up 시작, 하한각 측정완료 후, 정지 카운터 up 시작	
		if(Measure_up)		//상한각 측정모드일 때
		{
			Measure_up = 0;	//하한각 측정모드로 변경
			Dir_of_rotation = !direction_1;	//이동 방향 전환
			
			// 24.10.07 도우가 함
			if(_USER_RUN_SETTING.motion == 0) // 운동 부위가 팔꿈치일때 ( 하한각 0 ~ 상한각 150 )
			{
				if(Current_angle >= 150.0) // 상한각이 150 이상이 되지 않게끔 보정
				{
				  High_angle_correction = 200.0; // ( 0 ~ 230 ) 통신후 -50 처리
				  High_angle = High_angle_correction; //상한 측정각(보정) 저장
				  total_high_angle += 150.0;
				}
				else
				{
				  High_angle = Current_angle + 50.0;
				  total_high_angle += Current_angle;
				}
			}
			else if(_USER_RUN_SETTING.motion == 1) // 운동 부위가 어깨일때 ( 하한각 20 ~ 상한각 180 )
			{
				if(Current_angle >= 180.0) // 상한각이 180 이상이 되지 않게끔 보정
				{
				  High_angle_correction = 230.0; // ( 0 ~ 230 ) 통신후 -50 처리
				  High_angle = High_angle_correction; //상한 측정각(보정) 저장
				  total_high_angle += 180.0;
				}
				else
				{
				  High_angle = Current_angle + 50.0;
				  total_high_angle += Current_angle;
				}
			}
			else if(_USER_RUN_SETTING.motion == 2) // 운동 부위가 무릎일때 ( 하한각 -10 ~ 상한각 140 )
			{
				if(Current_angle >= 140.0) // 상한각이 140 이상이 되지 않게끔 보정
				{
				  High_angle_correction = 190.0; // ( 0 ~ 230 ) 통신후 -50 처리
				  High_angle = High_angle_correction; //상한 측정각(보정) 저장
				  total_high_angle += 140.0;
				}
				else
				{
				  High_angle = Current_angle + 50.0;
				  total_high_angle += Current_angle;
				}
			}
			else if(_USER_RUN_SETTING.motion == 3) // 운동 부위가 손목일때 ( 하한각 -70 ~ 상한각 70 )
			{
				if(Current_angle >= 70.0) // 상한각이 70 이상이 되지 않게끔 보정
				{
				  High_angle_correction = 120.0; // ( 0 ~ 230 ) 통신후 -50 처리
				  High_angle = High_angle_correction; //상한 측정각(보정) 저장
				  total_high_angle += 70.0;
				}
				else
				{
				  High_angle = Current_angle + 50.0;
				  total_high_angle += Current_angle;
				}
			}
			else if(_USER_RUN_SETTING.motion == 4) // 운동 부위가 발목일때 ( 하한각 -50 ~ 상한각 50 )
			{
				if(Current_angle >= 50.0) // 상한각이 50 이상이 되지 않게끔 보정
				{
				  High_angle_correction = 100.0; // ( 0 ~ 230 ) 통신후 -50 처리
				  High_angle = High_angle_correction; //상한 측정각(보정) 저장
				  total_high_angle += 50.0;
				}
				else
				{
				  High_angle = Current_angle + 50.0;
				  total_high_angle += Current_angle;
				}
			}
			_SENSOR_DATA.upper_limit_angle = (byte)High_angle;//상한 측정각 저장(LCD보드에 보낼 데이터)
			if(_SENSOR_DATA.upper_limit_angle == 0)
			{
			  _SENSOR_DATA.upper_limit_angle = 1;
			}
			else if(_SENSOR_DATA.lower_limit_angle == 0x32)
			{
			  _SENSOR_DATA.lower_limit_angle = 0x33;
			}
		}
		else if(Measure_down)	//하한각 측정모드일 때
		{
			Measure_down = 0;	//하한각 측정모드로 변경
			Dir_of_rotation = !direction_2;	//이동 방향 전환
			
			if(_USER_RUN_SETTING.motion == 0) // 운동 부위가 팔꿈치일때 ( 하한각 0 ~ 상한각 150 )
			{
				if(Current_angle <= 0) // 하한각이 0 이하가 되지 않게끔 보정
				{
				  Lower_angle_correction = 70.0; // ( 0 ~ 230 ) 통신후 -70 처리
				  Lower_angle = Lower_angle_correction; //상한 측정각(보정) 저장
				  total_low_angle += 0;
				}
				else
				{
				  Lower_angle = Current_angle + 70.0;
				  total_low_angle += Current_angle;
				}
			}
			else if(_USER_RUN_SETTING.motion == 1) // 운동 부위가 어깨일때 ( 하한각 20 ~ 상한각 180 )
			{
				if(Current_angle <= 20.0) // 하한각이 20 이하가 되지 않게끔 보정
				{
				  Lower_angle_correction = 90.0; // ( 0 ~ 230 ) 통신후 -70 처리
				  Lower_angle = Lower_angle_correction; //상한 측정각(보정) 저장
				  total_low_angle += 20.0;
				}
				else
				{
				  Lower_angle = Current_angle + 70.0;
				  total_low_angle += Current_angle;
				}
			}
			else if(_USER_RUN_SETTING.motion == 2) // 운동 부위가 무릎일때 ( 하한각 -10 ~ 상한각 140 )
			{
				if(Current_angle <= -10.0) // 하한각이 -10 이하 되지 않게끔 보정
				{
				  Lower_angle_correction = 60.0; // ( 0 ~ 230 ) 통신후 -70 처리
				  Lower_angle = Lower_angle_correction; //상한 측정각(보정) 저장
				  total_low_angle += -10.0;
				}
				else
				{
				  Lower_angle = Current_angle + 70.0;
				  total_low_angle += Current_angle;
				}
			}
			else if(_USER_RUN_SETTING.motion == 3) // 운동 부위가 손목일때 ( 하한각 -70 ~ 상한각 70 )
			{
				if(Current_angle <= -70.0) // 하한각이 -70 이하가 되지 않게끔 보정
				{
				  Lower_angle_correction = 0; // ( 0 ~ 230 ) 통신후 -70 처리
				  Lower_angle = Lower_angle_correction; //상한 측정각(보정) 저장
				  total_low_angle += -70.0;
				}
				else
				{
				  Lower_angle = Current_angle + 70.0;
				  total_low_angle += Current_angle;
				}
			}
			else if(_USER_RUN_SETTING.motion == 4) // 운동 부위가 발목일때 ( 하한각 -50 ~ 상한각 50 )
			{
				if(Current_angle <= -50.0) // 하한각이 -50 이하가 되지 않게끔 보정
				{
				  Lower_angle_correction = 20.0; // ( 0 ~ 230 ) 통신후 -70 처리
				  Lower_angle = Lower_angle_correction; //상한 측정각(보정) 저장
				  total_low_angle += -50.0;
				}
				else
				{
				  Lower_angle = Current_angle + 70.0;
				  total_low_angle += Current_angle;
				}
			}
			_SENSOR_DATA.lower_limit_angle = (byte)Lower_angle;//하한 측정각 저장(LCD보드에 보낼 데이터)
			if(_SENSOR_DATA.lower_limit_angle == 0)
			{
			  _SENSOR_DATA.lower_limit_angle = 1;
			}
			else if(_SENSOR_DATA.lower_limit_angle == 0x46)
			{
			  _SENSOR_DATA.lower_limit_angle = 0x47;
			}
		}
		if(Dir_of_rotation != Bf_Dir_of_rotation) //방향이 전환 되면
		{
			Change_dir = 1;		//3초 뒤에 Cal_rate_of_change_f = 1수행	
		}
		Measurement_CNT++;//측정 횟수 증가
		Rate_2 = Rate;
		Current_2 = Current_actual;
	}	
	if(Measurement_CNT >= 6)
	{	
		Measurement_CNT = 0;		//측정 횟수 초기화		
		set_min_angle = total_low_angle/3;
		
		avr_total_high_angle = total_high_angle/3;
		avr_total_low_angle = total_low_angle/3;
		// 24.10.07 도우가 함
		if(_USER_RUN_SETTING.motion == 0) // 운동 부위가 팔꿈치일때 ( 하한각 0 ~ 상한각 150 )
		{
			if(avr_total_high_angle >= 150.0) // 상한각이 150 이상이 되지 않게끔 보정
			{
			  avr_total_high_angle = 200.0; // ( 0 ~ 230 ) 통신후 -50 처리
			}
			else
			{
			  avr_total_high_angle = avr_total_high_angle + 50.0;
			}
		}
		else if(_USER_RUN_SETTING.motion == 1) // 운동 부위가 어깨일때 ( 하한각 20 ~ 상한각 180 )
		{
			if(avr_total_high_angle >= 180.0) // 상한각이 180 이상이 되지 않게끔 보정
			{
			  avr_total_high_angle = 230.0; // ( 0 ~ 230 ) 통신후 -50 처리
			}
			else
			{
			  avr_total_high_angle = avr_total_high_angle + 50.0;
			}
		}
		else if(_USER_RUN_SETTING.motion == 2) // 운동 부위가 무릎일때 ( 하한각 -10 ~ 상한각 140 )
		{
			if(avr_total_high_angle >= 140.0) // 상한각이 140 이상이 되지 않게끔 보정
			{
			  avr_total_high_angle = 190.0; // ( 0 ~ 230 ) 통신후 -50 처리
			}
			else
			{
			  avr_total_high_angle = avr_total_high_angle + 50.0;
			}
		}
		else if(_USER_RUN_SETTING.motion == 3) // 운동 부위가 손목일때 ( 하한각 -70 ~ 상한각 70 )
		{
			if(avr_total_high_angle >= 70.0) // 상한각이 70 이상이 되지 않게끔 보정
			{
			  avr_total_high_angle = 120.0; // ( 0 ~ 230 ) 통신후 -50 처리
			}
			else
			{
			  avr_total_high_angle = avr_total_high_angle + 50.0;
			}
		}
		else if(_USER_RUN_SETTING.motion == 4) // 운동 부위가 발목일때 ( 하한각 -50 ~ 상한각 50 )
		{
			if(avr_total_high_angle >= 50.0) // 상한각이 50 이상이 되지 않게끔 보정
			{
			  avr_total_high_angle = 100.0; // ( 0 ~ 230 ) 통신후 -50 처리
			}
			else
			{
			  avr_total_high_angle = avr_total_high_angle + 50.0;
			}
		}
		
		if(_USER_RUN_SETTING.motion == 0) // 운동 부위가 팔꿈치일때 ( 하한각 0 ~ 상한각 150 )
		{
			if(avr_total_low_angle <= 0) // 하한각이 0 이하가 되지 않게끔 보정
			{
			  avr_total_low_angle = 70.0; // ( 0 ~ 230 ) 통신후 -70 처리
			}
			else
			{
			  avr_total_low_angle = avr_total_low_angle + 70.0;
			}
		}
		else if(_USER_RUN_SETTING.motion == 1) // 운동 부위가 어깨일때 ( 하한각 20 ~ 상한각 180 )
		{
			if(avr_total_low_angle <= 20.0) // 하한각이 20 이하가 되지 않게끔 보정
			{
			  avr_total_low_angle = 90.0; // ( 0 ~ 230 ) 통신후 -70 처리
			}
			else
			{
			  avr_total_low_angle = avr_total_low_angle + 70.0;
			}
		}
		else if(_USER_RUN_SETTING.motion == 2) // 운동 부위가 무릎일때 ( 하한각 -10 ~ 상한각 140 )
		{
			if(avr_total_low_angle <= -10.0) // 하한각이 140 이상이 되지 않게끔 보정
			{
			  avr_total_low_angle = 60.0; // ( 0 ~ 230 ) 통신후 -70 처리
			}
			else
			{
			  avr_total_low_angle = avr_total_low_angle + 70.0;
			}
		}
		else if(_USER_RUN_SETTING.motion == 3) // 운동 부위가 손목일때 ( 하한각 -70 ~ 상한각 70 )
		{
			if(avr_total_low_angle <= -70.0) // 하한각이 -70 이하가 되지 않게끔 보정
			{
			  avr_total_low_angle = 0; // ( 0 ~ 230 ) 통신후 -70 처리
			}
			else
			{
			  avr_total_low_angle = avr_total_low_angle + 70.0;
			}
		}
		else if(_USER_RUN_SETTING.motion == 4) // 운동 부위가 발목일때 ( 하한각 -50 ~ 상한각 50 )
		{
			if(avr_total_low_angle <= -50.0) // 하한각이 -50 이하가 되지 않게끔 보정
			{
			  avr_total_low_angle = 20.0; // ( 0 ~ 230 ) 통신후 -70 처리
			}
			else
			{
			  avr_total_low_angle = avr_total_low_angle + 70.0;
			}
		}
		
		total_low_angle = 0, total_high_angle = 0; //기록 후 초기화 시킴
		/*_VOICE_DATA.x06_1B = 0x15; //측정이 완료 되었습니다 음성 출력
		_LoRa_Data.use_state = 0x08;//운동종료
		//_SENSOR_DATA.state = 1;		//운동 종료(LCD에 보낼 데이터)
		//_SENSOR_DATA.state = 1;		//운동 종료(LCD보드에 보낼 데이터)
		//_USER_RUN_SETTING.exerc_start = 0x0A;//6회 측정
		timer2_ms_cnt_start = 0;	//정지 카운터 flag 종료
		_Ble_Data.exer_state = 2; // 어플로 측정 완료 신호를 보냄 // 추가한것
		Not_Send_Ble = 1; // 운동각 측정이 종료되었을때는 블루투스로 운동기록 데이터를 보내지 않도록 함( 운동각 측정 기록 신호만 보냄 ).*/
		timer2_ms_cnt_start = 0;	//정지 카운터 flag 종료
		exerc_end = 1;//+10
		mode_flag = 10;	
		Move_to_lower = 1;
		Measurement_mode = 0;
	}
}
//하한, 상한 정지 시간 counter
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
//운동시간 모드의 운동시간 측정용 함수
byte motor_exerc_time(word exerc_time_min)
{
	if(t2_min_cnt < exerc_time_min)
		return 1;
	else if(t2_min_cnt >= exerc_time_min)
	{
		Time_mode_end = 1;		//운동시간 끝났음을 알려주기 위한 flag(22.10.31 운동횟수모드가 끝나고 운동시간모드를 실행했을 때 시간카운팅이 제대로 되지 않는 버그 수정)
		exerc_time_start = 0;	//1ms timer counting end
		return 2;
	}
	return 0;
}
word exerc_cnt = 0;	//운동횟수(왕복)
byte Reach_high_angle_flag = 0; //상한각을 도달해야만 카운트 되게끔 구현하였음. 2024.09.25
//일반운동 모드					 하한 각도					상한 각도					속도모드			   속도단계			운동횟수			운동시간(min)			  하한 정지시간				상한 정지시간	
void Normal_exercise(float lower_limit_angle, float upper_limit_angle, byte velocity_mode, byte speed_step, byte num_of_exerc, word exerc_time, dword lower_limit_time, dword upper_limit_time)
{
	ROM = upper_limit_angle - lower_limit_angle; 	//총 운동범위(ROM)
	static byte operating;
	//operating = num_of_exerc ? num_of_exerc : motor_exerc_time(exerc_time);
	if((num_of_exerc > 0) && (exerc_time == 0))			//운동횟수 모드일 경우
	{
		operating = num_of_exerc;	
	}
	else if((exerc_time > 0) && (num_of_exerc == 0))	//운동시간 모드일 경우
	{
		operating = motor_exerc_time(exerc_time);	
	}
	static byte direction_1, direction_2;
	//관절 부위에 따른 방향 설정
	if(_USER_RUN_SETTING.motion == Knee)	//무릎
	{
		direction_1 = CCW;
		direction_2 = CW;
	}
	else	//어깨, 팔꿈치, 손목, 발목
	{
		direction_1 = CW;
		direction_2 = CCW;
	}
	if(operating)
	{
		static byte bf = 0;
		if(bf != Dir_of_rotation)    //Dir_of_rotation가 바뀌었을 때 한번만 수행됨
		{
			if(Dir_of_rotation == direction_1)		
			{
				DISTANCE = upper_limit_angle - Current_angle;	//운동 방향이 바뀌고 이동할 거리(각도) 계산
			}
			else if(Dir_of_rotation == direction_2)
			{
				DISTANCE = Current_angle - lower_limit_angle;	//운동 방향이 바뀌고 이동할 거리(각도) 계산
			}
			bf = Dir_of_rotation;
		}
		
		if(Dir_of_rotation == direction_1)		//시계방향, 상한각도로 이동
		{	
 			angle_adjustment_PI(upper_limit_angle, speed_step, velocity_mode);
			if(motor_delay_ms(upper_limit_time))	//설정한 상한정지시간이 되면 1
			{
				if(upper_limit_angle <= Current_angle)		//현재 각도가 상한 각도 이상일 때
				{
					Dir_of_rotation = direction_2;	//반시계방향으로 전환
					toggle_cnt++;
				}
			}
		}
		else if(Dir_of_rotation == direction_2)	//반시계방향, 하한각도로 이동
		{
			angle_adjustment_PI(lower_limit_angle, speed_step, velocity_mode);
			if(motor_delay_ms(lower_limit_time))	//설정한 하한정지시간이 되면 1
			{
				if(lower_limit_angle >= Current_angle)		//현재 각도가 하한 각도 이하일 때
				{				
					Dir_of_rotation = direction_1;			//시계방향으로 전환
					toggle_cnt++;
				}
			}
		}
		
		
		if((toggle_cnt >= 2) && (num_of_exerc > 0) && (exerc_time == 0))	//운동횟수 모드일때만 카운팅 (23.01.16)//한번 왕복할때 마다//상한각을 도달해야만 카운트 되게끔 구현하였음. 2024.09.25
		{
			toggle_cnt = 0;
			exerc_cnt++;	//운동횟수 카운팅
			num_of_workouts++;	//운동횟수 카운팅(통신용)
		}
			
		
		if((num_of_exerc > 0) && (exerc_cnt >= operating))		//운동횟수 끝날시
		{
			exercise_mode = 0;	//일반운동 모드 종료
			deaccel_flag = 0;	
			pwm_step = 0;
			exerc_cnt = 0;
			exerc_end = 1;		//운동 종류 후, 하한각으로 이동을 위한 flag
			mode_flag = 10;		//운동 종류 후, 하한각 이동시 DISTANCE값 계산을 위한 flag
			
		}
		else if((exerc_time > 0) && (Time_mode_end == 1)) //운동시간 끝날시   //(exerc_time_start == 0)) //(motor_exerc_time(exerc_time) == 2))	
		{
			mode_flag = 10;		//운동 종류 후, 하한각 이동시 DISTANCE값 계산을 위한 flag
			if(Dir_of_rotation == direction_1)
			{
				deaccel_flag = 1;	//운동시간 끝나면 감속시작
				if((upper_limit_angle <= Current_angle) || (pwm_step <= 0))	//감속시작 후, 현재각도가 상한각 이상이 되거나 감속이 끝나면 일반운동모드 종료
				{
					exercise_mode = 0;
					deaccel_flag = 0;
					pwm_step = 0;
					exerc_cnt = 0;
					exerc_end = 1;		//운동 종류 후, 하한각으로 이동을 위한 flag
					Time_mode_end = 0;
				}
			}
			else if(Dir_of_rotation == direction_2)
			{
				deaccel_flag = 1;	//운동시간 끝나면 감속시작
				if((lower_limit_angle >= Current_angle) || (pwm_step <= 0)) //감속시작 후, 현재각도가 하한각 이하가 되거나 감속이 끝나면 일반운동모드 종료
				{
					exercise_mode = 0;
					deaccel_flag = 0;
					pwm_step = 0;
					exerc_cnt = 0;
					exerc_end = 1;		//운동 종류 후, 하한각으로 이동을 위한 flag
					Time_mode_end = 0;
				}
			}
		}
	}
}
byte Move_to_lower = 1;
//일반운동모드 종료 후, 하한각 이동 및 하한각+10도에서 멈춤
void exerc_stop_mode(float target_angle, byte speed_step, byte Dir)
{
	static int rpm;
	switch(speed_step)		//속도단계에 따른 모터 등속,가속 rpm 설정(7단계까지 설정가능)
	{						//DC모터와 기구물의 기어비 = 1500:1
		case 1:
			rpm = 460;	//기구물 RPM = 0.277  
			break;
		case 2:
			rpm = 665; //기구물 RPM = 0.37  
			break;
		case 3:
			rpm = 830; //기구물 RPM = 0.463 
			break;
		case 4:
			rpm = 1015; //기구물 RPM = 0.556 
			break;
		case 5:
			rpm = 1200; //기구물 RPM = 0.639 
			break;
		case 6:
			rpm = 1340; //기구물 RPM = 0.789 
			break;
		case 7:
			rpm = 1460; //기구물 RPM = 0.833 
			break;
		case 8:
			rpm = 1615; //기구물 RPM = 0.917 
			break;
		case 9:
			rpm = 1800; //기구물 RPM = 1.000 
			break;			
		default:
			break;
	}
	static byte cal_accel_angle;
	if(Move_to_lower == 1)
		angle_distance = Current_angle - target_angle; //하한 각도와 현재 각도의 차(실시간)
	else if(Move_to_lower == 0)
		angle_distance = target_angle - Current_angle; //하한 각도+10와 현재 각도의 차(실시간)
	if(!deaccel_flag)
	{
		/*
		if((speed_step >= 1) && (target_speed < rpm))
		{
			target_speed = ((rpm-720)/(float)acceltime)*pwm_step + 720;	//Soft Start
			pid_val = PID_control_sys(target_speed, speed_RPM);
			Motor_PWM = (unsigned int)set_motor_pwm(pid_val);	//모터 PWM PID 제어
			
			
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
		target_speed = ((rpm-240)/(float)acceltime)*pwm_step + 240;	//Soft Start 240에서 480으로 변경하였음.
		pid_val = PID_control_sys(target_speed, speed_RPM);
		Motor_PWM = (unsigned int)set_motor_pwm(pid_val);	//모터 PWM PID 제어
		if(pwm_step < acceltime)
		{
			pwm_step++;
			if(pwm_step == acceltime-1)
			{
				cal_accel_angle = 1;
			}
		}
		
		static byte direction_1, direction_2;
		//무릎기구 방향 및 각도 조정 테스트 시작(23.02.02)
		if(_USER_RUN_SETTING.motion == Knee)	//무릎
		{
			direction_1 = CCW;
			direction_2 = CW;
		}
		else	//어깨, 무릎
		{
			direction_1 = CW;
			direction_2 = CCW;
		}
		
		if(target_speed >= rpm) //설정속도가 될 때
		{
			if(cal_accel_angle == 1)
			{
				if(Dir == direction_1)	  //시계방향
					accel_angle = DISTANCE - angle_distance + upper_correction_angle;	//최고속도까지 도달하는 동안 이동한 각도 계산
				else if(Dir == direction_2) //반시계방향
					accel_angle = DISTANCE - angle_distance + lower_correction_angle;	//최고속도까지 도달하는 동안 이동한 각도 계산
				
				cal_accel_angle = 0;
			}
			if(angle_distance <= accel_angle)	//감속조건 달성 
				deaccel_flag = 1;	//감속 flag  ON
		}
		
	}
	else	//deaccel_flag == 1
	{
		if(--pwm_step > deacceltime)	//step이 0밑으로 떨어져 65535 되는걸 방지
			pwm_step = 0;
		target_speed = ((rpm-300)/(float)deacceltime)*pwm_step + 300;		//Soft Stop  //Soft Start 240
		pid_val = PID_control_sys(target_speed, speed_RPM);
		Motor_PWM = (unsigned int)set_motor_pwm(pid_val);	//모터 PWM PID 제어
	}
	

	if((Move_to_lower == 1) && (target_angle >= Current_angle))		//하한각 도달시
	{
		Motor_PWM = 0;				//설정 각도가 되면 정지
		pwm_step = 0;
		deaccel_flag = 0;
		mode_flag = 10;
		//하한각 도달 후, 200ms 동안 일시정지(delay time이 없으면 모터 드라이버에 무리가 감) => 적용시, 각도 표시 제한이 풀리는 버그가 있어서 보류(23.02.07)
		//timer2_ms_cnt_start = 1;
		//if(motor_delay_ms(200))
		Move_to_lower = !Move_to_lower;		//Move_to_lower : 1 => 0
	}
	else if((Move_to_lower == 0) && (target_angle <= Current_angle))
	{
		Motor_PWM = 0;//설정 각도가 되면 정지
		pwm_step = 0;
		deaccel_flag = 0;
		exerc_end = 0;
		Move_to_lower = !Move_to_lower;//Move_to_lower : 0 => 1
		//Dir_of_rotation = STOP;//운동이 끝나고 방향을 STOP으로 초기화
		toggle_cnt = 0;
		num_of_workouts = 0;
		_LoRa_Data.state = 1;//운동종료(PC보드에 보낼 데이터)
		_SENSOR_DATA.state = 1;//운동종료(LCD보드에 보낼 데이터)
		voice_en = 1;
		
		if(exercise_mode == 0x09)
		{
		  	exercise_mode = 0;
		  	_VOICE_DATA.x06_1B = 0x15; //측정이 완료 되었습니다 음성 출력
			_LoRa_Data.use_state = 0x81;//운동종료
			_Ble_Data.exer_state = 2; // 어플로 측정 완료 신호를 보냄 // 추가한것
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
			_Ble_Data.lower_limit_angle = (byte)avr_total_low_angle;	//하한 측정각 저장(어플에 보낼 데이터)//평균값으로 수정해야함
			_LoRa_Data.lower_limit_angle = (byte)avr_total_low_angle;	//하한 측정각 저장(로라보드에 보낼 데이터)//평균값으로 수정해야함		
			_Ble_Data.upper_limit_angle = (byte)avr_total_high_angle;	//상한 측정각 저장(어플에 보낼 데이터)//평균값으로 수정해야함
			_LoRa_Data.upper_limit_angle = (byte)avr_total_high_angle;	//상한 측정각 저장(로라보드에 보낼 데이터)//평균값으로 수정해야함	
			Not_Send_Ble = 1; // 운동각 측정이 종료되었을때는 블루투스로 운동기록 데이터를 보내지 않도록 함( 운동각 측정 기록 신호만 보냄 ).
		}
		else
		{
			_VOICE_DATA.x06_1B = 0x28;  //"운동을 종료합니다" 음성 출력	
		}
		
		if(Not_Send_Ble != 1) // 운동종료 버튼으로 운동을 종료했을때는 운동 기록이 되지 않음
		{
			_Ble_Data.exer_state = 1; // 추가한것
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
//적응운동 모드						 하한 각도 				  상한 각도			적응각도		 반복횟수			 적용위치	 		속도단계		       하한 정지시간				상한 정지시간
void Adaptive_exercise(float lower_limit_angle, float upper_limit_angle, float PA, byte repeat_num, byte location, byte speed_step, dword lower_limit_time, dword upper_limit_time)
{
	static float lower_initial_angle, upper_initial_angle;
	static word exerc_num;
	_LoRa_Data.mode = 1;	//적응운동
	_Ble_Data.mode = 1;		//적응운동
	//하한초기각도, 상한초기각도 계산
	lower_initial_angle = lower_limit_angle + PA;
	upper_initial_angle	= upper_limit_angle - PA;
	
	//운동횟수 계산
	exerc_num = (word)(PA+1)*repeat_num;	//운동횟수 = (적응각도+1)*반복횟수
	
	static byte direction_1, direction_2;
	//무릎기구 방향 및 각도 조정 테스트 시작(23.02.02)
	if(_USER_RUN_SETTING.motion == Knee)	//무릎
	{
		direction_1 = CCW;
		direction_2 = CW;
	}
	else	//어깨, 무릎
	{
		direction_1 = CW;
		direction_2 = CCW;
	}
	
	
	if(exerc_num)	//적응운동은 운동횟수만 가능
	{
		static byte bf = 0;
		if(bf != Dir_of_rotation)		//Dir_of_rotation가 바뀌었을 때 한번만 수행됨
		{
			switch(location)	//적용위치에 따른 DISTANCE값 계산
			{
				case UPPER:	//상한 집중
					if(Dir_of_rotation == direction_1)		
						DISTANCE = (upper_initial_angle + angle_step) - Current_angle;	//운동 방향이 바뀌고 이동할 거리(각도) 계산
					else if(Dir_of_rotation == direction_2)
						DISTANCE = Current_angle - lower_limit_angle;					//운동 방향이 바뀌고 이동할 거리(각도) 계산
					break;
				case LOWER:	//하한 집중
					if(Dir_of_rotation == direction_1)		
						DISTANCE = upper_limit_angle - Current_angle;					//운동 방향이 바뀌고 이동할 거리(각도) 계산
					else if(Dir_of_rotation == direction_2)
						DISTANCE = Current_angle - (lower_initial_angle - angle_step);	//운동 방향이 바뀌고 이동할 거리(각도) 계산
					break;
				case UP_LOW: //상하한 집중
					if(Dir_of_rotation == direction_1)		
						DISTANCE = (upper_initial_angle + angle_step) - Current_angle;	//운동 방향이 바뀌고 이동할 거리(각도) 계산
					else if(Dir_of_rotation == direction_2)
						DISTANCE = Current_angle - (lower_initial_angle - angle_step);	//운동 방향이 바뀌고 이동할 거리(각도) 계산
					break;
				default:
					break;
			}
			bf = Dir_of_rotation;
		}			
		static float target_angle;
		if(Dir_of_rotation == direction_1)		//시계방향, 상한각도로 이동
		{
			//적용위치에 따른 목표 각도 설정
			if(location == UPPER)			//적용위치 : 상한각
				target_angle = upper_initial_angle + angle_step;
			else if(location == LOWER)		//적용위치 : 하한각
				target_angle = upper_limit_angle;
			else if(location == UP_LOW)		//적용위치 : 상하한각
				target_angle = upper_initial_angle + angle_step;
			angle_adjustment_PI(target_angle, speed_step, CONSTANT_MODE);	//적응운동은 등속모드만 가능 
			if(motor_delay_ms(upper_limit_time))	//설정한 상한정지시간이 되면 1
			{
				if(target_angle <= Current_angle)
				{
					Dir_of_rotation = direction_2;	//반시계방향으로 전환
					toggle_cnt++;	
				}
			}
		}
		else if(Dir_of_rotation == direction_2)	//반시계방향, 하한각도로 이동
		{
			//적용위치에 따른 목표 각도 설정
			if(location == UPPER)			//적용위치 : 상한각
				target_angle = lower_limit_angle;
			else if(location == LOWER)		//적용위치 : 하한각
				target_angle = lower_initial_angle - angle_step;
			else if(location == UP_LOW)		//적용위치 : 상하한각
				target_angle = lower_initial_angle - angle_step;
			angle_adjustment_PI(target_angle, speed_step, CONSTANT_MODE);		//적응운동은 등속모드만 가능  
			if(motor_delay_ms(lower_limit_time))	//설정한 하한정지시간이 되면 1
			{
				if(target_angle >= Current_angle)		//현재 각도가 하한 각도 이하일 때
				{
					Dir_of_rotation = direction_1;			//시계방향으로 전환
					toggle_cnt++;
				}
			}
		}
		
		if(toggle_cnt >= 2)	
		{
			toggle_cnt = 0;
			num_of_workouts++;
			exerc_cnt++;		//1회 왕복할때마다 exer_cnt 1씩 증가
			if(exerc_cnt >= repeat_num)
			{
				exerc_cnt = 0;	//22.12.20. exerc_cnt가 0으로 초기화 되지 않아서 angle_step이 반복횟수와 관계없이 왕복할떄마다 증가하는 버그 수정
				angle_step += 1; 
			}
		}
							//22.12.20. 버그수정
		//if((exerc_num > 0) && (exerc_cnt >= exerc_num))	//운동횟수 끝날시
		if((exerc_num > 0) && (num_of_workouts >= exerc_num))	//운동횟수 끝날시
		{
			exercise_mode = 0;	//집중운동 모드 종료
			deaccel_flag = 0;	
			pwm_step = 0;
			exerc_end = 1;		//운동 종류 후, 하한각으로 이동을 위한 flag
			mode_flag = 10;		//운동 종류 후, 하한각 이동시 DISTANCE값 계산을 위한 flag
			exerc_cnt = 0;
		}
	}
}

//집중운동 변경사항(23.01.17)
//적용위치가 하한각 또는 상한각일 경우 한곳에서 계속 반복운동하는 것이 아닌, 집중운동을 반복횟수만큼 다채우고, 반대각(상한 또는 하한)을 한번 찍고 돌아와서 다시 반복운동함(이때 운동횟수 1카운트)

byte repeat_ok = 0, fake_cnt = 0;	
byte Focus_f = 0;	//각도표시 제한을 위한 flag(0:하한 또는 상한으로 이동중, 1:하한집중, 2:상한집중)
//집중운동 모드						 하한 각도 				  상한 각도					집중각도				   반복횟수			적용위치	 		속도단계			운동횟수				운동시간		      하한 정지시간				상한 정지시간
void Intensive_exercise(float lower_limit_angle, float upper_limit_angle, float concentration_angle, byte repeat_num, byte location, byte speed_step, byte num_of_exerc, word exerc_time, dword lower_limit_time, dword upper_limit_time)
{
	static float lower_conc_anlge, upper_conc_angle;
	static byte operating;
	_LoRa_Data.mode = 2;	//집중운동
	_Ble_Data.mode = 2;		//집중운동	
	//하한집중각도, 상한집중각도 계산, 	22.12.19. 각도보정 1도
	if(speed_step >= 6)	//6단계 이상일 때만 각도보정
	{
		lower_conc_anlge = (lower_limit_angle) + (concentration_angle);	
		upper_conc_angle = (upper_limit_angle) - (concentration_angle);
	}
	else	//if(speed_step < 6)
	{
		lower_conc_anlge = lower_limit_angle + concentration_angle;	
		upper_conc_angle = upper_limit_angle - concentration_angle;
	}
	if((num_of_exerc > 0) && (exerc_time == 0))			//운동횟수 모드일 경우
		operating = num_of_exerc;	
	else if((exerc_time > 0) && (num_of_exerc == 0))	//운동시간 모드일 경우
		operating = motor_exerc_time(exerc_time);	
	
	
	static byte direction_1, direction_2;
	//무릎기구 방향 및 각도 조정 테스트 시작(23.02.02)
	if(_USER_RUN_SETTING.motion == Knee)	//무릎
	{
		direction_1 = CCW;
		direction_2 = CW;
	}
	else	//어깨, 무릎
	{
		direction_1 = CW;
		direction_2 = CCW;
	}
	if(operating)
	{
		static byte bf = 2;
		if(bf != Dir_of_rotation)	//Dir_of_rotation가 바뀌었을 때 한번만 수행됨
		{	//적용위치에 따른 DISTANCE값 계산
			switch(location)
			{
				case UPPER:	//상한 집중
					if(Dir_of_rotation == direction_1)		
						DISTANCE = upper_limit_angle - Current_angle;	//운동 방향이 바뀌고 이동할 거리(각도) 계산
					else if(Dir_of_rotation == direction_2)
					{
						if(repeat_ok)//상한각에서 반복횟수만큼 집중운동을 하고, 하한각으로 이동
							DISTANCE = Current_angle - lower_limit_angle;	
						else
							DISTANCE = Current_angle - upper_conc_angle;	
					}
					break;
				case LOWER:	//하한 집중
					if(Dir_of_rotation == direction_1)		
					{
						if(repeat_ok)		//하한각에서 반복횟수만큼 집중운동을 하고, 상한각으로 이동
						{
							DISTANCE = upper_limit_angle - Current_angle;	
							//Focus_f = 0;	//상한으로 이동중
						}
						else
						{
							DISTANCE = lower_conc_anlge - Current_angle;
							//Focus_f = 1;	//하한집중
						}
					}
					else if(Dir_of_rotation == direction_2)
					{
						DISTANCE = Current_angle - lower_limit_angle;	//운동 방향이 바뀌고 이동할 거리(각도) 계산
						//Focus_f = 1;	//하한집중
					}
					break;
				case UP_LOW: //상하한 집중
					if((exerc_cnt+1)%2 == 1)	
					{
						if(Dir_of_rotation == direction_1)		
							DISTANCE = upper_limit_angle - Current_angle;	//운동 방향이 바뀌고 이동할 거리(각도) 계산
						else if(Dir_of_rotation == direction_2)
							DISTANCE = Current_angle - upper_conc_angle;	//운동 방향이 바뀌고 이동할 거리(각도) 계산
					}
					else if((exerc_cnt+1)%2 == 0)	
					{
						if(Dir_of_rotation == direction_1)		
							DISTANCE = lower_conc_anlge - Current_angle;	//운동 방향이 바뀌고 이동할 거리(각도) 계산
						else if(Dir_of_rotation == direction_2)
							DISTANCE = Current_angle - lower_limit_angle;	//운동 방향이 바뀌고 이동할 거리(각도) 계산
					}
					break;
				default:
					break;
			}
			bf = Dir_of_rotation;
		}
		
		if((location == UP_LOW) && (DISTANCE > (upper_limit_angle-upper_conc_angle)*1.7))
			except_f = 1;	//각도제한 예외(적용위치 상한각), 23.02.14(상하한각 모드 각도제한을 위함)
		
		
		static float target_angle;
		if(Dir_of_rotation == direction_1)		//시계방향
		{
			//적용위치에 따른 목표 각도 설정
			if(location == UPPER)			//적용위치 : 상한각
			{
				target_angle = upper_limit_angle;
				Focus_f = 1;	//상한각이동
			}
			else if(location == LOWER)		//적용위치 : 하한각
			{
				if(repeat_ok)		//하한각에서 반복횟수만큼 집중운동을 하고, 상한각으로 이동
				{
					target_angle = upper_limit_angle;
					Focus_f = 1;	//상한각이동
				}
				else
				{
					target_angle = lower_conc_anlge;
					Focus_f = 2;	//하한 집중각 이동
				}
			}
			else if(location == UP_LOW)		//적용위치 : 상하한각
			{
				if((exerc_cnt+1)%2 == 1)		//상한 집중
				{
					target_angle = upper_limit_angle;
					Focus_f = 1;	//상한각이동
				}
				else if((exerc_cnt+1)%2 == 0)	//하한 집중
				{
					target_angle = lower_conc_anlge;
					Focus_f = 2;	//하한 집중각 이동
				}
			}
			
			angle_adjustment_PI(target_angle, speed_step, CONSTANT_MODE);	//집중운동은 등속모드만 가능 
			if(motor_delay_ms(upper_limit_time))	//설정한 상한정지시간이 되면 1
			{
				if(target_angle <= Current_angle)
				{
					Dir_of_rotation = direction_2;	//반시계방향으로 전환
					if((location != LOWER) && (exerc_time > 0))	//적용위치가 상한 또는 상하한 일때
					{
						if((DISTANCE > 0) && (DISTANCE <= (upper_limit_angle-upper_conc_angle)*1.7))	//불필요한 카운팅을 없애기 위함
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
					}										//집중각도 5도에서 카운팅되지않아서 기준을 1.2배에서 1.7배로 늘림(23.01.17)
					else if((DISTANCE > 0) && (DISTANCE <= (upper_limit_angle-upper_conc_angle)*1.7))	//불필요한 카운팅을 없애기 위함
						toggle_cnt++;														//ex) 상한집중운동 종료후 하한각 이동시 카운팅 하지 않음
					
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
		else if(Dir_of_rotation == direction_2)	//반시계방향
		{
			//적용위치에 따른 목표 각도 설정
			if(location == UPPER)			//적용위치 : 상한각
			{
				if(repeat_ok)		//상한각에서 반복횟수만큼 집중운동을 하고, 하한각으로 이동
				{
					target_angle = lower_limit_angle;
					Focus_f = 0;	//하한각이동
				}
				else
				{
					target_angle = upper_conc_angle;
					Focus_f = 3;	//상한 집중각 이동
				}
			}
			else if(location == LOWER)		//적용위치 : 하한각
			{
				target_angle = lower_limit_angle;
				Focus_f = 0;	//하한각이동
			}
			else if(location == UP_LOW)		//적용위치 : 상하한각
			{
				if((exerc_cnt+1)%2 == 1)		//상한 집중
				{
					target_angle = upper_conc_angle;
					Focus_f = 3;	//상한 집중각 이동
				}
				else if((exerc_cnt+1)%2 == 0)	//하한 집중
				{
					target_angle = lower_limit_angle;
					Focus_f = 0;	//하한각이동
				}
			}
			
			angle_adjustment_PI(target_angle, speed_step, CONSTANT_MODE);	//집중운동은 등속모드만 가능
			if(motor_delay_ms(lower_limit_time))	//설정한 하한정지시간이 되면 1 
			{
				if(target_angle >= Current_angle)
				{
					Dir_of_rotation = direction_1;	//시계방향으로 전환
					if((location == LOWER) && (exerc_time > 0))		//하한각, 운동시간 모드일 경우
					{
						if((DISTANCE > 0) && (DISTANCE <= (lower_conc_anlge-lower_limit_angle)*1.7) && (DISTANCE >= (lower_conc_anlge-lower_limit_angle)*0.7))	//불필요한 카운팅을 없애기 위함
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
					else if((DISTANCE > 0) && (DISTANCE <= (lower_conc_anlge-lower_limit_angle)*1.7) && (DISTANCE >= (lower_conc_anlge-lower_limit_angle)*0.7))	//불필요한 카운팅을 없애기 위함
						toggle_cnt++;																										//ex) 상한집중운동 종료후 하한각 이동시 카운팅 하지 않음
					
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
			exerc_cnt++;	//상한각 또는 하한각에서 반복횟수를 모두 채울때 마다 exerc_cnt 1씩 증가(운둥횟수)
			if(location != UP_LOW)	//적용위치가 상한각 또는 하한각일 경우
				repeat_ok = 1;		//적용위치가 상한 또는 하한각일 경우, 반복횟수를 다채우면 하한 또는 상한각으로 이동하기 위한 flag(DISTANCE값 계산) 23.01.17			
			if(location == UP_LOW)	//적용위치가 상하한각
			{
				if(exerc_cnt%2 == 0)
					num_of_workouts++;	//LCD보드로 보낼 실제 운동횟수 데이터
			}
			else
				num_of_workouts++;
		}
		

		static word i;
		i = (location == UP_LOW) ? operating*2 : operating;
		if((num_of_exerc > 0) && exerc_cnt >= i)	//운동횟수 끝날시
		{
			exercise_mode = 0;	//집중운동 모드 종료
			deaccel_flag = 0;	
			pwm_step = 0;
			exerc_end = 1;		//운동 종류 후, 하한각으로 이동을 위한 flag
			mode_flag = 10;		//운동 종류 후, 하한각 이동시 DISTANCE값 계산을 위한 flag
			exerc_cnt = 0;
			
			repeat_ok = 0;
			fake_cnt = 0;
		}
		else if((exerc_time > 0) && (Time_mode_end == 1)) //운동시간 끝날시 
		{
			_SENSOR_DATA.state = 2;	//정지중(하한각이동중)
			mode_flag = 10;		//운동 종류 후, 하한각 이동시 DISTANCE값 계산을 위한 flag
			if(Dir_of_rotation == direction_1)
			{
				deaccel_flag = 1;	//상하한각 집중운동 모드일 경우, 상한(또는 하한)집중에서 하한(또는 상한)집중으로 이동하고 있을때 운동시간이 끝나게 되면 감속시작
				if((upper_limit_angle <= Current_angle) || (pwm_step <= 0))	//감속시작 후, 현재각도가 상한각 이상이 되거나 감속이 끝나면 집중운동모드 종료
				{
					exercise_mode = 0;
					deaccel_flag = 0;
					pwm_step = 0;
					exerc_end = 1;		//운동 종류 후, 하한각으로 이동을 위한 flag
					Time_mode_end = 0;
					exerc_cnt = 0;
					repeat_ok = 0;
					fake_cnt = 0;
				}
			}
			else if(Dir_of_rotation == direction_2)
			{
				deaccel_flag = 1;	//상하한각 집중운동 모드일 경우, 상한(또는 하한)집중에서 하한(또는 상한)집중으로 이동하고 있을때 운동시간이 끝나게 되면 감속시작
				if((lower_limit_angle >= Current_angle) || (pwm_step <= 0)) //감속시작 후, 현재각도가 하한각 이하가 되거나 감속이 끝나면 집중운동모드 종료
				{
					exercise_mode = 0;
					deaccel_flag = 0;
					pwm_step = 0;
					exerc_end = 1;		//운동 종류 후, 하한각으로 이동을 위한 flag
					Time_mode_end = 0;
					exerc_cnt = 0;
					repeat_ok = 0;
					fake_cnt = 0;
				}
			}
		}
	}
}


/*******************각도 보정 기록*******************/
float up_corr = 0.2, down_corr = 0.2;
float total_high_angle= 0, total_low_angle= 0;
byte toggle_cnt_exception = 0, init_skip_flag = 0;
void Exercise_ctrl()//10ms task
{//LCD 컨트롤러 제어 부분(_USER_RUN_SETTING 구조체의 데이터 사용)	
	if(_USER_RUN_SETTING.exerc_start == 0x01 && (_SENSOR_DATA.state == 1 || _SENSOR_DATA.state == 4))//LCD제어보드로 부터 운동시작 신호가 왔을 때
	{//대기중 or 일시정지 완료 상태에서만 시작이 가능           //대기중                 //일시정지 완료
		PWM_NOT_SLEEP;
		if(_SENSOR_DATA.state != 4)
			init_skip_flag = 0;
		if(exercise_mode == 0x09)	//측정모드 일시정지 상태에서 운동을 시작할때
		{
			voice_en = 1;
			exercise_mode = 0x00; 		//0으로 초기화
			Measurement_CNT = 0;		//측정 횟수 초기화
			_LoRa_Data.state = 1;		//운동 종료(PC보드에 보낼 데이터)
			_SENSOR_DATA.state = 1;		//운동 종료(LCD보드에 보낼 데이터)
			timer2_ms_cnt_start = 0;	//정지 카운터 flag 종료
		}
		if(exercise_mode == 0) //일반운동
		{
			min_cnt = 0x00;		//운동 시작시 남은시간 계산을 위한 min_cnt을 0으로 초기화
			num_of_workouts = 0x00;	//운동 시작시 진행 운동 횟수 계산을 위한 num_of_workouts을 0으로 초기화]
			_USER_RUN_SETTING.exerc_start = 0x00;
			_LoRa_Data.state = 0;		//운동중(PC보드에 보낼 데이터)
			_SENSOR_DATA.state = 0;		//운동중(LCD보드에 보낼 데이터)
		}
		//상한각, 하한각 설정
		set_min_angle = _USER_RUN_SETTING.lower_angle;				//하한각
		set_max_angle = _USER_RUN_SETTING.upper_angle;				//상한각
		_Ble_Data.upper_angle = (byte)(set_max_angle + 50.0);//상한각;
		_Ble_Data.lower_angle = (byte)(set_min_angle + 70.0);//하한각;
		_LoRa_Data.upper_angle = (byte)(set_max_angle + 50.0);//상한각;
		_LoRa_Data.lower_angle = (byte)(set_min_angle + 70.0);//하한각;	
		if(_USER_RUN_SETTING.mode == 1 || _USER_RUN_SETTING.mode == 2)//적응 집중일 경우
		{
			/*if(bf_lcd_location != set_location)
			{		
				set_location = _USER_RUN_SETTING.application_location;//적용위치
				bf_lcd_location = set_location;	
			}			*/	
			if(Tx_lora_data)
			{
				Tx_lora_data = 0;
				set_location = _LoRa_Data.special_location;//적용위치
				_SENSOR_DATA.special_location = _LoRa_Data.special_location;
			}
			else 
				set_location = _USER_RUN_SETTING.application_location;//적용위치	
		}		
		switch(_USER_RUN_SETTING.mode)
		{
			case 0:	//일반 운동 모드	
				if(exercise_mode == 0)	//운동을 처음시작했을 때만 운동모드를 NORMAL로 설정해야함. 일시정지 상태에서 다시 시작할때는 설정되면 안됨(22.02.07)
					exercise_mode = NORMAL;
				set_velocity_mode = _USER_RUN_SETTING.speed_mode;			//속도모드
				set_speed_step = _USER_RUN_SETTING.speed;					//운동속도
				_Ble_Data.velocity_mode = _USER_RUN_SETTING.speed;//운동 속도 단계
				_LoRa_Data.velocity_mode = _USER_RUN_SETTING.speed;//운동 속도 단계						
				set_lower_limit_time = _USER_RUN_SETTING.lower_stop_time * 1000;	//하한정지시간(s->ms 환산)
				set_upper_limit_time = _USER_RUN_SETTING.upper_stop_time * 1000;	//상한정지시간(s->ms 환산)
				_Ble_Data.stop_time = 0x0F & _USER_RUN_SETTING.lower_stop_time;	//하한정지시간(s->ms 환산) 하위 4bit
				_Ble_Data.stop_time |= 0xF0 & (_USER_RUN_SETTING.upper_stop_time<<4);	//상한정지시간(s->ms 환산) 상위 4bit
				_LoRa_Data.stop_time = 0x0F & _USER_RUN_SETTING.lower_stop_time;	//하한정지시간(s->ms 환산) 하위 4bit
				_LoRa_Data.stop_time |= 0xF0 & (_USER_RUN_SETTING.upper_stop_time<<4);	//상한정지시간(s->ms 환산) 상위 4bit			
				if(_USER_RUN_SETTING.cnt_mode == 0x01)	//운동횟수 모드일때
				{
					set_exerc_num = _USER_RUN_SETTING.exerc_num;			//운동횟수
					_Ble_Data.exerc_time_and_exercnum = (0x80 | _USER_RUN_SETTING.exerc_num);//운동횟수//MSB : 1
					_LoRa_Data.exerc_time_and_exercnum = (0x80 | _USER_RUN_SETTING.exerc_num);//운동횟수//MSB : 1				
					set_exerc_time = 0;
				}
				else if(_USER_RUN_SETTING.cnt_mode == 0x00)	//운동시간 모드일때
				{
					set_exerc_time = _USER_RUN_SETTING.exerc_time;			//운동시간
					_Ble_Data.exerc_time_and_exercnum = (0x7F & _USER_RUN_SETTING.exerc_time);//운동횟수//MSB : 0
					_LoRa_Data.exerc_time_and_exercnum = (0x7F & _USER_RUN_SETTING.exerc_time);//운동횟수//MSB : 0					
					set_exerc_num = 0;
				}			
				break;
				
			case 1:	//적응 운동 모드				
				if(exercise_mode == 0)	//운동을 처음시작했을 때만 운동모드를 NORMAL로 설정해야함. 일시정지 상태에서 다시 시작할때는 설정되면 안됨(22.02.07)
					exercise_mode = ADAPTIVE;
				
				set_PA = _USER_RUN_SETTING.adaptation_angle;				//적응각도
				_Ble_Data.special_angle =  _USER_RUN_SETTING.adaptation_angle;//적응각도
				_LoRa_Data.special_angle =  _USER_RUN_SETTING.adaptation_angle;//적응각도
				set_speed_step = _USER_RUN_SETTING.speed;					//운동속도
				_Ble_Data.velocity_mode = _USER_RUN_SETTING.speed;//운동 속도 단계
				_LoRa_Data.velocity_mode = _USER_RUN_SETTING.speed;//운동 속도 단계				

				set_lower_limit_time = _USER_RUN_SETTING.lower_stop_time * 1000;	//하한정지시간(s->ms 환산)
				set_upper_limit_time = _USER_RUN_SETTING.upper_stop_time * 1000;	//상한정지시간(s->ms 환산)
				_Ble_Data.stop_time = 0x0F & _USER_RUN_SETTING.lower_stop_time;	//하한정지시간(s->ms 환산) 하위 4bit
				_Ble_Data.stop_time |= 0xF0 & (_USER_RUN_SETTING.upper_stop_time<<4);	//상한정지시간(s->ms 환산) 상위 4bit
				_LoRa_Data.stop_time = 0x0F & _USER_RUN_SETTING.lower_stop_time;	//하한정지시간(s->ms 환산) 하위 4bit
				_LoRa_Data.stop_time |= 0xF0 & (_USER_RUN_SETTING.upper_stop_time<<4);	//상한정지시간(s->ms 환산) 상위 4bit						
				set_repeat_num = _USER_RUN_SETTING.repeat_num;				//반복횟수
				_Ble_Data.repeat_num = _USER_RUN_SETTING.repeat_num;//반복횟수
				_LoRa_Data.repeat_num = _USER_RUN_SETTING.repeat_num;//반복횟수
				if(_USER_RUN_SETTING.cnt_mode == 0x01)	//운동횟수 모드일때
				{
					set_exerc_num = _USER_RUN_SETTING.exerc_num;			//운동횟수
					_Ble_Data.exerc_time_and_exercnum = (0x80 | _USER_RUN_SETTING.exerc_num);//운동횟수//MSB : 1
					_LoRa_Data.exerc_time_and_exercnum = (0x80 | _USER_RUN_SETTING.exerc_num);//운동횟수//MSB : 1				
					set_exerc_time = 0;
				}
				else if(_USER_RUN_SETTING.cnt_mode == 0x00)	//운동시간 모드일때
				{
					set_exerc_time = _USER_RUN_SETTING.exerc_time;			//운동시간
					_Ble_Data.exerc_time_and_exercnum = (0x7F & _USER_RUN_SETTING.exerc_time);//운동횟수//MSB : 0
					_LoRa_Data.exerc_time_and_exercnum = (0x7F & _USER_RUN_SETTING.exerc_time);//운동횟수//MSB : 0					
					set_exerc_num = 0;
				}				
				set_exerc_time = 0;
				break;
				
			case 2:	//집중 운동 모드				
				if(exercise_mode == 0)	//운동을 처음시작했을 때만 운동모드를 NORMAL로 설정해야함. 일시정지 상태에서 다시 시작할때는 설정되면 안됨(22.02.07)
					exercise_mode = INTENSIVE;
				
				set_conc_angle = _USER_RUN_SETTING.concentration_angle;		//집중각도
				_Ble_Data.special_angle =  _USER_RUN_SETTING.concentration_angle;//집중각도
				_LoRa_Data.special_angle =  _USER_RUN_SETTING.concentration_angle;//집중각도
				set_speed_step = _USER_RUN_SETTING.speed;					//운동속도
				
				_Ble_Data.velocity_mode = _USER_RUN_SETTING.speed;//운동 속도 단계
				_LoRa_Data.velocity_mode = _USER_RUN_SETTING.speed;//운동 속도 단계			
				set_lower_limit_time = _USER_RUN_SETTING.lower_stop_time * 1000;	//하한정지시간(s->ms 환산)
				set_upper_limit_time = _USER_RUN_SETTING.upper_stop_time * 1000;	//상한정지시간(s->ms 환산)
				_Ble_Data.stop_time = 0x0F & _USER_RUN_SETTING.lower_stop_time;	//하한정지시간(s->ms 환산) 하위 4bit
				_Ble_Data.stop_time |= 0xF0 & (_USER_RUN_SETTING.upper_stop_time<<4);	//상한정지시간(s->ms 환산) 상위 4bit
				_LoRa_Data.stop_time = 0x0F & _USER_RUN_SETTING.lower_stop_time;	//하한정지시간(s->ms 환산) 하위 4bit
				_LoRa_Data.stop_time |= 0xF0 & (_USER_RUN_SETTING.upper_stop_time<<4);	//상한정지시간(s->ms 환산) 상위 4bit					
				set_repeat_num = _USER_RUN_SETTING.repeat_num;				//반복횟수
				_Ble_Data.repeat_num = _USER_RUN_SETTING.repeat_num;//반복횟수
				_LoRa_Data.repeat_num = _USER_RUN_SETTING.repeat_num;//반복횟수				
				if(_USER_RUN_SETTING.cnt_mode == 0x01)	//운동횟수 모드일때
				{
					set_exerc_num = _USER_RUN_SETTING.exerc_num;			//운동횟수
					_Ble_Data.exerc_time_and_exercnum = (0x80 | _USER_RUN_SETTING.exerc_num);//운동횟수//MSB : 1
					_LoRa_Data.exerc_time_and_exercnum = (0x80 | _USER_RUN_SETTING.exerc_num);//운동횟수//MSB : 1				
					set_exerc_time = 0;
				}
				else if(_USER_RUN_SETTING.cnt_mode == 0x00)	//운동시간 모드일때
				{
					set_exerc_time = _USER_RUN_SETTING.exerc_time;			//운동시간
					_Ble_Data.exerc_time_and_exercnum = (0x7F & _USER_RUN_SETTING.exerc_time);//운동횟수//MSB : 0
					_LoRa_Data.exerc_time_and_exercnum = (0x7F & _USER_RUN_SETTING.exerc_time);//운동횟수//MSB : 0					
					set_exerc_num = 0;
				}
				break;
			default:
				break;
		}
	}
	else if(_USER_RUN_SETTING.exerc_start == 0x04)	//수동모드 UP(1도)
	{
		PWM_NOT_SLEEP;
		_USER_RUN_SETTING.exerc_start = 0x00;
		exercise_mode = 0x05;		//수동모드 UP(1도)
		_LoRa_Data.use_state = 0x04;
		_LoRa_Data.state = 0; // 운동중
		_SENSOR_DATA.state = 0; // 운동중
	}
	else if(_USER_RUN_SETTING.exerc_start == 0x05)	//수동모드 DOWN(1도)
	{
		PWM_NOT_SLEEP;
		_USER_RUN_SETTING.exerc_start = 0x00;
		exercise_mode = 0x06;		//수동모드 DOWN(1도)
		_LoRa_Data.use_state = 0x04;
		_LoRa_Data.state = 0; // 운동중
		_SENSOR_DATA.state = 0; // 운동중		
	}
	else if(_USER_RUN_SETTING.exerc_start == 0x06)	//수동모드 키다운 UP ing(누르고 있으면 지정한 속도대로 계속 올라감)
	{
		PWM_NOT_SLEEP;
		_USER_RUN_SETTING.exerc_start = 0x00;
		exercise_mode = 0x07;		//수동모드 키다운 UP
		_LoRa_Data.state = 0; // 운동중
		_LoRa_Data.use_state = 0x04;
		_SENSOR_DATA.state = 0; // 운동중		
	}
	else if(_USER_RUN_SETTING.exerc_start == 0x07)	//수동모드 키다운 DOWN ing(누르고 있으면 지정한 속도대로 계속 내려감)
	{
		PWM_NOT_SLEEP;
		_USER_RUN_SETTING.exerc_start = 0x00;
		_LoRa_Data.use_state = 0x04;
		exercise_mode = 0x08;		//수동모드 키다운 DOWN
		_LoRa_Data.state = 0; // 운동중
		_SENSOR_DATA.state = 0; // 운동중		
	}
	else if(_USER_RUN_SETTING.exerc_start == 0x08)	//ROM(Range of Motion) 측정모드
	{
		PWM_NOT_SLEEP;
		_VOICE_DATA.x06_1B = 0x13; //"측정을 시작합니다" 음성출력	
		Measurement_mode = 1;
		_USER_RUN_SETTING.exerc_start = 0x00;
		_LoRa_Data.state = 0; // 운동중
		_SENSOR_DATA.state = 0; // 운동중
		exercise_mode = 0x09;	//측정모드
	}
//	else 
//	{
//		static byte no_action = 0;
//	}

	static byte bf_mode = 0;
	if(bf_mode != exercise_mode)	//exercis_mode가 바뀌었을 때 한번만 수행됨
	{	//23.01.12 둥동시작시 target_speed = 0 으로 초기화 추가
		if(init_skip_flag)	//일시정지 후, 다시 시작버튼을 눌렀을 때 운동진행상태를 초기화하지 않음
		{
			init_skip_flag = 0;
		}
		else
		{
			switch(exercise_mode)
			{
				case NORMAL:	//일반운동
					pwm_step = 0;
					deaccel_flag = 0;
					mode_flag = 1;
					toggle_cnt = 0;
					exerc_cnt = 0;
					exerc_end = 0;
					target_speed = 0;
					break;
				case ADAPTIVE:	//적응운동
					pwm_step = 0;
					deaccel_flag = 0;
					mode_flag = 2;
					toggle_cnt = 0;
					exerc_cnt = 0;
					exerc_end = 0;
					angle_step = 0;
					target_speed = 0;
					break;
				case INTENSIVE:	//집중운동
					pwm_step = 0;
					deaccel_flag = 0;
					mode_flag = 3;
					toggle_cnt = 0;
					exerc_cnt = 0;
					exerc_end = 0;
					target_speed = 0;
					break;
				case ADJUSTMENT: //설정각 이동
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
				case 0x05:	//수동모드 UP(1도)
					if(!Passive_ing)	//수동조작으로 이동중이 아닐때 0으로 초기화(23.02.08)
						pwm_step = 0;
					deaccel_flag = 0;
					mode_flag = 5;
					target_speed = 0;
					break;
				case 0x06:	//수동모드 DOWN(1도)
					if(!Passive_ing)	//수동조작으로 이동중이 아닐때 0으로 초기화(23.02.08)
						pwm_step = 0;
					deaccel_flag = 0;
					mode_flag = 6;
					target_speed = 0;
					break;
				case 0x07: //수동모드 키다운 UP ING
					pwm_step = 0;
					deaccel_flag = 0;
					mode_flag = 7;
					target_speed = 0;
					break;
				case 0x08: //수동모드 키다운 DOWN ING
					pwm_step = 0;
					deaccel_flag = 0;
					mode_flag = 8;
					target_speed = 0;
					break;
				case 0x09:	//측정모드
					pwm_step = 0;
					deaccel_flag = 0;
					mode_flag = 9;
					target_speed = 0;
					break;
				case 0x10:	//일시정지시
					break;
				case 0x20:	//일시정지시
					break;
				case 0x30:	//일시정지시
					break;
				default:	//운동 종료
					deaccel_flag = 0;
					Motor_PWM = 0;
					target_speed = 0;
					break;
			}
		}
		bf_mode = exercise_mode;
	}
	
	
	
	//수동조작 키다운, Soft Stop용
	static word RPM_num;
	switch(_USER_RUN_SETTING.speed)		//속도단계에 따른 모터 rpm 설정(7단계까지 설정가능)
	{									//DC모터와 기구물의 기어비 = 1500:1
		case 1:
			RPM_num = 460;	//기구물 RPM = 0.277
			break;
		case 2:
			RPM_num = 665; //기구물 RPM = 0.37
			break;
		case 3:
			RPM_num = 830; //기구물 RPM = 0.463
			break;
		case 4:
			RPM_num = 1015; //기구물 RPM = 0.556
			break;
		case 5:
			RPM_num = 1200; //기구물 RPM = 0.639
			break; 
		case 6:
			RPM_num = 1340; //기구물 RPM = 0.789
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

		case NORMAL:	//일반운동 모드
			if(mode_flag == 1)		//일반 운동 시작시 한번만 수행
			{
				if(set_min_angle > Current_angle) 		//현재각도가 하한각도보다 작은 경우
				{
					//무릎기구 방향 및 각도 조정 테스트 시작(23.02.02)
					if(_USER_RUN_SETTING.motion == Knee)	//무릎
						Dir_of_rotation = CCW;				//반시계방향으로
					else	//어깨, 팔꿈치
						Dir_of_rotation = CW;				//시계방향으로
					
					
					//22.12.19. 1도 각도보정
					DISTANCE = (set_min_angle-1) - Current_angle;	//기준 각도차 계산
					//correction_angle = (set_min_angle);
					
					distance_cal_f = 1;		//DISTANCE값이 방향전환시에만 계산하도록 되어있어서 운동시작시 현재각도가 하한각보다 작은 경우 DISTANCE를 계산하기위한 flag(23.01.17)
				}
				else if(set_min_angle <= Current_angle)	//현재각도가 하한각 이상일 경우
				{
					//무릎기구 방향 및 각도 조정 테스트 시작(23.02.02)
					if(_USER_RUN_SETTING.motion == Knee)	//무릎
						Dir_of_rotation = CW;				//시계방향으로
					else	//어깨, 팔꿈치, 손목, 발목
						Dir_of_rotation = CCW;				//반시계방향으로
					
					
					//22.12.19. 1도 각도보정
					DISTANCE = Current_angle - (set_min_angle+1);	//기준 각도차 계산
					toggle_cnt_exception = 1;	//toggle_cnt 예외처리를 위한 flag
				}
				move_to_lower = 1;	//일반운동 처음 실행시 하한각으로 먼저 이동하기 위한 flag
				mode_flag = 0;
			}
			
			static byte bf_move = 1;
			if(bf_move != move_to_lower)	//move_to_lower가 1에서 0이 될때 한번만 수행
			{
				if((move_to_lower == 0) && (set_exerc_num == 0))	//운동횟수 모드가 아닐때
					exerc_time_start = 1;	//운동시간 카운팅 시작 flag ON
				bf_move = move_to_lower;		//과거 <- 현재
			}
			if(move_to_lower)	//처음 일반운동 실행시 하한각으로 먼저 이동
			{					//1도 각도보정, 하한각 이동하는 속도를 조절하는 위치
				if(set_speed_step < 8)//1,2,3,4,5,6,7
					angle_adjustment_PI(set_min_angle, set_speed_step, CONSTANT_MODE);
				else if(set_speed_step >= 8)	//8,9단계 이상일 때만 속도 7단계로 고정
					angle_adjustment_PI(set_min_angle, 7, CONSTANT_MODE);				
			}
			else if((set_min_angle < set_max_angle) && (move_to_lower == 0))//하한각에서 일반운동 시작
			{	
				if(gen_ada_con_back == 0)
				{
					PWM_NOT_SLEEP;
					gen_ada_con_back = 1;
					_VOICE_DATA.x06_1B = 0x07;  //"운동을 시작합니다" 음성출력	
					voice_en = 1;
				}
				if(distance_cal_f)
				{
					distance_cal_f = 0;
					DISTANCE = set_max_angle - Current_angle;	//운동 방향이 바뀌고 이동할 거리(각도) 계산
				}
					//22.12.19. 1도 각도보정
				if(set_speed_step >= 6)	//6단계 이상일 때만 각도보정(상한각, 하한각 1도 보정)
					Normal_exercise(set_min_angle, set_max_angle, set_velocity_mode, set_speed_step, set_exerc_num, set_exerc_time, set_lower_limit_time, set_upper_limit_time);				
				else if(set_speed_step < 6)	//6단계 미만일 경우, 상한각 0.5도 보정
					Normal_exercise(set_min_angle, set_max_angle, set_velocity_mode, set_speed_step, set_exerc_num, set_exerc_time, set_lower_limit_time, set_upper_limit_time);
			}				
			//예외처리 : 일반운동 모드 처음 실행시 현재각도보다 상한각이 클 경우, toggle_cnt(횟수)카운팅이 한번더 되기 때문에 -1 해줌
			if(toggle_cnt_exception == 1)
			{
				toggle_cnt = toggle_cnt - 1;	//일반운동 모드에서는 toggle_cnt가 운동횟수를 의미함
				toggle_cnt_exception = 0;
			}
			break;
		case ADAPTIVE:	//적응운동 모드
			if(mode_flag == 2)		//일반 운동 시작시 한번만 수행
			{
				if(set_min_angle > Current_angle) 		//현재각도가 하한각도보다 작은 경우
				{
					if(_USER_RUN_SETTING.motion == Knee)	//무릎
						Dir_of_rotation = CCW;				//반시계방향으로
					else	//어깨, 팔꿈치
						Dir_of_rotation = CW;				//시계방향으로
					
					//22.12.19. 1도 각도보정
					DISTANCE = (set_min_angle-1) - Current_angle;	//기준 각도차 계산
					//correction_angle = (set_min_angle-1);
					
					distance_cal_f = 1;	//DISTANCE값이 방향전환시에만 계산하도록 되어있어서 운동시작시 현재각도가 하한각보다 작은 경우 DISTANCE를 계산하기위한 flag(23.01.17)
				}
				else if(set_min_angle <= Current_angle)	//현재각도가 하한각 이상일 경우
				{
					if(_USER_RUN_SETTING.motion == Knee)	//무릎
						Dir_of_rotation = CW;				//시계방향으로
					else	//어깨, 팔꿈치, 손목, 발목
						Dir_of_rotation = CCW;		//반시계방향으로
					
					//22.12.19. 1도 각도보정
					DISTANCE = Current_angle - (set_min_angle+1);	//기준 각도차 계산
					//correction_angle = (set_min_angle+1);
					
					toggle_cnt_exception = 1;	//toggle_cnt 예외처리를 위한 flag
				}
				move_to_lower = 1;	//일반운동 처음 실행시 하한각으로 먼저 이동하기 위한 flag
				mode_flag = 0;
			}
			
			
			if(move_to_lower)	//처음 일반운동 실행시 하한초기각도(하한각도+적응각도)로 이동
			{			//22.12.19. 1도 각도보정
				if(set_location == UPPER)	//적용위치가 상한각일때는 처음에 하한각으로 이동
				{//하한각 이동하는 속도를 조절하는 위치
					if(set_speed_step < 8)//1~7단계
						angle_adjustment_PI(set_min_angle, set_speed_step, CONSTANT_MODE);	
					else if(set_speed_step >= 8)	//8,9단계 이상일 때만 각도보정
						angle_adjustment_PI(set_min_angle, 7, CONSTANT_MODE);					
				}
				else
				{
					if(set_speed_step >= 8)	//8, 9단계 이상일 때만 각도보정
						angle_adjustment_PI(set_min_angle+set_PA ,7 , CONSTANT_MODE);	
					else if(set_speed_step < 8)	//1 ~ 7단계 이상일 때만 각도보정
						angle_adjustment_PI(set_min_angle+set_PA, set_speed_step, CONSTANT_MODE);	
				}
			}
			else if((set_min_angle < set_max_angle) && (move_to_lower == 0))	//하한각에서 적응운동 시작
			{
				if(gen_ada_con_back == 0)
				{
					gen_ada_con_back = 1;
					_VOICE_DATA.x06_1B = 0x07;  //"운동을 시작합니다" 음성출력	
					voice_en = 1;
				}				
				if(distance_cal_f)
				{
					distance_cal_f = 0;
					if(set_location == LOWER)	//적용위치가 하한각일 경우
						DISTANCE = set_max_angle - Current_angle;	//DISTANCE 계산
					else	//적용위치가 상한각, 상하한각일 경우
						DISTANCE = (set_max_angle - set_PA) - Current_angle;
				}
				//22.12.19. 1도 각도보정
				if(set_speed_step >= 6)	//6단계 이상일 때만 각도보정
					Adaptive_exercise(set_min_angle, set_max_angle, set_PA, set_repeat_num, set_location, set_speed_step, set_lower_limit_time, set_upper_limit_time);	//적응운동은 운동시간 설정 불가능
				else if(set_speed_step < 6)
					Adaptive_exercise(set_min_angle, set_max_angle, set_PA, set_repeat_num, set_location, set_speed_step, set_lower_limit_time, set_upper_limit_time);
			}
			//예외처리 : 적응운동 모드 처음 실행시 현재각도보다 상한각이 클 경우, toggle_cnt(횟수)카운팅이 한번더 되기 때문에 -1 해줌
			if(toggle_cnt_exception == 1)
			{
				toggle_cnt = toggle_cnt - 1;
				toggle_cnt_exception = 0;
			}
			break;
			
		case INTENSIVE:	//집중운동 모드
			if(mode_flag == 3)		//집중 운동 시작시 한번만 수행
			{
				if(set_min_angle > Current_angle) 		//현재각도가 하한각도보다 작은 경우
				{
					if(_USER_RUN_SETTING.motion == Knee)	//무릎
						Dir_of_rotation = CCW;				//반시계방향으로
					else	//어깨, 팔꿈치, 손목, 발목
						Dir_of_rotation = CW;				//시계방향으로
					
					//22.12.19. 1도 각도보정
					DISTANCE = (set_min_angle-1) - Current_angle;	//기준 각도차 계산
					//correction_angle = (set_min_angle-1);
					
					distance_cal_f = 1;		//DISTANCE값이 방향전환시에만 계산하도록 되어있어서 운동시작시 현재각도가 하한각보다 작은 경우 DISTANCE를 계산하기위한 flag(23.01.17)
					
					Focus_f = 'U';
				}
				else if(set_min_angle <= Current_angle)	//현재각도가 하한각 이상일 경우
				{
					if(_USER_RUN_SETTING.motion == Knee)	//무릎
						Dir_of_rotation = CW;		//시계방향으로
					else
						Dir_of_rotation = CCW;		//반시계방향으로
					
					//22.12.19. 1도 각도보정
					DISTANCE = Current_angle - (set_min_angle+1);	//기준 각도차 계산
					//correction_angle = (set_min_angle+1);
					
					distance_cal_f = 1;		//하한각 도달 후, 집중운동 처음 시작시, DISTANCE 값 계산이 안되는 버그가 있어서 flag on(23.02.07)
					
					Focus_f = 'D';
				}
				move_to_lower = 1;	//집중운동 처음 실행시 하한각으로 먼저 이동하기 위한 flag
				mode_flag = 0;
				if(set_location == LOWER)	//적용위치가 하한각일 경우 상한각으로 먼저 이동하기 위함
					repeat_ok = 1;
			}
			
			static byte dir_f = 0;
			if(move_to_lower)	//처음 집중운동 실행시 하한각으로 먼저 이동
			{// 1도 각도보정, 하한각 이동하는 속도를 조절하는 위치
				//22.12.19. 1도 각도보정
				if(set_speed_step < 8)//1 ~ 7단계
					angle_adjustment_PI(set_min_angle, set_speed_step, CONSTANT_MODE);	
				else if(set_speed_step >= 8)//8,9단계
					angle_adjustment_PI(set_min_angle, 7, CONSTANT_MODE);
			
				
				//하한각 도달후, 집중운동 시작할때 목표 각도가 잘못설정되는 버그를 막기위함(22.02.06)
				dir_f = 1;
			}
			else if((set_min_angle < set_max_angle) && (move_to_lower == 0))	//하한각에서 집중운동 시작
			{
				if(gen_ada_con_back == 0)
				{
					gen_ada_con_back = 1;
					_VOICE_DATA.x06_1B = 0x07;  //"운동을 시작합니다" 음성출력	
					voice_en = 1;
				}				
				if(distance_cal_f)
				{
					distance_cal_f = 0;
					DISTANCE = set_max_angle - Current_angle;	//DISTANCE 계산
					if((set_location == LOWER) && (_USER_RUN_SETTING.cnt_mode == 0))
						exerc_time_start = 1;
				}
				
				if(dir_f)
				{
					dir_f = 0;
					//처음에 Target_Angle 설정 오류 버그때문에 집중운동 함수 들어가기 전에 운동방향 설정해줌(무릎은 반시계 방향이고  //어깨, 팔꿈치, 손목, 발목는 시계 방향으로 미리 설정) 22.02.06
					if(_USER_RUN_SETTING.motion == Knee)
						Dir_of_rotation = CCW;
					else
						Dir_of_rotation = CW;
					
					except_f = 1;	//각도제한 예외(적용위치 상한각)
				}

				//22.12.19. 1도 각도보정
				if(set_speed_step >= 6)	//6단계 이상일 때만 각도보정
					Intensive_exercise(set_min_angle, set_max_angle, set_conc_angle, set_repeat_num, set_location, set_speed_step, set_exerc_num, set_exerc_time, set_lower_limit_time, set_upper_limit_time);
				else if(set_speed_step < 6)
					Intensive_exercise(set_min_angle, set_max_angle, set_conc_angle, set_repeat_num, set_location, set_speed_step, set_exerc_num, set_exerc_time, set_lower_limit_time, set_upper_limit_time);
			}
			break;
		case ADJUSTMENT: //각도 조정
			if(mode_flag == 4)	//한번만 수행
			{
				if(set_angle_value > Current_angle)	//설정 각도가 현재 각도보다 클 때
				{	
					if(_USER_RUN_SETTING.motion == Knee)	//무릎
						Dir_of_rotation = CCW;		//반시계방향으로
					else	//어깨, 팔꿈치, 손목, 발목
						Dir_of_rotation = CW;				//시계방향으로
					TIM1->CNT = 0xffff;	//엔코더 각도측정 테스트중(모터 운동시작할때 CNT를 0xffff으로 초기화)
					bf_CNT = 0xffff;	//엔코더 각도측정 테스트
					Start_angle_1[Record_Index] = Current_angle;	//시작 각도 저장(엔코더 각도측정테스트)
					
					DISTANCE = set_angle_value - Current_angle;	//기준 각도차 계산
				}
				else if(set_angle_value < Current_angle)
				{
					if(_USER_RUN_SETTING.motion == Knee)	//무릎
						Dir_of_rotation = CW;		//시계방향으로
					else	//어깨, 팔꿈치, 손목, 발목
						Dir_of_rotation = CCW;		//반시계방향으로
					
					TIM1->CNT = 0x0000;	//엔코더 각도측정 테스트중(모터 운동시작할때 CNT를 0으로 초기화)
					bf_CNT = 0x0000;	//엔코더 각도측정 테스트
					Start_angle_2[Record_Index] = Current_angle;	//시작 각도 저장(엔코더 각도측정테스트)
					
					DISTANCE = Current_angle - set_angle_value;	//기준 각도차 계산
				}
				
				mode_flag = 0;
				Under_cnt = 0;	//엔코더 각도측정 테스트
				Over_cnt = 0;	//엔코더 각도측정 테스트
				
				OFFSET_angle = P_M_actual;	//엔코더 각도측정에 필요
			}
			Record_F = 1;
			
			angle_adjustment_PI(set_angle_value, set_speed_step, set_velocity_mode);
			
			break;
		static float target_angle_1 = 0.0;	
		case 0x05: 	//수동모드 UP(1도)
			if(Passive_ing)		//수동조작으로 이동중일 때
			{
				if (GPIOD->ODR & GPIO_PIN_10) // PD10의 상태가 1일 때 실행할 코드
				{//모터가 돌아갈 중에만 각도값을 보냄
					_SENSOR_DATA.current_angle = Current_angle;
				}		
				if(--pwm_step > 300)	//step이 0밑으로 떨어져 65535 되는걸 방지 (감속시간 => 10x100ms)
					pwm_step = 0;
						
				target_speed = ((RPM_num-240)/(float)300)*pwm_step + 240; 	 //Soft Stop[soft stop의 최소속도는 240(duty=15%)]
				pid_val = PID_control_sys(target_speed, speed_RPM);
				Motor_PWM = (unsigned int)set_motor_pwm(pid_val);	//모터 PWM PID 제어
				
				if(pwm_step <= 0)
				{
					pwm_step = 0;
					Motor_PWM = 0;
					exercise_mode = 0;			
					_SENSOR_DATA.state = 1;		//운동 종료 and 대기중
					_LoRa_Data.state = 1;		//운동 종료 and 대기중
					Passive_ing = 0;	//수동조작으로 이동 종료
				}
			}
			else
			{
				if(mode_flag == 5)	//한번만 수행
				{
					if(_USER_RUN_SETTING.motion == Knee)	//무릎
					{
						if(Current_angle <= 0)
							target_angle_1 = (signed long)(Current_angle-0.5) - 1;
						else
							target_angle_1 = (signed long)(Current_angle+0.5) - 1;
					}
					else	//어깨, 팔꿈치, 손목, 발목
						target_angle_1 = (dword)(Current_angle+0.5) + 1;
					Dir_of_rotation = CW;	//시계방향으로
					DISTANCE = 1.0;			//기준 각도차 계산
					mode_flag = 0;
					if(_USER_RUN_SETTING.calibration == 0x01)	//Calibration 진행중 일때 
						Motor_Run_F = 1; 
				}
				
				if(_USER_RUN_SETTING.calibration == 0x01)	//Calibration 진행중 일때
				{
					if(Motor_Run_F == 1)
						Motor_PWM = 416;
					else	//500ms후 정지.
					{
						PWM_SLEEP;
						Motor_PWM = 0;
						exercise_mode = 0;
						_SENSOR_DATA.state = 1;
					}
				}
				else	//평상시
				{
					switch(_USER_RUN_SETTING.motion)
					{
						case Elbow:	//팔꿈치
							if(Current_angle >= 134.5)	//팔꿈치 상한각이상일 때, 동작안함
							{
								exercise_mode = 0;			
								_LoRa_Data.state = 1;		//운동 종료 and 대기중
								_SENSOR_DATA.state = 1;		//운동 종료 and 대기중
							}
							else
								angle_adjustment_PI(target_angle_1-up_corr, 1, CONSTANT_MODE);
							break;
						case Shoulder:	//어깨
							if(Current_angle >= 179.5)	//어깨 상한각이상일 때, 동작안함
							{
								exercise_mode = 0;			
								_LoRa_Data.state = 1;		//운동 종료 and 대기중
								_SENSOR_DATA.state = 1;		//운동 종료 and 대기중
							}
							else
								angle_adjustment_PI(target_angle_1-up_corr, 1, CONSTANT_MODE);
							break;
						case Knee:	//무릎
							if(Current_angle <= (-9.5))	//무릎 하한각이하일 때, 동작안함
							{
								exercise_mode = 0;			
								_LoRa_Data.state = 1;		//운동 종료 and 대기중
								_SENSOR_DATA.state = 1;		//운동 종료 and 대기중
							}
							else
								angle_adjustment_PI(target_angle_1/*+up_corr*/, 1, CONSTANT_MODE);
							break;
						case wrist:	//손목
							if(Current_angle >= 69.5)	//손목 상한각 이상일 때, 동작안함
							{
								exercise_mode = 0;			
								_LoRa_Data.state = 1;		//운동 종료 and 대기중
								_SENSOR_DATA.state = 1;		//운동 종료 and 대기중
							}
							else
								angle_adjustment_PI(target_angle_1-up_corr, 1, CONSTANT_MODE);
							break;
						case ankle:	//발목
							if(Current_angle >= 49.5)	//발목 상한각 이상일 때, 동작안함
							{
								exercise_mode = 0;			
								_LoRa_Data.state = 1;		//운동 종료 and 대기중
								_SENSOR_DATA.state = 1;		//운동 종료 and 대기중
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
		case 0x06:	//수동모드 DOWN(1도)
			if(Passive_ing)		//수동조작으로 이동중일 때
			{
				_SENSOR_DATA.current_angle = Current_angle;	
				if(--pwm_step > 300)	//step이 0밑으로 떨어져 65535 되는걸 방지 (감속시간 => 10x100ms)
					pwm_step = 0;	
				target_speed = ((RPM_num-240)/(float)300)*pwm_step + 240; 	 //Soft Stop[soft stop의 최소속도는 240(duty=15%)]
				pid_val = PID_control_sys(target_speed, speed_RPM);
				Motor_PWM = (unsigned int)set_motor_pwm(pid_val);	//모터 PWM PID 제어
				if(pwm_step <= 0)
				{
					pwm_step = 0;
					Motor_PWM = 0;
					exercise_mode = 0;			
					_LoRa_Data.state = 1;		//운동 종료 and 대기중
					_SENSOR_DATA.state = 1;		//운동 종료 and 대기중
					Passive_ing = 0;	//수동조작으로 이동 종료
				}
			}
			else
			{
				if(mode_flag == 6)	//한번만 수행
				{
					if(_USER_RUN_SETTING.motion == Knee)	//무릎
					{
						if(Current_angle <= 0)
							target_angle_2 = (signed long)(Current_angle-0.5) + 1;
						else
							target_angle_2 = (signed long)(Current_angle+0.5) + 1;
					}
					else	//어깨, 팔꿈치, 손목, 발목
						target_angle_2 = (dword)(Current_angle+0.5) - 1;
					
					Dir_of_rotation = CCW;	//반시계방향으로
					DISTANCE = 1.0;			//기준 각도차 계산
					mode_flag = 0;
					
					if(_USER_RUN_SETTING.calibration == 0x01)	//Calibration 모드일 때만
						Motor_Run_F = 1;
				}
				if(_USER_RUN_SETTING.calibration == 0x01)	//Calibration 진행중 일때
				{
					if(Motor_Run_F == 1)
						Motor_PWM = 416;
					else// 정지.                  //이 곳은 원본파일과 비교할 것
					{
						PWM_SLEEP;
						Motor_PWM = 0;
						exercise_mode = 0;
						_SENSOR_DATA.state = 1;//운동 종료 and 대기중 
					}
				}
				else	//평상시
				{
					switch(_USER_RUN_SETTING.motion)
					{
						case Elbow:	//팔꿈치
							if(Current_angle <= 0.4)	//팔꿈치 하한각일 때, 동작안함
							{
								exercise_mode = 0;			
								_LoRa_Data.state = 1;		//운동 종료 and 대기중
								_SENSOR_DATA.state = 1;		//운동 종료 and 대기중
							}
							else
								angle_adjustment_PI(target_angle_2/*+down_corr*/, 1, CONSTANT_MODE);
							break;
						case Shoulder://어깨
							if(Current_angle <= 20.4)	//어깨 하한각이하일 때, 동작안함
							{
								exercise_mode = 0;			
								_LoRa_Data.state = 1;		//운동 종료 and 대기중
								_SENSOR_DATA.state = 1;		//운동 종료 and 대기중
							}
							else
								angle_adjustment_PI(target_angle_2/*+down_corr*/, 1, CONSTANT_MODE);
							break;
						case Knee:	//무릎
							if(Current_angle >= 139.5)	//무릎 상한각이상일 때, 동작안함
							{
								exercise_mode = 0;			
								_LoRa_Data.state = 1;		//운동 종료 and 대기중
								_SENSOR_DATA.state = 1;		//운동 종료 and 대기중
							}
							else
								angle_adjustment_PI(target_angle_2-down_corr, 1, CONSTANT_MODE);
							break;
						case wrist:	//손목
							if(Current_angle <= (-69.4))	//손목 하한각이하일 때, 동작안함
							{
								exercise_mode = 0;			
								_LoRa_Data.state = 1;		//운동 종료 and 대기중
								_SENSOR_DATA.state = 1;		//운동 종료 and 대기중
							}
							else
								angle_adjustment_PI(target_angle_2/*+down_corr*/, 1, CONSTANT_MODE);
							break;
						case ankle:	//발목
							if(Current_angle <= (-49.4))	//발목 하한각이하일 때, 동작안함
							{
								exercise_mode = 0;			
								_LoRa_Data.state = 1;		//운동 종료 and 대기중
								_SENSOR_DATA.state = 1;		//운동 종료 and 대기중
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
		case 0x07:	//수동모드 키다운 UP 계속 길게 누름
			if(mode_flag == 7)
			{			
				switch(_USER_RUN_SETTING.motion)
				{
					case Elbow:	//팔꿈치
						target_angle_3 = 135 - 0.5;	//0.5는 각도보정
						DISTANCE = 135 - Current_angle;		//기준 각도차 계산
						break;
					case Shoulder:	//어깨
						target_angle_3 = 180 - 0.5;	//0.5는 각도보정
						DISTANCE = 180 - Current_angle;		//기준 각도차 계산
						break;
					case Knee:	//무릎
						target_angle_3 = -10 + 0.5;	//0.5는 각도보정
						DISTANCE = Current_angle - (-10);	//기준 각도차 계산
						break;
					case wrist:	//손목
						target_angle_3 = 70 - 0.5;	//0.5는 각도보정
						DISTANCE = 70 - Current_angle;		//기준 각도차 계산
						break;
					case ankle:	//발목
						target_angle_3 = 50 - 0.5;	//0.5는 각도보정
						DISTANCE = 50 - Current_angle;		//기준 각도차 계산
						break;						
					default:
						break;
				}
				Dir_of_rotation = CW;			//시계방향으로
				mode_flag = 0;
				Passive_ing = 1;	//수동조작으로 이동중
			}
			angle_adjustment_PI(target_angle_3, _USER_RUN_SETTING.speed, CONSTANT_MODE);
			break;
		case 0x08:	//수동모드 키다운 DOWN 계속 길게 누름
			if(mode_flag == 8)
			{
				switch(_USER_RUN_SETTING.motion)
				{
					case Elbow:	//팔꿈치
						target_angle_4 = 0 + 0.5;	//0.5는 각도보정
						DISTANCE = Current_angle - 0;		//기준 각도차 계산
						break;
					case Shoulder:	//어깨
						target_angle_4 = 20 + 0.5;	//0.5는 각도보정
						DISTANCE = Current_angle - 20;		//기준 각도차 계산
						break;
					case Knee:	//무릎
						target_angle_4 = 140 - 0.5;	//0.5는 각도보정
						DISTANCE = 140 - Current_angle;		//기준 각도차 계산
						break;
					case wrist:	//손목
						target_angle_4 = (-70)+ 0.5;	//0.5는 각도보정
						DISTANCE = Current_angle - 20;		//기준 각도차 계산
						break;
					case ankle:	//발목
						target_angle_4 = (-50)+ 0.5;	//0.5는 각도보정
						DISTANCE = Current_angle - 20;		//기준 각도차 계산
						break;								
					default:
						break;
				}			
				Dir_of_rotation = CCW;			//반시계방향으로
				mode_flag = 0;
				Passive_ing = 1;	//수동조작으로 이동중
			}
			angle_adjustment_PI(target_angle_4, _USER_RUN_SETTING.speed, CONSTANT_MODE);
			break;
		case 0x09:	//측정모드 테스트(23.06.22), 어깨 기준으로 작성중(20~180도사이에서 측정)
			if(mode_flag == 9)	//측정시작할때 한번만 수행
			{
				DISTANCE = 180 - Current_angle;		//기준 각도차 계산
				
				if(_USER_RUN_SETTING.motion == Knee)
					Dir_of_rotation = CCW;
				else //어깨, 팔꿈치, 발목, 손목
					Dir_of_rotation = CW;
				mode_flag = 0;
				_LoRa_Data.use_state = 0x02;
				Measure_up = 1;			//상한각 먼저 측정
				Change_dir = 1;//3초 뒤에 Cal_rate_of_change_f = 1수행
				Measurement_CNT = 0;
				Bf_value = 0;
				Bf_avr = 0;
				Detect_load = 0;
				Cal_rate_of_change_f = 0;
			}
			//Exercise_ctrl()은 10ms 에 존재
			static word measure_10mcnt = 0;
			if(measure_10mcnt++ >= 10)//히스토리 배열에 전류 저장 간격은 10으로 설정할 경우 100ms마다 저장됨
			{
				measure_10mcnt = 0;
				current_history[current_index] = Current_actual;
			}
			current_index = (current_index + 1) % NUM_SAMPLES; // 순환 저장
			test_rate = calculate_slope(current_history, NUM_SAMPLES);
			baseline_current = calculate_average(current_history, NUM_SAMPLES);// 기준값 업데이트 (평균 전류 계산)
			relative_change = (Current_actual - baseline_current) / baseline_current;	//기준값 대비 상대적인 변화율로 부하 감지		
			if(Cal_rate_of_change_f)
			{
				if(relative_change > Load_sensitivity) //40%, 0.4
				{
					Cal_rate_of_change_f = 0;
					Detect_load = 1; // 부하 감지
				} 				
			}
			Rate = test_rate;		//변화율 모니터링용(테스트가 끝나면 삭제해도 됨)
			test_rate = 0;
			if(motor_delay_ms(1000))	//1초 뒤에 하한각 측정시작
			{
				if((Measurement_CNT % 2) == 1)
					Measure_down = 1;		//하한각 측정시작 flag ON
				else if((Measurement_CNT % 2) == 0)
					Measure_up = 1;	//상한각 측정시작 flag ON
			}			
			if(_USER_RUN_SETTING.exerc_start == 0x09)	//측정모드 일시정지시 or 비상정지시
			{//여기서 측정모드 관련 변수 모두 초기화 시켜야함.
				total_high_angle= 0, total_low_angle= 0;				
				Measurement_CNT = 0;		//측정 횟수 초기화
				exercise_mode = 0;			//운동 종료
				_VOICE_DATA.x06_1B = 0x14;//"측정을 정지합니다"음성 출력
				//_LoRa_Data.use_state = 0x08;//운동종료
				_LoRa_Data.state = 1;		//운동 종료(LCD보드에 보낼 데이터)
				_SENSOR_DATA.state = 1;		//운동 종료(PC에 보낼 데이터)
				timer2_ms_cnt_start = 0;	//정지 카운터 flag 종료
				PWM_SLEEP;				
				if(measurement_stop == 0)
				{
					measurement_stop = 1; 
					if(EMS_STATE != 1)
						_VOICE_DATA.x06_1B = 0x14;//"측정을 정지합니다"음성 출력
				}				
				pwm_step = 0;
			}
			else
			{//측정모드 정상작동시 진입
				PWM_NOT_SLEEP;
				if(Measure_up)			//상한각 측정
				{
					switch(_USER_RUN_SETTING.motion)
					{
						case Elbow:	//팔꿈치
							target_angle_5 = 135;
							DISTANCE = 135 - 0;		//기준 각도차 계산
							break;
						case Shoulder:	//어깨
							target_angle_5 = 180;
							DISTANCE = 180 - 20;		//기준 각도차 계산
							break;
						case Knee:	//무릎
							target_angle_5 = 140;
							DISTANCE = 140 - (-10);		//기준 각도차 계산
							break;
						case wrist:	//손목
							target_angle_5 = 70;
							DISTANCE = 70 - (-70);		//기준 각도차 계산
							break;
						case ankle:	//발목
							target_angle_5 = 50;
							DISTANCE = 50 - (-50);		//기준 각도차 계산
							break;							
						default:
							break;
					}
					angle_adjustment_PI(target_angle_5, 1, CONSTANT_MODE);	//상한각으로 이동
					
				}
				else if(Measure_down)	//하한각 측정
				{
					switch(_USER_RUN_SETTING.motion)
					{
						case Elbow:	//팔꿈치
							target_angle_5 = 0;
							DISTANCE = 135 - 0;		//기준 각도차 계산
							break;
						case Shoulder:	//어깨
							target_angle_5 = 20;
							DISTANCE = 180 - 20;		//기준 각도차 계산
							break;
						case Knee:	//무릎
							target_angle_5 = -10;
							DISTANCE = 140 - (-10);		//기준 각도차 계산
							break;
						case wrist:	//손목
							target_angle_5 = -70;
							DISTANCE = 70 - (-70);		//기준 각도차 계산
							break;
						case ankle:	//발목
							target_angle_5 = -50;
							DISTANCE = 50 - (-50);		//기준 각도차 계산
							break;							
						default:
							break;
					}
					angle_adjustment_PI(target_angle_5, 1, CONSTANT_MODE);		//하한각으로 이동
				}
			}
			break;
		default:
			break;
	}
	/****************************비상정지 버튼 눌렀을 때******************************/

	switch(_USER_RUN_SETTING.exerc_start)//동작중에 lcd 또는 비상스위치로 제어하는 부분이며 처음 시작으로는 진입못함.
	{
		case 0x01:	//시작버튼 눌렀을 시
			if(!Motor_BUSY)
				pause_state = 1;
			break;
		case 0x02:	//일시정지 버튼 눌렀을 시
			pause_state = 2;
			break;
		case 0x03:	//종료버튼 눌렀을 시
			if(!Motor_BUSY)
				pause_state = 3;
			break;			
		default:
			break;
	}
	static byte direction_1, direction_2;
	//무릎기구 방향 및 각도 조정 테스트 시작(23.02.02)
	if(_USER_RUN_SETTING.motion == Knee)	//무릎
	{
		direction_1 = CCW;
		direction_2 = CW;
	}
	else	//어깨, 무릎, 손목, 발목 24.03.07
	{
		direction_1 = CW;
		direction_2 = CCW;
	}
	if(measure_restart)
	{
		measure_restart = 0;
		//restart_f = 1;//비상정지 스위치 해제시 Soft Start로 다시 운동을 시작함 2024/09/13 창명측의 요구로 비상정지 해제 시 이전 운동기능 유지하는 기능 없앰.
	}
	/*
	if(Measurement_mode && EMS_STATE == 0)
		_SENSOR_DATA.state = 0x01;//대기중 or 운동종료
	
	else if(restart_f)
	{	
	    //도우가 운동을 시작합니다 음성 출력 제거함
		restart_f = 0;
		if(_SENSOR_DATA.state == 0 || exerc_end || _USER_RUN_SETTING.exerc_start == 0x09)//운동중일 때만 and 하한각으로 이동중일 경우만
		{
			PWM_NOT_SLEEP;
			init_skip_flag = 1;	//일시정지 후, 다시 시작버튼을 눌렀을 때 운동진행상태를 초기화하지 않기 위한 flag
		}
		if(_USER_RUN_SETTING.exerc_start == 0x09)//일시정지상태이면
		{
			_USER_RUN_SETTING.exerc_start = 0x08;//측정시작	
			//Measurement_mode = 1;
		}
		_LoRa_Data.state = 0;	//운동중
		//_SENSOR_DATA.state = 0;	//운동중		
		pause_state = 0;
		deaccel_flag = 0;
		PAUSE_F = 0;	//타이머 일시정지 중지
		exercise_mode = exercise_mode>>4;		
		if(exercise_mode == 0x01)//일반운동 모드일 때, 다시 시작하면 반대방향으로 운동진행
		{				
			if(Dir_of_rotation == direction_1)
			{
				toggle_cnt++;
				Dir_of_rotation = direction_2;
				DISTANCE = Current_angle - (set_min_angle+1);	//기준 각도차 계산	
			}
			else if(Dir_of_rotation == direction_2)
			{
				toggle_cnt--;
				Dir_of_rotation = direction_1;
				DISTANCE = (set_max_angle-1) - Current_angle;	//기준 각도차 계산
			}	
		}
		else	//적응, 집중운동 모드는 다시 시작하면 진행 방향 그대로 유지
		{		
			if(Dir_of_rotation == direction_1)
				DISTANCE = (set_max_angle-1) - Current_angle;	//기준 각도차 계산
			else if(Dir_of_rotation == direction_2)
				DISTANCE = Current_angle - (set_min_angle+1);	//기준 각도차 계산
		}
	}
	*/
	/***************************************************************************/
	static byte Pause_back = 0;
	static byte	stop_puase = 0;
	/****************************일시정지 버튼 눌렀을 때******************************/
	if(pause_state == 2)	//일시정지 버튼 눌렸을 때
	{
		stop_puase = 1;
		_SENSOR_DATA.state = 3;	//일시정지 상태
		Motor_BUSY = 1;		//Soft stop 중
		deaccel_flag = 1;
		PAUSE_F = 1;	//타이머 일시정지
		if(Pause_back == 0)
		{	
			toggle_cnt = toggle_cnt - 2;			
			Pause_back = 1;
			_VOICE_DATA.x06_1B = 0x19;//모터가 일시정지 되었습니다. 음성 출력
			voice_en = 1;
		}
		if(Dir_of_rotation == direction_1)
		{
			if((set_max_angle <= Current_angle) || (pwm_step <= 0))	//감속시작 후, 현재각도가 상한각 이상이 되거나 감속이 끝나면 일반운동모드 종료
			{
				pwm_step = 0;
				Motor_PWM = 0;
				pause_state = 0;	//_USER_RUN_SETTING.exerc_start = 0x00;
				if(exercise_mode <= 0x03)	//일반,적응,집중 운동중에서만
				{
					exercise_mode = exercise_mode<<4; //일시정지시
				}
				
				Motor_BUSY = 0;
				_SENSOR_DATA.state = 0x04;	//일시정지 완료
			}
		}
		else if(Dir_of_rotation == direction_2)
		{
			if((set_min_angle >= Current_angle) || (pwm_step <= 0)) //감속시작 후, 현재각도가 하한각 이하가 되거나 감속이 끝나면 일반운동모드 종료
			{
				pwm_step = 0;
				Motor_PWM = 0;
				pause_state = 0;	//_USER_RUN_SETTING.exerc_start = 0x00;
				if(exercise_mode <= 0x03)	//일반,적응,집중 운동중에서만
				{
					exercise_mode = exercise_mode<<4; //일시정지시
				}
				Motor_BUSY = 0;
				_SENSOR_DATA.state = 0x04;	//일시정지 완료
			}
		}
		
	}
	else if(((pause_state == 1) && (Motor_BUSY == 0)))	//다시 시작버튼 눌렀을 시
	{
		gen_ada_con_back = 0; //운동을 시작합니다 음성출력(일반,적응,집중)
		measurement_stop = 0;  //측정모드 관련 변수
		//voice_en = 1;
		Pause_back = 0;
		init_skip_flag = 1;	//일시정지 후, 다시 시작버튼을 눌렀을 때 운동진행상태를 초기화하지 않기 위한 flag
		_USER_RUN_SETTING.exerc_start = 0x00;
		pause_state = 0;
		deaccel_flag = 0;
		_LoRa_Data.state = 0;	//운동중
		_SENSOR_DATA.state = 0;	//운동중		
		PAUSE_F = 0;	//타이머 일시정지 중지
		exercise_mode = exercise_mode>>4;		//일단 일반운동 일시정지만 테스트중.23.01.12
		if(exercise_mode == 0x01)	//일반운동 모드일 때, 다시 시작하면 반대방향으로 운동진행
		{
			if(Dir_of_rotation == direction_1)
			{
				toggle_cnt++;
				Dir_of_rotation = direction_2;
				DISTANCE = Current_angle - (set_min_angle+1);	//기준 각도차 계산	
			}
			else if(Dir_of_rotation == direction_2)
			{
				toggle_cnt--;
				Dir_of_rotation = direction_1;
				DISTANCE = (set_max_angle-1) - Current_angle;	//기준 각도차 계산
			}	
		}
		else	//적응, 집중운동 모드는 다시 시작하면 진행 방향 그대로 유지
		{
			if(Dir_of_rotation == direction_1)
				DISTANCE = (set_max_angle-1) - Current_angle;	//기준 각도차 계산
			else if(Dir_of_rotation == direction_2)
				DISTANCE = Current_angle - (set_min_angle+1);	//기준 각도차 계산
		}
	}
	/***************************************************************************/
	
	
	/****************************종료 버튼 눌렀을 때********************************/
	if(pause_state == 3 && (Motor_BUSY == 0))	//종료버튼 눌렀을 때
	{
		exerc_time_start = 0;	//운동 시간모드일 경우 시간 카운팅 종료
		deaccel_flag = 0;
		pwm_step = 0;
		exerc_cnt = 0;
		if (exercise_mode != 5 && exercise_mode != 6 && exercise_mode != 7 && exercise_mode != 8 && exercise_mode != 9) //수동모드, 측정모드가 아니면 진입
			exerc_end = 1;
		exercise_mode = 0;
		Pause_back = 0;
		Move_to_lower = 1;
		mode_flag = 10;
		
		Time_mode_end = 0;
		_USER_RUN_SETTING.exerc_start = 0x00;
		pause_state = 0;
		PAUSE_F = 0;	//타이머 일시정지 중지
		
		//집중운동에서만 사용
		repeat_ok = 0;	
		fake_cnt = 0;
		
		Not_Send_Ble = 1; // 종료 버튼을 눌렀을때는 블루투스로 운동기록 데이터를 보내지 않도록 함.
	}
	/**************************************************************************/
	/*****************************운동 끝나고 정지시작******************************/
	if(exerc_end)	//운동이 끝난 후 하한각 이동, 하한각+10에서 정지
	{	
		PWM_NOT_SLEEP;
		_SENSOR_DATA.state = 2;	//정지중(하한각이동중) 
		if(stop_puase)
		{
			stop_puase = 0;
			//restart_f = 1;	//비상정지 스위치 해제시 Soft Start로 다시 운동을 시작함
		}
		_LoRa_Data.state = 0;		//하한각으로 이동중 운동중상태
		_SENSOR_DATA.velocity_mode = 0x00;	//등속
		if(Move_to_lower)//하한 각도로 이동(반시계방향)
		{
			if(mode_flag == 10)
			{		//22.12.19. 1도 각도보정
				if(set_speed_step >= 6)	//6단계 이상일 때만 보정
					DISTANCE = Current_angle - (set_min_angle);
					//DISTANCE = Current_angle - (set_min_angle+0.5);
				else if(set_speed_step < 6)
					DISTANCE = Current_angle - set_min_angle;
				mode_flag = 0;
			}
			Dir_of_rotation = direction_2;	//하한각으로 이동하기 위해 반시계 방향으로 전환
			if(set_speed_step < 8)//1 ~ 7 단계
				exerc_stop_mode(set_min_angle, set_speed_step, direction_2);
			else if(set_speed_step >= 8)
				exerc_stop_mode(set_min_angle, 6, direction_2);				

		}
		else//하한 각도+10으로 이동(시계방향)
		{
			if(mode_flag == 10)
			{			//22.12.19. 1도 각도보정
				DISTANCE = (set_min_angle+10) - Current_angle;//민수
				mode_flag = 0;
			}
			Dir_of_rotation = direction_1;
			exerc_stop_mode(set_min_angle + 10, 3, direction_1);	//하한각+10도로 이동(속도 3단계로 임의로 고정)
		}
	}
	/**************************************************************************/
	
	
	
	/*********************************속도 측정***********************************/
	static byte tcnt_10ms = 0;
	if(++tcnt_10ms >= 10)
	{//100ms task
		tcnt_10ms = 0;
		cnt2 = TIM1->CNT;//엔코더입력
		//TIM1의 CNT값은 시계방향으로 회전할 때 감소, 반시계방향으로 회전할 때 증가함
		/* 회전 방향 확인 */
		if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim1))
		{
			dir = 1;	//반시계 방향
			
			if((TIM1->CNT) > bf_CNT)	//CNT가 under되어 현재 CNT값이 이전 CNT값보다 클 때
				Under_cnt++;
			
			bf_CNT = TIM1->CNT;

			
			/* __HAL_TIM_IS_TIM_COUNTING_DOWN 매크로가 방향성을 잘못 알려주는 경우 예외처리 */
			if((cnt2 > cnt1) && (cnt2 - cnt1 < 100)) 
				dir = 0;
		}
		else
		{
			dir = 0;	//시계 방향
			
			if((TIM1->CNT) < bf_CNT)	//CNT가 over되어 현재 CNT값이 이전 CNT값보다 낮을 때
				Over_cnt++;
			
			bf_CNT = TIM1->CNT;
			
			/* __HAL_TIM_IS_TIM_COUNTING_DOWN 매크로가 방향성을 잘못 알려주는 경우 예외처리 */
			if((cnt1 > cnt2) && (cnt1 - cnt2 < 100)) 
				dir = 1;
		}
		if(dir)    //시계 방향(모터)
		{
			/* Down Counting 방향으로 회전할 때 */
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
			Encoder_dir = 'U';		//엔코더 에러체크용(방향성 체크)	//UP
		}
		else       //반시계 방향(모터)
		{
			/* Up Counting 방향으로 회전할 때 */
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
			Encoder_dir = 'D';		//엔코더 에러체크용(방향성 체크)	//DOWN
		}
		//SMCR : Slave Mode Control Registe
		if((TIM1->SMCR & 0x03) == 0x03)	//Encoder mode 3 : X4 mode 일때 (SMCR레지스터의 SMS가 011일때 엔코더 4체배모드)
		{
			/* X4 Mode 일때는 카운터가 4 증가할 때, 1개의 Pulse */
			speed_RPM = diff * 600 / 4 / 500;  //ONE_ROTATION_PULSES/15; //RPM 계산을 위해서는 x 60
									//1사이클당 500펄스 : 500cpr
		}	
		cnt1 = TIM1->CNT;// 현재 값을 이전 값으로 업데이트 아래랑 같은 의미
		//cnt1 = cnt2;// 현재 값을 이전 값으로 업데이트
	}
	/******************************속도 측정 END******************************** **/

	
	/**************************LCD 컨트롤러 제어(읽기)******************************/
	_SENSOR_DATA.current = Current_actual;					//전류(A)
	_SENSOR_DATA.current_speed = speed_RPM;					//현재 속도(RPM)
	_SENSOR_DATA.dir = Dir_of_rotation;						//현재 운동 방향
	if(_USER_RUN_SETTING.cnt_mode == 0)// || (_LoRa_Data.remain_time_and_cnt >> 7)) == 0)	//운동시간 모드일때
	{
		_SENSOR_DATA.remain_time = set_exerc_time - min_cnt;	//남은 운동시간(분)
		_LoRa_Data.remain_time_and_cnt = set_exerc_time - min_cnt;	//남은 운동시간(분) MSB : 0
		_Ble_Data.remain_time_and_cnt = set_exerc_time - min_cnt;	//남은 운동시간(분) MSB : 0
		_SENSOR_DATA.cnt = 0;//운동 진행횟수 0으로 처리, Lora는통합 되어서 안해줘도 됨 
	}
	else	//운동횟수 모드일때
	{
		_SENSOR_DATA.cnt = num_of_workouts;//운동 진행횟수
		_LoRa_Data.remain_time_and_cnt = 0x80 | num_of_workouts;	//운동 진행 횟수 MSB : 1
		_Ble_Data.remain_time_and_cnt =  0x80 | num_of_workouts;	//운동 진행 횟수 MSB : 1
		_SENSOR_DATA.remain_time = 0;//남은 운동시간 0으로 처리, Lora는통합 되어서 안해줘도 됨 
	}
	/**************************************************************************/
	
	/*****************************LCD 표시각도 제한********************************/
	static float limit_angle;
	if((_USER_RUN_SETTING.exerc_start == 0x06) || (_USER_RUN_SETTING.exerc_start == 0x07) || (Passive_ing == 1))	//수동모드 
	{
		switch(_USER_RUN_SETTING.motion)
		{
			case Elbow:	//팔꿈치
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
					if (GPIOD->ODR & GPIO_PIN_10) // PD10의 상태가 1일 때 실행할 코드
					{//모터가 돌아갈 중에만 각도값을 보냄
						_SENSOR_DATA.current_angle = Current_angle;
						_Ble_Data.current_angle = Current_angle;	//현재각도 표시
					}							
				}
				break;
			case Shoulder:	//어깨
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
			case Knee:	//무릎
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
					if (GPIOD->ODR & GPIO_PIN_10) // PD10의 상태가 1일 때 실행할 코드
					{//모터가 돌아갈 중에만 각도값을 보냄
						_SENSOR_DATA.current_angle = Current_angle;
						_Ble_Data.current_angle = Current_angle;	//현재각도 표시
					}							
				}				
				break;
			case wrist:	//손목
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
					if (GPIOD->ODR & GPIO_PIN_10) // PD10의 상태가 1일 때 실행할 코드
					{//모터가 돌아갈 중에만 각도값을 보냄
						_SENSOR_DATA.current_angle = Current_angle;
						_Ble_Data.current_angle = Current_angle;	//현재각도 표시
					}							
				}				
				break;	
			case ankle:	//발목
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
					if (GPIOD->ODR & GPIO_PIN_10) // PD10의 상태가 1일 때 실행할 코드
					{//모터가 돌아갈 중에만 각도값을 보냄
						_SENSOR_DATA.current_angle = Current_angle;
						_Ble_Data.current_angle = Current_angle;	//현재각도 표시
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
			case 0:	//일반운동 설정되어 있을 때
				if((exercise_mode == NORMAL) || (exerc_end))	//일반운동 진행중 또는 종료중
				{
					if(Current_angle >= set_max_angle)	//상한각 넘어갈때 상한각으로 표시
					{
						_SENSOR_DATA.current_angle = set_max_angle;
						_Ble_Data.current_angle = set_max_angle;
					}
					else if(Current_angle <= set_min_angle)	//하한각 넘어갈때 하한각으로 표시
					{
						if(move_to_lower)
						{
							_SENSOR_DATA.current_angle = Current_angle;	//처음에 하한각 이동중에는 현재각 표시
							_Ble_Data.current_angle = Current_angle;	//처음에 하한각 이동중에는 현재각 표시
						}
						else
						{
							_SENSOR_DATA.current_angle = set_min_angle;
							_Ble_Data.current_angle = set_min_angle;	
						}
					}
					else//상한각, 하한각 범위내 일때
					{
						_SENSOR_DATA.current_angle = Current_angle;	//현재각도 표시
						_Ble_Data.current_angle = Current_angle;	//현재각도 표시
					}
					if(!Move_to_lower)	//하한각+10도로 이동중일 때
					{
						_SENSOR_DATA.current_angle = set_min_angle + 10;
						_Ble_Data.current_angle = set_min_angle + 10;
					}
				}
				else if(!exercise_mode)		//평상시
				{
					if(GPIOD->ODR & GPIO_PIN_10) // PD10의 상태가 1일 때 실행할 코드
					{
						_SENSOR_DATA.current_angle = Current_angle;	//현재각도 표시
						_Ble_Data.current_angle = Current_angle;	//현재각도 표시
					}
				}
				break;
			case 1:	//적응운동 설정되어 있을 때
				if(exercise_mode == ADAPTIVE)	//적응운동 진행중
				{
					switch(set_location)	//적용위치
					{
						case 0:	//LOWER
							if(Current_angle >= set_max_angle)	//상한각 넘어갈때 상한각으로 표시
							{
								_SENSOR_DATA.current_angle = set_max_angle;
								_Ble_Data.current_angle = set_max_angle;	
							}
							else if(Current_angle <= (set_min_angle+set_PA-angle_step))
							{
								if(move_to_lower)
								{
									_SENSOR_DATA.current_angle = Current_angle; //현재각도 표시
									_Ble_Data.current_angle = Current_angle;	//현재각도 표시
								}
								else
								{
									_SENSOR_DATA.current_angle = (set_min_angle+set_PA-angle_step);
									_Ble_Data.current_angle =(set_min_angle+set_PA-angle_step);
								}
							}
							else	//상한각, 하한각 범위내 일때
							{
								_SENSOR_DATA.current_angle = Current_angle;	//현재각도 표시
								_Ble_Data.current_angle = Current_angle;	//현재각도 표시
							}
							break;
						case 1:	//UP_LOW
							if(Current_angle >= (set_max_angle-set_PA+angle_step))	//상한각 넘어갈때 상한각으로 표시
								_SENSOR_DATA.current_angle = (set_max_angle-set_PA+angle_step);
							else if(Current_angle <= (set_min_angle+set_PA-angle_step))	//하한각 넘어갈때 하한각으로 표시
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
							else	//상한각, 하한각 범위내 일때
							{
								_SENSOR_DATA.current_angle = Current_angle;	//현재각도 표시
								_Ble_Data.current_angle = Current_angle;	//현재각도 표시
							}
							break;
						case 2:	//UPPER
							if(Current_angle >= (set_max_angle-set_PA+angle_step))	
							{
								_SENSOR_DATA.current_angle = (set_max_angle-set_PA+angle_step);
								_Ble_Data.current_angle = (set_max_angle-set_PA+angle_step);
							}
							else if(Current_angle <= set_min_angle)	//하한각 넘어갈때 하한각으로 표시
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
							else	//상한각, 하한각 범위내 일때
							{
								_SENSOR_DATA.current_angle = Current_angle;	//현재각도 표시
								_Ble_Data.current_angle = Current_angle; //현재각도 표시
							}
								
							break;
						default:
							break;
					}	
				}
				else if(!exercise_mode)		//평상시 또는 종료중일 때
				{
					if(exerc_end)	//적응운동 종료 중일때
					{
						if(Move_to_lower)	//하한각으로 이동중일 때
						{
							if(Current_angle <= set_min_angle)	//하한각 넘어갈 때 하한각 표시
							{
								_SENSOR_DATA.current_angle = set_min_angle;
								_Ble_Data.current_angle = set_min_angle;	
							}
							else
							{
								_SENSOR_DATA.current_angle = Current_angle;	//현재각도 표시
								_Ble_Data.current_angle = Current_angle;	//현재각도 표시
							}
						}
						else	//하한각+10도로 이동중일 때
						{
							if(Current_angle >= (set_min_angle+10))	//하한각+10 넘어갈때 하한각+10도로 표시
							{
								_SENSOR_DATA.current_angle = set_min_angle + 10;
								_Ble_Data.current_angle = set_min_angle + 10;
							}
							else
							{
								_SENSOR_DATA.current_angle = Current_angle;	//현재각도 표시
								_Ble_Data.current_angle = Current_angle;	//현재각도 표시
							}
						}
					}
					else	//평상시 또는 운동종료가 완전히 끝났을 때
					{
						if(GPIOD->ODR & GPIO_PIN_10) // PD10의 상태가 1일 때 실행할 코드
							_SENSOR_DATA.current_angle = Current_angle;	//현재각도 표시
					}
				}
			
				break;
			case 2:	//집중운동 설정되어 있을 때
				if(exercise_mode == INTENSIVE)	//집중운동 진행중
				{
					switch(set_location)	//적용위치
					{
						case 0:	//LOWER
							switch(Focus_f)
							{
								case 0:	//하한각이동
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
								case 1:	//상한각이동
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
								case 2:	//하한 집중각 이동
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
								case 0:	//하한각이동
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
								case 1:	//상한각이동
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
								case 2:	//하한 집중각 이동
									except_f = 0;	//각도제한 예외(적용위치 상한각)
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
								case 3:	//상한 집중각 이동
									except_f = 0;	//각도제한 예외(적용위치 상한각)
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
								case 0:	//하한각이동
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
								case 1:	//상한각이동
									limit_angle = (repeat_ok == 1) ? set_min_angle : (set_max_angle-set_conc_angle);
									if(except_f)
										limit_angle = set_min_angle;	//처음 상한각이동시 예외처리
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
								case 3:	//상한 집중각 이동
									except_f = 0;	//각도제한 예외(적용위치 상한각)
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
										_SENSOR_DATA.current_angle = Current_angle;//현재각도 표시
										_Ble_Data.current_angle = Current_angle;//현재각도 표시
									}
									break;
								default:
									break;
							}
							break;
						default:
							break;
					}	
					if(Focus_f == 'U')		//운동시작전 하한각 이동중(현재각이 하한각보다 작을때)
					{
						_SENSOR_DATA.current_angle = Current_angle;
						_Ble_Data.current_angle = Current_angle;//현재각도 표시
					}
					else if(Focus_f == 'D')	//운동시작전 하한각 이동중(현재각이 하한각보다 클때)
					{
						if(Current_angle <= set_min_angle)
						{
							_SENSOR_DATA.current_angle = set_min_angle;
							_Ble_Data.current_angle = Current_angle;//현재각도 표시
						}
						else
						{
							_SENSOR_DATA.current_angle = Current_angle;
							_Ble_Data.current_angle = Current_angle;//현재각도 표시
						}
					}	
				}
				else if(!exercise_mode)//평상시 또는 종료중일 때 민수
				{
					if(exerc_end)//적응운동 종료 중일때
					{
						if(Move_to_lower)	//하한각으로 이동중일 때
						{
							if(Current_angle >= set_max_angle)
								_SENSOR_DATA.current_angle = set_max_angle;
							else if(Current_angle <= set_min_angle)	//하한각 넘어갈 때 하한각 표시
								_SENSOR_DATA.current_angle = set_min_angle;
							else
							{
								_SENSOR_DATA.current_angle = Current_angle;	//현재각도 표시
								_Ble_Data.current_angle = Current_angle;//현재각도 표시
							}
						}
						else//하한각+10도로 이동중일 때
						{
							if(Current_angle <= set_min_angle)
								_SENSOR_DATA.current_angle = set_min_angle;
							else if(Current_angle >= (set_min_angle + 10))	//하한각+10 넘어갈때 하한각+10도로 표시
								_SENSOR_DATA.current_angle = set_min_angle + 10;
							else
							{
								_SENSOR_DATA.current_angle = Current_angle;	//현재각도 표시
								_Ble_Data.current_angle = Current_angle;//현재각도 표시
							}
						}
					}
					else	//평상시 또는 운동종료가 완전히 끝났을 때
					{
						if(GPIOD->ODR & GPIO_PIN_10) // PD10의 상태가 1일 때 실행할 코드
							_SENSOR_DATA.current_angle = Current_angle;	//현재각도 표시
					}
				}
				break;
			default:
				break;
		}
	}
	/***************************LCD 표시각도 제한 END*****************************/
	Motor_ctrl();	//모터 PWM 시작  
}//여개가 Exercise_ctrl 함수의 끝!
float Chang_angle = 0;
unsigned int Motor_PWM_CCW = 1800, Motor_PWM_CW = 1800; 
word Log_buf[1024], Log_index=0, Log_enable=0;
void Motor_ctrl()	//Call by 10msec
{
	if(!Error_Check)
	{
		if(Dir_of_rotation == CCW)	//역방향(반시계 방향),1
		{
			Motor_PWM_CCW = (unsigned int)(-0.5*Motor_PWM + 1800);	//Motor_PWM:0~3600
			if(Motor_PWM_CCW >= 3600 * 0.9)			//PWM's Duty : 90%(3240)	
				Motor_PWM_CCW = 3600 * 0.9;			//3240
			else if(Motor_PWM_CCW <= 3600 * 0.1)	//PWM's Duty : 10%(360)	
				Motor_PWM_CCW = 3600 * 0.1;			//360
			htim8.Instance->CCR1 = Motor_PWM_CCW; 
		}
		else if(Dir_of_rotation == CW)	//정방향(시계	방향),0
		{	
			Motor_PWM_CW = (unsigned int)(0.5*Motor_PWM + 1800);	//Motor_PWM:0~3600
			if(Motor_PWM_CW >= 3600 * 0.9)			//PWM's Duty : 90%(3240)	
				Motor_PWM_CW = 3600 * 0.9;			//3240
			else if(Motor_PWM_CW <= 3600 * 0.1)	//PWM's Duty : 10%(360)	
				Motor_PWM_CW = 3600 * 0.1;			//360
			htim8.Instance->CCR1 = Motor_PWM_CW; 
		}
		

		if(Log_enable)//Motor_PWM 변수 값이 실시간으로 변하는 것을 확인하는 테스트용 변수이며 프로그램 동작에 관여하지 않아 삭제하여도 동작에는 관여 x
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
		if(Dir_of_rotation == CCW)	//역방향(반시계 방향)
		{
		//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);    //PWM2_H   (OFF) 
		//	htim4.Instance->CCR2 = 0;  								 //PWM1_L	(OFF)
			TIM4_CCR2_buffer = 0;  								 //PWM1_L	(OFF)
		//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);      //PWM1_H	(ON)
		//	htim4.Instance->CCR1 = 	Motor_PWM; 						 //PWM2_L	(ON) : 기존의 값을 저장하기 위해서 사용함.
			TIM4_CCR1_buffer = Motor_PWM; 						 //PWM2_L	(ON) : 기존의 값을 저장하기 위해서 사용함.
		}
		else if(Dir_of_rotation == CW)	//정방향(시계	방향)
		{
			//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);     //PWM1_H	(OFF)
		//	htim4.Instance->CCR1 = 	0; 						 		  //PWM2_L	(OFF)
			TIM4_CCR2_buffer = Motor_PWM;
		//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);       //PWM2_H	(ON) 
		//	htim4.Instance->CCR2 = Motor_PWM;  						  //PWM1_L	(ON)
			TIM4_CCR1_buffer = 0;
		}
		//엔코더 각도측정 테스트(오차가 심해서 사용하지 않음)
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
			if(motor_delay_ms(2000))	//2초후 다음 실험진행
			{
				if(Record_data_f == 1)
				{
					set_angle_value = Current_angle - 2;
					Record_data_f = 0;
					if(Record_Index < 100)
					{
						End_angle_1[Record_Index] = Current_angle;	//끝 각도 저장(엔코더 각도측정)
						Under_cnt_arr[Record_Index] = Under_cnt;	//오버 카운트 저장(엔코더 각도측정)
						Current_CNT_1[Record_Index++] = TIM1->CNT;	//현재 CNT값 저장
					}
					
				}
				else if(Record_data_f == 2)
				{
					set_angle_value = Current_angle + 2;
					Record_data_f = 0;
					if(Record_Index < 100)
					{
						End_angle_2[Record_Index] = Current_angle;	//끝 각도 저장(엔코더 각도측정)
						Over_cnt_arr[Record_Index] = Over_cnt;	//오버 카운트 저장(엔코더 각도측정)
						Current_CNT_2[Record_Index++] = TIM1->CNT;	//현재 CNT값 저장
					}
				}
				exercise_mode = 4;	//실험 시작
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
  hi2c1.Init.ClockSpeed = 400000;//eeprom은 400khz까지 허용 , i2c는 100khz ~ 400khz 설정 가능
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;//APB2 72MHz를 8분주 하여 음성모듈 동작 주파수를 9MHz로 해주었다. 음성모듈의 최대 동작 주파수는 16MHz로 계산하였다.
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
{//엔코더 전용 타이머
  /* USER CODE BEGIN TIM1_Init 0 */
  /* USER CODE END TIM1_Init 0 */
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  /* USER CODE BEGIN TIM1_Init 1 */
  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;//72MHz APB2 
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;   //엔코더 전용 타이머
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
{//0.1msec타이머
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
  htim2.Init.Period = 72e6/10e3-1;// = 7200 -> 10000  0.1msec 카운터      
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
	htim3.Init.Prescaler = 2 - 1;	//2분주(72MHz/2=32MHz : TIM3's Clock)
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = (36e6 / 1e3 - 1);// = 36000 -> 1msec 카운터     
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
{//모터드라이버 개조 전에 사용된 타이머이며 현재는 TIM8을 사용함
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
{//0.01ms 타이머
  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;//0분주(72MHz/2=36MHz : TIM3's Clock)
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 65535;//   0.91ms 카운터
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
{//lcd and 모터드라이버 통신용
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
//블루투스 or 로라 통신
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
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET); //음성모듈 RDY 평상시 HIGH
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
  GPIO_InitStruct.Pin = RLY_M_SEL_Pin|GPIO_PIN_2;//PE2 : 음성모듈 CS
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
{//pwm전용 타이머
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
