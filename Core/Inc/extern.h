//extern.h

typedef unsigned char	byte; //8bit
typedef unsigned short	word;//16bit
typedef unsigned long	dword;//32bit = uint


extern byte ADC1_index;
extern word ADC1_data[4];

extern double Current_mes;
extern double Volt_mes;

extern dword P_M_sum;
extern float P_M_avr;
extern float P_M_actual;
extern float Current_angle;

extern float CT_ad_avr;

extern dword timer2_ms_cnt;
extern byte timer2_ms_cnt_start;

extern word t2_min_cnt;
extern byte exerc_time_start;
extern word min_cnt; 

extern float Current_actual;

extern byte PAUSE_F;
extern byte Motor_Run_F;

extern byte num_of_workouts;
extern byte Motor_BUSY;

extern byte set_exerc_time;

extern byte Dir_of_rotation;

extern byte Passive_ing;

//셀프체크
extern byte Error_Check, Restart_self_check;
extern byte check_step;
extern byte shift_step;
extern byte Priority_dir_set;
extern signed int diff;

extern float Bf_pm_avr;
extern float Af_pm_avr; 
extern float Change_pm_avr;

extern float Bf_ct_avr;
extern float Af_ct_avr; 
extern float Change_ct_avr;

extern int Bf_encoder_avr;
extern int Af_encoder_avr;
extern int Change_encoder_avr;

extern byte CW_pm_check_ok;
extern byte CCW_pm_check_ok;

extern byte CW_ct_check_ok;
extern byte CCW_ct_check_ok;

extern byte CW_encoder_check_ok;
extern byte CCW_encoder_check_ok;



extern byte CCW_priority_mode;

extern byte Self_check_end;
extern byte Encoder_dir;

//음성
extern byte voice_out;
extern void voice_output(void);
extern byte voice_error_check;
extern void SPI_voice_set(void);

//TIM4 CCR변수 확인용
extern dword TIM4_CCR1_buffer, TIM4_CCR2_buffer; 

//디버깅
extern word adc_value;
extern byte exercise_mode;

//eeprom
extern byte wait_1400m_flag, wait_1400m_start;
extern byte save_angle_end, save_angle_flag, default_save_angle, default_save_ct, save_ct_end, page1_1400msec_wait, page2_1400msec_wait, page1_end_flag,page2_end_flag, save_angle_end_flag, real_angle_value_page2_end, Last_page_end, Last_page;
extern byte anlge_ad_err_flag, anlge_real_err_flag, ct_err_flag;
extern void EEPROM_Write(byte device_wr_address,byte start_page, byte *Wr_Data);
extern byte EEPOM_Read(byte device_rd_address, byte Size, byte flag_state);
extern void int_to_bytes(uint32_t value, byte *bytes_data);
extern word word_to_uint32(word value);
extern uint32_t float_to_uint32(float value);// 부동 소수점 값을 4바이트 정수로 변환하는 함수
extern byte save_ct_flag;
extern byte angle_cal_ad_1, angle_cal_ad_2, angle_cal_ad_3, angle_cal_ad_4, angle_cal_ad_5;
extern byte ct_cal_ad_1, ct_cal_ad_2;
extern float real_angle;
//user_define
#define LED1_OFF	GPIOB->ODR |= GPIO_PIN_12;	 //PB12---->1
#define LED1_ON		GPIOB->ODR &= ~GPIO_PIN_12;	 //PB12---->0
//각도 캘리브레이션
extern float previous_count, current_count, delta_count,final_count;
extern byte anlge_cal_flag, angle_measure_1start,angle_measure_2start, low_to_high,high_to_low, angle_recording_end, Not_save_arr,read_ad_angle_flag;
extern word angle_arr[33];
extern byte Read_ad_angle_state;

extern byte Move_to_lower;
extern byte cur_index;
extern float cur_measurement1[250];
extern float cur_measurement2[250];
extern byte SPI_tx_buf[10], SPI_rx_buf[10], Voice_error, voice_en;
extern void SPI_voice_set(void);
extern void delayMicroseconds(uint32_t us);
extern void voice_err_check(void);
extern SPI_HandleTypeDef hspi1;
extern void copy_Pc_to_Motordriver();
//extern void SPI_voice(void)
//RS232 통신(UART2)
#define UART_STX	0x01	
//핸드스위치
extern byte START_STATE, STOP_STATE;
extern void UART4_exe(void);

//타이머5 시간측정 함수
extern void start_timer();
extern float stop_timer();

extern void copyLoRaDataToSensorData();

//각도센서(Potentiometer)
extern dword P_M_sum;
extern float P_M_avr;
extern float P_M_slop;
extern float P_M_offset;
extern float P_M_actual;
extern float PM_avr_value;
extern float Current_angle;
extern float bf_pm_actual;
extern byte Opposition;

extern byte Start_self_check;
//각도 캘리브레이션
extern float lower_angle, upper_angle, lower_ad, upper_ad, purse1_down_1angle;
extern byte SYSTEM_INITIALIZATION_VALUE_ANGLE;
extern int find_lower_index(uint16_t* arr, int size, float value);
extern int find_upper_index(uint16_t* arr, int size, float value);
extern float interpolate(uint16_t* angle_arr, int size, float P_M_avr, float real_angle_point4, float real_angle_point2, uint16_t angle_ad_point4, uint16_t angle_ad_point2, float* bf_pm_actual);
extern byte angle_record_flag, calibration_state, eeprom_page_state;
extern word ad_count_down, remain_ad, low_to_high_pwm, high_to_low_pwm;
extern dword purse1_up_1angle;
//exercise_mode 1 : 일반, 2 : 집중, 3 : 적응, 4: 설정각이동, 5 : 수동모드 up 1도, 6 : 수동모드 down 1도, 7 : Up ing, 8 : Down ing, 9 : ROM(측정모드)
typedef struct USER_RUN_SETTING
{
	byte motion;				//운동 부위(0:팔꿈치, 1:어깨, 2:무릎, 3:손목, 4:발목)
	byte mode;					//운동 모드(0:일반, , 1:적응, 2:집중)
	byte exerc_time;			//운동 시간(분)									
	byte exerc_num;				//운동 횟수		
	
	byte adaptation_angle;		//적응 각도	
	byte repeat_num;			//반복 횟수	
	byte concentration_angle;	//집중 각도
	byte application_location;	//적용 위치(0:하한각, 1:상하한각, 2:상한각)
	
	byte speed_mode;			//속도 모드(0:가속, 1:등속)	
	byte speed;					//운동 속도 단계(1~7)
	byte upper_stop_time;		//상한 정지 시간(초)	
	byte lower_stop_time;		//하한 정지 시간(초)	
	
	float upper_angle;			//상한 각도
	float lower_angle;			//하한 각도	
	
	byte cnt_mode;				//운동 시간,횟수모드 설정(0:운동시간, 1:운동횟수)			
	byte exerc_start;			//(0:평상시, 1:운동시작, 2:일시정지, 3:종료, 4:수동모드 1도 up, 5:수동모드 1도 down, 6:수동모드 up ing, 7:수동모드 down ing, 8:측정모드, 9:측정모드 일시정지, 0A:측정 카운트 초기화)
	byte calibration;			//(0:평상시, 1:Calibration 진행중, 2 : 부하 민감도 설정)
	byte ad_start;				//(0:평상시, 1:각도 AD시작1, 2:각도 AD시작2 3:각도 AD시작3, 4:각도 AD시작4 5:전류 AD시작1, 6:전류 AD시작2)
	
	float real_value;			//(0:평상시, 캘리브레이션시 정확한 각도,전류값)
	
	byte equip_up_down;			//(0:평상시, 1: 장비 up, 2:장비 up ing 3:장비 down, 4:장비 down ing)
	byte x06_1B;				//
	byte equipment_num;			//장비번호
	byte re_self_check;	
}USER_RUN_SETTING;  //32Byte
extern USER_RUN_SETTING		_USER_RUN_SETTING;
#define USER_RUN_ADDRESS	&_USER_RUN_SETTING

typedef struct SENSOR_DATA
{
	word user_name_1;            //0xD64D (홍)   
	word user_name_2;            //0xAE38 (길) 
	
	word user_name_3;            //0xB3D9 (동) 
	word user_name_4;            //0x0000 ( ) 
	
	word user_name_5;            //0x0000 ( ) 
	word user_name_6;            //0x0000 ( ) 	
	//12byte
	word current_speed;			//현재속도(RPM)	
	byte patient_sex_age;	    //MSB로 구분 남 : 0, 여 : 1 Ex) 여성 65세 -> 1100 0001 / 남성 65세 -> 0100 0001
	byte remain_time;			//남은 운동시간(분)	
	//16Byte
	byte  cnt;						//운동진행횟수
	byte  dir;						//방향(0:up_dir, 1:down_dir)
	byte  velocity_mode;			//현재속도모드(0:등속, 1:가속)
	byte  state;					//현재 운동상황(0 : 운동중 or 사용중 , 1 : 대기중 or 운동종료 , 2 : 하한각이동중 , 3 : 일시정지 중 , 4: 일시정지 완료)
	//20Byte
	float current_angle;			//현재각도
	float current;					//전류
	//28Byte
	byte upper_limit_angle;	    	//측정모드(상한각)
	byte lower_limit_angle;	    	//측정모드(하한각)
	/*****Error para*****/
	byte EMS_error;					//비상정지 스위치(0:체크중, 1:비상정지 해제, 2:비상정지 상태)
	byte CT_error;					//전류센서 에러(0:체크중, 1:정상, 2:에러)
	//32Byte
	byte PM_error;					//각도센서 에러(0:체크중, 1:정상, 2:에러)
	byte encoder_error;				//엔코더  에러(0:체크중, 1:정상, 2:에러)
	byte speed_error;				//설정범위이탈 에러(0:정상, 1:에러)
	byte ESB_state;					//비상정지버튼(0:안눌림, 1:눌림)
	//36Byte
	byte start_sw;					//START SW	
	byte stop_sw;					//STOP SW
	byte lora_wr_fg;				// 0 로라로 부터 데이터를 받아와서 LCD로 데이터 전송한다는 Flag,  1 : LoRa로 부터 데이터를 받아와 LCD로 전송
	byte upper_angle;				// 0 상한각 ( 0 ~ 230 통신후 실제 변수에 담기는 값은 -50 처리후 넣을것 ) ( -50 ~ 180 )
	//40Byte
	byte lower_angle;				// 0 하한각 ( 0 ~ 230 통신후 실제 변수에 담기는 값은  -70 처리후 넣을것 ) ( -70 ~ 160 )
	byte upper_stop_time;			//상한 정지 시간(초)	
	byte lower_stop_time;			//하한 정지 시간(초)	
	byte mode;						// 운동 모드(0:일반등속 , 1:적응, 2:집중 3: 일반가속)
	//44Byte
	byte speed;						// 0 ( 운동 속도 : 1 ~ 9단계 )
	byte exerc_time;				//운동 시간(분) // LCD에서 받으면  _USER_RUN_SETTING.cnt_mode = 0(운동 시간 모드)로 변경
	byte exerc_num;					//운동 횟수	// LCD에서 받으면  _USER_RUN_SETTING.cnt_mode = 1(운동 횟수 모드)로 변경
	byte special_angle;				// 5 ~ 15 ( 집중/적응 운동 각도 최소 5°, 최대 15° )
	//48Byte
	byte repeat_num;				// 3 ~ 10 ( 집중/적응 운동 반복 횟수 최소 3회, 최대 10회 )
	byte special_location;			// 0 ~ 2( 상한각 : 0x00, 하한각 : 0x01, 상하한각 : 0x02 )
	byte load_sensitivity;			// 1 ~ 10( 모터 드라이버 EEPROM에 저장되어 있는 부하 민감도를 LCD에 보냄	 )					
	byte slave_lora_number;			// 1 ~ 255 Slave LoRa 국번( LCD에 띄움 )
	//52Byte
}SENSOR_DATA;//52Byte
extern SENSOR_DATA  _SENSOR_DATA;
#define SENSOR_ADDRESS	&_SENSOR_DATA

typedef struct CALIBRATION_SETTING
{
	word angle_ad_point_1;			//210도에서의 AD평균값
	word angle_ad_point_2;			//170도에서의 AD평균값
	word angle_ad_point_3;			//-30도에서의 AD평균값	
	word angle_ad_point_4;			//10도에서의 AD평균값
	
	float current_500mA_AD;		//0.5A에서의 AD평균값
	float current_2500mA_AD;	//2.5A에서의 AD평균값
	byte AD_ok;					//(0:평상시, 1:AD완료)
	byte dummy_1;			
	byte dummy_2;					
	byte dummy_3;					
}CALIBRATION_SETTING; // 20byte
extern CALIBRATION_SETTING		_CALIBRATION_SETTING;
#define _CALIBRATION_ADDRESS	&_CALIBRATION_SETTING
/***********************환자 이름 데이터*******************************/
/*환자 이름은 한글자당 최대 2byte씩, 총 글자수는 6개까지 가능하다.( ex : 대한민국 사람 )
 SD 카드에 저장하는 이름 데이터의 종류는 CP949이며, 한글 길이는 최대 4자리 까지 가능하다.
LCD에 출력되는 한글 글자수는 최대 6자리까지 가능하며 UTF-8로 표시된다.
모터드라이버와 LCD간 UART 통신을 통해 데이터를 받아와서 글자를 출력할 때는 
UART는 1byte씩 받아옴으로 받아온 UTF-8 데이터 2개를 합쳐서 2byte로 만들어야 한다( 이게 한글자 )
또한 PC에서 모터 드라이브로 환자 정보 데이터를 보낼때는 UTF-8 형식으로 보내야한다.*/
/***********************환자 이름 데이터*******************************/
typedef struct LoRa_Data
{
    // Read / Write 
    word user_name_1;				// 0x2000_0000 : 0x0000 ~ 0xFFFF ( 환자 이름(성) ) 표준값 : 0xD64D( 홍 )
	word user_name_2;				// 0x2000_0002 : 0x0000 ~ 0xFFFF ( 환자 이름(이름1) ) 표준값 : 0xAE38( 길 )
	
	word user_name_3;				// 0x2000_0004 : 0x0000 ~ 0xFFFF ( 환자 이름(이름2) ) 표준값 : 0xB3D9( 동 )
	word user_name_4;				// 0x2000_0006 : 0x0000 ~ 0xFFFF ( 환자 이름(이름3) )
	
 	word user_name_5;				// 0x2000_0008 : 0x0000 ~ 0xFFFF ( 환자 이름(이름4) )
	word user_name_6;				// 0x2000_000A : 0x0000 ~ 0xFFFF ( 환자 이름(이름5) )		
	
	byte user_age;					// 0x2000_000C : 1 ~ 120세 
	byte user_sex;					// 0x2000_000D	   : 남 : 0, 여 : 1
	
	word user_num_high;				// 0x2000_000E : 0 ( 환자 등록 번호 1 ~ 99999999 ) ( 상위 2byte ) 
	
	word user_num_low;				// 0x2000_0010 : 0 ( 환자 등록 번호 1 ~ 99999999 ) ( 하위 2byte ) 
	
	byte upper_angle;				// 0x2000_0012 : 0 상한각 ( 0 ~ 230 통신후 실제 변수에 담기는 값은 -50 처리후 넣을것 ) ( -50 ~ 180 )
	byte lower_angle;				// 0x2000_0013 : 0 하한각 ( 0 ~ 230 통신후 실제 변수에 담기는 값은  -70 처리후 넣을것 ) ( -70 ~ 160 )

	byte stop_time;					// 0x2000_0014 : ( 상한정지시간 : 상위 4bit, 하한정지시간 : 하위 4bit )
	byte mode;						// 0x2000_0015 :  //운동 모드(0:일반등속 , 1:적응, 2:집중 3: 일반가속)
	
	byte velocity_mode;				// 0x2000_0016 : 0 ( 운동 속도 : 1 ~ 9단계 )
	byte exerc_time_and_exercnum;	// 0x2000_0017 : 0 ( 1 ~ 99 윤동시간 또는 운동횟수 선택이므로 MSB가 0이면 운동시간, 1이면 운동횟수 )
	
	byte special_angle;				// 0x2000_0018 : 5 ~ 15 ( 집중/적응 운동 각도 최소 5°, 최대 15° )
	byte repeat_num;				// 0x2000_0019 : 3 ~ 10 ( 집중/적응 운동 반복 횟수 최소 3회, 최대 10회 )
	
	byte special_location;			// 0x2000_001A : 0 ~ 2( 상한각 : 0x00, 하한각 : 0x01, 상하한각 : 0x02 )
	byte measure_check;				// 0x2000_001B : 0 ~ 1( 측정 데이터를 받았다는 신호 ) 1이면 PC에서 모터드라이버로 받았다는 신호를 보내서 state값을 0x00 -> 0x01로 변경, use_state 값을 0x81 -> 0x01로 변경
	
	//Read Only
	byte check_state;               // 0x2000 001C : 0 에러체크 완료 : 0x00, 전류 에러체크 진행중 : 0x02, 엔코더 에러체크 진행중 : 0x04, 각도 에러체크 진행중: 0x08
	byte state;						// 0x2000_001D : 주기적으로 통신(통신 주기 미정) / 0x00 : 운동중 or 사용중, 0x01 : 대기중 or 운동종료, 0x02 : 고장, 0x04 : 연결실패
	
	//use_state는 state가 무조건 0x00 : 운동중일 때만 바뀐다.
	byte use_state;					// 0x2000_001E : 운동중 : 0x00, 정지 : 0x01, 측정중 : 0x02, 수동운동 : 0x04, 운동종료 : 0x08  //state가 0일 경우에만 변경   수동운동 0x04는 lcd에서 수동모드 화면 들어가면 0x04가 되게끔 만들어야함 아직 구현안함24.04.09
	byte error_state;				// 0x2000_001F : 0 : 에러없음 ( 0 ~ 5, 비상정지 : 0x01, 과전류 : 0x02, 엔코더 : 0x04, 각도이탈 : 0x08, 통신에러 : 0x10 )
	
	byte remain_time_and_cnt; 		// 0x2000_0020 : 0 ( 0 ~ 99 남은 운동시간 또는 남은 운동횟수 선택이므로 MSB가 0이면 남은 운동시간, 1이면 남은 운동횟수 )
	byte upper_limit_angle;			// 0x2000_0021 : 0 측정 상한각 ( 0 ~ 230 통신후 실제 변수에 담기는 값은 -50 처리후 넣을것 ) ( -50 ~ 180 )
	
	byte lower_limit_angle;			// 0x2000_0022 : 0 측정 하한각 ( 0 ~ 230 통신후 실제 변수에 담기는 값은  -70 처리후 넣을것 ) ( -70 ~ 160 )
	byte motion;					// 0x2000_0023 : 0 ~ 4 ( 표준값은 : 0x00 ( 어깨 ), [ 0x00 : 어깨, 0x01 : 팔꿈치, 0x02 : 무릎 , 0x04 : 손목, 0x08 : 발목 ] )
	
	int16_t reception_sensitivity;     // 0x2000_0024 : -120 ~ 0 dB ( 수신 감도 Slave LoRa로 부터 받아옴 )
	byte rsv1;						// Dummy_1
	byte rsv2;						// Dummy_2
}LoRa_Data;         // 40byte
extern LoRa_Data	_LoRa_Data;
#define LORA_MEMORY &_LoRa_Data

typedef struct Voice_value
{
	byte x06_1B;
	byte Mute_Flag;
	byte Volum_up_down;	
	byte equipment_num; //장비번호(1~255)
}Voice_value;  
extern Voice_value		_VOICE_DATA;
#define VOICE_DATA_ADDRESS	&_VOICE_DATA

typedef struct MD_EEPROM_DATA
{
	byte value_0;
	byte value_1;
	byte value_2;
	byte value_3;
	byte value_4;
	byte value_5;
	byte value_6;
	byte value_7;
	byte value_8;
	byte value_9;
	byte value_10;
	byte value_11;
	byte value_12;
	byte value_13;
	byte value_14;
	byte value_15;
}MD_EEPROM_DATA;  
extern MD_EEPROM_DATA		_MD_EEPROM_DATA;
#define MD_EEPROM_ADDRESS	&_MD_EEPROM_DATA

extern byte Tx_lora_data;

extern word CRC16(byte *buf, word size);
extern word CheckSum(byte *Data, byte Len);
extern void RS232_putchar(byte RS232_tx_val);

extern word angle_ad_point1, angle_ad_point2, angle_ad_point3, angle_ad_point4, angle_ad_point5;
extern float real_angle_point1, real_angle_point2, real_angle_point3, real_angle_point4, real_angle_point5;
extern float real_current_500mA, real_current_2500mA, cal_500mA_val, cal_2500mA_val;

extern byte Rx4_buf[100];
extern float bytes_to_float(byte *bytes_data);
//CRC16 버그때문에 테스트용으로
extern byte CheckSum_EN;
extern byte Rx4_index;
extern word CheckSum_data;
extern word LCD_crc;
extern word MT_Driver_crc;

extern byte Rx4_Checksum_H;
extern byte Rx4_Checksum_L;
extern byte CheckSum_ing;

extern word Rcv2_ok;
extern byte RS485_dead_time;	//1msec down counter
extern byte Rx4_step;


extern byte wait_high_to_low, start_high_to_low,Measurement_CNT;
/*****************************************LoRa*************************************************/
extern byte ENQ; // 로라 모듈 국번

extern byte Rx2_index, Rx2_next_data_no,plus_cnt;
extern word LoRa_CheckSum_data;
extern word Error2_cnt;
extern byte Rx2_step;

extern byte Rx2_CRC_H, Rx2_CRC_L ,Rx4_CRC_H ,Rx4_CRC_L;
extern byte LoRa_CheckSum_ing;
extern byte LoRa_CheckSum_EN;

extern byte LoRa_Rxbuf[255];

extern word LoRa_Rcv_ok;
extern byte LoRa_RS485_dead_time;

extern word LoRa_test_crc1, LoRa_test_crc2;

extern byte EMS_STATE;
extern byte over_angle, start_over_angle, Detect_ok;
extern void LoRa_RS232_putchar(byte rs232_tx_num);
extern void LoRa_UART2_exe(void);
extern float Rate_of_change(float value);
extern void Soft_start_stop_PI(word accel_time, word deaccel_time, byte speed_step, byte velocity_mode);

extern byte read_sensitivity,start_read_sense, wait_measurement;
extern byte over_angle__start_flag;
extern byte LoRa_Tx_buf[255];
extern byte LoRa_Tx_index, LoRa_Equipment_Num;
extern byte LoRa_Tx_send_number,TX_EMS_CNT;
//typedef unsigned short word;
//typedef unsigned char byte;
extern float test_rate;
extern byte Benchmark, Detect_load, Change_dir;
//LoRa 모듈 관련
extern byte lora_tx_EN;
extern byte Tx2_buf[100];
/*****************************************LoRa*************************************************/
extern unsigned int Motor_PWM_CCW, Motor_PWM_CW, Motor_PWM;
/*****************************************BlueTooth*************************************************/
//음성모듈
extern byte Setting_voicem, no_repeat, Volume_step, Voice_data, Setting_voice, Setting_in;
struct Ble_Data // 추가한것
{
    //Read Only
	byte current_angle;			//현재각도
	byte upper_angle;				// 0x2000_0000 : 0 상한각 ( 0 ~ 230 통신후 실제 변수에 담기는 값은 -50 처리후 넣을것 ) ( -50 ~ 180 )
	byte lower_angle;				// 0x2000_0001 : 0 하한각 ( 0 ~ 230 통신후 실제 변수에 담기는 값은  -70 처리후 넣을것 ) ( -70 ~ 160 )
	byte stop_time;					// 0x2000_0002 : ( 상한정지시간 : 상위 4bit, 하한정지시간 : 하위 4bit )
	
	byte motion;					// 0x2000_0003 : 0 ~ 4 ( 표준값은 : 0x00 ( 어깨 ), [ 0x00 : 어깨, 0x01 : 팔꿈치, 0x02 : 무릎 , 0x04 : 손목, 0x08 : 발목 ] )
	byte mode;						// 0x2000_0004 : //운동 모드(0:일반등속 , 1:적응, 2:집중 3: 일반가속, 4 : 수동모드, 5 : 측정모드)
	byte velocity_mode;				// 0x2000_0005 : 0 ( 운동 속도 : 1 ~ 9단계 )
	byte exerc_time_and_exercnum;	// 0x2000_0006 : 0 ( 1 ~ 99 윤동시간 또는 운동횟수 선택이므로 MSB가 0이면 운동시간, 1이면 운동횟수 )
	
	byte upper_limit_angle;			// 0x2000_0007 : 0 측정 상한각 ( 0 ~ 230 통신후 실제 변수에 담기는 값은 -50 처리후 넣을것 ) ( -50 ~ 180 )
	byte lower_limit_angle;			// 0x2000_0008 : 0 측정 하한각 ( 0 ~ 230 통신후 실제 변수에 담기는 값은  -70 처리후 넣을것 ) ( -70 ~ 160 )
	byte remain_time_and_cnt; 		// 0x2000_000A : 0 ( 0 ~ 99 남은 운동시간 또는 남은 운동횟수 선택이므로 MSB가 0이면 남은 운동시간, 1이면 남은 운동횟수 )//남은 운동시간
	
	byte special_angle;				// 0x2000_0018 : 5 ~ 15 ( 집중/적응 운동 각도 최소 5°, 최대 15° )
	byte repeat_num;				// 0x2000_0019 : 3 ~ 10 ( 집중/적응 운동 반복 횟수 최소 3회, 최대 10회 )
	byte exerc_time;				// 운동 시간(분)									
	byte exerc_num;					// 운동 횟수	
	
	byte exer_state;				// 0x2000_0020 : 0 : 평상시, 1 : 운동 종료, 2 : 측정 완료
	byte rsv1;						// dummy_1
	byte rsv2;						// dummy_2								
	byte rsv3;						// dummy_3
}; // 20byte
extern struct Ble_Data _Ble_Data;

extern unsigned int bluetooth_AT;
extern unsigned char Ble_buffer[50];
extern void Ble_Data_Tx(byte ble_tx_num);	//UART3
extern byte Rx3_buf[100],Ble_Rcv_ok,Ble_CheckSum_EN;
extern void Ble_UART3_exe(void);
/*****************************************BlueTooth*************************************************/
extern byte BF_EMS_SW, Measurement_mode, restart_f;
extern unsigned int ble_name_change_fg;
extern unsigned char AT_BTNAME[21]; 
/***************************Motor-Driver**********************************/
extern byte Settings, cant_read_id;
extern byte Settings_cancel;
extern byte Settings_save;
/***************************Motor-Driver**********************************/
extern float total_low_angle, total_high_angle, Load_sensitivity;
extern byte measure_restart;
extern float MPC_control_sys(float target, float current);