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

//����üũ
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

//����
extern byte voice_out;
extern void voice_output(void);
extern byte voice_error_check;
extern void SPI_voice_set(void);

//TIM4 CCR���� Ȯ�ο�
extern dword TIM4_CCR1_buffer, TIM4_CCR2_buffer; 

//�����
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
extern uint32_t float_to_uint32(float value);// �ε� �Ҽ��� ���� 4����Ʈ ������ ��ȯ�ϴ� �Լ�
extern byte save_ct_flag;
extern byte angle_cal_ad_1, angle_cal_ad_2, angle_cal_ad_3, angle_cal_ad_4, angle_cal_ad_5;
extern byte ct_cal_ad_1, ct_cal_ad_2;
extern float real_angle;
//user_define
#define LED1_OFF	GPIOB->ODR |= GPIO_PIN_12;	 //PB12---->1
#define LED1_ON		GPIOB->ODR &= ~GPIO_PIN_12;	 //PB12---->0
//���� Ķ���극�̼�
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
//RS232 ���(UART2)
#define UART_STX	0x01	
//�ڵ彺��ġ
extern byte START_STATE, STOP_STATE;
extern void UART4_exe(void);

//Ÿ�̸�5 �ð����� �Լ�
extern void start_timer();
extern float stop_timer();

extern void copyLoRaDataToSensorData();

//��������(Potentiometer)
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
//���� Ķ���극�̼�
extern float lower_angle, upper_angle, lower_ad, upper_ad, purse1_down_1angle;
extern byte SYSTEM_INITIALIZATION_VALUE_ANGLE;
extern int find_lower_index(uint16_t* arr, int size, float value);
extern int find_upper_index(uint16_t* arr, int size, float value);
extern float interpolate(uint16_t* angle_arr, int size, float P_M_avr, float real_angle_point4, float real_angle_point2, uint16_t angle_ad_point4, uint16_t angle_ad_point2, float* bf_pm_actual);
extern byte angle_record_flag, calibration_state, eeprom_page_state;
extern word ad_count_down, remain_ad, low_to_high_pwm, high_to_low_pwm;
extern dword purse1_up_1angle;
//exercise_mode 1 : �Ϲ�, 2 : ����, 3 : ����, 4: �������̵�, 5 : ������� up 1��, 6 : ������� down 1��, 7 : Up ing, 8 : Down ing, 9 : ROM(�������)
typedef struct USER_RUN_SETTING
{
	byte motion;				//� ����(0:�Ȳ�ġ, 1:���, 2:����, 3:�ո�, 4:�߸�)
	byte mode;					//� ���(0:�Ϲ�, , 1:����, 2:����)
	byte exerc_time;			//� �ð�(��)									
	byte exerc_num;				//� Ƚ��		
	
	byte adaptation_angle;		//���� ����	
	byte repeat_num;			//�ݺ� Ƚ��	
	byte concentration_angle;	//���� ����
	byte application_location;	//���� ��ġ(0:���Ѱ�, 1:�����Ѱ�, 2:���Ѱ�)
	
	byte speed_mode;			//�ӵ� ���(0:����, 1:���)	
	byte speed;					//� �ӵ� �ܰ�(1~7)
	byte upper_stop_time;		//���� ���� �ð�(��)	
	byte lower_stop_time;		//���� ���� �ð�(��)	
	
	float upper_angle;			//���� ����
	float lower_angle;			//���� ����	
	
	byte cnt_mode;				//� �ð�,Ƚ����� ����(0:��ð�, 1:�Ƚ��)			
	byte exerc_start;			//(0:����, 1:�����, 2:�Ͻ�����, 3:����, 4:������� 1�� up, 5:������� 1�� down, 6:������� up ing, 7:������� down ing, 8:�������, 9:������� �Ͻ�����, 0A:���� ī��Ʈ �ʱ�ȭ)
	byte calibration;			//(0:����, 1:Calibration ������, 2 : ���� �ΰ��� ����)
	byte ad_start;				//(0:����, 1:���� AD����1, 2:���� AD����2 3:���� AD����3, 4:���� AD����4 5:���� AD����1, 6:���� AD����2)
	
	float real_value;			//(0:����, Ķ���극�̼ǽ� ��Ȯ�� ����,������)
	
	byte equip_up_down;			//(0:����, 1: ��� up, 2:��� up ing 3:��� down, 4:��� down ing)
	byte x06_1B;				//
	byte equipment_num;			//����ȣ
	byte re_self_check;	
}USER_RUN_SETTING;  //32Byte
extern USER_RUN_SETTING		_USER_RUN_SETTING;
#define USER_RUN_ADDRESS	&_USER_RUN_SETTING

typedef struct SENSOR_DATA
{
	word user_name_1;            //0xD64D (ȫ)   
	word user_name_2;            //0xAE38 (��) 
	
	word user_name_3;            //0xB3D9 (��) 
	word user_name_4;            //0x0000 ( ) 
	
	word user_name_5;            //0x0000 ( ) 
	word user_name_6;            //0x0000 ( ) 	
	//12byte
	word current_speed;			//����ӵ�(RPM)	
	byte patient_sex_age;	    //MSB�� ���� �� : 0, �� : 1 Ex) ���� 65�� -> 1100 0001 / ���� 65�� -> 0100 0001
	byte remain_time;			//���� ��ð�(��)	
	//16Byte
	byte  cnt;						//�����Ƚ��
	byte  dir;						//����(0:up_dir, 1:down_dir)
	byte  velocity_mode;			//����ӵ����(0:���, 1:����)
	byte  state;					//���� ���Ȳ(0 : ��� or ����� , 1 : ����� or ����� , 2 : ���Ѱ��̵��� , 3 : �Ͻ����� �� , 4: �Ͻ����� �Ϸ�)
	//20Byte
	float current_angle;			//���簢��
	float current;					//����
	//28Byte
	byte upper_limit_angle;	    	//�������(���Ѱ�)
	byte lower_limit_angle;	    	//�������(���Ѱ�)
	/*****Error para*****/
	byte EMS_error;					//������� ����ġ(0:üũ��, 1:������� ����, 2:������� ����)
	byte CT_error;					//�������� ����(0:üũ��, 1:����, 2:����)
	//32Byte
	byte PM_error;					//�������� ����(0:üũ��, 1:����, 2:����)
	byte encoder_error;				//���ڴ�  ����(0:üũ��, 1:����, 2:����)
	byte speed_error;				//����������Ż ����(0:����, 1:����)
	byte ESB_state;					//���������ư(0:�ȴ���, 1:����)
	//36Byte
	byte start_sw;					//START SW	
	byte stop_sw;					//STOP SW
	byte lora_wr_fg;				// 0 �ζ�� ���� �����͸� �޾ƿͼ� LCD�� ������ �����Ѵٴ� Flag,  1 : LoRa�� ���� �����͸� �޾ƿ� LCD�� ����
	byte upper_angle;				// 0 ���Ѱ� ( 0 ~ 230 ����� ���� ������ ���� ���� -50 ó���� ������ ) ( -50 ~ 180 )
	//40Byte
	byte lower_angle;				// 0 ���Ѱ� ( 0 ~ 230 ����� ���� ������ ���� ����  -70 ó���� ������ ) ( -70 ~ 160 )
	byte upper_stop_time;			//���� ���� �ð�(��)	
	byte lower_stop_time;			//���� ���� �ð�(��)	
	byte mode;						// � ���(0:�Ϲݵ�� , 1:����, 2:���� 3: �Ϲݰ���)
	//44Byte
	byte speed;						// 0 ( � �ӵ� : 1 ~ 9�ܰ� )
	byte exerc_time;				//� �ð�(��) // LCD���� ������  _USER_RUN_SETTING.cnt_mode = 0(� �ð� ���)�� ����
	byte exerc_num;					//� Ƚ��	// LCD���� ������  _USER_RUN_SETTING.cnt_mode = 1(� Ƚ�� ���)�� ����
	byte special_angle;				// 5 ~ 15 ( ����/���� � ���� �ּ� 5��, �ִ� 15�� )
	//48Byte
	byte repeat_num;				// 3 ~ 10 ( ����/���� � �ݺ� Ƚ�� �ּ� 3ȸ, �ִ� 10ȸ )
	byte special_location;			// 0 ~ 2( ���Ѱ� : 0x00, ���Ѱ� : 0x01, �����Ѱ� : 0x02 )
	byte load_sensitivity;			// 1 ~ 10( ���� ����̹� EEPROM�� ����Ǿ� �ִ� ���� �ΰ����� LCD�� ����	 )					
	byte slave_lora_number;			// 1 ~ 255 Slave LoRa ����( LCD�� ��� )
	//52Byte
}SENSOR_DATA;//52Byte
extern SENSOR_DATA  _SENSOR_DATA;
#define SENSOR_ADDRESS	&_SENSOR_DATA

typedef struct CALIBRATION_SETTING
{
	word angle_ad_point_1;			//210�������� AD��հ�
	word angle_ad_point_2;			//170�������� AD��հ�
	word angle_ad_point_3;			//-30�������� AD��հ�	
	word angle_ad_point_4;			//10�������� AD��հ�
	
	float current_500mA_AD;		//0.5A������ AD��հ�
	float current_2500mA_AD;	//2.5A������ AD��հ�
	byte AD_ok;					//(0:����, 1:AD�Ϸ�)
	byte dummy_1;			
	byte dummy_2;					
	byte dummy_3;					
}CALIBRATION_SETTING; // 20byte
extern CALIBRATION_SETTING		_CALIBRATION_SETTING;
#define _CALIBRATION_ADDRESS	&_CALIBRATION_SETTING
/***********************ȯ�� �̸� ������*******************************/
/*ȯ�� �̸��� �ѱ��ڴ� �ִ� 2byte��, �� ���ڼ��� 6������ �����ϴ�.( ex : ���ѹα� ��� )
 SD ī�忡 �����ϴ� �̸� �������� ������ CP949�̸�, �ѱ� ���̴� �ִ� 4�ڸ� ���� �����ϴ�.
LCD�� ��µǴ� �ѱ� ���ڼ��� �ִ� 6�ڸ����� �����ϸ� UTF-8�� ǥ�õȴ�.
���͵���̹��� LCD�� UART ����� ���� �����͸� �޾ƿͼ� ���ڸ� ����� ���� 
UART�� 1byte�� �޾ƿ����� �޾ƿ� UTF-8 ������ 2���� ���ļ� 2byte�� ������ �Ѵ�( �̰� �ѱ��� )
���� PC���� ���� ����̺�� ȯ�� ���� �����͸� �������� UTF-8 �������� �������Ѵ�.*/
/***********************ȯ�� �̸� ������*******************************/
typedef struct LoRa_Data
{
    // Read / Write 
    word user_name_1;				// 0x2000_0000 : 0x0000 ~ 0xFFFF ( ȯ�� �̸�(��) ) ǥ�ذ� : 0xD64D( ȫ )
	word user_name_2;				// 0x2000_0002 : 0x0000 ~ 0xFFFF ( ȯ�� �̸�(�̸�1) ) ǥ�ذ� : 0xAE38( �� )
	
	word user_name_3;				// 0x2000_0004 : 0x0000 ~ 0xFFFF ( ȯ�� �̸�(�̸�2) ) ǥ�ذ� : 0xB3D9( �� )
	word user_name_4;				// 0x2000_0006 : 0x0000 ~ 0xFFFF ( ȯ�� �̸�(�̸�3) )
	
 	word user_name_5;				// 0x2000_0008 : 0x0000 ~ 0xFFFF ( ȯ�� �̸�(�̸�4) )
	word user_name_6;				// 0x2000_000A : 0x0000 ~ 0xFFFF ( ȯ�� �̸�(�̸�5) )		
	
	byte user_age;					// 0x2000_000C : 1 ~ 120�� 
	byte user_sex;					// 0x2000_000D	   : �� : 0, �� : 1
	
	word user_num_high;				// 0x2000_000E : 0 ( ȯ�� ��� ��ȣ 1 ~ 99999999 ) ( ���� 2byte ) 
	
	word user_num_low;				// 0x2000_0010 : 0 ( ȯ�� ��� ��ȣ 1 ~ 99999999 ) ( ���� 2byte ) 
	
	byte upper_angle;				// 0x2000_0012 : 0 ���Ѱ� ( 0 ~ 230 ����� ���� ������ ���� ���� -50 ó���� ������ ) ( -50 ~ 180 )
	byte lower_angle;				// 0x2000_0013 : 0 ���Ѱ� ( 0 ~ 230 ����� ���� ������ ���� ����  -70 ó���� ������ ) ( -70 ~ 160 )

	byte stop_time;					// 0x2000_0014 : ( ���������ð� : ���� 4bit, ���������ð� : ���� 4bit )
	byte mode;						// 0x2000_0015 :  //� ���(0:�Ϲݵ�� , 1:����, 2:���� 3: �Ϲݰ���)
	
	byte velocity_mode;				// 0x2000_0016 : 0 ( � �ӵ� : 1 ~ 9�ܰ� )
	byte exerc_time_and_exercnum;	// 0x2000_0017 : 0 ( 1 ~ 99 �����ð� �Ǵ� �Ƚ�� �����̹Ƿ� MSB�� 0�̸� ��ð�, 1�̸� �Ƚ�� )
	
	byte special_angle;				// 0x2000_0018 : 5 ~ 15 ( ����/���� � ���� �ּ� 5��, �ִ� 15�� )
	byte repeat_num;				// 0x2000_0019 : 3 ~ 10 ( ����/���� � �ݺ� Ƚ�� �ּ� 3ȸ, �ִ� 10ȸ )
	
	byte special_location;			// 0x2000_001A : 0 ~ 2( ���Ѱ� : 0x00, ���Ѱ� : 0x01, �����Ѱ� : 0x02 )
	byte measure_check;				// 0x2000_001B : 0 ~ 1( ���� �����͸� �޾Ҵٴ� ��ȣ ) 1�̸� PC���� ���͵���̹��� �޾Ҵٴ� ��ȣ�� ������ state���� 0x00 -> 0x01�� ����, use_state ���� 0x81 -> 0x01�� ����
	
	//Read Only
	byte check_state;               // 0x2000 001C : 0 ����üũ �Ϸ� : 0x00, ���� ����üũ ������ : 0x02, ���ڴ� ����üũ ������ : 0x04, ���� ����üũ ������: 0x08
	byte state;						// 0x2000_001D : �ֱ������� ���(��� �ֱ� ����) / 0x00 : ��� or �����, 0x01 : ����� or �����, 0x02 : ����, 0x04 : �������
	
	//use_state�� state�� ������ 0x00 : ����� ���� �ٲ��.
	byte use_state;					// 0x2000_001E : ��� : 0x00, ���� : 0x01, ������ : 0x02, ����� : 0x04, ����� : 0x08  //state�� 0�� ��쿡�� ����   ����� 0x04�� lcd���� ������� ȭ�� ���� 0x04�� �ǰԲ� �������� ���� ��������24.04.09
	byte error_state;				// 0x2000_001F : 0 : �������� ( 0 ~ 5, ������� : 0x01, ������ : 0x02, ���ڴ� : 0x04, ������Ż : 0x08, ��ſ��� : 0x10 )
	
	byte remain_time_and_cnt; 		// 0x2000_0020 : 0 ( 0 ~ 99 ���� ��ð� �Ǵ� ���� �Ƚ�� �����̹Ƿ� MSB�� 0�̸� ���� ��ð�, 1�̸� ���� �Ƚ�� )
	byte upper_limit_angle;			// 0x2000_0021 : 0 ���� ���Ѱ� ( 0 ~ 230 ����� ���� ������ ���� ���� -50 ó���� ������ ) ( -50 ~ 180 )
	
	byte lower_limit_angle;			// 0x2000_0022 : 0 ���� ���Ѱ� ( 0 ~ 230 ����� ���� ������ ���� ����  -70 ó���� ������ ) ( -70 ~ 160 )
	byte motion;					// 0x2000_0023 : 0 ~ 4 ( ǥ�ذ��� : 0x00 ( ��� ), [ 0x00 : ���, 0x01 : �Ȳ�ġ, 0x02 : ���� , 0x04 : �ո�, 0x08 : �߸� ] )
	
	int16_t reception_sensitivity;     // 0x2000_0024 : -120 ~ 0 dB ( ���� ���� Slave LoRa�� ���� �޾ƿ� )
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
	byte equipment_num; //����ȣ(1~255)
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
//CRC16 ���׶����� �׽�Ʈ������
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
extern byte ENQ; // �ζ� ��� ����

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
//LoRa ��� ����
extern byte lora_tx_EN;
extern byte Tx2_buf[100];
/*****************************************LoRa*************************************************/
extern unsigned int Motor_PWM_CCW, Motor_PWM_CW, Motor_PWM;
/*****************************************BlueTooth*************************************************/
//�������
extern byte Setting_voicem, no_repeat, Volume_step, Voice_data, Setting_voice, Setting_in;
struct Ble_Data // �߰��Ѱ�
{
    //Read Only
	byte current_angle;			//���簢��
	byte upper_angle;				// 0x2000_0000 : 0 ���Ѱ� ( 0 ~ 230 ����� ���� ������ ���� ���� -50 ó���� ������ ) ( -50 ~ 180 )
	byte lower_angle;				// 0x2000_0001 : 0 ���Ѱ� ( 0 ~ 230 ����� ���� ������ ���� ����  -70 ó���� ������ ) ( -70 ~ 160 )
	byte stop_time;					// 0x2000_0002 : ( ���������ð� : ���� 4bit, ���������ð� : ���� 4bit )
	
	byte motion;					// 0x2000_0003 : 0 ~ 4 ( ǥ�ذ��� : 0x00 ( ��� ), [ 0x00 : ���, 0x01 : �Ȳ�ġ, 0x02 : ���� , 0x04 : �ո�, 0x08 : �߸� ] )
	byte mode;						// 0x2000_0004 : //� ���(0:�Ϲݵ�� , 1:����, 2:���� 3: �Ϲݰ���, 4 : �������, 5 : �������)
	byte velocity_mode;				// 0x2000_0005 : 0 ( � �ӵ� : 1 ~ 9�ܰ� )
	byte exerc_time_and_exercnum;	// 0x2000_0006 : 0 ( 1 ~ 99 �����ð� �Ǵ� �Ƚ�� �����̹Ƿ� MSB�� 0�̸� ��ð�, 1�̸� �Ƚ�� )
	
	byte upper_limit_angle;			// 0x2000_0007 : 0 ���� ���Ѱ� ( 0 ~ 230 ����� ���� ������ ���� ���� -50 ó���� ������ ) ( -50 ~ 180 )
	byte lower_limit_angle;			// 0x2000_0008 : 0 ���� ���Ѱ� ( 0 ~ 230 ����� ���� ������ ���� ����  -70 ó���� ������ ) ( -70 ~ 160 )
	byte remain_time_and_cnt; 		// 0x2000_000A : 0 ( 0 ~ 99 ���� ��ð� �Ǵ� ���� �Ƚ�� �����̹Ƿ� MSB�� 0�̸� ���� ��ð�, 1�̸� ���� �Ƚ�� )//���� ��ð�
	
	byte special_angle;				// 0x2000_0018 : 5 ~ 15 ( ����/���� � ���� �ּ� 5��, �ִ� 15�� )
	byte repeat_num;				// 0x2000_0019 : 3 ~ 10 ( ����/���� � �ݺ� Ƚ�� �ּ� 3ȸ, �ִ� 10ȸ )
	byte exerc_time;				// � �ð�(��)									
	byte exerc_num;					// � Ƚ��	
	
	byte exer_state;				// 0x2000_0020 : 0 : ����, 1 : � ����, 2 : ���� �Ϸ�
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