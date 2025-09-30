//Buzzer(PC7)
#define	BUZ_H		(HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET))
#define	BUZ_L		(HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET))

//FA_RESET(PA1)
#define FA_RESET_H	GPIOA->ODR |= GPIO_PIN_1	//PA1 <= 1
#define FA_RESET_L	GPIOA->ODR &= ~GPIO_PIN_1	//PA1 <= 0

//START SW Input(PE12)
#define START_SW		((GPIOE->IDR)>>12 & 0x01)	//PE12's IDR : Input Data Register

//STOP SW Input(PE13)
#define STOP_SW		((GPIOE->IDR)>>13 & 0x01)	//PE13's IDR : Input Data Register

//Emergency Stop Switch Input(PE14)
#define EMS_SW		((GPIOE->IDR)>>14 & 0x01)	//PE14's IDR : Input Data Register

//Motor drive
#define CW		0	//시계방향(상한각으로 올라갈때)
#define CCW		1	//반시계방향(하한각으로 내려갈깨)
#define STOP	2	//운동중이 아닐때(일시정지상태는 제외)

#define CONSTANT_MODE	1	//등속모드
#define ACCEL_MODE		0	//가속모드

#define NORMAL			1	//일반운동
#define ADAPTIVE		2	//적응운동
#define INTENSIVE		3	//집중운동
#define ADJUSTMENT		4	//각도조정

//캘리브레이션 측정 범위
#define ANGLE_RANGE 32 

//적용위치
#define LOWER	0	//하한각
#define	UP_LOW	1	//상하한각
#define UPPER	2	//상한각

//운동부위
#define Elbow		0	//팔꿈치
#define Shoulder	1	//어깨
#define Knee		2	//무릎
#define wrist       3   //손목
#define ankle       4   //발목

//Self check
#define Self_Check 	1	//0:셀프체크 기능 OFF(엔코더, 전류, 포텐셜미터 센서 값들을 임의적으로 정상 처리시켜 에러화면을 PASS함), 1:셀프체크 기능 ON 

//System Initialization value


//VOICE_CS(PD6) 
#define VOICE_CS_HIGH	GPIOD->ODR |= GPIO_PIN_6	//PD6 <= 1
#define VOICE_CS_LOW	GPIOD->ODR &= ~GPIO_PIN_6	//PD6 <= 0

//PWM SLEEP(PD10) 
#define PWM_NOT_SLEEP	GPIOD->ODR |= GPIO_PIN_10	//PD10 <= 1
#define PWM_SLEEP		GPIOD->ODR &= ~GPIO_PIN_10	//PD10 <= 0

//모터 높낮이, 모터 회전(PE10) 
#define MOTOR_ROTATE	GPIOE->ODR |= GPIO_PIN_10	//PE10 <= 1
#define MOTOR_UP_DOWN	GPIOE->ODR &= ~GPIO_PIN_10	//PE10 <= 0

//VOICE RESET(PB6) 
#define VOICE_RESET_HIGH	GPIOB->ODR |= GPIO_PIN_6	//PB6 <= 1
#define VOICE_RESET_LOW     GPIOB->ODR &= ~GPIO_PIN_6	//PB6 <= 0

#define LORA_RS485_DEAD_TIME 0 //모터드라이버가 로라로부터 Rx 받고 나서 Tx 하기까지 기다리는 시간이며 0으로 해야 정상이다.없어도 됨.