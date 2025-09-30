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
#define CW		0	//�ð����(���Ѱ����� �ö󰥶�)
#define CCW		1	//�ݽð����(���Ѱ����� ��������)
#define STOP	2	//����� �ƴҶ�(�Ͻ��������´� ����)

#define CONSTANT_MODE	1	//��Ӹ��
#define ACCEL_MODE		0	//���Ӹ��

#define NORMAL			1	//�Ϲݿ
#define ADAPTIVE		2	//�����
#define INTENSIVE		3	//���߿
#define ADJUSTMENT		4	//��������

//Ķ���극�̼� ���� ����
#define ANGLE_RANGE 32 

//������ġ
#define LOWER	0	//���Ѱ�
#define	UP_LOW	1	//�����Ѱ�
#define UPPER	2	//���Ѱ�

//�����
#define Elbow		0	//�Ȳ�ġ
#define Shoulder	1	//���
#define Knee		2	//����
#define wrist       3   //�ո�
#define ankle       4   //�߸�

//Self check
#define Self_Check 	1	//0:����üũ ��� OFF(���ڴ�, ����, ���ټȹ��� ���� ������ ���������� ���� ó������ ����ȭ���� PASS��), 1:����üũ ��� ON 

//System Initialization value


//VOICE_CS(PD6) 
#define VOICE_CS_HIGH	GPIOD->ODR |= GPIO_PIN_6	//PD6 <= 1
#define VOICE_CS_LOW	GPIOD->ODR &= ~GPIO_PIN_6	//PD6 <= 0

//PWM SLEEP(PD10) 
#define PWM_NOT_SLEEP	GPIOD->ODR |= GPIO_PIN_10	//PD10 <= 1
#define PWM_SLEEP		GPIOD->ODR &= ~GPIO_PIN_10	//PD10 <= 0

//���� ������, ���� ȸ��(PE10) 
#define MOTOR_ROTATE	GPIOE->ODR |= GPIO_PIN_10	//PE10 <= 1
#define MOTOR_UP_DOWN	GPIOE->ODR &= ~GPIO_PIN_10	//PE10 <= 0

//VOICE RESET(PB6) 
#define VOICE_RESET_HIGH	GPIOB->ODR |= GPIO_PIN_6	//PB6 <= 1
#define VOICE_RESET_LOW     GPIOB->ODR &= ~GPIO_PIN_6	//PB6 <= 0

#define LORA_RS485_DEAD_TIME 0 //���͵���̹��� �ζ�κ��� Rx �ް� ���� Tx �ϱ���� ��ٸ��� �ð��̸� 0���� �ؾ� �����̴�.��� ��.