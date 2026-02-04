/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "dma.h"
#include "fdcan.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "fdcan.h"
#include "pid.h"
#include "CAN_receive.h"
#include "Yobotics.h"
#include "newTrot.h"
#include "newIK.h"
#include "math.h"
#include "gyro.h"
#include "imu.h"
#include "bound.h"
#include "bianliang.h"
#include "remote.h" 
uint8_t uart4_rxbuf[1]={0};
uint8_t uart4_txbuf[1]={0};
FDCAN_RxHeaderTypeDef CAN_RxHeaderStruct;
uint8_t rx_data[8]={0};
int m=0;
uint8_t uart_rx_data = 0;
uint8_t RxBuf[15],RxBuf_UART8;
uint8_t RxBuf2[10];
uint8_t	U7_RxBuf;
uint8_t RemoteBuffer[20u];
uint8_t RxBuf_USART1;
uint8_t USART1Flag=0;
float delta_k_p21 = 0;
float delta_p_des12 = 0;
float delta_p_des22 = 0;
float delta_p_des23 = 0;		

RxText rx_text;

extern int32_t gyro_vel[3]; // �����ǽ��ս��ٶ�����

extern int32_t gyro_a[3]; // �����ǽ��սǼ��ٶ�����

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

uint8_t RemoteBuffer_s[100];

unsigned int remote_time;
extern moto_measure_Yobotics motor[8];
extern moto_measure_Yobotics Yobotics[8];
extern int trotDirectionState;
extern float leg_Mode[5];
int chongshi_flag=0;

float change_angle=0;
extern int flag;
int number=0;
extern int step_n;
int tiaozero_flag;
uint8_t tap;
extern float detla_kp1,detla_kp2;
extern float detlam1,detlam2;
float k_try1,k_try2=0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LENGTH 1

uint8_t RxBuffer[LENGTH];
uint8_t Uart_RxFlag = 0;
extern float Gamma,Omega;// ŷ���Ǽ����ٶ�
extern int leg_mode[5];
/*
  * @brief  The application entry point.
  * @retval int
  */
uint8_t  USART_send_data[15];
void HAL_UART_RxCpltCallback(UART_HandleTypeDef*huart);
void INIT(void);
void M_stop(); 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern int trotDirectionState;

uint32_t time[4]={0};

uint8_t RxBuf_USART3;
uint8_t USART3Flag=0;
void printTrotMenu();
extern double heightControlNow;
extern float centre;
extern float bufu;
#include <ctype.h>  // ���� isdigit ���� (Include isdigit function)
double stringToNumber(char *str);
uint8_t fixedValue[20]="";
char *fixedValuestr = (char *)fixedValue;
int fixedValueBit=0;
double fixedNumber=0;
extern double xpre;
extern double xfinal;
extern double zHeight;
extern double M_T;
extern double zHeight_L;
extern double zHeight_R;
	extern float __kpp[5];
	extern float __kdd[5];
#include <string.h>
int sw1_last_state;
char data[] = "666";// ���ڷ������� (Serial port send data)
uint16_t size = sizeof(data) - 1; // ��ȥ�ַ���ĩβ�Ŀ��ַ� (Subtract the null character at the end of the string)





int change_Kpp1_Flag=0;// �Ƿ��޸ı�־ (Whether to modify flag)
float __Kpp1;// �޸�ֵ (Modified value)
int change_Kpp2_Flag=0;// �Ƿ��޸ı�־ (Whether to modify flag)
float __Kpp2;// �޸�ֵ (Modified value)
int change_Kpp3_Flag=0;// �Ƿ��޸ı�־ (Whether to modify flag)
float __Kpp3;// �޸�ֵ (Modified value)
int change_Kpp4_Flag=0;// �Ƿ��޸ı�־ (Whether to modify flag)
float __Kpp4;// �޸�ֵ (Modified value)
int change_Kdd1_Flag=0;// �Ƿ��޸ı�־ (Whether to modify flag)
float __Kdd1;// �޸�ֵ (Modified value)
int change_Kdd2_Flag=0;// �Ƿ��޸ı�־ (Whether to modify flag)
float __Kdd2;// �޸�ֵ (Modified value)
int change_Kdd3_Flag=0;// �Ƿ��޸ı�־ (Whether to modify flag)
float __Kdd3;// �޸�ֵ (Modified value)
int change_Kdd4_Flag=0;// �Ƿ��޸ı�־ (Whether to modify flag)
float __Kdd4;// �޸�ֵ (Modified value)

int change_xpre_Flag=0;
double __xpre;
int change_xfinal_Flag=0;
double __xfinal;
int change_zHeight_Flag=0;
double __zHeight;
int change_heightControlNow_Flag=0;
double __heightControlNow;
int change_M_T_Flag=0;// �Ƿ��޸ı�־ (Whether to modify flag)
double __M_T;// �޸�ֵ (Modified value)


uint8_t computer_buffer;
int flag_print=0;

double x_reset=0.0;
double y_reset=0.0;
double angle_reset=0.0;
int printf_flag=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */



#define RX_BUF_SIZE 100

uint8_t rxBuffer;
char rxData[RX_BUF_SIZE];
uint16_t rxIndex = 0;
uint8_t rightBracketCount = 0;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define ACC_UPDATE		0x01
#define GYRO_UPDATE		0x02
#define ANGLE_UPDATE	0x04
#define MAG_UPDATE		0x08
#define READ_UPDATE		0x80
static char s_cDataUpdate = 0;
float fAcc[3], fGyro[3], fAngle[3], fYaw=0;
float aimangle;
int i=0;
	float current_angle=0;
	int fYaw_flag_lr = 0; // 1: ˳ʱ��, 2: ��ʱ��, 3: �Ƕȹ���
typedef struct {
    double x;
    double y;
    double z;
    double yaw;
}angle;
angle angleofdog;

uint8_t RxBuf_USART_1;//����1λ�������ݵĻ�����
uint8_t USART_Flag_1=0;
double stringToNumber_1(char *str);
uint8_t fixedValue_1[20]="";
char *fixedValuestr_1 = (char *)fixedValue_1;
int fixedValueBit_1=0;
double fixedNumber_1=0;//������ת�����double������
int radar_data_type=0;

int sign=0;
extern int places_have_reached;
extern path pathArray[4];

void deal_data()
{
	get_gyro_data();
	q_angle_update(0.4);
	
//	printf("%f %f %f",get_gyro_angle(0),get_gyro_angle(1),get_gyro_angle(2));

}



void rotate_to_correct(void){// �Ƿ�Ӧ����ת�������ǶȽ�Сʱ�ı�kp,kdֵ���Ƕȴ�ʱֱ����ת��kp,kd��Ҫ��С����Ȼ��鴤
    if(fYaw_flag_lr==1){// Ӧ��˳ʱ��ת 
        trotDirectionState=14;
        fYaw_flag_lr=0;
    }else if(fYaw_flag_lr==2){// Ӧ����ʱ��ת,1,2�෴
        trotDirectionState=13;
        fYaw_flag_lr=0;
    }
	else if(fYaw_flag_lr==3)
		{
	    trotDirectionState=0;
        fYaw_flag_lr=0;
	}
      M_statechange();
}


void correct(float correct_angle)
{
	float angle_error = 0.0;

while(1){
	angle_error=gyro_angle-correct_angle;

	if (fabs(angle_error) < 2.0) {
    fYaw_flag_lr = 3;// �Ѷ�׼
} else if (angle_error < 0) {
    fYaw_flag_lr = 2; // Ӧ����ʱ��ת
} else {
    fYaw_flag_lr = 1; // Ӧ��˳ʱ��ת
}
	rotate_to_correct();

if(trotDirectionState==0)break;
}	

}

void parseData(const char* data) {
    const char* p;
    char temp[20];

    p = strstr(data, "(1#");
    if (p) {
        sscanf(p, "(1#%[^)])", temp);
        rx_text.x = atof(temp);
    }

    p = strstr(data, "(2#");
    if (p) {
        sscanf(p, "(2#%[^)])", temp);
        rx_text.y = atof(temp);
    }

//    p = strstr(data, "(3#");
//    if (p) {
//        sscanf(p, "(3#%[^)])", temp);
//        rx_text.angle = atof(temp);
//    }
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
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
  MX_DMA_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_FDCAN3_Init();
  /* USER CODE BEGIN 2 */
	//HAL_UART_Receive_IT(&huart4, uart4_rxbuf, sizeof(uart4_rxbuf));
		__HAL_UART_ENABLE_IT(&huart5,UART_IT_IDLE);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart5, RemoteBuffer_s, 100);
	HAL_UART_Receive_IT(&huart3,(uint8_t *)&RxBuf_USART3,1);
	HAL_UART_Receive_IT(&huart2,(uint8_t*) gyro_buffer,1);
		HAL_UART_Receive_IT(&huart1,(uint8_t*) &RxBuf_USART_1,1);
	FDCAN1_RxFilter_Config();
	FDCAN2_RxFilter_Config();
	FDCAN3_RxFilter_Config();

	HAL_Delay(500);

  float  k_p,k_d,angle0=3.1415926535/2,angle1=3.1415926535/2;
	k_p=30;
	k_d=400;


	//HAL_UART_Transmit(&huart3, (uint8_t*)data, size, 0xFFFF); // 0xFFFF�ǳ�ʱֵ�����Ը�����Ҫ�޸� 
	printTrotMenu();
	

	
	INIT();
	trotDirectionState=0;
	M_statechange();
	IMU_Init();
	//M_statechang	e();
//	time[0]=HAL_GetTick();
//	time[1]=time[0];
//	time[2]=time[0];
//	time[3]=time[0];
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);
//		HAL_Delay(500);
//		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);
//		HAL_Delay(500);
		//go_jiaozhun();
//		if(flag_print==1){
//			printf("ok");
//			flag_print=0;
//		}
		
		//SET_RESET_MODE(3,1);
		//printf("k1:%f,k2:%f",k_try1,k_try2);

//		if(printf_flag==1){
		if (imu_new_data_flag) {
        printf("Yaw: %.2f\r\n", g_imu_data.yaw);
        imu_new_data_flag = 0; // ??????
		}
		printf("gyro_angle: %.2f\r\n", gyro_angle);
		printf("TARGET SPOT:%f",fixedNumber);
				printf("x:%f",rx_text.x);
				printf("y:%f",rx_text.y);
////		
////				//printf("pitch:%f",rx_text.pitch);
//				printf("yaw:%f",rx_text.yaw);
				//printf("roll:%f",rx_text.roll);
				//printf("1:%d",rc.ch1);
				//printf("angle:%f\n",gyro_angle);
//				printf_flag=0;
//		}
			
//				printf("rc.sw1:%d",rc.sw1);
//				
//		printf("k1:%f k2:%f",k_try1,k_try2);
		
//		if(rc.sw1==1)//��¼��ģʽ
//		{
//			if(rc.sw2==1&&sw1_last_state!=1){
//				printf("x:%f\n",rx_text.x);
//				printf("y:%f\n",rx_text.y);
//				printf("yaw:%f\n",rx_text.yaw);
//				sw1_last_state=1;
//			}
//		}
//		else if(rc.sw1==3)//����ģʽ
//		{
//			sw1_last_state=0;
//			if(rc.sw2==1)//����
//			{
//				if(rc.ch4>150 || rc.ch4<-150)
//				{
//					
//					trotDirectionState=59;
//				}
//				else
//			{
//				trotDirectionState=0;
//				}
//		
//			}
//			else if(rc.sw2==3)//����ǰ��
//			{
//			if(rc.ch4>150 || rc.ch4<-150)
//				{
//					
//					trotDirectionState=28;
//				}
//			else if(rc.ch1<-100)//��ת
//			{
//				
//						trotDirectionState=28;

//				}
//				else if(rc.ch1>100)//��ת
//				{
//					
//					trotDirectionState=28;

//				}
//			else
//			{
//				trotDirectionState=0;
//				}
//			}
//			else if(rc.sw2==2)//����
//			{
//			
//				if(rc.ch4>150 || rc.ch4<-150)
//				{
//					
//					trotDirectionState=60 ;
//				}
//				else
//			{
//				trotDirectionState=0;
//				}
//			}
//		
//		
//		}
//		else if(rc.sw1==2)//��Ծģʽ
//		{
//				sw1_last_state=0;
//			if(rc.sw2==1)//����
//			{
////			trotDirectionState=2;
//			}
//			else if(rc.sw2==2)//С��
//			{
//				trotDirectionState=16;
//			}
//		else
//		{
//		trotDirectionState=0;
//		}
//		
//		}

//		
//				if(rc.ch4>150 || rc.ch4<-150)
//				{
//					
//					trotDirectionState=28;
//				}
//				else if(rc.ch1<-100)//��ת
//				{
//				
//									trotDirectionState=28;

//				}
//				else if(rc.ch1>100)//��ת
//				{
//					
//									trotDirectionState=28;

//				}
//				else if(rc.sw1==2)
//				{
//						
//						trotDirectionState=16;
//				}	
//				else 
//				{
//					
//					trotDirectionState=0;
//				}

////		if(rc.sw1==1)//�ϰ������м俪ʼ
////		{
////			if(rc.sw2==1){
////				places_have_reached=0;
////			}
////			else if(rc.sw2==3){
////				places_have_reached=4;
////			}
////			else if(rc.sw2==2){
////				places_have_reached=5;
////			}
////			trotDirectionState=39;
////		}
////		else if(rc.sw1==2)//�ϰ������м俪ʼ
////		{
////			if(rc.sw2==1){
////				places_have_reached=6;
////			}
////			else if(rc.sw2==3){
////				places_have_reached=7;
////			}
////			else if(rc.sw2==2){
////				places_have_reached=8;
////			}
////			trotDirectionState=39;
////		}
		
		

//				printf("4:%d",rc.ch4);
//			printf("ok%d",places_have_reached);
//		printf("���룺%f",pathArray[places_have_reached].distance_square_to_next_point);
		
		
//				pathArray[places_have_reached].pitch_now=acos(-1*(pathArray[places_have_reached].x-rx_text.x)/\
//								sqrt(pow(pathArray[places_have_reached].x-rx_text.x,2)+pow(pathArray[places_have_reached].y-rx_text.y,2)))*\
//								(180.0 / 3.14159265358979323846);//��ǰλ�õ���һ��Ŀ��������빷��ǰ�������ļн�
//	if(pathArray[places_have_reached].y-rx_text.y<0){
//		pathArray[places_have_reached].pitch_now=-pathArray[places_have_reached].pitch_now;
//	}
		
		
//		printf("#Ŀ��ת�ǣ�%f",pathArray[places_have_reached].pitch_now);
//		printf("#ʵ��ת�ǣ�%f\n",rx_text.yaw);
//		printf("Flag:%d",flag_print);
//		if(change_M_T_Flag==1)M_T=__M_T;
//		if(change_Kpp1_Flag==1)__kpp[1]=__Kpp1;
//		if(change_Kpp2_Flag==1)__kpp[2]=__Kpp2;
//		if(change_Kpp3_Flag==1)__kpp[3]=__Kpp3;
//		if(change_Kpp4_Flag==1)__kpp[4]=__Kpp4;
//		if(change_Kdd1_Flag==1)__kdd[1]=__Kdd1;
//		if(change_Kdd2_Flag==1)__kdd[2]=__Kdd2;
//		if(change_Kdd3_Flag==1)__kdd[3]=__Kdd3;
//		if(change_Kdd4_Flag==1)__kdd[4]=__Kdd4;
//		if(change_xpre_Flag==1)xpre=__xpre;
//		if(change_xfinal_Flag==1)xfinal=__xfinal;
//		if(change_zHeight_Flag==1)zHeight=__zHeight;
//		if(change_heightControlNow_Flag==1)heightControlNow=__heightControlNow;
		M_statechange();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
////		time[0]=HAL_GetTick();
////		if(time[0]-time[1]>200)   
////		{
//////					printf("%f",gyro_angle);
////			time[1]=time[0];
////		}
//		
////		Cal_Yobotics_Data(1,0,4,30,0,2,2);
////		HAL_Delay(1000);
////		Cal_Yobotics_Data(-1,0,4,30,0,2,2);
////		HAL_Delay(1000);

		
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV5;
  RCC_OscInitStruct.PLL.PLLN = 68;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/* USER CODE BEGIN 4 */



// ���ڽ����ж�

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//printf("enter_USART");
//HAL_UART_Transmit(&huart3, (uint8_t*)data, size, 0xFFFF); // 0xFFFF�ǳ�ʱֵ�����Ը�����Ҫ�޸�
	//printTrotMenu();
	if(huart->Instance==USART1)
{
	//sign=1;
	//printf("enter_USART1");
//	if(USART_Flag_1==3&&RxBuf_USART_1!=')')
//		{
//			fixedValue_1[fixedValueBit_1]=RxBuf_USART_1;//�����ַ����д���������
//			fixedValueBit_1++;
//		}
		if(USART_Flag_1==1){
			switch (RxBuf_USART_1) {
        case '1':
						radar_data_type=1;
            break;
        case '2':
						radar_data_type=2;
            break;
        case '3':
						radar_data_type=3;
            break;
        case '4':
						radar_data_type=4;
            break;
        case '5':
						radar_data_type=5;
            break;
		}
			USART_Flag_1=2;
		sign=2;
	}
		if(RxBuf_USART_1=='('&&USART_Flag_1==0)
		{
			USART_Flag_1=1;
			sign=1;
		}

		if(USART_Flag_1==3&&RxBuf_USART_1!=')')
		{
			fixedValue_1[fixedValueBit_1]=RxBuf_USART_1;
			fixedValueBit_1++;
			sign=4;
		}
		else if(USART_Flag_1==3&&RxBuf_USART_1==')'){
			fixedNumber_1=stringToNumber(fixedValuestr_1);
			switch (radar_data_type) {
        case 1:
						rx_text.x=fixedNumber_1;
//				printf("x:%f",rx_text.x);
            break;
        case 2:
						rx_text.y=fixedNumber_1;
//						printf("y:%f",rx_text.y);
            break;
        case 3:
						rx_text.pitch=fixedNumber_1;
//						printf("pitch:%f",rx_text.pitch);
            break;
        case 4:
						rx_text.yaw=fixedNumber_1;
						//printf("yaw:%f",rx_text.yaw);
            break;
        case 5:
						rx_text.roll=fixedNumber_1;
						//printf("roll:%f",rx_text.roll);
            break;
		}
			radar_data_type=0;
			USART_Flag_1=0;
			fixedValueBit_1=0;
			memset(fixedValue_1, 0, sizeof(fixedValue_1));
		sign=5;
		}
		if(USART_Flag_1==2){
			if(RxBuf_USART_1=='#'){
				USART_Flag_1=3;
				sign=3;
			}
		}
		
//	switch(computer_buffer){
//		case 'R':
//			trotDirectionState=21;
//		break;
//		case 'B':
//			trotDirectionState=22;
//		break;
//		case 'P':
//			trotDirectionState=2;
//		break;
//	}
	
	
//	 if (rxIndex < RX_BUF_SIZE - 1) {
//        rxData[rxIndex++] = rxBuffer;

//        if (rxBuffer == ')') {
//            rightBracketCount++;
//        }

//        // check if we've received enough complete fields
//        if (rightBracketCount >= 3) {
//            rxData[rxIndex] = '\0';   // �ַ���������

//            parseData(rxData);        // ������յ�������

//					printf("x=%.3f, y=%.3f, angle=%.3f\n",rx_text.x, rx_text.y, rx_text.angle);
//					
//            // ���û�����״̬
//            rxIndex = 0;
//            rightBracketCount = 0;
//            memset(rxData, 0, RX_BUF_SIZE);
//        }
//    } else {
//        // ��������������ݸ�ʽ��������
//        rxIndex = 0;
//        rightBracketCount = 0;
//        memset(rxData, 0, RX_BUF_SIZE);
//    }
//	
//	
//	HAL_UART_Receive_IT(&huart1,(uint8_t*) &rxBuffer,1);
HAL_UART_Receive_IT(&huart1,(uint8_t*) &RxBuf_USART_1,1);
}
//	if(huart->Instance==UART4)
//	{
//	 if((RxBuf[0]==0XF0&&RxBuf[9]==0XF1))
//		{
//				uint16_t measure_angle=(RxBuf[3]<<8)|(RxBuf[4]);
//				int16_t measure_speed=((RxBuf[5]<<4)|((RxBuf[6]&0xF0)>>4))&(0x0FFF);
//				int16_t measure_current=(((RxBuf[6]&0xF)<<8)|(RxBuf[7]))&0x0FFF;
//				Yobotics[RxBuf[1]*2+RxBuf[2]+1].angle=uint_to_float(measure_angle,-12.56f,12.56f,16);
//        Yobotics[RxBuf[1]*2+RxBuf[2]+1].speed_rpm=uint_to_float(measure_speed,-30,30,12);
//        Yobotics[RxBuf[1]*2+RxBuf[2]+1].real_current=uint_to_float(measure_current,0,40,12);
//		}
//		else if(RxBuf[0]==0XAA&&RxBuf[9]==0XAA)
//		{
//			trotDirectionState=50;
////			printf("trotDirectionState=666,����!!!\n");
//		
//		}
//		HAL_UART_Transmit_IT(&huart4,(uint8_t *)RxBuf,8);
//}
if(huart->Instance==USART3)
	{
		//printTrotMenu();


		if(RxBuf_USART3=='!')
		{
			printTrotMenu();	
		}
//		else{
//			printTrotMenu();
//			printTrotMenu();
//		}
		
		if(USART3Flag==1&&RxBuf_USART3!='#')
		{
			fixedValue[fixedValueBit]=RxBuf_USART3;
			fixedValueBit++;
			//printTrotMenu();
		}
		
		if(RxBuf_USART3=='#'&&USART3Flag==1)
		{
			USART3Flag=2;
			//printTrotMenu();
		}
		else if(RxBuf_USART3=='#'&&USART3Flag==0)
		{
			USART3Flag=1;
			//printTrotMenu();
		}


		if(USART3Flag==3){
			switch (RxBuf_USART3) {
				case '0':
					printTrotMenu();
						trotDirectionState=fixedNumber;
						break;
        case '1':
            __Kpp1=fixedNumber;
						change_Kpp1_Flag=1;
						printTrotMenu();
            break;
        case '2':
            __Kpp2=fixedNumber;
						change_Kpp2_Flag=1;
						printTrotMenu();
            break;
        case '3':
            __Kpp3=fixedNumber;
						change_Kpp3_Flag=1;
						printTrotMenu();
            break;
        case '4':
            __Kpp4=fixedNumber;
						change_Kpp4_Flag=1;
						printTrotMenu();
            break;
        case '5':
            __Kdd1=fixedNumber;
						change_Kdd1_Flag=1;
						printTrotMenu();
            break;
        case '6':
            __Kdd2=fixedNumber;
						change_Kdd2_Flag=1;
						printTrotMenu();
            break;
        case '7':
            __Kdd3=fixedNumber;
						change_Kdd3_Flag=1;
						printTrotMenu();
            break;
				case '8':
            __Kdd4=fixedNumber;
						change_Kdd4_Flag=1;
						printTrotMenu();
            break;
        case '9':
            bufu=fixedNumber;
						//printTrotMenu();
            break;
        case 'A':
            __xpre=fixedNumber;
						change_xpre_Flag=1;
						printTrotMenu();
            break;
        case 'B':
            centre=fixedNumber;
						//change_xpre_Flag=1;
            break;
        case 'C':
            __xfinal=fixedNumber;
						change_xfinal_Flag=1;
						printTrotMenu();
            break;
				case 'D':
            __zHeight=fixedNumber;
						change_zHeight_Flag=1;
						printTrotMenu();
            break;
        case 'E':
            __heightControlNow=fixedNumber;
						change_heightControlNow_Flag=1;
						printTrotMenu();
            break;
				case 'F':
						__M_T=fixedNumber;
						change_M_T_Flag=1;
						printTrotMenu();
						break;
				case 'G':
						zHeight_L=fixedNumber;
						break;
				case 'H':
						zHeight_R=fixedNumber;
				//jiaozhun_flag=1;
				printf("gfh");
						break;
				case 'I':
						x_reset=rx_text.x;
						y_reset=rx_text.y;
						angle_reset=rx_text.yaw;
						break;
				case 'J':
						places_have_reached+=fixedNumber;
						break;
				case 'K':
						printf_flag=1;
						break;
				case 'L':                                                                 
						delta_k_p21+=(float)fixedNumber;
						printf("CURRENT_delta_k_p21:%f\n",delta_k_p21);
						M_stop();
						break;
				case 'M':
						delta_p_des12+=(float)fixedNumber;
						printf("CURRENT_delta_p_des12:%f\n",delta_p_des12);
						M_stop();
						break; 
				case 'N':
						delta_p_des22+=(float)fixedNumber;
						printf("CURRENT_delta_p_des22:%f\n",delta_p_des22);
						M_stop();
						break;
				case 'O':
						delta_p_des23+=(float)fixedNumber;
						printf("CURRENT_delta_p_des23:%f\n",delta_p_des23);
						M_stop();
						break;
        default:
            printf("Invalid input!\n");
				
		}
		USART3Flag=0;
		fixedValueBit=0;
		memset(fixedValue, 0, sizeof(fixedValue));

	}
		if(USART3Flag==2)
		{
			USART3Flag=3;
			fixedNumber=stringToNumber(fixedValuestr);
		}
			HAL_UART_Receive_IT(&huart3,(uint8_t *)&RxBuf_USART3,1);

}
//if(flag_print<500){
//	trotDirectionState=23;
//	flag_print++;
//}
//else if(flag_print<1000){
//	trotDirectionState=0;
//	flag_print++;
//}
//else{
//	flag_print=0;
//}
	

if(huart->Instance==USART2)
{
	if(!if_gyro_right)
		{
//			USART_printf("get3");
			if(gyro_buffer[0]==0xBD)
			{
				if_gyro_right=1;
				//USART_printf("get3");
			}
		}
		else
		{
	//		USART_printf("get2");
			if(!(gyro_buffer[0]==0xDB&&gyro_buffer[1]==0x0A))
			{
				//USART_printf("get2");
				if_gyro_right=0; 
				return;
			}
			else
			{
				//USART_printf("wait cal\n");

			//	USART_printf("gggg");
				gyrodata_update();


//				double pul0 = Get_Encoder_Pulse_Count(0);
//				double pul1 = Get_Encoder_Pulse_Count(1);
//				encoderRes.distance[0] += pul0-QEI_pulse[0];
//				encoderRes.distance[1] += pul1-QEI_pulse[1];
//				QEI_pulse[0] = pul0;
//				QEI_pulse[1] = pul1;
//				encoderRes.timeStamp = GenerateTimeStamp();			
				//process_locator_data();
			}

	

}
}


	}

void printTrotMenu()
{
	printf("0: trotDirectionState\n");
	printf("1: __kp[1]\n");
	printf("2: __kd[1]\n");
	printf("3: __kp[2]\n");
	printf("4: __kd[2]\n");
	printf("5: __kp[3]\n");
	printf("6: __kd[3]\n");
	printf("7: __kp[4]\n");
	printf("8: __kd[4]\n");
	printf("9: bufu\n");
	printf("A: xpre\n");
	printf("B: centre\n");
	printf("C: xfinal\n");
	printf("D: zHeight\n");
	printf("E: heightControlNow\n");
	printf("F: M_T\n");
	printf("G: zHeight_L\n");
	printf("H: zHeight_R\n");
	printf("I: Reset_radar_zero\n");
	
}

double stringToNumber(char *str) {
    double number = 0.0;
    int sign = 1;  // Ĭ��Ϊ����
    int decimal_point_seen = 0;  // �Ƿ�������С����
    int decimal_places = 0;  // С�����λ��

    // ����ǰ���ո�
    while (isspace(*str)) {
        str++;
    }

    // ������
    if (*str == '-') {
        sign = -1;
        str++;
    } else if (*str == '+') {
        str++;
    }

    // ������������
    while (isdigit(*str)) {
        number = number * 10 + (*str - '0');
        str++;
    }

    // ����С���㲿��
    if (*str == '.') {
        decimal_point_seen = 1;
        str++;

        while (isdigit(*str)) {
            number = number * 10 + (*str - '0');
            str++;
            decimal_places++;
        }
    }

    // ��ԭС�����λ��
    if (decimal_point_seen) {
        double factor = 1.0;
        for (int i = 0; i < decimal_places; i++) {
            factor *= 10;
        }
        number /= factor;
    }

    // Ӧ�÷���
    number *= sign;

    // ����Ƿ�����Ч�ַ�
    if (*str != '\0' && !isspace(*str)) {
        //printf("��Ч�ַ���\n");
        return 0.0;
    }

    return number;
}



void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)// dji������� 
{
    if(hfdcan==&hfdcan3)
	{
		//printf("hhh");
    //printf("abc");
		
		HAL_FDCAN_GetRxMessage(&hfdcan3,FDCAN_RX_FIFO0,&CAN_RxHeaderStruct,rx_data);

	CAN_RxHeaderStruct.Identifier=0;

	}

}


/**
  * @brief  ���Ͱ�ȫָֹͣ��
  * @param  ��
  * @retval ��
  * @�÷�������Ҫ��  
  * 1. ���ڳ���������ĳ�ʼ��ȫ״̬����ң����������ָͣ��ʱ���á�
  * 2. ͨ������ Cal_Yobotics_Data �����ƺ����������е������������/�����ָ�
  * @����  
  * ȷ�������˳�ʼ����ɺ�HAL_Delay(1000)֮����ִ�У���ֹ�����˶���
  */
void M_stop() 
{

float  k_p,k_d;
		k_p=8;
	k_d=50;
	Cal_Yobotics_Data(0.9,0,k_p,k_d,0,1,1);
	Cal_Yobotics_Data(0.9+delta_p_des12,0,k_p,k_d,0,1,2);//0.9+1.02
	Cal_Yobotics_Data(0.9,0,k_p,k_d,0,2,1);
	Cal_Yobotics_Data(0.9,0,k_p,k_d,0,2,2);
	Cal_Yobotics_Data2(-0.9,0,k_p+delta_k_p21,k_d,0,1,1);//k_p+10
	Cal_Yobotics_Data2(-0.9,0,k_p,k_d,0,1,2);
	Cal_Yobotics_Data2(-0.9+delta_p_des23,0,k_p,k_d,0,2,1);//-0.9-0.02
	Cal_Yobotics_Data2(-0.9,0,k_p,k_d,0,2,2);
	HAL_Delay(500);
	k_p=8;
	k_d=50;
	Cal_Yobotics_Data(1.5,0,k_p,k_d,0,1,1);
	Cal_Yobotics_Data(1.5+delta_p_des12,0,k_p,k_d,0,1,2);//1.5+1.02
	Cal_Yobotics_Data(1.5,0,k_p,k_d,0,2,1);
	Cal_Yobotics_Data(1.5,0,k_p,k_d,0,2,2);
	Cal_Yobotics_Data2(-1.5,0,k_p,k_d,0,1,1);//kp+10
	Cal_Yobotics_Data2(-1.5+delta_p_des22,0,k_p,k_d,0,1,2);//-1.5-0.02
	Cal_Yobotics_Data2(-1.5,0,k_p,k_d,0,2,1);
	Cal_Yobotics_Data2(-1.5,0,k_p,k_d,0,2,2);
	HAL_Delay(1500);
//	k_p=8;
//	k_d=50;
//	Cal_Yobotics_Data(0.9,0,k_p,k_d,0,1,1);
//	Cal_Yobotics_Data(0.9+1.02,0,k_p,k_d,0,1,2);
//	Cal_Yobotics_Data(0.9,0,k_p,k_d,0,2,1);
//	Cal_Yobotics_Data(0.9,0,k_p,k_d,0,2,2);
//	Cal_Yobotics_Data2(-0.9,0,k_p+10,k_d,0,1,1);
//	Cal_Yobotics_Data2(-0.9,0,k_p,k_d,0,1,2);
//	Cal_Yobotics_Data2(-0.9-0.02,0,k_p,k_d,0,2,1);
//	Cal_Yobotics_Data2(-0.9,0,k_p,k_d,0,2,2);
//	HAL_Delay(1500);


}

void INIT()
{
//	    USART_send_data[0]=0xFC;// ͨ�õ�����ݣ�0ms����4����Ϣ�����idΪ11,12��21,22��
//    USART_send_data[1]=1;// ������2���Ȱ�2�ĵ��id��0��11λ���жϣ���1λ�Ǹ��ĸ�CAN�ڷ��ͣ���2λ�Ǹ��ĸ��Ȱ巢����Ϣ��CAN�Ǳ�׼��ʶ�� 
//    USART_send_data[2]=1;
//    USART_send_data[3]=0xFF;//��3��10λ�ǽ��������ΪFOC�ŷ�ģʽ
//    USART_send_data[4]=0xFF;
//    USART_send_data[5]=0xFF;
//    USART_send_data[6]=0xFF;
//    USART_send_data[7]=0xFF;
//    USART_send_data[8]=0xFF;
//    USART_send_data[9]=0xFF;
//    USART_send_data[10]=0xFD;
//    USART_send_data[11]=0xFD;
//    HAL_UART_Transmit_IT(&huart4,USART_send_data,12);//��1�ֽ�����׼���ú󣬲���CPU�����ж����󣬽��з��ͣ��������CPU��������HAL_UART_Transmitʹ����ѯ�������������CPUһֱ�ȴ�
//    HAL_Delay(10);
//    USART_send_data[0]=0xFC;
//    USART_send_data[1]=1;
//    USART_send_data[2]=2;
//    USART_send_data[3]=0xFF;
//    USART_send_data[4]=0xFF;
//    USART_send_data[5]=0xFF;
//    USART_send_data[6]=0xFF;
//    USART_send_data[7]=0xFF;
//    USART_send_data[8]=0xFF;
//    USART_send_data[9]=0xFF;
//    USART_send_data[10]=0xFD;
//    USART_send_data[11]=0xFD;
//    HAL_UART_Transmit_IT(&huart4,USART_send_data,12);
//    HAL_Delay(10);
//    USART_send_data[0]=0xFC;
//    USART_send_data[1]=2;
//    USART_send_data[2]=1;
//    USART_send_data[3]=0xFF;
//    USART_send_data[4]=0xFF;
//    USART_send_data[5]=0xFF;
//    USART_send_data[6]=0xFF;
//    USART_send_data[7]=0xFF;
//    USART_send_data[8]=0xFF;
//    USART_send_data[9]=0xFF;
//    USART_send_data[10]=0xFD;
//    USART_send_data[11]=0xFD;
//    HAL_UART_Transmit_IT(&huart4,USART_send_data,12);
//    HAL_Delay(10);
//    USART_send_data[0]=0xFC;
//    USART_send_data[1]=2;
//    USART_send_data[2]=2;
//    USART_send_data[3]=0xFF;
//    USART_send_data[4]=0xFF;
//    USART_send_data[5]=0xFF;
//    USART_send_data[6]=0xFF;
//    USART_send_data[7]=0xFF;
//    USART_send_data[8]=0xFF;
//    USART_send_data[9]=0xFF;
//    USART_send_data[10]=0xFD;
//    USART_send_data[11]=0xFD;
//    HAL_UART_Transmit_IT(&huart4,USART_send_data,12);
//    HAL_Delay(10);
////    
//    SET_RESET_MODE(1,2);//���õ��FOC�ŷ�ģʽ��ͨ��can1������Ϣ������2�ŵ����?
//    SET_RESET_MODE(1,1);//ͨ��can1������Ϣ������1�ŵ��?
//    //SET_MOTOR_ZERO0(1,1);
//    
//    SET_RESET_MODE(2,2);//ͨ��can2������Ϣ������2�ŵ��?
//    SET_RESET_MODE(2,1);//ͨ��can2������Ϣ������1�ŵ��?
//    
    
//    
//        HAL_Delay(50);
//    USART_send_data[0]=0xFC;//ͨ�����ڼ��?0ms����4����Ϣ����2,3������������Ϊ11,12��21,22�������������䣩
//    USART_send_data[1]=1;//���崮��2�븱�崮��2��������0��11λ�����жϣ���1λ���߸������ĸ�can�ڷ��ͣ���2λ�Ǹ��巢����Ϣ��can��׼��ʶ��
//    USART_send_data[2]=1;
//    USART_send_data[3]=0xFF;//��3��10λ�ǽ��������ΪFOC�ŷ�ģʽ
//    USART_send_data[4]=0xFF;
//    USART_send_data[5]=0xFF;
//    USART_send_data[6]=0xFF;
//    USART_send_data[7]=0xFF;
//    USART_send_data[8]=0xFF;
//    USART_send_data[9]=0xF1;
//    USART_send_data[10]=0x00;
//    USART_send_data[11]=0xFD;
//    HAL_UART_Transmit_IT(&huart4,USART_send_data,12);//��1�ֽ�����׼���ú󣬲���CPU�����ж����󣬽��з��ͣ��������CPU��������HAL_UART_Transmitʹ����ѯ�������������CPUһֱ�ȴ�
//    HAL_Delay(10);
//    USART_send_data[0]=0xFC;
//    USART_send_data[1]=1;
//    USART_send_data[2]=2;
//    USART_send_data[3]=0xFF;
//    USART_send_data[4]=0xFF;
//    USART_send_data[5]=0xFF;
//    USART_send_data[6]=0xFF;
//    USART_send_data[7]=0xFF;
//    USART_send_data[8]=0xFF;
//    USART_send_data[9]=0xF1;
//    USART_send_data[10]=0x00;
//    USART_send_data[11]=0xFD;
//    HAL_UART_Transmit_IT(&huart4,USART_send_data,12);
//    HAL_Delay(10);
//    USART_send_data[0]=0xFC;
//    USART_send_data[1]=2;
//    USART_send_data[2]=1;
//    USART_send_data[3]=0xFF;
//    USART_send_data[4]=0xFF;
//    USART_send_data[5]=0xFF;
//    USART_send_data[6]=0xFF;
//    USART_send_data[7]=0xFF;
//    USART_send_data[8]=0xFF;
//    USART_send_data[9]=0xF1;
//    USART_send_data[10]=0x00;
//    USART_send_data[11]=0xFD;
//    HAL_UART_Transmit_IT(&huart4,USART_send_data,12);
//    HAL_Delay(10);
//    USART_send_data[0]=0xFC;
//    USART_send_data[1]=2;
//    USART_send_data[2]=2;
//    USART_send_data[3]=0xFF;
//    USART_send_data[4]=0xFF;
//    USART_send_data[5]=0xFF;
//    USART_send_data[6]=0xFF;
//    USART_send_data[7]=0xFF;
//    USART_send_data[8]=0xFF;
//    USART_send_data[9]=0xF1;
//    USART_send_data[10]=0x00;
//    USART_send_data[11]=0xFD;
//    HAL_UART_Transmit_IT(&huart4,USART_send_data,12);
//    HAL_Delay(10);
////    
//    SET_MOTOR_ZERO0(1,2);//���õ��FOC�ŷ�ģʽ��ͨ��can1������Ϣ������2�ŵ����?
//    SET_MOTOR_ZERO0(1,1);//ͨ��can1������Ϣ������1�ŵ��?
//    //SET_MOTOR_ZERO0(1,1);
//    
//    SET_MOTOR_ZERO0(2,2);//ͨ��can2������Ϣ������2�ŵ��?
//    SET_MOTOR_ZERO0(2,1);//ͨ��can2������Ϣ������1�ŵ��?




	
	HAL_Delay(10);// ȷ�����㹻��ʱ�䴦���жϺ�DMA���󣬷�������޷���ʱ����
	HAL_UART_Receive_DMA(&huart5, (uint8_t*)RemoteBuffer,18u);// ��DMA��ʽ�Ӵ���5����18�ֽ����ݣ��洢��ң��������
	HAL_UART_Receive_IT(&huart4,RxBuf,10);// ���жϽ��շ�ʽ�Ӵ���4����10�ֽ����ݣ��洢�����ջ��棬���ڴ���Cal_Yobotics_Data2������
	HAL_UART_Receive_IT(&huart1,&RxBuf_UART8,1);
	HAL_UART_Receive_IT(&huart2,&RxBuf_UART8,1);// ȷ�����ĸ�RxBuf_UART8 (Confirm which RxBuf_UART8 it is)
	HAL_UART_Receive_IT(&huart3,(uint8_t*)gyro_buffer,1);
	
	
	
	
	HAL_Delay(1000);
	USART_send_data[0]=0xFC;// ͨ�õ�����ݣ�0ms����4����Ϣ�����idΪ11,12��21,22��
	USART_send_data[1]=1;// ������2���Ȱ�2�ĵ��id��0��11λ���жϣ���1λ�Ǹ��ĸ�CAN�ڷ��ͣ���2λ�Ǹ��ĸ��Ȱ巢����Ϣ��CAN�Ǳ�׼��ʶ��
	USART_send_data[2]=1;
	USART_send_data[3]=0xFF;//��3��10λ�ǽ��������ΪFOC�ŷ�ģʽ
	USART_send_data[4]=0xFF;
	USART_send_data[5]=0xFF;
	USART_send_data[6]=0xFF;
	USART_send_data[7]=0xFF;
	USART_send_data[8]=0xFF;
	USART_send_data[9]=0xFF;
	USART_send_data[10]=0xFC;
	USART_send_data[11]=0xFD;
	HAL_UART_Transmit_IT(&huart4,USART_send_data,12);//��1�ֽ�����׼���ú󣬲���CPU�����ж����󣬽��з��ͣ��������CPU��������HAL_UART_Transmitʹ����ѯ�������������CPUһֱ�ȴ�
	HAL_Delay(10);
	USART_send_data[0]=0xFC;
	USART_send_data[1]=1;
	USART_send_data[2]=2;
	USART_send_data[3]=0xFF;
	USART_send_data[4]=0xFF;
	USART_send_data[5]=0xFF;
	USART_send_data[6]=0xFF;
	USART_send_data[7]=0xFF;
	USART_send_data[8]=0xFF;
	USART_send_data[9]=0xFF;
	USART_send_data[10]=0xFC;
	USART_send_data[11]=0xFD;
	HAL_UART_Transmit_IT(&huart4,USART_send_data,12);
	HAL_Delay(10);
	USART_send_data[0]=0xFC;
	USART_send_data[1]=2;
	USART_send_data[2]=1;
	USART_send_data[3]=0xFF;
	USART_send_data[4]=0xFF;
	USART_send_data[5]=0xFF;
	USART_send_data[6]=0xFF;
	USART_send_data[7]=0xFF;
	USART_send_data[8]=0xFF;
	USART_send_data[9]=0xFF;
	USART_send_data[10]=0xFC;
	USART_send_data[11]=0xFD;
	HAL_UART_Transmit_IT(&huart4,USART_send_data,12);
	HAL_Delay(10);
	USART_send_data[0]=0xFC;
	USART_send_data[1]=2;
	USART_send_data[2]=2;
	USART_send_data[3]=0xFF;
	USART_send_data[4]=0xFF;
	USART_send_data[5]=0xFF;
	USART_send_data[6]=0xFF;
	USART_send_data[7]=0xFF;
	USART_send_data[8]=0xFF;
	USART_send_data[9]=0xFF;
	USART_send_data[10]=0xFC;
	USART_send_data[11]=0xFD;
	HAL_UART_Transmit_IT(&huart4,USART_send_data,12);
	HAL_Delay(10);
	
SET_MOTOR_MODE(1,2);// ���õ��ΪFOC����ģʽ��ͨ��can1������Ϣ����2���Ȱ�ĵ��
		//HAL_Delay(100);
	SET_MOTOR_MODE(1,1);// ͨ��can1������Ϣ����1���Ȱ�ĵ��
		//HAL_Delay(100);
	//SET_MOTOR_ZERO0(1,1);
	
	SET_MOTOR_MODE(2,2);// ͨ��can2������Ϣ����2���Ȱ�ĵ��
		//HAL_Delay(100);
	SET_MOTOR_MODE(2,1);// ͨ��can2������Ϣ����1���Ȱ�ĵ��
		//HAL_Delay(1000);
	
	M_stop();
//	back_to_zero(1,1);
//	back_to_zero(1,2);
//		back_to_zero(2,1);
//	back_to_zero(2,2);
	
//	SET_MOTOR_BACK_TO_ZERO0(1,2);
//	SET_MOTOR_BACK_TO_ZERO0(1,1);
//	SET_MOTOR_BACK_TO_ZERO0(2,1);
//	SET_MOTOR_BACK_TO_ZERO0(2,2);

	//HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_SET);
}








void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(huart == &huart1)
    {
        __HAL_UNLOCK(huart);
        HAL_UART_Receive_IT(&huart3,&RxBuf_USART3,1);
    }
}




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
#ifdef USE_FULL_ASSERT
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
