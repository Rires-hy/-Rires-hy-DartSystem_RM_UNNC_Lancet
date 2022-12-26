/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_key.h"
#include "bsp_led.h"
#include "bsp_can.h"
#include "bsp_pwm.h"
#include "pid.h"
#include "RC.h"


#include "cmsis_os.h"

#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <stdarg.h>
#include "string.h"


//#include "debugio.h"
#include "buffer.h"
#include "ringbuf.h"

#include "CRCcrc_util.h"
#include "protocol_def.h"



// USB COMMUNICATE $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
typedef unsigned char bool_t;
typedef float float32_t;
typedef struct
{
    uint32_t new_time;                  //latest timestamp
    uint32_t last_time;                 //last timestamp
    uint32_t lost_time;                 //the timestamp that this device is lost
    uint32_t work_time;                 //the latest timestamp that this device is still working
    uint16_t set_offline_time : 12;     //larger than this timestamp, device is regarded as lost
    uint16_t set_online_time : 12;      //max allowed timestamp of device to startup
    uint8_t enable : 1;                 //1->device is enabled  0->device is disabled
    uint8_t priority : 4;               //the priority of the device
    uint8_t error_exist : 1;            //1->device is works incorrectly (not means lost)    0->device works fine
    uint8_t is_lost : 1;                //1->device is lost     0->device is on-line
    uint8_t data_is_error : 1;          //1->data has error     0->data is correct

    float32_t frequency;
    bool_t (*data_is_error_fun)(void);
    void (*solve_lost_fun)(void);
    void (*solve_data_error_fun)(void);
} __packed error_t;

static void usb_printf(const char *fmt,...);

static uint8_t usb_buf[256];
//static const char status[2][7] = {"OK", "ERROR!"};
//const error_t *error_list_usb_local;
const error_t *error_list_usb_local;
extern UART_HandleTypeDef huart6;
float AI_R[2];
enum VisionCmdState recv_state = SOF;
enum VisionCmdState recv_state3 = SOF;
struct VisionCommandFrame frame;
struct VisionCommandFrame frame2;
int aaaa=-1;
uint8_t lightBuf;
IO_BufferTypeDef uart6RxBuffer;

RingBuffer cmdBuffer;

IO_BufferTypeDef uart3RxBuffer;

RingBuffer cmdBufferUart3;

/* Definitions for defaultTask */

osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for USART6RxTask */
osThreadId_t USART6RxTaskHandle;
const osThreadAttr_t USART6RxTask_attributes = {
  .name = "USART6RxTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for visionCmdRxTask */
osThreadId_t visionCmdRxTaskHandle;
const osThreadAttr_t visionCmdRxTask_attributes = {
  .name = "visionCmdRxTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for DBGSerialMutex */
osMutexId_t DBGSerialMutexHandle;
const osMutexAttr_t DBGSerialMutex_attributes = {
  .name = "DBGSerialMutex"
};
/* Definitions for usart6RxBinarySemaphore */
osSemaphoreId_t usart6RxBinarySemaphoreHandle;
StaticSemaphore_t usart6RxBinarySemaphoreControlBlock;
const osSemaphoreAttr_t usart6RxBinarySemaphore_attributes = {
  .name = "usart6RxBinarySemaphore",
  .cb_mem = &usart6RxBinarySemaphoreControlBlock,
  .cb_size = sizeof(usart6RxBinarySemaphoreControlBlock),
};


/* Definitions for USART3RxTask */
osThreadId_t USART3RxTaskHandle;
const osThreadAttr_t USART3RxTask_attributes = {
  .name = "USART3RxTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for usart3RxBinarySemaphore */
osSemaphoreId_t usart3RxBinarySemaphoreHandle;
StaticSemaphore_t usart3RxBinarySemaphoreControlBlock;
const osSemaphoreAttr_t usart3RxBinarySemaphore_attributes = {
  .name = "usart3RxBinarySemaphore",
  .cb_mem = &usart3RxBinarySemaphoreControlBlock,
  .cb_size = sizeof(usart3RxBinarySemaphoreControlBlock),
};



// &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&










/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
float unit_linearSpeed = 5;
float unit_angularSpeed = 5; //平移暂时不用w�???????????????????????????????????????????以设�???????????????????????????????????????????0
float carWidth=0.5; //
float carLength=0.7;//
int Electronic_limit[2];
int ready=2;
int encoder_DC=0;
int encoder_DC_filter=0;
int encoder_DC_list[1000];
int encoder_DC_last=0;
int last_state=-1;
int last_state1=-1;
int error;
int error_last;
int count=0;
int error_p=0;
int limit=130;
int optical_switch_adc=0;
int optical_switch_adc_filter=0;
int optical_switch_adc_list[100];
int switc_result=-1;
int limit1=-1;
int limit2=-1;
int limit_state1=0;

int limit1_last=-1;
int limit2_last=-1;
int limit_state2=0;
int mode=0;
int rotate_flag_set=0;
int initial_flag=1;
int stop_flag=1;
int start_shoot=0;
int counter_check=0;

unsigned char buffer[5];
unsigned int rawE;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;
DMA_HandleTypeDef hdma_usart6_rx;
DMA_HandleTypeDef hdma_usart6_tx;

/* Definitions for Chasiss_Task */
osThreadId_t Chasiss_TaskHandle;
const osThreadAttr_t Chasiss_Task_attributes = {
  .name = "Chasiss_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Gimbal_Task */
osThreadId_t Gimbal_TaskHandle;
const osThreadAttr_t Gimbal_Task_attributes = {
  .name = "Gimbal_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Engineering */
osThreadId_t EngineeringHandle;
const osThreadAttr_t Engineering_attributes = {
  .name = "Engineering",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Chasiss_M1_Ctrl */
osThreadId_t Chasiss_M1_CtrlHandle;
const osThreadAttr_t Chasiss_M1_Ctrl_attributes = {
  .name = "Chasiss_M1_Ctrl",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Chasiss_M2_Ctrl */
osThreadId_t Chasiss_M2_CtrlHandle;
const osThreadAttr_t Chasiss_M2_Ctrl_attributes = {
  .name = "Chasiss_M2_Ctrl",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Chasiss_M3_Ctrl */
osThreadId_t Chasiss_M3_CtrlHandle;
const osThreadAttr_t Chasiss_M3_Ctrl_attributes = {
  .name = "Chasiss_M3_Ctrl",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Chasiss_M4_Ctrl */
osThreadId_t Chasiss_M4_CtrlHandle;
const osThreadAttr_t Chasiss_M4_Ctrl_attributes = {
  .name = "Chasiss_M4_Ctrl",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

int16_t led_cnt;
extern pid_struct_t motor_pid[4];

float target_speed;
uint16_t pwm_pulse = 1080;  // default pwm pulse width:1080~1920
int v[10];
extern float set_spd[4];
extern pid_struct_t motor_pid[4];
extern motor_measure_t motor_info[MOTOR_MAX_NUM];
extern motor_measure_t motor_info2[MOTOR_MAX_NUM];
extern RC_ctrl_t rc_ctrl;
extern int IRQ_flag;

//Engineering Motors Control
int DC_target=0;
int DC_now=0;
int DC_laps=0;

int GM6020Lap1 = 0;
int M3508Lap1 = 0;
int GM6020LastAngle1 = 0;
int M3508LastAngle1 = 0;
int GM6020CurrentAngle1 = 0;
int M3508CurrentAngle1 = 0;
int GM6020CurrentTotalAngle1=0;
int M3508CurrentTotalAngle1=0;
int GM6020TargetAngle1 = -696;
int M3508TargetAngle1 = 0;
int count_shoot=0;


int GM6020errorAngle1 = 0;
int GM6020errorSpeed1 = 0;
int GM6020errorTotal1 = 0;
int M3508errorAngle1 = 0;
int M3508errorSpeed1 = 0;
int M3508errorTotal1 = 0;

int GM6020errorangleTotal1=0;
int M3508errorangleTotal1=0;
int GM6020initail1=0;
int M3508initail1=0;

float kPAngle = 80;
float kPSpeed = -20;
float KIangle = 0.00;

float kPAngle1 = 0.010;
float kPSpeed1 = -0.04;
float KIangle1 = 0.0000;

//Chasiss Motors Control
MotorControlInfo m3508CtrlInfo1 =
{ .isSpeedCtrl = 0, .isAngleCtrl = 0, .Lap = 0, .LastAngle = 0, .CurrentAngle = 0, .LastSpeed = 0, .CurrentSpeed = 0, .TargetAngle = 0, .TargetLap = 0, .TargetSpeed = 0, .errorAngle = 0, .errorSpeed = 0, .errorTotal = 0, .kPAngle = -0.1, .kPSpeed = 1 };
MotorControlInfo m3508CtrlInfo2 =
{ .isSpeedCtrl = 0, .isAngleCtrl = 0, .Lap = 0, .LastAngle = 0, .CurrentAngle = 0, .LastSpeed = 0, .CurrentSpeed = 0, .TargetAngle = 0, .TargetLap = 0, .TargetSpeed = 0, .errorAngle = 0, .errorSpeed = 0, .errorTotal = 0, .kPAngle = -0.1, .kPSpeed = 1 };
MotorControlInfo m3508CtrlInfo3 =
{ .isSpeedCtrl = 0, .isAngleCtrl = 0, .Lap = 0, .LastAngle = 0, .CurrentAngle = 0, .LastSpeed = 0, .CurrentSpeed = 0, .TargetAngle = 0, .TargetLap = 0, .TargetSpeed = 0, .errorAngle = 0, .errorSpeed = 0, .errorTotal = 0, .kPAngle = -0.1, .kPSpeed = 1 };
MotorControlInfo m3508CtrlInfo4 =
{ .isSpeedCtrl = 0, .isAngleCtrl = 0, .Lap = 0, .LastAngle = 0, .CurrentAngle = 0, .LastSpeed = 0, .CurrentSpeed = 0, .TargetAngle = 0, .TargetLap = 0, .TargetSpeed = 0, .errorAngle = 0, .errorSpeed = 0, .errorTotal = 0, .kPAngle = -0.1, .kPSpeed = 1 };

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_CAN2_Init(void);
static void MX_TIM5_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM8_Init(void);
static void MX_ADC2_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C3_Init(void);
static void MX_USART6_UART_Init(void);
void Start_Chasiss_Task(void *argument);
void Start_Gimbal_Task(void *argument);
void Engineering_Motor_Control(void *argument);
void Start_Chasiss_M1_Ctrl(void *argument);
void Start_Chasiss_M2_Ctrl(void *argument);
void Start_Chasiss_M3_Ctrl(void *argument);
void Start_Chasiss_M4_Ctrl(void *argument);

/* USER CODE BEGIN PFP */
void PushBarHandler(uint8_t);
void ChasissMotorSpeedControl(MotorControlInfo*, uint8_t);

void StartUSART3RxTask(void *argument);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_CAN1_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_CAN2_Init();
  MX_TIM5_Init();
  MX_ADC1_Init();
  MX_TIM8_Init();
  MX_ADC2_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOH, POWER1_CTRL_Pin|POWER2_CTRL_Pin|POWER3_CTRL_Pin|POWER4_CTRL_Pin, GPIO_PIN_SET); // switch on 24v power
  HAL_GPIO_WritePin(GPIOF,GPIO_PIN_10,1);
  pwm_init();                              // start pwm output
 can_user_init(&hcan1);                   // config can filter, start can
 can2_user_init(&hcan2);


 remote_control_init();
	  HAL_GPIO_WritePin(GPIOF, IN2_Pin|LED_G_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(ENA_GPIO_Port, ENA_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(TNT_GPIO_Port, TNT_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOF,GPIO_PIN_10,1);



//  for(int i=0; i<4; i++)
//  {
//	  pid_init(&motor_pid[i],1.5f,0.1f,	0.0f,20000,4500);
//  }
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  usart3RxBinarySemaphoreHandle = osSemaphoreNew(1, 1, &usart3RxBinarySemaphore_attributes);
  RB_Init(&cmdBufferUart3, 512);
  IO_Buffer_Init(&uart3RxBuffer, 512, usart3RxBinarySemaphoreHandle);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Chasiss_Task */
  Chasiss_TaskHandle = osThreadNew(Start_Chasiss_Task, NULL, &Chasiss_Task_attributes);

  /* creation of Gimbal_Task */
  Gimbal_TaskHandle = osThreadNew(Start_Gimbal_Task, NULL, &Gimbal_Task_attributes);

  /* creation of Engineering */
  EngineeringHandle = osThreadNew(Engineering_Motor_Control, NULL, &Engineering_attributes);

  /* creation of Chasiss_M1_Ctrl */
  Chasiss_M1_CtrlHandle = osThreadNew(Start_Chasiss_M1_Ctrl, NULL, &Chasiss_M1_Ctrl_attributes);

  /* creation of Chasiss_M2_Ctrl */
  Chasiss_M2_CtrlHandle = osThreadNew(Start_Chasiss_M2_Ctrl, NULL, &Chasiss_M2_Ctrl_attributes);

  /* creation of Chasiss_M3_Ctrl */
  Chasiss_M3_CtrlHandle = osThreadNew(Start_Chasiss_M3_Ctrl, NULL, &Chasiss_M3_Ctrl_attributes);

  /* creation of Chasiss_M4_Ctrl */
  Chasiss_M4_CtrlHandle = osThreadNew(Start_Chasiss_M4_Ctrl, NULL, &Chasiss_M4_Ctrl_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  USART3RxTaskHandle = osThreadNew(StartUSART3RxTask, NULL, &USART3RxTask_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	 // set_motor_voltage(0, 0, 0, 0);
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_9TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 3;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_9TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = ENABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 167;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 19;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 83;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 19999;
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
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 100000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart3.Init.BaudRate = 115200;
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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, switch_limit_Pin|LED_R_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8|LED_G_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TNT_GPIO_Port, TNT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ENA_GPIO_Port, ENA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SWITCH_Pin */
  GPIO_InitStruct.Pin = SWITCH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SWITCH_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : switch_limit_Pin */
  GPIO_InitStruct.Pin = switch_limit_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(switch_limit_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IN1_Pin */
  GPIO_InitStruct.Pin = IN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(IN1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PH2 PH3 PH4 PH5 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pin : IN2_Pin */
  GPIO_InitStruct.Pin = IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(IN2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PF8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : limit1_Pin */
  GPIO_InitStruct.Pin = limit1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(limit1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : TNT_Pin */
  GPIO_InitStruct.Pin = TNT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TNT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ENA_Pin */
  GPIO_InitStruct.Pin = ENA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ENA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_R_Pin */
  GPIO_InitStruct.Pin = LED_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_R_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : limit2_Pin */
  GPIO_InitStruct.Pin = limit2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(limit2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_G_Pin */
  GPIO_InitStruct.Pin = LED_G_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_G_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void PushBarHandler(uint8_t isUp)
{
	if (isUp == 0)
	{
		HAL_GPIO_WritePin(GPIOF, IN2_Pin | LED_G_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOF, IN2_Pin | LED_G_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
	}
}

void ChasissMotorSpeedControl(MotorControlInfo *m3508CtrlInfo, uint8_t motorID)
{


}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_Start_Chasiss_Task */
/**
  * @brief  Function implementing the Chasiss_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Start_Chasiss_Task */
void Start_Chasiss_Task(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
	/* Infinite loop */
	for (;;)
	{


		osDelay(1);
	}

  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Start_Gimbal_Task */
/**
* @brief Function implementing the Gimbal_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Gimbal_Task */
void Start_Gimbal_Task(void *argument)
{
  /* USER CODE BEGIN Start_Gimbal_Task */
	int angle;
	int angle2;
	int flag1;
	int flag2;
//	while(!start_shoot);




	/* Infinite loop */
	for (;;)
	{



		 int yaw_angle = 1080 + ( motor_info[0].rotor_angle ) * 840 / 8191;
			  		int pitch_angle = 1080 + ( motor_info[2].rotor_angle ) * 840 / 8191;
			  		int	L_X = 0;
			  		int	L_Y = 0;
			  		int	R_X = 0;
			  		int	R_Y = 0;
			  		int max_rpm = 3000;
			  		int CH1 = 0;
			  		int CH2 = 1;
			  		int CH3 = 2;
			  		int CH4 = 3;
			  		int GM6020;





			  		v[6]=GM6020*3000;

			  		L_X = rc_ctrl.rc.ch[2] * 30000 / 660;
			  		L_Y = rc_ctrl.rc.ch[3] * 30000 / 660;
			  		R_X = rc_ctrl.rc.ch[0] * 30000 / 660;
			  		R_Y = rc_ctrl.rc.ch[1] * 30000 / 660;

			  		set_spd[CH4] = 0;	  // CH1 = Thr + Rud + Ail
			  		set_spd[CH2] = L_X;		// CH2 = -Thr + Rud + Ail
			  		set_spd[CH3] = 0;		// CH3 = Thr + Rud - Ail
			  		set_spd[CH1] = L_Y;		// CH4 = -Thr + Rud - Ail

			  		if((rc_ctrl.rc.ch[2]<=5)&&(rc_ctrl.rc.ch[3]<=5)&&(rc_ctrl.rc.ch[2]>=-5)&&(rc_ctrl.rc.ch[3]>=-11))
			  		{
			  			if (flag1 == 0)
			  			{
			  				angle = motor_info[0].rotor_angle;
			  				flag1 = 1;
			  					}
			  			if (flag2 == 0)
			  			{
			  				angle2 = motor_info[0].rotor_angle;
			  				flag2 = 1;
			  					}

			  			set_motor_voltage_CAN1(-100*((motor_info[0].rotor_angle)-angle),(int16_t) GM6020errorTotal1,v[6],GM6020*300);
			  		}

			  		else
			  		{
			  		for (int i=0;i<4;i++)
			  		{
			  			pid_calc(&motor_pid[i],set_spd[i] ,motor_info[i].rotor_speed);
			  		}

			  		//set_motor_voltage(motor_pid[0].output, motor_pid[1].output,motor_pid[2].output,motor_pid[3].output);//ÓÐPID
			  		//in order to use PID, the above locking should also be changed. For testing, here just use code without PID:
			  		//without PID:
			  			set_motor_voltage_CAN1(L_Y,(int16_t) GM6020errorTotal1,v[6],GM6020*300);//ÎÞPID

			  			v[4]=L_Y;
						v[5]=L_X;
			  			//resit would shake until you control it. This problem could be solved by initializing the origin place of motor.
			  			flag1=0;
			  			flag2=0;
			  		}
					HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);


		osDelay(1);
	}
  /* USER CODE END Start_Gimbal_Task */
}

/* USER CODE BEGIN Header_Engineering_Motor_Control */
/**
* @brief Function implementing the Engineering thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Engineering_Motor_Control */
void Engineering_Motor_Control(void *argument)
{
  /* USER CODE BEGIN Engineering_Motor_Control */
//	ready=HAL_GPIO_ReadPin(Limit_switch_GPIO_Port, Limit_switch_Pin);//movement switch
	int current3=1000;
//	while(rc_ctrl.rc.s[1]!=2);
//	while(!start_shoot);

	while(0)
	{
		current3=current3+1.0*(3000+motor_info2[2].rotor_speed);
		if(current3>10000){current3=10000;}
		if(current3<-10000){current3=-10000;}
//		ready=HAL_GPIO_ReadPin(Limit_switch_GPIO_Port, Limit_switch_Pin);
		set_motor_voltage_CAN2(-1000, -current3, -current3, -1000);
	}
	set_motor_voltage_CAN2(0, 0, 0, 0);
	ready=3;



	GM6020initail1=-4792;
	M3508initail1=4000;
	M3508TargetAngle1=M3508initail1;
	int slowlypass=0;
	int slowlypass1=0;



	/* Infinite loop */
	for (;;)
	{
		if(rotate_flag_set)
			{
				if(initial_flag){GM6020TargetAngle1=GM6020initail1;initial_flag=0;}
				else if(count_shoot<5)
				{GM6020TargetAngle1-=8192;}

				rotate_flag_set=0;
			}

		if((rc_ctrl.rc.s[1]==1)&&(last_state!=1)){
			GM6020TargetAngle1-=8192;
		}
		last_state=rc_ctrl.rc.s[1];

		if((rc_ctrl.rc.s[0]==2)&&(last_state1!=2)){
			DC_target+=4096*60;
		}
		if((rc_ctrl.rc.s[0]==3)&&(last_state1!=3)){
			DC_target-=4096*60;
		}
		last_state1=rc_ctrl.rc.s[0];


		GM6020LastAngle1=GM6020CurrentAngle1;
		GM6020CurrentAngle1=motor_info[1].rotor_angle;
		GM6020CurrentTotalAngle1=GM6020Lap1*8192+motor_info[1].rotor_angle;

		M3508LastAngle1=M3508CurrentAngle1;
		M3508CurrentAngle1=motor_info2[2].rotor_angle;
		M3508CurrentTotalAngle1=M3508Lap1*8192+motor_info2[2].rotor_angle;
//DC
		if (encoder_DC_last-encoder_DC>2048)
		{
			DC_laps++;
		}
		else if (encoder_DC-encoder_DC_last>2048)
		{
			DC_laps--;
		}
		DC_now=DC_laps*4096+encoder_DC;
		encoder_DC_last=encoder_DC;
//lap count
		//6020
//		if ((4096 < GM6020LastAngle1) && (GM6020LastAngle1 <= 8192) && (0 <= GM6020CurrentAngle1) && (GM6020CurrentAngle1 < 4096)&&(motor_info[1].rotor_speed>0))
//		{
//			GM6020Lap1++;
//		}
//		else if ((4096 < GM6020CurrentAngle1) && (GM6020CurrentAngle1 <= 8192) && (0 <= GM6020LastAngle1) && (GM6020LastAngle1 < 4096)&&(motor_info[1].rotor_speed<0))
//		{
//			GM6020Lap1--;
//		}
		if (GM6020LastAngle1-GM6020CurrentAngle1>4096)
		{
			GM6020Lap1++;
		}
		else if (GM6020CurrentAngle1-GM6020LastAngle1>4096)
		{
			GM6020Lap1--;
		}

		//3508
		if ((4096 < M3508LastAngle1) && (M3508LastAngle1 <= 8192) && (0 <= M3508CurrentAngle1) && (M3508CurrentAngle1 < 4096)&&(motor_info2[2].rotor_speed>0))
		{
			M3508Lap1++;
			slowlypass1=0;
		}
		else if ((4096 < M3508CurrentAngle1) && (M3508CurrentAngle1 <= 8192) && (0 <= M3508LastAngle1) && (M3508LastAngle1 < 4096)&&(motor_info2[2].rotor_speed<0))
		{
			M3508Lap1--;
			slowlypass1=0;
		}
		else if ((M3508CurrentAngle1==M3508LastAngle1)&&(M3508CurrentAngle1==8192))
		{
			if(slowlypass1==-1){M3508Lap1--;}
			slowlypass1=1;
		}
		else if ((M3508CurrentAngle1==M3508LastAngle1)&&(M3508CurrentAngle1==0))
		{
			if(slowlypass==1){M3508Lap1++;}
			slowlypass1=-1;
		}

// pid error calculation
		//6020
		GM6020errorAngle1 = GM6020TargetAngle1 - GM6020CurrentTotalAngle1;
		GM6020errorangleTotal1=GM6020errorangleTotal1+GM6020errorAngle1;

		GM6020errorSpeed1 = 0 - motor_info[1].rotor_speed;

		GM6020errorTotal1 = kPAngle * GM6020errorAngle1 + kPSpeed * GM6020errorSpeed1+KIangle*GM6020errorangleTotal1;


//DC

		error=DC_target - DC_now;
//		if((error<4000)&&(error>-4000))
//		{
//			limit=7080;
//		}
//		else if((error<8000)&&(error>-8000))
//		{
//			limit=100;
//		}
//		else{limit=200;}

		error_p=error*0.01;
		int error_d=0*(error-error_last);
		error_last=error;

		int error_total_pid=error_p+error_d;

		if(error_total_pid>170){error_total_pid=170;}
		else if(error_total_pid<-130){error_total_pid=-130;}
//		if(error_total_pid>0){TIM5->CCR3=1530-(error_total_pid);}
//		else{TIM5->CCR3=1470-(error_total_pid);}
//		if(rc_ctrl.rc.s[0]!=1)
//		{
////			if((limit1==1)&&(limit2==1)){TIM5->CCR3=1500-(error_total_pid);}
//
//		}


//3508
		M3508errorAngle1 = M3508TargetAngle1 - M3508CurrentTotalAngle1;
		M3508errorangleTotal1=M3508errorangleTotal1+M3508errorAngle1;

		M3508errorSpeed1 = 0 - motor_info2[2].rotor_speed;

		M3508errorTotal1 = kPAngle1 * M3508errorAngle1 + kPSpeed1 * M3508errorSpeed1+KIangle1*M3508errorangleTotal1;
//limit
		//6020
		if (GM6020errorTotal1 > 10000)
		{
			GM6020errorTotal1 = 10000;
		}
		else if (GM6020errorTotal1 < -10000)
		{
			GM6020errorTotal1 = -10000;
		}
		//3508
		if (M3508errorTotal1 > 10000)
		{
			M3508errorTotal1 = 10000;
		}
		else if (M3508errorTotal1 < -10000)
		{
			M3508errorTotal1 = -10000;
		}

		set_motor_voltage_CAN1(0,(int16_t) GM6020errorTotal1, 0, 0);

//		osDelay(1);

	}
  /* USER CODE END Engineering_Motor_Control */
}

/* USER CODE BEGIN Header_Start_Chasiss_M1_Ctrl */
/**
 * @brief Function implementing the Chasiss_M1_Ctrl thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Start_Chasiss_M1_Ctrl */
void Start_Chasiss_M1_Ctrl(void *argument)
{
  /* USER CODE BEGIN Start_Chasiss_M1_Ctrl */
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
	TIM5->CCR3=1500;
//  dc motor
	/* Infinite loop */

	int flag3=0;
	int flag4=0;
//	while(!start_shoot);



		for(;;)
		{
			limit1=HAL_GPIO_ReadPin(limit1_GPIO_Port, limit1_Pin);
			limit2=HAL_GPIO_ReadPin(limit2_GPIO_Port, limit2_Pin);

			if((!limit1)&&(limit1_last)&&(limit_state1==0))
			{
				limit_state1=1;
			}
			if((limit1)&&(!limit1_last)&&(limit_state1==1))
			{
				limit_state1=2;
			}
			if((!limit1)&&(limit1_last)&&(limit_state1==2))
			{
				limit_state1=3;
			}
			if((limit1)&&(!limit1_last)&&(limit_state1==3))
			{
				limit_state1=0;
			}

			if((!limit2)&&(limit2_last)&&(limit_state2==0))
			{
				limit_state2=1;
			}
			if((limit2)&&(!limit2_last)&&(limit_state2==1))
			{
				limit_state2=2;
			}
			if((!limit2)&&(limit2_last)&&(limit_state2==2))
			{
				limit_state2=3;
			}
			if((limit2)&&(!limit2_last)&&(limit_state2==3))
			{
				limit_state2=0;
			}

			limit1_last=limit1;
			limit2_last=limit2;



//			if(rc_ctrl.rc.s[0]==1)
//			{
//				//UP
//				if (rc_ctrl.rc.ch[1]>=50)
//				{
//
//					if(!limit_state1){TIM5->CCR3=1200;}	//switch stop
//					else if(TIM5->CCR3<1500)
//					{
//						TIM5->CCR3=1500;
//					}
//
//				}
//				//DOWN
//				else if (rc_ctrl.rc.ch[1]<=-50)
//				{
//					if(!limit_state2){TIM5->CCR3=1750;}	//switch stop
//					else if(TIM5->CCR3>1500)
//					{
//						TIM5->CCR3=1500;
//					}
//
//				}
//				else{
//				TIM5->CCR3=1500;
//				}
//				if((!flag3)&&(limit_state2==2))
//				{
//
//						TIM5->CCR3=1200;
//						osDelay(100);
//						TIM5->CCR3=1500;
//						flag3=1;
//
//				}
//				if((!flag4)&&(limit_state1==2))
//				{
//
//						TIM5->CCR3=1800;
//						osDelay(0);
//						TIM5->CCR3=1500;
//						flag4=1;
//				}
//				if(limit_state2==0){flag3=0;}
//				if(limit_state1==0){flag4=0;}

			osDelay(1);

	}
  /* USER CODE END Start_Chasiss_M1_Ctrl */
}

/* USER CODE BEGIN Header_Start_Chasiss_M2_Ctrl */
/**
 * @brief Function implementing the Chasiss_M2_Ctrl thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Start_Chasiss_M2_Ctrl */
void Start_Chasiss_M2_Ctrl(void *argument)
{
  /* USER CODE BEGIN Start_Chasiss_M2_Ctrl */



	int light_old;
	/* Infinite loop */
	for (;;)
	{
		HAL_UART_Receive(&huart3, &(lightBuf), 1, 10);


		if (	(lightBuf==1)	&&	(light_old==0)	)
		{
			osDelay(6000);
			start_shoot=1;
			HAL_GPIO_WritePin(GPIOF, LED_G_Pin, 0);
		}

		light_old=lightBuf;


		osDelay(1);
	}
  /* USER CODE END Start_Chasiss_M2_Ctrl */
}

/* USER CODE BEGIN Header_Start_Chasiss_M3_Ctrl */
/**
 * @brief Function implementing the Chasiss_M3_Ctrl thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Start_Chasiss_M3_Ctrl */
void Start_Chasiss_M3_Ctrl(void *argument)
{
  /* USER CODE BEGIN Start_Chasiss_M3_Ctrl */
	 //MX_USB_DEVICE_Init();
		DBGSerialMutexHandle = osMutexNew(&DBGSerialMutex_attributes);
		usart6RxBinarySemaphoreHandle = osSemaphoreNew(1, 1, &usart6RxBinarySemaphore_attributes);
		RB_Init(&cmdBuffer, 512);
		IO_Buffer_Init(&uart6RxBuffer, 512, usart6RxBinarySemaphoreHandle);

//	    error_list_usb_local = get_error_list_point();
	     uint32_t res, res16;

	     size_t recv_state_pos = 0;

	     __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
	     HAL_UART_Receive_DMA(&huart6, uart6RxBuffer.buffer, uart6RxBuffer.size);
	     //initiate USB
	    // MX_USB_DEVICE_Init();
	     aaaa=0;
		  HAL_GPIO_WritePin(GPIOF, IN2_Pin|LED_G_Pin, 1);

		  int con_old=0;
	/* Infinite loop */
//	while(rc_ctrl.rc.s[1]!=2);
	for (;;)
	{
		aaaa=1;
		IO_Buffer_Acquire(&uart6RxBuffer);
aaaa=2;
//		    	    dbgprintf("\nbufferlen=%d\n",uart6RxBuffer.len);
//		    	    dbgbuf(uart6RxBuffer.buffer, uart6RxBuffer.len);
		    	    size_t i = 0;
		    	    while (i < uart6RxBuffer.len)
		    	    {
		    	      uint8_t ch = uart6RxBuffer.buffer[i];
		    	      switch (recv_state)
		    	      {
		    	      case SOF:
		    	    	if (ch == 0xA5)
		    	    	{
		    	    	  frame.SOF = ch;
		    	    	  recv_state_pos = 0;
		    	    	  recv_state = DLEN;
		    	    	}
		    	    	break;
		    	      case DLEN:
		    	    	*(((uint8_t *)&(frame.data_length))+recv_state_pos) = ch;
		    	    	recv_state_pos++;
		    	    	if (recv_state_pos >=2)
		    	    	{
		    	    	  recv_state_pos = 0;
		    	    	  recv_state = SEQ;
		    	    	}
		    	    	break;
		    	      case SEQ:
		    	    	frame.sequence = ch;
		    	    	recv_state_pos = 0;
		    	    	recv_state = CRC8;
		    	    	break;
		    	      case CRC8:
		    	    	frame.CRC8 = ch;
		    	    	crc8compute((void *)&frame.SOF, 5, &res);
		    	    	recv_state_pos = 0;
		    	    	if (res == 0)
		    	    	{
		    	          recv_state = CMD;
		    	    	}
		    	    	else
		    	    	{
		    	          recv_state = SOF;
		    	    	}
		    	      	break;
		    	      case CMD:
		    	      	*(((uint8_t *)&(frame.cmd_id))+recv_state_pos) = ch;
		    	      	recv_state_pos++;
		    	      	if (recv_state_pos >=2)
		    	      	{
		    	      	  recv_state_pos = 0;
		    	      	  recv_state = DATA;
		    	      	}
		    	    	break;
		    	      case DATA:
		    	        *(((uint8_t *)&(frame.data))+recv_state_pos) = ch;
		    	        recv_state_pos++;
		    	        if (recv_state_pos >= frame.data_length)
		    	        {
		    	//          recv_state_pos = 0;
		    	          recv_state = CRC16;
		    	        }
		    	    	break;
		    	      case CRC16:
		    	//        *(((uint8_t *)&(frame.CRC16))+recv_state_pos) = ch;
		    	    	*(((uint8_t *)&(frame.data))+recv_state_pos) = ch;
		    	        recv_state_pos++;
		    	        if (recv_state_pos >= frame.data_length + 2)
		    	        {
		    	          crc16compute((void *)&frame, (uint64_t)9+(uint64_t)frame.data_length, &res16);
		    	          recv_state_pos = 0;
		    	          if (res16 == 0)
		    	          {
		    	            // TODO: Process function goes here
		    	        	//dbgprintf("\ndx = %.1f, dy = %.1f\n", frame.data.demoCmd.dx, frame.data.demoCmd.dy);
		    	        	// Assign value to X and Y after receiving.


//		    	        	 if (	(frame.cmd_id == 0x0105) && (frame.data.dartRemainingTime.dartRemainingTime>0))
//		    	        			    	      	{
//		    	        			    	      		start_shoot=1;
//		    	        			    	      	  HAL_GPIO_WritePin(GPIOF, LED_G_Pin, 0);
//
//		    	        			    	      	}
//		    	        	 if (	(frame.cmd_id == 0x0104) && (frame.data.dartRemainingTime.dartRemainingTime>0))
//		    	        			    	      	{
//		    	        			    	      		start_shoot=1;
//		    	        			    	      	  HAL_GPIO_WritePin(GPIOF, LED_G_Pin, 0);
//
//		    	        			    	      	}

		    	          	 if (	(frame.cmd_id == 0x020A)  && 	(frame.data.demoCmd.dart_launch_opening_status==0) 	)

		    	        		    	        	{
		    	        		    	        			    	      		start_shoot=8;
		    	        		    	        			    	      	  HAL_GPIO_WritePin(GPIOF, LED_G_Pin, 0);

		    	        		    	        }
		    	          	 if (frame.cmd_id == 0x020A)
								{
		    	          		 if(	frame.data.demoCmd.dart_attack_target	!=	con_old	){
		    	          		 counter_check++;

		    	          		 }
		    	          		 if (counter_check>=4){
												start_shoot=8;
											  HAL_GPIO_WritePin(GPIOF, LED_G_Pin, 0);
		    	          		 }

		    	          		 con_old=frame.data.demoCmd.dart_attack_target;

								}


//		    	        	 if(  (rc_ctrl.rc.s[1]==2)&&(rc_ctrl.rc.s[0]==2)	)
//		    	        	   			{
//		    	        	   		start_shoot=1;
//					    	      	  HAL_GPIO_WritePin(GPIOF, LED_G_Pin, 0);
//		    	        	   			}
//		    	        	   	else {
//		    	        	   		start_shoot=1;
//		    	        	   	}

		    	          }
		    	          recv_state = SOF;
		    	        }
		    	    	break;
		    	      }

		    	      i++;
		    	    }

		    	    HAL_UART_Receive_DMA(&huart6, uart6RxBuffer.buffer, uart6RxBuffer.size);
		//    	HAL_UART_Transmit(&huart6,TxData,10,0xffff);
		        osDelay(1);
	}
  /* USER CODE END Start_Chasiss_M3_Ctrl */
}

/* USER CODE BEGIN Header_Start_Chasiss_M4_Ctrl */
/**
 * @brief Function implementing the Chasiss_M4_Ctrl thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Start_Chasiss_M4_Ctrl */
void Start_Chasiss_M4_Ctrl(void *argument)
{
  /* USER CODE BEGIN Start_Chasiss_M4_Ctrl */
	/* Infinite loop */
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
	TIM5->CCR3=1500;
	osDelay(2000);
	while(start_shoot!=8);
	mode=-1;
	mode=MODE_3_RELEASE;

	limit1=HAL_GPIO_ReadPin(limit1_GPIO_Port, limit1_Pin);
	limit2=HAL_GPIO_ReadPin(limit2_GPIO_Port, limit2_Pin);

	for (;;)
	{
		if(count_shoot<5)
		{



		if((mode==MODE_1_READY_FEED)&&(!limit1))
		{	TIM5->CCR3=1800;
			osDelay(40);
			TIM5->CCR3=1500;
			osDelay(500);
			mode=MODE_2_FEED_DART;
			rotate_flag_set=1;
		}

		if((mode==MODE_2_FEED_DART)&&((GM6020TargetAngle1-GM6020CurrentTotalAngle1)<200)&&((GM6020TargetAngle1-GM6020CurrentTotalAngle1)>-200))
		{
			count_shoot++;
			TIM5->CCR3=1500;
			osDelay(500);
			mode=MODE_3_RELEASE;
		}

		if((mode==MODE_3_RELEASE)&&(!limit2))
		{
			TIM5->CCR3=1200;
			osDelay(100);
			TIM5->CCR3=1500;
			osDelay(500);
			mode=MODE_1_READY_FEED;

		}

	    switch(mode){
	        case MODE_1_READY_FEED:
	        	TIM5->CCR3=1330;//UP
	        	break;

	        case MODE_2_FEED_DART:
	        	TIM5->CCR3=1500;
	        	while(start_shoot!=8);


	        	break;

	        case MODE_3_RELEASE:
	        	TIM5->CCR3=1650;//DOWN
	        	break;

	        default:printf("error\n"); break;
	    }
		}
		if(count_shoot==5)
		{
			TIM5->CCR3=1500;
			if(stop_flag)
			{
				TIM5->CCR3=1600;
				osDelay(2000);
				TIM5->CCR3=1500;
				GM6020TargetAngle1-=4096;
				stop_flag=0;
			}
		}



//		if(mode==MODE_1_READY_FEED)
//		{
//			TIM5->CCR3=1200;
//		}



	}
  /* USER CODE END Start_Chasiss_M4_Ctrl */
}


/* USER CODE BEGIN Header_StartUSART3RxTask */
/**
* @brief Function implementing the USART3RxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUSART3RxTask */
__weak void StartUSART3RxTask(void *argument)
{
  /* USER CODE BEGIN StartUSART3RxTask */
  struct VisionCommandFrame frame;
  uint32_t res, res16;
  enum VisionCmdState recv_state = SOF;
  size_t recv_state_pos = 0;
  int light_old=9999;

  __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
  HAL_UART_Receive_DMA(&huart3, uart3RxBuffer.buffer, uart3RxBuffer.size);
  /* Infinite loop */
  for (;;)
  {
    IO_Buffer_Acquire(&uart3RxBuffer);

//    dbgprintf("\nbufferlen=%d\n",uart3RxBuffer.len);
//    dbgbuf(uart3RxBuffer.buffer, uart3RxBuffer.len);
    size_t i = 0;
    while (i < uart3RxBuffer.len)
    {
      uint8_t ch = uart3RxBuffer.buffer[i];
      switch (recv_state)
      {
      case SOF:
      if (ch == 0x05)
      {
        frame.SOF = ch;
        recv_state_pos = 0;
        recv_state = DLEN;
      }
      break;
      case DLEN:
      *(((uint8_t *)&(frame.data_length))+recv_state_pos) = ch;
      recv_state_pos++;
      if (recv_state_pos >=2)
      {
        recv_state_pos = 0;
        recv_state = SEQ;
      }
      break;
      case SEQ:
      frame.sequence = ch;
      recv_state_pos = 0;
      recv_state = CRC8;
      break;
      case CRC8:
      frame.CRC8 = ch;
      crc8compute((void *)&frame.SOF, 5, &res);
      recv_state_pos = 0;
      if (res == 0)
      {
          recv_state = CMD;
      }
      else
      {
          recv_state = SOF;
      }
        break;
      case CMD:
        *(((uint8_t *)&(frame.cmd_id))+recv_state_pos) = ch;
        recv_state_pos++;
        if (recv_state_pos >=2)
        {
          recv_state_pos = 0;
          recv_state = DATA;
        }
      break;
      case DATA:
        *(((uint8_t *)&(frame.data))+recv_state_pos) = ch;
        recv_state_pos++;
        if (recv_state_pos >= frame.data_length)
        {
//          recv_state_pos = 0;
          recv_state = CRC16;
        }
      break;
      case CRC16:
//        *(((uint8_t *)&(frame.CRC16))+recv_state_pos) = ch;
      *(((uint8_t *)&(frame.data))+recv_state_pos) = ch;
        recv_state_pos++;
        if (recv_state_pos >= frame.data_length + 2)
        {
          crc16compute((void *)&frame, (uint64_t)9+(uint64_t)frame.data_length, &res16);
          recv_state_pos = 0;
          if (res16 == 0)
          {
            // TODO: Process function goes here

        	  if (	(frame.data.demoCmd2.dx>=5000) && (light_old<=5000)	)
        	  {
        		  osDelay(2000);
      			start_shoot=8;
      			HAL_GPIO_WritePin(GPIOF, LED_G_Pin, 0);
        	  }

        	  light_old=frame.data.demoCmd2.dx;

          }
          recv_state = SOF;
        }
      break;
      }

      i++;
    }

    HAL_UART_Receive_DMA(&huart3, uart3RxBuffer.buffer, uart3RxBuffer.size);
    osDelay(1);
  }
  /* USER CODE END StartUSART3RxTask */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
