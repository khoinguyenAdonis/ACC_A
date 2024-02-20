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
//PC13		Gap_Increase
//PC0		Gap_Decrease
//PE2		Engine_Status x
//PE3		Mode x
//PE4		Set
//PE5		Speed_Increase
//PE6		Speed_Decrease
//TIM1 SR04  PE9 Echo
//			 PE8 TRIG
//TIM2 ENCODER
//
//TIM3 PWM PA6
//		   PA7
//TIM4 0.12S INTERUPT
//TIM5 0.1S  INTERUPT
//ADC1 IN0 	 PA0
//I2C2 SCL
//	   SDA
//PB0 KY012
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "i2c-lcd.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include "math.h"
#include "HC_SR04.h"
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	uint16_t Dset;
	uint16_t Vset;
	bool firstSet;
}controlValue;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define speedSetMax 250
#define speedSetMin 50
#define distanceSetMax 30
#define distanceSetMin 10
#define speedStep 10
#define distanceStep 5
#define DEBOUNCE_DELAY 1000
#define Kp 4.0
#define Ki 0.00011
#define Kd 1.55
#define max_crr_value  999
#define wheelPerimeter 20.420335
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
enum Cmnd
{
	UserControl =1 ,
	CruiseControl,
	AdaptiveCruiseControl
};
enum Cmnd ucmd = UserControl;
controlValue control = {10, 50, false};
const uint8_t ESTOP_DISTANCE = 5, SENSING_ZONE =50 , DECREASE_ZONE =5;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */
// BIẾN TOÀN CỤC C�? �?ỊNH.
uint32_t last_debounce_time = 0,value=0;
uint16_t engineStat = 0, speedSet=50,distanceSet=10,gas_val = 0;
uint16_t RPM,RTDistance,crr_value;
int16_t deltaRPM;
float  pid_value = 0;



// TẠM THỜI
int a, b=0,c;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
void debounceButton(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin, uint32_t *last_debounce_time);
void LCD_StartingScreen();
void display();
uint32_t convert_pid_to_crr(float pid_value);
float calculate_pid(float setpoint, float actual_value);
void cruiseControl( controlValue *control,uint16_t RPM );
void adaptiveCruiseControl(uint16_t RPM, uint16_t deltaRPM, uint16_t RTDistance, controlValue *control);
void userControl(uint8_t gas_val,uint16_t RPM);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t MAP(uint32_t au32_IN, uint32_t au32_INmin, uint32_t au32_INmax, uint32_t au32_OUTmin, uint32_t au32_OUTmax)
{
	return ((((au32_IN - au32_INmin)*(au32_OUTmax - au32_OUTmin))/(au32_INmax - au32_INmin)) + au32_OUTmin);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	gas_val = MAP(value, 0, 4095, 0, 100); //gas_val = MAP(value, 1010, 3191, 0, 100);
}

uint32_t convert_pid_to_crr(float pid_value)
{

    float max_pid_value = 1500.0;
    // Chuyển đổi giá trị PID thành giá trị CRR bằng cách sử dụng tỷ lệ tương ứng
    crr_value = (uint32_t)( fabs(pid_value / max_pid_value * max_crr_value));

    // Giới hạn giá trị CRR trong phạm vi cho phép
    if (crr_value > max_crr_value) {
      return  crr_value = max_crr_value;
    } else if (crr_value < 0) {
        return crr_value = 0;
    }else   return crr_value;

}

float calculate_pid(float setpoint, float actual_value)
{
  static float current_error = 0, previous_error = 0;

  static float integral = 0, derivative = 0;
  // Tính lỗi hiện tại
  current_error = setpoint - actual_value;


  // Tính phần tích phân của lỗi
  integral += current_error;

  // Tính phần đạo hàm của lỗi
  derivative = current_error - previous_error;

  // Tính giá trị đi�?u khiển PID
  pid_value = Kp * current_error + Ki * integral + Kd * derivative;

  // Cập nhật lỗi trước đó
  previous_error = current_error;

  return pid_value;
}

void cruiseControl(controlValue *control,uint16_t RPM)
{
	if (control->firstSet == false)
	{
		control->Vset=RPM;
		speedSet = RPM;
		control->firstSet = true;
	}
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, convert_pid_to_crr(calculate_pid(control->Vset,RPM)));
}

void adaptiveCruiseControl(uint16_t RPM, uint16_t deltaRPM, uint16_t RTDistance, controlValue *control)
{
	uint16_t v_Object;
	if (control->firstSet == false)
	{
		control->Vset=RPM;
		speedSet = RPM;
		control->firstSet = true;
	}
	if (RTDistance <= SENSING_ZONE)// step into the calculate zone
	{
        // Turn on calculate object speed
		HAL_TIM_Base_Start_IT(&htim4);
		if ( deltaRPM < 0 && fabs(deltaRPM) > RPM)
		{
			v_Object = 0;
		}else{
			v_Object = RPM + deltaRPM;
		}

		if (RTDistance >= ESTOP_DISTANCE + control->Dset + DECREASE_ZONE)               // run like CCS
		{
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, convert_pid_to_crr(calculate_pid(control->Vset,RPM)));
		}
		else if (RTDistance <= ESTOP_DISTANCE) //  E-stop if to close
		{
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,0);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2,0);
			 ucmd = UserControl;
		}
		else if (RTDistance < ESTOP_DISTANCE + control->Dset + DECREASE_ZONE && RTDistance > ESTOP_DISTANCE ) //
		{ // sync speed with object
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, convert_pid_to_crr(calculate_pid(v_Object,RPM)));

		}
	}
	else
	{
		HAL_TIM_Base_Stop_IT(&htim4);       // turn of calculate speed run like CCS
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, convert_pid_to_crr(calculate_pid(control->Vset,RPM)));
	}
}

void debounceButton(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin, uint32_t *last_debounce_time)
{
	uint32_t current_time = HAL_GetTick();
	if (current_time - *last_debounce_time > DEBOUNCE_DELAY)
	{
		*last_debounce_time = current_time;
		while (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == 0)
		{
			if ( HAL_GetTick()- *last_debounce_time >= DEBOUNCE_DELAY)
			{
				break;
			}
		}
	}
}

void LCD_StartingScreen()
{

	char welcome_str[] = "Welcome"; //7
	char system_str[] = "System turning on"; //17
	char loading_str[] = "Loading"; //7
	lcd_put_cur(0, 6);
	for (int i = 0; i < 7; i++)
	{
		lcd_send_data(welcome_str[i]);
		HAL_Delay(100);
	}
	HAL_Delay(1000);
	lcd_put_cur(1, 1);
	for (int i = 0; i < 17; i++)
	{
		lcd_send_data(system_str[i]);
		HAL_Delay(100);
	}
	HAL_Delay(500);
	for (int i = 0; i < 2; i++)
	{
		lcd_clear();
		for (int j = 1; j <= 3; j++)
		{
			lcd_put_cur(1, 4);
			if (i == 0)
			{
				for (int k = 0; k < 7; k++)
				{
                lcd_send_data(loading_str[k]);
                HAL_Delay(90);
				}
			}
			else
			{
				lcd_send_string("Loading");
			}
			for (int k = 0; k < j; k++)
			{
				lcd_send_data('.');
				HAL_Delay(70);
			}
			HAL_Delay(400);
		}
	}
	lcd_clear();
}

void display()
{
		char Buffer[4][20];
		if (ucmd == CruiseControl)
		{
			sprintf(Buffer[0],"%s%s%s","Mode :","CC   |EN:",(engineStat>0)?"ON":"OFF");
		}
		else if (ucmd == AdaptiveCruiseControl)
		{
			sprintf(Buffer[0],"%s%s%s","Mode :","ACC  |EN:",(engineStat>0)?"ON":"OFF");
		}
		else
		{
			sprintf(Buffer[0],"%s%s%s","Mode :","UC   |EN:",(engineStat>0)?"ON":"OFF");
		}
		sprintf(Buffer[1],"%s%d%s","Speed:",RPM," RPM");
		sprintf(Buffer[2],"%s%d%s%d","Vset :",speedSet," |Dis:", RTDistance);
		sprintf(Buffer[3],"%s%d%s","Dset :",distanceSet, "cm");
		lcd_clear();
		for(int i = 0 ; i < 4; i ++)
		{
			lcd_put_cur(i,0);
			lcd_send_string(Buffer[i]);
		}

}



void userControl(uint8_t gas_val,uint16_t RPM)
{
	uint16_t ccr_val;// xong b�? dòng này vào h
	ccr_val = (gas_val * 999) / 100;
	TIM3 ->CCR1 = ccr_val;
	if (RPM> 50)
		HAL_GPIO_WritePin(GPIOE, LEDSTAT_Pin,0);
    else
    	HAL_GPIO_WritePin(GPIOE, LEDSTAT_Pin,1);
      
}

void Cmd_Handle(void)
{
	switch (ucmd) {
		case 1:
			userControl(gas_val,RPM);
			break;
		case 2:
//      if (RPM> 50)
//      {
			cruiseControl(&control,RPM);
//      }else
//      {
//        ucmd = UserControl;
//      }
			break;
		case 3:
//          if (RPM> 50)
//      {
			adaptiveCruiseControl(RPM, deltaRPM, RTDistance,&control);
//      }else
//      {
//        ucmd = UserControl;
//      }
			break;

	}
}

void warningDis(uint16_t RTDistance)
{
	static uint32_t lastime = 0;
	if (RTDistance <= ESTOP_DISTANCE)
	{
		if (HAL_GetTick() - lastime >= RTDistance *10)
		{
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
			lastime = HAL_GetTick();
		}
	}else if (RTDistance > ESTOP_DISTANCE) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
		lastime = HAL_GetTick();
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch (GPIO_Pin)
	{
		case CruiseControl_Pin :
				debounceButton(GPIOC,GPIO_Pin, &last_debounce_time);
				ucmd = (ucmd != 2 ) ? 2 :  1;
				break;
		case AdaptiveCruiseControl_Pin:
				debounceButton(GPIOC,GPIO_Pin, &last_debounce_time);
				ucmd = (ucmd != 3 ) ? 3 :  1;
				break;
		case Engine_Status_Pin:
				debounceButton(GPIOC,GPIO_Pin, &last_debounce_time);
				engineStat = engineStat ^ 1;
				HAL_GPIO_TogglePin(GPIOA, ENGINE_CONTROL_Pin);
				break;
		case Set_Pin:
				debounceButton(GPIOE,GPIO_Pin, &last_debounce_time);
				control.Dset = distanceSet ;
				control.Vset = speedSet ;
				if(ucmd ==2)
				{
					cruiseControl(&control, RPM);
				}else if(ucmd == 3)
				{
					adaptiveCruiseControl(RPM, deltaRPM, RTDistance, &control);
				}
				break;
		case Speed_Increase_Pin:
				debounceButton(GPIOE,GPIO_Pin, &last_debounce_time);
				speedSet += speedStep;
				if(speedSet >= speedSetMax)
					speedSet = speedSetMax;
				break;
		case Speed_Decrease_Pin:
				debounceButton(GPIOE,GPIO_Pin, &last_debounce_time);
				if(speedSet > speedSetMin)
					speedSet -= speedStep;
				break;
		case Gap_Increase_Pin:
				debounceButton(GPIOC,GPIO_Pin, &last_debounce_time);
				distanceSet += distanceStep;
				if(distanceSet >= distanceSetMax)
					distanceSet = distanceSetMax;
				break;
		case Gap_Decrease_Pin:
				debounceButton(GPIOC,GPIO_Pin, &last_debounce_time);
				if(distanceSet > distanceSetMin)
					distanceSet -= distanceStep;
				break;
		case StopControl_Pin:
				debounceButton(GPIOE,GPIO_Pin, &last_debounce_time);
				ucmd = UserControl;
				break;
		case StopControl2_Pin:
				debounceButton(GPIOE,GPIO_Pin, &last_debounce_time);
				ucmd = UserControl;
				break;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	if		(htim == &htim5)
	{
		static   uint32_t Last_Encoder = 0 ;
		volatile uint32_t Realtime_Encoder = __HAL_TIM_GET_COUNTER(&htim2);

		int32_t Delta_Encoder = Realtime_Encoder - Last_Encoder;
		if 		(Delta_Encoder < -(0xffffffff / 2)) // Xử lý trư�?ng hợp quay ngược
			{
				Delta_Encoder += 0xffffffff;
			}
		else if (Delta_Encoder > (0xffffffff / 2)) // Xử lý trư�?ng hợp quay xuôi
			{
				Delta_Encoder -= 0xffffffff;
			}
		Last_Encoder = Realtime_Encoder;
		RPM = fabs(Delta_Encoder / 1320.0 * 600);
		RTDistance = HCSR04_GetDis();
		a= engineStat;
		b=ucmd;
		c=RPM;
	}
	else if ( htim == &htim4)
	{	float deltaSpeed;
		static uint16_t lasRTDistance = 0;
		int16_t deltaDistance;
		deltaDistance = RTDistance - lasRTDistance;
		deltaSpeed    = deltaDistance / 0.12; // speed() cm/s every 0.12s has interupt
		deltaRPM =((deltaSpeed * 60)/wheelPerimeter);
		lasRTDistance = RTDistance;
	}
	else if (htim == &htim6)
	{
		display();
	}
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
  MX_ADC1_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_Base_Start_IT(&htim5);
  HAL_TIM_Base_Stop_IT(&htim6);

  HAL_TIM_Base_Stop_IT(&htim4);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_ADC_Start_DMA(&hadc1,&value,1);
  lcd_init();
  LCD_StartingScreen();
  HAL_TIM_Base_Start_IT(&htim6);

//  lcd_put_cur(0, 0);
//  lcd_send_string("11");
//  lcd_put_cur(1, 0);
//  lcd_send_string("22");
//  lcd_put_cur(2, 0);
//  lcd_send_string("33");
//  lcd_put_cur(3, 0);
//  lcd_send_string("44");
   //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 0);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {

	  //HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, 1);
	  Cmd_Handle();
	  warningDis(RTDistance);
	 // adaptiveCruiseControl(RPM, deltaRPM, RTDistance, &control);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c2.Init.ClockSpeed = 100000;
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
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 100-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
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
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
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

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
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
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 50000-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 120-1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 50000-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 100-1;
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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 50000-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 450-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ENGINE_CONTROL_GPIO_Port, ENGINE_CONTROL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(KY_SIGNAL_GPIO_Port, KY_SIGNAL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LEDSTAT_Pin|TRIGGER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Set_Pin Speed_Increase_Pin Speed_Decrease_Pin StopControl_Pin
                           StopControl2_Pin */
  GPIO_InitStruct.Pin = Set_Pin|Speed_Increase_Pin|Speed_Decrease_Pin|StopControl_Pin
                          |StopControl2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : Gap_Increase_Pin Gap_Decrease_Pin AdaptiveCruiseControl_Pin Engine_Status_Pin
                           CruiseControl_Pin */
  GPIO_InitStruct.Pin = Gap_Increase_Pin|Gap_Decrease_Pin|AdaptiveCruiseControl_Pin|Engine_Status_Pin
                          |CruiseControl_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : ENGINE_CONTROL_Pin */
  GPIO_InitStruct.Pin = ENGINE_CONTROL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ENGINE_CONTROL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : KY_SIGNAL_Pin */
  GPIO_InitStruct.Pin = KY_SIGNAL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(KY_SIGNAL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LEDSTAT_Pin */
  GPIO_InitStruct.Pin = LEDSTAT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LEDSTAT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TRIGGER_Pin */
  GPIO_InitStruct.Pin = TRIGGER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TRIGGER_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
