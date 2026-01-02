/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "hc165.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum{
	on_hold  	= 0x00,
	released	= 0x01,
	pressed		= 0x02,
	on_none		= 0x03,
}key_state_t;

typedef enum{
	sensor_idle_state = 1,
	sensor_active_state = 0,
}sensor_state_t;
typedef void (*key_cb_t)(void);
typedef struct{
	uint8_t					newSample;
	uint8_t 				pressedFlag;
	key_state_t			state;
	key_cb_t				callBack;
}key_t;
typedef void (*sensor_cb_t)(void);
typedef struct{
	uint8_t					newSample;
	sensor_state_t	state;
	uint8_t 				pressedFlag;
	sensor_cb_t			callBack;
}sensor_t;
typedef struct{
	uint8_t keyReadFlag			: 1;
	uint8_t sensorReadFlag	: 1;
	uint8_t hc165ReadFlag		: 1;
}Flag_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SWITCH_CLOSE1				0
#define SWITCH_OPEN1				1
#define SWITCH_LASER1				2
#define BUTTON_OP						3
#define BUTTON_SET					4
#define SWITCH_LASER2				5
#define SWITCH_CLOSE2				6
#define SWITCH_OPEN2				7
#define HC165_CONVERSION_TO_BIT(BYTE,BIT)    ((BYTE) >> (BIT) & 0x01)
#define OP_KEY							0
#define SET_KEY							1
#define CLOSE_MICRO_SWITCH1	0	
#define OPEN_MICRO_SWITCH1	1
#define LASER_OUTPUT				2
#define CLOSE_MICRO_SWITCH2	3
#define OPEN_MICRO_SWITCH2	4
#define SET									1
#define RESET								0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t key_timer = 0;
Flag_t flags = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* sensor read prototype*/
void key_init(key_t* key ,uint16_t len);
void update_keyState(key_t* key ,uint16_t len);
void key_check_pressed(key_t* key ,uint16_t len);

void sensor_init(sensor_t* sensor);
void update_sensorState(sensor_t* sensor);
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
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	
	key_t keys[2];
	key_init(keys,2);
	
	sensor_t sensors[5];
	
	HAL_GPIO_WritePin(NSLEEP_GPIO_Port,NSLEEP_Pin,GPIO_PIN_SET);
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if(flags.hc165ReadFlag){
			flags.hc165ReadFlag = RESET;
			keys[OP_KEY].newSample = HC165_CONVERSION_TO_BIT(hc165_out,BUTTON_OP);
			keys[SET_KEY].newSample = HC165_CONVERSION_TO_BIT(hc165_out,BUTTON_SET);
			hc165_stateMachine();
		}
		if(flags.sensorReadFlag){
			flags.sensorReadFlag = RESET;
			
		}
		// قبلش بايد نمونه کليد رو آپديت کنم از hc165
		if(flags.keyReadFlag){ // update key state
			flags.keyReadFlag = RESET;
			// آپديت وضعيت هر کليد
			update_keyState(keys,2);
			// بررسي فشرده شدن  هر کليد
			key_check_pressed(keys,2);
			if(keys[OP_KEY].pressedFlag){
				keys[OP_KEY].pressedFlag = RESET;
				//keys[OP_KEY].callBack();
				HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
				//HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
				if(HAL_GPIO_ReadPin(LED1_GPIO_Port,LED1_Pin)){
					HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
				}
				else{
					HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
				}
			}
			if(keys[SET_KEY].pressedFlag){
				keys[SET_KEY].pressedFlag = RESET;
				//keys[SET_KEY].callBack();
				HAL_GPIO_TogglePin(LED1_GPIO_Port,LED2_Pin);
			}
		}
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void key_init(key_t* key ,uint16_t len){
	while(len-- > 0){
		key->newSample 		= SET;
		key->state				= on_none;
		key->pressedFlag 	= RESET;
		key++;
	}
}
void update_keyState(key_t* key ,uint16_t len){
	while(len-- > 0){
		key->state = ((key->state << 1) | key->newSample) & 0x03;
		key++;
	}
}
void key_check_pressed(key_t* key ,uint16_t len){
	while(len-- > 0){
		if(key->state == pressed){
			key->pressedFlag = SET;
		}
		key++;
	}
}
void sensor_init(sensor_t* sensor){
	sensor->newSample = sensor_idle_state;
	sensor->state			= sensor_idle_state;
}
void update_sensorState(sensor_t* sensor){
	sensor->state = sensor->newSample;
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim){
	if(htim->Instance == TIM2){
		flags.hc165ReadFlag = SET;
		key_timer++;
		if(key_timer >= 30){
			key_timer 						= RESET;
			flags.keyReadFlag 		= SET;
		}
	}
	if(htim->Instance == TIM4){
		flags.sensorReadFlag 	= SET;
		key_timer++;
		if(key_timer >= 3){
			key_timer 						= RESET;
			flags.keyReadFlag 		= SET;
		}
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
