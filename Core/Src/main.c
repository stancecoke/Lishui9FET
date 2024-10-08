/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
//#include <stdio.h>
#include "config.h"
#include "print.h"
#include "display_kingmeter.h"

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
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
//#ifdef __GNUC__
//#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
//#else
//#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
//#endif
//  PUTCHAR_PROTOTYPE
//  {
//    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
//    return ch;
//  }

#define ST_BOOTLOADER_STARTADRESSE 0x1FFFEC00// 0x1FFFC800 //0x1FFFEC00 0x1fffedbc
void TimerCommutationEvent_Callback(void);
void Get_Direction(void);
uint32_t PI_control (PI_control_t* PI_c);
int32_t map (int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);
int16_t internal_tics_to_speedx100 (uint32_t tics);
int16_t external_tics_to_speedx100 (uint32_t tics);
//void JumpToBootloader (void);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM1_Init(void);
void kingmeter_update(void);
void print_debug_info(void);
/* USER CODE BEGIN PFP */
volatile uint16_t adcData[10]; //Buffer for ADC1 Input
uint8_t ui8_adc_regular_flag =0;
uint8_t ui8_direction_flag =0;
uint16_t ui16_halltics =64000;
uint16_t ui16_timing_counter =0;
static uint16_t ui16_battery_current_offset =0;
int16_t i16_battery_current =0;
int16_t i16_battery_current_raw =0;
int16_t i16_battery_current_cumulated =0;
uint8_t ui8_cal_battery_current = CAL_I;
uint16_t ui16_battery_current_max_raw = 0;
int32_t i32_motor_current_raw =0;
uint16_t ui16_throttle =0;
uint16_t ui16_throttle_offset = THROTTLE_OFFSET;
uint8_t ui8_hallstate =0;
uint8_t ui8_hallstate_old =0;
uint8_t uwStep=0;
uint8_t slow_loop_counter=0;
uint8_t i=0;
uint8_t ui8_PAS_flag=0;
uint8_t ui8_UART_flag=0;
uint16_t ui16_PAS_counter=PAS_TIMEOUT+1;
uint16_t ui16_PAS=64000;
uint16_t ui16_setpoint_temp=0;
uint8_t ui8_SPEED_flag=0;
uint8_t ui8_Push_Assist_flag=0;
uint8_t ui8_assist_level=127;
uint16_t ui16_SPEED_counter=64000;
uint16_t ui16_SPEED=0;
uint16_t ui16_SPEEDx100_kph=0;
uint16_t uint16_mapped_PAS=0;
uint16_t uint16_mapped_Throttle=0;
//uint8_t hall_sequence[7]={4,5,1,3,2,6};
uint8_t hall_sequence[2][7]={
		{0,3,5,4,1,2,6},
		{0,4,6,5,2,3,1}};


uint16_t ui16_dutycycle = 1;
uint16_t ui16_throttle_cumulated = 0;
uint16_t ui16_torque_cumulated = 0;
char tx_buffer[100];

int16_t battery_percent_fromcapacity = 50; 			//Calculation of used watthours not implemented yet
int16_t power;
int32_t temp1;
int32_t temp2;
int32_t temp3;

enum {Powerlevels, NCTE, ERider, Spare};

PI_control_t PI_battery_current;
KINGMETER_t KM;

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

    // LED ON
	//JumpToBootloader ();
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
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_ADC_Init();
  MX_TIM1_Init();
  KingMeter_Init (&KM);
  /* USER CODE BEGIN 2 */

  PI_battery_current.gain_i=I_FACTOR;
  PI_battery_current.gain_p=P_FACTOR;
  PI_battery_current.setpoint = 0;
  PI_battery_current.limit_output =PERIOD;
  PI_battery_current.max_step=250;
  PI_battery_current.shift=9;
  PI_battery_current.limit_i=PERIOD;

  ui16_battery_current_max_raw = BATTERY_CURRENT_MAX/ui8_cal_battery_current;


  HAL_ADC_Start_DMA(&hadc,(uint32_t*)adcData, 10);
  if(HAL_TIM_Base_Start_IT(&htim1) != HAL_OK)
    {
      /* Counter Enable Error */
      Error_Handler();
    }
  /*##-5- Start signals generation ###########################################*/
  /*--------------------------------------------------------------------------*/
  /* Start channel 1 */
  if(HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }
  /* Start channel 1N */
  if(HAL_TIMEx_OCN_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }
  /*--------------------------------------------------------------------------*/


  /*--------------------------------------------------------------------------*/
  /* Start channel 2 */
  if(HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_2) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }
  /* Start channel 2N */
  if(HAL_TIMEx_OCN_Start(&htim1, TIM_CHANNEL_2) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }
  /*--------------------------------------------------------------------------*/


  /*--------------------------------------------------------------------------*/
  /* Start channel 3 */
  if(HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_3) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }
  /* Start channel 3N */
  if(HAL_TIMEx_OCN_Start(&htim1, TIM_CHANNEL_3) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }

  LL_TIM_DisableAllOutputs(TIM1);
  LL_TIM_OC_SetCompareCH1(TIM1, 1);
  LL_TIM_OC_SetCompareCH2(TIM1, 1);
  LL_TIM_OC_SetCompareCH3(TIM1, 1);

  //HAL_Delay(1000); //wait for stable conditions


  while(!adcData[8]){}

  while(adcData[8]!=ui16_battery_current_offset){
	ui16_battery_current_offset=0;
	for(i=0;i<56;i++){
	  	while(!ui8_adc_regular_flag){}
	  	if(i>23)	ui16_battery_current_offset+=adcData[8];
	  //printf_("%d, \r\n ",  adcData[8]);
	  	ui8_adc_regular_flag=0;
	  }
	ui16_battery_current_offset=(ui16_battery_current_offset>>5);
  }
  i16_battery_current_cumulated=ui16_battery_current_offset<<2;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(ui8_UART_flag){
		  KingMeter_Service(&KM);
		  ui8_UART_flag=0;
	  }
	  if(ui8_SPEED_flag&&ui16_SPEED_counter>100){
		  ui16_SPEED=ui16_SPEED_counter;
		  ui16_SPEED_counter=0;
		  ui8_SPEED_flag=0;
		  //calculate speed
		  ui16_SPEEDx100_kph = external_tics_to_speedx100 (ui16_SPEED);
	  }

	  if(ui8_PAS_flag&&ui16_PAS_counter>100){
		  ui16_PAS=ui16_PAS_counter;
		  ui16_PAS_counter=0;
		  ui16_torque_cumulated -= ui16_torque_cumulated>>5;

		  if(KM.Settings.RideMode==NCTE){
			  if(adcData[2]<NCTE_OFFSET)ui16_torque_cumulated += (NCTE_OFFSET-adcData[2]);
		  	  }
		  if(KM.Settings.RideMode==ERider){
			  if(adcData[2]>ERIDER_OFFSET)ui16_torque_cumulated += (adcData[2]-ERIDER_OFFSET);
		  	  }

		  ui8_PAS_flag=0;

	  }

	  if(ui8_adc_regular_flag){
		  ui16_throttle_cumulated-=ui16_throttle_cumulated>>4;
		  if(adcData[4]>ui16_throttle_offset) ui16_throttle_cumulated+=adcData[4];
		  ui16_throttle = ui16_throttle_cumulated>>4;

		  i16_battery_current_cumulated-=i16_battery_current_cumulated>>2;
		  i16_battery_current_cumulated+=adcData[8];
		  i16_battery_current_raw=((i16_battery_current_cumulated>>2)-ui16_battery_current_offset);
		  i16_battery_current=i16_battery_current_raw*ui8_cal_battery_current;

		  PI_battery_current.recent_value=i16_battery_current_raw;
		  if(ui16_halltics>5000) PI_battery_current.max_step=50;
		  else  PI_battery_current.max_step=500;
		  if(LL_TIM_IsEnabledAllOutputs(TIM1)) ui16_dutycycle = PI_control(&PI_battery_current);
		  //ui16_dutycycle = PI_control(&PI_battery_current);
		  // Motorcurrent = Battery current / DutyCycle
		  i32_motor_current_raw =i16_battery_current_raw*PERIOD/ui16_dutycycle;
		  // limit Motorcurrent
		  ui16_dutycycle = map(i32_motor_current_raw, 1200, 1350, ui16_dutycycle,1);
		  LL_TIM_OC_SetCompareCH1(TIM1, ui16_dutycycle);
		  LL_TIM_OC_SetCompareCH2(TIM1, ui16_dutycycle);
		  LL_TIM_OC_SetCompareCH3(TIM1, ui16_dutycycle);
		  ui8_adc_regular_flag=0;

	  	  }

	  if(ui16_timing_counter>16){ //run control @1kHz
		  slow_loop_counter++;
		  if(slow_loop_counter>20){//debug printout @50Hz
			  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
			// if(!HAL_GPIO_ReadPin(BKL_Brake_GPIO_Port, BKL_Brake_Pin))JumpToBootloader();

			 #ifdef DEBUG
			  print_debug_info();
#endif
			  slow_loop_counter=0;
		  	  }
		  // reset speed if, no speed pulse occurs
		  if(ui16_SPEED_counter>63000){
			  ui16_SPEEDx100_kph=0;
			  ui16_SPEED=64000;
		  }
		  //check break lever switch
		  if(!HAL_GPIO_ReadPin(BKL_Brake_GPIO_Port, BKL_Brake_Pin))ui16_setpoint_temp=0;
		  //calculate wanted battery current from sensor data
		  else{
		  if(KM.Settings.RideMode==NCTE || KM.Settings.RideMode==ERider){
			//calculate current target form torque, cadence and assist level
		    uint16_mapped_PAS = (TS_COEF*(int16_t)(ui8_assist_level)* (ui16_torque_cumulated>>5)/ui16_PAS)>>8; //>>5 aus Mittelung Ã¼ber eine Kurbelumdrehung, >>8 aus KM5S-Protokoll Assistlevel 0..255

			//limit currest target to max value
			if(uint16_mapped_PAS>ui16_battery_current_max_raw) uint16_mapped_PAS=ui16_battery_current_max_raw;
			//set target to zero, if pedals are not turning
			if(ui16_PAS_counter>PAS_TIMEOUT){
				uint16_mapped_PAS=0;
				if(ui16_torque_cumulated>0)ui16_torque_cumulated--; //ramp down cumulated torque value
			}
		  }
			if(KM.Settings.RideMode == Powerlevels){

			  uint16_mapped_PAS = map(ui16_PAS, KM.Settings.Ramp_End, PAS_TIMEOUT, ((ui16_battery_current_max_raw*(int32_t)(ui8_assist_level)))>>8, 0); // level in range 0...255
			  if(ui16_PAS_counter>PAS_TIMEOUT)uint16_mapped_PAS=0;

			}
		  //check for Throttle override
		  uint16_mapped_Throttle = map(ui16_throttle, ui16_throttle_offset , THROTTLE_MAX, 0, ui16_battery_current_max_raw);
		  if(uint16_mapped_PAS>uint16_mapped_Throttle)ui16_setpoint_temp=(int32_t)uint16_mapped_PAS;
		  else ui16_setpoint_temp=uint16_mapped_Throttle;
		  //push assist with constant power
		  if(ui8_Push_Assist_flag)ui16_setpoint_temp=150*KM.Settings.DoublePushAssist;
		  }//end else break

		  // limit speed if legal Flag is set
		  if(KM.Settings.LegalFlag){
			if(ui16_PAS_counter<PAS_TIMEOUT){//ramp down setpoint at speed limit
				PI_battery_current.setpoint=map(ui16_SPEEDx100_kph, KM.Rx.SPEEDMAX_Limit,KM.Rx.SPEEDMAX_Limit+200,ui16_setpoint_temp,0);
				}
			else{ //limit to 6km/h if pedals are not turning
				PI_battery_current.setpoint=map(ui16_SPEEDx100_kph, 500,700,ui16_setpoint_temp,0);
				}
		  	  }
		  else PI_battery_current.setpoint=ui16_setpoint_temp;


		  // disable PWM if motor is at standstill and dutycycle is zero
		  if(ui16_halltics>5000&&!PI_battery_current.setpoint) LL_TIM_DisableAllOutputs(TIM1);

		  // enable PWM if motor is at standstill and power is wanted
		  else if (!LL_TIM_IsEnabledAllOutputs(TIM1)&&PI_battery_current.setpoint) LL_TIM_EnableAllOutputs(TIM1);



		   ui16_timing_counter=0;

	  	  }
	  } //end while (1)
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
} //end main

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_TRGO;//ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = PERIOD;
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
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ITR1;
  if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;//TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_SET;
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
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 32;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */


  HAL_TIMEx_ConfigCommutEvent_IT(&htim1,TIM_TS_ITR1,TIM_COMMUTATION_TRGI);

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_HallSensor_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 96;// timer increments with 500kHz
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
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
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.Commutation_Delay = 1;
  if (HAL_TIMEx_HallSensor_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC2REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  HAL_TIMEx_HallSensor_Start_IT(&htim2);


  /* USER CODE END TIM2_Init 2 */

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
#ifdef DEBUG
  huart1.Init.BaudRate = 56000;
#else
  huart1.Init.BaudRate = 9600;
#endif
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

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
	  __HAL_RCC_GPIOC_CLK_ENABLE();
	  __HAL_RCC_GPIOA_CLK_ENABLE();
	  __HAL_RCC_GPIOB_CLK_ENABLE();

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

	  /*Configure GPIO pin : BKL_Brake_Pin */
	  GPIO_InitStruct.Pin = BKL_Brake_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_PULLUP;
	  HAL_GPIO_Init(BKL_Brake_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pin : LED_Pin */
	  GPIO_InitStruct.Pin = LED_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pin : Light_Pin */
	  GPIO_InitStruct.Pin = Light_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(Light_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pins : TA_PAS_Pin SS_Speed_Pin */
	  GPIO_InitStruct.Pin = TA_PAS_Pin|SS_Speed_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING
			  ;
	  GPIO_InitStruct.Pull = GPIO_PULLUP;
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	  /* EXTI interrupt init*/
	  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
	  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */
// regular ADC callback
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	ui8_adc_regular_flag=1;
	//HAL_GPIO_TogglePin(Light_GPIO_Port, Light_Pin);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	//HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
	//PAS processing
	if(GPIO_Pin == TA_PAS_Pin)
	{
		ui8_PAS_flag = 1;
	}

	//Speed processing
	if(GPIO_Pin == SS_Speed_Pin)
	{

			ui8_SPEED_flag = 1; //with debounce

	}
	//HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
}



void HAL_TIMEx_CommutCallback(TIM_HandleTypeDef *htim)
{

  	TimerCommutationEvent_Callback();

}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim)
{

//nothing to do here....
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	//nothing to do here....
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	ui8_UART_flag=1;

}

//void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
//{
//  /* Prevent unused argument(s) compilation warning */
//  UNUSED(huart);
//  UNUSED(Size);
//  ui8_UART_flag=1;
//  /* NOTE : This function should not be modified, when the callback is needed,
//            the HAL_UARTEx_RxEventCallback can be implemented in the user file.
//   */
//}
void UART_IdleItCallback(void)
{
	ui8_UART_flag=1;

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim==&htim1&&ui16_timing_counter<64000)ui16_timing_counter++;
	if(htim==&htim1&&ui16_SPEED_counter<64000)ui16_SPEED_counter++;
	if(htim==&htim1&&ui16_PAS_counter<64000)ui16_PAS_counter++;
	//HAL_ADC_Start(&hadc);
}

void Get_Direction(void){
	//forward sequence {4,5,1,3,2,6}
	if(ui8_hallstate!=ui8_hallstate_old){
	switch (ui8_hallstate){
		case 1:
			{
				if (ui8_hallstate_old==5)ui8_direction_flag=0;
				else if (ui8_hallstate_old==3)ui8_direction_flag=1;
				break;
			}
		case 2:
			{
				if (ui8_hallstate_old==3)ui8_direction_flag=0;
				else if (ui8_hallstate_old==6) ui8_direction_flag=1;
				break;
			}
		case 3:
			{
				if (ui8_hallstate_old==1)ui8_direction_flag=0;
				else if (ui8_hallstate_old==2)ui8_direction_flag=1;
				break;
			}
		case 4:
			{
				if (ui8_hallstate_old==6)ui8_direction_flag=0;
				else if (ui8_hallstate_old==5)ui8_direction_flag=1;
				break;
			}
		case 5:
			{
				if (ui8_hallstate_old==4)ui8_direction_flag=0;
				else if (ui8_hallstate_old==1) ui8_direction_flag=1;
				break;
			}
		case 6:
			{
				if (ui8_hallstate_old==2)ui8_direction_flag=0;
				else if (ui8_hallstate_old==4)ui8_direction_flag=1;
				break;
			}
		}
		ui8_hallstate_old = ui8_hallstate;
	}
}
//PI Control for battery current
uint32_t PI_control (PI_control_t* PI_c)
{

	int32_t p_part; //proportional part
	p_part = ((PI_c->setpoint - PI_c->recent_value)*PI_c->gain_p);
	if (((PI_c->setpoint - PI_c->recent_value)*PI_c->gain_i)>PI_c->max_step)PI_c->integral_part+=PI_c->max_step;

	else 	PI_c->integral_part += ((PI_c->setpoint - PI_c->recent_value)*PI_c->gain_i);


  if (PI_c->integral_part > PI_c->limit_i << PI_c->shift) PI_c->integral_part = PI_c->limit_i << PI_c->shift;
  if (PI_c->integral_part < 0) PI_c->integral_part = 0;

  if(!LL_TIM_IsEnabledAllOutputs(TIM1)&&!PI_c->setpoint)PI_c->integral_part = 0 ; //reset integral part if PWM is disabled

    //avoid too big steps in one loop run
  if (p_part+PI_c->integral_part > PI_c->out+PI_c->max_step) PI_c->out+=PI_c->max_step;
  else if  (p_part+PI_c->integral_part < PI_c->out-PI_c->max_step)PI_c->out-=PI_c->max_step;
  else PI_c->out=(p_part+PI_c->integral_part);


  if (PI_c->out>PI_c->limit_output << PI_c->shift) PI_c->out = PI_c->limit_output<< PI_c->shift;
  if (PI_c->out<0) PI_c->out=0; // allow no negative voltage.
 // if(!LL_TIM_IsEnabledAllOutputs(TIM1)&&!PI_c->setpoint)PI_c->out = 0 ; //reset output if PWM is disabled
  int32_t out_temp =PI_c->out>>PI_c->shift;//avoid jerk at startup
  if(out_temp<1)out_temp=1;
  return (out_temp);
}
int16_t internal_tics_to_speedx100 (uint32_t tics){
	return KM.Settings.WheelSize_mm*50*3600/(6*GEAR_RATIO*tics);
}

int16_t external_tics_to_speedx100 (uint32_t tics){
	return KM.Settings.WheelSize_mm*16*360/(KM.Settings.SPS_SpdMagnets*tics); //Faktor kontrollieren! 16kHz Zählerfrequenz
}

int32_t map (int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max)
{
  // if input is smaller/bigger than expected return the min/max out ranges value
  if (x < in_min)
    return out_min;
  else if (x > in_max)
    return out_max;

  // map the input to the output range.
  // round up if mapping bigger ranges to smaller ranges
  else  if ((in_max - in_min) > (out_max - out_min))
    return (x - in_min) * (out_max - out_min + 1) / (in_max - in_min + 1) + out_min;
  // round down if mapping smaller ranges to bigger ranges
  else
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void TimerCommutationEvent_Callback(void)
{


  	ui8_hallstate = ((GPIOB->IDR)>>10 & 0b1)+(((GPIOB->IDR)>>3 & 0b1)<<1)+(((GPIOA->IDR)>>15 & 0b1)<<2); //Mask input register with Hall 1 - 3 bits
  	ui16_halltics = TIM2->CCR1;

  /* Entry state */
	  if (uwStep == 0)
	  {
	    /* Initial Step Configuration (executed only once) ---------------------- */
	    /*  Channel1 configuration */
	    LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1);

	    /*  Channel3 configuration */
	    LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_PWM1);
	    LL_TIM_OC_SetCompareCH1(TIM1, ui16_dutycycle);
	    LL_TIM_OC_SetCompareCH3(TIM1, PERIOD);

	    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1 |
	                                  LL_TIM_CHANNEL_CH3N);

	    LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH1N |
	                                   LL_TIM_CHANNEL_CH2  |
	                                   LL_TIM_CHANNEL_CH2N |
	                                   LL_TIM_CHANNEL_CH3);
	    ui8_hallstate=ui8_hallstate_old;
	    uwStep = 1;



	  }
	  Get_Direction();


switch (hall_sequence[0][ui8_hallstate]){ //ui8_direction_flag on first row of array
case 1:
  {

    /*
            Phase A | Phase B | Phase C || Hall C | Hall B | Hall A
            +         -         NC         0        1        1
            CH1|CH1N||CH2|CH2N||CH3|CH3N
            PWM1/2  ||OFF| ON ||OFF|OFF
           */
            /* Next step: Step 1 Configuration -------------------------------------- */
            /*  Channel1 configuration */
	  	  if(ui16_dutycycle){
            LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1);
            LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N);
	  	  	  }
	  	  else{
	  		LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_INACTIVE);
	  	  }

            /*  Channel2 configuration */
            LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_ACTIVE);
            LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2N);
            LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH2);

            /*  Channel3 configuration */
            LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_INACTIVE);
            LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3);
            LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH3N);
            break;

  }

case 2:
  {

    /*
        Phase A | Phase B | Phase C || Hall C | Hall B | Hall A
        NC        -         +          0        0        1
        CH1|CH1N||CH2|CH2N||CH3|CH3N
        OFF|OFF ||OFF| ON ||PWM1/2
       */
        /* Next step: Step 1 Configuration -------------------------------------- */
        /*  Channel1 configuration - CH1:OFF, CH1N:OFF */
        LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_INACTIVE);
        LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
        LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH1N);

        /*  Channel2 configuration - CH2:OFF, CH2N:ON */
        LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_ACTIVE);
        LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2N);
        LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH2);

        /*  Channel3 configuration - CH3:PWM1, CH3N:PWM2 */
      if(ui16_dutycycle){
        LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_PWM1);
        LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N);
        }
      else{
    	  LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_INACTIVE);
      }
        break;
  }

case 3:
  {


    /*
      Phase A | Phase B | Phase C || Hall C | Hall B | Hall A
      -         NC        +          1        0        1
      CH1|CH1N||CH2|CH2N|CH3|CH3N
      OFF| ON ||OFF|OFF |PWM1/2
     */
      /* Next step: Step 1 Configuration -------------------------------------- */
      /*  Channel1 configuration */
      LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_ACTIVE);
      LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1N);
      LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH1);

      /*  Channel2 configuration */
      LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_INACTIVE);
      LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2);
      LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH2N);

      /*  Channel3 configuration */
    if(ui16_dutycycle){
      LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_PWM1);
      LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N);
      }
    else{
          	  LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_INACTIVE);
            }
      break;
  }

case 4:
  {

    /*
      Phase A | Phase B | Phase C || Hall C | Hall B | Hall A
      -         +         NC         0        0        1
      CH1|CH1N||CH2|CH2N||CH3|CH3N
      OFF| ON ||PWM1/2  ||OFF|OFF
     */
      /* Next step: Step 1 Configuration -------------------------------------- */
      /*  Channel1 configuration */
      LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_ACTIVE);
      LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1N);
      LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH1);

      /*  Channel2 configuration */
     if(ui16_dutycycle){
      LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_PWM1);
      LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N);
      }
     else{
           	  LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_INACTIVE);
             }

      /*  Channel3 configuration */
      LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_INACTIVE);
      LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3);
      LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH3N);
      break;
  }

case 5:
  {


    /*
      Phase A | Phase B | Phase C || Hall C | Hall B | Hall A
      NC        +         -          0        1        1
      CH1|CH1N||CH2|CH2N|CH3|CH3N
      OFF|OFF ||PWM1/2  |OFF| ON
     */
      /* Next step: Step 1 Configuration -------------------------------------- */
      /*  Channel1 configuration */
      LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_INACTIVE);
      LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1N);
      LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH1);

      /*  Channel2 configuration */
      if(ui16_dutycycle){
       LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_PWM1);
       LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N);
       }
      else{
            	  LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_INACTIVE);
              }

      /*  Channel3 configuration */
      LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_ACTIVE);
      LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3N);
      LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH3);
      break;
  }

case 6:
  {

    /*
           Phase A | Phase B | Phase C || Hall C | Hall B | Hall A
           +         NC        -          0        1        0
           CH1|CH1N||CH2|CH2N||CH3|CH3N
           PWM1/2  ||OFF|OFF ||OFF| ON
          */
           /* Next step: Step 1 Configuration -------------------------------------- */
           /*  Channel1 configuration */
  	  if(ui16_dutycycle){
        LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1);
        LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N);
  	  	  }
  	  else{
  		LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_INACTIVE);
  	  }

           /*  Channel2 configuration */
           LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_INACTIVE);
           LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2);
           LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH2N);

           /*  Channel3 configuration */
           LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_ACTIVE);
           LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3N);
           LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH3);
           break;

  }
	} //end switch
}

void print_debug_info(void){
		sprintf_(tx_buffer,"%d, %d, %d, %d, %d, %d, %d\r\n ",
				PI_battery_current.integral_part>>PI_battery_current.shift,
				ui16_dutycycle,
				PI_battery_current.setpoint,
				i16_battery_current_raw,
				ui16_battery_current_offset,
				HAL_GPIO_ReadPin(BKL_Brake_GPIO_Port, BKL_Brake_Pin),
				ui16_halltics);
		i=0;
		while (tx_buffer[i] != '\0'){i++;}

		HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&tx_buffer, i);
}

void kingmeter_update(void)
{
    /* Prepare Tx parameters */

    if(battery_percent_fromcapacity > 10) //not updated yet
    {
        KM.Tx.Battery = KM_BATTERY_NORMAL;
    }
    else
    {
        KM.Tx.Battery = KM_BATTERY_LOW;
    }


    if(ui16_SPEED_counter<63000)KM.Tx.Wheeltime_ms = ((ui16_SPEED*KM.Settings.SPS_SpdMagnets)>>4); //>>4 because of 16 kHz counter frequency, so 16 tics per ms
    else KM.Tx.Wheeltime_ms = 64000;


    KM.Tx.Error = KM_ERROR_NONE;


    KM.Tx.Current_x10 = (uint16_t) (i16_battery_current/100); //MS.Battery_Current is in mA





    /* Apply Rx parameters */

    ui8_assist_level = KM.Rx.AssistLevel;

    if(KM.Rx.Headlight == KM_HEADLIGHT_OFF)
        {
        	//HAL_GPIO_WritePin(LIGHT_GPIO_Port, LIGHT_Pin, GPIO_PIN_RESET);

        }
        else // KM_HEADLIGHT_ON, KM_HEADLIGHT_LOW, KM_HEADLIGHT_HIGH
        {
        	//HAL_GPIO_WritePin(LIGHT_GPIO_Port, LIGHT_Pin, GPIO_PIN_SET);

        }


    if(KM.Rx.PushAssist == KM_PUSHASSIST_ON)
    {
    	ui8_Push_Assist_flag=1;
    }
    else
    {
    	ui8_Push_Assist_flag=0;
    }
//    MP.speedLimit=KM.Rx.SPEEDMAX_Limit;
    ui16_battery_current_max_raw = KM.Rx.CUR_Limit_mA/ui8_cal_battery_current;
//    MP.battery_current_max = KM.Rx.CUR_Limit_mA;



}


//void JumpToBootloader (void)
//{
//uint32_t i=0;
//void (*SysMemBootJump)(void);
//
//
//	/* Disable all interrupts */
//	__disable_irq();
//
//	/* Disable Systick timer */
//	SysTick->CTRL = 0;
//
//	/* Set the clock to the default state */
//	HAL_RCC_DeInit();
//
//	/* Clear Interrupt Enable Register & Interrupt Pending Register */
//	for (i=0;i<5;i++)
//	{
//		NVIC->ICER[i]=0xFFFFFFFF;
//		NVIC->ICPR[i]=0xFFFFFFFF;
//	}
//
//	/* Re-enable all interrupts */
//	__enable_irq();
//
//	/* Set up the jump to boot loader address + 4 */
//	SysMemBootJump = (void (*)(void)) (*((uint32_t *) ((0x1FFFEC00 + 4))));
//
//	/* Set the main stack pointer to the boot loader stack */
//	  /* Enable the SYSCFG peripheral clock*/
//	  __HAL_RCC_SYSCFG_CLK_ENABLE();
//	  /* Remap SRAM at 0x00000000 */
//	  __HAL_SYSCFG_REMAPMEMORY_SRAM();
//
//
//	__set_MSP(*(uint32_t *)0x1FFFEC00);
//
//
//	/* Call the function to jump to boot loader location */
//	SysMemBootJump();
//
//	/* Jump is done successfully */
//	while (1)
//	{
//		/* Code should never reach this loop */
//	}
//
//}






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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
