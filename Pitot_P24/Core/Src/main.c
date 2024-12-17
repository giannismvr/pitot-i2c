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
#include <math.h>
/* USER CODE END Includes */
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */
/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define I2C_PITOT 0
#define ADC_PITOT 1
#define FUNCTION I2C_PITOT
#define CAN_TIME 30
#define TIME_OUT 1000
#define DPUpperRange 2047
#define DPLowerRange 0
#define MS45x5DO_Fault 0
#define Pmax 14746
#define Pmin 1638
#define Tmax 150
#define Tmin -50
#define ADC_BUF_LEN 1
#define smooth  2000
/* USER CODE END PD */
/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */
/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
CAN_HandleTypeDef hcan;
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim4;
/* USER CODE BEGIN PV */
/* USER CODE END PV */
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
#if (FUNCTION == ADC_PITOT)
uint8_t msg[8]={0,0,0,0,0,0,0,0};
uint16_t adc_buf[ADC_BUF_LEN];
uint8_t x[3] = {0,0,0};
volatile uint8_t overr_flag=0, allow=1;
uint16_t adcVal1[smooth], test = 0;
long long int final[1] = {0};
int time=0;
char val[100];
unsigned int current = 0, msg_previous = 0, msg_interval, default_msg_interval, can_int_prev=0, can_int_interval=100;
int adc_pitot =0;
float analog_pitot=0;
float pressure = 0;
float digits = 0;
#endif
float U;
float fp_avg;
float fpfilt;
uint8_t fpcnt;
float tmp_avg;
float tmpfilt;
uint8_t tmpcnt;
uint32_t time,tick,timecan,tickcan;
uint8_t startup_flag;
uint16_t pcnt;
uint32_t psum;
uint16_t error_cnt;
uint16_t cant_receive;
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint32_t TxMailbox;
uint8_t RxData[8], TxData[8], temp, pressure, t;;
uint8_t ACCU[8];
uint32_t time_since_last_message_accu, time_since_last_message_elcon_tsac_can_bus_rx,display_time;
uint8_t counter, flag,flagg;
/* USER CODE END PFP */
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#if (FUNCTION == I2C_PITOT)
typedef struct MS45x5DOObject {
   uint8_t devAddress;
   union {
       struct {
           uint16_t pressure:14;          // to purpose to union einai to ejhs:
           uint16_t status:2;
           uint16_t insignificance:5;  //otan kaneis access to msData apo pData tote kaneis access ta memebers tou struct, ENW
           uint16_t temperature:11;    //otan kaneis acces to msdata apo rData tote kaneis access ta raw bytes tou rData[4] pinaka (4 bytes)
       }pData;                          // etsi ta 4 bytes tou rData periexoun ta temperature, pressure, insignificance klp..
       uint8_t rData[4];              // diladi, to rData[0] periexei ta LSB gia to "pressure" kai to "status", alla oxi olo to value se bit gia to kathena.....
   }msData;
   int type;
   float pUpperRange;
   float pLowerRange;
   float fTemperature;
   float fPressure;
   void (*Write)(struct MS45x5DOObject *ms,uint8_t *wData,uint16_t wSize);
   void (*Read)(struct MS45x5DOObject *ms,uint8_t *rData,uint16_t rSize);
   void (*Delayms)(volatile uint32_t nTime);
   uint32_t pavg;
   int8_t sign;
}MS45x5DOObjectType;
MS45x5DOObjectType msDP;
//i2c
typedef void (*MS45x5DOWrite)(struct MS45x5DOObject *ms,uint8_t *wData,uint16_t wSize);
typedef void (*MS45x5DORead)(struct MS45x5DOObject *ms,uint8_t *rData,uint16_t rSize);
typedef void (*MS45x5DODelayms)(volatile uint32_t nTime);
static void WriteToDP(MS45x5DOObjectType *ms,uint8_t *wData,uint16_t wSize)
{
   HAL_I2C_Master_Transmit(&hi2c1,ms->devAddress,wData,wSize,1000);
}
static void ReadFromDP(MS45x5DOObjectType *ms,uint8_t *rData,uint16_t rSize)
{
   HAL_I2C_Master_Receive(&hi2c1,ms->devAddress,rData, rSize, 1000);
}
void MS45x5DOInitialization(MS45x5DOObjectType *ms,
                           uint8_t devAddress,
                           int type,
                           float pMax,
                           float pMin,
                           MS45x5DOWrite write,
                           MS45x5DORead read,
                           MS45x5DODelayms delayms
                               )
{
   if((ms==NULL)||(write==NULL)||(read==NULL)||(delayms==NULL))
   {
       return;
   }
   ms->Write=write;
   ms->Read=read;
   ms->Delayms=delayms;
   if((devAddress==0x28)||(devAddress==0x36)||(devAddress==0x46)||((0x48<=devAddress)&&(devAddress<=0x51))) //I2C slave address
   {
       ms->devAddress=(devAddress<<1);
   }
   else if((devAddress==0x50)||(devAddress==0x6C)||(devAddress==0x8C)||((0x48<=(devAddress/2))&&((devAddress/2)<=0x51)))
   {
       ms->devAddress=devAddress;
   }
   else
   {
       ms->devAddress=0x00;
   }
   ms->type=type;
   ms->fPressure=0.0;
   ms->fTemperature=0.0;
   ms->msData.rData[0]=0;
   ms->msData.rData[1]=0;
   ms->msData.rData[2]=0;
   ms->msData.rData[3]=0;
   if((pMax<=0.0000001)&&(pMin<=0.0000001))
   {
       ms->pUpperRange=100.0;
       ms->pLowerRange=0.0;
   }
   else
   {
       ms->pUpperRange=pMax;
       ms->pLowerRange=pMin;
   }
}
void GetMS45x5DOConversionValue(MS45x5DOObjectType *ms)
{
   uint8_t rData[4]={0,0,0,0};
   float maxCount=16383;
   float minCount=0;
   if(ms->type==0)
   {
       maxCount=13106;
       minCount=1638;
   }
   else
   {
       maxCount=14746;
       minCount=819;
   }
   ms->Read(ms,rData,4); //this receives data constantly------------------------------------------nomizw kapws i msdoconversion (sthn while(1)) kallei thn *Read pou deixnei (mesa sthn conversion) sthn local "read" h opoia efoson einai orisma ths void kalei thn MS45x5DORead. omws, sthn initialize, h MS45x5DORead einai proteleftaio orisma. h initialize kaleitai sthn int main me parametro ReadfromDP, h opoia kanei read mesw ths I2C_master_transmit. h while epetai ths klhshs initialization sthn int main, ara etsi lambanei data synexws sthn while (1)
   ms->msData.rData[0]=rData[1]; // where rData[1]==pressu ---- edw anadiatassei ta data pou kanei receive o sensoras, symfwna me thn seira ton data pou exei thesei o idios
   ms->msData.rData[1]=rData[0]; // where rData[0]=status    ----px thelei prwta thn thermokrasia (leme twra) ara data[0] enw o sensoras thn stelnei teleytaia data[3]
   ms->msData.rData[2]=rData[3]; // ...insignificance
   ms->msData.rData[3]=rData[2]; // .... temperature
   if(ms->msData.pData.status!=MS45x5DO_Fault)
   {
      //ms->fPressure = (((float)))
   	ms->fPressure=(((float)(ms->msData.pData.pressure + (8192 - 8090)))/16384.0f);
   	//ms->fPressure=(((float)(8192 + (8192 - ms->pavg)))/16384.0f);
   	if (ms->fPressure >= 0.5) {
   		ms->fPressure -= 0.5;
   		ms->sign = 1;
   	}
   	else {
   		ms->fPressure =0.5 - ms->fPressure;
   		ms->sign = -1;
   	}
   	ms->fPressure *= 6894.75729 * 2;
       ms->fTemperature=((float)ms->msData.pData.temperature/2047.0)*200.0-50.0;
   }
}
#endif
////CANBUS
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint32_t TxMailbox;
int i, lasttick;
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
 MX_CAN_Init();
 MX_I2C1_Init();
 MX_TIM4_Init();
 MX_ADC1_Init();
 /* USER CODE BEGIN 2 */
//#if (FUNCTION == I2C_PITOT)
//  MS45x5DOInitialization(&msDP,0x28,0,DPUpperRange,DPLowerRange,WriteToDP,ReadFromDP,HAL_Delay);
//#endif
 HAL_Delay(1000);
 //HAL_ADC_Start_DMA(&hadc1,(uint32_t*)adc_buf,ADC_BUF_LEN);
 uint8_t r = 1.2047; //1.1.1687; grammars/liter
 /* USER CODE END 2 */
 /* Infinite loop */
 /* USER CODE BEGIN WHILE */
 while (1)
 {
   /* USER CODE END WHILE */
   /* USER CODE BEGIN 3 */
	  //--------------------------------------------------------------------------------delete from here
	  	  	  TxHeader.StdId = 0x499;
			  TxData[0]=69;
			  TxData[1]= 54;    //ayto to kanoume genika epd to canbus stelnei mono integers kai oxi floats
	     	  TxData[2]= 54;   //opote tis kanoume convert se integers kai meta tis stelnoume
	     	  TxData[3]= 54;
	     	  TxData[4]= 54; //kaneis >>8 epd exeis uint16_t kai ara kaneis shift 8 theseis dejia
	     	  TxData[5]= 54;
	     	  TxData[6]= 0;
	     	  TxData[7]= 0;
	     	 if(HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK){
	     	     		  error_cnt++;;
	     	     	  }
	     	 //HAL_CAN_RxFifo0MsgPendingCallback(&hcan); // only for loopback mode
	   //-------------------------------------------------------------------------------- to here
//	  GetMS45x5DOConversionValue(&msDP);
//
//	  if(fpcnt < 10){
//		  fp_avg += msDP.fPressure;
//		  fpcnt++;
//	  }
//	  else{
//		  fpfilt = (float)fp_avg/(float)fpcnt;  //in PSI
//		  fpcnt = 0;
//		  fp_avg = 0;
//		  time = HAL_GetTick() - tick;
//		  tick = HAL_GetTick();
//	  }
//
//	  if(tmpcnt < 200){
//		  tmp_avg += msDP.fTemperature;
//		  tmpcnt++;
//	  }
//	  else{
//		  tmpfilt = (float)tmp_avg/(float)tmpcnt;
//		  tmpcnt = 0;
//		  tmp_avg = 0;
//
//	  }
//
//	  U = sqrt(2*fabsf(fpfilt)/r); // nmz m/sec
//
//	  //the expression (2*fabsf(fpfilt)/r) represents the dynamic pressure of the fluid flow,
//	  //and taking the sqrt of it, you get the airspeed
//
//	  timecan = HAL_GetTick() - tickcan;
//
//
//      if(timecan > CAN_TIME){
//    	  TxHeader.StdId = 0x499;
//    	  //TxData[0]= (uint16_t)(fpfilt*1000) >> 8; //gnka exeis times tou typou 25.654 kai 25.654*1000==25654
//    	  TxData[1]= (uint16_t)(fpfilt*1000);    //ayto to kanoume genika epd to canbus stelnei mono integers kai oxi floats
//    	  TxData[2]= (uint16_t)(U*1000) >> 8;    //opote tis kanoume convert se integers kai meta tis stelnoume
//    	  TxData[3]= (uint16_t)(U*1000);
//    	  TxData[4]= (uint16_t)(tmpfilt*1000) >> 8;  //kaneis >>8 epd exeis uint16_t kai ara kaneis shift 8 theseis dejia
//    	  TxData[5]= (uint16_t)(tmpfilt*1000);
//    	  TxData[6]= 0;
//    	  TxData[7]= 0;
//    	  if(HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK){
//    		  error_cnt++;;
//    	  }
//
//    	  //HAL_CAN_RxFifo0MsgPendingCallback(&hcan); // only for loopback mode
//
//
////    	  if (HAL_CAN_GetRxMessage(&hcan, &RxHeader, Rxdata, &RxMailbox) != HAL_OK) {
////    	      		  cant_receive++;
////    	      	  }
//    	  tickcan = HAL_GetTick();
//	  }
//
//	  if(!startup_flag && msDP.msData.pData.pressure != 0) {
//		  if(pcnt < 1000){
//			  psum += msDP.msData.pData.pressure;
//		  	  pcnt++;
//		  }
//		  else {
//			  msDP.pavg = psum/pcnt;
//			  startup_flag++;
//		  }
//	  }
	  //U = sqrt(2*fabsf(msDP.fPressure))/r;
#if (FUNCTION == ADC_PITOT)
	  HAL_ADC_Start_DMA(&hadc1,(uint32_t*)adc_buf,ADC_BUF_LEN);
	  	  current = HAL_GetTick();
	  	  if (msg_previous < current) {
	  		  analog_pitot = ((float)adc_pitot)*0.001241;
	  		  pressure = analog_pitot - 2.5;
	  	  uint8_t r = 1.2047;
	  	  U = sqrt((2000*fabsf(pressure))/r);
	  	  adcVal1[time]=adc_buf[0];
	  	  if(time==smooth-1) {
	  		  for(int i=0; i<smooth; i++) {
	  			  final[0]+=adcVal1[i];
	  		  }
	  		  final[0]/=smooth;
	  		  adc_pitot=final[0];
	  		  time=0;
	  	  }
	  	  else {
	  		  time++;
	  	  }
#endif
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
 hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
 hadc1.Init.ContinuousConvMode = ENABLE;
 hadc1.Init.DiscontinuousConvMode = DISABLE;
 hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
 hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
 hadc1.Init.NbrOfConversion = 1;
 if (HAL_ADC_Init(&hadc1) != HAL_OK)
 {
   Error_Handler();
 }
 /** Configure Regular Channel
 */
 sConfig.Channel = ADC_CHANNEL_2;
 sConfig.Rank = ADC_REGULAR_RANK_1;
 sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
 if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
 {
   Error_Handler();
 }
 /* USER CODE BEGIN ADC1_Init 2 */
 /* USER CODE END ADC1_Init 2 */
}
/**
 * @brief CAN Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN_Init(void)
{
 /* USER CODE BEGIN CAN_Init 0 */
 /* USER CODE END CAN_Init 0 */
 /* USER CODE BEGIN CAN_Init 1 */
 /* USER CODE END CAN_Init 1 */
 hcan.Instance = CAN1;
 hcan.Init.Prescaler = 4;
 hcan.Init.Mode = CAN_MODE_NORMAL;
 hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
 hcan.Init.TimeSeg1 = CAN_BS1_6TQ;
 hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
 hcan.Init.TimeTriggeredMode = DISABLE;
 hcan.Init.AutoBusOff = DISABLE;
 hcan.Init.AutoWakeUp = DISABLE;
 hcan.Init.AutoRetransmission = DISABLE;
 hcan.Init.ReceiveFifoLocked = DISABLE;
 hcan.Init.TransmitFifoPriority = DISABLE;
 if (HAL_CAN_Init(&hcan) != HAL_OK)
 {
   Error_Handler();
 }
 /* USER CODE BEGIN CAN_Init 2 */
 CAN_FilterTypeDef canfilterconfig;
   canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
   canfilterconfig.FilterBank = 10;
   canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
   canfilterconfig.FilterIdHigh = 0;
   canfilterconfig.FilterIdLow = 0;
   canfilterconfig.FilterMaskIdHigh =0;
   canfilterconfig.FilterMaskIdLow = 0;
   canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
   canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
   canfilterconfig.SlaveStartFilterBank = 0;
   HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);
   HAL_CAN_Start(&hcan);
   HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
   TxHeader.DLC = 8;
   TxHeader.IDE = CAN_ID_STD;
   TxHeader.RTR = CAN_RTR_DATA;
   TxHeader.TransmitGlobalTime = DISABLE;
//
		RxHeader.DLC = 8;
		RxHeader.IDE = CAN_ID_STD;
       RxHeader.RTR = CAN_RTR_DATA;
       //RxHeader.ReceiveGlobalTime = DISABLE;
 /* USER CODE END CAN_Init 2 */
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
 /* USER CODE BEGIN I2C1_Init 2 */
 /* USER CODE END I2C1_Init 2 */
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
 TIM_SlaveConfigTypeDef sSlaveConfig = {0};
 TIM_MasterConfigTypeDef sMasterConfig = {0};
 /* USER CODE BEGIN TIM4_Init 1 */
 /* USER CODE END TIM4_Init 1 */
 htim4.Instance = TIM4;
 htim4.Init.Prescaler = 0;
 htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
 htim4.Init.Period = 65535;
 htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
 htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
 if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
 {
   Error_Handler();
 }
 sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
 sSlaveConfig.InputTrigger = TIM_TS_ITR0;
 if (HAL_TIM_SlaveConfigSynchro(&htim4, &sSlaveConfig) != HAL_OK)
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
}
/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */
 /* GPIO Ports Clock Enable */
 __HAL_RCC_GPIOD_CLK_ENABLE();
 __HAL_RCC_GPIOA_CLK_ENABLE();
 __HAL_RCC_GPIOB_CLK_ENABLE();
/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}
/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
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

