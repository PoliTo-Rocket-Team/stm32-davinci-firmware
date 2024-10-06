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
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "e220.h"
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
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* Definitions for StartupTask */
osThreadId_t StartupTaskHandle;
const osThreadAttr_t StartupTask_attributes = {
  .name = "StartupTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for FlashWriteTask */
osThreadId_t FlashWriteTaskHandle;
const osThreadAttr_t FlashWriteTask_attributes = {
  .name = "FlashWriteTask",
  .stack_size = 256*4,//128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SensorsReadTask */
osThreadId_t SensorsReadTaskHandle;
const osThreadAttr_t SensorsReadTask_attributes = {
  .name = "SensorsReadTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for COMBoardTask */
osThreadId_t COMBoardTaskHandle;
const osThreadAttr_t COMBoardTask_attributes = {
  .name = "COMBoardTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for FlightFSMTask */
osThreadId_t FlightFSMTaskHandle;
const osThreadAttr_t FlightFSMTask_attributes = {
  .name = "FlightFSMTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SystemHealthCheckTask */
osThreadId_t SystemHealthCheckTaskHandle;
const osThreadAttr_t SystemHealthCheckTask_attributes = {
  .name = "SystemHealthCheckTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */
/* SENSORS AND DEVICES DECLARATION */
W25Q128_t flash = {0};
ExtU rtU = {0};
ExtY rtY = {0};

struct bmp3_dev bmp390_1 = {0}, bmp390_2 = {0};
stmdev_ctx_t imu_1 = {0}, imu_2 = {0};

pitot_sensor_t pitot = {0};
uint8_t output[4000] = {0};
servo_t servo = {0};

buzzer_t buzzer = {0};
uint32_t flash_addr1 = FLASH_START_ADDRESS;
uint32_t flash_addr2 = FLASH_START_ADDRESS +(uint32_t)32;
uint32_t flash_addr3 = FLASH_START_ADDRESS +(uint32_t)44;
uint32_t flash_addr4 = FLASH_START_ADDRESS +(uint32_t)76;
uint32_t flash_addr5 = FLASH_START_ADDRESS +(uint32_t)88;
uint32_t flash_addr6 = FLASH_START_ADDRESS +(uint32_t)120;
uint32_t flash_addr7 = FLASH_START_ADDRESS +(uint32_t)132;
uint32_t flash_addr8 = FLASH_START_ADDRESS +(uint32_t)164;
uint32_t flash_addr9 = FLASH_START_ADDRESS +(uint32_t)176;
uint32_t flash_addr10 = FLASH_START_ADDRESS +(uint32_t)208;
uint32_t flash_addr11 = FLASH_START_ADDRESS +(uint32_t)220;
uint32_t flash_addr12 = FLASH_START_ADDRESS +(uint32_t)252;
uint32_t flash_addr13 = FLASH_START_ADDRESS +(uint32_t)264;
uint32_t flash_addr14 = FLASH_START_ADDRESS +(uint32_t)296;
uint32_t flash_addr15 = FLASH_START_ADDRESS +(uint32_t)308;
uint32_t flash_addr16 = FLASH_START_ADDRESS +(uint32_t)340;


/* DECLARATION AND DEFINITION OF THE GLOBAL VARIABLES FOR THE FLIGHT */

flight_phase_t flight_phase = CALIBRATING;

flight_fsm_t flight_state;


//char num_acc_modify[]={0,0,0};
//char num_gyro_modify[]={0,0,0};

struct bmp3_data barometer_data_1 = {-1, -1};
struct bmp3_data barometer_data_2 = {-1, -1};

static int16_t data_raw_angular_rate_1[3] = {0};
static int16_t data_raw_acceleration_1[3] = {0};
static int16_t data_raw_angular_rate_2[3] = {0};
static int16_t data_raw_acceleration_2[3] = {0};

static float_t acceleration_mg_1[3] = {0};
static float_t angular_rate_mdps_1[3] = {0};
static float_t acceleration_mg_2[3] = {0};
static float_t angular_rate_mdps_2[3] = {0};

linear_acceleration_t curr_acc = {0};
linear_acceleration_t prev_acc = {0};

#pragma pack(push, 1)
typedef struct {
	float_t acc_x;
	float_t acc_y;
	float_t acc_z;

	float_t dps_x;
	float_t dps_y;
	float_t dps_z;

	float_t temperature;
	float_t pressure;

} sensor_data;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct{
    float_t altitude;
    float_t velocity;
    float_t phase;
} sensor_data_2;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
	char carattere;
	float_t acc_x;
	float_t acc_y;
	float_t acc_z;

	float_t dps_x;
	float_t dps_y;
	float_t dps_z;

	float_t temperature;
	float_t pressure;

    float_t altitude;
    float_t velocity;
    float_t phase;
} Lora_Package;
#pragma pack(pop)





uint32_t num_meas_stored_in_buffer = (uint32_t)0;
uint32_t num_meas_stored_in_buffer_2 = (uint32_t)0;
uint32_t num_big_meas_stored_in_flash = (uint32_t)0;
uint32_t num_small_meas_stored_in_flash = (uint32_t)0;
//uint32_t num_times_lol_was_written = (uint32_t)0;
bool flash_flag = false;

//uint8_t *measurements_buffer;
sensor_data measurements_buffer[FLASH_NUMBER_OF_STORE_EACH_TIME * 2];
//uint8_t *measurements_buffer_2;
sensor_data_2 measurements_buffer_2[FLASH_NUMBER_OF_STORE_EACH_TIME * 2];



float_t altitude;
bool first_measure = true;
float_t velocity;
float_t Pressure_1;
float_t Pressure_2;

uint8_t send_buffer[35]="Hello There!bro";
uint8_t receive_buffer[35];
int16_t Lora_result;
uint8_t all_reg_rx[8], all_reg_tx[8];

struct LoRa_Handler LoraTX={0},LoraRX={0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
void Startup(void *argument);
void FlashWrite(void *argument);
void SensorsRead(void *argument);
void CommunicationBoard(void *argument);
void FlightFSM(void *argument);
void SystemHealthCheck(void *argument);

/* USER CODE BEGIN PFP */

//static uint32_t GetSector(uint32_t Address)
//{
//  uint32_t sector = 0;
//
//  if((Address < 0x08003FFF) && (Address >= 0x08000000))
//  {
//    sector = FLASH_SECTOR_0;
//  }
//  else if((Address < 0x08007FFF) && (Address >= 0x08004000))
//  {
//    sector = FLASH_SECTOR_1;
//  }
//  else if((Address < 0x0800BFFF) && (Address >= 0x08008000))
//  {
//    sector = FLASH_SECTOR_2;
//  }
//  else if((Address < 0x0800FFFF) && (Address >= 0x0800C000))
//  {
//    sector = FLASH_SECTOR_3;
//  }
//  else if((Address < 0x0801FFFF) && (Address >= 0x08010000))
//  {
//    sector = FLASH_SECTOR_4;
//  }
//  else if((Address < 0x0803FFFF) && (Address >= 0x08020000))
//  {
//    sector = FLASH_SECTOR_5;
//  }
//  else if((Address < 0x0805FFFF) && (Address >= 0x08040000))
//  {
//    sector = FLASH_SECTOR_6;
//  }
//  else if((Address < 0x0807FFFF) && (Address >= 0x08060000))
//  {
//    sector = FLASH_SECTOR_7;
//  }
//
//  return sector;
//}
//
//
//uint32_t Flash_Write_Data (uint32_t StartSectorAddress, uint32_t *Data, uint16_t numberofwords)
//{
//
//	static FLASH_EraseInitTypeDef EraseInitStruct;
//	uint32_t SECTORError;
//	int sofar=0;
//
//
//	 /* Unlock the Flash to enable the flash control register access *************/
//	  HAL_FLASH_Unlock();
//
//	  /* Erase the user Flash area */
//
//	  /* Get the number of sector to erase from 1st sector */
//
//	  uint32_t StartSector = GetSector(StartSectorAddress);
//	  uint32_t EndSectorAddress = StartSectorAddress + numberofwords*4;
//	  uint32_t EndSector = GetSector(EndSectorAddress);
//
//	  /* Fill EraseInit structure*/
//	  EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
//	  EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
//	  EraseInitStruct.Sector        = StartSector;
//	  EraseInitStruct.NbSectors     = (EndSector - StartSector) + 1;
//
//	  /* Note: If an erase operation in Flash memory also concerns data in the data or instruction cache,
//	     you have to make sure that these data are rewritten before they are accessed during code
//	     execution. If this cannot be done safely, it is recommended to flush the caches by setting the
//	     DCRST and ICRST bits in the FLASH_CR register. */
//	  if (HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError) != HAL_OK)
//	  {
//		  return HAL_FLASH_GetError ();
//	  }
//
//	  /* Program the user Flash area word by word
//	    (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/
//
//	   while (sofar<numberofwords)
//	   {
//	     if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, StartSectorAddress, Data[sofar]) == HAL_OK)
//	     {
//	    	 StartSectorAddress += 4;  // use StartPageAddress += 2 for half word and 8 for double word
//	    	 sofar++;
//	     }
//	     else
//	     {
//	       /* Error occurred while writing data in Flash memory*/
//	    	 return HAL_FLASH_GetError ();
//	     }
//	   }
//
//	  /* Lock the Flash to disable the flash control register access (recommended
//	     to protect the FLASH memory against possible unwanted operation) *********/
//	  HAL_FLASH_Lock();
//
//	   return 0;
//}
//
//void Flash_Read_Data (uint32_t StartSectorAddress, uint32_t *RxBuf, uint16_t numberofwords)
//{
//	while (1)
//	{
//
//		*RxBuf = *(__IO uint32_t *)StartSectorAddress;
//		StartSectorAddress += 4;
//		RxBuf++;
//		if (!(numberofwords--)) break;
//	}
//}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
W25Q128_t* get_flash(){
	return &flash;
}

servo_t* get_servo(){
	return &servo;
}

float_t readAltitude(float_t seaLevelPa,float_t currentPa) {
  float_t altitude;

  altitude = (float_t)44330.0 * ((float_t)1.0 - pow(currentPa / seaLevelPa, (float_t)0.1903));

  return altitude;
}

//uint32_t GetAddr(){
//	return addr;
//}

//void SetAddr(){
//	addr += 352;
//}

uint32_t Getnum_meas_stored_in_buffer(){
	return num_meas_stored_in_buffer;
}

uint32_t Getnum_big_meas_stored_in_flash(){
	return num_big_meas_stored_in_flash;
}

uint32_t Getnum_small_meas_stored_in_flash(){
	return num_small_meas_stored_in_flash;
}

void Setnum_meas_stored_in_flash(){
	num_big_meas_stored_in_flash += 8;
	num_small_meas_stored_in_flash +=3;
}

void Setnum_meas_stored_in_buffer(float num){
	if(num <0){
		num_meas_stored_in_buffer = (uint32_t)0;
		num_meas_stored_in_buffer_2 = (uint32_t)0;
	}else{
		num_meas_stored_in_buffer += (uint32_t)num;
		num_meas_stored_in_buffer_2 += (uint32_t)num;
	}

}


void Setnum_meas_stored_in_buffermod(){
	num_meas_stored_in_buffer %= (uint32_t)8;
	num_meas_stored_in_buffer_2 %= (uint32_t)8;
}

void Set_Flash_Flag(bool val){
	flash_flag = val;
}

bool Get_Flash_Flag(){
	return flash_flag;
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
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  	codegen_model_initialize();
  	rtU.Altitudeinput = (double)0.0;
  	rtU.Verticalvelocityinput = (double)0.0;
  	size_t i = 0;
	int8_t result = 0;
//	int8_t result2 = 0;


	/* deactivate chip select of IMU and BMP390 */
	IMU1_CS_HIGH;
	IMU2_CS_HIGH;
	BMP390_1_CS_HIGH;
	BMP390_2_CS_HIGH;

	LED_ON(DEBUG_LED_FLASH_GPIO_Port, DEBUG_LED_FLASH_Pin);

	result = init_imu1(&imu_1, LSM6DSO32_16g, LSM6DSO32_2000dps, LSM6DSO32_XL_ODR_208Hz_NORMAL_MD, LSM6DSO32_GY_ODR_208Hz_HIGH_PERF);
	result = init_imu2(&imu_2, LSM6DSO32_16g, LSM6DSO32_2000dps, LSM6DSO32_XL_ODR_208Hz_NORMAL_MD, LSM6DSO32_GY_ODR_208Hz_HIGH_PERF);

	result = init_bmp390_1(&bmp390_1);
	result = init_bmp390_2(&bmp390_2);

	result = init_flash(&flash, ERASE); //XXX use this once to erase the chip
//	result = init_flash(&flash, BYPASS);//XXX use this everytime the chip does not need to be erased

//	if (result == 0)	LED_OFF(DEBUG_LED_FLASH_GPIO_Port, DEBUG_LED_FLASH_Pin);
//  	uint8_t angle = 0;

    LoraTX.uart_handler=&huart1;
    LoraTX.M_GPIO_PORT=GPIOA;
    LoraTX.AUX_GPIO_PORT=GPIOA;
    LoraTX.M0_PIN=GPIO_PIN_2;
    LoraTX.M1_PIN=GPIO_PIN_3;
    LoraTX.MAUX_PIN=GPIO_PIN_8;
//
    LoraRX.uart_handler=&huart1;
    LoraRX.M_GPIO_PORT=GPIOA;
    LoraRX.AUX_GPIO_PORT=GPIOA;
    LoraRX.M0_PIN=GPIO_PIN_2;
    LoraRX.M1_PIN=GPIO_PIN_3;
    LoraRX.MAUX_PIN=GPIO_PIN_8;



    while (E220_enter_config_mode(&LoraTX)!=1);
    while (E220_enter_config_mode(&LoraRX)!=1);
//    E220_reset(&LoraRX);
//    E220_reset(&LoraTX);
//    E220_set_packetsize_32k(&LoraTX);
//    E220_set_packetsize_32k(&LoraRX);
//    E220_set_datarate_62k(&LoraRX);
//    E220_set_datarate_62k(&LoraTX);
//    Lora_result=E220_read_register_all(&LoraTX,all_reg_tx);
//    Lora_result=E220_read_register_all(&LoraRX,all_reg_rx);


    E220_enter_normal_mode(&LoraRX);
    E220_enter_normal_mode(&LoraTX);
    E220_read_register(&LoraTX,REG1);


	flight_state.flight_state = flight_phase;


	calibrateIMU(&imu_1, 1000, HWOFFSET);
//	calibrateIMU(&imu_1, 1, RESET_HWOFFSET);


	buzzerInit(&buzzer);
	servo_init(&servo);
	servo_rawmove(&servo, 2000);

//	note_t notes[20] = {
//			C1, G1, C2, E2,
//			D1, A1, D2, F2,
//			E1, B1, E2, G2,
//			F1, C2, F2, A2,
//			G1, D2, G2, B2
//	};

//	for (size_t i = 0; i < 20; i++) {
//		LED_ON(Status_LED_GPIO_Port, Status_LED_Pin);
//		beepBuzzer(&buzzer, 250, 10, notes[i]);
//		LED_OFF(Status_LED_GPIO_Port, Status_LED_Pin);
//		HAL_Delay(75);
//	}
//
//  	HAL_Delay(1000);
//
//  	servo_moveto_deg(&servo, 90); // Muovi servo a 90°
//
//  	HAL_Delay(1000);
//
//  	servo_moveto_deg(&servo, 135); // Muovi servo a 135°
	
//	beepBuzzer(&buzzer, 500, 100, C5);
//	HAL_Delay(500);
//	beepBuzzer(&buzzer, 1000, 100, C5);
//	HAL_Delay(500);
//	beepBuzzer(&buzzer, 1000, 100, C5);
//	HAL_Delay(500);
//	beepBuzzer(&buzzer, 1000, 100, C5);
//	HAL_Delay(500);


//
//	uint8_t buf[32] = {0};
//
//	result = W25Q128_read_data(&flash, 32, buf, 32);
//
//	float_t temperature = 0;
//	memcpy(&temperature, buf + 24, sizeof(float_t));
//
//	float_t pressure = 0;
//	memcpy(&pressure, buf + 28, sizeof(float_t));

  	HAL_Delay(1000);

  	servo_moveto_deg(&servo,180); //chiudere
//
  	HAL_Delay(1000);
////
  	servo_moveto_deg(&servo, 0); //aprire

//  	Flash_Write(0,(uint8_t*)"lopi",4);



//	char *data = "internal flash writing test\0";
//
//	uint8_t Rx_Data[30] = {0};
//
//	int numofwords = (strlen(data)/4)+((strlen(data)%4)!=0);
//	Flash_Write_Data(0x08008100 , (uint32_t *)data, numofwords);
//	Flash_Read_Data(0x08008100 , Rx_Data, numofwords);












//  uint16_t diff_pressure = 0;
//
//  uint8_t result = init_pitot_sensor(&pitot, &hadc1);
//
//  for (int i = 0; i < 100; i++) {
//
//	  result = read_diff_pressure(&pitot);
//
//	  diff_pressure = pitot.diff_pressure > diff_pressure ? pitot.diff_pressure : diff_pressure;
//
//	  HAL_Delay(10);
//
//  }
//
//  uint16_t velocity = compute_velocity(temperature,pression,diff_pressure); /* measurement unit m/s */













  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
//  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
//  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
//  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
//  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of StartupTask */
  StartupTaskHandle = osThreadNew(Startup, NULL, &StartupTask_attributes);

  /* creation of FlashWriteTask */
  FlashWriteTaskHandle = osThreadNew(FlashWrite, NULL, &FlashWriteTask_attributes);

  /* creation of SensorsReadTask */
  SensorsReadTaskHandle = osThreadNew(SensorsRead, NULL, &SensorsReadTask_attributes);

  /* creation of COMBoardTask */
  COMBoardTaskHandle = osThreadNew(CommunicationBoard, NULL, &COMBoardTask_attributes);

  /* creation of FlightFSMTask */
  FlightFSMTaskHandle = osThreadNew(FlightFSM, NULL, &FlightFSMTask_attributes);

  /* creation of SystemHealthCheckTask */
  SystemHealthCheckTaskHandle = osThreadNew(SystemHealthCheck, NULL, &SystemHealthCheckTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
//  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
//  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  rtU.Altitudeinput += 10.0;
	  rtU.Verticalvelocityinput = 200.0;
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  codegen_model_step();

	  output[i++] = rtY.Airbrakesextoutput * 100;

	  servo_moveto_deg(&servo, rtY.Airbrakesextoutput * 60);

	  HAL_Delay(10);
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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 96;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

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
  sConfig.Channel = ADC_CHANNEL_11;
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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
  htim3.Init.Period = 65535;
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  huart1.Init.BaudRate = 9600;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, E_Match_Parachute_1_Pin|E_Match_Parachute_2_Pin|IMU_1_nCS_Pin|BARO_1_nCS_Pin
                          |BARO_2_nCS_Pin|IMU_2_nCS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, M0_LORA_Pin|M1_LORA_Pin|FLASH_nCS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Channel1_PYRO_Pin|Channel2_PYRO_Pin|Status_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DEBUG_LED_FLASH_GPIO_Port, DEBUG_LED_FLASH_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : E_Match_Parachute_1_Pin E_Match_Parachute_2_Pin IMU_1_nCS_Pin BARO_1_nCS_Pin
                           BARO_2_nCS_Pin IMU_2_nCS_Pin */
  GPIO_InitStruct.Pin = E_Match_Parachute_1_Pin|E_Match_Parachute_2_Pin|IMU_1_nCS_Pin|BARO_1_nCS_Pin
                          |BARO_2_nCS_Pin|IMU_2_nCS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC15 PC0 PC2 PC3
                           PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : M0_LORA_Pin M1_LORA_Pin FLASH_nCS_Pin */
  GPIO_InitStruct.Pin = M0_LORA_Pin|M1_LORA_Pin|FLASH_nCS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB12 PB3 PB5
                           PB6 PB7 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_12|GPIO_PIN_3|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Channel1_PYRO_Pin Channel2_PYRO_Pin Status_LED_Pin */
  GPIO_InitStruct.Pin = Channel1_PYRO_Pin|Channel2_PYRO_Pin|Status_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LORA_AUX_Pin */
  GPIO_InitStruct.Pin = LORA_AUX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(LORA_AUX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DEBUG_LED_FLASH_Pin */
  GPIO_InitStruct.Pin = DEBUG_LED_FLASH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DEBUG_LED_FLASH_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_Startup */
/**
  * @brief  Function implementing the StartupTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Startup */
void Startup(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */

	/* Infinite loop */
	for(;;)
	{
		osDelay(1);
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_FlashWrite */
/**
* @brief Function implementing the FlashWriteTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_FlashWrite */
void FlashWrite(void *argument)
{
  /* USER CODE BEGIN FlashWrite */
	UBaseType_t uxHighWaterMark;
	uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );

	uint32_t tick;

	tick = osKernelGetTickCount();
  /* Infinite loop */
	for(;;)
	{
//		do {
//		if (num_meas_stored > (FLASH_NUMBER_OF_STORE_EACH_TIME - 1) && (flight_phase > READY)) {
		uint32_t num = Getnum_meas_stored_in_buffer();
		if (num > (FLASH_NUMBER_OF_STORE_EACH_TIME - 1))
		{
			Set_Flash_Flag(true);


			/* this value 7 depends on the number of measurements are performed
			* between each time the flash task is invoked. In this case it is 7
			* because flash task has a period of 80 ticks while sensor task of
			* 10 ticks therefore 80/10 = 8 measurements, so > 7, means that when
			* 8 measurements are performed we can enter. In general the value will
			* be (flash_task_period / sensor_task_period) - 1
			*/

//			size_t addr = (num_big_meas_stored_in_flash + num_meas_stored_in_buffer - FLASH_NUMBER_OF_STORE_EACH_TIME) * sizeof(sensor_data) + (num_small_meas_stored_in_flash + num_meas_stored_in_buffer_2 - FLASH_NUMBER_OF_STORE_EACH_TIME) * sizeof(sensor_data_2);
//			uint32_t addr_2 = 0;
			uint32_t num2 = 0;
			uint32_t num3 = 0;
			num2 = Getnum_big_meas_stored_in_flash();
			num3 = Getnum_small_meas_stored_in_flash();
			uint32_t addr = num2 * sizeof(sensor_data) + sizeof(sensor_data_2) * num3;
			//addr_2 = (size_t)4;//*num_times_lol_was_written;
			//uint8_t *testiamo;
			//sensor_data testiamo[FLASH_NUMBER_OF_STORE_EACH_TIME * 2];
			//testiamo = malloc(sizeof(sensor_data) * (FLASH_NUMBER_OF_STORE_EACH_TIME * 2));

//			uint8_t result = W25Q128_write_data(&flash, flash_address,(uint8_t*)measurements_buffer, 256);
			//uint8_t result2 = W25Q128_write_data(&flash, flash_address,(uint8_t*)measurements_buffer, 256);
			//uint8_t result2 = W25Q128_read_data(&flash,flash_address,testiamo,256);
//			float misura_princ[8][8];
//			for (int j = 0; j<8; j++){
//				for(int i = 0; i<8; i++){
//					misura_princ[j][i] = 0.0;
//				}
//			}
////
//				for (int j = 0; j<8; j++){
//					for(int i = 0; i<8; i++){
//						memcpy(misura_princ[j]+i,(uint8_t*)measurements_buffer + 32*j+i*4,4);
//				}
//			}
//			for (int j = 0; j<8; j++){
//				for(int i = 0; i<3; i++){
//					misura_sec[j][i] = 0.0;
//				}
//			}
////
//			for (int j = 0; j<8; j++){
//				for(int i = 0; i<3; i++){
//					memcpy(misura_sec[j]+i,(uint8_t*)measurements_buffer_2 + 12*j+i*4,4);
//				}
//			}
////			//  cazzi[0] = 2.0;
//			for(int j = 0; j<8; j++){
//				Flash_Write_float(addr_2+j*(sizeof(misura_princ[j])+sizeof(misura_sec[j])),misura_princ[j],sizeof(misura_princ[j]));
//				Flash_Write_float(addr_2+j*(sizeof(misura_princ[j])+sizeof(misura_sec[j]))+sizeof(misura_princ[j]),misura_sec[j],sizeof(misura_sec[j]));
//			}
			//static uint32_t addr = 0;

//			Flash_Write_float(flash_addr1, (float*)&measurements_buffer[0], 8*4);
//			Flash_Write_float(flash_addr2, (float*)&measurements_buffer_2[0], 3*4);
//			Flash_Write_float(flash_addr3, (float*)&measurements_buffer[1], 8*4);
//			Flash_Write_float(flash_addr4, (float*)&measurements_buffer_2[1], 3*4);
//			Flash_Write_float(flash_addr5, (float*)&measurements_buffer[2], 8*4);
//			Flash_Write_float(flash_addr6, (float*)&measurements_buffer_2[2], 3*4);
//			Flash_Write_float(flash_addr7, (float*)&measurements_buffer[3], 8*4);
//			Flash_Write_float(flash_addr8, (float*)&measurements_buffer_2[3], 3*4);
//			Flash_Write_float(flash_addr9, (float*)&measurements_buffer[4], 8*4);
//			Flash_Write_float(flash_addr10, (float*)&measurements_buffer_2[4], 3*4);
//			Flash_Write_float(flash_addr11, (float*)&measurements_buffer[5], 8*4);
//			Flash_Write_float(flash_addr12, (float*)&measurements_buffer_2[5], 3*4);
//			Flash_Write_float(flash_addr13, (float*)&measurements_buffer[6], 8*4);
//			Flash_Write_float(flash_addr14, (float*)&measurements_buffer_2[6], 3*4);
//			Flash_Write_float(flash_addr15, (float*)&measurements_buffer[7], 8*4);
//			Flash_Write_float(flash_addr16, (float*)&measurements_buffer_2[7], 3*4);

//			flash_addr1 += 352;
//			flash_addr2 += 352;
//			flash_addr3 += 352;
//			flash_addr4 += 352;
//			flash_addr5 += 352;
//			flash_addr6 += 352;
//			flash_addr7 += 352;
//			flash_addr8 += 352;
//			flash_addr9 += 352;
//			flash_addr10 += 352;
//			flash_addr11 += 352;
//			flash_addr12 += 352;
//			flash_addr13 += 352;
//			flash_addr14 += 352;
//			flash_addr15 += 352;
//			flash_addr16 += 352;

//			osDelay(10000);
//			addr_2 += 0xc;
//			osDelay(5000);
//			addr = addr_2;
			//SetAddr();
			for (int j = 0; j < 8; j++) {
//			    // Assuming sensor_data contains an array of 8 floats inside it.
//			    // Write 8 floats from the j-th object in measurements_buffer
			    Flash_Write_float(addr, (uint8_t*)&measurements_buffer[j], 8*4);  // Write 8 floats (32 bytes)
//
//
//			    // Assuming sensor_data_2 contains an array of 3 floats inside it.
//			    // Write 3 floats from the j-th object in measurements_buffer_2
			    addr = addr + 8*sizeof(float);
			    Flash_Write_float(addr, (uint8_t*)&measurements_buffer_2[j], 3*4);  // Write 3 floats (12 bytes)
//
//			    // Update the flash address to account for the data just written (32 + 12 = 44 bytes)
			    addr = addr + 3 * sizeof(float);
////			    addr += 10;
			}

//			float test[8][8];
//			for (int j = 0; j<8; j++){
//				for(int i = 0; i<8; i++){
//					test[j][i] = 0.0;
//				}
//			}
			//  test[0] = 0.0;
//			for(int j = 0; j<8 ; j++){
//				Flash_Read_float(addr+j*sizeof(misura_princ[j]),test[j],sizeof(misura_princ[j]));
//			}
//			cazzo = 0x0;
			//LETTURA
//			Flash_Write(addr,(uint8_t*)"lopi",(uint32_t)4);
//			num_times_lol_was_written++;
//			cazzo += 3;
			float r = (float)-8.0;
			Setnum_meas_stored_in_buffer(r);
//		    num_meas_stored_in_buffer -= FLASH_NUMBER_OF_STORE_EACH_TIME;
//			num_meas_stored_in_buffer_2 -= FLASH_NUMBER_OF_STORE_EACH_TIME;
			Setnum_meas_stored_in_flash();
			Set_Flash_Flag(false);


		}
//		} while ((num_meas_stored > (FLASH_NUMBER_OF_STORE_EACH_TIME - 1)));

		tick += FLASH_WRITE_TASK_PERIOD;
		uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
		osDelayUntil(tick);
	}
  /* USER CODE END FlashWrite */
}

/* USER CODE BEGIN Header_SensorsRead */
/**
* @brief Function implementing the SensorsReadTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SensorsRead */
void SensorsRead(void *argument)
{
  /* USER CODE BEGIN SensorsRead */
	UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );

	uint32_t tick;

	tick = osKernelGetTickCount();
  /* Infinite loop */
	for(;;)
	{
		bool flag = Get_Flash_Flag();
		uint32_t num = Getnum_meas_stored_in_buffer();
		if(!flag && num != 8){
			sensor_data data_1 = {0}, data_2 = {0};
			sensor_data_2 data_1_2 = {0};
			sensor_data_2 data_2_2 ={0};
			uint8_t result = 1;

			/* retrieving data from a couple of sensor and doing required conversions */
			result = bmp3_get_sensor_data(BMP3_PRESS_TEMP, &barometer_data_1, &bmp390_1);
			result = bmp3_get_sensor_data(BMP3_PRESS_TEMP, &barometer_data_2, &bmp390_2);

			result = lsm6dso32_angular_rate_raw_get(&imu_1, data_raw_angular_rate_1);
			result = lsm6dso32_acceleration_raw_get(&imu_1, data_raw_acceleration_1);
			result = lsm6dso32_angular_rate_raw_get(&imu_2, data_raw_angular_rate_2);
			result = lsm6dso32_acceleration_raw_get(&imu_2, data_raw_acceleration_2);

			angular_rate_mdps_1[0] = lsm6dso32_from_fs2000_to_mdps(data_raw_angular_rate_1[0]);
			angular_rate_mdps_1[1] = lsm6dso32_from_fs2000_to_mdps(data_raw_angular_rate_1[1]);
			angular_rate_mdps_1[2] = lsm6dso32_from_fs2000_to_mdps(data_raw_angular_rate_1[2]);
			angular_rate_mdps_2[0] = lsm6dso32_from_fs2000_to_mdps(data_raw_angular_rate_2[0]);
			angular_rate_mdps_2[1] = lsm6dso32_from_fs2000_to_mdps(data_raw_angular_rate_2[1]);
			angular_rate_mdps_2[2] = lsm6dso32_from_fs2000_to_mdps(data_raw_angular_rate_2[2]);

			acceleration_mg_1[0] = lsm6dso32_from_fs16_to_mg(data_raw_acceleration_1[0]);
			acceleration_mg_1[1] = lsm6dso32_from_fs16_to_mg(data_raw_acceleration_1[1]);
			acceleration_mg_1[2] = lsm6dso32_from_fs16_to_mg(data_raw_acceleration_1[2]);
			acceleration_mg_2[0] = lsm6dso32_from_fs16_to_mg(data_raw_acceleration_2[0]);
			acceleration_mg_2[1] = lsm6dso32_from_fs16_to_mg(data_raw_acceleration_2[1]);
			acceleration_mg_2[2] = lsm6dso32_from_fs16_to_mg(data_raw_acceleration_2[2]);

			/* storing measurements to sensor_data variable */

			data_1.acc_x = acceleration_mg_1[0];
			data_1.acc_y = acceleration_mg_1[1];
			data_1.acc_z = acceleration_mg_1[2];
			data_1.dps_x = angular_rate_mdps_1[0];
			data_1.dps_y = angular_rate_mdps_1[1];
			data_1.dps_z = angular_rate_mdps_1[2];
			data_1.temperature = barometer_data_1.temperature;
			data_1.pressure = barometer_data_1.pressure;

			data_2.acc_x = acceleration_mg_2[0];
			data_2.acc_y = acceleration_mg_2[1];
			data_2.acc_z = acceleration_mg_2[2];
			data_2.dps_x = angular_rate_mdps_2[0];
			data_2.dps_y = angular_rate_mdps_2[1];
			data_2.dps_z = angular_rate_mdps_2[2];
			data_2.temperature = barometer_data_2.temperature;
			data_2.pressure = barometer_data_2.pressure;

			prev_acc.accX = curr_acc.accX;
			prev_acc.accY = curr_acc.accY;
			prev_acc.accZ = curr_acc.accZ;

			curr_acc.accX = acceleration_mg_1[0];
			curr_acc.accY = acceleration_mg_1[1];
			curr_acc.accZ = acceleration_mg_1[2];

			if(first_measure){
				Pressure_1 = data_1.pressure;
				Pressure_2 = data_2.pressure;
				first_measure = false;
			}

			altitude = readAltitude(Pressure_1,data_1.pressure);
			data_1_2.altitude = altitude;
			altitude = readAltitude(Pressure_2,data_2.pressure);
			data_2_2.altitude = altitude;



			switch (flight_state.flight_state){

				case CALIBRATING:
					data_1_2.phase = 1.0;
					data_2_2.phase = 1.0;
					break;
				case READY:
					data_1_2.phase = 2.0;
					data_2_2.phase = 2.0;
					break;
				case BURNING:
					data_1_2.phase = 3.0;
					data_2_2.phase = 3.0;
					break;
				case COASTING:
					data_1_2.phase = 4.0;
					data_2_2.phase = 4.0;
					break;
				case DROGUE:
					data_1_2.phase = 5.0;
					data_2_2.phase = 5.0;
				case MAIN:
					data_1_2.phase = 6.0;
					data_2_2.phase = 6.0;
					break;
				case TOUCHDOWN:
					data_1_2.phase = 7.0;
					data_2_2.phase = 7.0;
					break;

				case INVALID:
					break;
			}

			/* computing offset of the buffer and storing the data to the buffer */


			size_t offset = num * sizeof(sensor_data);
			size_t offset_2 = num * sizeof(sensor_data_2);

			memcpy((uint8_t*)measurements_buffer + offset, &data_2, sizeof(sensor_data));
			memcpy((uint8_t*)measurements_buffer_2 + offset_2, &data_2_2, sizeof(sensor_data_2));
	//		sensor_data_2 buffer_data_2;
	//        sensor_data buffer_data;
	//		memcpy(&buffer_data,(uint8_t*)measurements_buffer + offset,sizeof(sensor_data));
	//		memcpy(&buffer_data_2,(uint8_t*)measurements_buffer_2 + offset_2, sizeof(sensor_data_2));




			/*
			 * increment of the number of stored measurements and
			 * modulo operation to avoid buffer overflow in the offset computation
			 */
			/*
			 * During READY phase data are not saved in the flash, therefore the data
			 * are always overwritten to the previous ones
			 */
			Setnum_meas_stored_in_buffermod();
			float r = 1.0;
			Setnum_meas_stored_in_buffer(r);
		}





		tick += SENSORS_TASK_PERIOD;
		uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
		osDelayUntil(tick);
	}
  /* USER CODE END SensorsRead */
}

/* USER CODE BEGIN Header_CommunicationBoard */
/**
* @brief Function implementing the COMBoardTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CommunicationBoard */
void CommunicationBoard(void *argument)
{
  /* USER CODE BEGIN CommunicationBoard */
	UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );

	uint32_t tick;

	tick = osKernelGetTickCount();
	Lora_Package package;
	uint8_t array[45];
	  /* Infinite loop */
	for(;;)
	{
		for(uint8_t i=0; i<45; i++){
			array[i] = i;
		}
		package.carattere = 'D';
	    package.acc_x = data_raw_acceleration_1[0];
	    package.acc_y = data_raw_acceleration_1[1];
	    package.acc_z = data_raw_acceleration_1[2];
	    package.dps_x = data_raw_angular_rate_1[0];
	    package.dps_y = data_raw_angular_rate_1[1];
	    package.dps_z = data_raw_angular_rate_1[2];
	    package.temperature = barometer_data_1.temperature;
	    package.pressure = barometer_data_1.pressure;
	    package.altitude =  altitude;
	    package.velocity = 0;
	    switch (flight_state.flight_state){
			case INVALID:
				package.phase = 2.0;
				break;
			case CALIBRATING:
				package.phase = 3.0;
				break;
			case READY:
				package.phase = 2.0;
				break;
			case BURNING:
				package.phase = 3.0;
				break;
			case COASTING:
				package.phase = 4.0;
				break;
			case DROGUE:
				package.phase = 5.0;
				break;
			case MAIN:
				package.phase = 6.0;
				break;
			case TOUCHDOWN:
				package.phase = 7.0;
				break;
	    }


		int tx_status = E220_transmit_payload(&LoraTX,"test", 4);
		if (tx_status != 1) {
		    printf("Transmission failed with status: %d\n", tx_status);
		}
//		if(HAL_UART_Transmit(&huart1, (uint8_t *)"test", 4, 100)!= HAL_OK){
//			printf("Transmission failed with status: %d\n", tx_status);
//		}
//		printf("CAZZI", tx_status);
		osDelay(2000);

		int8_t rssi = E220_receive_payload(&LoraRX,receive_buffer,sizeof(receive_buffer)); //RSSI

		tick += COMMUNICATION_BOARD_TASK_PERIOD;
		uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
		osDelayUntil(tick);
	}
  /* USER CODE END CommunicationBoard */
}

/* USER CODE BEGIN Header_FlightFSM */
/**
* @brief Function implementing the FlightFSMTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_FlightFSM */
void FlightFSM(void *argument)
{
  /* USER CODE BEGIN FlightFSM */

	UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );

	uint32_t tick;

	tick = osKernelGetTickCount();
	/* Infinite loop */
	for(;;)
	{
		linear_acceleration_t acc1;
		acc1.accX = acceleration_mg_1[0];
		acc1.accY = acceleration_mg_1[1];
		acc1.accZ = acceleration_mg_1[2];

		linear_acceleration_t acc2;
		acc2.accX = angular_rate_mdps_1[0];
		acc2.accY = angular_rate_mdps_1[1];
		acc2.accZ = angular_rate_mdps_1[2];

		estimation_output_t MotionData;
		MotionData.acceleration = sqrt(acceleration_mg_1[0]*acceleration_mg_1[0] + acceleration_mg_1[1]*acceleration_mg_1[1] + acceleration_mg_1[2]*acceleration_mg_1[2]);
		MotionData.height = altitude;

		/* Check Flight Phases */
		check_flight_phase(&flight_state,MotionData, acc1, acc2);



		tick += FLIGHT_FSM_TASK_PERIOD;
		uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
		osDelayUntil(tick);
	}
  /* USER CODE END FlightFSM */
}

/* USER CODE BEGIN Header_SystemHealthCheck */
/**
* @brief Function implementing the SystemHealthCheckTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SystemHealthCheck */
void SystemHealthCheck(void *argument)
{
  /* USER CODE BEGIN SystemHealthCheck */
	UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );

	uint32_t tick;

	tick = osKernelGetTickCount();
	/* Infinite loop */
	for(;;)
	{


		tick += FLIGHT_HEALTH_CHECK_TASK_PERIOD;
		uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
		osDelayUntil(tick);
	}
  /* USER CODE END SystemHealthCheck */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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
