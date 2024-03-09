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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "BMP180.h"
#include "ssd1306.h"				// simdilik kullanilmadi
#include "ssd1306_fonts.h"			// simdilik kullanilmadi
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define x_address		(0x29)		// LIS sensorunun x registeri
#define y_address		(0x2B)		// LIS sensorunun y registeri
#define z_address		(0x2D)		// LIS sensorunun z registeri
#define LIS_WHO_AM_I	(0x0F)		// LIS sensorunun sorgulama registeri
#define CTRL_REG4		(0x20)		// LIS sensorunun control registeri
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

									/* TASK, QUEUE, SEMAPHORE (Mutex) icin handle*/
osThreadId Menu_TaskHandle;
osThreadId myTask02Handle;
osThreadId WriteSensorDataHandle;
osMessageQId MenuQueueHandle;
osMessageQId SensorDataQueueHandle;
osMutexId MutexForUartHandle;
/* USER CODE BEGIN PV */

								/* FOR STM32F407 DISCOVERY */
								/*      DOGUKAN AKCA	   */

uint8_t rx_Usart[10];			// UART kesmesinden gelen verinin tutulacagi dizi
uint8_t rx_Data[10];			// UART kesmesinden gelen verinin karakter karakter yazilacagi dizi
uint8_t rx_count = 0;			// Diziye yazilacak karakterleri kontrol edecek sayac degiskeni

								/* Seri ekrana basilacak menu dizisi*/
char menuDisplay[] = {"\r\n USER LEDS ON 	    --> 1 \
					   \r\n USER LEDS OFF 	    --> 2 \
					   \r\n READ LIS3DSH DATA 	    --> 3 \
					   \r\n READ BMP180 DATA	    --> 4 \
					   \r\n\n Choose you option here : "};

char *firstMessage = "Hello, press ENTER to start the system!";		// Giris mesaji
int typeOfMsg = 0;													// queue kontrolu icin tanimlandi

struct xMessage							// sensor verilerinin queue uzerinden gonderilmesi icin olustrulan struct yapisi
{
	char ucMessageID;
	int8_t sensorData[3];
} xMessage_t;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
void MenuTask(void const * argument);
void StartTask02(void const * argument);
void WriteSensorDataTask(void const * argument);

/* USER CODE BEGIN PFP */
void PrintMessageToDisplay(char *message);									// seri ekrana veri yazacak fonksiyon
int8_t ReadSpi(uint8_t addres, SPI_HandleTypeDef *spiX);					// LIS sensorunden veri okumak icin fonksiyon
void SensorInit(void);														// sensor baslatma
void WriteSpi(uint8_t addres, uint8_t data, SPI_HandleTypeDef *spiX);		// LIS sensorune veri yazmak icin fonksiyon
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  SensorInit();

  HAL_UART_Receive_IT(&huart2, rx_Usart, 1);			// UART kesmesi icin baslatma
  PrintMessageToDisplay(firstMessage);					// seri ekrana ilk mesaji yaz

  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of MutexForUart */
  osMutexDef(MutexForUart);										// mutex olusturma
  MutexForUartHandle = osMutexCreate(osMutex(MutexForUart));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of MenuQueue */
  osMessageQDef(MenuQueue, 2, uint16_t);						// seri ekrana veri yazmak icin kullanılan kuyrugu olusturma
  MenuQueueHandle = osMessageCreate(osMessageQ(MenuQueue), NULL);

  /* definition and creation of SensorDataQueue */
  osMessageQDef(SensorDataQueue, 3, sizeof(struct xMessage*));  // sensor verilerini yazmak ve okumak icin kuyruk olusturma
  SensorDataQueueHandle = osMessageCreate(osMessageQ(SensorDataQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of Menu_Task */
  osThreadDef(Menu_Task, MenuTask, osPriorityNormal, 0, 128);			// TASK'larin olusturulmasi
  Menu_TaskHandle = osThreadCreate(osThread(Menu_Task), NULL);

  /* definition and creation of myTask02 */
  osThreadDef(myTask02, StartTask02, osPriorityNormal, 0, 128);
  myTask02Handle = osThreadCreate(osThread(myTask02), NULL);

  /* definition and creation of WriteSensorData */
  osThreadDef(WriteSensorData, WriteSensorDataTask, osPriorityAboveNormal, 0, 128);
  WriteSensorDataHandle = osThreadCreate(osThread(WriteSensorData), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();					// Zamanlayiciyi baslatma

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void vApplicationIdleHook( void )						// IDLE task'i ekledim
{
	__WFI();											// uyku moduna alir
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart2)
	{
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;			// baglam degisikligi icin gerekli TASK durum degiskeni

		rx_Data[rx_count++] = rx_Usart[0];			// Seri ekrandan gelen her karakteri rx_data dizisine kaydeder
		if(rx_Data[rx_count-1] == '\n')				// ENTER'a basildi mi?
		{
			rx_count = 0;
			xQueueSendFromISR(MenuQueueHandle, rx_Data, &xHigherPriorityTaskWoken);				// kuyruk dolunca Task02'nin engelini kaldiracaktir
			xTaskNotifyFromISR(Menu_TaskHandle, 0, eNoAction, &xHigherPriorityTaskWoken);		// Menu_Task icin bildirim gonderir ve engelini kaldırarak calismasini saglar
		}

		__HAL_UART_CLEAR_FLAG(&huart2, UART_FLAG_RXNE);			// UART bayragini temizle
		HAL_UART_Receive_IT(&huart2, rx_Usart, 1);				// Tekrardan kesmeye girebilmek icin aktif ettik

		if(xHigherPriorityTaskWoken == pdTRUE)					// Eger bildirim sayesinde daha yuksek oncelikli bir Task varsa baglam degisikligi yap
		{
				portYIELD_FROM_ISR(xHigherPriorityTaskWoken);	// Baglam degisikligi
		}
	}
}

void PrintMessageToDisplay(char *message)						// seri ekrana veri yazmak icin fonksiyon
{
	while(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TXE) != SET);				// UART gonderim bufferi bos mu? kontrol et
	HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), 500);	// veriyi seri ekrana yaz
}

void SensorInit(void)
{
	ReadSpi(LIS_WHO_AM_I, &hspi1);			// LIS sensor baslatma
	HAL_Delay(100);							// bekleme suresi
	WriteSpi(CTRL_REG4, 0xE7, &hspi1);		// Veri cikis araliği 12Hz, X,Y ve Z ekseni aktif
	HAL_Delay(40);							// bekleme suresi
	BMP180_Start();							// BMP sensor baslatma
	HAL_Delay(40);							// bekleme suresi
}

int8_t ReadSpi(uint8_t addres, SPI_HandleTypeDef *spiX)				// LIS sensorunden okuma yapmak icin fonksiyon
{

	int8_t buffer;

	if(HAL_SPI_GetState(spiX) == HAL_SPI_STATE_READY)
	{
		addres = addres | 0x80;							// addresin basina 1 yazarak okuma yapcagimizi soyluyoruz

		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);		// sensorun CS pinini resetle (sensorle haberlesmeyi baslat)

		HAL_SPI_Transmit(spiX, &addres, 1, 10);

		HAL_SPI_Receive(spiX, &buffer, 1, 10);			// alinan veriyi buffer'a koy

		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);			// sensorun CS pinini setle (sensorle haberlesmeyi durdur)
	}

	return buffer;

}

void WriteSpi(uint8_t addres, uint8_t data, SPI_HandleTypeDef *spiX)
{

	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);		//sensorun CS pinini resetle (sensorle haberlesmeyi baslat)

	HAL_SPI_Transmit(spiX, &addres, 1, 100);

	HAL_SPI_Transmit(spiX, &data, 1, 100);

	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);			// sensorun CS pinini setle (sensorle haberlesmeyi durdur)

}


/* USER CODE END 4 */

/* USER CODE BEGIN Header_MenuTask */
/**
  * @brief  Function implementing the Menu_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_MenuTask */
void MenuTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
	//char *display = menuDisplay;
  /* Infinite loop */
  for(;;)
  {
    if(xTaskNotifyWait(0, 0, NULL, portMAX_DELAY) == pdTRUE)		// UART'dan kesme bekle (ENTER'a basilirsa bu taskin engeli kalkar)
    {
    	xSemaphoreTake(MutexForUartHandle, portMAX_DELAY);			// Bu task ile StartTask aynı seri ekrana veri yazmak icin ikiside UART2'yi kullanmaktadir
    																// fakat ayni anda sadece biri kullanabilmektedir. Aynı anda kullanmlarini engellemek icin
    	PrintMessageToDisplay(menuDisplay);							// Mutex kullaniyoruz boylece iki gorevde ayni anda UART2'yi kullanamayacak.
    	xSemaphoreGive(MutexForUartHandle);							// Mutexi ilk alan UART2'yi kullanacak daha sonra mutexi birakacak ve digeri mutexi alarak UART2'yi kullanacak.
    }
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void const * argument)			// USART'dan (seri ekran) gelen verileri isleyecek TASK
{
  /* USER CODE BEGIN StartTask02 */
	uint8_t rx_que[2];							// USART2'ye gelen verinin bufferlanacagi dizi
	struct xMessage *pxTXMessage;				// sensor verilerini kuyruga yazmak icin olusturulan struct yapisi
  /* Infinite loop */
  for(;;)
  {
	  pxTXMessage = &xMessage_t;
	  xQueueReceive(MenuQueueHandle, rx_que, portMAX_DELAY);			// Seri ekrana yazilan veriyi al
	  if(rx_que[0] == '1' && rx_que[1] == '\n')							// Veriye gore kontrol yap
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_All, GPIO_PIN_SET);			// Tum kullanici ledlerini yak
	  else if(rx_que[0] == '2' && rx_que[1] == '\n')
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_All, GPIO_PIN_RESET);		// Tum kullanici ledlerini sondur
	  else if(rx_que[0] == '3' && rx_que[1] == '\n')
	  {
		  typeOfMsg = 1;
		  pxTXMessage->sensorData[0] = ReadSpi(x_address, &hspi1);		// LIS sensor verilerini struct'a yaz
		  pxTXMessage->sensorData[1] = ReadSpi(y_address, &hspi1);
		  pxTXMessage->sensorData[2] = ReadSpi(z_address, &hspi1);

		  xQueueSend(SensorDataQueueHandle,&pxTXMessage,portMAX_DELAY);	// Struct'a yazilan verileri kuyruga yaz
	  }
	  else if(rx_que[0] == '4' && rx_que[1] == '\n')
	  {
		  typeOfMsg = 2;
		  pxTXMessage->sensorData[0] 	= BMP180_GetTemp();				// BMP sensor verilerini struct'a yaz
		  pxTXMessage->sensorData[1] 	= BMP180_GetPress (0);
		  pxTXMessage->sensorData[2] 	= BMP180_GetAlt(0);

		  xQueueSend(SensorDataQueueHandle,&pxTXMessage,portMAX_DELAY);	// Struct'a yazilan verileri kuyruga yaz
	  }
	  else
	  {
		  PrintMessageToDisplay("\n\nPLEASE TRY AGAIN!!!\n\n");
	  }
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_WriteSensorDataTask */
/**
* @brief Function implementing the WriteSensorData thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_WriteSensorDataTask */
void WriteSensorDataTask(void const * argument)				// sensor verilerini kuyruktan okuyup seri ekrana yazacak TASK
{
  /* USER CODE BEGIN WriteSensorDataTask */
	char Data[50];											// kuyruktan struct'a alinan verileri buraya kopyaliyorum
	struct xMessage *pxRXMessage;							// kuyruktan okunacak verileri aynı tip struct'a alinmasi gerekli!
  /* Infinite loop */
  for(;;)
  {
    xQueueReceive(SensorDataQueueHandle, &pxRXMessage, portMAX_DELAY);		// kuyruk doldugunda bu task'in engeli kaldirilacak ve calismaya basliyacak

    if(typeOfMsg == 1)														// LIS sensor icin seri ekran yazisi
    	sprintf(Data, "\r\n\nX axis : %d\nY axis : %d\nZ axis : %d\n\n", pxRXMessage->sensorData[0], pxRXMessage->sensorData[1], pxRXMessage->sensorData[2]);

    else if(typeOfMsg == 2)													// BMP sensor icin seri ekran yazisi
    	sprintf(Data, "\r\n\nTemperature : %d\nPressure : %d\nAltitude : %d\n\n", pxRXMessage->sensorData[0], pxRXMessage->sensorData[1], pxRXMessage->sensorData[2]);

    while(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TXE) != SET);				// UART gonderim bufferi bos mu? kontrol et

    xSemaphoreTake(MutexForUartHandle, portMAX_DELAY);						// MenuTask'i icinde anlattigim mutex yapisi!
    HAL_UART_Transmit(&huart2, (uint8_t*)Data, strlen(Data), 500);
    xSemaphoreGive(MutexForUartHandle);

  }
  /* USER CODE END WriteSensorDataTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
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
