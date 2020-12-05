/**
  ******************************************************************************
  * @file    Demonstrations/Src/main.c 
  * @author  MCD Application Team
  * @version V1.3.0
  * @date    01-July-2015
  * @brief   This demo describes how to use accelerometer to control mouse on 
  *          PC.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/** @addtogroup STM32F4xx_HAL_Demonstrations
  * @{
  */

/** @addtogroup Demo
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
typedef unsigned char           bool; //!< Boolean.
/* Private define ------------------------------------------------------------*/
#define false     																											0
#define true      																											1
	
#define I2C_ACC_ADDRESS_BMA280											0x30
#define I2C_GYR_ADDRESS_BMG160											0xD0
#define I2C_MAG_ADDRESS_MXG3300										0x18
	
/* WIFI STATUS */
#define WIFI_NO_OPR																						0
#define WIFI_ERROR_S																					15
#define WIFI_ERROR_RE																				16
#define WIFI_BAUD_CHG																				1
#define WIFI_CFG_START																				2
#define WIFI_CFG_END																						3
#define WIFI_CFG_NEWTWORK_START									4
#define WIFI_CFG_NEWTWORK_END											5
#define WIFI_CFG_WMODE_START												6
#define WIFI_CFG_WMODE_END														7
#define WIFI_CREATE_LIMITEDAP_START							8
#define WIFI_CREATE_LIMITEDAP_END									9
#define WIFI_ENABLE_DHCP_SER_START							10
#define WIFI_ENABLE_DHCP_SER_END									11
#define WIFI_UDP_LP_SET_START													12 
#define WIFI_TCP_LP_SET_START													13 
#define WIFI_TCP_LP_SET_END															14 
#define WIFI_UDP_LP_SET_END															15 
#define WIFI_UDP_START																			16

#define WIFI_BUF_MAX																						256
#define OK_MSG_LENGTH																			2
#define CONNECT_MSG_LENGTH													7
#define DISCONNECT_MSG_LENGTH											10


#define USB_TX_BUFFER_SIZE 100//20160215
#define USB_RX_BUFFER_SIZE 100//20160215

/* Private macro -------------------------------------------------------------*/
#define ABS(x)         (x < 0) ? (-x) : x
#define MAX_AB(a,b)       (a < b) ? (b) : a

/* Private variables ---------------------------------------------------------*/
/* UART handler declaration */
UART_HandleTypeDef UartHandle;
__IO ITStatus UartReady = RESET;

/* TIM handler declaration */
TIM_HandleTypeDef    TimHandle;
TIM_OC_InitTypeDef   sConfig;

/* I2C handler declaration */
I2C_HandleTypeDef I2cHandle;

/* SPI handler declaration */
SPI_HandleTypeDef SpiHandle;

uint32_t uwPrescalerValue = 0;
uint32_t uwCapturedValue = 0;

/* flag to use wifi status */
volatile static uint8_t wifi_opr_mode=WIFI_NO_OPR;
volatile static uint8_t wifi_http_mode=0;
uint8_t wifi_buffer[WIFI_BUF_MAX]={0};

volatile static bool wifi_msg_updated=false;
volatile static bool wifi_Client_msg_updated=false;

volatile bool bServerSend=false;																	//Send or Stop
volatile bool bSelectAccel=false;																	//Send or Stop
volatile bool bSelectGyro=false;																		//Send or Stop
volatile bool bSelectMagnetic=false;														//Send or Stop

volatile int nClientPort=0;
volatile int nPortIndex=0;						
volatile int nMsgDataStartIndex=0;
volatile int nMsgDataEndIndex=0;
volatile int nIPStartIndex=0;
volatile int nIPEndIndex=0;

const char ok_msg[]="OK";
const char connect_msg[]="CONNECT";
const char disconnect_msg[]="DISCONNECT";


//uint8_t tx_buffer_wifi_cfg_start[15]="AT+BDATA=1\r\n";
//uint8_t tx_buffer_wifi_cfg_network_start[16]="AT+NSTCP=48596\r\n";
uint8_t tx_buffer_wifi_cfg_start[7]="AT+WD\r\n";//disassociate from the current network
uint8_t tx_buffer_wifi_cfg_network_start[12]="AT+NDHCP=1\r\n";//enable DHCP
uint8_t tx_buffer_wifi_cfg_wmode_start[49]="AT+NSET=192.168.1.73,255.255.255.0,192.168.1.73\r\n";//configure network parameter set
uint8_t tx_buffer_wifi_create_limitedap_start[9]="AT+WM=2\r\n";//configure wireless mode to limited AP set
uint8_t tx_buffer_wifi_enable_dhcp_ser_start[19]="AT+WA=SE88_AP,,11\r\n";//associate to an access point name set
uint8_t tx_buffer_wifi_udp_lp_set_start[15]="AT+DHCPSRVR=1\r\n";//enable DHCP server
uint8_t tx_buffer_wifi_tcp_lp_set_start[15]="AT+BDATA=1\r\n";
uint8_t tx_buffer_wifi_bulk_data_set[16]="AT+NSTCP=48588\r\n"; //start TCP Server port set
//uint8_t tx_buffer_wifi_dns_data_set[22]="AT+WEBPROV=admin,admin";

uint8_t tx_buffer_wifi_http1_data_set[130]="AT+HTTPCONF=20,Mozilla/5.0 (Windows; U; Windows NT 5.1; en-US) AppleWebKit/534.7(KHTML, like Gecko) Chrome/7.0.517.44 Safari/534.7";
uint8_t tx_buffer_wifi_http2_data_set[47]="AT+HTTPCONF=7,application/x-www-form-urlencoded";
uint8_t tx_buffer_wifi_http3_data_set[27]="AT+HTTPCONF=11,192.168.1.73";
uint8_t tx_buffer_wifi_http4_data_set[24]="AT+HTTPCONF=3,keep-alive";
uint8_t tx_buffer_wifi_http5_data_set[27]="AT+HTTPOPEN=192.168.1.73,80";
uint8_t tx_buffer_wifi_http6_data_set[42]="AT+HTTPSEND=0,1,10,/gswebserver/index.html";

/*uint8_t tx_buffer_wifi_cfg_start[16]="AT+WRXACTIVE=1\r\n";
uint8_t tx_buffer_wifi_cfg_network_start[49]="AT+NSET=192.168.1.96,255.255.255.0,192.168.1.96\r\n";
uint8_t tx_buffer_wifi_cfg_wmode_start[9]="AT+WM=2\r\n";
uint8_t tx_buffer_wifi_create_limitedap_start[19]="AT+WA=SE96_AP,,11\r\n";
uint8_t tx_buffer_wifi_enable_dhcp_ser_start[15]="AT+DHCPSRVR=1\r\n";
uint8_t tx_buffer_wifi_udp_lp_set_start[16]="AT+NSUDP=48596\r\n";
uint8_t tx_buffer_wifi_bulk_data_set[6]="ATE0\r\n";*/

uint8_t i2c_rd_data;
uint8_t i2c_rd_addr;
uint8_t i2c_tx_data;
uint8_t i2c_tx_addr;
uint8_t i2c_rd_array[6];

uint8_t usbCdcTxBuffer[USB_TX_BUFFER_SIZE];//20160215
uint8_t usbCdcRxBuffer[USB_RX_BUFFER_SIZE];//20160215

/* Variables used for USB */
USBD_HandleTypeDef  hUSBDDevice;

/* Buffer used for transmission */
uint8_t aTxBuffer[] = "****SPI - Two Boards communication based on Interrupt **** SPI Message ******** SPI Message ******** SPI Message ****\r\n";

/* Buffer used for reception */
uint8_t aRxBuffer[BUFFERSIZE];

uint8_t testInt = 0;


/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);
static void EXTI_ACC_GYR_MAG_Config(void);//20160215

/* Private functions ---------------------------------------------------------*/
void wifi_module_status_check(void);
void wifi_buffer_update_check(void);
void wifi_buffer_clear(void);
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
 

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
	char szBuffer[128]={0};
	
  /* STM32F4xx HAL library initialization:
       - Configure the Flash prefetch, instruction and Data caches
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Global MSP (MCU Support Package) initialization
     */
  HAL_Init();
  
  /* Configure LED0, LED1, LED2 */
  BSP_LED_Init(LED0);
  BSP_LED_Init(LED1);
  BSP_LED_Init(LED2);

  /* Configure the system clock to 84 MHz */
  SystemClock_Config();

 /*##-1- Configure the TIM peripheral #######################################*/ 
  /* -----------------------------------------------------------------------
    In this example TIM3 input clock (TIM3CLK) is set to 2 * APB1 clock (PCLK1), 
    since APB1 prescaler is different from 1.   
      TIM3CLK = 2 * PCLK1  
      PCLK1 = HCLK / 2 
      => TIM3CLK = HCLK = SystemCoreClock
    To get TIM3 counter clock at 10 KHz, the Prescaler is computed as following:
    Prescaler = (TIM3CLK / TIM3 counter clock) - 1
    Prescaler = (SystemCoreClock /10 KHz) - 1
       
    Note: 
     SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f4xx.c file.
     Each time the core clock (HCLK) changes, user had to update SystemCoreClock 
     variable value. Otherwise, any configuration based on this variable will be incorrect.
     This variable is updated in three ways:
      1) by calling CMSIS function SystemCoreClockUpdate()
      2) by calling HAL API function HAL_RCC_GetSysClockFreq()
      3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency  
  ----------------------------------------------------------------------- */ 
		
  /* Compute the prescaler value to have TIM3 counter clock equal to 10 KHz */
  uwPrescalerValue = (uint32_t) ((SystemCoreClock / 10000) - 1);
  
  /* Set TIMx instance */
  TimHandle.Instance = TIMx;
   
  /* Initialize TIM3 peripheral as follow:
       + Period = 10000 - 1
       + Prescaler = ((SystemCoreClock/2)/10000) - 1
       + ClockDivision = 0
       + Counter direction = Up
  */
  TimHandle.Init.Period = 10000 - 1;
  TimHandle.Init.Prescaler = uwPrescalerValue;
  TimHandle.Init.ClockDivision = 0;
  TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
	
  if(HAL_TIM_Base_Init(&TimHandle) != HAL_OK)
  {
			/* Initialization Error */
			Error_Handler();
  }
	
  /*##-2- Start the TIM Base generation in interrupt mode ####################*/
  /* Start Channel1 */
  if(HAL_TIM_Base_Start_IT(&TimHandle) != HAL_OK)
  {
			/* Starting Error */
			Error_Handler();
  }

	  /*##-1- Configure the I2C peripheral ######################################*/
  I2cHandle.Instance             = I2Cx;
  
  I2cHandle.Init.AddressingMode  	= I2C_ADDRESSINGMODE_7BIT;
  I2cHandle.Init.ClockSpeed      			= 400000;
  I2cHandle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  I2cHandle.Init.DutyCycle       				= I2C_DUTYCYCLE_16_9;
  I2cHandle.Init.GeneralCallMode 	= I2C_GENERALCALL_DISABLE;
  I2cHandle.Init.NoStretchMode   		= I2C_NOSTRETCH_DISABLE;
  I2cHandle.Init.OwnAddress1     		= 0;
  I2cHandle.Init.OwnAddress2     		= 0;
  
  if(HAL_I2C_Init(&I2cHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();    
  }
	
#if 1	//20160215
	/* I2C read test */
	i2c_rd_addr=0x00;
	HAL_I2C_Mem_Read(&I2cHandle, I2C_ACC_ADDRESS_BMA280, i2c_rd_addr, I2C_MEMADD_SIZE_8BIT, &i2c_rd_data, sizeof(i2c_rd_data), I2C_FLAG_TIMEOUT);
	
	if(i2c_rd_data==0x0F)
	{
			BSP_LED_Toggle(LED2);
	}
	#endif
	
	
	  /*##-1- Configure the SPI peripheral #######################################*/
  /* Set the SPI parameters */
  SpiHandle.Instance               = SPIx;
  SpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  SpiHandle.Init.Direction         = SPI_DIRECTION_2LINES;
  SpiHandle.Init.CLKPhase          = SPI_PHASE_1EDGE;
  SpiHandle.Init.CLKPolarity       = SPI_POLARITY_HIGH;
  SpiHandle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  SpiHandle.Init.CRCPolynomial     = 7;
  SpiHandle.Init.DataSize          = SPI_DATASIZE_8BIT;
  SpiHandle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  SpiHandle.Init.NSS               = SPI_NSS_SOFT;
  SpiHandle.Init.TIMode            = SPI_TIMODE_DISABLE;
  SpiHandle.Init.Mode = SPI_MODE_MASTER; //SPI_MODE_SLAVE;
	
  if(HAL_SPI_Init(&SpiHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
	
  if(HAL_SPI_TransmitReceive_IT(&SpiHandle, (uint8_t *)aTxBuffer, (uint8_t *)aRxBuffer, BUFFERSIZE) != HAL_OK)//SPIx_TIMEOUT_MAX
  {
    /* Transfer error in transmission process */
    Error_Handler();
  }
	
	
	/*##-1- Configure the UART peripheral ######################################*/
  /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
  /* UART2 configured as follow:
      - Word Length = 8 Bits
      - Stop Bit = One Stop bit
      - Parity = None
      - BaudRate = 115200 baud
      - Hardware flow control disabled (RTS and CTS signals) */
  UartHandle.Instance          					= USARTx;
  
	UartHandle.Init.BaudRate     				= 115200;
  UartHandle.Init.WordLength   				= UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits     						= UART_STOPBITS_1;
  UartHandle.Init.Parity       							= UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl    					= UART_HWCONTROL_NONE;
  UartHandle.Init.Mode         						= UART_MODE_TX_RX;
  UartHandle.Init.OverSampling 			= UART_OVERSAMPLING_16;
    
  if(HAL_UART_Init(&UartHandle) != HAL_OK)
  {
			Error_Handler();
  }

  if(HAL_UART_Receive_IT(&UartHandle, (uint8_t *)wifi_buffer, sizeof(wifi_buffer)) != HAL_OK)
  {
			Error_Handler();
  }  
	
  /* Init Device Library */
  USBD_Init(&hUSBDDevice, &FS_Desc, 0);
	
  /* Add Supported Class */
  USBD_RegisterClass(&hUSBDDevice, USBD_CDC_CLASS);
	
	USBD_CDC_RegisterInterface(&hUSBDDevice, &USBD_CDC_Template_fops);//20160215
  
  /* Start Device Process */
  USBD_Start(&hUSBDDevice);
	
	EXTI_ACC_GYR_MAG_Config();//20160215
	
	#if 0 //20160215
	i2c_tx_addr=0x10;	/* PMU_BW */
	i2c_tx_data=0x08;	/* 7.81Hz */
	
	HAL_I2C_Mem_Write(&I2cHandle, I2C_ACC_ADDRESS_BMA280, i2c_tx_addr, I2C_MEMADD_SIZE_8BIT, &i2c_tx_data, sizeof(i2c_tx_data), I2C_FLAG_TIMEOUT);

	i2c_tx_addr=0x1A;	/* INT_MAP_1 */
	i2c_tx_data=0x01;	/* data ready interrupt */
	
	HAL_I2C_Mem_Write(&I2cHandle, I2C_ACC_ADDRESS_BMA280, i2c_tx_addr, I2C_MEMADD_SIZE_8BIT, &i2c_tx_data, sizeof(i2c_tx_data), I2C_FLAG_TIMEOUT);

	i2c_tx_addr=0x20;	/* INT_OUT_CTRL */
	i2c_tx_data=0x01;	/* active high level for INT1 */
	
	HAL_I2C_Mem_Write(&I2cHandle, I2C_ACC_ADDRESS_BMA280, i2c_tx_addr, I2C_MEMADD_SIZE_8BIT, &i2c_tx_data, sizeof(i2c_tx_data), I2C_FLAG_TIMEOUT);

	i2c_tx_addr=0x17;	/* INT_EN_1 */
	i2c_tx_data=0x10;	/* data ready interrupt enable */
	
	HAL_I2C_Mem_Write(&I2cHandle, I2C_ACC_ADDRESS_BMA280, i2c_tx_addr, I2C_MEMADD_SIZE_8BIT, &i2c_tx_data, sizeof(i2c_tx_data), I2C_FLAG_TIMEOUT);
#endif//20160215	
	
	HAL_Delay(10);		
	
  /* Infinite loop */
  while (1)
  {
			wifi_module_status_check();
			
			memset(szBuffer,0,sizeof(char)*128);
			HAL_Delay(10);
	
			CDC_Transmit_FS((uint8_t*)&wifi_buffer, sizeof(wifi_buffer));
			HAL_Delay(10);
		
		
			if(0)//if(wifi_opr_mode==WIFI_UDP_START)
			{
				if(HAL_UART_Transmit_IT(&UartHandle, (uint8_t*)&testInt, sizeof(testInt))!= HAL_OK)
				{
						Error_Handler();
				}
				testInt++;
			HAL_Delay(10);

				
				if(HAL_UART_Transmit_IT(&UartHandle, (uint8_t*)aTxBuffer, sizeof(aTxBuffer))!= HAL_OK)
				{
						Error_Handler();
				}
			}
			
			//usb cdc test
			//CDC_Transmit_FS((uint8_t*)"AA\r\n", 4);
	}
}

//20160215
/**
  * @brief  Configures EXTI ACC, GYR and MAG in interrupt mode
  * @param  None
  * @retval None
  */
static void EXTI_ACC_GYR_MAG_Config(void)
{
		GPIO_InitTypeDef   GPIO_InitStructure;
	
		/* Enable GPIOB clock */
		__HAL_RCC_GPIOB_CLK_ENABLE();
	
		/* Configure PB4(GYR), PB5(MAG), PB8(ACC) pin as input floating */
		GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
		GPIO_InitStructure.Pull = GPIO_NOPULL;
		GPIO_InitStructure.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_8;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
	
		/* Enable and set EXTI Interrupt to the lowest priority */
		HAL_NVIC_SetPriority(EXTI4_IRQn, 0x0F, 0);
		HAL_NVIC_EnableIRQ(EXTI4_IRQn);
	
		HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0x0F, 0);
		HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}
//20160215

/**
  * @brief  EXTI line detection callbacks.
  * @param  GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	BSP_LED_Toggle(LED2);//20160215
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
		BSP_LED_Toggle(LED2);//20160215
		while(1)
		{
		}
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 84000000
  *            HCLK(Hz)                       = 84000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 336
  *            PLL_P                          = 4
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale2 mode
  *            Flash Latency(WS)              = 2
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  
  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);
 
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}



/**
  * @brief  Tx Transfer completed callback
  * @param  UartHandle: UART handle. 
  * @note   This example shows a simple way to report end of IT Tx transfer, and 
  *         you can add your own implementation. 
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{	
		BSP_LED_Toggle(LED1);
}

/**
  * @brief  Rx Transfer completed callback
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report end of IT Rx transfer, and 
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
		BSP_LED_Toggle(LED1);
}

/**
  * @brief  UART error callbacks
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
		BSP_LED_Toggle(LED2);
}


/**
  * @brief  Period elapsed callback in non blocking mode
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
		BSP_LED_Toggle(LED0);
}


/**
  * @brief  Tx Transfer completed callback.
  * @param  I2cHandle: I2C handle 
  * @note   This example shows a simple way to report end of IT Tx transfer, and 
  *         you can add your own implementation. 
  * @retval None
  */
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
//  BSP_LED_On(LED2);
}

/**
  * @brief  Rx Transfer completed callback.
  * @param  I2cHandle: I2C handle
  * @note   This example shows a simple way to report end of IT Rx transfer, and 
  *         you can add your own implementation.
  * @retval None
  */
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
//  BSP_LED_Toggle(LED2);
}

/**
  * @brief  I2C error callbacks
  * @param  I2cHandle: I2C handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
 void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *I2cHandle)
{
//  BSP_LED_On(LED2); 
}

/**
  * @brief  TxRx Transfer completed callback
  * @param  hspi: SPI handle. 
  * @note   This example shows a simple way to report end of Interrupt TxRx transfer, and 
  *         you can add your own implementation. 
  * @retval None
  */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  /* Turn LED4 on: Transfer in transmission process is correct */
  BSP_LED_On(LED2);
}

/**
  * @brief  SPI error callbacks
  * @param  hspi: SPI handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
 void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  /* Turn LED5 on: Transfer error in reception/transmission process */
  BSP_LED_On(LED1); 
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @brief  This function is used to setup the WIFI module.(UDP Server)
  * @param  None
  * @retval None
  */
void wifi_module_status_check(void)
{	
		/* checking message from WIFI module(GS2100MIP)*/
		wifi_buffer_update_check();
	
		/* Below routines makes the WIFI module to UDP server. */
	  /* To do this, some AT commands are needed. */
		/* "AT+WRXACTIVE=1\r\n"; */
		/* "AT+NSET=192.168.1.96,255.255.255.0,192.168.1.96\r\n"; */
		/* "AT+WM=2\r\n"; */
		/* "AT+WA=SE96_AP,,11\r\n"; */
		/* "AT+DHCPSRVR=1\r\n"; */
		/* "AT+NSUDP=48596\r\n"; */
		/* "ATE0\r\n"; */
	
		switch(wifi_opr_mode)
		{
				case WIFI_NO_OPR:
					wifi_msg_updated=false;
				
					wifi_opr_mode =WIFI_CFG_START ;
					wifi_buffer_clear();
					break;

				case WIFI_CFG_START:
					if(HAL_UART_Transmit_IT(&UartHandle, (uint8_t*)tx_buffer_wifi_cfg_start, sizeof(tx_buffer_wifi_cfg_start))!= HAL_OK)
					{
							Error_Handler();
					}
					wifi_opr_mode = WIFI_CFG_END;
					break;

				case WIFI_CFG_END:
					if(wifi_msg_updated)
					{
							wifi_msg_updated=false;
						
							wifi_opr_mode = WIFI_CFG_NEWTWORK_START;
							wifi_buffer_clear();
					}
					break;

				case WIFI_CFG_NEWTWORK_START:
					HAL_Delay(10);
				
					if(HAL_UART_Transmit_IT(&UartHandle, (uint8_t*)tx_buffer_wifi_cfg_network_start, sizeof(tx_buffer_wifi_cfg_network_start))!= HAL_OK)
					{
							Error_Handler();
					}
					wifi_opr_mode = WIFI_CFG_NEWTWORK_END;
					break;

				case WIFI_CFG_NEWTWORK_END:
					if(wifi_msg_updated)
					{
							wifi_msg_updated=false;
						
							wifi_opr_mode = WIFI_CFG_WMODE_START;
							wifi_buffer_clear();
					}
					break;

				case WIFI_CFG_WMODE_START:
					HAL_Delay(10);
				
					if(HAL_UART_Transmit_IT(&UartHandle, (uint8_t*)tx_buffer_wifi_cfg_wmode_start, sizeof(tx_buffer_wifi_cfg_wmode_start))!= HAL_OK)
					{
							Error_Handler();
					}
					wifi_opr_mode = WIFI_CFG_WMODE_END;	
					break;

				case WIFI_CFG_WMODE_END:
					if(wifi_msg_updated)
					{
							wifi_msg_updated=false;
						
							wifi_opr_mode = WIFI_CREATE_LIMITEDAP_START;
							wifi_buffer_clear();
					}
					break;

				case WIFI_CREATE_LIMITEDAP_START:
					HAL_Delay(10);
				
					if(HAL_UART_Transmit_IT(&UartHandle, (uint8_t*)tx_buffer_wifi_create_limitedap_start, sizeof(tx_buffer_wifi_create_limitedap_start))!= HAL_OK)
					{
							Error_Handler();
					}
					wifi_opr_mode = WIFI_CREATE_LIMITEDAP_END;	
					break;

				case WIFI_CREATE_LIMITEDAP_END:
					if(wifi_msg_updated)
					{
							wifi_msg_updated=false;
						
							wifi_opr_mode = WIFI_ENABLE_DHCP_SER_START;
							wifi_buffer_clear();
					}
					break;

				case WIFI_ENABLE_DHCP_SER_START:
					HAL_Delay(10);
				
					if(HAL_UART_Transmit_IT(&UartHandle, (uint8_t*)tx_buffer_wifi_enable_dhcp_ser_start, sizeof(tx_buffer_wifi_enable_dhcp_ser_start))!= HAL_OK)
					{
							Error_Handler();
					}
					wifi_opr_mode = WIFI_ENABLE_DHCP_SER_END;		
					break;

				case WIFI_ENABLE_DHCP_SER_END:
					if(wifi_msg_updated)
					{
							wifi_msg_updated=false;
						
							wifi_opr_mode = WIFI_UDP_LP_SET_START;
							wifi_buffer_clear();
					}
					break;

				case WIFI_UDP_LP_SET_START:
					HAL_Delay(10);
				
					if(HAL_UART_Transmit_IT(&UartHandle, (uint8_t*)tx_buffer_wifi_udp_lp_set_start, sizeof(tx_buffer_wifi_udp_lp_set_start))!= HAL_OK)
					{
							Error_Handler();
					}
					wifi_opr_mode = WIFI_TCP_LP_SET_START;
					break;

				case WIFI_TCP_LP_SET_START:
					if(wifi_msg_updated)
					{
							wifi_msg_updated=false;
						
							wifi_opr_mode = WIFI_TCP_LP_SET_END;
							wifi_buffer_clear();
					}
					break;

				case WIFI_TCP_LP_SET_END:
					HAL_Delay(10);
				
					if(HAL_UART_Transmit_IT(&UartHandle, (uint8_t*)tx_buffer_wifi_bulk_data_set, sizeof(tx_buffer_wifi_bulk_data_set))!= HAL_OK)
					{
							Error_Handler();
					}
					wifi_opr_mode = WIFI_UDP_LP_SET_END;
					
					//HAL_Delay(100);
					//if(HAL_UART_Transmit_IT(&UartHandle, (uint8_t*)tx_buffer_wifi_dns_data_set, sizeof(tx_buffer_wifi_dns_data_set))!= HAL_OK)
					//{
					//		Error_Handler();
					//}
					HAL_Delay(100);
					break;

				case WIFI_UDP_LP_SET_END:
					if(wifi_msg_updated)
					{
							wifi_msg_updated=false;
						
							wifi_opr_mode = WIFI_UDP_START;
							wifi_buffer_clear();
						
							wifi_Client_msg_updated=false;
					}
					break;
					
				default:
					break;
		}
}

/**
  * @brief  This function check messages from WIFI module.
  * @param  None
  * @retval None
  */
void wifi_buffer_update_check(void)
{
		uint16_t i;
		
		if(wifi_opr_mode!=WIFI_UDP_START)
		{
				for(i=0; i<WIFI_BUF_MAX-1; i++)
				{
						if((wifi_buffer[i]=='\r') && (wifi_buffer[i+1]=='\n'))	// Wifi module message end 
						{
								wifi_msg_updated=true;
						}
				}
		}
		else	//UDP Start
		{
				switch(wifi_http_mode)
				{				
					case 0:
					HAL_Delay(10);
					if(HAL_UART_Transmit_IT(&UartHandle, (uint8_t*)tx_buffer_wifi_http1_data_set, sizeof(tx_buffer_wifi_http1_data_set))!= HAL_OK)
					{
							Error_Handler();
					}
					wifi_http_mode=1;
					wifi_msg_updated=true;
					break;
					
					case 1:
					if(wifi_msg_updated)
					{
							wifi_msg_updated=false;
						
							wifi_http_mode = 2;
							wifi_buffer_clear();
					}
					break;
					
					case 2:
					HAL_Delay(10);
					if(HAL_UART_Transmit_IT(&UartHandle, (uint8_t*)tx_buffer_wifi_http2_data_set, sizeof(tx_buffer_wifi_http2_data_set))!= HAL_OK)
					{
							Error_Handler();
					}
					wifi_http_mode=3;
					wifi_msg_updated=true;
					break;
					
					case 3:
					if(wifi_msg_updated)
					{
							wifi_msg_updated=false;
						
							wifi_http_mode = 4;
							wifi_buffer_clear();
					}
					break;
					
					case 4:
					HAL_Delay(10);
					if(HAL_UART_Transmit_IT(&UartHandle, (uint8_t*)tx_buffer_wifi_http3_data_set, sizeof(tx_buffer_wifi_http3_data_set))!= HAL_OK)
					{
							Error_Handler();
					}
					wifi_http_mode=5;
					wifi_msg_updated=true;
					break;
					
					case 5:
					if(wifi_msg_updated)
					{
							wifi_msg_updated=false;
						
							wifi_http_mode = 6;
							wifi_buffer_clear();
					}
					break;
					
					case 6:
					HAL_Delay(10);
					if(HAL_UART_Transmit_IT(&UartHandle, (uint8_t*)tx_buffer_wifi_http4_data_set, sizeof(tx_buffer_wifi_http4_data_set))!= HAL_OK)
					{
							Error_Handler();
					}
					wifi_http_mode=7;
					wifi_msg_updated=true;
					break;
					
					case 7:
					if(wifi_msg_updated)
					{
							wifi_msg_updated=false;
						
							wifi_http_mode = 8;
							wifi_buffer_clear();
					}
					break;
					
					case 8:
					HAL_Delay(10);
					if(HAL_UART_Transmit_IT(&UartHandle, (uint8_t*)tx_buffer_wifi_http5_data_set, sizeof(tx_buffer_wifi_http5_data_set))!= HAL_OK)
					{
							Error_Handler();
					}
					wifi_http_mode=9;
					wifi_msg_updated=true;
					break;
					
					case 9:
					if(wifi_msg_updated)
					{
							wifi_msg_updated=false;
						
							wifi_http_mode = 10;
							wifi_buffer_clear();
					}
					break;
					
					case 10:
					HAL_Delay(10);
					if(HAL_UART_Transmit_IT(&UartHandle, (uint8_t*)tx_buffer_wifi_http6_data_set, sizeof(tx_buffer_wifi_http6_data_set))!= HAL_OK)
					{
							Error_Handler();
					}
					wifi_http_mode=11;
					wifi_msg_updated=true;
					break;
					
					case 11:
					if(wifi_msg_updated)
					{
							wifi_msg_updated=false;
						
							wifi_http_mode = 12;
							wifi_buffer_clear();
					}
					break;
					
				default:
					break;
				}
				
				
				for(i=0;i<WIFI_BUF_MAX-1;i++)
				{
						if((wifi_buffer[i]=='\x01b') && (wifi_buffer[i+1]=='u') && (nIPStartIndex==0))		//Client Send Message start
						{
								nIPStartIndex=i+3;
						}
								
						if((wifi_buffer[i]=='\x020') && (nPortIndex==0) && (nIPStartIndex!=0))		//space 
						{
								nPortIndex=i+1;//Port Index info.
								nIPEndIndex=i-1;
						}
											 
						if((wifi_buffer[i]=='\t') && (nMsgDataStartIndex==0) &&(nIPStartIndex!=0))		//Tap
						{
								nMsgDataStartIndex=i+1;
						}
											 
						if((wifi_buffer[i]=='\x01b') && (wifi_buffer[i+1]=='E') && (nMsgDataStartIndex!=0) && (nMsgDataEndIndex==0) &&(nIPStartIndex!=0))		//client Send Message End
						{
								nMsgDataEndIndex=i-1;		 		
								wifi_Client_msg_updated=true;
						}				 
				}
		}	
}

/**
  * @brief  This function clears WIFI message buffer.
  * @param  None
  * @retval None
  */
void wifi_buffer_clear(void)
{
		memset(wifi_buffer,0,WIFI_BUF_MAX);

		/* Uart interrupt buffer pointer and counts initialize */
		UartHandle.pRxBuffPtr = wifi_buffer;
		UartHandle.RxXferCount = WIFI_BUF_MAX;
	  UartHandle.RxXferSize = WIFI_BUF_MAX;
}

//20160215
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 7 */ 
  USBD_CDC_SetTxBuffer(&hUSBDDevice, Buf, Len);   
  result = USBD_CDC_TransmitPacket(&hUSBDDevice);
  /* USER CODE END 7 */ 
  return result;
}
//20160215

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

