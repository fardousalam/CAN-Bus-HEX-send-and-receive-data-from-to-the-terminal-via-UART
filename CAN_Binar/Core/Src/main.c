/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */


                              /*  Beschreibung des Programms
                               *  Dieses Programm hat drei Hauptaufgaben
                               *  1. CAN filter Maskierung (std Id=0x80 und ID zweischen 0x200 und 0x300 sind erlaubt, Daten zu empfangen. alle anderen sollten nicht erlaubt sein
                               *  2. Einfügen des Hex Wertes(NICHT ASCII) in das Terminal über uart und Senden des empfangenen Hex Wertes über den CAN Controller.
                               *  3. den Hex Wert vom Can Controller empfangen und über uart an das Terminal senden. */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f3xx_hal.h"
#include "stm32f3xx_it.h"
#include <string.h>
#include<stdio.h>
#include<stdlib.h>

#define MAX 100
#define TRUE 1
#define FALSE 0


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

CAN_HandleTypeDef hcan;             // structure für CAN peripheral
CAN_FilterTypeDef sFilterConfig1;   // Filter Konfigurationen struktur für filter id zweishen 0x200 und 0x300
CAN_FilterTypeDef sFilterConfig2;  // Filter Konfigurationen struktur für filter id  0x80
UART_HandleTypeDef huart2;        //UART2 Konfigurationen
CAN_RxHeaderTypeDef RxHeader;     // CAN Rx message Konfigurationen

uint8_t RxDaTa[8];              // Empfangen von Datenpuffer CAN BUS
int hexbase;                    //
char *user_data = "The program is running\r\n";
	uint8_t recvd_data;
	uint8_t data_buffer[31];
    uint32_t count=0;
	uint8_t reception_complete = FALSE;
	uint8_t bin_buffer[10];

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN_Init(void);
static void MX_NVIC_Init(void);
void convert_hexa(char* input, char* output);
void asciitobinary(char*data_buffer);


/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

CAN_TxHeaderTypeDef   TxHeader;
char               TxData[8];
uint32_t              TxMailbox;
uint8_t               UART_buffer[8];
uint32_t              UART_CAN_ID=0x123;
 char        buff[8];

char bin_str[20];

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
  MX_CAN_Init();
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  uint16_t len_of_data = strlen(user_data);
  HAL_UART_Transmit(&huart2,(uint8_t*)user_data,len_of_data,HAL_MAX_DELAY);


  while(reception_complete != TRUE)
            	 	          	        {


    	                                    	HAL_UART_Receive_IT(&huart2,&recvd_data,1);


            	 	          	        }


  /* USER CODE END 2 */

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
/*diese Funktion wird aktuell nicht verwendet */
void convert_hexa(char* input, char* output)
{
   int loop=0;
   int i=0;
   while(input[loop] != '\0'){
      sprintf((char*)(output+i),"%02X", input[loop]);
      loop+=1;
      i+=2;
   }
   //marking the end of the string
   output[i++] = '\0';
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) // Callback-Funktion für Empfangs interrupt
{
	HAL_UART_Receive_IT(&huart2,&recvd_data,1);        //  Empfangs interrupt funktion

	 if(recvd_data == '\r')                         //   wenn "Enter" gedrückt wird, ist der Empfang abgeschlossen
		 {
			 reception_complete = TRUE;

                    char *args[15];
			        int i =0; int j=0;
			        char *token = strtok(data_buffer, " \t\n"); //string aufteilen
			        while (token != NULL) {
			            args[i++] = token;                 //Sammeln der gespaltenen String
			            token = strtok(NULL, " \t\n");
			        }
			        //to print the array
			        for (j = 0; j < i; j++) {
			            printf("%s\r\n", args[j]);
			        }

			        /*  Konvertierung von string in das Hex-Format  */

			 int  hexbase0=(int) strtol (args[0],NULL,16);
			 int  hexbase1=(int) strtol (args[1],NULL,16);
			 int  hexbase2=(int) strtol (args[2],NULL,16);
			 int  hexbase3=(int) strtol (args[3],NULL,16);
			 int  hexbase4=(int) strtol (args[4],NULL,16);
			 int  hexbase5=(int) strtol (args[5],NULL,16);
			 int  hexbase6=(int) strtol (args[6],NULL,16);
			 int  hexbase7=(int) strtol (args[7],NULL,16);
			 int  hexbase8=(int) strtol (args[8],NULL,16);


			 /*Übertragung der Hex-Werte */

			 TxData[0] = hexbase1;
			 TxData[1] = hexbase2;
			 TxData[2] = hexbase3;
			 TxData[3] = hexbase4 ;
			 TxData[4] = hexbase5;
			 TxData[5] = hexbase6;
			 TxData[6] = hexbase7;
			 TxData[7] = hexbase8;

			 TxHeader.IDE = CAN_ID_STD;
			 TxHeader.StdId = hexbase0;
			 TxHeader.RTR = CAN_RTR_DATA;
			 TxHeader.DLC = 8;

			 //Übertragung der CAN Nachtricht

              if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, (uint8_t*)TxData, &TxMailbox) != HAL_OK)
						 {
							 Error_Handler ();
						 }

             //HAL_UART_Transmit(huart,data_buffer,count,HAL_MAX_DELAY);
			 data_buffer[count++]='\r';

			 for(count=10; count>0; count--)                        //buffer leer machen
			  {
                 data_buffer[count]=0;
			  }
		 }
		 else
		 {
			 data_buffer[count++] = recvd_data;
		 }

}





/* Systemclock Konfigurationen */

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */

 /*CAN Initialisierung */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 16;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_12TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_5TQ;
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

// Filter bank 1  Konfigurationen


  sFilterConfig1.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig1.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig1.FilterBank = 1;
  sFilterConfig1.FilterIdHigh=0x200<<5;
  sFilterConfig1.FilterMaskIdHigh=0x300<<5;
  sFilterConfig1.FilterIdLow = 0x0000;
  sFilterConfig1.FilterMaskIdLow = 0x0000;
  sFilterConfig1.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig1.FilterActivation = ENABLE;

  //Filter bank 2   Konfigurationen

    sFilterConfig2.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig2.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig2.FilterBank = 2;
    sFilterConfig2.FilterIdHigh=0x80<<5;
    sFilterConfig2.FilterMaskIdHigh=0xff<<5;
    sFilterConfig2.FilterIdLow = 0x0000;
    sFilterConfig2.FilterMaskIdLow = 0x0000;
    sFilterConfig2.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig2.FilterActivation = ENABLE;



if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)		// Enable FIFO0 data interrupt reception
          	{
          		Error_Handler();
          	}

      if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig1) != HAL_OK)        //Filter bank 1 Initialisierung
        {
          Error_Handler();
        }

      if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig2) != HAL_OK)      //Filter bank 2 Initialisierung
             {
               Error_Handler();
             }
    /* USER CODE BEGIN CAN_Init 2 */



      	if (HAL_CAN_Start(&hcan) != HAL_OK)
      	     	{
      	     		Error_Handler();
      	     	}

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
/* Interrupt konfiguration */
static void MX_NVIC_Init(void)
{


	  /* EXTI interrupt init*/
   HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
   HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
   /*CAN1_RX0_IRQn interrupt configuration*/
   HAL_NVIC_SetPriority(CAN_RX0_IRQn, 0, 0);
   HAL_NVIC_EnableIRQ(CAN_RX0_IRQn);
}
 /*diese Funktion wird aktuell nicht verwendet */
void asciitobinary(char*data_buffer)
{
	int i=0;
	while(i<10)
	{
		int j;
		int m[8];
		for(j=0;j<8;j++)
		{
			m[j]=data_buffer[i]%2;
			data_buffer[i]=data_buffer[i]/2;
		}

		int top,bottom;
		for(bottom=0,top=7;bottom<8;bottom++,top--)
		{
			bin_buffer[i]=m[top];
		}
		i++;
	}

}

/*USART2 Initialization Function*/
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

int __io_putchar(int ch)
{
 uint8_t c[1];
 c[0] = ch & 0x00FF;
 HAL_UART_Transmit(&huart2, &*c, 1, 10);
 return ch;
}

int _write(int file,char *ptr, int len)
{
 int DataIdx;
 for(DataIdx= 0; DataIdx< len; DataIdx++)
 {
 __io_putchar(*ptr++);
 }
return len;
}
/*Callback function für CAN empfang interrupt */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{


	//char msg[50];


	if(HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&RxHeader,RxDaTa) != HAL_OK)
	{
		Error_Handler();
	}

	     uint16_t ID = RxHeader.StdId;

	    printf("ID in Hexadecimal :%x\r\n",ID);
	    printf("Message received in Hexadecimal :\r\n");
		printf("first   byte : %x\r\n",RxDaTa[0]);
		printf("second  byte : %x\r\n",RxDaTa[1]);
		printf("third   byte : %x\r\n",RxDaTa[2]);
		printf("fourth  byte : %x\r\n",RxDaTa[3]);
		printf("fifth   byte : %x\r\n",RxDaTa[4]);
		printf("sixth   byte : %x\r\n",RxDaTa[5]);
		printf("seventh byte : %x\r\n",RxDaTa[6]);
		printf("eighth  byte : %x\r\n",RxDaTa[7]);
		//printf("Message received in Binary :\r\n");

		//Hextobin(hex_str);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
