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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "string.h"
#include "math.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint8_t buffer_usb[64];
uint8_t lenght;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t digitalPins =0;
// analog pin values are stored in array of ints
uint32_t analogPins[7] = {0};

unsigned int rpm = 4;
unsigned int kpa = 992; // 99.2
unsigned int tps = 965; // 96.5
unsigned int clt = 80;  // 80 - 100
unsigned int textCounter = 0;

// incoming data
char incomingFrame[17] = { 0 };
unsigned int incomingFramePos = 0;

// if READWRITE_PINS is defined, the values are read from and written to Arduino
// digital and analog pins.
#define READWRITE_PINS


void ReadDigitalStatuses()
{
#if defined (READWRITE_PINS)
  // read status of digital pins (1-13)
  digitalPins = 0;

  int bitposition = 0;

    if (HAL_GPIO_ReadPin(pin_1_GPIO_Port,pin_1_Pin) == 1) digitalPins |= (1 << bitposition);
    bitposition++;
    if (HAL_GPIO_ReadPin(pin_2_GPIO_Port,pin_2_Pin) == 1) digitalPins |= (1 << bitposition);
    bitposition++;
    if (HAL_GPIO_ReadPin(pin_3_GPIO_Port,pin_3_Pin) == 1) digitalPins |= (1 << bitposition);
    bitposition++;
    if (HAL_GPIO_ReadPin(pin_4_GPIO_Port,pin_4_Pin) == 1) digitalPins |= (1 << bitposition);
    bitposition++;
    if (HAL_GPIO_ReadPin(pin_5_GPIO_Port,pin_5_Pin) == 1) digitalPins |= (1 << bitposition);
    bitposition++;
    if (HAL_GPIO_ReadPin(pin_6_GPIO_Port,pin_6_Pin) == 1) digitalPins |= (1 << bitposition);
    bitposition++;
    if (HAL_GPIO_ReadPin(pin_7_GPIO_Port,pin_7_Pin) == 1) digitalPins |= (1 << bitposition);
    bitposition++;
    if (HAL_GPIO_ReadPin(pin_8_GPIO_Port,pin_8_Pin) == 1) digitalPins |= (1 << bitposition);
    bitposition++;
    if (HAL_GPIO_ReadPin(pin_9_GPIO_Port,pin_9_Pin) == 1) digitalPins |= (1 << bitposition);
    bitposition++;
    if (HAL_GPIO_ReadPin(pin_10_GPIO_Port,pin_10_Pin) == 1) digitalPins |= (1 << bitposition);
    bitposition++;
    if (HAL_GPIO_ReadPin(pin_11_GPIO_Port,pin_11_Pin) == 1) digitalPins |= (1 << bitposition);
    bitposition++;
    if (HAL_GPIO_ReadPin(pin_12_GPIO_Port,pin_12_Pin) == 1) digitalPins |= (1 << bitposition);
    bitposition++;
 // if (HAL_GPIO_ReadPin(pin_13_GPIO_Port, pin_13_Pin) == 1) digitalPins |= (1 << bitposition); // no pull-up
    if (HAL_GPIO_ReadPin(pin_13_GPIO_Port, pin_13_Pin) == 0) digitalPins |= (1 << bitposition);  // pull-up
    bitposition++;



#endif
}


//

void ReadAnalogStatuses()
{
#if defined (READWRITE_PINS)
  // read analog pins (0-7)


   HAL_ADC_Start(&hadc1);
   HAL_ADC_PollForConversion(&hadc1, 50);
   analogPins[0] = HAL_ADC_GetValue(&hadc1);
   HAL_ADC_PollForConversion(&hadc1, 50);
   analogPins[1] = HAL_ADC_GetValue(&hadc1);
   HAL_ADC_PollForConversion(&hadc1, 50);
   analogPins[2] = HAL_ADC_GetValue(&hadc1);
   HAL_ADC_PollForConversion(&hadc1, 50);
   analogPins[3] = HAL_ADC_GetValue(&hadc1);
   HAL_ADC_PollForConversion(&hadc1, 50);
   analogPins[4] = HAL_ADC_GetValue(&hadc1);
   HAL_ADC_PollForConversion(&hadc1, 50);
   analogPins[5] = HAL_ADC_GetValue(&hadc1);
   HAL_ADC_Stop (&hadc1);



#endif
}


void SendCANFramesToSerial()
{

  // build & send CAN frames to RealDash.
  // a CAN frame payload is always 8 bytes containing data in a manner
  // described by the RealDash custom channel description XML file
  // all multibyte values are handled as little endian by default.
  // endianess of the values can be specified in XML file if it is required to use big endian values

 unsigned char frameData[8]={0};


  // build 1st CAN frame, RPM, MAP, CLT, TPS (just example data)
  memcpy(frameData, &rpm, 2);
  memcpy(frameData + 2, &kpa, 2);
  memcpy(frameData + 4, &clt, 2);
  memcpy(frameData + 6, &tps, 2);

  // write first CAN frame to serial
 SendCANFrameToSerial(3200, frameData);
 memset(frameData,'\0',8);


  // build 2nd CAN frame, Arduino digital pins and 2 analog values
  memcpy(frameData, &digitalPins, 2);
  memcpy(frameData + 2, &analogPins[0], 2);
  memcpy(frameData + 4, &analogPins[1], 2);
  memcpy(frameData + 6, &analogPins[2], 2);

  // write 2nd CAN frame to serial
  SendCANFrameToSerial(3201, frameData);
  memset(frameData,'\0',8);


  // build 3rd CAN frame, rest of Arduino analog values
  memcpy(frameData, &analogPins[3], 2);
  memcpy(frameData + 2, &analogPins[4], 2);
  memcpy(frameData + 4, &analogPins[5], 2);
  memcpy(frameData + 6, &analogPins[6], 2);

  // write 3rd CAN frame to serial
  SendCANFrameToSerial(3202, frameData);
  memset(frameData,'\0',8);


  // build 4th frame, this is a text extension frame

  // only send once at 1000 loops
  if (textCounter == 0)
  {
    SendTextExtensionFrameToSerial(3203, "Hello RealDash, this is STM32 sending some text data");
  }
  else if (textCounter == 2000)
  {
    SendTextExtensionFrameToSerial(3203, "Tomorrow's forecast: Lots of sun and 30 degrees centigate");
  }
  else if (textCounter == 4000)
  {
    SendTextExtensionFrameToSerial(3203, "Now Playing: Insert your favorite song info here");
  }
  else if (textCounter == 6000)
  {
    SendTextExtensionFrameToSerial(3203, "Message from STM32: All systems running at nominal efficiency");
  }
  memset(frameData,'\0',8);

}

void SendCANFrameToSerial(unsigned long canFrameId, unsigned char* frameData)
{
  // the 4 byte identifier at the beginning of each CAN frame
  // this is required for RealDash to 'catch-up' on ongoing stream of CAN frames
  unsigned char serialBlockTag[4] = { 0x44, 0x33, 0x22, 0x11 };


  while( CDC_Transmit_FS((uint8_t*)serialBlockTag,4));

  // the CAN frame id number (as 32bit little endian value)

  while(CDC_Transmit_FS((uint8_t*)&canFrameId,4));

  // CAN frame payload

 while( CDC_Transmit_FS((uint8_t*)frameData,8));

}

void SendTextExtensionFrameToSerial(unsigned long canFrameId,  char* text)
{
  if (text)
  {
    // the 4 byte identifier at the beginning of each CAN frame
    // this is required for RealDash to 'catch-up' on ongoing stream of CAN frames
    unsigned char textExtensionBlockTag[4] = { 0x55, 0x33, 0x22, 0x11 };
    while(CDC_Transmit_FS((uint8_t*)textExtensionBlockTag,4));



    // the CAN frame id number (as 32bit little endian value)
    while(CDC_Transmit_FS((uint8_t*)&canFrameId,4));

    // text payload
    while(CDC_Transmit_FS((uint8_t*)text,strlen(text)+15));


  }
}



void ReadIncomingSerialData()
{
  while (lenght > 0)
  {
    /*
      little bit of extra effort here, since especially Bluetooth connections
     may leave unsent/received data in internal buffer for a long time
     therefore, we cannot be sure that incoming byte stream really starts at
     where we expect it to start.

     read one byte from serial stream

	*/

	  memcpy(incomingFrame,buffer_usb,17);
	  int incomingFramePos=17;

    // check the first incoming bytes tag (0x44, 0x33, 0x22, 0x11)
    if (incomingFrame[0] != 0x44)
    {
      // first incoming byte is not 0x44,
      // the tag at the beginning of the frame does not match, this is an invalid frame
      // just zero the incomingFrame buffer and start expecting first byte again
      memset(incomingFrame, 0, 17);
      incomingFramePos = 0;
    }

    if (incomingFramePos >= 17)
    {
      // frame complete, process it
      ProcessIncomingFrame(incomingFrame);

      // zero the incomingFrame buffer and start expecting first byte again
      memset(incomingFrame, 0, 17);
      incomingFramePos = 0;
      memset(buffer_usb,'\0',64);
      lenght=0;
    }
  }
}

void ProcessIncomingFrame( char* frame)
{
  // first four bytes contain set value frame separator bytes, always 0x44,0x33,0x22,x11
  // check that first 4 bytes match the tag
  if (frame[0] != 0x44 ||
      frame[1] != 0x33 ||
      frame[2] != 0x22 ||
      frame[3] != 0x11)
  {
    // frame tag does not match, wait for another frame
    return;
  }

  // next four bytes contain set value CAN frame id in little endian form
  unsigned long canFrameId = 0;
  memcpy(&canFrameId, frame + 4, 4);

  // next 8 bytes are the frame data
  // ...

  // last byte is check byte calculated as sum of previous 13 bytes (ignore overflow)
  char checkByte = 0;
  for (int i=0; i<16; i++)
  {
    checkByte += frame[i];
  }

  if (frame[16] == checkByte)
  {
    // checksum match, this is a valid set value-frame:
    // the frame payload data is in frame + 8 bytes
    HandleIncomingSetValueFrame(canFrameId, frame + 8);
  }
}

void HandleIncomingSetValueFrame(unsigned long canFrameId,  char* frameData)
{
  if (canFrameId == 3201)
  {
    memcpy(&digitalPins, frameData, 2);
    memcpy(&analogPins[0], frameData + 2, 2);
    memcpy(&analogPins[1], frameData + 4, 2);
    memcpy(&analogPins[2], frameData + 6, 2);

#if defined (READWRITE_PINS)
    // write digital pins

    HAL_GPIO_WritePin(pin_1_GPIO_Port,pin_1_Pin,digitalPins & (1 << 0) ? 1 : 0);
    HAL_GPIO_WritePin(pin_2_GPIO_Port,pin_2_Pin,digitalPins & (1 << 1) ? 1 : 0);
    HAL_GPIO_WritePin(pin_3_GPIO_Port,pin_3_Pin,digitalPins & (1 << 2) ? 1 : 0);
    HAL_GPIO_WritePin(pin_4_GPIO_Port,pin_4_Pin,digitalPins & (1 << 3) ? 1 : 0);
    HAL_GPIO_WritePin(pin_5_GPIO_Port,pin_5_Pin,digitalPins & (1 << 4) ? 1 : 0);
    HAL_GPIO_WritePin(pin_6_GPIO_Port,pin_6_Pin,digitalPins & (1 << 5) ? 1 : 0);
    HAL_GPIO_WritePin(pin_7_GPIO_Port,pin_7_Pin,digitalPins & (1 << 6) ? 1 : 0);
    HAL_GPIO_WritePin(pin_8_GPIO_Port,pin_8_Pin,digitalPins & (1 << 7) ? 1 : 0);
    HAL_GPIO_WritePin(pin_9_GPIO_Port,pin_9_Pin,digitalPins & (1 << 8) ? 1 : 0);
    HAL_GPIO_WritePin(pin_10_GPIO_Port,pin_10_Pin,digitalPins & (1 << 9) ? 1 : 0);
    HAL_GPIO_WritePin(pin_11_GPIO_Port,pin_11_Pin,digitalPins & (1 << 10) ? 1 : 0);
    HAL_GPIO_WritePin(pin_12_GPIO_Port,pin_12_Pin,digitalPins & (1 << 11) ? 1 : 0);
 //   HAL_GPIO_WritePin(pin_13_GPIO_Port,pin_13_Pin,digitalPins & (1 << 12) ? 1 : 0);
    HAL_GPIO_WritePin(pin_13_GPIO_Port,pin_13_Pin,digitalPins & (1 << 12) ? 0 : 1); // pull-up led PC13 , also CubeMx GPIO config set to be Pull-Up



  /*  analogWrite(0, analogPins[0]);
    analogWrite(1, analogPins[1]);
    analogWrite(2, analogPins[2]);
    */
#endif
  }
  else if (canFrameId == 3202)
  {
    memcpy(&analogPins[3], frameData + 0, 2);
    memcpy(&analogPins[4], frameData + 2, 2);
    memcpy(&analogPins[5], frameData + 4, 2);
    memcpy(&analogPins[6], frameData + 6, 2);

#if defined (READWRITE_PINS)
   /* analogWrite(3, analogPins[3]);
    analogWrite(4, analogPins[4]);
    analogWrite(5, analogPins[5]);
    analogWrite(6, analogPins[6]);
    */
#endif
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
  MX_USB_DEVICE_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  ReadDigitalStatuses();
	  ReadAnalogStatuses();
	  SendCANFramesToSerial();
	 // ReadIncomingSerialData(); // move to usbd_cdc_if.c/CDC_Receive_FS

	  if (rpm++ > 5000)
	   {
	     rpm = 700;
	   }
	   if (kpa++ > 2500)
	   {
	     kpa = 10;
	   }
	   if (tps++ > 1000)
	   {
	     tps = 0;
	   }
	   if (clt++ > 230)
	   {
	     // all values in frame are handled as unsigned values. To use negative values,
	     // offset actual value and write corresponding conversion to XML file imported to RealDash
	     // From RealDash 1.7.6 its also possible to specify value as signed="true" in XML file.
	     clt = 0;
	   }
	   // simple counter for sending the text frame to avoid sending it too often.
	   if (textCounter++ > 8000)
	   {
	     textCounter = 0;
	   }



	   HAL_Delay(5);

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
