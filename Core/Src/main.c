
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */


#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "stdio.h"
#include "i2c-lcd.h"
#include "si5351.h"
#define LONGPRESSINMS 300

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim2;


void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);

int row=0;
int col=0;
const int32_t correction = 978;

void USB_TransmitString(const char* str) {
  CDC_Transmit_FS((uint8_t*)str, strlen(str));
}

void processUSBData(uint8_t* Buf, uint32_t *Len) {
  char rBuf[100];
  sprintf(rBuf, "Received2: %s", Buf);
  USB_TransmitString(rBuf);
}

int main(void)
{
  int32_t oldEncValue;
  int32_t frequency[] = {1000000,100000};
  int32_t position = 100000;
  uint32_t oldcursortime = 0;

  char strBuffer[16];
  uint8_t needupdate=1;
  uint8_t cursor=0;
  uint8_t freqIdx=0;

  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_USB_DEVICE_Init();

  lcd_init ();
  lcd_send_string ("SI5351");
  HAL_Delay(1000);
  lcd_put_cur(1, 0);
  lcd_send_string("Freq Gen");
  HAL_Delay(2000);
  lcd_clear ();

  si5351_Init(correction);
  si5351_SetupCLK0(frequency[0], SI5351_DRIVE_STRENGTH_4MA);
  si5351_SetupCLK2(frequency[1], SI5351_DRIVE_STRENGTH_4MA);
  si5351_EnableOutputs((1<<0) | (1<<2));

  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

  oldEncValue = TIM2->CNT>>2;
  while (1)
  {
    uint32_t newEncVal = TIM2->CNT>>2;
    int32_t delta = (newEncVal - oldEncValue) >= 512 ? 
      -1 * (int32_t)( (oldEncValue - newEncVal) & 0x3ff ) 
      : 
      newEncVal - oldEncValue;

    if (delta) {
      //lcd_put_cur(1, 0);
      //sprintf(strBuffer,"%lu", newEncVal);
      //lcd_send_string(strBuffer);
      if ( 
          ((frequency[freqIdx] + delta * position)  > 8000L) 
          &&  
          ((frequency[freqIdx] + delta * position)  < 160000000L) 
        ){
        frequency[freqIdx] += delta * position;
        if(!freqIdx) {
          si5351_SetupCLK0((u_int32_t)frequency[0], SI5351_DRIVE_STRENGTH_4MA);
        } else {
          si5351_SetupCLK2((u_int32_t)frequency[1], SI5351_DRIVE_STRENGTH_4MA);
        }
        needupdate = 1;
        cursor = 0;
        oldcursortime = HAL_GetTick();
        HAL_Delay(20);
      }
      oldEncValue = newEncVal;
    }

    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == GPIO_PIN_RESET) {
      uint32_t c_time = HAL_GetTick();
      while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == GPIO_PIN_RESET);
      if (HAL_GetTick() - c_time > LONGPRESSINMS) {
        freqIdx = freqIdx ^ 0x01;
        needupdate = 1;
      } else {
        position *= 10;
        if (position > 10000000L) {
          position = 1;
        }
      }
      HAL_Delay(20);
    }

    if( !cursor && ((HAL_GetTick()-oldcursortime) > 500)) {
      cursor = 1;
      needupdate = 1;
    }

    if( cursor && ((HAL_GetTick()-oldcursortime) > 600)) {
      cursor = 0;
      oldcursortime = HAL_GetTick();
      needupdate = 1;
    }

    if (needupdate) {   
      lcd_put_cur(0, 0);
      strBuffer[0]='C';
      strBuffer[1]='H';
      strBuffer[2]=0x30+freqIdx*2;
      strBuffer[3]=':';
      strBuffer[4]=' ';
      sprintf(strBuffer+5,"%2lu", frequency[freqIdx]/1000000L);
      sprintf(strBuffer+7,".");
      sprintf(strBuffer+8,"%03lu", (frequency[freqIdx] % 1000000L)/1000L);
      sprintf(strBuffer+11,".");
      sprintf(strBuffer+12,"%03lu", (frequency[freqIdx] % 1000L));
      if (cursor) {
        switch(position) {
          case 1:
            cursor = 14; 
            break;
          case 10:
            cursor = 13;
            break;
          case 100:
            cursor = 12;
            break;
          case 1000:
            cursor = 10;
            break;
          case 10000:
            cursor = 9;
            break;
          case 100000:
            cursor = 8;
            break;
          case 1000000:
            cursor = 6;
            break;
          case 10000000:
            cursor = 5;
            break;
          default:
            cursor = 8;
        }
        strBuffer[cursor] = '_';
      }
      lcd_send_string(strBuffer);
      needupdate = 0;
      cursor = 0;
    }
  }
}

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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}


static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};
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
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_I2C1_Init(void)
{
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
}

static void MX_TIM2_Init(void)
{
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4096;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
}

static void MX_DMA_Init(void)
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */