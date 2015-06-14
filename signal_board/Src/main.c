/**
  ******************************************************************************
  * File Name          : main.c
  * Date               : 13/06/2015 14:55:49
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
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
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include <string.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
#define NUM_ELEMENTS(x) (sizeof(x)/sizeof(x[0]))
#define NUM_ADC_CHANNELS 3
#define ADC_SAMPLES_PER_FRAME 100
#define ADC_DATA_BUFFER_SIZE (NUM_ADC_CHANNELS * ADC_SAMPLES_PER_FRAME) 
static uint32_t adc_data[ADC_DATA_BUFFER_SIZE]; //__attribute__ ((aligned));

#define DP0 GPIO_PIN_8
#define DP1 GPIO_PIN_9
#define DP2 GPIO_PIN_10
#define SELECT_DP(m) HAL_GPIO_WritePin(GPIOD, m, GPIO_PIN_RESET) 
#define UNSELECT_DP() HAL_GPIO_WritePin(GPIOD, DP0|DP1|DP2, GPIO_PIN_SET)

// #define UART_TX_COMPLETE_EVT (1ul << 0)
#define START_BYTE  0xAB
#define STOP_BYTE  0xBA


typedef enum {
  KEY_START = 0,
  KEY_STOP,
  MAX_KEY
} hw_keys_e;

typedef struct {
  const GPIO_TypeDef* port;
  const uint16_t pin;
  BaseType_t was_released;
  uint32_t debounce_cnt;
} hw_key_s;

typedef enum {
  PACKET_CONTROL = 0,
  PACKET_INDICATION,
  PACKET_KEYCODE,
  PACKET_DATA,
  PACKET_MAX
} packet_type_e;

#define MAX_PACKET_DATA_SIZE  100

typedef struct __attribute__((packed)) 
{
  uint8_t type;
  uint8_t payload_size;
} packet_header_s;

typedef struct __attribute__((packed)) 
{
  union __attribute__((packed))
  {
    uint8_t data[MAX_PACKET_DATA_SIZE];
  };
} packet_payload_s;

typedef struct __attribute__((packed)) 
{
  packet_header_s h;
  packet_payload_s p;
} packet_s;

static TaskHandle_t xMainHandle = NULL;
static TaskHandle_t xKeybHandle = NULL;
static TaskHandle_t xUsartTXHandle = NULL;
static TaskHandle_t xUsartRXHandle = NULL;

static QueueHandle_t xUartTxQueue = NULL;
static QueueHandle_t xUartRxQueue = NULL;

static uint8_t gain = 127;
static uint8_t uart_data;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
static void DP_Set_Value(uint32_t mask, uint8_t val);
void vMainTask( void * pvParameters );
void vKeybTask( void * pvParameters );
void vUartTxTask( void * pvParameters );
void vUartRxTask( void * pvParameters );
static BaseType_t send_packet(packet_type_e type, const void *payload, uint8_t payload_size);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  UNSELECT_DP();
  // HAL_ADC_Start_IT(&hadc1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_data, NUM_ELEMENTS(adc_data));
  HAL_TIM_Base_Start(&htim2);

  DP_Set_Value(DP0|DP1|DP2, gain);

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  
  xTaskCreate( vMainTask, "MAIN", configMINIMAL_STACK_SIZE*2, NULL, 1, &xMainHandle );
  configASSERT( xMainHandle );  
  xTaskCreate( vKeybTask, "KEYB", configMINIMAL_STACK_SIZE*2, NULL, 2, &xKeybHandle );
  configASSERT( xKeybHandle ); 
  xTaskCreate( vUartTxTask, "UATX", configMINIMAL_STACK_SIZE*2, NULL, 3, &xUsartTXHandle );
  configASSERT( xUsartTXHandle );  
  xTaskCreate( vUartRxTask, "UARX", configMINIMAL_STACK_SIZE*2, NULL, 3, &xUsartRXHandle );
  configASSERT( xUsartRXHandle );  

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  xUartTxQueue = xQueueCreate( 5, sizeof( packet_s * ) );
  configASSERT( xUartTxQueue );  
  xUartRxQueue = xQueueCreate( 10, sizeof( uint8_t ) );
  configASSERT( xUartRxQueue );  




  HAL_UART_Receive_IT(&huart1, &uart_data, 1);

  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  vTaskStartScheduler();
  
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

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 210;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

}

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV6;   /* APB2 clock == 84 MHz (FCLK == 168) -/6-> 14 MHz */
  hadc1.Init.Resolution = ADC_RESOLUTION12b;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = EOC_SINGLE_CONV;
  HAL_ADC_Init(&hadc1);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 2;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 3;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

}

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLED;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
  HAL_SPI_Init(&hspi1);

}

/* TIM2 init function */
void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  uint32_t uwPrescalerValue = (uint32_t) ((SystemCoreClock /2) / 100000) - 1;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = uwPrescalerValue;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100 - 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim2);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

}

/* USART1 init function */
void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart1);

}

/** 
  * Enable DMA controller clock
  */
void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOE_CLK_ENABLE();
  __GPIOH_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOC_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();

  // /*Configure GPIO pins : PA0 */
  // GPIO_InitStruct.Pin = GPIO_PIN_0;
  // GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  // GPIO_InitStruct.Pull = GPIO_NOPULL;
  // HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PE2 PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 PD10 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
  // |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

static void DP_Set_Value(uint32_t mask, uint8_t val)
{
  SELECT_DP(mask);
  __attribute__((unused)) HAL_StatusTypeDef st = HAL_SPI_Transmit(&hspi1, &val, 1, 0x1000);
  UNSELECT_DP();
}

static BaseType_t send_packet(packet_type_e type, const void *payload, uint8_t payload_size)
{
  uint16_t size = sizeof(packet_header_s) + payload_size;
  packet_s *b = (packet_s *)pvPortMalloc(size);

  configASSERT( b );

  b->h.type = type;
  b->h.payload_size = payload_size;

  if(payload_size)
    memcpy(b->p.data, payload, payload_size);

  return xQueueSend( xUartTxQueue, &b, ( TickType_t ) 0 );
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{

}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  vTaskNotifyGiveFromISR(xUsartTXHandle, NULL);
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  xQueueSendFromISR( xUartRxQueue, &uart_data, NULL);
  HAL_UART_Receive_IT(&huart1, &uart_data, 1);
}

static size_t unstuff_packet(packet_s *p, const uint8_t *s, size_t size)
{
  uint8_t *out = (uint8_t *)p;
  uint8_t *in = (uint8_t *)s + 1;

  if(size <= 2)
    return 0;

  int sz = size -= 2; // -(start + stop)
  
  for(; sz > 0; sz--, in++, out++)
  {
    *out = *in;
    if(*in == START_BYTE || *in == STOP_BYTE)
    {
      in++;
      if(*in !=  START_BYTE && *in != STOP_BYTE)
        return 0; // packet corrupted
      sz--;
    }
  }

  return (size_t)(out - (uint8_t *)p);
}
static void process_packet(const uint8_t *p, size_t size)
{
  static packet_s packet;
  size_t sz = unstuff_packet(&packet, p, size);

  if(sz == 0)
    return; // packet corrupted

  


}

void vUartRxTask( void * pvParameters )
{
  enum reciever_state_e {
    WAIT_FOR_START_SYNC,
    PREVIOUS_WAS_START,
    PREVIOUS_WAS_TRIVIAL,
    PREVIOUS_WAS_STOP,
    MAX_STATE
  } state = WAIT_FOR_START_SYNC;
  size_t packet_len = 0;
  static uint8_t rx_buffer[(MAX_PACKET_DATA_SIZE + sizeof(packet_header_s))*2+2];

#define RESET_RX_MACHINE  do {state = WAIT_FOR_START_SYNC; packet_len = 0;} while(0)

  for( ;; )
  {
    uint8_t data;
    BaseType_t status = xQueueReceive(xUartRxQueue, &data, pdMS_TO_TICKS(10));

    switch(state)
    {
    case WAIT_FOR_START_SYNC:
      if(status == pdTRUE && data == START_BYTE)
      {
        rx_buffer[packet_len++] = data;
        state = PREVIOUS_WAS_TRIVIAL;
      }
      break;

    case PREVIOUS_WAS_START:
      if(status == pdTRUE)
      {
        rx_buffer[packet_len++] = data;
        if(data == STOP_BYTE)
          state = PREVIOUS_WAS_STOP;
        else  // start || trivial
          state = PREVIOUS_WAS_TRIVIAL;
      } else {
        RESET_RX_MACHINE;
      }
      break;

    case PREVIOUS_WAS_TRIVIAL:
      if(status == pdTRUE)
      {
        rx_buffer[packet_len++] = data;
        if(data == START_BYTE)
          state = PREVIOUS_WAS_START;
        else if(data == STOP_BYTE)
          state = PREVIOUS_WAS_STOP;
        else
          state = PREVIOUS_WAS_TRIVIAL;
      } else {
        RESET_RX_MACHINE;
      }
      break;

    case PREVIOUS_WAS_STOP:
      if(status == pdTRUE)
      {
        if(data != STOP_BYTE)
        {
          process_packet(rx_buffer, packet_len);
          RESET_RX_MACHINE;
          if(data == START_BYTE)
          {
            rx_buffer[packet_len++] = data;
            state = PREVIOUS_WAS_TRIVIAL;            
          }
        } else {
          rx_buffer[packet_len++] = data;
          state = PREVIOUS_WAS_TRIVIAL;
        }
      } else {
        process_packet(rx_buffer, packet_len);
        RESET_RX_MACHINE;
      }
      break;     
    default:
      ; 
    }

  }
}

static size_t stuff_packet(const packet_s *p, uint8_t *s, size_t size)
{
  uint8_t *in = (uint8_t *)p;
  uint8_t *out = s;

  *out++ = START_BYTE;
  for(; size; size--, in++, out++)
  {
    *out = *in;
    if(*in == START_BYTE)
      *++out = START_BYTE;
    else if(*in == STOP_BYTE)
      *++out = STOP_BYTE;
  }
  *out++ = STOP_BYTE;

  return (size_t)(out - s);
}

void vUartTxTask( void * pvParameters )
{
  for( ;; )
  {
    packet_s * p_packet = NULL;
    xQueueReceive(xUartTxQueue, &p_packet, portMAX_DELAY);
    if(p_packet)
    {
      uint8_t packet_size = sizeof(packet_header_s) + p_packet->h.payload_size;
      uint8_t *b = (uint8_t *)pvPortMalloc((packet_size)*2 + 2);
      configASSERT( b );
      size_t tx_size = stuff_packet(p_packet, b, packet_size);
      vPortFree(p_packet);
      HAL_StatusTypeDef st __attribute__((unused)) = 
        HAL_UART_Transmit_DMA(&huart1, b, tx_size);
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      vPortFree(b);
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);
    }
  }
}

void vMainTask( void * pvParameters )
{
  
  for( ;; )
  {
    uint32_t events = 0;

    if(xTaskNotifyWait( 0, -1, &events, pdMS_TO_TICKS(10)) == pdTRUE)
    {
      if(events & (1 << KEY_START))
      {
        if(gain) {
          DP_Set_Value(DP0|DP1|DP2, --gain);
        }
      }
      // if(events & (1 << KEY_STOP))
      // {
      //   if(gain < 127) {
      //     DP_Set_Value(DP0|DP1|DP2, ++gain);
      //   }
      // }
    }
  }
}

void vKeybTask( void * pvParameters )
{
#define KEY_STATE(k) HAL_GPIO_ReadPin((GPIO_TypeDef* )k.port, k.pin)

  static hw_key_s keys[MAX_KEY] = { 
                                   // {GPIOA, GPIO_PIN_0, pdFALSE, 0},
                                    {GPIOE, GPIO_PIN_2, pdFALSE, 0},
                                    {GPIOE, GPIO_PIN_4, pdFALSE, 0}
                                  };
  for( ;; )
  {
    int i;
    for(i = 0; i < MAX_KEY; i++)
    {
      GPIO_PinState state = KEY_STATE(keys[i]);
      switch(state)
      {
        case GPIO_PIN_SET:
          keys[i].debounce_cnt = 0;
          keys[i].was_released = pdTRUE;
          break;
        case GPIO_PIN_RESET:
          if(keys[i].was_released && ++keys[i].debounce_cnt > 5)
          {
            uint8_t keycode = (1 << i);
            keys[i].was_released = pdFALSE;

            uint8_t buf[] = {0xab, 0xab, 0x01, 0xba, 0xad, 0xba};
            // xTaskNotify(xMainHandle, (1 << i), eSetBits);
            send_packet(PACKET_KEYCODE, buf, sizeof(buf));
            
          }
          break;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
