/**
  ******************************************************************************
  * File Name          : main.c
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
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include <string.h>
#include <math.h>
#include "I2Cdev.h"
#include "MPU6050.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
typedef struct
{
	int16_t Accelerometer_X;
	int16_t Accelerometer_Y;
    int16_t Accelerometer_Z;
    int16_t Temperature;
    int16_t Gyroscope_X;
    int16_t Gyroscope_Y;
    int16_t Gyroscope_Z;
    uint32_t delta_time;
} MPU6050_MotionData_t;


static MPU6050_MotionData_t motion_data_struct;
static uint8_t motion_data_buffer[sizeof(MPU6050_MotionData_t)];
const size_t max_string_lengh = 127;


enum UART_Commands {
	UART_COMMAND_NOT_RECEIVED = 0,
    UART_REQUEST_SEND,
	UART_REQUEST_STOP,
	UART_REQUEST_SEND_BYTE,
	UART_REQUEST_SEND_MPU6050_TEST_DATA,
	UART_REQUEST_SEND_MPU6050_DATA,
	UART_REQUEST_SEND_MPU6050_PACKET
};


volatile uint8_t GYRO_XOUT_OFFSET;
volatile uint8_t GYRO_YOUT_OFFSET;
volatile uint8_t GYRO_ZOUT_OFFSET;
volatile uint8_t ACCEL_XOUT;
volatile uint8_t ACCEL_YOUT;
volatile uint8_t ACCEL_ZOUT;
volatile uint8_t GYRO_XRATE;
volatile uint8_t GYRO_YRATE;
volatile uint8_t GYRO_ZRATE;
volatile float ACCEL_XANGLE;
volatile float ACCEL_YANGLE;



#define LED_PIN GPIO_PIN_8
#define LED_PORT GPIOC

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
extern float truncf(float x);

void init_led_hal();
void init_led();
void led_on();
void led_off();
void led_hal();
bool usart_send_str_ok(const char *input);
void usart_send_str(const char *input);
void Calibrate_Gyros();
void Get_Accel_Values();
void Get_Accel_Angles();
void Get_Gyro_Rates();
void MPU6050_getAllData();
void usart_wait_exec_loop();
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
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim4);

  init_led_hal();
  I2Cdev_hi2c = &hi2c1;
  MPU6050_setAddress(MPU6050_ADDRESS_AD0_LOW);
  while(!MPU6050_testConnection());
  MPU6050_initialize();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  usart_wait_exec_loop();
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
  HAL_I2C_Init(&hi2c1);

}

/* I2C2 init function */
void MX_I2C2_Init(void)
{

  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
  HAL_I2C_Init(&hi2c2);

}

/* TIM4 init function */
void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 24000;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 24000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim4);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig);

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

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __GPIOB_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
void init_led_hal()
{
    __GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef gpio;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pin = LED_PIN;
    gpio.Speed = GPIO_SPEED_LOW;
    gpio.Pull = GPIO_PULLDOWN;

    HAL_GPIO_Init(LED_PORT, &gpio);
}
void init_led()
{
    //====================================
    // enable clock for gpio port C
    //====================================
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;

    //====================================
    // setup GPIO port C pin 8 for output
    //====================================
    // output mode push-pull - by default
    // setup speed 2Mhz
    GPIOC->CRH &= (~GPIO_CRH_CNF8);
    GPIOC->CRH |= GPIO_CRH_MODE8_1;

}
void led_on()
{
    GPIOC->BSRR |= GPIO_BSRR_BS8;
}
void led_off()
{
    GPIOC->BRR |= GPIO_BRR_BR8;
}
void led_hal()
{
    HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
    HAL_Delay(1000);
    //HAL_GPIO_WritePin(GPIOC, LED_PIN, GPIO_PIN_SET);
}
void init_uart()
{

    /* Initialize GPIO for transmit/receive pin */

    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

    //Transmit register - pin 9
    //MODE: 11: Output mode, max speed 50 MHz.
    GPIOA->CRH |= GPIO_CRH_MODE9;

    //10: Alternate function output Push-pull
    GPIOA->CRH &= (~GPIO_CRH_CNF9);
    GPIOA->CRH |= GPIO_CRH_CNF9_1;

    //Receive register - pin 10
    // MODE: 00: Input mode (reset state)
    GPIOA->CRH &= (~GPIO_CRH_MODE10);
    // CNF: 01: Floating input (reset state)
    GPIOA->CRH &= (~GPIO_CRH_CNF10);
    GPIOA->CRH |= GPIO_CRH_CNF10_0;


    /* Initialize uart peripheral*/

    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_USART1EN;
    //1. Enable the USART by writing the UE bit in USART_CR1 register to 1.
    USART1->CR1 |= USART_CR1_UE;                // Uart Enable
    //2. Program the M bit in USART_CR1 to define the word length.
    USART1->CR1 &= (~USART_CR1_M);              // 8 bit word - bit M is reset
    //3. Program the number of stop bits in USART_CR2.
    USART1->CR2 &= (~USART_CR2_STOP);           // 1 stop bit - 00
    /*4. Select DMA enable (DMAT) in USART_CR3 if Multi buffer Communication is to take
    place. Configure the DMA register as explained in multibuffer communication.*/
    USART1->CR3 &= (~USART_CR3_DMAT);           // DMA disabled
    //5. Select the desired baud rate using the USART_BRR register.
    USART1->BRR = (SystemCoreClock / 115200);   //9600 or 115200 bps
    //6. Set the TE bit in USART_CR1 to send an idle frame as first transmission.
    USART1->CR1 |= USART_CR1_TE;

}
bool usart_send_str_ok(const char *input)
{
    char string[max_string_lengh + 1];
    strncpy(string, input, sizeof(string));   // copy input without exceeding the length of the destination
    string[sizeof(string) - 1] = '\0';        // if strlen(input) == sizeof(str) then strncpy won't NUL terminate
    size_t bytes_to_sent = strlen(string) + 1; // length of input string + 1 if it is less than max_string_lengh; max_string_lengh otherwise

    // TODO: study signed/unsigned conversion below
    HAL_StatusTypeDef trans = HAL_UART_Transmit(&huart1, (uint8_t*) string, bytes_to_sent, 1000);
    if(trans == HAL_OK)
    {
        return true;
    }
    return false;
}
void usart_send_str(const char *input)
{
    usart_send_str_ok(input);
}


void Calibrate_Gyros()
{
    uint8_t GYRO_XOUT_H;
    uint8_t GYRO_XOUT_L;
    uint8_t GYRO_YOUT_H;
    uint8_t GYRO_YOUT_L;
    uint8_t GYRO_ZOUT_H;
    uint8_t GYRO_ZOUT_L;
    uint8_t GYRO_XOUT_OFFSET_1000SUM = 0;
    uint8_t GYRO_YOUT_OFFSET_1000SUM = 0;
    uint8_t GYRO_ZOUT_OFFSET_1000SUM = 0;

    int x = 0;
    for(x = 0; x<1000; x++)
    {
        HAL_I2C_Mem_Read(I2Cdev_hi2c, MPU6050_DEFAULT_ADDRESS << 1, MPU6050_RA_GYRO_XOUT_H, 1, &GYRO_XOUT_H, 1, 1000);
        HAL_I2C_Mem_Read(I2Cdev_hi2c, MPU6050_DEFAULT_ADDRESS << 1, MPU6050_RA_GYRO_XOUT_L, 1, &GYRO_XOUT_L, 1, 1000);
        HAL_I2C_Mem_Read(I2Cdev_hi2c, MPU6050_DEFAULT_ADDRESS << 1, MPU6050_RA_GYRO_YOUT_H, 1, &GYRO_YOUT_H, 1, 1000);
        HAL_I2C_Mem_Read(I2Cdev_hi2c, MPU6050_DEFAULT_ADDRESS << 1, MPU6050_RA_GYRO_YOUT_L, 1, &GYRO_YOUT_L, 1, 1000);
        HAL_I2C_Mem_Read(I2Cdev_hi2c, MPU6050_DEFAULT_ADDRESS << 1, MPU6050_RA_GYRO_ZOUT_H, 1, &GYRO_ZOUT_H, 1, 1000);
        HAL_I2C_Mem_Read(I2Cdev_hi2c, MPU6050_DEFAULT_ADDRESS << 1, MPU6050_RA_GYRO_ZOUT_L, 1, &GYRO_ZOUT_L, 1, 1000);

        GYRO_XOUT_OFFSET_1000SUM += ((GYRO_XOUT_H<<8)|GYRO_XOUT_L);
        GYRO_YOUT_OFFSET_1000SUM += ((GYRO_YOUT_H<<8)|GYRO_YOUT_L);
        GYRO_ZOUT_OFFSET_1000SUM += ((GYRO_ZOUT_H<<8)|GYRO_ZOUT_L);

        HAL_Delay(1);
    }
    GYRO_XOUT_OFFSET = GYRO_XOUT_OFFSET_1000SUM/1000;
    GYRO_YOUT_OFFSET = GYRO_YOUT_OFFSET_1000SUM/1000;
    GYRO_ZOUT_OFFSET = GYRO_ZOUT_OFFSET_1000SUM/1000;
}

//Gets raw accelerometer data, performs no processing
void Get_Accel_Values()
{
    uint8_t ACCEL_XOUT_H;
    uint8_t ACCEL_XOUT_L;
    uint8_t ACCEL_YOUT_H;
    uint8_t ACCEL_YOUT_L;
    uint8_t ACCEL_ZOUT_H;
    uint8_t ACCEL_ZOUT_L;

    HAL_I2C_Mem_Read(I2Cdev_hi2c, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_XOUT_H, 1, &ACCEL_XOUT_H, 1, 1000);
    HAL_I2C_Mem_Read(I2Cdev_hi2c, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_XOUT_L, 1, &ACCEL_XOUT_L, 1, 1000);
    HAL_I2C_Mem_Read(I2Cdev_hi2c, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_YOUT_H, 1, &ACCEL_YOUT_H, 1, 1000);
    HAL_I2C_Mem_Read(I2Cdev_hi2c, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_YOUT_L, 1, &ACCEL_YOUT_L, 1, 1000);
    HAL_I2C_Mem_Read(I2Cdev_hi2c, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_ZOUT_H, 1, &ACCEL_ZOUT_H, 1, 1000);
    HAL_I2C_Mem_Read(I2Cdev_hi2c, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_ZOUT_L, 1, &ACCEL_ZOUT_L, 1, 1000);

    ACCEL_XOUT = ((ACCEL_XOUT_H<<8)|ACCEL_XOUT_L);
    ACCEL_YOUT = ((ACCEL_YOUT_H<<8)|ACCEL_YOUT_L);
    ACCEL_ZOUT = ((ACCEL_ZOUT_H<<8)|ACCEL_ZOUT_L);
}

//Converts the already acquired accelerometer data into 3D euler angles
void Get_Accel_Angles()
{
    ACCEL_XANGLE = 57.295*atan((float)ACCEL_YOUT/ sqrt(pow((float)ACCEL_ZOUT,2)+pow((float)ACCEL_XOUT,2)));
    ACCEL_YANGLE = 57.295*atan((float)-ACCEL_XOUT/ sqrt(pow((float)ACCEL_ZOUT,2)+pow((float)ACCEL_YOUT,2)));
}

//Function to read the gyroscope rate data and convert it into degrees/s
void Get_Gyro_Rates()
{
    uint8_t GYRO_XOUT_H;
    uint8_t GYRO_XOUT_L;
    uint8_t GYRO_YOUT_H;
    uint8_t GYRO_YOUT_L;
    uint8_t GYRO_ZOUT_H;
    uint8_t GYRO_ZOUT_L;

    HAL_I2C_Mem_Read(I2Cdev_hi2c, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_XOUT_H, 1, &GYRO_XOUT_H, 1, 1000);
    HAL_I2C_Mem_Read(I2Cdev_hi2c, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_XOUT_L, 1, &GYRO_XOUT_L, 1, 1000);
    HAL_I2C_Mem_Read(I2Cdev_hi2c, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_YOUT_H, 1, &GYRO_YOUT_H, 1, 1000);
    HAL_I2C_Mem_Read(I2Cdev_hi2c, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_YOUT_L, 1, &GYRO_YOUT_L, 1, 1000);
    HAL_I2C_Mem_Read(I2Cdev_hi2c, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_ZOUT_H, 1, &GYRO_ZOUT_H, 1, 1000);
    HAL_I2C_Mem_Read(I2Cdev_hi2c, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_ZOUT_L, 1, &GYRO_ZOUT_L, 1, 1000);

    uint8_t GYRO_XOUT = ((GYRO_XOUT_H<<8)|GYRO_XOUT_L) - GYRO_XOUT_OFFSET;
    uint8_t GYRO_YOUT = ((GYRO_YOUT_H<<8)|GYRO_YOUT_L) - GYRO_YOUT_OFFSET;
    uint8_t GYRO_ZOUT = ((GYRO_ZOUT_H<<8)|GYRO_ZOUT_L) - GYRO_ZOUT_OFFSET;


    uint8_t gyro_xsensitivity = 131;
    uint8_t gyro_ysensitivity = 131;
    uint8_t gyro_zsensitivity = 131;
    GYRO_XRATE = (float)GYRO_XOUT/gyro_xsensitivity;
    GYRO_YRATE = (float)GYRO_YOUT/gyro_ysensitivity;
    GYRO_ZRATE = (float)GYRO_ZOUT/gyro_zsensitivity;
}

void MPU6050_getAllData()
{
	// get new data
    I2Cdev_readBytes(MPU6050_ADDRESS_AD0_LOW, MPU6050_RA_ACCEL_XOUT_H, 14, motion_data_buffer, 0);
    // get time of arrival and reset timer
    //HAL_TIM_Base_Stop(&htim4);
    __HAL_TIM_GET_COUNTER(&htim4);
    motion_data_struct.delta_time = htim4.Instance->CNT;
   // HAL_TIM_Base_Start(&htim4);
    //__HAL_TIM_SET_COUNTER(&htim4, 0);

    motion_data_struct.Accelerometer_X = (((int16_t)motion_data_buffer[0]) << 8) | motion_data_buffer[1];
    motion_data_struct.Accelerometer_Y = (((int16_t)motion_data_buffer[2]) << 8) | motion_data_buffer[3];
    motion_data_struct.Accelerometer_Z = (((int16_t)motion_data_buffer[4]) << 8) | motion_data_buffer[5];
    motion_data_struct.Temperature     = (((int16_t)motion_data_buffer[6]) << 8) | motion_data_buffer[7];
    motion_data_struct.Gyroscope_X     = (((int16_t)motion_data_buffer[8]) << 8) | motion_data_buffer[9];
    motion_data_struct.Gyroscope_Y     = (((int16_t)motion_data_buffer[10])<< 8) | motion_data_buffer[11];
    motion_data_struct.Gyroscope_Z     = (((int16_t)motion_data_buffer[12])<< 8) | motion_data_buffer[13];
}

void usart_wait_exec_loop()
{
	while( 1 )
	  {
	      uint8_t byte = UART_COMMAND_NOT_RECEIVED;
	      HAL_UART_Receive(&huart1, &byte, 1, 1000);

          if(byte == UART_REQUEST_SEND_MPU6050_DATA )
	      {
              MPU6050_getAllData();
              HAL_StatusTypeDef status = HAL_UART_Transmit(&huart1, (uint8_t*)&motion_data_struct, sizeof(motion_data_struct), 1000);
              if(status != HAL_OK)
              {
                  // TODO: process error
              }
	      }
          else if(byte == UART_REQUEST_SEND_MPU6050_TEST_DATA )
          {
              motion_data_struct.Accelerometer_X = 1;
              motion_data_struct.Accelerometer_Y = 2;
              motion_data_struct.Accelerometer_Z = 3;
              motion_data_struct.Temperature     = 4;
              motion_data_struct.Gyroscope_X     = 5;
              motion_data_struct.Gyroscope_Y     = 6;
              motion_data_struct.Gyroscope_Z     = 7;
              HAL_StatusTypeDef status = HAL_UART_Transmit(&huart1, (uint8_t*)&motion_data_struct, sizeof(motion_data_struct), 1000);
              if(status != HAL_OK)
              {
                  // TODO: process error
              }
          }



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
