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
#include "I2Cdev.h"
#include "MPU6050.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
UART_HandleTypeDef huart1;

typedef struct {
    int16_t Accelerometer_X;
    int16_t Accelerometer_Y;
    int16_t Accelerometer_Z;
    int16_t Gyroscope_X;
    int16_t Gyroscope_Y;
    int16_t Gyroscope_Z;
    int16_t Temperature;
} MPU6050_data_t;


/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define LED_PIN GPIO_PIN_8
#define LED_PORT GPIOC

const size_t max_string_lengh = 127;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
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
void combine_bytes(uint8_t h, uint8_t l, uint16_t result);
void mpu6050_read_to_struct(MPU6050_data_t* mpu6050data);
void mpu6050_send_data_uart(MPU6050_data_t* mpu6050data);
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
    MX_USART1_UART_Init();

    /* USER CODE BEGIN 2 */
    init_led_hal();

    I2Cdev_hi2c = &hi2c1; // init of i2cdevlib.
    // You can select other i2c device anytime and
    // call the same driver functions on other sensors

    MPU6050_setAddress(MPU6050_ADDRESS_AD0_LOW);
    while(!MPU6050_testConnection());
    MPU6050_initialize();

    /* USER CODE END 2 */

    /* Infinite loop */
    while (1)
    {
        /* USER CODE BEGIN WHILE */
        MPU6050_data_t mpu6050data = {0};
        mpu6050_read_to_struct(&mpu6050data);
        mpu6050_send_data_uart(&mpu6050data);
        //const char * str = "Test string\r\n\0";
        //bool result = usart_send_str_ok(buf);
        //HAL_StatusTypeDef trans = HAL_UART_Transmit(&huart1, (uint8_t*) temperature_reg_value, 2, 1000);
        HAL_Delay(300);
        /* USER CODE END WHILE */
    }

    /* USER CODE BEGIN 3 */
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
    // TODO: move to outside
    char str[max_string_lengh + 1];

    strncpy(str, input, sizeof(str));   // copy input without exceeding the length of the destination
    str[sizeof(str) - 1] = '\0';        // if strlen(input) == sizeof(str) then strncpy won't NUL terminate
    size_t bytes_to_sent = strlen(str) + 1; // length of input string + 1 if it is less than max_string_lengh; max_string_lengh otherwise

    // TODO: study signed/unsigned conversion below
    HAL_StatusTypeDef trans = HAL_UART_Transmit(&huart1, (uint8_t*) str, bytes_to_sent, 1000);
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
void combine_bytes(uint8_t h, uint8_t l, uint16_t result)
{
    result = (((int16_t) h) << 8) | l;
}
void mpu6050_read_to_struct(MPU6050_data_t *mpu6050data)
{
    mpu6050data->Accelerometer_X = MPU6050_getAccelerationX();
    mpu6050data->Accelerometer_Y = MPU6050_getAccelerationY();
    mpu6050data->Accelerometer_Z = MPU6050_getAccelerationZ();
    mpu6050data->Temperature =     MPU6050_getTemperature();
    mpu6050data->Gyroscope_X =     MPU6050_getRotationX();
    mpu6050data->Gyroscope_Y =     MPU6050_getRotationY();
    mpu6050data->Gyroscope_Z =     MPU6050_getRotationZ();
    // TODO: read all registers at once - from 0x3B (ACCEL_XOUT) to 0x48 (GYRO_ZOUT)
//    uint16_t rawdata[7]={0};
//    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(I2Cdev_hi2c, MPU6050_ADDRESS_AD0_LOW << 1, 0x3B, 2, (uint8_t*)&rawdata[0], 7, 3000);
//    MPU6050_data_t mpu6050rawdata;
//    mpu6050rawdata.Accelerometer_X = rawdata[0];
//    mpu6050rawdata.Accelerometer_Y = rawdata[1];
//    mpu6050rawdata.Accelerometer_Z = rawdata[2];
//    mpu6050rawdata.Temperature =     rawdata[3];
//    mpu6050rawdata.Gyroscope_X =     rawdata[4];
//    mpu6050rawdata.Gyroscope_Y =     rawdata[5];
//    mpu6050rawdata.Gyroscope_Z =     rawdata[6];

}

void mpu6050_send_data_uart(MPU6050_data_t* mpu6050data)
{
    uint8_t buf[sizeof(mpu6050data)+1]; // last byte for 0 (stop bit)
    memcpy(buf, &mpu6050data, sizeof(mpu6050data));
    buf[sizeof(mpu6050data)] = 0; // last byte for 0 (stop bit)
    HAL_StatusTypeDef trans = HAL_UART_Transmit(&huart1, buf, sizeof(buf), 1000);
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
