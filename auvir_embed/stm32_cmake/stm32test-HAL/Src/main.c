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

    float angle_x;
    float angle_y;
    float angle_z;
} MPU6050_MotionData_t;

typedef struct
{
    int mean_ax;
    int mean_ay;
    int mean_az;
    int mean_gx;
    int mean_gy;
    int mean_gz;
    int var_ax;
    int var_ay;
    int var_az;
    int var_gx;
    int var_gy;
    int var_gz;
    // TODO: offsets
    int16_t offset_ax;
    int16_t offset_ay;
    int16_t offset_az;
    int16_t offset_gx;
    int16_t offset_gy;
    int16_t offset_gz;
} MPU6050_CalibrationData_t;

enum UART_Commands {
    UART_COMMAND_NOT_RECEIVED = 0,
    UART_REQUEST_SEND,
    UART_REQUEST_STOP,
    UART_REQUEST_SEND_BYTE,
    UART_REQUEST_SEND_MPU6050_TEST_DATA,
    UART_REQUEST_SEND_MPU6050_DATA,
    UART_REQUEST_CALIB_DATA,

    UART_TEST_CONNECTION,			// request
    UART_CONNECTION_FAILURE,
    UART_CONENCTION_OK,

    UART_MPU6050_TEST_CONNECTION,	// request
    UART_MPU6050_CONENCTION_FAILURE,
    UART_MPU6050_CONENCTION_OK,
    UART_NULL_RESPONSE,

    UART_MPU6050_RESET,              // request
    UART_MPU6050_RESET_OK,
    UART_MPU6050_RESET_FAILURE
};

static MPU6050_MotionData_t motion_data;
static MPU6050_CalibrationData_t calibration_data;
static uint8_t motion_data_buffer[sizeof(MPU6050_MotionData_t)];
const size_t max_string_lengh = 127;

// TODO: define frequency automatically based on chip name (thus based on #defined symbol like STM32F100xB)
#define CHIP_FREQUENCY    24000000
//Timer setup. timer frequency = (chip frequency / prescaler) = (24Mhz / 24) = 1 Mhz 1*10^6
#define TIMER_PRESCALER     24
#define TIMER_RESET_PERIOD  1000000

const int chip_frequency = CHIP_FREQUENCY;
const int timer_prescaler =  TIMER_PRESCALER;
const int timer_reset_period = TIMER_RESET_PERIOD;
const float timer_frequency = CHIP_FREQUENCY / TIMER_PRESCALER;
const float timer_period = TIMER_PRESCALER / CHIP_FREQUENCY;

const float filter_gain = 0.95;

// These variables are private, used in MPU6050_GetAllData(), don't use them directly.
// Use MPU6050_ResetCalculatedData() to init them to default values before using sensor
float timeStep= TIMER_PRESCALER / CHIP_FREQUENCY;
bool  is_firts_sample = true;
float acc_angle_x = 0.0;
float acc_angle_y = 0.0;
float acc_angle_z = 0.0;
float gyro_angular_x = 0.0;
float gyro_angular_y = 0.0;
float gyro_angular_z = 0.0;
float gyro_integrated_angle_x = 0.0;
float gyro_integrated_angle_y = 0.0;
float gyro_integrated_angle_z = 0.0;
// *



#define PI 3.14159265358979323846
static const float radian = 180 / PI;
static const int gyroScale = 131;



//Change this 3 variables if you want to fine tune the skecth to your needs.
static const int number_of_samples=200;     //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
static const int acel_deadzone=50;     //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
static const int gyro_deadzone=10;     //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)

static int16_t ax, ay, az,gx, gy, gz;
static int state=0;


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
int ipow(int base, int exp);
int square(int base);
void MPU6050_getAllData();
void usart_wait_exec_loop();
void mpu6050_loop();
void meansensors();
void calibration();
void MPU6050_ResetCalculatedData();
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
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
    MPU6050_resetSensors();
    MPU6050_ResetCalculatedData();
    MPU6050_initialize();

    /* USER CODE ENDint ipow(int a, int b); 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        //usart_wait_exec_loop();
        //mpu6050_loop();
    	led_on();
    	HAL_Delay(200);
    	led_off();
    	HAL_Delay(200);
    }
    /* USER CODE END 3 */

}

/* USER CODE END 0 */

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
    htim4.Init.Prescaler = timer_prescaler;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = timer_period;
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
    // = = = = = = = = = = = = = = = = = =
    // enable clock for gpio port C
    // = = = = = = = = = = = = = = = = = =
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;

    // = = = = = = = = = = = = = = = = = =
    // setup GPIO port C pin 8 for output
    // = = = = = = = = = = = = = = = = = =
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
    string[sizeof(string) - 1] = '\0';        // if strlen(input) = sizeof(str) then strncpy won't NUL terminate
    size_t bytes_to_sent = strlen(string) + 1; // length of input string + 1 if it is less than max_string_lengh; max_string_lengh otherwise

    // TODO: study signed/unsigned conversion below
    HAL_StatusTypeDef trans = HAL_UART_Transmit(&huart1, (uint8_t*) string, bytes_to_sent, 1000);
    if(HAL_OK == trans)
    {
        return true;
    }
    return false;
}
void usart_send_str(const char *input)
{
    usart_send_str_ok(input);
}
int ipow(int base, int exp)
{
    int result = 1;
    while (exp)
    {
        if (exp & 1)
            result *= base;
        exp >>= 1;
        base *= base;
    }

    return result;
}
int square(int base)
{
    return ipow(base, 2);
}
void MPU6050_getAllData()
{
    // get time of arrival
    motion_data.delta_time = __HAL_TIM_GET_COUNTER(&htim4);
    // get new data to buffer
    I2Cdev_readBytes(MPU6050_ADDRESS_AD0_LOW, MPU6050_RA_ACCEL_XOUT_H, 14, motion_data_buffer, 100);
    // reset timer
    __HAL_TIM_SET_COUNTER(&htim4, 0);

    // fill struct from buffer
    motion_data.Accelerometer_X = (((int16_t)motion_data_buffer[0]) << 8) | motion_data_buffer[1];
    motion_data.Accelerometer_Y = (((int16_t)motion_data_buffer[2]) << 8) | motion_data_buffer[3];
    motion_data.Accelerometer_Z = (((int16_t)motion_data_buffer[4]) << 8) | motion_data_buffer[5];
    motion_data.Temperature     = (((int16_t)motion_data_buffer[6]) << 8) | motion_data_buffer[7];
    motion_data.Gyroscope_X     = (((int16_t)motion_data_buffer[8]) << 8) | motion_data_buffer[9];
    motion_data.Gyroscope_Y     = (((int16_t)motion_data_buffer[10])<< 8) | motion_data_buffer[11];
    motion_data.Gyroscope_Z     = (((int16_t)motion_data_buffer[12])<< 8) | motion_data_buffer[13];

    // delta t
    timeStep = motion_data.delta_time * timer_period;

    // apply gyro scale from datasheet
    gyro_angular_x = motion_data.Gyroscope_X / gyroScale;
    gyro_angular_y = motion_data.Gyroscope_Y / gyroScale;
    gyro_angular_z = motion_data.Gyroscope_Z / gyroScale;

    // calculate accelerometer angles
    acc_angle_x = atan(motion_data.Accelerometer_X / sqrt(square(motion_data.Accelerometer_Y) + square(motion_data.Accelerometer_Z)));
    acc_angle_y = atan(motion_data.Accelerometer_Y / sqrt(square(motion_data.Accelerometer_X) + square(motion_data.Accelerometer_Z)));
    acc_angle_z = atan(sqrt(square(motion_data.Accelerometer_Y) + square(motion_data.Accelerometer_X)) / motion_data.Accelerometer_Z);

    acc_angle_x *= radian;
    acc_angle_y *= radian;
    acc_angle_z *= radian;

    // set initial values equal to accel values
    if (is_firts_sample)
    {
        is_firts_sample = false;
        gyro_integrated_angle_x = acc_angle_x;
        gyro_integrated_angle_y = acc_angle_y;
        gyro_integrated_angle_z = acc_angle_z;
    }
    // integrate to find the gyro angle
    else
    {
        gyro_integrated_angle_x = gyro_integrated_angle_x + (timeStep * gyro_angular_x);
        gyro_integrated_angle_y = gyro_integrated_angle_y + (timeStep * gyro_angular_y);
        gyro_integrated_angle_z = gyro_integrated_angle_z + (timeStep * gyro_angular_z);
    }

    // apply filter
    motion_data.angle_x = ((1-filter_gain) * acc_angle_x) + (filter_gain * gyro_integrated_angle_x);
    motion_data.angle_y = ((1-filter_gain) * acc_angle_y) + (filter_gain * gyro_integrated_angle_y);
    motion_data.angle_z = ((1-filter_gain) * acc_angle_z) + (filter_gain * gyro_integrated_angle_z);

}

void usart_wait_exec_loop()
{
    HAL_StatusTypeDef status;
    while( 1 )
    {
        //reset command
        uint8_t command = UART_COMMAND_NOT_RECEIVED;
        uint8_t response = UART_NULL_RESPONSE;
        status = HAL_UART_Receive(&huart1, &command, 1, 1000);

        // if received command, dispatch it
        if(status == HAL_OK && UART_COMMAND_NOT_RECEIVED != command)
        {
            if(UART_REQUEST_SEND_MPU6050_DATA == command)
            {
                MPU6050_getAllData();
                HAL_StatusTypeDef status = HAL_UART_Transmit(&huart1, (uint8_t*)&motion_data, sizeof(motion_data), 1000);
                if(status != HAL_OK)
                {
                    // TODO: process error
                }
            }
            else if(UART_REQUEST_SEND_MPU6050_TEST_DATA == command)
            {
                motion_data.Accelerometer_X = 1;
                motion_data.Accelerometer_Y = 2;
                motion_data.Accelerometer_Z = 3;
                motion_data.Temperature    = 4;
                motion_data.Gyroscope_X    = 5;
                motion_data.Gyroscope_Y    = 6;
                motion_data.Gyroscope_Z    = 7;
                status = HAL_UART_Transmit(&huart1, (uint8_t*)&motion_data, sizeof(motion_data), 1000);
            }
            else if (UART_REQUEST_CALIB_DATA == command)
            {
                meansensors();
                calibration();
                meansensors();

                status = HAL_UART_Transmit(&huart1, (uint8_t*)&calibration_data, sizeof(calibration_data), 1000);
            }
            else if(UART_TEST_CONNECTION == command)
            {
                response = UART_CONENCTION_OK;
                status = HAL_UART_Transmit(&huart1, &response, sizeof(response), 1000);
            }
            else if(UART_MPU6050_TEST_CONNECTION == command)
            {
                if(MPU6050_testConnection())
                {
                    response = UART_MPU6050_CONENCTION_OK;
                }
                else
                {
                    response = UART_MPU6050_CONENCTION_FAILURE;
                }
                status = HAL_UART_Transmit(&huart1, &response, sizeof(response), 1000);
            }
            else if(UART_MPU6050_RESET == command)
            {
                MPU6050_resetSensors();
                MPU6050_ResetCalculatedData();
                MPU6050_initialize();

                // TODO: detect error
                // if(error) response = response = UART_MPU6050_RESET_FAILURE;
                response = UART_MPU6050_RESET_OK;

                status = HAL_UART_Transmit(&huart1, &response, sizeof(response), 1000);
            }

            // Process error
            if(status != HAL_OK)
            {
                // TODO: process error
            }
        }

        //reset command
        //byte = UART_COMMAND_NOT_RECEIVED
    } // end while
}

void mpu6050_loop()
{
    HAL_StatusTypeDef status;
    if (state==0)
    {
        //usart_send_str("\nReading sensors for first time...\0");
        meansensors();
        state++;
        HAL_Delay(1000);
      }

    if (state==1)
      {
        //usart_send_str("\nCalculating offsets...\0");
        calibration();
        state++;
        HAL_Delay(1000);
      }

    if (state==2)
      {
        meansensors();

        status = HAL_UART_Transmit(&huart1, (uint8_t*)&calibration_data, sizeof(calibration_data), 1000);
                    if(status != HAL_OK)
                    {
                         // TODO: process error
                    }
        // go back
        state = 0;
      }
}

void meansensors()
{
    long i = 0, buff_ax = 0, buff_ay = 0, buff_az = 0, buff_gx = 0, buff_gy = 0, buff_gz = 0;

    // array of samples, each sample - array of values
    uint8_t sample_array[number_of_samples][sizeof(MPU6050_MotionData_t)];

    // collect data
    HAL_Delay(200);
    for (i = 0; i < number_of_samples; i++)
    {
        I2Cdev_readBytes(MPU6050_ADDRESS_AD0_LOW, MPU6050_RA_ACCEL_XOUT_H, 14, &(sample_array[i])[0], 100);
        HAL_Delay(2);// Wait so that not get repeated samples
    }

    // calculate mean
    for (i = 0; i < number_of_samples; i++)
    {
        ax = (((int16_t)sample_array[i][0]) << 8) | sample_array[i][1];
        ay = (((int16_t)sample_array[i][2]) << 8) | sample_array[i][3];
        az = (((int16_t)sample_array[i][4]) << 8) | sample_array[i][5];
        gx = (((int16_t)sample_array[i][8]) << 8) | sample_array[i][9];
        gy = (((int16_t)sample_array[i][10])<< 8) | sample_array[i][11];
        gz = (((int16_t)sample_array[i][12])<< 8) | sample_array[i][13];

        buff_ax = buff_ax + ax;
        buff_ay = buff_ay + ay;
        buff_az = buff_az + az;
        buff_gx = buff_gx + gx;
        buff_gy = buff_gy + gy;
        buff_gz = buff_gz + gz;

        if (i == number_of_samples - 1)
        {
            calibration_data.mean_ax = buff_ax/number_of_samples;
            calibration_data.mean_ay = buff_ay/number_of_samples;
            calibration_data.mean_az = buff_az/number_of_samples;
            calibration_data.mean_gx = buff_gx/number_of_samples;
            calibration_data.mean_gy = buff_gy/number_of_samples;
            calibration_data.mean_gz = buff_gz/number_of_samples;
        }
    }

    // calculate variance
    int squared_var_ax = 0;
    int squared_var_ay = 0;
    int squared_var_az = 0;
    int squared_var_gx = 0;
    int squared_var_gy = 0;
    int squared_var_gz = 0;

    for (i = 0; i < number_of_samples; i++)
    {
        ax = (((int16_t)motion_data_buffer[0]) << 8) | motion_data_buffer[1];
        ay = (((int16_t)motion_data_buffer[2]) << 8) | motion_data_buffer[3];
        az = (((int16_t)motion_data_buffer[4]) << 8) | motion_data_buffer[5];
        gx = (((int16_t)motion_data_buffer[8]) << 8) | motion_data_buffer[9];
        gy = (((int16_t)motion_data_buffer[10])<< 8) | motion_data_buffer[11];
        gz = (((int16_t)motion_data_buffer[12])<< 8) | motion_data_buffer[13];

        squared_var_ax = squared_var_ax + ipow((calibration_data.mean_ax - ax), 2);
        squared_var_ay = squared_var_ay + ipow((calibration_data.mean_ay - ay), 2);
        squared_var_az = squared_var_az + ipow((calibration_data.mean_az - az), 2);
        squared_var_gx = squared_var_gx + ipow((calibration_data.mean_gx - gx), 2);
        squared_var_gy = squared_var_gy + ipow((calibration_data.mean_gy - gy), 2);
        squared_var_gz = squared_var_gz + ipow((calibration_data.mean_gz - gz), 2);

        if (i == number_of_samples - 1)
        {
            calibration_data.var_ax = sqrt(squared_var_ax);
            calibration_data.var_ay = sqrt(squared_var_ay);
            calibration_data.var_az = sqrt(squared_var_az);
            calibration_data.var_gx = sqrt(squared_var_gx);
            calibration_data.var_gy = sqrt(squared_var_gy);
            calibration_data.var_gz = sqrt(squared_var_gz);
        }
    }

}

void calibration()
{
    calibration_data.offset_ax = calibration_data.mean_ax / acel_deadzone;
    calibration_data.offset_ay = calibration_data.mean_ay / acel_deadzone;
    calibration_data.offset_az = calibration_data.mean_az / acel_deadzone - 16384;//16384
    calibration_data.offset_gx = calibration_data.mean_gx / gyro_deadzone;
    calibration_data.offset_gy = calibration_data.mean_gy / gyro_deadzone;
    calibration_data.offset_gz = calibration_data.mean_gz / gyro_deadzone;

    int mean_ax_prev = 0;
    int mean_ay_prev = 0;
    int mean_az_prev = 0;
    int mean_gx_prev = 0;
    int mean_gy_prev = 0;
    int mean_gz_prev = 0;

    int iter_numer = 10;
    while (iter_numer-- != 0)
    {
        int ready = 0;
        MPU6050_setXAccelOffset(calibration_data.offset_ax);
        MPU6050_setYAccelOffset(calibration_data.offset_ay);
        MPU6050_setZAccelOffset(calibration_data.offset_az);
        MPU6050_setXGyroOffset (calibration_data.offset_gx);
        MPU6050_setYGyroOffset (calibration_data.offset_gy);
        MPU6050_setZGyroOffset (calibration_data.offset_gz);

        mean_ax_prev = calibration_data.offset_ax;
        mean_ay_prev = calibration_data.offset_ay;
        mean_az_prev = calibration_data.offset_az;
        mean_gx_prev = calibration_data.offset_gx;
        mean_gy_prev = calibration_data.offset_gy;
        mean_gz_prev = calibration_data.offset_gz;

        meansensors();

        if (abs(calibration_data.mean_ax)<= acel_deadzone) ready++ ;
        else
        {
            if (calibration_data.mean_ax < mean_ax_prev)
            {
                calibration_data.offset_ax = calibration_data.offset_ax + calibration_data.mean_ax / acel_deadzone;
            }
            else
            {
                calibration_data.offset_ax = calibration_data.offset_ax - calibration_data.mean_ax / acel_deadzone;
            }
            mean_ax_prev = calibration_data.mean_ax;
        }

        if (abs(calibration_data.mean_ay)<= acel_deadzone) ready++ ;
        else
        {
            //calibration_data.offset_ay = calibration_data.offset_ay - calibration_data.mean_ay / acel_deadzone;
            if (calibration_data.mean_ay < mean_ay_prev)
            {
                calibration_data.offset_ay = calibration_data.offset_ay + calibration_data.mean_ay / acel_deadzone;
            }
            else
            {
                calibration_data.offset_ay = calibration_data.offset_ay - calibration_data.mean_ay / acel_deadzone;
            }
            mean_ay_prev = calibration_data.mean_ay;
        }


        if (abs(16384 - calibration_data.mean_az)<= acel_deadzone) ready++ ;
        else
        {
            //calibration_data.offset_az = calibration_data.offset_az + (16384 - calibration_data.mean_az) / acel_deadzone;
            if (calibration_data.mean_az < mean_az_prev)
            {
                calibration_data.offset_az = calibration_data.offset_az + (calibration_data.mean_az - 16384) / acel_deadzone;
            }
            else
            {
                calibration_data.offset_az = calibration_data.offset_az - (calibration_data.mean_az - 16384) / acel_deadzone;
            }
            mean_az_prev = calibration_data.mean_az;
        }

        if (abs(calibration_data.mean_gx)<= gyro_deadzone) ready++ ;
        else
        {
            if (calibration_data.mean_gx < mean_gx_prev)
            {
                calibration_data.offset_gx = calibration_data.offset_gx + calibration_data.mean_gx / gyro_deadzone;
            }
            else
            {
                calibration_data.offset_gx = calibration_data.offset_gx - calibration_data.mean_gx / gyro_deadzone;
            }
            mean_gx_prev = calibration_data.mean_gx;
        }

        if (abs(calibration_data.mean_gy)<= gyro_deadzone) ready++ ;
        else
        {
            if (calibration_data.mean_gy < mean_gy_prev)
            {
                calibration_data.offset_gy = calibration_data.offset_gy + calibration_data.mean_gy / gyro_deadzone;
            }
            else
            {
                calibration_data.offset_gy = calibration_data.offset_gy - calibration_data.mean_gy / gyro_deadzone;
            }
            mean_gy_prev = calibration_data.mean_gy;
        }

        if (abs(calibration_data.mean_gz)<= gyro_deadzone) ready++ ;
        else
        {
            if (calibration_data.mean_gz < mean_gz_prev)
            {
                calibration_data.offset_gz = calibration_data.offset_gz + calibration_data.mean_gz / gyro_deadzone;
            }
            else
            {
                calibration_data.offset_gz = calibration_data.offset_gz - calibration_data.mean_gz / gyro_deadzone;
            }
            mean_gz_prev = calibration_data.mean_gz;
        }

        if (ready == 6) break;

    }
}

void MPU6050_ResetCalculatedData()
{
    memset(&motion_data, 0, sizeof(motion_data));

    is_firts_sample = true;
    timeStep= TIMER_PRESCALER / CHIP_FREQUENCY;
    gyro_integrated_angle_x = 0.0;
    gyro_integrated_angle_y = 0.0;
    gyro_integrated_angle_z = 0.0;
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
