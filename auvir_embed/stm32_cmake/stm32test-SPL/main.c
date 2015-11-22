#include "stm32f10x.h"
#include "stm32f10x_i2c.h"
#include "MPU6050.h"
#include "I2CRoutines.h"

char * example_string = "Hello World!\r\n\0";
char* msg_conenction_success = "Connected!\r\n\0";
char* msg_conenction_fail = "Oops, connection failed.\r\n\0";
char* msg_MPU6050_I2C_Init = "MPU6050_I2C_Init!\r\n\0";
char* msg_MPU6050_Initialize = "MPU6050_Initialize!\r\n\0";

void _exit(void)
{
    while(1)
    {
        ;
    }
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

void blue_led_on()
{
    GPIOC->BSRR |= GPIO_BSRR_BS8;
}

void blue_led_off()
{
    GPIOC->BRR |= GPIO_BRR_BR8;
}

void init_button()
{
    // enable clock for port A (where button is located)
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    // setup button
    // input push/pull
    GPIOA->CRL &= (~GPIO_CRL_MODE0);
    GPIOA->CRL &= (~GPIO_CRL_CNF0);
    GPIOA->CRL |= GPIO_CRL_CNF0_1;
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

void usart_sent_byte(char ch)
{
    /*7. Write the data to send in the USART_DR register (this clears the TXE bit). Repeat this
for each data to be transmitted in case of single buffer.*/

    /*
   * Before write data to DR register, we check state of TXE (transmitter empty) flag. When this flag is clear, we can't write to DR register.
   * High value of this flag mean that is no data in transmit buffer (previous data just be sent or there is first tranmission) and we can write new data to send.
  */
    while(!(USART1->SR & USART_SR_TXE))
    {
        ;
    }
    USART1->DR = ch;
    /*8. After writing the last data into the USART_DR register, wait until TC=1. This indicates
that the transmission of the last frame is complete. This is required for instance when
the USART is disabled or enters the Halt mode to avoid corrupting the last
transmission.*/
    while ((USART1->SR & USART_SR_TC) == 0);

    // wait for TX
    //while ((USART1->SR & USART_SR_TXE) == 0);
}

void usart_send_str(char * str)
{
    while(*str != '\0')
    {
        usart_sent_byte(*str);
        str++;
    }
}

void button_led()
{

    init_led();
    init_button();
    init_uart();

    while(1)
    {
        unsigned int is_button_off = (GPIOA->IDR & 0x1);
        if( is_button_off == 0)
        {
            blue_led_on();
        }
        else
        {
            blue_led_off();
            usart_send_str((char *)example_string);
        }
    }
}




/** @addtogroup Optimized I2C examples
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
ErrorStatus HSEStartUpStatus;
/* Buffer of data to be received by I2C1 */
uint8_t Buffer_Rx1[255];
/* Buffer of data to be transmitted by I2C1 */
uint8_t Buffer_Tx1[255] = {0x5, 0x6,0x8,0xA};
/* Buffer of data to be received by I2C2 */
uint8_t Buffer_Rx2[255];
/* Buffer of data to be transmitted by I2C2 */
uint8_t Buffer_Tx2[255] = {0xF, 0xB, 0xC,0xD};
extern __IO uint8_t Tx_Idx1 , Rx_Idx1;
extern __IO uint8_t Tx_Idx2 , Rx_Idx2;

/* Private function prototypes -----------------------------------------------*/
void GPIO_Configuration(void);
void NVIC_Configuration(void);
/* Private functions ---------------------------------------------------------*/


short x,y,z;
static __IO uint32_t TimingDelay;

void Delay(uint32_t nTime);
void TimingDelay_Decrement(void);
void InitSysClock(void);


//
void GWriteReg(unsigned char reg, unsigned char value) // Write sensor reg
{
        Buffer_Tx1[0]=reg;
        Buffer_Tx1[1]=value;

        I2C_Master_BufferWrite(I2C1, Buffer_Tx1,2,Polling, MPU6050_DEFAULT_ADDRESS);
}
unsigned char GReadReg(unsigned char reg) // Read sensor reg
{
        Buffer_Tx1[0]=reg;
        I2C_Master_BufferWrite(I2C1, Buffer_Tx1,1,Interrupt, MPU6050_DEFAULT_ADDRESS);
        I2C_Master_BufferRead(I2C1,Buffer_Rx1,1,Polling, MPU6050_DEFAULT_ADDRESS);
        return Buffer_Rx1[0];
}
void GDataRead() // Read data from sensor
{
    MPU6050_t DataStruct;

        Buffer_Tx1[0] = MPU6050_RA_ACCEL_XOUT_H |(1<<7);
        if(I2C_Master_BufferWrite(I2C1, Buffer_Tx1, 1, DMA, MPU6050_DEFAULT_ADDRESS)==Success)
        {
                        if(I2C_Master_BufferRead(I2C1,Buffer_Rx1,6,DMA, MPU6050_DEFAULT_ADDRESS)==Success)
                        {
                            /* Format accelerometer data */
                            DataStruct.Accelerometer_X = (int16_t)(Buffer_Rx1[0] << 8 | Buffer_Rx1[1]);
                            DataStruct.Accelerometer_Y = (int16_t)(Buffer_Rx1[2] << 8 | Buffer_Rx1[3]);
                            DataStruct.Accelerometer_Z = (int16_t)(Buffer_Rx1[4] << 8 | Buffer_Rx1[5]);
                        }
        }
}

void GInit() // Init sensor
{
    // TODO: init sensor
    //GWriteReg(GYR_REG1,0x0F);
    //GWriteReg(GYR_REG4,0x20);

    MPU6050_GPIO_Init();
    MPU6050_I2C_Init();

    bool connected = MPU6050_TestConnection();

    MPU6050_Initialize();

    while (1)
    {
    }

    //MPU6050_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
}

int main(void)
{
	init_led();
    init_uart();
    char * str = "Helloooo!\r\n\0";
    char * str2 = "Buyyyyy!\r\n\0";
    while(1)
    {
        blue_led_on();
        usart_send_str(str);
        delay(10000);
        blue_led_off();
        usart_send_str(str2);
        delay(10000);

    }

    /* Use I2C1 as Slave */
    /*! When using Slave with DMA, uncomment //#define SLAVE_DMA_USE in the stm32f10x_it.c file.*/
    /*I2C_Slave_BufferReadWrite(I2C1, DMA);
                while(1); */
}

/**
  * @brief  Configures NVIC and Vector Table base location.
  * @param  None
  * @retval : None
  */
void InitSysClock(void)
{
        if (SysTick_Config(SystemCoreClock / 1000))
  {
     while (1);
  }
}
void NVIC_Configuration(void)
{
    /* 1 bit for pre-emption priority, 3 bits for subpriority */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    NVIC_SetPriority(I2C1_EV_IRQn, 0x00);
    NVIC_EnableIRQ(I2C1_EV_IRQn);

    NVIC_SetPriority(I2C1_ER_IRQn, 0x01);
    NVIC_EnableIRQ(I2C1_ER_IRQn);

    NVIC_SetPriority(I2C2_EV_IRQn, 0x00);
    NVIC_EnableIRQ(I2C2_EV_IRQn);

    NVIC_SetPriority(I2C2_ER_IRQn, 0x01);
    NVIC_EnableIRQ(I2C2_ER_IRQn);
}

#ifdef  USE_FULL_ASSERT


/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval : None
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
  * @}
  */
void Delay(uint32_t nTime)
{
  TimingDelay = nTime;

  while(TimingDelay != 0);
}
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
      TimingDelay--;
}
