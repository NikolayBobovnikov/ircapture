#include "stm32f10x.h"
#include "mpu6050.h"

const char* const example_string = "Hello World!\r\n\0";

void _exit(void)
{
    while(1)
    {
        ;
    }
}


void delay(unsigned int num)
{
    unsigned int i;
    for (i = 0; i < num; i++)
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
        USART1->BRR = (SystemCoreClock / 115200);   //115200 bps
    //6. Set the TE bit in USART_CR1 to send an idle frame as first transmission.
        USART1->CR1 |= USART_CR1_TE;


        USART1->CR1 |= USART_CR1_RE;


}

void usart_sent_byte(uint8_t ch)
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

void usart_send_str(uint8_t * str)
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
            blue_led_off();
        }
        else
        {
            blue_led_on();
            usart_send_str((uint8_t *)example_string);
        }
    }
}

void MPU6050_Initialize(void)
{
MPU6050_Write(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, 1<<7);//reset the whole module first

delay(50);	//wait for 50ms for the gyro to stable

MPU6050_Write(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_CLOCK_PLL_ZGYRO);//PLL with Z axis gyroscope reference

MPU6050_Write(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_CONFIG, 0x01);		//DLPF_CFG = 1: Fs=1khz; bandwidth=42hz

MPU6050_Write(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_SMPLRT_DIV, 0x01);	//500Hz sample rate ~ 2ms

MPU6050_Write(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, MPU6050_GYRO_FS_2000);	//Gyro full scale setting

MPU6050_Write(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACCEL_FS_16);	//Accel full scale setting

MPU6050_Write(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_INT_PIN_CFG, 1<<4);		//interrupt status bits are cleared on any read operation

MPU6050_Write(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_INT_ENABLE, 1<<0);		//interupt occurs when data is ready. The interupt routine is in the receiver.c file.

MPU6050_Write(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_SIGNAL_PATH_RESET, 0x07);//reset gyro and accel sensor
}
void MPU6050_Write(uint8_t slaveAddr, uint8_t regAddr, uint8_t data)
{
    uint8_t tmp;
    tmp = data;
    MPU6050_I2C_ByteWrite(slaveAddr,&tmp,regAddr);
}
//------------------------------------------------------------------
void MPU6050_I2C_ByteWrite(u8 slaveAddr, u8* pBuffer, u8 writeAddr)
{

/* Send START condition */
I2C_GenerateSTART(MPU6050_I2C, ENABLE);
/* Test on EV5 and clear it */
while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_MODE_SELECT));
/* Send MPU6050 address for write */
I2C_Send7bitAddress(MPU6050_I2C, slaveAddr, I2C_Direction_Transmitter);
/* Test on EV6 and clear it */
while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
/* Send the MPU6050's internal address to write to */
I2C_SendData(MPU6050_I2C, writeAddr);
/* Test on EV8 and clear it */
//while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTING));
/* Send the byte to be written */
if (pBuffer!=0) I2C_SendData(MPU6050_I2C, pBuffer);
/* Test on EV8_2 and clear it */
while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
/* Send STOP condition */
I2C_GenerateSTOP(MPU6050_I2C, ENABLE);

}
NVIC_InitTypeDef NVIC_InitStructure;
DMA_InitTypeDef  DMA_InitStructure;

DMA_DeInit(MPU6050_DMA_Channel); //reset DMA1 channe1 to default values;

DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)I2C_DR_Address; //=0x40005410 : address of data reading register of I2C1
DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)I2C_Rx_Buffer; //variable to store data
DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; //channel will be used for peripheral to memory transfer
DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;	//setting normal mode (non circular)
DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;	//medium priority
DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;	//Location assigned to peripheral register will be source
DMA_InitStructure.DMA_BufferSize = 14;	//number of data to be transfered
DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; //automatic memory increment disable for peripheral
DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;	//automatic memory increment enable for memory
DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;	//source peripheral data size = 8bit
DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;	//destination memory data size = 8bit
DMA_Init(MPU6050_DMA_Channel, &DMA_InitStructure);
DMA_ITConfig(MPU6050_DMA_Channel, DMA_IT_TC, ENABLE);

NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel7_IRQn; //I2C1 connect to channel 7 of DMA1
NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x05;
NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x05;
NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
NVIC_Init(&NVIC_InitStructure);

int main()
{
    button_led();
    return 0;
}
