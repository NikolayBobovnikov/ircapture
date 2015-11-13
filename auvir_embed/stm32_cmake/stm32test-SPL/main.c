#include "stm32f10x.h"
#include "stm32f10x_i2c.h"
#include "MPU6050.h"

const char* const example_string = "Hello World!\r\n\0";
const char* const msg_conenction_success = "Connected!\r\n\0";
const char* const msg_conenction_fail = "Oops, connection failed.\r\n\0";
const char* const msg_MPU6050_I2C_Init = "MPU6050_I2C_Init!\r\n\0";
const char* const msg_MPU6050_Initialize = "MPU6050_Initialize!\r\n\0";

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
        USART1->BRR = (SystemCoreClock / 115200);   //9600 or 115200 bps
    //6. Set the TE bit in USART_CR1 to send an idle frame as first transmission.
        USART1->CR1 |= USART_CR1_TE;

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
            blue_led_on();
        }
        else
        {
            blue_led_off();
            usart_send_str((uint8_t *)example_string);
        }
    }
}



/// i2c functions
/*

void I2C_LowLevel_Init(I2C_TypeDef* I2Cx , int ClockSpeed , int
                       OwnAddress)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    I2C_InitTypeDef I2C_InitStructure;
    // Enable GPIOB clocks
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);
    // Configure I2C clock and GPIO
    GPIO_StructInit (& GPIO_InitStructure);
    if (I2Cx == I2C1){
        // I2C1 clock enable
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1 , ENABLE);
        // I2C1 SDA and SCL configuration
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
        GPIO_Init(GPIOB , &GPIO_InitStructure);
        // I2C1 Reset
        RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1 , ENABLE);
        RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1 , DISABLE);
    }
    else
    {
        // I2C2 ...
    }
    // Configure I2Cx
    I2C_StructInit (& I2C_InitStructure);
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = OwnAddress;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress =
            I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = ClockSpeed;
    I2C_Init(I2Cx , &I2C_InitStructure);
    I2C_Cmd(I2Cx , ENABLE);
}

#define Timed(x) Timeout = 0xFFFF; while (x) \
{ if (Timeout -- == 0) goto errReturn ;}


ErrorStatus I2C_Write(I2C_TypeDef* I2Cx , const uint8_t* buf ,
                 uint32_t nbyte , uint8_t SlaveAddress)
{
    __IO uint32_t Timeout = 0;
    if (nbyte)
    {
        Timed(I2C_GetFlagStatus(I2Cx , I2C_FLAG_BUSY));
        // Intiate Start Sequence
        I2C_GenerateSTART(I2Cx , ENABLE);
        Timed (! I2C_CheckEvent(I2Cx , I2C_EVENT_MASTER_MODE_SELECT));
        // Send Address EV5
        I2C_Send7bitAddress(I2Cx , SlaveAddress ,
                            I2C_Direction_Transmitter);
        Timed (! I2C_CheckEvent(I2Cx ,
                                I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
        // EV6 Write first byte EV8_1
        I2C_SendData(I2Cx , *buf ++);
        while (--nbyte) {
            // wait on BTF
            Timed (! I2C_GetFlagStatus(I2Cx , I2C_FLAG_BTF));
            I2C_SendData(I2Cx , *buf ++);
        }
        Timed (! I2C_GetFlagStatus(I2Cx , I2C_FLAG_BTF));
        I2C_GenerateSTOP(I2Cx , ENABLE);
        Timed(I2C_GetFlagStatus(I2C1 , I2C_FLAG_STOPF));
    }
    return SUCCESS;
errReturn:
    return ERROR;
}

ErrorStatus I2C_Read(I2C_TypeDef* I2Cx , uint8_t *buf ,
                uint32_t nbyte , uint8_t SlaveAddress)
{
    __IO uint32_t Timeout = 0;
    if (! nbyte)
        return SUCCESS;
    // Wait for idle I2C interface
    Timed(I2C_GetFlagStatus(I2Cx , I2C_FLAG_BUSY));
    // Enable Acknowledgment , clear POS flag
    I2C_AcknowledgeConfig(I2Cx , ENABLE);
    I2C_NACKPositionConfig(I2Cx , I2C_NACKPosition_Current);
    // Intiate Start Sequence (wait for EV5)
    I2C_GenerateSTART(I2Cx , ENABLE);
    Timed (! I2C_CheckEvent(I2Cx , I2C_EVENT_MASTER_MODE_SELECT));
    // Send Address
    I2C_Send7bitAddress(I2Cx , SlaveAddress , I2C_Direction_Receiver);
    // EV6
    Timed (! I2C_GetFlagStatus(I2Cx , I2C_FLAG_ADDR));

    // process 1, 2 and more bytes differently
    if (nbyte == 1)
    {
        // Clear Ack bit
        I2C_AcknowledgeConfig(I2Cx , DISABLE);
        // EV6_1 -- must be atomic -- Clear ADDR , generate STOP
        __disable_irq ();
        (void) I2Cx ->SR2;
        I2C_GenerateSTOP(I2Cx ,ENABLE);
        __enable_irq ();
        // Receive data EV7
        Timed (! I2C_GetFlagStatus(I2Cx , I2C_FLAG_RXNE));
        *buf++ = I2C_ReceiveData(I2Cx);
    }
    else if (nbyte == 2)
    {
        // Set POS flag
        I2C_NACKPositionConfig(I2Cx , I2C_NACKPosition_Next);
        // EV6_1 -- must be atomic and in this order
        __disable_irq ();
        (void) I2Cx ->SR2; // Clear ADDR flag
        I2C_AcknowledgeConfig(I2Cx , DISABLE); // Clear Ack bit
        __enable_irq ();
        // EV7_3 -- Wait for BTF , program stop , read data twice
        Timed (! I2C_GetFlagStatus(I2Cx , I2C_FLAG_BTF));
        __disable_irq ();
        I2C_GenerateSTOP(I2Cx ,ENABLE);
        *buf++ = I2Cx ->DR;
        __enable_irq ();
        *buf++ = I2Cx ->DR;
    }
    else {
        (void) I2Cx ->SR2; // Clear ADDR flag
        while (nbyte -- != 3)
        {
            // EV7 -- cannot guarantee 1 transfer completion time ,
            // wait for BTF instead of RXNE
            Timed (! I2C_GetFlagStatus(I2Cx , I2C_FLAG_BTF));
            *buf++ = I2C_ReceiveData(I2Cx);
        }
        Timed (! I2C_GetFlagStatus(I2Cx , I2C_FLAG_BTF));
        // EV7_2 -- Figure 1 has an error , doesn 't read N-2 !
        I2C_AcknowledgeConfig(I2Cx , DISABLE); // clear ack bit
        __disable_irq ();
        *buf++ = I2C_ReceiveData(I2Cx); // receive byte N-2
        I2C_GenerateSTOP(I2Cx ,ENABLE); // program stop
        __enable_irq ();
        *buf++ = I2C_ReceiveData(I2Cx); // receive byte N-1
        // wait for byte N
        Timed (! I2C_CheckEvent(I2Cx , I2C_EVENT_MASTER_BYTE_RECEIVED));
        *buf++ = I2C_ReceiveData(I2Cx);
        nbyte = 0;
    }
    // Wait for stop
    Timed(I2C_GetFlagStatus(I2Cx , I2C_FLAG_STOPF));
    return SUCCESS;
errReturn:
    return ERROR;
}

/// i2c functions
*/

int main()
{
    //button_led();
    init_led();
    init_button();
    init_uart();

    // wait to press button
    bool button_pressed = false;
    while(!button_pressed)
    {
        unsigned int is_button_off = (GPIOA->IDR & 0x1);
        if( is_button_off != 0)
        {
            button_pressed = true;
        }
    }

    /*
    // Init
    #define MPU6050_ADDRESS 0x69
    const uint8_t buf[] = {0xf0 , 0x55};
    const uint8_t buf2[] = {0xfb , 0x00};
    I2C_Write(I2C1 , buf , 2, MPU6050_ADDRESS);
    I2C_Write(I2C1 , buf2 , 2, MPU6050_ADDRESS);

    // Read
    uint8_t data [6];
    const uint8_t buf3[] = {0};
    I2C_Write(I2C1 , buf3 , 1, MPU6050_ADDRESS);
    I2C_Read(I2C1 , data , 6, MPU6050_ADDRESS);
    */

    usart_send_str(msg_MPU6050_I2C_Init);
    MPU6050_I2C_Init();

    usart_send_str(msg_MPU6050_Initialize);
    MPU6050_Initialize();
    int connection_ok = MPU6050_TestConnection();
    //MPU6050_Write();
    //MPU6050_Read();




    while(1)
    {
        unsigned int is_button_off = (GPIOA->IDR & 0x1);
        bool message_sent = false;
        if( is_button_off == 0)
        {
            blue_led_off();
        }
        else
        {
            blue_led_on();
            usart_send_str((uint8_t *)example_string);
            /*
            if(!message_sent)
            {
                // test connection and send a message
                if(MPU6050_TestConnection())
                {
                    usart_send_str((uint8_t *)msg_conenction_success);
                }
                else
                {
                    usart_send_str((uint8_t *)msg_conenction_fail);
                }
                usart_send_str((uint8_t *)example_string);
                message_sent = true;
            }
            */
        }
    }


    return 0;
}
