//this is a copy and paste job made by F2k

#include "stm32f1xx_hal.h"
#include "se8r01.h"

///
/// on initialization se8r01 check http://forum.easyelectronics.ru/viewtopic.php?f=9&t=21484
///

extern SPI_HandleTypeDef hspi1;
extern char mode;      //r=rx, t=tx
void delay_us(uint8_t us);
uint8_t gtemp[5];

// Define a static TX address
uint8_t TX_ADDRESS[TX_ADR_WIDTH]  = {0x10,0x20,0x30,0xab,0xab};
uint8_t rx_buf[TX_PLOAD_WIDTH] = {0}; // initialize value
uint8_t tx_buf[TX_PLOAD_WIDTH] = {0};

//===============  Function prototypes
static void nrf24_ce_set(uint8_t state);
static void nrf24_csn_set(uint8_t state);
static void radio_settings();
static void set_rx_tx_mode();
static void init_io();

static void se8r01_switch_bank(uint8_t bankindex);
static void se8r01_powerup();
static void se8r01_calibration();
static void se8r01_setup();

static void power_off();
static void power_on_tx();
static void power_on_rx();
//

// use in dynamic length mode //
static uint8_t nrf24_get_rx_fifo_pending_data_length();

// post transmission analysis //
static TransmissionStatus nrf24_last_messageStatus();
static uint8_t nrf24_get_last_msg_retransmission_count();

// power management //
static void nrf24_powerDown();
static void nrf24_reset();

// low level interface ... //
static uint8_t SPI_RW(uint8_t tx);
static uint8_t SPI_RW_Reg(uint8_t reg, uint8_t value);
static uint8_t SPI_Read(uint8_t reg);
static uint8_t SPI_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t bytes);
static uint8_t SPI_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t bytes);

static void nrf24_transmitSync(uint8_t* dataout,uint8_t len);
static void nrf24_transferSync(uint8_t* dataout,uint8_t* datain,uint8_t len);

static void nrf24_read_register_buf(uint8_t reg, uint8_t* value, uint8_t len);
static void nrf24_write_register_buf(uint8_t reg, uint8_t* value, uint8_t len);
static void nrf24_write_register(uint8_t reg, uint8_t value);

//=============== Function definitions
uint8_t nrf24_get_rx_fifo_pending_data_length()
{
    uint8_t status;
    nrf24_csn_set(LOW);
    SPI_RW(R_RX_PL_WID);
    status = SPI_RW(0x00);
    nrf24_csn_set(HIGH);
    return status;
}

bool nrf24_is_data_ready()
{
    // See note in getData() function - just checking RX_DR isn't good enough
    uint8_t status = SPI_Read(iRF_BANK0_STATUS);

    // We can short circuit on RX_DR, but if it's not set, we still need
    // to check the FIFO for any pending packets
    if ( status & (1 << RX_DR) )
    {
        return true;
    }

    return !nrf24_is_rx_fifo_empty();
}

bool nrf24_is_rx_fifo_empty()
{
    uint8_t fifoStatus;

    nrf24_read_register_buf(FIFO_STATUS, &fifoStatus,1);

    //return (fifoStatus & (1 << RX_EMPTY)); // TODO: verify correctness
    if(fifoStatus & (1 << RX_EMPTY)){
        return true;
    }
    return false;
}

uint8_t nrf24_get_last_msg_retransmission_count()
{
    uint8_t rv;
    nrf24_read_register_buf(OBSERVE_TX,&rv,1);
    rv = rv & 0x0F;
    return rv;
}

TransmissionStatus nrf24_last_messageStatus()
{
    uint8_t rv = SPI_Read(iRF_BANK0_STATUS);
    // 0x1e=0b00011110
    // Transmission went OK //
    if(rv & iSTATUS_TX_DS)
    {
        return NRF24_TRANSMISSON_OK;
    }
    // Maximum retransmission count is reached //
    // Last message probably went missing ... //
    if(rv & iSTATUS_MAX_RT)
    {
        return NRF24_MESSAGE_LOST;
    }
    // Probably still sending ... //
    else
    {
        return NRF24_MESSAGE_SENDING;
    }
}

void nrf24_powerDown()
{
    nrf24_ce_set(LOW);
    nrf24_write_register(CONFIG,0<<PWR_UP);
}

void nrf24_reset()
{

    //1)use power down mode (PWR_UP = 0)
    nrf24_ce_set(LOW);
    nrf24_write_register(CONFIG,0<<PWR_UP);

    //2)clear data ready flag and data sent flag in status register
    nrf24_write_register(STATUS,(1<<RX_DR)|(1<<TX_DS)|(1<<MAX_RT));

    //3)flush tx/rx buffer
    nrf24_csn_set(LOW);
    SPI_RW(FLUSH_RX);
    nrf24_csn_set(HIGH);

    //4)write status register as 0x0e;
    nrf24_write_register(STATUS,0x0E);
}


//========================================
// Interface functions

static void nrf24_ce_set(uint8_t state)
{
    assert_param(state == LOW || state == HIGH);
    HAL_GPIO_WritePin(NRF24_CE_PORT,NRF24_CE_PIN, state);
    delay_us(10);
}

static void nrf24_csn_set(uint8_t state)
{
    assert_param(state == LOW || state == HIGH);
    HAL_GPIO_WritePin(NRF24_CSN_PORT,NRF24_CSN_PIN, state);
    if(state == HIGH){
        delay_us(100);
    }
    else{
        delay_us(11);
    }
}

// send and receive multiple bytes over SPI //
static void nrf24_transferSync(uint8_t* dataout,uint8_t* datain,uint8_t len)
{
    uint8_t i;

    for(i=0;i<len;i++)
    {
        datain[i] = SPI_RW(dataout[i]);
    }

}

// send multiple bytes over SPI //
static void nrf24_transmitSync(uint8_t* dataout,uint8_t len)
{
    uint8_t i;
    for(i=0;i<len;i++)
    {
        SPI_RW(dataout[i]);
    }

}

void nrf24_setup_gpio(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    //Configure IRQ pin
    GPIO_InitStruct.Pin = NRF24_IRQ_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(NRF24_IRQ_PORT, &GPIO_InitStruct);

    //HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
    //HAL_NVIC_EnableIRQ(EXTI4_IRQn);

    //Configure CSN pin
    GPIO_InitStruct.Pin = NRF24_CSN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(NRF24_CSN_PORT, &GPIO_InitStruct);

    //Configure CE pin
    GPIO_InitStruct.Pin = NRF24_CE_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(NRF24_CE_PORT, &GPIO_InitStruct);

    /* CSN high = disable SPI */
    HAL_GPIO_WritePin(NRF24_CSN_PORT, NRF24_CSN_PIN, GPIO_PIN_SET);

    /* CE low = disable TX/RX */
    HAL_GPIO_WritePin(NRF24_CE_PORT, NRF24_CE_PIN, GPIO_PIN_RESET);
}

void setup()
{
    init_io();                        // Initialize IO port
    HAL_Delay(5);

    //set EN_AA, EN_RXADDR, RF_CH (needless?), RF_SETUP (needless?), AW, SETUP_RETR, TX_ADDR, RX_ADDR_P*
    radio_settings();

    // set CONFIG register according to rx/tx mode
    set_rx_tx_mode();

    se8r01_calibration();
    se8r01_setup();

    // set CONFIG register according to rx/tx mode
    set_rx_tx_mode();
}

void nrf_receive_handler()
{
    uint8_t status = SPI_Read(iRF_BANK0_STATUS);

    if(status & STA_MARK_RX)                                // if receive data ready (TX_DS) interrupt
    {
        SPI_Read_Buf(R_RX_PAYLOAD, rx_buf, TX_PLOAD_WIDTH);    // read playload to rx_buf
        nrf24_write_register(FLUSH_RX,0);
        // clear RX_FIFO. TODO: verify
        for(uint8_t i=0; i<TX_PLOAD_WIDTH; i++)
        {
        }
        nrf24_write_register(iRF_BANK0_STATUS,0xff);
    }
    else{
        nrf24_write_register(iRF_BANK0_STATUS,0xff);
    }
}

void RXX()
{
    uint8_t status = SPI_Read(iRF_BANK0_STATUS);
    if( HAL_GPIO_ReadPin(NRF24_IRQ_PORT,NRF24_IRQ_PIN) == LOW){
        int a = 0;

        delay_us(10);      //read reg too close after irq low not good

        //if(status & STA_MARK_RX || !(SPI_Read(R_REGISTER | FIFO_STATUS) ))                                // if receive data ready (TX_DS) interrupt
        if(nrf24_is_data_ready())                                // if receive data ready (TX_DS) interrupt
        {
            SPI_Read_Buf(R_RX_PAYLOAD, rx_buf, TX_PLOAD_WIDTH);    // read playload to rx_buf
            SPI_RW_Reg(FLUSH_RX,0);
            // clear RX_FIFO
            for(uint8_t i=0; i<TX_PLOAD_WIDTH; i++)
            {
            }
            SPI_RW_Reg(iRF_CMD_WRITE_REG+iRF_BANK0_STATUS,0xff);
            //TODO: this is for debug
            HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
        }
        else{

            SPI_RW_Reg(iRF_CMD_WRITE_REG+iRF_BANK0_STATUS,0xff);

        }
    }

}

void TXX()
{
    //power on
    //Bit 7    | Bit 6      | Bit 5      | Bit 4       | Bit 3  | Bit 2 | Bit 1  | Bit 0   |
    //Reserved | MASK_RX_DR | MASK_TX_DS | MASK_MAX_RT | EN_CRC | CRCO  | PWR_UP | PRIM_RX |
    //By setting one of the MASK bits high, the corresponding IRQ source is disabled. By default all IRQ sources are enabled.
    //nrf24_write_register(iRF_BANK0_CONFIG, (1 << MASK_RX_DR) | (0 << MASK_TX_DS) | (0 << MASK_MAX_RT) | (1 << EN_CRC) | (1 << CRCO)  | (1 << PWR_UP) | (0 << PRIM_RX) );
    //delay_us(200);

    //for(uint8_t i=0; i<TX_PLOAD_WIDTH; i++)
      //  tx_buf[i] = k++;
    const char* test_str = "HelloWireless!\0";
    memcpy(tx_buf, test_str, strlen(test_str));

    SPI_RW_Reg(iRF_CMD_FLUSH_TX,0);
    SPI_Write_Buf(iRF_CMD_WR_TX_PLOAD,tx_buf,TX_PLOAD_WIDTH);

    //start transmission by toggling SE high for more than 10 us
    nrf24_ce_set(HIGH);

//============== TODO: investigate
    delay_us(10);
    //HAL_Delay(2);
#if 1
    int Delay = 100;
    uint32_t tickstart = HAL_GetTick();

    uint8_t packet_not_sent = !SPI_Read(iRF_BANK0_STATUS) & (iSTATUS_TX_DS | iSTATUS_MAX_RT);
    uint8_t elapsed = HAL_GetTick() - tickstart;

    while( packet_not_sent && elapsed < Delay){
        packet_not_sent = ! SPI_Read(iRF_BANK0_STATUS) & (iSTATUS_TX_DS | iSTATUS_MAX_RT);
        elapsed = HAL_GetTick() - tickstart;
    }
#endif
    // stop transmission
    nrf24_ce_set(LOW);
    //==============

    TransmissionStatus tx_status = nrf24_last_messageStatus();
    GPIO_PinState irq = HAL_GPIO_ReadPin(NRF24_IRQ_PORT,NRF24_IRQ_PIN);
    uint8_t status = SPI_Read(iRF_BANK0_STATUS);

    //TODO: this is for debug
    if(tx_status == NRF24_TRANSMISSON_OK)
    {
        HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
    }
    else if(tx_status == NRF24_MESSAGE_LOST){
        //HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
        //HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13, GPIO_PIN_RESET);//nop
    }
    else{

    }

    SPI_RW_Reg(iRF_CMD_WRITE_REG + iRF_BANK0_STATUS,0xff);   // clear RX_DR or TX_DS or MAX_RT interrupt flag

}

static void radio_settings()
{

    // Enable Auto Acknowledgment
    nrf24_write_register(iRF_BANK0_EN_AA, (1<<ENAA_P0)|(0<<ENAA_P1)|(0<<ENAA_P2)|(0<<ENAA_P3)|(0<<ENAA_P4)|(0<<ENAA_P5));

    // Enable RX addresses (pipes)
    nrf24_write_register(iRF_BANK0_EN_RXADDR,(1<<ERX_P0)|(0<<ERX_P1)|(0<<ERX_P2)|(0<<ERX_P3)|(0<<ERX_P4)|(0<<ERX_P5));


    //4 byte adress, but use 5 byte address! TODO: research http://forum.easyelectronics.ru/viewtopic.php?f=9&t=21484
    // 11 5 bytes
    // 10 4 bytes
    // 01 Illegal
    // 00 Illegal
    uint8_t SETUP_AW_value = 0x02;
    if(TX_ADR_WIDTH == 5){
        SETUP_AW_value = 0x3;
    }
    //TODO
    nrf24_write_register(iRF_BANK0_SETUP_AW, 0x02);

    // Auto retransmit delay and count (ARD, ARC)
    //lowest 4 bits 0-15 rt transmisston higest 4 bits 256-4096us Auto Retransmit
    //nrf24_write_register(iRF_BANK0_SETUP_RETR, 0xB00001010);
    nrf24_write_register(iRF_BANK0_SETUP_RETR, 0x1a);// 500us + 86us, 10 retrans...?

    // Set RF channel
    nrf24_write_register(iRF_BANK0_RF_CH, RF_CHANNEL);

    // RF_SETUP
    ///RF_SETUP register
    //Bit 7     | Bit 6    | Bit 5    | Bit 4    | Bit 3     | Bit 2 Bit 1 Bit 0 |
    //CONT_WAVE | PA_PWR_3 | RF_DR_LO | Reserved | RF_DR_HIG | PA_PWR            |

    /// Power                           |  DataRate
    // PA_PWR[3:0]                      | [RF_DR_LOW, RF_DR_HIGH]:
    // 1111 Output +5 dbm  0b111 = 0x7  | 11 500Kbps
    // 1000 Output 0 dbm                | 10 reserved
    // 0100 Output -6 dbm               | 01 2Mbps
    // 0010 Output -12 dbm              | 00 1Mbps
    // 0001 Output -18 dbm              |

    nrf24_write_register(iRF_BANK0_RF_SETUP, (0 << CONT_WAVE) | (1 << PA_PWR_3) | (0 << RF_DR_LO) | (1 << RF_DR_HIG) | (0x7 << PA_PWR) );
    //original comment: 2mps 0x4f, which is 1001111 TODO: 2mps is (RF_DR_LO,RF_DR_HIG) = (0,1) according to datasheet, and 0x4f stands for 1mps wtf?
    //0x47 1000111
    //nrf24_write_register(iRF_BANK0_RF_SETUP, 0x4f);

    // TODO: if setup is 0 or ff then there was no response from module
    uint8_t rf_setup = 0;
    nrf24_read_register_buf(iRF_BANK0_RF_SETUP, &rf_setup, 1);
    if(rf_setup == 0 && setup == 0xff){
        int error = 1;
    }


#if 0
    //Dynamic length configurations:
    //pipe0 pipe1 enable dynamic payload length data
    nrf24_write_register(iRF_BANK0_DYNPD, 0x01);
    nrf24_write_register(DYNPD,(0<<DPL_P0)|(0<<DPL_P1)|(0<<DPL_P2)|(0<<DPL_P3)|(0<<DPL_P4)|(0<<DPL_P5));

    // enable dynamic paload lenght; enbale payload with ack enable w_tx_payload_noack
    //SPI_RW_Reg(iRF_CMD_WRITE_REG|iRF_BANK0_FEATURE, 0x07);
#endif

    //Set transmit address
    nrf24_write_register_buf(iRF_BANK0_TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);

    // Use the same address on the RX device as the TX device
    nrf24_write_register_buf(iRF_BANK0_RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);

    // Select same RX payload width as TX Payload width
    nrf24_write_register(iRF_BANK0_RX_PW_P0, TX_PLOAD_WIDTH);
    nrf24_write_register(iRF_BANK0_RX_PW_P1, 0x00);
    nrf24_write_register(iRF_BANK0_RX_PW_P2, 0x00);
    nrf24_write_register(iRF_BANK0_RX_PW_P3, 0x00);
    nrf24_write_register(iRF_BANK0_RX_PW_P4, 0x00);
    nrf24_write_register(iRF_BANK0_RX_PW_P5, 0x00);

}

static void set_rx_tx_mode()
{
    if (mode=='r') {
        //Bit 7    | Bit 6      | Bit 5      | Bit 4       | Bit 3  | Bit 2 | Bit 1  | Bit 0   |
        //Reserved | MASK_RX_DR | MASK_TX_DS | MASK_MAX_RT | EN_CRC | CRCO  | PWR_UP | PRIM_RX |
        // turn on irq for receiver; turn off irq for transmitter
        //By setting one of the MASK bits high, the corresponding IRQ source is disabled. By default all IRQ sources are enabled.
        //TODO refactoring//
        nrf24_write_register(iRF_BANK0_CONFIG, (0 << MASK_RX_DR) | (1 << MASK_TX_DS) | (1 << MASK_MAX_RT) | (1 << EN_CRC) | (0 << CRCO)  | (1 << PWR_UP) | (1 << PRIM_RX) );
        //SPI_RW_Reg(iRF_CMD_WRITE_REG|iRF_BANK0_CONFIG, 0x3f);
        // start listening
        delay_us(10);
        nrf24_ce_set(HIGH);
        delay_us(210);
    }
    else {
        //Bit 7    | Bit 6      | Bit 5      | Bit 4       | Bit 3  | Bit 2 | Bit 1  | Bit 0   |
        //Reserved | MASK_RX_DR | MASK_TX_DS | MASK_MAX_RT | EN_CRC | CRCO  | PWR_UP | PRIM_RX |
        //By setting one of the MASK bits high, the corresponding IRQ source is disabled. By default all IRQ sources are enabled.
        //TODO refactoring//
        nrf24_write_register(iRF_BANK0_CONFIG, (1 << MASK_RX_DR) | (0 << MASK_TX_DS) | (0 << MASK_MAX_RT) | (1 << EN_CRC) | (0 << CRCO)  | (1 << PWR_UP) | (0 << PRIM_RX) );
        //SPI_RW_Reg(iRF_CMD_WRITE_REG|iRF_BANK0_CONFIG, 0x3E);
        delay_us(10);
    }
}

static void init_io()
{
    nrf24_ce_set(LOW);
    nrf24_csn_set(HIGH);
}

/**************************************************
 * Function: TX_Mode();
 *
 * Description:
 * This function initializes one IRQqRF24L01 device to
 * TX mode, set TX address, set RX address for auto.ack,
 * fill TX payload, select RF channel, datarate & TX pwr.
 * PWR_UP is set, CRC(2 uint8_ts) is enabled, & PRIM:TX.
 *
 * ToDo: One high pulse(>10us) on CE will now send this
 * packet and expext an acknowledgment from the RX device.
 **************************************************/
static void se8r01_switch_bank(uint8_t bankindex)
{
    uint8_t temp0,temp1;
    temp1 = bankindex;
    temp0 = SPI_RW(iRF_BANK0_STATUS);
    if((temp0&0x80)!=temp1)
    {
        SPI_RW_Reg(iRF_CMD_ACTIVATE,0x53);
    }
}

static void se8r01_powerup()
{
    se8r01_switch_bank(iBANK0);

    ///CONFIG
    //Bit 7    | Bit 6      | Bit 5      | Bit 4       | Bit 3  | Bit 2 | Bit 1  | Bit 0   |
    //Reserved | MASK_RX_DR | MASK_TX_DS | MASK_MAX_RT | EN_CRC | CRCO  | PWR_UP | PRIM_RX |
    nrf24_write_register(iRF_BANK0_CONFIG, (0 << MASK_RX_DR) | (0 << MASK_TX_DS) | (0 << MASK_MAX_RT) | (1 << EN_CRC) | (0 << CRCO)  | (1 << PWR_UP) | (1 << PRIM_RX) );

    //Setup RF channel TODO: check necessity
    nrf24_write_register(iRF_BANK0_RF_CH, RF_CHANNEL);


    ///RF_SETUP
    //Bit 7     | Bit 6    | Bit 5    | Bit 4    | Bit 3     | Bit 2 Bit 1 Bit 0 |
    //CONT_WAVE | PA_PWR_3 | RF_DR_LO | Reserved | RF_DR_HIG | PA_PWR            |
    /// Power                           |  DataRate
    // PA_PWR[3:0]                      | [RF_DR_LOW, RF_DR_HIGH]:
    // 1111 Output +5 dbm  0b111 = 0x7  | 11 500Kbps
    // 1000 Output 0 dbm                | 10 reserved
    // 0100 Output -6 dbm               | 01 2Mbps
    // 0010 Output -12 dbm              | 00 1Mbps
    // 0001 Output -18 dbm              |

    nrf24_write_register(iRF_BANK0_RF_SETUP, (0 << CONT_WAVE) | (1 << PA_PWR_3) | (0 << RF_DR_LO) | (1 << RF_DR_HIG) | (0x7 << PA_PWR) );

    // TODO: reveal the Magic
    nrf24_write_register(iRF_BANK0_PRE_GURD,0x77); //2450 calibration
}

static void se8r01_calibration()
{
    //iBANK1
    se8r01_switch_bank(iBANK1);

    //iRF_BANK1_PLL_CTL0 <= [0]=0x40 [1]=0x00 [2]=0x10 [3]=0xE6
    gtemp[0]=0x40;
    gtemp[1]=0x00;
    gtemp[2]=0x10;
    gtemp[3]=0xE6;
    nrf24_write_register_buf(iRF_BANK1_PLL_CTL0, gtemp, 4);

    //iRF_BANK1_CAL_CTL <= [0]=0x20 [1]=0x08 [2]=0x50 [3]=0x40 [4]=0x50
    gtemp[0]=0x20;
    gtemp[1]=0x08;
    gtemp[2]=0x50;
    gtemp[3]=0x40;
    gtemp[4]=0x50;
    nrf24_write_register_buf(iRF_BANK1_CAL_CTL, gtemp, 5);

    //iRF_BANK1_IF_FREQ <= [0]=0x00 [1]=0x00 [2]=0x1E
    gtemp[0]=0x00;
    gtemp[1]=0x00;
    gtemp[2]=0x1E;
    nrf24_write_register_buf(iRF_BANK1_IF_FREQ, gtemp, 3);

    //iRF_BANK1_FDEV <= [0]=0x29
    gtemp[0]=0x29;
    nrf24_write_register_buf(iRF_BANK1_FDEV, gtemp, 1);

    //iRF_BANK1_DAC_CAL_LOW <= [0]=0x00
    gtemp[0]=0x00;
    nrf24_write_register_buf(iRF_BANK1_DAC_CAL_LOW, gtemp, 1);

    //iRF_BANK1_DAC_CAL_HI <= [0]=0x7F
    gtemp[0]=0x7F;
    nrf24_write_register_buf(iRF_BANK1_DAC_CAL_HI, gtemp, 1);

    //iRF_BANK1_AGC_GAIN <= [0]=0x02 [1]=0xC1 [2]=0xEB [3]=0x1C
    gtemp[0]=0x02;
    gtemp[1]=0xC1;
    gtemp[2]=0xEB;
    gtemp[3]=0x1C;
    nrf24_write_register_buf(iRF_BANK1_AGC_GAIN, gtemp, 4);

    //iRF_BANK1_RF_IVGEN <= [0]=0x97 [1]=0x64 [2]=0x00 [3]=0x81
    gtemp[0]=0x97;
    gtemp[1]=0x64;
    gtemp[2]=0x00;
    gtemp[3]=0x81;
    nrf24_write_register_buf(iRF_BANK1_RF_IVGEN, gtemp, 4);

//    iBANK0
//    CE 1
//    Delay 30 us
//    CE 0
//    Delay 15 ms
//    CE 1
//    Delay 30 us
//    CE 0
//    Delay 15 ms
    se8r01_switch_bank(iBANK0);
    nrf24_ce_set(HIGH);
    delay_us(30);
    nrf24_ce_set(LOW);
    HAL_Delay(15);
    nrf24_ce_set(HIGH);
    delay_us(30);
    nrf24_ce_set(LOW);
    HAL_Delay(15);

}

static void se8r01_setup()
{

    //    iRF_BANK0_SETUP_VALUE <= [0]=0x28 [1]=0x32 [2]=0x80 [3]=0x90 [4]=0x00
    gtemp[0]=0x28;
    gtemp[1]=0x32;
    gtemp[2]=0x80;
    gtemp[3]=0x90;
    gtemp[4]=0x00;
    nrf24_write_register_buf(iRF_BANK0_SETUP_VALUE, gtemp, 5);

    //    Delay 2 us
    delay_us(2);

    //    iBANK1
    se8r01_switch_bank(iBANK1);

    //    iRF_BANK1_PLL_CTL0 <= [0]=0x40 [1]=0x01 [2]=0x30 [3]=0xE2
    gtemp[0]=0x40;
    gtemp[1]=0x01;
    gtemp[2]=0x30;
    gtemp[3]=0xE2;
    nrf24_write_register_buf(iRF_BANK1_PLL_CTL0, gtemp, 4);

    //    iRF_BANK1_CAL_CTL <= [0]=0x29 [1]=0x89 [2]=0x55 [3]=0x40 [4]=0x50
    gtemp[0]=0x29;
    gtemp[1]=0x89;
    gtemp[2]=0x55;
    gtemp[3]=0x40;
    gtemp[4]=0x50;
    nrf24_write_register_buf(iRF_BANK1_CAL_CTL, gtemp, 5);

    //    iRF_BANK1_FDEV <= [0]=0x29
    gtemp[0]=0x29;
    nrf24_write_register_buf(iRF_BANK1_FDEV, gtemp, 1);

    //    iRF_BANK1_RX_CTRL <= [0]=0x55 [1]=0xC2 [2]=0x09 [3]=0xAC
    gtemp[0]=0x55;
    gtemp[1]=0xC2;
    gtemp[2]=0x09;
    gtemp[3]=0xAC;
    nrf24_write_register_buf(iRF_BANK1_RX_CTRL, gtemp, 4);

    //    iRF_BANK1_FAGC_CTRL_1 <= [0]=0x00 [1]=0x14 [2]=0x08 [3]=0x29
    gtemp[0]=0x00;
    gtemp[1]=0x14;
    gtemp[2]=0x08;
    gtemp[3]=0x29;
    nrf24_write_register_buf(iRF_BANK1_FAGC_CTRL_1, gtemp, 4);

    //    iRF_BANK1_AGC_GAIN <= [0]=0x02 [1]=0xC1 [2]=0xCB [3]=0x1C
    gtemp[0]=0x02;
    gtemp[1]=0xC1;
    gtemp[2]=0xCB;
    gtemp[3]=0x1C;
    nrf24_write_register_buf(iRF_BANK1_AGC_GAIN, gtemp, 4);

    //    iRF_BANK1_RF_IVGEN <= [0]=0x97 [1]=0x64 [2]=0x00 [3]=0x01
    gtemp[0]=0x97;
    gtemp[1]=0x64;
    gtemp[2]=0x00;
    gtemp[3]=0x01;
    nrf24_write_register_buf(iRF_BANK1_RF_IVGEN, gtemp, 4);

    //    iRF_BANK1_TEST_PKDET <= [0]=0x2A [1]=0x04 [2]=0x00 [3]=0x7D
    gtemp[0]=0x2A;
    gtemp[1]=0x04;
    gtemp[2]=0x00;
    gtemp[3]=0x7D;
    nrf24_write_register_buf(iRF_BANK1_TEST_PKDET, gtemp, 4);

    //    iBANK0
    se8r01_switch_bank(iBANK0);

    //    Delay 2 us
    delay_us(2);
}

static void power_off()
{
    nrf24_csn_set(LOW);
    SPI_RW_Reg(W_REGISTER + CONFIG, 0x0D);//0x0D=1101
    nrf24_csn_set(HIGH);
    delay_us(20);
}

static void power_on_tx()
{
    uint8_t config = 0;
    uint8_t power_on_tx_mask = 0xff & (0 << PWR_UP) | (0 << PRIM_RX);
    nrf24_ce_set(LOW);
    //CONFIG
    //Bit 7    | Bit 6      | Bit 5      | Bit 4       | Bit 3  | Bit 2 | Bit 1  | Bit 0   |
    //Reserved | MASK_RX_DR | MASK_TX_DS | MASK_MAX_RT | EN_CRC | CRCO  | PWR_UP | PRIM_RX |
    nrf24_read_register_buf(iRF_BANK0_CONFIG, &config, 1);
    nrf24_write_register(iRF_BANK0_CONFIG, config | power_on_tx_mask);
    HAL_Delay(5);
    //delay_us(150);
}

static void power_on_rx()
{
    nrf24_ce_set(LOW);
    //CONFIG
    //Bit 7    | Bit 6      | Bit 5      | Bit 4       | Bit 3  | Bit 2 | Bit 1  | Bit 0   |
    //Reserved | MASK_RX_DR | MASK_TX_DS | MASK_MAX_RT | EN_CRC | CRCO  | PWR_UP | PRIM_RX |
    nrf24_write_register(iRF_BANK0_CONFIG, (0 << MASK_RX_DR) | (1 << MASK_TX_DS) | (1 << MASK_MAX_RT) | (1 << EN_CRC) | (0 << CRCO)  | (1 << PWR_UP) | (1 << PRIM_RX) );
    delay_us(2);
    nrf24_ce_set(HIGH);
    delay_us(150);
}

// Clocks only one byte into the given nrf24 register //
static void nrf24_write_register(uint8_t reg, uint8_t value)
{
    delay_us(10);
    nrf24_csn_set(LOW);
    SPI_RW(iRF_CMD_WRITE_REG | (REGISTER_MASK & reg));
    SPI_RW(value);
    nrf24_csn_set(HIGH);
}

// Read single register from nrf24 //
static void nrf24_read_register_buf(uint8_t reg, uint8_t* value, uint8_t len)
{
    nrf24_csn_set(LOW);
    SPI_Read_Buf(iRF_CMD_READ_REG | (REGISTER_MASK & reg), value, len);
    nrf24_csn_set(HIGH);
}

// Write to a single register of nrf24 //
static void nrf24_write_register_buf(uint8_t reg, uint8_t* value, uint8_t len)
{
    nrf24_csn_set(LOW);
    SPI_Write_Buf(iRF_CMD_WRITE_REG | (REGISTER_MASK & reg), value, len);
    nrf24_csn_set(HIGH);

}


/**************************************************
 * Function: SPI_RW();
 *
 * Description:
 * Writes one uint8_t to nRF24L01, and return the uint8_t read
 * from nRF24L01 during write, according to SPI protocol
 **************************************************/
static uint8_t SPI_RW(uint8_t tx)
{
    uint8_t rx = 0;
    HAL_SPI_TransmitReceive_IT(&hspi1, &tx, &rx, 1);
    return rx;
}
/**************************************************/

/**************************************************
 * Function: SPI_RW_Reg();
 *
 * Description:
 * Writes value 'value' to register 'reg'
/**************************************************/
static uint8_t SPI_RW_Reg(uint8_t reg, uint8_t value)
{
    uint8_t status;

    nrf24_csn_set(LOW);                   // CSN low, init SPI transaction
    status = SPI_RW(reg);                   // select register
    SPI_RW(value);                          // ..and write value to it..
    nrf24_csn_set(HIGH);                   // CSN high again

    return(status);                   // return nRF24L01 status uint8_t
}
/**************************************************/

/**************************************************
 * Function: SPI_Read();
 *
 * Description:
 * Read one uint8_t from nRF24L01 register, 'reg'
/**************************************************/
uint8_t SPI_Read(uint8_t reg)
{
    uint8_t reg_val;

    nrf24_csn_set(LOW);           // CSN low, initialize SPI communication...
    SPI_RW(reg);                   // Select register to read from..
    reg_val = SPI_RW(0);           // ..then read register value
    nrf24_csn_set(HIGH);          // CSN high, terminate SPI communication

    return(reg_val);               // return register value
}
/**************************************************/

/**************************************************
 * Function: SPI_Read_Buf();
 *
 * Description:
 * Reads 'uint8_ts' #of uint8_ts from register 'reg'
 * Typically used to read RX payload, Rx/Tx address
/**************************************************/
static uint8_t SPI_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t bytes)
{
    uint8_t status,i;

    nrf24_csn_set(LOW);                  // Set CSN low, init SPI tranaction
    status = SPI_RW(reg);       	    // Select register to write to and read status uint8_t

    for(i=0;i<bytes;i++)
    {
        pBuf[i] = SPI_RW(0);    // Perform SPI_RW to read uint8_t from nRF24L01
    }

    nrf24_csn_set(HIGH);                   // Set CSN high again

    return(status);                  // return nRF24L01 status uint8_t
}
/**************************************************/

/**************************************************
 * Function: SPI_Write_Buf();
 *
 * Description:
 * Writes contents of buffer '*pBuf' to nRF24L01
 * Typically used to write TX payload, Rx/Tx address
/**************************************************/
static uint8_t SPI_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t bytes)
{
    uint8_t status,i;

    nrf24_csn_set(LOW);                  // Set CSN low, init SPI tranaction
    status = SPI_RW(reg);             // Select register to write to and read status uint8_t
    for(i=0;i<bytes; i++)             // then write all uint8_t in buffer(*pBuf)
    {
        SPI_RW(*pBuf++);
    }
    nrf24_csn_set(HIGH);                   // Set CSN high again
    return(status);                  // return nRF24L01 status uint8_t
}
/**************************************************/




