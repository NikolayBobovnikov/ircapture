//this is a copy and paste job made by F2k

#include "stm32f1xx_hal.h"
#include "se8r01.h"

///
/// on initialization se8r01 check http://forum.easyelectronics.ru/viewtopic.php?f=9&t=21484
///

extern SPI_HandleTypeDef hspi1;
extern char mode;      //r=rx, t=tx
void delay_us(uint8_t us);
void nrf24_setup_gpio();
uint8_t gtemp[5];

// Define a static TX address
extern uint8_t TX_ADDRESS[TX_ADR_WIDTH];
extern uint8_t rx_buf[TX_PLOAD_WIDTH]; // initialize value
extern uint8_t tx_buf[TX_PLOAD_WIDTH];
uint8_t payload_len = TX_PLOAD_WIDTH;


void nrf24_init()
{
    nrf24_setup_gpio();
    nrf24_ce_set(LOW);
    nrf24_csn_set(HIGH);
}

void nrf24_config(uint8_t channel)
{
    // Set RF channel
    nrf24_write_register(RF_CH,channel);

    // Set length of incoming payload
    nrf24_write_register(RX_PW_P0, payload_len); // Auto-ACK pipe ...
    nrf24_write_register(RX_PW_P1, 0x00); // Data payload pipe
    nrf24_write_register(RX_PW_P2, 0x00); // Pipe not used
    nrf24_write_register(RX_PW_P3, 0x00); // Pipe not used
    nrf24_write_register(RX_PW_P4, 0x00); // Pipe not used
    nrf24_write_register(RX_PW_P5, 0x00); // Pipe not used

    // 1 Mbps, TX gain: 0dbm
    nrf24_write_register(RF_SETUP, (0<<RF_DR_HIGH)|((0x03)<<RF_PWR));

    // CRC, number of bytes CRC length
    nrf24_write_register(CONFIG,((1<<EN_CRC)|(0<<CRCO)));

    // Auto Acknowledgment
    nrf24_write_register(EN_AA,(0<<ENAA_P0)|(0<<ENAA_P1)|(0<<ENAA_P2)|(0<<ENAA_P3)|(0<<ENAA_P4)|(0<<ENAA_P5));

    // Enable RX addresses
    nrf24_write_register(EN_RXADDR,(1<<ERX_P0)|(0<<ERX_P1)|(0<<ERX_P2)|(0<<ERX_P3)|(0<<ERX_P4)|(0<<ERX_P5));

    // Auto retransmit delay: 1000 us and Up to 15 retransmit trials
    nrf24_write_register(SETUP_RETR,(0x04<<ARD)|(0x0F<<ARC));

    // Dynamic length configurations: No dynamic length
    nrf24_write_register(DYNPD,(0<<DPL_P0)|(0<<DPL_P1)|(0<<DPL_P2)|(0<<DPL_P3)|(0<<DPL_P4)|(0<<DPL_P5));

    // Start listening
    nrf24_powerUpRx();
}

void nrf24_config_rx(uint8_t *pipe_addr, uint8_t channel)
{
    // setup addresses for pipe
    nrf24_write_register_buf(RX_ADDR_P1, pipe_addr, TX_ADR_WIDTH);

    // Set RF channel
    nrf24_write_register(RF_CH,channel);

    // Set length of incoming payload
    nrf24_write_register(RX_PW_P0, 0x00); // Auto-ACK pipe ...
    nrf24_write_register(RX_PW_P1, payload_len); // Data payload pipe
    nrf24_write_register(RX_PW_P2, 0x00); // Pipe not used
    nrf24_write_register(RX_PW_P3, 0x00); // Pipe not used
    nrf24_write_register(RX_PW_P4, 0x00); // Pipe not used
    nrf24_write_register(RX_PW_P5, 0x00); // Pipe not used

    // 1 Mbps, TX gain: 0dbm
    nrf24_write_register(RF_SETUP, (0<<RF_DR_HIGH)|((0x03)<<RF_PWR));

    //                          (CRC enabled,  1 byte)
    nrf24_write_register(CONFIG,(1<<EN_CRC) | (0<<CRCO));

    // Auto Acknowledgment
    nrf24_write_register(EN_AA,(0<<ENAA_P0)|(0<<ENAA_P1)|(0<<ENAA_P2)|(0<<ENAA_P3)|(0<<ENAA_P4)|(0<<ENAA_P5));

    // Enable RX addresses
    nrf24_write_register(EN_RXADDR,(0<<ERX_P0)|(1<<ERX_P1)|(0<<ERX_P2)|(0<<ERX_P3)|(0<<ERX_P4)|(0<<ERX_P5));

    // Auto retransmit delay: 1000 us and Up to 15 retransmit trials
    nrf24_write_register(SETUP_RETR,(0x04<<ARD)|(0x0F<<ARC));

    // Dynamic length configurations: No dynamic length
    nrf24_write_register(DYNPD,(0<<DPL_P0)|(0<<DPL_P1)|(0<<DPL_P2)|(0<<DPL_P3)|(0<<DPL_P4)|(0<<DPL_P5));

    // Start listening
    nrf24_powerUpRx();
}

void nrf24_config_tx(uint8_t *pipe_addr, uint8_t channel)
{
    // setup addresses for pipes
    //nrf24_write_register_multi(RX_ADDR_P0, pipe_addr, nrf24_ADDR_LEN);
    nrf24_write_register_buf(TX_ADDR, pipe_addr, TX_ADR_WIDTH);

    // Set RF channel
    nrf24_write_register(RF_CH,channel);

    // Set length of incoming payload
    nrf24_write_register(RX_PW_P0, 0x00); // Auto-ACK pipe ...
    nrf24_write_register(RX_PW_P1, 0x00); // Data payload pipe
    nrf24_write_register(RX_PW_P2, 0x00); // Pipe not used
    nrf24_write_register(RX_PW_P3, 0x00); // Pipe not used
    nrf24_write_register(RX_PW_P4, 0x00); // Pipe not used
    nrf24_write_register(RX_PW_P5, 0x00); // Pipe not used

    // 1 Mbps, TX gain: 0dbm
    nrf24_write_register(RF_SETUP, (0<<RF_DR_HIGH)|((0x03)<<RF_PWR));

    //                          (CRC enabled,  1 byte)
    nrf24_write_register(CONFIG,(1<<EN_CRC) | (0<<CRCO));

    // Auto Acknowledgment
    nrf24_write_register(EN_AA,(0<<ENAA_P0)|(0<<ENAA_P1)|(0<<ENAA_P2)|(0<<ENAA_P3)|(0<<ENAA_P4)|(0<<ENAA_P5));

    // Enable RX addresses
    nrf24_write_register(EN_RXADDR,(0<<ERX_P0)|(0<<ERX_P1)|(0<<ERX_P2)|(0<<ERX_P3)|(0<<ERX_P4)|(0<<ERX_P5));

    // Auto retransmit delay: 1000 us and Up to 15 retransmit trials
    nrf24_write_register(SETUP_RETR,(0x04<<ARD)|(0x0F<<ARC));

    // Dynamic length configurations: No dynamic length
    nrf24_write_register(DYNPD,(0<<DPL_P0)|(0<<DPL_P1)|(0<<DPL_P2)|(0<<DPL_P3)|(0<<DPL_P4)|(0<<DPL_P5));

    // Start listening
    nrf24_powerUpTx();
}

void nrf24_set_rx_address(uint8_t * adr)
{
    //nrf24_ce_set(LOW);
    nrf24_write_register_buf(RX_ADDR_P0,adr,TX_ADR_WIDTH);
    //nrf24_ce_set(HIGH);
}

void nrf24_set_tx_address(uint8_t* adr)
{
    // RX_ADDR_P0 must be set to the sending addr for auto ack to work. //
   nrf24_write_register_buf(RX_ADDR_P0, adr, TX_ADR_WIDTH);
   nrf24_write_register_buf(TX_ADDR, adr, TX_ADR_WIDTH);
}

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
    uint8_t status = nrf24_get_status_register();

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

// Reads payload bytes into data array //
void nrf24_receive(uint8_t* data)
{
    // Pull down chip select //
    nrf24_csn_set(LOW);

    // Send cmd to read rx payload //
    SPI_RW( R_RX_PAYLOAD );

    // Read payload //
    nrf24_transferSync(data,data,payload_len);

    // Pull up chip select //
    nrf24_csn_set(HIGH);

    // Reset status register //
    nrf24_write_register(STATUS,(1<<RX_DR));
}

// Returns the number of retransmissions occured for the last message //
uint8_t nrf24_get_last_msg_retransmission_count()
{
    uint8_t rv;
    nrf24_read_register_buf(OBSERVE_TX,&rv,1);
    rv = rv & 0x0F;
    return rv;
}

// Sends a data package to the default address. Be sure to send the correct
// amount of bytes as configured as payload on the receiver.
void nrf24_send(uint8_t* value)
{
    // Go to Standby-I first //
    nrf24_ce_set(LOW);

    // Set to transmitter mode , Power up if needed //
    nrf24_powerUpTx();

    // Do we really need to flush TX fifo each time ? // TODO
#if 1
    // Pull down chip select //
    nrf24_csn_set(LOW);

    // Write cmd to flush transmit FIFO //
    SPI_RW(FLUSH_TX);

    // Pull up chip select //
    nrf24_csn_set(HIGH);
#endif

    // Pull down chip select //
    nrf24_csn_set(LOW);

    // Write cmd to write payload //
    SPI_RW(W_TX_PAYLOAD);

    // Write payload //
    // TODO func(value,payload_len);

    // Pull up chip select //
    nrf24_csn_set(HIGH);

    // Start the transmission //
    nrf24_ce_set(HIGH);

    HAL_Delay(1); //should be >10 microseconds
    nrf24_ce_set(LOW);
}

bool nrf24_is_sending()
{
    uint8_t status;

    // read the current status //
    status = nrf24_get_status_register();

    // if sending successful (TX_DS) or max retries exceded (MAX_RT). //
    if((status & ((1 << TX_DS)  | (1 << MAX_RT))))
    {
        return false;
    }

    return true;

}

uint8_t nrf24_get_status_register()
{
    uint8_t rv;
    nrf24_csn_set(LOW);
    rv = SPI_RW(NOP);
    nrf24_csn_set(HIGH);
    return rv;
}

TransmissionStatus nrf24_last_messageStatus()
{
    uint8_t rv;

    rv = nrf24_get_status_register();

    // Transmission went OK //
    if(is_register_bit_set(STATUS, TX_DS))
    {
        nrf24_reset_register_bit(STATUS, TX_DS);
        return NRF24_TRANSMISSON_OK;
    }
    // Maximum retransmission count is reached //
    // Last message probably went missing ... //
    else if(is_register_bit_set(STATUS, MAX_RT))
    {
        nrf24_reset_register_bit(STATUS, MAX_RT);
        return NRF24_MESSAGE_LOST;
    }
    // Probably still sending ... //
    else
    {
        return NRF24_MESSAGE_SENDING;
    }
}

void nrf24_powerUpRx()
{
    nrf24_csn_set(LOW);
    SPI_RW(FLUSH_RX);
    nrf24_csn_set(HIGH);

    nrf24_write_register(STATUS,(1<<RX_DR)|(1<<TX_DS)|(1<<MAX_RT));

    nrf24_ce_set(LOW);
    nrf24_write_register(CONFIG,(1<<PWR_UP)|(1<<PRIM_RX));

    // start listening
    nrf24_ce_set(HIGH);

}

void nrf24_powerUpTx()
{
    nrf24_write_register(STATUS,(1<<RX_DR)|(1<<TX_DS)|(1<<MAX_RT));
    nrf24_write_register(CONFIG,(1<<PWR_UP)|(0<<PRIM_RX));
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

void nrf24_ce_set(uint8_t state)
{
    assert_param(state == LOW || state == HIGH);
    HAL_GPIO_WritePin(NRF24_CE_PORT,NRF24_CE_PIN, state);
}

void nrf24_csn_set(uint8_t state)
{
    assert_param(state == LOW || state == HIGH);
    HAL_GPIO_WritePin(NRF24_CSN_PORT,NRF24_CSN_PIN, state);
}


// send and receive multiple bytes over SPI //
void nrf24_transferSync(uint8_t* dataout,uint8_t* datain,uint8_t len)
{
    uint8_t i;

    for(i=0;i<len;i++)
    {
        datain[i] = SPI_RW(dataout[i]);
    }

}

// send multiple bytes over SPI //
void nrf24_transmitSync(uint8_t* dataout,uint8_t len)
{
    uint8_t i;
    for(i=0;i<len;i++)
    {
        SPI_RW(dataout[i]);
    }

}

void nrf24_reset_register_bit(uint8_t reg_name, uint8_t bit)
{
    uint8_t reg = 0;
    nrf24_read_register_buf(reg_name, &reg, 1);
    nrf24_write_register(reg_name, reg | (1 << bit));
}

bool is_register_bit_set(uint8_t reg_name, uint8_t bit)
{
    uint8_t reg = 0;
    nrf24_read_register_buf(reg_name, &reg, 1);
    if(reg & (1 << bit)){
        return true;
    }
    return false;
}







void setup()
{
    init_io();                        // Initialize IO port
    uint8_t status = SPI_Read(STATUS);
    nrf24_ce_set(LOW);
    delay_us(150);
    se8r01_powerup();
    se8r01_calibration();
    se8r01_setup();
    radio_settings();

    if (mode=='r') {
        SPI_RW_Reg(iRF_CMD_WRITE_REG|iRF_BANK0_CONFIG, 0x3f);
        // start listening
        nrf24_ce_set(HIGH);
    }
    else {
        SPI_RW_Reg(iRF_CMD_WRITE_REG|iRF_BANK0_CONFIG, 0x3E);
        nrf24_ce_set(LOW);
    }


}

void loop()
{
    if (mode=='r')
    {
        RXX();
    }
    else
    {
        TXX();
    }
}

void RXX()
{
    if( HAL_GPIO_ReadPin(NRF24_IRQ_PORT,NRF24_IRQ_PIN) == LOW)
    {
        delay_us(10);      //read reg too close after irq low not good
        uint8_t status = SPI_Read(iRF_BANK0_STATUS);

        if(status & STA_MARK_RX)                                // if receive data ready (TX_DS) interrupt
        {
            SPI_Read_Buf(R_RX_PAYLOAD, rx_buf, TX_PLOAD_WIDTH);    // read playload to rx_buf
            SPI_RW_Reg(FLUSH_RX,0);
            // clear RX_FIFO
            for(uint8_t i=0; i<TX_PLOAD_WIDTH; i++)
            {
            }
            SPI_RW_Reg(iRF_CMD_WRITE_REG+iRF_BANK0_STATUS,0xff);
        }
        else{

            SPI_RW_Reg(iRF_CMD_WRITE_REG+iRF_BANK0_STATUS,0xff);

        }

    }
    HAL_Delay(1);

}

void TXX()
{

    //for(uint8_t i=0; i<TX_PLOAD_WIDTH; i++)
      //  tx_buf[i] = k++;
    const char* test_str = "HelloWireless!\0";
    memcpy(tx_buf, test_str, strlen(test_str));


    uint8_t status = SPI_Read(iRF_BANK0_STATUS);

    SPI_RW_Reg(iRF_CMD_FLUSH_TX,0);
    SPI_Write_Buf(iRF_CMD_WR_TX_PLOAD,tx_buf,TX_PLOAD_WIDTH);

    uint8_t tx_status = nrf24_last_messageStatus();
                GPIO_PinState irq = HAL_GPIO_ReadPin(NRF24_IRQ_PORT,NRF24_IRQ_PIN);
                uint8_t retr = nrf24_get_last_msg_retransmission_count();
                switch(tx_status){
                    case NRF24_TRANSMISSON_OK:{
                        int a = 0;
                        break;
                    }
                    case NRF24_MESSAGE_LOST:{
                        int a = 0;
                        break;
                    }
                    case NRF24_MESSAGE_SENDING:{
                        int a = 0;
                        break;
                    }
                }

    SPI_RW_Reg(iRF_CMD_WRITE_REG + iRF_BANK0_STATUS,0xff);   // clear RX_DR or TX_DS or MAX_RT interrupt flag

    HAL_Delay(500);

}

void radio_settings()
{

    // Enable Auto Acknowledgment
    nrf24_write_register(iRF_BANK0_EN_AA, (1<<ENAA_P0)|(0<<ENAA_P1)|(0<<ENAA_P2)|(0<<ENAA_P3)|(0<<ENAA_P4)|(0<<ENAA_P5));

    // Enable RX addresses (pipes)
    //nrf24_write_register(iRF_BANK0_EN_RXADDR, 0x01);      //enable pip 1
    nrf24_write_register(iRF_BANK0_EN_RXADDR,(1<<ERX_P0)|(0<<ERX_P1)|(0<<ERX_P2)|(0<<ERX_P3)|(0<<ERX_P4)|(0<<ERX_P5));


    //4 byte adress, but use 5 byte address! TODO: research http://forum.easyelectronics.ru/viewtopic.php?f=9&t=21484
    // 11 5 bytes
    // 10 4 bytes
    // 01 Illegal
    // 00 Illegal
    nrf24_write_register(iRF_BANK0_SETUP_AW, 0x02);

    // Auto retransmit delay and count (ARD, ARC)
    //lowest 4 bits 0-15 rt transmisston higest 4 bits 256-4096us Auto Retransmit
    nrf24_write_register(iRF_BANK0_SETUP_RETR, 0xB00001010);

    // Set RF channel
    nrf24_write_register(iRF_BANK0_RF_CH, 40);

    //original comment: 2mps 0x4f, which is 1001111 TODO: 2mps is (RF_DR_LO,RF_DR_HIG) = (0,1) according to datasheet, and 0x4f stands for 1mps wtf?
    //RF_SETUP register
    //Bit 7     | Bit 6    | Bit 5    | Bit 4    | Bit 3     | Bit 2 Bit 1 Bit 0 |
    //CONT_WAVE | PA_PWR_3 | RF_DR_LO | Reserved | RF_DR_HIG | PA_PWR            |
    nrf24_write_register(iRF_BANK0_RF_SETUP, (0 << CONT_WAVE) | (1 << PA_PWR_3) | (0 << RF_DR_LO) | (1 << RF_DR_HIG) | (1 << CRCO)  | (0b111 << PA_PWR) );


    //Dynamic length configurations:
    //pipe0 pipe1 enable dynamic payload length data
    //SPI_RW_Reg(iRF_CMD_WRITE_REG|iRF_BANK0_DYNPD, 0x01);
    //nrf24_write_register(DYNPD,(0<<DPL_P0)|(0<<DPL_P1)|(0<<DPL_P2)|(0<<DPL_P3)|(0<<DPL_P4)|(0<<DPL_P5));

    // enable dynamic paload lenght; enbale payload with ack enable w_tx_payload_noack
    //SPI_RW_Reg(iRF_CMD_WRITE_REG|iRF_BANK0_FEATURE, 0x07);

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


//    // Start listening
//    nrf24_powerUpRx();

}

void init_io(void)
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
void se8r01_switch_bank(uint8_t bankindex)
{
    uint8_t temp0,temp1;
    temp1 = bankindex;
    temp0 = SPI_RW(iRF_BANK0_STATUS);
    if((temp0&0x80)!=temp1)
    {
        SPI_RW_Reg(iRF_CMD_ACTIVATE,0x53);
    }
}

void se8r01_powerup()
{
    se8r01_switch_bank(iBANK0);

    //CONFIG
    //Bit 7    | Bit 6      | Bit 5      | Bit 4       | Bit 3  | Bit 2 | Bit 1  | Bit 0   |
    //Reserved | MASK_RX_DR | MASK_TX_DS | MASK_MAX_RT | EN_CRC | CRCO  | PWR_UP | PRIM_RX |
    nrf24_write_register(iRF_BANK0_CONFIG, (0 << MASK_RX_DR) | (0 << MASK_TX_DS) | (0 << MASK_MAX_RT) | (1 << EN_CRC) | (1 << CRCO)  | (1 << PWR_UP) | (1 << PRIM_RX) );

    //Setup RF channel TODO: check necessity
    nrf24_write_register(iRF_BANK0_RF_CH, RF_CHANNEL);

    //RF_SETUP register
    //Bit 7     | Bit 6    | Bit 5    | Bit 4    | Bit 3     | Bit 2 Bit 1 Bit 0 |
    //CONT_WAVE | PA_PWR_3 | RF_DR_LO | Reserved | RF_DR_HIG | PA_PWR            |
    nrf24_write_register(iRF_BANK0_RF_SETUP, (0 << CONT_WAVE) | (1 << PA_PWR_3) | (0 << RF_DR_LO) | (1 << RF_DR_HIG) | (1 << CRCO)  | (0b111 << PA_PWR) );

    // TODO: reveal the Magic
    SPI_RW_Reg(iRF_CMD_WRITE_REG|iRF_BANK0_PRE_GURD,0x77); //2450 calibration
}

void se8r01_calibration()
{
    //iBANK1
    se8r01_switch_bank(iBANK1);

    //iRF_BANK1_PLL_CTL0 <= [0]=0x40 [1]=0x00 [2]=0x10 [3]=0xE6
    gtemp[0]=0x40;
    gtemp[1]=0x00;
    gtemp[2]=0x10;
    gtemp[3]=0xE6;
    SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK1_PLL_CTL0, gtemp, 4);

    //iRF_BANK1_CAL_CTL <= [0]=0x20 [1]=0x08 [2]=0x50 [3]=0x40 [4]=0x50
    gtemp[0]=0x20;
    gtemp[1]=0x08;
    gtemp[2]=0x50;
    gtemp[3]=0x40;
    gtemp[4]=0x50;
    SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK1_CAL_CTL, gtemp, 5);

    //iRF_BANK1_IF_FREQ <= [0]=0x00 [1]=0x00 [2]=0x1E
    gtemp[0]=0x00;
    gtemp[1]=0x00;
    gtemp[2]=0x1E;
    SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK1_IF_FREQ, gtemp, 3);

    //iRF_BANK1_FDEV <= [0]=0x29
    gtemp[0]=0x29;
    SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK1_FDEV, gtemp, 1);

    //iRF_BANK1_DAC_CAL_LOW <= [0]=0x00
    gtemp[0]=0x00;
    SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK1_DAC_CAL_LOW, gtemp, 1);

    //iRF_BANK1_DAC_CAL_HI <= [0]=0x7F
    gtemp[0]=0x7F;
    SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK1_DAC_CAL_HI, gtemp, 1);

    //iRF_BANK1_AGC_GAIN <= [0]=0x02 [1]=0xC1 [2]=0xEB [3]=0x1C
    gtemp[0]=0x02;
    gtemp[1]=0xC1;
    gtemp[2]=0xEB;
    gtemp[3]=0x1C;
    SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK1_AGC_GAIN, gtemp, 4);

    //iRF_BANK1_RF_IVGEN <= [0]=0x97 [1]=0x64 [2]=0x00 [3]=0x81
    gtemp[0]=0x97;
    gtemp[1]=0x64;
    gtemp[2]=0x00;
    gtemp[3]=0x81;
    SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK1_RF_IVGEN, gtemp, 4);

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
    delay_us(15);
    nrf24_ce_set(HIGH);
    delay_us(30);
    nrf24_ce_set(LOW);
    delay_us(15);

}

void se8r01_setup()
{
    gtemp[0]=0x28;
    gtemp[1]=0x32;//RF_CH
    gtemp[2]=0x80;
    gtemp[3]=0x90;
    gtemp[4]=0x00;
    SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK0_SETUP_VALUE, gtemp, 5);

    delay_us(2);

    se8r01_switch_bank(iBANK1);

    gtemp[0]=0x40;
    gtemp[1]=0x01;
    gtemp[2]=0x30;
    gtemp[3]=0xE2;
    SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK1_PLL_CTL0, gtemp, 4);

    gtemp[0]=0x29;
    gtemp[1]=0x89;
    gtemp[2]=0x55;
    gtemp[3]=0x40;
    gtemp[4]=0x50;
    SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK1_CAL_CTL, gtemp, 5);

    gtemp[0]=0x29;
    SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK1_FDEV, gtemp, 1);

    gtemp[0]=0x55;
    gtemp[1]=0xC2;
    gtemp[2]=0x09;
    gtemp[3]=0xAC;
    SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK1_RX_CTRL, gtemp, 4);

    gtemp[0]=0x00;
    gtemp[1]=0x14;
    gtemp[2]=0x08;
    gtemp[3]=0x29;
    SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK1_FAGC_CTRL_1, gtemp, 4);

    gtemp[0]=0x02;
    gtemp[1]=0xC1;
    gtemp[2]=0xCB;
    gtemp[3]=0x1C;
    SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK1_AGC_GAIN, gtemp, 4);

    gtemp[0]=0x97;
    gtemp[1]=0x64;
    gtemp[2]=0x00;
    gtemp[3]=0x01;
    SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK1_RF_IVGEN, gtemp, 4);

    gtemp[0]=0x2A;
    gtemp[1]=0x04;
    gtemp[2]=0x00;
    gtemp[3]=0x7D;
    SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK1_TEST_PKDET, gtemp, 4);

    se8r01_switch_bank(iBANK0);
}



// Clocks only one byte into the given nrf24 register //
void nrf24_write_register(uint8_t reg, uint8_t value)
{
    nrf24_csn_set(LOW);
    SPI_RW(iRF_CMD_WRITE_REG | (REGISTER_MASK & reg));
    SPI_RW(value);
    nrf24_csn_set(HIGH);
}

// Read single register from nrf24 //
void nrf24_read_register_buf(uint8_t reg, uint8_t* value, uint8_t len)
{
    nrf24_csn_set(LOW);
    SPI_Read_Buf(iRF_CMD_READ_REG | (REGISTER_MASK & reg), value, len);
    nrf24_csn_set(HIGH);
}

// Write to a single register of nrf24 //
void nrf24_write_register_buf(uint8_t reg, uint8_t* value, uint8_t len)
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
uint8_t SPI_RW(uint8_t tx)
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
uint8_t SPI_RW_Reg(uint8_t reg, uint8_t value)
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
uint8_t SPI_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t bytes)
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
uint8_t SPI_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t bytes)
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




