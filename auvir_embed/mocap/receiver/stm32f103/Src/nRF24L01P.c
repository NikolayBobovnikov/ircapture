#include "stm32f1xx_hal.h"
#include "nRF24L01P.h"

extern SPI_HandleTypeDef hspi1;
extern void nrf24_setup_gpio();

/*
void nrf24_set_tx_address(nrf24_addr *addr)
{
 TX_ADDRESS[0] = addr->byte_0;
 TX_ADDRESS[1] = addr->byte_1;
 TX_ADDRESS[2] = addr->byte_2;
 TX_ADDRESS[3] = addr->byte_3;
 TX_ADDRESS[4] = addr->byte_4;
}
void nrf24_set_rx_address(	nrf24_addr *addr)
{
 RX_ADDRESS[0] = addr->byte_0;
 RX_ADDRESS[1] = addr->byte_1;
 RX_ADDRESS[2] = addr->byte_2;
 RX_ADDRESS[3] = addr->byte_3;
 RX_ADDRESS[4] = addr->byte_4;
}

uint8_t nrf24_setup(uint8_t freq, uint8_t power, uint8_t Rate)
{
 nRF24L01_Freq = 0;
 nRF24L01_power_rate = 0;

 if(freq > 125){
  // max channel is 125
  freq = 125;
 }
 nRF24L01_Freq = freq;

 switch(power){
  case P0dBm:{
   nRF24L01_power_rate|=0x06;
   break;
  }
  case Pm6dBm:{
   nRF24L01_power_rate|=0x04;
   break;
  }
  case Pm12dBm:{
   nRF24L01_power_rate|=0x02;
   break;
  }
  case Pm18dBm:{
   nRF24L01_power_rate|=0x00;
   break;
  }
  default:{
   //P0dBm
   nRF24L01_power_rate|=0x06;
  }
 }

 return 1;

}

void nrf24_init(NRF24_InitTypeDef *params)
{
 nrf24_init_pins();
 HAL_GPIO_WritePin(NRF24L01_CE_PORT,NRF24L01_CE_PIN, GPIO_PIN_RESET);
 HAL_GPIO_WritePin(NRF24L01_CSN_PORT,NRF24L01_CSN_PIN, GPIO_PIN_SET);

 nrf24_setup(params->frequency,params->power, params->rate);

 nrf24_set_rx_mode();
}


void nrf24_set_rx_mode(void)
{
 uint8_t buf[5]={0};

 nrf24_read_buf(TX_ADDR, buf, ADR_WIDTH);

 nrf24_write_buf(W_REGISTER + RX_ADDR_P0, RX_ADDRESS, ADR_WIDTH);

 nrf24_write_register(EN_AA, 0);
 nrf24_write_register(EN_RXADDR, 0x01);
 nrf24_write_register(SETUP_RETR, 0x1a);
 nrf24_write_register(RF_CH, nRF24L01_Freq);
 nrf24_write_register(RX_PW_P0, RX_PLOAD_WIDTH);
 nrf24_write_register(RF_SETUP, nRF24L01_power_rate);

 nrf24_write_register(CONFIG, 0x03);

 HAL_Delay(200);
}

void nrf24_set_tx_mode(void)
{
 nrf24_write_buf(TX_ADDR, TX_ADDRESS, ADR_WIDTH);
 nrf24_write_buf(RX_ADDR_P0, RX_ADDRESS, ADR_WIDTH);
 nrf24_write_register(EN_AA, 0);
 nrf24_write_register(EN_RXADDR, 0x01);
 nrf24_write_register(SETUP_RETR, 0x1a);
 nrf24_write_register(RF_CH,nRF24L01_Freq);
 nrf24_write_register(RF_SETUP,  nRF24L01_power_rate);
 nrf24_write_register(CONFIG, 0x02);

}


void nrf24_transmit_packet(uint8_t * tx_buf)
{

 nrf24_write_buf(RX_ADDR_P0, TX_ADDRESS, ADR_WIDTH);
 nrf24_write_buf(W_TX_PAYLOAD, tx_buf, TX_PLOAD_WIDTH);
}

uint8_t nrf24_receive_packet(uint8_t* rx_buf)
{
 uint8_t flag=0;
 uint8_t status;

 status=nrf24_read_register(STATUS);

 if(status & 0x40)
 {

   nrf24_read_buf(R_RX_PAYLOAD,rx_buf,TX_PLOAD_WIDTH);
   flag =1;
 }
 nrf24_write_register(STATUS, status);
 return flag;
}

//Define the layer2 functions
uint8_t nrf24_read_register(uint8_t reg)
{
 uint8_t reg_val;
 uint8_t read_reg_command = R_REGISTER | reg;
 // CSN low, initialize SPI communication...
 nrf24_start_spi_communication();

 // Select register to read from..
 HAL_SPI_Transmit_IT(&hspi1, &read_reg_command, 1);

 // Read the register
 HAL_SPI_Receive_IT(&hspi1, &reg_val, 1);

 // CSN high, terminate SPI communication
 nrf24_stop_spi_communication();

 // return register value
 return(reg_val);
}

uint8_t nrf24_write_register(uint8_t reg, uint8_t value)
{
 uint8_t status;
 uint8_t write_reg_command = W_REGISTER | reg;
 HAL_StatusTypeDef hal_status;

 // CSN low, init SPI transaction
 nrf24_start_spi_communication();

 // select register
 hal_status = HAL_SPI_Transmit_IT(&hspi1, &write_reg_command, 1);

 // ..and write value to it..
 hal_status = HAL_SPI_Transmit_IT(&hspi1, &value, 1);

 // CSN high again
 nrf24_stop_spi_communication();

 return(status);
}

uint8_t nrf24_read_buf(uint8_t reg, uint8_t *pBuf, uint8_t Len)
{
 uint8_t status;
 HAL_StatusTypeDef hal_status;
 uint8_t read_reg_command = R_REGISTER | reg;

 // Set CSN low, init SPI tranaction
 nrf24_start_spi_communication();

 // Select register to write to and read status uint8_t
 uint8_t read_reg_command = R_REGISTER | reg;
 hal_status = HAL_SPI_Transmit_IT(&hspi1, &read_reg_command, 1);
 for(size_t i=0;i<Len;i++)
 {
  HAL_SPI_Receive_IT(&hspi1, &pBuf[i], 1);
 }

 // CSN high, terminate SPI communication
 nrf24_stop_spi_communication();

 status = hal_status; // TODO
 return(status);
}

uint8_t nrf24_write_buf(uint8_t reg, uint8_t *pBuf, uint8_t Len)
{
 unsigned int status,i;
 uint8_t write_reg_command = W_REGISTER | reg;

 nrf24_start_spi_communication();
 HAL_SPI_Transmit_IT(&hspi1, &write_reg_command, 1);
 for(i=0; i<Len; i++) //
 {
  HAL_SPI_Transmit_IT(&hspi1, &pBuf[i], 1);
 }
 nrf24_stop_spi_communication();
 return(status);
}


void nrf24_stop_spi_communication(void)
{
 HAL_GPIO_WritePin(NRF24L01_CSN_PORT,NRF24L01_CSN_PIN, GPIO_PIN_SET);
}

void nrf24_start_spi_communication(void)
{
 HAL_GPIO_WritePin(NRF24L01_CSN_PORT,NRF24L01_CSN_PIN, GPIO_PIN_RESET);
}

void nrf24_init_pins()
{
 HAL_GPIO_WritePin(NRF24L01_CE_PORT,NRF24L01_CE_PIN, GPIO_PIN_RESET);
 HAL_GPIO_WritePin(NRF24L01_CSN_PORT,NRF24L01_CSN_PIN, GPIO_PIN_SET);
}
*/

uint8_t payload_len;

void nrf24_init()
{
    nrf24_setup_gpio();

    nrf24_ce_set(LOW);
    nrf24_csn_set(HIGH);
}

void nrf24_config(uint8_t channel, uint8_t pay_length)
{
    // Use static payload length ... //
    payload_len = pay_length;

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

void nrf24_config_rx(uint8_t *pipe_addr, uint8_t channel, uint8_t pay_length)
{
    // setup addresses for pipe
    nrf24_write_register_multi(RX_ADDR_P1, pipe_addr, nrf24_ADDR_LEN);

    // Use static payload length ... //
    payload_len = pay_length;

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

void nrf24_config_tx(uint8_t *pipe_addr, uint8_t channel, uint8_t pay_length)
{
    // setup addresses for pipes
    //nrf24_write_register_multi(RX_ADDR_P0, pipe_addr, nrf24_ADDR_LEN);
    nrf24_write_register_multi(TX_ADDR, pipe_addr, nrf24_ADDR_LEN);

    // Use static payload length ... //
    payload_len = pay_length;

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
    nrf24_write_register_multi(RX_ADDR_P0,adr,nrf24_ADDR_LEN);
    //nrf24_ce_set(HIGH);
}

void nrf24_set_tx_address(uint8_t* adr)
{
    // RX_ADDR_P0 must be set to the sending addr for auto ack to work. //
   nrf24_write_register_multi(RX_ADDR_P0, adr, nrf24_ADDR_LEN);
   nrf24_write_register_multi(TX_ADDR, adr, nrf24_ADDR_LEN);
}

uint8_t nrf24_get_payload_len()
{
    return payload_len;
}

uint8_t nrf24_get_rx_fifo_pending_data_length()
{
    uint8_t status;
    nrf24_csn_set(LOW);
    nrf24_spi_transaction(R_RX_PL_WID);
    status = nrf24_spi_transaction(0x00);
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

    nrf24_read_register_multi(FIFO_STATUS, &fifoStatus,1);

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
    nrf24_spi_transaction( R_RX_PAYLOAD );

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
    nrf24_read_register_multi(OBSERVE_TX,&rv,1);
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
    nrf24_spi_transaction(FLUSH_TX);

    // Pull up chip select //
    nrf24_csn_set(HIGH);
#endif

    // Pull down chip select //
    nrf24_csn_set(LOW);

    // Write cmd to write payload //
    nrf24_spi_transaction(W_TX_PAYLOAD);

    // Write payload //
    nrf24_transmitSync(value,payload_len);

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
    rv = nrf24_spi_transaction(NOP);
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
    nrf24_spi_transaction(FLUSH_RX);
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
    nrf24_spi_transaction(FLUSH_RX);
    nrf24_csn_set(HIGH);

    //4)write status register as 0x0e;
    nrf24_write_register(STATUS,0x0E);
}
// Clocks only one byte into the given nrf24 register //
void nrf24_write_register(uint8_t reg, uint8_t value)
{
    nrf24_csn_set(LOW);
    nrf24_spi_transaction(W_REGISTER | (REGISTER_MASK & reg));
    nrf24_spi_transaction(value);
    nrf24_csn_set(HIGH);
}

// Read single register from nrf24 //
void nrf24_read_register_multi(uint8_t reg, uint8_t* value, uint8_t len)
{
    nrf24_csn_set(LOW);
    nrf24_spi_transaction(R_REGISTER | (REGISTER_MASK & reg));
    nrf24_transferSync(value,value,len);
    nrf24_csn_set(HIGH);
}

// Write to a single register of nrf24 //
void nrf24_write_register_multi(uint8_t reg, uint8_t* value, uint8_t len)
{

    nrf24_csn_set(LOW);
    nrf24_spi_transaction(W_REGISTER | (REGISTER_MASK & reg));
    nrf24_transmitSync(value,len);
    nrf24_csn_set(HIGH);
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

// @param: byte to be sent
// @returns: nrf24 STATUS register
uint8_t nrf24_spi_transaction(uint8_t tx)
{
    uint8_t rx = 0;

    HAL_SPI_TransmitReceive_IT(&hspi1, &tx, &rx, 1);

    return rx;
}

// send and receive multiple bytes over SPI //
void nrf24_transferSync(uint8_t* dataout,uint8_t* datain,uint8_t len)
{
    uint8_t i;

    for(i=0;i<len;i++)
    {
        datain[i] = nrf24_spi_transaction(dataout[i]);
    }

}

// send multiple bytes over SPI //
void nrf24_transmitSync(uint8_t* dataout,uint8_t len)
{
    uint8_t i;
    for(i=0;i<len;i++)
    {
        nrf24_spi_transaction(dataout[i]);
    }

}

void nrf24_reset_register_bit(uint8_t reg_name, uint8_t bit)
{
    uint8_t reg = 0;
    nrf24_read_register_multi(reg_name, &reg, 1);
    nrf24_write_register(reg_name, reg | (1 << bit));
}

bool is_register_bit_set(uint8_t reg_name, uint8_t bit)
{
    uint8_t reg = 0;
    nrf24_read_register_multi(reg_name, &reg, 1);
    if(reg & (1 << bit)){
        return true;
    }
    return false;
}
