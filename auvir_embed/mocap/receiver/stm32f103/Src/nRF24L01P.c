#include "stm32f1xx_hal.h"
#include "nRF24L01P.h"

//define the private constants in this library
//#define TX_ADR_WIDTH 5
//#define RX_ADR_WIDTH 5
extern SPI_HandleTypeDef hspi1;

uint8_t TxBuf[Buffer_Size] = {0};
uint8_t RxBuf[Buffer_Size] = {0};

uint8_t nRF24L01_Freq = 0;
uint8_t nRF24L01_power_rate = 0;

//define the initial Address
uint8_t  TX_ADDRESS[ADR_WIDTH]= {0xE7,0xE7,0xE7,0xE7,0xE7};
uint8_t  RX_ADDRESS[ADR_WIDTH]= {0xE7,0xE7,0xE7,0xE7,0xE7};

//Define the layer1:HW operation
void nrf24_start_spi_communication();
void nrf24_stop_spi_communication();



//Define the layer3:application operation
/****************************************

All the functions are in "nRF24l01P.h"

****************************************/

//Define the layer3 functions

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

uint8_t nrf24_configure(uint8_t freq, uint8_t power, uint8_t Rate)
{
    nRF24L01_Freq = 0;
    nRF24L01_power_rate = 0;

    if((freq>125)&&(freq<0))
        return 0;
    else
        nRF24L01_Freq = freq;

    if (P0dBm == power)
        nRF24L01_power_rate|=0x06;
    else if (Pm6dBm == power)
        nRF24L01_power_rate|=0x04;
    else if (Pm12dBm == power)
        nRF24L01_power_rate|=0x02;
    else if (Pm18dBm == power)
        nRF24L01_power_rate|=0x00;
    else
        return 0;

    if (R2mbps == Rate)
        {nRF24L01_power_rate|=0x08;}
    else if (Rate == R1mbps)
        {nRF24L01_power_rate|=0x00;}
    else if (Rate == R250kbps)
        nRF24L01_power_rate|=0x20;
    else
        return 0;

    return 1;

}

void nrf24_set_rx_mode(void)
{
	uint8_t buf[5]={0};

    nrf24_read_buf(TX_ADDR, buf, ADR_WIDTH);

    nrf24_write_buf(WRITE_nRF_REG + RX_ADDR_P0, RX_ADDRESS, ADR_WIDTH);

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
    nrf24_write_buf(WR_TX_PLOAD, tx_buf, TX_PLOAD_WIDTH);


}

uint8_t nrf24_receive_packet(uint8_t* rx_buf)
{
    uint8_t flag=0;
    uint8_t status;

    status=nrf24_read_register(NRFRegSTATUS);

    if(status & 0x40)
    {

         nrf24_read_buf(RD_RX_PLOAD,rx_buf,TX_PLOAD_WIDTH);
         flag =1;
    }
    nrf24_write_register(NRFRegSTATUS, status);
    return flag;
}

//Define the layer2 functions
uint8_t nrf24_read_register(uint8_t reg)
{
    uint8_t reg_val;
    uint8_t read_reg_command = READ_nRF_REG | reg;
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
    uint8_t write_reg_command = WRITE_nRF_REG | reg;
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
    uint8_t read_reg_command = READ_nRF_REG | reg;

    // Set CSN low, init SPI tranaction
    nrf24_start_spi_communication();

    // Select register to write to and read status uint8_t
    hal_status = HAL_SPI_Transmit_IT(&hspi1, &read_reg_command, 1);
    for(size_t i=0;i<Len;i++)
    {
        HAL_SPI_Receive_IT(&hspi1, &pBuf[i], 1);
    }

    // CSN high, terminate SPI communication
    nrf24_stop_spi_communication();

    status = hal_status;    // TODO
    return(status);
}

uint8_t nrf24_write_buf(uint8_t reg, uint8_t *pBuf, uint8_t Len)
{
    unsigned int status,i;
    uint8_t write_reg_command = WRITE_nRF_REG | reg;

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
