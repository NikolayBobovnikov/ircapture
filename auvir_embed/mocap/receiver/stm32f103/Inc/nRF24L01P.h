/// Definitions for nrf24l01. TODO: move back to .c file

//Define the commands for operate the nRF24L01P
#define READ_nRF_REG    0x00  	// Command for read register
#define WRITE_nRF_REG   0x20 	// Command for read register
#define RD_RX_PLOAD     0x61  	// Command for read Rx payload
#define WR_TX_PLOAD     0xA0  	// Command for write Tx payload
#define FLUSH_TX        0xE1 	// Command for flush Tx FIFO
#define FLUSH_RX        0xE2  	// Command for flush Rx FIFO
#define REUSE_TX_PL     0xE3  	// Command for reuse Tx payload
#define NOP             0xFF  	// Reserve

//Define the register address for nRF24L01P
#define CONFIG          0x00  //  Configurate the status of transceiver, mode of CRC and the replay of transceiver status
#define EN_AA           0x01  //  Enable the atuo-ack in all channels
#define EN_RXADDR       0x02  //  Enable Rx Address
#define SETUP_AW        0x03  // Configurate the address width
#define SETUP_RETR      0x04  //  setup the retransmit
#define RF_CH           0x05  // Configurate the RF frequency
#define RF_SETUP        0x06  // Setup the rate of data, and transmit power
#define NRFRegSTATUS    0x07  //
#define OBSERVE_TX      0x08  //
#define CD              0x09  //    //Carrier detect
#define RX_ADDR_P0      0x0A  // Receive address of channel 0
#define RX_ADDR_P1      0x0B  // Receive address of channel 1
#define RX_ADDR_P2      0x0C  // Receive address of channel 2
#define RX_ADDR_P3      0x0D  // Receive address of channel 3
#define RX_ADDR_P4      0x0E  // Receive address of channel 4
#define RX_ADDR_P5      0x0F  // Receive address of channel 5
#define TX_ADDR         0x10  //       Transmit address
#define RX_PW_P0        0x11  //  Size of receive data in channel 0
#define RX_PW_P1        0x12  //  Size of receive data in channel 1
#define RX_PW_P2        0x13  //  Size of receive data in channel 2
#define RX_PW_P3        0x14  //  Size of receive data in channel 3
#define RX_PW_P4        0x15  // Size of receive data in channel 4
#define RX_PW_P5        0x16  //  Size of receive data in channel 5
#define FIFO_STATUS     0x17  // FIFO Status

///**************************************************************************************
#define ADR_WIDTH 5

typedef struct nrf24_addr{
    uint8_t byte_0;
    uint8_t byte_1;
    uint8_t byte_2;
    uint8_t byte_3;
    uint8_t byte_4;
} nrf24_addr;

typedef enum {
    P0dBm1,
    Pm6dBm1,
    Pm12dBm1,
    Pm18dBm1
} NRF24_Power;

typedef enum {
    R2mbps,
    R1mbps,
    R250kbps
} NRF24_Rate;

typedef struct NRF24_InitTypeDef{
    uint8_t frequency;
    NRF24_Power power;
    NRF24_Rate rate;
    uint8_t auto_ack;
    uint8_t crc_enabled;
    uint8_t crc_encoding;
    uint8_t address_width;
    uint8_t num_retries;
    uint8_t EN_RXADDR_PipeEnable;
    uint8_t rf_setup_reg;
} NRF24_InitTypeDef;


#define RX_PLOAD_WIDTH 20
#define TX_PLOAD_WIDTH 20
#define Buffer_Size 32

//Define RF power value
#define P0dBm 0
#define Pm6dBm 1
#define Pm12dBm 2
#define Pm18dBm 3

//#define RF rate
#define R2mbps 0
#define R1mbps 1
#define R250kbps 3

#define nrf24l01_SPI_Instance       SPI1
/* SPI chip enable pin */
#ifndef NRF24L01_CSN_PIN
#define NRF24L01_CSN_PORT           GPIOA
#define NRF24L01_CSN_PIN            GPIO_PIN_0
#endif

/* Chip enable for transmitting */
#ifndef NRF24L01_CE_PIN
#define NRF24L01_CE_PORT            GPIOA
#define NRF24L01_CE_PIN             GPIO_PIN_1
#endif

/// Exported functions
void nrf24_init_pins(void);
uint8_t nrf24_config(uint8_t freq, uint8_t power, uint8_t rate);
void nrf24_init(NRF24_InitTypeDef* params);

void nrf24_set_rx_mode(void);
void nrf24_set_tx_mode(void);
void nrf24_transmit_packet(uint8_t * tx_buf);
uint8_t nrf24_receive_packet(uint8_t* rx_buf);

void nrf24_set_tx_address(nrf24_addr *addr);
void nrf24_set_rx_address(nrf24_addr *addr);

/// Private functions. TODO: move back to .c file
uint8_t nrf24_read_register(uint8_t reg);
uint8_t nrf24_write_register(uint8_t reg, uint8_t value);

uint8_t nrf24_read_buf(uint8_t reg, uint8_t *pBuf, uint8_t Len);
uint8_t nrf24_write_buf(uint8_t reg, uint8_t *pBuf, uint8_t Len);

void nrf24_start_spi_communication();
void nrf24_stop_spi_communication();

