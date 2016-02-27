#ifndef NRF24
#define NRF24

#include <stdbool.h>

// Memory Map //
#define CONFIG   0x00
#define EN_AA    0x01
#define EN_RXADDR   0x02
#define SETUP_AW 0x03
#define SETUP_RETR  0x04
#define RF_CH    0x05
#define RF_SETUP 0x06
#define STATUS   0x07
#define OBSERVE_TX  0x08
#define RPD    0x09 // aka CD, carrier detect
#define RX_ADDR_P0  0x0A
#define RX_ADDR_P1  0x0B
#define RX_ADDR_P2  0x0C
#define RX_ADDR_P3  0x0D
#define RX_ADDR_P4  0x0E
#define RX_ADDR_P5  0x0F
#define TX_ADDR  0x10
#define RX_PW_P0 0x11
#define RX_PW_P1 0x12
#define RX_PW_P2 0x13
#define RX_PW_P3 0x14
#define RX_PW_P4 0x15
#define RX_PW_P5 0x16
#define FIFO_STATUS 0x17
#define DYNPD    0x1C

// Bit Mnemonics //

// CONFIG register //
#define MASK_RX_DR  6
#define MASK_TX_DS  5
#define MASK_MAX_RT 4
#define EN_CRC   3
#define CRCO  2
#define PWR_UP   1
#define PRIM_RX  0

// enable auto acknowledgment //
#define ENAA_P5  5
#define ENAA_P4  4
#define ENAA_P3  3
#define ENAA_P2  2
#define ENAA_P1  1
#define ENAA_P0  0

// enable rx addresses //
#define ERX_P5   5
#define ERX_P4   4
#define ERX_P3   3
#define ERX_P2   2
#define ERX_P1   1
#define ERX_P0   0

// setup of address width //
#define AW    0 // 2 bits //

// setup of auto re-transmission //
#define ARD   4 // 4 bits //
#define ARC   0 // 4 bits //

// RF setup register //
#define PLL_LOCK    4
#define RF_DR_LOW   5
#define RF_DR_HIGH  3
#define RF_PWR      1 // 2 bits //

//RF_DR_HIGH Select between the high speed data rates. This bit
//is don’t care if RF_DR_LOW is set.
//[RF_DR_LOW, RF_DR_HIGH]:
//‘00’ – 1Mbps
//‘01’ – 2Mbps
//‘10’ – 250kbps
//‘11’ – Reserved


// general status register //
#define RX_DR    6
#define TX_DS    5
#define MAX_RT   4
#define RX_P_NO  1 // 3 bits //
#define TX_FULL  0

// transmit observe register //
#define PLOS_CNT 4 // 4 bits //
#define ARC_CNT  0 // 4 bits //

// fifo status //
#define TX_REUSE 6
#define FIFO_FULL   5
#define TX_EMPTY 4
#define RX_FULL  1
#define RX_EMPTY 0

// dynamic length //
#define DPL_P0   0
#define DPL_P1   1
#define DPL_P2   2
#define DPL_P3   3
#define DPL_P4   4
#define DPL_P5   5

// Instruction Mnemonics //
#define R_REGISTER 0x00 // last 4 bits will indicate reg. address //
#define W_REGISTER 0x20 // last 4 bits will indicate reg. address //
#define REGISTER_MASK 0x1F
#define R_RX_PAYLOAD  0x61
#define W_TX_PAYLOAD  0xA0
#define FLUSH_TX   0xE1
#define FLUSH_RX   0xE2
#define REUSE_TX_PL   0xE3
#define ACTIVATE   0x50
#define R_RX_PL_WID   0x60
#define NOP     0xFF


///==========================================================================
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

typedef enum{
    NRF24_TRANSMISSON_OK,
    NRF24_MESSAGE_LOST,
    NRF24_MESSAGE_SENDING
}TransmissionStatus;

// SPI chip enable pin //
#ifndef NRF24_CSN_PIN
#define NRF24_CSN_PORT  GPIOA
#define NRF24_CSN_PIN   GPIO_PIN_0
#endif

// Chip enable for transmitting //
#ifndef NRF24_CE_PIN
#define NRF24_CE_PORT   GPIOA
#define NRF24_CE_PIN    GPIO_PIN_1
#endif

#define LOW GPIO_PIN_RESET
#define HIGH GPIO_PIN_SET

#define nrf24_ADDR_LEN 5
#define nrf24_ENABLE_1_BYTE_CRC ((0<<EN_CRC)|(0<<CRCO))

// adjustment functions //
void nrf24_init();
void nrf24_set_rx_address(uint8_t* adr);
void nrf24_set_tx_address(uint8_t* adr);
void nrf24_config(uint8_t channel, uint8_t pay_length);

// state check functions
uint8_t nrf24_get_status_register();
bool nrf24_is_data_ready();
bool nrf24_is_sending();
bool nrf24_is_rx_fifo_empty();

// core rx & tx functions //
void nrf24_send(uint8_t* value);
void nrf24_receive(uint8_t* data);

// Returns the payload length //
uint8_t nrf24_get_payload_len();

// use in dynamic length mode //
uint8_t nrf24_get_rx_fifo_pending_data_length();

// post transmission analysis //
TransmissionStatus nrf24_last_messageStatus();
uint8_t nrf24_get_last_msg_retransmission_count();


bool is_register_bit_set(uint8_t reg_name, uint8_t bit);
void nrf24_reset_register_bit(uint8_t reg_name, uint8_t bit);

// power management //
void nrf24_powerUpRx();
void nrf24_powerUpTx();
void nrf24_powerDown();
void nrf24_reset();

// low level interface ... //
uint8_t nrf24_spi_transaction(uint8_t tx);
void nrf24_transmitSync(uint8_t* dataout,uint8_t len);
void nrf24_transferSync(uint8_t* dataout,uint8_t* datain,uint8_t len);

void nrf24_read_register_multi(uint8_t reg, uint8_t* value, uint8_t len);
void nrf24_write_register_multi(uint8_t reg, uint8_t* value, uint8_t len);
void nrf24_write_register(uint8_t reg, uint8_t value);


// -------------------------------------------------------------------------- //
// You should implement the platform spesific functions in your code //
// -------------------------------------------------------------------------- //


// -------------------------------------------------------------------------- //
// nrf24 CE pin control function
// - state:1 => Pin HIGH
// - state:0 => Pin LOW  //
// -------------------------------------------------------------------------- //
void nrf24_ce_set(uint8_t state);

// -------------------------------------------------------------------------- //
// nrf24 CE pin control function
// - state:1 => Pin HIGH
// - state:0 => Pin LOW  //
// -------------------------------------------------------------------------- //
void nrf24_csn_set(uint8_t state);


#endif
