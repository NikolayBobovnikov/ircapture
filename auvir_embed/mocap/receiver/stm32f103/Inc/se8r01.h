#ifndef se8r01_h
#define se8r01_h

///
/// TODO: on initialization se8r01 check http://forum.easyelectronics.ru/viewtopic.php?f=9&t=21484
///

#include <stdbool.h>

// Memory Map //
#define CONFIG      0x00
#define EN_AA       0x01
#define EN_RXADDR   0x02
#define SETUP_AW    0x03
#define SETUP_RETR  0x04
#define RF_CH       0x05
#define RF_SETUP    0x06
#define STATUS      0x07
#define OBSERVE_TX  0x08
#define RPD         0x09 // aka CD, carrier detect
#define RX_ADDR_P0  0x0A
#define RX_ADDR_P1  0x0B
#define RX_ADDR_P2  0x0C
#define RX_ADDR_P3  0x0D
#define RX_ADDR_P4  0x0E
#define RX_ADDR_P5  0x0F
#define TX_ADDR     0x10
#define RX_PW_P0    0x11
#define RX_PW_P1    0x12
#define RX_PW_P2    0x13
#define RX_PW_P3    0x14
#define RX_PW_P4    0x15
#define RX_PW_P5    0x16
#define FIFO_STATUS 0x17
#define DYNPD       0x1C

// Bit Mnemonics //

// CONFIG register //
#define MASK_RX_DR  6
#define MASK_TX_DS  5
#define MASK_MAX_RT 4
#define EN_CRC      3
#define CRCO        2
#define PWR_UP      1
#define PRIM_RX     0

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
// for NRF24L01
#define RF_PWR      1   //2 bits
//for SE8R01
#define PA_PWR      0   //3 bits

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
#define R_REGISTER      0x00 // last 4 bits will indicate reg. address //
#define W_REGISTER      0x20 // last 4 bits will indicate reg. address //
#define REGISTER_MASK   0x1F
#define R_RX_PAYLOAD    0x61
#define W_TX_PAYLOAD    0xA0
#define FLUSH_TX        0xE1
#define FLUSH_RX        0xE2
#define REUSE_TX_PL     0xE3
#define ACTIVATE        0x50
#define R_RX_PL_WID     0x60
#define NOP             0xFF

//
#define STA_MARK_RX     0X40
#define STA_MARK_TX     0X20
#define STA_MARK_MX     0X10

#define iBANK0                  0x00
#define iBANK1                  0x80

// SE8R01 SPI Commands
#define iRF_CMD_READ_REG        0x1F            // 000x xxxx Define read command to register
#define iRF_CMD_WRITE_REG       0x20            // 001x xxxx Define write command to register
#define iRF_CMD_RD_RX_PLOAD     0x61            // 0110 0001 Define RX payload register address
#define iRF_CMD_WR_TX_PLOAD     0xA0            // 1010 0000 Define TX payload register address
#define iRF_CMD_FLUSH_TX        0xE1            // 1110 0001 Define flush TX register command
#define iRF_CMD_FLUSH_RX        0xE2            // 1110 0010 Define flush RX register command
#define iRF_CMD_REUSE_TX_PL     0xE3            // 1110 0011 Define reuse TX payload register command
#define iRF_CMD_W_TX_PAYLOAD_NOACK 0xB0         // 1011 0000
#define iRF_CMD_W_ACK_PAYLOAD   0xa8            // 1010 1xxx
#define iRF_CMD_ACTIVATE        0x50            // 0101 0000
#define iRF_CMD_R_RX_PL_WID     0x60            // 0110 0000
#define iRF_CMD_NOP             0xFF            // 1111 1111 Define No Operation, might be used to read status register


// SE8R01 registers addresses
#define iRF_BANK0_CONFIG        0x00            // 'Config' register address
#define iRF_BANK0_EN_AA         0x01            // 'Enable Auto Acknowledgment' register address
#define iRF_BANK0_EN_RXADDR     0x02            // 'Enabled RX addresses' register address
#define iRF_BANK0_SETUP_AW      0x03            // 'Setup address width' register address
#define iRF_BANK0_SETUP_RETR    0x04            // 'Setup Auto. Retrans' register address
#define iRF_BANK0_RF_CH         0x05            // 'RF channel' register address
#define iRF_BANK0_RF_SETUP      0x06            // 'RF setup' register address
#define iRF_BANK0_STATUS        0x07            // 'Status' register address
#define iRF_BANK0_OBSERVE_TX    0x08            // 'Observe TX' register address
#define iRF_BANK0_RPD           0x09            // 'Received Power Detector' register address
#define iRF_BANK0_RX_ADDR_P0    0x0A            // 'RX address pipe0' register address
#define iRF_BANK0_RX_ADDR_P1    0x0B            // 'RX address pipe1' register address
#define iRF_BANK0_RX_ADDR_P2    0x0C            // 'RX address pipe2' register address
#define iRF_BANK0_RX_ADDR_P3    0x0D            // 'RX address pipe3' register address
#define iRF_BANK0_RX_ADDR_P4    0x0E            // 'RX address pipe4' register address
#define iRF_BANK0_RX_ADDR_P5    0x0F            // 'RX address pipe5' register address
#define iRF_BANK0_TX_ADDR       0x10            // 'TX address' register address
#define iRF_BANK0_RX_PW_P0      0x11            // 'RX payload width, pipe0' register address
#define iRF_BANK0_RX_PW_P1      0x12            // 'RX payload width, pipe1' register address
#define iRF_BANK0_RX_PW_P2      0x13            // 'RX payload width, pipe2' register address
#define iRF_BANK0_RX_PW_P3      0x14            // 'RX payload width, pipe3' register address
#define iRF_BANK0_RX_PW_P4      0x15            // 'RX payload width, pipe4' register address
#define iRF_BANK0_RX_PW_P5      0x16            // 'RX payload width, pipe5' register address
#define iRF_BANK0_FIFO_STATUS   0x17            // 'FIFO Status Register' register address
#define iRF_BANK0_DYNPD         0x1C            // 'Enable dynamic payload length' register address
#define iRF_BANK0_FEATURE       0x1D            // 'Feature' register address
#define iRF_BANK0_SETUP_VALUE   0x1E
#define iRF_BANK0_PRE_GURD      0x1F

//SE8R01 Bank1 register
#define iRF_BANK1_LINE          0x00
#define iRF_BANK1_PLL_CTL0      0x01
#define iRF_BANK1_PLL_CTL1      0x02
#define iRF_BANK1_CAL_CTL       0x03
#define iRF_BANK1_A_CNT_REG     0x04
#define iRF_BANK1_B_CNT_REG     0x05
#define iRF_BANK1_RESERVED0     0x06
#define iRF_BANK1_STATUS        0x07
#define iRF_BANK1_STATE         0x08
#define iRF_BANK1_CHAN          0x09
#define iRF_BANK1_IF_FREQ       0x0A
#define iRF_BANK1_AFC_COR       0x0B
#define iRF_BANK1_FDEV          0x0C
#define iRF_BANK1_DAC_RANGE     0x0D
#define iRF_BANK1_DAC_IN        0x0E
#define iRF_BANK1_CTUNING       0x0F
#define iRF_BANK1_FTUNING       0x10
#define iRF_BANK1_RX_CTRL       0x11
#define iRF_BANK1_FAGC_CTRL     0x12
#define iRF_BANK1_FAGC_CTRL_1   0x13
#define iRF_BANK1_DAC_CAL_LOW   0x17
#define iRF_BANK1_DAC_CAL_HI    0x18
#define iRF_BANK1_RESERVED1     0x19
#define iRF_BANK1_DOC_DACI      0x1A
#define iRF_BANK1_DOC_DACQ      0x1B
#define iRF_BANK1_AGC_CTRL      0x1C
#define iRF_BANK1_AGC_GAIN      0x1D
#define iRF_BANK1_RF_IVGEN      0x1E
#define iRF_BANK1_TEST_PKDET    0x1F

// SE8R01 interrupt status
#define iSTATUS_RX_DR           0x40
#define iSTATUS_TX_DS           0x20
#define iSTATUS_MAX_RT          0x10
#define iSTATUS_TX_FULL         0x01

// SE8R01 FIFO status
#define iFIFO_STATUS_TX_REUSE   0x40
#define iFIFO_STATUS_TX_FULL    0x20
#define iFIFO_STATUS_TX_EMPTY   0x10

#define iFIFO_STATUS_RX_FULL    0x02
#define iFIFO_STATUS_RX_EMPTY   0x01

///==========================================================================
#define TX_ADR_WIDTH    5   // 5 uint8_ts TX(RX) address width
#define TX_PLOAD_WIDTH  32  // 32 uint8_ts TX payload


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

// NRF IRQ pin//
#ifndef NRF24_IRQ_PIN
#define NRF24_IRQ_PORT   GPIOA
#define NRF24_IRQ_PIN    GPIO_PIN_4
#endif

#define LOW GPIO_PIN_RESET
#define HIGH GPIO_PIN_SET

typedef enum {
 Power_plus5dBm1,
 Power_0dBm1,
 Power_minus6dBm1,
 Power_minus12dBm1,
 Power_minus18dBm1
} NRF24_Power;

typedef enum {
 DataRate_2mbps,
 DataRate_1mbps,
 DataRate_500kbps,
 DataRate_250kbps
} NRF24_Rate;

//TODO
typedef struct NRF24_InitTypeDef{
 uint8_t frequency;
 NRF24_Power power;
 NRF24_Rate rate;
 bool auto_ack_enabled;
 bool crc_enabled;
 uint8_t crc_num_bytes;
 uint8_t address_width;
 uint8_t num_retries;
 uint8_t pipe;
} NRF24_InitTypeDef;

typedef enum{
    NRF24_TRANSMISSON_OK,
    NRF24_MESSAGE_LOST,
    NRF24_MESSAGE_SENDING
}TransmissionStatus;

void setup_radio(NRF24_InitTypeDef* settings);

void nrf24_ce_set(uint8_t state);
void nrf24_csn_set(uint8_t state);

void setup();
void loop();
void RXX();
void TXX();
void radio_settings();
void init_io(void);

uint8_t SPI_RW(uint8_t tx);
uint8_t SPI_RW_Reg(uint8_t reg, uint8_t value);
uint8_t SPI_Read(uint8_t reg);
uint8_t SPI_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t bytes);
uint8_t SPI_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t bytes);

void se8r01_switch_bank(uint8_t bankindex);
void se8r01_powerup();
void se8r01_calibration();
void se8r01_setup();

//
// adjustment functions //
void nrf24_init();
void nrf24_set_rx_address(uint8_t* adr);
void nrf24_set_tx_address(uint8_t* adr);
void nrf24_config(uint8_t channel);
void nrf24_config_rx(uint8_t *pipe_addr, uint8_t channel);
void nrf24_config_tx(uint8_t *pipe_addr, uint8_t channel);

// state check functions
uint8_t nrf24_get_status_register();
bool nrf24_is_data_ready();
bool nrf24_is_sending();
bool nrf24_is_rx_fifo_empty();

// core rx & tx functions //
void nrf24_send(uint8_t* value);
void nrf24_receive(uint8_t* data);

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
uint8_t SPI_RW(uint8_t tx);
void nrf24_transmitSync(uint8_t* dataout,uint8_t len);
void nrf24_transferSync(uint8_t* dataout,uint8_t* datain,uint8_t len);

void nrf24_read_register_buf(uint8_t reg, uint8_t* value, uint8_t len);
void nrf24_write_register_buf(uint8_t reg, uint8_t* value, uint8_t len);
void nrf24_write_register(uint8_t reg, uint8_t value);


#endif
