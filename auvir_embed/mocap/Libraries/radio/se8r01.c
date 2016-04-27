//this is a copy and paste job made by F2k

#include "se8r01.h"
#include "common.h"

///
/// on initialization se8r01 check http://forum.easyelectronics.ru/viewtopic.php?f=9&t=21484
///

extern SPI_HandleTypeDef hspi1;
extern char mode;      //r=rx, t=tx
extern GPIO_TypeDef * GPIO_LED_PORT;
extern uint16_t GPIO_LED_PIN;

uint8_t gtemp[5];

// Use two separate radiomodules for usb device
NRF_Module default_module = {0};
NRF_Module data_module = {0};

// Define a static TX address
const uint8_t Sensor_Beamer_DefaultAddress[TX_ADR_WIDTH] = {0x10,0x20,0x30,0xab,0xab};
uint8_t TX_ADDRESS[TX_ADR_WIDTH]  = {0x10,0x20,0x30,0xab,0xab};
uint8_t rx_buf[TX_PLOAD_WIDTH] = {0};
uint8_t tx_buf[TX_PLOAD_WIDTH] = {0};

extern RadioMessage rx_message;
extern RadioMessage tx_message;


//===============  Function prototypes
// should be defined in the application interface
static void nrf24_setup_modules_gpio();

static void nrf24_ce_set(NRF_Module * radiomodule, GPIO_PinState state);
static void nrf24_csn_set(NRF_Module * radiomodule, GPIO_PinState state);
static void radio_settings(NRF_Module * radiomodule);
static void set_rx_tx_mode(NRF_Module * radiomodule);

static void se8r01_switch_bank(NRF_Module * radiomodule, uint8_t bankindex);
static void se8r01_powerup(NRF_Module * radiomodule);
static void se8r01_calibration(NRF_Module * radiomodule);
static void se8r01_setup(NRF_Module * radiomodule);

static void set_power(NRF_Module * radiomodule, NRF_PowerState power_state);
static void power_on_tx(NRF_Module * radiomodule);
static void power_on_rx(NRF_Module * radiomodule);

static bool interrupt_happened(NRF_Module * radiomodule);

static uint8_t nrf_getStatus(NRF_Module * radiomodule);

//

// use in dynamic length mode //
static uint8_t nrf24_get_rx_fifo_pending_data_length(NRF_Module * radiomodule);

// post transmission analysis //
static TransmissionStatus nrf24_last_messageStatus(NRF_Module * radiomodule);
static uint8_t nrf24_get_last_msg_retransmission_count(NRF_Module * radiomodule);

// power management //
static void nrf24_powerDown(NRF_Module * radiomodule);
static void nrf24_reset(NRF_Module * radiomodule);

// low level interface ... //
static uint8_t SPI_RW(uint8_t tx);
static uint8_t SPI_RW_Reg(NRF_Module * radiomodule, uint8_t reg, uint8_t value);
static uint8_t SPI_Read(NRF_Module * radiomodule, uint8_t reg);
static uint8_t SPI_Read_Buf(NRF_Module * radiomodule, uint8_t reg, uint8_t *pBuf, uint8_t bytes);
static uint8_t SPI_Write_Buf(NRF_Module * radiomodule, uint8_t reg, uint8_t *pBuf, uint8_t bytes);

static void nrf24_transmitSync(uint8_t* dataout,uint8_t len);
static void nrf24_transferSync(uint8_t* dataout,uint8_t* datain,uint8_t len);

static void nrf24_read_register_buf(NRF_Module * radiomodule, uint8_t reg, uint8_t* value, uint8_t len);
static void nrf24_write_register_buf(NRF_Module * radiomodule, uint8_t reg, uint8_t* value, uint8_t len);
static void nrf24_write_register(NRF_Module * radiomodule, uint8_t reg, uint8_t value);

//=============== Function definitions
uint8_t nrf24_get_rx_fifo_pending_data_length(NRF_Module * radiomodule)
{
    uint8_t status;
    nrf24_csn_set(radiomodule, LOW);
    SPI_RW(R_RX_PL_WID);
    status = SPI_RW(0x00);
    nrf24_csn_set(radiomodule, HIGH);
    return status;
}

bool nrf24_is_data_ready(NRF_Module * radiomodule)
{
    // See note in getData() function - just checking RX_DR isn't good enough
    uint8_t status = nrf_getStatus(radiomodule);

    // We can short circuit on RX_DR, but if it's not set, we still need
    // to check the FIFO for any pending packets
    if ( status & (1 << RX_DR) )
    {
        return true;
    }

    return !nrf24_is_rx_fifo_empty(radiomodule);
}

bool nrf24_is_rx_fifo_empty(NRF_Module * radiomodule)
{
    uint8_t fifoStatus;

    nrf24_read_register_buf(radiomodule,FIFO_STATUS, &fifoStatus,1);

    //return (fifoStatus & (1 << RX_EMPTY)); // TODO: verify correctness
    if(fifoStatus & (1 << RX_EMPTY)){
        return true;
    }
    return false;
}

uint8_t nrf24_get_last_msg_retransmission_count(NRF_Module * radiomodule)
{
    uint8_t rv;
    nrf24_read_register_buf(radiomodule,OBSERVE_TX,&rv,1);
    rv = rv & 0x0F;
    return rv;
}

TransmissionStatus nrf24_last_messageStatus(NRF_Module * radiomodule)
{
    uint8_t rv = nrf_getStatus(radiomodule);
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

void nrf24_powerDown(NRF_Module * radiomodule)
{
    nrf24_ce_set(radiomodule, LOW);
    nrf24_write_register(radiomodule,CONFIG,0<<PWR_UP);
}

void nrf24_reset(NRF_Module * radiomodule)
{

    //1)use power down mode (PWR_UP = 0)
    nrf24_ce_set(radiomodule, LOW);
    nrf24_write_register(radiomodule,CONFIG,0<<PWR_UP);

    //2)clear data ready flag and data sent flag in status register
    nrf24_write_register(radiomodule,STATUS,(1<<RX_DR)|(1<<TX_DS)|(1<<MAX_RT));

    //3)flush tx/rx buffer
    nrf24_csn_set(radiomodule, LOW);
    SPI_RW(FLUSH_RX);
    nrf24_csn_set(radiomodule, HIGH);

    //4)write status register as 0x0e;
    nrf24_write_register(radiomodule,STATUS,0x0E);
}


//========================================
// Interface functions

static void nrf24_setup_modules_gpio()
{
    default_module.CE.Pin = NRF24_CE1_Pin;
    default_module.CE.Port = NRF24_CE1_GPIO_Port;
    default_module.CSN.Pin = NRF24_CSN1_Pin;
    default_module.CSN.Port = NRF24_CSN1_GPIO_Port;
    default_module.IRQ.Pin = NRF24_IRQ1_Pin;
    default_module.IRQ.Port = NRF24_IRQ1_GPIO_Port;

    data_module.CE.Pin  = NRF24_CE2_Pin;
    data_module.CE.Port = NRF24_CE2_GPIO_Port;
    data_module.CSN.Pin = NRF24_CSN2_Pin;
    data_module.CSN.Port= NRF24_CSN2_GPIO_Port;
    data_module.IRQ.Pin = NRF24_IRQ2_Pin;
    data_module.IRQ.Port= NRF24_IRQ2_GPIO_Port;
}


static void nrf24_ce_set(NRF_Module * radiomodule, GPIO_PinState state)
{
    assert_param(state == LOW || state == HIGH);
#if 1
    HAL_GPIO_WritePin(radiomodule->CE.Port, radiomodule->CE.Pin, state);
#else
    HAL_GPIO_WritePin(NRF24_CE1_GPIO_Port, NRF24_CE1_Pin, state);
#endif
    delay_us(10);
}

static void nrf24_csn_set(NRF_Module * radiomodule, GPIO_PinState state)
{
    assert_param(state == LOW || state == HIGH);
#if 1
    HAL_GPIO_WritePin(radiomodule->CSN.Port, radiomodule->CSN.Pin, state);
#else
    HAL_GPIO_WritePin(NRF24_CSN1_GPIO_Port, NRF24_CSN1_Pin, state);
#endif
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

void setup(NRF_Module * radiomodule)
{
    nrf24_setup_modules_gpio();

    // choose required module using SPI
    nrf24_csn_set(radiomodule, HIGH);
    // disable module (go to STANDBY-1 mode)
    nrf24_ce_set(radiomodule, LOW);

    HAL_Delay(5);

    //TODO FIXME NOTE: Check below
    se8r01_powerup(radiomodule);


    //set EN_AA, EN_RXADDR, RF_CH (needless?), RF_SETUP (needless?), AW, SETUP_RETR, TX_ADDR, RX_ADDR_P*
    radio_settings(radiomodule);

    // set CONFIG register according to rx/tx mode
    set_rx_tx_mode(radiomodule);

    se8r01_calibration(radiomodule);
    se8r01_setup(radiomodule);

    // set CONFIG register according to rx/tx mode
    set_rx_tx_mode(radiomodule);
}

void radio_update_settings(NRF_Module *radiomodule, RadioDevInfo * devinfo)
{
    // deselect module using SPI
    nrf24_csn_set(radiomodule, HIGH);
    // disable module (go to STANDBY-1 mode)
    nrf24_ce_set(radiomodule, LOW);


    //Transmit address
    nrf24_write_register_buf(radiomodule, iRF_BANK0_TX_ADDR, devinfo->address_usbdevice.byte_array, TX_ADR_WIDTH);

    //Receive address
    nrf24_write_register_buf(radiomodule, iRF_BANK0_RX_ADDR_P0, devinfo->address.byte_array, TX_ADR_WIDTH);

    //RF channel
    nrf24_write_register(radiomodule, iRF_BANK0_RF_CH, devinfo->radio_channel);

    //resume listening
    nrf24_ce_set(radiomodule, HIGH);

}

void nrf_receive_handler(NRF_Module * radiomodule)
{
    // volatile is used to prevent optimizing out
    volatile uint8_t status = SPI_Read(radiomodule, iRF_BANK0_STATUS);

    //if(nrf24_is_data_ready())
    if ( status & (1 << RX_DR) ){
        // read playload to rx_buf
        //SPI_Read_Buf(R_RX_PAYLOAD, rx_buf, TX_PLOAD_WIDTH);
        SPI_Read_Buf(radiomodule, R_RX_PAYLOAD, (uint8_t*)&rx_message, TX_PLOAD_WIDTH);
        // TODO: flushing breaks rx stuff
        //nrf24_write_register(FLUSH_RX,0);
        // clear RX_FIFO. TODO: verify
        status = SPI_Read(radiomodule, iRF_BANK0_STATUS);
        nrf_receive_callback();
    }
    else{

    }
    // Clear IRQ bits
    nrf24_write_register(radiomodule, iRF_BANK0_STATUS,status);
}

void RXX(NRF_Module * radiomodule)
{
    if( interrupt_happened(radiomodule)){
        delay_us(10);      //read reg too close after irq low not good
        //TODO: pause receiving on current radiomodule
        // ...

        //pause receiving. TODO: redundant in the case of polling?
        //set_power(radiomodule, PowerOFF);
        nrf24_ce_set(radiomodule, LOW);

        //process data
        nrf_receive_handler(radiomodule);

        //resume receiving. TODO: redundant in the case of polling?
        //set_power(radiomodule, PowerON);
        nrf24_ce_set(radiomodule, HIGH);

    }
}

void TXX(NRF_Module * radiomodule)
{
    //power on
    //Bit 7    | Bit 6      | Bit 5      | Bit 4       | Bit 3  | Bit 2 | Bit 1  | Bit 0   |
    //Reserved | MASK_RX_DR | MASK_TX_DS | MASK_MAX_RT | EN_CRC | CRCO  | PWR_UP | PRIM_RX |
    //By setting one of the MASK bits high, the corresponding IRQ source is disabled. By default all IRQ sources are enabled.
    //nrf24_write_register(radiomodule,iRF_BANK0_CONFIG, (1 << MASK_RX_DR) | (0 << MASK_TX_DS) | (0 << MASK_MAX_RT) | (1 << EN_CRC) | (1 << CRCO)  | (1 << PWR_UP) | (0 << PRIM_RX) );
    //delay_us(200);

    /// Important note: functions below are critical
    SPI_RW_Reg(radiomodule,iRF_CMD_FLUSH_TX,0);
    //SPI_Write_Buf(radiomodule,iRF_CMD_WR_TX_PLOAD,tx_buf,TX_PLOAD_WIDTH);
    SPI_Write_Buf(radiomodule,iRF_CMD_WR_TX_PLOAD,(uint8_t*)&tx_message,TX_PLOAD_WIDTH);

    //start transmission by toggling SE high for more than 10 us
    nrf24_ce_set(radiomodule, HIGH);

//============== TODO: investigate
    delay_us(10);
    //HAL_Delay(2);
#if 1
    int Delay = 100;
    uint32_t tickstart = HAL_GetTick();

    uint8_t packet_not_sent = ((!SPI_Read(radiomodule, iRF_BANK0_STATUS)) & (iSTATUS_TX_DS | iSTATUS_MAX_RT));
    uint8_t elapsed = HAL_GetTick() - tickstart;

    while( packet_not_sent && elapsed < Delay){
        packet_not_sent = ((!SPI_Read(radiomodule, iRF_BANK0_STATUS)) & (iSTATUS_TX_DS | iSTATUS_MAX_RT));
        elapsed = HAL_GetTick() - tickstart;
    }
#endif
    // stop transmission
    nrf24_ce_set(radiomodule, LOW);
    //==============

    TransmissionStatus tx_status = nrf24_last_messageStatus(radiomodule);
    GPIO_PinState irq = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0);
    uint8_t status = SPI_Read(radiomodule,iRF_BANK0_STATUS);

    //TODO: this is for debug
    if(tx_status == NRF24_TRANSMISSON_OK)
    {
        HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
        //HAL_GPIO_TogglePin(GPIO_LED_PORT,GPIO_LED_PIN);
    }
    else if(tx_status == NRF24_MESSAGE_LOST){
        //HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
        //HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13, GPIO_PIN_RESET);//nop
    }
    else{

    }

    nrf24_write_register(radiomodule, iRF_CMD_WRITE_REG + iRF_BANK0_STATUS,0xff);   // clear RX_DR or TX_DS or MAX_RT interrupt flag

}

static void radio_settings(NRF_Module * radiomodule)
{

    // Enable Auto Acknowledgment
    nrf24_write_register(radiomodule,iRF_BANK0_EN_AA, (1<<ENAA_P0)|(0<<ENAA_P1)|(0<<ENAA_P2)|(0<<ENAA_P3)|(0<<ENAA_P4)|(0<<ENAA_P5));

    // Enable RX addresses (pipes)
    nrf24_write_register(radiomodule,iRF_BANK0_EN_RXADDR,(1<<ERX_P0)|(0<<ERX_P1)|(0<<ERX_P2)|(0<<ERX_P3)|(0<<ERX_P4)|(0<<ERX_P5));


    // SETUP_AW number of bytes for address
    //4 byte adress, but use 5 byte address! TODO: research http://forum.easyelectronics.ru/viewtopic.php?f=9&t=21484
    // 11 5 bytes
    // 10 4 bytes
    // 01 Illegal
    // 00 Illegal
    uint8_t SETUP_AW_value = 0x02;
    if(TX_ADR_WIDTH == 5){
      //TODO:
        //SETUP_AW_value = 0x3;
    }
    //TODO
    nrf24_write_register(radiomodule,iRF_BANK0_SETUP_AW, SETUP_AW_value);

    // Auto retransmit delay and count (ARD, ARC)
    //lowest 4 bits 0-15 rt transmisston higest 4 bits 256-4096us Auto Retransmit
    //nrf24_write_register(radiomodule,iRF_BANK0_SETUP_RETR, 0xB00001010);
    nrf24_write_register(radiomodule,iRF_BANK0_SETUP_RETR, 0x1a);// 500us + 86us, 10 retrans...?

    // Set RF channel
    nrf24_write_register(radiomodule,iRF_BANK0_RF_CH, RF_CHANNEL);

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

    nrf24_write_register(radiomodule,iRF_BANK0_RF_SETUP, (0 << CONT_WAVE) | (1 << PA_PWR_3) | (0 << RF_DR_LO) | (1 << RF_DR_HIG) | (0x7 << PA_PWR) );
    //original comment: 2mps 0x4f, which is 1001111 TODO: 2mps is (RF_DR_LO,RF_DR_HIG) = (0,1) according to datasheet, and 0x4f stands for 1mps wtf?
    //0x47 1000111
    //nrf24_write_register(radiomodule,iRF_BANK0_RF_SETUP, 0x4f);

    // TODO: if setup is 0 or ff then there was no response from module
    uint8_t rf_setup = 0;
    nrf24_read_register_buf(radiomodule,iRF_BANK0_RF_SETUP, &rf_setup, 1);

#if 0
    //Dynamic length configurations:
    //pipe0 pipe1 enable dynamic payload length data
    nrf24_write_register(radiomodule,iRF_BANK0_DYNPD, 0x01);
    nrf24_write_register(DYNPD,(0<<DPL_P0)|(0<<DPL_P1)|(0<<DPL_P2)|(0<<DPL_P3)|(0<<DPL_P4)|(0<<DPL_P5));

    // enable dynamic paload lenght; enbale payload with ack enable w_tx_payload_noack
    //SPI_RW_Reg(radiomodule,iRF_CMD_WRITE_REG|iRF_BANK0_FEATURE, 0x07);
#endif

    //Set transmit address
    nrf24_write_register_buf(radiomodule,iRF_BANK0_TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);

    // Use the same address on the RX device as the TX device
    nrf24_write_register_buf(radiomodule,iRF_BANK0_RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);

    // Select same RX payload width as TX Payload width
    nrf24_write_register(radiomodule,iRF_BANK0_RX_PW_P0, TX_PLOAD_WIDTH);
    nrf24_write_register(radiomodule,iRF_BANK0_RX_PW_P1, 0x00);
    nrf24_write_register(radiomodule,iRF_BANK0_RX_PW_P2, 0x00);
    nrf24_write_register(radiomodule,iRF_BANK0_RX_PW_P3, 0x00);
    nrf24_write_register(radiomodule,iRF_BANK0_RX_PW_P4, 0x00);
    nrf24_write_register(radiomodule,iRF_BANK0_RX_PW_P5, 0x00);

}

static void set_rx_tx_mode(NRF_Module * radiomodule)
{
    if (mode=='r') {
        power_on_rx(radiomodule);
    }
    else {
        power_on_tx(radiomodule);
    }
}

static bool interrupt_happened(NRF_Module * radiomodule)
{
    return (HAL_GPIO_ReadPin(radiomodule->IRQ.Port, radiomodule->IRQ.Pin) == LOW);
}

static void se8r01_switch_bank(NRF_Module * radiomodule, uint8_t bankindex)
{
    uint8_t temp0,temp1;
    temp1 = bankindex;
    temp0 = SPI_RW(iRF_BANK0_STATUS);

    if((temp0&0x80)!=temp1)
    {
        SPI_RW_Reg(radiomodule,iRF_CMD_ACTIVATE,0x53);
    }

}

static void se8r01_powerup(NRF_Module * radiomodule)
{
    se8r01_switch_bank(radiomodule,iBANK0);

    ///CONFIG
    //Bit 7    | Bit 6      | Bit 5      | Bit 4       | Bit 3  | Bit 2 | Bit 1  | Bit 0   |
    //Reserved | MASK_RX_DR | MASK_TX_DS | MASK_MAX_RT | EN_CRC | CRCO  | PWR_UP | PRIM_RX |
    nrf24_write_register(radiomodule,iRF_BANK0_CONFIG, (0 << MASK_RX_DR) | (0 << MASK_TX_DS) | (0 << MASK_MAX_RT) | (1 << EN_CRC) | (0 << CRCO)  | (1 << PWR_UP) | (1 << PRIM_RX) );

    //Setup RF channel TODO: check necessity
    nrf24_write_register(radiomodule,iRF_BANK0_RF_CH, RF_CHANNEL);

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

    nrf24_write_register(radiomodule,iRF_BANK0_RF_SETUP, (0 << CONT_WAVE) | (1 << PA_PWR_3) | (0 << RF_DR_LO) | (1 << RF_DR_HIG) | (0x7 << PA_PWR) );

    // TODO: reveal the Magic
    nrf24_write_register(radiomodule,iRF_BANK0_PRE_GURD,0x77); //2450 calibration
}

static void se8r01_calibration(NRF_Module * radiomodule)
{
    //iBANK1
    se8r01_switch_bank(radiomodule,iBANK1);

    //iRF_BANK1_PLL_CTL0 <= [0]=0x40 [1]=0x00 [2]=0x10 [3]=0xE6
    gtemp[0]=0x40;
    gtemp[1]=0x00;
    gtemp[2]=0x10;
    gtemp[3]=0xE6;
    nrf24_write_register_buf(radiomodule,iRF_BANK1_PLL_CTL0, gtemp, 4);

    //iRF_BANK1_CAL_CTL <= [0]=0x20 [1]=0x08 [2]=0x50 [3]=0x40 [4]=0x50
    gtemp[0]=0x20;
    gtemp[1]=0x08;
    gtemp[2]=0x50;
    gtemp[3]=0x40;
    gtemp[4]=0x50;
    nrf24_write_register_buf(radiomodule,iRF_BANK1_CAL_CTL, gtemp, 5);

    //iRF_BANK1_IF_FREQ <= [0]=0x00 [1]=0x00 [2]=0x1E
    gtemp[0]=0x00;
    gtemp[1]=0x00;
    gtemp[2]=0x1E;
    nrf24_write_register_buf(radiomodule,iRF_BANK1_IF_FREQ, gtemp, 3);

    //iRF_BANK1_FDEV <= [0]=0x29
    gtemp[0]=0x29;
    nrf24_write_register_buf(radiomodule,iRF_BANK1_FDEV, gtemp, 1);

    //iRF_BANK1_DAC_CAL_LOW <= [0]=0x00
    gtemp[0]=0x00;
    nrf24_write_register_buf(radiomodule,iRF_BANK1_DAC_CAL_LOW, gtemp, 1);

    //iRF_BANK1_DAC_CAL_HI <= [0]=0x7F
    gtemp[0]=0x7F;
    nrf24_write_register_buf(radiomodule,iRF_BANK1_DAC_CAL_HI, gtemp, 1);

    //iRF_BANK1_AGC_GAIN <= [0]=0x02 [1]=0xC1 [2]=0xEB [3]=0x1C
    gtemp[0]=0x02;
    gtemp[1]=0xC1;
    gtemp[2]=0xEB;
    gtemp[3]=0x1C;
    nrf24_write_register_buf(radiomodule,iRF_BANK1_AGC_GAIN, gtemp, 4);

    //iRF_BANK1_RF_IVGEN <= [0]=0x97 [1]=0x64 [2]=0x00 [3]=0x81
    gtemp[0]=0x97;
    gtemp[1]=0x64;
    gtemp[2]=0x00;
    gtemp[3]=0x81;
    nrf24_write_register_buf(radiomodule,iRF_BANK1_RF_IVGEN, gtemp, 4);

//    iBANK0
//    CE 1
//    Delay 30 us
//    CE 0
//    Delay 15 ms
//    CE 1
//    Delay 30 us
//    CE 0
//    Delay 15 ms
    se8r01_switch_bank(radiomodule,iBANK0);
    nrf24_ce_set(radiomodule, HIGH);
    delay_us(30);
    nrf24_ce_set(radiomodule, LOW);
    HAL_Delay(15);
    nrf24_ce_set(radiomodule, HIGH);
    delay_us(30);
    nrf24_ce_set(radiomodule, LOW);
    HAL_Delay(15);

}

static void se8r01_setup(NRF_Module * radiomodule)
{

    //    iRF_BANK0_SETUP_VALUE <= [0]=0x28 [1]=0x32 [2]=0x80 [3]=0x90 [4]=0x00
    gtemp[0]=0x28;
    gtemp[1]=0x32;
    gtemp[2]=0x80;
    gtemp[3]=0x90;
    gtemp[4]=0x00;
    nrf24_write_register_buf(radiomodule,iRF_BANK0_SETUP_VALUE, gtemp, 5);

    //    Delay 2 us
    delay_us(2);

    //    iBANK1
    se8r01_switch_bank(radiomodule,iBANK1);

    //    iRF_BANK1_PLL_CTL0 <= [0]=0x40 [1]=0x01 [2]=0x30 [3]=0xE2
    gtemp[0]=0x40;
    gtemp[1]=0x01;
    gtemp[2]=0x30;
    gtemp[3]=0xE2;
    nrf24_write_register_buf(radiomodule,iRF_BANK1_PLL_CTL0, gtemp, 4);

    //    iRF_BANK1_CAL_CTL <= [0]=0x29 [1]=0x89 [2]=0x55 [3]=0x40 [4]=0x50
    gtemp[0]=0x29;
    gtemp[1]=0x89;
    gtemp[2]=0x55;
    gtemp[3]=0x40;
    gtemp[4]=0x50;
    nrf24_write_register_buf(radiomodule,iRF_BANK1_CAL_CTL, gtemp, 5);

    //    iRF_BANK1_FDEV <= [0]=0x29
    gtemp[0]=0x29;
    nrf24_write_register_buf(radiomodule,iRF_BANK1_FDEV, gtemp, 1);

    //    iRF_BANK1_RX_CTRL <= [0]=0x55 [1]=0xC2 [2]=0x09 [3]=0xAC
    gtemp[0]=0x55;
    gtemp[1]=0xC2;
    gtemp[2]=0x09;
    gtemp[3]=0xAC;
    nrf24_write_register_buf(radiomodule,iRF_BANK1_RX_CTRL, gtemp, 4);

    //    iRF_BANK1_FAGC_CTRL_1 <= [0]=0x00 [1]=0x14 [2]=0x08 [3]=0x29
    gtemp[0]=0x00;
    gtemp[1]=0x14;
    gtemp[2]=0x08;
    gtemp[3]=0x29;
    nrf24_write_register_buf(radiomodule,iRF_BANK1_FAGC_CTRL_1, gtemp, 4);

    //    iRF_BANK1_AGC_GAIN <= [0]=0x02 [1]=0xC1 [2]=0xCB [3]=0x1C
    gtemp[0]=0x02;
    gtemp[1]=0xC1;
    gtemp[2]=0xCB;
    gtemp[3]=0x1C;
    nrf24_write_register_buf(radiomodule,iRF_BANK1_AGC_GAIN, gtemp, 4);

    //    iRF_BANK1_RF_IVGEN <= [0]=0x97 [1]=0x64 [2]=0x00 [3]=0x01
    gtemp[0]=0x97;
    gtemp[1]=0x64;
    gtemp[2]=0x00;
    gtemp[3]=0x01;
    nrf24_write_register_buf(radiomodule,iRF_BANK1_RF_IVGEN, gtemp, 4);

    //    iRF_BANK1_TEST_PKDET <= [0]=0x2A [1]=0x04 [2]=0x00 [3]=0x7D
    gtemp[0]=0x2A;
    gtemp[1]=0x04;
    gtemp[2]=0x00;
    gtemp[3]=0x7D;
    nrf24_write_register_buf(radiomodule,iRF_BANK1_TEST_PKDET, gtemp, 4);

    //    iBANK0
    se8r01_switch_bank(radiomodule,iBANK0);

    //    Delay 2 us
    delay_us(2);
}

static void set_power(NRF_Module * radiomodule, NRF_PowerState power_state)
{
    // TODO FIXME: check if it works
    uint8_t config_reg = SPI_Read(radiomodule,CONFIG);
    if(power_state == PowerOFF){
        config_reg &= (~(1 << PWR_UP));//(~(1 << PWR_UP)) -  PWR_UP bit to 0 and all other bits to 1
    }
    else{
        config_reg |= (1 << PWR_UP);//(1 << PWR_UP) -  PWR_UP bit to 1 and all other bits to 0
    }
    SPI_RW_Reg(radiomodule, W_REGISTER + CONFIG, config_reg);
    delay_us(20);
}

static void power_on_tx(NRF_Module * radiomodule)
{
    nrf24_ce_set(radiomodule, LOW);
    //CONFIG
    //Bit 7    | Bit 6      | Bit 5      | Bit 4       | Bit 3  | Bit 2 | Bit 1  | Bit 0   |
    //Reserved | MASK_RX_DR | MASK_TX_DS | MASK_MAX_RT | EN_CRC | CRCO  | PWR_UP | PRIM_RX |
    //By setting one of the MASK bits high, the corresponding IRQ source is disabled. By default all IRQ sources are enabled.
    //TODO refactoring//
    nrf24_write_register(radiomodule,iRF_BANK0_CONFIG, (1 << MASK_RX_DR) | (0 << MASK_TX_DS) | (0 << MASK_MAX_RT) | (1 << EN_CRC) | (0 << CRCO)  | (1 << PWR_UP) | (0 << PRIM_RX) );
    delay_us(10);
}

static void power_on_rx(NRF_Module * radiomodule)
{
    //turn on irq for receiver; turn off irq for transmitter
    //By setting one of the MASK bits high, the corresponding IRQ source is disabled. By default all IRQ sources are enabled.
    //TODO refactoring//

    nrf24_ce_set(radiomodule, LOW);

    //CONFIG
    //Bit 7    | Bit 6      | Bit 5      | Bit 4       | Bit 3  | Bit 2 | Bit 1  | Bit 0   |
    //Reserved | MASK_RX_DR | MASK_TX_DS | MASK_MAX_RT | EN_CRC | CRCO  | PWR_UP | PRIM_RX |
    nrf24_write_register(radiomodule,iRF_BANK0_CONFIG, (0 << MASK_RX_DR) | (1 << MASK_TX_DS) | (1 << MASK_MAX_RT) | (1 << EN_CRC) | (0 << CRCO)  | (1 << PWR_UP) | (1 << PRIM_RX) );
    // start listening
    delay_us(10);
    nrf24_ce_set(radiomodule, HIGH);
    delay_us(210);
}

// Clocks only one byte into the given nrf24 register //
static void nrf24_write_register(NRF_Module * radiomodule, uint8_t reg, uint8_t value)
{
    delay_us(10);
    nrf24_csn_set(radiomodule, LOW);
    SPI_RW(iRF_CMD_WRITE_REG | (REGISTER_MASK & reg));
    SPI_RW(value);
    nrf24_csn_set(radiomodule, HIGH);
}

// Read single register from nrf24 //
static void nrf24_read_register_buf(NRF_Module * radiomodule, uint8_t reg, uint8_t* value, uint8_t len)
{
    nrf24_csn_set(radiomodule, LOW);
    SPI_Read_Buf(radiomodule,iRF_CMD_READ_REG | (REGISTER_MASK & reg), value, len);
    nrf24_csn_set(radiomodule, HIGH);
}

// Write to a single register of nrf24 //
static void nrf24_write_register_buf(NRF_Module * radiomodule, uint8_t reg, uint8_t* value, uint8_t len)
{
    nrf24_csn_set(radiomodule, LOW);
    SPI_Write_Buf(radiomodule,iRF_CMD_WRITE_REG | (REGISTER_MASK & reg), value, len);
    nrf24_csn_set(radiomodule, HIGH);
}

static uint8_t SPI_RW(uint8_t tx)
{
    uint8_t rx = 0;
    HAL_SPI_TransmitReceive_IT(&hspi1, &tx, &rx, 1);
    return rx;
}

static uint8_t SPI_RW_Reg(NRF_Module * radiomodule, uint8_t reg, uint8_t value)
{
    uint8_t status;
    nrf24_csn_set(radiomodule, LOW);                   // CSN low, init SPI transaction
    status = SPI_RW(reg);                   // select register
    SPI_RW(value);                          // ..and write value to it..
    nrf24_csn_set(radiomodule, HIGH);                   // CSN high again
    return(status);                   // return nRF24L01 status uint8_t
}

uint8_t SPI_Read(NRF_Module * radiomodule, uint8_t reg)
{
    uint8_t reg_val;

    nrf24_csn_set(radiomodule, LOW);           // CSN low, initialize SPI communication...
    SPI_RW(reg);                   // Select register to read from..
    reg_val = SPI_RW(0);           // ..then read register value
    nrf24_csn_set(radiomodule, HIGH);          // CSN high, terminate SPI communication

    return(reg_val);               // return register value
}

static uint8_t SPI_Read_Buf(NRF_Module * radiomodule, uint8_t reg, uint8_t *pBuf, uint8_t bytes)
{
    uint8_t status,i;

    nrf24_csn_set(radiomodule, LOW);                  // Set CSN low, init SPI tranaction
    status = SPI_RW(reg);       	    // Select register to write to and read status uint8_t

    for(i=0;i<bytes;i++)
    {
        pBuf[i] = SPI_RW(0);    // Perform SPI_RW to read uint8_t from nRF24L01
    }

    nrf24_csn_set(radiomodule, HIGH);                   // Set CSN high again

    return(status);                  // return nRF24L01 status uint8_t
}

static uint8_t SPI_Write_Buf(NRF_Module * radiomodule, uint8_t reg, uint8_t *pBuf, uint8_t bytes)
{
    uint8_t status,i;

    nrf24_csn_set(radiomodule, LOW);                  // Set CSN low, init SPI tranaction
    status = SPI_RW(reg);             // Select register to write to and read status uint8_t
    for(i=0;i<bytes; i++)             // then write all uint8_t in buffer(*pBuf)
    {
        SPI_RW(*pBuf++);
    }
    nrf24_csn_set(radiomodule, HIGH);                   // Set CSN high again
    return(status);                  // return nRF24L01 status uint8_t
}


void nrf_without_this_interrupts_not_work(NRF_Module * radiomodule)
{
    SPI_RW_Reg(radiomodule, iRF_CMD_WRITE_REG+iRF_BANK0_STATUS,0xff);
}

static uint8_t nrf_getStatus(NRF_Module * radiomodule)
{
    return SPI_Read(radiomodule, iRF_BANK0_STATUS);
}
