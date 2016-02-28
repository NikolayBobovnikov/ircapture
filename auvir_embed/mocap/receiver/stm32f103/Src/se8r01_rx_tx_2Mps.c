//this is a copy and paste job made by F2k

#include "stm32f1xx_hal.h"
#include "nRF24L01P.h"
#include "se8r01.h"

extern SPI_HandleTypeDef hspi1;
extern char mode;      //r=rx, t=tx

uint8_t gtemp[5];
uint8_t k=0;
//***************************************************
#define TX_ADR_WIDTH    4   // 5 uint8_ts TX(RX) address width
#define TX_PLOAD_WIDTH  6  // 32 uint8_ts TX payload

uint8_t TX_ADDRESS[TX_ADR_WIDTH]  =
{
    0x34,0x43,0x10,0x10
}; // Define a static TX address

uint8_t rx_buf[TX_PLOAD_WIDTH] = {0}; // initialize value
uint8_t tx_buf[TX_PLOAD_WIDTH] = {0};
//***************************************************
void setup()
{

    init_io();                        // Initialize IO port
    uint8_t status=SPI_Read(STATUS);

    nrf24_ce_set(LOW);
    HAL_Delay(1);
    se8r01_powerup();
    se8r01_calibration();
    se8r01_setup();
    radio_settings();

    if (mode=='r') {
        SPI_RW_Reg(W_REGISTER|iRF_BANK0_CONFIG, 0x3f);
    }
    else {
        SPI_RW_Reg(W_REGISTER|iRF_BANK0_CONFIG, 0x3E);
    }

    nrf24_ce_set(HIGH);
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
        HAL_Delay(1);      //read reg too close after irq low not good
        uint8_t status = SPI_Read(STATUS);

        if(status & STA_MARK_RX)                                // if receive data ready (TX_DS) interrupt
        {
            SPI_Read_Buf(R_RX_PAYLOAD, rx_buf, TX_PLOAD_WIDTH);    // read playload to rx_buf
            SPI_RW_Reg(FLUSH_RX,0); // clear RX_FIFO
            for(uint8_t i=0; i<TX_PLOAD_WIDTH; i++)
            {
                int a = 0;
                int b = a;
            }
            SPI_RW_Reg(W_REGISTER+STATUS,0xff);
        }
        else{

            SPI_RW_Reg(W_REGISTER+STATUS,0xff);

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


    uint8_t status = SPI_Read(STATUS);

    SPI_RW_Reg(FLUSH_TX,0);
    SPI_Write_Buf(W_TX_PAYLOAD,tx_buf,TX_PLOAD_WIDTH);

    SPI_RW_Reg(W_REGISTER + STATUS,0xff);   // clear RX_DR or TX_DS or MAX_RT interrupt flag

    HAL_Delay(500);

}

void radio_settings()
{

    SPI_RW_Reg(W_REGISTER|iRF_BANK0_EN_AA, 0x01);          //enable auto acc on pip 1
    SPI_RW_Reg(W_REGISTER|iRF_BANK0_EN_RXADDR, 0x01);      //enable pip 1
    SPI_RW_Reg(W_REGISTER|iRF_BANK0_SETUP_AW, 0x02);        //4 byte adress

    SPI_RW_Reg(W_REGISTER|iRF_BANK0_SETUP_RETR, 0xB00001010);        //lowest 4 bits 0-15 rt transmisston higest 4 bits 256-4096us Auto Retransmit HAL_Delay
    SPI_RW_Reg(W_REGISTER|iRF_BANK0_RF_CH, 40);
    SPI_RW_Reg(W_REGISTER|iRF_BANK0_RF_SETUP, 0x4f);        //2mps 0x4f
    //SPI_RW_Reg(W_REGISTER|iRF_BANK0_DYNPD, 0x01);          //pipe0 pipe1 enable dynamic payload length data
    //SPI_RW_Reg(W_REGISTER|iRF_BANK0_FEATURE, 0x07);        // enable dynamic paload lenght; enbale payload with ack enable w_tx_payload_noack

    SPI_Write_Buf(W_REGISTER + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);  //from tx
    SPI_Write_Buf(W_REGISTER + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH); // Use the same address on the RX device as the TX device
    SPI_RW_Reg(W_REGISTER + RX_PW_P0, TX_PLOAD_WIDTH); // Select same RX payload width as TX Payload width

}

void init_io(void)
{
    //digitalWrite(IRQq, 0);
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
    SPI_RW_Reg(iRF_CMD_WRITE_REG|iRF_BANK0_CONFIG,0x03);
    SPI_RW_Reg(iRF_CMD_WRITE_REG|iRF_BANK0_RF_CH,0x32);
    SPI_RW_Reg(iRF_CMD_WRITE_REG|iRF_BANK0_RF_SETUP,0x48);
    SPI_RW_Reg(iRF_CMD_WRITE_REG|iRF_BANK0_PRE_GURD,0x77); //2450 calibration


}

void se8r01_calibration()
{


    se8r01_switch_bank(iBANK1);

    gtemp[0]=0x40;
    gtemp[1]=0x00;
    gtemp[2]=0x10;
    gtemp[3]=0xE6;
    SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK1_PLL_CTL0, gtemp, 4);

    gtemp[0]=0x20;
    gtemp[1]=0x08;
    gtemp[2]=0x50;
    gtemp[3]=0x40;
    gtemp[4]=0x50;
    SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK1_CAL_CTL, gtemp, 5);

    gtemp[0]=0x00;
    gtemp[1]=0x00;
    gtemp[2]=0x1E;
    SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK1_IF_FREQ, gtemp, 3);

    gtemp[0]=0x29;
    SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK1_FDEV, gtemp, 1);

    gtemp[0]=0x00;
    SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK1_DAC_CAL_LOW, gtemp, 1);

    gtemp[0]=0x7F;
    SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK1_DAC_CAL_HI, gtemp, 1);

    gtemp[0]=0x02;
    gtemp[1]=0xC1;
    gtemp[2]=0xEB;
    gtemp[3]=0x1C;
    SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK1_AGC_GAIN, gtemp, 4);

    gtemp[0]=0x97;
    gtemp[1]=0x64;
    gtemp[2]=0x00;
    gtemp[3]=0x81;
    SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK1_RF_IVGEN, gtemp, 4);

    se8r01_switch_bank(iBANK0);

    nrf24_ce_set(HIGH);
    HAL_Delay(1);
    nrf24_ce_set(LOW);
    //digitalWrite(CEq, 1);
    //delayMicroseconds(30);
    //digitalWrite(CEq, 0);

    HAL_Delay(1);
    //delayMicroseconds(50);                            // delay 50ms waitting for calibaration.

    nrf24_ce_set(HIGH);
    HAL_Delay(1);
    nrf24_ce_set(LOW);
    //digitalWrite(CEq, 1);
    //delayMicroseconds(30);
    //digitalWrite(CEq, 0);

}

void se8r01_setup()
{
    gtemp[0]=0x28;
    gtemp[1]=0x32;
    gtemp[2]=0x80;
    gtemp[3]=0x90;
    gtemp[4]=0x00;
    SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK0_SETUP_VALUE, gtemp, 5);

    HAL_Delay(1);//delayMicroseconds(2);

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




