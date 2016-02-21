#include "stm32f1xx_hal.h"
#include "nRF24L01P.h"

//define the private constants in this library
//#define TX_ADR_WIDTH 5
//#define RX_ADR_WIDTH 5
extern SPI_HandleTypeDef hspi1;

unsigned char TxBuf[Buffer_Size] = {0};
unsigned char RxBuf[Buffer_Size] = {0};

unsigned char nRF24L01_Freq = 0;
unsigned char nRF24L01_power_rate = 0;

//define the initial Address
unsigned char  TX_ADDRESS[ADR_WIDTH]= {0xE7,0xE7,0xE7,0xE7,0xE7};
unsigned char  RX_ADDRESS[ADR_WIDTH]= {0xE7,0xE7,0xE7,0xE7,0xE7};

//Define the layer1:HW operation
unsigned char nRF24L01_SPI_Send_Byte(unsigned char dat);
void nRF24L01_SPI_NSS_L(void);
void nRF24L01_SPI_NSS_H(void);

//Define the layer2:Reg operation
unsigned char SPI_WR_Reg(unsigned char reg, unsigned char value);
unsigned char SPI_Read_Buf(unsigned char reg, unsigned char *pBuf, unsigned char Len);
unsigned char SPI_Write_Buf(unsigned char reg, unsigned char *pBuf, unsigned char Len);
unsigned char SPI_RD_Reg(unsigned char reg);

//Define the layer3:application operation
/****************************************

All the functions is in "nRF24l01P.h"

****************************************/

//Define the other function
void nRF24L01_Delay_us(unsigned long n);

//Define the layer3 functions

void nRF24L01_Set_TX_Address(	unsigned char A,
                                unsigned char B,
                                unsigned char C,
                                unsigned char D,
                                unsigned char E)
{
    TX_ADDRESS[0] = A;
    TX_ADDRESS[1] = B;
    TX_ADDRESS[2] = C;
    TX_ADDRESS[3] = D;
    TX_ADDRESS[4] = E;
}
void nRF24L01_Set_RX_Address(	unsigned char A,
                                unsigned char B,
                                unsigned char C,
                                unsigned char D,
                                unsigned char E)
{
    RX_ADDRESS[0] = A;
    RX_ADDRESS[1] = B;
    RX_ADDRESS[2] = C;
    RX_ADDRESS[3] = D;
    RX_ADDRESS[4] = E;
}

unsigned char nRF24L01_Config(unsigned char freq, unsigned char power, unsigned char Rate)
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

void RX_Mode(void)
{unsigned char buf[5]={0};


    SPI_Read_Buf(TX_ADDR, buf, ADR_WIDTH);

    SPI_Write_Buf(WRITE_nRF_REG + RX_ADDR_P0, RX_ADDRESS, ADR_WIDTH);

    SPI_WR_Reg(WRITE_nRF_REG + EN_AA, 0);
    SPI_WR_Reg(WRITE_nRF_REG + EN_RXADDR, 0x01);
    SPI_WR_Reg(WRITE_nRF_REG + SETUP_RETR, 0x1a);
    SPI_WR_Reg(WRITE_nRF_REG + RF_CH,nRF24L01_Freq);
    SPI_WR_Reg(WRITE_nRF_REG + RX_PW_P0, RX_PLOAD_WIDTH);
    SPI_WR_Reg(WRITE_nRF_REG + RF_SETUP, nRF24L01_power_rate);

    SPI_WR_Reg(WRITE_nRF_REG + CONFIG, 0x03);

    nRF24L01_Delay_us(200);
}

void TX_Mode(void)
{


    SPI_Write_Buf(WRITE_nRF_REG + TX_ADDR, TX_ADDRESS, ADR_WIDTH);
    SPI_Write_Buf(WRITE_nRF_REG + RX_ADDR_P0, RX_ADDRESS, ADR_WIDTH);

    SPI_WR_Reg(WRITE_nRF_REG + EN_AA, 0);
    SPI_WR_Reg(WRITE_nRF_REG + EN_RXADDR, 0x01);
    SPI_WR_Reg(WRITE_nRF_REG + SETUP_RETR, 0x1a);
    SPI_WR_Reg(WRITE_nRF_REG + RF_CH,nRF24L01_Freq);
    SPI_WR_Reg(WRITE_nRF_REG + RF_SETUP,  nRF24L01_power_rate);
    SPI_WR_Reg(WRITE_nRF_REG + CONFIG, 0x02);

}

void nRF24L01_TxPacket(unsigned char * tx_buf)
{

    SPI_Write_Buf(WRITE_nRF_REG + RX_ADDR_P0, TX_ADDRESS, ADR_WIDTH);
    SPI_Write_Buf(WR_TX_PLOAD, tx_buf, TX_PLOAD_WIDTH);


}

unsigned char nRF24L01_RxPacket(unsigned char* rx_buf)
{unsigned char flag=0;
 unsigned char status;

    status=SPI_RD_Reg(NRFRegSTATUS);

    if(status & 0x40)
    {

         SPI_Read_Buf(RD_RX_PLOAD,rx_buf,TX_PLOAD_WIDTH);
         flag =1;
    }
    SPI_WR_Reg(WRITE_nRF_REG+NRFRegSTATUS, status);
    return flag;
}

//Define the layer2 functions
unsigned char SPI_RD_Reg(unsigned char reg)
{
    unsigned char reg_val;

    nRF24L01_SPI_NSS_L();                // CSN low, initialize SPI communication...
    nRF24L01_SPI_Send_Byte(reg);            // Select register to read from..
    reg_val = nRF24L01_SPI_Send_Byte(0);    // ..then read register value
    nRF24L01_SPI_NSS_H();                // CSN high, terminate SPI communication

    return(reg_val);        // return register value
}

unsigned char SPI_WR_Reg(unsigned char reg, unsigned char value)
{
    unsigned char status;

    nRF24L01_SPI_NSS_L();                  // CSN low, init SPI transaction
    status = nRF24L01_SPI_Send_Byte(reg);// select register
    nRF24L01_SPI_Send_Byte(value);             // ..and write value to it..
    nRF24L01_SPI_NSS_H();                   // CSN high again

    return(status);            // return nRF24L01 status unsigned char
}

unsigned char SPI_Read_Buf(unsigned char reg, unsigned char *pBuf, unsigned char Len)
{
    unsigned int status,i;

    nRF24L01_SPI_NSS_L();                    		// Set CSN low, init SPI tranaction
    status = nRF24L01_SPI_Send_Byte(reg);       		// Select register to write to and read status unsigned char

  for(i=0;i<Len;i++)
  {
     pBuf[i] = nRF24L01_SPI_Send_Byte(0);
  }

    nRF24L01_SPI_NSS_H();

    return(status);                    // return nRF24L01 status unsigned char
}

unsigned char SPI_Write_Buf(unsigned char reg, unsigned char *pBuf, unsigned char Len)
{
    unsigned int status,i;

    nRF24L01_SPI_NSS_L();
    status = nRF24L01_SPI_Send_Byte(reg);
    for(i=0; i<Len; i++) //
    {
        nRF24L01_SPI_Send_Byte(*pBuf);
         pBuf ++;
    }
    nRF24L01_SPI_NSS_H();
    return(status);
}



//Define the layer1 functions
unsigned char nRF24L01_SPI_Send_Byte(unsigned char dat)
{
    /*
  // Loop while DR register in not emplty
  while(__HAL_SPI_GET_FLAG(SPI1, SPI_FLAG_TXE) == RESET);


  // Send byte through the SPI2 peripheral
  HAL_SPI_Transmit_IT(hspi1, dat, 1);

  // Wait to receive a byte
  while(__HAL_SPI_GET_FLAG(SPI1, SPI_FLAG_RXNE) == RESET);

  // Return the byte read from the SPI bus
  return SPI_I2S_ReceiveData(SPI2);
  */

  HAL_StatusTypeDef result = HAL_SPI_Transmit_IT(&hspi1, &dat, 1);
  return result;
}

void nRF24L01_SPI_NSS_H(void)
{
    HAL_GPIO_WritePin(NRF24L01_CSN_PORT,NRF24L01_CSN_PIN, GPIO_PIN_SET);
}

void nRF24L01_SPI_NSS_L(void)
{
    HAL_GPIO_WritePin(NRF24L01_CSN_PORT,NRF24L01_CSN_PIN, GPIO_PIN_RESET);
}

void nRF24L01_Delay_us(unsigned long n)
{
    unsigned long i;

    while(n--)
    {
       i=2;
       while(i--);
  }
}
