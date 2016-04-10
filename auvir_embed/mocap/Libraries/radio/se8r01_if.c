#include "se8r01.h"
#include "se8r01_if.h"

#define NRF24_CSN1_Pin GPIO_PIN_0
#define NRF24_CSN1_GPIO_Port GPIOA
#define NRF24_CE1_Pin GPIO_PIN_1
#define NRF24_CE1_GPIO_Port GPIOA
#define NRF24_CSN2_Pin GPIO_PIN_2
#define NRF24_CSN2_GPIO_Port GPIOA
#define NRF24_CE2_Pin GPIO_PIN_3
#define NRF24_CE2_GPIO_Port GPIOA
#define NRF24_IRQ1_Pin GPIO_PIN_0
#define NRF24_IRQ1_GPIO_Port GPIOB
#define NRF24_IRQ2_Pin GPIO_PIN_1
#define NRF24_IRQ2_GPIO_Port GPIOB


// Use two separate radiomodules for usb device
NRF_Module default_module = {0};
NRF_Module data_module = {0};


void nrf24_setup_modules_gpio()
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

