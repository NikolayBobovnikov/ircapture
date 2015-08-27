#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include <stdio.h>


void initLED()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	GPIO_InitTypeDef gpio;

	gpio.GPIO_Pin = GPIO_Pin_8;
	gpio.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOC, &gpio);

	gpio.GPIO_Pin = GPIO_Pin_9;
	gpio.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOC, &gpio);
}

void delay(int num)
{
	unsigned int i;
	for (i = 0; i < num; i++)
	{
		;
	}
}

void print()
{
	setvbuf(stdin, NULL, _IONBF, 0);
	setvbuf(stdout, NULL, _IONBF, 0);
	setvbuf(stderr, NULL, _IONBF, 0);
	printf("Hello, World!!!");
}


int run_led_example(void)
{
	initLED();

	while(1)
    {
		GPIO_WriteBit(GPIOC, GPIO_Pin_8, Bit_SET);
	    GPIO_WriteBit(GPIOC, GPIO_Pin_9, Bit_RESET);
	    delay(1000000);
	    GPIO_WriteBit(GPIOC, GPIO_Pin_8, Bit_RESET);
	    GPIO_WriteBit(GPIOC, GPIO_Pin_9, Bit_SET);
	    delay(1000000);

	    print();
    }
}


//Структуры для инициализации GPIOA и USART1
GPIO_InitTypeDef    GPIO_InitStruct;
USART_InitTypeDef    USART_InitStruct;

void Init(void); //Объявление функции инициализации периферии
void Usart1_Send_symbol(uint8_t); //Объявление функции передачи символа
void Usart1_Send_String(char* str); //Объявление функции передачи строки
char* str;
uint8_t data;
//Функция инициализации периферии
void Init()
{
  //Включаем тактирование GPIOA, USART1 и альтернативных функций AFIO
  RCC_APB2PeriphClockCmd((RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO), ENABLE);

  //Инициализации вывода PA9 - USART1_Tx
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9; //Настройки вывода PA9
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; //Скорость порта максимальная
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP; //Режим альтернативной функции, выход Push-Pull
  GPIO_Init(GPIOA, &GPIO_InitStruct); //Заданные настройки сохраняем в регистрах GPIOА

  //Инициализации вывода PA10 - USART1_Rx
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10; //Настройки вывода PA10
  GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IN_FLOATING; //Input floating
  GPIO_Init(GPIOA, &GPIO_InitStruct); //Заданные настройки сохраняем в регистрах GPIOА


  //Инициализация USART1
  USART_InitStruct.USART_BaudRate = 9600; //Скорость обмена 9600 бод
  USART_InitStruct.USART_WordLength = USART_WordLength_8b; //Длина слова 8 бит
  USART_InitStruct.USART_StopBits = USART_StopBits_1; //1 стоп-бит
  USART_InitStruct.USART_Parity = USART_Parity_No ; //Без проверки четности
  USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //Без аппаратного контроля
  USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //Включен передатчик и приемник USART1
  USART_Init(USART1, &USART_InitStruct); //Заданные настройки сохраняем в регистрах USART1

  USART_Cmd(USART1, ENABLE); //Включаем USART1
}

//Функция передачи символа
void Usart1_Send_symbol(uint8_t data)
{
  while(!(USART1->SR & USART_SR_TC)); //Проверяем установку флага TC - завершения предыдущей передачи
  USART1->DR = data; //Записываем значение в регистр данных - передаем символ
}

//Функция передачи строки через USART
void Usart1_Send_String(char* str)
{
  uint8_t i=0;
  while(str[i])
  {
    Usart1_Send_symbol(str[i]);
    i++;
  }
  Usart1_Send_symbol('n');
  Usart1_Send_symbol('r');
}

int run_usart_example()
{
  Init(); //Вызов функции инициализации периферии
  //Передаем строку, сообщающую о готовности микроконтроллера к обмену данными
  Usart1_Send_String("I'm ready");
  while(1)
  {
    if((USART1->SR & USART_SR_RXNE)) //Проверяем поступление данных от компьютера
    {
      data = USART1->DR; //Считываем принятые данные
      Usart1_Send_symbol(data); //И тут же отсылаем их обратно
    }
  }
}

int main(void)
{
	return run_usart_example();
}


