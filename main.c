#define F_CPU 72000000UL
#include "stm32f10x.h"
//#include "stdlib.h"

#define DELAY_SV 1000000
#define Baud_rate_9600_72 0x1D4C
#define DELAY_1us 9						//подобрано экспериментально

void test_sv(int);
//void Button_PB0 (void*);
void UART_Initial(void);
//void UART_Transmit(char*);
void USART1_Send_Char(char);
void USART1_Send_String(char*);
void GPIO_initial(void);
void SPI_initial(void);
uint8_t SPI_Read_Write_Function(uint8_t);

void s_delay_us(int);					//на основе цикла
void s_delay_ms(long);


int main(void)
{
	GPIO_initial();		//порты ввода-вывода общего назначения
	UART_Initial();
	SPI_initial();
	
	s_delay_ms(1000);
	
	USART1_Send_String("Hello world! \n");
	
	//отправляем команду, включающую 4 пакета. По мере отправки устройство отвечает.
	//int tmp=0;
	//char num_to_str[5];
	//tmp=(int)SPI_Read_Write_Function((uint8_t)0x0F);
	//itoa(tmp, num_to_str, 16);
	//USART1_Send_Char((char)SPI_Read_Write_Function((uint8_t)0x0F));
	while(1)
	{
			
		//костыли!!!!!
		GPIOA->CRL&=~(GPIO_CRL_CNF2);  //сброс 2 пина                
		GPIOA->CRL|= GPIO_CRL_CNF2_1;	//2output push-pull
		//без этого не читает несколько раз сподряд
		
		
		GPIOA->BSRR=GPIO_BSRR_BR2;		//chip-select
		s_delay_us(1);	//ждём >21 ns
		
		char ch_ar[5];
		uint8_t Frame[]={0x0F, 0x00, 0x19, 0x00};
		for(int i=0; i<4; i++)
		{
			ch_ar[i]=(char)SPI_Read_Write_Function(Frame[i]);
		}
		s_delay_us(1);	//ждём >21 ns
		//SPI1->CR1 &= ~SPI_CR1_SPE;
		GPIOA->BSRR=GPIO_BSRR_BS2;		//chip-select off
		
		for(int i=0; i<4; i++)
		{
			USART1_Send_Char(ch_ar[i]);
			//USART1_Send_Char(' ');
			//USART1_Send_Char('\n');
		}
		
		
		s_delay_ms(1000);
	}
	
	
	while(1)
	{
		//USART1_Send_String("Test1!");
		//SPI_Read_Write_Function(0xFF);
		//test_sv(3);
		//s_delay_ms(1000);
	};
}

void GPIO_initial(void)
{
	RCC->APB2ENR	|= RCC_APB2ENR_IOPAEN;			//тактируем PA
	//настраиваем PA0 на вывод - светодиод
	GPIOA->CRL&=~GPIO_CRL_CNF0;				//сбрасываем настройки пина
	GPIOA->CRL&=~GPIO_CRL_MODE0;			//сбрасываем настройки пина
	
	GPIOA->CRL=GPIO_CRL_MODE0_1;				//Режим MODE0_1 (max 2MHz)
																			//CNF==0b00 - general purpose output push-pull
	/*00: Input mode (reset state)
		01: Output mode, max speed 10 MHz.
		10: Output mode, max speed 2 MHz.
		11: Output mode, max speed 50 MHz
	*/
	
	
}
void test_sv(int counter)				//светодиод на линии PA0
{
	for(int l=0; l<counter; l++)
	{
		GPIOA->BSRR=GPIO_BSRR_BS0;
		for(unsigned long i=0;i<DELAY_SV;i++){__NOP;}

		GPIOA->BSRR=GPIO_BSRR_BR0;
		for(unsigned long i=0;i<DELAY_SV;i++){__NOP;}
	}
	
	for(unsigned long i=0;i<DELAY_SV*3;i++){__NOP;}
}


void UART_Initial(void)
{
	//настройка линий
	GPIOA->CRH &= ~(GPIO_CRH_MODE9|GPIO_CRH_CNF9);					//сброс настроек пина
	GPIOA->CRH &= ~(GPIO_CRH_MODE10|GPIO_CRH_CNF10);				//сброс настроек пина
	
	GPIOA->CRH |= GPIO_CRH_CNF9_1;						//0b10 Alt pp
	GPIOA->CRH |= GPIO_CRH_MODE9;							//50MHz
	
	GPIOA->CRH |= GPIO_CRH_CNF10_0;						//0b01 inp floating
																						//mode=00
	if (!(RCC->APB2ENR & RCC_APB2ENR_IOPAEN))			//проверяем, тактируется ли порт
	{
		RCC->APB2ENR	|= RCC_APB2ENR_IOPAEN;
	}
	RCC->APB2ENR  |= RCC_APB2ENR_USART1EN;		//тактирование USART1
	
	USART1->BRR = Baud_rate_9600_72;					//настройка несущей
	
	USART1->CR1 |= USART_CR1_UE | USART_CR1_TE;	//вкл USART и режим передатчика
																							//отправляет IDLE пакет 0xFF
}


void USART1_Send_Char(char chr) {
  while(!(USART1->SR & USART_SR_TC));
  USART1->DR = chr;
}

void USART1_Send_String(char* str) {
  int i=0;
  while((str[i])!='\0')
    USART1_Send_Char(str[i++]);
}

/*
void UART_Transmit(char *data)
{
	char c;
	char* str=data;					//формируем строку для отправки
	do 
	{
		while(((USART1->SR)&USART_SR_TC)==0){__NOP;};			//ждём установки флага TC
																											//при установке данного флага также генеритуется прерывание
		c=(*str++);						//отправляем посимвольно, пока не встретится '\0'
		USART1->DR=c;	
	}
	while(c!='\0');
}
*/

void SPI_initial(void)
{
	//	master, mode0, 8 bit, fpclk/256, full-duplex
	RCC->APB2ENR |= (RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPAEN); //Тактируем AF and PORTA

	//PA5 - SCK
	//PA6 - MISO
	//PA7 - MOSI
	//PA2 - управляющий для ведомого NSS
	GPIOA->CRL&=~(GPIO_CRL_MODE7|GPIO_CRL_MODE6|GPIO_CRL_MODE5|GPIO_CRL_MODE2);              //Делаем сброс всех нужных пинов
	GPIOA->CRL&=~(GPIO_CRL_CNF7|GPIO_CRL_CNF6|GPIO_CRL_CNF5|GPIO_CRL_CNF2);                   //Делаем сброс всех нужных пинов

	GPIOA->CRL|=(GPIO_CRL_MODE7|GPIO_CRL_MODE5|GPIO_CRL_MODE2);      //7,5 pins AF push pull/6 Input pull-up /2output push-pull/ 50MHz
	GPIOA->CRL|=(GPIO_CRL_CNF7_1|GPIO_CRL_CNF6_0|GPIO_CRL_CNF5_1|GPIO_CRL_CNF2_1);	//7,5 pins AF push pull/6 Input floating /2output push-pull

	//GPIOA->CRL&=~GPIO_CRL_MODE6;//6 pin Input mode
	//GPIOA->CRL|=GPIO_CRL_CNF6_1;//6 pin input Floating

	RCC->APB2ENR|=RCC_APB2ENR_SPI1EN;     //Тактируем SPI
	SPI1->CR1=0;
	SPI1->CR2=0;

	SPI1->CR1 |= SPI_CR1_BR;							//  fpclk/256   br==111
	SPI1->CR1 |= SPI_CR1_MSTR;            //Режим Master
	SPI1->CR1 |= SPI_CR1_SSM;							//Software NSS
	SPI1->CR1 |= SPI_CR1_SSI;							//master
	/*
	SPI1->CR1 &= ~SPI_CR1_BR;                //пред делитель F_CPU 256
	SPI1->CR1 &= ~SPI_CR1_CPHA;             //По переднему фронту
	SPI1->CR1 &= ~SPI_CR1_CPOL;             //Тактовый сигнал в 0 в режиме простоя
	SPI1->CR1 &= ~SPI_CR1_DFF;                             //8 бит данных
	SPI1->CR1 &= ~SPI_CR1_LSBFIRST;         //MSB передается первым
	SPI1->CR1 &=~SPI_CR1_SSM;
	SPI1->CR1 |= SPI_CR1_MSTR;                      //Режим Master
	SPI1->CR1 &= ~SPI_CR1_RXONLY;           //ФУЛ дуплекс
	*/
	SPI1->CR1 |= SPI_CR1_SPE;             //Включаем SPI1
}


uint8_t SPI_Read_Write_Function(uint8_t data)
{
	while( !(SPI1->SR & SPI_SR_TXE)){__NOP;};         //ждём, пока не установили txe
	SPI1->DR=data;														//пересылаем данные
  while( !(SPI1->SR & SPI_SR_RXNE)){__NOP;};				//ждём флага готовности принятых данных
  return SPI1->DR;													//возвращаем полученные данные в вызывающую инструкцию
}

void s_delay_us(int number)
{
	for(int j=0; j<number; j++)
	{
		for(int i=0; i<DELAY_1us;i++)
		{
			__NOP;
		}
	}
}

void s_delay_ms(long number)
{
	for(int i=0; i<number; i++)
	{
		s_delay_us(1000);
	}
}