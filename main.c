#define F_CPU 72000000UL
#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
 
// Отдадочная затычка. Сюда можно вписать код обработки ошибок.
#define	ERROR_ACTION(CODE,POS)		do{}while(1)
#define DELAY_SV 0xfffff
#define Baud_rate_9600_72 0x1D4C
#define DELAY_1us 9						//подобрано экспериментально
#define Q_SPI_SIZE 8
#define Q_USART_SIZE 8

portBASE_TYPE *pxHigherPriorityTaskWoken;
xSemaphoreHandle SemSPI_tx, SemSPI_rx, SemUSART;
xQueueHandle Q_SPI, Q_USART;

char TX_flag;
char RX_flag;
char dataSPI;
char U_TX_fl=0;

void test_sv(int);
void test_sv_vDelay(int);
void UART_Initial(void);
void USART1_Send_Char(char);
void USART1_Send_String(char*);
void GPIO_initial(void);
void SPI_initial(void);
char SPI_Read_Write_Function(char);

void Main_Thread (void *);
void SPI_transmitter (void *);
void USART_transmitter (void *);


void s_delay_us(int);					//на основе цикла
void s_delay_ms(long);

int main(void)
{
	SystemInit();
		
	GPIO_initial();		//порты ввода-вывода общего назначения
	UART_Initial();
	SPI_initial();
	
	Q_SPI = xQueueCreate( Q_SPI_SIZE, sizeof(uint32_t));  //настраиваем очередь для SPI-передачи
	Q_USART = xQueueCreate( Q_USART_SIZE, sizeof(char));  //настраиваем очередь для USART-передачи
	

	vSemaphoreCreateBinary(SemSPI_tx);	//инициализируем двоичные семафоры
	vSemaphoreCreateBinary(SemSPI_rx);	//инициализируем двоичные семафоры
	vSemaphoreCreateBinary(SemUSART);	//инициализируем двоичные семафоры
	//test_sv(5);
	
	__enable_irq();
	NVIC_EnableIRQ(SPI1_IRQn);
	NVIC_EnableIRQ(USART1_IRQn);
	
	
	if(pdTRUE != xTaskCreate(Main_Thread,(char *)"Main_Thread", 	configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL)) 
	{
		ERROR_ACTION(TASK_NOT_CREATE,0);
	}
	
	if(pdTRUE != xTaskCreate(SPI_transmitter,(char *)"SPI_transmitter", 	configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL)) 
	{
		ERROR_ACTION(TASK_NOT_CREATE,0);
	}
	
	if(pdTRUE != xTaskCreate(USART_transmitter,(char *)"USART_transmitter", 	configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL)) 
	{
		ERROR_ACTION(TASK_NOT_CREATE,0);
	}
	
	
	vTaskStartScheduler();
	while(1){};
}

void Main_Thread (void *pvParameters)
{
	//test_sv_vDelay(2);
	//test_sv(2);
	while(1)
	{			
		//каждые 2 секунды выдаём последовательность байтов для передачи по SPI
		//char Frame[4]={0x0F, 0x00, 0x19, 0x00};
		uint32_t Fr=0x0F001900;
		xQueueSend( Q_SPI, &(Fr), portMAX_DELAY );
		
		vTaskDelay(2000);
	}
}

void SPI_transmitter (void *pvParameters)
{	
	while(1)
	{
		uint32_t Fr;
		xQueueReceive( Q_SPI, &(Fr), portMAX_DELAY ); //в очереди есть элементы для передачи 
		
		char rec[4];
		for (int i=0; i<4; i++)
		{
				rec[3-i]=Fr&(0xFF);
				Fr=(Fr>>8);
		}
		
		GPIOA->CRL&=~(GPIO_CRL_CNF2);  //сброс 2 пина                
		GPIOA->CRL|= GPIO_CRL_CNF2_1;	//2output push-pull
		//без этого не читает несколько раз сподряд

		GPIOA->BSRR=GPIO_BSRR_BR2;		//chip-select

		for(int i=0; i<4; i++)
		{
			//xQueueSend( Q_USART, &(rec[i]), portMAX_DELAY);//записываем данные в другую очередь
			char data_to_USART=SPI_Read_Write_Function(rec[i]);
			xQueueSend( Q_USART, &(data_to_USART), portMAX_DELAY);//записываем данные в другую очередь
		}
		
		GPIOA->BSRR=GPIO_BSRR_BS2;		//chip-select off
	}
}

void USART_transmitter (void *pvParameters)
{
	while(1)
	{
		char data;
		xQueueReceive( Q_USART, &data, portMAX_DELAY ); //байт из очереди убираем после прочтения 
		USART1_Send_Char(data);
	}
}

void GPIO_initial(void)
{
	RCC->APB2ENR	|= RCC_APB2ENR_IOPAEN;			//тактируем PA
	//настраиваем PA0 на вывод - светодиод
	GPIOA->CRL&=~GPIO_CRL_CNF0;				//сбрасываем настройки пина
	GPIOA->CRL&=~GPIO_CRL_MODE0;			//сбрасываем настройки пина
	
	GPIOA->CRL=GPIO_CRL_MODE0_1;				//Режим MODE0_1 (max 2MHz)
																			//CNF==0b00 - general purpose output push-pull
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
	
	USART1->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_TCIE ;	//вкл USART и режим передатчика, прерывание по завершении передачи																						//отправляет IDLE пакет 0xFF
}

void USART1_Send_Char(char chr) 
{
	xSemaphoreTake(SemUSART, portMAX_DELAY);
	//while(U_TX_fl==0){};
	//U_TX_fl=0;
	//USART1->SR |= USART_SR_TC;
	USART1->DR = chr;
	//test_sv(1);
}

void USART1_Send_String(char* str) 
{
  int i=0;
  while((str[i])!='\0')
    USART1_Send_Char(str[i++]);
}

void SPI_initial(void)
{
	//	master, mode0, 8 bit, fpclk/256, full-duplex
	if (!(RCC->APB2ENR & RCC_APB2ENR_IOPAEN))			//проверяем, тактируется ли порт
	{
		RCC->APB2ENR	|= RCC_APB2ENR_IOPAEN;
	}
	//PA5 - SCK
	//PA6 - MISO
	//PA7 - MOSI
	//PA2 - управляющий для ведомого NSS
	GPIOA->CRL&=~(GPIO_CRL_MODE7|GPIO_CRL_MODE6|GPIO_CRL_MODE5|GPIO_CRL_MODE2);              //Делаем сброс всех нужных пинов
	GPIOA->CRL&=~(GPIO_CRL_CNF7|GPIO_CRL_CNF6|GPIO_CRL_CNF5|GPIO_CRL_CNF2);                   //Делаем сброс всех нужных пинов

	GPIOA->CRL|=(GPIO_CRL_MODE7|GPIO_CRL_MODE5|GPIO_CRL_MODE2);      //7,5 pins AF push pull/6 Input pull-up /2output push-pull/ 50MHz
	GPIOA->CRL|=(GPIO_CRL_CNF7_1|GPIO_CRL_CNF6_0|GPIO_CRL_CNF5_1|GPIO_CRL_CNF2_1);	//7,5 pins AF push pull/6 Input floating /2output push-pull

	RCC->APB2ENR|=RCC_APB2ENR_SPI1EN;     //Тактируем SPI
	SPI1->CR1=0;
	SPI1->CR2=0;

	SPI1->CR1 |= SPI_CR1_BR;							//  fpclk/256   br==111
	SPI1->CR1 |= SPI_CR1_MSTR;            //Режим Master
	SPI1->CR1 |= SPI_CR1_SSM;							//Software NSS
	SPI1->CR1 |= SPI_CR1_SSI;							//master
	
	SPI1->CR2 |= (SPI_CR2_RXNEIE|SPI_CR2_TXEIE);						//прерывания
	SPI1->CR1 |= SPI_CR1_SPE;             //Включаем SPI1
}

char SPI_Read_Write_Function(char data)
{
	xSemaphoreTake(SemSPI_tx, portMAX_DELAY);
	//while(TX_flag==0){__NOP;};         //ждём, пока не установили txe
	TX_flag=0;
	SPI1->DR=data;														//пересылаем данные
	SPI1->CR2|=SPI_CR2_RXNEIE;
	
	xSemaphoreTake(SemSPI_rx, portMAX_DELAY);
	SPI1->CR2|=SPI_CR2_TXEIE;
	//while(RX_flag==0){__NOP;};				//ждём флага готовности принятых данных
	RX_flag=0;
	return SPI1->DR;													//возвращаем полученные данные в вызывающую инструкцию
}

void test_sv(int counter)				//светодиод на линии PA0
{
	for(int l=0; l<counter; l++)
	{
		GPIOA->BSRR=GPIO_BSRR_BS0;
		s_delay_ms(200);
	
		GPIOA->BSRR=GPIO_BSRR_BR0;
		s_delay_ms(200);
		
	}
	s_delay_ms(1000);
}

void test_sv_vDelay(int counter)				//светодиод на линии PA0
{
	for(int l=0; l<counter; l++)
	{
		GPIOA->BSRR=GPIO_BSRR_BS0;
		//s_delay_ms(200);
		vTaskDelay(200);
		GPIOA->BSRR=GPIO_BSRR_BR0;
		//s_delay_ms(200);
		vTaskDelay(200);
	}
	vTaskDelay(1000);
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

//-------------------------------------------------
//-------------------------------------------------
//обработчики прерываний

void USART1_IRQHandler()
{
	if(USART1->SR & USART_SR_TC)
	{
		//test_sv(2);
		//if (!U_TX_fl)
		//{
		xSemaphoreGiveFromISR(SemUSART, pxHigherPriorityTaskWoken);
		//}
		//доработать реакцию на pxHigherPriorityTaskWoken
	//	U_TX_fl=1;
		
		//USART1->CR1 &= ~USART_CR1_TCIE;
		USART1->SR &= ~USART_SR_TC;
	}
}

void SPI1_IRQHandler()
{
	//возможны ошибки!!
	//прерывание срабатывает несколько раз сподряд!!
	//организовать разделение прерываний TX и RX??
	if(SPI1->SR & SPI_SR_TXE)
	{
		if(!TX_flag)		//костыль!!
		{
			//pxHigherPriorityTaskWoken=pdFALSE;
			xSemaphoreGiveFromISR(SemSPI_tx, pxHigherPriorityTaskWoken);
			//if(pxHigherPriorityTaskWoken==pdTRUE){};
			//доработать реакцию на pxHigherPriorityTaskWoken
			SPI1->CR2 &= ~SPI_CR2_TXEIE;		//запрещаем прерывание
			SPI1->SR &= ~SPI_SR_TXE;				//лишнее - должны сбрасываться аппаратно??
			TX_flag=1;
		}
	}
	if(SPI1->SR & SPI_SR_RXNE)
	{
		if(!RX_flag)		//костыль!!
		{
			xSemaphoreGiveFromISR(SemSPI_rx, pxHigherPriorityTaskWoken);
			//доработать реакцию на pxHigherPriorityTaskWoken
			SPI1->CR2 &= ~SPI_CR2_RXNEIE;	//запрещаем прерывание
			SPI1->SR &= ~SPI_SR_RXNE;			//лишнее - должны сбрасываться аппаратно??
			RX_flag=1;
		}
		
	}	
}
//-------------------------------------------------
//-------------------------------------------------


void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName ) 
{
	test_sv(4); 
} 
void vApplicationTickHook( void ) 
{ 
//не трогаем, потому что не будем ловить события при тике 
} 
void vApplicationIdleHook( void ) 
{ 
//в идл состоянии ни чего не делаем 
} 
void vApplicationMallocFailedHook( void ) 
{ 
	test_sv(4);
}
