#include <stm32f0xx_ll_bus.h>
#include <stm32f0xx_ll_gpio.h>
#include <stm32f0xx_ll_utils.h>
#include <stm32f0xx_ll_i2c.h>
#include <stdio.h>
#include <string.h>


/*
 *class ds18b20
{
	
	
public:
	void begin(void)
	{
		RCC->AHBENR |= RCC_AHBENR_GPIOAEN;   //Enable Clock GPIOA
		GPIOA->BSRR |= GPIO_BSRR_BS_1;
	
		GPIOA->MODER |= GPIO_MODER_MODER1_0;
		GPIOA->OTYPER |= (1 << 1);
		GPIOA->OSPEEDR |= (0X03 << 2);
	
	}
	uint8_t flag;
	volatile unsigned char Temp_H = 0, Temp_L = 0, OK_Flag = 0, temp_flag;
	volatile unsigned char TempInt = 0;      // переменные для целого значения температуры
	volatile unsigned int temppoint = 0, temppoint1;      // переменные для дробного значения температуры

	ds18b20(GPIO_TypeDef *GPIO, uint32_t ODR_PIN, uint32_t ID_PIN)
	{
	}
	


	uint8_t Reset_One_wire(GPIO_TypeDef *GPIO, uint32_t ODR_PIN, uint32_t ID_PIN)
	{
		
		GPIO->ODR &= ~ODR_PIN;
		us_delay(280);
		GPIO->ODR |= ODR_PIN;	
		us_delay(20);
	
		if (GPIO->IDR & ID_PIN)
			return 0x00;
	
		else
			return 0x01;
	
	}
	void write_18b20(unsigned char dat, GPIO_TypeDef *GPIO, uint32_t ODR_PIN)
	{
		unsigned char i;
		for (i = 0; i < 8; i++)
		{
			GPIO->ODR &= ~ODR_PIN;
			__NOP();
			if (dat & 0x01)
			{
				GPIO->ODR |= ODR_PIN;
			}
			else
			{
				GPIO->ODR &= ~ODR_PIN;
			}
			dat = dat >> 1;
			us_delay(75);
			GPIO->ODR |= ODR_PIN;
			us_delay(2);
		}
	}
	unsigned char read_18b20(GPIO_TypeDef *GPIO, uint32_t ODR_PIN, uint32_t ID_PIN)
	{
		unsigned char i;
		unsigned char dat = 0;
	
		for (i = 0; i < 8; i++)
		{
			GPIOA->ODR &= ~ODR_PIN;
			us_delay(2);
			GPIOA->ODR |= ODR_PIN;
			us_delay(4);
			dat = dat >> 1;
			if (GPIOA->IDR & ID_PIN)
			{
				dat |= 0x80;
			}
			us_delay(100);
		}
		return dat;
	}
	
	void DS18b20_Get_Temp(GPIO_TypeDef *GPIO, uint32_t ODR_PIN, uint32_t ID_PIN)
	{
	
		flag = Reset_One_wire(GPIO, ODR_PIN, ID_PIN);               // инициализация DS18B20
		if(flag)
		{
			us_delay(80);
			write_18b20(0xCC, GPIO, ODR_PIN);            // проверка кода датчика
			write_18b20(0x44, GPIO, ODR_PIN);            // запуск температурного преобразования
			LL_mDelay(10);
			flag = Reset_One_wire(GPIO, ODR_PIN, ID_PIN);                 // инициализация DS18B20
			us_delay(80);
			write_18b20(0xCC, GPIO, ODR_PIN);            // проверка кода датчика
			write_18b20(0xBE, GPIO, ODR_PIN);            // считываем содержимое ОЗУ
			us_delay(80);
			Temp_L = read_18b20(GPIO, ODR_PIN, ID_PIN);         // читаем первые 2 байта блокнота
			Temp_H = read_18b20(GPIO, ODR_PIN, ID_PIN);
			LL_mDelay(50);	
		}
	
	
	}


private:

	GPIO_TypeDef *GPIO;
	uint32_t ODR_PIN;
	uint32_t ID_PIN;
	void us_delay(uint32_t time_delay)
	{
		uint32_t i = 0;
		for (i = 0; i < time_delay; i++);
	}
};

ds18b20 ds(GPIOA, GPIO_ODR_1, GPIO_IDR_1);
*/

void Init_OneWire(void);
void us_delay(uint32_t time_delay);

uint8_t Reset_One_wire(GPIO_TypeDef *GPIO, uint32_t ODR_PIN, uint32_t ID_PIN);
void write_18b20(unsigned char dat, GPIO_TypeDef *GPIO, uint32_t ODR_PIN);
unsigned char read_18b20(GPIO_TypeDef *GPIO, uint32_t ODR_PIN, uint32_t ID_PIN);
uint8_t *DS18b20_Get_Temp(void);
float ds18b20_Convert(uint16_t dt);
void USART_Init(uint32_t baudRate);
void Usart_Send_byte(uint8_t data);
void USART_Str_Print(char *data, uint8_t size);

void i2c_write();
void init_i2c();

typedef struct
{
	volatile uint8_t tx_Err : 1;
	volatile uint8_t tx_TC : 1;
	volatile uint8_t tx_HT : 1;
	volatile uint8_t rx_Err : 1;
	volatile uint8_t rx_TC : 1;
	volatile uint8_t rx_HT : 1;
	
} UART_Flag;
UART_Flag UsartFlag;


char str[100];

uint8_t flag;

float temp_data;
uint16_t data_t;

uint8_t *p;
uint8_t temper[2];
char buf[30];

int main(void)
{
	LL_InitTick(8000000, 1000);
	
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;   //Enable Clock GPIOA
	
	
	GPIOA->MODER |= GPIO_MODER_MODER4_0;
	
	GPIOA->BSRR |= GPIO_BSRR_BS_4;
	
	USART_Init(9600);
	//init_i2c();
	Init_OneWire();
	
	//ds.begin();
	
	LL_mDelay(100);
	

	
	for (;;)
	{
		
		p =	DS18b20_Get_Temp();
	
		temper[0] = *p;
		temper[1] = *(p+1);
		
		
		temp_data = ds18b20_Convert(data_t);
		
		
		sprintf(buf, "Температура = %u.%u", temper[0], temper[1]);

		USART_Str_Print(buf, strlen(buf));
		Usart_Send_byte(0x0d);		
		
		
		sprintf(str, "y= %d.%01d", (uint32_t)temp_data, (uint16_t)((temp_data - (uint32_t)temp_data) * 1000));
		USART_Str_Print(str, strlen(str));
		Usart_Send_byte(0x0d);
		
		//i2c_write();
		LL_mDelay(700);
	
	}
}


void init_i2c()
{
	I2C1->CR1 = 0;
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;   //Enable Clock GPIOA
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
	GPIOA->AFR[1] |= (1 << 6) | (1 << 10);
	GPIOA->MODER |= GPIO_MODER_MODER10_Msk;
	GPIOA->MODER |= GPIO_MODER_MODER9_Msk;
	GPIOA->OTYPER |= (0X3<<9);
	GPIOA->PUPDR |= (1 << 20) | (1 << 18);
	GPIOA->OSPEEDR |= (0X0F << 18);
	
	I2C1->OAR1 |= (0x1D<<1);
	I2C1->TIMINGR = 0x10420F0D;
	I2C1->CR1 |= 0X01;
	
}

void i2c_write()
{
	LL_I2C_HandleTransfer(I2C1, 0X0D, 7, 1, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);
	LL_I2C_TransmitData8(I2C1,0XAB);
	
	
	
	
	
}

void Init_OneWire(void)
{
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;  //Enable Clock GPIOA
	GPIOA->BSRR |= GPIO_BSRR_BS_1;
	
	GPIOA->MODER |= GPIO_MODER_MODER1_0;
	GPIOA->OTYPER |= (1 << 1);
	GPIOA->OSPEEDR |= (0X03 << 2);
	
}

void us_delay(uint32_t time_delay)
{
	uint32_t i = 0;
	for (i = 0; i < time_delay; i++) ;
}

uint8_t Reset_One_wire(GPIO_TypeDef *GPIO, uint32_t ODR_PIN, uint32_t ID_PIN)
{
		
	GPIO->ODR &= ~ODR_PIN;
	us_delay(280);
	GPIO->ODR |= ODR_PIN;	
	us_delay(20);
	
	if(GPIO->IDR & ID_PIN)
		return 0x00;
	
	else
		return 0x01;
	
}	

void write_18b20(unsigned char dat, GPIO_TypeDef *GPIO, uint32_t ODR_PIN)
{
	unsigned char i;
	for (i = 0; i < 8; i++)
	{
		GPIO->ODR &= ~ODR_PIN;
		__NOP();
		if(dat & 0x01)
		{
			GPIO->ODR |= ODR_PIN;
		}
		else
		{
			GPIO->ODR &= ~ODR_PIN;
		}
		dat = dat >> 1;
		us_delay(75);
		GPIO->ODR |= ODR_PIN;
		us_delay(2);
	}
}

unsigned char read_18b20(GPIO_TypeDef *GPIO, uint32_t ODR_PIN, uint32_t ID_PIN)
{
	unsigned char i;
	unsigned char dat = 0;
	
	for (i = 0; i < 8; i++)
	{
		GPIOA->ODR &= ~ODR_PIN;
		us_delay(2);
		GPIOA->ODR |= ODR_PIN;
		us_delay(4);
		dat = dat >> 1;
		if (GPIOA->IDR & ID_PIN)
		{
			dat |= 0x80;
		}
		us_delay(100);
	}
	return dat;
}

uint8_t *DS18b20_Get_Temp(void)
{
	uint8_t Temp_H = 0, Temp_L = 0, temp_flag;
	uint8_t TempInt = 0;       // переменные для целого значения температуры
	unsigned int temppoint = 0, temppoint1;       // переменные для дробного значения температуры
	static uint8_t data[2];
	temp_flag = 1; 

	
	
	
	
	
	flag = Reset_One_wire(GPIOA,GPIO_ODR_1,GPIO_IDR_1);            // инициализация DS18B20
	if(flag)
	{
		us_delay(80);
		write_18b20(0xCC, GPIOA, GPIO_ODR_1);                  // проверка кода датчика
		write_18b20(0x44, GPIOA, GPIO_ODR_1);                  // запуск температурного преобразования
		LL_mDelay(10);
		flag = Reset_One_wire(GPIOA, GPIO_ODR_1, GPIO_IDR_1);  // инициализация DS18B20
		us_delay(80);
		write_18b20(0xCC, GPIOA, GPIO_ODR_1);                  // проверка кода датчика
		write_18b20(0xBE, GPIOA, GPIO_ODR_1);                  // считываем содержимое ОЗУ
		us_delay(80);
		Temp_L = read_18b20(GPIOA, GPIO_ODR_1, GPIO_IDR_1);      // читаем первые 2 байта блокнота
		Temp_H = read_18b20(GPIOA, GPIO_ODR_1, GPIO_IDR_1);
		LL_mDelay(50);	
		
		data_t = (Temp_H << 8)|Temp_L;
		
		if (Temp_H &(1 << 3))   // проверяем бит знака температуры на равенство единице
		{
        	
				signed int tmp;
				temp_flag = 0;         // флаг знака равен 0(минус)
				tmp = (Temp_H << 8) | Temp_L;
				tmp = -tmp;
				Temp_L = tmp;
				Temp_H = tmp >> 8;
		
		}
		
		
		
		TempInt = ((Temp_H << 8) | Temp_L) * 0.0625;    // вычисляем целое значение температуры

		temppoint = Temp_L & 0x0F;    // вычисляем дробное значение температуры
		temppoint = temppoint * 625;          // точность температуры
		temppoint1 = temppoint / 1000;
	
		
		
		data[0] = TempInt;
		data[1] = temppoint1;
	
		return data;
		
	}
}


float ds18b20_Convert(uint16_t dt)
{
	float t;

	t = (float)((dt & 0x07FF) >> 4);  //отборосим знаковые и дробные биты

	//Прибавим дробную часть

	t += (float)(dt & 0x000F) / 16.0f;

	return t;

}




void USART_Init(uint32_t baudRate)
{
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;  //Enable clock Usart
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	
	GPIOA->MODER |= (1 << 7) | (1 << 5);   //Set mode Alternete function
	GPIOA->AFR[0] |= (1 << 12) | (1 << 8);  // Selection Alternete function 
	
	USART1->BRR = 8000000 / baudRate;  //Baud rate 
	USART1->CR1 = USART_CR1_RXNEIE | USART_CR1_RE | USART_CR1_TE | USART_CR1_UE;	
	USART1->CR1 |= (1 << 8);
	UsartFlag.tx_TC = 1;
}


void USART_Str_Print(char *data, uint8_t size)
{		
	for (uint8_t i = 0; i < (size); i++)
	{
		while (!(USART1->ISR&(1 << 7)))
			continue;
		USART1->TDR = data[i];
	}
}

void Usart_Send_byte(uint8_t data)
{
	
	while (!(USART1->ISR&(1 << 7)))
		continue;
	USART1->TDR = data;
}

