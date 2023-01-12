#include "stm32f10x.h"
#include <stdio.h>
#define LM75_ADDR 0x90 // LM75 address
#define LM75_REG_TEMP 0x00 // Temperature

GPIO_InitTypeDef  GPIO_InitStructure;
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
NVIC_InitTypeDef  NVIC_InitStructure;
EXTI_InitTypeDef  EXTI_InitStructure;
I2C_InitTypeDef I2C_InitStructure;
ADC_InitTypeDef ADC_InitStructure;
USART_InitTypeDef USART_InitStructure;

float temperature=0;
float LM75_Temperature(void);
extern float threshold=0;
extern int pressed=0;
uint8_t data;

void GPIO_Config();
void I2C_Config();
void TIM3_Config();
void UART1_config();
void ADC1_Config();
void NVIC_Config();
	
void delay(uint32_t time)
{
	time = time*16000;
	while(time-- )
	{
	}
}

void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET) // UART data received
    {
				data = USART_ReceiveData(USART1); // Get 8-bit data
				temperature=LM75_Temperature();
				if ((data == '1')&& (temperature>threshold)) GPIO_SetBits(GPIOA,GPIO_Pin_0);
				else if ((data == '0') && (temperature<threshold)) GPIO_ResetBits(GPIOA,GPIO_Pin_0);
		  	USART_ClearITPendingBit(USART1, USART_IT_RXNE); // Clear interrupt flag
    }
}

void EXTI1_IRQHandler(void)
{
if(EXTI_GetITStatus(EXTI_Line1) != RESET){
threshold=((int)ADC_GetConversionValue(ADC1))*50/4069; //50 IS THE MAX DEGREE
}
EXTI_ClearITPendingBit(EXTI_Line1);
}

void TIM3_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) == SET) // Periodic IRQ
	{
		int i=0;
		char bufferToSend[6];
		
		while(i<8){
		sprintf(bufferToSend, "%.3f\n", LM75_Temperature());
		USART_SendData(USART1,bufferToSend[i]);
		delay(20);
			i++; 
		}
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	}
}

int main(void){
	 GPIO_Config();
	 I2C_Config();
	 TIM3_Config();
	 UART1_config();
	 ADC1_Config();
	 NVIC_Config();
	 GPIO_ResetBits(GPIOA,GPIO_Pin_0);
		/* Configure the EXTI interrupt */
		EXTI_InitStructure.EXTI_Line =EXTI_Line1;
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;
		EXTI_Init(&EXTI_InitStructure);

		NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
	while (1){
}
}

//read the data from temperature register
float LM75_Temperature(void){
	 uint16_t value;
	 uint8_t raw;
	 // Enable I2C acknowledgment
	 I2C_AcknowledgeConfig(I2C1,ENABLE);
	 I2C_GenerateSTART(I2C1,ENABLE);
	 // Wait for EV5
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT));
	 // Send slave address
	I2C_Send7bitAddress(I2C1,LM75_ADDR,I2C_Direction_Transmitter);
	 // Wait for EV6
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	 // Send register address
	I2C_SendData(I2C1,LM75_REG_TEMP);
	 // Wait for EV8
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	 // Send repeated START condition
	I2C_GenerateSTART(I2C1,ENABLE);
	 // Wait for EV5
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT));
	 // Send slave address for READ
	I2C_Send7bitAddress(I2C1,LM75_ADDR,I2C_Direction_Receiver);
	 // Wait for EV6
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

	 // Wait for EV7
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED));
	 // Receive high byte
	value = (I2C_ReceiveData(I2C1) << 8); //resetting value
	 // Disable I2C acknowledgment
	I2C_AcknowledgeConfig(I2C1,DISABLE);
	 // Send STOP condition
	I2C_GenerateSTOP(I2C1,ENABLE);
	 // Wait for EV7
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED));
	 // Receive low byte
	value |= I2C_ReceiveData(I2C1);
	raw = value >> 5;
	return raw*0.125;
	}

	
	void GPIO_Config(){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	// LED Pin
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// Button 1 config
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//I2C Config
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
		
	// Configue UART RX - UART module's TX should be connected to this pin
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	// Configue UART TX - UART module's RX should be connected to this pin
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// Configure input PA4 (Analog Input) for potentiometer
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource1);
	EXTI_DeInit();
	}
	
	void I2C_Config(){
		
	//IC21 Initilization
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0x00;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = 100000;
	I2C_Init(I2C1, &I2C_InitStructure);
	I2C_Cmd(I2C1, ENABLE);
	}
	
  void TIM3_Config(){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
	//TIM Config
	TIM_TimeBaseStructure.TIM_Period = 35999;		
	TIM_TimeBaseStructure.TIM_Prescaler = 1999;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	
	TIM_ITConfig(TIM3, TIM_IT_Update , ENABLE);
	
	TIM_Cmd(TIM3, ENABLE);
}
	void UART1_config(){
	USART_InitTypeDef USART_InitStructure;
//  USART_ClockInitTypeDef USART_ClockInitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(USART1, &USART_InitStructure);
	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
//	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART1, ENABLE);
}
	
  void ADC1_Config(){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	// Set ADC clock (Maximum 14 MHz | 72/6 = 12 MHz)
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);
	
	//ADC Settings
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	// Enable/disable external conversion trigger (EXTI | TIM | etc.)
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	// Configure data alignment (Right | Left)
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	// Set the number of channels to be used and initialize ADC
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	ADC_InitStructure.ADC_ScanConvMode=DISABLE;
	ADC_Init(ADC1, &ADC_InitStructure);

	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 1, ADC_SampleTime_7Cycles5);
	ADC_Cmd(ADC1, ENABLE);

	ADC_ResetCalibration(ADC1);
	while (ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);
	while (ADC_GetCalibrationStatus(ADC1));
	// Start the conversion
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

  void NVIC_Config(){
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	//TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}