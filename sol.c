#include "stm32f10x.h"
#include <stdio.h>
#include <math.h>

GPIO_InitTypeDef GPIO_InitStructure;
EXTI_InitTypeDef EXTI_InitStructure;
NVIC_InitTypeDef NVIC_InitStructure;
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
ADC_InitTypeDef ADC_InitStructure;
USART_InitTypeDef USART_InitStructure;

void delay(uint32_t time)
{
	time = time*16000;
	while(time-- )
	{
	}
}

void PIN_Config();
void PWM_config();
void EXTI_Config();
void config();

void UART_Transmit(char *string);
double ExplicitEulerEq1(double (*f)(double,double,double), double y2, double ts, double Vin);
double ExplicitEulerEq2(double (*f)(double,double,double), double y1, double ts, double Vin);
double eq1(double Vc1, double Vc2, double Vin);
double eq2(double Vc1, double Vc2, double Vin);
double Gz(double,double);

double C1 = 0.220, C2 = 0.100, R1 = 10, R2 = 10;
char send[20];

volatile double outputToPC;
double stepSize = 0.1;
volatile double y2=0, y1=0; // Initial values
double Reelinput = 0;
volatile double input = 1;

volatile double y = 0 ;
volatile double input2 = 0;

void EXTI15_10_IRQHandler(void)
{
 if(EXTI_GetITStatus(EXTI_Line12) != RESET)
 {
		if(Reelinput == 0)
			Reelinput = 1;
		else
			Reelinput = 0;
		delay(50);
		EXTI_ClearITPendingBit(EXTI_Line12);
 }

}

void TIM3_IRQHandler(void)
{
	
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) == SET) // Periodic IRQ
	{
		input = Gz(Reelinput,y2);
		
		
		input2 = Gz(Reelinput,y);
		
		TIM_OCInitStructure.TIM_Pulse = input2*3499/3.3;    //PWM Signal
		TIM_OC3Init(TIM2, &TIM_OCInitStructure);
		
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	}
} 


int main(void)
{
	config();

	while(1)
	{
		y1 = ExplicitEulerEq1(eq1, y2, stepSize, input);
		y2 = ExplicitEulerEq2(eq2, y1, stepSize, input);
		
		y	= (double)(ADC_GetConversionValue(ADC1));
		y = (y/4095)*3.3;
		sprintf(send,"%0.4lf\r",y);
		UART_Transmit(send);
		
	}
	
}







void EXTI_Config(void)
{
	// Configuring external interrupt
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource12);
	
	//Button interupt
	EXTI_DeInit();
	EXTI_InitStructure.EXTI_Line = EXTI_Line12;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);	

	
	//Button interupt NVIC
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
		

	//TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	/*
	//Uart recive data 
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	*/
}

void ADC_Config()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE );

	// Set ADC clock (Maximum 14 MHz | 72/6 = 12 MHz)
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);
	
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	 // Enable/disable external conversion trigger (EXTI | TIM | etc.)	
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	// Configure data alignment (Right | Left)
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	// Set the number of channels to be used and initialize ADC
	ADC_InitStructure.ADC_NbrOfChannel = 1; 
	ADC_Init(ADC1, &ADC_InitStructure);
	
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 1, ADC_SampleTime_7Cycles5);
	ADC_Cmd(ADC1, ENABLE);
	
	//Calibiration
	ADC_ResetCalibration(ADC1);
	while(ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus(ADC1));
	
	// Start the conversion 
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);

}
void PIN_Config()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB ,ENABLE);
	
	//PWM output Config/Output pin Config
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//Button Config/Digital input Config (GPIO_Mode_IPD)
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	

	//Analog input (GPIO_Mode_AIN)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
}
void PWM_Config()
{
	//pwm TIM2 ENABLE
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	//Insignificant for pwm its for interupt
	TIM_ITConfig(TIM2, TIM_IT_Update | TIM_IT_CC2 | TIM_IT_CC3, ENABLE);
	
	TIM_Cmd(TIM2, ENABLE);
	
	//TIM Config
	TIM_TimeBaseStructure.TIM_Period = 3499;		
	TIM_TimeBaseStructure.TIM_Prescaler = 200;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	
	//PWM Pins Config
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
		
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);
		
	
}
void TIMx_Config()
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	//Insignificant for pwm its for interupt
	TIM_ITConfig(TIM3, TIM_IT_Update , ENABLE);
	
	TIM_Cmd(TIM3, ENABLE);
	
	//TIM Config
	TIM_TimeBaseStructure.TIM_Period = 16000000/100-1;		
	TIM_TimeBaseStructure.TIM_Prescaler = 100-1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

}
void UART_Config()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 ,ENABLE);
	
	// Configue UART RX
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	// Configue UART TX
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
		// USART settings
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(USART1, &USART_InitStructure);
	// Enable data receive interrupt & USART1
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); 
	USART_Cmd(USART1, ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	
}
void config()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB ,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE );
	
	PIN_Config();
	PWM_Config();
	ADC_Config();
	TIMx_Config();
	UART_Config();
	EXTI_Config();
	

}


//Vc1 = Vc1*(-R1-R2)/(C1*R1*R2) + Vc2*1/(R2*C1) + Vin*1/(C1*R1);
double eq1(double Vc1, double Vc2, double Vin)
{

	return Vc1*(-R1-R2)/(C1*R1*R2) + Vc2*1/(R2*C1) + Vin*1/(C1*R1);

}

//Vc2 = Vc1*(1)/(C2*R2) - Vc2*1/(R2*C2) ;
double eq2(double Vc1, double Vc2, double Vin)
{

	return Vc1*(1)/(C2*R2) - Vc2*1/(R2*C2) ;

}	

double ExplicitEulerEq1(double (*f)(double,double,double), double y2, double ts, double Vin)
{
    static double ykp1 = 0;
    double yk1 = ykp1 + ts * f(ykp1, y2, Vin);
    ykp1 = yk1;
    return yk1;
}

double ExplicitEulerEq2(double (*f)(double,double,double), double y1, double ts, double Vin)
{
    static double ykp2 = 0; // Previous value of the output
    double yk1 = ykp2 + ts * f(y1,ykp2, Vin);
    ykp2 = yk1;
    return yk1;
}

double Gz(double r, double y) 
{	
		static double yk_1=0, ik_1=0;
		double ik = r - y;
    
    double yk = yk_1 + 2.8*ik - 2.72*ik_1;
    ik_1 = ik;
		yk_1 = yk;
    return yk;
}
void UART_Transmit(char *string)
{
while(*string)
	{
		while(!(USART1->SR & 0x00000040));
		USART_SendData(USART1,*string);
		*string++;
	}
}