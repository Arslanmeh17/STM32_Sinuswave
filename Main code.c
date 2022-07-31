#include "stm32f10x.h"
#include "delay.h" 
#include "math.h"
#define pi 3.14159265358979323846  //Defining pi


GPIO_InitTypeDef GPIO_InitStructure; // Peripheral libraries
EXTI_InitTypeDef EXTI_InitStructure; //External interrupt library
NVIC_InitTypeDef NVIC_InitStructure; //NVIC Library
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure; //Timer library
TIM_OCInitTypeDef TIM_OCInitStructure; //Oc library 
ADC_InitTypeDef ADC_InitStructure; //ADC library

void GPIO_config(void);
void ADC_config(void);
void TIM_config(void);


uint32_t potValue=0;  //Our potentiometer value.

static uint32_t t=0;  //Our time value.

void TIM3_IRQHandler(void){   //TIMER Function

		 if((TIM_GetITStatus(TIM3, TIM_IT_Update) == SET) ){  //50 Hz = 0.02 seconds
			
     t++; //our time which increases every 0.02 seconds
	   
		 
		 TIM_ClearITPendingBit(TIM3, TIM_IT_Update);   //we need to clear line pending bit manually
	 }
		}

		
 int main(void) {		 			 
 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //A port clock enabled
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //B port clock enabled
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,  ENABLE); //AFIO clock enabled
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); // Timer clock enabled
	RCC_ADCCLKConfig(RCC_PCLK2_Div6); // Setting Adc clock 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);	// ADC clock 

	delayInit(); //delay for prevent arcs on the button
  
	GPIO_config(); 
  ADC_config();
  TIM_config();
	 
	 
	 
     while(1)
     {				 
			 
			 potValue = ADC_GetConversionValue(ADC1); // getting the value from our potentiometer.
			 potValue = potValue*2.8;  // Calibrated for when potentiometer is max, pot value is ~ 10K. The max value of duty. I checked from value viewer of stmstudio.
			 
		   float frequency = potValue/1000;  // potValue is between 0-10000 nearly. So aligning the potValue.
			 
  		 float rads = pi/180;  //to use radians and align the sample value.
			 
			 TIM3->CCR1 = sin(1.8*t*frequency*rads-0.75)*10000;  
				 
       TIM3->CCR2 = sin(1.8*t*frequency*rads-0.15)*10000;
			 
			 TIM3->CCR3 = sin(1.8*t*frequency*rads+0.45)*10000;
			 
			 TIM3->CCR4 = sin(1.8*t*frequency*rads+1.05)*10000;  			 
        
   } //Closing while
 } //Closing main
 
 
 
 void GPIO_config(void)
{  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;  // Configuring pin A0 for analog input (potentiometer).
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// configure leds' output
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;  //Leds' button pins  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; //clock Speed
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // Alternate Function Push-pull mode
	GPIO_Init(GPIOA, &GPIO_InitStructure); //A port	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;  //Leds' button pins  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; //clock Speed
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // Alternate Function Push-pull mode
	GPIO_Init(GPIOB, &GPIO_InitStructure); //B port	
}

void ADC_config(void)   // ADC configuration 
{
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	ADC_Init(ADC1, &ADC_InitStructure);	 

	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_7Cycles5);
	ADC_Cmd(ADC1, ENABLE);

	ADC_ResetCalibration(ADC1);
	while(ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus(ADC1));
	// Start the conversion
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);	
}

void TIM_config(void)    // TIMER configuration for TIM3
{
  TIM_TimeBaseStructure.TIM_Period = 9999; 
	TIM_TimeBaseStructure.TIM_Prescaler = 143; // 72M /144*10K = 50Hz.
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  
	NVIC_InitTypeDef NvicStructure;
  NvicStructure.NVIC_IRQChannel = TIM3_IRQn;
  NvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NvicStructure.NVIC_IRQChannelSubPriority = 1;
  NvicStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NvicStructure);

  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
  TIM_Cmd(TIM3, ENABLE); //Enabling the timer
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; // Our PWM
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	
	TIM_OCInitStructure.TIM_Pulse = 0;       // initial is zero.
	TIM_OC1Init(TIM3, &TIM_OCInitStructure); // for red1 led (pin A6)

	TIM_OCInitStructure.TIM_Pulse = 0;       
	TIM_OC2Init(TIM3, &TIM_OCInitStructure); // for yellow led (pin A7)      (oc1 for pin0, so i used oc2-oc3-oc4)

	 
	TIM_OCInitStructure.TIM_Pulse = 0;       
	TIM_OC3Init(TIM3, &TIM_OCInitStructure); // for green led (pin B0)


	TIM_OCInitStructure.TIM_Pulse = 0;       
	TIM_OC4Init(TIM3, &TIM_OCInitStructure); // for red2 led (pin B1)
  
}