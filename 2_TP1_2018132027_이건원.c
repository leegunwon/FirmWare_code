// 거리센서 1 : PA1(ADC3_IN1)
// 거리센서 2 : PF3(ADC3_IN9)
// 추종자 엔진 : 
// 거리센서1: 마이컴키트상의 가변저항, PA1(ADC3_IN1)
// 거리센서2: 40pin Box header connector에 연결된 외부 가변저항, PF3
//(ADC3_IN9)
// 추종차 엔진: PB7(TIM4_CH2(PWM mode)), LED로 PWM 변화확인
// LED: 40pin Box header connector에 추가 부착한 외부 LED
// 추종차 핸들: PF9(TIM14_CH1(PWM mode)), Buzzer로 PWM 변화확인
// Off-line 추종차 시동: Move-key(SW4(EXTI12)), Stop-key(SW6(EXTI14))
// 원격(PC 통신프로그램) 추종차 시동: Move-key(‘M’), Stop-key(‘S’)
// 거리값 표시: GLCD(D), PC
// 추종차 속도 표시: GLCD(DR)
// 추종차 방향 표시: GLCD(‘L’, ‘R’, ‘F’)

#include "stm32f4xx.h"
#include "GLCD.h"
#include "ACC.h"

void DisplayTitle(void);
void Display_Process(int16 *pBuf);

void _ADC_Init(void);
void DMAInit(void);

void TIMER1_Init(void);
void TIMER14_PWM_Init(void);
void TIMER4_PWM_Init(void);

void _EXTI_Init(void);

void SerialSendChar(uint8_t c);
void SerialSendString(char* s);

void USART_BRR_Configuration(uint32_t USART_BaudRate);
void DelayMS(unsigned short wMS);
void DelayUS(unsigned short wUS);


void USART1_Init();

void StopMode(void);

uint16_t ADC_value[2], Voltage[2], Distance[2], DR;
uint8_t mode = 0;
uint8_t str[20];


int main(void)
{
	LCD_Init();	
	DelayMS(10);	
 	DisplayTitle();
	_ADC_Init();
    TIMER1_Init();
    DMAInit();
	_EXTI_Init();
	TIMER14_PWM_Init();
	TIMER4_PWM_Init();
	USART1_Init();
  //  Fram_Init();   
    //Fram_Status_Config();  
	//mode = Fram_Read(50);
	if (mode == 1){
		StopMode();
	}
	while(1){

	}
}
void _ADC_Init(void)
{
	// ADC1_IN1: (PA1)
	// ADC1_IN9: (PF3)
	/* 1st Analog signal */
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;  	// RCC_AHB1ENR GPIOA Enable
	GPIOA->MODER |= 0x0C;	// GPIOA PIN1(PA1) �������� : Analog mode
    

	/* 2nd Analog signal */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;  // RCC_AHB1ENR GPIOF Enable
	GPIOF->MODER |= 0x30;	// GPIOF PIN3(PF3) 


	/* ADC Common Init **********************************************************/
	RCC->APB2ENR |= (1<<10);	// RCC_APB2ENR ADC3 Enable

	ADC->CCR &= ~0X0000001F;// ADC_Mode_Independent
	ADC->CCR |= (1<<16);	// ADC_Prescaler_Div4 (ADC MAX Clock 36Mhz, 84Mhz(APB2)/4 = 21Mhz

	/* ADC3 Init ****************************************************************/
	ADC3->CR1 |= (1 << 24);	// RES[1:0]=0b00 : 12bit Resolution
	ADC3->CR1 |= (1<<8);	// ADC_ScanCovMode Enable (SCAN=1)
	ADC3->CR1 |=  (1<<5);		// EOCIE=1: Interrupt enable for EOC
	ADC3->CR2 &= ~(1<<1);	// ADC_ContinuousConvMode DISABLE (CONT=0)
	ADC3->CR2 |= (2 << 24);		// EXTSEL[3:0]= 0b0010: Timer1_CH3 clock

    ADC3->CR2 |= (3<<28);
	ADC3->CR2 &= ~(1 << 11);	// ALIGN=0: ADC_DataAlign_Right
	ADC3->CR2 &= ~(1 << 10);	// EOCS=0: The EOC bit is set at the end of each sequence of regular conversions

	ADC3->SQR1 |= (1 <<20); // ADC Regular channel sequece length = 2 conversion
	// �̰�

	/* ADC_RegularChannelConfig *********************************************/
	ADC3->SMPR2 |= 0x07 << (3 * 1);	// ADC1_CH1 Sample TIme_480Cycles (3*Channel_1)
	ADC3->SQR3 |= 1 << (5 * (1 - 1));	// ADC1_CH1 << (5 * (Rank - 1)),  Rank = 1 (1������ ��ȯ: ��������)

	ADC3->SMPR2 |= 0x07 << (3 * 9);	//ADC1_CH0 Sample Time_480Cycles (3*Channel_0)
	ADC3->SQR3 |= (9 << (5 * (2 - 1)));//ADC1_CH0 << (5*(Rank-1)), Rank = 2 (2������ ��ȯ: �Ÿ�����)


	/* Enable DMA request after last transfer (Single-ADC mode) */
	ADC3->CR2 |= 0x00000200;	// DMA requests are issued as long as data are converted and DMA=1	
				// for single ADC mode
	/* Enable ADC1 DMA */
	ADC3->CR2 |= 0x00000100;	// DMA mode enabled  (DMA=1)
	ADC3->CR2 |= 0x00000001;	// Enable ADC3:  ADON=1

	NVIC->ISER[0] |= (1<<18);	// Enable ADC global Interrupt
}

void ADC_IRQHandler(void)
{
	ADC3->SR &= ~(1<<1);	// EOC flag clear
    Voltage[0] = ADC_value[0]*(16.5 * 10) / 4095;  
    Voltage[1] = ADC_value[1]*(16.5 * 10) / 4095;

	Distance[0] = Voltage[0]*5 + 3;
	Distance[1] = Voltage[0]*5 + 3;
	if (mode == 0){
    // ���� ���

		LCD_SetBrushColor(RGB_WHITE);
		LCD_DrawFillRect(20, 27, 98, 10);
		LCD_DrawFillRect(20, 40, 98, 10);

    	LCD_SetBrushColor(RGB_RED);
		LCD_DisplayChar(2,16,Voltage[0]/10 + 0x30);
		LCD_DisplayChar(2,17,Voltage[0]%10 + 0x30);
		LCD_DrawFillRect(20, 27, Distance[0], 10);
		LCD_SetBrushColor(RGB_GREEN);
		LCD_DrawFillRect(20, 40, Distance[1], 10);
		LCD_DisplayChar(3,16,Voltage[1]/10 + 0x30);
		LCD_DisplayChar(3,17,Voltage[1]%10 + 0x30);

		DR = (Voltage[0]+20)/20;
		LCD_DisplayChar(4,7,DR + 0x30);
		LCD_DisplayChar(4,8,'0');
	}
	sprintf(str,"%2dm ",Distance[0]);
    SerialSendString(str);

}


void DMAInit(void)
{
 	// DMA2 Stream0 channel0 configuration *************************************
	RCC->AHB1ENR |= (1<<22);		//DMA2 clock enable
	DMA2_Stream0->CR |= (2<<25);	//DMA2 Stream0 channel 0 selected

	// ADC1->DR(Peripheral) ==> ADC_vlaue(Memory)
	DMA2_Stream0->PAR |= (uint32_t)&ADC3->DR;	   //Peripheral address - ADC1->DR(Regular data) Address
	DMA2_Stream0->M0AR |= (uint32_t)&ADC_value; //Memory address - ADC_Value address 
	DMA2_Stream0->CR &= ~(3<<6);		  //Data transfer direction : Peripheral-to-memory (P=>M)
	DMA2_Stream0->NDTR = 2;			  //DMA_BufferSize = 2 (ADC_Value[2])

	DMA2_Stream0->CR &= ~(1<<9); 	//Peripheral increment mode  - Peripheral address pointer is fixed
	DMA2_Stream0->CR |= (1<<10);	//Memory increment mode - Memory address pointer is incremented after each data transferd 
	DMA2_Stream0->CR |= (1<<11);	//Peripheral data size - halfword(16bit)
	DMA2_Stream0->CR |= (1<<13);	//Memory data size - halfword(16bit)   
	DMA2_Stream0->CR |= (1<<8);	//Circular mode enabled   
	DMA2_Stream0->CR |= (2<<16);	//Priority level - High

	DMA2_Stream0->FCR &= ~(1<<2);	//DMA_FIFO_direct mode enabled
	DMA2_Stream0->FCR |= (1<<0);	//DMA_FIFO Threshold_HalfFull , Not used in direct mode

	DMA2_Stream0->CR &= ~(3<<23);	//Memory burst transfer configuration - single transfer
	DMA2_Stream0->CR &= ~(3<<21);	//Peripheral burst transfer configuration - single transfer  
	DMA2_Stream0->CR |= (1<<0);	//DMA2_Stream0 enabled
}

void TIMER1_Init(void)
{
// TIM1_CH2 (PE13) : 400ms �̺�Ʈ �߻�
// Clock Enable : GPIOE & TIMER1
	RCC->AHB1ENR	|= (1<<4);	// GPIOE Enable
	RCC->APB2ENR 	|= (1<<0);	// TIMER1 Enable 
    						
// PE11�� ��¼����ϰ� Alternate function(TIM1_CH2)���� ��� ���� 
	GPIOE->MODER 	|= (2<<2*13);	// PE13 Output Alternate function mode					
	GPIOE->OSPEEDR 	|= (3<<2*13);	// PE13 Output speed (100MHz High speed)
	GPIOE->OTYPER	&= ~(1<<13);	// PE13 Output type push-pull (reset state)
	GPIOE->AFR[1]	|= (1 <<4*(13-8)); 	// 0x00000001	(Connect TIM1 pins(PE13) to AF1
						// TIM1_CH2: AF1(TIM1..2)

	// Assign 'Interrupt Period' and 'Output Pulse Period'
	TIM1->PSC = 1680-1;	// Prescaler 84MHz/840 = 0.1MHz (10us)
	TIM1->ARR = 40000-1;	// Auto reload  : 10us * 40K = 400ms(period)

	// CR1 : Up counting
	TIM1->CR1 &= ~(1<<4);	// DIR=0(Up counter)(reset state)
	TIM1->CR1 &= ~(1<<1);	// UDIS=0(Update event Enabled): By one of following events
				//	- Counter Overflow/Underflow, 
				// 	- Setting the UG bit Set,
				//	- Update Generation through the slave mode controller 
	TIM1->CR1 &= ~(1<<2);	// URS=0(Update event source Selection): one of following events
				//	- Counter Overflow/Underflow, 
				// 	- Setting the UG bit Set,
				//	- Update Generation through the slave mode controller 
	TIM1->CR1 &= ~(1<<3);	// OPM=0(The counter is NOT stopped at update event) (reset state)
	TIM1->CR1 &= ~(1<<7);	// ARPE=0(ARR is NOT buffered) (reset state)
	TIM1->CR1 &= ~(3<<8); 	// CKD(Clock division)=00(reset state)
	TIM1->CR1 &= ~(3<<5); 	// CMS(Center-aligned mode Sel)=00 (Edge-aligned mode) (reset state)
				// Center-aligned mode: The counter counts Up and DOWN alternatively

	// Event & Interrup Enable : UI  
	TIM1->EGR |= (1<<0);    // UG: Update generation    

	////////////////////////////////
	// Disable Tim1 Update interrupt
    
	// Define the corresponding pin by 'Output'  
	TIM1->CCER |= (1<<8);	// CC3E=1: CC2 channel Output Enable
				// OC2(TIM1_CH2) Active: �ش����� ���� ��ȣ���
	TIM1->CCER &= ~(1<<9);	// CC3P=0: CC3 channel Output Polarity (OCPolarity_High : OC2���� �������� ���)  

	// 'Mode' Selection : Output mode, toggle  
	TIM1->CCMR2 &= ~(3<<0); // CC3S(CC3 channel) = '0b00' : Output 
	TIM1->CCMR2 &= ~(1<<3); // OC3P=0: Output Compare 2 preload disable
	TIM1->CCMR2 |= (3<<4);	// OC3M=0b011: Output Compare 2 Mode : toggle
				// OC2REF toggles when CNT = CCR2

 	TIM1->CCR3 = 30000;	// TIM1 CCR3 TIM1_Pulse
    TIM1->BDTR |= (1<<15); 
      
	TIM1->CR1 |= (1<<0);						
}

void TIMER14_PWM_Init(void)  //\BE\C6\C1\F7 TIM14_CH1 PF9
{  
// \B8\F0\C5\CD\C6޽\BA(PWM)\C7\C9:PF9(TIM14_CH1), \B8\F0\C5͹\E6\C7\E2(DIR)\C7\C9:?
// Clock Enable : GPIOF & TIMER14
	RCC->AHB1ENR	|= (1<<5);	// GPIOF Enable
	RCC->APB1ENR 	|= (1<<8);	// TIMER14 Enable 
    						
// PA0\C0\BB \C3\E2\B7¼\B3\C1\A4\C7ϰ\ED Alternate function(TIM14_CH1)\C0\B8\B7\CE \BB\E7\BF\EB \BC\B1\BE\F0 : PWM \C3\E2\B7\C2
	GPIOF->MODER 	|= (2<<18);	// PF9 Output Alternate function mode					
	GPIOF->OSPEEDR 	|= (3<<18);	// PF9 Output speed (100MHz High speed)
	GPIOF->OTYPER	&= ~(1<<18);	// PF9 Output type push-pull (reset state)
	GPIOF->AFR[1]	|= (9<<4); 	// 0x00000002	(Connect TIM14 pins(PF9) to AF9
					
    
// PA3\C0\BB GPIO  \C3\E2\B7¼\B3\C1\A4 : Dir (\B8\F0\C5͹\E6\C7\E2)  \C1\F6\B1\DD \C0\DF \B8𸣰ڳ\D7
//	GPIOF->MODER 	|= (1<<6);	// PF3 Output  mode					
//	GPIOF->OSPEEDR 	|= (1<<6);	// PF3 Output speed (25MHz High speed)
//	GPIOF->OTYPER	&= ~(1<<3);	// PF3 Output type push-pull (reset state)
          
// TIM14 Channel 1 : PWM 2 mode
	// Assign 'PWM Pulse Period'
	TIM14->PSC	= 420-1;	// Prescaler 84,000,000Hz/420 = 2000000(5us)  (1~65536)
	TIM14->ARR	= 80-1;	// Auto reload  (5us * 80 = 400us : PWM Period)
    
	// Define the corresponding pin by 'Output'  
	// CCER(Capture/Compare Enable Register) : Enable "Channel 1" 
	TIM14->CCER	|= (1<<0);	// CC1E=1: OC1(TIM5_CH1) Active(Capture/Compare 1 output enable)
	TIM14->CCER	&= ~(1<<1);	// CC1P=0: CC1 output Polarity High (OC1\C0\B8\B7\CE \B9\DD\C0\FC\BE\F8\C0\CC \C3\E2\B7\C2)

	// Duty Ratio  CCR1/80 = 10%
	TIM14->CCR1= 70;		// CCR1 value

	// 'Mode' Selection : Output mode, PWM 2
	// CCMR1(Capture/Compare Mode Register 1) : Setting the MODE of Ch1 or Ch2
	TIM14->CCMR1 	&= ~(3<<0); 	// CC1S(CC1 channel)='0b00' : Output 
	TIM14->CCMR1 	|= (1<<3); 	// OC1PE=1: Output Compare 1 preload Enable

	TIM14->CCMR1	|= (7<<4);	// OC1M: Output compare 1 mode: PWM 2 mode
	TIM14->CCMR1	|= (1<<7);	// OC1CE: Output compare 1 Clear enable

	// CR1 : Up counting & Counter TIM14 enable
	TIM14->CR1 	&= ~(1<<4);	// DIR: Countermode = Upcounter (reset state)
	TIM14->CR1 	&= ~(3<<8);	// CKD: Clock division = 1 (reset state)
	TIM14->CR1        &= ~(3<<5); // CMS(Center-aligned mode Sel): No(reset state)
	TIM14->CR1	|= (1<<7);	// ARPE: Auto-reload preload enable
	TIM14->CR1	|= (1<<0);	// CEN: Counter TIM14 enable
}

void TIMER4_PWM_Init(void)
{   
// TIM CH2 : PB7 (167\B9\F8 \C7\C9)
// Clock Enable : GPIOB & TIMER4
	RCC->AHB1ENR	|= (1<<1);	// GPIOB CLOCK Enable
	RCC->APB1ENR 	|= (1<<2);	// TIMER4 CLOCK Enable 
    						
// PB8\C0\BB \C3\E2\B7¼\B3\C1\A4\C7ϰ\ED Alternate function(TIM4_CH3)\C0\B8\B7\CE \BB\E7\BF\EB \BC\B1\BE\F0 : PWM \C3\E2\B7\C2
	GPIOB->MODER 	|= (2<<14);	// 0x00020000 PB8 Output Alternate function mode					
	GPIOB->OSPEEDR 	|= (3<<14);	// 0x00030000 PB8 Output speed (100MHz High speed)
	GPIOB->OTYPER	&= ~(1<<7);	// PB8 Output type push-pull (reset state)
	GPIOB->PUPDR	|= (1<<14);	// 0x00010000 PB8 Pull-up
 	GPIOB->AFR[0]	|= (2<<28);	// 0x00000002 (AFR[1].(3~0)=0b0010): Connect TIM4 pins(PB8) to AF2(TIM3..5)
    
// TIM4 Channel 3 : PWM 1 mode
	// Assign 'PWM Pulse Period'
	TIM4->PSC	= 840-1;	// Prescaler 84,000,000Hz/8400 = 100,000 Hz(0.01ms)  (1~65536)
	TIM4->ARR	= 40-1;	// Auto reload  (10us * 40 = 40us : PWM Period)

	// Setting CR1 : 0x0000 (Up counting)
	TIM4->CR1 &= ~(1<<4);	// DIR=0(Up counter)(reset state)
	TIM4->CR1 &= ~(1<<1);	// UDIS=0(Update event Enabled)
	TIM4->CR1 &= ~(1<<2);	// URS=0(Update event source Selection)g events
	TIM4->CR1 &= ~(1<<3);	// OPM=0(The counter is NOT stopped at update event) (reset state)
	TIM4->CR1 |= (1<<7);	// ARPE=1(ARR is buffered): ARR Preload Enable 
	TIM4->CR1 &= ~(3<<8); 	// CKD(Clock division)=00(reset state)
	TIM4->CR1 &= ~(3<<5); 	// CMS(Center-aligned mode Sel)=00 : Edge-aligned mode(reset state)
	   
	// Define the corresponding pin by 'Output'  
	TIM4->CCER |= (1<<4);	// CC2E=1: 'm'
	// Duty Ratio 
	TIM4->CCR3	= 10;		// CCR3 value


	// 'Mode' Selection : Output mode, toggle  
	TIM4->CCMR1 &= ~(3<<8); // CC2S(CC3 channel) = '0b00' : Output 
	TIM4->CCMR1 &= ~(1<<11); // OC2P=0: Output Compare 2 preload disable
	TIM4->CCMR1 |= (3<<12);	// OC2M=0b011: Output Compare 2 Mode : toggle
				// OC2REF toggles when CNT = CCR2

 	TIM4->CCR2 = 30000;	// TIM1 CCR2 TIM1_Pulse
    TIM4->BDTR |= (1<<15); //main output enable
	////////////////////////////////
	// Disable Tim1 CC3 interrupt
      
	TIM4->CR1 |= (1<<0);	// CEN: Enable the Tim5 Counter  
}


void _EXTI_Init(void)
{
	RCC->AHB1ENR |= 0x00000080;   // RCC_AHB1ENR GPIOH Enable
    RCC->APB2ENR |= 0x00004000;   // Enable System Configuration Controller Clock
	GPIOH->MODER    &= ~0xFFFF0000;   // GPIOH PIN8~PIN15 Input mode (reset state)             
   
    // EXTI14 \BC\B3\C1\A4
    SYSCFG->EXTICR[3] |= 0x0700;    // EXTI14\BF\A1   
	SYSCFG->EXTICR[3] |= 0x0007;	// EXTI12\BF\A1 
    EXTI->FTSR |= 0x005000;      // EXTI14, 11: Falling Trigger Enable
    EXTI->IMR  |= 0x005000;     // EXTI14, 11 \C0\CE\C5ͷ\B4Ʈ mask (Interrupt Enable) \BC\B3\C1\A4
      
    NVIC->ISER[1] |= (1 << (40-32));// 0x00000100
}

/* EXTI15 ISR */
void EXTI15_10_IRQHandler(void)      
{
     if(EXTI->PR & 0x4000)                   // EXTI14 Interrupt Pending(\B9߻\FD) \BF\A9\BA\CE?
    { 
      	EXTI->PR |= 0x4000;       // Pending bit Clear 
	  	LCD_DisplayChar(1,16,'M');
		mode = 0;
		//Fram_Write(50, 0);
    }     
	
	if(EXTI->PR & 0x1000)		// EXTI12 Interrupt Pending(\B9߻\FD) \BF\A9\BA\CE?
	{
		EXTI->PR |= 0x1000;		// Pending bit Clear (clear\B8\A6 \BE\C8\C7ϸ\E9 \C0\CE\C5ͷ\B4Ʈ \BC\F6\C7\E0\C8\C4 \B4ٽ\C3 \C0\CE\C5ͷ\B4Ʈ \B9߻\FD)
		StopMode();
		mode = 1;
		//Fram_Write(50, 1);
	}
}
    void DisplayTitle(void)
{
	LCD_Clear(RGB_WHITE);
	LCD_SetFont(&Gulim8);		//��Ʈ 
	LCD_SetBackColor(RGB_WHITE);	//���ڹ���
	LCD_SetTextColor(RGB_BLACK);	//���ڻ�
	LCD_DisplayText(0, 0, "LGW 2018132027");
	LCD_DisplayText(1, 0, "Tracking Car");
	LCD_DisplayText(2, 0, "D1");
	LCD_DisplayText(3, 0, "D2");
	LCD_DisplayText(4, 0, "SP(DR): % DIR(DR):");
	LCD_SetTextColor(RGB_BLUE);
	LCD_SetBrushColor(RGB_RED);
}

void USART1_Init(void)
{
	// USART1 : TX(PA9)
	RCC->AHB1ENR |= (1 << 0);	// RCC_AHB1ENR GPIOA Enable
	GPIOA->MODER |= (2 << 2 * 9);	// GPIOB PIN9 Output Alternate function mode					
	GPIOA->OSPEEDR |= (3 << 2 * 9);	// GPIOB PIN9 Output speed (100MHz Very High speed)
	GPIOA->AFR[1] |= (7 << 4);	// Connect GPIOA pin9 to AF7(USART1)

	// USART1 : RX(PA10)
	GPIOA->MODER |= (2 << 2 * 10);	// GPIOA PIN10 Output Alternate function mode
	GPIOA->OSPEEDR |= (3 << 2 * 10);	// GPIOA PIN10 Output speed (100MHz Very High speed
	GPIOA->AFR[1] |= (7 << 8);	// Connect GPIOA pin10 to AF7(USART1)

	RCC->APB2ENR |= (1 << 4);	// RCC_APB2ENR USART1 Enable

	USART_BRR_Configuration(19200); // USART Baud rate Configuration

	USART1->CR1 &= ~(1 << 12);	// USART_WordLength 8 Data bit
	USART1->CR1 &= ~(1 << 10);	// NO USART_Parity
	USART1->CR1 |= (1 << 2);	// 0x0004, USART_Mode_RX Enable
	USART1->CR1 |= (1 << 3);	// 0x0008, USART_Mode_Tx Enable
	USART1->CR2 &= ~(3 << 12);	// 0b00, USART_StopBits_1
	USART1->CR3 = 0x0000;	// No HardwareFlowControl, No DMA

	USART1->CR1 |= (1 << 5);	// 0x0020, RXNE interrupt Enable
	USART1->CR1 &= ~(1 << 7); // 0x0080, TXE interrupt Disable 

	NVIC->ISER[1] |= (1 << (37 - 32));// Enable Interrupt USART1 (NVIC 37��)
	USART1->CR1 |= (1 << 13);	//  0x2000, USART1 Enable
}

void SerialSendChar(uint8_t Ch) // 1���� ������ �Լ�
{
	while ((USART1->SR & USART_SR_TXE) == RESET); // USART_SR_TXE=(1<<7), �۽� ������ ���±��� ���

	USART1->DR = (Ch & 0x01FF);	// ���� (�ִ� 9bit �̹Ƿ� 0x01FF�� masking)
}

void SerialSendString(char* str) // �������� ������ �Լ�
{
	while (*str != '\0') // ���Ṯ�ڰ� ������ ������ ����, ���Ṯ�ڰ� �����Ŀ��� ������ �޸� ���� �߻����ɼ� ����.
	{
		SerialSendChar(*str);	// �����Ͱ� ����Ű�� ���� �����͸� �۽�
		str++; 			// ������ ��ġ ����
	}
}




void USART_BRR_Configuration(uint32_t USART_BaudRate)
{
	uint32_t tmpreg = 0x00;
	uint32_t APB2clock = 84000000;	//PCLK2_Frequency
	uint32_t integerdivider = 0x00;
	uint32_t fractionaldivider = 0x00;

	// Determine the integer part 
	if ((USART1->CR1 & USART_CR1_OVER8) != 0) // USART_CR1_OVER8=(1<<15)
	{                                         // USART1->CR1.OVER8 = 1 (8 oversampling)
		// Computing 'Integer part' when the oversampling mode is 8 Samples 
		integerdivider = ((25 * APB2clock) / (2 * USART_BaudRate));
	}
	else  // USART1->CR1.OVER8 = 0 (16 oversampling)
	{	// Computing 'Integer part' when the oversampling mode is 16 Samples 
		integerdivider = ((25 * APB2clock) / (4 * USART_BaudRate));
	}
	tmpreg = (integerdivider / 100) << 4;

	// Determine the fractional part 
	fractionaldivider = integerdivider - (100 * (tmpreg >> 4));

	// Implement the fractional part in the register 
	if ((USART1->CR1 & USART_CR1_OVER8) != 0)	// 8 oversampling
	{
		tmpreg |= (((fractionaldivider * 8) + 50) / 100) & (0x07);
	}
	else 			// 16 oversampling
	{
		tmpreg |= (((fractionaldivider * 16) + 50) / 100) & (0x0F);
	}

	// Write to USART BRR register
	USART1->BRR = (uint16_t)tmpreg;
}


void USART1_IRQHandler(void)
{
	// RX Buffer Full interrupt
	if ((USART1->SR & USART_SR_RXNE))		// USART_SR_RXNE=(1<<5) 
	{
		char ch;
		ch = (uint16_t)(USART1->DR & (uint16_t)0x01FF);	// ���ŵ� ���� ����
		LCD_DisplayChar(0, 16, ch); 	// LCD display
		if(ch == 'M'){
	  		LCD_DisplayChar(1,16,'M');
			mode = 0;
			//Fram_Write(50, 0);

        }
        else if(ch == 'S'){
            LCD_DisplayChar(1,16,'S');
			mode = 1;
			//Fram_Write(50, 1);
        }
	}
}





void StopMode(void){

	LCD_SetBrushColor(RGB_WHITE);
	LCD_DrawFillRect(20, 27, 98, 10);
	LCD_DrawFillRect(20, 40, 98, 10);

	LCD_SetBrushColor(RGB_GREEN);
	LCD_DrawFillRect(20, 40, 15, 10);
	LCD_DisplayChar(4,7,'0');
	LCD_DisplayChar(4,8,'0');
	LCD_DisplayChar(4,17,'0');
	LCD_DisplayChar(1,16,'S');
	LCD_DisplayChar(2,16,' ');
	LCD_DisplayChar(2,17,'0');
	LCD_DisplayChar(3,16,' ');
	LCD_DisplayChar(3,17,'1');
}

void DelayMS(unsigned short wMS)
{
	register unsigned short i;
	for (i=0; i<wMS; i++)
		DelayUS(1000);   // 1000us => 1ms
}

void DelayUS(unsigned short wUS)
{
	volatile int Dly = (int)wUS*17;
		for(; Dly; Dly--);
}

uint8_t key_flag = 0;
uint16_t KEY_Scan(void)	// input key SW0 - SW7 
{ 
	uint16_t key;
	key = GPIOH->IDR & 0xFF00;	// any key pressed ?
	if(key == 0xFF00)		// if no key, check key off
	{	if(key_flag == 0)
			return key;
		else
		{	DelayMS(10);
			key_flag = 0;
			return key;
		}
	}
	else				// if key input, check continuous key
	{	if(key_flag != 0)	// if continuous key, treat as no key input
			return 0xFF00;
		else			// if new key,delay for debounce
		{	key_flag = 1;
			DelayMS(10);
 			return key;
		}
	}
}
