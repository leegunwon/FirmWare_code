/////////////////////////////////////////////////////////////////////////
// HW2: ���� ������ �޽� �߻�
// ������: 2018132027 �̰ǿ�
// �ֿ� ���� �� ���� ���
//  COUNTER(up counter) : TIMER4 (100ms), TIMER3 (20ms)
//  step ��ġ ��ɰ� : TIM5_CH2 (count�� ����: 0~4)
//  DC ��ġ ��ɰ� :  TIM8_CH1 (count�� ����: 0~7)
//  DC motor ���� �޽�: TIM14_CH1 (400us)
//  step motor �޽� : TIMER1_CH3 (1s)
/////////////////////////////////////////////////////////////////////////
#include "stm32f4xx.h"
#include "GLCD.h"


void DisplayInitScreen(void);

void TIMER14_PWM_Init(void);
void TIMER8_COUNTER_Init(void);
void TIMER5_COUNTER_Init(void);
void TIMER4_Init(void);
void TIMER1_OC_Init(void);
void TIMER3_Init(void);

void DelayMS(unsigned short wMS);
void DelayUS(unsigned short wUS);

int Flag = 0;
int main(void)
{
	LCD_Init();		// LCD ���� �Լ�
	DelayMS(10);		// LCD���� ������
	DisplayInitScreen();	// LCD �ʱ�ȭ�鱸�� �Լ�

	TIMER14_PWM_Init();
  	TIMER8_COUNTER_Init();	// TIM8 CH1 init (Count mode)
	TIMER5_COUNTER_Init();	// TIM4 Init (Count mode)
	TIMER4_Init();
        TIMER3_Init();
	TIMER1_OC_Init();
        
        while(1){

        }
}

void TIMER5_COUNTER_Init(void) // TIM5_CH2 up count ��� 
{  
// Encoder �Է�(Counting) ��: PH11 (TIM5_CH2)
// Clock Enable : GPIOH & TIMER5
	RCC->AHB1ENR	|= (1<<7);	// 0x80, GPIOH Enable
	RCC->APB1ENR 	|= (1<<3);	// 0x08, TIMER4 Enable 

// PH11: TIM5_CH2
// PH11�� �Է¼����ϰ� Alternate function(TIM5_CH2)���� ��� ����
	GPIOH->MODER 	|= (2<<22);	// 0x00000000(MODER.(25,24)=0b10), GPIOH PIN11 intput Alternate function mode 					
	GPIOH->OSPEEDR 	|= (2<<22);	// 0x02000000(OSPEEDER.(25,24)=0b11), GPIOH PIN11 Output speed (50MHz High speed)
	GPIOH->PUPDR	&= ~(3<<22); 	// GPIOH PIN11 NO Pull-up
  					// PH11 ==> TIM5_CH2
	GPIOH->AFR[1]	|= (2<<12);	// 0x00020000(AFR[1].(19~16)=0b0010): Connect TIM5 pins(PH11) to AF2(TIM3..5)
  
// Time base Mode
	// Setting CR1 : 0x0000 
	TIM5->CR1 &= ~(1<<4);	// DIR=0(Up counter)(reset state)
	TIM5->CR1 &= ~(1<<1);	// UDIS=0(Update event Enabled)
	TIM5->CR1 &= ~(1<<2);	// URS=0(Update event source Selection)
	TIM5->CR1 &= ~(1<<3);	// OPM=0(The counter is NOT stopped at update event) (reset state)
	TIM5->CR1 |=  (1<<7);	// ARPE=1(ARR Preload Enable)
	TIM5->CR1 &= ~(3<<8); 	// CKD(Clock division)=00(reset state)
	TIM5->CR1 &= ~(3<<5); 	// CMS(Center-aligned mode Sel)=00 :Edge-aligned mode(reset state)

	// PSC, ARR
	TIM5->PSC = 1-1;	// Prescaler=1
	TIM5->ARR = 5-1;	// Auto reload  :  count�� ����: 0~4
        
	// Update(Clear) the Counter
	TIM5->EGR |= (1<<0);    // UG=1, REG's Update (CNT clear) 

// External Clock Mode 1
	// CCMR1(Capture/Compare Mode Register 1) : Setting the MODE of Ch2
	TIM5->CCMR1 |= (1<<8); 	// CC1S(CC1 channel) = '0b01' : Input 
	TIM5->CCMR1 &= ~(15<<12); // IC1F='0b0000: No Input Filter 
				
	// CCER(Capture/Compare Enable Register) : Enable "Channel 2" 
	TIM5->CCER &= ~(1<<4);	// CC1E=0: Capture Disable
	// TI1FP1 Inverting / Falling Edge  
	TIM5->CCER |= (1<<5);	// CC1P=1 Falling Trigger 
	TIM5->CCER &= ~(1<<7);	// CC1NP=0   

	// SMCR(Slave Mode Control Reg.) : External Clock Enable
	TIM5->SMCR |= (6<<4);	// TS(Trigger Selection)=0b110 :TI1FP2(Filtered Timer Input 1 ��½�ȣ)
	TIM5->SMCR |= (7<<0);	// SMS(Slave Mode Selection)=0b111 : External Clock Mode 1

	TIM5->CR1 |= (1<<0);	// CEN: Enable the TIM5 Counter  	
}

void TIMER1_OC_Init(void) //TIM1_CH3 CC 
{
	// PE13: TIM1_CH3
	// PE13�� ��¼����ϰ� Alternate function(TIM1_CH3)���� ��� ����
	RCC->AHB1ENR   |= (1<<4);   // RCC_AHB1ENR GPIOE Enable
	GPIOE->MODER   |= (2<<2*13);   // GPIOE PIN13 Output Alternate function mode               
	GPIOE->OSPEEDR |= (3<<2*13);   // GPIOE PIN13 Output speed (100MHz High speed)
	GPIOE->OTYPER  = 0x00000000;   // GPIOE PIN13 Output type push-pull (reset state)
	GPIOE->PUPDR   |= (1<<2*13);   // GPIOE PIN13 Pull-up
	GPIOE->AFR[1]  |= (1<<4*(13-8)); // Connect TIM1 pins(PE13) to AF1(TIM1/2)

	// PE14 : Step Driver Direction (GPIO)
	GPIOE->MODER   |= (1<<(2*14));  
	GPIOE->OSPEEDR |= (3<<(2*14));
	GPIOE->PUPDR   |= (1<<(2*14));  
	GPIOE->ODR     |= (1<<14); 

	// PE15 : Step Driver Reset (GPIO)
	GPIOE->MODER   |= (1<<(2*15));  
	GPIOE->OSPEEDR |= (3<<(2*15));
	GPIOE->PUPDR   |= (1<<(2*15));  
	GPIOE->ODR     |= (1<<15); 
   
	// Timerbase Mode
	RCC->APB2ENR   |= 0x01;// RCC_APB1ENR TIMER1 Enable

	TIM1->PSC = 8400 - 1;	// Prescaler=8400, 168MHz/8400 = 20KHz (50us)
	TIM1->ARR = 20000 - 1;	// Auto reload  : 50us * 20K = 1s(period) : ���ͷ�Ʈ�ֱ⳪ ��½�ȣ�� �ֱ� ����
	
	TIM1->CR1 &= ~(1<<4);   // DIR: Countermode = Upcounter (reset state)
	TIM1->CR1 &= ~(3<<8);   // CKD: Clock division = 1 (reset state)
	TIM1->CR1 &= ~(3<<5);    // CMS(Center-aligned mode Sel): No(reset state)

	TIM1->EGR |= (1<<0);    // UG: Update generation 
    
	// Output/Compare Mode
	TIM1->CCER &= ~(1<<4*(3-1));   // CC3E: OC3 Active 
	TIM1->CCER |= (1<<(4*(3-1)+1));  // CC3P: OCPolarity_Active Low
//	TIM1->CCER &= ~(1<<(4*(3-1)+1));  // CC3P: OCPolarity_Active High

	TIM1->CCR3 = 100;   // TIM1_Pulse

	TIM1->BDTR |= (1<<15);  // main output enable
   
	TIM1->CCMR2 &= ~(3<<8*0); // CC3S(CC1channel): Output 
	TIM1->CCMR2 &= ~(1<<3); // OC3PE: Output Compare 3 preload disable
	TIM1->CCMR2 |= (3<<4);   // OC3M: Output Compare 3 Mode : toggle

	TIM1->CR1 &= ~(1<<7);   // ARPE: Auto reload preload disable
	TIM1->DIER |= (1<<3);   // CC3IE: Enable the Tim1 CC3 interrupt
   
	NVIC->ISER[0] |= (1<<27); // TIM1_CC
	TIM1->CR1 &= ~(1<<0);   // CEN: Disable the Tim1 Counter 
}

int INT_COUNT = 0;
void TIM1_CC_IRQHandler(void)      //RESET: 0
{
	if ((TIM1->SR & 0x08) != RESET)	// CC3 interrupt flag 
	{
          TIM1->SR &= ~0x08;	// CC3 Interrupt Claer
          if(Flag ==1){
		INT_COUNT++;
		if (INT_COUNT >= 4*(TIM5->CNT))  // ��� �޽��� ���� 
		{
 			TIM1->CCER &= ~(1<<4*(3-1));// CC3E Disable 
			TIM1->CR1 &= ~(1<<0); // TIM1 Disable
			INT_COUNT= 0;
                        Flag = 0;
		}

          }
	}
}


void TIMER4_Init(void) // TIM4 up counting mode
{
	RCC->APB1ENR |= 0x04;	// RCC_APB1ENR TIMER4 Enable

	// Setting CR1 : 0x0000 
	TIM4->CR1 &= ~(1 << 4);  // DIR=0(Up counter)(reset state)
	TIM4->CR1 &= ~(1 << 1);	// UDIS=0(Update event Enabled): By one of following events
							//  Counter Overflow/Underflow, 
							//  Setting the UG bit Set,
							//  Update Generation through the slave mode controller 
							// UDIS=1 : Only Update event Enabled by  Counter Overflow/Underflow,
	TIM4->CR1 &= ~(1 << 2);	// URS=0(Update Request Source  Selection):  By one of following events
							//	Counter Overflow/Underflow, 
							// Setting the UG bit Set,
							//	Update Generation through the slave mode controller 
							// URS=1 : Only Update Interrupt generated  By  Counter Overflow/Underflow,
	TIM4->CR1 &= ~(1 << 3);	// OPM=0(The counter is NOT stopped at update event) (reset state)
	TIM4->CR1 &= ~(1 << 7);	// ARPE=0(ARR is NOT buffered) (reset state)
	TIM4->CR1 &= ~(3 << 8); 	// CKD(Clock division)=00(reset state)
	TIM4->CR1 &= ~(3 << 5); 	// CMS(Center-aligned mode Sel)=00 (Edge-aligned mode) (reset state)
							// Center-aligned mode: The counter counts UP and DOWN alternatively


	// Deciding the Period
	TIM4->PSC = 8400 - 1;	// Prescaler 84,000,000Hz/8400 = 10,000 Hz (0.1ms)  (1~65536)
	TIM4->ARR = 1000 - 1;	// Auto reload  0.1ms * 1000 = 100ms

	// Clear the Counter
	TIM4->EGR |= (1 << 0);	// UG(Update generation)=1 
						// Re-initialize the counter(CNT=0) & generates an update of registers   

	// Setting an UI(UEV) Interrupt 
	NVIC->ISER[0] |= (1 << 30); // Enable Timer4 global Interrupt
	TIM4->DIER |= (1 << 0);	// Enable the TIM4 Update interrupt

	TIM4->CR1 |= (1 << 0);	// Enable the TIM4 Counter (clock enable)   
}

int BEF = 0;

void TIM4_IRQHandler(void)  	// 100ms Interrupt
{

	TIM4->SR &= ~(1 << 0);	// Interrupt flag Clear
	LCD_DisplayChar(4, 9, 0x30+(TIM5->CNT));   // TIM5 count  ���
        if(TIM5->CNT != BEF){
          Flag = 1;    
          TIM1->CCER |= (1 << 8);	// CC3E=1: CC3 channel Output Enable
          TIM1->CR1 |= (1<<0);	// TIM1 Enable
        } 
        BEF = TIM5->CNT;  // ���� ī��Ʈ ����
        
        
}

void TIMER8_COUNTER_Init(void)  //���� TIM8_CH1 center aligned mode 
{
	// Encoder �Է�(Counting) ��: PI5 (TIM8_CH1)
	// Clock Enable : GPIOI & TIMER8
	RCC->AHB1ENR |= (1 << 8);	// 0x100, GPIOI Enable
	RCC->APB2ENR |= (1 << 1);	// 0x04, TIMER8 Enable 

// PI5: TIM4_CH1
// PI5�� �Է¼����ϰ� Alternate function(TIM8_CH1)���� ��� ����
	GPIOI->MODER |= (2 << 10);	// 0x00000000(MODER.(25,24)=0b10), GPIOI PIN5 intput Alternate function mode 					
	GPIOI->OSPEEDR |= (2 << 10);	// 0x02000000(OSPEEDER.(25,24)=0b11), GPIOD PIN5 Output speed (50MHz High speed)
	GPIOI->PUPDR &= ~(3 << 10); 	// GPIOI PIN5 NO Pull-up
					// PI5 ==> TIM8_CH1
	GPIOI->AFR[0] |= (3 << 20);	// Connect TIM8 pins(PI5) to AF3

// Time base Mode
	// Setting CR1 : 0x0000 
	TIM8->CR1 &= ~(1 << 4);	// DIR=0(Up counter)(reset state)
	TIM8->CR1 &= ~(1 << 1);	// UDIS=0(Update event Enabled)
	TIM8->CR1 &= ~(1 << 2);	// URS=0(Update event source Selection)
	TIM8->CR1 &= ~(1 << 3);	// OPM=0(The counter is NOT stopped at update event) (reset state)
	TIM8->CR1 |= (1 << 7);	// ARPE=1(ARR Preload Enable)
	TIM8->CR1 &= ~(3 << 8); 	// CKD(Clock division)=00(reset state)
	TIM8->CR1 |= (1 << 5); 	// CMS(Center-aligned mode Sel)=00 :Edge-aligned mode(reset state)

	// PSC, ARR
	TIM8->PSC = 1 - 1;	// Prescaler=1
	TIM8->ARR = 8 - 1;	// Auto reload  :  count�� ����: 0~7

	// Update(Clear) the Counter
	TIM8->EGR |= (1 << 0);    // UG=1, REG's Update (CNT clear) 

// External Clock Mode 1
	// CCMR1(Capture/Compare Mode Register 1) : Setting the MODE of Ch1 or Ch2
	TIM8->CCMR1 |= (1 << 0); 	// CC1S(CC1 channel) = '0b01' : Input 
	TIM8->CCMR1 &= ~(15 << 4); // IC1F='0b0000: No Input Filter 

	// CCER(Capture/Compare Enable Register) : Enable "Channel 1" 
	TIM8->CCER &= ~(1 << 0);	// CC1E=0: Capture Disable
	// TI1FP1 NonInverting / Rising Edge  
	TIM8->CCER &= ~(1 << 1);	// CC1P=0 
	TIM8->CCER &= ~(1 << 3);	// CC1NP=0   

	// SMCR(Slave Mode Control Reg.) : External Clock Enable
	TIM8->SMCR |= (5 << 4);	// TS(Trigger Selection)=0b101 :TI1FP1(Filtered Timer Input 1 ��½�ȣ)
	TIM8->SMCR |= (7 << 0);	// SMS(Slave Mode Selection)=0b111 : External Clock Mode 1

	TIM8->CR1 |= (1 << 0);	// CEN: Enable the Tim4 Counter  	
}

void TIMER14_PWM_Init(void)  //���� TIM14_CH1 PF9
{  
// �����޽�(PWM)��:PF9(TIM14_CH1), ���͹���(DIR)��:?
// Clock Enable : GPIOF & TIMER14
	RCC->AHB1ENR	|= (1<<5);	// GPIOF Enable
	RCC->APB1ENR 	|= (1<<8);	// TIMER14 Enable 
    						
// PA0�� ��¼����ϰ� Alternate function(TIM14_CH1)���� ��� ���� : PWM ���
	GPIOF->MODER 	|= (2<<18);	// PF9 Output Alternate function mode					
	GPIOF->OSPEEDR 	|= (3<<18);	// PF9 Output speed (100MHz High speed)
	GPIOF->OTYPER	&= ~(1<<18);	// PF9 Output type push-pull (reset state)
	GPIOF->AFR[1]	|= (9<<4); 	// 0x00000002	(Connect TIM14 pins(PF9) to AF9
					
    
// PA3�� GPIO  ��¼��� : Dir (���͹���)  ���� �� �𸣰ڳ�
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
	TIM14->CCER	&= ~(1<<1);	// CC1P=0: CC1 output Polarity High (OC1���� �������� ���)

	// Duty Ratio  CCR1/80 = 10%
	TIM14->CCR1= 80;		// CCR1 value

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

void TIMER3_Init(void)
{
	RCC->APB1ENR |= 0x02;	// RCC_APB1ENR TIMER3 Enable

	// Setting CR1 : 0x0000 
	TIM3->CR1 &= ~(1 << 4);  // DIR=0(Up counter)(reset state)
	TIM3->CR1 &= ~(1 << 1);	// UDIS=0(Update event Enabled): By one of following events
							//  Counter Overflow/Underflow, 
							//  Setting the UG bit Set,
							//  Update Generation through the slave mode controller 
							// UDIS=1 : Only Update event Enabled by  Counter Overflow/Underflow,
	TIM3->CR1 &= ~(1 << 2);	// URS=0(Update Request Source  Selection):  By one of following events
							//	Counter Overflow/Underflow, 
							// Setting the UG bit Set,
							//	Update Generation through the slave mode controller 
							// URS=1 : Only Update Interrupt generated  By  Counter Overflow/Underflow,
	TIM3->CR1 &= ~(1 << 3);	// OPM=0(The counter is NOT stopped at update event) (reset state)
	TIM3->CR1 &= ~(1 << 7);	// ARPE=0(ARR is NOT buffered) (reset state)
	TIM3->CR1 &= ~(3 << 8); 	// CKD(Clock division)=00(reset state)
	TIM3->CR1 &= ~(3 << 5); 	// CMS(Center-aligned mode Sel)=00 (Edge-aligned mode) (reset state)
							// Center-aligned mode: The counter counts UP and DOWN alternatively


	// Deciding the Period
	TIM3->PSC = 8400 - 1;	// Prescaler 84,000,000Hz/8400 = 10,000 Hz (0.1ms)  (1~65536)
	TIM3->ARR = 200 - 1;	// Auto reload  0.1ms * 200 = 20ms

	// Clear the Counter
	TIM3->EGR |= (1 << 0);	// UG(Update generation)=1 
						// Re-initialize the counter(CNT=0) & generates an update of registers   

	// Setting an UI(UEV) Interrupt 
	NVIC->ISER[0] |= (1 << 29); // Enable Timer3 global Interrupt
	TIM3->DIER |= (1 << 0);	// Enable the Tim3 Update interrupt

	TIM3->CR1 |= (1 << 0);	// Enable the Tim3 Counter (clock enable)   
}

void TIM3_IRQHandler(void)  	// 1ms Interrupt
{

	TIM3->SR &= ~(1 << 0);	// Interrupt flag Clear
        TIM14->CCR1 = 8* (10-TIM8->CNT); // TIM8 duty ratio�� ����
	LCD_DisplayChar(6, 7, 0x30+TIM8->CNT);  // TIM8 count ���

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

void DisplayInitScreen(void)
{
	LCD_Clear(RGB_YELLOW);		// ȭ�� Ŭ����
	LCD_SetFont(&Gulim8);		// ��Ʈ : ���� 8
	LCD_SetBackColor(RGB_BLACK);	// ���ڹ��� : BLACK
	LCD_SetTextColor(RGB_WHITE);	// ���ڻ� : WHITE

	LCD_DisplayText(0,0,"Motor Control    ");  // Title
	LCD_DisplayText(1,0,"2018132027 LGW   ");  // Title
	LCD_SetBackColor(RGB_YELLOW);	//���ڹ���
	LCD_SetTextColor(RGB_BLUE);	// ���ڻ� : BLUE
	LCD_DisplayText(3,0,"Step Motor");
	LCD_DisplayText(5,0,"DC Motor");
	LCD_SetTextColor(RGB_BLACK);	// ���ڻ� : BLACK
	LCD_DisplayText(4,0, "Position:");
	LCD_DisplayText(6,0,"Torque:");
	LCD_SetTextColor(RGB_RED);	// ���ڻ� : RED
	LCD_DisplayChar(4, 9, '0');
	LCD_DisplayChar(6, 7, '0');
}