//////////////////////////////////////////////////////////////////////
// HW3 USART 통신을 이용한 센서 모니터링
// 제출자: 2018132027 이건원
// 주요 내용 및 구현 방법
// ADC1:CH1(PA1, pin 41) : 키트 상의 가변저항
// ADC1:CH0(PA0, pin 40) : 외부 가변 저항
// ADC1:IN16 : 내부 온도 센서
// 변환시작: Timer1CH2 사용
// USART 통신을 통해 컴퓨터로부터 1, 2, 3 숫자를 받으면 값에 해당하는 온도 통신으로 전달함.
//////////////////////////////////////////////////////////////////////
#include "stm32f4xx.h"
#include "GLCD.h"


void DisplayTitle(void);

void _ADC_Init(void);
void TIMER1_Init(void);

void USART1_Init(void);
void USART_BRR_Configuration(uint32_t USART_BaudRate);

void SerialSendChar(uint8_t c);
void SerialSendString(char* s);

void DelayMS(unsigned short wMS);
void DelayUS(unsigned short wUS);
void DMAInit(void);

uint16_t Temperature[3], Voltage[3], bef[3];

uint8_t str[20];
uint16_t ADC_value[3];
static int MAX_LEN = 140;


int main(void)
{
	LCD_Init();	// LCD 구동 함수
	DelayMS(10);	// LCD구동 딜레이
 	DisplayTitle();	//LCD 초기화면구동 함수
        
	USART1_Init();
	_ADC_Init();
       TIMER1_Init();
       DMAInit();

	while(1)
	{
          if (ADC1->SR && ADC_SR_EOC == ADC_SR_EOC) // ADC1 EOC int
            {
		ADC1->SR &= ~(1<<1);	// EOC flag clear
                
                // 전압 계산
                Voltage[0] = ADC_value[0]*(3.3 * 10) / 4095;   
                Voltage[1] = ADC_value[1]*(3.3 * 10) / 4095;
                Voltage[2] = ADC_value[2]*(3.3 * 10) / 4095;
                
                // 온도 계산
		Temperature[0] = 3.5 * Voltage[0]/10 * Voltage[0]/10 + 1;
                Temperature[1] = 3.5 * Voltage[1]/10 * Voltage[1]/10 + 1;
                Temperature[2] = (Voltage[2]/10 -0.76) / 2.5 + 25;
                
                //온도 및 전압 디스플레이
                if (Temperature[0]/10 == 0){
                LCD_DisplayChar(1,9,' ');
                }
                else{
                    LCD_DisplayChar(1,9,Temperature[0]/10 + 0x30);
                }
		LCD_DisplayChar(1,10,Temperature[0]%10 + 0x30);
                  
                LCD_DisplayChar(1,13,Voltage[0]/ 10 + 0x30);
		LCD_DisplayChar(1,14,'.');                
                LCD_DisplayChar(1,15,Voltage[0] % 10 + 0x30);
		
                if (Temperature[1]/10 == 0){
                  LCD_DisplayChar(3,9,' ');
                }
                else{
                   LCD_DisplayChar(3,9,Temperature[1]/10 + 0x30);
                }
		LCD_DisplayChar(3,10,Temperature[1]%10 + 0x30);
  
                
                LCD_DisplayChar(3,13,Voltage[1]/ 10 + 0x30);
		LCD_DisplayChar(3,14,'.');                
                LCD_DisplayChar(3,15,Voltage[1] % 10 + 0x30);       
                
                if (Temperature[2]/10 == 0){
                LCD_DisplayChar(5,8,' ');
                }
                else{
                  LCD_DisplayChar(5,8,Temperature[2]/10 + 0x30);
                }
		LCD_DisplayChar(5,9,Temperature[2]%10 + 0x30);
                 
                LCD_DisplayChar(5,12,Voltage[2]/ 10 + 0x30);
		LCD_DisplayChar(5,13,'.');                
                LCD_DisplayChar(5,14,Voltage[2] % 10 + 0x30);   
                
              
                // 온도 박스 디스플레이
                if (bef[0] != Temperature[0]){
                
                LCD_SetBrushColor(RGB_WHITE);
                LCD_DrawFillRect(10, 36, MAX_LEN, 11);
               
                LCD_SetBrushColor(RGB_RED);
                LCD_DrawFillRect(10, 36, MAX_LEN/39 *Temperature[0], 11);  //x, y, 가로 세로
                }

                if (bef[1] != Temperature[1]){
                
                 LCD_SetBrushColor(RGB_WHITE);
                LCD_DrawFillRect(10, 69, MAX_LEN, 11);

                
                LCD_SetBrushColor(RGB_GREEN);
                LCD_DrawFillRect(10, 69, MAX_LEN/39 *Temperature[1], 11);
                }
                
                if (bef[2] != Temperature[2]){
                
                 LCD_SetBrushColor(RGB_WHITE);
                LCD_DrawFillRect(10, 103, MAX_LEN, 11);

                LCD_SetBrushColor(RGB_BLUE);
                LCD_DrawFillRect(10, 103, MAX_LEN/39 *Temperature[2], 11);
                }
                
                // 이전과 온도가 바꼈을 때만 업데이트하기 위해 이전 온도 저장
                bef[0] = Temperature[0];
                bef[1] = Temperature[1];
                bef[2] = Temperature[2];   	 	
                //Starts conversion of regular channels
		ADC1->CR2 |= ADC_CR2_SWSTART; 	// 0x40000000 (1<<30)
          }
 	}
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

	NVIC->ISER[1] |= (1 << (37 - 32));// Enable Interrupt USART1 (NVIC 37번)
	USART1->CR1 |= (1 << 13);	//  0x2000, USART1 Enable
}

void SerialSendChar(uint8_t Ch) // 1문자 보내기 함수
{
	while ((USART1->SR & USART_SR_TXE) == RESET); // USART_SR_TXE=(1<<7), 송신 가능한 상태까지 대기

	USART1->DR = (Ch & 0x01FF);	// 전송 (최대 9bit 이므로 0x01FF과 masking)
}

void SerialSendString(char* str) // 여러문자 보내기 함수
{
	while (*str != '\0') // 종결문자가 나오기 전까지 구동, 종결문자가 나온후에도 구동시 메모리 오류 발생가능성 있음.
	{
		SerialSendChar(*str);	// 포인터가 가르키는 곳의 데이터를 송신
		str++; 			// 포인터 수치 증가
	}
}

// Baud rate  
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
		ch = (uint16_t)(USART1->DR & (uint16_t)0x01FF);	// 수신된 문자 저장
		LCD_DisplayChar(0, 16, ch); 	// LCD display
		if(ch == '1'){
                  sprintf(str,"%2d",Temperature[0]);
                  SerialSendString(str);
                }
                if(ch == '2'){
                  sprintf(str,"%2d",Temperature[1]);
                  SerialSendString(str);
                }
                if(ch == '3'){
                  sprintf(str,"%2d",Temperature[2]);
                  SerialSendString(str);
                }
	}
}



void _ADC_Init(void)
{
	// ADC1_IN1: (PA1)
	// ADC1_IN8(PB0)
       // ADC1_IN16
	/* 1st Analog signal */
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;  	// RCC_AHB1ENR GPIOA Enable
	GPIOA->MODER |= 0x0C;	// GPIOA PIN1(PA1) 가변저항 : Analog mode

	/* 2nd Analog signal */
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;  // RCC_AHB1ENR GPIOA Enable
	GPIOB->MODER |= 0x03;	// GPIOA PIN0(PB0) 

	/* ADC Common Init **********************************************************/
	RCC->APB2ENR |= (1<<8);	// RCC_APB2ENR ADC1 Enable

	ADC->CCR &= ~0X0000001F;// ADC_Mode_Independent
	ADC->CCR |= 0x00010000;	// ADC_Prescaler_Div4 (ADC MAX Clock 36Mhz, 84Mhz(APB2)/4 = 21Mhz
    ADC->CCR |= (1<<23);

	/* ADC1 Init ****************************************************************/
	ADC1->CR1 &= ~(3 << 24);	// RES[1:0]=0b00 : 12bit Resolution
	ADC1->CR1 |= 0x00000100;	// ADC_ScanCovMode Enable (SCAN=1)
	ADC1->CR2 &= ~0x00000002;	// ADC_ContinuousConvMode DISABLE (CONT=0)
	ADC1->CR2 |= (1 << 24);		// EXTSEL[3:0]= 0b0001: Timer1_CH2 clock

    ADC1->CR2 |= (3<<28);
	ADC1->CR2 &= ~(1 << 11);	// ALIGN=0: ADC_DataAlign_Right
	ADC1->CR2 &= ~(1 << 10);	// EOCS=0: The EOC bit is set at the end of each sequence of regular conversions

	ADC1->SQR1 |= (2 <<20); // ADC Regular channel sequece length = 3 conversion
	// 이거

	/* ADC_RegularChannelConfig *********************************************/
	ADC1->SMPR2 |= 0x07 << (3 * 1);	// ADC1_CH1 Sample TIme_480Cycles (3*Channel_1)
	ADC1->SQR3 |= 0x01 << (5 * (1 - 1));	// ADC1_CH1 << (5 * (Rank - 1)),  Rank = 1 (1순위로 변환: 가변저항)

	ADC1->SMPR2 |= 0x07 << (3 * 8);	//ADC1_CH0 Sample Time_480Cycles (3*Channel_0)
	ADC1->SQR3 |= (0x08 << (5 * (2 - 1)));//ADC1_CH0 << (5*(Rank-1)), Rank = 2 (2순위로 변환: 거리센서)
	
  	ADC1->SMPR1 |= 0x07 << (3 * (16-10));	
	ADC1->SQR3 |= (0x10 << (5 * (3 - 1)));   
        // 이거

	/* Enable DMA request after last transfer (Single-ADC mode) */
	ADC1->CR2 |= 0x00000200;	// DMA requests are issued as long as data are converted and DMA=1	
				// for single ADC mode
	/* Enable ADC1 DMA */
	ADC1->CR2 |= 0x00000100;	// DMA mode enabled  (DMA=1)
	ADC1->CR2 |= 0x00000001;	// Enable ADC1:  ADON=1

        ADC1->CR2 |= ADC_CR2_SWSTART; // 0x40000000 (1<<30)  
}

void DMAInit(void)
{
	// DMA2 Stream0 channel0 configuration *************************************
	RCC->AHB1ENR |= (1 << 22);		//DMA2 clock enable
	DMA2_Stream0->CR &= ~(7 << 25);	//DMA2 Stream0 channel 0 selected

	// ADC1->DR(Peripheral) ==> ADC_vlaue(Memory)
	DMA2_Stream0->PAR |= (uint32_t)&ADC1->DR;	   //Peripheral address - ADC1->DR(Regular data) Address
	DMA2_Stream0->M0AR |= (uint32_t)&ADC_value; //Memory address - ADC_Value address 
	DMA2_Stream0->CR &= ~(3 << 6);		  //Data transfer direction : Peripheral-to-memory (P=>M)
	DMA2_Stream0->NDTR = 3;			  //DMA_BufferSize = 3 (ADC_Value[3])

	DMA2_Stream0->CR &= ~(1 << 9); 	//Peripheral increment mode  - Peripheral address pointer is fixed
	DMA2_Stream0->CR |= (1 << 10);	//Memory increment mode - Memory address pointer is incremented after each data transferd 
	DMA2_Stream0->CR |= (1 << 11);	//Peripheral data size - halfword(16bit)
	DMA2_Stream0->CR |= (1 << 13);	//Memory data size - halfword(16bit)   
	DMA2_Stream0->CR |= (1 << 8);	//Circular mode enabled   
	DMA2_Stream0->CR |= (2 << 16);	//Priority level - High

	DMA2_Stream0->FCR &= ~(1 << 2);	//DMA_FIFO_direct mode enabled
	DMA2_Stream0->FCR |= (1 << 0);	//DMA_FIFO Threshold_HalfFull , Not used in direct mode

	DMA2_Stream0->CR &= ~(3 << 23);	//Memory burst transfer configuration - single transfer
	DMA2_Stream0->CR &= ~(3 << 21);	//Peripheral burst transfer configuration - single transfer  
	DMA2_Stream0->CR |= (1 << 0);	//DMA2_Stream0 enabled
}

void TIMER1_Init(void)
{
// TIM1_CH2 (PE11) : 400ms 이벤트 발생
// Clock Enable : GPIOE & TIMER1
	RCC->AHB1ENR	|= (1<<4);	// GPIOE Enable
	RCC->APB2ENR 	|= (1<<0);	// TIMER1 Enable 
    						
// PE11을 출력설정하고 Alternate function(TIM1_CH2)으로 사용 선언 
	GPIOE->MODER 	|= (2<<2*11);	// PE11 Output Alternate function mode					
	GPIOE->OSPEEDR 	|= (3<<2*11);	// PE11 Output speed (100MHz High speed)
	GPIOE->OTYPER	&= ~(1<<11);	// PE11 Output type push-pull (reset state)
	GPIOE->AFR[1]	|= (1 <<4*(11-8)); 	// 0x00000200	(AFR[0].(11~8)=0b0010): Connect TIM5 pins(PA2) to AF2(TIM3..5)
					// PA2 ==> TIM5_CH3

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
	TIM1->CCER |= (1<<4);	// CC2E=1: CC2 channel Output Enable
				// OC2(TIM1_CH2) Active: 해당핀을 통해 신호출력
	TIM1->CCER &= ~(1<<5);	// CC2P=0: CC3 channel Output Polarity (OCPolarity_High : OC2으로 반전없이 출력)  

	// 'Mode' Selection : Output mode, toggle  
	TIM1->CCMR1 &= ~(3<<8); // CC2S(CC3 channel) = '0b00' : Output 
	TIM1->CCMR1 &= ~(1<<11); // OC2P=0: Output Compare 2 preload disable
	TIM1->CCMR1 |= (3<<12);	// OC2M=0b011: Output Compare 2 Mode : toggle
				// OC2REF toggles when CNT = CCR2

 	TIM1->CCR2 = 30000;	// TIM1 CCR2 TIM1_Pulse
        TIM1->BDTR |= (1<<15); //main output enable
	////////////////////////////////
	// Disable Tim1 CC3 interrupt
      
	TIM1->CR1 |= (1<<0);	// CEN: Enable the Tim5 Counter  					
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

void DisplayTitle(void)
{
	LCD_Clear(RGB_WHITE);
	LCD_SetFont(&Gulim10);		//폰트 
	LCD_SetBackColor(RGB_WHITE);	//글자배경색
	LCD_SetTextColor(RGB_BLACK);	//글자색
	LCD_DisplayText(0, 0, "LGW 2018132027");
	LCD_SetTextColor(RGB_BLUE);
	LCD_DisplayText(1, 0, "EXR1 TMP:  C(   V)");
	LCD_DisplayText(3, 0, "EXT2 TMP:  C(   V)");
	LCD_DisplayText(5, 0, "INT TMP:  C(   V)");
	LCD_SetTextColor(RGB_RED);
	LCD_SetBrushColor(RGB_RED);
	LCD_DrawFillRect(10, 36, MAX_LEN/10, 11);  //x, y, 가로 세로
        LCD_SetBrushColor(RGB_GREEN);
	LCD_DrawFillRect(10, 69, MAX_LEN/39, 11);
        LCD_SetBrushColor(RGB_BLUE);
	LCD_DrawFillRect(10, 103, MAX_LEN, 11);
        LCD_SetTextColor(RGB_RED);
	// -가로길이(X): LCD X축 총길이(LCD 좌우모서리에서 10pixel 간격은 제외)
	// -세로길이(Y) : 폰트(글자)크기 / 2 <= X <= 폰트크기
}
