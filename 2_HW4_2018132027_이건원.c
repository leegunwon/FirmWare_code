//////////////////////////////////////////////////////////////////////////
//  HW4. 가속도값을 이용한 Ball game
//  학과 : 메카트로닉스학과
//  학번 : 2018132027
//  이름 : 이건원
//  NSS pin:  PA8 (PA4(SPI1_CS) 대신에 사용)
//  SCK pin:  PA5 (SPI1_SCK)
//  MOSI pin: PA7 (SPI1_MOSI)
//  MISO pin: PA6 (SPI1_MISO)
//  TIM10 CC인터럽트 이용하여 200ms마다 센서 데이터 수신
// 가속도센서(LIS2HH12, Slave mode) X,Y,Z 값을 이용하여 공을 움직임.
//////////////////////////////////////////////////////////////////////////
#include "stm32f4xx.h"
#include "GLCD.h"
#include "ACC.h"

void DisplayTitle(void);
void _GPIO_Init(void);
void SPI1_Init(void);
void TIMER3_Init(void);
void Display_Process(int16 *pBuf);

void DelayMS(unsigned short wMS);
void DelayUS(unsigned short wUS);
void display_ball(int, int);

//// void SPI1_Process(UINT16 *pBuf);  // ACC.c (ACC.h) 
//// void ACC_Init(void) // ACC.c (ACC.h)
//// void LCD_Init(void); // GLCD.c (GLCD.h)

UINT8 bControl;
int x_value = 52;
int y_value = 52;
int x_value_past = 55;
int y_value_past = 55;



int main(void)
{
	int16 buffer[3];
    
	LCD_Init();		// LCD 구동 함수
	DelayMS(10);		// LCD구동 딜레이
	DisplayTitle();		// LCD 초기화면구동 함수
    
	_GPIO_Init();		// LED, SW 초기화
	SPI1_Init();        	// SPI1 초기화
	ACC_Init();		// 가속도센서 초기화
	TIMER3_Init();		// 가속도센서 스캔 주기 생성
   
	while(1)
	{
		if(bControl)
		{
			bControl = FALSE;     
			SPI1_Process(&buffer[0]);	// SPI통신을 이용하여 가속도센서 측정
			Display_Process(&buffer[0]);	// 측정값을 LCD에 표시
		}
	}
}

///////////////////////////////////////////////////////
// Master mode, Full-duplex, 8bit frame(MSB first), 
// fPCLK/32 Baud rate, Software slave management EN
void SPI1_Init(void)
{
	/*!< Clock Enable  *********************************************************/
	RCC->APB2ENR 	|= (1<<12);	// 0x1000, SPI1 Clock EN
	RCC->AHB1ENR 	|= (1<<0);	// 0x0001, GPIOA Clock EN		
  
	/*!< SPI1 pins configuration ************************************************/
	
	/*!< SPI1 NSS pin(PA8) configuration : GPIO 핀  */
	GPIOA->MODER 	|= (1<<(2*8));	// 0x00010000, PA8 Output mode
	GPIOA->OTYPER 	&= ~(1<<8); 	// 0x0100, push-pull(reset state)
	GPIOA->OSPEEDR 	|= (3<<(2*8));	// 0x00030000, PA8 Output speed (100MHZ) 
	GPIOA->PUPDR 	&= ~(3<<(2*8));	// 0x00030000, NO Pullup Pulldown(reset state)
    
	/*!< SPI1 SCK pin(PA5) configuration : SPI1_SCK */
	GPIOA->MODER 	|= (2<<(2*5)); 	// 0x00000800, PA5 Alternate function mode
	GPIOA->OTYPER 	&= ~(1<<5); 	// 0020, PA5 Output type push-pull (reset state)
	GPIOA->OSPEEDR 	|= (3<<(2*5));	// 0x00000C00, PA5 Output speed (100MHz)
	GPIOA->PUPDR 	|= (2<<(2*5)); 	// 0x00000800, PA5 Pull-down
	GPIOA->AFR[0] 	|= (5<<(4*5));	// 0x00500000, Connect PA5 to AF5(SPI1)
    
	/*!< SPI1 MOSI pin(PA7) configuration : SPI1_MOSI */    
	GPIOA->MODER 	|= (2<<(2*7));	// 0x00008000, PA7 Alternate function mode
	GPIOA->OTYPER	&= ~(1<<7);	// 0x0080, PA7 Output type push-pull (reset state)
	GPIOA->OSPEEDR 	|= (3<<(2*7));	// 0x0000C000, PA7 Output speed (100MHz)
	GPIOA->PUPDR 	|= (2<<(2*7)); 	// 0x00008000, PA7 Pull-down
	GPIOA->AFR[0] 	|= (5<<(4*7));	// 0x50000000, Connect PA7 to AF5(SPI1)
    
	/*!< SPI1 MISO pin(PA6) configuration : SPI1_MISO */
	GPIOA->MODER 	|= (2<<(2*6));	// 0x00002000, PA6 Alternate function mode
	GPIOA->OTYPER 	&= ~(1<<6);	// 0x0040, PA6 Output type push-pull (reset state)
	GPIOA->OSPEEDR 	|= (3<<(2*6));	// 0x00003000, PA6 Output speed (100MHz)
	GPIOA->PUPDR 	|= (2<<(2*6));	// 0x00002000, PA6 Pull-down
	GPIOA->AFR[0] 	|= (5<<(4*6));	// 0x05000000, Connect PA6 to AF5(SPI1)

	// Init SPI1 Registers 
	SPI1->CR1 |= (1<<2);	// MSTR(Master selection)=1, Master mode
	SPI1->CR1 &= ~(1<<15);	// SPI_Direction_2 Lines_FullDuplex
	SPI1->CR1 &= ~(1<<11);	// SPI_DataSize_8bit
	SPI1->CR1 |= (1<<9);  	// SSM(Software slave management)=1, 
				// NSS 핀 상태가 코딩에 의해 결정
	SPI1->CR1 |= (1<<8);	// SSI(Internal_slave_select)=1,
				// 현재 MCU가 Master이므로 NSS 상태는 'High' 
	SPI1->CR1 &= ~(1<<7);	// LSBFirst=0, MSB transmitted first    
	SPI1->CR1 |= (4<<3);	// BR(BaudRate)=0b100, fPCLK/32 (84MHz/32 = 2.625MHz)
	SPI1->CR1 |= (1<<1);	// CPOL(Clock polarity)=1, CK is 'High' when idle
	SPI1->CR1 |= (1<<0);	// CPHA(Clock phase)=1, 두 번째 edge 에서 데이터가 샘플링
 
	SPI1->CR1 |= (1<<6);	// SPE=1, SPI1 Enable 
}

void TIMER10_Init(void)	// 가속도센서 측정 주기 생성: 200ms
{
	RCC->APB2ENR 	|= (1<<17);	// TIMER10 Clock Enable
     
	TIM10->PSC 	= 16800-1;	// Prescaler 168MHz/8400 = 10KHz (0.1ms)  
	TIM10->ARR 	= 2000-1;	// Auto reload  0.1ms * 2000 = 200ms
        TIM10->CCR1 	= 1000-1;
        
	TIM10->CR1	&= ~(1<<4);	// Countermode = Upcounter (reset state)
	TIM10->CR1 	&= ~(3<<8);	// Clock division = 1 (reset state)
	TIM10->EGR 	|=(1<<0);	// Update Event generation    

	TIM10->DIER 	|= (1<<1);	// Enable Tim10  Capture/Compare Update interrupt
	TIM10->CR1 	|= (1<<0);	// Enable Tim10 Counter    
}

void Display_Process(int16 *pBuf)
{ 
        UINT16 G_VALUE;
	// X 축 가속도 표시		
	if (pBuf[0] < 0)  //음수
	{
                G_VALUE = abs(pBuf[0]);
		LCD_DisplayChar(2, 22, '+'); // g 부호 표시
                G_VALUE = 10 * G_VALUE / 0x4009;
                x_value = x_value_past +  (G_VALUE/10 * 10 + G_VALUE %10)*(G_VALUE/10 * 10 + G_VALUE %10);
	}
	else				// 양수
	{
                G_VALUE = pBuf[0];
		LCD_DisplayChar(2, 22, '-'); // g 부호 표시
                G_VALUE = 10 * G_VALUE / 0x4009;
                x_value = x_value_past -  (G_VALUE/10 * 10 + G_VALUE %10)*(G_VALUE/10 * 10 + G_VALUE %10);
	}
        
        if(x_value > 980){
            x_value = 980;
        }
        else if (x_value < 60){
            x_value = 60;
        }
        
	LCD_DisplayChar(2, 23, ((G_VALUE/10) +0x30));
        LCD_DisplayChar(2, 24, '.');
        LCD_DisplayChar(2, 25, ((G_VALUE%10)  +0x30));
        
        
	// Y 축 가속도 표시	
	if (pBuf[1] < 0)  //음수
	{
                G_VALUE = abs(pBuf[1]);
		LCD_DisplayChar(3,22,'-'); // g 부호 표시
                G_VALUE = 15 * G_VALUE / 0x4009;
                y_value = y_value_past +  (G_VALUE/10 * 10 + G_VALUE %10)*(G_VALUE/10 * 10 + G_VALUE %10);
	}
	else				// 양수
	{
                G_VALUE = pBuf[1];
		LCD_DisplayChar(3,22,'+'); // g 부호 표시
                G_VALUE = 15 * G_VALUE / 0x4009;
                y_value = y_value_past -  (G_VALUE/10 * 10 + G_VALUE %10)*(G_VALUE/10 * 10 + G_VALUE %10);
	}
        
        if(y_value > 1130){
            y_value = 1130;
        }
        else if (y_value < 210){
            y_value = 210;
        }
	LCD_DisplayChar(3, 23, ((G_VALUE /10) +0x30));
        LCD_DisplayChar(3, 24, '.');
        LCD_DisplayChar(3, 25, ((G_VALUE %10)  +0x30));

	// Z 축 가속도 표시	
	if (pBuf[2] < 0)  //음수
	{
                G_VALUE = abs(pBuf[2]);
		LCD_DisplayChar(4, 22, '-'); // g 부호 표시

	}
	else				// 양수
	{
                G_VALUE = pBuf[2];
		LCD_DisplayChar(4, 22, '+'); // g 부호 표시
	}
        G_VALUE = 15 * G_VALUE / 0x4009;
	LCD_DisplayChar(4, 23, ((G_VALUE /10) +0x30));
        LCD_DisplayChar(4, 24, '.');
        LCD_DisplayChar(4, 25, ((G_VALUE%10) +0x30));
        
	display_ball((x_value/10), (y_value/10));
        x_value_past =x_value;
        y_value_past = y_value;
}

void display_ball(int x, int y) {
        LCD_SetBrushColor(RGB_WHITE);
        LCD_DrawFillRect(6, 21,98,98);
        LCD_SetPenColor(RGB_RED);
	LCD_DrawRectangle(x, y,5,5);
        
                        
}

void _GPIO_Init(void)
{
	// LED (GPIO G) 설정
	RCC->AHB1ENR	|=  0x00000040;	// RCC_AHB1ENR : GPIOG(bit#6) Enable							
	GPIOG->MODER 	|=  0x00005555;	// GPIOG 0~7 : Output mode (0b01)						
	GPIOG->OTYPER	&= ~0x00FF;	// GPIOG 0~7 : Push-pull  (GP8~15:reset state)	
	GPIOG->OSPEEDR 	|=  0x00005555;	// GPIOG 0~7 : Output speed 25MHZ Medium speed 
    
	// SW (GPIO H) 설정 
	RCC->AHB1ENR    |=  0x00000080;	// RCC_AHB1ENR : GPIOH(bit#7) Enable							
	GPIOH->MODER 	&= ~0xFFFF0000;	// GPIOH 8~15 : Input mode (reset state)				
	GPIOH->PUPDR 	&= ~0xFFFF0000;	// GPIOH 8~15 : Floating input (No Pull-up, pull-down) :reset state

	// Buzzer (GPIO F) 설정 
	RCC->AHB1ENR	|=  0x00000020; // RCC_AHB1ENR : GPIOF(bit#5) Enable							
	GPIOF->MODER 	|=  0x00040000;	// GPIOF 9 : Output mode (0b01)						
	GPIOF->OTYPER 	&= ~0x0200;	// GPIOF 9 : Push-pull  	
	GPIOF->OSPEEDR 	|=  0x00040000;	// GPIOF 9 : Output speed 25MHZ Medium speed 

	GPIOG->ODR &= 0x00;	// LED0~7 Off 
}

void DelayMS(unsigned short wMS)
{
	register unsigned short i;

	for (i=0; i<wMS; i++)
		DelayUS(1000);		//1000us => 1ms
}

void DelayUS(unsigned short wUS)
{
        volatile int Dly = (int)wUS*17;
         for(; Dly; Dly--);
}

void DisplayTitle(void)
{
	LCD_Clear(RGB_WHITE);
        LCD_SetFont(&Gulim7);
	LCD_SetBackColor(RGB_WHITE);
	LCD_SetTextColor(RGB_BLACK);    //글자색
        LCD_DisplayText(0, 0, "Ball game: LGW 2018132027");
        LCD_DisplayText(2, 18, "Ax:");
        LCD_DisplayText(3, 18, "Ay:");
        LCD_DisplayText(4, 18, "Az:");
        LCD_SetPenColor(RGB_BLUE);
        LCD_DrawRectangle(5, 20,100,100); //5~105, 20 ~120
        LCD_SetPenColor(RGB_RED);
        LCD_SetTextColor(RGB_RED);
        LCD_SetPenColor(RGB_RED);
	LCD_DrawRectangle(52, 60,5,5);

}
