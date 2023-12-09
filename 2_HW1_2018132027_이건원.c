/////////////////////////////////////////////////////////////
// HW1: ���� ������ �ð� ����
// ������: 2018132027 �̰ǿ�
// �ֿ� ���� �� ���� ���
// -���� �ֿ䵵���� �ð踦 ǥ��  ICN, ORD, LHR
// -�� ���ø� �����ϴ� ��� SW0, SW1, SW2�� ����
// -���� �ð� ICN: TIM2�� Down-Counting mode(100msec) �̿�
// -���� �ð� ORD: TIM6�� up-Counting mode(100msec) �̿�
// -���� �ð� LHR: TIM9�� OC mode(100msec) �̿�
// -���� �ð� reset : ICN 00:00 ORD 10:00 LHR 16:00
// -���� �ð� ���� �ӵ� ����: 10msec, 100msec
/////////////////////////////////////////////////////////////
#include "stm32f4xx.h"
#include "GLCD.h"

#define SW0_PUSH        0xFE00  //PH8
#define SW1_PUSH        0xFD00  //PH9
#define SW2_PUSH        0xFB00  //PH10
#define SW3_PUSH        0xF700  //PH11
#define SW4_PUSH        0xEF00  //PH12
#define SW5_PUSH        0xDF00  //PH13
#define SW6_PUSH        0xBF00  //PH14
#define SW7_PUSH        0x7F00  //PH15


void _RCC_Init(void);
void _GPIO_Init(void);
void _EXTI_Init(void);
uint16_t KEY_Scan(void);

void TIMER2_Init(void);
void TIMER6_Init(void);
void TIMER9_OC_Init(void);

void ICN(void);     // ICN �ð�
void ORD(void);     // ORD �ð�
void LHR(void);     // LHR �ð�

void DisplayInitScreen(void);

void DelayMS(unsigned short wMS);
void DelayUS(unsigned short wUS);
void BEEP(void);

int FLAG = 0;
int reset_flag = 0;
int period = 100;


int main(void)
{
    _GPIO_Init();  		// GPIO �ʱ�ȭ
    _EXTI_Init();		// �ܺ����ͷ�Ʈ �ʱ�ȭ
    LCD_Init();		// GLCD �ʱ�ȭ
    DelayMS(10);			
    //BEEP();			// Beep �ѹ� 

    GPIOG->ODR &= 0xFF00;	// �ʱⰪ: LED0~7 Off
    DisplayInitScreen();	// LCD �ʱ�ȭ��

    TIMER2_Init();		// ����Ÿ�̸�(TIM2) �ʱ�ȭ : down counting mode
    TIMER6_Init();
    TIMER9_OC_Init();
    
    while(1)
    {   
      switch(KEY_Scan())
        {
            case SW0_PUSH : //SW0
              FLAG = 0; // ICN display
            break;
            case SW1_PUSH : //SW1
              FLAG = 1; // ORD display           
            break;
            case SW2_PUSH : //SW2
              FLAG = 2; // LHR display           
            break;
        }
    }
}

void TIMER2_Init(void)
{
	RCC->APB1ENR |= 0x01;	// RCC_APB1ENR TIMER2 Enable

	// Setting CR1 : 0x0000 
        TIM2->CR1 |= (1<<4);   // DIR=0(Down counter)(reset state)
	TIM2->CR1 &= ~(1<<1);	// UDIS=0(Update event Enabled): By one of following events
                            //  Counter Overflow/Underflow, 
                            //  Setting the UG bit Set,
                            //  Update Generation through the slave mode controller 
                            // UDIS=1 : Only Update event Enabled by  Counter Overflow/Underflow,
	TIM2->CR1 &= ~(1<<2);	// URS=0(Update Request Source  Selection):  By one of following events
                            //	Counter Overflow/Underflow, 
                            // Setting the UG bit Set,
                            //	Update Generation through the slave mode controller 
                            // URS=1 : Only Update Interrupt generated  By  Counter Overflow/Underflow,
	TIM2->CR1 &= ~(1<<3);	// OPM=0(The counter is NOT stopped at update event) (reset state)
	TIM2->CR1 &= ~(1<<7);	// ARPE=0(ARR is NOT buffered) (reset state)
	TIM2->CR1 &= ~(3<<8); 	// CKD(Clock division)=00(reset state)
	TIM2->CR1 &= ~(3<<5); 	// CMS(Center-aligned mode Sel)=00 (Edge-aligned mode) (reset state)
                            // Center-aligned mode: The counter counts UP and DOWN alternatively


    // Deciding the Period
	TIM2->PSC = 42-1;	// Prescaler 82,000,000Hz/42 = 2,000,000 Hz (0.0005ms)  
	TIM2->ARR = (2000*period);	// Auto reload  0.0005ms * 200000 = 100ms

   	// Clear the Counter
	TIM2->EGR |= (1<<0);	// UG(Update generation)=1 
                        // Re-initialize the counter(CNT=0) & generates an update of registers   

	// Setting an UI(UEV) Interrupt 
	NVIC->ISER[0] |= (1<<28); // Enable Timer2 global Interrupt
 	TIM2->DIER |= (1<<0);	// Enable the Tim2 Update interrupt

	TIM2->CR1 |= (1<<0);	// Enable the Tim2 Counter (clock enable)   
}

void TIM2_IRQHandler(void)  	// 100ms Interrupt
{
  	TIM2->SR &= ~(1<<0);	// Interrupt flag Clear
        ICN(); // ICN �ð� ī��Ʈ
}

void TIMER6_Init(void)
{
	RCC->APB1ENR |= 0x10;	// RCC_APB1ENR TIMER6 Enable

	TIM6->CR1 &= ~(1<<4);  // DIR=1(up counter)(reset state)


	TIM6->CR1 &= ~(1<<1);	// UDIS=0(Update event Enabled): By one of following events
                            //  Counter Overflow/Underflow, 
                            //  Setting the UG bit Set,
                            //  Update Generation through the slave mode controller 
                            // UDIS=1 : Only Update event Enabled by  Counter Overflow/Underflow,
	TIM6->CR1 &= ~(1<<2);	// URS=0(Update Request Source  Selection):  By one of following events
                            //	Counter Overflow/Underflow, 
                            // Setting the UG bit Set,
                            //	Update Generation through the slave mode controller 
                            // URS=1 : Only Update Interrupt generated  By  Counter Overflow/Underflow,
	TIM6->CR1 &= ~(1<<3);	// OPM=0(The counter is NOT stopped at update event) (reset state)
	TIM6->CR1 &= ~(1<<7);	// ARPE=0(ARR is NOT buffered) (reset state)
	TIM6->CR1 &= ~(3<<8); 	// CKD(Clock division)=00(reset state)
	TIM6->CR1 &= ~(3<<5); 	// CMS(Center-aligned mode Sel)=00 (Edge-aligned mode) (reset state)
                            // Center-aligned mode: The counter counts UP and DOWN alternatively


    // Deciding the Period
	TIM6->PSC = 1680-1;	// Prescaler 84,000,000Hz/1680 = 50,000 Hz (0.02ms)  

	TIM6->ARR = (50*period)-1;	// Auto reload  5000ms/ 0.02ms =  100ms

   	// Clear the Counter
	TIM6->EGR |= (1<<0);	// UG(Update generation)=1 
                        // Re-initialize the counter(CNT=0) & generates an update of registers   

	// Setting an UI(UEV) Interrupt 
	NVIC->ISER[1] |= (1<<22); // Enable Timer6 global Interrupt
 	TIM6->DIER |= (1<<0);	// Enable the Tim6 Update interrupt

	TIM6->CR1 |= (1<<0);	// Enable the Tim6 Counter (clock enable)   
}

void TIM6_DAC_IRQHandler(void)  	// 1ms Interrupt
{
  	TIM6->SR &= ~(1<<0);	// Interrupt flag Clear
        ORD();  // ORD �ð� ī��Ʈ
}

void TIMER9_OC_Init(void)
{
// PE5: TIM9_CH1
// PE5�� ��¼����ϰ� Alternate function(TIM9_CH1)���� ��� ����
 	RCC->AHB1ENR	|= (1<<4);	// RCC_APB1ENR PE5 Enable

	GPIOE->MODER    |= (2<<10);	// 0x02001000, GPIOE PIN5 Output Alternate function mode 					
	GPIOE->OSPEEDR 	|= (3<<24);	// 0x00001800, GPIOE PIN5 Output speed (100MHz High speed)
	GPIOE->OTYPER	&= ~(1<<12);	// ~0x0010, GPIOE PIN5 Output type push-pull (reset state)
	GPIOE->PUPDR    |= (1<<10); 	// 0x00000800, GPIOE PIN5 Pull-up
  					// PE5 ==> TIM4_CH1
	GPIOE->AFR[0]	|= 0x00300000;  // (AFR[1].(19~16)=0b0010): Connect TIM9 pins(PE5) to AF2(TIM3..5)
// Time base ����
	RCC->APB2ENR |= (1<<16);	// 0x80, RCC_APB1ENR TIMER9 Enable

	// Setting CR1 : 0x0000 
	TIM9->CR1 &= ~(1<<4);	// DIR=0(Up counter)(reset state)
	TIM9->CR1 &= ~(1<<1);	// UDIS=0(Update event Enabled): By one of following events
                            //  Counter Overflow/Underflow, 
                            //  Setting the UG bit Set,
                            //  Update Generation through the slave mode controller 
                            // UDIS=1 : Only Update event Enabled by  Counter Overflow/Underflow,
	TIM9->CR1 &= ~(1<<2);	// URS=0(Update event source Selection): one of following events
                            //	Counter Overflow/Underflow, 
                            // Setting the UG bit Set,
                            //	Update Generation through the slave mode controller 
                            // URS=1 : Only Update Interrupt generated  By  Counter Overflow/Underflow,
	TIM9->CR1 &= ~(1<<3);	// OPM=0(The counter is NOT stopped at update event) (reset state)
	TIM9->CR1 |= (1<<7);	// ARPE=1(ARR is buffered): ARR Preload Enalbe 
	TIM9->CR1 &= ~(3<<8); 	// CKD(Clock division)=00(reset state)
	TIM9->CR1 &= ~(3<<5); 	// CMS(Center-aligned mode Sel)=00 : Edge-aligned mode(reset state)

	// Setting the Period
	TIM9->PSC = 16800-1;	// Prescaler=168, 168MHz/16800 = 10000Hz (0.1ms)
        TIM9->ARR = (10*period)-1;	// Auto reload  : 0.1ms * 1000 = 100ms(period) : ���ͷ�Ʈ�ֱ⳪ ��½�ȣ�� �ֱ� ����
        
	// Update(Clear) the Counter
	TIM9->EGR |= (1<<0);    // UG: Update generation    

// Output Compare ����
	// CCMR1(Capture/Compare Mode Register 1) : Setting the MODE of Ch1 or Ch2
	TIM9->CCMR1 &= ~(3<<0); // CC1S(CC1 channel) = '0b00' : Output 
	TIM9->CCMR1 &= ~(1<<2); // OC1FE=0: Output Compare 1 Fast disable 
	TIM9->CCMR1 &= ~(1<<3); // OC1PE=0: Output Compare 1 preload disable(CCR1�� �������� ���ο� ���� loading ����) 
	TIM9->CCMR1 |= (3<<4);	// OC1M=0b011 (Output Compare 1 Mode : toggle)
				// OC1REF toggles when CNT = CCR1
				
	// CCER(Capture/Compare Enable Register) : Enable "Channel 1" 
	TIM9->CCER |= (1<<0);	// CC1E=1: CC1 channel Output Enable
				// OC1(TIM4_CH1) Active: �ش���(100��)�� ���� ��ȣ���
	TIM9->CCER &= ~(1<<1);	// CC1P=0: CC1 channel Output Polarity (OCPolarity_High : OC1���� �������� ���)  


	TIM9->CCR1 = 15000;	// TIM9 CCR1 TIM9_Pulse

	TIM9->DIER |= (1<<0);	// UIE: Enable Tim9 Update interrupt
	TIM9->DIER |= (1<<1);	// CC1IE: Enable the Tim9 CC1 interrupt

	NVIC->ISER[0] 	|= (1<<24);	// Enable Timer9 global Interrupt on NVIC

	TIM9->CR1 |= (1<<0);	// CEN: Enable the Tim9 Counter 
}

void TIM1_BRK_TIM9_IRQHandler(void)  	// 100ms Interrupt
{
	if ((TIM9->SR & 0x01) != RESET)	// Update interrupt flag 
	{
		TIM9->SR &= ~(1<<0);	// Update Interrupt Claer
	}
        if((TIM9->SR & 0x02) != RESET)	// Capture/Compare 1 interrupt flag
	{
		TIM9->SR &= ~(1<<1);	// CC 1 Interrupt Claer
	}
        LHR(); // LHR �ð� ī��Ʈ
}

void LHR(void)
{
    	static int L_min1 = 0;
        static int  L_min10 = 5;
        static int L_hour1 = 5;
        static int L_hour10 = 1;
    
        if(FLAG ==2){        // LHR xx:xx ���÷���
          LCD_DisplayText(1, 0,"LHR");        
          LCD_DisplayChar(1, 8, L_min1+0x30);
          LCD_DisplayChar(1, 7, L_min10+0x30);
          LCD_DisplayChar(1, 5, L_hour1+0x30);
          LCD_DisplayChar(1, 4, L_hour10+0x30); 
        }
        
        
        if(reset_flag==0){  // �� ���� count
          L_min1++;
          if (L_min1==10) { //10�� ���� count
            L_min1= 0;
            L_min10 ++;
            if (L_min10 == 6){ // 1�ð� ���� count
              L_min10 = 0;
              L_hour1 ++;
              if(L_hour1 == 10){ // 10�ð� ���� count
                L_hour1 =0;
                L_hour10 ++;
               }
              if(L_hour1 == 4 && L_hour10 == 2){ // �Ϸ� ���� ����
              L_hour1 =0;
              L_hour10 = 0;
              }
            }
            }
          }
        else{   // reset���� �� ��
         L_min1 = 0;
         L_min10 = 0;
         L_hour1 = 6;
         L_hour10 = 1;
        }
}

void ORD(void)
{
  	static int O_min1 = 0;
        static int  O_min10 = 5;
        static int O_hour1 = 9;
        static int O_hour10 = 0;
        
        
        if(FLAG ==1){   // ORD xx:xx ���÷���
          LCD_DisplayText(1,0,"ORD");
          LCD_DisplayChar(1, 8, O_min1+0x30);
          LCD_DisplayChar(1, 7, O_min10+0x30);
          LCD_DisplayChar(1, 5, O_hour1+0x30);
          LCD_DisplayChar(1, 4, O_hour10+0x30); 
        }
        
        
        if(reset_flag == 0){ // �� ���� count
          O_min1++;
          if (O_min1==10) {  //10�� ���� count
            O_min1= 0;
            O_min10 ++;
            if (O_min10 == 6){  // 1�ð� ���� count
              O_min10 = 0;
              O_hour1 ++;
                if(O_hour1 == 10){  // 10�ð� ���� count
                  O_hour1 =0;
                  O_hour10 ++;
                }
                if(O_hour1 == 4 && O_hour10 == 2){  // �Ϸ� ���� ����
                  O_hour1 =0;
                  O_hour10 = 0;
                }
            }    
          }
        }  
        else{  // reset���� �� ��
         O_min1 = 0;
         O_min10 = 0;
         O_hour1 = 0;
         O_hour10 = 1;
        }

}


void ICN(void)  
{
	static int I_min1 = 0;
        static int  I_min10 = 5;
        static int I_hour1 = 3;
        static int I_hour10 = 2;
         
        if(FLAG ==0){    // ICN xx:xx ���÷���
          LCD_DisplayText(1,0,"ICN");
          LCD_DisplayChar(1, 8, I_min1+0x30);
          LCD_DisplayChar(1, 7, I_min10+0x30);
          LCD_DisplayChar(1, 5, I_hour1+0x30);
          LCD_DisplayChar(1, 4, I_hour10+0x30); 
        }
        
        if (reset_flag == 0){   // �� ���� count
          I_min1++;
          if (I_min1==10) {   //10�� ���� count
            I_min1= 0;
            I_min10 ++;
            if (I_min10 == 6){   // 1�ð� ���� count
              I_min10 = 0;
              I_hour1 ++;
            
              if(I_hour1 == 10){   // 10�ð� ���� count
                I_hour1 =0;
                I_hour10 ++;
               }
              if(I_hour1 == 4 && I_hour10 == 2){  // �Ϸ� ���� ����
                I_hour1 =0; 
                I_hour10 = 0;
              }
            } 
           }
         }
        else{  // reset���� �� ��
         I_min1 = 0;
         I_min10 = 0;
         I_hour1 = 0;
         I_hour10 = 0;
        }
}

void _GPIO_Init(void)
{
	// LED (GPIO G) ����
    	RCC->AHB1ENR	|=  0x00000040;	// RCC_AHB1ENR : GPIOG(bit#6) Enable							
	GPIOG->MODER 	|=  0x00005555;	// GPIOG 0~7 : Output mode (0b01)						
	GPIOG->OTYPER	&= ~0x00FF;	// GPIOG 0~7 : Push-pull  (GP8~15:reset state)	
 	GPIOG->OSPEEDR 	|=  0x00005555;	// GPIOG 0~7 : Output speed 25MHZ Medium speed 
    
	// SW (GPIO H) ���� 
	RCC->AHB1ENR    |=  0x00000080;	// RCC_AHB1ENR : GPIOH(bit#7) Enable							
	GPIOH->MODER 	&= ~0xFFFF0000;	// GPIOH 8~15 : Input mode (reset state)				
	GPIOH->PUPDR 	&= ~0xFFFF0000;	// GPIOH 8~15 : Floating input (No Pull-up, pull-down) :reset state

	// Buzzer (GPIO F) ���� 
        RCC->AHB1ENR	|=  0x00000020;     // RCC_AHB1ENR : GPIOF(bit#5) Enable							
	GPIOF->MODER 	|=  0x00040000;	// GPIOF 9 : Output mode (0b01)						
	GPIOF->OTYPER 	&= ~0x0200;	    // GPIOF 9 : Push-pull  	
 	GPIOF->OSPEEDR 	|=  0x00040000;	// GPIOF 9 : Output speed 25MHZ Medium speed 
        
}
        
void _EXTI_Init(void)
{
	RCC->AHB1ENR 	|= 0x0080;	// RCC_AHB1ENR GPIOH Enable
	RCC->APB2ENR 	|= 0x4000;	// Enable System Configuration Controller Clock
	
	GPIOH->MODER 	&= 0x0000FFFF;	// GPIOH PIN8~PIN15 Input mode (reset state)				 
	
	SYSCFG->EXTICR[3] |= 0x7700; 	// EXTI14,15�� ���� �ҽ� �Է��� GPIOH�� ���� (EXTICR3) (reset value: 0x0000)	
	
	EXTI->FTSR 	|= 0x004000;	// Falling Trigger Enable  (EXTI8:PH8)
	EXTI->RTSR 	|= 0x008000;	 
	EXTI->IMR  	|= 0x00C000;  	// EXTI 15,14 ���ͷ�Ʈ mask (Interrupt Enable)
		
	NVIC->ISER[1] |= ( 1 << 8 );   // Enable Interrupt EXTI15,10 Vector table Position ����
}

void EXTI15_10_IRQHandler(void)		// EXTI 5~9 ���ͷ�Ʈ �ڵ鷯
{
	if(EXTI->PR & 0x4000) 		// EXTI14 nterrupt Pending?
          
	{
          EXTI->PR |= 0x4000; 	// Pending bit Clear
        
          
          if(FLAG == 0){
            TIM2->CR1 &= 0xFFFE;  // TIM2(TIM6, TIM9)->CR1.CEN=0
            TIM6->CR1 &= 0xFFFE;
            TIM9->CR1 &= 0xFFFE;
            
            if(period == 100){
              period = 10;
              LCD_DisplayText(2,11," 10");
            }
            else{
              period = 100;
              LCD_DisplayText(2,11,"100");
            }
            
            TIM2->ARR = (2000*period); // TIM2(TIM6, TIM9)->ARR �� ����
            TIM6->ARR = (50*period)-1;
            TIM9->ARR = (10*period)-1;
            
            TIM2->CNT &= 0; // TIM2(TIM6, TIM9)->CNT=0
            TIM6->CNT &= 0;
            TIM9->CNT &= 0;
            
            TIM2->CR1 |= (1<<0); // TIM2(TIM6, TIM9)->CR1.CEN=1
            TIM6->CR1 |= (1<<0);
            TIM9->CR1 |= (1<<0);
          }
        }
	else if(EXTI->PR & 0x8000) 	// EXTI15 Interrupt Pending?
	{
          EXTI->PR |= 0x8000; 	// Pending bit Clear
          if(FLAG == 0){  //ICN���� ����
                TIM2->CR1 &= 0xFFFE; //TIM2->CR1.CEN=0 

                GPIOG->ODR |= (1<<7); // LED7 ON
                BEEP();  // ���� 1ȸ
                DelayMS(3000);  // 3�� delay
                GPIOG->ODR &= ~(1<<7); // LED7 OFF
                TIM2->CNT &= 0; // TIM2->CNT=0

                reset_flag = 1;
                ICN(); // ��ICN 00:00�� ǥ��
                ORD();
                LHR();
                reset_flag = 0;
                TIM2->CR1 |= (1<<0); // TIM2->CR1.CEN=1

          }
	}
}

void BEEP(void)			/* beep for 30 ms */
{ 	GPIOF->ODR |= 0x0200;	// PF9 'H' Buzzer on
	DelayMS(30);		// Delay 30 ms
	GPIOF->ODR &= ~0x0200;	// PF9 'L' Buzzer off
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
    LCD_Clear(RGB_YELLOW);	// ȭ�� Ŭ���� ���� yellow
    LCD_SetFont(&Gulim8);	// ��Ʈ : ���� 8
    LCD_SetBackColor(RGB_YELLOW); 
    LCD_SetTextColor(RGB_BLUE);// ���ڻ� : Blue
    LCD_DisplayText(0,0,"LGW 2018132027");
    LCD_DisplayText(2,0,"Int period    ms");
    LCD_SetTextColor(RGB_BLACK);// ���ڻ� : Black
    LCD_DisplayText(2,11,"100");
    LCD_DisplayText(1,0,"ICN 23:50");  

}

uint8_t key_flag = 0;
uint16_t KEY_Scan(void)	// input key SW0 - SW7 
{ 
	uint16_t key;
	key = GPIOH->IDR & 0xFF00;	// any key pressed ?
	if(key == 0xFF00)		// if no key, check key off
	{  	if(key_flag == 0)
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
