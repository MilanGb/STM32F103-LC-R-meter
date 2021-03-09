//   milangbr@gmail.com 20_02_2021
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_flash.h"
#include "stm32f10x_pwr.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_adc.h"
#include "stdio.h"
#include "stdlib.h"
#include "math.h"

#include "delay.h"
#include "lcd.h"

#define HZ_IN_ERROR     14492.754   //no used
#define Pi2 39.4784176
#define R330 GPIOB->BSRR=GPIO_BSRR_BR8|GPIO_BSRR_BR5|GPIO_BSRR_BR6|GPIO_BSRR_BR7|GPIO_BSRR_BR9; GPIOA->BSRR=GPIO_BSRR_BS11;
#define R3k3 GPIOB->BSRR=GPIO_BSRR_BR8|GPIO_BSRR_BR5|GPIO_BSRR_BR6|GPIO_BSRR_BR7|GPIO_BSRR_BS9; GPIOA->BSRR=GPIO_BSRR_BR11;
#define R33k GPIO_ResetBits(GPIOB, GPIO_Pin_9|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8);GPIOA->BSRR=GPIO_BSRR_BR11;

#define RM33 GPIO_ResetBits(GPIOB, GPIO_Pin_5|GPIO_Pin_7);GPIO_SetBits(GPIOB, GPIO_Pin_6|GPIO_Pin_8);
#define R3M3 GPIO_ResetBits(GPIOB, GPIO_Pin_5|GPIO_Pin_6);GPIO_SetBits(GPIOB, GPIO_Pin_7|GPIO_Pin_8);
#define R33M GPIO_ResetBits(GPIOB, GPIO_Pin_6|GPIO_Pin_7);GPIO_SetBits(GPIOB, GPIO_Pin_5|GPIO_Pin_8);

void RCC_Conf(void);
void GPIO_Conf(void);
void Init_ADC1(void);
void EXT_FrequencyCounterConfig(void);
void EXT_FrequencyCounterRun(void);

volatile u32 RawFreq, TrueFreq ,Fh=0, Fk=0;
volatile u32 ErrorNumbers;
volatile FlagStatus DataReady = RESET;

   char Buf1[7];
   char Buf2[12];
   char Buf3[6];
   char Buf4[11];
 int8_t menu=0,i=0, j=0, l=0;
 uint16_t Cpa=1270, Cka, ALM , Re1=0, Re2=0;
 uint32_t  VA1, VA3  ;
float Fmf=0, Lin, Lme, Cme, Lpa;

void flash_erase_page(uint32_t address) {
FLASH->CR|= FLASH_CR_PER; //Устанавливаем бит стирания одной страницы
FLASH->AR = address; // Задаем её адрес
FLASH->CR|= FLASH_CR_STRT; // Запускаем стирание
while(!(FLASH->SR & FLASH_SR_EOP)); //Ждем пока страница сотрется.
FLASH->CR&= ~FLASH_CR_PER; //Сбрасываем бит обратно
}
void read_Re1() {
	for(l=0;l<10;l++) {Re1 += ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_1); delay_ms(30); }
	VA1=Re1/10; Re1=0; VA1*=3300; VA1/=4095;
}
void read_Re2() {
	for(l=0;l<10;l++) {Re2 += ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_3); delay_ms(30);}
	VA3=Re2/10; Re2=0; VA3*=3300; VA3/=4095;
}
int main(void)
{
//   u8 k=0;
	RCC_Conf();
 	GPIO_Conf();
    LCD_Init();
    Init_ADC1();
    EXT_FrequencyCounterConfig();
    EXT_FrequencyCounterRun();

   Cpa = (*(__IO uint16_t*) 0x0800F000); if(Cpa>65534)Cpa=1270;
   Lpa = (*(__IO uint16_t*) 0x0800F002);
 Lin =Lpa*1e-9;

 while(1)
	{
    uint16_t VA2;
    uint8_t Enter, Rez;

    VA2=ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_2);
  //  delay_ms(100);
  uint8_t  VaU=0, VaD=0, Bck=0, Nxt=0;
    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5)==0) {Bck=1;}
    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6)==0) {Nxt=1;}
    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_3)==0) {VaD=1;}
    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_4)==0) {VaU=1;}

    if(Rez>5){Rez=0;}
    if(Bck==1){menu--;}
    if(Nxt==1){menu++;}
    if(menu>10){menu=0;} if(menu<0){menu=10;}
    if(Enter>3){Enter=0;}
    sprintf(Buf1,"M:%d",menu);
 //------------------------menu----------------------------------------------------------------
 switch (menu){

   case 0: sprintf(Buf2,"Meranie_U _"); sprintf(Buf3,"Nap ="); sprintf(Buf4,"%8dmV",VA2);
         break;
   case 1: sprintf(Buf2,"Meranie_L/C _");
        if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)==1){sprintf(Buf3,"Cap =");
          Cme=Pi2*TrueFreq*TrueFreq*Lin;  Cme=1/Cme; Cme-=Cpa*1e-12;
          u32 Cdi = Cme*1e13; sprintf(Buf4,"%6d,%01dpF",Cdi/10,Cdi%10);  }
        else{sprintf(Buf3,"Ind ="); Lme=Pi2*TrueFreq*TrueFreq*Cpa*1e-12;
         Lme=1/Lme;  Lme-=Lin;
         u32 Ldi = Lme * 1e10; if(Ldi<1e5){ sprintf(Buf4,"%5d,%01dnaH",Ldi/10,Ldi%10);}
         else {Ldi/=1000; sprintf(Buf4,"%5d,%01dmiH",Ldi/10,Ldi%10); }} break;
   case 2: sprintf(Buf2,"Zadaj Cpa _");
           if(VaU==1){Cpa++;} if(VaD==1){Cpa--;}sprintf(Buf3,"Cpa ="); sprintf(Buf4,"%8dpF",Cpa);
	     break;
   case 3:  sprintf(Buf2,"Kalibracia>");
          if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)==1){sprintf(Buf3,"C/->L");}
          else if (TrueFreq<500000){sprintf(Buf4,"mala f kal");}

          else { if(menu==3){sprintf(Buf3,"L->OK");delay_ms(500);i=0; menu++;}}
       break;
   case 4:  GPIO_ResetBits(GPIOC, GPIO_Pin_13); if (DataReady == SET)
           { if(i<11){i++; if(i>1){ Fmf+=TrueFreq; sprintf(Buf4,"%10d",TrueFreq);}}
           else{Fh=Fmf/10;i=0;Fmf=0;sprintf(Buf4,"%10d",Fh);
           GPIO_SetBits(GPIOC, GPIO_Pin_13);delay_ms(1500); menu=5;} }break;
   case 5: if (DataReady == SET)
           { if(i<11){i++; if(i>1){ Fmf+=TrueFreq;sprintf(Buf4,"%10d",TrueFreq);}}
           else{Fk=Fmf/10;i=0;Fmf=0;sprintf(Buf4,"%10d",Fk);
           GPIO_ResetBits(GPIOC, GPIO_Pin_13); menu=6;} }break;
   case 6:  if(Bck==1){menu=3;} sprintf(Buf2,"%11d",Fh); sprintf(Buf4,"%10d",Fk);
             sprintf(Buf3,"M+>-<");   break;
   case 7:  Fmf=(float)Fh/Fk; Fmf=Fmf*Fmf; Fmf-=1;  Fmf*=Cpa; Cka=10*Fmf; //Cka=roundf(Fmf);
           u16 Ho1, Ho3;
           u8 Ho2;
           Ho1 = roundf(Fmf); ///25
           Ho3 = Ho1*10;  //250
           Ho2 = Cka-Ho3;  // 253-250
           sprintf(Buf4,"_%4d,%01dpF_",Ho1,Ho2);
             break;
   case 8: Lin=1/(Pi2*Fh*Fh*Cpa*1e-12); uint16_t Lpa=Lin*1e9;
            sprintf(Buf3,"Ind ="); sprintf(Buf4,"%2d,%3dmikH",Lpa/1000,Lpa%1000);
             break;
   case 9: if(VaU==1) Enter++; if(VaD==1){Enter--;}
           sprintf(Buf2,"Ulozenie _"); sprintf(Buf4,"For3=SAVE%01d",Enter);sprintf(Buf3,"Va+/-");
            if(Enter==3){FLASH_Unlock(); flash_erase_page(0x0800F000) ;
           FLASH_ProgramHalfWord(0x0800F000, Cpa);FLASH_ProgramHalfWord(0x0800F002, Lpa);
         //  FLASH_ProgramHalfWord(0x0800F004, ALM);
           Enter=0; FLASH_Lock(); sprintf(Buf4, ">>:SAVED<<"); delay_ms(500); menu=0;}; break;
   case 10:  sprintf(Buf1,"R:%d",Rez);  sprintf(Buf2,"Meranie_R _");
	   switch (Rez){
            case 0: R330; read_Re1(); if(VA1>3200){Rez=1;}
            sprintf(Buf3,"REZ ="); sprintf(Buf4,"%3d,%01d Ohm<",VA1/10, VA1%10); break;
            case 1: R3k3; read_Re1(); if(VA1>3200){Rez=2;} if(VA1<300){Rez=0;}
            sprintf(Buf3,"REZ ="); sprintf(Buf4,"%4d Ohm_<",VA1); break;
            case 2: R33k; read_Re1(); if(VA1>3200){Rez=3;delay_ms(100);} if(VA1<300){Rez=1;}
            sprintf(Buf3,"REZ ="); sprintf(Buf4,"%2d,%02dkOhm",VA1/100, VA1%100); break;
            case 3: RM33; read_Re2(); if(VA3>3200){Rez=4;} if(VA3<300){Rez=2; delay_ms(100);}
            sprintf(Buf3,"REZ ="); sprintf(Buf4,"%3d,%01dkOhm",VA3/10,VA3%10); break;
            case 4: R3M3; read_Re2(); if(VA3>3200){Rez=5;}if(VA3<300){Rez=3;}
            sprintf(Buf3,"REZ ="); sprintf(Buf4,"%1d,%03dMOhm",VA3/1000,VA3%1000); break;
            case 5: R33M; read_Re2(); if(VA3<300){Rez=4;}
            sprintf(Buf3,"REZ ="); sprintf(Buf4,"%2d,%02dM_Ohm",VA3/100,VA3%100); break;
              }
              break;
 }

   if (DataReady == SET)   { DataReady = RESET;
                            ErrorNumbers = lroundf((float)(RawFreq) / HZ_IN_ERROR);
                            TrueFreq  = RawFreq; // + ErrorNumbers
                                // GPIOC->ODR ^= GPIO_Pin_13;
                            }
//-------------------------------------------------------------------------------------------------------
    lcd_locate(0,0); lcd_str(Buf1);
    lcd_locate(5,0); lcd_str(Buf2);
    lcd_locate(0,1); lcd_str(Buf3);
    lcd_locate(6,1); lcd_str(Buf4);

if(VA2>4090){TIM2->PSC=999, TIM2->ARR=126, TIM2->CCR2=63;delay_ms(1000);}else{TIM2->CCR2=0;}

//  GPIO_SetBits(GPIOC, GPIO_Pin_13);
   delay_ms(200);
 //  GPIO_ResetBits(GPIOC, GPIO_Pin_13);
 }
}
//-------------------------------------------------------------------------------------
void RCC_Conf(void)
{
	ErrorStatus HSEStartUpStatus;
	  RCC_DeInit();
	  // Wlacz HSE
	  RCC_HSEConfig(RCC_HSE_ON);
	  // Czekaj za HSE bedzie gotowy
	  HSEStartUpStatus = RCC_WaitForHSEStartUp();
	  if(HSEStartUpStatus == SUCCESS)
	  {
	    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
	    // zwloka dla pamieci Flash
	    FLASH_SetLatency(FLASH_Latency_2);
	    // HCLK = SYSCLK
	    RCC_HCLKConfig(RCC_SYSCLK_Div1);
	    // PCLK2 = HCLK
	    RCC_PCLK2Config(RCC_HCLK_Div1);
	    // PCLK1 = HCLK/2
	    RCC_PCLK1Config(RCC_HCLK_Div2);
	    // ADCCLK = PCLK2/4
	    RCC_ADCCLKConfig(RCC_PCLK2_Div4);
	   // PLLCLK = 8MHz * 7 = 56 MHz
	    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_7);
	    // Wlacz PLL
	    RCC_PLLCmd(ENABLE);
	    // Czekaj az PLL poprawnie sie uruchomi
	    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
	    // PLL bedzie zrodlem sygnalu zegarowego
	    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
	    // Czekaj az PLL bedzie sygnalem zegarowym systemu
	    while(RCC_GetSYSCLKSource()!= 0x08){};
		// Wlacz sygnal zegarowy dla ADC1, GPIOC i funkcji alternatywnych
	  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |RCC_APB2Periph_GPIOB |
	  			RCC_APB2Periph_GPIOC, ENABLE);
		// Wlacz DMA
	//  	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	  }
	}
void GPIO_Conf(void)
	{
	  	GPIO_InitTypeDef GPIO_InitStructure;
// ledPC13 - žltá
	  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	  	GPIO_Init(GPIOC, &GPIO_InitStructure);
// tlačítka
	  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 |GPIO_Pin_4| GPIO_Pin_5| GPIO_Pin_6 ;
	  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	  	GPIO_Init(GPIOA, &GPIO_InitStructure);
//ADC vstup
	  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 |GPIO_Pin_1 ;
	  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	  	GPIO_Init(GPIOB, &GPIO_InitStructure);

	  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 ;
	  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	  	GPIO_Init(GPIOA, &GPIO_InitStructure);
//Frekv vstup
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
        GPIO_Init(GPIOA, &GPIO_InitStructure);
// bzuciak Tim2 ch2 PA1
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
        GPIO_Init(GPIOA, &GPIO_InitStructure);
 // rozsahy, rele

	  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5| GPIO_Pin_6 |GPIO_Pin_7 |GPIO_Pin_8|GPIO_Pin_9 ;
	  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	  	GPIO_Init(GPIOB, &GPIO_InitStructure);

	  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 ;
	  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	  	GPIO_Init(GPIOA, &GPIO_InitStructure);

	}
void Init_ADC1(void){

	ADC_InitTypeDef ADC_InitStructure;
//	RCC_ADCCLKConfig (RCC_PCLK2_Div6);
    /* Enable ADC1 and GPIOA clock */
	RCC_APB2PeriphClockCmd (RCC_APB2Periph_ADC1, ENABLE);

	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_NbrOfChannel = 2;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_Init(ADC1, &ADC_InitStructure);

	ADC_InjectedSequencerLengthConfig(ADC1, 3);
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_8, 1, ADC_SampleTime_55Cycles5);  // PB0 maly rozsah <33k
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_9, 2, ADC_SampleTime_239Cycles5);  //PB1  Volt
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_7, 3, ADC_SampleTime_55Cycles5);  // PA7 velky rozsah  >33k

	ADC_ExternalTrigInjectedConvConfig( ADC1, ADC_ExternalTrigInjecConv_None );
	ADC_Cmd ( ADC1 , ENABLE ) ;

	ADC_ResetCalibration(ADC1);
	while(ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus(ADC1));

	ADC_AutoInjectedConvCmd( ADC1, ENABLE );
	ADC_SoftwareStartInjectedConvCmd ( ADC1 , ENABLE ) ;
}
//////////////////////////////////////////////////////////////////////////////
void TIM3_IRQHandler(void)
{
        TIM3->SR &= ~TIM_SR_UIF; //reset interrupt flag

        RawFreq = ((u32)(TIM4->CNT) << 16) | ((u32)(TIM1->CNT));
        DataReady = SET;
        TIM1->CNT = 0;
        TIM4->CNT = 0;

        TIM3->CR1 |= TIM_CR1_CEN; //another circle
}
void EXT_FrequencyCounterConfig(void)
{
        RCC->APB1ENR |= RCC_APB1ENR_TIM2EN |RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN;//TIM3, TIM4
        RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;//TIM1

        //------------------------------------------------------------------------------------------
        //TIM3 master counter, period = 1sec.
        //------------------------------------------------------------------------------------------
        TIM3->PSC       = 5599 ; //5600 - 1; //new clock = 10kHz
        TIM3->ARR       = 9999; //10000 - 1; //period = 1sec calibre!
        TIM3->CR1       |= TIM_CR1_DIR; //used as downcounter
        TIM3->CR1       |= TIM_CR1_OPM; //counter stops counting at the next update event
        TIM3->CR2       |= TIM_CR2_MMS_0; //COUNTER_ENABLE signal to TIM1, used as trigger output (TRGO)
        TIM3->DIER      |= TIM_DIER_UIE; //enable interrupt

        //------------------------------------------------------------------------------------------
        //TIM1 slave counter, TIM1_ETR pin is input
        //------------------------------------------------------------------------------------------
        TIM1->PSC       = 0;
        TIM1->ARR       = 0xFFFF; //counter max value, may be don't set
        TIM1->CR1       &= ~TIM_CR1_DIR; //used as upcounter
        TIM1->CR1       &= ~TIM_CR1_OPM; //is not stopped at update event
        TIM1->CR2       |= TIM_CR2_MMS_1; //update event is selected as trigger output to TIM4
        TIM1->SMCR      |= TIM_SMCR_ECE; //ext. clock mode2 enabled, counter is clocked by any active edge on the ETRF signal
        TIM1->SMCR      &= ~TIM_SMCR_ETPS; //no external trigger prescaller
        TIM1->SMCR      &= ~TIM_SMCR_ETF; //no external trigger filter
        TIM1->SMCR      |= TIM_SMCR_TS_1; //internal trigger_2 (ITR2) from TIM3
        TIM1->SMCR      |= TIM_SMCR_SMS_0 | TIM_SMCR_SMS_2; //while trigger input (TRGI) is high, counter is on

        //------------------------------------------------------------------------------------------
        //TIM4 additional counter
        //------------------------------------------------------------------------------------------
        TIM4->PSC       = 0;
        //TIM4->ARR     = 0xFFFF; //counter max value
        TIM4->CR1       &= ~TIM_CR1_DIR; //used as upcounter
        TIM4->SMCR      &= ~TIM_SMCR_ETPS; //no external trigger prescaller
        TIM4->SMCR      &= ~TIM_SMCR_ETF; //no external trigger filter
        TIM4->SMCR      |= TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1 | TIM_SMCR_SMS_2; //rising edges of TIM1 (TRGI) clock the counter
        //------------------------------------------------------------------
        //TIM2 trubka
        //------------------------------------------------------------------
        TIM_TimeBaseInitTypeDef timer;
        TIM_OCInitTypeDef timerPWM;
        timer.TIM_ClockDivision =0;
        timer.TIM_CounterMode = TIM_CounterMode_Up;
        TIM_TimeBaseInit(TIM2, &timer);
        TIM_OCStructInit(&timerPWM);
        timerPWM.TIM_Pulse = 0;
        timerPWM.TIM_OCMode = TIM_OCMode_PWM1;
        timerPWM.TIM_OutputState = TIM_OutputState_Enable;
        timerPWM.TIM_OCPolarity = TIM_OCPolarity_High;

        TIM_OC2Init(TIM2, &timerPWM);
        TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
        TIM_Cmd(TIM2, ENABLE);
}

void EXT_FrequencyCounterRun(void)
{
        NVIC_EnableIRQ(TIM3_IRQn);
//TIM_Cmd(TIM2, ENABLE);
        TIM4->CR1       |= TIM_CR1_CEN;
        TIM1->CR1       |= TIM_CR1_CEN;
        TIM3->CR1       |= TIM_CR1_CEN;
}


