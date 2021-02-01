/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright(c) 2019 Nuvoton Technology Corp. All rights reserved.                                         */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/

/***********************************************************************************************************/
/* Website: http://www.nuvoton.com                                                                         */
/*  E-Mail : MicroC-8bit@nuvoton.com                                                                       */
/*  Date   : Jan/21/2019                                                                                   */
/***********************************************************************************************************/

/************************************************************************************************************/
/*  File Function: MS51 DEMO project                                                                        */
/************************************************************************************************************/

#include "MS51_16K.h"

enum{
	TARGET_CH0 = 0 ,
	TARGET_CH1 ,
	TARGET_CH2 ,
	TARGET_CH3 ,	
	
	TARGET_CH4 ,
	TARGET_CH5 ,
	TARGET_CH6 ,
	TARGET_CH7 ,

	TARGET_CH_DEFAULT	
}Channel_TypeDef;

enum
{
	Function1 ,
	Function2 ,
	Function3 ,
};

/*_____ M A C R O S ________________________________________________________*/

#define ABS(X)  									((X) > 0 ? (X) : -(X)) 

#define MAKE12BIT(v1,v2)         				((((uint16_t)(v1))<<4)+(uint16_t)(v2))      //v1,v2 is uint8_t


#define ENABLE_CONVERT_ADC_TO_DUTY

#define ENABLE_ADC_RAW_DATA
//#define ENABLE_ADC_GET_N_DEL_X
//#define ENABLE_ADC_AVG
#define ENABLE_ADC_LOW_PASS_FILTER


#define _debug_log_ADC_to_Duty_					(0)
#define _debug_log_ADC_to_Volt_					(0)
#define _debug_log_ADC_Convert_					(0)
#define _debug_log_application_					(1)
#define _debug_log_LOWPASSFILTER_				(0)


/*_____ D E F I N I T I O N S ______________________________________________*/
#define SYS_CLOCK 								(24000000ul)

#define PWM_FREQ 								(16000ul)


#define ADC_RESOLUTION							(4096ul)
//#define ADC_REF_VOLTAGE							(3300ul)	//(float)(3.3f)

#define ADC_MAX_TARGET							(4095ul)	//(float)(2.612f)
#define ADC_MIN_TARGET							(0ul)	//(float)(0.423f)

#define ADC_SAMPLE_COUNT 						(8ul)			// 8
#define ADC_SAMPLE_POWER 						(3ul)			//(5)	 	// 3	,// 2 ^ ?
#define ADC_SAMPLE_DELETE 						(4ul)
#define ADC_SAMPLE_DROP 						(4ul)

#define ADC_DIGITAL_SCALE(void) 					(0xFFFU >> ((0) >> (3U - 1U)))		//0: 12 BIT 

#define CUSTOM_INPUT_VOLT_MAX(VREF)			(VREF)			//(3300ul)
#define CUSTOM_INPUT_VOLT_MIN					(0)	//(600ul)

#define DUTY_RESOLUTION							(10000ul)
#define DUTY_MAX								(DUTY_RESOLUTION)
#define DUTY_MIN								(1ul)

/*_____ D E C L A R A T I O N S ____________________________________________*/
volatile uint8_t u8TH0_Tmp = 0;
volatile uint8_t u8TL0_Tmp = 0;


//UART 0
bit BIT_TMP;
bit BIT_UART;
bit uart0_receive_flag=0;
unsigned char uart0_receive_data;

// ADC
double  Bandgap_Voltage,AVdd,Bandgap_Value;      //please always use "double" mode for this
unsigned char xdata ADCdataVBGH, ADCdataVBGL;

unsigned long int u32adc_sum_target = 0;
uint16_t u16adc_convert_target = 0;
uint16_t u16adc_ref_voltage = 0;

volatile uint16_t u16adc_data[3] = {0};
volatile uint16_t u16adc_OldFilterValue[3] = {0};

typedef enum{
	flag_START = 0 ,


	flag_200us ,
	flag_1ms ,
	flag_5ms ,
	flag_10ms ,	
	flag_DEFAULT	
}Flag_Index;

volatile uint32_t BitFlag = 0;
#define BitFlag_ON(flag)							(BitFlag|=flag)
#define BitFlag_OFF(flag)							(BitFlag&=~flag)
#define BitFlag_READ(flag)							((BitFlag&flag)?1:0)
#define ReadBit(bit)								(uint32_t)(1<<bit)

#define is_flag_set(idx)							(BitFlag_READ(ReadBit(idx)))
#define set_flag(idx,en)							( (en == 1) ? (BitFlag_ON(ReadBit(idx))) : (BitFlag_OFF(ReadBit(idx))))


/*_____ F U N C T I O N S __________________________________________________*/

void send_UARTString(uint8_t* Data)
{
	#if 1
	uint16_t i = 0;

	while (Data[i] != '\0')
	{
		#if 1
		SBUF = Data[i++];
		#else
		UART_Send_Data(UART0,Data[i++]);		
		#endif
	}

	#endif

	#if 0
	uint16_t i = 0;
	
	for(i = 0;i< (strlen(Data)) ;i++ )
	{
		UART_Send_Data(UART0,Data[i]);
	}
	#endif

	#if 0
    while(*Data)  
    {  
        UART_Send_Data(UART0, (unsigned char) *Data++);  
    } 
	#endif
}

void send_UARTASCII(uint16_t Temp)
{
    uint8_t print_buf[16];
    uint16_t i = 15, j;

    *(print_buf + i) = '\0';
    j = (uint16_t)Temp >> 31;
    if(j)
        (uint16_t) Temp = ~(uint16_t)Temp + 1;
    do
    {
        i--;
        *(print_buf + i) = '0' + (uint16_t)Temp % 10;
        (uint16_t)Temp = (uint16_t)Temp / 10;
    }
    while((uint16_t)Temp != 0);
    if(j)
    {
        i--;
        *(print_buf + i) = '-';
    }
    send_UARTString(print_buf + i);
}

//(P1.5 , PWM0_CH5) ,  motor PWM control output , 16k freq , duty 50 %
void PWM0_CHx_Init(uint16_t uFrequency,uint16_t d)	// ex : duty 83.5% , d = 8350 , duty 26% , d = 2600
{
	unsigned long res = 0;

	P15_PUSHPULL_MODE;			//Add this to enhance MOS output capability
    PWM5_P15_OUTPUT_ENABLE;	
 
    PWM_IMDEPENDENT_MODE;
    PWM_CLOCK_DIV_2;

/*
	PWM frequency   = Fpwm/((PWMPH,PWMPL)+1) = (24MHz/2)/(PWMPH,PWMPL)+1) = 20KHz
*/	
	res = (SYS_CLOCK>>1);
	res = res/uFrequency;		//value 375 for 16K
	res = res - 1;
	
    PWMPH = HIBYTE(res);
    PWMPL = LOBYTE(res);
	
//	res = d*(MAKEWORD(PWMPH,PWMPL)+1)/DUTY_RESOLUTION;	
	res = d*(res+1)/DUTY_RESOLUTION;

	set_SFRS_SFRPAGE;
    PWM5H = HIBYTE(res);
    PWM5L = LOBYTE(res);
    clr_SFRS_SFRPAGE;
	
    set_PWMCON0_LOAD;
    set_PWMCON0_PWMRUN;		

}

void GPIO_Init(void)
{
	P17_QUASI_MODE;		
	P30_PUSHPULL_MODE;	
}


void ADC_ReadAVdd(void)
{
    UINT8 BandgapHigh,BandgapLow,BandgapMark;
    double bgvalue;

/*Read bandgap value */	
    set_CHPCON_IAPEN;
    IAPCN = READ_UID;
    IAPAL = 0x0d;
    IAPAH = 0x00;
    set_IAPTRG_IAPGO;
    BandgapLow = IAPFD;
    BandgapMark = BandgapLow&0xF0;
    BandgapLow = BandgapLow&0x0F;
    IAPAL = 0x0C;
    IAPAH = 0x00;
    set_IAPTRG_IAPGO;
    BandgapHigh = IAPFD;
    Bandgap_Value = (BandgapHigh<<4)+BandgapLow;
    Bandgap_Voltage= Bandgap_Value*3/4;
    clr_CHPCON_IAPEN;

/* ADC Low speed initial*/  
    ENABLE_ADC_BANDGAP;
    ADCCON1|=0x30;            /* clock divider */
    ADCCON2|=0x0E;            /* AQT time */
    AUXR1|=SET_BIT4;          /* ADC clock low speed */
	
/*start bandgap ADC */
    clr_ADCCON0_ADCF;
    set_ADCCON0_ADCS;                                
    while(ADCF == 0);
    ADCdataVBGH = ADCRH;
    ADCdataVBGL = ADCRL;
	
/* to convert VDD value */
    bgvalue = MAKE12BIT(ADCRH,ADCRL);
    AVdd = (0x1000/bgvalue)*Bandgap_Voltage;


}

uint16_t ADC_Drop(uint8_t drop)
{
	uint8_t n = 0;
	uint16_t res = 0;
	
	for ( n = 0 ; n < drop ; n++)
	{
		res = MAKE12BIT(ADCRH,ADCRL);
	
		set_ADCCON0_ADCS; //after convert , trigger again	
		while(ADCF == 0);		
	}	
	
	return 0;
}

#if defined (ENABLE_ADC_GET_N_DEL_X)
void Sort_tab(uint16_t tab[], uint8_t length)
{
	uint8_t l = 0x00, exchange = 0x01; 
	uint16_t tmp = 0x00;

	/* Sort tab */
	while(exchange==1) 
	{ 
		exchange=0; 
		for(l=0; l<length-1; l++) 
		{
			if( tab[l] > tab[l+1] ) 
			{ 
				tmp = tab[l]; 
				tab[l] = tab[l+1]; 
				tab[l+1] = tmp; 
				exchange=1; 
			}
		}
	} 
}

uint16_t ADC_GetNDelXAvg(uint16_t GetN , uint16_t DelX)
{
	uint16_t n = 0;
	uint16_t total = GetN + DelX;
	uint16_t adc_sample[ADC_SAMPLE_COUNT + ADC_SAMPLE_DELETE]= {0};
	uint16_t adc_target = 0;
	
	for ( n = 0 ; n < total ; n++)
	{
		adc_sample[n]= MAKE12BIT(ADCRH,ADCRL);	//adc_data;	
		
		set_ADCCON0_ADCS; //after convert , trigger again
		while(ADCF == 0);		
	}

	/* Sort the samples */
	Sort_tab(adc_sample,total);
	
	/* Add the samples */
	for (n = ADC_SAMPLE_DELETE/2; n < total - ADC_SAMPLE_DELETE/2; n++)
	{
		u32adc_sum_target += adc_sample[n];
	}

	/* get avg */
	adc_target = u32adc_sum_target >> ADC_SAMPLE_POWER ;
	
	u32adc_sum_target = 0;

	return adc_target;
}
#endif

#if defined (ENABLE_ADC_AVG)
uint16_t ADC_Average (uint16_t avg)
{
	uint8_t n = 0;
	uint16_t adc_target = 0;
	
	for ( n = 0 ; n < avg ; n++)
	{

		u32adc_sum_target += MAKE12BIT(ADCRH,ADCRL);	//adc_data;	

		set_ADCCON0_ADCS; //after convert , trigger again
		while(ADCF == 0);
	}
	adc_target = u32adc_sum_target >> ADC_SAMPLE_POWER ;
	
	u32adc_sum_target = 0;

	return adc_target;
}
#endif


#if defined (ENABLE_ADC_LOW_PASS_FILTER)
unsigned int LowPassFilter(unsigned int OldFilterValue , unsigned int NewADCValue)
{
    static unsigned int FiltedValue;

	/*
		half Sampling rate=10, BW = 4.41Hz
		Filter factor
		= exp(-pi*BW / Fs)
		= exp (-3.14*4.4/10) = 0.251
		= 256/1024 = (2^8) / (2^10)
		right shift two times
	*/

    if(NewADCValue < OldFilterValue)
    {
        FiltedValue=OldFilterValue-NewADCValue;
        FiltedValue=FiltedValue>>2;
        FiltedValue=OldFilterValue-FiltedValue;
    }
    else if(NewADCValue > OldFilterValue)
    {
        FiltedValue=NewADCValue-OldFilterValue;
        FiltedValue=FiltedValue>>2;
        FiltedValue=OldFilterValue+FiltedValue;
    }


	#if (_debug_log_LOWPASSFILTER_ == 1)
	send_UARTString("OldFilterValue:");	
	send_UARTASCII(OldFilterValue);
	send_UARTString(",NewADCValue:");
	send_UARTASCII(NewADCValue);
	send_UARTString(",FiltedValue:");
	send_UARTASCII(FiltedValue);
	
	send_UARTString("\r\n");
	#endif


    return FiltedValue;
}

#endif

uint16_t ADC_To_Voltage(uint16_t adc_value)
{
	uint16_t volt = 0;

	volt = (AVdd*adc_value)/ADC_DIGITAL_SCALE();

	#if (_debug_log_ADC_to_Volt_ == 1)
	send_UARTString("adc_value:");	
	send_UARTASCII(adc_value);
	send_UARTString(",volt:");
	send_UARTASCII(volt);
	send_UARTString("mv,AVdd:");
	send_UARTASCII(AVdd);	
	send_UARTString("mv\r\n");
	#endif

	return volt;	
}

uint16_t ADC_To_Duty(uint16_t adc_value)
{
	uint16_t adc_max = 0;
	uint16_t adc_min = 0;
	uint16_t volt_max = CUSTOM_INPUT_VOLT_MAX(AVdd);//CUSTOM_INPUT_VOLT_MAX(0);
	uint16_t volt_min = CUSTOM_INPUT_VOLT_MIN;
	uint16_t duty = 0;
	uint16_t adc_target = 0;	
	uint16_t interval = DUTY_MAX - DUTY_MIN + 1;	
	u16adc_ref_voltage = AVdd;
	
	adc_max = (ADC_RESOLUTION * volt_max)/u16adc_ref_voltage ;
	adc_min = (ADC_RESOLUTION * volt_min)/u16adc_ref_voltage ;	
	
	adc_target = (adc_value <= adc_min) ? (adc_min) : (adc_value) ;
	adc_target = (adc_target >= adc_max) ? (adc_max) : (adc_target) ;

	duty = (float)(adc_target - adc_min)*interval/(adc_max - adc_min) + 1;
	duty = (duty >= DUTY_MAX) ? (DUTY_MAX) : (duty) ;

	#if (_debug_log_ADC_to_Duty_ == 1)
	send_UARTString("adc_value:");	
	send_UARTASCII(adc_value);
	send_UARTString(",adc_min:");
	send_UARTASCII(adc_min);
	send_UARTString(",adc_max:");
	send_UARTASCII(adc_max);
	send_UARTString(",adc_target:");
	send_UARTASCII(adc_target);
	send_UARTString(",duty:");
	send_UARTASCII(duty);
	send_UARTString("\r\n");
	#endif

	return duty;	
}


uint16_t ADC_ConvertChannel(void)
{
	volatile uint16_t adc_value = 0;

	u16adc_ref_voltage = AVdd;
	
	u16adc_convert_target = (ADC_MIN_TARGET*ADC_RESOLUTION/u16adc_ref_voltage);

	adc_value = ADC_Drop(ADC_SAMPLE_DROP);

	#if defined (ENABLE_ADC_RAW_DATA)
	adc_value = MAKE12BIT(ADCRH,ADCRL);
	
	#endif

	#if defined (ENABLE_ADC_GET_N_DEL_X)
	adc_value = ADC_GetNDelXAvg(ADC_SAMPLE_COUNT , ADC_SAMPLE_DELETE);
	#endif

	#if defined (ENABLE_ADC_AVG)
	adc_value = ADC_Average(ADC_SAMPLE_COUNT);
	#endif

	#if (_debug_log_ADC_Convert_ == 1)
	send_UARTString("adc_value :");	
	send_UARTASCII(adc_value);
	send_UARTString("\r\n");
	#endif

	adc_value = (adc_value <= u16adc_convert_target) ? (u16adc_convert_target) : (adc_value); 
	adc_value = (adc_value >= ADC_RESOLUTION) ? (ADC_RESOLUTION) : (adc_value); 
	
	return adc_value;
}

/*
	(P0.4 , ADC_CH5) : Function1 , ADC
	(P0.3 , ADC_CH6) : Function2 , ADC
	(P1.1 , ADC_CH7) : Function3 , ADC
*/

uint16_t ADC_InitChannel(uint8_t CH)
{
	
	switch(CH)
	{
		case TARGET_CH5: 
		    ENABLE_ADC_AIN5;
			break;

		case TARGET_CH6: 
		    ENABLE_ADC_AIN6;
			break;

		case TARGET_CH7: 
		    ENABLE_ADC_AIN7;
			break;		
	}

  /* ADC Low speed initial*/  
    ADCCON1|=0X30;            /* clock divider */
    ADCCON2|=0X0E;            /* AQT time */

	#if 0
    AUXR1|=SET_BIT4;          /* ADC clock low speed */
	#else
    AUXR1 &= ~SET_BIT4;			//high speed , 500k sps
	#endif

	clr_ADCCON0_ADCF;
	set_ADCCON0_ADCS;                  // ADC start trig signal

	#if 1	//polling
	while(ADCF == 0);
	#else	// Enable ADC interrupt (if use interrupt)
    set_IE_EADC;                        
    ENABLE_GLOBAL_INTERRUPT;
	#endif

//	return (MAKE12BIT(ADCRH,ADCRL));

	return (ADC_ConvertChannel());

}


void application (uint16_t adc_value)
{
	volatile uint16_t duty_value = 0;

	#if defined (ENABLE_CONVERT_ADC_TO_DUTY)
	duty_value = ADC_To_Duty(adc_value);	

	PWM0_CHx_Init(PWM_FREQ , duty_value);
//	PWM0_CHx_Init(2000 , duty_value);

	#else
	ADC_To_Voltage(adc_value);
	#endif

	#if (_debug_log_application_ == 1)
	send_UARTString("adc_value:");	
	send_UARTASCII(adc_value);
	send_UARTString(",duty:");
	send_UARTASCII(duty_value);
	send_UARTString("\r\n");
	#endif
		
}

void task_10ms(void)
{	
	static uint16_t value = 0;
	
	if (is_flag_set(flag_10ms))
	{
		set_flag(flag_10ms,Disable);
		
		//application
		P30 = ~P30;
		
		u16adc_data[Function3] = ADC_InitChannel(TARGET_CH7);		
		u16adc_data[Function2] = ADC_InitChannel(TARGET_CH6);
		u16adc_data[Function1] = ADC_InitChannel(TARGET_CH5);

		#if defined (ENABLE_ADC_LOW_PASS_FILTER)
		value = LowPassFilter(u16adc_OldFilterValue[Function1],u16adc_data[Function1]);
	
		u16adc_OldFilterValue[Function1] = value;
		u16adc_data[Function1] = value;

		#endif
		
		application(u16adc_data[Function1]);
			
	}
}

void task_5ms(void)
{
	if (is_flag_set(flag_5ms))
	{
		set_flag(flag_5ms,Disable);

		//application
//		P17 = ~P17;


	
	}
}

void task_1ms(void)
{
	if (is_flag_set(flag_1ms))
	{
		set_flag(flag_1ms,Disable);

		//application
//		P17 = ~P17;

		
	
	}
}

void task_200us(void)
{
	if (is_flag_set(flag_200us))
	{
		set_flag(flag_200us,Disable);

		//application
//		P17 = ~P17;


	
	}
}

void task_loop(void)
{

	task_200us();
	task_1ms();
	task_5ms();
	task_10ms();

}


void Timer0_IRQHandler(void)
{
	/*
	  	200us base	

	  	1ms = 1000 / 200 = 5
	  	5ms = 5000 / 200 = 25
	  	10ms = 10000 / 200 = 50
	  	
	*/

	const uint16_t div_1ms = 5;	
	static uint16_t cnt_1ms = 0;

	const uint16_t div_5ms = 25;	
	static uint16_t cnt_5ms = 0;	
	
	const uint16_t div_10ms = 50;		//accurate 42 , with ADC conv.
	static uint16_t cnt_10ms = 0;

	set_flag(flag_200us,Enable);

	if (++cnt_1ms >= div_1ms)	
	{
		set_flag(flag_1ms,Enable);	
		cnt_1ms = 0;	
	}
	
	if (++cnt_5ms >= div_5ms)	
	{
		set_flag(flag_5ms,Enable);
		cnt_5ms = 0;	
	}

	if (++cnt_10ms >= div_10ms)
	{
		set_flag(flag_10ms,Enable);
		cnt_10ms = 0;		
	}
	
}

void Timer0_ISR(void) interrupt 1        // Vector @  0x0B
{
    _push_(SFRS);	
	
    TH0 = u8TH0_Tmp;
    TL0 = u8TL0_Tmp;
    clr_TCON_TF0;
	
	Timer0_IRQHandler();

    _pop_(SFRS);	
}

void TIMER0_Init(void)
{
	uint16_t res = 0;

	/*
		formula : 16bit 
		(0xFFFF+1 - target)  / (24MHz/psc) = time base 

	*/	
	const uint16_t TIMER_DIVx_VALUE_200us_FOSC_240000 = 65536-400;

	ENABLE_TIMER0_MODE1;	// mode 0 : 13 bit , mode 1 : 16 bit
    TIMER0_FSYS_DIV12;
	
	u8TH0_Tmp = HIBYTE(TIMER_DIVx_VALUE_200us_FOSC_240000);
	u8TL0_Tmp = LOBYTE(TIMER_DIVx_VALUE_200us_FOSC_240000); 

    TH0 = u8TH0_Tmp;
    TL0 = u8TL0_Tmp;

    ENABLE_TIMER0_INTERRUPT;                       //enable Timer0 interrupt
    ENABLE_GLOBAL_INTERRUPT;                       //enable interrupts
  
    set_TCON_TR0;                                  //Timer0 run
}


void Serial_ISR (void) interrupt 4 
{
    _push_(SFRS);

    if (RI)
    {   
      uart0_receive_flag = 1;
      uart0_receive_data = SBUF;
      clr_SCON_RI;                                         // Clear RI (Receive Interrupt).
    }
    if  (TI)
    {
      if(!BIT_UART)
      {
          TI = 0;
      }
    }

    _pop_(SFRS);	
}

void UART0_Init(void)
{
	#if 1
	const unsigned long u32Baudrate = 115200;
	P06_QUASI_MODE;    //Setting UART pin as Quasi mode for transmit
	
	SCON = 0x50;          //UART0 Mode1,REN=1,TI=1
	set_PCON_SMOD;        //UART0 Double Rate Enable
	T3CON &= 0xF8;        //T3PS2=0,T3PS1=0,T3PS0=0(Prescale=1)
	set_T3CON_BRCK;        //UART0 baud rate clock source = Timer3

	RH3    = HIBYTE(65536 - (SYS_CLOCK/16/u32Baudrate));  
	RL3    = LOBYTE(65536 - (SYS_CLOCK/16/u32Baudrate));  
	
	set_T3CON_TR3;         //Trigger Timer3
	set_IE_ES;

	ENABLE_GLOBAL_INTERRUPT;

	set_SCON_TI;
	BIT_UART=1;
	#else	
    UART_Open(SYS_CLOCK,UART0_Timer3,115200);
    ENABLE_UART0_PRINTF; 
	#endif
}


void MODIFY_HIRC_24(void)
{
	unsigned char u8HIRCSEL = HIRC_24;
    unsigned char data hircmap0,hircmap1;
//    unsigned int trimvalue16bit;
    /* Check if power on reset, modify HIRC */
    SFRS = 0 ;
	#if 1
    IAPAL = 0x38;
	#else
    switch (u8HIRCSEL)
    {
      case HIRC_24:
        IAPAL = 0x38;
      break;
      case HIRC_16:
        IAPAL = 0x30;
      break;
      case HIRC_166:
        IAPAL = 0x30;
      break;
    }
	#endif
	
    set_CHPCON_IAPEN;
    IAPAH = 0x00;
    IAPCN = READ_UID;
    set_IAPTRG_IAPGO;
    hircmap0 = IAPFD;
    IAPAL++;
    set_IAPTRG_IAPGO;
    hircmap1 = IAPFD;
    clr_CHPCON_IAPEN;

	#if 0
    switch (u8HIRCSEL)
    {
		case HIRC_166:
		trimvalue16bit = ((hircmap0 << 1) + (hircmap1 & 0x01));
		trimvalue16bit = trimvalue16bit - 15;
		hircmap1 = trimvalue16bit & 0x01;
		hircmap0 = trimvalue16bit >> 1;

		break;
		default: break;
    }
	#endif
	
    TA = 0xAA;
    TA = 0x55;
    RCTRIM0 = hircmap0;
    TA = 0xAA;
    TA = 0x55;
    RCTRIM1 = hircmap1;
    clr_CHPCON_IAPEN;
    PCON &= CLR_BIT4;
}


void SYS_Init(void)
{
    MODIFY_HIRC_24();

    ALL_GPIO_QUASI_MODE;
    ENABLE_GLOBAL_INTERRUPT;                // global enable bit	
}

void main (void) 
{
    SYS_Init();

    UART0_Init();
	GPIO_Init();					

	//TimerService  : 200us , 1ms , 5ms , 10ms
	TIMER0_Init();	// us base
	
	ADC_ReadAVdd();
	
    while(1)
    {
		task_loop();
		
    }
}



