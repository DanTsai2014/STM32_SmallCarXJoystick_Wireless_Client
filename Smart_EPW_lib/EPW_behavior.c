#include "FreeRTOS.h"
#include "timers.h"
#include "stm32f4xx_syscfg.h"
#include "EPW_behavior.h"
#include "timers.h"
#include "stm32f4xx_usart.h"
#include "uart.h"

char buff_JOY_x [] = "";
char buff_JOY_y [] = "";
//char buff_JOY_xx [] = "";
//char buff_JOY_yy [] = "";
char buff_acc_x_1 [] = "";
char buff_acc_y_1 [] = "";
char buff_acc_z_1 [] = "";
char buff_ang_x_1 [] = "";
char buff_ang_y_1 [] = "";
char buff_ang_z_1 [] = "";
char buff_acc_x_2 [] = "";
char buff_acc_y_2 [] = "";
char buff_acc_z_2 [] = "";
char buff_ang_x_2 [] = "";
char buff_ang_y_2 [] = "";
char buff_ang_z_2 [] = "";
s16 accgyo_1[6]={0};
s16 accgyo_2[6]={0};
//fuzzy variable
float membership[5];
int16_t xthreshold1 = 1500;
int16_t xthreshold2 = 3000;
int16_t ythreshold1 = 1500;
int16_t ythreshold2 = 3000;
char xthreshold1_buffer [] = "";
char xthreshold2_buffer [] = "";
char ythreshold1_buffer [] = "";
char ythreshold2_buffer [] = "";
char dir;
char correct [] = "";
/*============================================================================*/
/*============================================================================*
 ** function : init_ADC
 ** brief : initialize ADC reading setting
 **param : None
 **retval : None
 **============================================================================*/
 /*============================================================================*/

void init_ADC1(void){
	ADC_InitTypeDef ADC_InitStructure; //Structure for adc configuration
    ADC_CommonInitTypeDef ADC_CommonInitStructure;
	GPIO_InitTypeDef GPIO_initStructre; //Structure for analog input pin
	ADC_StructInit(&ADC_InitStructure);
    ADC_CommonStructInit(&ADC_CommonInitStructure);
	//Clock configuration
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); //The ADC1 is connected the APB2 peripheral bus thus we will use its clock source
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_DMA2, ENABLE); //Clock for the ADC port!! Do not forget about this one ;)
    //Analog pin configuration
	GPIO_StructInit(&GPIO_initStructre);
	GPIO_initStructre.GPIO_Pin = JOYSTICK_X_AXIS_PIN | JOYSTICK_Y_AXIS_PIN; //The channel 10 is connected to PC0; PC1 if multiple channels
	GPIO_initStructre.GPIO_Mode = GPIO_Mode_AN; //The PC0 pin is configured in analog mode
	GPIO_initStructre.GPIO_PuPd = GPIO_PuPd_NOPULL; //We don't need any pull up or pull down
	GPIO_Init(JOYSTICK_PORT, &GPIO_initStructre); //Affecting the port with the initialization structure configuration

	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    ADC_CommonInit(&ADC_CommonInitStructure);
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; //data converted will be shifted to right
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b; //Input voltage is converted into a 12bit number giving a maximum value of 4095
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; //the conversion is continuous, the input data is converted more than once
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1; // conversion is synchronous with TIM1 and CC1 (use timer 1 capture/compare channel 1 for external trigger)
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None; //no trigger for conversion
	ADC_InitStructure.ADC_NbrOfConversion = 2; //Number of used ADC channels;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE; //The scan is configured in muptiple channels
	ADC_Init(ADC1, &ADC_InitStructure); //Initialize ADC with the previous configuration
	

	DMA_InitTypeDef DMA_InitStructure; //Structure for DMA configuration
	DMA_DeInit(DMA2_Stream4);
	DMA_StructInit(&DMA_InitStructure);
    //DMA2 Channel0 stream0 configuration
	DMA_InitStructure.DMA_Channel = DMA_Channel_0; //DMA channel
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR; //DMA address
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; //u32 //Peripheral Data Size 32bit (DMA_{PeripheralDataSize_HalfWord 16bit})
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC1ConvertedVoltage; //buffer address
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;//傳輸方向單向
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;//DMA Memory Data Size 32bit
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; //接收一次數據後，目標內存地址是否後移--重要概念，用來采集多個數據的，多通道的時候需要使能它
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//接收一次數據後，設備地址是否後移
    DMA_InitStructure.DMA_Mode  = DMA_Mode_Circular;//轉換模式，循環緩存模式，常用
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;//DMA優先級，高
    DMA_InitStructure.DMA_BufferSize = 2;//DMA緩存大小，1*2個
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = 0;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    //send values to DMA registers
    DMA_Init(DMA2_Stream4, &DMA_InitStructure);
    // Enable DMA2 Channel Transfer Complete interrupt
    DMA_ITConfig(DMA2_Stream4, DMA_IT_TC, ENABLE);
    //Enable DMA1 Channel transfer
    DMA_Cmd(DMA2_Stream4, ENABLE);
	//Select the channel to be read from
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_144Cycles); //// use channel 10 from ADC1, with sample time 144 cycles
	ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 2, ADC_SampleTime_144Cycles); //ADC1 multiple channels (channel 11)
	//Enable DMA request after last transfer (Single-ADC mode)
    ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
    //Enable using ADC_DMA
    ADC_DMACmd(ADC1, ENABLE);
    //Enable ADC conversion
	ADC_Cmd(ADC1, ENABLE);
}

/*============================================================================*/
/*============================================================================*
 ** function : parse_Joystick_dir
 ** brief : parse Joystick direction from the uart siganl
 ** param : Joystick_cmd
 ** retval :  None
 **============================================================================*/
/*============================================================================*/

void parse_Joystick_dir(void *pvParameters)
{
	while(1)
	{ //2305: static x-axis mean, 2362: static y-axis mean
		
        //right fuzzy
        if((ADC1ConvertedVoltage[0]-xthreshold1)>=-400 && (ADC1ConvertedVoltage[0]-xthreshold1)<=400)
        {
            //float membership[5]={-800,-400,0,400,800};
        	xthreshold1=xthreshold1+fuzzy_th(ADC1ConvertedVoltage[0]-xthreshold1);
            if(xthreshold1<1000)
            {
            	xthreshold1=1100;
            }
            if(xthreshold1>2000)
            {
            	xthreshold1=1900;
            }
        }

        //left fuzzy
        if((ADC1ConvertedVoltage[0]-xthreshold2)>=-400 && (ADC1ConvertedVoltage[0]-xthreshold2)<=400)
        {
        	//float membership[5]={-800,-400,0,400,800};
        	xthreshold2=xthreshold2+fuzzy_th(ADC1ConvertedVoltage[0]-xthreshold2);
            if(xthreshold2<2500)
            {
            	xthreshold2=2600;
            }
            if(xthreshold2>3500)
            {
        	    xthreshold2=3400;
        	}
        }
        //backward fuzzy
        if((ADC1ConvertedVoltage[1]-ythreshold1)>=-400 && (ADC1ConvertedVoltage[1]-ythreshold1)<=400)
        {
        	//float membership[5]={-800,-400,0,400,800};
            ythreshold1=ythreshold1+fuzzy_th(ADC1ConvertedVoltage[1]-ythreshold1);
            if(ythreshold1<1000)
            {
            	ythreshold1=1100;
            }
            if(ythreshold1>2000)
            {
            	ythreshold1=1900;
            }
        }
        //forward fuzzy
        if((ADC1ConvertedVoltage[1]-ythreshold2)>=-400 && (ADC1ConvertedVoltage[1]-ythreshold2)<=400)
        {
        	//float membership[5]={-800,-400,0,400,800};
        	ythreshold2=ythreshold2+fuzzy_th(ADC1ConvertedVoltage[1]-ythreshold2);
            if(ythreshold2<2500)
            {
            	ythreshold2=2600;
            }
            if(ythreshold2>3500)
            {
            	ythreshold2=3400;
            }
        }
        vTaskDelay(10);       
    }
}

void send_Joystick_dir(){
    //movements
    while(1){
        if(ADC1ConvertedVoltage[0]>xthreshold1 && ADC1ConvertedVoltage[0]<xthreshold2 && ADC1ConvertedVoltage[1]>ythreshold1 && ADC1ConvertedVoltage[1]<ythreshold2)
        //if(ADC1ConvertedVoltage[0]>1500 && ADC1ConvertedVoltage[0]<3000 && ADC1ConvertedVoltage[1]>1500 && ADC1ConvertedVoltage[1]<3000)
        {
            USART_puts(USART3, "s"); //stop
            //USART_puts(USART2, "sd");
            //vTaskDelay(10);
        }
        if(ADC1ConvertedVoltage[0] < xthreshold1) //right
        {
            USART_puts(USART3, "r");
            //USART_puts(USART2, "rd");
            //vTaskDelay(10);
        }
        if(ADC1ConvertedVoltage[0] > xthreshold2) //left
        {
            USART_puts(USART3, "l");
            //USART_puts(USART2, "ld");
            //vTaskDelay(10);
        }
        if(ADC1ConvertedVoltage[1] < ythreshold1) //backward
        {
            USART_puts(USART3, "b");
            //USART_puts(USART2, "bd");
            //vTaskDelay(10);
        }
        if(ADC1ConvertedVoltage[1] > ythreshold2) //forward
        {
            USART_puts(USART3, "f");
            //USART_puts(USART2, "fd");
            //vTaskDelay(10);
        }
        //vTaskDelay(1000);
    }
}

void send_Joystick_MPU6050_data(){
	while(1){
		USART_puts(USART2, "DATA,TIME,,");

		sprintf(buff_JOY_x, "%d,", ADC1ConvertedVoltage[0]);
		Usart2_Printf(buff_JOY_x);
		sprintf(buff_JOY_y, "%d,", ADC1ConvertedVoltage[1]);
		Usart2_Printf(buff_JOY_y);

        //dynamic threshold (fuzzy)
		/*sprintf(xthreshold1_buffer, "x1=%d\r\n", xthreshold1);
		Usart2_Printf(xthreshold1_buffer);
		sprintf(xthreshold2_buffer, "x2=%d\r\n", xthreshold2);
		Usart2_Printf(xthreshold2_buffer);
		sprintf(ythreshold1_buffer, "y1=%d\r\n", ythreshold1);
		Usart2_Printf(ythreshold1_buffer);
		sprintf(ythreshold2_buffer, "y2=%d\r\n", ythreshold2);
		Usart2_Printf(ythreshold2_buffer);*/

        //USART_puts(USART3, dir);
/*
		sprintf(buff_JOY_xx, "%d,",Joystick_x_Filter);
		Usart3_Printf(buff_JOY_xx);
		sprintf(buff_JOY_yy, "%d,",Joystick_y_Filter);
		Usart3_Printf(buff_JOY_yy);*/

		MPU6050_GetRawAccelGyro_1(accgyo_1);
		//MPU6050_GetRawAccelGyro_2(accgyo_2);

        //sprintf(buff_acc_x_1, "%d,", accgyo_1[0]);
        sprintf(buff_acc_x_1, "%d,", map(accgyo_1[0]+1478, -32768, 32767, 0, 4095));
        Usart2_Printf(buff_acc_x_1);
        //sprintf(buff_acc_y_1, "%d,", accgyo_1[1]);
        sprintf(buff_acc_y_1, "%d,", map(accgyo_1[1]+348, -32768, 32767, 0, 4095));
        Usart2_Printf(buff_acc_y_1);
        //sprintf(buff_acc_z_1, "%d,", accgyo_1[2]);
        sprintf(buff_acc_z_1, "%d,", map(accgyo_1[2]+3363, -32768, 32767, 0, 4095));
        Usart2_Printf(buff_acc_z_1);
/*
        sprintf(buff_acc_x_2, "%d,", accgyo_2[0]);
        Usart2_Printf(buff_acc_x_2);
        sprintf(buff_acc_y_2, "%d,", accgyo_2[1]);
        Usart2_Printf(buff_acc_y_2);
        sprintf(buff_acc_z_2, "%d,", accgyo_2[2]);
        Usart2_Printf(buff_acc_z_2);
*/
        //sprintf(buff_ang_x_1, "%d,", accgyo_1[3]);
        sprintf(buff_ang_x_1, "%d,", map(accgyo_1[3]+518, -32768, 32767, 0, 4095));
        Usart2_Printf(buff_ang_x_1);
        //sprintf(buff_ang_y_1, "%d,", accgyo_1[4]);
        sprintf(buff_ang_y_1, "%d,", map(accgyo_1[4]+9, -32768, 32767, 0, 4095));
        Usart2_Printf(buff_ang_y_1);
        //sprintf(buff_ang_z_1, "%d\r\n", accgyo_1[5]);
        sprintf(buff_ang_z_1, "%d\r\n", map(accgyo_1[5]+238, -32768, 32767, 0, 4095));
        Usart2_Printf(buff_ang_z_1);
/*
        sprintf(buff_ang_x_2, "%d,", accgyo_2[3]);
        Usart2_Printf(buff_ang_x_2);
        sprintf(buff_ang_y_2, "%d,", accgyo_2[4]);
        Usart2_Printf(buff_ang_y_2);
        sprintf(buff_ang_z_2, "%d\r\n", accgyo_2[5]);
        Usart2_Printf(buff_ang_z_2);
*/
        vTaskDelay(200);
    }
}

int fuzzy_th(int error)
        {
            char n;
            float membership[5]={-800,-400,0,400,800};
            float rule[5]={50,25,0,-25,-50};
            float num=0,den=0,crisp=0,grade1,grade2,slope;
            
            if(error<=membership[0])
            {
                grade1=1; grade2=0; num=num+rule[0]; den=den+1;
            }
            else if(error>membership[4])
            {
                grade1=0; grade2=1; num=num+rule[4]; den=den+1;
            }
            else
            {
                for(n=0;n<4;n++)
                {
                    slope=-1/(membership[n+1]-membership[n]);
                    if((error>membership[n])&&(error<=membership[n+1]))
                    {
                        grade1=slope*(error-membership[n+1]);
                        grade2=slope*(membership[n]-error);
                        num=num+grade1*rule[n]+grade2*rule[n+1];
                        den=den+grade1+grade2;
                    }
                }
            }
            crisp=num/den;
            return crisp;
        }

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}