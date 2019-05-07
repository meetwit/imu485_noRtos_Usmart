#include "main.h" 

//初始化IO 串口4
//pclk2:PCLK2时钟频率(Mhz)
//bound:波特率 
void uart4_init(u32 pclk2,u32 bound)
{  	 
	float temp;
	u16 mantissa;
	u16 fraction;	   
	temp=(float)(pclk2*1000000)/(bound*16);//得到USARTDIV@OVER8=0
	mantissa=temp;				 //得到整数部分
	fraction=(temp-mantissa)*16; //得到小数部分@OVER8=0 
    mantissa<<=4;
	mantissa+=fraction; 
	RCC->AHB1ENR|=1<<0;   	//使能PORTA口时钟  
	RCC->APB1ENR|=1<<19;  	//使能串口4时钟 
	GPIO_Set(GPIOA,PIN0|PIN1,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_PU);
 	GPIO_AF_Set(GPIOA,0,8);	//PA0,AF8
	GPIO_AF_Set(GPIOA,1,8);//PA1,AF8  		   
	//波特率设置
 	UART4->BRR=mantissa; 	//波特率设置	 
	UART4->CR1&=~(1<<15); 	//设置OVER8=0 
	UART4->CR1|=1<<3;  	//串口发送使能 

//	UART4->CR1|=1<<2;  		//串口接收使能
//	UART4->CR1|=1<<5;    		//接收缓冲区非空中断使能	    	
//	MY_NVIC_Init(3,3,UART4_IRQn,2);//组2，最低优先级 
	
	UART4->CR1|=1<<13;  	//串口使能
}

void PcTx_Byte4(u8 Tx_data4){
//	rt_interrupt_enter();
	while(!(UART4->SR &(0x1<<7)));
	UART4 ->DR = Tx_data4;
//	rt_interrupt_leave();
}
//初始化IO 串口5
//pclk2:PCLK2时钟频率(Mhz)
//bound:波特率 
void uart5_init(u32 pclk2,u32 bound)
{  	 
	float temp;
	u16 mantissa;
	u16 fraction;	   
	temp=(float)(pclk2*1000000)/(bound*16);//得到USARTDIV@OVER8=0
	mantissa=temp;				 //得到整数部分
	fraction=(temp-mantissa)*16; //得到小数部分@OVER8=0 
    mantissa<<=4;
	mantissa+=fraction;  
	RCC->AHB1ENR|=1<<2;   	//使能PORTC口时钟  
	RCC->AHB1ENR|=1<<3;   	//使能PORTD口时钟  
	RCC->APB1ENR|=1<<20;  	//使能串口5时钟 
	GPIO_Set(GPIOD,PIN2,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_PU);
	GPIO_Set(GPIOC,PIN12,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_PU);
 	GPIO_AF_Set(GPIOD,2,8);	//PD2,AF8
	GPIO_AF_Set(GPIOC,12,8);//PC12,AF8  		   
	//波特率设置
 	UART5->BRR=mantissa; 	//波特率设置	 
	UART5->CR1&=~(1<<15); 	//设置OVER8=0 
	UART5->CR1|=1<<3;  	//串口发送使能 

	UART5->CR1|=1<<2;  		//串口接收使能
	UART5->CR1|=1<<5;    		//接收缓冲区非空中断使能	    	
	MY_NVIC_Init(2,2,UART5_IRQn,2);//组2，最低优先级 
	
	UART5->CR1|=1<<13;  	//串口使能
}

void PcTx_Byte5(u8 Tx_data5){
	while(!(UART5->SR &(0x1<<7)));
	UART5 ->DR = Tx_data5;
}



void UART5_IRQHandler(void)
{
	u8 res;	    
	if(UART5->SR&(1<<5))//接收到数据
	{	 
		res=UART5->DR; 		
		if(swt[6]==0)
		cope_uart5(res);
	}  										
} 


void cope_uart5(u8 data){
	if(data=='R'){
		send_F7();
	}
}


