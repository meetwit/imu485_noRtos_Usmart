#include "main.h" 

//��ʼ��IO ����4
//pclk2:PCLK2ʱ��Ƶ��(Mhz)
//bound:������ 
void uart4_init(u32 pclk2,u32 bound)
{  	 
	float temp;
	u16 mantissa;
	u16 fraction;	   
	temp=(float)(pclk2*1000000)/(bound*16);//�õ�USARTDIV@OVER8=0
	mantissa=temp;				 //�õ���������
	fraction=(temp-mantissa)*16; //�õ�С������@OVER8=0 
    mantissa<<=4;
	mantissa+=fraction; 
	RCC->AHB1ENR|=1<<0;   	//ʹ��PORTA��ʱ��  
	RCC->APB1ENR|=1<<19;  	//ʹ�ܴ���4ʱ�� 
	GPIO_Set(GPIOA,PIN0|PIN1,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_PU);
 	GPIO_AF_Set(GPIOA,0,8);	//PA0,AF8
	GPIO_AF_Set(GPIOA,1,8);//PA1,AF8  		   
	//����������
 	UART4->BRR=mantissa; 	//����������	 
	UART4->CR1&=~(1<<15); 	//����OVER8=0 
	UART4->CR1|=1<<3;  	//���ڷ���ʹ�� 

//	UART4->CR1|=1<<2;  		//���ڽ���ʹ��
//	UART4->CR1|=1<<5;    		//���ջ������ǿ��ж�ʹ��	    	
//	MY_NVIC_Init(3,3,UART4_IRQn,2);//��2��������ȼ� 
	
	UART4->CR1|=1<<13;  	//����ʹ��
}

void PcTx_Byte4(u8 Tx_data4){
//	rt_interrupt_enter();
	while(!(UART4->SR &(0x1<<7)));
	UART4 ->DR = Tx_data4;
//	rt_interrupt_leave();
}
//��ʼ��IO ����5
//pclk2:PCLK2ʱ��Ƶ��(Mhz)
//bound:������ 
void uart5_init(u32 pclk2,u32 bound)
{  	 
	float temp;
	u16 mantissa;
	u16 fraction;	   
	temp=(float)(pclk2*1000000)/(bound*16);//�õ�USARTDIV@OVER8=0
	mantissa=temp;				 //�õ���������
	fraction=(temp-mantissa)*16; //�õ�С������@OVER8=0 
    mantissa<<=4;
	mantissa+=fraction;  
	RCC->AHB1ENR|=1<<2;   	//ʹ��PORTC��ʱ��  
	RCC->AHB1ENR|=1<<3;   	//ʹ��PORTD��ʱ��  
	RCC->APB1ENR|=1<<20;  	//ʹ�ܴ���5ʱ�� 
	GPIO_Set(GPIOD,PIN2,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_PU);
	GPIO_Set(GPIOC,PIN12,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_PU);
 	GPIO_AF_Set(GPIOD,2,8);	//PD2,AF8
	GPIO_AF_Set(GPIOC,12,8);//PC12,AF8  		   
	//����������
 	UART5->BRR=mantissa; 	//����������	 
	UART5->CR1&=~(1<<15); 	//����OVER8=0 
	UART5->CR1|=1<<3;  	//���ڷ���ʹ�� 

	UART5->CR1|=1<<2;  		//���ڽ���ʹ��
	UART5->CR1|=1<<5;    		//���ջ������ǿ��ж�ʹ��	    	
	MY_NVIC_Init(2,2,UART5_IRQn,2);//��2��������ȼ� 
	
	UART5->CR1|=1<<13;  	//����ʹ��
}

void PcTx_Byte5(u8 Tx_data5){
	while(!(UART5->SR &(0x1<<7)));
	UART5 ->DR = Tx_data5;
}



void UART5_IRQHandler(void)
{
	u8 res;	    
	if(UART5->SR&(1<<5))//���յ�����
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


