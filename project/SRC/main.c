#include "main.h"

int main(void)
{			
	u32 hb=0;
	Stm32_Clock_Init(336,8,2,7);
	delay_init(168);
	hx711_init();
	uart_init(84,460800);			//USART1 460800
	usmart_dev.init(84); 		//初始化USMART
	RS485_Init(42,115200);		//初始化RS485 usart2
	RS485_Init2(42,115200);		//初始化RS485 usart3
	uart4_init(42,500000);		//匿名 500000
	uart5_init(42,460800);		//F4<->F7 460800
	printf("\r\nsys init\r\n\r\n");
	
	while(1){
		hb++;
		read_Imu();																						//swt 789 控制角度 角速度 加速度	
		imu_find_point();																			//主要的imu逻辑
		send_ANO();																						//匿名打印
//		if(swt[0]) printf("\r\r run time %13d\r\n",hb);			//show systerm runing
		if(hb%2) read_Hx711();																//swt 0123控制hx711 1234
	}
	
}
