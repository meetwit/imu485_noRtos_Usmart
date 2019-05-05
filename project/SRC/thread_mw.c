#include "main.h"

/*
swt index  
0				开关串口1的时钟打印， 			等于0，关闭；等于1，开启
1				拉力传感器1的读取控制， 		等于0，关闭；等于1，开启
2				拉力传感器2的读取控制， 		等于0，关闭；等于1，开启
3				拉力传感器3的读取控制， 		等于0，关闭；等于1，开启
4				拉力传感器4的读取控制， 		等于0，关闭；等于1，开启
5				串口4匿名上位机显示控制， 	等于0，关闭；等于1，开启
6				串口5与F7的通信控制， 			等于0，关闭；等于1，开启
7				imu角度测量控制，						等于0，关闭；等于1，开启
8				imu角速度测量控制，					等于0，关闭；等于1，开启
9				imu加速度测量控制，					等于0，关闭；等于1，开启
*/
/*
ANO index  
0				拉力1
1				拉力2
2				拉力3
3				拉力4
4				ADDR为双数的 角度
5				ADDR为单数的 角度
6				ADDR为双数的 角速度
7				ADDR为单数的 角速度
8				触发点
9				
*/
/*							 0 1 2 3 4 5 6 7 8 9*/
uint8_t swt[10]={1,1,1,1,1,1,1,1,1,0}; //全局一些状态开关的控制


void com_F7(void){
	;
}


void send_ANO(void){
	if(swt[5]){
		ANO_send(0xf2,(unsigned char *)ano_o,sizeof(float),sizeof(ano_o));
	}
}


void swt_f(int num,int val){
		swt[num] = val;
		switch(num){
			case 0 :
				if(val){
					rt_kprintf("全局时钟打印开启\r\n");
				}else{
					rt_kprintf("全局时钟打印关闭\r\n");
				}
			break;
				
			case 1 :
				if(val){
					rt_kprintf("拉力传感器com1开启测量\r\n");
				}else{
					rt_kprintf("拉力传感器com1关闭测量\r\n");
				}
			break;
				
			case 2 :
				if(val){
					rt_kprintf("拉力传感器com2开启测量\r\n");
				}else{
					rt_kprintf("拉力传感器com2关闭测量\r\n");
				}
			break;
				
			case 3 :
				if(val){
					rt_kprintf("拉力传感器com3开启测量\r\n");
				}else{
					rt_kprintf("拉力传感器com3关闭测量\r\n");
				}
			break;
				
			case 4 :
				if(val){
					rt_kprintf("拉力传感器com4开启测量\r\n");
				}else{
					rt_kprintf("拉力传感器com4关闭测量\r\n");
				}
			break;
				
			case 5 :
				if(val){
					rt_kprintf("匿名上位机开启显示\r\n");
				}else{
					rt_kprintf("匿名上位机关闭显示\r\n");
				}
			break;
				
			case 6 :
				if(val){
					rt_kprintf("开启F4&F7通信\r\n");
				}else{
					rt_kprintf("关闭F4&F7通信\r\n");
				}
			break;
				
			case 7 :
				if(val){
					rt_kprintf("imu角度开启测量\r\n");
				}else{
					rt_kprintf("imu角度关闭测量\r\n");
				}
			break;
				
			case 8 :
				if(val){
					rt_kprintf("imu角速度开启测量\r\n");
				}else{
					rt_kprintf("imu角速度关闭测量\r\n");
				}
			break;
				
			case 9 :
				if(val){
					rt_kprintf("imu加速度开启测量\r\n");
				}else{
					rt_kprintf("imu加速度关闭测量\r\n");
				}
			break;
		}
	
}


void swt_help(){
//		rt_kprintf("你输入的格式不对，参考如下：\r\n");
//		rt_kprintf("swt_f 0 0 全局时钟打印关闭\r\n");
//		rt_kprintf("swt_f 0 1 全局时钟打印开启\r\n");
		rt_kprintf("\r\nswt_f 1 0 拉力传感器com1关闭测量\r\n");
		rt_kprintf("swt_f 1 1 拉力传感器com1开启测量\r\n");
		rt_kprintf("swt_f 2 0 拉力传感器com2关闭测量\r\n");
		rt_kprintf("swt_f 2 1 拉力传感器com2开启测量\r\n");
		rt_kprintf("swt_f 3 0 拉力传感器com3关闭测量\r\n");
		rt_kprintf("swt_f 3 1 拉力传感器com3开启测量\r\n");
		rt_kprintf("swt_f 4 0 拉力传感器com4关闭测量\r\n");
		rt_kprintf("swt_f 4 1 拉力传感器com4开启测量\r\n");
		rt_kprintf("swt_f 5 0 匿名上位机关闭显示\r\n");
		rt_kprintf("swt_f 5 1 匿名上位机开启显示\r\n");
		rt_kprintf("swt_f 6 0 关闭F4&F7通信\r\n");
		rt_kprintf("swt_f 6 1 开启F4&F7通信\r\n");
		rt_kprintf("swt_f 7 0 imu角度关闭测量\r\n");
		rt_kprintf("swt_f 7 1 imu角度开启测量\r\n");
		rt_kprintf("swt_f 8 0 imu角速度关闭测量\r\n");
		rt_kprintf("swt_f 8 1 imu角速度开启测量\r\n");
		rt_kprintf("swt_f 9 0 imu加速度关闭测量\r\n");
		rt_kprintf("swt_f 9 1 imu加速度开启测量\r\n");
}
