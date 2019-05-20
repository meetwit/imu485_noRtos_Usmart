#include "main.h"

#define MaxN 200
#define MinN 10


/*
algorithm 0 first version
algorithm 1 计算角度到目标输出力

*/
#define algorithm 0		

/*		      		 0 1 2 3 4 5 6 7 8 9*/
uint8_t swt[10]={1,1,1,1,1,1,0,1,1,0}; //全局一些状态开关的控制
float target_N[4]={10,10,10,10};			//0:双数髋前 1：单数髋前 2:双数髋后 3：单数髋后 

void send_F7(void){
//将目标力 target_N[4]，测量力 hx711_N[4] 通过串口5发送至F7	float
//串口5的字节发送函数	PcTx_Byte5(u8 data)		unsigned char
	PcTx_Byte5('T');
	
	PcTx_Byte5((u8)(hx711_N[0]/2));
	PcTx_Byte5((u8)(hx711_N[1]/2));
	PcTx_Byte5((u8)(hx711_N[2]/2));
	PcTx_Byte5((u8)(hx711_N[3]/2));
	
	PcTx_Byte5((u8)(target_N[0]/2));
	PcTx_Byte5((u8)(target_N[1]/2));
	PcTx_Byte5((u8)(target_N[2]/2));
	PcTx_Byte5((u8)(target_N[3]/2));
	
	PcTx_Byte5('E');
}

void com_F7(void){
	float temp_N[2]={0};
	temp_N[1] = generatrForce(state_F,stcIMU[3].x_Angle,stcIMU[3].x_w);
	ano_o_F2[9] = state_F;
	if(state_F==1){											//状态切换
		state_F=12;
	}
	else
	if(state_F==2){
		state_F=23;
	}
	else
	if(state_F==3){
		state_F=34;
	}
	else
	if(state_F==4){
		state_F=41;
	}
	temp_N[0] = generatrForce2(state_S,stcIMU[4].x_Angle,stcIMU[4].x_w);
	ano_o_F2[8] = state_S;
	if(state_S==1){
		state_S=12;
	}
	else
	if(state_S==2){
		state_S=23;
	}
	else
	if(state_S==3){
		state_S=34;
	}
	else
	if(state_S==4){
		state_S=41;
	}
	
	if(state_F==12||state_F==23){					//标记切换
		target_N[3] = temp_N[1];
		target_N[1] = MinN;
	}
	if(state_F==34||state_F==41){
		target_N[1] = temp_N[1];
		target_N[3] = MinN;
	}
	if(state_S==12||state_S==23){
		target_N[2] = temp_N[0];
		target_N[0] = MinN;
	}
	if(state_S==34||state_S==41){
		target_N[0] = temp_N[0];
		target_N[2] = MinN;
	}
	
	ano_o_F3[0] = target_N[0];						//输出到匿名的F3 0123
	ano_o_F3[1] = target_N[1];
	ano_o_F3[2] = target_N[2];
	ano_o_F3[3] = target_N[3];
	
	
	if(swt[6]){
		send_F7();
	}
}


void send_ANO(void){
	if(swt[5]){
		ANO_send(0xf2,(unsigned char *)ano_o_F2,sizeof(float),sizeof(ano_o_F2));
		ANO_send(0xf3,(unsigned char *)ano_o_F3,sizeof(float),sizeof(ano_o_F3));
	}
}



void swt_f(int num,int val){
		swt[num] = val;
		switch(num){
//			case 0 :
//				if(val){
//					rt_kprintf("全局时钟打印开启\r\n");
//				}else{
//					rt_kprintf("全局时钟打印关闭\r\n");
//				}
//			break;
				
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
					rt_kprintf("F4主动发送数据到F7\r\n");
				}else{
					rt_kprintf("F4被动发送数据到F7\r\n");
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
		rt_kprintf("swt_f 6 0 F4被动发送数据到F7\r\n");
		rt_kprintf("swt_f 6 1 F4主动发送数据到F7\r\n");
		rt_kprintf("swt_f 7 0 imu角度关闭测量\r\n");
		rt_kprintf("swt_f 7 1 imu角度开启测量\r\n");
		rt_kprintf("swt_f 8 0 imu角速度关闭测量\r\n");
		rt_kprintf("swt_f 8 1 imu角速度开启测量\r\n");
		rt_kprintf("swt_f 9 0 imu加速度关闭测量\r\n");
		rt_kprintf("swt_f 9 1 imu加速度开启测量\r\n");
}

void swt_check(){
		rt_kprintf("\r\n\n状态查询：\r\n");
//		if(swt[0]){
//					rt_kprintf("全局时钟打印已开启\r\n");
//				}else{
//					rt_kprintf("全局时钟打印已关闭\r\n");
//				}
		if(swt[1]){
					rt_kprintf("拉力传感器com1已开启测量\r\n");
				}else{
					rt_kprintf("拉力传感器com1已关闭测量\r\n");
				}
		if(swt[2]){
					rt_kprintf("拉力传感器com2已开启测量\r\n");
				}else{
					rt_kprintf("拉力传感器com2已关闭测量\r\n");
				}
		if(swt[3]){
					rt_kprintf("拉力传感器com3已开启测量\r\n");
				}else{
					rt_kprintf("拉力传感器com3已关闭测量\r\n");
				}
		if(swt[4]){
					rt_kprintf("拉力传感器com4已开启测量\r\n");
				}else{
					rt_kprintf("拉力传感器com4已关闭测量\r\n");
				}
		if(swt[5]){
					rt_kprintf("匿名上位机已开启显示\r\n");
				}else{
					rt_kprintf("匿名上位机已关闭显示\r\n");
				}
		if(swt[6]){
					rt_kprintf("F4主动发送数据到F7\r\n");
				}else{
					rt_kprintf("F4被动发送数据到F7\r\n");
				}
		if(swt[7]){
					rt_kprintf("imu角度已开启测量\r\n");
				}else{
					rt_kprintf("imu角度已关闭测量\r\n");
				}
		if(swt[8]){
					rt_kprintf("imu角速度已开启测量\r\n");
				}else{
					rt_kprintf("imu角速度已关闭测量\r\n");
				}
		if(swt[9]){
					rt_kprintf("imu加速度已开启测量\r\n");
				}else{
					rt_kprintf("imu加速度已关闭测量\r\n");
				}
}


float generatrForce(int flag, float angle, float v){
	static float max_angle=110,min_angle=60,mid2_angle=85,mid4_angle=85;
	float k12,k23,k34,k41,N;
	static u8 cout=0;

	if(flag==1){
		max_angle = angle;
		N = MinN;
	}else if(flag==3){
		min_angle = angle;
		N = MinN;
	}else if(flag==2){
		mid2_angle = angle;
		if(v>-50&&v<50) {
			cout = 11;
			return MinN;
		}
		N = MaxN;
	}else if(flag==4){
		mid4_angle = angle;
		if(v>-50&&v<50) {
			cout = 11;
			return MinN;
		}
		N = MaxN;
	}
	if(-10<v&&v<10){
		cout++;
		if(cout>200){
			cout = 11;
		}
	}else{
		cout=0;
	}
	if(cout>10){
		return MinN;
	}
	
	if(max_angle - min_angle<20){
		return MinN;
	}
	if(max_angle - mid2_angle<10){
		return MinN;
	}
	if(mid2_angle - min_angle<10){
		return MinN;
	}
	if(mid4_angle - min_angle<10){
		return MinN;
	}
	if(max_angle - mid4_angle<10){
		return MinN;
	}
	k12 = MaxN/(max_angle-mid2_angle);
	k23 = MaxN/(mid2_angle-min_angle);
	k34 = MaxN/(mid4_angle-min_angle);
	k41 = MaxN/(max_angle-mid4_angle);
	
	if(flag==12){
			N=k12*(max_angle-angle);
	}
	else
	if(flag==23){
			N=MaxN - k23*(mid2_angle-angle);
	}
	else
	if(flag==34){
			N=k34*(angle-min_angle);
	}
	else
	if(flag==41){
			N=MaxN - k41*(angle-mid4_angle);
	}
	if(N>MaxN) N = MaxN;
	if(N<MinN) N = MinN;
	return N;
}

/*
函数名：generatrForce2
参 	数：int flag, 		状态机状态
				float angle,	角度 
				float v				角速度
返回值：(float)				目标力
*/
float generatrForce2(int flag, float angle, float v){
	//预定义大概的角度范围，后面采用实时测量值，仅第一步有用
	static float max_angle=110,min_angle=60,mid2_angle=85,mid4_angle=85;		
	float k12,k23,k34,k41,N,cal_max_N;	
	static u8 cout=0;

	if(flag==1){															//记录各个位置角度
		max_angle = angle;
		N = MinN;
	}else if(flag==3){
		min_angle = angle;
		N = MinN;
	}else if(flag==2){
		mid2_angle = angle;
//		if(v>-50&&v<50) {												//判断是否原地停下
//			cout = 11;
//			return MinN;
//		}
		N=cal_max_N;//N = MaxN;
	}else if(flag==4){
		mid4_angle = angle;	
//		if(v>-50&&v<50) {												//判断是否原地停下
//			cout = 11;
//			return MinN;
//		}
		N=cal_max_N;//N = MaxN;
	}
//	if(-10<v&&v<10){													//判断是否停下，可任意位置，20可能更好
//		cout++;
//		if(cout>200){
//			cout = 11;
//		}
//	}else{
//		cout=0;
//	}
//	if(cout>10){															//满足停下，可任意位置
//		return MinN;
//	}
//	
//	if(max_angle - min_angle<20){							//判断角度是否过小
//		return MinN;
//	}
//	if(max_angle - mid2_angle<10){						//判断角度是否过小
//		return MinN;
//	}
//	if(mid2_angle - min_angle<10){						//判断角度是否过小
//		return MinN;
//	}
//	if(mid4_angle - min_angle<10){						//判断角度是否过小
//		return MinN;
//	}
//	if(max_angle - mid4_angle<10){						//判断角度是否过小
//		return MinN;
//	}

//cal_max_N = (max_angle- min_angle)*5;

//	k12 = cal_max_N/(max_angle-mid2_angle);				//计算各段斜率
//	k23 = cal_max_N/(mid2_angle-min_angle);
//	k34 = cal_max_N/(mid4_angle-min_angle);
//	k41 = cal_max_N/(max_angle-mid4_angle);
	

//	k12 = MaxN/(max_angle-mid2_angle);				//计算各段斜率
//	k23 = MaxN/(mid2_angle-min_angle);
//	k34 = MaxN/(mid4_angle-min_angle);
//	k41 = MaxN/(max_angle-mid4_angle);

	k12 = 5;
	k23 = 5;
	k34 = 5;
	k41 = 5;

	if(flag==12){
			N=k12*(max_angle-angle);							//计算12段目标力
	}
	else
	if(flag==23){
			N=cal_max_N - k23*(mid2_angle-angle);			//计算23段目标力
	}
	else
	if(flag==34){
			N=k34*(angle-min_angle);							//计算34段目标力
	}
	else
	if(flag==41){
			N=cal_max_N - k41*(angle-mid4_angle);			//计算41段目标力
	}
	if(N>MaxN) N = MaxN;											//限制目标力输出
	if(N<MinN) N = MinN;
	return N;
}
