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
8				触发点A
9				出发点
*/
/*							 0 1 2 3 4 5 6 7 8 9*/
uint8_t swt[10]={1,1,1,1,1,1,1,1,1,0}; //全局一些状态开关的控制


void com_F7(void){
	generatrForce(state_F,stcIMU[3].x_Angle,stcIMU[3].x_w);
	ano_o[9] = state_F;
	if(state_F==1){
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
	ano_o[8] = generatrForce2(state_S,stcIMU[4].x_Angle,stcIMU[4].x_w);
//	ano_o[8] = state_S;
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

void swt_check(){
		rt_kprintf("\r\nr\n状态查询：\r\n");
		if(swt[0]){
					rt_kprintf("全局时钟打印已开启\r\n");
				}else{
					rt_kprintf("全局时钟打印已关闭\r\n");
				}
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
					rt_kprintf("已开启F4&F7通信\r\n");
				}else{
					rt_kprintf("已关闭F4&F7通信\r\n");
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
	float MaxN=300;
	float c1,k1,c2,k2,N,count;
    //最大值
    if(flag==1 || flag==3){
        c1=-angle;
        k1=MaxN/c1;
        c2=angle/3;
        k2=MaxN/c2;
    }
    if(flag==12 || flag==34){
        N=k1*(angle+c1);
    }
    //过零点
    if(flag==2){

    }
    if(flag==23 || flag==41){
        N=k2*(angle+c2);
    }
    //最小值
    if(flag==3){

    }
    if(N<0)
        N=0;
    //停止点
    if(v>-20 && v<20)
        count++;
    else
        count=0;
    if(count==5)
         N=0;
		return N;
}

float generatrForce2(int flag, float angle, float v){
	static float max_angle=130,min_angle=70,mid_angle=100;
	float MaxN=300;
	float k1,k2,N;
    //最大值
	if(flag==1){
		max_angle = angle;
		N = 0;
	}else if(flag==3){
		min_angle = angle;
		N = 0;
	}else if(flag==2){
		mid_angle = angle;
		N = MaxN;
	}else if(flag==4){
		mid_angle = angle;
		N = MaxN;
	}
	k1 = MaxN/(max_angle-mid_angle);
	k2 = MaxN/(mid_angle-min_angle);
	
	if(flag==12||flag==41){
			N=k1*(max_angle-angle);
	}
	else
	if(flag==23||flag==34){
			N=k2*(angle-min_angle);
	}
	if(N>MaxN) N = MaxN;
	if(N<0) N = 0;
	return N;
}
