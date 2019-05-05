#include "main.h"

/*
swt index  
0				���ش���1��ʱ�Ӵ�ӡ�� 			����0���رգ�����1������
1				����������1�Ķ�ȡ���ƣ� 		����0���رգ�����1������
2				����������2�Ķ�ȡ���ƣ� 		����0���رգ�����1������
3				����������3�Ķ�ȡ���ƣ� 		����0���رգ�����1������
4				����������4�Ķ�ȡ���ƣ� 		����0���رգ�����1������
5				����4������λ����ʾ���ƣ� 	����0���رգ�����1������
6				����5��F7��ͨ�ſ��ƣ� 			����0���رգ�����1������
7				imu�ǶȲ������ƣ�						����0���رգ�����1������
8				imu���ٶȲ������ƣ�					����0���رգ�����1������
9				imu���ٶȲ������ƣ�					����0���رգ�����1������
*/
/*
ANO index  
0				����1
1				����2
2				����3
3				����4
4				ADDRΪ˫���� �Ƕ�
5				ADDRΪ������ �Ƕ�
6				ADDRΪ˫���� ���ٶ�
7				ADDRΪ������ ���ٶ�
8				������A
9				������
*/
/*							 0 1 2 3 4 5 6 7 8 9*/
uint8_t swt[10]={1,1,1,1,1,1,1,1,1,0}; //ȫ��һЩ״̬���صĿ���


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
					rt_kprintf("ȫ��ʱ�Ӵ�ӡ����\r\n");
				}else{
					rt_kprintf("ȫ��ʱ�Ӵ�ӡ�ر�\r\n");
				}
			break;
				
			case 1 :
				if(val){
					rt_kprintf("����������com1��������\r\n");
				}else{
					rt_kprintf("����������com1�رղ���\r\n");
				}
			break;
				
			case 2 :
				if(val){
					rt_kprintf("����������com2��������\r\n");
				}else{
					rt_kprintf("����������com2�رղ���\r\n");
				}
			break;
				
			case 3 :
				if(val){
					rt_kprintf("����������com3��������\r\n");
				}else{
					rt_kprintf("����������com3�رղ���\r\n");
				}
			break;
				
			case 4 :
				if(val){
					rt_kprintf("����������com4��������\r\n");
				}else{
					rt_kprintf("����������com4�رղ���\r\n");
				}
			break;
				
			case 5 :
				if(val){
					rt_kprintf("������λ��������ʾ\r\n");
				}else{
					rt_kprintf("������λ���ر���ʾ\r\n");
				}
			break;
				
			case 6 :
				if(val){
					rt_kprintf("����F4&F7ͨ��\r\n");
				}else{
					rt_kprintf("�ر�F4&F7ͨ��\r\n");
				}
			break;
				
			case 7 :
				if(val){
					rt_kprintf("imu�Ƕȿ�������\r\n");
				}else{
					rt_kprintf("imu�Ƕȹرղ���\r\n");
				}
			break;
				
			case 8 :
				if(val){
					rt_kprintf("imu���ٶȿ�������\r\n");
				}else{
					rt_kprintf("imu���ٶȹرղ���\r\n");
				}
			break;
				
			case 9 :
				if(val){
					rt_kprintf("imu���ٶȿ�������\r\n");
				}else{
					rt_kprintf("imu���ٶȹرղ���\r\n");
				}
			break;
		}
	
}


void swt_help(){
//		rt_kprintf("������ĸ�ʽ���ԣ��ο����£�\r\n");
//		rt_kprintf("swt_f 0 0 ȫ��ʱ�Ӵ�ӡ�ر�\r\n");
//		rt_kprintf("swt_f 0 1 ȫ��ʱ�Ӵ�ӡ����\r\n");
		rt_kprintf("\r\nswt_f 1 0 ����������com1�رղ���\r\n");
		rt_kprintf("swt_f 1 1 ����������com1��������\r\n");
		rt_kprintf("swt_f 2 0 ����������com2�رղ���\r\n");
		rt_kprintf("swt_f 2 1 ����������com2��������\r\n");
		rt_kprintf("swt_f 3 0 ����������com3�رղ���\r\n");
		rt_kprintf("swt_f 3 1 ����������com3��������\r\n");
		rt_kprintf("swt_f 4 0 ����������com4�رղ���\r\n");
		rt_kprintf("swt_f 4 1 ����������com4��������\r\n");
		rt_kprintf("swt_f 5 0 ������λ���ر���ʾ\r\n");
		rt_kprintf("swt_f 5 1 ������λ��������ʾ\r\n");
		rt_kprintf("swt_f 6 0 �ر�F4&F7ͨ��\r\n");
		rt_kprintf("swt_f 6 1 ����F4&F7ͨ��\r\n");
		rt_kprintf("swt_f 7 0 imu�Ƕȹرղ���\r\n");
		rt_kprintf("swt_f 7 1 imu�Ƕȿ�������\r\n");
		rt_kprintf("swt_f 8 0 imu���ٶȹرղ���\r\n");
		rt_kprintf("swt_f 8 1 imu���ٶȿ�������\r\n");
		rt_kprintf("swt_f 9 0 imu���ٶȹرղ���\r\n");
		rt_kprintf("swt_f 9 1 imu���ٶȿ�������\r\n");
}

void swt_check(){
		rt_kprintf("\r\nr\n״̬��ѯ��\r\n");
		if(swt[0]){
					rt_kprintf("ȫ��ʱ�Ӵ�ӡ�ѿ���\r\n");
				}else{
					rt_kprintf("ȫ��ʱ�Ӵ�ӡ�ѹر�\r\n");
				}
		if(swt[1]){
					rt_kprintf("����������com1�ѿ�������\r\n");
				}else{
					rt_kprintf("����������com1�ѹرղ���\r\n");
				}
		if(swt[2]){
					rt_kprintf("����������com2�ѿ�������\r\n");
				}else{
					rt_kprintf("����������com2�ѹرղ���\r\n");
				}
		if(swt[3]){
					rt_kprintf("����������com3�ѿ�������\r\n");
				}else{
					rt_kprintf("����������com3�ѹرղ���\r\n");
				}
		if(swt[4]){
					rt_kprintf("����������com4�ѿ�������\r\n");
				}else{
					rt_kprintf("����������com4�ѹرղ���\r\n");
				}
		if(swt[5]){
					rt_kprintf("������λ���ѿ�����ʾ\r\n");
				}else{
					rt_kprintf("������λ���ѹر���ʾ\r\n");
				}
		if(swt[6]){
					rt_kprintf("�ѿ���F4&F7ͨ��\r\n");
				}else{
					rt_kprintf("�ѹر�F4&F7ͨ��\r\n");
				}
		if(swt[7]){
					rt_kprintf("imu�Ƕ��ѿ�������\r\n");
				}else{
					rt_kprintf("imu�Ƕ��ѹرղ���\r\n");
				}
		if(swt[8]){
					rt_kprintf("imu���ٶ��ѿ�������\r\n");
				}else{
					rt_kprintf("imu���ٶ��ѹرղ���\r\n");
				}
		if(swt[9]){
					rt_kprintf("imu���ٶ��ѿ�������\r\n");
				}else{
					rt_kprintf("imu���ٶ��ѹرղ���\r\n");
				}
}


float generatrForce(int flag, float angle, float v){
	float MaxN=300;
	float c1,k1,c2,k2,N,count;
    //���ֵ
    if(flag==1 || flag==3){
        c1=-angle;
        k1=MaxN/c1;
        c2=angle/3;
        k2=MaxN/c2;
    }
    if(flag==12 || flag==34){
        N=k1*(angle+c1);
    }
    //�����
    if(flag==2){

    }
    if(flag==23 || flag==41){
        N=k2*(angle+c2);
    }
    //��Сֵ
    if(flag==3){

    }
    if(N<0)
        N=0;
    //ֹͣ��
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
    //���ֵ
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
