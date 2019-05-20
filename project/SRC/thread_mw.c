#include "main.h"

#define MaxN 200
#define MinN 10


/*
algorithm 0 first version
algorithm 1 ����Ƕȵ�Ŀ�������

*/
#define algorithm 0		

/*		      		 0 1 2 3 4 5 6 7 8 9*/
uint8_t swt[10]={1,1,1,1,1,1,0,1,1,0}; //ȫ��һЩ״̬���صĿ���
float target_N[4]={10,10,10,10};			//0:˫����ǰ 1��������ǰ 2:˫���ź� 3�������ź� 

void send_F7(void){
//��Ŀ���� target_N[4]�������� hx711_N[4] ͨ������5������F7	float
//����5���ֽڷ��ͺ���	PcTx_Byte5(u8 data)		unsigned char
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
	if(state_F==1){											//״̬�л�
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
	
	if(state_F==12||state_F==23){					//����л�
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
	
	ano_o_F3[0] = target_N[0];						//�����������F3 0123
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
//					rt_kprintf("ȫ��ʱ�Ӵ�ӡ����\r\n");
//				}else{
//					rt_kprintf("ȫ��ʱ�Ӵ�ӡ�ر�\r\n");
//				}
//			break;
				
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
					rt_kprintf("F4�����������ݵ�F7\r\n");
				}else{
					rt_kprintf("F4�����������ݵ�F7\r\n");
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
		rt_kprintf("swt_f 6 0 F4�����������ݵ�F7\r\n");
		rt_kprintf("swt_f 6 1 F4�����������ݵ�F7\r\n");
		rt_kprintf("swt_f 7 0 imu�Ƕȹرղ���\r\n");
		rt_kprintf("swt_f 7 1 imu�Ƕȿ�������\r\n");
		rt_kprintf("swt_f 8 0 imu���ٶȹرղ���\r\n");
		rt_kprintf("swt_f 8 1 imu���ٶȿ�������\r\n");
		rt_kprintf("swt_f 9 0 imu���ٶȹرղ���\r\n");
		rt_kprintf("swt_f 9 1 imu���ٶȿ�������\r\n");
}

void swt_check(){
		rt_kprintf("\r\n\n״̬��ѯ��\r\n");
//		if(swt[0]){
//					rt_kprintf("ȫ��ʱ�Ӵ�ӡ�ѿ���\r\n");
//				}else{
//					rt_kprintf("ȫ��ʱ�Ӵ�ӡ�ѹر�\r\n");
//				}
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
					rt_kprintf("F4�����������ݵ�F7\r\n");
				}else{
					rt_kprintf("F4�����������ݵ�F7\r\n");
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
��������generatrForce2
�� 	����int flag, 		״̬��״̬
				float angle,	�Ƕ� 
				float v				���ٶ�
����ֵ��(float)				Ŀ����
*/
float generatrForce2(int flag, float angle, float v){
	//Ԥ�����ŵĽǶȷ�Χ���������ʵʱ����ֵ������һ������
	static float max_angle=110,min_angle=60,mid2_angle=85,mid4_angle=85;		
	float k12,k23,k34,k41,N,cal_max_N;	
	static u8 cout=0;

	if(flag==1){															//��¼����λ�ýǶ�
		max_angle = angle;
		N = MinN;
	}else if(flag==3){
		min_angle = angle;
		N = MinN;
	}else if(flag==2){
		mid2_angle = angle;
//		if(v>-50&&v<50) {												//�ж��Ƿ�ԭ��ͣ��
//			cout = 11;
//			return MinN;
//		}
		N=cal_max_N;//N = MaxN;
	}else if(flag==4){
		mid4_angle = angle;	
//		if(v>-50&&v<50) {												//�ж��Ƿ�ԭ��ͣ��
//			cout = 11;
//			return MinN;
//		}
		N=cal_max_N;//N = MaxN;
	}
//	if(-10<v&&v<10){													//�ж��Ƿ�ͣ�£�������λ�ã�20���ܸ���
//		cout++;
//		if(cout>200){
//			cout = 11;
//		}
//	}else{
//		cout=0;
//	}
//	if(cout>10){															//����ͣ�£�������λ��
//		return MinN;
//	}
//	
//	if(max_angle - min_angle<20){							//�жϽǶ��Ƿ��С
//		return MinN;
//	}
//	if(max_angle - mid2_angle<10){						//�жϽǶ��Ƿ��С
//		return MinN;
//	}
//	if(mid2_angle - min_angle<10){						//�жϽǶ��Ƿ��С
//		return MinN;
//	}
//	if(mid4_angle - min_angle<10){						//�жϽǶ��Ƿ��С
//		return MinN;
//	}
//	if(max_angle - mid4_angle<10){						//�жϽǶ��Ƿ��С
//		return MinN;
//	}

//cal_max_N = (max_angle- min_angle)*5;

//	k12 = cal_max_N/(max_angle-mid2_angle);				//�������б��
//	k23 = cal_max_N/(mid2_angle-min_angle);
//	k34 = cal_max_N/(mid4_angle-min_angle);
//	k41 = cal_max_N/(max_angle-mid4_angle);
	

//	k12 = MaxN/(max_angle-mid2_angle);				//�������б��
//	k23 = MaxN/(mid2_angle-min_angle);
//	k34 = MaxN/(mid4_angle-min_angle);
//	k41 = MaxN/(max_angle-mid4_angle);

	k12 = 5;
	k23 = 5;
	k34 = 5;
	k41 = 5;

	if(flag==12){
			N=k12*(max_angle-angle);							//����12��Ŀ����
	}
	else
	if(flag==23){
			N=cal_max_N - k23*(mid2_angle-angle);			//����23��Ŀ����
	}
	else
	if(flag==34){
			N=k34*(angle-min_angle);							//����34��Ŀ����
	}
	else
	if(flag==41){
			N=cal_max_N - k41*(angle-mid4_angle);			//����41��Ŀ����
	}
	if(N>MaxN) N = MaxN;											//����Ŀ�������
	if(N<MinN) N = MinN;
	return N;
}
