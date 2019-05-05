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
8				������
9				
*/
/*							 0 1 2 3 4 5 6 7 8 9*/
uint8_t swt[10]={1,1,1,1,1,1,1,1,1,0}; //ȫ��һЩ״̬���صĿ���


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
