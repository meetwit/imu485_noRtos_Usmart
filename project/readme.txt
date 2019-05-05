
	static  float ano_out[10] = {0},angle_change_history[3] = {0},angle_change_time[2] = {0},assi_time[2] = {0},GX_history[2][3]={0},P_flag = 0;
	
	static float temp1 = 208.98;
	static float temp2 = 16.384;
	static float temp3 = 182.04;

	if(stcIMU[index].addrReg==Roll){				//�Ƕ�
			stcAngle.Angle[0] = ((unsigned short)ucRxBuffer[3]<<8)|ucRxBuffer[4];
			stcAngle.Angle[1] = ((unsigned short)ucRxBuffer[5]<<8)|ucRxBuffer[6];
			stcAngle.Angle[2] = ((unsigned short)ucRxBuffer[7]<<8)|ucRxBuffer[8];
			stcIMU[index].x_Angle = stcAngle.Angle[0]/temp3;		//��λ����
			stcIMU[index].y_Angle = stcAngle.Angle[1]/temp3;
			stcIMU[index].z_Angle = stcAngle.Angle[2]/temp3;
			
			/*��ǰ�������ֿ�ʼ*/
//	if(stcIMU[index].x_Angle>110&&curId[index]){
//		//printf("%d",stcIMU[index].addrId);
//		while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
//		USART1->DR =  stcIMU[index].addrId;  
//		curId[index] = 0;
//		if(index==1) LED0=0;
//		if(index==2) LED1=0;
//	}else if(stcIMU[index].x_Angle<=110){
//		curId[index] = 1;
//		if(index==1) LED0=1;
//		if(index==2) LED1=1;
//	}
			/*��ǰ�������ֽ���*/
			
			/*ano_out[8] Ϊ������� �����Ʒ�ֵΪ����100*/
		if(stcIMU[4].x_Angle>stcIMU[5].x_Angle){
			if(stcIMU[4].x_w>20&&(stcIMU[4].x_Angle>100&&stcIMU[4].x_Angle<130)){
				ano_out[8] = 100;
					assi_time[0]++;
				assi_time[1] = 0;
			}else{
				ano_out[8] = 0;
					assi_time[1] ++;
				assi_time[0] = 0;
			}
			if(assi_time[0]==2){
				out_trigger('s');
				ano_out[9] = 150;
			}else if(assi_time[1]==2){
				out_trigger('e');
				ano_out[9] = 300;
			}
			
		}
//		else{
//			if(stcIMU[5].x_w>10&&(stcIMU[5].x_Angle>100&&stcIMU[5].x_Angle<130)){
//				printf("rigt start\r\n");
//			}
//		}	
			
//printf("%d,%d,%d,%d,%7.2f,%7.2f,%7.2f\r\n",Roll,ucRxBuffer[0],stcIMU[index].addrId,ucRxCnt,stcIMU[index].x_Angle,stcIMU[index].y_Angle,stcIMU[index].z_Angle);
//printf("%d,%7.2f\r\n",stcIMU[index].addrId,stcIMU[index].x_Angle);
		/*ano_out[0 1] ������ǽǶ�*/
ano_out[stcIMU[index].addrId%2] = stcIMU[index].x_Angle;
		/*ano_out[4] ����������ҽŽǶȵĲ�*/
ano_out[4] = ano_out[0]-ano_out[1];
ano_out[5] = -ano_out[4];

		/*ano_out[6] ��������� �ǶȵĲ�ı仯�� �� �����˲�*/
ano_out[6] = (ano_out[4]-angle_change_history[0])*5+(angle_change_history[0]-angle_change_history[1])*3+(angle_change_history[1]-angle_change_history[2])*2;
		/*�ǶȵĲ�ı仯�� ת��Ϊ ���� ��ֵΪ+-200*/
		if(ano_out[6]>10){
//			ano_out[6] = 200;
			angle_change_time[1]++;
		}else if(ano_out[6]<-10){
//			ano_out[6] = -200;
			angle_change_time[0]++;
			P_flag = 1;
		}else{
//			ano_out[6] = 0;
			angle_change_time[1] = 0;
			angle_change_time[0] = 0;
		}
		if(angle_change_time[1] >3){
			ano_out[6] = 200;
		}else if(angle_change_time[0] >3){
			ano_out[6] = -200;
		}else{
			ano_out[6] = 0;
		}
		
		if(angle_change_time[1]==3&&P_flag){
			out_trigger('m');
			ano_out[9] = 50;
			P_flag = 0;
		}

ano_out[7] = -ano_out[6];

angle_change_history[2] = angle_change_history[1];
angle_change_history[1] = angle_change_history[0];
angle_change_history[0] = ano_out[4];
out_ano(0xf2,(unsigned char *)ano_out,sizeof(float),sizeof(ano_out));
				
		}else if(stcIMU[index].addrReg==AX){		//���ٶ�
			stcAcc.a[0] = ((unsigned short)ucRxBuffer[3]<<8)|ucRxBuffer[4];
			stcAcc.a[1] = ((unsigned short)ucRxBuffer[5]<<8)|ucRxBuffer[6];
			stcAcc.a[2] = ((unsigned short)ucRxBuffer[7]<<8)|ucRxBuffer[8];
			stcIMU[index].x_a = stcAcc.a[0]/temp1;			//��λ��m/s^2
			stcIMU[index].y_a = stcAcc.a[1]/temp1;
			stcIMU[index].z_a = stcAcc.a[2]/temp1;
//printf("%d,%d,%d,%d,%7.2f,%7.2f,%7.2f\r\n",AX,ucRxBuffer[0],stcIMU[index].addrId,ucRxCnt,stcIMU[index].x_a,stcIMU[index].y_a,stcIMU[index].z_a);
		}else if(stcIMU[index].addrReg==GX){		//���ٶ�
			stcGyro.w[0] = ((unsigned short)ucRxBuffer[3]<<8)|ucRxBuffer[4];
			stcGyro.w[1] = ((unsigned short)ucRxBuffer[5]<<8)|ucRxBuffer[6];
			stcGyro.w[2] = ((unsigned short)ucRxBuffer[7]<<8)|ucRxBuffer[8];
			stcIMU[index].x_w = stcGyro.w[0]/temp2;		//��λ����/s
			stcIMU[index].y_w = stcGyro.w[1]/temp2;
			stcIMU[index].z_w = stcGyro.w[2]/temp2;
			/*ano_out[2 3] ������ǽ��ٶȡ������˲�*/
			ano_out[stcIMU[index].addrId%2+2] = (stcIMU[index].x_w)*0.5+GX_history[stcIMU[index].addrId%2][0]*0.3+GX_history[stcIMU[index].addrId%2][1]*0.2;
			GX_history[stcIMU[index].addrId%2][2] = GX_history[stcIMU[index].addrId%2][1];
			GX_history[stcIMU[index].addrId%2][1] = GX_history[stcIMU[index].addrId%2][0];
			GX_history[stcIMU[index].addrId%2][0] = stcIMU[index].x_w;
//printf("%d,%d,%d,%d,%7.2f,%7.2f,%7.2f\r\n",GX,ucRxBuffer[0],stcIMU[index].addrId,ucRxCnt,stcIMU[index].x_w,stcIMU[index].y_w,stcIMU[index].z_w);
		}
