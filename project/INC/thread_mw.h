#ifndef __THREAD_MW_H
#define __THREAD_MW_H
#include "stdio.h"
#include "stm32f4xx.h"
#include "sys.h"
#include "string.h"

extern u8 swt[10]; //ȫ��һЩ״̬���صĿ���


void com_F7(void);
void send_ANO(void);
void time_thread(void);
void swt_f(int num,int val);
void swt_help(void);
void swt_check(void);
float generatrForce(int flag, float angle, float v);
float generatrForce2(int flag, float angle, float v);

#endif
