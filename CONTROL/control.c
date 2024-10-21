#include "control.h"

#define Speed_Y 10
#define Speed_Z 60

int Vertical(float Med,float Angle,float gyro_Y);
int Velocity(int Target,int Encoder_left,int Encoder_right);
int Turn(int gyro_Z,int YK);

float Med_Angle=0;//机械中值
int Vertical_out,Velocity_out,Turn_out;
float Vertical_Kp=-280,       //-400    
			Vertical_Kd=-1.2;			//-1.5  
float Velocity_Kp=1.6,				//+1.6	
			Velocity_Ki=0.008;			//+0.006
float Turn_Kp=30,							//+30	
			Turn_Kd=0.8;						  //+0.6
float Target_Speed=0;
float Turn_Speed=0;

void EXTI9_5_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line5)!=0)
	{
		int PWM_out;
		if(PBin(5)==0)
		{
			EXTI_ClearITPendingBit(EXTI_Line5);
			Encoder_Left=Read_speed(3);
			Encoder_Right=Read_speed(4);
			mpu_dmp_get_data(&Pitch,&Roll,&Yaw);
			MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);
			MPU_Get_Accelerometer(&aacx,&aacy,&aacz);
			SR04_Start();
			if(avoid==1)
			{
				Target_Speed=5;
				if(distance<30)
				{
					Target_Speed=0;
					Turn_Speed=60;
				}
				else
				{
					Target_Speed=5;
					Turn_Speed=0;
				}
			}
			else
			{
				if(fore==0&&back==0)Target_Speed=0;
				if(fore==1)Target_Speed++;
				if(back==1)Target_Speed--;
				Target_Speed=Target_Speed>Speed_Y?Speed_Y:(Target_Speed<-Speed_Y?(-Speed_Y):Target_Speed);
				if(left==0&&right==0)Turn_Speed=0;
				if(left==1)Turn_Speed-=20;
				if(right==1)Turn_Speed+=20;
				Turn_Speed=Turn_Speed>Speed_Z?Speed_Z:(Turn_Speed<-Speed_Z?(-Speed_Z):Turn_Speed);
				if(left==0&&right==0)Turn_Kd=0.6;
				else if(left==1||right==1)Turn_Kd=0;
			}
			Vertical_out=Vertical(Med_Angle,Pitch,gyroy);
			Velocity_out=Velocity(Target_Speed,Encoder_Left,Encoder_Right);
			Turn_out=Turn(gyroz,Turn_Speed);
			PWM_out=Vertical_out-Vertical_Kp*Velocity_out;
			MOTO1=PWM_out-Turn_out;
			MOTO2=PWM_out+Turn_out;
			Limit(&MOTO1,&MOTO2);
			Load(MOTO1,MOTO2);
			Stop(&Med_Angle,&Pitch);
		}
	}
}
/*
直立环PD
入口：期望角度，真实角度，真实角速度
出口：直立环输出
*/

int Vertical(float Med,float Angle,float gyro_Y)
{
	int PWM_out;
	PWM_out=Vertical_Kp*(Angle-Med)+Vertical_Kd*(gyro_Y-0);
	return PWM_out;
}


/*
速度环PI
*/

int Velocity(int Target,int Encoder_left,int Encoder_right)
{
	int PWM_out,Encoder_Err,EnC_Err_Lowout;
	static int Encoder_S,EnC_Err_Lowout_last;
	float a=0.7;
	
	//1.计算速度偏差
	Encoder_Err=((Encoder_left+Encoder_right)-Target);
	//2.对速度偏差进行低通滤波
	EnC_Err_Lowout=(1-a)*Encoder_Err+a*EnC_Err_Lowout_last;//是波形更平滑，滤除高频干扰，防止速度突变影响直立环工作
	EnC_Err_Lowout_last=EnC_Err_Lowout;
	//3.对速度偏差进行积分，积分出位移
	Encoder_S+=EnC_Err_Lowout;
	//4.积分限幅
	Encoder_S=Encoder_S>10000?10000:(Encoder_S<(-10000)?(-10000):Encoder_S);
	if(stop==1)Encoder_S=0,stop=0;
	//5.速度环控制输出计算
	PWM_out=Velocity_Kp*EnC_Err_Lowout+Velocity_Ki*Encoder_S;
	
	return PWM_out;
}

/*
转向环：系数*Z轴角速度
*/

int Turn(int gyro_Z,int YK)
{
	int PWM_out;
	
	PWM_out=Turn_Kd*gyro_Z+Turn_Kp*YK;
	return PWM_out;
}




