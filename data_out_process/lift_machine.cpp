/* 步科驱动器CanOpen协议 */

#include "lift_machine.h"
#include <iostream>
#include <math.h>
#include "../common/configuration.h"
/**
 * @brief 电机使能
 *
 * @return
 */

Lifter::Lifter()
{
	//ret = LiftMachine_gotoZeroPos();
	lift_height = get_configs()->get_float("lift", "max_height", nullptr);
	if(lift_height<0) lift_height = 0.45;
    lift_height *= 100;
	
	lift_target_h = get_configs()->get_float("lift", "height", nullptr);
	if(lift_target_h<0) lift_target_h = 0.25;
    lift_target_h *= 100;
}

Lifter::~Lifter()
{

}

/*顶升电机抬升动作*/
void Lifter::Lift_MotorRise(void)//加参数
{
	//Lift_Motor_setExpectpose(lift_target_h, LiftMachine_expectPosture.Angle);
	Lift_Motor_setExpectpose(lift_target_h, 0.0);
	Motor_LIFT.SpeedRatio = 1000 * 17896; /* r/min */
	Motor_TURN.SpeedRatio = 1000 * 17896;

}

/*顶升电机下降动作*/
void Lifter::Lift_MotorFall(void)//加参数
{
	Lift_Motor_setExpectpose(0, LiftMachine_expectPosture.Angle);
	Motor_LIFT.SpeedRatio = 1000 * 17896; /* r/min */
	Motor_TURN.SpeedRatio = 1000 * 17896;
}
void Lifter::Lift_TurnZero(void)////顶升托盘回转到0
{
    Lift_Motor_setExpectpose(LiftMachine_expectPosture.Height,0.0 );
    Motor_LIFT.SpeedRatio = 1000 * 17896; /* r/min */
    Motor_TURN.SpeedRatio = 1000 * 17896;
}
void Lifter::Lift_ALLZero(void)////顶升托盘回转到0
{
    Lift_Motor_setExpectpose(0.0,0.0 );
    Motor_LIFT.SpeedRatio = 1000 * 17896; /* r/min */
    Motor_TURN.SpeedRatio = 1000 * 17896;
}
/*旋转电机动作*/
void Lifter::Turn_MotorRun(float expect_angle, float omega)//加参数
{
	float speed_ratio = 0;
	omega = fabs(omega);
    if(omega<0.1) omega=0.1;
	Lift_Motor_setExpectpose2(LiftMachine_expectPosture.Height, expect_angle);

	speed_ratio = omega / (2 * M_PI) * 60 * 2.62 * 20 * 17896;
	Motor_LIFT.SpeedRatio = speed_ratio;
	Motor_TURN.SpeedRatio = speed_ratio;

}

/*旋转、顶升电机停止*/
void Lifter::Turn_MotorStop(void)//加参数
{
	Motor_LIFT.SpeedRatio = 0;
	Motor_TURN.SpeedRatio = 0;

}
/*获取旋转、顶升电机绝对零点位置编码器值*/
void Lifter::LT_Motor_getZeroPose(void)
{
	LiftMachine_zeroPosture.ActualPosition_L = 0;/////////////////////////这里还没改
	LiftMachine_zeroPosture.ActualPosition_T = 0;
	LiftMachine_zeroPosture.Height = 0;
	LiftMachine_zeroPosture.Angle = 0;
	
}
/*计算旋转、顶升电机实时姿态*/
void Lifter::LT_Motor_calcCurrentPose(void)
{
	float n;
	float L_AP, T_AP, H, A;

	//LiftMachine_currentPosture.ActualPosition_L = Motor_LIFT.ActualPosition;
	//LiftMachine_currentPosture.ActualPosition_T = Motor_TURN.ActualPosition;
    LiftMachine_currentPosture.ActualPosition_L = Motor_TURN.ActualPosition;
	LiftMachine_currentPosture.ActualPosition_T = Motor_LIFT.ActualPosition;
		
	//printf("555555555555555%d,%d\n",Motor_TURN.ActualPosition,Motor_LIFT.ActualPosition);
	/* 相对零点 编码器值 */
	L_AP = LiftMachine_currentPosture.ActualPosition_L - LiftMachine_zeroPosture.ActualPosition_L;
	T_AP = LiftMachine_currentPosture.ActualPosition_T - LiftMachine_zeroPosture.ActualPosition_T;

	/* 计算 转换系数  * 编码器分辨率 * 一级减速比 * 二级减速比   (转换后单位 r , 10mm螺距 , 1转360°) */
	n = 65536 * 20 * 2.62;
	H = (L_AP / n + T_AP / n) * (-10.0);
	A = (T_AP / n) * 360.0;

	/* 当前实时姿态 mm  ° */
	LiftMachine_currentPosture.Height = H;
	LiftMachine_currentPosture.Angle = A;
}

/*设置旋转期望姿态对应编码器值（相对于此时刻的这个相对零点）*/
void Lifter::Lift_Motor_setExpectpose(float expect_H, float expect_A)
{
	float n;
	float L_AP, T_AP, H, A;
	u16 angle_range = 360;

	/* 限制小车顶升高度 极限位置 40mm */
	if (expect_H < 0)
		expect_H = 0;
	else if(expect_H > lift_height)
		expect_H = lift_height;

    //expect_A = expect_A/(M_PI)*180;
	/* 限制转角范围  */
	while (expect_A > angle_range)
		expect_A -= 360;
	while (expect_A < -angle_range)
		expect_A += 360;

	LiftMachine_expectPosture.Height = expect_H;
	LiftMachine_expectPosture.Angle = expect_A;
	H = expect_H;
	A = expect_A;

	/* 计算 转换系数   编码器分辨率 * 一级减速比 * 二级减速比   (转换后单位 r , 10mm导程 , 1转360°) */
	n = 65536 * 20 * 2.62;
	L_AP = (H / (-10.0) - (A / 360.0)) * n;
	T_AP = (A / 360.0) * n;

	/* 加上 零点位置值 */
	L_AP += LiftMachine_zeroPosture.ActualPosition_L;
	T_AP += LiftMachine_zeroPosture.ActualPosition_T;

	/* 根据期望姿态 计算 期望下发数字量 */
	LiftMachine_expectPosture.ActualPosition_L = L_AP;
	LiftMachine_expectPosture.ActualPosition_T = T_AP;

	/* 驱动器下发目标位置值 */
	{
		Motor_LIFT.ExpectPositionRatio = L_AP;
		Motor_TURN.ExpectPositionRatio = T_AP;
		//Motor_LIFT.ExpectPositionRatio = T_AP;
		//Motor_TURN.ExpectPositionRatio = L_AP;
	}
	
}

/*顶升机构任务完成状态
* 任务完成返回1；未完成返回0
*/
bool Lifter::LiftMachine_TaskComplete(void) {
	bool result = false;
	LT_Motor_calcCurrentPose();
	if (fabs(LiftMachine_currentPosture.Angle - LiftMachine_expectPosture.Angle) < 1
            && fabs(LiftMachine_currentPosture.Height - LiftMachine_expectPosture.Height) < 1)
		result = true;

	if (fabs(LiftMachine_currentPosture.Height - lift_target_h) < 2) 
	lift_up_flag = true;
	else
    lift_up_flag = false;
	//if (fabs(LiftMachine_currentPosture.Height - LiftMachine_expectPosture.Height) < 2)
	//	result = true;
	//printf("curAngle%f,expang%f,curh=%f,exph=%f\n",LiftMachine_currentPosture.Angle,LiftMachine_expectPosture.Angle,LiftMachine_currentPosture.Height ,LiftMachine_expectPosture.Height);
	
	//printf("111111111111111111111111111111%f,%f\n",LiftMachine_currentPosture.Angle - LiftMachine_expectPosture.Angle,LiftMachine_currentPosture.Height - LiftMachine_expectPosture.Height);

	return result;
}

/*初始化顶升、旋转电机归置零位*/
u8 Lifter::LiftMachine_gotoZeroPos(void) {

	Lift_Motor_setExpectpose(0, 0);

	Motor_LIFT.SpeedRatio = 100 * 17896;      //r/min 500
	Motor_TURN.SpeedRatio = 100 * 17896;
	return 0;
}
void Lifter::Lift_MotorRun(Data_out logic_data)
{

    if(logic_data.lift_info.lift_state == LIFT_UP)
    Lift_MotorRise();
    else if(logic_data.lift_info.lift_state == LIFT_DOWN)
    Lift_MotorFall();
    else if(logic_data.lift_info.lift_state == LIFT_TURN)
    Turn_MotorRun(logic_data.lift_info.lift_angle,logic_data.lift_info.angle_speed);	
    else if(logic_data.lift_info.lift_state == LIFT_TURN_ZERO)
    Lift_TurnZero();
    else if(logic_data.lift_info.lift_state == LIFT_ALL_ZERO)
    Lift_ALLZero();
}

/*设置旋转期望姿态对应编码器值（相对于此时刻的这个相对零点）*/
void Lifter::Lift_Motor_setExpectpose2(float expect_H, float expect_A)
{
    float n;
    float L_AP, T_AP, H, A;
    u16 angle_range = 360;

    /* 限制小车顶升高度 极限位置 40mm */
    if (expect_H < 0)
        expect_H = 0;
    else if(expect_H > lift_height)
        expect_H = lift_height;

    expect_A = expect_A/(M_PI)*180;
    /* 限制转角范围  */
    while (expect_A > angle_range)
        expect_A -= 360;
    while (expect_A < -angle_range)
        expect_A += 360;

    LiftMachine_expectPosture.Height = expect_H;
    LiftMachine_expectPosture.Angle += expect_A;
    H = expect_H;
    A = LiftMachine_expectPosture.Angle;

	/* 计算 转换系数   编码器分辨率 * 一级减速比 * 二级减速比   (转换后单位 r , 10mm导程 , 1转360°) */
	n = 65536 * 20 * 2.62;
	L_AP = (H / (-10.0) - (A / 360.0)) * n;
	T_AP = (A / 360.0) * n;

	/* 加上 零点位置值 */
	L_AP += LiftMachine_zeroPosture.ActualPosition_L;
	T_AP += LiftMachine_zeroPosture.ActualPosition_T;

	/* 根据期望姿态 计算 期望下发数字量 */
	LiftMachine_expectPosture.ActualPosition_L = L_AP;
	LiftMachine_expectPosture.ActualPosition_T = T_AP;

	/* 驱动器下发目标位置值 */
	{
		Motor_LIFT.ExpectPositionRatio = L_AP;
		Motor_TURN.ExpectPositionRatio = T_AP;
		//Motor_LIFT.ExpectPositionRatio = T_AP;
		//Motor_TURN.ExpectPositionRatio = L_AP;
	}
	
}