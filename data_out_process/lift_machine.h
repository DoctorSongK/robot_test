#ifndef LIFT_DRIVE_H_
#define LIFT_DRIVE_H_
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include "../common/struct_drive.h"
#include "../data_out_process/data_out.h"

class Lifter
{
public:
    Lifter();
    ~Lifter();
    
    void Lift_MotorRun(Data_out data);
    void Lift_MotorRise(void);
    void Lift_MotorFall(void);
    void Lift_TurnZero(void);////顶升托盘回转到0
    void Lift_ALLZero(void);////顶升托盘回转到0
    void Turn_MotorRun(float expect_angle, float omega);
    void Turn_MotorStop(void);
    void LT_Motor_getZeroPose(void);
    void LT_Motor_calcCurrentPose(void);
    void Lift_Motor_setExpectpose(float expect_H, float expect_A);
    void Lift_Motor_setExpectpose2(float expect_H, float expect_A);
    bool LiftMachine_TaskComplete(void);
    u8 LiftMachine_gotoZeroPos(void);
    MotorTypedef Motor_LIFT = { .NodeID = 8 };  /* 左行走 */
    MotorTypedef Motor_TURN = { .NodeID = 7 };

    LiftMachineTypedef LiftMachine_zeroPosture = { 0, 0, 0, 0 };
    LiftMachineTypedef LiftMachine_currentPosture = { 0, 0, 0, 0 };
    LiftMachineTypedef LiftMachine_expectPosture = { 0, 0, 0, 0 };

    bool lift_up_flag;
 
private:
    /**
 * @brief Motor Externed
 */
    float lift_target_h;
    float lift_height;

};

#endif /* MOTOR_H_ */
