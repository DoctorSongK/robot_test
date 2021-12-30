#ifndef MESSAGE_CHANNEL_INDEX_H
#define MESSAGE_CHANNEL_INDEX_H

#define CHANNEL_IGNORE  0   //不处理
#define CHANNEL_IMU     1   //IMU数据
#define CHANNEL_RADAR   2   //激光雷达数据
#define CHANNEL_WHEEL   3   //轮子编码器数据
#define CHANNEL_MOVE    4   //移动指令
#define CHANNEL_POSE    5   //实时位姿数据
#define CHANNEL_ODOM    6   //实时位姿变化量
#define CHANNEL_CONTROL 7   //外部控制指令	//暂未使用
#define CHANNEL_ACTION  8   //额外动作指令	//暂未使用
#define CHANNEL_FINISH  9   //任务完成信息	//暂未使用
#define CHANNEL_TIMER   10  //定时器		//暂未使用
#define CHANNEL_ARRIVE  11  //int,1:Turn,2:Arrive
#define CHANNEL_STATE  	12	
#define CHANNEL_STOP  	13
#define CHANNEL_MOTOR_CTL    14   //伺服控制指令
#define CHANNEL_TWO_DIMENTION    15   //two dimention recognize cmd
#define CHANNEL_CTL_BOARD    16   //控制板 cmd
#define CHANNEL_WARN_ERR    17   //报警错误输出通道
#define CHANNEL_LIFT    18   //顶升动作信息通道
#define CHANNEL_LOGIC_OUT    19   //逻辑层输出通道
#define CHANNEL_LASER_OBS_OUT    20   //激光避障状态输出
#define CHANNEL_ROBOT_STS_OUT    21   //激光避障输出
#define CHANNEL_ODOM_IMU     22   //里程计IMU数据
#define CHANNEL_ERROR   233 //错误信息输出
#define CHANNEL_3D_TO_MAP_RAW 24 //3D雷达建图原始数据输出
#define CHANNEL_3D_TO_OB_RAW 25 //3D雷达避障原始数据输出
#define CHANNEL_OBAVOID 26 //实时避障数据输出
#define CHANNEL_3D_LASER_OBS_OUT 27 //3D激光避障状态输出


#endif
