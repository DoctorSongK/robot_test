/* 步科驱动器CanOpen协议 */

#include "motor.h"
#include <iostream>
#include <math.h>
//#include "agv.h"
#include "../common/configuration.h"
#include "../common/ctl_data.h"
/**
 * @brief 电机使能
 *
 * @return
 */

const int cmd[2] = {0xc103,0xc104};//0xc103,0xc104分别为通道顶升电机上升或下降到达的信号值
Motor::Motor(MsgQueue *queue, int motor_chan, int wheel_chan, int move_chan)
{
	//ret = LiftMachine_gotoZeroPos();
	Model_type = get_configs()->get_int("odom", "robot_control_model", nullptr);

    voice_volum = get_configs()->get_int("io_board", "voice_volume", nullptr);
    if(voice_volum<0)voice_volum = 10;
    io_board_can_enable = get_configs()->get_int("io_board", "isolate_can", nullptr);

    if(io_board_can_enable<0) io_board_can_enable = 0;
	if(Model_type<ROBOT_DIFF_MODEL_CARRY) Model_type = ROBOT_DIFF_MODEL_CARRY;////默认采用差分模型
	motor_c = motor_chan;
	wheel_c = wheel_chan;
	move_c = move_chan;
	velocity = 0;
	angular = 0;
	this->queue = queue;
	queue->add_listener(motor_c, this);
	queue->add_listener(move_c, this);
	pthread_mutex_init(&recieve_2wheel,nullptr);
	pthread_mutex_init(&monitor_motor,nullptr);
	//pthread_create(&send_wheel_thread, NULL, send_wheel, this);
	pthread_create(&recieve_wheel_thread, NULL, recieve_wheel, this);
	pthread_create(&monitor_motor_thread, NULL, monitor_motor_state, this);
	recieve_thread_running = true;
}

/**
 * @brief 电机数据解析
 * @param thisMotor	电机结构体指针
 *
 * @return
 */
Motor::~Motor()
{
	//pthread_join(send_wheel_thread, NULL);
	pthread_join(recieve_wheel_thread, NULL);
	pthread_join(monitor_motor_thread, NULL);
	queue->remove_listener(motor_c, this);
	queue->remove_listener(move_c, this);
	end_can(s);
}

void Motor::MotorDataAnalyze(MotorTypedef *thisMotor, u8 *data)
{

	thisMotor->StatusWord = (data[0] | (data[1] << 8));
	thisMotor->ActualPosition = (data[2] | (data[3] << 8) | (data[4] << 16) | (data[5] << 24));

	thisMotor->EnableStatus = (u8)(thisMotor->StatusWord & 0x04) >> 2;
	thisMotor->Fault = (u8)(thisMotor->StatusWord & 0x08) >> 3;

	//电机故障时，将MotorError_Flag置1
	if (thisMotor->Fault == 1)
		MotorError_Flag = 1;

	thisMotor->timeStamp = get_current_time_us();
	//printf("%x ,%x ,%x ,%x ,%ld\n", thisMotor->StatusWord, thisMotor->ActualPosition, thisMotor->EnableStatus, thisMotor->Fault, thisMotor->timeStamp);
}
void Motor::MotorDataAnalyze_YB(MotorTypedef *thisMotor, u8 *data,char an_type)
{
    if(an_type==1)////解析伺服的状态反馈
    {
        //printf("status %x %x %x %x \n",data[0],data[1],data[2],data[3]);
    thisMotor->StatusWord = (data[0]| (data[1] << 8));
    thisMotor->StatusWord2 = (data[2]| (data[3] << 8));

    thisMotor->EnableStatus = ((thisMotor->StatusWord & 0x03)==0x03)?1:0;
    thisMotor->EnableStatus2 = ((thisMotor->StatusWord2 & 0x03)==0x03)?1:0;
    thisMotor->Fault = ((thisMotor->StatusWord & 0x80)>0)?1:0;
    }
    else if(an_type==2)////解析伺服的位置
    {
        
    thisMotor->ActualPosition = (data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24));
    thisMotor->ActualPosition2 = (data[4] | (data[5] << 8) | (data[6] << 16) | (data[7] << 24));
    //printf("position %d %d \n",thisMotor->ActualPosition,thisMotor->ActualPosition2);
    thisMotor->timeStamp = get_current_time_us();
    }
}
/**
 * @brief 开启站号
 *
 * @param	nodeID 站号
 * @return
 */
void Motor::SwitchPDO(MotorTypedef thisMotor)
{

	u8 data[8] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	data[1] = thisMotor.NodeID;
	int ret;
	//CanSend(0x000, data);
	ret = can_send(s, 0x000, data);
	//if(ret < 0)
	//printf("canid not ready!\n");
}

/**
 * @brief 设为速度模式
 *
 * @param	nodeID 站号
 * @return
 */
void Motor::motorSetSpeedMode(MotorTypedef thisMotor)
{
	int ret;
	u8 data[8] = {0x0F, 0X00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00};

	ret = can_send(s, (0x200 + thisMotor.NodeID), data);
	//if(ret < 0)
	//printf("speed_model not ready!\n");
	//CanSend(s,(0x200 + thisMotor.NodeID), data);
}
/**
 * @brief 设为速度模式
 *
 * @param	nodeID 站号
 * @return
 */
void Motor::motorSetSpeedMode_YB(MotorTypedef thisMotor)
{
    int ret;
    u8 data1[8] = {0x2F, 0X0F, 0x20, 0x00, 0x01, 0x00, 0x00, 0x00};
    //ret = can_send(s, (0x600 + thisMotor.NodeID), data1);
    //usleep(5000);
    u8 data[8] = {0x2F, 0X60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00};
    ret = can_send(s, (0x600 + thisMotor.NodeID), data);



}
void Motor::motorDisable(MotorTypedef thisMotor)
{
	int ret;
	u8 data[8] = {0x06, 0X00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};

	ret = can_send(s, (0x200 + thisMotor.NodeID), data);
	//if(ret < 0)
	//printf("speed_model not ready!\n");
	//CanSend(s,(0x200 + thisMotor.NodeID), data);
}
void Motor::motorDisable_YB(MotorTypedef thisMotor)
{
    int ret;
    u8 data[8] = {0x2B, 0X40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00};
    ret = can_send(s, (0x600 + thisMotor.NodeID), data); 
}
void Motor::motorEnable_YB(MotorTypedef thisMotor)
{
    int ret;
    u8 data[8] = {0x2B, 0X40, 0x60, 0x00, 0x07, 0x00, 0x00, 0x00};
    ret = can_send(s, (0x600 + thisMotor.NodeID), data); 
    usleep(5000);
    u8 data1[8] = {0x2B, 0X40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00};
    ret = can_send(s, (0x600 + thisMotor.NodeID), data1); 
}
/**
 * @brief 设为位置模式
 *
 * @param	nodeID 站号
 * @return
 */
void Motor::motorSetPositionMode(MotorTypedef thisMotor)
{
	int ret;
	u8 data[8] = {0x3F, 0x10, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
	ret = can_send(s, (0x200 + thisMotor.NodeID), data);
	//if(ret < 0)
	//printf("position_model not ready!\n");
	//CanSend((0x200 + thisMotor.NodeID), data);
}

/**
 * @brief 下发速度
 *
 * @param	nodeID 站号
 * @param	speed 速度
 * @return
 */
void Motor::motorSpeedControl(MotorTypedef thisMotor)
{

	int speed, ret;

	u8 data[8] = {0x00, 0X00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

	speed = thisMotor.SpeedRatio;

	data[0] = (u8)speed;
	data[1] = (u8)(speed >> 8);
	data[2] = (u8)(speed >> 16);
	data[3] = (u8)(speed >> 24);

	// for (int i = 0; i < 8; i++)
	// 	printf("%x\t", data[i]);
	// printf("\n");

	ret = can_send(s, (0x300 + thisMotor.NodeID), data);
	//if(ret < 0)
	//printf("speed_control_model not ready!\n");
	//CanSend((0x300 + thisMotor.NodeID), data);
}
void Motor::motorSpeedControl_YB(MotorTypedef thisMotor)
{

    int ret;
    short speed,speed1;
    u8 data[8] = {0x23, 0xFF, 0x60, 0x01, 0x00, 0x00, 0x00, 0x00};
    speed = (short)thisMotor.SpeedRatio;
    speed1 = (short)thisMotor.SpeedRatio2;
    data[4] = (u8)speed;
    data[5] = (u8)(speed >> 8);
    data[6] = (u8)(speed >> 16);
	data[7] = (u8)(speed >> 24);
    //data[6] = (u8)(speed1);
    //data[7] = (u8)(speed1 >> 8);
    //printf("%x %x %x %x\n",data[4],data[5],data[6],data[7]);
	ret = can_send(s, (0x600 + thisMotor.NodeID), data);
    usleep(50);
    data[3] = 0x02;
	data[4] = (u8)speed1;
    data[5] = (u8)(speed1 >> 8);
    data[6] = (u8)(speed1 >> 16);
	data[7] = (u8)(speed1 >> 24);
    ret = can_send(s, (0x600 + thisMotor.NodeID), data);
}
/**
 * @brief 下发目标位置
 *
 * @param	nodeID 站号
 * @param	speed 速度
 * @param	position 目标位置
 * @return
 */
void Motor::motorPositionControl(MotorTypedef thisMotor)
{

	u32 speed;
	int position, ret;

	u8 data[8] = {0x00, 0X00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

	speed = abs(thisMotor.SpeedRatio);

	data[0] = (u8)(speed);
	data[1] = (u8)(speed >> 8);
	data[2] = (u8)(speed >> 16);
	data[3] = (u8)(speed >> 24);

	position = thisMotor.ExpectPositionRatio;
	data[4] = (u8)(position);
	data[5] = (u8)(position >> 8);
	data[6] = (u8)(position >> 16);
	data[7] = (u8)(position >> 24);

	ret = can_send(s, (0x400 + thisMotor.NodeID), data);
	//if(ret < 0)
	//printf("position_control_model not ready!\n");
	//CanSend((0x400 + thisMotor.NodeID), data);
}

/**
 * @brief 电机驱动
 *
 * @return
 */
// void Motor::MotorRun(void)
// {

// 	motorSpeedControl(Motor_L);
// 	motorSpeedControl(Motor_R);
// }

/**
 * @brief 电机停止
 *
 * @return
 */
void Motor::MotorStop(void)
{

	u8 i;

	for (i = 0; i < 4; i++)
		MotorRun(0, 0);
}
void *Motor::monitor_motor_state(void *param)
{
    Motor *ptr = (Motor *)param;
    int num = 0;
    sleep(3);
    while (true)
    {
        ptr->detect_cnt++;
        if(ptr->detect_cnt>=10) 
        {
            ptr->motor_disconnect = true;
            printf("the motor is not connected,please check\n");
            ptr->detect_cnt = 0;

        }
        if(ptr->Model_type == ROBOT_DIFF_MODEL_CARRY)
        {
        if (!(ptr->diff_drv.Motor_L.EnableStatus && ptr->diff_drv.Motor_R.EnableStatus && ptr->lft_drv.Motor_LIFT.EnableStatus && ptr->lft_drv.Motor_TURN.EnableStatus))
        ptr->motor_error = true;
        else
        ptr->motor_error = false;
        }
        else if(ptr->Model_type == ROBOT_DUAL_STEER_MODEL)
        {
        if (!(ptr->dual_steer_drv.Motor_Run_Front.EnableStatus && 
		ptr->dual_steer_drv.Motor_Run_Back.EnableStatus &&
		 ptr->dual_steer_drv.Motor_Turn_Front.EnableStatus && 
		 ptr->dual_steer_drv.Motor_Turn_Back.EnableStatus))
        ptr->motor_error = true;
        else
        ptr->motor_error = false;
        }///ROBOT_DIFF_MODEL_COMMON
        else if(ptr->Model_type == ROBOT_DIFF_MODEL_COMMON)
        {
        if (!(ptr->diff_drv.Motor_L.EnableStatus && ptr->diff_drv.Motor_R.EnableStatus))
        ptr->motor_error = true;
        else
        ptr->motor_error = false;			
        }
        else if(ptr->Model_type == ROBOT_DIFF_MODEL_COMMON_YB)
        {
        if (!(ptr->diff_drv.Motor_YB.EnableStatus&&ptr->diff_drv.Motor_YB.EnableStatus2))
        ptr->motor_error = true;
        else
        ptr->motor_error = false;			
        }

        usleep(200000);
    }
}
void Motor::recieve_message(const int channel, char *buf, const int size)
{

	if (channel == motor_c)
	{
        Data_out data;
        data.from_char_array(buf, size);
        handle_music_data(data);
        handle_out_data(data);
        //printf("recv line_vel=%f,ang_vel=%f\n",data.move_info.vel_l,data.move_info.vel_r);
        if(Model_type == ROBOT_DIFF_MODEL_CARRY)
        {handle_motor_data(data);
        handle_lift_data(data);
        }
        else if(Model_type == ROBOT_DUAL_STEER_MODEL)
        handle_steer_motor_data(data);
        else if(Model_type == ROBOT_DIFF_MODEL_COMMON)
        handle_motor_data(data);
        else if(Model_type == ROBOT_DIFF_MODEL_COMMON_YB)
        handle_motor_data(data);
	}

	// printf("odom insert imu data %ld\n", data.timestamp_us);
}
void Motor::handle_lift_data(Data_out logic_data)
{
    if(logic_data.f_lift == 0) return;
    if(logic_data.lift_info.lift_state == LIFT_UP)dir = 1;
    if(logic_data.lift_info.lift_state == LIFT_DOWN)dir = 0;
    if(logic_data.lift_info.lift_state == LIFT_TURN)dir = 1;
    if(logic_data.lift_info.lift_state == LIFT_TURN_ZERO)dir = 0;
    if(logic_data.lift_info.lift_state == LIFT_ALL_ZERO)dir = 0;
    lft_drv.Lift_MotorRun(logic_data);
	motorPositionControl(lft_drv.Motor_LIFT);
	motorPositionControl(lft_drv.Motor_TURN);
}

void *Motor::recieve_wheel(void *param)
{
	//printf("ok1\n");

	Motor *re_motor = (Motor *)param;
	int ret;
	re_motor->s=re_motor->init_can();
    re_motor->reset_motor();
	int j = 0, k = 0;
	//sleep(2);
	//printf("%x\n",re_motor->fr_recieve.can_id);

	while (re_motor->recieve_thread_running)
	{
		ret = re_motor->can_receive(re_motor->s, &(re_motor->fr_recieve));
		//printf("aaaaaaaaaaaaaaaaaaaaaaaaaaa%x\n",re_motor->fr_recieve.can_id);
        re_motor->detect_cnt = 0;
        if(re_motor->io_board_can_enable <= 0) 
        if(re_motor->fr_recieve.can_id == 0x08)////if recv the control board data
        {
            Ctl_board_data cltdata(0,0);
            cltdata.data[0]=re_motor->fr_recieve.data[0];     cltdata.data[1]=re_motor->fr_recieve.data[1];    
            cltdata.data[2]=re_motor->fr_recieve.data[2];     cltdata.data[3]=re_motor->fr_recieve.data[3];  
            cltdata.data[4]=re_motor->fr_recieve.data[4];     cltdata.data[5]=re_motor->fr_recieve.data[5];  
            cltdata.data[6]=re_motor->fr_recieve.data[6];     cltdata.data[7]=re_motor->fr_recieve.data[7];
            std::vector<char> sendbuf;
            cltdata.to_char_array(&sendbuf);
            re_motor->queue->send(CHANNEL_CTL_BOARD, sendbuf.data(), sendbuf.size());       
        }

        if(re_motor->Model_type == ROBOT_DIFF_MODEL_CARRY||(re_motor->Model_type == ROBOT_DIFF_MODEL_COMMON))
		{
        switch (re_motor->fr_recieve.can_id)
		{
		case 0x185:
		{
            //printf("ok30000000000000000000000000000000000000000\n");
            re_motor->MotorDataAnalyze(&re_motor->diff_drv.Motor_L, re_motor->fr_recieve.data);
            std::vector<char> tmp;
            WheelSensorData data;
            data.timestamp = re_motor->diff_drv.Motor_L.timeStamp;
            data.data = -re_motor->diff_drv.Motor_L.ActualPosition;
            data.type = WHEEL_DATA_TYPE_LEFT;
            //std::cout<<data.data<<std::endl;
            data.to_char_array(&tmp);
            re_motor->queue->send(re_motor->wheel_c, tmp.data(), tmp.size());
		}
		break;
		case 0x186:
		{
            //printf("ok4999999999999999999999999999999999999\n");

            re_motor->MotorDataAnalyze(&re_motor->diff_drv.Motor_R, re_motor->fr_recieve.data);
            std::vector<char> tmp;
            WheelSensorData data;
            data.timestamp = re_motor->diff_drv.Motor_R.timeStamp;
            data.data = re_motor->diff_drv.Motor_R.ActualPosition;
            data.type = WHEEL_DATA_TYPE_RIGHT;
            //std::cout<<data.data<<std::endl;
            data.to_char_array(&tmp);
            re_motor->queue->send(re_motor->wheel_c, tmp.data(), tmp.size());
		}
		break;
		case 0x187:
		{
            re_motor->MotorDataAnalyze(&re_motor->lft_drv.Motor_LIFT, re_motor->fr_recieve.data);
            re_motor->is_arrive = re_motor->lft_drv.LiftMachine_TaskComplete();
            //printf("111111111111111111%d\n",re_motor->is_arrive);
            if (re_motor->is_arrive == true)
            {
            	if (re_motor->queue != nullptr && re_motor->dir == 1)
            	{
            		re_motor->queue->send(CHANNEL_ARRIVE, (char *)(cmd), sizeof(int));
            		re_motor->dir = -1;
            		std::cout<<"顶升完成!!!\n"<<std::endl;
            	}
            	else if (re_motor->queue != nullptr && re_motor->dir == 0)
            	{
            		re_motor->queue->send(CHANNEL_ARRIVE, (char *)(cmd+1), sizeof(int));
            		re_motor->dir = -1;
            		std::cout<<"顶升下降完成!!!\n"<<std::endl;
            	}
            }
		}
            break;
		case 0x188:
		{
            re_motor->MotorDataAnalyze(&re_motor->lft_drv.Motor_TURN, re_motor->fr_recieve.data);
            re_motor->is_arrive = re_motor->lft_drv.LiftMachine_TaskComplete();
            //printf("22222222222222%d\n",re_motor->is_arrive);
            if (re_motor->is_arrive == true)
            {
            	if (re_motor->queue != nullptr && re_motor->dir == 1)
            	{
            		re_motor->queue->send(CHANNEL_ARRIVE, (char *)(cmd), sizeof(int));
            		re_motor->dir = -1;
            		std::cout<<"顶升完成!!!\n"<<std::endl;
            	}
            	else if (re_motor->queue != nullptr && re_motor->dir == 0)
            	{
            		re_motor->queue->send(CHANNEL_ARRIVE, (char *)(cmd+1), sizeof(int));
            		re_motor->dir = -1;
            		std::cout<<"顶升下降完成!!!\n"<<std::endl;

            	}
            }
		}
		break;
		default:
            //printf("wheel data error!!!\n");
        break;
        }
        }
        else if(re_motor->Model_type == ROBOT_DUAL_STEER_MODEL)
        {
        switch (re_motor->fr_recieve.can_id)
        {
        case 0x183:
        {
            re_motor->MotorDataAnalyze(&re_motor->dual_steer_drv.Motor_Run_Front, re_motor->fr_recieve.data);
            std::vector<char> tmp;
            WheelSensorData data;
            data.timestamp = re_motor->dual_steer_drv.Motor_Run_Front.timeStamp;
            data.data = re_motor->dual_steer_drv.Motor_Run_Front.ActualPosition;
            data.type = WHEEL_DATA_TYPE_STEER_FV1;
            data.to_char_array(&tmp);
            re_motor->queue->send(re_motor->wheel_c, tmp.data(), tmp.size());
        }
        break;
        case 0x185:
        {
            re_motor->MotorDataAnalyze(&re_motor->dual_steer_drv.Motor_Run_Back, re_motor->fr_recieve.data);
            std::vector<char> tmp;
            WheelSensorData data;
            data.timestamp = re_motor->dual_steer_drv.Motor_Run_Back.timeStamp;
            data.data = re_motor->dual_steer_drv.Motor_Run_Back.ActualPosition;
            data.type = WHEEL_DATA_TYPE_STEER_BV1;
            data.to_char_array(&tmp);
            re_motor->queue->send(re_motor->wheel_c, tmp.data(), tmp.size());
        }
        break;
        case 0x184:
        {
            re_motor->MotorDataAnalyze(&re_motor->dual_steer_drv.Motor_Turn_Front, re_motor->fr_recieve.data);
            std::vector<char> tmp;
            WheelSensorData data;
            data.timestamp = re_motor->dual_steer_drv.Motor_Turn_Front.timeStamp;
            data.data = re_motor->dual_steer_drv.Motor_Turn_Front.ActualPosition;
            data.type = WHEEL_DATA_TYPE_STEER_FR1;
            data.to_char_array(&tmp);
            re_motor->queue->send(re_motor->wheel_c, tmp.data(), tmp.size());
        }
        break;
		case 0x186:
		{
            re_motor->MotorDataAnalyze(&re_motor->dual_steer_drv.Motor_Turn_Back, re_motor->fr_recieve.data);
            std::vector<char> tmp;
            WheelSensorData data;
            data.timestamp = re_motor->dual_steer_drv.Motor_Turn_Back.timeStamp;
            data.data = re_motor->dual_steer_drv.Motor_Turn_Back.ActualPosition;
            data.type = WHEEL_DATA_TYPE_STEER_BR1;
            data.to_char_array(&tmp);
            re_motor->queue->send(re_motor->wheel_c, tmp.data(), tmp.size());
        }
        break;
        default:
        printf("wheel data error!!!\n");
        break;
        }
        }
        else if(re_motor->Model_type == ROBOT_DIFF_MODEL_COMMON_YB)
        {
            //printf("接收到can id %x 数据\n",re_motor->fr_recieve.can_id);
        switch (re_motor->fr_recieve.can_id)
        {
        case 0x285:
        {    
            re_motor->MotorDataAnalyze_YB(&re_motor->diff_drv.Motor_YB, re_motor->fr_recieve.data,2);
            std::vector<char> tmp;
            WheelSensorData data;
            data.timestamp = re_motor->diff_drv.Motor_YB.timeStamp;
            data.data = re_motor->diff_drv.Motor_YB.ActualPosition2;
            data.type = WHEEL_DATA_TYPE_RIGHT;
            data.to_char_array(&tmp);
            re_motor->queue->send(re_motor->wheel_c, tmp.data(), tmp.size());
        }
        break;
        case 0x185:
        {re_motor->MotorDataAnalyze_YB(&re_motor->diff_drv.Motor_YB, re_motor->fr_recieve.data,1);
            std::vector<char> tmp;
            WheelSensorData data;
            data.timestamp = re_motor->diff_drv.Motor_YB.timeStamp;
            data.data = -re_motor->diff_drv.Motor_YB.ActualPosition;
            data.type = WHEEL_DATA_TYPE_LEFT;
            data.to_char_array(&tmp);
            re_motor->queue->send(re_motor->wheel_c, tmp.data(), tmp.size());
        }
            break;
        default:
        printf("wheel data error!!!\n");
        break;
        }

        }
		re_motor->motor_disconnect = false;
        //usleep(10000);
    }
}

// m/s   rad/s
void Motor::MotorRun(Data_out data)
{
    if(Model_type == ROBOT_DIFF_MODEL_COMMON||(Model_type == ROBOT_DIFF_MODEL_CARRY))
    {
    diff_drv.Diff_MotorRun(data);
    motorSpeedControl(diff_drv.Motor_L);
    motorSpeedControl(diff_drv.Motor_R);
    }
    else if(Model_type == ROBOT_DIFF_MODEL_COMMON_YB)
    {
    diff_drv.Diff_MotorRun_YB(data);
    
    motorSpeedControl_YB(diff_drv.Motor_YB);

    }
}

void Motor::MotorRun(float vl, float vr)
{

    diff_drv.Diff_MotorRun(vl,vr);
	motorSpeedControl(diff_drv.Motor_L);
	motorSpeedControl(diff_drv.Motor_R);
}
/*多级避障处理*////此处已放到logic层处理，这里需要删除
void Motor::handle_motor_data(Data_out data)
{
	//printf("ok222222222222222222\n");
	//level = get_global_agv_instance()->collision_level;
    if(data.f_move == 0) return;

	//printf("ok111\n");
    /*	if (level == 1)
		MotorRun(0, 0);
	else if (level == 2)
	{
		MotorRun(data.move_info.vel_l/2, data.move_info.vel_r/2);
	}
	else if (level == 3)
	{
		MotorRun(data);
		printf("Action!!!\n");
	}
	else*/
		MotorRun(data);
}
void Motor::SteerWheel_MotorRun(Data_out data)
{
    dual_steer_drv.SteerWheel_MotorRun(data);
    motorSpeedControl(dual_steer_drv.Motor_Run_Front);
    motorSpeedControl(dual_steer_drv.Motor_Run_Back);
    motorPositionControl(dual_steer_drv.Motor_Turn_Front);
    motorPositionControl(dual_steer_drv.Motor_Turn_Back);
}
void Motor::SteerWheel_MotorRun(double front_Velocity,double back_Velocity,double front_Angular,double back_Angular)
{
    dual_steer_drv.SteerWheel_MotorRun(front_Velocity,back_Velocity,front_Angular,back_Angular);
    motorSpeedControl(dual_steer_drv.Motor_Run_Front);
    motorSpeedControl(dual_steer_drv.Motor_Run_Back);
    motorPositionControl(dual_steer_drv.Motor_Turn_Front);
    motorPositionControl(dual_steer_drv.Motor_Turn_Back);
}

void Motor::handle_steer_motor_data(Data_out logic_out)
{
    //level = get_global_agv_instance()->collision_level;
	if(logic_out.f_move == 0) return;

    //double front_velocity = logic_out.move_info.vel_f_steer;
    //double back_velocity = logic_out.move_info.vel_b_steer;///////////////////////////////////////////////////////////////////////////////這裡要和師兄對接
    //double front_angular = logic_out.move_info.rot_f_steer;
    //double back_angular = logic_out.move_info.rot_b_steer;
/*	if (level == 1)
	{
		front_velocity = 0;
		back_velocity = 0;///////////////////////////////////////////////////////////////////////////////這裡要和師兄對接
		SteerWheel_MotorRun(front_velocity,back_velocity,front_angular,back_angular);
	}
	else if (level == 2)
	{
		front_velocity *= 0.5;
		back_velocity *= 0.5;///////////////////////////////////////////////////////////////////////////////這裡要和師兄對接
		SteerWheel_MotorRun(front_velocity,back_velocity,front_angular,back_angular);
	}
	else if (level == 3)
	{
		SteerWheel_MotorRun(logic_out);
		printf("Action!!!\n");
	}
	else*/
		SteerWheel_MotorRun(logic_out);
}


bool Motor::reset_motor(void) 
{
	u8 i;
    if(Model_type == ROBOT_DIFF_MODEL_CARRY||(Model_type == ROBOT_DIFF_MODEL_COMMON))
    {
    motorDisable(diff_drv.Motor_R);
    motorDisable(diff_drv.Motor_L);
    motorDisable(lft_drv.Motor_LIFT);
    motorDisable(lft_drv.Motor_TURN);
    for (i = 0; i < 4; i++)
    {

        SwitchPDO(diff_drv.Motor_R);
        SwitchPDO(diff_drv.Motor_L);
        SwitchPDO(lft_drv.Motor_LIFT);
        SwitchPDO(lft_drv.Motor_TURN);

        motorSetSpeedMode(diff_drv.Motor_R);
        motorSetSpeedMode(diff_drv.Motor_L);
        motorSetPositionMode(lft_drv.Motor_LIFT);
        motorSetPositionMode(lft_drv.Motor_TURN);
    }
    }
	else if(Model_type == ROBOT_DUAL_STEER_MODEL)
	{
        motorDisable(dual_steer_drv.Motor_Run_Front);
        motorDisable(dual_steer_drv.Motor_Turn_Front);
        motorDisable(dual_steer_drv.Motor_Run_Back);
        motorDisable(dual_steer_drv.Motor_Turn_Back);
		//printf("11111111111111111111111111111111111111111111111111111111\n");
		for (i = 0; i < 4; i++)
		{
			SwitchPDO(dual_steer_drv.Motor_Run_Front);
			SwitchPDO(dual_steer_drv.Motor_Turn_Front);
			SwitchPDO(dual_steer_drv.Motor_Run_Back);
			SwitchPDO(dual_steer_drv.Motor_Turn_Back);

			motorSetSpeedMode(dual_steer_drv.Motor_Run_Front);
			motorSetSpeedMode(dual_steer_drv.Motor_Run_Back);
			motorSetPositionMode(dual_steer_drv.Motor_Turn_Front);
			motorSetPositionMode(dual_steer_drv.Motor_Turn_Back);
		}
	}
    else if(Model_type == ROBOT_DIFF_MODEL_COMMON_YB)
    {
    SwitchPDO(diff_drv.Motor_YB);     
    motorDisable_YB(diff_drv.Motor_YB);
    motorSetSpeedMode_YB(diff_drv.Motor_YB);
    for (i = 0; i < 4; i++)
    {
        //SwitchPDO(diff_drv.Motor_R);
        motorEnable_YB(diff_drv.Motor_YB);
    }
    //printf("");
    }

    return true;
}
int Motor::init_can()
{
    int ret, s;
    struct sockaddr_can addr;
    struct ifreq ifr;
    bool can_enable = true;

    //system("sudo ifconfig can0 down");
    //system("sudo ip link set can0 type can bitrate 1000000");
    //system("sudo ifconfig can0 up");
    printf("this is a can send demo\r\n");

    //1.Create socket
    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0)
    {
        perror("socket PF_CAN failed");
        can_enable = false;
    }

    //2.Specify can0 device
    strcpy(ifr.ifr_name, "can0");
    ret = ioctl(s, SIOCGIFINDEX, &ifr);
    while (ret < 0)
    {
        ret = ioctl(s, SIOCGIFINDEX, &ifr);
        //perror("ioctl failed");
        can_enable = false;
        usleep(200000);
    }

    //3.Bind the socket to can0
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    ret = bind(s, (struct sockaddr *)&addr, sizeof(addr));
    while (ret < 0)
    {
        //perror("bind failed");
        ret = bind(s, (struct sockaddr *)&addr, sizeof(addr));
        can_enable = false;
        usleep(200000);
    }
    can_enable = true;
    //4.Disable filtering rules, do not receive packets, only send
	struct can_filter rfilter[5];
	rfilter[0].can_id = 0x185;
	rfilter[0].can_mask = CAN_SFF_MASK;
	rfilter[1].can_id = 0x186;
	rfilter[1].can_mask = CAN_SFF_MASK;
	rfilter[2].can_id = 0x187;
	rfilter[2].can_mask = CAN_SFF_MASK;
	rfilter[3].can_id = 0x188;
	rfilter[3].can_mask = CAN_SFF_MASK;
	rfilter[4].can_id = 0x285;
	rfilter[4].can_mask = CAN_SFF_MASK;
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));
	
    if (!can_enable)
    {
        printf("can device not initialize!\n");
        can_enable = true;
        return -1;
    }
	printf("can device initialize success=%d!\n",s);
    return s;
}

void Motor::end_can(int sock)
{
    close(sock);
    //system("sudo ifconfig can0 down");
}

int Motor::can_send(int sock, canid_t id, unsigned char *data)
{
    usleep(1000);
        int nbytes;
    struct can_frame fr;
    memset(&fr, 0, sizeof(struct can_frame));
    fr.can_id = id;
    fr.can_dlc = 8;

    for (int i = 0; i < fr.can_dlc; i++)
    {
        fr.data[i] = data[i];
    }
    nbytes = write(sock, &fr, sizeof(fr));
    
    if (nbytes < 0)
        return -1;
    else
        return 0;
}

int Motor::can_receive(int sock,can_frame *fr2)
{
	//printf("niubi66666666666666666666666,%d\n",sock);
    int nbytes;
     nbytes = read(sock, fr2, sizeof(can_frame));
	//printf("niubi99999999999999999999999\n");

       // printf("%x\n",fr2->can_id);
	
    if (nbytes < 0)
        return -1;
    else
        return 0;
}
void Motor::handle_out_data(Data_out data)
{
    if(io_board_can_enable) return;
    if(data.f_led_voice == 0) return;
    send_io_ctl(0x09,data.led_info);
}
void  Motor::send_io_ctl(u8 can_id,st_led out)
{
    int ret;
    u8 data[8] = {0x01, 0x51, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02};
    data[0] = out.led_color;
    data[1] = out.io_extra;
    data[2] = out.led_state;

    ret = can_send(1, (can_id), data);

}
void Motor::play_music(u8 can_id,u8 index,u8 volume)
{
    //if(can_handle<=0) return;
    int ret;
    u8 data[8] = {0x01, 0x51, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02};
    data[2] = index;
    if(volume<10) volume = 10;
    if(volume>28) volume = 28;

    data[3] = volume;
    data[6] = CalculateChecksum(data,6);
    ret = can_send(s, (0x200 + can_id), data);

}

u8 Motor::CalculateChecksum(u8 *aubData_p, int auwDataLength)
{
    u8 aubChecksum = 0;
    int auwCnt = 0;
 
    while(auwCnt < auwDataLength)
    {
        aubChecksum ^= aubData_p[auwCnt];
        auwCnt++;
    }
 
    return aubChecksum;
}

void Motor::handle_music_data(Data_out data)
{
    if(data.f_led_voice == 0) return;
    play_music(0,data.voice_info.index,voice_volum);
}