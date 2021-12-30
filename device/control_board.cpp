/* 步科驱动器CanOpen协议 */

#include "control_board.h"
#include <iostream>
#include <math.h>
//#include "agv.h"
#include "../common/configuration.h"
#include "../common/ctl_data.h"
//int can_handle = 0;
Control_board::Control_board(MsgQueue *queue,  int _chan)
{
    pthread_mutex_init(&mutex, nullptr);
    logic_out_c = _chan;
    this->queue = queue;
    queue->add_listener(logic_out_c, this);
    queue->add_listener(CHANNEL_CTL_BOARD, this);
    io_board_can_enable = get_configs()->get_int("io_board", "isolate_can", nullptr);
    if(io_board_can_enable<0) io_board_can_enable = 0;
    if(io_board_can_enable)
    pthread_create(&recieve_io_thd, NULL, recieve_io_data, this);//commu_detect_thread
    pthread_create(&commu_detect_thd, NULL, commu_detect_thread, this);
    handle_can = 0;
}


Control_board::~Control_board()
{
    pthread_mutex_destroy(&mutex);
    pthread_join(recieve_io_thd, NULL);
    pthread_join(commu_detect_thd, NULL);
    queue->remove_listener(logic_out_c, this);
    queue->remove_listener(CHANNEL_CTL_BOARD, this);
    end_can(handle_can);
}

int Control_board::init_can()
{
    int ret, s;
    struct sockaddr_can addr;
    struct ifreq ifr;
    bool can_enable = true;
    //1.Create socket
    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0)
    {
        printf("socket PF_CAN failed");
        can_enable = false;
    }

    //2.Specify can0 device
    strcpy(ifr.ifr_name, "can1");
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
		ret = bind(s, (struct sockaddr *)&addr, sizeof(addr));
        can_enable = false;
		usleep(200000);
    }
    can_enable = true;

    //4.Disable filtering rules, do not receive packets, only send
	struct can_filter rfilter[1];
	rfilter[0].can_id = 0x08;
	rfilter[0].can_mask = CAN_SFF_MASK;
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));
	printf("can device two dev initialize success=%d!\n",s);
    handle_can = s;
    return s;
}

void Control_board::end_can(int sock)
{
    close(sock);
}



int Control_board::can_receive(int sock,can_frame *fr2)
{
    int nbytes;
     nbytes = read(sock, fr2, sizeof(can_frame));
    if (nbytes < 0)
        return -1;
    else
        return 0;
}

void Control_board::recieve_message(const int channel, char *buf, const int size)
{

	if (channel == logic_out_c)
	{
        Data_out data;
        data.from_char_array(buf, size);
        handle_out_data(data);
	}
	
    if (channel == CHANNEL_CTL_BOARD)
    {
        Ctl_board_data cltdata(0,0);
        cltdata.from_char_array(buf, size);
        IoDataAnalyze(cltdata.data);
        detect_cnt = 0;
        //handle_out_data(cltdata);
    }
}
void *Control_board::commu_detect_thread(void *param)
{
    Control_board *ptr = (Control_board *)param;
    int device_enable = get_configs()->get_int("io_board", "io_enable", nullptr);
    bool running = (device_enable>0)?true:false;
    sleep(2);
    while(running)
    {
        ptr->detect_cnt++;

        if(ptr->detect_cnt>=10) 
        {
            ptr->board_disconnect = true;
            //printf("control board is not connected,warning_code=,error_code=\n");
           // printf("two dim connected,f=%d,s=%d\n",get_global_agv_instance()->two_dim_->device2_disconnect
           // ,get_global_agv_instance()->two_dim_->device1_disconnect);
            ptr->detect_cnt = 0;
        }
        //ptr->update_agv_warning_error();
        usleep(200000);
    }

    printf("can 2 detect thread quit!\n");
    return nullptr;
}
void *Control_board::recieve_io_data(void *param)
{
    Control_board *ptr = (Control_board *)param;
    int ret;
    bool board_err;
    int device_enable = get_configs()->get_int("io_board", "io_enable", nullptr); 
    can_frame _recieve;
    printf("11111111  io board enable =%d \n",device_enable);
    ptr->init_can();
    ptr->io_thread_running = (device_enable>0)?true:false;
    usleep(20000);
	while (ptr->io_thread_running)
	{
        //printf("can 1111111111111111111111111 recv is running\n");
        ret = ptr->can_receive(ptr->handle_can, &(_recieve));
        ptr->detect_cnt = 0;
        board_err = false;
        switch (_recieve.can_id)
        {
        case 0x08:
            ptr->IoDataAnalyze(_recieve.data);
            break;
        default:
            board_err = true;
            break;
        }
        ptr->board_disconnect = false;

    }
    //ptr->end_can(ptr->handle_can);
    printf("can 2 thread quit!\n");
    return nullptr;
}
void Control_board::IoDataAnalyze( u8 *data)
{
    _battary = (char)data[0];
    charging_flag = (char)data[5];
    SignalIn.Input = data[1]*0x100*0x100*0x100+data[2]*0x100*0x100+data[3]*0x100+data[4];
    SignalInputDataAnalyze();
    obs_avoid_l = (SignalIn.LeftInfrared1|SignalIn.LeftInfrared2);
    obs_avoid_r = (SignalIn.RightInfrared1|SignalIn.RightInfrared2);
    obs_avoid_f = (SignalIn.FrontInfrared1|SignalIn.FrontInfrared2);
    obs_avoid_b = (SignalIn.BackInfrared1|SignalIn.BackInfrared2); 

    //printf("obs_avoid_l=%d,obs_avoid_r=%d,obs_avoid_f=%d,obs_avoid_b=%d\n",obs_avoid_l,obs_avoid_r,obs_avoid_f,obs_avoid_b);
    anti_colli_f = 	SignalIn.FrontAntiCrush; /*!< 前 防撞条 */
    anti_colli_b = SignalIn.RearAntiCrush; /*!< 后 防撞条 */
    anti_colli_l = false;
    anti_colli_r = false;

    _emgcy = SignalIn.EmergeButton;
    _emgcy = !_emgcy;
    //_ireset = GreenButton; /*!< 绿色按钮 */
	_reset =  SignalIn.BlueButton; /*!< 蓝色按钮 */
    _continu = SignalIn.GreenButton;
    if((SignalIn.OrangeButton)>0) _manual_auto = AUTO_MODE; else _manual_auto = MANUAL_MODE;
    //printf("io board sts GreenButton=%d  BlueButton=%d\n",SignalIn.GreenButton,SignalIn.BlueButton);
    //printf("io board sts EmergeButton=%d  OrangeButton=%d  battery=%d\n",SignalIn.EmergeButton,SignalIn.OrangeButton,_battary);
    /*int iobs_avoid_f = 0x01;//////避障传感器
    int iobs_avoid_b = 0x02;
    int iobs_avoid_l = 0x04;
    int iobs_avoid_r = 0x08;

    int ianti_colli_f = 0x10;//////防撞条
    int ianti_colli_b = 0x20;
    int ianti_colli_l = 0x40;
    int ianti_colli_r = 0x80;

    int _iemgcy = 0x01;//////急停
    int _istop = 0x02;//////暂停
    int _icontinu = 0x04;////////继续
    int _ireset = 0x08;//////复位
    int _imanual_auto = 0x10;////手动自动
    if((data[0]&iobs_avoid_f)>0) obs_avoid_f = true; else obs_avoid_f = false;
    if((data[0]&iobs_avoid_b)>0) obs_avoid_b = true; else obs_avoid_b = false;
    if((data[0]&iobs_avoid_l)>0) obs_avoid_l = true; else obs_avoid_l = false;
    if((data[0]&iobs_avoid_r)>0) obs_avoid_r = true; else obs_avoid_r = false;

    if((data[0]&ianti_colli_f)>0) anti_colli_f = true; else anti_colli_f = false;
    if((data[0]&ianti_colli_b)>0) anti_colli_b = true; else anti_colli_b = false;
    if((data[0]&ianti_colli_l)>0) anti_colli_l = true; else anti_colli_l = false;
    if((data[0]&ianti_colli_r)>0) anti_colli_r = true; else anti_colli_r = false;

    if((data[1]&_iemgcy)>0)     
    _emgcy = false; 
    else
    {
        _emgcy = true;
    }
    if((data[1]&_istop)>0) 
    {_stop = true;///////

    }

    if((data[1]&_icontinu)>0) 
    {_continu = true;///////
    }

    /////////////////////如果接收到reset指令，则先清除运行的命令，再重新使能伺服
    if((data[1]&_ireset)>0) 
    {_ireset = true;///////
    }

    if((data[1]&_imanual_auto)>0) _manual_auto = AUTO_MODE; else _manual_auto = MANUAL_MODE;
    _battary = (char)data[2];*/
    update_agv_status();
}
void Control_board::update_agv_status()
{

    if(_battary<warning_bat)
    _battary_warning = true;/////电量报警
    else 
    _battary_warning = false;/////电量报警
   

}
void Control_board::update_agv_warning_error()
{
;
}
void Control_board::SignalInputDataAnalyze(void) {
	
	/* 按钮数据处理（1-4） */
	SignalIn.GreenButton = (SignalIn.Input >> 0) & 0x1;
	SignalIn.BlueButton = (SignalIn.Input >> 1) & 0x1;
	//ggggg = (SignalIn.Input >> 1) & 0x1;
	SignalIn.EmergeButton = (SignalIn.Input >> 2) & 0x1;
	SignalIn.OrangeButton = (SignalIn.Input >> 3) & 0x1;

	/* 避障和激光数据处理（12） */
	SignalIn.FrontAntiCrush = (SignalIn.Input >> 11) & 0x1;
	//SignalIn.FrontLaser = (SignalIn.Input >> 12) & 0x7;
	/* 16 */
	SignalIn.RearAntiCrush = (SignalIn.Input >> 15) & 0x1;

	/* 红外	17-24 */
	SignalIn.LeftInfrared1 = (SignalIn.Input >> 18) & 0x1;
	SignalIn.RightInfrared1 = (SignalIn.Input >> 17) & 0x1;
	SignalIn.LeftInfrared2 = (SignalIn.Input >> 16) & 0x1;
	SignalIn.RightInfrared2 = (SignalIn.Input >> 19) & 0x1;
	SignalIn.FrontInfrared1 = (SignalIn.Input >> 20) & 0x1;
	SignalIn.FrontInfrared2 = (SignalIn.Input >> 21) & 0x1;
	SignalIn.BackInfrared1 = (SignalIn.Input >> 23) & 0x1;
	SignalIn.BackInfrared2 = (SignalIn.Input >> 22) & 0x1;
	
}
void Control_board::handle_out_data(Data_out data)
{
    if(io_board_can_enable <=0 ) return ;
    if(data.f_led_voice == 0) return;
    send_io_ctl(0x09,data.led_info);
}
void  Control_board::send_io_ctl(u8 can_id,st_led out)
{
    int ret;
    u8 data[8] = {0x01, 0x51, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02};
    data[0] = out.led_color;
    data[1] = out.io_extra;
    data[2] = out.led_state;

    ret = can_send(1, (can_id), data);

}
int Control_board::can_send(int sock, canid_t id, unsigned char *data)
{
    //return -1;
    if(handle_can<=0) return -1;
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
    nbytes = write(handle_can, &fr, sizeof(fr));
    
    if (nbytes < 0)
        return -1;
    else
        return 0;
}