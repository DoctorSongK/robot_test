/* 步科驱动器CanOpen协议 */

#include "logic_process.h"
#include <iostream>
#include <math.h>
#include "agv.h"
Logic_process::Logic_process(MsgQueue *queue,  int war_err_chan, int speed_chan, int lift_chan)
{
    pthread_mutex_init(&mutex, nullptr);
    war_err_c = war_err_chan;
    speed_c = speed_chan;
    lift_c = lift_chan;
    //output_c = out_chan;
    kine_slove = new Model_kine();
    this->queue = queue;
    queue->add_listener(CHANNEL_ARRIVE, this);
    queue->add_listener(CHANNEL_LASER_OBS_OUT, this);
    
    queue->add_listener(war_err_c, this);
    queue->add_listener(speed_c, this);
    queue->add_listener(lift_c, this);
    use_radar_type = get_configs()->get_int("radar", "use_radar_type", nullptr);
    pthread_create(&output_thd, NULL, output_disposed_data, this);//commu_detect_thread
}


Logic_process::~Logic_process()
{
    pthread_mutex_destroy(&mutex);
    pthread_join(output_thd, NULL);
    queue->remove_listener(war_err_c, this);
    queue->remove_listener(speed_c, this);
    queue->remove_listener(lift_c, this);
    queue->remove_listener(CHANNEL_ARRIVE, this);
    queue->remove_listener(CHANNEL_LASER_OBS_OUT, this);
}


void Logic_process::recieve_message(const int channel, char *buf, const int size)
{

    if (channel == CHANNEL_LASER_OBS_OUT && use_radar_type == 0)
    {
        obs_data.from_char_array(buf, size);
	}
    if (channel == CHANNEL_LASER_OBS_OUT && use_radar_type == 1){
        obs_3d_data.from_char_array(buf,size);
       // printf("2222222222222222222 %d,%d,%d\n",obs_3d_data.laser_obs_level,obs_3d_data.laser_obs_level_l,obs_3d_data.laser_obs_level_p);
    }
    if(channel == CHANNEL_ARRIVE)
    {
        int cmd=*((int*)buf);
        switch(cmd){
            case 0xc103://上报顶升上升完成
                if(up_flag)
                {
                    lift_up_cnt++;
                    up_flag = false;
                }
                get_global_agv_instance()->lift_complete = true;
                break;
            case 0xc104://上报顶升下降完成
                if(down_flag)
                {
                    lift_down_cnt++;
                    down_flag = false;
                }
                get_global_agv_instance()->lift_complete = true;
                break;
            default:
                break;
        }
    }    

    if (channel == war_err_c)
    {
        Warn_err_data data;
        data.from_char_array(buf, size);
        warn_code = data.warn_code;
        err_code = data.err_code;
        board_sts = data.ctl_sts;
        device_sts = data.dev_sts;
        ob_3d_sts = data.ob_3d_sts;
	if(err_code>0) agv_cur_state = ERRORSTATE;
        else if(warn_code>0) agv_cur_state = WARNINGSTATE;
        else agv_cur_state = NORMALSTATE;

        f_led_voice_recv = 1;
    }
    if (channel == speed_c)
    {
        speed_data.from_char_array(buf, size);
        ////
        
        //printf("recv line vel =%f  ang_vel =%f\n",speed_data.vel_line,speed_data.vel_ang);
        f_move_recv = 1;
    }
    if (channel == lift_c)
    {
        //Lift_data data; 
        //data.from_char_array(buf, size);
        //lift_in.lift_height = data.lift_height;
        //lift_in.lift_angle = data.lift_angle;
        //lift_in.lift_state = data.lift_state;
        MoveInstruction data;
        data.from_char_array(buf, size);
/*        if(data.type = MOVE_INSTRUCTION_TYPE_TURN)
        {lift_in.lift_state = LIFT_TURN;
        lift_in.lift_angle = data.turn.angle;
        printf("logic process 接收到顶升旋转指令\n");
        }
*/
        if(data.type == MOVE_INSTRUCTION_TYPE_LIFT)
        {
        if(data.lift.dir == 1){
        lift_in.lift_state = LIFT_UP;
        up_flag = true;
        get_global_agv_instance()->lift_complete = false;
        }
        else if(data.lift.dir == 0){
        lift_in.lift_state = LIFT_DOWN;
        down_flag = true;
        get_global_agv_instance()->lift_complete = false;
        }
        else
        lift_in.lift_state = LIFT_NONE;
        f_lift_recv = 1;
        printf("logic process 接收到顶升升降指令\n");
        
        }
        else if(data.type == MOVE_INSTRUCTION_TYPE_TURN)
        {
            if(data.turn.dir == 4)
            {
            lift_in.lift_state = LIFT_TURN;
            lift_in.lift_angle = data.turn.angle;
            //if(warn_code>0)
            //lift_in.angle_speed = data.turn.angle_speed*0.5;
            //else
            lift_in.angle_speed = data.turn.angle_speed;
            f_lift_recv = 1;
            printf("logic process 接收到顶升旋转指令\n");
            get_global_agv_instance()->lift_complete = false;
            }
            else if(data.turn.dir == 2)
            {
            lift_in.lift_state = LIFT_TURN_ZERO;
            lift_in.lift_angle = data.turn.angle;
            //if(warn_code>0)
            //lift_in.angle_speed = data.turn.angle_speed*0.5;
            //else
            lift_in.angle_speed = data.turn.angle_speed;
            f_lift_recv = 1;
            printf("logic process 接收到顶升旋转回零指令\n");    
            get_global_agv_instance()->lift_complete = false;        
            }
            else if(data.turn.dir == 3)
            {
            lift_in.lift_state = LIFT_ALL_ZERO;
            lift_in.lift_angle = data.turn.angle;
            //if(warn_code>0)
            //lift_in.angle_speed = data.turn.angle_speed*0.5;
            //else
            lift_in.angle_speed = data.turn.angle_speed;
            f_lift_recv = 1;
            printf("logic process 接收到顶升全回零指令\n");   
            get_global_agv_instance()->lift_complete = false;        
            }
            else
            lift_in.lift_state = LIFT_NONE;
        }
        

        //lift_in.lift_angle = data.turn.angle;
        
        
    }
	
}
void *Logic_process::output_disposed_data(void *param)
{
    Logic_process *ptr = (Logic_process *)param;
    sleep(1);
    while(true)
    {
        ptr->warning_code_dispose();////报警代码处理
        ptr->error_code_dispose();/////错误代码处理
        
        switch(ptr->agv_cur_state)
        {
        case NORMALSTATE:
        ptr->normal_state_run();
        break;
        case WARNINGSTATE:
        ptr->warnning_state_run();
        //ptr->normal_state_run();
        break;
        case ERRORSTATE:
        ptr->error_state_run();
        //ptr->normal_state_run();
        break;
        default:
        break;

        }


        usleep(20000);
    }

    printf(" logic out process thread quit!\n");
    return nullptr;
}
void Logic_process::normal_state_run()/////机器人正常运行
{
    speed_ratio = 1.0;
    if(f_led_voice_recv)
    {
        logic_out.f_led_voice = 1;
        f_led_voice_recv = 0;
    }
    if(f_lift_recv)
    {
        logic_out.f_lift = 1;
        f_lift_recv = 0;
    }
    if(f_move_recv)
    {
        logic_out.f_move = 1;
        f_move_recv = 0;
    }
    logic_out.led_info = led_in;
    logic_out.voice_info = voice_in;
    logic_out.lift_info = lift_in;
    speed_data.vel_line *= speed_ratio;////线速度减半
    speed_data.vel_ang *= speed_ratio;////角速度减半
    
    if((!device_sts.laser_disconnect)&&(!device_sts.pose_initilized))
    {
        speed_data.vel_line *= 0.0;////如果机器人处于激光雷达匹配阶段，则不能驱动机器人运动，必须等待机器人初始定位完成
        speed_data.vel_ang *= 0.0;////
    }
    if(board_sts.anti_colli_f&&(!board_sts.anti_colli_b))/////如果前向防撞触发,只能向后运动
    {
    if(speed_data.vel_line>0) speed_data.vel_line = 0;
    }
    if(board_sts.anti_colli_b&&(!board_sts.anti_colli_f))/////如果前向防撞触发，只能向前运动
    {
    if(speed_data.vel_line<0) speed_data.vel_line = 0;
    }
   /* if(ob_3d_sts.ahead_obstacle==emergency_braking&&ob_3d_sts.rear_obstacle>=warning){
        if(speed_data.vel_line>0) speed_data.vel_line = 0;
    }
    if(ob_3d_sts.rear_obstacle==emergency_braking&&ob_3d_sts.ahead_obstacle>=warning){
        if(speed_data.vel_line<0) speed_data.vel_line = 0;
    }*/

    kine_slove->back_kine(speed_data,move_in);
    logic_out.move_info = move_in;
    std::vector<char> tmp;
    logic_out.to_char_array(&tmp);
    queue->send(CHANNEL_LOGIC_OUT, tmp.data(), tmp.size()); 
    logic_out.f_led_voice = 0;
    logic_out.f_lift = 0;
    logic_out.f_move = 0;
}
void Logic_process::warnning_state_run()/////机器人报警运行
{
    speed_ratio = 0.5;
    if(f_led_voice_recv)
    {
        logic_out.f_led_voice = 1;
        f_led_voice_recv = 0;
    }
    if(f_lift_recv)
    {
        logic_out.f_lift = 1;
        f_lift_recv = 0;
    }
    if(f_move_recv)
    {
        logic_out.f_move = 1;
        f_move_recv = 0;
    }
    logic_out.led_info = led_in;
    logic_out.voice_info = voice_in;
    logic_out.lift_info = lift_in;

    speed_data.vel_line *= speed_ratio;////线速度减半
    speed_data.vel_ang *= speed_ratio;////角速度减半
    if((!device_sts.laser_disconnect)&&(!device_sts.pose_initilized))
    {
        speed_data.vel_line *= 0.0;////如果机器人处于激光雷达匹配阶段，则不能驱动机器人运动，必须等待机器人初始定位完成
        speed_data.vel_ang *= 0.0;////
    }
    if(board_sts.anti_colli_f&&(!board_sts.anti_colli_b))/////如果前向防撞触发,只能向后运动
    {
    if(speed_data.vel_line>0) speed_data.vel_line = 0;
    }
    if(board_sts.anti_colli_b&&(!board_sts.anti_colli_f))/////如果前向防撞触发，只能向前运动
    {
    if(speed_data.vel_line<0) speed_data.vel_line = 0;
    }
    /*if(ob_3d_sts.ahead_obstacle==emergency_braking&&ob_3d_sts.rear_obstacle>=warning){
        if(speed_data.vel_line>0) speed_data.vel_line = 0;
    }
    if(ob_3d_sts.rear_obstacle==emergency_braking&&ob_3d_sts.ahead_obstacle>=warning){
        if(speed_data.vel_line<0) speed_data.vel_line = 0;
    }*/
    kine_slove->back_kine(speed_data,move_in);
    logic_out.move_info = move_in;

    std::vector<char> tmp;
    logic_out.to_char_array(&tmp);
    queue->send(CHANNEL_LOGIC_OUT, tmp.data(), tmp.size()); 
    logic_out.f_led_voice = 0;
    logic_out.f_lift = 0;
    logic_out.f_move = 0;
}
void Logic_process::error_state_run()/////机器人错误运行
{
    speed_ratio = 0.0;
    float slow_ratio = 0.1;
    float mid_ratio = 0.5;
    float min_speed = 0.005;
    if(f_led_voice_recv)
    {
        logic_out.f_led_voice = 1;
        f_led_voice_recv = 0;
    }
    if(f_lift_recv)
    {
        logic_out.f_lift = 1;
        f_lift_recv = 0;
    }
    if(f_move_recv)
    {
        logic_out.f_move = 1;
        f_move_recv = 0;
    }

    if(board_sts.anti_colli_f&&(!board_sts.anti_colli_b))/////如果前向防撞触发,只能向后运动
    {
    if(speed_data.vel_line>0) speed_data.vel_line = 0;
    }
    if(board_sts.anti_colli_b&&(!board_sts.anti_colli_f))/////如果前向防撞触发，只能向前运动
    {
    if(speed_data.vel_line<0) speed_data.vel_line = 0;
    }
    if(ob_3d_sts.ahead_obstacle==emergency_braking&&ob_3d_sts.rear_obstacle>=warning){
        if(speed_data.vel_line>0) speed_data.vel_line = 0;
    }
    if(ob_3d_sts.rear_obstacle==emergency_braking&&ob_3d_sts.ahead_obstacle>=warning){
        if(speed_data.vel_line<0) speed_data.vel_line = 0;
    }
    if(ob_3d_sts.left_obstacle==emergency_braking&&ob_3d_sts.right_obstacle>=normal){
        if(speed_data.vel_ang>0) speed_data.vel_ang = 0;
    }
    if(ob_3d_sts.right_obstacle==emergency_braking&&ob_3d_sts.left_obstacle>=normal){
        if(speed_data.vel_ang<0) speed_data.vel_ang = 0;
    }
    logic_out.led_info = led_in;
    logic_out.voice_info = voice_in;
    logic_out.lift_info = lift_in;
    speed_data.vel_line *= slow_ratio;////
    speed_data.vel_ang *= slow_ratio;////
    if((!device_sts.laser_disconnect)&&(!device_sts.pose_initilized))
    {
        speed_data.vel_line *= 0.0;////如果机器人处于激光雷达匹配阶段，则不能驱动机器人运动，必须等待机器人初始定位完成
        speed_data.vel_ang *= 0.0;////
    }
    if(err_code == ERROR_LASER_OBS||(err_code == ERROR_BACK_OBS_INVOKE)
    ||(err_code == ERROR_LEFT_OBS_INVOKE)||(err_code == ERROR_RIGHT_OBS_INVOKE))/////如果是激光或近红外导致的报错，是可以反向运动的
    {

        if((obs_data.laser_obs_level_p>=EMC_OBS))
        {
            
            if(board_sts.obs_avoid_f&&(!board_sts.obs_avoid_b))
            if(speed_data.vel_line>0.0) speed_data.vel_line = 0.0;

            if(board_sts.obs_avoid_b&&(!board_sts.obs_avoid_f))
            if(speed_data.vel_line<0.0) speed_data.vel_line = 0.0;

            if(board_sts.obs_avoid_l||board_sts.obs_avoid_r) 
            speed_data.vel_ang *= speed_ratio;

            if(board_sts.obs_avoid_l&&board_sts.obs_avoid_r) 
            speed_data.vel_ang *= speed_ratio;

            if(board_sts.obs_avoid_b&&(board_sts.obs_avoid_f))
            speed_data.vel_line *= speed_ratio;

        }
        if(obs_data.laser_obs_level_l>=EMC_OBS)
        {
            if(speed_data.vel_line>0.0) speed_data.vel_line *= speed_ratio;////
            
        }
        
         if((obs_3d_data.laser_obs_level_p>=EMC_OBS))
        {
            
            if(board_sts.obs_avoid_f&&(!board_sts.obs_avoid_b))
            if(speed_data.vel_line>0.0) speed_data.vel_line = 0.0;

            if(board_sts.obs_avoid_b&&(!board_sts.obs_avoid_f))
            if(speed_data.vel_line<0.0) speed_data.vel_line = 0.0;

            if(board_sts.obs_avoid_l||board_sts.obs_avoid_r) 
            speed_data.vel_ang *= speed_ratio;

            if(board_sts.obs_avoid_l&&board_sts.obs_avoid_r) 
            speed_data.vel_ang *= speed_ratio;

            if(board_sts.obs_avoid_b&&(board_sts.obs_avoid_f))
            speed_data.vel_line *= speed_ratio;
        }
        if(obs_3d_data.laser_obs_level>=EMC_OBS)
        {
		
            //if(speed_data.vel_line>0.0) speed_data.vel_line *= speed_ratio;////
	    if(ob_3d_sts.ahead_obstacle==emergency_braking&&ob_3d_sts.rear_obstacle>=warning)
            if(speed_data.vel_line>0.0) speed_data.vel_line = 0.0;

            if(ob_3d_sts.rear_obstacle==emergency_braking&&ob_3d_sts.ahead_obstacle>=warning)
            if(speed_data.vel_line<0.0) speed_data.vel_line = 0.0;

            if(ob_3d_sts.left_obstacle==emergency_braking&&ob_3d_sts.right_obstacle>=normal) 
	    if(speed_data.vel_ang>0) speed_data.vel_ang *= speed_ratio;
            
	    if(ob_3d_sts.right_obstacle==emergency_braking&&ob_3d_sts.left_obstacle>=normal) 
	    if(speed_data.vel_ang<0) speed_data.vel_ang *= speed_ratio;
            
            if(ob_3d_sts.right_obstacle==emergency_braking&&ob_3d_sts.left_obstacle==emergency_braking){
                speed_data.vel_line *= speed_ratio;
            }
            if(ob_3d_sts.ahead_obstacle==emergency_braking&&ob_3d_sts.rear_obstacle==emergency_braking){
                speed_data.vel_line *= speed_ratio;
            }

            
        }
    /*    if(board_sts.obs_avoid_f&&(!board_sts.obs_avoid_b)) 
            if(speed_data.vel_line>0) speed_data.vel_line *= speed_ratio;
        if(board_sts.obs_avoid_b&&(!board_sts.obs_avoid_f)) 
            if(speed_data.vel_line<0) speed_data.vel_line *= speed_ratio;  
        if(board_sts.obs_avoid_l||board_sts.obs_avoid_r) 
            speed_data.vel_ang *= speed_ratio;*//// 
        /*if((!board_sts.obs_avoid_f)&&(!board_sts.obs_avoid_b)&&(!board_sts.obs_avoid_l)&&(!board_sts.obs_avoid_r))
                //if(speed_data.vel_line>min_speed) speed_data.vel_line = min_speed;
                if(speed_data.vel_line>0.0) speed_data.vel_line = 0.0;

        if(board_sts.obs_avoid_f&&(!board_sts.obs_avoid_b)){
            //if(speed_data.vel_line>0) speed_data.vel_line *= slow_ratio;
            //if(speed_data.vel_line>min_speed) speed_data.vel_line = min_speed;
            if(speed_data.vel_line>0.0) speed_data.vel_line = 0.0;
        }
        if(board_sts.obs_avoid_b&&(!board_sts.obs_avoid_f)){
            //if(speed_data.vel_line<0) speed_data.vel_line *= slow_ratio;  
            //if(speed_data.vel_line<-min_speed) speed_data.vel_line = -min_speed;
            if(speed_data.vel_line<0.0) speed_data.vel_line = 0.0;
        }

        ///speed_data.vel_line *= slow_ratio;////
        //if(speed_data.vel_line>0.01) speed_data.vel_line = 0.01;
        //if(speed_data.vel_line<-0.01) speed_data.vel_line = -0.01;
        if(board_sts.obs_avoid_l||board_sts.obs_avoid_r) 
        {
        //speed_data.vel_ang *= slow_ratio;////
        if(speed_data.vel_ang>min_speed) speed_data.vel_ang = min_speed;
        if(speed_data.vel_ang<-min_speed) speed_data.vel_ang = -min_speed;
        speed_data.vel_ang *= speed_ratio;
        }
        if(board_sts.obs_avoid_l&&board_sts.obs_avoid_r) 
        //if(speed_data.vel_ang>min_speed) speed_data.vel_ang *= speed_ratio;
        speed_data.vel_ang *= speed_ratio;

        if(board_sts.obs_avoid_b&&(board_sts.obs_avoid_f))
        //if(speed_data.vel_line<-min_speed) speed_data.vel_line *= speed_ratio;
        speed_data.vel_line *= speed_ratio;
        */

    }
    else
    {
        //if(speed_data.vel_line>min_speed) speed_data.vel_line = min_speed;
        //if(speed_data.vel_line<-min_speed) speed_data.vel_line = -min_speed;
        
        speed_data.vel_line *= speed_ratio;////
        speed_data.vel_ang *= speed_ratio;////
    }
    logic_out.f_lift = 0;////停止顶升的一切动作，避免发生危险
    kine_slove->back_kine(speed_data,move_in);
    logic_out.move_info = move_in;
    std::vector<char> tmp;
    logic_out.to_char_array(&tmp);
    queue->send(CHANNEL_LOGIC_OUT, tmp.data(), tmp.size());     
    logic_out.f_led_voice = 0;
    logic_out.f_lift = 0;
    logic_out.f_move = 0;
}
void Logic_process::warning_code_dispose()
{
    voice_in.index = 0;
    switch (warn_code) {
        //case WARNING_LASER_OBS_INVOKE: 
        //    voice_in.index = VOICE_INSIDEOBAV;
        //    break;

        //case WARNING_FB_OBS_INVOKE:

        case WARNING_LASER_OBS_INVOKE: 
            voice_in.index = VOICE_INSIDEOBAV;
            break;
        case WARNING_FB_OBS_INVOKE:
            voice_in.index = ERROR_BACK_OBS_INVOKE;//后方报警
            break;
        case WARNING_TWO_DIM_DISCONNECT: 
            voice_in.index = WARNING_TWO_DIM_DISCONNECT;
            break;
        case WARNING_BOARD_DISCONNECT: 
            voice_in.index = WARNING_BOARD_DISCONNECT;
            break;
        case WARNING_LASER_DISCONNECT: 
            voice_in.index = WARNING_LASER_DISCONNECT;
            break;
        case WARNING_IMU_DISCONNECT: 
            voice_in.index = WARNING_IMU_DISCONNECT;
            break;
        case WARNING_LOWPOWER: 
            voice_in.index = VOICE_CHARGENOTICE;
            break; 
        default:
            break;
    }
}

void Logic_process::error_code_dispose()
{
    /*voice_in.volume = 28;
    if(err_code!=voice_in.index&&err_code>0)
    {voice_in.index = VOICE_ANTICRUSH;
    voice_in.run_state = 1;
    }else voice_in.run_state = 0;*/
    switch (err_code) {
        //case WARNING_LASER_OBS_INVOKE: 
        //    voice_in.index = VOICE_INSIDEOBAV;
        //    break;
        case ERROR_EMERGE: 
            voice_in.index = VOICE_EMERGE;
            break;
        case ERROR_LASER_OBS: 
            voice_in.index = VOICE_INSIDEOBAV;
            break;
        case ERROR_BACK_OBS_INVOKE: 
            voice_in.index = ERROR_BACK_OBS_INVOKE;
            break;
        case ERROR_LEFT_OBS_INVOKE: 
            voice_in.index = ERROR_LEFT_OBS_INVOKE;
            break;
        case ERROR_RIGHT_OBS_INVOKE: 
            voice_in.index = ERROR_RIGHT_OBS_INVOKE;
            break;
        case ERROR_ANTICRUSH:
            voice_in.index = VOICE_ANTICRUSH;
            break;
        case ERROR_MOTORERROR: 
            voice_in.index = VOICE_MOTORERROR;
            break;
        case ERROR_PAUSE:
            voice_in.index = VOICE_EMERGE;
            break;
        default:
            break;
    }
    
    //voice_in.run_state = 1;
    //if(voice_in.index>0)voice_in.run_state = 1;
    //else voice_in.run_state = 0;

    if(err_code>0)
    led_in.led_color = LED_RED;
    else if(get_global_agv_instance()->get_system_state() == SYSTEM_STATE_FREE&&device_sts.pose_initilized)
    led_in.led_color = LED_WHITE;
    else if(get_global_agv_instance()->get_system_state() == SYSTEM_STATE_MAPPING)
    led_in.led_color = LED_WHITE;
    else if(warn_code == WARNING_LASER_OBS_INVOKE||warn_code == WARNING_FB_OBS_INVOKE)
    led_in.led_color = LED_PURPLE; 
    else if(get_global_agv_instance()->get_system_state() == SYSTEM_STATE_NAVIGATING)
    led_in.led_color = LED_GREEN;  
 
    if((last_led!=led_in.led_color)||(last_voice!=voice_in.index))
    {
       last_led =  led_in.led_color;
       last_voice = voice_in.index;
       f_led_voice_recv = 1;
    }


}