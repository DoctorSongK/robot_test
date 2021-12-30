#include <map>
#include <iostream>
#include <cmath>
#include <unistd.h>

#include "../common/data_transform.h"
#include "../common/configuration.h"
#include "walking_machine.h"
#include "agv.h"

Model_kine::Model_kine()
{
	robot_control_type = get_configs()->get_int("odom", "robot_control_model", nullptr);
    if(robot_control_type<ROBOT_DIFF_MODEL_CARRY) robot_control_type = ROBOT_DIFF_MODEL_CARRY;////默认采用差分模型
    printf("robot_control_type=%d\n",robot_control_type);
    distance=get_configs()->get_float("odom", "wheel_distance", nullptr);
    diameter=get_configs()->get_float("odom", "wheel_diameter", nullptr);

    len_f=get_configs()->get_float("odom", "axis_distance_f", nullptr);
    len_b=get_configs()->get_float("odom", "axis_distance_b", nullptr);
    encoder_per_round=get_configs()->get_float("odom", "encoder_per_round", nullptr);
    encoder_per_round_r=get_configs()->get_float("odom", "encoder_per_round_r", nullptr);

    _half_d = distance*0.5;/* 輪子到中心距離m   这里要加入文件读取函数 */
    _radius = diameter*0.5;
	
    len_f = (len_f);
    len_b = (len_b);
    float axis_distance = fabs(len_f)+fabs(len_b);
    zero_turn_angle = atan2(axis_distance,fabs(distance));
    alpha_a = get_configs()->get_float("odom", "alpha_a", nullptr);
    alpha_b = get_configs()->get_float("odom", "alpha_b", nullptr); 
    printf("the robot model _half_d=%f,_radius=%f,zero_turn_angle=%f\n",_half_d,_radius,zero_turn_angle);
    printf("encoder_per_round=%f,encoder_per_round_r=%f,\n",encoder_per_round,encoder_per_round_r);
    sin_a = sin(alpha_a);cos_a = cos(alpha_a);
    sin_b = sin(alpha_b);cos_b = cos(alpha_b);

};

Model_kine::~Model_kine() 
{
    
};
int  Model_kine::back_kine(Speed_data sp_data,st_motion &move_out)
{
    if(ROBOT_DUAL_STEER_MODEL==robot_control_type)
    back_kine_dual_steer( sp_data,move_out);
    else if(ROBOT_DIFF_MODEL_CARRY==robot_control_type||((robot_control_type == ROBOT_DIFF_MODEL_COMMON))||((robot_control_type == ROBOT_DIFF_MODEL_COMMON_YB)))
    back_kine_diff(sp_data,move_out);
}
int  Model_kine::back_kine_dual_steer(Speed_data sp_data,st_motion &move_out)
{/////双舵轮模型求解
    double vrot, vline;
    vrot = sp_data.vel_ang;
    vline = sp_data.vel_line;
    Position realpose_map = get_global_agv_instance()->expolate_current_position();
    float lenf,lenr,deltaf,deltar,len_fb,beta,zero_theta,velx,vely;
    double v1ctheta,v1stheta,v2ctheta,v2stheta,ang_theta1,ang_theta2,v1,v2;
    switch(sp_data.run_state)
    {
    case NORMAL_RUN:
    lenf = len_f;////前悬长度
    lenr = len_b;////后悬长度
    deltaf = realpose_map.rot_rf1;////前轮摆角	
    deltar = realpose_map.rot_rb1;////后轮摆角
	len_fb = (lenf+lenr)+0.000000001;
    beta = atan((lenf*tan(deltar)+lenr*tan(deltaf))/len_fb);
    zero_theta = zero_turn_angle/2;
	////////////////计算前后轮偏摆角度
	/*double vline_md = 2*vline_md*cos(beta)+0.000000001;
    float pbangle = atan((vrot*(lenf+lenr))/(vline_md));

	if(isnanl(pbangle)) return -1;
	//printf("inversebeta=%f pbangle=%f\n",beta,pbangle);
    if (pbangle>zero_theta) pbangle = zero_theta;
    if (pbangle<-zero_theta) pbangle = -zero_theta;*/
    ////////////////在计算两轮速度分量是，采用简化的模型，默认deltaf=-deltar；
    velx = vline*cos(beta);
    vely = vline*sin(beta);
    v1ctheta = velx-vrot*lenr*sin_a;
    v1stheta = vely-vrot*lenr*cos_a;

    v2ctheta = velx+vrot*lenf*sin_b;
    v2stheta = vely+vrot*lenf*cos_b;
    ////////////////计算前后轮偏摆角度
    ang_theta1=atan(v1stheta/v1ctheta);
    ang_theta2=atan(v2stheta/v2ctheta);
    v1 = sqrt(v1ctheta*v1ctheta+v1stheta*v1stheta);    
    v2 = sqrt(v2ctheta*v2ctheta+v2stheta*v2stheta);   
    if(vline<0)
    {v1=-v1;v2=-v2;}

    move_out.vel_f_steer = v2;
    move_out.vel_b_steer = v1;
    move_out.rot_f_steer = ang_theta1;
    move_out.rot_b_steer = ang_theta2;
    break;
    case STRAIGHT:
    move_out.vel_f_steer = 0.0;
    move_out.vel_b_steer = 0.0;
    move_out.rot_f_steer = 0.0;
    move_out.rot_b_steer = 0.0;
    break;
    case HOME_TURN:
    move_out.vel_f_steer = vrot*(len_f+len_b)/2;
    move_out.vel_b_steer = -move_out.vel_f_steer;
    move_out.rot_f_steer = zero_turn_angle;
    move_out.rot_b_steer = zero_turn_angle;
    break;
    default:
    return -1;
    break;
    }


    return 1;
}

int  Model_kine::back_kine_diff(Speed_data sp_data,st_motion &move_out)
{/////差分模型求解
    double vrot, vline;
    vrot = sp_data.vel_ang;
    vline = sp_data.vel_line;

	float vl, vr, l, r;

    l = _half_d; /* 輪子到中心距離m   这里要加入文件读取函数 */
    r = _radius;

    vl = (vline - vrot * l)/r;
    vr = (vline + vrot * l)/r;

    move_out.vel_l = vl;
    move_out.vel_r = vr;
    //printf("invers ine vl=%f,vr=%f\n",vl,vr);
}

int Model_kine::forward_kine(Position *buf, ImuData2D *imubuf,long delta_t, long delta_fl1, long _fr1,long delta_bl1, long _br1, double delta_theta,double delta_vel) 
{

   
    if(delta_t <= 0) delta_t = 20000;

    double PI = 3.14159265358979;
    if(fabs(delta_fl1)>encoder_per_round||fabs(delta_bl1)>encoder_per_round||
    fabs(_fr1)>encoder_per_round_r||fabs(_br1)>encoder_per_round_r)
    {
    printf("cur delta exceed the total round\n");
    delta_fl1 = 0.0;_fr1 = 0.0;
    delta_bl1 = 0.0;_br1 = 0.0;
    return -1;
    }
    else
    {
    delta_fl1 = delta_fl1;_fr1 = _fr1;
    delta_bl1 = delta_bl1;_br1 = _br1;
    }
    delta_theta = - delta_theta;

    double fvel1=delta_fl1*2*PI/(encoder_per_round*delta_t)*1000000*diameter/2.;///// change to line velocity of the left wheel m/s
    double bvel1=delta_bl1*2*PI/(encoder_per_round*delta_t)*1000000*diameter/2.;
    double theta_f1 = _fr1/encoder_per_round_r*2*PI;
    double theta_b1 = _br1/encoder_per_round_r*2*PI;
    double cf1,sf1,cb1,sb1;
    cf1 = cos(theta_f1);sf1 = sin(theta_f1);
    cb1 = cos(theta_b1);sb1 = sin(theta_b1);
    Matrix<double, 4, 3> matrix_43;
    matrix_43<<1.,0.,-len_b*sin_a,0,1,-len_b*cos_a,1,0,len_f*sin_b,0,1,len_f*cos_b;
    //printf("len_b=%f,len_f=%f,sin_a=%f,sin_b=%f,cos_a=%f,cos_b=%f\n",len_b,len_f,sin_a,sin_b,cos_a,cos_b);
    Matrix<double, 4, 1> matrix_41;
    matrix_41<<bvel1*cb1,bvel1*sb1,fvel1*cf1,fvel1*sf1;
    //printf("cf1=%f,sf1=%f,cb1=%f,sb1=%f\n",cf1,sf1,cb1,sb1);
    //Matrix<double, 3, 4> matrix_34 = matrix_43.transpose();
    Matrix<double, 3, 3> matrix_33 = matrix_43.transpose()*matrix_43;
    Matrix<double, 3, 4> matrix_btbnbt = matrix_33.inverse()*matrix_43.transpose();
    Matrix<double, 3, 1> result= matrix_btbnbt*matrix_41;
    //cout<<"the result"<<endl<<result<<endl;
	//车体转过的位姿
    double vel_x = result(0);
    double vel_y = result(1);
    double  WOdom= result(2);
    
    bool is_static = true;
    if(fabs(fvel1)>0.007||fabs(bvel1)>0.007)
    {buf->delt_t = 20000;/////如果机器人轮速度都大于0.007，则认为机器人在动，否则认为机器人静止
    is_static = false;
    }
    else
    {
    buf->delt_t = 0;
    is_static = true;
    imu_vel_sum = 0;
    }
    

    double delta_x;//=vel_x*delta_t*0.000001;
    double delta_y;//=vel_y*delta_t*0.000001;
    double vel_xy = sqrt(vel_x*vel_x+vel_y*vel_y);
    if (vel_x<0) vel_xy = -vel_xy;
    delta_x=vel_xy*cos(delta_theta);
    delta_y=vel_xy*sin(delta_theta);
    double del_tt = (delta_t*0.000001);

    WOdom = delta_theta / del_tt;
    double acc_x = (delta_x-last_vel_x)/del_tt;
    double acc_y = (delta_y-last_vel_y)/del_tt;
    last_vel_x = delta_x;
    last_vel_y = delta_y;

    imubuf->linear_acceleration_x = acc_x;
    imubuf->linear_acceleration_y = acc_y;
    imubuf->angular_velocity_z = WOdom;

    delta_x*=del_tt;
    delta_y*=del_tt;



    if(!is_static)
    {
        imu_vel_sum+=delta_vel;
        //delta_x=imu_vel_sum*delta_t*cos(delta_theta)*0.000001;
        //delta_y=imu_vel_sum*delta_t*sin(delta_theta)*0.000001;
    }
    if(isnanl(delta_x)||isnanl(delta_y)) 
    {delta_x =0.0;delta_y=0.0;printf("the number is not a number\n");}
	//if(!is_static)
	//printf("vel_x=%f,vel_y=%f,WOdom=%f,fv1=%f,bv1=%f,thetaf1=%f,thetab1=%f\n",vel_x,vel_y,WOdom,fvel1,bvel1,theta_f1,theta_b1);
    buf->x+=delta_x;
    buf->y+=delta_y;
    buf->theta += delta_theta;

    buf->rot_rf1 = theta_f1;
    buf->rot_rb1 = theta_b1;
    return 0;
}

//180
//600

int Model_kine::forward_kine(Position *buf, ImuData2D *imubuf,long delta_t, long delta_left, long delta_right, double delta_theta,double delta_vel) {

    //printf("delta_t=%d,delta_left=%d,delta_right=%d,delta_theta=%f\n",delta_t,delta_left,delta_right,delta_theta);
    if(delta_t <= 0) delta_t = 20000;

    double PI = 3.14159265358979;
    if(fabs(delta_left)>encoder_per_round||fabs(delta_right)>encoder_per_round)
    {
    printf("cur delta exceed the total round\n");
    delta_left = 0.0;
    delta_right = 0.0;
    }
    else
    {
    delta_left = delta_left;
    delta_right = delta_right;
    }
    delta_theta = - delta_theta;
    double W_left=delta_left*2*PI/(encoder_per_round*delta_t)*1000000;///// chang to line velocity of the left wheel m/s
    double W_right=delta_right*2*PI/(encoder_per_round*delta_t)*1000000;
    double V_left=W_left*diameter/2.;
    double V_right=W_right*diameter/2.;
    double VOdom=(V_left+V_right)/2.;
	
    double WOdom=(V_right-V_left)/distance;

    double delta_x=0;
    double delta_y=0;

    bool is_static = true;
    //printf("VOdom=%f,WOdom=%f,encoder_per_round=%f,diameter=%f\n",VOdom,WOdom,encoder_per_round,diameter);
    if(fabs(V_left)>0.007||fabs(V_right)>0.007)
    {buf->delt_t = 20000;/////如果机器人轮速度都大于0.007，则认为机器人在动，否则认为机器人静止
    is_static = false;
    }
    else
    {
    buf->delt_t = 0;
    is_static = true;
    imu_vel_sum = 0;
    }
    //imubuf->angular_velocity_z = -delta_theta / delta_t*1000000;
    imubuf->angular_velocity_z = -WOdom;

    if(buf->delt_t == 0)////如果里程计运动缓慢，则取里程计的角速度数据，否则取陀螺仪的角速度数据
    delta_theta =  (WOdom*delta_t)*0.000001;
    else
    WOdom = delta_theta / delta_t*1000000;

    //delta_x=VOdom*delta_t*cos(delta_theta)*0.000001;
    //delta_y=VOdom*delta_t*sin(delta_theta)*0.000001;
    delta_x=VOdom*cos(delta_theta);
    delta_y=VOdom*sin(delta_theta);
    double del_tt = (delta_t*0.000001);
    double acc_x = (delta_x-last_vel_x)/del_tt;
    double acc_y = (delta_y-last_vel_y)/del_tt;
    last_vel_x = delta_x;
    //last_vel_x = VOdom;
    last_vel_y = delta_y;
    
    imubuf->linear_acceleration_x = -acc_x;//acc_x;
    imubuf->linear_acceleration_y = -acc_y;//acc_y;
    

    delta_x*=del_tt;
    delta_y*=del_tt;

    if(!is_static)
    {
        imu_vel_sum+=delta_vel;
        //delta_x=imu_vel_sum*delta_t*cos(delta_theta)*0.000001;
        //delta_y=imu_vel_sum*delta_t*sin(delta_theta)*0.000001;
    }

    if(isnanl(delta_x)||isnanl(delta_y)) 
    {delta_x =0.0;delta_y=0.0;printf("the number is not a number\n");}
    //if(fabs(V_left)>0.007||fabs(V_right)>0.007)
    //printf("V_right=%f,V_right=%f,delt_t=%d,delta_theta=%f\n",V_left,V_right,delta_t,delta_theta*1000000);
    buf->x+=delta_x;
    buf->y+=delta_y;
    buf->theta += delta_theta;
	
    return 0;
}