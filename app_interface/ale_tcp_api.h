#ifndef ALE_COMMU_H
#define ALE_COMMU_H

#include <vector>

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include <math.h>
#include <sys/ioctl.h> 
#include <signal.h>
#include "myLog.h"
#include "../common/thpool.h"
#include "../common/configuration.h"
#include "../logic/agv.h"
#include "../logic/storage.h"
#include "../common/capture_point.h"

extern int local_port = 12345;
extern int Port_task = 19005;
using namespace std;

int byte2int(char *buf)
{
    int value = 0;
    for (int i = 0; i < 4; i++)
    {
        value |= ((int)(buf[i] & 0xff)) << (8 * i);
    }
    return value;
}
void int2byte(char *buf, int value)
{
    for (int i = 0; i < 4; i++)
    {
        buf[i] = (char)((value >> 8 * i) & 0xff);
    }
}

void do_sha256(char *buf, int size, char *output)
{
    return;
}

#define HEAD_SIZE 24
class TCPDataPackage
{
public:
    TCPDataPackage(int func) : function(func){};
    TCPDataPackage(int func, int arg1, int arg2, int data_len) : function(func), arg1(arg1), arg2(arg2), data_size(data_len)
    {
        if (data_size > 0)
        {
            data_buf = (char *)malloc(data_size);
        }
    };
    TCPDataPackage(char *head, int *check_result)
    {
        function = head[4] & 0xff;
        arg1 = byte2int(head + 8);
        arg2 = byte2int(head + 12);
        data_size = byte2int(head + 16);
        //@Todo
        //check sum
        *check_result = 0;

        if (data_size > 0)
        {
            data_buf = (char *)malloc(data_size);
        }
    }
    ~TCPDataPackage()
    {
        if (data_buf)
        {
            free(data_buf);
        }
    }
    int function;
    int arg1 = 0;
    int arg2 = 0;
    int data_size = 0;
    char *data_buf = nullptr;

    void reserve_buf(int size)
    {
        if (size < 0)
        {
            return;
        }
        data_size = size;
        if (data_buf)
        {
            delete data_buf;
            data_buf = nullptr;
        }
        if (data_size > 0)
        {
            data_buf = (char *)malloc(data_size);
        }
    }

    void to_bytes(char *buf)
    {
        int timestamp = (int)(get_current_time_us() / 1000000);
        int2byte(buf, timestamp);
        buf[4] = (char)function;
        int2byte(buf + 8, arg1);
        int2byte(buf + 12, arg2);
        int2byte(buf + 16, data_size);
        //@Todo
        //check sum
    }
};
class TCPDataPackage2
{
public:
    TCPDataPackage2(int func) : function(func){};
    TCPDataPackage2(int func, int arg1, int arg2, int data_len) : function(func), arg1(arg1), arg2(arg2), data_size(data_len)
    {
        if (data_size > 0)
        {
            data_buf = (char *)malloc(data_size);
        }
    };
    TCPDataPackage2(char *head, int *check_result)
    {
        function = head[4] & 0xff;
        arg1 = byte2int(head + 8);
        arg2 = byte2int(head + 12);
        data_size = byte2int(head + 16);
        //@Todo
        //check sum
        *check_result = 0;

        if (data_size > 0)
        {
            data_buf = (char *)malloc(data_size);
        }
    }
    ~TCPDataPackage2()
    {
        if (data_buf)
        {
            free(data_buf);
        }
    }
    int function;
    int arg1 = 0;
    int arg2 = 0;
    int data_size = 0;
    char *data_buf = nullptr;
	int byte2int(char *buf)
	{
		int value = 0;
		for (int i = 0; i < 4; i++)
		{
			value |= ((int)(buf[i] & 0xff)) << (8 * i);
		}
		return value;
	}
	void int2byte(char *buf, int value)
	{
		for (int i = 0; i < 4; i++)
		{
			buf[i] = (char)((value >> 8 * i) & 0xff);
		}
	}
    void reserve_buf(int size)
    {
        if (size < 0)
        {
            return;
        }
        data_size = size;
        if (data_buf)
        {
            delete data_buf;
            data_buf = nullptr;
        }
        if (data_size > 0)
        {
            data_buf = (char *)malloc(data_size);
        }
    }

    void to_bytes(char *buf)
    {
        int timestamp = (int)(get_current_time_us() / 1000000);
//        int2byte(buf, timestamp);
		buf[0]=0xa0;
		buf[1]=0xb0;
		buf[2]=0x00;
		buf[3]=0x00;
        buf[4] = (char)function;
        int2byte(buf + 8, arg1);
        int2byte(buf + 12, arg2);
        int2byte(buf + 16, data_size);
		buf[20]=0xff;
		buf[21]=0xff;
		buf[22]=0xff;
		buf[23]=0xff;
		
        //@Todo
        //check sum
    }
};
void handle_package(TCPDataPackage2 *input, TCPDataPackage2 *output)
{
    int func = input->function;
    if(func != 2) {
        //printf("Recieve function %d with %d bytes data\n", func, input->data_size);
    }
    if (func == 0)//停止
    {
        MoveInstruction move;
        move.type = MOVE_INSTRUCTION_TYPE_SPEED;
        move.speed.speed = 0;
        move.speed.turn_left = 0;
        std::vector<char> data;
        move.to_char_array(&data);
        get_global_agv_instance()->get_message_queue()->send(CHANNEL_MOVE, data.data(), data.size());
    }
    else if (func == 1)//运动
    {
        MoveInstruction move;
        move.type = MOVE_INSTRUCTION_TYPE_SPEED;
        move.speed.speed = input->arg1;
        move.speed.turn_left = input->arg2;
        std::vector<char> data;
        move.to_char_array(&data);
        //printf("move %d, %d\n", input->arg1, input->arg2);
        get_global_agv_instance()->get_message_queue()->send(CHANNEL_MOVE, data.data(), data.size());
    }
    else if (func == 47)//运动
    {
        MoveInstruction move;
        move.type = MOVE_INSTRUCTION_TYPE_MOTION_MODE;
        move.speed.speed = input->arg1;
        move.speed.turn_left = input->arg2;
        std::vector<char> data;
        move.to_char_array(&data);
        printf("recv cmd=47 move %d, %d\n", input->arg1, input->arg2);
        get_global_agv_instance()->get_message_queue()->send(CHANNEL_MOVE, data.data(), data.size());
    }
    else if (func == 2)//获取状态
    {
        Position pose = get_global_agv_instance()->expolate_current_position();
        double score = 1;
        output->arg1 = get_global_agv_instance()->get_system_state();
        output->arg2 = get_global_agv_instance()->collision_level;
        output->reserve_buf(sizeof(double) * 3 + sizeof(int)*10);
//		printf("###agv_state=%d\n",get_global_agv_instance()->get_system_state());
		
		char *pc=output->data_buf;
        double *pd = (double *)pc;
        pd[0] = pose.x;
        pd[1] = pose.y;
        pd[2] = pose.theta;
		
		pc+=24;
		int * pi=(int *)pc;

        pi[0]=	get_global_agv_instance()->exec_cap;
        pi[1]=	get_global_agv_instance()->warning_code;
        pi[2]=	get_global_agv_instance()->error_code;
		if(!get_global_agv_instance()->lift_complete)
		pi[3]=	0;
		else
		{
        if(get_global_agv_instance()->get_system_state()==SYSTEM_STATE_NAVIGATING)
        pi[3]=	0;
        else if (get_global_agv_instance()->get_system_state()==SYSTEM_STATE_FREE)
        pi[3]=	1;
		}
        if((get_global_agv_instance()->exec_route&0x10000)==0)
        pi[4] = 0;
        else 
        pi[4] = get_global_agv_instance()->exec_route;
        pi[5] = int(get_global_agv_instance()->locate_modual->today_odom);
        pi[6] = int(get_global_agv_instance()->locate_modual->total_odom);
        pi[7] = get_global_agv_instance()->board_sts._battary;
        pi[8] = get_global_agv_instance()->logic_out_process->speed_data.vel_line*1000;
        pi[9] = get_global_agv_instance()->logic_out_process->speed_data.vel_ang/M_PI*180;
        //pi[8] = get_global_agv_instance()->mcu_speed;
        //pi[9] = get_global_agv_instance()->mcu_omega;		
//		printf("mcu_battery_level=%d,mcu_speed=%d,mcu_omega=%d\n",pi[5],pi[6],pi[7]);
    }
    else if (func == 3)//获取地图
    {

        MapSharedPointer p_map;
        p_map = get_global_agv_instance()->get_current_map();
        if(! p_map.is_nullptr()) {
            std::vector<char> map_data;
            p_map->to_char_array(&map_data);
            int size = map_data.size();
            char *p = map_data.data();
            output->reserve_buf(size);
            memcpy(output->data_buf, p, size);
            printf("###lwg:rcv msg 3,获取地图size=%d\n",size);
        }
    }
    else if (func == 5)//获取地图,未压缩
    {
        MapSharedPointer p_map;
        p_map = get_global_agv_instance()->get_current_map();
		/*
		SimpleGridMap *gridmap;
        if(! p_map.is_nullptr()) {
			gridmap = p_map.get();
			std::vector<char> datas=gridmap->datas;
			
            int size = datas.size();
            char *p = datas.data();
            output->reserve_buf(size);
            memcpy(output->data_buf, p, size);
        }*/
		if(! p_map.is_nullptr()) {
            std::vector<char> map_data;
            p_map->to_uncompress(&map_data);
            int size = map_data.size();
            char *p = map_data.data();
            output->reserve_buf(size);
            memcpy(output->data_buf, p, size);
			printf("###lwg:rcv msg 5,获取地图size=%d\n",size);
        }
		
    }
    else if (func == 129)//获取导航状态
    {
	    bool first_cal = true;
        float neareast_dis = 0.0;
        get_global_agv_instance()->exec_cap=0xfffe;
        Position pose = get_global_agv_instance()->expolate_current_position();
        if((get_global_agv_instance()->exec_route&0x10000)==0){//没在跑路径,匹配全部点
        //printf("没在跑路径\n");
        for(CapPoint &point : get_global_agv_instance()->caps) {
                double dis=pow(point.x-pose.x,2)+pow(point.y-pose.y,2);
                if(first_cal)
                {
                    neareast_dis = dis;
                    first_cal = false;
                    get_global_agv_instance()->exec_cap=point.id;
                }
                if (dis<neareast_dis){
					neareast_dis = dis;
                    get_global_agv_instance()->exec_cap=point.id;
                }
            }
        }else{
        //printf("正在跑路径\n");
        for(CapRoute &route : get_global_agv_instance()->routes) {
        if(route.id==(get_global_agv_instance()->exec_route&0xffff)){
        for(CapPoint &point : route.cap_points) {
        double dis=pow(point.x-pose.x,2)+pow(point.y-pose.y,2);
        if (dis<pow(0.3,2)){
        printf(" cap id=%d,x=%lf,y=%lf,dir=%d\n",point.id,point.x,point.y,point.dir);
        get_global_agv_instance()->exec_cap=point.id;
        break;
        }
        }
        }
        }
        }
		output->arg1 = get_global_agv_instance()->get_system_state();
		output->arg2 = get_global_agv_instance()->collision_level;
        //output->arg1 = get_global_agv_instance()->exec_route;
        /*if(get_global_agv_instance()->get_system_state()==SYSTEM_STATE_NAVIGATING)
        output->arg1|=(1<<16);
        else if (get_global_agv_instance()->get_system_state()==SYSTEM_STATE_FREE)
        output->arg1&=~(1<<16);*/

        //output->arg2 = get_global_agv_instance()->exec_cap;
		/*if(get_global_agv_instance()->lift_complete)
        output->arg2 |= 0x10000;
		else
		output->arg2 &= 0x0ffff;*/

        output->reserve_buf(sizeof(double)*3+ sizeof(int)*10);

        char *pc=output->data_buf;
        double *pd = (double *)pc;
        pd[0] = pose.x;
        pd[1] = pose.y;
        pd[2] = pose.theta;
		
        pc+=24;
        int * pi=(int *)pc;

        pi[0]=	get_global_agv_instance()->exec_cap;
        pi[1]=	get_global_agv_instance()->warning_code;
        pi[2]=	get_global_agv_instance()->error_code;
		if(!get_global_agv_instance()->lift_complete)
		pi[3]=	0;
		else
		{
        if(get_global_agv_instance()->get_system_state()==SYSTEM_STATE_NAVIGATING)
        pi[3]=	0;
        else if (get_global_agv_instance()->get_system_state()==SYSTEM_STATE_FREE)
        pi[3]=	1;
		}
        if((get_global_agv_instance()->exec_route&0x10000)==0)
        pi[4] = 0;
        else 
        pi[4] = get_global_agv_instance()->exec_route;
        pi[5] = int(get_global_agv_instance()->locate_modual->today_odom);
        pi[6] = int(get_global_agv_instance()->locate_modual->total_odom);
        pi[7] = get_global_agv_instance()->board_sts._battary;

        pi[8] = get_global_agv_instance()->logic_out_process->speed_data.vel_line*1000;
        pi[9] = get_global_agv_instance()->logic_out_process->speed_data.vel_ang/M_PI*180;
        //pi[8] = get_global_agv_instance()->mcu_speed;
        //pi[9] = get_global_agv_instance()->mcu_omega;


	
		printf("###clm:rcv msg 129,查询任务,route=%d,cap=%d\n",output->arg1,output->arg2);
    }
    else if (func == 134)//获取cap
    {
		get_log_instance()->task_msg("###clm:rcv msg 134,获取cap\n");
		output->arg1=get_global_agv_instance()->caps.size();
		output->reserve_buf(get_global_agv_instance()->caps.size()*28);
		char* pc=output->data_buf;
		
		std::list<CapPoint>::iterator itc;
		for(itc=get_global_agv_instance()->caps.begin();itc!=get_global_agv_instance()->caps.end();itc++){
			CapPoint act = *itc;
			printf("##x=%lf,y=%lf,id=%d\n",act.x,act.y,act.id);
			double *pd = (double *)pc;
			pd[0] = act.x;
			pd[1] = act.y;
			pd[2] = act.theta;
			pc+=24;
			int *pi=(int *)pc;
			pi[0]=act.id;
			pc+=4;
//			printf("@@x=%lf,y=%lf,id=%d\n",pd[0],pd[1],pi[0]);
		}
    }
    else if (func == 137)//下发cap点
    {
        get_log_instance()->task_msg("###lwg:rcv msg 137,下发cap\n");
        int cap_size = input->arg1;
        char* pc=input->data_buf;
        get_global_agv_instance()->caps.clear();

        for(int i=0;i<cap_size;i++){
            double *pd = (double *)pc;
            pc+=24;
            int *pi=(int *)pc;
            pc+=4;
            CapPoint cap(pi[0],	pd[0],pd[1],pd[2]);
            printf("##x=%lf,y=%lf,theta=%f,id=%d\n",cap.x,cap.y,cap.theta,cap.id);
            get_global_agv_instance()->caps.push_back(cap);
        }
		get_global_storage()->save_caps(get_global_agv_instance()->caps);

    }
	else if (func == 140)//获取路径号
    {
		get_log_instance()->task_msg("###clm:rcv msg 140,获取路径号\n");
		output->arg1=get_global_agv_instance()->routes.size();
		output->reserve_buf(get_global_agv_instance()->routes.size()*4);
		int* pi=(int *)output->data_buf;
		for(CapRoute &route : get_global_agv_instance()->routes) {
			printf("route %d:\n",route.id);
			pi[0]=route.id;
			pi++;
			int i=0;
			for(CapPoint &point : route.cap_points) {
				printf("	cap index=%d,id=%d,x=%lf,y=%lf,dir=%d\n",i,point.id,point.x,point.y,point.dir);
				i++;
			}
			i=0;
			printf("operation num= %d:\n",route.op_point.ops.size());
			for(Operation &op : route.op_point.ops) {
				printf("	op %d=%s\n",i,op.get_op_name());
				i++;
			}
		}
		printf("clm:leave msg 140,route_num=%d\n",get_global_agv_instance()->routes.size());
    }
	else if (func == 145)//获取路径
    {
		get_log_instance()->task_msg("###clm:rcv msg 145,获取路径\n");
		int id=input->arg1;
		int i=0;
		for(CapRoute &route : get_global_agv_instance()->routes) {
			if(route.id==id){
				printf("route %d:\n",route.id);
				
				output->arg1=route.cap_points.size();
				output->reserve_buf(route.cap_points.size()*30);
				char* pc=output->data_buf;
				
				for(CapPoint &point : route.cap_points) {
					printf("	cap index=%d,id=%d,x=%lf,y=%lf,dir=%d,dmcode=%d,compensation=%d\n",i,point.id,point.x,point.y,point.dir,point.dm_recgnize_enable,point.turn_compensation);
					double *pd = (double *)pc;
					pd[0] = point.x;
					pd[1] = point.y;
					pc+=16;
					int *pi=(int *)pc;
					pi[0]=point.id;
					pc+=4;
					pc[0] = point.dir;
					pc[1] = point.collision;
					pc[2] = point.dm_recgnize_enable;
					pc[3] = point.turn_compensation;
					int2byte(&pc[4], point.speed);
					pc+=10;
					i++;

				}
			}
		}
		printf("clm:leave msg 145,route_num=%d\n",get_global_agv_instance()->routes.size());
    }
	else if (func == 149)//获取任务链号
    {
		get_log_instance()->task_msg("###clm:rcv msg 149,获取任务链号\n");
		output->arg1=get_global_agv_instance()->chains.size();
		output->reserve_buf(get_global_agv_instance()->chains.size()*4);
		int* pi=(int *)output->data_buf;
		for(RouteChain &chain : get_global_agv_instance()->chains) {
			printf("chain %d:\n",chain.id);
			pi[0]=chain.id;
			pi++;
			int i=0;
			for(CapRoute &route : chain.routes) {
				printf("	route index=%d,id=%d\n",i,route.id);
				i++;
			}
		}
		printf("clm:leave msg 149,chain_num=%d\n",get_global_agv_instance()->chains.size());
    }
	else if (func == 150)//获取任务链
    {
		get_log_instance()->task_msg("###clm:rcv msg 150,获取任务链\n");
		int id=input->arg1;
		int i=0;
		for(RouteChain &chain : get_global_agv_instance()->chains) {
			if(chain.id==id){
				printf("chain %d:\n",chain.id);
				
				output->arg1=chain.routes.size();
				output->reserve_buf(chain.routes.size()*20);
				char* pc=output->data_buf;
				
				for(CapRoute &route : chain.routes) {
					printf("	route index=%d,id=%d\n",i,route.id);
					int *pi=(int *)pc;
					pi[0]=route.id;
					pc+=4;					
					i++;
				}
			}
		}
		printf("clm:leave msg 150,route_num=%d\n",get_global_agv_instance()->routes.size());
    }
    else if (func == 143)//获取cloud
    {
		printf("###clm:rcv msg 143,获取cloud\n");
        PointCloudData laser_cloud_data = get_global_agv_instance()->get_last_radar_data();
		output->arg1=laser_cloud_data.points.size();
		output->reserve_buf(output->arg1*16);
		double* pc=(double*)output->data_buf;
		Position base=get_global_agv_instance()->expolate_current_position();
		
		printf("###now.x=%lf,y=%lf\n",base.x,base.y);
		for (int i = 0; i<output->arg1; i++)
        {
        double xp = laser_cloud_data.points[output->arg1-i-1](0);
        double yp = laser_cloud_data.points[output->arg1-i-1](1);
        Position p=Position(0,xp,yp,0);
        p=base*p;
        pc[0] = p.x;
        pc[1] = p.y;
        pc+=2;		
        }
		/*std::vector<Eigen::Vector2d>::iterator itc;
		for(itc=get_global_agv_instance()->get_last_radar_data().points.begin();itc!=get_global_agv_instance()->get_last_radar_data().points.end();itc++){
			Eigen::Vector2d act = *itc;
			
			Position p=Position(0,act(0),act(1),0);
			p=base*p;
			pc[0] = p.x;
			pc[1] = p.y;
//			printf("##x=%lf,y=%lf\n",pc[0],pc[1]);
			pc+=2;
		}*/
        


    }
    else if (func == 141)//获取leg
    {
		printf("###clm:rcv msg 141,获取leg\n");
		output->arg1=2;
		output->reserve_buf(2*16);
		char* pc=output->data_buf;
		
		Position pose1,pose2;
		get_global_agv_instance()->masks.clear();
//		get_global_agv_instance()->legs_recognize(1.25,0.8,170,20,pose1,pose2);
		get_global_agv_instance()->legs_recognize(0.8,1.25,170,20,pose1,pose2);
		double radar_x = get_configs()->get_float("radar", "radar_position_x", nullptr);
		pose1.x+=radar_x;
		pose2.x+=radar_x;
		
		pose1 = get_global_agv_instance()->expolate_current_position()*pose1;
		pose2 = get_global_agv_instance()->expolate_current_position()*pose2;
		double *pd = (double *)pc;
		pd[0] = pose1.x;
		pd[1] = pose1.y;
		pd[2] = pose2.x;
		pd[3] = pose2.y;
    }
    else if (func == 17)//开始定位
    {
        if(input->data_size > 0) {
            get_global_agv_instance()->load_grid_map(input->data_buf + sizeof(double)*2, input->data_size - sizeof(double)*2);
           // get_global_agv_instance()->start_global_locating();
           get_global_agv_instance()->loadPoseFromserver();
        }
    }
    else if (func == 117)//开始定位2
    {
		get_log_instance()->task_msg("###clm:rcv msg 117,开始定位2\n");
        if(input->data_size > 0) {
		    //printf("###input->data_size=%d,sizeof(double)*3=%d\n",input->data_size,sizeof(double)*3);	
            get_global_agv_instance()->load_grid_map(input->data_buf + sizeof(double)*3, input->data_size - sizeof(double)*3);
			char* pc=input->data_buf;
			double *pd = (double *)pc;
			double xp = pd[0];
			double yp = pd[1];
			double theta = pd[2];
			get_global_agv_instance()->loadPose(xp,yp,theta);
		    char   filename[128];
			char   runpath[128];
			getcwd(runpath, 127);
			sprintf(filename,"%s/map/default_map",runpath);
			FILE *fp;
			if((fp=fopen(filename,"wb"))==NULL){
			printf("error open file==%s\n",filename);
			return;
			}else{
			int ret;//
			int write_len=0;
			int file_size= input->data_size - sizeof(double)*3;
			while(write_len<file_size){
				ret=fwrite(input->data_buf+sizeof(double)*3,1,input->data_size - sizeof(double)*3,fp);
				write_len+=ret;
			}
			fclose(fp);
			printf("file=%s save success!!!!!!!!!!!\n",filename);
			}
        }
    }
    else if (func == 18)//开始建图
    {
        get_global_agv_instance()->start_mapping();
        get_log_instance()->task_msg("###lwg:rcv msg 18,开始建图，清除所有cap点\n");
        get_global_agv_instance()->caps.clear();
        get_global_storage()->save_caps(get_global_agv_instance()->caps);
        get_global_agv_instance()->routes.clear();
        get_global_storage()->save_routes(get_global_agv_instance()->routes);
    }
    else if (func == 21)//停止建图
    {
        get_global_agv_instance()->stop_mapping();
		//在全局优化后,保存地图
    }
    else if (func == 6)//暂停任务
    {
		get_log_instance()->task_msg("###clm:rcv msg 6,暂停任务\n");
        get_global_agv_instance()->stop_path();
    }
    else if (func == 7)//继续任务
    {
		get_log_instance()->task_msg("###clm:rcv msg 7,继续任务\n");
        get_global_agv_instance()->continue_path();
    }
	else if (func == 8)//停止导航
    {
		get_log_instance()->task_msg("###clm:rcv msg 8,停止任务\n");
		//get_global_agv_instance()->cancel_path();
		get_global_agv_instance()->stop_navigating();
    }
	else if (func == 133)//停止导航
    {
		get_log_instance()->task_msg("###clm:rcv msg 133,停止导航\n");
		//get_global_agv_instance()->cancel_path();
		get_global_agv_instance()->stop_navigating();
    }
	else if (func == 130)//捕捉cap点
    {
		get_log_instance()->task_msg("###clm:rcv msg 130,捕捉cap点\n");
		Position pose = get_global_agv_instance()->expolate_current_position();
		
		int id=input->arg1;
		std::list<CapPoint>::iterator itc;
		
		for(itc=get_global_agv_instance()->caps.begin();itc!=get_global_agv_instance()->caps.end();itc++){
			CapPoint act = *itc;
			if(act.id==id){
				printf("before erase\n");
				get_global_agv_instance()->caps.erase(itc);
				printf("after erase\n");
				break;
			}
		}
		
		CapPoint cap(id,pose.x,pose.y,pose.theta);
		get_global_agv_instance()->caps.push_back(cap);
		get_global_agv_instance()->dm_code_record = true;
		for(itc=get_global_agv_instance()->caps.begin();itc!=get_global_agv_instance()->caps.end();itc++){
			CapPoint act = *itc;
			printf("x=%lf,y=%lf,theta=%f,id=%d\n",act.x,act.y,act.theta,act.id);
		}
		get_global_storage()->save_caps(get_global_agv_instance()->caps);
		
    }
	else if (func == 131)//下发路径
    {
		get_log_instance()->task_msg("###clm:rcv msg 131,下发路径\n");
		
		std::list<CapRoute>::iterator itc;
		
		for(itc=get_global_agv_instance()->routes.begin();itc!=get_global_agv_instance()->routes.end();itc++){
			CapRoute cr = *itc;
			if(cr.id==input->arg1){
				printf("before erase\n");
				get_global_agv_instance()->routes.erase(itc);
				printf("after erase\n");
				break;
			}
		}
		CapRoute cap_route;
		cap_route.id=input->arg1;
		cap_route.op_id=0xff;
		int size = input->arg2;
        int *caps = (int*)input->data_buf;
		for(int i=0;i<size;i++){
			int cap_id=caps[i];
			printf("###cap_id=%d\n",cap_id);
			for(CapPoint &point : get_global_agv_instance()->caps) {
				if(point.id==cap_id){
					CapPoint p(point.id,point.x,point.y,point.theta,point.dir,point.collision);//20210122新对象
					cap_route.cap_points.push_back(p);
					printf("id=%d,x=%lf,y=%lf\n",p.id,p.x,p.y);
				}
			}
		}
		get_global_agv_instance()->routes.push_back(cap_route);
		
		get_global_storage()->save_routes(get_global_agv_instance()->routes);
		printf("route_id=%d\n",cap_route.id);
		for(CapPoint &point : cap_route.cap_points) {
			printf("id=%d,x=%lf,y=%lf,dir=%d\n",point.id,point.x,point.y,point.dir);
		}
		printf("clm:leave msg 31,route_num=%d\n",get_global_agv_instance()->routes.size());
    }
	else if (func == 132)//执行路径
    {
		get_log_instance()->task_msg("###clm:rcv msg 132,执行路径\n");
		
		for(CapRoute &route : get_global_agv_instance()->routes) {
			if(route.id==input->arg1){
				printf("clm:route.id=%d\n",input->arg1);
				RouteChain chain;
				CapRoute r=route;
				chain.id=-1;
				chain.routes.push_back(r);
				get_global_agv_instance()->start_navigating(chain);
				return;
			}
		}
		printf("clm:leave msg 32,route_num=%d\n",get_global_agv_instance()->routes.size());
    }
	else if (func == 148)//执行路径链
    {
		get_log_instance()->task_msg("###clm:rcv msg 1132,执行路径链\n");
		
		for(RouteChain &chain : get_global_agv_instance()->chains) {
			if(chain.id==input->arg1){
				printf("clm:chain.id=%d\n",input->arg1);
				int i=0;
				for(CapRoute &route : chain.routes) {
					printf("route index=%d,id=%d,x=%lf,y=%lf,dir=%d\n",i,route.id);
					i++;
				}
				get_global_agv_instance()->start_navigating(chain);
				return;
			}
		}
		printf("clm:leave msg 32,route_num=%d\n",get_global_agv_instance()->routes.size());
    }
	else if (func == 147)//下发任务链
    {
		get_log_instance()->task_msg("###clm:rcv msg 147,下发任务链\n");
		
		std::list<RouteChain>::iterator itc;
		
		for(itc=get_global_agv_instance()->chains.begin();itc!=get_global_agv_instance()->chains.end();itc++){
			RouteChain rc = *itc;
			if(rc.id==input->arg1){
				printf("before erase\n");
				get_global_agv_instance()->chains.erase(itc);
				printf("after erase\n");
				break;
			}
		}
		RouteChain chain;
		chain.id=input->arg1;
		int size = input->arg2;
        int *routes = (int*)input->data_buf;
		for(int i=0;i<size;i++){
			int route_id=routes[i];
			printf("###route_id=%d\n",route_id);
			for(CapRoute &r : get_global_agv_instance()->routes) {
				if(r.id==route_id){
					CapRoute route=r;	//todo  test
					chain.routes.push_back(route);
					printf("route id=%d,points.size=%d\n",route_id,route.cap_points.size());
				}
			}
		}
		get_global_agv_instance()->chains.push_back(chain);
		
		get_global_storage()->save_chains(get_global_agv_instance()->chains);
		printf("chain_id=%d\n",chain.id);
		for(CapRoute &route : chain.routes) {
			printf("route id=%d\n",route.id);
		}
		printf("clm:leave msg 147,chain_num=%d\n",get_global_agv_instance()->chains.size());
    }
	else if (func == 144)//设置速度
    {
		//printf("###clm:rcv msg 144,设置速度\n");
        get_log_instance()->task_msg("###lwg:rcv msg 144,设置手动速度\n");
        get_global_agv_instance()->my_runcurve->set_manual_ratio(input->arg1);
    }
	else if (func == 136)//定义动作
    {
		get_log_instance()->task_msg("###clm:rcv msg 136,定义动作\n");
		
		for(CapRoute &route : get_global_agv_instance()->routes) {
			if(route.id==input->arg1){
				if(input->arg2==0){//上升
					printf("###路径%d,动作为%d\n",input->arg1,input->arg2);
					route.op_id=0;
					route.op_point.ops.clear();
					//等5秒,上升,等5秒,雷达扫描,开桌腿屏蔽
					Operation op1,op2,op3,op4,op5;
					op1.type=OP_DELAY;
					op1.delay.sec=3;
					route.op_point.ops.push_back(op1);
					op2.type=OP_LIFT_UP;
					op2.lift.dir=1;
					route.op_point.ops.push_back(op2);
					op3.type=OP_DELAY;
					op3.delay.sec=3;
					route.op_point.ops.push_back(op3);
					op4.type=OP_RADAR;
					op4.radar.mode=1;
					route.op_point.ops.push_back(op4);
					op5.type=OP_MASK;
					op5.mask.onoff=true;
					route.op_point.ops.push_back(op5);
				}else if(input->arg2==1){//下降
					route.op_id=1;
					route.op_point.ops.clear();
					printf("###路径%d,动作为%d\n",input->arg1,input->arg2);
					//等5秒,下降,等5秒,关桌腿屏蔽
					Operation op1,op2,op3,op4;
					op1.type=OP_DELAY;
					op1.delay.sec=3;
					route.op_point.ops.push_back(op1);
					op2.type=OP_LIFT_DOWN;
					op2.lift.dir=0;
					route.op_point.ops.push_back(op2);
					op3.type=OP_DELAY;
					op3.delay.sec=3;
					route.op_point.ops.push_back(op3);
					op4.type=OP_MASK;
					op4.mask.onoff=false;
					route.op_point.ops.push_back(op4);
				}
				get_global_storage()->save_routes(get_global_agv_instance()->routes);
			}
		}
    }
	else if (func == 139)//定义点属性
    {
		printf("###clm:rcv msg 139,定义方向,route=%d,index=%d,dir=%d,collision=%d,dmcode=%d,turn_compensate=%d\n",input->arg1,input->arg2,
		input->data_buf[0],input->data_buf[1],input->data_buf[2],input->data_buf[3]);
		std::list<CapPoint>::iterator iter;
		for(CapRoute &route : get_global_agv_instance()->routes) {//找route
            if(route.id==input->arg1){
                int index=input->arg2;
                if(route.cap_points.size()>index){
                    iter = route.cap_points.begin();
                    advance(iter,index);
                    iter->dir=input->data_buf[0];
                    iter->collision=input->data_buf[1];
                    //iter->dm_recgnize_enable= input->data_buf[2];
                    iter->dm_recgnize_enable= (input->data_buf[2] == 1) ? 1 : 0;
                    iter->turn_compensation = (input->data_buf[3] == 1) ? 1 : 0;
                    iter->speed = byte2int(&input->data_buf[4]);
                    if(iter->speed<10) iter->speed = 500;
                    iter->action =input->data_buf[8];
                    iter->rotate_ang = byte2int(&input->data_buf[9]);
                    printf("dm_recgnize_enable=%d,turn_compensation=%d,speed=%d\n",iter->dm_recgnize_enable,iter->turn_compensation,iter->speed);
                    printf("doact=%d,rotate_ang=%d\n",iter->action,iter->rotate_ang);
                }
            }else{
//				printf("###no match\n");
			}
		}
		get_global_storage()->save_routes(get_global_agv_instance()->routes);
    }
    else if (func == 142)//顶升动作,1上升,0下降
    {
		get_log_instance()->task_msg("###clm:rcv msg 142,顶升动作\n");
		MoveInstruction move;
		move.type = MOVE_INSTRUCTION_TYPE_LIFT;
		move.lift.dir=input->arg1;
		std::vector<char> data;
		move.to_char_array(&data);
		get_global_agv_instance()->get_message_queue()->send(CHANNEL_MOVE, data.data(), data.size());
		//get_global_agv_instance()->lift_complete = false;
    }
    else if (func == 151)//顶升旋转动作,4以一定角速度旋转固定角度,2旋转回零,3
    {
		get_log_instance()->task_msg("###clm:rcv msg 151,顶升旋转动作\n");
		MoveInstruction move;
		move.type = MOVE_INSTRUCTION_TYPE_TURN;
		move.turn.dir=input->arg1;
		move.turn.angle=(double)input->arg2/180.*M_PI;
		std::vector<char> data;
		move.to_char_array(&data);
		get_global_agv_instance()->get_message_queue()->send(CHANNEL_MOVE, data.data(), data.size());
		//get_global_agv_instance()->lift_complete = false;
    }
    else if (func == 152)//设置手动速度
    {
        get_log_instance()->task_msg("###lwg:rcv msg 152,设置手动速度\n");
        get_global_agv_instance()->my_runcurve->set_manual_ratio(input->arg1);
    }
    else if (func == 153)//获取手动速度
    {
        get_log_instance()->task_msg("###lwg:rcv msg 153,获取手动速度\n");
        output->arg1 = get_global_agv_instance()->my_runcurve->get_manual_ratio();
        //get_global_agv_instance()->my_runcurve->set_manual_ratio(input->arg1);
    }
    else if (func == 146)//顶升高度
    {
		get_log_instance()->task_msg("###clm:rcv msg 146,顶升设置高度\n");
		MoveInstruction move;
		move.type = MOVE_INSTRUCTION_TYPE_LIFT_HEIGHT;
		move.lift.height=input->arg1;
		std::vector<char> data;
		move.to_char_array(&data);
		get_global_agv_instance()->get_message_queue()->send(CHANNEL_MOVE, data.data(), data.size());
		//get_global_agv_instance()->lift_complete = false;
    }
	else if (func == 135)//全局避障开关
    {
		get_log_instance()->task_msg("###clm:rcv msg 135,避障开关\n");
    	
		if(input->arg1>=1){//打开
			get_global_agv_instance()->collision_switch=true;
			get_log_instance()->task_msg("###clm:rcv msg 135,collision=on\n");
		}else{
			get_global_agv_instance()->collision_switch=false;
			get_global_agv_instance()->collision_level=0;
			get_log_instance()->task_msg("###clm:rcv msg 135,collision=off\n");
			MoveInstruction move;
			move.type = MOVE_INSTRUCTION_TYPE_COLLISION;
			move.collision.level=0;
			std::vector<char> data;
			move.to_char_array(&data);
			get_global_agv_instance()->get_message_queue()->send(CHANNEL_MOVE, data.data(), data.size());
		}
    }
	else if (func == 250)//屏蔽腿
    {
		printf("###clm:rcv msg 250,屏蔽腿\n");
    	
		if(input->arg1==1){//打开
			get_global_agv_instance()->mask_switch=true;
			printf("###clm:rcv msg 250,mask_switch=on\n");
		}else{
			get_global_agv_instance()->mask_switch=false;
			printf("###clm:rcv msg 250,mask_switch=off\n");
		}
    }
	else if (func == 38)//开始建图-反光板
    {
		int size = input->data_size;
		get_global_agv_instance()->map_id=input->arg1;
		memcpy(get_global_agv_instance()->map_name, input->data_buf, size);
		printf("msg 40,开始建图:id=%d,name=%s\n",get_global_agv_instance()->map_id, input->data_buf);
		
        get_global_agv_instance()->start_mapping_landmark();
    }
    else if (func == 39)//停止建图
    {
        get_global_agv_instance()->stop_mapping_landmark();
		//在全局优化后,保存地图
    }
	else if (func == 40)//开始建图
    {
		int size = input->data_size;
		get_global_agv_instance()->map_id=input->arg1;
		memcpy(get_global_agv_instance()->map_name, input->data_buf, size);
		printf("msg 40,开始建图:id=%d,name=%s\n",get_global_agv_instance()->map_id, input->data_buf);
		
        get_global_agv_instance()->start_mapping();
    }
	else if (func == 41)//查询地图列表
    {
		string s=get_global_storage()->get_map_list();
		printf("msg 41,查询地图列表:%s\n",s.c_str());
		int size=s.size();
		output->arg1=size;
		output->reserve_buf(size);
		memcpy(output->data_buf,s.c_str(),size);
    }
	else if(func == 42){	//获取地图
		int id=input->arg1;
		string map_name=get_global_storage()->get_map_slot(id);
		printf("msg 42,获取地图:id=%d,name=%s\n",id,map_name.c_str());
		FILE *fp;
		if((map_name.size()==0)||((fp=fopen(map_name.c_str(),"rb"))==NULL)){
			printf("error open file\n");
			return;
		}else{
			fseek(fp, 0, SEEK_END);
			long size = ftell(fp);
			output->reserve_buf(size);
			printf("size=%d\n",size);
			fseek(fp, 0, SEEK_SET);
			int ret;
			int read_size=0;
			while (read_size<size) {
				ret=fread(output->data_buf+read_size,1,size-read_size,fp);
				read_size+=ret;
			}
			fclose(fp);
		}
	}
	else if(func == 43){	//下发指定地图
	
		int id=input->arg1;
		int map_name_len=input->arg2;
		char map_name[100];
		memset(map_name,0,100);
		memcpy(map_name,input->data_buf,map_name_len);
		int file_size= input->data_size-map_name_len;
		
		printf("msg 43:id=%d,name=%s,file_size=%d\n",id,map_name,file_size);
		FILE *fp;
		if((fp=fopen(map_name,"wb"))==NULL){
			printf("error open file\n");
			return;
		}else{
			int ret;//
			int write_len=0;
			while(write_len<file_size){
				ret=fwrite(input->data_buf+map_name_len+write_len,1,file_size-write_len,fp);
				write_len+=ret;
			}
			fclose(fp);
			get_global_storage()->modify_map_slot(id,map_name);
		}
	}
	else if(func == 44){	//删除指定地图
	
		int id=input->arg1;
		printf("msg 44:id=%d\n",id);
		get_global_storage()->delete_map_slot(id);
	}
	else if(func == 45){	//指定默认地图
		int id=input->arg1;
		printf("msg 45:id=%d\n",id);
		get_global_agv_instance()->default_locate_map_id=id;
		get_global_storage()->modify_default_map(id);
	}
}

void error_handling(char *message)
{
    fputs(message, stderr);
    fputc('\n', stderr);
    exit(1);
}

void handle_socket(void *param){
	int clnt_sock = *((int*)param);
    delete param;
    int extra_len = 0, rec_len = 0, total_len = 0, snd_len = 0;
    int ret = 0;
    char buf_head[HEAD_SIZE];

	while(1){
		extra_len = 0;
		rec_len = 0;
		total_len = 0;
		snd_len = 0;
		
		ret = read(clnt_sock, buf_head, HEAD_SIZE);
		if(ret<=0){
//			printf("remote disconnect,thread quit\n");
			close(clnt_sock);
			break;
		}
		if (ret != HEAD_SIZE)
		{
			printf("[LOG] Read package head error .ret=%d\n",ret);
			continue;
		}
		TCPDataPackage2 *recv = new TCPDataPackage2(buf_head, &ret);
		if (ret != 0)
		{
			printf("[LOG] Package check failed \n");
			delete recv;
			continue;
		}
//		printf("[LOG] valid package,cmd=%d\n",recv->function);
		extra_len = recv->data_size;
		rec_len = 0;
		if(recv->data_size > 0)
		{
			while (rec_len < extra_len)
			{
				ret = read(clnt_sock, recv->data_buf + rec_len, extra_len - rec_len);
				if (ret < 0)
				{
					break;
				}
				rec_len += ret;
			}
		}
		if(rec_len < extra_len)
		{
			printf("[LOG] Read extra data failed \n");
			delete recv;
			continue;
		}
		TCPDataPackage2 *respond = new TCPDataPackage2(recv->function);
		handle_package(recv, respond);

		respond->to_bytes(buf_head);
		write(clnt_sock, buf_head, HEAD_SIZE);
//		printf("[LOG] Send data\n");
		extra_len = respond->data_size;
		snd_len = 0;
		if (extra_len > 0)
		{
			while (snd_len < extra_len)
			{
				ret = write(clnt_sock, respond->data_buf + snd_len, extra_len - snd_len);
				if (ret < 0){
					break;
				}
				snd_len += ret;
			}
		}
		if (snd_len < extra_len){
			printf("[LOG] Send data failed \n");
		}

		delete recv;
		delete respond;
	}
}


void handle_pipe(int sig){//不做任何处理即可
}
void* thread_report_11110(void *param){
	int clnt_sock = *((int*)param);
    delete param;
    int extra_len = 0, rec_len = 0, total_len = 0, snd_len = 0;
    int ret = 0;
    char buf_head[HEAD_SIZE];

	while(1){
		TCPDataPackage2 *respond = new TCPDataPackage2(2);
		Position pose = get_global_agv_instance()->expolate_current_position();
		double score = 1;
		respond->arg1 = get_global_agv_instance()->get_system_state();
		respond->arg2 = get_global_agv_instance()->collision_level;
		respond->reserve_buf(sizeof(double) * 3 + sizeof(int)*10);
//		printf("###agv_state=%d\n",get_global_agv_instance()->get_system_state());
		
        char *pc=respond->data_buf;
        double *pd = (double *)pc;
        pd[0] = pose.x;
        pd[1] = pose.y;
        pd[2] = pose.theta;
		
        pc+=24;
        int * pi=(int *)pc;

        pi[0]=	get_global_agv_instance()->exec_cap;
        pi[1]=	get_global_agv_instance()->warning_code;
        pi[2]=	get_global_agv_instance()->error_code;
		if(!get_global_agv_instance()->lift_complete)
		pi[3]=	0;
		else
		{
        if(get_global_agv_instance()->get_system_state()==SYSTEM_STATE_NAVIGATING)
        pi[3]=	0;
        else if (get_global_agv_instance()->get_system_state()==SYSTEM_STATE_FREE)
        pi[3]=	1;
		}
        if((get_global_agv_instance()->exec_route&0x10000)==0)
        pi[4] = 0;
        else 
        pi[4] = get_global_agv_instance()->exec_route;

        pi[5] = int(get_global_agv_instance()->locate_modual->today_odom);
        pi[6] = int(get_global_agv_instance()->locate_modual->total_odom);
        pi[7] = get_global_agv_instance()->board_sts._battary;
        pi[8] = get_global_agv_instance()->mcu_speed;
        pi[9] = get_global_agv_instance()->mcu_omega;

		respond->to_bytes(buf_head);
		
		
struct sigaction action;
action.sa_handler = handle_pipe;
sigemptyset(&action.sa_mask);
action.sa_flags = 0;
sigaction(SIGPIPE, &action, NULL);

		ret = write(clnt_sock, buf_head, HEAD_SIZE);
		//printf("[LOG] Send %d\n",ret);
		if (ret < 0){
			close(clnt_sock);
			return nullptr;
		}
		extra_len = respond->data_size;
		snd_len = 0;
		if (extra_len > 0)
		{
			while (snd_len < extra_len)
			{
				ret = write(clnt_sock, respond->data_buf + snd_len, extra_len - snd_len);
				//printf("[LOG] Send %d\n",ret);
				if (ret < 0){
					close(clnt_sock);
					return nullptr;
				}
				snd_len += ret;
			}
		}
		if (snd_len < extra_len){
			printf("[LOG] Send data failed \n");
		}
		delete respond;
		
		usleep(200000);
	}
	return nullptr;
}
/*
class CLI{
	public:
	int socket;
	struct sockaddr_in addr;
	int closed;
};

std::list<CLI>	clis;

void report_state(){
	char buf_head[HEAD_SIZE];
	int ret;
	
	std::list<CLI>::iterator it;
	for(it=clis.begin();it!=clis.end();it++){
		int extra_len = 0, rec_len = 0, total_len = 0, snd_len = 0;
		TCPDataPackage *respond = new TCPDataPackage(0x02);
		respond->to_bytes(buf_head);
		write(it->socket, buf_head, HEAD_SIZE);
		
        Position pose = get_global_agv_instance()->expolate_current_position();
        double score = 1;
        respond->arg1 = get_global_agv_instance()->get_system_state();
        respond->arg2 = get_global_agv_instance()->collision_level;
        respond->reserve_buf(sizeof(double) * 4 + sizeof(int)*8);
		
		char *pc=respond->data_buf;
        double *pd = (double *)pc;
        pd[0] = pose.x;
        pd[1] = pose.y;
        pd[2] = pose.theta;
        pd[3] = score;
		
		pc+=32;
        int * pi=(int *)pc;
		pi[0]=	get_global_agv_instance()->colli_cnt1;
		pi[1]=	get_global_agv_instance()->colli_cnt2;
		pi[2]=	get_global_agv_instance()->colli_cnt3;
        pi[3] = get_global_agv_instance()->mcu_input;
        pi[4] = get_global_agv_instance()->mcu_output;
        pi[5] = get_global_agv_instance()->mcu_battery_level;
        pi[6] = get_global_agv_instance()->mcu_speed;
		extra_len = respond->data_size;
		snd_len = 0;
		if (extra_len > 0)
		{
			while (snd_len < extra_len)
			{
				ret = write(it->socket, respond->data_buf + snd_len, extra_len - snd_len);
				if (ret < 0){
					break;
				}
				snd_len += ret;
			}
		}
	}
}
*/
pthread_t state_port_thread;
pthread_t rpt_position_thread;
pthread_t state_rpt_thread;

void *thread_state_port(void *param) {
	int serv_sock;
    int clnt_sock;

    struct sockaddr_in serv_addr;
    struct sockaddr_in clnt_addr;
    socklen_t clnt_addr_size;
	
//    threadpool thread_pool = thpool_init(20);
	
    serv_sock = socket(PF_INET, SOCK_STREAM, 0);
    if (serv_sock == -1)
    {
        error_handling("socket() error");
    }
    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    serv_addr.sin_port = htons(11110);
	
	const int on=1;
	setsockopt(serv_sock,SOL_SOCKET,SO_REUSEADDR,&on,sizeof(on));

    if (bind(serv_sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) == -1)
    {
        error_handling("bind() error");
    }
    if (listen(serv_sock, 5) == -1)
    {
        error_handling("listen() error");
    }

    clnt_addr_size = sizeof(clnt_addr);
    printf("START LISTENING AT PORT %d\n", 11110);

    while (1)
    {
		printf("before accept\n");
        clnt_sock = accept(serv_sock, (struct sockaddr *)&clnt_addr, &clnt_addr_size);
        if (clnt_sock == -1){
            printf("[LOG] Accept error :%s\n", strerror(errno));
            continue;
        }
		
		printf("[LOG] Accept client :%s:%d\n",inet_ntoa(clnt_addr.sin_addr),clnt_addr.sin_port);
		/*
		CLI c;
		c.socket=clnt_sock;
		c.addr=clnt_addr;
		c.closed=0;
		clis.push_back(c);
		*/
        int *p_socket = new int(clnt_sock);
//        thpool_add_work(thread_pool, thread_report_11110, (void*)p_socket);
		pthread_create(&state_rpt_thread, NULL, thread_report_11110, (void*)p_socket);
		
    }
		printf("after while\n");
//    thpool_destroy(thread_pool);
}
void *thread_rpt_position(void *param) {
	
    printf("位置线程进入\n");
    int cnt_num = 0;
    int cnt_num1 = 0;
    bool first_cal = true;
    float neareast_dis = 0.0;
    while(1){
        get_global_agv_instance()->exec_cap=0xfffe;
        Position pose = get_global_agv_instance()->expolate_current_position();
        if((get_global_agv_instance()->exec_route&0x10000)==0){//没在跑路径,匹配全部点
            //printf("没在跑路径\n");
            for(CapPoint &point : get_global_agv_instance()->caps) {
                double dis=pow(point.x-pose.x,2)+pow(point.y-pose.y,2);
                if(first_cal)
                {
                    neareast_dis = dis;
                    first_cal = false;
                    get_global_agv_instance()->exec_cap=point.id;
                }
                if (dis<neareast_dis){
					neareast_dis = dis;
                    get_global_agv_instance()->exec_cap=point.id;
                    
                }
            }
            first_cal = true;
            if(!(++cnt_num%10))
            printf("no route running-----cur cap id=%d \n",get_global_agv_instance()->exec_cap);
		}else{
			//printf("正在跑路径\n");
			for(CapRoute &route : get_global_agv_instance()->routes) {
				if(route.id==(get_global_agv_instance()->exec_route&0xffff)){
					for(CapPoint &point : route.cap_points) {
						double dis=pow(point.x-pose.x,2)+pow(point.y-pose.y,2);
						///&&(!get_global_agv_instance()->my_runcurve->busying)
						if (dis<pow(0.3,2)){
							printf("	cap id=%d,x=%lf,y=%lf,dir=%d\n",point.id,point.x,point.y,point.dir);
							get_global_agv_instance()->exec_cap=point.id;
							break;
						}
					}
				}
			}
            first_cal = true;
            if(!(++cnt_num1%10))
            printf("route is running-----cur cap id=%d \n",get_global_agv_instance()->exec_cap);
		}
		sleep(1);
	//检查cap列表是否有数据
	//如果有,挨个计算与当前点的距离
	}
	printf("位置线程退出\n");
}
void running_network()
{
    int serv_sock;
    int clnt_sock;

    struct sockaddr_in serv_addr;
    struct sockaddr_in clnt_addr;
    socklen_t clnt_addr_size;

    threadpool thread_pool = thpool_init(get_configs()->get_int("network", "num_threads", nullptr));

    serv_sock = socket(PF_INET, SOCK_STREAM, 0);
    if (serv_sock == -1)
    {
        error_handling("socket() error");
    }
    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    serv_addr.sin_port = htons(local_port);
	
	const int on=1;
	setsockopt(serv_sock,SOL_SOCKET,SO_REUSEADDR,&on,sizeof(on));
	
    if (bind(serv_sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) == -1)
    {
        error_handling("bind() error");
    }
    if (listen(serv_sock, 5) == -1)
    {
        error_handling("listen() error");
    }

    clnt_addr_size = sizeof(clnt_addr);
    printf("START LISTENING AT PORT %d\n", local_port);

    while (1)
    {
        clnt_sock = accept(serv_sock, (struct sockaddr *)&clnt_addr, &clnt_addr_size);
		
        if (clnt_sock == -1){
            printf("[LOG] Accept error :%s\n", strerror(errno));
            continue;
        }
		
//		printf("[LOG] Accept client :%s:%d\n",inet_ntoa(clnt_addr.sin_addr),clnt_addr.sin_port);
/*
		CLI c;
		c.socket=clnt_sock;
		c.addr=clnt_addr;
		c.closed=0;
		clis.push_back(c);
		*/
        int *p_socket = new int(clnt_sock);
        thpool_add_work(thread_pool, handle_socket, (void*)p_socket);
		
    }

    thpool_destroy(thread_pool);
}


#endif