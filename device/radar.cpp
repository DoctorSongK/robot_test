
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/time.h>

#include <termios.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ipc.h>
#include <sys/msg.h>

//#include "../logic/agv.h"
#include "radar.h"
#include "../common/time.h"
#include "../common/configuration.h"

RadarModule::RadarModule()
{

	pthread_mutex_init(&map_mutex, nullptr);
	pthread_create(&radar_thread, NULL, thread_function, this);
	pthread_create(&monitor_radar_thread, NULL, monitor_radar_state, this);
};

RadarModule::~RadarModule()
{
	thread_running = false;
	close_radar();
	pthread_mutex_destroy(&map_mutex);
	pthread_join(radar_thread, NULL);
	pthread_join(monitor_radar_thread, NULL);
};

#define PI 3.1415926
#define MAXLINE 40960
void RadarModule::make_cmd_frame(enum RadarModule::CMD cmd)
{
	char const *tmp;
	memset(send_buf, 0, 1024);
	send_buf[0] = 0x02;
	switch (cmd)
	{
	case SINGLE_READ:
		tmp = cmd_single_read;
		break;
	case CONTINUE_READ_START:
		tmp = cmd_continue_read_start;
		break;
	case CONTINUE_READ_STOP:
		tmp = cmd_continue_read_stop;
		break;
	default:
		break;
	}
	memcpy(send_buf + 1, tmp, strlen(tmp));
	send_buf[strlen(tmp) + 1] = 0x03;
};

void RadarModule::make_cmd_frame2(enum RadarModule::CMD cmd)
{
	memset(send_buf, 0, 1024);
	send_buf[0] = 0x02;
	send_buf[1] = 0x73;
	send_buf[2] = 0x52;
	send_buf[3] = 0x4e;
	send_buf[4] = 0x20;
	send_buf[5] = 0x4c;
	send_buf[6] = 0x4d;
	send_buf[7] = 0x44;
	send_buf[8] = 0x73;
	send_buf[9] = 0x63;
	send_buf[10] = 0x61;
	send_buf[11] = 0x6e;
	send_buf[12] = 0x64;
	send_buf[13] = 0x61;
	send_buf[14] = 0x74;
	send_buf[15] = 0x61;
	send_buf[16] = 0x03;
};
int RadarModule::laser_reconnect()
{
	close(sockfd);
	usleep(5000);
	printf("laser reconnect\n");
    int port = get_configs()->get_int("radar", "radar_port", nullptr);
    char host_name[20];
    get_configs()->get_string("radar", "radar_ip", host_name, nullptr);
	printf("radar device ip=%s,port=%d\n",host_name,port);
	if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
	{
		printf("create socket error: %s\n", strerror(errno));
		return -1;
	}
	struct sockaddr_in servaddr;
	memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(port);
	if (inet_pton(AF_INET, host_name, &servaddr.sin_addr) <= 0)
	{
        printf("inet_pton error for %s\n", host_name);
        return -1;
	}
    int old_option = 0;
    int new_option = 0;
	
    old_option = fcntl(sockfd, F_GETFL,0);
    new_option = old_option | O_NONBLOCK;

	if (connect(sockfd, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0)
	{
		printf("connect error: %s(errno: %d) \n", strerror(errno), errno);
		return -1;
	}
	fcntl(sockfd, F_SETFL, new_option);
	if (send(sockfd, send_buf, strlen(send_buf), 0) < 0)
	{
		printf("send msg error: %s(errno: %d)\n", strerror(errno), errno);
		return -1;
	}

	printf("laser reconnect success\n");
	
	return 1;	
}
void *RadarModule::thread_function(void *param)
{

	RadarModule *ptr = (RadarModule *)param;

	//这里是线程参数
	//初始化并启动IMU
	//在这里用 ptr->xxx 的形式访问你定义的中间变量，就像下面while那一行里那样
	//------------------

	int *range, *rssi;
	int lazer_cnt = 0;
	char *temp1, *temp2;
	char *cmdtype, *cmd, *channel_content;
	int version, device_number, serial_number, device_status_l, device_status_h;
	int telegram_counter, scan_counter, start_time, xmission_time, dinput_l, dinput_h, doutput_l, doutput_h, rsvd;
	int scan_freq, measure_freq;
	int encoder_num, encoder_pos, encoder_speed;
	int channel_num, scale_fac, start_point, start_angle, angle_step, data_num;
	int i, j;

	int from_index = get_configs()->get_int("radar", "first_point_index", nullptr);
	if (from_index<0) from_index = 60;
	int to_index = get_configs()->get_int("radar", "last_point_index", nullptr);
	if (to_index<0) to_index = 480;
    int point_number = get_configs()->get_int("radar", "point_number", nullptr);
    if (point_number<0) point_number = 540;
	
    int inverse_install = get_configs()->get_int("radar", "inverse_install", nullptr);
    if (inverse_install<0) inverse_install = 0;
    int device_enable = get_configs()->get_int("radar", "radar_enable", nullptr);
    ptr->thread_running = (device_enable>0)?true:false;
    int port = get_configs()->get_int("radar", "radar_port", nullptr);
    char host_name[20];
    get_configs()->get_string("radar", "radar_ip", host_name, nullptr);
	printf("radar device ip=%s,port=%d,enable=%d install_dir=%d\n",host_name,port,ptr->thread_running,inverse_install);
	int n, rec_len, total_len;
	char recvline[4096], sendline[4096];
	char buf[MAXLINE];
	struct sockaddr_in servaddr;
	char *pbuf;
	if ((ptr->sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
	{
		printf("create socket error: %s\n", strerror(errno));
		return nullptr;
	}

	memset(&servaddr, 0, sizeof(servaddr));
	servaddr.sin_family = AF_INET;
	servaddr.sin_port = htons(port);
	if (inet_pton(AF_INET, host_name, &servaddr.sin_addr) <= 0)
	{
		printf("inet_pton error for %s\n", host_name);
		return nullptr;
	}
	int old_option = 0;
    int new_option = 0;
	int reconnect_time_interval = 2000000;
	old_option = fcntl(ptr->sockfd, F_GETFL,0);
	new_option = old_option | O_NONBLOCK;
	
	bool read_laser_success;
	int read_laser_error_cnt = 0;
	while (connect(ptr->sockfd, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0)
	{
		printf("connect error: %s(errno: %d) \n", strerror(errno), errno);
		//read_laser_error_cnt = 300;
		//return nullptr;
		usleep(reconnect_time_interval);
	}
	fcntl(ptr->sockfd, F_SETFL, new_option);////set the socket in noblock mode.
	//ptr->make_cmd_frame2(SINGLE_READ);
	ptr->make_cmd_frame(CONTINUE_READ_START);
	usleep(reconnect_time_interval);
	printf("radar :send msg to server: %s,len=%d read_laser_error_cnt=%d\n", ptr->send_buf, (int)strlen(ptr->send_buf),read_laser_error_cnt);

	if(read_laser_error_cnt == 0)
	{
	
	if (send(ptr->sockfd, ptr->send_buf, strlen(ptr->send_buf), 0) < 0)
	{
		printf("send msg error: %s(errno: %d)\n", strerror(errno), errno);
		//return nullptr;
	}
	else
        ptr->radar_disconnect = false;
	}
	int mtu_f = 0;
    char *token;
	//usleep(reconnect_time_interval);
    range = (int *)malloc(sizeof(int) * point_number);
    rssi = (int *)malloc(sizeof(int) * point_number);
	printf("radar start\n");
	int cycle_time = 20000;
    while (ptr->thread_running)
    //while(true)
    {
		
        usleep(cycle_time);
        //循环读取数据
        read_laser_success = false;
        mtu_f = 0;
        rec_len = 0;
        total_len = 0;
		//
        if(read_laser_error_cnt>=40) ////if the laser connect failed, then reconnect the laser.
        {
        ptr->radar_disconnect = true;
        if(ptr->laser_reconnect()==1)
        {
        usleep(cycle_time);
        read_laser_error_cnt=0;
        ptr->radar_disconnect = false;
        continue;
        }
        else
        {
        usleep(reconnect_time_interval);
        printf("excuting the laser reconnet thread!!!!!!!!!\n");
        continue;
        }
		
		}
		//send(ptr->sockfd, ptr->send_buf, strlen(ptr->send_buf), 0);
	 	while (1)
		{
			
			
			if ((rec_len = recv(ptr->sockfd, buf + total_len, MAXLINE, 0)) ==-1)
			{
				printf("laser recv error cnt=%d\n",read_laser_error_cnt);
				//return nullptr;
				read_laser_success = false;
				read_laser_error_cnt++;
				usleep(cycle_time);
				break;
				//continue;
			}
			total_len += rec_len;
			if (buf[total_len - 1] == 0x03)
			{
				read_laser_error_cnt = 0;
				read_laser_success = true;
				break;
			}
			else
			{
				mtu_f = 1;
				read_laser_success = false;
			}
		}
		lazer_cnt++;
		if(!read_laser_success) continue;

		ptr->detect_cnt = 0;
		ptr->radar_error = false;
		rec_len = total_len;
		buf[rec_len] = '\0';
		//		printf("#rcv lazer frame. time=%ld\n",get_current_time());
		rec_len = total_len;
		buf[rec_len] = '\0';
		if (buf[0] == 0x02)
		{
			//printf("Received : %s \n\n",buf);
			cmdtype = strtok(buf + 1, " ");
			if (strcmp(cmdtype, "sSN") == 0)
			{
				cmd = strtok(NULL, " ");
				if(cmd==NULL) continue;
                token = strtok(NULL, " ");
                if(token==NULL) continue;
                else
				sscanf(token, "%x", &version);
                token = strtok(NULL, " ");
                if(token==NULL) continue;
                else
				sscanf(token, "%x", &device_number);
                token = strtok(NULL, " ");
                if(token==NULL) continue;
                else
				sscanf(token, "%x", &serial_number);
                token = strtok(NULL, " ");
                if(token==NULL) continue;
                else
				sscanf(token, "%x", &device_status_l);
                token = strtok(NULL, " ");
                if(token==NULL) continue;
                else
				sscanf(token, "%x", &device_status_h);
                token = strtok(NULL, " ");
                if(token==NULL) continue;
                else
				sscanf(token, "%x", &telegram_counter);
                token = strtok(NULL, " ");
                if(token==NULL) continue;
                else
				sscanf(token, "%x", &scan_counter);
                token = strtok(NULL, " ");
                if(token==NULL) continue;
                else
				sscanf(token, "%x", &start_time);
                token = strtok(NULL, " ");
                if(token==NULL) continue;
                else
				sscanf(token, "%x", &xmission_time);
                token = strtok(NULL, " ");
                if(token==NULL) continue;
                else
				sscanf(token, "%x", &dinput_l);
                token = strtok(NULL, " ");
                if(token==NULL) continue;
                else
				sscanf(token, "%x", &dinput_h);
                token = strtok(NULL, " ");
                if(token==NULL) continue;
                else
				sscanf(token, "%x", &doutput_l);
                token = strtok(NULL, " ");
                if(token==NULL) continue;
                else
				sscanf(token, "%x", &doutput_h);
                token = strtok(NULL, " ");
                if(token==NULL) continue;
                else
				sscanf(token, "%x", &rsvd);
                token = strtok(NULL, " ");
                if(token==NULL) continue;
                else
				sscanf(token, "%x", &scan_freq);
                token = strtok(NULL, " ");
                if(token==NULL) continue;
                else
				sscanf(token, "%x", &measure_freq);
                token = strtok(NULL, " ");
                if(token==NULL) continue;
                else
				sscanf(token, "%x", &encoder_num);

				if (encoder_num != 0)
				{
                token = strtok(NULL, " ");
                if(token==NULL) continue;
                else					
				sscanf(token, "%x", &encoder_pos);
                token = strtok(NULL, " ");
                if(token==NULL) continue;
                else
				sscanf(token, "%x", &encoder_speed);
				}
                token = strtok(NULL, " ");
                if(token==NULL) continue;
                else				
				sscanf(token, "%x", &channel_num);
				channel_content = strtok(NULL, " ");
                if(channel_content==NULL) continue;

                token = strtok(NULL, " ");
                if(token==NULL) continue;
                else
				sscanf(token, "%x", &scale_fac);
                token = strtok(NULL, " ");
                if(token==NULL) continue;
                else
				sscanf(token, "%x", &start_point);
                token = strtok(NULL, " ");
                if(token==NULL) continue;
                else
				sscanf(token, "%x", &start_angle);
                token = strtok(NULL, " ");
                if(token==NULL) continue;
                else
				sscanf(token, "%x", &angle_step);

                token = strtok(NULL, " ");
                if(token==NULL) continue;
                else
				sscanf(token, "%x", &data_num);
                if(data_num != point_number) continue;

				for (i = 0; i < data_num; i++)
				{
                    token = strtok(NULL, " ");
                    if(token==NULL){
                    read_laser_success=false;
                    break;
                    }
                    if (inverse_install==0)
                    sscanf(token, "%x", range + point_number -1 -  i);
                    else
                    sscanf(token, "%x", range +  i);
					//sscanf(strtok(NULL, " "), "%x", range + i);
				}
                if(!read_laser_success) continue;
                token = strtok(NULL, " ");
                if(token==NULL) continue;
                else
				sscanf(token, "%x", &channel_num);

				channel_content = strtok(NULL, " ");
                if(channel_content==NULL) continue;

                token = strtok(NULL, " ");
                if(token==NULL) continue;
                else
				sscanf(token, "%x", &scale_fac);

                token = strtok(NULL, " ");
                if(token==NULL) continue;
                else				
				sscanf(token, "%x", &start_point);
                token = strtok(NULL, " ");
                if(token==NULL) continue;
                else
				sscanf(token, "%x", &start_angle);
                token = strtok(NULL, " ");
                if(token==NULL) continue;
                else				
				sscanf(token, "%x", &angle_step);
                token = strtok(NULL, " ");
                if(token==NULL) continue;
                else
				sscanf(token, "%x", &data_num);
				//printf("\n ");
				for (i = 0; i < data_num; i++)
				{
                    token = strtok(NULL, " ");
                    if(token==NULL){
                    read_laser_success=false;
                    break;
                    }
                    if (inverse_install==0)
                    {
                    sscanf(token, "%x", rssi +  point_number -1 -  i);
                    if (rssi[point_number -1 -  i] > 100)
                    rssi[point_number -1 -  i] = 100;
                    }
                    else
                    {
                    sscanf(token, "%x", rssi +  i);
                    //sscanf(strtok(NULL, " "), "%x", rssi +  i);
                    if (rssi[i] > 100)
                    rssi[i] = 100;
                    }
					//printf("%d ",rssi[i]);
				}
                if(!read_laser_success) continue;
				//printf("\n ");
				/*
				^  x
				|	
				|	
				*/
				
				if ((lazer_cnt >= 5))
				{
					lazer_cnt = 0;
					long timestamp_us = get_current_time_us();
					int data_size = point_number; //点的数量，用于预分配空间
					PointCloudData data(timestamp_us, data_size);
					// printf("#rcv lazer frame. time=%ld\n",timestamp_us);
					int cnt=0;
                    for (int i = from_index; i <= to_index; i++)//60-480
                    {
                        double x = (0. - sin((45.0 - i / 2.) / 180. * PI) * range[i]) / 1000. ;
                        double y = (0. + cos((45.0 - i / 2.) / 180. * PI) * range[i]) / 1000. ;
                        bool leg=false;
                        data.add_point(x, y, rssi[i] / 100.0);
						/*if(get_global_agv_instance()->mask_switch&&(get_global_agv_instance()->masks.size()>0)){
							for (auto iter = get_global_agv_instance()->masks.begin();; iter++){
								double dis=pow(x-iter->x,2)+pow(y-iter->y,2);
								if(iter==get_global_agv_instance()->masks.end()){
									break;
								}
								if (dis<pow(get_global_agv_instance()->mask_radius,2)){
									leg=true;
									break;
								}
							}
							if(leg==false){
								data.add_point(x, y, rssi[i] / 100.0);
								cnt++;
							}
						}else{
							data.add_point(x, y, rssi[i] / 100.0);
							cnt++;
						}*/
						
						
					}
					
//					printf("####rcv lazer frame. report %d point,masks.size=%d\n",cnt,get_global_agv_instance()->masks.size());

					
					pthread_mutex_lock(&ptr->map_mutex);
					if (ptr->p_msg_queue != nullptr)
					{
						//printf("lwg radar data send\n");
						std::vector<char> tmp;
						data.to_char_array(&tmp);
						ptr->p_msg_queue->send(ptr->channel, tmp.data(), tmp.size());
					}
					pthread_mutex_unlock(&ptr->map_mutex);
				}
				//free(range);
				//free(rssi);
			}
		}
		else
		{
			//printf("radar wrong frame!\n");
			continue;
		}
	}
	printf("radar thread quit!\n");
	return nullptr;
};
/*
void RadarModule::send_collision_msg(int level){
	MoveInstruction move;
	move.type = MOVE_INSTRUCTION_TYPE_COLLISION;
	move.collision.level=level;
	std::vector<char> data;
	move.to_char_array(&data);
	get_global_agv_instance()->get_message_queue()->send(CHANNEL_MOVE, data.data(), data.size());
}
*/
void RadarModule::close_radar()
{
	//在这里关闭雷达
	//在这里直接用变量名就可以访问你定义的中间变量
	//如直接调用 thread_running
	//------------------
	close(sockfd);
};

void RadarModule::set_target(MsgQueue *target, int channel)
{
	pthread_mutex_lock(&map_mutex);
	p_msg_queue = target;
	this->channel = channel;
	pthread_mutex_unlock(&map_mutex);
};
void *RadarModule::monitor_radar_state(void *param)
{
    RadarModule *ptr = (RadarModule *)param;
    int num = 0;
    sleep(3);
    while (true)
    {
        ptr->detect_cnt++;
        if(ptr->detect_cnt>=10) 
        {
            ptr->radar_error = true;
            printf("the radar is not connected,please check\n");
            ptr->detect_cnt = 0;
        }
        usleep(200000);
    }
}