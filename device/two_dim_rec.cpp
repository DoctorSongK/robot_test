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

#include "two_dim_rec.h"
//#include "../logic/agv.h"
#include "../common/time.h"
#include "../common/configuration.h"

Two_Dim_recModule::Two_Dim_recModule()
{
	pthread_mutex_init(&map_mutex, nullptr);
    pthread_create(&device1_thread, NULL, thread1_function, this);
    pthread_create(&device2_thread, NULL, thread2_function, this);
    pthread_create(&monitor_2d_code_thread, NULL, monitor_2d_code_state, this);
};

Two_Dim_recModule::~Two_Dim_recModule()
{
    close_device();
	pthread_mutex_destroy(&map_mutex);
    pthread_join(device1_thread, NULL);
	pthread_join(device2_thread, NULL);
    pthread_join(monitor_2d_code_thread, NULL);
};

void Two_Dim_recModule::close_device()
{
    thread1_running = false;
    thread2_running = false;
    close(sockfd_1);
    close(sockfd_2);
};
int Two_Dim_recModule::device1_reconnect()
{
    close(sockfd_1);
    usleep(5000);
    printf("device 1 reconnect\n");
    int sockfd;
    if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
    printf("create socket 1 error: %s\n", strerror(errno));
    return -1;
    }
    int port = get_configs()->get_int("two_dimention_code", "locate_port", nullptr);
    char host_name[20];
    get_configs()->get_string("two_dimention_code", "locate_ip", host_name, nullptr);
    printf("two dimention ip=%s,port=%d\n",host_name,port);
    struct sockaddr_in servaddr;
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(port);
    if (inet_pton(AF_INET, host_name, &servaddr.sin_addr) <= 0)
    {
    printf("inet_pton 1 error for %s\n", host_name);
    return -1;
    }
    int old_option = 0;
    int new_option = 0;
    old_option = fcntl(sockfd, F_GETFL,0);
    new_option = old_option | O_NONBLOCK;

    if (connect(sockfd, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0)
    {
    printf("connect 1 error: %s(errno: %d) \n", strerror(errno), errno);
    return -1;
    }
    fcntl(sockfd, F_SETFL, new_option);
    sockfd_1 = sockfd;
    printf("device 1 locate reconnect success\n");
    return 1; 
}
int Two_Dim_recModule::device2_reconnect()
{
    close(sockfd_2);
    usleep(5000);
    printf("device 2 reconnect\n");
    int sockfd;
    if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
    printf("create socket 2 error: %s\n", strerror(errno));
    return -1;
    }
    int port = get_configs()->get_int("two_dimention_code", "second_port", nullptr);
    char host_name[20];
    get_configs()->get_string("two_dimention_code", "second_ip", host_name, nullptr);
    printf("two dimention ip=%s,port=%d\n",host_name,port);
    struct sockaddr_in servaddr;
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(port);
    if (inet_pton(AF_INET, host_name, &servaddr.sin_addr) <= 0)
    {
    printf("inet_pton 2 error for %s\n", host_name);
    return -1;
    }
    int old_option = 0;
    int new_option = 0;
    old_option = fcntl(sockfd, F_GETFL,0);
    new_option = old_option | O_NONBLOCK;

    if (connect(sockfd, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0)
    {
    printf("connect 2 error: %s(errno: %d) \n", strerror(errno), errno);
    return -1;
    }
    fcntl(sockfd, F_SETFL, new_option);
    sockfd_2 = sockfd;
    printf("device 2 reconnect success\n");
    return 1; 
}
void *Two_Dim_recModule::thread1_function(void *param)
{
    Two_Dim_recModule *ptr = (Two_Dim_recModule *)param;
    struct sockaddr_in servaddr;
    int sockfd;
    if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    printf("create socket 1 error: %s\n", strerror(errno));

    int port = get_configs()->get_int("two_dimention_code", "locate_port", nullptr);
    int device_enable = get_configs()->get_int("two_dimention_code", "locate_enable", nullptr);
	double fix_ang = get_configs()->get_float("two_dimention_code", "locate_fixang", nullptr);
    ptr->thread1_running = (device_enable>0)?true:false;
    char host_name[20];
    get_configs()->get_string("two_dimention_code", "locate_ip", host_name, nullptr);
    printf("two dimention 1 ip=%s,port=%d,enable=%d,fix_ang=%f\n",host_name,port,ptr->thread1_running,fix_ang);
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(port);
    if (inet_pton(AF_INET, host_name, &servaddr.sin_addr) <= 0)
    printf("inet_pton 1 error for %s\n", host_name);
    
    int old_option = 0;
    int new_option = 0;
    old_option = fcntl(sockfd, F_GETFL,0);
    new_option = old_option | O_NONBLOCK;
    int read_error_cnt = 0;
    if(device_enable>0)
    {
    ptr->device1_disconnect = true; 
    if (connect(sockfd, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0)
    {
    printf("connect 1 error: %s(errno: %d) \n", strerror(errno), errno);
    read_error_cnt = 300;
    }
    fcntl(sockfd, F_SETFL, new_option);
    if(read_error_cnt==0)ptr->device1_disconnect = false; 
    }
    else
    ptr->device1_disconnect = false; 

    ptr->sockfd_1 = sockfd;
    usleep(1000000);
    printf("device 1 start  read_error_cnt=%d\n",read_error_cnt);
    char send_buf[2];
    char buf_rev[30];
    memset(send_buf, 0, 2);
    send_buf[0] = 0xC8;
    send_buf[1] = 0x37;
    bool read_success = false;
    long    t_timestamp;
    float t_agv_x;
    float t_agv_y;
    float t_agv_theta;
    int t_tag_num;
    int rec_len;
    //get_global_agv_instance()->load_two_dims_file();
    while (ptr->thread1_running)
    {

		if(read_error_cnt>=70) ////if the laser connect failed, then reconnect the laser
        {
            ptr->device1_disconnect = true;
            if(ptr->device1_reconnect()==1)
            {
                usleep(100000);
                read_error_cnt=0;
                ptr->device1_disconnect = false;;
                continue;
            }
            else
            {
                usleep(2000000);
                printf("excuting the device 1 reconnet thread!!!!!!!!!\n");
                continue;
            }
    
        }
        usleep(10000);
        read_success = false;
        write(ptr->sockfd_1,send_buf,2);
		usleep(10000);
		if ((rec_len = recv(ptr->sockfd_1, buf_rev, sizeof(buf_rev), 0)) ==-1)
        {
        	read_error_cnt++;
            read_success = false;
        }
        else
        {
        	read_error_cnt = 0;
            read_success = true;
        }
        ptr->detect_2d_code1_cnt = 0;
        ptr->td_code_1_error = false;
        if(read_success)
        {
            if((buf_rev[0]&0x02)>0||(buf_rev[1]&0x40)==0) 
            {
                //printf("no data received\n");
            	continue;////if no data received ,then continue the next read;
                memset(buf_rev,0,sizeof(buf_rev));
            }
            else
            {
                t_timestamp = get_current_time_us();
                //for (int i=0;i<rec_len;i++)
                //printf("%x ",buf_rev[i]);
                //printf("\n");
                t_agv_x = ((buf_rev[4]&0x40)>0)?((short)(buf_rev[4]*0x80+buf_rev[5]+0xc000)):((short)(buf_rev[4]*0x80+buf_rev[5]));
                t_agv_y = ((buf_rev[6]&0x40)>0)?((short)(buf_rev[6]*0x80+buf_rev[7]+0xc000)):((short)(buf_rev[6]*0x80+buf_rev[7]));
                t_agv_theta = ((short)(buf_rev[10]*0x80+buf_rev[11]))+(fix_ang*10);
                t_tag_num = (buf_rev[13]&0xf)*0x4000*0x4000+buf_rev[14]*0x80*0x4000+buf_rev[15]*0x4000+buf_rev[16]*0x80+buf_rev[17];
                t_agv_x = (float)t_agv_x*0.001*0.1;////the rate is 0.1
                t_agv_y = (float)t_agv_y*0.001*0.1;
                t_agv_theta = -(float)t_agv_theta/180*M_PI*0.1;
                if (t_agv_theta>M_PI) t_agv_theta -= 2*M_PI;
                if (t_agv_theta<-M_PI) t_agv_theta += 2*M_PI;
                //printf("recv locate tagnum=%d,posx=%f,y=%f,theta=%f\n",t_tag_num,t_agv_x,t_agv_y,t_agv_theta);
                memset(buf_rev,0,sizeof(buf_rev));
                Two_dim_data my_two_dim(t_timestamp,t_agv_x,t_agv_y,t_agv_theta,t_tag_num);
                my_two_dim.device_send = TWO_CODE1;
                pthread_mutex_lock(&ptr->map_mutex);
                if (ptr->p_msg_queue != nullptr)
                {
                    std::vector<char> tmp;
                    my_two_dim.to_char_array(&tmp);
                    ptr->p_msg_queue->send(ptr->channel, tmp.data(), tmp.size());
                	//printf("device 1 receiced the dimetion code \n");
                }
                pthread_mutex_unlock(&ptr->map_mutex);
            }
        }		
        else
        continue;

    }
 printf("device 1 thread quit!\n");
 return nullptr;
};

void *Two_Dim_recModule::thread2_function(void *param)
{
    Two_Dim_recModule *ptr = (Two_Dim_recModule *)param;
    struct sockaddr_in servaddr;
    int sockfd;
    if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    printf("create socket 1 error: %s\n", strerror(errno));

    int port = get_configs()->get_int("two_dimention_code", "second_port", nullptr);
    char host_name[20];
    get_configs()->get_string("two_dimention_code", "second_ip", host_name, nullptr);
    int device_enable = get_configs()->get_int("two_dimention_code", "second_enable", nullptr);
    double fix_ang = get_configs()->get_float("two_dimention_code", "second_fixang", nullptr);
    ptr->thread2_running = (device_enable>0)?true:false;
    printf("two dimention 2 ip=%s,port=%d,enable=%d,fix_ang=%f\n",host_name,port,ptr->thread2_running,fix_ang);
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(port);
    if (inet_pton(AF_INET, host_name, &servaddr.sin_addr) <= 0)
    printf("inet_pton 2 error for %s\n", host_name);

    int old_option = 0;
    int new_option = 0;
    old_option = fcntl(sockfd, F_GETFL,0);
    new_option = old_option | O_NONBLOCK;
    int read_error_cnt = 0;
    if(device_enable>0)
    {
    ptr->device2_disconnect = true; 
    if (connect(sockfd, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0)
    {
    printf("connect 2 error: %s(errno: %d) \n", strerror(errno), errno);
    read_error_cnt = 300;
    }
    fcntl(sockfd, F_SETFL, new_option);
    if(read_error_cnt==0)ptr->device2_disconnect = false; 
    }
    else
    ptr->device2_disconnect = false; 

    ptr->sockfd_2 = sockfd;
    usleep(1000000);

    printf("device 2 start  read_error_cnt=%d\n",read_error_cnt);
    char send_buf[2];
	char buf_rev[30];
	memset(send_buf, 0, 2);
	send_buf[0] = 0xC8;
	send_buf[1] = 0x37;
    bool read_success = false;
    long    t_timestamp;
    float t_agv_x;
    float t_agv_y;
    float t_agv_theta;
    int t_tag_num;
	int rec_len;
    while (ptr->thread2_running)
    {
		if(read_error_cnt>=100) ////if the laser connect failed, then reconnect the laser
        {
            ptr->device2_disconnect = true;
            if(ptr->device2_reconnect()==1)
            {
                usleep(100000);
                read_error_cnt=0;
                ptr->device2_disconnect = false;;
                continue;
            }
            else
            {
                usleep(2000000);
                printf("excuting the device 2 reconnet thread!!!!!!!!!\n");
                continue;
            }
    
        }
        usleep(10000);
        read_success = false;
        write(ptr->sockfd_2,send_buf,2);
		usleep(10000);
		if ((rec_len = recv(ptr->sockfd_2, buf_rev, sizeof(buf_rev), 0)) ==-1)
        {
        	read_error_cnt++;
            read_success = false;
        }
        else
        {
        	read_error_cnt = 0;
            read_success = true;
        }
        ptr->detect_2d_code2_cnt = 0;
        ptr->td_code_2_error = false;
        if(read_success)
        {
            if((buf_rev[0]&0x02)>0||(buf_rev[1]&0x40)==0) 
            {
                Two_dim_data my_two_dim(0,0,0,0,0);
                my_two_dim.device_send = TWO_CODE2;
                my_two_dim.state = -1;
                if (ptr->p_msg_queue != nullptr)
                {
                    std::vector<char> tmp;
                    my_two_dim.to_char_array(&tmp);
                    ptr->p_msg_queue->send(ptr->channel, tmp.data(), tmp.size());
                	//printf("device 2 receiced the dimetion code \n");
                }
                //pthread_mutex_unlock(&ptr->map_mutex);
            	continue;////if no data received ,then continue the next read;
                memset(buf_rev,0,sizeof(buf_rev));
            }
            else
            {
                t_timestamp = get_current_time_us();
                t_agv_x = ((buf_rev[4]&0x40)>0)?((short)(buf_rev[4]*0x80+buf_rev[5]+0xc000)):((short)(buf_rev[4]*0x80+buf_rev[5]));
                t_agv_y = ((buf_rev[6]&0x40)>0)?((short)(buf_rev[6]*0x80+buf_rev[7]+0xc000)):((short)(buf_rev[6]*0x80+buf_rev[7]));
                t_agv_theta = ((short)(buf_rev[10]*0x80+buf_rev[11]))+(fix_ang*10);
                t_tag_num = (buf_rev[13]&0xf)*0x4000*0x4000+buf_rev[14]*0x80*0x4000+
                buf_rev[15]*0x4000+
                buf_rev[16]*0x80+buf_rev[17];
                
                t_agv_x = (float)t_agv_x*0.001*0.1;/////0.1 is the rate . 
                t_agv_y = (float)t_agv_y*0.001*0.1;///0.1 the same to 
                t_agv_theta = -(float)t_agv_theta/180*M_PI*0.1;///0.1 the same to 
                
                if (t_agv_theta>M_PI) t_agv_theta -= 2*M_PI;
                if (t_agv_theta<-M_PI) t_agv_theta += 2*M_PI;
                //printf("recv second tagnum=%d,posx=%f,y=%f,theta=%f,buf_rev[17]=%d\n",t_tag_num,t_agv_x,t_agv_y,t_agv_theta,buf_rev[17]);
                memset(buf_rev,0,sizeof(buf_rev));
                /////////because the up two dim is opposite to down two dim code
                Two_dim_data my_two_dim(t_timestamp,t_agv_x,-t_agv_y,-t_agv_theta,t_tag_num);
                my_two_dim.device_send = TWO_CODE2;
                my_two_dim.state = 1;
                pthread_mutex_lock(&ptr->map_mutex);
                if (ptr->p_msg_queue != nullptr)
                {
                    std::vector<char> tmp;
                    my_two_dim.to_char_array(&tmp);
                    ptr->p_msg_queue->send(ptr->channel, tmp.data(), tmp.size());
                	//printf("device 2 receiced the dimetion code \n");
                }
                pthread_mutex_unlock(&ptr->map_mutex);
            }
        }		
        else
        continue;

    }
    printf("device 2 thread quit!\n");
    return nullptr;
};



void Two_Dim_recModule::set_target(MsgQueue *target, int channel)
{
    pthread_mutex_lock(&map_mutex);
    p_msg_queue = target;
    this->channel = channel;
    pthread_mutex_unlock(&map_mutex);
};
void *Two_Dim_recModule::monitor_2d_code_state(void *param)
{
    Two_Dim_recModule *ptr = (Two_Dim_recModule *)param;
    int device1_enable = get_configs()->get_int("two_dimention_code", "locate_enable", nullptr);
    int device2_enable = get_configs()->get_int("two_dimention_code", "second_enable", nullptr);
    bool check_1 = (device1_enable>0)?true:false;
    bool check_2 = (device2_enable>0)?true:false;
    int num = 0;
    sleep(3);
    while (check_1||check_2)
    {
        ptr->detect_2d_code1_cnt++;
        if(ptr->detect_2d_code1_cnt>=10&&check_1) 
        {
            ptr->td_code_1_error = true;
            printf("the 2d code one not connected,please check\n");
            ptr->detect_2d_code1_cnt = 0;
        }
        ptr->detect_2d_code2_cnt++;
        if(ptr->detect_2d_code2_cnt>=10&&check_2) 
        {
            ptr->td_code_2_error = true;
            printf("the 2d code two not connected,please check\n");
            ptr->detect_2d_code2_cnt = 0;
        }
        usleep(200000);
    }
}