#include <stdlib.h>  
#include <stdio.h> 
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
#include "imu.h"

FILE *fp_2;
IMUModule::IMUModule() {
    pthread_mutex_init(&mutex, nullptr);
    pthread_create(&imu_thread, NULL, thread_function, this);
	fp_2=fopen("imu1.txt","w");
};

IMUModule::~IMUModule() {
    thread_running = false;
    close_imu();
    pthread_mutex_destroy(&mutex);
    pthread_join(imu_thread, NULL);
};

speed_t IMUModule::getBaudrate(int baudrate)
{
	switch(baudrate) {
	case 0: return B0;
	case 50: return B50;
	case 75: return B75;
	case 110: return B110;
	case 134: return B134;
	case 150: return B150;
	case 200: return B200;
	case 300: return B300;
	case 600: return B600;
	case 1200: return B1200;
	case 1800: return B1800;
	case 2400: return B2400;
	case 4800: return B4800;
	case 9600: return B9600;
	case 19200: return B19200;
	case 38400: return B38400;
	case 57600: return B57600;
	case 115200: return B115200;
	case 230400: return B230400;
	case 460800: return B460800;
	case 500000: return B500000;
	case 576000: return B576000;
	case 921600: return B921600;
	case 1000000: return B1000000;
	case 1152000: return B1152000;
	case 1500000: return B1500000;
	case 2000000: return B2000000;
	case 2500000: return B2500000;
	case 3000000: return B3000000;
	case 3500000: return B3500000;
	case 4000000: return B4000000;
	default: return -1;
	}
}

int IMUModule::open_tty(char const *path, int baudrate, int flags)
{
	int fd;
	speed_t speed;

	if ((speed = getBaudrate(baudrate)) == -1) {
		return -1;
	}
	if ((fd = open(path, O_RDWR )) == -1){
		return -1;
	}

	struct termios cfg;
	if (tcgetattr(fd, &cfg)){
		close(fd);
		return -1;
	}
	cfmakeraw(&cfg);
	cfsetispeed(&cfg, speed);
	cfsetospeed(&cfg, speed);
	cfg.c_cc[VMIN] = 1;
	cfg.c_cc[VTIME] = 1;
	if(flags == 1)
	{
	cfg.c_cflag |= PARENB;
	cfg.c_cflag &= ~PARODD;
	}
	//cfg.c_cflag &= ~CSTOPB;
	//cfg.c_cflag &= ~CSIZE;
	//cfg.c_cflag |= CS8;
		if (tcsetattr(fd, TCSANOW, &cfg)){
		close(fd);
		return -1;
	}
	
	printf("imu :open %s ok,fd=%d\n",path,fd); 
	return fd;
}
int IMUModule::toBCD(int x){
	return ((x%256)/16)*10+(x%16);
}
#define PI 3.1415926
int IMUModule::check_CRC(char* p,int len){
	int i,n;
	int tmp = 0xFFFF;
	for(n = 0; n < (len-2); n++){
		tmp = p[n] ^ tmp;
		for(i = 0;i < 8;i++){ 
			if(tmp & 0x01){
				 tmp = tmp >> 1;
				 tmp = tmp ^ 0xa001;
			}
			else{
					tmp = tmp >> 1;
			}
		}
  }
	if((p[len-2] | p[len-1] << 8) == tmp)
		return 1;
	else
		return 0;
}
int IMUModule::check_CS(char* p,int len){
	int sum=0;
	for(int j=0;j<len-1;j++){
		sum+=p[j];
	}
	if((sum%0x100)==p[len-1])
		return 1;
	else
		return 0;
}
#define ACCEL_G	9.80665 
void IMUModule::handle_frame(char* p,int len,IMUModule* ptr){
//	int checksum;
	long timestamp;
	//int addr=p[1];
	//int cmd=p[2];
	int temp=0;
	if(commu_flag == 1)
	{///if the proto is modbus
        temp = p[3] | p[4]<<8 | p[5]<<16 |	p[6]<<24 ;
	z_angular_velocity =  (temp-150000)/100.0;

	temp = p[7] | p[8]<<8 |	p[9]<<16 |	p[10]<<24 ;
	forward_linear_accel =  (temp-20000)/1000.0;

	temp = p[11] | p[12]<<8 | p[13]<<16 | p[14]<<24 ;
	z_angle = (temp-18000)/100.0;
	
	}
	else
	{
	z_angular_velocity=(float)((p[3]&0x10)==0x10?
	(0-toBCD(p[3]&0x0f)*10000-toBCD(p[4])*100-toBCD(p[5])):
	(toBCD(p[3]&0x0f)*10000+toBCD(p[4])*100+toBCD(p[5])))/100;
	
	//m/s2
	forward_linear_accel=(float)((p[6]&0x10)==0x10?
	(0-toBCD(p[6]&0x0f)*10000-toBCD(p[7])*100-toBCD(p[8])):
	(toBCD(p[6]&0x0f)*10000+toBCD(p[7])*100+toBCD(p[8])))/1000;
	
	z_angle=(float)((p[9]&0x10)==0x10?
	(0-toBCD(p[9]&0x0f)*10000-toBCD(p[10])*100-toBCD(p[11])):
	(toBCD(p[9]&0x0f)*10000+toBCD(p[10])*100+toBCD(p[11])))/100;
	}

	imu_cnt++;
	if((imu_cnt%1)==0){
		
        long timestamp_us = get_current_time_us();
        double acc_x = forward_linear_accel * ACCEL_G;
        double acc_y = 0.;
        double vel_z_axis = 0.-z_angular_velocity/180.*PI;
		
	imu_output_cnt++;
	if(imu_output_cnt>500)
	{
	printf("imu data received=forward_vel=%f    z_vel=%f    z_angle=%f\n",forward_linear_accel,z_angular_velocity,z_angle);
	imu_output_cnt = 0;
	}
		z_angle =  z_angle/180.*PI;
		//fprintf(fp_2,"%ld,%s,%.2f,%.4f\n",timestamp_us,"1",z_angle,z_angular_velocity);
////////////////
        //四个参数分别是时间戳，x加速度，y加速度，z轴角速度
        pthread_mutex_lock(&mutex);
		if(p_msg_queue != nullptr) {
            ImuData2D data(timestamp_us, acc_x, acc_y, vel_z_axis);
			data.angle_z = z_angle;
			std::vector<char> tmp;
			data.to_char_array(&tmp);
			p_msg_queue->send(channel, tmp.data(), tmp.size());
        }
        pthread_mutex_unlock(&mutex);
    }
}

void *IMUModule::thread_function(void *param) {

    IMUModule* ptr = (IMUModule *) param;

    //这里是线程参数
    //初始化并启动IMU
    //在这里用 ptr->xxx 的形式访问你定义的中间变量，就像下面while那一行里那样
    //------------------
    int     byte_index=0,len=0,new_frame=0;
    char    in[100];
    char	frame[100];
    char	dev_name[200]= "/dev/ttyTHS2";
    char	commu_type[30];
    int read_imu_error_cnt = 0;

    //get_configs()->get_string("imu", "dev_name", dev_name, nullptr);
    get_configs()->get_string("imu", "commu_type", commu_type, nullptr);
    if(strcmp(commu_type,"modbus")==0)
    ptr->commu_flag = 1;
    else
    ptr->commu_flag = 0;

    ptr->fd=-1;
	sleep(3);
    int baund_rate = get_configs()->get_int("imu", "imu_baudrate", nullptr);
    if (baund_rate<0) baund_rate = 115200;
    printf("imu name =%s,baund_rate=%d commu_flag=%s----%d\n",dev_name,baund_rate,commu_type,ptr->commu_flag);
    while(ptr->fd<=0)
    {
        usleep(1000000);
        ptr->fd=ptr->open_tty(dev_name, baund_rate,ptr->commu_flag);
    }
	//printf("open_tty(\"/dev/ttyUSB0\",115200,0),fd=%d\n",fd);
    if(ptr->fd<=0){
        printf("IMU failed!!!!!!!!!!!!!!!!!!!\n");
        return 0;
    }
    fcntl(ptr->fd,F_SETFL,FNDELAY);////set com in no block mode.
    usleep(50000);
    printf("IMU start\n");
    bool flag_check = false;
    int ret = 0;
	/*char send_buf[8];
	memset(send_buf, 0, 8);
	send_buf[0] = 0x01;
	send_buf[1] = 0x03;
	send_buf[2] = 0x00;
	send_buf[3] = 0x02;
	send_buf[4] = 0x00;
	send_buf[5] = 0x06;
	send_buf[6] = 0x64;
	send_buf[7] = 0x08;*/
    while(ptr->thread_running) 
    {
        usleep(20000);
        flag_check = false;
        if(ptr->commu_flag == 1)///if the imu commu type is modbus 
        {
        //usleep(10000);////read the imu data in 100 Hz speed rate.
        //循环读取IMU数据
        //write(ptr->fd,send_buf,8);
        ret=read(ptr->fd,in,100);
        //printf("recv data size=%d\n",ret);
        if(ret>0)
        {
        for(int i=0;i<ret;i++)
        {
            if(in[i]==0x01&&in[i+1]==0x03)
            {	//收到H
            len = in[i+2]+5;
            //printf("data recv len=%d\n",len);
            for(int j=0;i+j<ret;j++)
            frame[j]=in[i+j];
            if(ptr->check_CRC(frame,len)){
            flag_check = true;
            ptr->handle_frame(frame,len, ptr);
            
            }else{
                printf("rcv 1 frame.wrong CS==len=%d\n",len);
            }
                memset(frame,0,100);
            
            }
           }
        }
        if(flag_check) read_imu_error_cnt = 0;
        else read_imu_error_cnt++;
        }
        else////if the commu type is not modbus
        {
        ret=read(ptr->fd,in,100);
        //printf("\nIMU data reading!!!!!!!!!!!!!!ret=%d\n",ret);
        if(ret>0){
        
        for(int i=0;i<ret;i++){
            if(new_frame==0){
            if(in[i]==0x68){	//收到H
                byte_index=0;
                len=0;
                new_frame=1;
                }
                }else{
                //printf("IMU data processing!!!!!!!!!!!!!!!!!!!\n");
                frame[byte_index]=in[i];
                byte_index++;
                if(byte_index==1)len=in[i];
                if(byte_index==len){
                
                if(ptr->check_CS(frame,len)){
                ptr->handle_frame(frame,len, ptr);
                flag_check = true;
                }else{
                printf("rcv 1 frame.wrong CS\n");
                }
                memset(frame,0,100);
                new_frame=0;
						
                }
                }
            }
			
        }
        if(flag_check) read_imu_error_cnt = 0;
        else read_imu_error_cnt++;
        }///else////if the commu type is not modbus
		//printf("cur read_imu_error_cnt=%d\n",read_imu_error_cnt);
        if(read_imu_error_cnt>=128)
        {
            ptr->imu_disconnect = true;
            read_imu_error_cnt = 128;
            //printf("the imu is disconnect\n");
        }
        else
        ptr->imu_disconnect = false;
    }
    return nullptr;
};

void IMUModule::close_imu() {
    //在这里关闭imu
    //在这里直接用变量名就可以访问你定义的中间变量
    //如直接调用 thread_running
    //------------------
	close(fd);
};

void IMUModule::set_target(MsgQueue * target, int channel) {
    pthread_mutex_lock(&mutex);
	p_msg_queue = target;
    this->channel = channel;
    pthread_mutex_unlock(&mutex);
}
    
