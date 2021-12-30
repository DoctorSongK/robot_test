#include "../logic/agv.h"
#include "log_ck_up_api.h"

std::string base64_chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
                           
int LOG_CK_UPModule::byte2int(char *buf)
{
    int value = 0;
    for (int i = 0; i < 4; i++)
    {
        value |= ((int)(buf[i] & 0xff)) << (8 * (3-i));
    }
    return value;
}
void LOG_CK_UPModule::int2byte(char *buf, int value)
{
    for (int i = 0; i < 4; i++)
    {
        buf[i] = (char)((value >> 8 * i) & 0xff);
    }
}
LOG_CK_UPModule::LOG_CK_UPModule() {
    pthread_mutex_init(&mutex, nullptr);
    pthread_create(&main_thread, NULL, task_robot_task, this);
};

LOG_CK_UPModule::~LOG_CK_UPModule() {
    thread_running = false;
    pthread_mutex_destroy(&mutex);
    pthread_join(main_thread, NULL);
};


int LOG_CK_UPModule::SocketConnected(int sock) 
{ 
    if(sock<=0) 
    return 0; 
    struct tcp_info info; 
    int len=sizeof(info); 
    getsockopt(sock, IPPROTO_TCP, TCP_INFO, &info, (socklen_t *)&len); 
    if((info.tcpi_state==TCP_ESTABLISHED)) 
    return 1; 
    else 
    return 0; 
}
void * LOG_CK_UPModule::pthread_func_task(void* param)// pthread_func(int* p_client_sockfd)            
{
    LOG_CK_UPModule* ptr = (LOG_CK_UPModule *) param;
    //int client_sockfd = ptr->client_sockfd;
    int clnt_sock = *((int*)param);

    int ret;
    char data[30];
    //printf("当前介入的任务客户端=%d\n",client_count_task);
    while(1)
    {
    
    ret = recv(clnt_sock,data,16,0);
    if(ret>=16)//////通信头部结构大小16子节
    {
    //printf("接收task端口的信息%d\n",clnt_sock);
    ptr->buffer_analysize(clnt_sock,ret,&data[0],ptr->Port_task);
    //printf("stop接收task端口的信息%d\n",clnt_sock);
    }
    else if(-1 == ret)
    {
    break;
    }
    usleep(100*1000);///////周期循环200ms
    if(ptr->SocketConnected(clnt_sock)==0) break;
    }
    close(clnt_sock);
    printf("任务客户端退出\n");
    pthread_exit(0);
}
void *LOG_CK_UPModule::task_robot_task(void* param)
{
    LOG_CK_UPModule* ptr = (LOG_CK_UPModule *) param;
    int ret;
    int client_count_task=0;
    int count_num = 0;
    char buf[20];
    int keepalive = 1; // 开启keepalive属性
    int keepidle = 60; // 如该连接在60秒内没有任何数据往来,则进行探测
    int keepinterval = 5; // 探测时发包的时间间隔为5 秒
    int keepcount = 3; // 探测尝试的次数.如果第1次探测包就收到响应了,则后2次的不再发.
    struct sockaddr_in addr_robottask_server;
    struct sockaddr_in client_addr;
    socklen_t client_addrlen = sizeof(struct sockaddr_in);
    int socket_robot_task=socket(AF_INET,SOCK_STREAM,0);

    setsockopt(socket_robot_task, SOL_SOCKET, SO_KEEPALIVE, (void *)&keepalive , sizeof(keepalive ));
    setsockopt(socket_robot_task, SOL_TCP, TCP_KEEPIDLE, (void*)&keepidle , sizeof(keepidle ));
    setsockopt(socket_robot_task, SOL_TCP, TCP_KEEPINTVL, (void *)&keepinterval , sizeof(keepinterval ));
    setsockopt(socket_robot_task, SOL_TCP, TCP_KEEPCNT, (void *)&keepcount , sizeof(keepcount ));
    int len;
    memset(&addr_robottask_server,0,sizeof(struct sockaddr_in));
    addr_robottask_server.sin_family=AF_INET;

    addr_robottask_server.sin_port=htons(ptr->Port_task);
    addr_robottask_server.sin_addr.s_addr=htonl(INADDR_ANY);
    len=sizeof(addr_robottask_server);
    if(bind(socket_robot_task,(struct sockaddr*)&addr_robottask_server,sizeof(addr_robottask_server))<0)
    {
    printf("task_robot_task bind error:\n");
    return NULL;
    }
    ret = listen(socket_robot_task,10);
    if(-1 == ret)
    {
    printf("task_robot_task listen\n");
    return NULL;
    }
    string version = CUR_VERSION;
    printf("cur software version is %s\n",version.c_str());
    while(1)
    {
    usleep(100*1000);
    ptr->client_sockfd = accept(socket_robot_task,(struct sockaddr*)&client_addr,&client_addrlen);
    if(ptr->client_sockfd > 0)
    {
    printf("TASK CHECK CLIENT IP is %s\n",inet_ntoa(client_addr.sin_addr));
    printf("TASK CHECK CLIENT Port is %d\n",ntohs(client_addr.sin_port));
    pthread_t pt;
    printf("s_addr--%d--%d\n",(int)client_addr.sin_addr.s_addr,ptr->client_sockfd);
    int *p_socket = new int(ptr->client_sockfd);
    ret = pthread_create(&pt,NULL,pthread_func_task,(void *)p_socket);
    if(0 != ret)
    printf("ip:%s connect failed",inet_ntoa(client_addr.sin_addr));
    }
    }
    close(socket_robot_task);
    return NULL; 
}
void LOG_CK_UPModule::set_target(MsgQueue * target, int channel) {
    pthread_mutex_lock(&mutex);
    p_msg_queue = target;
    this->channel = channel;
    pthread_mutex_unlock(&mutex);
}
    
int LOG_CK_UPModule::buffer_analysize(int socket_hd,int recv_len,char *recv_buf,int port)
{
    int iLength; //数据长度
    T_COMMU_HEADER my_header;////通信头部结构体
    char swritebuf[500];////接收的JSON字符串
    char buf_send[200000];////发送的JSON字符串
    int cmd_type=0;
    int cmd_ret;
    int lift_c_typ;
    my_header.herder_syn = 0X5A;////同步头
    my_header.version_num = 0X01;/////版本号
    my_header.serial_num1 = 0X00;/////编号1
    my_header.serial_num2 = 0X00;/////编号2
    int json_data_len;
    char json_data_char[4];

    const char * log_data_temp;
    char * log_data;
    char * update_data;
    int dispose_data_len;
    Position cur_pose;
    cJSON *root = NULL;
    cJSON *root_recv = NULL;////接收json根节点
    cJSON *addobj = NULL;
    cJSON *array = NULL;
    char *jsonout = NULL;
    cJSON *item = NULL;
    string xml_str, json_str;
    char *update_path;

    string version = CUR_VERSION;
    MapSharedPointer p_map;
    MoveInstruction movet;
    ifstream inf;
    ofstream outf;
    ostringstream oss;
    FILE *Update_File;
    if((recv_len)>0)
    {////////////////从接受的数据中解析出上位机下发的指令
    for (int i=0;i<recv_len;i++)
    {
        if (0x5a == recv_buf[i])/////起始码
        {
            if((0x01 == recv_buf[i+1])&&(0x00 == recv_buf[i+2])) //收到请求信号0100
            {
                //iLength = ((int)(recv_buf[i+7])+((int)(recv_buf[i+6]))<<8+((int)(recv_buf[i+5]&0xff))<<16+((int)(recv_buf[i+4]&0xff))<<24);//////////收数据长度
                iLength = byte2int(&recv_buf[i+4]);
                //iLength = (int)recv_buf[i+7]+256*(int)recv_buf[i+6]+65536*(int)recv_buf[i+5]+16777216*(int)recv_buf[i+4];//////////
                cmd_type =(((int)(recv_buf[i+8]&0xff))<<8)|((int)(recv_buf[i+9]&0xff));////////////收命令类型
                //printf("recv_buf[4]=%d>>recv_buf[5]=%d>>recv_buf[6]=%d>>recv_buf[7]=%d>",recv_buf[i+4],recv_buf[i+5],recv_buf[i+6],recv_buf[i+7]);
                //printf("recv_buf[8]=%x>>recv_buf[9]=%x>>>cmd_type====%d iLength=%d\n",recv_buf[i+8],recv_buf[i+9],cmd_type,iLength);
                /*if(cmd_type!=robot_config_uploadmap_req&&(cmd_type!=robot_program_update_req)&&(cmd_type!=robot_config_ini_config_pull_req)
                &&(cmd_type!=robot_config_ini_config_push_req)&&(cmd_type!=robot_obsmap_update_req))////如果指令不是上传到控制器地图指令
                {
                dispose_data_len = my_read(socket_hd,swritebuf,iLength);
                if(dispose_data_len!=iLength){printf("接收指令数据不正确\n");return false;}
                }*/
                break;
            }

        }////if (0x02 == recv_buf[i])

    }/////for (int i=0;i<iNum;i++)
    }/////////if(recvfrom(sock_createmap,
    else
    return -1;
    char   filename[128];
    char   runpath[128];
    int cap_point_index;
    int route_index;
    double rotate_angle;
    switch(cmd_type)
    {
    case robot_config_ini_config_push_req:
        printf("接受config下发任务指令\n");
        
        log_data = (char*)malloc(iLength+1);
        memset(log_data,0,iLength+1);
        json_data_len = my_read(socket_hd,log_data,iLength);
        if(json_data_len!=iLength)
        {get_log_instance()->task_msg("接收config file不正确");free(log_data);return false;}
        root_recv=cJSON_Parse(log_data);
        if(!root_recv) 
        {get_log_instance()->task_msg("cur json context error");free(log_data);return false;}
        getcwd(runpath, 127);
        sprintf(filename,"%s/config.txt",runpath);
        outf.open(filename);
        
        item=cJSON_GetObjectItem(root_recv,"config_file_data");
        if(item!=NULL)
            json_str = item->valuestring;
        outf << json_str;
        outf.close();
        usleep(10000);
        //////////////////////////////回传指令赋值
        cmd_ret = cmd_type+10000;
        my_header.cmdtype_HB = HIBYTE(cmd_ret);////命令类型高字节
        my_header.cmdtype_LB = LOBYTE(cmd_ret);////命令类型低字节
        memcpy(&buf_send[0],&my_header,sizeof(my_header));//////////收数据长度
        
        dispose_data_len = my_write(socket_hd,buf_send,16);
    
        if(dispose_data_len>=(16)) 
        printf("config data file %s saved success \n","config.txt");

        cJSON_Delete(root_recv);usleep(1000);

    break;
    case robot_config_ini_config_pull_req:
        printf("接受config上传任务指令\n");
        getcwd(runpath, 127);
        sprintf(filename,"%s/config.txt",runpath);
        //inf.open("/usr/local/ale_bot/config.txt");
        inf.open(filename);
        oss.str("");
        oss << inf.rdbuf();
        xml_str = oss.str();
    
        log_data_temp = xml_str.c_str();
        if(strlen(log_data_temp)<=20) 
        {
            cmd_ret = cmd_type+10000;
            my_header.cmdtype_HB = HIBYTE(cmd_ret);////命令类型高字节
            my_header.cmdtype_LB = LOBYTE(cmd_ret);////命令类型低字节
            memcpy(&buf_send[0],&my_header,sizeof(my_header));//////////收数据长度
            send(socket_hd,buf_send,16,0); 
            get_log_instance()->task_msg("当前config文件有错误\n");
            return -1;
        }

        root = cJSON_CreateObject();////////创建根节点
        cJSON_AddStringToObject(root,"config_file_name","config.txt");
        cJSON_AddStringToObject(root,"config_file_data",log_data_temp);
        jsonout=cJSON_Print(root);
        //////////////////////////////回传指令赋值
        cmd_ret = cmd_type+10000;
        printf("上传config文件大小=%d---jsonout=%d\n",strlen(log_data_temp),strlen(jsonout));
        json_data_len = strlen(jsonout);/////数据长度
        log_data = (char*)malloc(json_data_len+20);
        memset(log_data,0,(json_data_len+20));
        memcpy(&json_data_char[0],&json_data_len,sizeof(json_data_len));
        my_header.data_len0 =json_data_char[0];my_header.data_len1 =json_data_char[1];
        my_header.data_len2 =json_data_char[2];my_header.data_len3 =json_data_char[3];

        my_header.cmdtype_HB = HIBYTE(cmd_ret);////命令类型高字节
        my_header.cmdtype_LB = LOBYTE(cmd_ret);////命令类型低字节
        memcpy(&log_data[0],&my_header,sizeof(my_header));//////////收数据长度
        memcpy(&log_data[16],jsonout,strlen(jsonout));//////////收数据长度
    
        dispose_data_len = my_write(socket_hd,log_data,json_data_len+16);
    
        if(dispose_data_len>=(json_data_len+16)) 
        printf("成功发送config %s数据config_file_len=%d\n","config.txt",dispose_data_len);

        cJSON_Delete(root);
        free(jsonout);usleep(1000);
        free(log_data);usleep(1000);
        usleep(50000);
    break;
    case robot_config_task_log_pull:
        printf("接受task log 上传任务指令\n");
        inf.open("/home/log/ale_task.log");
        oss.str("");
        oss << inf.rdbuf();
        xml_str = oss.str();
    
        log_data_temp = xml_str.c_str();
        if(strlen(log_data_temp)<=20) 
        {
        get_log_instance()->task_msg("ale_task.log文件有错误");
        return -1;
        }
        root = cJSON_CreateObject();////////创建根节点
        cJSON_AddStringToObject(root,"task_log_name","ale_task.log");
        cJSON_AddStringToObject(root,"task_log_data",log_data_temp);
        jsonout=cJSON_Print(root);
    //////////////////////////////回传指令赋值
        cmd_ret = cmd_type+10000;
        printf("上传ale_task 文件大小=%d---jsonout=%d\n",strlen(log_data_temp),strlen(jsonout));
        json_data_len = strlen(jsonout);/////数据长度
        log_data = (char*)malloc(json_data_len+20);
        memset(log_data,0,(json_data_len+20));
        memcpy(&json_data_char[0],&json_data_len,sizeof(json_data_len));
        my_header.data_len0 =json_data_char[0];my_header.data_len1 =json_data_char[1];
        my_header.data_len2 =json_data_char[2];my_header.data_len3 =json_data_char[3];

        my_header.cmdtype_HB = HIBYTE(cmd_ret);////命令类型高字节
        my_header.cmdtype_LB = LOBYTE(cmd_ret);////命令类型低字节
        memcpy(&log_data[0],&my_header,sizeof(my_header));//////////收数据长度
        memcpy(&log_data[16],jsonout,strlen(jsonout));//////////收数据长度
    
        dispose_data_len = my_write(socket_hd,log_data,json_data_len+16);
    
        if(dispose_data_len>=(json_data_len+16)) 
        printf("成功发送log%s数据log_file_len=%d\n","ale_task.log",dispose_data_len);

        cJSON_Delete(root);
        free(jsonout);usleep(1000);
        free(log_data);usleep(1000);
        usleep(50000);
    break;
    case robot_config_debug_fileup_req:
        printf("接受debug上传任务指令\n");
        inf.open("/home/log/ALE_SMART.log");
        oss.str("");
        oss << inf.rdbuf();
        xml_str = oss.str();
    
        log_data_temp = xml_str.c_str();
        if(strlen(log_data_temp)<=20) 
        {
        get_log_instance()->task_msg("当前debug.log文件有错误");
        return -1;
        }
        root = cJSON_CreateObject();////////创建根节点
        cJSON_AddStringToObject(root,"debug_file_name","ALE_SMART.log");
        cJSON_AddStringToObject(root,"debug_file_data",log_data_temp);
        jsonout=cJSON_Print(root);
    //////////////////////////////回传指令赋值
        cmd_ret = cmd_type+10000;
        printf("上传debug文件大小=%d---jsonout=%d\n",strlen(log_data_temp),strlen(jsonout));
        json_data_len = strlen(jsonout);/////数据长度
        log_data = (char*)malloc(json_data_len+20);
        memset(log_data,0,(json_data_len+20));
        memcpy(&json_data_char[0],&json_data_len,sizeof(json_data_len));
        my_header.data_len0 =json_data_char[0];my_header.data_len1 =json_data_char[1];
        my_header.data_len2 =json_data_char[2];my_header.data_len3 =json_data_char[3];

        my_header.cmdtype_HB = HIBYTE(cmd_ret);////命令类型高字节
        my_header.cmdtype_LB = LOBYTE(cmd_ret);////命令类型低字节
        memcpy(&log_data[0],&my_header,sizeof(my_header));//////////收数据长度
        memcpy(&log_data[16],jsonout,strlen(jsonout));//////////收数据长度
    
        dispose_data_len = my_write(socket_hd,log_data,json_data_len+16);
    
        if(dispose_data_len>=(json_data_len+16)) 
        printf("成功发送debug%s数据debug_file_len=%d\n","ALE_SMART.log",dispose_data_len);

        cJSON_Delete(root);
        free(jsonout);usleep(1000);
        free(log_data);usleep(1000);
        usleep(50000);
        break;
    case robot_program_update_req:
        printf("接收控制程序更新指令\n");
        cmd_ret = cmd_type+10000;
        my_header.cmdtype_HB = HIBYTE(cmd_ret);////命令类型高字节
        my_header.cmdtype_LB = LOBYTE(cmd_ret);////命令类型低字节
        memcpy(&buf_send[0],&my_header,sizeof(my_header));//////////收数据长度
        send(socket_hd,buf_send,16,0);
        update_data = (char*)malloc(iLength+1);
        memset(update_data,0,iLength+1);
        dispose_data_len = my_read(socket_hd,update_data,iLength);
        if(dispose_data_len!=iLength){get_log_instance()->task_msg("接收控制程序文件数据不正确");free(update_data);return false;}

        update_path = (char*)malloc(strlen(UPDATE_PATH_NAME) + 50);
        memset(update_path,0,strlen(UPDATE_PATH_NAME) + 50);
        strcpy(update_path,UPDATE_PATH_NAME);
        strcat(update_path,"bak_agv_main");
        Update_File = fopen(update_path, "wb+");
        
        usleep(100000);
        dispose_data_len = fwrite(update_data,1, iLength, Update_File);
        printf("写入文件名称%s,写入字节数%d\n",update_path,iLength);
        usleep(100000);
        fclose(Update_File);
        if(dispose_data_len != iLength) return false;///文件写入不成功
        
        usleep(10000);

        //printf("接收软件更新指令%d-----%d----\n",my_header.cmdtype_HB,my_header.cmdtype_LB);

        pox_system("/bin/sh /usr/local/ale_bot/update.sh");
        usleep(100000);
        //cJSON_Delete(root_recv);usleep(1000);
        free(update_path);usleep(1000);
        free(update_data);usleep(1000);
        usleep(1000);
        printf("接收软件更新指令111111------success\n");
        break;
    case robot_task_pause_req:
        get_global_agv_instance()->stop_path();
        cmd_ret = cmd_type+10000;
        my_header.cmdtype_HB = HIBYTE(cmd_ret);////命令类型高字节
        my_header.cmdtype_LB = LOBYTE(cmd_ret);////命令类型低字节
        memcpy(&buf_send[0],&my_header,sizeof(my_header));//////////收数据长度
        send(socket_hd,buf_send,16,0);
        break;
    case robot_task_resume_req:
        get_global_agv_instance()->continue_path();
        cmd_ret = cmd_type+10000;
        my_header.cmdtype_HB = HIBYTE(cmd_ret);////命令类型高字节
        my_header.cmdtype_LB = LOBYTE(cmd_ret);////命令类型低字节
        memcpy(&buf_send[0],&my_header,sizeof(my_header));//////////收数据长度
        send(socket_hd,buf_send,16,0);
        break;
    case   robot_task_cancel_req:
        get_global_agv_instance()->stop_navigating();
        cmd_ret = cmd_type+10000;
        my_header.cmdtype_HB = HIBYTE(cmd_ret);////命令类型高字节
        my_header.cmdtype_LB = LOBYTE(cmd_ret);////命令类型低字节
        memcpy(&buf_send[0],&my_header,sizeof(my_header));//////////收数据长度
        send(socket_hd,buf_send,16,0);
        break;
    case robot_task_antiepidemic_req:////启动防疫任务指令
        //if(get_global_agv_instance()->board_sts._manual_auto==0)
        get_global_agv_instance()->robot_Antiepidemic.run_the_antiepidmic_path();
        cmd_ret = cmd_type+10000;
        my_header.cmdtype_HB = HIBYTE(cmd_ret);////命令类型高字节
        my_header.cmdtype_LB = LOBYTE(cmd_ret);////命令类型低字节
        memcpy(&buf_send[0],&my_header,sizeof(my_header));//////////收数据长度
        send(socket_hd,buf_send,16,0);
        break;
    case robot_status_info_req://////机器人状态查询指令
        printf("接受机器人状态查询指令\n");

        root = cJSON_CreateObject();////////创建根节点
        cur_pose = get_global_agv_instance()->expolate_current_position();
        cJSON_AddNumberToObject(root,"battery",get_global_agv_instance()->board_sts._battary);
        cJSON_AddNumberToObject(root,"theta",cur_pose.theta);
        cJSON_AddNumberToObject(root,"xpos",cur_pose.x);
        cJSON_AddNumberToObject(root,"ypos",cur_pose.y);
        cJSON_AddNumberToObject(root,"exec_cap",get_global_agv_instance()->exec_cap);
        cJSON_AddNumberToObject(root,"exec_route",get_global_agv_instance()->exec_route);
        cJSON_AddNumberToObject(root,"today_odom",get_global_agv_instance()->locate_modual->today_odom);
        cJSON_AddNumberToObject(root,"total_odom",get_global_agv_instance()->locate_modual->total_odom);

        cJSON_AddNumberToObject(root,"manual_auto",get_global_agv_instance()->board_sts._manual_auto);
        cJSON_AddBoolToObject(root,"robot_ready",get_global_agv_instance()->locate_modual->pose_initilized);  
        if(get_global_agv_instance()->get_system_state()==SYSTEM_STATE_NAVIGATING||(!get_global_agv_instance()->lift_complete))    
        cJSON_AddBoolToObject(root,"task_complete",false);
        else if (get_global_agv_instance()->get_system_state()==SYSTEM_STATE_FREE)
        cJSON_AddBoolToObject(root,"task_complete",true);

        if(get_global_agv_instance()->my_runcurve->task_status == ROBOT_TASK_STS_SUSPENDING)
        cJSON_AddBoolToObject(root,"task_stopping",true);
        else
        cJSON_AddBoolToObject(root,"task_stopping",false);
        
        cJSON_AddStringToObject(root,"cur_map",get_global_agv_instance()->locate_modual->map_name_saved.c_str());
        cJSON_AddBoolToObject(root,"trajdone",get_global_agv_instance()->my_runcurve->TRAJDONE);
        cJSON_AddBoolToObject(root,"radar_error",get_global_agv_instance()->laser_error);
        cJSON_AddBoolToObject(root,"radar_disconnect",get_global_agv_instance()->laser_disconnect);
        cJSON_AddBoolToObject(root,"motor_error",get_global_agv_instance()->motor_error);
        cJSON_AddBoolToObject(root,"motor_disconnect",get_global_agv_instance()->motor_disconnect);
        cJSON_AddBoolToObject(root,"board_disconnect",get_global_agv_instance()->board_disconnect);
        cJSON_AddBoolToObject(root,"imu_disconnect",get_global_agv_instance()->imu_disconnect);
        cJSON_AddNumberToObject(root,"warning_code",get_global_agv_instance()->warning_code);
        cJSON_AddNumberToObject(root,"error_code",get_global_agv_instance()->error_code);
        cJSON_AddNumberToObject(root,"vel_line",get_global_agv_instance()->logic_out_process->speed_data.vel_line);
        cJSON_AddNumberToObject(root,"vel_angle",get_global_agv_instance()->logic_out_process->speed_data.vel_ang);
        jsonout=cJSON_Print(root);
        //////////////////////////////回传指令赋值
        cmd_ret = cmd_type+10000;
        printf("上传状态jsonout=%d\n",strlen(jsonout));
        
        json_data_len = strlen(jsonout);/////数据长度
        log_data = (char*)malloc(json_data_len+20);
        memcpy(&json_data_char[0],&json_data_len,sizeof(json_data_len));
        my_header.data_len0 =json_data_char[0];my_header.data_len1 =json_data_char[1];
        my_header.data_len2 =json_data_char[2];my_header.data_len3 =json_data_char[3];
        
        my_header.cmdtype_HB = HIBYTE(cmd_ret);////命令类型高字节
        my_header.cmdtype_LB = LOBYTE(cmd_ret);////命令类型低字节
        memcpy(&log_data[0],&my_header,sizeof(my_header));//////////收数据长度
        memcpy(&log_data[16],jsonout,strlen(jsonout));//////////收数据长度
    
        dispose_data_len = my_write(socket_hd,log_data,json_data_len+16);
        if(dispose_data_len>=(json_data_len+16)) 
        printf("成功发送状态json_data_len=%d\n",dispose_data_len);

        cJSON_Delete(root);
        free(jsonout);usleep(1000);
        free(log_data);usleep(1000);
        usleep(50000);
        break;
    case robot_config_downloadmap_req:
        printf("接受地图下发任务指令\n");
        cmd_ret = cmd_type+10000;
        my_header.cmdtype_HB = HIBYTE(cmd_ret);////命令类型高字节
        my_header.cmdtype_LB = LOBYTE(cmd_ret);////命令类型低字节
        memcpy(&buf_send[0],&my_header,sizeof(my_header));//////////收数据长度
        send(socket_hd,buf_send,16,0); 
        log_data = (char*)malloc(iLength+1);
        memset(log_data,0,iLength+1);
        json_data_len = my_read(socket_hd,log_data,iLength);
        if(json_data_len!=iLength)
        {get_log_instance()->task_msg("接收地图json文件不正确");free(log_data);return false;}
        root_recv=cJSON_Parse(log_data);
        if(!root_recv) 
        {get_log_instance()->task_msg("map file json context error");free(log_data);return false;}
        free(log_data);

        item=cJSON_GetObjectItem(root_recv,"map_name");
        if(item!=NULL)
        {
            int map_size;
            string map_name_recv;
            json_str = item->valuestring;
            map_name_recv = json_str;
            getcwd(runpath, 127);
            sprintf(filename,"%s/map/%s",runpath,json_str.c_str());

            item=cJSON_GetObjectItem(root_recv,"map_size");
            if(item!=NULL)
            map_size = item->valueint;
            printf("接收的地图文件名为%s  大小为%d\n",filename,map_size);
            save_map_name(json_str);

            item=cJSON_GetObjectItem(root_recv,"map_data");
            string str_map_data;
            if(item!=NULL)
            str_map_data = item->valuestring;
            else
            get_log_instance()->task_msg("接收地图data文件不正确");
            //cout<<"recv map  data "<<str_map_data.length()<<endl;   
            string str_map_save = base64_decode(str_map_data.c_str());


            /*log_data = (char*)malloc(map_size+1);
            memset(log_data,0,map_size+1);
            strcpy(log_data,str_map_save.c_str());*/
            //json_data_len = my_read(socket_hd,log_data,map_size);
            //if(json_data_len!=map_size){
            //    get_log_instance()->task_msg("接收地图data文件不正确");
            //free(log_data);return false;}
            //printf("str_map_save size=%d map_size=%d\n",str_map_save.size(),map_size);
            get_global_agv_instance()->load_grid_map((char *)str_map_save.c_str(), map_size);
            //get_global_agv_instance()->load_grid_map(log_data, map_size);

            double xp,yp,theta;
            item=cJSON_GetObjectItem(root_recv,"x_origin");
            if(item!=NULL)
            xp = item->valuedouble;
            else xp = 0;
            item=cJSON_GetObjectItem(root_recv,"y_origin");
            if(item!=NULL)
            yp = item->valuedouble;
            else yp = 0;
            item=cJSON_GetObjectItem(root_recv,"theta_origin");
            if(item!=NULL)
            theta = item->valuedouble;
            else theta = 0;
            
            char str_print[128];
            sprintf(str_print,"mapname=%s,x_origin=%f,y_origin=%f,theta=%f mapsize=%d",map_name_recv.c_str(),xp,yp,theta,str_map_save.size());
            get_log_instance()->task_msg(str_print);
            get_global_agv_instance()->loadPose(xp,yp,theta);

           /* FILE *fp;
            if((fp=fopen(filename,"wb"))==NULL){
            char str_print[100];
            sprintf(str_print,"error open file==%s",filename);
            get_log_instance()->task_msg(str_print);
            return false;
            }else{
            int ret;//
            int write_len=0;
            int file_size= map_size;
            while(write_len<file_size){
                ret=fwrite(log_data,1,map_size,fp);
                write_len+=ret;
            }
            fclose(fp);
            std::ofstream fout(filename, std::ios::binary);
            fout.write(str_map_save.c_str(), str_map_save.size());
            fout.close();
            printf("file=%s save success!!!!!!!!!!!\n",filename);
            get_global_agv_instance()->locate_modual->set_map_name(map_name_recv);
            }*/
            std::ofstream fout(filename, std::ios::binary);
            fout.write(str_map_save.c_str(), str_map_save.size());
            fout.close();
            printf("file=%s save success!!!!!!!!!!!\n",filename);
            get_global_agv_instance()->locate_modual->set_map_name(map_name_recv);
        }
        
        cJSON_Delete(root_recv);usleep(1000);
        free(log_data);usleep(1000);
        break;
    case robot_config_uploadmap_req:
        
        p_map = get_global_agv_instance()->get_current_map();
        if(! p_map.is_nullptr()) {
        std::vector<char> map_data;
        p_map->to_char_array(&map_data);
        int map_size = map_data.size();
        char *pmap_data = map_data.data();
        //string map_str(p);
        char str_print[100];
        sprintf(str_print,"###lwg:rcv msg 4010,获取地图size=%d\n",map_size);
        get_log_instance()->task_msg(str_print);
        //get_log_instance()->task_msg("###lwg:rcv msg 4010,获取地图size=%d\n",map_size);
        root = cJSON_CreateObject();////////创建根节点
        cJSON_AddStringToObject(root,"map_name",get_global_agv_instance()->locate_modual->map_name_saved.c_str());
        cJSON_AddNumberToObject(root,"map_size",map_size);
        cJSON_AddNumberToObject(root,"map_width",p_map.get()->get_info().width);
        cJSON_AddNumberToObject(root,"map_height",p_map.get()->get_info().height);
        cJSON_AddNumberToObject(root,"origen_x",p_map.get()->get_info().origen_x);
     
        cJSON_AddNumberToObject(root,"origen_y",p_map.get()->get_info().origen_y);
        
        cJSON_AddNumberToObject(root,"resolution",p_map.get()->get_info().resolution);
        //printf("11111111111111--size of mapinfo=%d\n",sizeof(MapInfo));
        string map_img = base64_encode(map_data.data(),map_size);
        //printf("222222222222--jsonout=%d\n");
        cJSON_AddStringToObject(root,"map_data",map_img.c_str());
        usleep(100);
        jsonout=cJSON_Print(root);
        //////////////////////////////回传指令赋值
        cmd_ret = cmd_type+10000;
        printf("上传map文件大小---jsonout=%d\n",strlen(jsonout));
        json_data_len = strlen(jsonout);/////数据长度
        log_data = (char*)malloc(json_data_len+20);
        memset(log_data,0,(json_data_len+20));
        //memcpy(&json_data_char[0],&json_data_len,sizeof(json_data_len));
       // char len_buf[4];
		int2byte(json_data_char,json_data_len);
        my_header.data_len0 =json_data_char[0];my_header.data_len1 =json_data_char[1];
        my_header.data_len2 =json_data_char[2];my_header.data_len3 =json_data_char[3];
        //printf("zhuan huan de size=%d\n",byte2int(&json_data_char[0]));
        my_header.cmdtype_HB = HIBYTE(cmd_ret);////命令类型高字节
        my_header.cmdtype_LB = LOBYTE(cmd_ret);////命令类型低字节
        memcpy(&log_data[0],&my_header,sizeof(my_header));//////////收数据长度
        memcpy(&log_data[16],jsonout,json_data_len);//////////收数据长度
        //memcpy(&log_data[16+json_data_len],pmap_data,map_size);//////////地图数据

        dispose_data_len = my_write(socket_hd,log_data,json_data_len+16);
    
        if(dispose_data_len>=(json_data_len+16)) 
        printf("成功发送map %s数据map_len=%d\n","default_map",dispose_data_len);

        cJSON_Delete(root);
        free(jsonout);usleep(1000);
        free(log_data);usleep(1000);
        usleep(50000);

        break;
        }
        else
        {
        cmd_ret = cmd_type+10000;
        my_header.cmdtype_HB = HIBYTE(cmd_ret);////命令类型高字节
        my_header.cmdtype_LB = LOBYTE(cmd_ret);////命令类型低字节
        memcpy(&buf_send[0],&my_header,sizeof(my_header));//////////收数据长度
        send(socket_hd,buf_send,16,0); 
        } 
        break;
    case robot_config_load_ori_map_req:
        p_map = get_global_agv_instance()->get_current_map();
        if(! p_map.is_nullptr()) {
        std::vector<char> map_data;
        p_map->to_uncompress(&map_data);
        int map_size = map_data.size();
        char *pmap_data = map_data.data();
        //string map_str(p);
        char str_print[100];
        sprintf(str_print,"###lwg:rcv msg 4014,获取地图size=%d\n",map_size);
        get_log_instance()->task_msg(str_print);
        root = cJSON_CreateObject();////////创建根节点
        cJSON_AddStringToObject(root,"map_name",get_global_agv_instance()->locate_modual->map_name_saved.c_str());
        cJSON_AddNumberToObject(root,"map_size",map_size);
        cJSON_AddNumberToObject(root,"map_width",p_map.get()->get_info().width);
        cJSON_AddNumberToObject(root,"map_height",p_map.get()->get_info().height);
        cJSON_AddNumberToObject(root,"origen_x",p_map.get()->get_info().origen_x);
        cJSON_AddNumberToObject(root,"origen_y",p_map.get()->get_info().origen_y);
        cJSON_AddNumberToObject(root,"resolution",p_map.get()->get_info().resolution);
        string map_img = base64_encode(pmap_data,map_size);
        cJSON_AddStringToObject(root,"map_data",map_img.c_str());

        jsonout=cJSON_Print(root);
        //////////////////////////////回传指令赋值
        cmd_ret = cmd_type+10000;
        printf("上传map文件大小---jsonout=%d\n",strlen(jsonout));
        json_data_len = strlen(jsonout);/////数据长度
        log_data = (char*)malloc(json_data_len+20);
        memset(log_data,0,(json_data_len+20));
        memcpy(&json_data_char[0],&json_data_len,sizeof(json_data_len));
        my_header.data_len0 =json_data_char[0];my_header.data_len1 =json_data_char[1];
        my_header.data_len2 =json_data_char[2];my_header.data_len3 =json_data_char[3];

        my_header.cmdtype_HB = HIBYTE(cmd_ret);////命令类型高字节
        my_header.cmdtype_LB = LOBYTE(cmd_ret);////命令类型低字节
        memcpy(&log_data[0],&my_header,sizeof(my_header));//////////收数据长度
        memcpy(&log_data[16],jsonout,json_data_len);//////////收数据长度
        //memcpy(&log_data[16+json_data_len],pmap_data,map_size);//////////地图数据

        dispose_data_len = my_write(socket_hd,log_data,json_data_len+16);
    
        if(dispose_data_len>=(json_data_len+16)) 
        printf("成功发送map %s数据map_len=%d\n",get_global_agv_instance()->locate_modual->map_name_saved.c_str(),dispose_data_len);

        cJSON_Delete(root);
        free(jsonout);usleep(1000);
        free(log_data);usleep(1000);
        usleep(50000);
        
        break;
        }
        else
        {
        cmd_ret = cmd_type+10000;
        my_header.cmdtype_HB = HIBYTE(cmd_ret);////命令类型高字节
        my_header.cmdtype_LB = LOBYTE(cmd_ret);////命令类型低字节
        memcpy(&buf_send[0],&my_header,sizeof(my_header));//////////收数据长度
        send(socket_hd,buf_send,16,0); 
        }     
        break;
    case robot_config_car_drv_forward_req:
        cmd_ret = cmd_type+10000;
        my_header.cmdtype_HB = HIBYTE(cmd_ret);////命令类型高字节
        my_header.cmdtype_LB = LOBYTE(cmd_ret);////命令类型低字节
        memcpy(&buf_send[0],&my_header,sizeof(my_header));//////////收数据长度
        send(socket_hd,buf_send,16,0); 

        log_data = (char*)malloc(iLength+1);
        memset(log_data,0,iLength+1);
        json_data_len = my_read(socket_hd,log_data,iLength);
        if(json_data_len!=iLength)
        {get_log_instance()->task_msg("接收car drv 指令不正确");free(log_data);return false;}
        root_recv=cJSON_Parse(log_data);
        if(!root_recv) 
        {get_log_instance()->task_msg("car drc json context error");free(log_data);return false;}

        movet.type = MOVE_INSTRUCTION_TYPE_SPEED;
        item=cJSON_GetObjectItem(root_recv,"line_speed");
        if(item!=NULL)
            movet.speed.speed = int(item->valuedouble);        
        item=cJSON_GetObjectItem(root_recv,"angle_speed");
        if(item!=NULL)
        {        
        movet.speed.turn_left = int(item->valuedouble);    
        std::vector<char> data;
        movet.to_char_array(&data);
        get_global_agv_instance()->get_message_queue()->send(CHANNEL_MOVE, data.data(), data.size());
        }
        cJSON_Delete(root_recv);usleep(1000);
        break;
    case robot_config_cap_points_pull:
        printf("接受cap 点拉取指令\n");
        getcwd(runpath, 127);
        sprintf(filename,"%s/cap.json",runpath);
        //inf.open("/usr/local/ale_bot/config.txt");
        inf.open(filename);
        oss.str("");
        oss << inf.rdbuf();
        xml_str = oss.str();
    
        log_data_temp = xml_str.c_str();
        if(strlen(log_data_temp)<=10) 
        {
            //cmd_ret = cmd_type+10000;
            //my_header.cmdtype_HB = HIBYTE(cmd_ret);////命令类型高字节
            //my_header.cmdtype_LB = LOBYTE(cmd_ret);////命令类型低字节
            //memcpy(&buf_send[0],&my_header,sizeof(my_header));//////////收数据长度
            //send(socket_hd,buf_send,16,0); 
            
            get_log_instance()->task_msg("当前cap.json文件为空");
            return -1;
        }

        root = cJSON_CreateObject();////////创建根节点
        cJSON_AddStringToObject(root,"caps_file_name","cap.json");
        cJSON_AddStringToObject(root,"caps_file_data",log_data_temp);
        jsonout=cJSON_Print(root);
        //////////////////////////////回传指令赋值
        cmd_ret = cmd_type+10000;
        json_data_len = strlen(jsonout);/////数据长度
        printf("上传cap.json文件大小=%d---jsonout=%d\n",strlen(log_data_temp),json_data_len);
        log_data = (char*)malloc(json_data_len+20);
        memset(log_data,0,(json_data_len+20));
        memcpy(&json_data_char[0],&json_data_len,sizeof(json_data_len));
        my_header.data_len0 =json_data_char[0];my_header.data_len1 =json_data_char[1];
        my_header.data_len2 =json_data_char[2];my_header.data_len3 =json_data_char[3];

        my_header.cmdtype_HB = HIBYTE(cmd_ret);////命令类型高字节
        my_header.cmdtype_LB = LOBYTE(cmd_ret);////命令类型低字节
        memcpy(&log_data[0],&my_header,sizeof(my_header));//////////收数据长度
        memcpy(&log_data[16],jsonout,strlen(jsonout));//////////收数据长度
    
        dispose_data_len = my_write(socket_hd,log_data,json_data_len+16);
    
        if(dispose_data_len>=(json_data_len+16)) 
        printf("成功发送cap.json %s数据dispose_file_len=%d\n","cap.json",dispose_data_len);

        cJSON_Delete(root);
        free(jsonout);usleep(1000);
        free(log_data);usleep(1000);
        usleep(50000);        
        break;
    case robot_config_cap_points_push:
        printf("接受cap.json下发任务指令\n");
        //cmd_ret = cmd_type+10000;
        //my_header.cmdtype_HB = HIBYTE(cmd_ret);////命令类型高字节
        //my_header.cmdtype_LB = LOBYTE(cmd_ret);////命令类型低字节
        //memcpy(&buf_send[0],&my_header,sizeof(my_header));//////////收数据长度
        //send(socket_hd,buf_send,16,0); 
        log_data = (char*)malloc(iLength+1);
        memset(log_data,0,iLength+1);
        json_data_len = my_read(socket_hd,log_data,iLength);
        if(json_data_len!=iLength)
        {get_log_instance()->task_msg("接收cap.json不正确");free(log_data);return false;}
        root_recv=cJSON_Parse(log_data);
        if(!root_recv) 
        {get_log_instance()->task_msg("cur json context error");free(log_data);return false;}
        getcwd(runpath, 127);
        sprintf(filename,"%s/cap.json",runpath);
        outf.open(filename);
        
        item=cJSON_GetObjectItem(root_recv,"caps_file_data");
        if(item!=NULL)
            json_str = item->valuestring;
        outf << json_str;
        outf.close();
        usleep(10000);
        //////////////////////////////回传指令赋值
        cmd_ret = cmd_type+10000;
        my_header.cmdtype_HB = HIBYTE(cmd_ret);////命令类型高字节
        my_header.cmdtype_LB = LOBYTE(cmd_ret);////命令类型低字节
        memcpy(&buf_send[0],&my_header,sizeof(my_header));//////////收数据长度
        send(socket_hd,buf_send,16,0); 

        cJSON_Delete(root_recv);usleep(1000);      
        break;
    case robot_config_cap_points_catch:
        printf("接受cap点捕捉任务指令\n");
 
        log_data = (char*)malloc(iLength+1);
        memset(log_data,0,iLength+1);
        json_data_len = my_read(socket_hd,log_data,iLength);
        if(json_data_len!=iLength)
        {get_log_instance()->task_msg("接收cap点捕捉指令不正确");free(log_data);return false;}
        root_recv=cJSON_Parse(log_data);
        if(!root_recv) 
        {get_log_instance()->task_msg("cap catch context error");free(log_data);return false;}

        item=cJSON_GetObjectItem(root_recv,"cap_index");
        if(item!=NULL)
        cap_point_index = item->valueint;
        if(cap_point_index>0){
        Position pose = get_global_agv_instance()->expolate_current_position();
        std::list<CapPoint>::iterator itc;
        for(itc=get_global_agv_instance()->caps.begin();itc!=get_global_agv_instance()->caps.end();itc++){
            CapPoint act = *itc;
            if(act.id==cap_point_index){
            get_global_agv_instance()->caps.erase(itc);
            break;
            }
        }
        CapPoint cap(cap_point_index,pose.x,pose.y,pose.theta);
        get_global_agv_instance()->caps.push_back(cap);
        get_global_agv_instance()->dm_code_record = true;
        for(itc=get_global_agv_instance()->caps.begin();itc!=get_global_agv_instance()->caps.end();itc++){
            CapPoint act = *itc;
            printf("x=%lf,y=%lf,theta=%f,id=%d\n",act.x,act.y,act.theta,act.id);
        }
        get_global_storage()->save_caps(get_global_agv_instance()->caps);   
        }  

        cmd_ret = cmd_type+10000;
        my_header.cmdtype_HB = HIBYTE(cmd_ret);////命令类型高字节
        my_header.cmdtype_LB = LOBYTE(cmd_ret);////命令类型低字节
        memcpy(&buf_send[0],&my_header,sizeof(my_header));//////////收数据长度
        send(socket_hd,buf_send,16,0);   
        cJSON_Delete(root_recv);usleep(1000);
        break;
    case robot_points_cloud_pull:
        if(true)
        {
        PointCloudData laser_cloud_data = get_global_agv_instance()->get_last_radar_data();

        root = cJSON_CreateObject();////////创建根节点
        cJSON_AddItemToObject(root,"beams",array=cJSON_CreateArray());
        Position base=get_global_agv_instance()->expolate_current_position();

        for (int i = 0; i<laser_cloud_data.points.size(); i++)
        {
        double xp = laser_cloud_data.points[laser_cloud_data.points.size()-i-1](0);
        double yp = laser_cloud_data.points[laser_cloud_data.points.size()-i-1](1);
        Position p=Position(0,xp,yp,0);
        p=base*p;
        cJSON_AddItemToArray(array,addobj=cJSON_CreateObject());
        cJSON_AddNumberToObject(addobj,"xpos",p.x);
        cJSON_AddNumberToObject(addobj,"ypos",p.y);

        }
        jsonout=cJSON_Print(root);
        
		cmd_ret = cmd_type+10000;
        json_data_len = strlen(jsonout);/////数据长度
        printf("上传激光数据大小---jsonout=%d\n",json_data_len);
        log_data = (char*)malloc(json_data_len+20);
        memset(log_data,0,(json_data_len+20));
        memcpy(&json_data_char[0],&json_data_len,sizeof(json_data_len));
        my_header.data_len0 =json_data_char[0];my_header.data_len1 =json_data_char[1];
        my_header.data_len2 =json_data_char[2];my_header.data_len3 =json_data_char[3];

        my_header.cmdtype_HB = HIBYTE(cmd_ret);////命令类型高字节
        my_header.cmdtype_LB = LOBYTE(cmd_ret);////命令类型低字节
        memcpy(&log_data[0],&my_header,sizeof(my_header));//////////收数据长度
        memcpy(&log_data[16],jsonout,strlen(jsonout));//////////收数据长度
    
        dispose_data_len = my_write(socket_hd,log_data,json_data_len+16);
    
        if(dispose_data_len>=(json_data_len+16)) 
        printf("成功发送激光 数据=%d\n",dispose_data_len);

        cJSON_Delete(root);
        free(jsonout);usleep(1000);
        free(log_data);usleep(1000);
        usleep(50000);
        }
        break;
    case robot_config_path_pull:
        printf("接受route 拉取指令\n");
        getcwd(runpath, 127);
        sprintf(filename,"%s/route.json",runpath);
        inf.open(filename);
        oss.str("");
        oss << inf.rdbuf();
        xml_str = oss.str();
    
        log_data_temp = xml_str.c_str();
        if(strlen(log_data_temp)<=10) 
        {
            char str_print[100];
            sprintf(str_print,"当前route.json文件%s为空","route.json");
            get_log_instance()->task_msg(str_print);
            return -1;
        }

        root = cJSON_CreateObject();////////创建根节点
        cJSON_AddStringToObject(root,"route_file_name","cap.json");
        cJSON_AddStringToObject(root,"route_file_data",log_data_temp);
        jsonout=cJSON_Print(root);
        //////////////////////////////回传指令赋值
        cmd_ret = cmd_type+10000;
        json_data_len = strlen(jsonout);/////数据长度
        printf("上传route.json文件大小=%d---jsonout=%d\n",strlen(log_data_temp),json_data_len);
        log_data = (char*)malloc(json_data_len+20);
        memset(log_data,0,(json_data_len+20));
        memcpy(&json_data_char[0],&json_data_len,sizeof(json_data_len));
        my_header.data_len0 =json_data_char[0];my_header.data_len1 =json_data_char[1];
        my_header.data_len2 =json_data_char[2];my_header.data_len3 =json_data_char[3];

        my_header.cmdtype_HB = HIBYTE(cmd_ret);////命令类型高字节
        my_header.cmdtype_LB = LOBYTE(cmd_ret);////命令类型低字节
        memcpy(&log_data[0],&my_header,sizeof(my_header));//////////收数据长度
        memcpy(&log_data[16],jsonout,strlen(jsonout));//////////收数据长度
    
        dispose_data_len = my_write(socket_hd,log_data,json_data_len+16);
    
        if(dispose_data_len>=(json_data_len+16)) 
        printf("成功发送route.json %s数据dispose_file_len=%d\n","route.json",dispose_data_len);

        cJSON_Delete(root);
        free(jsonout);usleep(1000);
        free(log_data);usleep(1000);
        usleep(50000);  
        break;
    case robot_config_path_push:
        printf("接受route.json下发任务指令\n");
        //cmd_ret = cmd_type+10000;
        //my_header.cmdtype_HB = HIBYTE(cmd_ret);////命令类型高字节
        //my_header.cmdtype_LB = LOBYTE(cmd_ret);////命令类型低字节
        //memcpy(&buf_send[0],&my_header,sizeof(my_header));//////////收数据长度
        //send(socket_hd,buf_send,16,0); 
        log_data = (char*)malloc(iLength+1);
        memset(log_data,0,iLength+1);
        json_data_len = my_read(socket_hd,log_data,iLength);
        if(json_data_len!=iLength)
        {get_log_instance()->task_msg("接收route.json不正确");free(log_data);return false;}
        root_recv=cJSON_Parse(log_data);
        if(!root_recv) 
        {get_log_instance()->task_msg("route json context error");free(log_data);return false;}
        getcwd(runpath, 127);
        sprintf(filename,"%s/route.json",runpath);
        outf.open(filename);
        
        item=cJSON_GetObjectItem(root_recv,"route_file_data");
        if(item!=NULL)
            json_str = item->valuestring;
        outf << json_str;
        outf.close();
        usleep(10000);
        cmd_ret = cmd_type+10000;
        my_header.cmdtype_HB = HIBYTE(cmd_ret);////命令类型高字节
        my_header.cmdtype_LB = LOBYTE(cmd_ret);////命令类型低字节
        memcpy(&buf_send[0],&my_header,sizeof(my_header));//////////收数据长度
        send(socket_hd,buf_send,16,0); 
        //////////////////////////////回传指令赋值
        cJSON_Delete(root_recv);usleep(1000);   

        get_global_agv_instance()->robot_Antiepidemic.load_caps_routes(); 
        break;
    case robot_config_path_set:
    /////路径设置
        printf("接受路径设置任务指令\n");
        
        log_data = (char*)malloc(iLength+1);
        memset(log_data,0,iLength+1);
        json_data_len = my_read(socket_hd,log_data,iLength);
        if(json_data_len!=iLength)
        {

            get_log_instance()->task_msg("接收路径设置指令不正确");free(log_data);return false;}
        root_recv=cJSON_Parse(log_data);
        if(!root_recv) 
        {

            get_log_instance()->task_msg("route set context error");free(log_data);return false;}
        item=cJSON_GetObjectItem(root_recv,"route_index");
        if(item!=NULL)
        route_index = item->valueint;
        else route_index = 0;
        if(route_index>0){
        std::list<CapRoute>::iterator itc;
        for(itc=get_global_agv_instance()->routes.begin();itc!=get_global_agv_instance()->routes.end();itc++){
            CapRoute cr = *itc;
            if(cr.id==route_index){
                get_global_agv_instance()->routes.erase(itc);
                break;
            }
        }
        CapRoute cap_route;
		cap_route.id = route_index;
        item=cJSON_GetObjectItem(root_recv,"action");
        if(item!=NULL)
		cap_route.op_id = item->valueint;
        if(cap_route.op_id<LIFT_NONE_CMD||cap_route.op_id>LIFT_FIX_ANGLE_CMD) cap_route.op_id = 0xff;

        cJSON *cap_arry     = cJSON_GetObjectItem( root_recv, "cap_array");  //cap点列表
        if( NULL != cap_arry ){
        cJSON *cap_list  = cap_arry->child;
        cJSON * item = NULL;
        Position pose = get_global_agv_instance()->expolate_current_position();
        double xp,yp,theta;
        int cap_id,collision,compensation,dir,dmcode,speed;
        int doact_cap,angle_cap;
        while( cap_list != NULL ){ 


            item=cJSON_GetObjectItem(cap_list,"collision");
            if(item!=NULL)
            collision = item->valueint;
            else collision = 0;
            item=cJSON_GetObjectItem(cap_list,"compensation");
            if(item!=NULL)
            compensation = item->valueint;
            else compensation = 0;
            item=cJSON_GetObjectItem(cap_list,"dir");
            if(item!=NULL)
            dir = item->valueint;
            else dir = 0; 
            item=cJSON_GetObjectItem(cap_list,"dmcode");
            if(item!=NULL)
            dmcode = item->valueint;
            else dmcode = 0;    
            item=cJSON_GetObjectItem(cap_list,"speed");
            if(item!=NULL)
            speed = item->valueint;
            else speed = 500;    
            item=cJSON_GetObjectItem(cap_list,"cap_id");
            if(item!=NULL)
            cap_id = item->valueint;
            else cap_id = 0;   
            item=cJSON_GetObjectItem(cap_list,"doact");
            if(item!=NULL)
            doact_cap = item->valueint;
            else doact_cap = 0;  
            item=cJSON_GetObjectItem(cap_list,"angle");
            if(item!=NULL)
            angle_cap = item->valueint;
            else angle_cap = 0;  

			for(CapPoint &point : get_global_agv_instance()->caps) {
				if(point.id==cap_id){
					xp=point.x;yp=point.y;theta=point.theta;
					break;
				}
			}
            char str_print[128];
            sprintf(str_print,"id=%d,dir=%d,colli=%d,dmcode=%d,commpen=%d,speed=%d\n",cap_id,dir,collision,dmcode,compensation,speed);
            //printf("id=%d,dir=%d,colli=%d,dmcode=%d,commpen=%d,speed=%d\n",cap_id,dir,collision,dmcode,compensation,speed);
            get_log_instance()->task_msg(str_print);
            //(int id,double x, double y,double theta,int dir,int collision,int dmcode,int compens,int speed_s)
            CapPoint p(cap_id,xp,yp,theta,dir,collision,dmcode,compensation,speed,doact_cap,angle_cap);//
            cap_route.cap_points.push_back(p);
            cap_list = cap_list->next ;
        }
        }  
        get_global_agv_instance()->routes.push_back(cap_route);
        get_global_storage()->save_routes(get_global_agv_instance()->routes);
        }   
        cmd_ret = cmd_type+10000;
        my_header.cmdtype_HB = HIBYTE(cmd_ret);////命令类型高字节
        my_header.cmdtype_LB = LOBYTE(cmd_ret);////命令类型低字节
        memcpy(&buf_send[0],&my_header,sizeof(my_header));//////////收数据长度
        send(socket_hd,buf_send,16,0);  
        cJSON_Delete(root_recv);usleep(1000);
        get_global_agv_instance()->robot_Antiepidemic.load_caps_routes(); 
        break;
    case robot_config_path_run:
        /////路径启动
        printf("接受路径启动指令%d\n",iLength);
        log_data = (char*)malloc(iLength+1);
        memset(log_data,0,iLength+1);
        json_data_len = my_read(socket_hd,log_data,iLength);
        if(json_data_len!=iLength){
            get_log_instance()->task_msg("接受路径启动指令不正确");free(log_data);return false;}
            
        root_recv=cJSON_Parse(log_data);
        if(!root_recv) {
            get_log_instance()->task_msg("route run context error");free(log_data);return false;}

        item=cJSON_GetObjectItem(root_recv,"route_index");
        if(item!=NULL)
        route_index = item->valueint;
        else 
        route_index = 0;

        if(route_index>0)
        {
            for(CapRoute &route : get_global_agv_instance()->routes) {
            if(route.id==route_index){
                char str_print[100];
                sprintf(str_print,"要启动的route id=%d",route_index);
                get_log_instance()->task_msg(str_print);
                RouteChain chain;
                CapRoute r=route;
                chain.id=-1;
                chain.routes.push_back(r);
                get_global_agv_instance()->start_navigating(chain);
            }
            }
        }
        cmd_ret = cmd_type+10000;
        my_header.cmdtype_HB = HIBYTE(cmd_ret);////命令类型高字节
        my_header.cmdtype_LB = LOBYTE(cmd_ret);////命令类型低字节
        memcpy(&buf_send[0],&my_header,sizeof(my_header));//////////收数据长度
        send(socket_hd,buf_send,16,0); 
        cJSON_Delete(root_recv);usleep(1000);
        break;
    case robot_control_slam_req:
        get_log_instance()->task_msg("收开始建图指令，清除所有cap点");
        get_global_agv_instance()->start_mapping();
        get_global_agv_instance()->caps.clear();
        get_global_storage()->save_caps(get_global_agv_instance()->caps);
        get_global_agv_instance()->routes.clear();
        get_global_storage()->save_routes(get_global_agv_instance()->routes);
        cmd_ret = cmd_type+10000;
        my_header.cmdtype_HB = HIBYTE(cmd_ret);////命令类型高字节
        my_header.cmdtype_LB = LOBYTE(cmd_ret);////命令类型低字节
        memcpy(&buf_send[0],&my_header,sizeof(my_header));//////////收数据长度
        send(socket_hd,buf_send,16,0);         
        break;
    case robot_control_endslam_req:
        get_log_instance()->task_msg("收停止建图指令");
        get_global_agv_instance()->stop_mapping();  
        cmd_ret = cmd_type+10000;
        my_header.cmdtype_HB = HIBYTE(cmd_ret);////命令类型高字节
        my_header.cmdtype_LB = LOBYTE(cmd_ret);////命令类型低字节
        memcpy(&buf_send[0],&my_header,sizeof(my_header));//////////收数据长度
        send(socket_hd,buf_send,16,0);       
        break;
    case robot_control_high_slam_req:
        get_log_instance()->task_msg("收高分辨率建图启动指令");
        log_data = (char*)malloc(iLength+1);
        memset(log_data,0,iLength+1);
        json_data_len = my_read(socket_hd,log_data,iLength);
        if(json_data_len!=iLength){
            get_log_instance()->task_msg("接收高分辨率 建图 指令不正确");free(log_data);return false;}
        /*root_recv=cJSON_Parse(log_data);
        if(!root_recv) {
            get_log_instance()->task_msg("high res slam error");free(log_data);return false;}

        item=cJSON_GetObjectItem(root_recv,"resolution");
        if(item!=NULL)
        {
            float reso,step_len,step_angle;
            reso = item->valuedouble;
            
            item=cJSON_GetObjectItem(root_recv,"step_len");
            if(item!=NULL)
            step_len = item->valuedouble; 

            item=cJSON_GetObjectItem(root_recv,"step_angle");
            if(item!=NULL)
            step_angle = item->valuedouble; 
            char str_print[128];
            sprintf(str_print,"resolution=%f,step_len=%f,step_angle=%f",reso,step_len,step_angle);
            get_log_instance()->task_msg(str_print);
        }
*/
        get_global_agv_instance()->start_mapping_landmark();
        cmd_ret = cmd_type+10000;
        my_header.cmdtype_HB = HIBYTE(cmd_ret);////命令类型高字节
        my_header.cmdtype_LB = LOBYTE(cmd_ret);////命令类型低字节
        memcpy(&buf_send[0],&my_header,sizeof(my_header));//////////收数据长度
        send(socket_hd,buf_send,16,0);   
        cJSON_Delete(root_recv);usleep(1000);
        break;
    case robot_control_end_high_slam_req:
        get_log_instance()->task_msg("收高分辨率建图停止指令");
        get_global_agv_instance()->stop_mapping_landmark();
        cmd_ret = cmd_type+10000;
        my_header.cmdtype_HB = HIBYTE(cmd_ret);////命令类型高字节
        my_header.cmdtype_LB = LOBYTE(cmd_ret);////命令类型低字节
        memcpy(&buf_send[0],&my_header,sizeof(my_header));//////////收数据长度
        send(socket_hd,buf_send,16,0);   
        break;
    case robot_config_car_drv_speed_pull:
        printf("收手动速度拉取指令\n");
        root = cJSON_CreateObject();////////创建根节点
        cJSON_AddNumberToObject(root,"speed",get_global_agv_instance()->my_runcurve->get_manual_ratio());
        
        jsonout=cJSON_Print(root);
        //////////////////////////////回传指令赋值
        cmd_ret = cmd_type+10000;        
        json_data_len = strlen(jsonout);/////数据长度
        log_data = (char*)malloc(json_data_len+20);
        memcpy(&json_data_char[0],&json_data_len,sizeof(json_data_len));
        my_header.data_len0 =json_data_char[0];my_header.data_len1 =json_data_char[1];
        my_header.data_len2 =json_data_char[2];my_header.data_len3 =json_data_char[3];
        
        my_header.cmdtype_HB = HIBYTE(cmd_ret);////命令类型高字节
        my_header.cmdtype_LB = LOBYTE(cmd_ret);////命令类型低字节
        memcpy(&log_data[0],&my_header,sizeof(my_header));//////////收数据长度
        memcpy(&log_data[16],jsonout,strlen(jsonout));//////////收数据长度
    
        dispose_data_len = my_write(socket_hd,log_data,json_data_len+16);
        if(dispose_data_len>=(json_data_len+16)) 
        printf("成功发送手动速度json_data_len=%d\n",dispose_data_len);
        cJSON_Delete(root);
        free(jsonout);usleep(1000);
        free(log_data);usleep(1000);
        usleep(50000);
        break;
    case robot_config_car_drv_speed_push:
        printf("接受手动速度下发指令%d\n",iLength);
        log_data = (char*)malloc(iLength+1);
        memset(log_data,0,iLength+1);
        json_data_len = my_read(socket_hd,log_data,iLength);
        if(json_data_len!=iLength){
            get_log_instance()->task_msg("接受手动速度下发指令不正确");free(log_data);return false;}
        root_recv=cJSON_Parse(log_data);
        if(!root_recv) {
            get_log_instance()->task_msg("speed set context error");free(log_data);return false;}

        item=cJSON_GetObjectItem(root_recv,"speed");
        if(item!=NULL)
        route_index = item->valueint;
        else 
        route_index = 0;

        if(route_index>0)
        {
            printf("speed = %d\n",route_index);
            get_global_agv_instance()->my_runcurve->set_manual_ratio(route_index);
        }
        cmd_ret = cmd_type+10000;
        my_header.cmdtype_HB = HIBYTE(cmd_ret);////命令类型高字节
        my_header.cmdtype_LB = LOBYTE(cmd_ret);////命令类型低字节
        memcpy(&buf_send[0],&my_header,sizeof(my_header));//////////收数据长度
        send(socket_hd,buf_send,16,0); 
        cJSON_Delete(root_recv);usleep(1000);
        break;
    case robot_config_cllision_switch:
        printf("接受避障 开关 指令%d\n",iLength);
        log_data = (char*)malloc(iLength+1);
        memset(log_data,0,iLength+1);
        json_data_len = my_read(socket_hd,log_data,iLength);
        if(json_data_len!=iLength){
            get_log_instance()->task_msg("接受避障 开关 指令不正确");free(log_data);return false;}
        root_recv=cJSON_Parse(log_data);
        if(!root_recv) {
            get_log_instance()->task_msg("collision switch context error");free(log_data);return false;}

        item=cJSON_GetObjectItem(root_recv,"switch");
        if(item!=NULL)
        get_global_agv_instance()->collision_switch = (item->type == cJSON_True)?true:false;

        cmd_ret = cmd_type+10000;
        my_header.cmdtype_HB = HIBYTE(cmd_ret);////命令类型高字节
        my_header.cmdtype_LB = LOBYTE(cmd_ret);////命令类型低字节
        memcpy(&buf_send[0],&my_header,sizeof(my_header));//////////收数据长度
        send(socket_hd,buf_send,16,0); 
        cJSON_Delete(root_recv);usleep(1000);
        break;
    case robot_config_lift_drv:
        printf("接收顶升 控制 指令%d\n",iLength);
        log_data = (char*)malloc(iLength+1);
        memset(log_data,0,iLength+1);
        json_data_len = my_read(socket_hd,log_data,iLength);
        if(json_data_len!=iLength){
            get_log_instance()->task_msg("接收顶升 控制 指令不正确");free(log_data);return false;}
            //printf("%s\n",log_data);
        root_recv=cJSON_Parse(log_data);
        if(!root_recv) {
            get_log_instance()->task_msg("lift control error");free(log_data);return false;}

        item=cJSON_GetObjectItem(root_recv,"contol_type");
        if(item!=NULL)
        lift_c_typ = item->valueint;
        else 
        lift_c_typ = -1;
        if((lift_c_typ>-1)&&(lift_c_typ<5))
        {
            MoveInstruction move;
            move.lift.dir=lift_c_typ;
            move.turn.dir = lift_c_typ;
            std::vector<char> data;
        switch(lift_c_typ)
        {
        case 0:///////顶升下降
        case 1:///////顶升下降
            move.type = MOVE_INSTRUCTION_TYPE_LIFT;
            break;
        case 2:///////旋转回零
        case 3:///////全回零
            move.type = MOVE_INSTRUCTION_TYPE_TURN;
            
            break;
        case 4:///////旋转固定角度
            move.type = MOVE_INSTRUCTION_TYPE_TURN;
            item=cJSON_GetObjectItem(root_recv,"target_angle");
            if(item!=NULL)
            move.turn.angle = -item->valuedouble/180*M_PI;
            else 
            move.turn.angle = 0;

            item=cJSON_GetObjectItem(root_recv,"target_speed");
            if(item!=NULL)
            move.turn.angle_speed = item->valuedouble/180*M_PI;
            else 
            move.turn.angle_speed = 0.3;
            
            break;
        default:
            break;
        }
        
        char str_print[128];
        sprintf(str_print,"type=%d,angle=%f,angle_speed=%f",move.type,move.turn.angle,move.turn.angle_speed);
        get_log_instance()->task_msg(str_print);

        move.to_char_array(&data);
        get_global_agv_instance()->get_message_queue()->send(CHANNEL_MOVE, data.data(), data.size());
        //get_global_agv_instance()->lift_complete = false;

        }
        
        cmd_ret = cmd_type+10000;
        my_header.cmdtype_HB = HIBYTE(cmd_ret);////命令类型高字节
        my_header.cmdtype_LB = LOBYTE(cmd_ret);////命令类型低字节
        memcpy(&buf_send[0],&my_header,sizeof(my_header));//////////收数据长度
        send(socket_hd,buf_send,16,0); 

        cJSON_Delete(root_recv);usleep(1000);
        break;
    case robot_config_path_point_run:
        printf("接受运行路径点指令%d\n",iLength);
        log_data = (char*)malloc(iLength+1);
        memset(log_data,0,iLength+1);
        json_data_len = my_read(socket_hd,log_data,iLength);
        if(json_data_len!=iLength){
            get_log_instance()->task_msg("接受运行路径点指令不正确");free(log_data);return false;}
        root_recv=cJSON_Parse(log_data);
        if(!root_recv) {
            get_log_instance()->task_msg("path point run context error");free(log_data);return false;}

        item=cJSON_GetObjectItem(root_recv,"path_point");
        if(item!=NULL)
        cap_point_index = item->valueint;
        else 
        cap_point_index = 0;

        if(cap_point_index>0)
        {
            printf("cap_point_index = %d\n",cap_point_index);
            get_global_agv_instance()->robot_Antiepidemic.find_best_path(cap_point_index);
            printf("route guihua success \n");
        }
        cmd_ret = cmd_type+10000;
        my_header.cmdtype_HB = HIBYTE(cmd_ret);////命令类型高字节
        my_header.cmdtype_LB = LOBYTE(cmd_ret);////命令类型低字节
        memcpy(&buf_send[0],&my_header,sizeof(my_header));//////////收数据长度
        send(socket_hd,buf_send,16,0); 
        cJSON_Delete(root_recv);usleep(1000);
        break;
    case robot_config_rotate_point_run:
        printf("接受原地转动指令%d\n",iLength);
        log_data = (char*)malloc(iLength+1);
        memset(log_data,0,iLength+1);
        json_data_len = my_read(socket_hd,log_data,iLength);
        if(json_data_len!=iLength){
            get_log_instance()->task_msg("接受原地转动指令不正确");free(log_data);return false;}
        root_recv=cJSON_Parse(log_data);
        if(!root_recv) {
            get_log_instance()->task_msg("rotate run context error");free(log_data);return false;}

        item=cJSON_GetObjectItem(root_recv,"rotate_angle");
        if(item!=NULL)
        rotate_angle = item->valuedouble;
        else 
        rotate_angle = 0;

        if(item!=NULL)
        {
            printf(" rotate_angle=%f\n",rotate_angle);
            get_global_agv_instance()->robot_Antiepidemic.rotate_fix_angle(rotate_angle);
        }
        cmd_ret = cmd_type+10000;
        my_header.cmdtype_HB = HIBYTE(cmd_ret);////命令类型高字节
        my_header.cmdtype_LB = LOBYTE(cmd_ret);////命令类型低字节
        memcpy(&buf_send[0],&my_header,sizeof(my_header));//////////收数据长度
        send(socket_hd,buf_send,16,0); 
        cJSON_Delete(root_recv);usleep(1000);
        break;
    default :
        break;
    }
    usleep(1000);
    return 1;
}


ssize_t      /* Read "n" bytes from a descriptor. */
LOG_CK_UPModule::my_read(int fd, void *vptr, size_t n)
{
    size_t nleft;
    ssize_t nread;
    char *ptr;
    
    ptr = (char*)vptr;
    nleft = n;
 
    while (nleft > 0) {
        if ( (nread = read(fd, ptr, nleft)) < 0) {
            if (errno == EINTR)
                nread = 0;  /* and call read() again */
            else
                return(-1);
        } else if (nread == 0)
   
        break;    /* EOF */
 
        nleft -= nread;
        ptr += nread;
    }
    
    return(n - nleft);  /* return >= 0 */
}
ssize_t LOG_CK_UPModule::my_write(int fd,void *buffer,size_t n)
{
size_t bytes_left;
ssize_t written_bytes;
char *ptr;

ptr=(char *)buffer;
bytes_left=n;
int count_error = 0;
while(bytes_left>0)
{
    written_bytes=write(fd,ptr,bytes_left);
    usleep(100);
    if(written_bytes<=0)
    {    
        count_error++;   
        written_bytes=0;
        continue;            
    }
    if(count_error > 5000) printf("tcp file write failed!!!!!!!!\n");//return(-1);
    bytes_left-=written_bytes;
    ptr+=written_bytes; 

}
return(n-bytes_left);
}
int LOG_CK_UPModule::pox_system(const char *cmd_line) 
{ 
int ret = 0;
sighandler_t old_handler;
old_handler = signal(SIGCHLD, SIG_DFL);
ret = system(cmd_line);
signal(SIGCHLD, old_handler);
printf("777777777执行cmd=%s----%d\n",cmd_line,ret);
return ret;
} 

std::string LOG_CK_UPModule::base64_encode(const char * bytes_to_encode, unsigned int in_len)
{
    std::string ret;
    int i = 0;
    int j = 0;
    unsigned char char_array_3[3];
    unsigned char char_array_4[4];

    while (in_len--)
    {
        char_array_3[i++] = *(bytes_to_encode++);
        if(i == 3)
        {
            char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
            char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
            char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
            char_array_4[3] = char_array_3[2] & 0x3f;
            for(i = 0; (i <4) ; i++)
            {
                ret += base64_chars[char_array_4[i]];
            }
            i = 0;
        }
    }
    if(i)
    {
        for(j = i; j < 3; j++)
        {
            char_array_3[j] = '\0';
        }

        char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
        char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
        char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
        char_array_4[3] = char_array_3[2] & 0x3f;

        for(j = 0; (j < i + 1); j++)
        {
            ret += base64_chars[char_array_4[j]];
        }

        while((i++ < 3))
        {
            ret += '=';
        }

    }
    return ret;
}
//解密
std::string LOG_CK_UPModule::base64_decode(std::string const & encoded_string)
{
    int in_len = (int) encoded_string.size();
    int i = 0;
    int j = 0;
    int in_ = 0;
    unsigned char char_array_4[4], char_array_3[3];
    std::string ret;

    while (in_len-- && ( encoded_string[in_] != '=') && is_base64(encoded_string[in_])) {
        char_array_4[i++] = encoded_string[in_]; in_++;
        if (i ==4) {
            for (i = 0; i <4; i++)
                char_array_4[i] = base64_chars.find(char_array_4[i]);

            char_array_3[0] = (char_array_4[0] << 2) + ((char_array_4[1] & 0x30) >> 4);
            char_array_3[1] = ((char_array_4[1] & 0xf) << 4) + ((char_array_4[2] & 0x3c) >> 2);
            char_array_3[2] = ((char_array_4[2] & 0x3) << 6) + char_array_4[3];

            for (i = 0; (i < 3); i++)
                ret += char_array_3[i];
            i = 0;
        }
    }
    if (i) {
        for (j = i; j <4; j++)
            char_array_4[j] = 0;

        for (j = 0; j <4; j++)
            char_array_4[j] = base64_chars.find(char_array_4[j]);

        char_array_3[0] = (char_array_4[0] << 2) + ((char_array_4[1] & 0x30) >> 4);
        char_array_3[1] = ((char_array_4[1] & 0xf) << 4) + ((char_array_4[2] & 0x3c) >> 2);
        char_array_3[2] = ((char_array_4[2] & 0x3) << 6) + char_array_4[3];

        for (j = 0; (j < i - 1); j++) ret += char_array_3[j];
    }

    return ret;
}

bool LOG_CK_UPModule::save_map_name(string name)/////
{
    char   filename[128];
    char   runpath[128];
    cJSON *root = NULL;
    cJSON *ZEROPOS = NULL;
    char *jsonout = NULL;
    //string map_name(name);
    get_global_agv_instance()->locate_modual->set_map_name(name);

    root = cJSON_CreateObject();////////创建根节点
    cJSON_AddStringToObject(root,"default_map",name.c_str());
    jsonout = cJSON_Print(root);
    //打开一个info.json文件，并写入json内容
    FILE *fp;

    getcwd(runpath, 127);

    sprintf(filename,"%s/map/default.json",runpath);
    fp = fopen(filename, "w");

    fwrite(jsonout, strlen(jsonout), 1, fp);
    fclose(fp);//关闭文件
    cJSON_Delete(root);
	free(jsonout);

	return true;
}

LOG_CK_UPModule *log_instance = nullptr;

void init_LOG_CK_UP_instance()
{
    if (log_instance == nullptr)
    {
        log_instance = new LOG_CK_UPModule();
    }
}
