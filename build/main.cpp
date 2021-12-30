#include "app_interface/ale_tcp_api.h"
#include "app_interface/log_ck_up_api.h"
int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        printf("Usage:");
        printf("  ./agv_main xxx/configuration_file.txt\n");
        return -1;
    }
    load_config(argv[1]);
    local_port = get_configs()->get_int("network", "local_port", nullptr);
    if(argc > 2) {
        local_port = atoi(argv[2]);
    }

    init_global_agv_instance();
    init_log_instance();
    
	get_global_agv_instance()->default_locate_map_id=get_global_storage()->init_map_db();
	
	
	get_global_storage()->init_db();
	
	get_global_agv_instance()->exec_route=0;
    
	pthread_create(&state_port_thread, NULL, thread_state_port, nullptr);
	pthread_create(&rpt_position_thread, NULL, thread_rpt_position, nullptr);
    init_LOG_CK_UP_instance();	

    //get_log_instance()->task_msg("my first task testing");
    //get_log_instance()->task_msg("my first task testing2222222");
    //get_log_instance()->task_msg("my first task testing5555555");
    //get_global_agv_instance()->robot_Antiepidemic.find_best_path(2);
    running_network();

    delete get_global_agv_instance();
    return 0;
};
