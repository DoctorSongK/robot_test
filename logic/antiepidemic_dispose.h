#ifndef ANTIEPIDEMIC_MODULE_H
#define ANTIEPIDEMIC_MODULE_H

#include <iostream>
#include <fstream>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <list>

#include <Eigen/Core>

#include "../common/pose.h"
#include "../common/capture_point.h"


#define MAX_DIJ         (1024)                 // 矩阵最大容量
#define INF_DIJ         (1000000)        		// 最大值1000000
#define NONSENSEVAL		(65535)//////轨迹点索引无效的值

// 图的邻接矩阵存储
typedef struct _graph
{
    short vexs[MAX_DIJ];       // 顶点集合
    int vexnum;           // 顶点数
    int edgnum;           // 边数
    int matrix[MAX_DIJ][MAX_DIJ]; // 邻接矩阵
}Graph, *PGraph;

// 边的结构体
typedef struct _EdgeData
{
    short start; // 边的起点
    short end;   // 边的终点
    int weight; // 边的权重
}EData;
typedef struct 
{
    short COUNT_ID; // 点的ID
    short POINT_IDX;   // 对应的索引
    float xp;
    float yp;
    float theta;
}ID2POINT;

typedef struct 
{
    short st_id; // 点的ID
    short ed_id;   // 对应的索引
    float speed;
    char  dir;
    char  collision;
    float xp1;
    float yp1;
    float theta1;
    float xp2;
    float yp2;
    float theta2;
}TRAJ_ITEM;

class Antiepidemic_modual 
{
public:
    Antiepidemic_modual();
    ~Antiepidemic_modual();
    std::list<CapPoint> caps_disinfect;
    void run_the_antiepidmic_path();
    void load_caps_routes();
    int find_best_path(int target_id);
    int rotate_fix_angle(double ang);
private:    
    int load_caps_disinfect();
    int find_nearst_pos();
	int findListPos(int pos,CapPoint &pos_ret);
    int start_index;
    int total_num;
    bool graph_ready = false;
	int list_p_num;
    float anti_speed = 0.2;
    Graph My_pG;//////路径表，用于路径规划
    std::vector <ID2POINT> myid2point;////点到索引的映射表
    int find_cap_from_id(int index,ID2POINT &p_ret);
    std::vector <TRAJ_ITEM> mytraj_list;

    int find_nearst_cap(int &idx_dij);/////找出距离当前点最近的cap点索引
    bool index2count(int idex, short &count);
    bool count2index(int count,int &idex);

    int get_position(Graph G, char ch);/////
    int first_vertex(Graph G, int v);
    int next_vertix(Graph G, int v, int w);
    void DFS(Graph G, int i, int *visited);
    int dijkstra(Graph G, int vs, int prev[MAX_DIJ], int dist[MAX_DIJ]);
    int searchPath(int *prev, int v, int u, int plist[],int dist[]);////查找最短路径，-1查找失败，0 查找成功
    void searchPathT(int *prev, int v, int u, int plist[]);
    void searchPathF(int *prev, int v, int u, int plist[]);
    int is_station_isolate(int target_id);
    pthread_t load_cap_route_threadt;
    static void* load_cap_route_thread(void* param);

    pthread_t antiRouteRunthreadt;
    static void* antiRouteRunthread(void* param);////防疫路径启动线程
};

#endif
