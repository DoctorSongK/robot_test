#include <map>
#include <iostream>
#include <cmath>
#include <unistd.h>

#include "../common/data_transform.h"
#include "../common/configuration.h"
#include "antiepidemic_dispose.h"
#include "agv.h"


Antiepidemic_modual::Antiepidemic_modual()  {
    start_index = get_configs()->get_int("antiepidemic", "start_index", nullptr);
    if(start_index<0) start_index = 300;
    total_num = get_configs()->get_int("antiepidemic", "total_num", nullptr);
    if(total_num<0) total_num = 100;
    anti_speed = get_configs()->get_float("antiepidemic", "anti_speed", nullptr);
    if(anti_speed<0) anti_speed = 0.2;
    printf("antiepidemic start_index=%d,total_num=%d\n",start_index,total_num);

    pthread_create(&load_cap_route_threadt, NULL, load_cap_route_thread, this);/////test the route plan function
    pthread_create(&antiRouteRunthreadt, NULL, antiRouteRunthread, this);/////test the route plan function
    
    
    //load_caps_routes();
};

Antiepidemic_modual::~Antiepidemic_modual() {
  
};

void *Antiepidemic_modual::load_cap_route_thread(void *param)
{
    Antiepidemic_modual *ptr = (Antiepidemic_modual *)param;
    sleep(5);
    ptr->load_caps_routes();
    //ptr->find_best_path(11);
    return NULL;
}
int Antiepidemic_modual::get_position(Graph G, char ch)
{
    int i;
    for (i = 0; i<G.vexnum; i++)
        if (G.vexs[i] == ch)
        return i;
    return -1;
}

int Antiepidemic_modual::first_vertex(Graph G, int v)
{
    int i;

    if (v<0 || v>(G.vexnum - 1))
        return -1;

    for (i = 0; i < G.vexnum; i++)
        if (G.matrix[v][i] != 0 && G.matrix[v][i] != INF_DIJ)
        return i;

    return -1;
}

int Antiepidemic_modual::next_vertix(Graph G, int v, int w)
{
    int i;

    if (v<0 || v>(G.vexnum - 1) || w<0 || w>(G.vexnum - 1))
        return -1;

    for (i = w + 1; i < G.vexnum; i++)
        if (G.matrix[v][i] != 0 && G.matrix[v][i] != INF_DIJ)
            return i;

    return -1;
}

void Antiepidemic_modual::DFS(Graph G, int i, int *visited)
{
    int w;

    visited[i] = 1;
    printf("%c ", G.vexs[i]);
    // ????????????????????????????????????????????????????????????????????????????????????
    for (w = first_vertex(G, i); w >= 0; w = next_vertix(G, i, w))
    {
        if (!visited[w])
        DFS(G, w, visited);
    }

}

bool Antiepidemic_modual::index2count(int idex, short &count)//////??????????????????????????????????????????????????????
{
	int tempnum = NONSENSEVAL;
    //printf("search inde=%d\n",idex);
	int icyc = ( myid2point).size();
	int jj;
	for(jj=0;jj<icyc;jj++)
	{
        //printf("count=%d inde=%d\n",myid2point[jj].COUNT_ID,myid2point[jj].POINT_IDX);
		if(idex ==  myid2point[jj].POINT_IDX)
		{
		tempnum =  myid2point[jj].COUNT_ID;
		break;
		}
	}
	count = tempnum;
    //printf("search inde=%d\n",count);
	if(tempnum == NONSENSEVAL)
	return false;
	else
	return true;	

}
bool Antiepidemic_modual::count2index(int count,int &idex)//////???????????????????????????????????????????????????
{
	int tempnum = NONSENSEVAL;
	int icyc = ( myid2point).size();
	int jj;
	for(jj=0;jj<icyc;jj++)
	{
		if(count ==  myid2point[jj].COUNT_ID)
		{
		tempnum =  myid2point[jj].POINT_IDX;
		break;
		}
	}
	idex = tempnum;
	if(tempnum == NONSENSEVAL)
	return false;
	else
	return true;	

}
/*
* Dijkstra???????????????
* ???????????????(G)???"??????vs"???????????????????????????????????????
*
* ???????????????
*        G -- ???
*       vs -- ????????????(start vertex)????????????"??????vs"?????????????????????????????????
*     prev -- ???????????????????????????prev[i]?????????"??????vs"???"??????i"???????????????????????????????????????????????????"??????i"????????????????????????
*     dist -- ?????????????????????dist[i]???"??????vs"???"??????i"???????????????????????????
*/
int Antiepidemic_modual::dijkstra(Graph G, int vs, int prev[MAX_DIJ], int dist[MAX_DIJ])
{
    int i, j, k;
    int min;
    int tmp;
    int flag[MAX_DIJ];      // flag[i]=1??????"??????vs"???"??????i"?????????????????????????????????
    printf("???????????????=%d,\n",vs);
    // ?????????
    for (i = 0; i < G.vexnum; i++)
    {
        flag[i] = 0;              // ??????i?????????????????????????????????
        prev[i] = 0;              // ??????i??????????????????0???
        dist[i] = G.matrix[vs][i];// ??????i??????????????????"??????vs"???"??????i"?????????
    }

    // ???"??????vs"?????????????????????
    flag[vs] = 1;
    dist[vs] = 0;
    //printf("iiiiiivs=%d,vexnum=%d\n",vs, G.vexnum);
    // ??????G.vexnum-1????????????????????????????????????????????????
    for (i = 1; i < G.vexnum; i++)
    //i = 5;
    {
    // ??????????????????????????????
    // ??????????????????????????????????????????????????????vs???????????????(k)???
    min = INF_DIJ;
    for (j = 0; j < G.vexnum; j++)
    {
        if (flag[j] == 0 && dist[j]<min)
        {
            min = dist[j];
            k = j;
        }
    
    }
    //if (min == INF_DIJ) return -1;
    //printf("ffffffffff++%c%d-j%d-dis%d", G.vexs[k], min, k, dist[k]);
    // ??????"??????k"??????????????????????????????
    flag[k] = 1;
    // ???????????????????????????????????????
    // ???????????????"??????k???????????????"???????????????"????????????????????????????????????????????????????????????"???
    for (j = 0; j < G.vexnum; j++)
    {
    //printf("--%d",min);
    tmp = (G.matrix[k][j] == INF_DIJ ? INF_DIJ : (min + G.matrix[k][j])); // ????????????
    if (flag[j] == 0 && (tmp  < dist[j]))
    {
        dist[j] = tmp;
        prev[j] = k;
           //printf("gggggggg>>%c%d", G.vexs[j+1],k);
    }
    }
    //printf("  prev1=%d,2=%d,3=%d,4=%d,5=%d\n", prev[0], prev[1], prev[2], prev[3], prev[4]);
    //printf(">>%d\n", tmp);
    }

    // ??????dijkstra?????????????????????
/*    printf("dijkstra(%c): \n", G.vexs[vs]);
    for (i = 0; i < G.vexnum; i++)
    printf("  shortest(%c, %c)=%d\n", G.vexs[vs], G.vexs[i], dist[i]);*/

    printf("  prev1=%d,2=%d,3=%d,4=%d,5=%d,6=%d,7=%d,8=%d\n", prev[0], prev[1], prev[2], prev[3], prev[4], prev[5], prev[6], prev[7]);
    return 1;
}
int Antiepidemic_modual::searchPath(int *prev, int v, int u, int plist[],int dist[])
{

    int path1[MAX_DIJ];
    int path2[MAX_DIJ];
    int idex = 0;
    int len_cal1 = 0;
    int len_cal2 = 0;
    bool isfirst = false;
    searchPathF(prev,  v,  u, path1);
    searchPathT(prev,  v,  u, path2);
    printf("\npath1=%d-%d-%d-%d-%d\n",path1[0],path1[1],path1[2],path1[3],path1[4]);
    printf("path2=%d-%d-%d-%d-%d\n",path2[0],path2[1],path2[2],path2[3],path2[4]);
    //printf("pG.matrix=%d--%d,",My_pG.matrix[0][1],My_pG.matrix[2][3]);
    while((path1[idex + 1] != NONSENSEVAL&& path1[idex] != NONSENSEVAL))
    {
        //printf("idex=%d,idex1p=%d,changdu =%d--",idex,idex+1, My_pG.matrix[path1[idex]][path1[idex + 1]]);
        
        if (My_pG.matrix[path1[idex]][path1[idex+1]] != INF_DIJ)
        {
            printf("changdu =%d--", My_pG.matrix[path1[idex]][path1[idex + 1]]);
                len_cal1 += My_pG.matrix[path1[idex]][path1[idex+1]];
        }
        else
        {
            len_cal1 = INF_DIJ;
            break;
        }
        idex++;
    }
    idex = 0;
    while((path2[idex + 1] != NONSENSEVAL&& path2[idex] != NONSENSEVAL))
    {
        if (My_pG.matrix[path2[idex]][path2[idex+1]] != INF_DIJ)
        {
            printf("changdu =%d--", My_pG.matrix[path2[idex]][path2[idex + 1]]);
                len_cal2 += My_pG.matrix[path2[idex]][path2[idex+1]];
        }
        else
        {
            len_cal2 = INF_DIJ;
            break;
        }
        idex++;
    }
    printf("\nlen_cal1=%d,len_cal2=%d,--dist[%d]=%d\n",len_cal1,len_cal2,u,dist[u]);
    if(len_cal1 == INF_DIJ&&len_cal2 == INF_DIJ) return -1;
    if(len_cal1 == INF_DIJ&&len_cal2 != INF_DIJ) 
    if(len_cal2!=dist[u])
    return -1;
    else
    memcpy(plist,path2,sizeof(int)*(MAX_DIJ));
    if(len_cal2 == INF_DIJ&&len_cal1 != INF_DIJ) 
    if(len_cal1!=dist[u])
    return -1;
    else
    memcpy(plist,path1,sizeof(int)*(MAX_DIJ));
    if(len_cal2 != INF_DIJ&&len_cal1 != INF_DIJ)
    {
    if(len_cal1==dist[u])
    memcpy(plist,path1,sizeof(int)*(MAX_DIJ));
    else if(len_cal2==dist[u])
    memcpy(plist,path2,sizeof(int)*(MAX_DIJ));
    else
    return -1;
    }
    printf("\npath????????????\n");
    idex =0;
    while(plist[idex++]!=NONSENSEVAL)
    printf(  "->%d" , plist[idex-1] );    //?????????????????????
    printf(  "->%d" , plist[idex-1] );    //?????????????????????
    printf("\n==============\n");
    return 0;
}
void Antiepidemic_modual::searchPathF(int *prev, int v, int u, int plist[])
{

    int i = u;
    //cout << v+1 << "???????????????" << i + 1  ;
    //cout << ",?????????????????????:" << end;
    //printf(  "%d???????????????%d", v+1,i + 1);     //?????????????????????
    //printf(  "?????????????????????" );     //?????????????????????
    int idex = 0;
    int p = prev[i];
    std::vector <int> que;
    bool first_zero = false;
    //int q_insert;
    if (p == 0&& first_zero == false){
        first_zero = true;
        if(0 == u)
        p=0;
        else
        {
        que.push_back(p);
        p = prev[p];
        }

    }
    {
        //plist[idex++] = v;
        
        //printf(  "%d" , v );     //?????????????????????
        while (p != 0){
            //printf(  "->%d" , p );     //?????????????????????
            //plist[idex++] = p;
            //q_insert =p;
            que.push_back(p);
            p = prev[p];
            if(p == 0&& first_zero == false)
            {
                first_zero = true;
                if(0 == u) break;
                que.push_back(p);
                p = prev[p];
            }
        }
        //que.push_back(p);
        plist[idex++] = v;
        for(i=que.size();i>0;i--)
        {
        plist[idex++] = que[i-1];
        //printf(  "%d->%d" ,que.size(), que[i-1] );     //?????????????????????
        }
        plist[idex++] = u;
        plist[idex++] = NONSENSEVAL;
        //for(i=0;i<idex;i++)
        //printf(  "FFFF->%d" , plist[i] );    //?????????????????????
    }

    
}
void Antiepidemic_modual::searchPathT(int *prev, int v, int u, int plist[])
{

    int i = u;
    //cout << v+1 << "???????????????" << i + 1  ;
    //cout << ",?????????????????????:" << end;
    //printf(  "%d???????????????%d", v+1,i + 1);     //?????????????????????
    //printf(  "?????????????????????" );     //?????????????????????
    int idex = 0;
    int p = prev[i];
    std::vector <int> que;
    bool first_zero = true;
    //int q_insert;
    if (p == 0&& first_zero == false){
        first_zero = true;
        if(0 == u)
        p=0;
        else
        {
        que.push_back(p);
        p = prev[p];
        }

    }
    {
        //plist[idex++] = v;
        
        //printf(  "%d" , v );     //?????????????????????
        while (p != 0){
            //printf(  "->%d" , p );     //?????????????????????
            //plist[idex++] = p;
            //q_insert =p;
            que.push_back(p);
            p = prev[p];
            if(p == 0&& first_zero == false)
            {
                first_zero = true;
                if(0 == u) break;
                que.push_back(p);
                p = prev[p];
            }
        }
        //que.push_back(p);
        plist[idex++] = v;
        for(i=que.size();i>0;i--)
        {
        plist[idex++] = que[i-1];
        //printf(  "%d->%d" ,que.size(), que[i-1] );     //?????????????????????
        }
        plist[idex++] = u;
        plist[idex++] = NONSENSEVAL;
        //for(i=0;i<idex;i++)
        //printf(  "TTTTT->%d" , plist[i] );    //?????????????????????
    }

    
}
void Antiepidemic_modual::load_caps_routes()
{

    vector <ID2POINT> myid2point_temp;////
    myid2point_temp.swap(myid2point);
    myid2point.clear();	

    vector <TRAJ_ITEM> mytraj_temp;////
    mytraj_temp.swap(mytraj_list);
    mytraj_list.clear();	
    cJSON *json,*ret;
    /////////////////////////////load the caps file
    /*if((f=fopen("cap.json","rb"))==NULL)
    {
        printf("error open cap file\n");
        return ;
    }
    fseek(f,0,SEEK_END);
    len=ftell(f);
    fseek(f,0,SEEK_SET);
    data=(char*)malloc(len+1);
    fread(data,1,len,f);*/
    ifstream inf;
    ostringstream oss;
    inf.open("cap.json",ios::binary);
    oss.str("");
    oss << inf.rdbuf();
    string xml_str = oss.str();

    json=cJSON_Parse(xml_str.c_str());
    //fclose(f);
    inf.close();
    if(!json)
    {
        printf("the caps file format is not correct\n");
        //free(data);
        return ;
    }
    ID2POINT ID2POINT_temp;
    cJSON *caps_list  = cJSON_GetArrayItem(json,0);
    int total_num = cJSON_GetArraySize(json);
    printf("cJSON_GetArraySize(json)=%d\n",total_num);
    if(total_num<=1)
    {graph_ready = false; cJSON_Delete(json);return;}
    int last_id;
    int count = 0;
    float p_x,p_y,p_theta,p_num;
    //while( count < total_num ){ 
    cJSON *item;
    while( caps_list != NULL ){ 
        item = cJSON_GetObjectItem( caps_list , "x");
        p_x   = (item!=NULL)? item->valuedouble:0 ;
        item = cJSON_GetObjectItem( caps_list , "y");
        p_y = (item!=NULL)? item->valuedouble:0 ;
        item = cJSON_GetObjectItem( caps_list , "theta");
        p_theta = (item!=NULL)? item->valuedouble:0 ;
        item = cJSON_GetObjectItem( caps_list , "id");
        p_num = (item!=NULL)? item->valueint:0 ;
        printf("the cur caps x=%f y=%f theta=%f id=%f\n",p_x,p_y,p_theta,p_num);
        ID2POINT_temp.COUNT_ID = count;
        ID2POINT_temp.POINT_IDX = p_num;
        ID2POINT_temp.xp = p_x;
        ID2POINT_temp.yp = p_y;
        ID2POINT_temp.theta = p_theta;
        myid2point.push_back(ID2POINT_temp);
        count++;
        caps_list = caps_list->next ;
    }
    //printf("end loading the caps\n");
    //free(data);
    cJSON_Delete(json);
    //printf("start loading the routes\n");
////////////////////load the routes file 

    inf.open("route.json",ios::binary);
    oss.str("");
    oss << inf.rdbuf();
    string xml_str2 = oss.str();
    cJSON * json2=cJSON_Parse(xml_str2.c_str());
    inf.close();
    if(!json2)
    {
        printf("the route file format is not correct\n");
        //free(data);
        return ;
    }
    total_num = cJSON_GetArraySize(json2);
    if(total_num<=0)
    {graph_ready = false; cJSON_Delete(json2);return;}
    //printf("route size (json2)=%d\n",total_num);

    cJSON *route_list  = cJSON_GetArrayItem(json2,0);

    double xp,yp,theta;
    int cap_id,cap_id_next;
    cJSON *cap_arry = NULL;
    cJSON *cap_list = NULL;

    My_pG.vexnum = MAX_DIJ;
    int ii,jj;
	// 1. ?????????"???"?????????
    for (ii = 0; ii < My_pG.vexnum; ii++)
    {
        for (jj = 0; jj < My_pG.vexnum; jj++)
        {
            if (ii == jj)
            My_pG.matrix[ii][jj] = 0;
            else
            My_pG.matrix[ii][jj] = INF_DIJ;
        }
    }

    while( route_list != NULL ){ 
        
        cap_arry     = cJSON_GetObjectItem( route_list, "caps");  //cap?????????
        
        total_num = cJSON_GetArraySize(cap_arry);
        printf("route size (caps)=%d\n",total_num);
        if(total_num<=1)
        {graph_ready = false; cJSON_Delete(json2);return;}

        if( NULL != cap_arry )
        cap_list  = cap_arry->child;

        if( cap_list != NULL ){
        item = cJSON_GetObjectItem( cap_list , "id");
        cap_id = (item!=NULL)? item->valueint:0 ;
        item = cJSON_GetObjectItem( cap_list , "x");
        p_x = (item!=NULL)? item->valuedouble:0 ;
        item = cJSON_GetObjectItem( cap_list , "y");
        p_y = (item!=NULL)? item->valuedouble:0 ;
        item = cJSON_GetObjectItem( cap_list , "theta");
        p_theta = (item!=NULL)? item->valuedouble:0 ;
        }
        cap_list = cap_list->next ;
        while( cap_list != NULL ){
        //for(ii = 1; ii < total_num; ii++){
            item = cJSON_GetObjectItem( cap_list , "id");
            cap_id_next = (item!=NULL)? item->valueint:0 ;
            item = cJSON_GetObjectItem( cap_list , "x");
            xp = (item!=NULL)? item->valuedouble:0 ;
            item = cJSON_GetObjectItem( cap_list , "y");
            yp = (item!=NULL)? item->valuedouble:0 ;
            item = cJSON_GetObjectItem( cap_list , "theta");
            theta = (item!=NULL)? item->valuedouble:0 ;
            item = cJSON_GetObjectItem( cap_list , "speed");
            float speed_p = (item!=NULL)? item->valuedouble:0 ;
            item = cJSON_GetObjectItem( cap_list , "dir");
            char dir_p = (item!=NULL)? item->valueint:0 ;
            item = cJSON_GetObjectItem( cap_list , "collision");
            char collision_p = (item!=NULL)? item->valueint:0 ;
            TRAJ_ITEM traj_temp;

            traj_temp.st_id = cap_id;traj_temp.ed_id = cap_id_next;
            traj_temp.speed = speed_p*0.001;traj_temp.dir = dir_p;
            traj_temp.collision = collision_p;
            traj_temp.xp1 = p_x;traj_temp.yp1 = p_y;traj_temp.theta1 = p_theta;
            traj_temp.xp2 = xp;traj_temp.yp2 = yp;traj_temp.theta2 = theta;
            mytraj_list.push_back(traj_temp);
            traj_temp.st_id = cap_id_next;traj_temp.ed_id = cap_id;     
            traj_temp.xp2 = p_x;traj_temp.yp2 = p_y;traj_temp.theta2 = p_theta;
            traj_temp.xp1 = xp;traj_temp.yp1 = yp;traj_temp.theta1 = theta;
            mytraj_list.push_back(traj_temp);

            xp -= p_x ; 
            yp -= p_y ;
            
            double len = sqrt(xp*xp+yp*yp)*1000;    

            short st_idx = cap_id;
            short ed_idx = cap_id_next;
            short st_count,ed_count;
            if((index2count(st_idx,st_count)==false)||(index2count(ed_idx,ed_count)==false))
            {return ;}
            if(st_count>MAX_DIJ||ed_count>MAX_DIJ){return ;}
            My_pG.matrix[st_count][ed_count] = len;
            My_pG.matrix[ed_count][st_count] = len;
            printf("the route  p1=%d p2=%d,dx=%f,dy=%f len=%f \n",cap_id,cap_id_next,xp,yp,len);
            cap_id = cap_id_next;
            p_x = traj_temp.xp1;
            p_y = traj_temp.yp1;
            p_theta= theta;
            cap_list = cap_list->next ;
        }
            route_list = route_list->next;
    }
    //free(data);
    cJSON_Delete(json2);


    graph_ready = true; 
    return ;

}
int Antiepidemic_modual::load_caps_disinfect() {

    int first_route = get_configs()->get_int("antiepidemic", "first_route", nullptr);
    if(first_route<0) first_route = 0;
    int second_route = get_configs()->get_int("antiepidemic", "second_route", nullptr);
    if(second_route<0) second_route = 0;
    
    ifstream inf;
    ostringstream oss;

    cJSON *json,*ret;
    inf.open("route.json",ios::binary);
    oss.str("");
    oss << inf.rdbuf();
    string xml_str2 = oss.str();
    cJSON * json2=cJSON_Parse(xml_str2.c_str());
    inf.close();
    if(!json2)
    {
        printf("the route file format is not correct\n");
        //free(data);
        return -1;
    }
    int total_num = cJSON_GetArraySize(json2);
    if(total_num<=0)
    {cJSON_Delete(json2);return -1;}
    //printf("route size (json2)=%d\n",total_num);
    cJSON *route_list  = cJSON_GetArrayItem(json2,0);
    caps_disinfect.clear();
    
    int last_id;
    int list_len = 0;
    cJSON *item;
    float p_x,p_y,p_theta;
    int p_num;
    cJSON *cap_arry = NULL;
    cJSON *cap_list = NULL;

    while( route_list != NULL ){ 
        item = cJSON_GetObjectItem( cap_list , "id");
        last_id = (item!=NULL)? item->valueint:0 ;
        if(last_id>0&&((last_id==first_route)||(last_id==second_route)))
        {
        cap_arry     = cJSON_GetObjectItem( route_list, "caps");  //cap?????????
        total_num = cJSON_GetArraySize(cap_arry);
        printf("route size (caps)=%d\n",total_num);
        if(total_num<=1)
        { cJSON_Delete(json2);return -1;}

        if( NULL != cap_arry )
        cap_list  = cap_arry->child;

        while( cap_list != NULL ){
        //for(ii = 1; ii < total_num; ii++){
            item = cJSON_GetObjectItem( cap_list , "id");
            p_num = (item!=NULL)? item->valueint:0 ;
            item = cJSON_GetObjectItem( cap_list , "x");
            p_x = (item!=NULL)? item->valuedouble:0 ;
            item = cJSON_GetObjectItem( cap_list , "y");
            p_y = (item!=NULL)? item->valuedouble:0 ;
            item = cJSON_GetObjectItem( cap_list , "theta");
            p_theta = (item!=NULL)? item->valuedouble:0 ;
            item = cJSON_GetObjectItem( cap_list , "speed");
            float speed_p = (item!=NULL)? item->valuedouble:0 ;
            item = cJSON_GetObjectItem( cap_list , "dir");
            char dir_p = (item!=NULL)? item->valueint:0 ;
            item = cJSON_GetObjectItem( cap_list , "collision");
            char collision_p = (item!=NULL)? item->valueint:0 ;
            printf("the caps disinfect caps x=%f y=%f theta=%f id=%d\n",p_x,p_y,p_theta,p_num);
            CapPoint caps_t(p_num,p_x,p_y,p_theta);
            caps_disinfect.push_back(caps_t);
            cap_list = cap_list->next ;
        }
        }
        route_list = route_list->next ;
    }

    list_p_num = caps_disinfect.size();
    if(list_p_num<=0) return -1;

    cJSON_Delete(json2);

    return 1;
};
int Antiepidemic_modual::findListPos(int pos,CapPoint &pos_ret)
{
    int ret = 1;
    if(list_p_num ==0 || pos < 0){
        ret = -1;
    }
    if(pos == 0)
    pos_ret = caps_disinfect.front();

    if(pos>(list_p_num-1)) ret = -1;
    std::list<CapPoint>::iterator itc;
    for(itc=caps_disinfect.begin();itc!=caps_disinfect.end();itc++){
    {
        pos_ret = *itc;
        if(pos<=0)break;
        pos--;
    }
    }
    
    return ret;
}
int Antiepidemic_modual::find_nearst_pos()
{
    int near_index = 0;
    int find_index = 0;
    bool first_find = true;
    float dist = 0;
    float dist_min = 0;
    Position pose = get_global_agv_instance()->expolate_current_position();
    
    for(CapPoint &point : caps_disinfect)
    {
        if(first_find)
        {dist_min = pow(point.x-pose.x,2)+pow(point.y-pose.y,2);
        near_index = 0;
        first_find = false;
        }
        dist = pow(point.x-pose.x,2)+pow(point.y-pose.y,2);
        if(dist_min>dist)
        {near_index = find_index;dist_min = dist;}

        find_index++;
    }

    if(list_p_num<=0) return -1;

    if(near_index>(list_p_num-1)) 
    return -1;
    else
    return near_index;

}

void *Antiepidemic_modual::antiRouteRunthread(void *param)
{
    Antiepidemic_modual *ptr = (Antiepidemic_modual *)param;
    sleep(30);
    int DelaySecond = get_configs()->get_int("antiepidemic", "secd_delay", nullptr);
    int Delay_cnt = DelaySecond;
    int anti_enable= get_configs()->get_int("antiepidemic", "anti_enable", nullptr);
    if(anti_enable<0) anti_enable = 0;
    if (DelaySecond<0) DelaySecond = 1800;
    int RuningIndex = 0;
    CapPoint cur_cap;
    while(true)
    {
    sleep(1);
    if(anti_enable==0) break;
    if(get_global_agv_instance()->board_sts._manual_auto!=1) continue;
    if(RuningIndex>=(ptr->list_p_num-1)){RuningIndex=0; Delay_cnt = 0;}
    if(get_global_agv_instance()->my_runcurve->busying)
    {
        continue;/////????????????????????????????????????????????????????????????????????????
    }
    Delay_cnt++;

    if(Delay_cnt>DelaySecond)////???????????????????????????????????????
    {
        if(ptr->load_caps_disinfect()<0){ sleep(10);continue;}////?????????????????????

        if(ptr->findListPos(RuningIndex,cur_cap)>0)
        {
            ptr->find_best_path(cur_cap.id);
            sleep(1);
            RuningIndex++;
        }
        else
        {sleep(1);
        continue;
        }
        /*if(ptr->load_caps_disinfect()<0) continue;////?????????????????????
        int run_start = ptr->find_nearst_pos();/////?????????????????????????????????????????????????????????
        if(run_start<0) continue;

        vector<Position> caps;
        vector<float>    speed;
        vector<char>    dir;
        vector<char>    occi;
        vector<char>    inv_cps;
        CapPoint cur_cap;
        int cur_cap_index=0;
        for (int idex=0;idex<ptr->list_p_num;idex++)
        {
            cur_cap_index = run_start+idex;
            if(cur_cap_index>=ptr->list_p_num) cur_cap_index -= ptr->list_p_num;
            if(ptr->findListPos(cur_cap_index,cur_cap)>0){
            caps.push_back(Position(cur_cap.id,cur_cap.x,cur_cap.y,cur_cap.theta));
            speed.push_back(ptr->anti_speed);
            dir.push_back(0);
            occi.push_back(1);
            inv_cps.push_back(0);
            }
        }
        get_global_agv_instance()->stop_navigating();
        get_global_agv_instance()->start_path(caps,speed,dir,0,occi,inv_cps);*/
        //ptr->run_the_antiepidmic_path();
    }


    }
    return NULL;
}
void Antiepidemic_modual::run_the_antiepidmic_path() {
    if(load_caps_disinfect()<0) return;////?????????????????????
    int run_start = find_nearst_pos();/////?????????????????????????????????????????????????????????
    if(run_start<0) return;

    vector<Position> caps;
    vector<float>    speed;
    vector<char>    dir;
    vector<char>    occi;
    vector<char>    inv_cps;
    vector<char>	do_acts;
    vector<float>	fix_angs;

    CapPoint cur_cap;
    int cur_cap_index;
    for (int idex=0;idex<list_p_num;idex++)
    {
        cur_cap_index = run_start+idex;
        if(cur_cap_index>=list_p_num) cur_cap_index -= list_p_num;
        if(findListPos(cur_cap_index,cur_cap)>0){
        caps.push_back(Position(cur_cap.id,cur_cap.x,cur_cap.y,cur_cap.theta));
        speed.push_back(anti_speed);
        dir.push_back(0);
        occi.push_back(1);
        inv_cps.push_back(0);
        do_acts.push_back(0);
        fix_angs.push_back(0);
        }
    }
    get_global_agv_instance()->stop_navigating();
    get_global_agv_instance()->start_path(caps,speed,dir,0,occi,inv_cps,do_acts,fix_angs);
    return ;
};
int Antiepidemic_modual::find_cap_from_id(int index,ID2POINT &p_ret)
{
    int icyc = myid2point.size();
    int near_index=NONSENSEVAL;
    for(int j=0;j<icyc;j++)
    {
        if(index == myid2point[j].POINT_IDX)
        {near_index = j;break;}
    }
    p_ret = myid2point[near_index];
    if(near_index==NONSENSEVAL)
    {
        printf("????????????????????????cap???\n");
    return -1;
    }
    else 
    return 1;
}
int Antiepidemic_modual::is_station_isolate(int target_id)
{/////??????????????????????????????
    int itemp=NONSENSEVAL;
    int j=0;
    int j_end = 0;
    int icyc = mytraj_list.size();
    if(icyc<=0) return itemp;
    for(j=0;j<icyc;j++)
    {
        if((target_id == mytraj_list[j].ed_id) || (target_id == mytraj_list[j].st_id))
        {//////????????????????????????????????????????????????????????????????????????
        itemp = j;
        break;
        }	
    }
    printf("?????????target_id=%d,??????ret=%d\n",target_id,itemp);
    return itemp;	
}
int Antiepidemic_modual::rotate_fix_angle(double ang)
{
    Position pose = get_global_agv_instance()->expolate_current_position();
    vector<Position> caps;
    vector<float>    speed;
    vector<char>    dir;
    vector<char>    occi;
    vector<char>    inv_cps;
    vector<char>	do_acts;
    vector<float>	fix_angs;    
    ang = ang/180*M_PI;
    Position delt_p = Position(1,0,0,ang);
    pose = pose + delt_p;
    if (pose.theta>M_PI) pose.theta-=2*M_PI;
    if (pose.theta<-M_PI) pose.theta+=2*M_PI;
/*    caps.push_back(Position(1,pose.x,pose.y,pose.theta));
    speed.push_back(0.5);
    dir.push_back(0);
    occi.push_back(0);
*/
    caps.push_back(Position(1,pose.x,pose.y,pose.theta));
    speed.push_back(0.5);
    dir.push_back(0);
    occi.push_back(1);
    inv_cps.push_back(0);
    do_acts.push_back(0);
    fix_angs.push_back(0);
    get_global_agv_instance()->stop_navigating();
    get_global_agv_instance()->start_path(caps,speed,dir,0,occi,inv_cps,do_acts,fix_angs);
}
int Antiepidemic_modual::find_best_path(int target_id)
{
    int ini_index;
    short end_index;
    int ret = find_nearst_cap(ini_index);
    //printf("the robot nearst index=%d\n",ini_index);
    int prev[MAX_DIJ] = {0};
    int dist[MAX_DIJ] = {0};
    int pathp[MAX_DIJ] = {0};
    int path_p;
    float speed_st= anti_speed;
    vector<int>    path_list;
    if (ret<0) return -1;
    if(!graph_ready) return -2;////????????????????????????????????????
    //printf("the robot target id=%d\n",target_id);
    vector<Position> caps;
    vector<float>    speed;
    vector<char>    dir;
    vector<char>    occi;
    vector<char>    inv_cps;
    vector<char>	do_acts;
    vector<float>	fix_angs;  
    ID2POINT p_cap;
    int m_cur_p;
    count2index(ini_index,m_cur_p);
    printf("cur p_index=%d\n",m_cur_p);
    if((is_station_isolate(target_id)!=NONSENSEVAL)&&(is_station_isolate(m_cur_p)!=NONSENSEVAL)){////??????????????????????????????????????????
    
    if(dijkstra(My_pG,ini_index,prev,dist)<0) //searchPath
    {
        printf("dijkstra???????????????\n");
        //return -1;
    }
    if(index2count(target_id,end_index)==false) return -1;////???????????????

    //printf("the robot target index=%d\n",end_index);
    if(searchPath(prev,ini_index,end_index,pathp,dist)==-1) return -1;//////???????????????????????????
    //////???????????????????????????????????????cap???
    //printf("the robot target 11111index=%d\n",end_index);

    for(int i_dij=0;i_dij<MAX_DIJ;i_dij++)
    {
        if(pathp[i_dij] != NONSENSEVAL)
        {
        if(!count2index(pathp[i_dij],path_p)) return -1;/////????????????????????????????????????????????????????????????
            path_list.push_back(path_p);
            
            if(find_cap_from_id(path_p,p_cap)>0)
            {
            caps.push_back(Position(path_p,p_cap.xp,p_cap.yp,p_cap.theta));
            speed.push_back(speed_st);
            dir.push_back(0);
            occi.push_back(1);
            inv_cps.push_back(0);
            do_acts.push_back(0);
            fix_angs.push_back(0);
            }
            printf("??????id=%d %f %f %f\n",path_p,p_cap.xp,p_cap.yp,p_cap.theta);
        }
        else
        {
            //size = i_dij;
            printf("end??????id=%d ",NONSENSEVAL);
            break;
        }
			
    }
    get_global_agv_instance()->stop_navigating();
    get_global_agv_instance()->start_path(caps,speed,dir,0,occi,inv_cps,do_acts,fix_angs);
    }
    else{////???????????????????????????

        Position pose = get_global_agv_instance()->expolate_current_position();
        caps.push_back(pose);
        speed.push_back(speed_st);
        dir.push_back(0);
        occi.push_back(1);
        inv_cps.push_back(0);
        do_acts.push_back(0);
        fix_angs.push_back(0);
        printf("??????id=%d %f %f %f\n",target_id,pose.x,pose.y,pose.theta);

        if(find_cap_from_id(target_id,p_cap)>0)
        {
        caps.push_back(Position(target_id,p_cap.xp,p_cap.yp,p_cap.theta));
        speed.push_back(speed_st);
        dir.push_back(0);
        occi.push_back(1);
        inv_cps.push_back(0);
        do_acts.push_back(0);
        fix_angs.push_back(0);
        printf("??????id=%d %f %f %f\n",target_id,p_cap.xp,p_cap.yp,p_cap.theta);
        }
    get_global_agv_instance()->stop_navigating();
       get_global_agv_instance()->start_path(caps,speed,dir,0,occi,inv_cps,do_acts,fix_angs); 
     ;
    }
    return 1;
}
int Antiepidemic_modual::find_nearst_cap(int &idx_dij)
{
    Position pose = get_global_agv_instance()->expolate_current_position();
    //myid2point
    //Position pointt;
    int icyc = myid2point.size();
    int near_index = 0;
    int find_index = 0;
    bool first_find = true;
    float dist = 0;
    float dist_min = 0;
    //printf("posx=%f  y=%f  \n",pose.x,pose.y);
    if(icyc<=0) return NULL;
    for(int j=icyc-1;j>=0;j--)
    {
        
        dist = pow(myid2point[j].xp-pose.x,2)+pow(myid2point[j].yp-pose.y,2);
        //printf("j=%d, x=%f  y=%f  dist=%f\n",j,myid2point[j].xp,myid2point[j].yp,dist);
        if(first_find){
        dist_min = dist;
        near_index = j;
        first_find = false;
        }
        if(dist_min>dist)
        {near_index = j;dist_min = dist;}
    }
    //printf(",,,,,,near_index=%d,dist_min=%f\n",near_index,dist_min);
    idx_dij = myid2point[near_index].COUNT_ID;

    printf("idx_dij=%d,POINT_IDX=%d\n",idx_dij,myid2point[near_index].POINT_IDX);
    if(near_index<0) 
    return -1;
    else
    return myid2point[near_index].POINT_IDX;

}
