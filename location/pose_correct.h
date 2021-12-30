#ifndef AGV_POS_CORRECT_H
#define AGV_POS_CORRECT_H

#include <vector>
#include <Eigen/Core>

#include "../common/grid_map.h"
#include "../common/sensor_data.h"
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
using namespace Eigen;
using namespace std;

typedef struct {
	//Position odom_raw_pos[10];
	//Position scan_match_raw_pos[10];
	vector<Position> odom_raw_pos;
	vector<Position> scan_match_pos;
	vector<Position> amcl_pos;
	vector<Position> land_mark_pos;
	vector<Position> two_dims_pos;
} raw_pos_for_correct;


Position raw_pos_correct(raw_pos_for_correct & rawpos,int num);
Position cross_pos_correct(raw_pos_for_correct & rawpos,int &id_ret,bool landmark_mapping,Position pos_predict);
bool check_pos_near(Position p0,Position p1,float len);
#endif
