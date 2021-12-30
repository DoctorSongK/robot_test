
#include <cmath>
#include <iostream>
#include <stdio.h>
#include <ceres/ceres.h>
#include <ceres/cubic_interpolation.h>
#include <Eigen/Core>

#include "../location/pose_correct.h"
bool check_pos_near(Position p0,Position p1,float len)
{
    float delta_xx = fabs(p0.x-p1.x);
    float delta_yy = fabs(p0.y-p1.y);
    float delta_theta = fabs(p0.theta-p1.theta);
	if((delta_xx<len)&&(delta_yy<len)&&(delta_theta<len))
	return true;
	else
	return false;
}
Position cross_pos_correct(raw_pos_for_correct & rawpos,int &id_ret,bool landmark_mapping,Position pos_predict)
{
    std::vector<std::vector<float>> vec_rawp{ {rawpos.odom_raw_pos[0].x,	rawpos.odom_raw_pos[0].y},
                    { rawpos.odom_raw_pos[1].x,	rawpos.odom_raw_pos[1].y},
                    { rawpos.odom_raw_pos[2].x,	rawpos.odom_raw_pos[2].y},
                    { rawpos.odom_raw_pos[3].x,	rawpos.odom_raw_pos[3].y},
                    { rawpos.odom_raw_pos[4].x,	rawpos.odom_raw_pos[4].y},
                    { rawpos.odom_raw_pos[5].x,	rawpos.odom_raw_pos[5].y},
                    { rawpos.odom_raw_pos[6].x,	rawpos.odom_raw_pos[6].y},
                    { rawpos.odom_raw_pos[7].x,	rawpos.odom_raw_pos[7].y},
                    { rawpos.odom_raw_pos[8].x,	rawpos.odom_raw_pos[8].y},
                    { rawpos.odom_raw_pos[9].x,	rawpos.odom_raw_pos[9].y} };
    std::vector<std::vector<float>> vec_scanp{ {rawpos.scan_match_pos[0].x,	rawpos.scan_match_pos[0].y},
                    { rawpos.scan_match_pos[1].x,	rawpos.scan_match_pos[1].y},
                    { rawpos.scan_match_pos[2].x,	rawpos.scan_match_pos[2].y},
                    { rawpos.scan_match_pos[3].x,	rawpos.scan_match_pos[3].y},
                    { rawpos.scan_match_pos[4].x,	rawpos.scan_match_pos[4].y},
                    { rawpos.scan_match_pos[5].x,	rawpos.scan_match_pos[5].y},
                    { rawpos.scan_match_pos[6].x,	rawpos.scan_match_pos[6].y},
                    { rawpos.scan_match_pos[7].x,	rawpos.scan_match_pos[7].y},
                    { rawpos.scan_match_pos[8].x,	rawpos.scan_match_pos[8].y},
                    { rawpos.scan_match_pos[9].x,	rawpos.scan_match_pos[9].y} };
    std::vector<std::vector<float>> vec_amclp{ {rawpos.amcl_pos[0].x,	rawpos.amcl_pos[0].y},
                    { rawpos.amcl_pos[1].x,	rawpos.amcl_pos[1].y},
                    { rawpos.amcl_pos[2].x,	rawpos.amcl_pos[2].y},
                    { rawpos.amcl_pos[3].x,	rawpos.amcl_pos[3].y},
                    { rawpos.amcl_pos[4].x,	rawpos.amcl_pos[4].y},
                    { rawpos.amcl_pos[5].x,	rawpos.amcl_pos[5].y},
                    { rawpos.amcl_pos[6].x,	rawpos.amcl_pos[6].y},
                    { rawpos.amcl_pos[7].x,	rawpos.amcl_pos[7].y},
                    { rawpos.amcl_pos[8].x,	rawpos.amcl_pos[8].y},
                    { rawpos.amcl_pos[9].x,	rawpos.amcl_pos[9].y} };
    std::vector<std::vector<float>> vec_lmkp{ {rawpos.land_mark_pos[0].x,	rawpos.land_mark_pos[0].y},
                    { rawpos.land_mark_pos[1].x,	rawpos.land_mark_pos[1].y},
                    { rawpos.land_mark_pos[2].x,	rawpos.land_mark_pos[2].y},
                    { rawpos.land_mark_pos[3].x,	rawpos.land_mark_pos[3].y},
                    { rawpos.land_mark_pos[4].x,	rawpos.land_mark_pos[4].y},
                    { rawpos.land_mark_pos[5].x,	rawpos.land_mark_pos[5].y},
                    { rawpos.land_mark_pos[6].x,	rawpos.land_mark_pos[6].y},
                    { rawpos.land_mark_pos[7].x,	rawpos.land_mark_pos[7].y},
                    { rawpos.land_mark_pos[8].x,	rawpos.land_mark_pos[8].y},
                    { rawpos.land_mark_pos[9].x,	rawpos.land_mark_pos[9].y} };
	
    const int rows{ 10 }, cols{ 2 };
    const int nsamples = rows;
    float scale = 1. / (nsamples /*- 1*/);
    float sp_v_raw,sp_v_scan,sp_v_amcl,sp_v_lmk,sp_v_min;
    Eigen::MatrixXf tmp(rows, cols);
    std::vector<float> vec_raw;
    for (int i = 0; i < rows; ++i) 
        vec_raw.insert(vec_raw.begin() + i * cols, vec_rawp[i].begin(), vec_rawp[i].end());

    Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> mraw(vec_raw.data(), rows, cols);
    Eigen::MatrixXf meanr = mraw.colwise().mean();
    //cout<<"the meanr"<<endl<<meanr<<endl;
    for (int y = 0; y < rows; ++y) 
        for (int x = 0; x < cols; ++x) 
            tmp(y, x) = mraw(y, x) - meanr(0, x);
    Eigen::MatrixXf covarraw = (tmp.adjoint() * tmp) /*/ float(nsamples - 1)*/;
    EigenSolver<MatrixXf> esraw(covarraw);
    MatrixXf D_raw = esraw.pseudoEigenvalueMatrix();
    //MatrixXf V = es.pseudoEigenvectors();
    sp_v_raw = D_raw(0,0)+D_raw(1,1);

    std::vector<float> vec_scan;
    for (int i = 0; i < rows; ++i) 
        vec_scan.insert(vec_scan.begin() + i * cols, vec_scanp[i].begin(), vec_scanp[i].end());

    Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> mscan(vec_scan.data(), rows, cols);
    Eigen::MatrixXf means = mscan.colwise().mean();
    //cout<<"the means"<<endl<<means<<endl;
    for (int y = 0; y < rows; ++y) 
        for (int x = 0; x < cols; ++x) 
            tmp(y, x) = mscan(y, x) - means(0, x);
    Eigen::MatrixXf covarscan = (tmp.adjoint() * tmp) /*/ float(nsamples - 1)*/;
    EigenSolver<MatrixXf> esscan(covarscan);
    MatrixXf D_scan = esscan.pseudoEigenvalueMatrix();
    sp_v_scan = D_scan(0,0)+D_scan(1,1);

    std::vector<float> vec_amcl;
    for (int i = 0; i < rows; ++i) 
        vec_amcl.insert(vec_amcl.begin() + i * cols, vec_amclp[i].begin(), vec_amclp[i].end());
    Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> mamcl(vec_amcl.data(), rows, cols);
    Eigen::MatrixXf meana = mamcl.colwise().mean();
	//cout<<"the meana"<<endl<<meana<<endl;
    for (int y = 0; y < rows; ++y) 
        for (int x = 0; x < cols; ++x) 
            tmp(y, x) = mamcl(y, x) - meana(0, x);
    Eigen::MatrixXf covaramcl = (tmp.adjoint() * tmp) /*/ float(nsamples - 1)*/;
    EigenSolver<MatrixXf> esamcl(covaramcl);
    MatrixXf D_amcl = esamcl.pseudoEigenvalueMatrix();
    sp_v_amcl = D_amcl(0,0)+D_amcl(1,1);

    std::vector<float> vec_lmk;
    for (int i = 0; i < rows; ++i) 
        vec_lmk.insert(vec_lmk.begin() + i * cols, vec_lmkp[i].begin(), vec_lmkp[i].end());
    Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> mlmk(vec_lmk.data(), rows, cols);
    Eigen::MatrixXf meanl = mlmk.colwise().mean();
    //cout<<"the meanl"<<endl<<meanl<<endl;
    for (int y = 0; y < rows; ++y) 
        for (int x = 0; x < cols; ++x) 
            tmp(y, x) = mlmk(y, x) - meanl(0, x);
    Eigen::MatrixXf covarlmk = (tmp.adjoint() * tmp) /*/ float(nsamples - 1)*/;
    EigenSolver<MatrixXf> eslmk(covarlmk);
    MatrixXf D_lmk = eslmk.pseudoEigenvalueMatrix();
    sp_v_lmk = D_lmk(0,0)+D_lmk(1,1);

    id_ret = 1;
    sp_v_min = fabs(sp_v_raw-sp_v_scan);

    float delta_xx ;
    float delta_yy ;
    /*id_ret = 0;
    if((fabs(means(0)-meana(0))<0.1)&&(fabs(means(1)-meana(1))<0.1))
    {
    if(check_pos_near(rawpos.scan_match_pos[9],pos_predict,0.2))
    id_ret = 1;/////如果amcl和scan-matching结果一致，判断scan-matching结果与预测结果，合适则作为最终结果
    }
    if(id_ret<=0)
    {/////scan-matching与amcl结果不确定
    if((fabs(meanr(0)-means(0))<0.1)&&(fabs(meanr(1)-means(1))<0.1)){
    if(check_pos_near(rawpos.scan_match_pos[9],pos_predict,0.1))
    id_ret = 1;
    }
    else if((fabs(meanr(0)-meana(0))<0.1)&&(fabs(meanr(1)-meana(1))<0.1)){
    if(check_pos_near(rawpos.amcl_pos[9],pos_predict,0.1))
    id_ret = 2;
    }
    else if((fabs(meanr(0)-meanl(0))<0.1)&&(fabs(meanr(1)-meanl(1))<0.1)){
    if(check_pos_near(rawpos.land_mark_pos[9],pos_predict,0.1))
    id_ret = 3;
    }
    else
    id_ret = 4;
    }*/
    if((fabs(means(0)-meana(0))<0.1)&&(fabs(means(1)-meana(1))<0.1))
    id_ret = 1;/////如果amcl和scan-matching结果一致，取scan-matching结果作为最终结果
	else if((fabs(meanr(0)-means(0))<0.1)&&(fabs(meanr(1)-means(1))<0.1))
    id_ret = 1;
    else if((fabs(meanr(0)-meana(0))<0.1)&&(fabs(meanr(1)-meana(1))<0.1))
    id_ret = 2;
    //else if((fabs(meanr(0)-meanl(0))<0.1)&&(fabs(meanr(1)-meanl(1))<0.1))
    //id_ret = 3;
    else
    id_ret = 4;

    float vel_ang = fabs((rawpos.odom_raw_pos[8].theta-rawpos.odom_raw_pos[9].theta)*1000000/(rawpos.odom_raw_pos[9].timestamp-rawpos.odom_raw_pos[8].timestamp));
    delta_xx = fabs(rawpos.scan_match_pos[9].x-rawpos.amcl_pos[9].x);
    delta_yy = fabs(rawpos.scan_match_pos[9].y-rawpos.amcl_pos[9].y);
    //if(sp_v_raw<0.001&&(vel_ang>0.1)&&(delta_xx<0.05)&&(delta_yy<0.05)) id_ret = 2;

    id_ret = 2;
    if(id_ret == 1) ;//printf("the scan position is used\n");
    else if(id_ret == 2);//printf("the amcl position is used!!!!!!!!!!!!!!!!!!!!!!!\n");
    else if(id_ret == 3)printf("the land mark position is used~~~~~~~~~~~~~~~~~~~\n");
    else if(id_ret == 4)printf("the raw position is used-----------------\n");

    if(id_ret == 1) return rawpos.scan_match_pos[9];
    else if(id_ret == 2) return rawpos.amcl_pos[9];
    else if(id_ret == 3) return rawpos.land_mark_pos[9];
    else if(id_ret == 4) return pos_predict;
	//else if(id_ret == 4) return rawpos.odom_raw_pos[9];
}
class RTCostFunctor2D {//////            ռ     תƽ Ʒ    Ĳв  
public:
	RTCostFunctor2D(double xo, double yo, double xd, double yd)
		: xorig(xo), yorig(yo), xdest(xd), ydest(yd) {}

	template <typename T>
	bool operator()(const T* const pose, T* residual) const {
		residual[0] = xorig*cos(pose[2]) - sin(pose[2])*yorig + pose[0] - xdest;
		residual[1] = sin(pose[2])*xorig + cos(pose[2])*yorig + pose[1] - ydest;
		return true;
	}

private:
	RTCostFunctor2D(const RTCostFunctor2D&) = delete;
	RTCostFunctor2D& operator=(const RTCostFunctor2D&) =
		delete;

	const double xorig;
	const double yorig;
	const double xdest;
	const double ydest;
};

class CalAngFunctor2D {//////            ռ  ĽǶ ƫ      Ĳв  
public:
	CalAngFunctor2D(double ao, double ad)
		: Aorig(ao), Adest(ad) {}

	template <typename T>
	bool operator()(const T* const delt_a, T* residual) const {
		residual[0] = Adest - Aorig - delt_a;
		return true;
	}

private:
	CalAngFunctor2D(const CalAngFunctor2D&) = delete;
	CalAngFunctor2D& operator=(const CalAngFunctor2D&) =
		delete;

	const double Aorig;
	const double Adest;

};
//---------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------

Position raw_pos_correct(raw_pos_for_correct & rawpos, int num)
{
	double opt_buf[3];
	Position ori_odom = rawpos.odom_raw_pos[0];
	Position ori_scan = rawpos.scan_match_pos[0];
	for (int i = 0; i < 10; i++)
	{///////     ݵ        д           ͬ        
		//printf("     ԭʼodo    %f=%f=%f===scan    %f=%f=%f\n",rawpos.odom_raw_pos[i].x,rawpos.odom_raw_pos[i].y,rawpos.odom_raw_pos[i].theta,rawpos.scan_match_raw_pos[i].x,rawpos.scan_match_raw_pos[i].y,rawpos.scan_match_raw_pos[i].theta);
		//rawpos.odom_raw_pos[i] = rawpos.odom_raw_pos[i] - ori_odom;
		//rawpos.scan_match_raw_pos[i] = rawpos.scan_match_raw_pos[i] - ori_odom;
	}
	////////   ȼ   һ    ʼλ     ݣ     cere Ż  ĳ ֵ
	/*double thetao = atan2(rawpos.odom_raw_pos[9].y - rawpos.odom_raw_pos[0].y, rawpos.odom_raw_pos[9].x - rawpos.odom_raw_pos[0].x);
	double thetad = atan2(rawpos.scan_match_raw_pos[9].y - rawpos.scan_match_raw_pos[0].y, rawpos.scan_match_raw_pos[9].x - rawpos.scan_match_raw_pos[0].x);
	double del_tehta = thetad - thetao;
	///////////////////////////////
	double cdel_theta = cos(del_tehta);
	double sdel_theta = sin(del_tehta);
	double mid_xo = (rawpos.odom_raw_pos[9].x + rawpos.odom_raw_pos[0].x)*0.5;
	double mid_yo = (rawpos.odom_raw_pos[9].y + rawpos.odom_raw_pos[0].y)*0.5;

	double rmid_xo = cdel_theta *mid_xo - sdel_theta *mid_yo;
	double rmid_yo = sdel_theta *mid_xo + cdel_theta *mid_yo;

	double delx = (rawpos.scan_match_raw_pos[9].x + rawpos.scan_match_raw_pos[0].x)*0.5 - rmid_xo;
	double dely = (rawpos.scan_match_raw_pos[9].y + rawpos.scan_match_raw_pos[0].y)*0.5 - rmid_yo;;
	double delt = del_tehta;*/

	double delx = ori_scan.x-ori_odom.x;
	double dely = ori_scan.y-ori_odom.y ;
	double delt = ori_scan.theta-ori_odom.theta;

	opt_buf[0] = delx;
	opt_buf[1] = dely;
	opt_buf[2] = delt;
	printf("From (%.3f, %.3f, %.3f)\n", delx, dely, delt);
	ceres::Solver::Options options;
	options.max_num_iterations = 10;
	options.linear_solver_type = ceres::DENSE_QR;
	ceres::Problem problem;
	double xo, yo, xd, yd, ao, ad;
	for (int j = 0; j < num; ++j)
	{
		xo = rawpos.odom_raw_pos[j].x;
		yo = rawpos.odom_raw_pos[j].y;

		xd = rawpos.scan_match_pos[j].x;
		yd = rawpos.scan_match_pos[j].y;

		ao = rawpos.odom_raw_pos[j].theta;
		ad = rawpos.scan_match_pos[j].theta;
		problem.AddResidualBlock(
			new ceres::AutoDiffCostFunction<RTCostFunctor2D, 2, 3>(
			new RTCostFunctor2D(xo, yo, xd, yd)),
			nullptr,
			opt_buf);

	}

	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);
	double mid_xo ;
	double mid_yo ;
	double cdel_theta;
	double sdel_theta;

	delt = opt_buf[2];
	delx = opt_buf[0];
	dely = opt_buf[1];
	cdel_theta = cos(delt);
	sdel_theta = sin(delt);
	mid_xo = rawpos.odom_raw_pos[9].x;
	mid_yo = rawpos.odom_raw_pos[9].y;
	double xdest = cdel_theta *mid_xo - sdel_theta *mid_yo + delx ;
	double ydest = sdel_theta *mid_xo + cdel_theta *mid_yo + dely ;
	double thedest = rawpos.scan_match_pos[9].theta;
	// std::cout << summary.BriefReport() << "\n";
	// printf("From (%.3f, %.3f, %.3f)\n", init.x, init.y, init.theta);
	printf("To   (%.3f, %.3f, %.3f)\n", delx, dely, delt);
	printf("Ŀ  λ      Ϊ  (%.3f, %.3f, %.3f)\n", xdest, ydest, delt);


	return Position(rawpos.scan_match_pos[9].timestamp, xdest, ydest, thedest );
}


