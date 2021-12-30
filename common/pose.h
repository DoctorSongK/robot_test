#ifndef AGV_POSITION_H
#define AGV_POSITION_H
#include <cmath>
#include <vector>

class Position {
public:
    Position() : Position(0, 0., 0., 0.) {};
    Position(long time_us, double x, double y, double theta)
        :timestamp(time_us), x(x), y(y), theta(theta){
        while(theta < -M_PI) {
            theta += (M_PI * 2);
        }
        while(theta > M_PI ) {
            theta -= (M_PI * 2);
        }
    };
    Position(long time_us, double x, double y, double theta,double _rf1,double _rb1,double _rf2,double _rb2)
        :timestamp(time_us), x(x), y(y), theta(theta),rot_rf1(_rf1),rot_rb1(_rb1),rot_rf2(_rf2),rot_rb2(_rb2){
        while(theta < -M_PI) {
            theta += (M_PI * 2);
        }
        while(theta > M_PI ) {
            theta -= (M_PI * 2);
        }
    };
    /***********************************************四舵轮******************************************************/
	Position(long time_us, double x, double y, double theta, double _rlf1, double _rrf1, double _rlb1, double _rrb1, double _rlf2, double _rrf2, double _rlb2, double _rrb2)
		:timestamp(time_us), x(x), y(y), theta(theta), rot_rlf1(_rlf1), rot_rrf1(_rrf1), rot_rlb1(_rlb1), rot_rrb1(_rrb1), rot_rlf2(_rlf2), rot_rrf2(_rrf2), rot_rlb2(_rlb2), rot_rrb2(_rrb2) {
		while (theta < -M_PI) {
			theta += (M_PI * 2);
		}
		while (theta > M_PI) {
			theta -= (M_PI * 2);
		}
	};
	/***********************************************************************************************************/
    Position & operator=(const Position &p) {
        timestamp = p.timestamp;
        x = p.x;
        y = p.y;
        theta = p.theta;
        delt_t = p.delt_t;
        rot_rf1 = p.rot_rf1;
        rot_rb1 = p.rot_rb1;
        rot_rf2 = p.rot_rf2;
        rot_rb2 = p.rot_rb2;
    /***********************************************四舵轮******************************************************/
		rot_rlf1 = p.rot_rlf1;
		rot_rrf1 = p.rot_rrf1;
		rot_rlb1 = p.rot_rlb1;
		rot_rrb1 = p.rot_rrb1;
		rot_rlf2 = p.rot_rlf2;
		rot_rrf2 = p.rot_rrf2;
		rot_rlb2 = p.rot_rlb2;
		rot_rrb2 = p.rot_rrb2;
	/***********************************************************************************************************/
        return *this;
        return *this;
    }

    long    timestamp;
    double  x;
    double  y;
    double  theta;
    long  delt_t;
    double  rot_rf1;
    double  rot_rb1;
    double  rot_rf2;
    double  rot_rb2;
    /******************************************四舵轮************************************************/
	double rot_rlf1;//左前轮摆角
	double rot_rrf1;//右前轮摆角
	double rot_rlb1;//左后轮摆角
	double rot_rrb1;//右后轮摆角
	double rot_rlf2;
	double rot_rrf2;
	double rot_rlb2;
	double rot_rrb2;
	/************************************************************************************************/
    double norm() {
        double t = theta;
        if(theta > M_PI) {
            t = M_PI * 2 - theta;
        }
        return x + y + t;
    };

    void to_char_array(std::vector<char> *output) {
        output->reserve(output->size() + sizeof(Position));
        char *p = (char*)this;
        for(int i = 0;i < sizeof(Position);i++) {
            output->push_back(p[i]);
        }
    }
    void from_char_array(char *buf, int size) {
        if(size >= sizeof(Position)) {
            Position *p = (Position*)buf;
            timestamp = p->timestamp;
            x = p->x;
            y = p->y;
            theta = p->theta;
            delt_t =  p->delt_t ;
            rot_rf1 = p->rot_rf1;
            rot_rb1 = p->rot_rb1;
            rot_rf2 = p->rot_rf2;
            rot_rb2 = p->rot_rb2;
    /******************************************四舵轮************************************************/
            rot_rlf1 = p->rot_rlf1;
            rot_rrf1 = p->rot_rrf1;
            rot_rlb1 = p->rot_rlb1;
            rot_rrb1 = p->rot_rrb1;
            rot_rlf2 = p->rot_rlf2;
            rot_rrf2 = p->rot_rrf2;
            rot_rlb2 = p->rot_rlb2;
            rot_rrb2 = p->rot_rrb2;
    /************************************************************************************************/
        }
    }
};

Position operator-(const Position& a, const Position& b);

Position operator+(const Position& a, const Position& b);

Position operator*(const Position& a, const Position& b);

Position operator/(const Position& p, const Position& a);

bool operator<(const Position& l, const Position& r);
bool operator==(const Position& a, const Position& b);

#endif

