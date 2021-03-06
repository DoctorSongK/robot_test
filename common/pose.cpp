
#include <cmath>

#include "pose.h"

Position operator-(const Position& a, const Position& b) {
    long timestamp = a.timestamp;
    if(b.timestamp > a.timestamp) {
        timestamp = b.timestamp;
    }
    return Position(timestamp, a.x-b.x , a.y-b.y, a.theta-b.theta,b.rot_rf1,b.rot_rb1,b.rot_rf2,b.rot_rb2);
}

Position operator+(const Position& a, const Position& b) {
    long timestamp = a.timestamp;
    if(b.timestamp > a.timestamp) {
        timestamp = b.timestamp;
    }
    return Position(timestamp, a.x+b.x , a.y+b.y, a.theta+b.theta,b.rot_rf1,b.rot_rb1,b.rot_rf2,b.rot_rb2);
}

Position operator*(const Position& a, const Position& b) {
    long timestamp = a.timestamp;
    if(b.timestamp > a.timestamp) {
        timestamp = b.timestamp;
    }
    double c = cos(a.theta);
    double s = sin(a.theta);
    double x = b.x * c - b.y * s + a.x;
    double y = b.x * s + b.y * c + a.y;
           double theta = a.theta+b.theta;
          while(theta < -M_PI) {
            theta += (M_PI * 2);
        }
        while(theta > M_PI ) {
            theta -= (M_PI * 2);
        }

    return Position(timestamp, x , y, theta,b.rot_rf1,b.rot_rb1,b.rot_rf2,b.rot_rb2);
}

Position operator/(const Position& p, const Position& a) {
    long timestamp = a.timestamp;
    if(p.timestamp > a.timestamp) {
        timestamp = p.timestamp;
    }
    double x = p.x - a.x;
    double y = p.y - a.y;
    double c = cos(a.theta);
    double s = sin(a.theta);
          double theta = p.theta-a.theta;
          while(theta < -3.141592653589793) {
            theta += (3.141592653589793 * 2);
        }
        while(theta > 3.141592653589793 ) {
            theta -= (3.141592653589793 * 2);
        }
    return Position(timestamp, x * c + y * s , -x * s + y * c, theta,a.rot_rf1,a.rot_rb1,a.rot_rf2,a.rot_rb2);
}

bool operator<(const Position& l, const Position& r) {
    if(l.x != r.x) {
        return (l.x < r.x);
    }
    else if(l.y != r.y) {
        return (l.y < r.y);
    }
    else {
        return (l.theta < r.theta);
    }
}

bool operator==(const Position& a, const Position& b) {
    return ((a.x == b.x) && (a.y == b.y) && (a.theta == b.theta));
}
