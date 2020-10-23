//
// Created by xcy on 2020/10/19.
//

#include "find_closest.h"

using namespace std;

Eigen::Vector3d point_trans(Eigen::Vector3d ob_point, Eigen::Quaterniond q, Eigen::Vector3d position)
{
    Eigen::Vector3d v_rotated = q * ob_point;
    return (v_rotated + position);
}

Eigen::Vector3d find_closest(Eigen::Vector3d w_point , std::vector<Eigen::Vector3d> map_point)
{
    double distance = 1000;
    Eigen::Vector3d match(0, 0, 0);
    for(vector<Eigen::Vector3d>::iterator iter = map_point.begin(); iter != map_point.end(); ++iter)
    {
        double temp = (w_point[0] - (*iter)[0]) * (w_point[0] - (*iter)[0]) +
                (w_point[1] - (*iter)[1]) * (w_point[1] - (*iter)[1]) +
                (w_point[2] - (*iter)[2]) * (w_point[2] - (*iter)[2]);
        //distance = (temp < distance ? temp : distance);
        if (temp <= distance){
            distance = temp;
            match = (*iter);
        }
//        if(distance > 3){
//            match = w_point;
//        }
    }
    return match;
}


