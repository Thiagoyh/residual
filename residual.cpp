//
// Created by xcy on 2020/10/19.
//

#include "residual.h"
#include "find_closest.h"

using namespace std;

double residual(Eigen::Vector3d position, std::vector<Eigen::Vector3d> observation, Eigen::Quaterniond q,
                vector<Eigen::Vector3d> map_point)
{
    double residual = 0;
    for(vector<Eigen::Vector3d>::iterator iter = observation.begin(); iter != observation.end(); ++iter)
    {
        Eigen::Vector3d w_point = point_trans((*iter), q, position);
        Eigen::Vector3d cor_point = find_closest(w_point, map_point);
        double distance = (w_point[0] - cor_point[0]) * (w_point[0] - cor_point[0])
                          + (w_point[1] - cor_point[1]) * (w_point[1] - cor_point[1])
                          + (w_point[2] - cor_point[2]) * (w_point[2] - cor_point[2]);
        if (distance > 3)
           residual += 3;
        else residual += distance;
        if(cor_point == w_point)
            residual += 3;
    }
    return residual;
}