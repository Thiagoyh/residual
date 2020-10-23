//
// Created by xcy on 2020/10/19.
//

#ifndef UNTITLED_FIND_CLOSEST_H
#define UNTITLED_FIND_CLOSEST_H

#include <vector>
#include <Eigen/Dense>
#include <Eigen/Core>

Eigen::Vector3d point_trans(Eigen::Vector3d ob_point, Eigen::Quaterniond q, Eigen::Vector3d position);

Eigen::Vector3d find_closest(Eigen::Vector3d w_point , std::vector<Eigen::Vector3d> map_point);



#endif //UNTITLED_FIND_CLOSEST_H
