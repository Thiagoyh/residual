//
// Created by xcy on 2020/10/19.
//

#ifndef UNTITLED_RESIDUAL_H
#define UNTITLED_RESIDUAL_H

#include <vector>
#include <Eigen/Dense>
#include <Eigen/Core>

double residual(Eigen::Vector3d position, std::vector<Eigen::Vector3d> observation, Eigen::Quaterniond q,
                std::vector<Eigen::Vector3d> map_point);

#endif //UNTITLED_RESIDUAL_H
