//
// Created by Miquel Ramirez on 23/10/2019.
//

#pragma once

#include <Eigen/Core>
#include "geometry.hxx"
namespace roborts_global_planner{
namespace lapkt {

    typedef Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>     OccupancyMap;

    struct MapParameters {
        Vector          origin;
        int             width;
        int             height;
        double          res;
    };
}
}