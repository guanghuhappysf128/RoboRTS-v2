//
// Created by Miquel Ramirez on 22/10/2019.
//
#pragma once

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <CGAL/Point_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
namespace roborts_global_planner{
namespace lapkt {
    typedef CGAL::Exact_predicates_inexact_constructions_kernel     Kernel;
    typedef Kernel::Vector_2                                        Vector;
    typedef Kernel::Point_2                                         Point;
}
}
