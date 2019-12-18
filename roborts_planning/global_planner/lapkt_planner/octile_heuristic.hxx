//
// Created by Miquel Ramirez on 23/10/2019.
//
#pragma once

#include "grid_map.hxx"
#include <cmath>
#include "state_model.hxx"
namespace roborts_global_planner{
namespace lapkt {

    class OctileHeuristic {

    public:
        typedef     unsigned        cost_t;

    private:
        const MapParameters& _params;
        static constexpr double ROOT_OF_TWO = 1.414213562f;
        static constexpr cost_t ONE = 10000;
        static constexpr cost_t Q_ROOT_TWO = ROOT_OF_TWO * ONE;

    public:
        OctileHeuristic(const MapParameters& params)
            : _params(params) {}

        ~OctileHeuristic() = default;

        inline cost_t
        h(const State& s, const State& g)
        {
            // NB: precision loss when warthog::cost_t is an integer
            double dx = std::abs(s.pos[1]-g.pos[1]);
            double dy = std::abs(s.pos[0]-g.pos[0]);
            if(dx < dy)
            {
                return dx * Q_ROOT_TWO + (dy - dx) * ONE;
            }
            return dy * Q_ROOT_TWO + (dx - dy) * ONE;
        }

        inline cost_t
        operator()(const State& s, const State& g) {
            return h(s, g);
        }

    };

    class OctileHeuristicWithBakedGoal {
    private:
        OctileHeuristic func;
        State           goal;
    public:
        typedef std::shared_ptr<OctileHeuristicWithBakedGoal> ptr;

        OctileHeuristicWithBakedGoal(const MapParameters& params, const State& g)
            : func(params), goal(g) {

        }

        inline OctileHeuristic::cost_t
        operator()(const State& s) {
            return func(s, goal);
        }
    };

}
}