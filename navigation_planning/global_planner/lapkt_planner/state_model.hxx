//
// Created by Miquel Ramirez on 23/10/2019.
//

#pragma once

#include "geometry.hxx"
#include "grid_map.hxx"
namespace roborts_global_planner{

namespace lapkt {

    class State {
    public:
        Eigen::Matrix<int, 2, 1>    pos;
        std::size_t                 _hash;

        State() = default;
        State(const State&) = default;
        State(int i, int j) {
            pos = {i, j};

        }
        ~State() = default;

        std::size_t hash() const {
            return _hash;
        }

        void update_hash();

        bool operator==(const State& o) const {
            return pos[0] == o.pos[0] && pos[1] == o.pos[1];
        }
    };

    class SearchModel {
    private:
        const MapParameters& _params;
        const OccupancyMap&  _omap;

    public:
        typedef std::tuple<State, bool> Transition;

        State   I;
        State   G;
        static  const   State null_state;

        const std::vector<Eigen::Matrix<int, 2, 1>> A = {
                {1, 0}, {1,1}, {0,1}, {1,1}, {-1, 0}, {-1, -1}, {0, -1}, {-1, -1}
        };

        SearchModel(const MapParameters& params, const OccupancyMap& omap, const Point& _I, const Point& _G);
        ~SearchModel() = default;

        static bool    out_of_bounds(const MapParameters&, const Point& pt);
        static int     cell_index(const MapParameters&, const Point& pt);

        const MapParameters& params() const { return _params; }

        [[nodiscard]] inline bool free(const State& s) const {
            return _omap(s.pos[0], s.pos[1]);
        }

        [[nodiscard]] inline bool
        at_least_one_neighbour_free(const State& s) const {
            auto imin = std::max(0, s.pos[0]-1);
            auto imax = std::min(_params.height-1, s.pos[0]+1);
            auto jmin = std::max(0, s.pos[1]-1);
            auto jmax = std::min(_params.width-1, s.pos[1]+1);

            for (auto i = imin; imin <= imax; i++)
                for(auto j = jmin; jmin <= jmax; j++)
                    if (_omap(i,j)) return true;

            return false;
        }

        [[nodiscard]] bool valid(const State& s) const;
        [[nodiscard]] bool is_goal(const State& s) const;

        [[nodiscard]] Transition
        next(const State& s, int a) const;
    };


}

}