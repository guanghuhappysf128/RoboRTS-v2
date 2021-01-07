//
// Created by Miquel Ramirez on 23/10/2019.
//

#include "state_model.hxx"
#include "grid_map.hxx"

namespace roborts_global_planner{

namespace lapkt {

    void
    State::update_hash() {
        std::size_t seed = 0;
        boost::hash_combine(seed, pos[0]);
        boost::hash_combine(seed, pos[1]);
        _hash = seed;
    }

    const State SearchModel::null_state = State(-1, -1);

    SearchModel::SearchModel(const MapParameters& params, const OccupancyMap& omap, const Point& _I, const Point& _G)
            : _params(params),
              _omap(omap),
              I(_I.x(), _I.y()), G(_G.x(), _G.y()) {

    }

    bool
    SearchModel::out_of_bounds(const MapParameters& params, const Point& pt) {
        return (pt.x() > (params.width * params.res))
                || (pt.x() < 0.0)
                || (pt.y() > (params.height * params.res))
                || (pt.y() < 0.0);
    }

    int
    SearchModel::cell_index(const MapParameters& params, const Point& pt) {
        auto x = pt.x() / params.res;
        auto y = pt.y() / params.res;
        return (int)x * params.width + (int)y;
    }

    bool
    SearchModel::valid(const State& s) const {
        return s.pos[0] >= 0 && s.pos[0] < _params.height
               && s.pos[1] >= 0 && s.pos[1] < _params.width
               && free(s)
               && at_least_one_neighbour_free(s);
    }

    bool
    SearchModel::is_goal(const State& s) const {
        return G.pos == s.pos;
    }

    SearchModel::Transition
    SearchModel::next(const State& s, int a) const {
        State succ;
        succ.pos = s.pos + A[a];
        succ.update_hash();
        if (!valid(succ)) {
            return std::make_tuple(null_state, false);
        }
        return std::make_tuple(succ, true);
    }


}
}
