//
// Created by Miquel Ramirez on 23/10/2019.
//

#pragma once

#include <memory>
#include "state_model.hxx"
#include "open_lists.hxx"
#include "octile_heuristic.hxx"

namespace roborts_global_planner::lapkt {

    namespace gbfs {

        class Node {
        public:
            using ptr = std::shared_ptr<Node>;

            State state;

            int action;
            long g;
            long h;

            std::shared_ptr<Node> parent;

        public:
            Node() = delete;

            ~Node() {}

            Node(const Node &) = delete;

            Node(Node &&) = delete;

            Node &operator=(const Node &) = delete;

            Node &operator=(Node &&) = delete;

            //! Constructor with full copying of the state (expensive)
            Node(const State &s, unsigned long gen_order = 0)
                    : state(s), action(-1), g(0), h(0), parent(nullptr) {}

            //! Constructor with move of the state (cheaper)
            Node(State &&_state, int _action,
                 std::shared_ptr<Node> _parent,
                 unsigned long gen_order = 0) :
                    state(std::move(_state)), action(_action), g(0), h(0), parent(_parent) {
                g = 1 + parent->g;
            }

            bool has_parent() const { return parent != nullptr; }

            //! Print the node into the given stream
            friend std::ostream &
            operator<<(std::ostream &os, const Node &object) { return object.print(os); }

            std::ostream &print(std::ostream &os) const {
                os << "{@ = " << this << ", s = " << state.pos << ", parent = " << parent << "}";
                return os;
            }

            bool operator==(const Node &o) const { return state == o.state; }

            std::size_t hash() const { return state.hash(); }

            bool operator>(const Node &other) const {
                if (h > other.h) return true;
                return false;
            }

            // MRJ: This is part of the required interface of the Heuristic
            template<typename Heuristic>
            long evaluate_with(Heuristic &heuristic) {

                h = heuristic(state);

                // If the heuristic is > 0, then we must have a relaxed plan and at least some atoms in the 1st layer of it.
                // TODO But this holds only for certain types of searches, other search engines using this very same node type
                // won't add anything to the 'relevant' vector
                // assert(h <= 0 || _relevant.size() > 0);
                return h;
            }

            void inherit_heuristic_estimate() {
                if (parent) h = parent->h;
            }

            //! What to do when an 'other' node is found during the search while 'this' node is already in
            //! the open list
            void update_in_open_list(ptr other) {
                if (other->g < this->g) {
                    this->g = other->g;
                    this->action = other->action;
                    this->parent = other->parent;
                }
            }

            bool dead_end() const { return h == std::numeric_limits<long>::max(); }


        };

        class SearchAlgorithm {
        private:

            const SearchModel&  _model;
            long long           _generated;
            long long           _expanded;

        public:
            DelayedEvaluationOpenList<Node, OctileHeuristicWithBakedGoal::ptr>  open;
            std::unordered_set<Node::ptr, node_hash<Node::ptr>, node_equal_to<Node::ptr>> closed;
            SearchAlgorithm(const SearchModel& m);
            ~SearchAlgorithm() = default;

            bool find_path(std::vector<State>& path);
            void retrieve_solution(Node::ptr node, std::vector<State>& solution) const;
        };

    }
}
