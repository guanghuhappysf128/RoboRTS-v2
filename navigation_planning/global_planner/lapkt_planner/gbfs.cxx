//
// Created by Miquel Ramirez on 24/10/2019.
//

#include "gbfs.hxx"

namespace roborts_global_planner::lapkt::gbfs {

    SearchAlgorithm::SearchAlgorithm(const SearchModel& m)
        : _model(m),
        _generated(0),
        _expanded(0),
        open(std::make_shared<OctileHeuristicWithBakedGoal>(m.params(), m.G)){

    }

    bool
    SearchAlgorithm::find_path(std::vector<State>& solution) {
        Node::ptr n = std::make_shared<Node>(_model.I, _generated++);
        open.insert(n);

        while ( !open.empty() ) {
            Node::ptr current = open.next( );

            if (_model.is_goal(current->state)) {
                retrieve_solution(current, solution);
                return true;
            }

            // close the node before the actual expansion so that children which are identical
            // to 'current' get properly discarded
            closed.insert(current);

            for ( int a =0; a < (int)_model.A.size(); a++ ) {
                auto[s_a, valid] = _model.next( current->state, a );
                if (!valid) continue;
                Node::ptr successor = std::make_shared<Node>(std::move(s_a), a, current, _generated++);

                if (closed.find(successor) != closed.end()) continue; // The node has already been closed
                if (open.updatable(successor)) continue; // The node is currently on the open list, we update some of its attributes but there's no need to reinsert it.

                open.insert( successor );
            }
        }
        return false;
    }

    //! Backward chaining procedure to recover a plan from a given node
    void
    SearchAlgorithm::retrieve_solution(Node::ptr node, std::vector<State>& solution) const {
        while (node->has_parent()) {
            solution.push_back(node->state);
            node = node->parent;
        }
        std::reverse( solution.begin(), solution.end() );
    }
}