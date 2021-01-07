/*
Lightweight Automated Planning Toolkit
Copyright (C) 2015

<contributors>
Miquel Ramirez <miquel.ramirez@rmit.edu.au>
</contributors>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <vector>
#include <cassert>
#include <functional>
#include <queue>
#include <memory>
#include <unordered_set>
namespace roborts_global_planner{
namespace lapkt {

//! We need to define custom hash, equality and comparison functions for the node-pointer type.

    template <typename NodePT>
    struct node_comparer { bool operator()(const NodePT& n1, const NodePT& n2) const { return *n1 > *n2; } };

    template <typename NodePT>
    struct node_hash { size_t operator() (const NodePT& node) const { return node->hash(); } };

    template <typename NodePT>
    struct node_equal_to { bool operator() (const NodePT& n1, const NodePT& n2) const { return *n1 == *n2; } };


//! An classical open list augmented with a hash table to allow for fast updates
//! of the nodes contained in the list.
    template <typename NodeT,
            typename NodePT = std::shared_ptr<NodeT>,
            typename ComparerT = node_comparer<NodePT>
    >
    class UpdatableOpenList : public std::priority_queue<NodePT, std::vector<NodePT>, ComparerT>
    {
    public:
        //! Constructor
        explicit UpdatableOpenList()
                : _index()
        {}

        virtual ~UpdatableOpenList() = default;

        // Disallow copy, but allow move
        UpdatableOpenList(const UpdatableOpenList&) = default;
        UpdatableOpenList(UpdatableOpenList&&) = default;
        UpdatableOpenList& operator=(const UpdatableOpenList&) = default;
        UpdatableOpenList& operator=(UpdatableOpenList&&) = default;

        bool insert(const NodePT& node) {
            if ( node->dead_end() ) return false;
            this->push( node );
            _index.insert( node );
            return true;
        }

        bool contains(const NodePT& node) const {
            return _index.find(node) != _index.end();
        }

        //! Check if the open list already contains a node 'previous' referring to the same state.
        //! If that is the case, there'll be no need to reinsert the node, and we signal so returning true.
        //! If, in addition, 'previous' had a higher g-value, we do an in-place modification of it.
        bool updatable(const NodePT& node) {
            auto it = _index.find(node);
            if (it == _index.end()) return false; // No node with the same state is in the open list

            // Else the node was already in the open list and we might want to update it

            // @TODO: Very important: updating g is correct provided that
            // the order of the nodes in the open list is not changed by
            // updating it. This is an open problem with the design,
            // and a more definitive solution needs to be found (soon).
            (*it)->update_in_open_list(node);

            return true;
        }

        //! Extract the "next" node from the data structure and return it
        NodePT next() {
            assert(!this->empty());
            NodePT node = this->top();
            this->pop();
            _index.erase(node);
            return node;
        }

    protected:
        //! An index of the nodes with are in the open list at any moment, for faster access
        using node_unordered_set = std::unordered_set<NodePT, node_hash<NodePT>, node_equal_to<NodePT>>;
        node_unordered_set _index;
    };

    template <  typename NodeT,
            typename HeuristicPT,
            typename NodePT = std::shared_ptr<NodeT>,
            typename ComparerT = node_comparer<NodePT>
    >
    class ClassicOpenList : public std::priority_queue<NodePT, std::vector<NodePT>, ComparerT>
    {
    public:
        //! Constructor
        explicit ClassicOpenList(HeuristicPT h)
                : _heuristic(h), _index()
        {}

        virtual ~ClassicOpenList() = default;

        // Disallow copy, but allow move
        ClassicOpenList(const ClassicOpenList&) = default;
        ClassicOpenList(ClassicOpenList&&) = default;
        ClassicOpenList& operator=(const ClassicOpenList&) = default;
        ClassicOpenList& operator=(ClassicOpenList&&) = default;

        bool insert(const NodePT& node) {
            node->evaluate_with(*_heuristic);
            if ( node->dead_end() ) return false;
            this->push( node );
            _index.insert( node );
            return true;
        }

        bool contains(const NodePT& node) const {
            return _index.find(node) != _index.end();
        }

        //! Check if the open list already contains a node 'previous' referring to the same state.
        //! If that is the case, there'll be no need to reinsert the node, and we signal so returning true.
        //! If, in addition, 'previous' had a higher g-value, we do an in-place modification of it.
        bool updatable(const NodePT& node) {
            auto it = _index.find(node);
            if (it == _index.end()) return false; // No node with the same state is in the open list

            // Else the node was already in the open list and we might want to update it

            // @TODO: Very important: updating g is correct provided that
            // the order of the nodes in the open list is not changed by
            // updating it. This is an open problem with the design,
            // and a more definitive solution needs to be found (soon).
            (*it)->update_in_open_list(node);

            return true;
        }

        //! Extract the "next" node from the data structure and return it
        NodePT next() {
            assert(!this->empty());
            NodePT node = this->top();
            this->pop();
            _index.erase(node);
            return node;
        }

    protected:
        //! An index of the nodes with are in the open list at any moment, for faster access
        using node_unordered_set = std::unordered_set<NodePT, node_hash<NodePT>, node_equal_to<NodePT>>;
        node_unordered_set _index;
        HeuristicPT       _heuristic;
    };

    template <  typename NodeT,
            typename HeuristicPT,
            typename NodePT = std::shared_ptr<NodeT>,
            typename ComparerT = node_comparer<NodePT>
    >
    class DelayedEvaluationOpenList : public std::priority_queue<NodePT, std::vector<NodePT>, ComparerT>
    {
    public:
        //! Constructor
        explicit DelayedEvaluationOpenList(HeuristicPT h)
                : _heuristic(h), _index()
        {}

        virtual ~DelayedEvaluationOpenList() = default;

        // Disallow copy, but allow move
        DelayedEvaluationOpenList(const DelayedEvaluationOpenList&) = default;
        DelayedEvaluationOpenList(DelayedEvaluationOpenList&&) = default;
        DelayedEvaluationOpenList& operator=(const DelayedEvaluationOpenList&) = default;
        DelayedEvaluationOpenList& operator=(DelayedEvaluationOpenList&&) = default;

        bool insert(const NodePT& node) {
            if (node->parent == nullptr)
                node->evaluate_with(*_heuristic);
            else
                node->inherit_heuristic_estimate();
            if ( node->dead_end() ) return false;
            this->push( node );
            _index.insert( node );
            return true;
        }

        bool contains(const NodePT& node) const {
            return _index.find(node) != _index.end();
        }

        //! Check if the open list already contains a node 'previous' referring to the same state.
        //! If that is the case, there'll be no need to reinsert the node, and we signal so returning true.
        //! If, in addition, 'previous' had a higher g-value, we do an in-place modification of it.
        bool updatable(const NodePT& node) {
            auto it = _index.find(node);
            if (it == _index.end()) return false; // No node with the same state is in the open list

            // Else the node was already in the open list and we might want to update it

            // @TODO: Very important: updating g is correct provided that
            // the order of the nodes in the open list is not changed by
            // updating it. This is an open problem with the design,
            // and a more definitive solution needs to be found (soon).
            (*it)->update_in_open_list(node);

            return true;
        }

        //! Extract the "next" node from the data structure and return it
        NodePT next() {
            assert(!this->empty());
            NodePT node = this->top();
            this->pop();
            _index.erase(node);
            if (node->parent != nullptr)
                node->evaluate_with(*_heuristic);
            return node;
        }

    protected:
        //! An index of the nodes with are in the open list at any moment, for faster access
        using node_unordered_set = std::unordered_set<NodePT, node_hash<NodePT>, node_equal_to<NodePT>>;
        HeuristicPT       _heuristic;
        node_unordered_set _index;
    };


//! A simple wrapper around a priority_queue, for compatibility
    template <typename NodeT,
            typename NodePT = std::shared_ptr<NodeT>,
            typename ComparerT = node_comparer<NodePT>
    >
    class SimpleOpenList : public std::priority_queue<NodePT, std::vector<NodePT>, ComparerT>
    {
    public:
        //! Constructor
        explicit SimpleOpenList() = default;
        ~SimpleOpenList() = default;

        // Disallow copy, but allow move
        SimpleOpenList(const SimpleOpenList&) = default;
        SimpleOpenList(SimpleOpenList&&) = default;
        SimpleOpenList& operator=(const SimpleOpenList&) = default;
        SimpleOpenList& operator=(SimpleOpenList&&) = default;

        bool insert(const NodePT& node) {
            if ( node->dead_end() ) return false;
            this->push( node );
            return true;
        }

        //! Extract the "next" node from the data structure and return it
        NodePT next() {
            assert(!this->empty());
            NodePT node = this->top();
            this->pop();
            return node;
        }
    };


//! A simple wrapper around a queue, for compatibility
    template <typename NodeT,
            typename NodePT = std::shared_ptr<NodeT>>
    class SimpleQueue : public std::queue<NodePT> {
    public:
        explicit SimpleQueue() = default;
        ~SimpleQueue() = default;
        SimpleQueue(const SimpleQueue&) = default;
        SimpleQueue(SimpleQueue&&) = default;
        SimpleQueue& operator=(const SimpleQueue&) = default;
        SimpleQueue& operator=(SimpleQueue&&) = default;

        bool insert(const NodePT& node) {
            this->push(node);
            return true;
        }

        //! Extract the "next" node from the data structure and return it
        NodePT next() {
            assert(!this->empty());
            NodePT node = this->front();
            this->pop();
            return node;
        }
    };


//! An acceptor that always accepts to insert any given state
    template <typename NodeT>
    class QueueAcceptorI {
    public:
        virtual ~QueueAcceptorI() = default;

        //! Accept any state
        virtual bool accept(NodeT& n) = 0;
    };


//! A queue where elements can be searched for efficiently
    template <typename NodeT,
            typename NodePT = std::shared_ptr<NodeT>>
    class SearchableQueue : public std::queue<NodePT> {
        using QueueAcceptorT = QueueAcceptorI<NodeT>;
    public:
        //! Constructor
        explicit SearchableQueue(QueueAcceptorT* acceptor = nullptr)
                : _index(), _acceptor(acceptor)
        {}

        virtual ~SearchableQueue() = default;

        // Disallow copy, but allow move
        SearchableQueue(const SearchableQueue&) = default;
        SearchableQueue(SearchableQueue&&) = default;
        SearchableQueue& operator=(const SearchableQueue&) = default;
        SearchableQueue& operator=(SearchableQueue&&) = default;

        bool insert(const NodePT& node) {
            if (_acceptor && !_acceptor->accept(*node)) return false;
            this->push( node );
            _index.insert( node );
            return true;
        }

        bool contains(const NodePT& node) const {
            return _index.find(node) != _index.end();
        }

        //! Returns a pointer to a node which is identical to the given node and was already in the list,
        //! if such a node exist, or nullptr otherwise
        virtual NodePT seek(NodePT& node) {
            auto it = this->_index.find(node);
            return (it == this->_index.end()) ? nullptr : *it;
        }

        //! For the sake of template compatibility
        bool updatable(const NodePT& node) { return contains(node); }

        //! Extract the "next" node from the data structure and return it
        NodePT next() {
            assert(!this->empty());
            NodePT node = this->front();
            this->pop();
            _index.erase(node);
            return node;
        }

    protected:
        //! An index of the nodes with are in the open list at any moment, for faster access
        using node_unordered_set = std::unordered_set<NodePT, node_hash<NodePT>, node_equal_to<NodePT>>;
        node_unordered_set _index;

        //! An (optional) acceptor object
        std::unique_ptr<QueueAcceptorT> _acceptor;
    };

}
}