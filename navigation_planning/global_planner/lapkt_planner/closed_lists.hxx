/*
Lightweight Automated Planning Toolkit (LAPKT)
Copyright (C) 2015

<contributors>
Miquel Ramirez <miquel.ramirez@gmail.com>
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

#include <memory>
#include <unordered_set>

namespace roborts_global_planner::lapkt {


// We need to define custom hash and equality functions for the node-pointer type.
// Indeed, we want to define hash and equality of a node as equivalent to hash and equality of
// the state that corresponds to a node.
    template <typename NodePT>
    struct node_hash {
        size_t operator() (const NodePT& node) const { return node->hash(); }
    };

    template <typename NodePT>
    struct node_equal_to {
        bool operator() (const NodePT& n1, const NodePT& n2) const { return *n1 == *n2; }
    };

// A simple typedef to improve legibility
    template <typename NodePT>
    using node_unordered_set = std::unordered_set<NodePT, node_hash<NodePT>, node_equal_to<NodePT>>;

// A closed list is now simply an unordered_set of node pointers, providing some shortcut operations
// plus an update
    template <typename NodeT>
    class StlUnorderedMapClosedList : public node_unordered_set<std::shared_ptr<NodeT>> {
    public:
        using NodePT = std::shared_ptr<NodeT>;

        virtual ~StlUnorderedMapClosedList() = default;

        virtual inline void put(const NodePT& node) { this->insert(node); }

        virtual inline void remove(const NodePT& node) { this->erase(node); }

        virtual inline bool check(const NodePT& node) const { return this->find(node) != this->end(); }

        //! Returns a pointer to a node which is identical to the given node and was already in the list,
        //! if such a node exist, or nullptr otherwise
        virtual NodePT seek(NodePT& node) {
            auto it = this->find(node);
            return (it == this->end()) ? nullptr : *it;
        }
    };

//! A fake closed list that acts as if no node was ever in it
    template <typename NodeT>
    class NullClosedList {
    public:
        using NodePT = std::shared_ptr<NodeT>;

        ~NullClosedList() = default;

        inline void put(const NodePT& node) {}

        inline void remove(const NodePT& node) {}

        inline bool check(const NodePT& node) const { return false; }

        //! Returns a pointer to a node which is identical to the given node and was already in the list,
        //! if such a node exist, or nullptr otherwise
        NodePT seek(NodePT& node) { return nullptr; }
    };

}
