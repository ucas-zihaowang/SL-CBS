
#ifndef CBS_NODE_H_
#define CBS_NODE_H_

#include "common.h"
#include "conflict.h"
#include "constraints.h"

template<typename TCBSNode>
struct compare_CBSNode
{
    bool operator()(const std::shared_ptr<TCBSNode> &p1, const std::shared_ptr<TCBSNode> &p2) const
    {
        if( p1->cost == p2->cost )
        {
            return p1->conflicts_num > p2->conflicts_num;
        }
        return p1->cost > p2->cost;
    }
};

template<typename TCBSNode>
using CBSPriorityQueue = boost::heap::d_ary_heap <
                       std::shared_ptr<TCBSNode>,
                       boost::heap::mutable_<true>,
                       boost::heap::arity<2>,
                       boost::heap::compare<compare_CBSNode<TCBSNode>> >;

template <int Dim>
struct CBSNode
{
    CBSNode(){}

    CBSNode(const CBSNode<Dim> &other)
    {
        this->node_id = other.node_id;
        this->sols = other.sols;
        this->cost = other.cost;
        this->allcons = other.allcons;
        this->conflicts = other.conflicts;
        this->conflicts_num = other.conflicts_num;
    }

    int node_id;
    std::vector<Solution<Dim>> sols;
    double cost{0};

    std::list< Conflict<Dim> > conflicts;
    int conflicts_num;

    std::vector< Constraints<Dim> > allcons;

    typename CBSPriorityQueue<CBSNode<Dim>>::handle_type open_handle;
};

template<int Dim>
using CBSNodePtr = std::shared_ptr<CBSNode<Dim>>;

typedef CBSNode<2> CBSNode2d;
typedef CBSNode<3> CBSNode3d;

#endif
