
#ifndef GRIDNODE_H_
#define GRIDNODE_H_

#include "common.h"

template<typename TGridNode>
struct compare_gridnode
{
    bool operator()( const std::shared_ptr<TGridNode>  &p1, const std::shared_ptr<TGridNode> &p2 ) const
    {
        return p1->g_value > p2->g_value;
    }
};

template <typename TGridNode >
using GridPriorityQueue = boost::heap::d_ary_heap< std::shared_ptr<TGridNode>, 
                boost::heap::mutable_<true>, 
                boost::heap::arity<2>, 
                boost::heap::compare< compare_gridnode<TGridNode>> >;


template <int Dim>
struct GridNode
{
    GridNode( Veci<Dim> grid_pos )
    {
        this->grid_pos = grid_pos;
    }

    Veci<Dim> grid_pos{Veci<Dim>::Zero()};

    double g_value{DBL_MAX};
    
    Veci<Dim> offset_dis{Veci<Dim>::Zero()};

    bool in_openlist{false};

    typename GridPriorityQueue<GridNode<Dim>>::handle_type open_handle;

    void printGridNodeInfo()
    {
        std::cout << ANSI_COLOR_YELLOW << "********************Grid Node Info********************" << ANSI_COLOR_RESET << std::endl;
        std::cout << "curr grid pos: ( " << grid_pos.transpose() << " )" << std::endl;
        std::cout << "curr g_value: " << g_value << std::endl;
        std::cout << ANSI_COLOR_YELLOW << "********************End********************" << ANSI_COLOR_RESET << std::endl;
    }
};

template<int Dim>
struct GridNodeHasher 
{
    std::size_t operator()(const std::shared_ptr<GridNode<Dim>> other) const 
    {
        std::size_t val = 0;
        int id;
        for (int i = 0; i < Dim; i++)
        {
            id = other->grid_pos(i);
            boost::hash_combine(val, id);
        }
        return val;
    }
};

template<int Dim>
struct EqualGridNode
{
    std::size_t operator()(const std::shared_ptr<GridNode<Dim>> n1, const std::shared_ptr<GridNode<Dim>> n2) const 
    {
        return (n1->grid_pos == n2->grid_pos);
    }
};

template <int Dim>
using GridCloseTable = boost::unordered_set< std::shared_ptr<GridNode<Dim>>, GridNodeHasher<Dim>, EqualGridNode<Dim> >;

template<int Dim>
using GridNodePtr = std::shared_ptr<GridNode<Dim>>;

typedef GridNode<2> GridNode2d;
typedef GridNode<3> GridNode3d;

#endif
