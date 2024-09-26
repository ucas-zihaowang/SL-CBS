
#ifndef STATENODE_H_
#define STATENODE_H_

#include "common.h"
#include "copter_common.h"

template<typename TStateNode>
struct compare_statenode
{
    bool operator()( const std::shared_ptr<TStateNode>  &p1, const std::shared_ptr<TStateNode> &p2) const
    {
        if ( p1->f_value == p2->f_value )
        {
            if ( p1->g_value == p2->g_value )
            {
                return p1->n_value > p2->n_value;
            }
            return p1->g_value < p2->g_value;

            // return p1->g_value > p2->g_value;
        }
        return p1->f_value > p2->f_value;
    }
};

template<typename TStateNode>
using StatePriorityQueue = boost::heap::d_ary_heap <
                      std::shared_ptr<TStateNode>,
                      boost::heap::mutable_<true>,
                      boost::heap::arity<2>,
                      boost::heap::compare<compare_statenode<TStateNode> >>;

template <int Dim>
struct StateNode
{
    StateNode( CopterState<Dim> s ): state(s) {}

    CopterState<Dim> state;

    double g_value = std::numeric_limits<double>::infinity();
    double h_value = std::numeric_limits<double>::infinity();
    double f_value = std::numeric_limits<double>::infinity();

    double n_value = std::numeric_limits<double>::infinity();

    bool in_openlist{false};
    bool in_closelist{false};

    typename StatePriorityQueue<StateNode<Dim>>::handle_type open_handle;


    vec_E<CopterState<Dim>> pred_state;
    std::vector<int> pred_action_id;
    std::vector<double> pred_action_cost;
    vec_E<Primitive<Dim>> pred_action_pr;

    void printStateNodeInfo()
    {
        state.printCopterStateInfo("Curr State Info:");
        std::cout << "curr g_value: " << g_value << std::endl;
        std::cout << "curr h_value: " << h_value << std::endl;
        std::cout << "curr f_value: " << f_value << std::endl;

        std::cout << "curr n_value: " << n_value << std::endl;

        std::cout << ANSI_COLOR_YELLOW << "********************End********************" << ANSI_COLOR_RESET << std::endl;
    }
};

template<int Dim>
struct StateNodeHasher 
{
    std::size_t operator()(const std::shared_ptr<StateNode<Dim>> other) const 
    {
        std::size_t val = 0;
        boost::hash_combine(val, other.state);
        return val;
    }
};

template<int Dim>
struct EqualStateNode
{
    std::size_t operator()(const std::shared_ptr<StateNode<Dim>> n1, const std::shared_ptr<StateNode<Dim>> n2) const 
    {
        return ( n1->state == n2->state );
    }
};

template<int Dim>
using StateNodePtr = std::shared_ptr<StateNode<Dim>>;

template <int Dim>
using StateCloseTable = boost::unordered_map< CopterState<Dim>, StateNodePtr<Dim>, boost::hash<CopterState<Dim>> >;

typedef StateNode<2> StateNode2d;
typedef StateNode<3> StateNode3d;

#endif
