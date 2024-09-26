
#ifndef CONSTRAINTS_H_
#define CONSTRAINTS_H_

#include "common.h"
#include "copter_common.h"

template <int Dim>
struct Constraint
{

    Constraint(){}


    Constraint( int type, int uav_id, int timestamp, CopterState<Dim>& con_state)
    {
        this->type = type;
        this->uav_id = uav_id;
        this->timestamp = timestamp;
        this->con_state = con_state;
    }

    Constraint( int type, int uav_id, int timestamp, Primitive<Dim>& con_pr)
    {
        this->type = type;
        this->uav_id = uav_id;
        this->timestamp = timestamp;
        this->con_pr = con_pr;
    }


    int type;
    int uav_id;
    int timestamp;
    CopterState<Dim> con_state;
    Primitive<Dim> con_pr;

    void printConstraintInfo() const {

        if ( type == 0 )
        {
            std::cout << ANSI_COLOR_YELLOW << "********************State Constraint Info********************" << ANSI_COLOR_RESET << std::endl;
            std::cout << "Constrained Uav ID: " << uav_id << std::endl;
            std::cout << "Constrained Timestamp: " << timestamp << std::endl;
            std::cout << "Constrained State's Postion: " << con_state.pos << std::endl;
            std::cout << ANSI_COLOR_YELLOW << "********************End********************" << ANSI_COLOR_RESET << std::endl;
        }
        else if ( type == 1 )
        {
            std::cout << ANSI_COLOR_YELLOW << "********************Primitive Constraint Info********************" << ANSI_COLOR_RESET << std::endl;
            std::cout << "Constrained Uav ID: " << uav_id << std::endl;
            std::cout << "Constrained Timestamp: " << timestamp << std::endl;
            std::cout << "Constrained Primitive: " << std::endl;
            con_pr.printMotionPrimitiveInfo();
            std::cout << ANSI_COLOR_YELLOW << "********************End********************" << ANSI_COLOR_RESET << std::endl;
        }
    }

};

template<int Dim>
struct ConstraintHasher 
{
    std::size_t operator()(const Constraint<Dim> &other) const 
    {
        std::size_t val = 0;
        boost::hash_combine( val, other.type );
        boost::hash_combine( val, other.uav_id );
        boost::hash_combine( val, other.timestamp );

        if ( other.type == 0 )
        {
            boost::hash_combine(val, other.con_state);
        }
        else if ( other.type == 1 )
        {
            boost::hash_combine(val, other.con_pr );
        }
        return val;
    }
};

template<int Dim>
struct EqualConstraint
{
    std::size_t operator()(const Constraint<Dim> &n1, const Constraint<Dim> &n2) const 
    {
        if ( n1.type == 0 && n2.type == 0 )
            return std::tie(n1.uav_id, n1.timestamp, n1.con_state) == std::tie(n2.uav_id, n2.timestamp, n2.con_state);
        else if ( n1.type == 1 && n2.type == 1 )
            return  std::tie(n1.uav_id, n1.timestamp, n1.con_pr) == std::tie(n2.uav_id, n2.timestamp, n2.con_pr);
        else
            return false;
    }
};

template <int Dim>
using Constraints = boost::unordered_set< Constraint<Dim>, ConstraintHasher<Dim>, EqualConstraint<Dim> >;

#endif