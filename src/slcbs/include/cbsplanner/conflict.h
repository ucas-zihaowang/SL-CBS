
#ifndef CONFLICT_H_
#define CONFLICT_H_

#include "common.h"
#include "copter_common.h"

template <int Dim>
struct Conflict
{

    int type; 
    int one_uav_id;
    int another_uav_id;
    
    int timestamp;
    CopterState<Dim> one_state;
    CopterState<Dim> another_state;

    Primitive<Dim> one_pred_pr;
    Primitive<Dim> another_pred_pr;

    void printConflictInfo()
    {
        if ( type == 0 )
        {
            std::cout << ANSI_COLOR_YELLOW << "********************State Conflict Info********************" << ANSI_COLOR_RESET << std::endl;
            std::cout << "Uav " << one_uav_id << " and Uav " << another_uav_id << " have Conflict in " << timestamp << " timestamp" << std::endl;
            std::string str1 = "Uav " + std::to_string(one_uav_id) + " State Info";
            std::string str2 = "Uav " + std::to_string(another_uav_id) + " State Info";
            one_state.printCopterStateInfo(str1);
            another_state.printCopterStateInfo(str2);
            std::cout << ANSI_COLOR_YELLOW << "********************End********************" << ANSI_COLOR_RESET << std::endl;
        }
        else if ( type == 1 )
        {
            std::cout << ANSI_COLOR_YELLOW << "********************Primitive Conflict Info********************" << ANSI_COLOR_RESET << std::endl;
            std::cout << "Uav " << one_uav_id << " and Uav " << another_uav_id << " have Conflict in " << timestamp << " timestamp" << std::endl;
            one_pred_pr.printMotionPrimitiveInfo();
            another_pred_pr.printMotionPrimitiveInfo();
            std::cout << ANSI_COLOR_YELLOW << "********************End********************" << ANSI_COLOR_RESET << std::endl;
        }
        else{}
    }
};

template <int Dim>
void printConflictsInfo( std::list< Conflict<Dim> > &conflicts )
{
    for( auto one_conflict: conflicts )
    {
        one_conflict.printConflictInfo();
    }
}

#endif