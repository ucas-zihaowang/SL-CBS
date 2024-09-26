
#ifndef SOLUTION_H_
#define SOLUTION_H_

#include "primitive.h"


template <int Dim>
struct Solution
{
    vec_E<CopterState<Dim>> states;
    std::vector<int> actions;
    std::vector<double> costs;
    vec_E<Primitive<Dim>> prs;

    double totalcost{0};

    void printSolutionInfo()
    {
        std::cout << ANSI_COLOR_GREEN << "********************Solution Result********************" << ANSI_COLOR_RESET << std::endl;
        std::cout << "Total Cost: " << totalcost << std::endl;

        // int states_num = states.size();
        // for ( int i = 0; i < states_num; i++ )
        // {
        //     std::string info = "State " + std::to_string(i);
        //     states[i].printCopterStateInfo(info);
        // }

        int action_num = actions.size();
        for( int i = 0; i < action_num; i++ )
        {
            if (  actions[i] == -1 )
            {
                // std::cout << ANSI_COLOR_RED <<  actions[i] << std::endl;
                std::cout << ANSI_COLOR_RED "Use the Stop Motion Primitive" << std::endl;
            }
            else
            {
                // std::cout << ANSI_COLOR_RED <<  actions[i] << std::endl;
            }
        }
        
        std::cout << ANSI_COLOR_GREEN << "********************End********************" << ANSI_COLOR_RESET << std::endl;
    }
};

typedef Solution<2> Solution2d;
typedef Solution<3> Solution3d;

#endif
