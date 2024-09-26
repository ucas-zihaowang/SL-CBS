
#ifndef SLPLANNER_H_
#define SLPLANNER_H_

#include "common.h"
#include "copter_common.h"
#include "globalmap.h"

#include "gridnode.h"
#include "statenode.h"
#include "constraints.h"

template <int Dim>
class SLPlanner
{
public:
    SLPlanner( GlobalMap<Dim> &gmap_, CopterParameter<Dim> &cp_, CopterState<Dim> &start_, CopterState<Dim> &goal_ , bool debug = true): gmap(gmap_), cp(cp_), start(start_), goal(goal_)
    {
		if ( debug )
        	printSLPlannerInfo();
        Timer one_clock(true);
        if ( cp.heur_type == 2 )
        {
            compute_preheuristic();
        }
        std::cout << ANSI_COLOR_GREEN "SLPlanner Initialize Time: "  << one_clock.Elapsed() << " ms"  ANSI_COLOR_RESET << std::endl;
    }

    ~SLPlanner()
    {
    }

    void printSLPlannerInfo()
    {
        gmap.printGlobalMapInfo();
        cp.printCopterParameterInfo();
        start.printCopterStateInfo("Start State Info:");
        goal.printCopterStateInfo("Goal State Info:");
    }


    bool search( Solution<Dim> &sol, const Constraints<Dim> &cons = Constraints<Dim>(), int minLength = 0 );
    bool isReachGoal( CopterState<Dim> &curr_state );
    bool traceSolution( StateNodePtr<Dim> curr_statenodeptr, Solution<Dim> &sol );
    void getAllNeighbors( StateNodePtr<Dim> curr_statenodeptr, vec_E<CopterState<Dim>> &succ_state,  std::vector<int> &succ_action_id, std::vector<double> &succ_action_cost, vec_E<Primitive<Dim>> &succ_action_pr );
    bool isBrakeable( CopterState<Dim> &curr_state, double max_acc, double dt, VecDf &New_U, double &work_time );


    double compute_Nvalue( CopterState<Dim> &curr_state );
    double compute_heuristic( CopterState<Dim> &curr_state );
    void compute_preheuristic();
    double computer_heuristic_with_control( CopterState<Dim> &curr_state );
    
    bool check_primitive_kinodynamic( Primitive<Dim> &pr, double mv = 0, double ma = 0, double mj = 0, double myaw = 0 );
    bool check_primitive_obstacles( Primitive<Dim> &pr );
    bool check_primitive_obstacles( Primitive<Dim> &pr, CopterState<Dim> &curr_state, CopterState<Dim> &succ_state );
    bool PrimitiveObstacleChecker( Primitive<Dim> &pr, Vecf<Dim> &obs );
    bool check_primitive_polyobstacles( Primitive<Dim> &pr );

    int computeMaxConsTimestamp( const Constraints<Dim> &cons );
    bool check_constraints( CopterState<Dim> &curr_state, Primitive<Dim> &pr, int timestamp, const Constraints<Dim> &cons );
    bool StateCollision( CopterState<Dim> &curr_state, const CopterState<Dim> &con_state );
    bool PrimitveCollision( Primitive<Dim>& curr_pr, const Primitive<Dim>& con_pr );
    bool StatePrimitveCollision( Primitive<Dim>& curr_pr, const CopterState<Dim> &con_state );
    
    double compute_primitive_cost( Primitive<Dim> &pr);
    double compute_yaw_softcon_cost( Primitive<Dim> &pr ); 
    double compute_penalty_factor_by_vel(  CopterState<Dim> &curr_state, CopterState<Dim> &next_state );
    double compute_penalty_factor_by_yaw(  CopterState<Dim> &next_state );
    double compute_bias_factor( CopterState<Dim> &next_state );
    
    vec_Vecf<Dim> getGenerateStateNodeSet(); 
    vec_Vecf<Dim> getExpandStateNodeSet();  

    std::vector<CopterState<Dim>> getExpandStates(){ return this->expanded_iter_states; }
    std::vector<std::vector<Primitive<Dim>>> getExpandPrimitive() { return this->expanded_iter_prs; }
    std::vector<std::vector<int>> getExpandedActions() {return this->expanded_iter_actions; }

private:
    GlobalMap<Dim> &gmap;
    CopterParameter<Dim> &cp;
    CopterState<Dim> start;
    CopterState<Dim> goal;

    std::vector<double> pre_heur;
    std::vector<Veci<Dim>> pre_heur_vec;

    StatePriorityQueue<StateNode<Dim>> state_open_list;
    StateCloseTable<Dim> state_close_list;

    std::vector<CopterState<Dim>> expanded_iter_states;
    std::vector<std::vector<Primitive<Dim>>> expanded_iter_prs;
    std::vector<std::vector<int>> expanded_iter_actions;

    int generateNodeNum;
    int expandNodeNum;

};

typedef SLPlanner<2> SLPlanner2d;
typedef SLPlanner<3> SLPlanner3d;

#endif