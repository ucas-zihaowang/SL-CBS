
#include "slplanner.h"

template<int Dim>
bool SLPlanner<Dim>::search( Solution<Dim> &sol, const Constraints<Dim> &cons, int minLength )
{
    state_open_list.clear();
    state_close_list.clear();
    expanded_iter_actions.clear();
    expanded_iter_prs.clear();
    expanded_iter_states.clear();


    StateNodePtr<Dim> root = state_close_list[start];
    root = std::make_shared<StateNode<Dim>>(start);
    root->g_value = 0;
    root->h_value = compute_heuristic( root->state );
    root->f_value = root->g_value + root->h_value;
    root->n_value = compute_Nvalue( root->state );

    root->open_handle = state_open_list.push( root );
    root->in_openlist = true;
    root->in_closelist = false;
    state_close_list[root->state] = root;

    int iteration = 0;


    while ( !state_open_list.empty() )
    {
		iteration++;

        StateNodePtr<Dim> curr = state_open_list.top();
        state_open_list.pop();
        curr->in_closelist = true;

#ifdef SLPLANNER_DEBUG
        curr->printStateNodeInfo();
#endif


        if ( isReachGoal( curr->state ) && (curr->state.timestamp > minLength) )
        {
#ifdef SLPLANNER_ITER_DEBUG
            std::cout << ANSI_COLOR_GREEN  "slplanner iteration num: " << iteration << ANSI_COLOR_RESET << std::endl;
#endif

            return traceSolution(curr, sol);
        }


        vec_E<CopterState<Dim>> succ_state;
        std::vector<int> succ_action_id;
        std::vector<double> succ_action_cost;
        vec_E<Primitive<Dim>> succ_action_pr;
        getAllNeighbors( curr, succ_state, succ_action_id, succ_action_cost, succ_action_pr );

        int neighbor_num = succ_state.size();
        for( unsigned int s = 0; s < neighbor_num; s++ )
        {

            if ( !check_constraints( succ_state[s], succ_action_pr[s], succ_state[s].timestamp, cons) )
            {
                continue;
            }


            StateNodePtr<Dim> &next = state_close_list[ succ_state[s] ];
            if ( !next )
            {
                next = std::make_shared<StateNode<Dim>>( succ_state[s] );
                // next->h_value = compute_heuristic( next->state ) + ( ( succ_action_id[s] == -1 ) ?  compute_bias_factor( next->state ) : 0 );
                next->h_value = compute_heuristic( next->state );
                next->n_value = compute_Nvalue( next->state );
            }

            next->pred_state.push_back( curr->state );
            next->pred_action_id.push_back( succ_action_id[s] );
            next->pred_action_cost.push_back( succ_action_cost[s] );
            next->pred_action_pr.push_back( succ_action_pr[s] );

            double new_g_value = curr->g_value + succ_action_cost[s];
            if ( new_g_value < next->g_value )
            {
                next->g_value = new_g_value;
                next->f_value = new_g_value + next->h_value;

                if ( next->in_openlist && !next->in_closelist )
                {
                    state_open_list.increase( next->open_handle );
                }
                else
                {
                    next->open_handle = state_open_list.push( next );
                    next->in_openlist = true;
                }
            }
#ifdef SLPLANNER_DEBUG
            std::cout << "Action ID: " << succ_action_id[s] << std::endl;
            std::cout << "Action Cost: " << succ_action_cost[s] << std::endl;
            std::cout << "Delta H: " << curr->h_value - next->h_value  << std::endl;
            std::cout << "Bias Factor: " << compute_bias_factor( next->state ) << std::endl;
            std::cout << "G H F: " << next->g_value << " " << next->h_value << " " << next->f_value << std::endl;
#endif
        } // end for

#ifdef MAX_LOW_ITERATION_NUM
        if ( iteration >= MAX_LOW_ITERATION_NUM )
        {
#ifdef SLPLANNER_ITER_DEBUG
            std::cout << ANSI_COLOR_GREEN  "slplanner iteration num: " << iteration << ANSI_COLOR_RESET << std::endl;
#endif
            break;
        }
#endif
    } // end while
    return false;
}

template<int Dim>
bool SLPlanner<Dim>::isReachGoal( CopterState<Dim> &curr_state )
{
    bool goaled = (curr_state.pos - goal.pos).template lpNorm<Eigen::Infinity>() <= cp.tol_pos;
    if ( goaled && cp.tol_vel >= 0 )
        goaled = (curr_state.vel - goal.vel).template lpNorm<Eigen::Infinity>() <= cp.tol_vel;
    if ( goaled && cp.tol_acc >= 0 )
        goaled = (curr_state.acc - goal.acc).template lpNorm<Eigen::Infinity>() <= cp.tol_acc;
    if ( goaled && cp.tol_yaw >= 0 )
        goaled = std::abs(curr_state.yaw - goal.yaw) <= cp.tol_yaw;

    if ( goaled )
    {
        if ( cp.map_type == 1 ) 
        {

            auto pns = gmap.rayTrace( curr_state.pos, goal.pos ); 
            for ( auto &it : pns ) 
            {
                if ( gmap.isOccupied(it) )
                    return false;
            }
        }
        else if ( cp.map_type == 2 ) 
        {

            auto pns = gmap.rayTrace( curr_state.pos, goal.pos, 0 ); 

            for ( auto &it : pns )
            {

                // for( auto obs: gmap.ObsGeoDes.getObstacles() )
                // {
                //     if ( ( it - obs ).template lpNorm<2>() <= gmap.Safe_Dis_Obstacle )
                //         return false;
                // }
                

                Vecf<Dim> pos_centre = (curr_state.pos + goal.pos) / 2;
                double search_dis = ( curr_state.pos - goal.pos ).template lpNorm<2>() / 2 + gmap.Safe_Dis_Obstacle;
                search_dis = power( search_dis, 2 );

                for( auto obs: gmap.ObsGeoDes.radiusSearch( pos_centre, search_dis ) )
                {
                    if ( ( it - obs ).template lpNorm<2>() <= gmap.Safe_Dis_Obstacle )
                        return false;
                }
                
            }
        }
    }
    return goaled;
}

template<int Dim>
bool SLPlanner<Dim>::traceSolution( StateNodePtr<Dim> curr_statenodeptr, Solution<Dim> &sol )
{
    sol.totalcost = curr_statenodeptr->g_value;

    vec_E<StateNodePtr<Dim>> best_child;
    while ( !curr_statenodeptr->pred_state.empty() )
    {
        best_child.push_back( curr_statenodeptr );


        int min_id = -1;
        double min_rhs = std::numeric_limits<double>::infinity();
        double min_g = std::numeric_limits<double>::infinity();

        int pred_state_num = curr_statenodeptr->pred_state.size();
        for ( unsigned int i = 0; i < pred_state_num; i++ )
        {
            CopterState<Dim> key = curr_statenodeptr->pred_state[i];

            if ( min_rhs > state_close_list[key]->g_value + curr_statenodeptr->pred_action_cost[i] )
            {
                min_rhs = state_close_list[key]->g_value + curr_statenodeptr->pred_action_cost[i];
                min_g = state_close_list[key]->g_value;
                min_id = i;
            }

            else if ( !std::isinf(curr_statenodeptr->pred_action_cost[i]) && min_rhs == state_close_list[key]->g_value + curr_statenodeptr->pred_action_cost[i] )
            {
                if ( min_g < state_close_list[key]->g_value )
                {
                    min_g = state_close_list[key]->g_value;
                    min_id = i;
                }
            }
        }

        CopterState<Dim> best_key = curr_statenodeptr->pred_state[min_id];
        int best_action_id = curr_statenodeptr->pred_action_id[min_id];
        double best_action_cost = curr_statenodeptr->pred_action_cost[min_id];
        Primitive<Dim> best_action_pr = curr_statenodeptr->pred_action_pr[min_id];

        sol.states.push_back(best_key);
        sol.actions.push_back(best_action_id);
        sol.costs.push_back(best_action_cost);
        sol.prs.push_back(best_action_pr);

        curr_statenodeptr = state_close_list[best_key];

        if ( curr_statenodeptr->state == start )
        {
            best_child.push_back( curr_statenodeptr );
            break;
        }
    } // end while


    std::reverse( best_child.begin(), best_child.end() );
    std::reverse( sol.states.begin(), sol.states.end() );
    std::reverse( sol.actions.begin(), sol.actions.end() );
    std::reverse( sol.costs.begin(), sol.costs.end() );
    std::reverse( sol.prs.begin(), sol.prs.end() );
    sol.states.push_back( best_child.back()->state ); // 添加了末状态

    return true;
}

template<int Dim>
void SLPlanner<Dim>::getAllNeighbors( StateNodePtr<Dim> curr_statenodeptr, vec_E<CopterState<Dim>> &succ_state, 
    std::vector<int> &succ_action_id, std::vector<double> &succ_action_cost, vec_E<Primitive<Dim>> &succ_action_pr )
{
    succ_state.clear();   
    succ_action_id.clear();  
    succ_action_cost.clear(); 
    succ_action_pr.clear();   

    this->expanded_iter_states.push_back( curr_statenodeptr->state );

    std::vector<Primitive<Dim>> iter_prs;
    std::vector<int> iter_act;

    for ( unsigned int i = 0; i <= cp.all_u_num; i++ )
    {
        CopterState<Dim> tn;
        Primitive<Dim> pr;
        int action_id;
        double StopPenalty = 1.0;

        if ( i == cp.all_u_num )
        {
            if ( STOP_MOTION_PRIMITVE == 0 || cp.cm == ControlMode::VEL )
                continue;
            
            VecDf New_U = cp.U_Zero;
            double work_time = 0;

            if ( !isBrakeable( curr_statenodeptr->state, 6 * cp.max_acc, cp.dt, New_U, work_time ) )
                continue;
            
            pr = Primitive<Dim>( curr_statenodeptr->state, New_U, cp.dt, work_time );
            tn = pr.evaluate( cp.dt );
            tn.time_t = curr_statenodeptr->state.time_t + cp.dt;
            tn.timestamp = curr_statenodeptr->state.timestamp + 1;

            action_id = -1;
            StopPenalty = 1.0;

        }
        else
        {
 
            pr = Primitive<Dim>( curr_statenodeptr->state, cp.All_U[i], cp.dt );

            tn = pr.evaluate( cp.dt );
            tn.time_t = curr_statenodeptr->state.time_t + cp.dt;
            tn.timestamp = curr_statenodeptr->state.timestamp + 1;
            action_id = i;
        }


        if ( tn == curr_statenodeptr->state )
        {
            // std::cout << "same state: " << action_id << std::endl;
            continue;
        }
            


        if( gmap.isOutside( tn.pos ) )
        {
            // std::cout << "outside state: " << action_id << std::endl;
            continue;
        }
            


        if ( !check_primitive_kinodynamic( pr, cp.max_vel, cp.max_acc, cp.max_jrk, cp.max_yaw) )
        {
            // std::cout << "out kinodynamic state: " << action_id << std::endl;
            continue;
        }
            


        if ( cp.map_type == 1 && (!check_primitive_obstacles(pr)) )
        {
            continue;
        }
        else if ( cp.map_type == 2 && (!check_primitive_obstacles( pr, curr_statenodeptr->state, tn)) )
        {
            continue;
        }
        else if ( cp.map_type == 3 && (!check_primitive_polyobstacles(pr)) )
        {
            // std::cout << "obs state: " << action_id << std::endl;
            continue;
        }
        

        double cost = compute_primitive_cost(pr) * StopPenalty;


        if ( cp.yaw_use )
        {
            if ( cp.yaw_motion_primitive_penalty )
                cost *= compute_penalty_factor_by_yaw( tn );
        }
        else
        {
            if ( cp.motion_primitive_penalty )
                cost *= compute_penalty_factor_by_vel( curr_statenodeptr->state, tn );
        }

        // std::cout << "action id: " << action_id << " action cost: " << cost << std::endl;

        iter_prs.push_back(pr);
        iter_act.push_back(action_id);

        succ_state.push_back(tn);
        succ_action_id.push_back( action_id );
        succ_action_cost.push_back(cost);
        succ_action_pr.push_back(pr);


    }
    
    this->expanded_iter_prs.push_back(iter_prs);
    this->expanded_iter_actions.push_back( iter_act );

    // std::cout << std::endl;

}

template<int Dim>
bool SLPlanner<Dim>::isBrakeable( CopterState<Dim> &curr_state, double max_acc, double dt, VecDf &New_U, double &work_time )
{
    Vecf<Dim> curr_vel = curr_state.vel;


    work_time = 0;
    for( int i = 0; i < Dim; i++ )
    {
        double res_t = std::abs( curr_vel(i) / max_acc );
        if ( res_t > dt )
            return false;
        if( work_time < res_t )
            work_time = res_t;
    }

    if ( work_time == 0 ) 
        return false;

    for( int i = 0; i < Dim; i++ )
    {
        if ( curr_vel(i) != 0 )
            New_U(i) = -1 * ( curr_vel(i) / work_time );
    }
    return true;
}

template<int Dim>
double SLPlanner<Dim>::compute_Nvalue( CopterState<Dim> &curr_state )
{
    return (curr_state.pos - goal.pos).template lpNorm<2>();
}

template<int Dim>
double SLPlanner<Dim>::compute_heuristic( CopterState<Dim> &curr_state )
{
    int heur_type_value = cp.heur_type;
    if ( heur_type_value == 0 )
    {
        if ( cp.max_vel > 0 )
            return cp.Wt * (curr_state.pos - goal.pos).template lpNorm<Eigen::Infinity>() / cp.max_vel;
        else
            return cp.Wt * (curr_state.pos - goal.pos).template lpNorm<Eigen::Infinity>();
    }
    else if ( heur_type_value == 1 )
    {
        return computer_heuristic_with_control( curr_state );
    }
    else if ( heur_type_value == 2 )
    {
        double dis = this->pre_heur[ gmap.getIndex(gmap.toGrid(curr_state.pos)) ] * gmap.getRes();
        return cp.Wt * dis / cp.max_vel;
    }
    else if ( heur_type_value == 3 )
    {
        double dis = this->pre_heur_vec[ gmap.getIndex(gmap.toGrid(curr_state.pos)) ].template lpNorm<Eigen::Infinity>() * gmap.getRes();
        return cp.Wt * dis / cp.max_vel;
    }
    else
        return 0;
}

template<int Dim>
void SLPlanner<Dim>::compute_preheuristic()
{
    int map_size = gmap.getGridMapSize();
    this->pre_heur.resize( map_size, DBL_MAX );
    this->pre_heur_vec.resize( map_size, Veci<Dim>::Constant(INT_MAX) );

    GridPriorityQueue<GridNode<Dim>> grid_open_list;
    GridCloseTable<Dim> grid_close_list;


    const Veci<Dim> goal_grid_pos = gmap.toGrid( goal.pos );
    GridNodePtr<Dim> root =  std::make_shared<GridNode<Dim>>( goal_grid_pos );
    root->g_value = 0;
    root->offset_dis = Veci<Dim>::Zero();
    root->open_handle = grid_open_list.push( root );
    root->in_openlist = true;
    grid_close_list.insert( root );

    int iteration = 0;
    while ( !grid_open_list.empty() )
    {
        GridNodePtr<Dim> curr = grid_open_list.top();
        grid_open_list.pop();
        // curr->printGridNodeInfo();

        for( int i = 0; i < cp.move_dir_num; i++ )
        {
            const Veci<Dim> new_grid = curr->grid_pos + cp.move_directions[i];

            if ( gmap.isOutside(new_grid) )
                continue;
            
            if ( gmap.isOccupied(new_grid) )
                continue;
            
            GridNodePtr<Dim> next = std::make_shared<GridNode<Dim>>( new_grid );
            double new_g_value = curr->g_value + 1;

            const Veci<Dim> new_offset_dis = curr->offset_dis + cp.move_directions_abs[i];

            auto it = grid_close_list.find( next );
            if ( it == grid_close_list.end() )
            {
                next->g_value = new_g_value;
                next->offset_dis = new_offset_dis;
                next->open_handle = grid_open_list.push( next );
                next->in_openlist = true;
                grid_close_list.insert( next );
            }
            else
            {
                GridNodePtr<Dim> exist_node = *it;
                if ( exist_node->g_value > new_g_value )
                {
                    exist_node->g_value = new_g_value;
                    exist_node->offset_dis = new_offset_dis;
                    grid_open_list.update( exist_node->open_handle );
                }
            }
        } // end for
        iteration++;
    } // end while

    for( auto each_girdnode: grid_close_list )
    {
        this->pre_heur[ gmap.getIndex(each_girdnode->grid_pos) ] = each_girdnode->g_value;
        this->pre_heur_vec[ gmap.getIndex(each_girdnode->grid_pos) ] = each_girdnode->offset_dis;
    }

    grid_open_list.clear();
    grid_close_list.clear();

    std::cout << ANSI_COLOR_GREEN  "compute_preheuristic iteration num: " << iteration << ANSI_COLOR_RESET << std::endl;
}

template<int Dim>
double SLPlanner<Dim>::computer_heuristic_with_control( CopterState<Dim> &curr_state )
{
    if ( curr_state.cm == ControlMode::JRK )
    {
        const Vecf<Dim> dp = goal.pos - curr_state.pos;
        const Vecf<Dim> v0 = curr_state.vel;
        const Vecf<Dim> v1 = goal.vel;
        const Vecf<Dim> a0 = curr_state.acc;
        const Vecf<Dim> a1 = goal.acc;
        double a = cp.Wt;
        double b = 0;
        double c = -9 * a0.dot(a0) + 6 * a0.dot(a1) - 9 * a1.dot(a1);
        double d = -144 * a0.dot(v0) - 96 * a0.dot(v1) + 96 * a1.dot(v0) + 144 * a1.dot(v1);
        double e = 360 * (a0 - a1).dot(dp) - 576 * v0.dot(v0) - 1008 * v0.dot(v1) - 576 * v1.dot(v1);
        double f = 2880 * dp.dot(v0 + v1);
        double g = -3600 * dp.dot(dp);

        std::vector<double> ts = solve(a, b, c, d, e, f, g);
        double t_bar = (curr_state.pos - goal.pos).template lpNorm<Eigen::Infinity>() / cp.max_vel;
        ts.push_back(t_bar);

        double min_cost = std::numeric_limits<double>::max();
        for (auto t : ts)
        {
            if (t < t_bar) 
                continue;
            double cost = a * t - c / t - d / 2 / t / t - e / 3 / t / t / t -
                                f / 4 / t / t / t / t - g / 5 / t / t / t / t / t;
            if (cost < min_cost) 
                min_cost = cost;
        }
        return min_cost;
    }
    else if( curr_state.cm == ControlMode::ACC )
    {
        const Vecf<Dim> dp = goal.pos - curr_state.pos;
        const Vecf<Dim> v0 = curr_state.vel;
        const Vecf<Dim> v1 = goal.vel;

        double c1 = -36 * dp.dot(dp);
        double c2 = 24 * (v0 + v1).dot(dp);
        double c3 = -4 * (v0.dot(v0) + v0.dot(v1) + v1.dot(v1));
        double c4 = 0;
        double c5 = cp.Wt;

        std::vector<double> ts = quartic(c5, c4, c3, c2, c1);
        double t_bar = (curr_state.pos - goal.pos).template lpNorm<Eigen::Infinity>() / cp.max_vel;
        ts.push_back(t_bar);

        double cost = std::numeric_limits<double>::max();
        for (auto t : ts)
        {
            if ( t < t_bar ) 
                continue;
            double c = -c1 / 3 / t / t / t - c2 / 2 / t / t - c3 / t + cp.Wt * t;
            if ( c < cost ) 
                cost = c;
        }
        return cost;
    }
    else if( curr_state.cm == ControlMode::VEL )
        return (cp.Wt+ 1) * (curr_state.pos - goal.pos).norm();
    else
        return cp.Wt * (curr_state.pos - goal.pos).template lpNorm<Eigen::Infinity>() / cp.max_vel;
}

template<int Dim>
bool SLPlanner<Dim>::check_primitive_kinodynamic( Primitive<Dim> &pr, double mv, double ma, double mj, double myaw)
{
    if ( cp.yaw_use )
        return validate_cm( pr, pr.getControl(), mv, ma, mj ) && validate_yaw( pr, myaw );
    else
        return validate_cm( pr, pr.getControl(), mv, ma, mj );
}

template<int Dim>
bool SLPlanner<Dim>::check_primitive_obstacles( Primitive<Dim> &pr )
{

    double max_v = 0;
    for (int i = 0; i < Dim; i++)
    {
        if ( pr.max_vel(i) > max_v )
            max_v = pr.max_vel(i);
    }

    int n = std::max( 5, (int) std::ceil( max_v * pr.getDT() / gmap.getRes() ) );

    double dt = pr.getDT() / n;
    for ( double t = 0; t < pr.getDT(); t += dt )
    {

        auto pt = pr.evaluate(t);

        Veci<Dim> pn = gmap.toGrid( pt.pos );

        if ( gmap.isOutside(pt.pos) )
            return false;

        if ( gmap.isOccupied(pn) )
            return false;
    }
    return true;
}


template<int Dim>
bool SLPlanner<Dim>::check_primitive_obstacles( Primitive<Dim> &pr, CopterState<Dim> &curr_state, CopterState<Dim> &succ_state )
{

#if CHECK_STATIC_OBS_METHOD == 1

    double max_v = 0;
    for (int i = 0; i < Dim; i++)
    {
        if ( pr.max_vel(i) > max_v )
            max_v = pr.max_vel(i);
    }

    int n = std::max( 5, (int) std::ceil( max_v * pr.getDT() / gmap.getRes() ) );


    double dt = pr.getDT() / n;
    for ( double t = 0; t < pr.getDT(); t += dt )
    {

        auto pt = pr.evaluate(t);

        if ( gmap.isOutside(pt.pos) )
            return false;


        for( auto obs: gmap.ObsGeoDes.getObstacles() )
        {
            if ( (pt.pos - obs ).template lpNorm<2>() <= ( gmap.Safe_Dis_Obstacle ) )
                return false;
        }
    }
    return true;

#elif CHECK_STATIC_OBS_METHOD == 2
    for( auto obs: gmap.ObsGeoDes.getObstacles() )
    {

        if ( PrimitiveObstacleChecker( pr, obs)  )
            return false;

    }
    return true;

#elif CHECK_STATIC_OBS_METHOD == 3

    Vecf<Dim> pos_centre = (curr_state.pos + succ_state.pos) / 2;
    double search_dis = (curr_state.pos - succ_state.pos ).template lpNorm<2>() / 2 + gmap.Safe_Dis_Obstacle;
    search_dis = power( search_dis, 2 );


    for( auto obs: gmap.ObsGeoDes.radiusSearch(pos_centre, search_dis) )
    {

        if ( PrimitiveObstacleChecker( pr, obs)  )
            return false;
    }
    return true;

#endif
}

template<int Dim>
bool SLPlanner<Dim>::check_primitive_polyobstacles( Primitive<Dim> &pr )
{

    for ( auto polyobs: gmap.PolyObsSet )
    {

        vec_E<Vec6f> cs(Dim);
        for (int i = 0; i < Dim; i++)
            cs[i] = pr.getPr(i).coeff();

        const auto p = polyobs.getP();


        for ( const auto& v : polyobs.getPolyObstacle().hyperplanes() ) 
        {
            const auto n = v.n_;
            decimal_t a = 0, b = 0, c = 0, d = 0, e = 0, f = 0;
            for ( int i = 0; i < Dim; i++ ) 
            {
                a += n(i) * cs[i](0);
                b += n(i) * cs[i](1);
                c += n(i) * cs[i](2);
                d += n(i) * cs[i](3);
                e += n(i) * cs[i](4);
                f += n(i) * cs[i](5);
            }
            a /= 120.0;
            b /= 24.0;
            c /= 6.0;
            d /= 2.0;
            e /= 1.0;
            f -= n.dot( v.p_ + p );


            std::vector<decimal_t> ts = solve(a, b, c, d, e, f);
            // printf("a, b, c, d, e: %f, %f, %f, %f, %f\n", a, b, c, d, e);
            for (const auto& it : ts) {
                if ( it >= 0 && it <= pr.getDT() )
                {
                    auto w = pr.evaluate(it);
                    if ( polyobs.inside(w.pos) ) 
                        return false;
                }
            }
        }
    }
    return true;
}


template<int Dim>
bool SLPlanner<Dim>::PrimitiveObstacleChecker( Primitive<Dim> &pr, Vecf<Dim> &obs )
{

    Eigen::VectorXd coeffs{Vec11f::Zero()};
    for( int i = 0; i < Dim; i++ )
    {
        Eigen::VectorXd one_coeffs(6);
        one_coeffs = pr.getPr(i).coeff();

        one_coeffs(0) = one_coeffs(0) / 120;
        one_coeffs(1) = one_coeffs(0) / 24;
        one_coeffs(2) = one_coeffs(0) / 6;
        one_coeffs(3) = one_coeffs(0) / 2;
        one_coeffs(5) -= obs(i);

        coeffs += RootFinder::polySqr( one_coeffs );
    }

    coeffs(10) -= power( gmap.Safe_Dis_Obstacle,2 );


    if ( coeffs(10) <= 0 ) 
    {
        return true;
    }

    double work_time = pr.getWorkTime(); 
    if ( RootFinder::countRoots( coeffs, 0, work_time ) > 0 ) 
    {
        return true;
    }

    return false;
}

template<int Dim>
int SLPlanner<Dim>::computeMaxConsTimestamp( const Constraints<Dim> &cons )
{
    int maxTimestamp = 0;
    for ( auto con : cons )
        if ( maxTimestamp < con.timestamp )
            maxTimestamp = con.timestamp;
    return maxTimestamp;
}

template<int Dim>
bool SLPlanner<Dim>::check_constraints( CopterState<Dim> &curr_state, Primitive<Dim> &pr, int timestamp, const Constraints<Dim> &cons )
{
    for ( auto it = cons.begin(); it != cons.end(); it++ )
    {
        if ( timestamp != it->timestamp  )
            continue;

        if ( it->type == 0 && StateCollision( curr_state, it->con_state) )
            return false;

        if ( it->type == 1 && PrimitveCollision( pr, it->con_pr) )
            return false;

        if ( it->type == 2 && StatePrimitveCollision( pr, it->con_state) )
        {
            // std::cout << "SLPlanner StatePrimitveCollision is checked" << std::endl;
            return false;
        }
            
    }
    return true;
}

template<int Dim>
bool SLPlanner<Dim>::StateCollision( CopterState<Dim> &curr_state, const CopterState<Dim> &con_state )
{
    if ( (curr_state.pos - con_state.pos ).template lpNorm<2>() <= gmap.Safe_Dis_Collision  )
        return true;
    return false;
}


template<int Dim>
bool SLPlanner<Dim>::PrimitveCollision( Primitive<Dim>& curr_pr, const Primitive<Dim>& con_pr )
{

    Eigen::VectorXd coeffs{Vec11f::Zero()};
    for( int i = 0; i < Dim; i++ )
    {
        Eigen::VectorXd one_coeffs(6);
        one_coeffs = curr_pr.getPr(i).coeff();
        Eigen::VectorXd another_coeffs(6);
        another_coeffs = con_pr.getPr(i).coeff();

        Eigen::VectorXd mid_coeffs = one_coeffs - another_coeffs;
        mid_coeffs(0) = mid_coeffs(0) / 120;
        mid_coeffs(1) = mid_coeffs(1) / 24;
        mid_coeffs(2) = mid_coeffs(2) / 6;
        mid_coeffs(3) = mid_coeffs(3) / 2;

        Eigen::VectorXd coeffs_res = RootFinder::polySqr( mid_coeffs );

        coeffs += coeffs_res;
    }
    coeffs(10) -= power( gmap.Safe_Dis_Collision, 2 );



    if( coeffs(10) <= 0 ) 
    {
        return true;
    }
    
    double work_time = curr_pr.getWorkTime() <= con_pr.getWorkTime()? curr_pr.getWorkTime() : con_pr.getWorkTime();

    if ( RootFinder::countRoots( coeffs, 0, work_time ) > 0 ) 
    {
        return true;
    }
    return false;
}


template<int Dim>
bool SLPlanner<Dim>::StatePrimitveCollision( Primitive<Dim>& curr_pr, const CopterState<Dim> &con_state )
{

    Eigen::VectorXd coeffs{Vec11f::Zero()};
    for( int i = 0; i < Dim; i++ )
    {
        Eigen::VectorXd one_coeffs(6);
        one_coeffs = curr_pr.getPr(i).coeff();
        one_coeffs(5) -= con_state.pos(i);

        Eigen::VectorXd mid_coeffs = one_coeffs;
        mid_coeffs(0) = mid_coeffs(0) / 120;
        mid_coeffs(1) = mid_coeffs(1) / 24;
        mid_coeffs(2) = mid_coeffs(2) / 6;
        mid_coeffs(3) = mid_coeffs(3) / 2;

        Eigen::VectorXd coeffs_res = RootFinder::polySqr( mid_coeffs );

        coeffs += coeffs_res;
    }
    coeffs(10) -= power( gmap.Safe_Dis_Collision, 2 );



    if( coeffs(10) <= 0 ) 
    {
        return true;
    }
    
    double work_time = curr_pr.getWorkTime();

    if ( RootFinder::countRoots( coeffs, 0, work_time ) > 0 ) 
    {
        return true;
    }
    return false;
}


template<int Dim>
double SLPlanner<Dim>::compute_primitive_cost( Primitive<Dim> &pr )
{
    if ( pr.getIsStopPr() )
        return cp.Wt * pr.getDT();

    if ( cp.yaw_use )
    {
        return cp.Wj * pr.J() + cp.Wt * pr.getDT() + cp.Wyaw * pr.Jyaw(); // normal case
        // return cp.Wj * pr.J() + cp.Wt * pr.getDT() + cp.Wyaw * compute_yaw_softcon_cost( pr ) ; // for mpl_ros case
    }
    else
    {
        return cp.Wj * pr.J() + cp.Wt * pr.getDT();
    }
}

template<int Dim>
double SLPlanner<Dim>::compute_yaw_softcon_cost( Primitive<Dim> &pr )
{
    double cost  = 0;

    double max_v = 0;
    for( int i = 0; i < Dim; i++ )
    {
        if ( pr.max_vel(i) > max_v )
            max_v = pr.max_vel(i);
    }

    int n = std::max( 5, (int)std::ceil( max_v * cp.dt / gmap.getRes() ) );
    double dt = cp.dt / n;


    for( double t = 0; t < cp.dt; t += dt )
    {
        const auto pt = pr.evaluate( t );
        const auto v = pt.vel.template topRows<2>();

        if ( v.norm() > 1e-5 )
        {
            double v_value = 1 - v.normalized().dot( Vec2f( cos(pt.yaw), sin(pt.yaw) ) );
            cost += cp.Wyaw * v_value * dt;
        }
    }

    return cost;
}

template<int Dim>
double SLPlanner<Dim>::compute_penalty_factor_by_vel( CopterState<Dim> &curr_state, CopterState<Dim> &next_state )
{
    double res = 1;
    const auto v1 = curr_state.vel.template topRows<2>();
    const auto v2 = next_state.vel.template topRows<2>();

    if ( v1.norm() > 1e-5 && v2.norm() > 1e-5 )
    {
        
        double d = v1.normalized().dot( v2.normalized() );
        if ( d <= 0 )
            res = 3;
        else if ( d <= 0.866 )
            res = 2;
        else
            res = 1;
    }
    return res;
}

template<int Dim>
double SLPlanner<Dim>::compute_penalty_factor_by_yaw(  CopterState<Dim> &next_state )
{
    double res = 1;

    const auto v = next_state.vel.template topRows<2>();


    if( v.norm() > 1e-5 & cp.yaw_use )
    {
        double d = v.normalized().dot( Vec2f( cos(next_state.yaw), sin(next_state.yaw)  )  );
        if ( d >= cos( cp.max_yaw / 3 ) )
            res = 1;
        else if ( d >= cos( cp.max_yaw / 2 ) )
            res = 2;
        else
            res = 3;
    }
    return res;
}

template<int Dim>
double SLPlanner<Dim>::compute_bias_factor(  CopterState<Dim> &next_state  )
{
    double dis1 = (next_state.pos - goal.pos).template lpNorm<Eigen::Infinity>();
    double dis2 = (next_state.pos - goal.pos).template lpNorm<2>();
    if ( dis1 == 0 )
        return dis1;
    else
        return dis2 / dis1 - 1;
}

template<int Dim>
vec_Vecf<Dim> SLPlanner<Dim>::getGenerateStateNodeSet()
{
   vec_Vecf<Dim> ps;
   for (const auto &it : state_open_list )
       ps.push_back( it->state.pos );
   this->generateNodeNum = ps.size();
   return ps;
}

template<int Dim>
vec_Vecf<Dim> SLPlanner<Dim>::getExpandStateNodeSet()
{
    vec_Vecf<Dim> ps;
    for (const auto &it : state_close_list )
    {
        if ( it.second && it.second->in_closelist )
            ps.push_back( it.second->state.pos );
    }
    this->expandNodeNum = ps.size();
    return ps;
}

template class SLPlanner<2>;
template class SLPlanner<3>;

// if ( succ_action_id[s] == -1 )
// {
//     if ( curr->state.timestamp == 20 )
//     {
//         std::cout << ANSI_COLOR_GREEN  "slplanner iteration num: " << iteration << ANSI_COLOR_RESET << std::endl;
//         std::cout << "curr state timestamp: " << curr->state.timestamp << std::endl;
//         std::cout << "state timestamp: " << succ_state[s].timestamp << std::endl;
//         // std::cout << "con timestamp: " << it->timestamp << std::endl;
//         std::cout << "Stop Primitive is Selected" << std::endl;
//         curr->state.printCopterStateInfo("curr state");

//         auto timestamp = succ_state[s].timestamp;
//         for ( auto it = cons.begin(); it != cons.end(); it++ )
//         {
//             if ( timestamp != it->timestamp  )
//                 continue;

//             if ( it->type == 0 && StateCollision( succ_state[s], it->con_state) )
//                 std::cout << "state collision" << std::endl;

//             if ( it->type == 1 && PrimitveCollision( succ_action_pr[s], it->con_pr) )
//             {
//                 std::cout << "primitive collision" << std::endl;
//                 succ_action_pr[s].printMotionPrimitiveInfo();
//                 it->con_pr.printMotionPrimitiveInfo();

//             }
            
//         }

//     }

// }

// if ( succ_action_id[s] == -1 )
// {
//     std::cout << ANSI_COLOR_GREEN  "slplanner iteration num: " << iteration << ANSI_COLOR_RESET << std::endl;
//     std::cout << "curr state timestamp: " << curr->state.timestamp << std::endl;
//     std::cout << "state timestamp: " << succ_state[s].timestamp << std::endl;
//     // std::cout << "con timestamp: " << it->timestamp << std::endl;
//     std::cout << "Stop Primitive is Passed" << std::endl;
//     curr->state.printCopterStateInfo("curr state");

//     auto timestamp = succ_state[s].timestamp;
//     for ( auto it = cons.begin(); it != cons.end(); it++ )
//     {
//         if ( timestamp != it->timestamp  )
//             continue;

//         if ( it->type == 0 && StateCollision( succ_state[s], it->con_state) )
//             std::cout << "state collision" << std::endl;

//         if ( it->type == 1 && PrimitveCollision( succ_action_pr[s], it->con_pr) )
//         {
//             std::cout << "primitive collision" << std::endl;
//             succ_action_pr[s].printMotionPrimitiveInfo();
//             it->con_pr.printMotionPrimitiveInfo();

//         }
            
//     }
// }