
#include "cbssearch.h"

template <int Dim>
bool CBSSearch<Dim>::search(std::vector<Solution<Dim>> &sols)
{
    root.node_id = 0;
    root.cost = 0;
    root.sols.resize( uav_num );
    root.allcons.resize( uav_num );

    for( int i = 0; i < uav_num; i++ )
    {
        Solution<Dim> sol;
        if( !slplanners[i]->search(sol)  )
        {
            std::cout << ANSI_COLOR_RED "Fail to Generate Root Node " << i << ANSI_COLOR_RESET << std::endl;
            return false;
        }
        this->iter_solutions[i] = sol;
        root.sols[i] = sol;
        root.cost += sol.totalcost;
    }
    std::cout << ANSI_COLOR_YELLOW "Success to Generate Root Node" ANSI_COLOR_RESET << std::endl;
    CBSNodePtr<Dim> rootPtr = std::make_shared<CBSNode<Dim>>(root);


    findAllConflicts( rootPtr );


    CBSPriorityQueue<CBSNode<Dim>> cbs_open_list;
    rootPtr->open_handle = cbs_open_list.push( rootPtr );

    int iteration = 0;
    int node_id = 1;
    Timer t_one_clock(true);
    while ( !cbs_open_list.empty() )
    {
        iteration++;
#ifdef CBSSEARCH_DEBUG
        std::cout << ANSI_COLOR_RED << "cbsplanner iteration num: " << iteration << ANSI_COLOR_RESET << std::endl;

        for( auto it = cbs_open_list.ordered_begin(); it != cbs_open_list.ordered_end(); it++ )
        {
            std::cout << (*it)->node_id << "    " << (*it)->cost << "    " << (*it)->conflicts_num << std::endl;
        }
#endif


        CBSNodePtr<Dim> currCBSNodePtr = cbs_open_list.top();
        cbs_open_list.pop();

        // printConflictsInfo( currCBSNodePtr->conflicts );
        // printSolutionsInfo( currCBSNodePtr->sols );


        if ( currCBSNodePtr->conflicts_num == 0 )
        {
#ifdef CBSSEARCH_ITER_DEBUG
            std::cout << ANSI_COLOR_YELLOW  "cbsplanner iteration num: " << iteration << ANSI_COLOR_RESET << std::endl;
#endif
            sols = currCBSNodePtr->sols;
            return true;
        }

#ifdef CBSSEARCH_DEBUG
        // if ( iteration == 20 )
        // {
        //     std::cout << ANSI_COLOR_YELLOW  "cbsplanner iteration num: " << iteration << ANSI_COLOR_RESET << std::endl;
        //     std::cout << ANSI_COLOR_YELLOW  "currCBSNodePtr->conflicts_num: " << currCBSNodePtr->conflicts_num << ANSI_COLOR_RESET << std::endl;
        //     printConflictsInfo( currCBSNodePtr->conflicts );
        //     sols = currCBSNodePtr->sols;
        //     // int sz_tmp = cbs_open_list.size();
        //     // int in_tmp = 0;
        //     // for( auto it = cbs_open_list.ordered_begin(); it != cbs_open_list.ordered_end(); it++ )
        //     // {
        //     //     if ( in_tmp == sz_tmp - 1 )
        //     //         sols = (*it)->sols;
        //     //     in_tmp++;
        //     // }
        //     return true;
        // }
#endif


        Conflict<Dim> conflict = chooseConflict( currCBSNodePtr );
        std::map< int, Constraints<Dim> > allcons;
        divideConstraints( conflict, allcons );


        for( const auto &cons: allcons )
        {
            int con_uav_id = cons.first;
            CBSNode<Dim> childCBSNode;
            childCBSNode.node_id = node_id;
            childCBSNode.sols = currCBSNodePtr->sols;
            childCBSNode.cost = currCBSNodePtr->cost;
            childCBSNode.allcons = currCBSNodePtr->allcons;
            childCBSNode.conflicts = currCBSNodePtr->conflicts;
            childCBSNode.conflicts_num = currCBSNodePtr->conflicts_num;

            childCBSNode.allcons[con_uav_id].insert( cons.second.begin(), cons.second.end() );
            childCBSNode.cost -= childCBSNode.sols[con_uav_id].totalcost;

            Solution<Dim> sol;
            int minLength = slplanners[con_uav_id]->computeMaxConsTimestamp( childCBSNode.allcons[con_uav_id] );

            if( !slplanners[con_uav_id]->search(sol, childCBSNode.allcons[con_uav_id], minLength ) )
            {
                continue;
            }


            // handle_conflicts.push_back( conflict );
            // handle_constraints.push_back( childCBSNode.allcons[con_uav_id] );
            // handle_solutions.push_back( sol );
            // handle_uav_id.push_back( con_uav_id );


            childCBSNode.sols[con_uav_id] = sol;
            childCBSNode.cost += sol.totalcost;
            CBSNodePtr<Dim> childCBSNodePtr = std::make_shared<CBSNode<Dim>>( childCBSNode );
            findAllConflicts( childCBSNodePtr, currCBSNodePtr, con_uav_id );
            childCBSNodePtr->open_handle = cbs_open_list.push( childCBSNodePtr );


            // printConflictsInfo( childCBSNodePtr->conflicts );
            // printSolutionsInfo( childCBSNodePtr->sols );

            node_id += 1;
#ifdef CBSSEARCH_END_DEBUG

            if ( childCBSNodePtr->conflicts_num == 0 )
            {
#ifdef CBSSEARCH_ITER_DEBUG
                std::cout << ANSI_COLOR_YELLOW  "cbsplanner iteration num: " << iteration << ANSI_COLOR_RESET << std::endl;
#endif
                sols = childCBSNodePtr->sols;
                return true;
            }
#endif
        } // end for

        if ( t_one_clock.Elapsed() > CBS_RUNTIME || iteration >= MAX_HIGH_ITERATION_NUM )
        {
#ifdef CBSSEARCH_ITER_DEBUG
            std::cout << ANSI_COLOR_YELLOW  "cbsplanner iteration num: " << iteration << ANSI_COLOR_RESET << std::endl;
#endif
            break;
        }

    } // end while
    return false;
}

template <int Dim>
bool CBSSearch<Dim>::cbssearch_id(std::vector<Solution<Dim>> &sols)
{

    bool res = generateCBSRootNode();
    if ( res )
        std::cout << ANSI_COLOR_YELLOW "Success to Generate Root Node" ANSI_COLOR_RESET << std::endl;
    else
        std::cout << ANSI_COLOR_RED "Fail to Generate Root Node" ANSI_COLOR_RESET << std::endl;
    

    int iteration_id = 0;
    Timer t_one_clock(true);
    while ( 1 )
    {

        if ( this->iter_conflicts_num == 0 )
        {
            std::cout << ANSI_COLOR_YELLOW "CBS Algorithm ID Iteration Num: " << iteration_id << ANSI_COLOR_RESET << std::endl;
            sols = iter_solutions;
            break;
        }

        std::vector< std::vector<int> > groups;
        divideGroups( iter_conflicts, groups );

#ifndef MULTI_THREAD

        for( auto group : groups )
        {

            int group_mem_num = group.size();

            // for( int i = 0; i < group_mem_num; i++ )
            //     std::cout << group[i] << " ";
            // std::cout << std::endl;

            if( group_mem_num == 1 )
                continue;
            
            std::vector<Solution<Dim>> group_sols;
            int retFlag;


            search_group(group_sols, group, retFlag);
            if ( retFlag == -1 )
                return false;


            for( int i = 0; i < group_mem_num; i++ )
                iter_solutions[ group[i] ] = group_sols[i];
        }
#else

        int groups_num = groups.size();
        std::vector< std::vector<Solution<Dim>> > all_group_sols;
        all_group_sols.resize(groups_num);

        std::vector<std::thread> threads;
        std::vector<int> retFlags(groups_num);
        for( int i = 0; i < groups_num; i++ )
        {
            // threads[i] = std::thread( search_group( all_group_sols[i], groups[i], retFlags[i] ) );
            threads.push_back( std::thread( &CBSSearch<Dim>::search_group, this, std::ref(all_group_sols[i]), std::ref(groups[i]), std::ref(retFlags[i]) ) );
        }
        for( int i = 0; i < groups_num; i++ )
            threads[i].join();

        for( int i = 0; i < groups_num; i++ )
        {
            if( retFlags[i] == -1 )
                return false;
            int group_mem_num = groups[i].size();
            for( int j = 0; j < group_mem_num; j++ )
            {
                iter_solutions[ groups[i][j] ] = all_group_sols[i][j];
            }
        }
#endif


        validateSolutions( iter_solutions );

        iteration_id++;

        if ( t_one_clock.Elapsed() > CBS_RUNTIME  )
        // if ( t_one_clock.Elapsed() > CBS_RUNTIME || iteration_id > 5 )
        {
            std::cout << ANSI_COLOR_YELLOW "CBS Algorithm ID Iteration Num: " << iteration_id << ANSI_COLOR_RESET << std::endl;
            // std::cout << ANSI_COLOR_YELLOW  "cbsplanner_id runtime is Over! " << ANSI_COLOR_RESET << std::endl;
            return false;
        }
    }
    return true;
}

template <int Dim>
bool CBSSearch<Dim>::generateCBSRootNode()
{
    root.node_id = 0;
    root.cost = 0;
    root.sols.resize( uav_num );
    root.allcons.resize( uav_num );

    for( int i = 0; i < uav_num; i++ )
    {
        Solution<Dim> sol;
        if( !slplanners[i]->search(sol)  )
        {
            return false;
        }
        this->iter_solutions[i] = sol;
        root.sols[i] = sol;
        root.cost += sol.totalcost;
    }
    
    CBSNodePtr<Dim> rootPtr = std::make_shared<CBSNode<Dim>>(root);


    findAllConflicts( rootPtr );
    this->iter_conflicts = rootPtr->conflicts;
    this->iter_conflicts_num = rootPtr->conflicts_num;

    return true;
}

template <int Dim>
void CBSSearch<Dim>::divideGroups( std::list<Conflict<Dim>> &conflicts, std::vector<std::vector<int>> &groups )
{
    static int step = 0;
    step += 1;

    if ( step >= STOP_GROUP )
    {

        std::vector<int> group;
        for( int i = 0; i < uav_num; i++ )
            group.push_back( i );
        groups.push_back( group );
        return;
    }

    int conflict_graph[ uav_num ][ uav_num ];
    int flags[ uav_num ];
    for(int i = 0; i < uav_num; i++ )
    {
        flags[i] = 0;
        for( int j = 0; j < uav_num; j++ )
        {
            conflict_graph[i][j] = 0;
        }
    }
    for( auto conf : conflicts )
    {
        conflict_graph[conf.one_uav_id][conf.another_uav_id] += 1;
        conflict_graph[conf.another_uav_id][conf.one_uav_id] += 1;
    }

    // std::cout << "print conflict graph " <<  std::endl;
    // for(int i = 0; i < uav_num; i++ )
    // {
    //     for( int j = 0; j < uav_num; j++ )
    //     {
    //         std::cout << conflict_graph[i][j] << " ";
    //     }
    //     std::cout << std::endl;
    // }

    // if ( step >= 5 )
    // {
    //     std::vector<int> group;
    //     // 所有发生冲突的分成一个组;
    //     for(int i = 0; i < uav_num; i++ )
    //     {
    //         std::vector<int> group1;
    //         group1.push_back(i);
    //         bool flagNoCon = 1;
    //         for( int j = 0; j < uav_num; j++ )
    //         {
    //             if ( conflict_graph[i][j] > 0 )
    //             {
    //                 flagNoCon = 0;
    //                 group.push_back(i);
    //                 break;
    //             }
    //         }
    //         if ( flagNoCon )
    //             groups.push_back( group1 );
    //     }
    //     groups.push_back(group);
    //     return;
    // }

    for( int i = 0; i < uav_num; i++ )
    {
        if ( flags[i] == 1 )
            continue;
        std::vector<int> group;
        group.push_back( i );

        std::queue<int> myqueue;
        myqueue.push(i);
        while( !myqueue.empty() )
        {
            int curr_id = myqueue.front();
            myqueue.pop();
            flags[curr_id] = 1;


            for( int j = i+1; j < uav_num; j++ )
            {

                int group_size = group.size();
                bool exist = false;
                for( int k = 0; k < group_size; k++ )
                {
                    if ( j == group[k] )
                    {
                        exist = true;
                        break;
                    }
                }

                if( conflict_graph[curr_id][j] > 0 && (flags[j] == 0) && (!exist) )
                {
                    myqueue.push( j );
                    group.push_back( j );
                }
            }
        }
        groups.push_back( group );
    }
}

template <int Dim>
void CBSSearch<Dim>::search_group( std::vector<Solution<Dim>> &sols, std::vector<int> &group, int &retFlag )
{

    int group_mem_size = group.size();

    CBSNode<Dim> newRoot;
    newRoot.node_id = 0;
    newRoot.cost = 0;
    newRoot.sols.resize( group_mem_size );
    newRoot.allcons.resize( group_mem_size );
    for( int i = 0; i < group_mem_size; i++ )
    {
        newRoot.sols[i] = this->iter_solutions[ group[i] ];
        newRoot.cost += this->iter_solutions[ group[i] ].totalcost;
        // newRoot.sols[i] = root.sols[ group[i] ];
        // newRoot.cost += root.sols[ group[i] ].totalcost;
    }
    CBSNodePtr<Dim> newRootPtr = std::make_shared<CBSNode<Dim>>( newRoot );
    findAllConflicts( newRootPtr );
    CBSPriorityQueue<CBSNode<Dim>> cbs_open_list;
    newRootPtr->open_handle = cbs_open_list.push( newRootPtr );

    int iteration = 0;
    int node_id = 1;
    Timer t_one_clock(true);

    while ( !cbs_open_list.empty() )
    {
        CBSNodePtr<Dim> currCBSNodePtr = cbs_open_list.top();
        cbs_open_list.pop();

        if ( currCBSNodePtr->conflicts_num == 0 )
        {
#ifdef CBSSEARCH_ITER_DEBUG
            std::cout << ANSI_COLOR_YELLOW  "cbsplanner iteration num: " << iteration << ANSI_COLOR_RESET << std::endl;
#endif
            sols = currCBSNodePtr->sols;
            retFlag = 0;
            return;
        }

        Conflict<Dim> conflict = chooseConflict( currCBSNodePtr );
        std::map< int, Constraints<Dim> > allcons;
        divideConstraints( conflict, allcons );


        for( const auto &cons: allcons )
        {
            int con_uav_id = cons.first;
            CBSNode<Dim> childCBSNode;
            childCBSNode.node_id = node_id;
            childCBSNode.sols = currCBSNodePtr->sols;
            childCBSNode.cost = currCBSNodePtr->cost;
            childCBSNode.allcons = currCBSNodePtr->allcons;
            childCBSNode.conflicts = currCBSNodePtr->conflicts;
            childCBSNode.conflicts_num = currCBSNodePtr->conflicts_num;

            childCBSNode.allcons[con_uav_id].insert( cons.second.begin(), cons.second.end() );
            childCBSNode.cost -= childCBSNode.sols[con_uav_id].totalcost;

            Solution<Dim> sol;
            int minLength = slplanners[group[con_uav_id]]->computeMaxConsTimestamp( childCBSNode.allcons[con_uav_id] );
            if( !slplanners[group[con_uav_id]]->search(sol, childCBSNode.allcons[con_uav_id], minLength ) )
            {
                continue;
            }

            childCBSNode.sols[con_uav_id] = sol;
            childCBSNode.cost += sol.totalcost;

            CBSNodePtr<Dim> childCBSNodePtr = std::make_shared<CBSNode<Dim>>( childCBSNode );
            findAllConflicts( childCBSNodePtr, currCBSNodePtr, con_uav_id );
            childCBSNodePtr->open_handle = cbs_open_list.push( childCBSNodePtr );

            node_id += 1;

#ifdef CBSSEARCH_END_DEBUG

            if ( childCBSNodePtr->conflicts_num == 0 )
            {
#ifdef CBSSEARCH_ITER_DEBUG
                std::cout << ANSI_COLOR_YELLOW  "cbsplanner iteration num: " << iteration << ANSI_COLOR_RESET << std::endl;
#endif
                sols = childCBSNodePtr->sols;
                retFlag = 0;
                return;
            }
#endif
        } // end for

        iteration++;

        if ( t_one_clock.Elapsed() > CBS_RUNTIME || iteration >= MAX_HIGH_ITERATION_NUM )
        {
#ifdef CBSSEARCH_ITER_DEBUG
            std::cout << ANSI_COLOR_YELLOW  "cbsplanner iteration num: " << iteration << ANSI_COLOR_RESET << std::endl;
#endif
            break;
        }
    } // end while

    retFlag = -1;
    return;
}

template <int Dim>
void CBSSearch<Dim>::findAllConflicts( CBSNodePtr<Dim> currCBSNodePtr, CBSNodePtr<Dim> parentCBSNodePtr, int uav_id )
{
    
    if ( currCBSNodePtr->node_id == 0 )
    {

        int maxTimestamp = 0;
        int curr_uav_num = currCBSNodePtr->sols.size();
        for( int i = 0; i  < curr_uav_num; i++ )
        {
            int tmp = currCBSNodePtr->sols[i].states.size();
            if ( maxTimestamp < tmp )
            {
                maxTimestamp = tmp;
            }
        }

        for( int t = 1; t < maxTimestamp; t++ )
        {
            for( int i = 0; i < curr_uav_num; i++ )
            {
                for( int j = i+1; j < curr_uav_num; j++ )
                {
                    Conflict<Dim> conflict;
                    if( checkConflict( currCBSNodePtr->sols, i, j, t, conflict) )
                        currCBSNodePtr->conflicts.push_back( conflict );
                }
            }
        }
        currCBSNodePtr->conflicts_num = currCBSNodePtr->conflicts.size();
    }
    else
    {

        currCBSNodePtr->conflicts.clear();
        for( auto it = parentCBSNodePtr->conflicts.begin(); it != parentCBSNodePtr->conflicts.end(); it++ )
            if ( (*it).one_uav_id != uav_id && (*it).another_uav_id != uav_id )
                currCBSNodePtr->conflicts.push_back( *it );
        

        int maxTimestamp = 0;
        int curr_uav_num = currCBSNodePtr->sols.size();
        for( int i = 0; i  < curr_uav_num; i++ )
        {
            int tmp = currCBSNodePtr->sols[i].states.size();
            if ( maxTimestamp < tmp )
            {
                maxTimestamp = tmp;
            }
        }


        for( int t = 1; t < maxTimestamp; t++ )
        {
            for( int i = 0; i < curr_uav_num; i++ )
            {
                if( i == uav_id )
                    continue;
                Conflict<Dim> conflict;
                if ( checkConflict( currCBSNodePtr->sols, i, uav_id, t, conflict) )
                    currCBSNodePtr->conflicts.push_back( conflict );
            }
        }
        currCBSNodePtr->conflicts_num = currCBSNodePtr->conflicts.size();
    }
}

template <int Dim>
bool CBSSearch<Dim>::checkConflict( std::vector<Solution<Dim>> &sols, int one_uav, int another_uav, int ts, Conflict<Dim> &conflict)
{
    CopterState<Dim> one_state;
    Primitive<Dim> one_pr;
    bool one_pr_exist = false;
    if ( ts < sols[one_uav].states.size() )
    {
        one_state = sols[one_uav].states[ts];
        one_pr = sols[one_uav].prs[ts-1];
        one_pr_exist = true;
    }
    else
    {
        one_state = sols[one_uav].states.back();

        // one_pr = Primitive<Dim>( one_state, cp.U_Zero, cp.dt, false );
        // one_pr_exist = true;
        // one_state.printCopterStateInfo();
        // one_pr.printMotionPrimitiveInfo();
    }
        

    CopterState<Dim> another_state;
    Primitive<Dim> another_pr;
    bool another_pr_exist = false;
    if ( ts < sols[another_uav].states.size() )
    {
        another_state = sols[another_uav].states[ts];
        another_pr = sols[another_uav].prs[ts-1];
        another_pr_exist = true;
    }
    else
    {
        another_state = sols[another_uav].states.back();
        // another_pr = Primitive<Dim>( another_state, cp.U_Zero, cp.dt, false );
        // another_pr_exist = true;
        // another_state.printCopterStateInfo();
        // another_pr.printMotionPrimitiveInfo();
    }
    

    if ( StateCollision( one_state,another_state ) )
    {
        conflict.type = 0;
        conflict.one_uav_id = one_uav;
        conflict.another_uav_id = another_uav;
        conflict.timestamp = ts;
        conflict.one_state = one_state;
        conflict.another_state = another_state;
        return true;
    }

#if PRIMITIVE_COLLISION_CHECK == 1
    if ( one_pr_exist == 1 && another_pr_exist == 1 )
    {
        // std::cout << "conflict type: 1" << std::endl;
        if ( PrimitiveCollision( one_pr, another_pr ) )
        {
            conflict.type = 1;
            conflict.one_uav_id = one_uav;
            conflict.another_uav_id = another_uav;
            conflict.timestamp = ts;
            conflict.one_pred_pr = one_pr;
            conflict.another_pred_pr = another_pr;
            return true;
        }
    }
    else if ( one_pr_exist == 1 && another_pr_exist == 0 )
    {
        
        if ( StatePrimitiveCollision( one_pr, another_state ) )
        {
            // std::cout << "conflict type: 2" << std::endl;
            conflict.type = 2;
            conflict.one_uav_id = one_uav;
            conflict.another_uav_id = another_uav;
            conflict.timestamp = ts;
            conflict.one_pred_pr = one_pr;
            conflict.another_state = another_state;
            return true;
        }

    }
    else if ( one_pr_exist == 0 && another_pr_exist == 1 )
    {
        
        if ( StatePrimitiveCollision( another_pr, one_state ) )
        {
            // std::cout << "conflict type: 3" << std::endl;
            conflict.type = 3;
            conflict.one_uav_id = one_uav;
            conflict.another_uav_id = another_uav;
            conflict.timestamp = ts;
            conflict.one_state = one_state;
            conflict.another_pred_pr = another_pr;
            return true;
        }
    }

#endif
    return false;
}


template <int Dim>
bool CBSSearch<Dim>::StateCollision( CopterState<Dim> &one_state, CopterState<Dim> &another_state )
{
    if ( (one_state.pos - another_state.pos).template lpNorm<2>() <= gmap.Safe_Dis_Collision )
        return true;
    return false;
}


template <int Dim>
bool CBSSearch<Dim>::PrimitiveCollision( Primitive<Dim> &one_pr, Primitive<Dim> &another_pr )
{

    Eigen::VectorXd coeffs{Vec11f::Zero()};
    for( int i = 0; i < Dim; i++ )
    {
        Eigen::VectorXd one_coeffs(6);
        one_coeffs = one_pr.getPr(i).coeff();
        Eigen::VectorXd another_coeffs(6);
        another_coeffs = another_pr.getPr(i).coeff();

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

        return false;
    }

    double work_time = one_pr.getWorkTime() <= another_pr.getWorkTime()? one_pr.getWorkTime() : another_pr.getWorkTime();


    if ( RootFinder::countRoots( coeffs, 0, work_time ) > 0 )
    {
        return true;
    }
    return false;
}


template <int Dim>
bool CBSSearch<Dim>::StatePrimitiveCollision( Primitive<Dim> &one_pr, CopterState<Dim> &another_state )
{

    Eigen::VectorXd coeffs{Vec11f::Zero()};
    for( int i = 0; i < Dim; i++ )
    {
        Eigen::VectorXd one_coeffs(6);
        one_coeffs = one_pr.getPr(i).coeff();
        one_coeffs(5) -= another_state.pos(i);
        
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

        return false;
    }

    double work_time = one_pr.getWorkTime();

    if ( RootFinder::countRoots( coeffs, 0, work_time ) > 0 )
    {
        return true;
    }
    return false;
}

template <int Dim>
Conflict<Dim> CBSSearch<Dim>::chooseConflict( CBSNodePtr<Dim> currCBSNode )
{

    int minTimestamp = INT_MAX;
    for( auto one_conflict: currCBSNode->conflicts )
    {
        if( one_conflict.timestamp < minTimestamp )
            minTimestamp = one_conflict.timestamp;
    }
    Conflict<Dim> choosedconflit;
    for( auto one_conflict: currCBSNode->conflicts )
    {
        if( one_conflict.timestamp == minTimestamp )
        {
            choosedconflit = one_conflict;
            break;
        }
    }
    return choosedconflit;
}

template <int Dim>
void CBSSearch<Dim>::divideConstraints( Conflict<Dim> &conflict, std::map< int, Constraints<Dim> > &allcons )
{
    if ( conflict.type == 0 )
    {
        Constraints<Dim> c1;
        c1.emplace( Constraint<Dim>( 0, conflict.another_uav_id, conflict.timestamp, conflict.another_state ) );
        allcons[conflict.one_uav_id] = c1;

        Constraints<Dim> c2;
        c2.emplace( Constraint<Dim>( 0, conflict.one_uav_id, conflict.timestamp, conflict.one_state ) );
        allcons[conflict.another_uav_id] = c2;
    }
    else if ( conflict.type == 1 )
    {
        Constraints<Dim> c1;
        c1.emplace( Constraint<Dim>( 1, conflict.another_uav_id, conflict.timestamp, conflict.another_pred_pr ) );
        allcons[conflict.one_uav_id] = c1;

        Constraints<Dim> c2;
        c2.emplace( Constraint<Dim>( 1, conflict.one_uav_id, conflict.timestamp, conflict.one_pred_pr ) );
        allcons[conflict.another_uav_id] = c2;
    }
    else if ( conflict.type == 2 )
    {
        // std::cout << "conflict type is 2" << std::endl;
        Constraints<Dim> c1;
        c1.emplace( Constraint<Dim>( 2, conflict.another_uav_id, conflict.timestamp, conflict.another_state ) );
        allcons[conflict.one_uav_id] = c1;

        Constraints<Dim> c2;
        c2.emplace( Constraint<Dim>( 1, conflict.one_uav_id, conflict.timestamp, conflict.one_pred_pr ) );
        allcons[conflict.another_uav_id] = c2;
    }
    else if ( conflict.type == 3 )
    {
        // std::cout << "conflict type is 3" << std::endl;
        Constraints<Dim> c1;
        c1.emplace( Constraint<Dim>( 1, conflict.another_uav_id, conflict.timestamp, conflict.another_pred_pr ) );
        allcons[conflict.one_uav_id] = c1;

        Constraints<Dim> c2;
        c2.emplace( Constraint<Dim>( 2, conflict.one_uav_id, conflict.timestamp, conflict.one_state ) );
        allcons[conflict.another_uav_id] = c2;
    }
}

template <int Dim>
void CBSSearch<Dim>::printSolutionsInfo( std::vector<Solution<Dim>> &sols )
{
    int sol_num = sols.size();
    double totoalSolsCost = 0;
    double makespan = 0;
    double sumoftime = 0;
    for( int i = 0; i < sol_num; i++ )
    {
        sols[i].printSolutionInfo();
        if ( makespan < sols[i].states.size() )
            makespan = sols[i].states.size() - 1;
        totoalSolsCost += sols[i].totalcost;
        sumoftime += (sols[i].states.size()-1);
    }
    std::cout << "All Solutions' Totalcost is: " << totoalSolsCost << std::endl;
    std::cout << "All Solutions' MakeSpan is: " << makespan * cp.dt  << std::endl;
    std::cout << "All Solutions' SumOfTime is: " << sumoftime * cp.dt   << std::endl;
        
}

template <int Dim>
void CBSSearch<Dim>::validateSolutions( std::vector<Solution<Dim>> &sols )
{

    int maxTimestamp = 0;
    int curr_uav_num = sols.size();
    for( int i = 0; i  < curr_uav_num; i++ )
    {
        int tmp = sols[i].states.size();
        if ( maxTimestamp < tmp )
        {
            maxTimestamp = tmp;
        }
    }

    this->iter_conflicts.clear();

    for( int t = 1; t < maxTimestamp; t++ )
    {
        for( int i = 0; i < curr_uav_num; i++ )
        {
            for( int j = i+1; j < curr_uav_num; j++ )
            {
                Conflict<Dim> conflict;
                if( checkConflict( sols, i, j, t, conflict) )
                {
                    this->iter_conflicts.push_back( conflict );
                }

            }
        }
    }
    this->iter_conflicts_num = this->iter_conflicts.size();
}

// template <int Dim>
// bool CBSSearch<Dim>::revalidateSolutions( std::vector<Solution<Dim>> &sols )
// {

//     int maxTimestamp = 0;
//     int curr_uav_num = sols.size();
//     for( int i = 0; i  < curr_uav_num; i++ )
//     {
//         int tmp = sols[i].states.size();
//         if ( maxTimestamp < tmp )
//         {
//             maxTimestamp = tmp;
//         }
//     }


//     double dt = cp.dt;
//     int N = 5;
//     double it = dt / N;


//     for( int t = 1; t < maxTimestamp; t++ )
//     {
//         for( int i = 0; i < curr_uav_num; i++ )
//         {
//             Solution<Dim> one_sol = sols[i];
//             for( int j = i+1; j < curr_uav_num; j++ )
//             {
//                 Solution<Dim> another_sol = sols[j];

//                 for( int k = 0; k < N; k++ )
//                 {
//                     CopterState<Dim> one_state = one_sol.prs[t-1].evaluate( k * it );
//                     CopterState<Dim> another_state = another_sol.prs[t-1].evaluate( k * it );
//                     if ( StateCollision(one_state,another_state) )
//                     {
//                         std::cout << "Collision Timstamp: " << t << std::endl;
//                         return true;
//                     }
                        
//                 }
                
//             }
//         }
//     }
//     return false;
// }


template class CBSSearch<2>;
template class CBSSearch<3>;

