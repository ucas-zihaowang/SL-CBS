
#include <ros/ros.h>
#include "common.h"
#include "copter_common.h"
#include "cbssearch.h"

#include "load_parameter.h"
#include "load_corridor.h"
#include "load_corridor_uavs.h"

#include "opencv_plot.h"

#define PLOT_LEVEL 1

static double ObsRadius = 0;
static double UAVRadius = 0;

int main( int argc, char **argv )
{
    std::string name = "test_slcbs_demo_by_corridor_node";
    ros::init(argc, argv, name );

    // ros::NodeHandle nh_private;

    ros::NodeHandle nh_private("~");
    std::string benchmarkpath;
    nh_private.getParam("benchmark", benchmarkpath );
    std::string parameterpath;
    nh_private.getParam("parameterpath", parameterpath );
    std::string solutionspath;
    nh_private.getParam("solutionspath", solutionspath );
    std::string mapname;
    nh_private.getParam("mapname", mapname );
    std::string outputpath = solutionspath  + mapname + "_result.yaml";
    std::cout << ANSI_COLOR_YELLOW << name << " is running" ANSI_COLOR_RESET << std::endl;


    std::string instancepath = benchmarkpath + mapname + ".yaml";
    // std::cout << "instancepath: " << instancepath << std::endl;
    InstanceLoader<Vec2i, Vec2f> instance(instancepath);
    if ( !instance.exist() )
    {
        std::cout << ANSI_COLOR_RED "failed to load instance!" ANSI_COLOR_RESET << std::endl;
        return -1;
    }
    // instance.printInstanceInfo();

    ParameterLoader para(parameterpath);
    if ( !para.exist() )
    {
        std::cout << ANSI_COLOR_RED "failed to load parameter!" ANSI_COLOR_RESET << std::endl;
        return -1;
    }
    // para.printParameterInfo();


    CopterGeo2d UAVGeoDes( UAVRadius );
    std::vector<Vec2f> obstacles;
    std::vector<Eigen::VectorXd> obstacles_kd;
    obstacles.resize(0); 
    obstacles_kd.resize(0); 
    MyKDTree obs_kd_tree( 2, obstacles_kd,  false, 10 );
    ObsGeo2d ObsGeoDes( obstacles, ObsRadius, obs_kd_tree );


    GlobalMap2d gmap( instance.dim(), instance.resolution(), instance.origin(), instance.data(),  UAVGeoDes, ObsGeoDes );
    // gmap.printGlobalMapInfo();


    CopterParameter2d cp( para.getCM(), para.getYawUse(), para.getU(), para.getUyaw(), para.getSampleNum(), para.getDt() );
    cp.setDynamicPara( para.getMaxVel(), para.getMaxAcc(), para.getMaxJrk(), para.getMaxYaw() );
    cp.setTolPara( para.getTolPos(), para.getTolVel(), para.getTolAcc(), para.getTolYaw() );
    cp.setCostWeightPara( para.getWJ(), para.getWT(), para.getWYaw() );
    cp.setPlannerPara( para.getMapType(), para.getHeurType(), para.getMPPenalty(), para.getYawMPPenalty() );
    cp.InitRelatedVariable();
    // cp.printCopterParameterInfo();


    std::string uavswarmstartgoalpath = benchmarkpath + "corridor_uavs.yaml";
    UavSwarmStartGoalLoader2d uavswarmstartgoal(uavswarmstartgoalpath);
    if ( !uavswarmstartgoal.exist() )
    {
        std::cout << ANSI_COLOR_RED "failed to load uav_swarm_start_goal!" ANSI_COLOR_RESET << std::endl;
        return -1;
    }
    // uavswarmstartgoal.printUavSwarmStartGoalInfo();
    std::vector<CopterState2d> starts, goals;
    for( int i = 0; i < uavswarmstartgoal.getUavNum(); i++ )
    {
        // CopterState2d start( cp.cm, cp.yaw_use, uavswarmstartgoal.getUavSwarmStartGoal(i).start_pos, uavswarmstartgoal.getUavSwarmStartGoal(i).start_yaw );
        // CopterState2d goal( cp.cm, cp.yaw_use, uavswarmstartgoal.getUavSwarmStartGoal(i).goal_pos,  uavswarmstartgoal.getUavSwarmStartGoal(i).goal_yaw );
        CopterState2d start( cp.cm, cp.yaw_use, uavswarmstartgoal.getUavSwarmStartGoal(i).start_pos );
        CopterState2d goal( cp.cm, cp.yaw_use, uavswarmstartgoal.getUavSwarmStartGoal(i).goal_pos );
        starts.push_back(start);
        goals.push_back(goal);
    }


    CBSSearch2d cbssolver( gmap, cp, starts, goals, false );


    std::vector<Solution2d> sols;
    Timer one_clock(true);
    bool res  = cbssolver.search(sols);
    // bool res  = cbssolver.cbssearch_id(sols);
    double spend_time = one_clock.Elapsed();
    if ( res )
    {
        // cbssolver.printSolutionsInfo( sols );
        std::cout << ANSI_COLOR_GREEN "Find the Solution: Take " <<  spend_time << "ms" ANSI_COLOR_RESET << std::endl;
    }
    else
    {
        std::cout << ANSI_COLOR_GREEN "Can Not Find the Solution: Take " <<  spend_time << "ms" ANSI_COLOR_RESET << std::endl;
        return 0;
    }


    OpenCVPlot plot( gmap );
    plot.drawPoints( gmap.getOccupiedSpace(), black ); 


#if PLOT_LEVEL == 0
    int uav_num = starts.size();
    for( int i = 0; i < uav_num; i++ )
    {

        plot.drawCircle( starts[i].pos, blue, 5, 2);
        plot.drawCircle( goals[i].pos, cyan, 5, 2);

        Trajectory2d traj( sols[i].prs );
        plot.drawTraj( traj, red, 2 );
    }
    plot.show( name );

#elif PLOT_LEVEL == 1
    int uav_num = starts.size();
    std::vector<Trajectory2d> trajs;
    for( int i = 0; i < uav_num; i++ )
    {

        plot.drawCircle( starts[i].pos, blue, 5, 2);
        plot.drawCircle( goals[i].pos, cyan, 5, 2);
        Trajectory2d traj( sols[i].prs );
        trajs.push_back( traj );
    }

    plot.drawMultiDynamicTraj( name, trajs, red, 2 );
#endif

    return 0;
}