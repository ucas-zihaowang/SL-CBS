
#include <ros/ros.h>
#include "common.h"
#include "copter_common.h"
#include "slplanner.h"

#include "load_parameter.h"
#include "load_corridor.h"

#include "opencv_plot.h"

#define PLOT_LEVEL 3

static double ObsRadius = 0;
static double UAVRadius = 0;

int main( int argc, char **argv )
{
    std::string name = "test_slplanner_demo_by_corridor_node";
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


    CopterState2d start( cp.cm, cp.yaw_use, Vec2f(instance.start(0), instance.start(1)) );
    CopterState2d goal( cp.cm, cp.yaw_use, Vec2f(instance.goal(0), instance.goal(1)) );
    // CopterState2d start( cp.cm, cp.yaw_use, Vec2f(2.5, -4.5) );
    // CopterState2d goal( cp.cm, cp.yaw_use, Vec2f(27, 1.0) );
    // start.printCopterStateInfo("start state info: ");
    // goal.printCopterStateInfo("goal state info: ");


    SLPlanner2d slsolver( gmap, cp, start, goal );


    Solution2d sol;
    Timer one_clock(true);
    bool res  = slsolver.search( sol );
    double spend_time = one_clock.Elapsed();
    if ( res )
    {
        std::cout << ANSI_COLOR_GREEN "Find the Solution: Take " <<  spend_time << "ms" ANSI_COLOR_RESET << std::endl;
        // std::cout << ANSI_COLOR_GREEN "ExpandStateNodeNum: " << slsolver.getExpandStateNodeSet().size() << ANSI_COLOR_RESET << std::endl;
        // std::cout << ANSI_COLOR_GREEN "ExpandStateNum: " << slsolver.getExpandStates().size() << ANSI_COLOR_RESET << std::endl;
        // std::cout << ANSI_COLOR_GREEN "ExpandPrNum: " << slsolver.getExpandPrimitive().size() << ANSI_COLOR_RESET << std::endl;
        sol.printSolutionInfo();
    }
    else
    {
        std::cout << ANSI_COLOR_GREEN "Can Not Find the Solution: Take " <<  spend_time << "ms" ANSI_COLOR_RESET << std::endl;
        return 0;
    }


    Trajectory2d traj(sol.prs);
    OpenCVPlot plot( gmap );
    plot.drawPoints( gmap.getOccupiedSpace(), black ); 
    plot.drawCircle( start.pos, blue, 5, 2); 
    plot.drawCircle( goal.pos, cyan, 5, 2); 
    plot.drawPoints( slsolver.getExpandStateNodeSet(), grey, 3 ); 


#if PLOT_LEVEL == 1
    plot.drawTraj(traj, red, 2);
    plot.show( name );

#elif PLOT_LEVEL == 2
    plot.drawDynamicTraj( name, traj, red, 2 );

#elif PLOT_LEVEL == 3
    plot.drawDynamicProcess( name, traj, slsolver.getExpandStates(), slsolver.getExpandPrimitive(), slsolver.getExpandedActions(),  magenta,  1);
#endif

    return 0;
}