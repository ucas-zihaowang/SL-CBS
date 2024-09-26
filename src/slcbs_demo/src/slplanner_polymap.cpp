
#include <ros/ros.h>
#include "common.h"
#include "copter_common.h"
#include "cbssearch.h"

#include "load_parameter.h"
#include "load_uavs.h"
#include "opencv_plot.h"

#include <decomp_ros_utils/data_ros_utils.h>
#include <planning_ros_utils/data_ros_utils.h>
#include <planning_ros_utils/primitive_ros_utils.h>

static double UAVRadius = 0.1;
static double ObsRadius = 0;
double resolution = 0.01;
int UAVID = 0;

#define PLOT_LEVEL 3

int main(int argc, char **argv)
{
    std::string name = "slplanner_polymap_node";
    ros::init(argc, argv, name );
 
    // ros::NodeHandle nh_private;

    ros::NodeHandle nh_private("~");


    ros::Publisher bound_pub = nh_private.advertise<decomp_ros_msgs::PolyhedronArray>("bound", 1, true);
    ros::Publisher poly_pub = nh_private.advertise<decomp_ros_msgs::PolyhedronArray>("polyhedrons", 1, true);
    ros::Publisher prs_pub = nh_private.advertise<planning_ros_msgs::PrimitiveArray>("primitives", 1, true);
    ros::Publisher traj_pub = nh_private.advertise<planning_ros_msgs::Trajectory>("trajectory", 1, true);
    ros::Publisher s_pub = nh_private.advertise<sensor_msgs::PointCloud>("starts", 1, true);
    ros::Publisher g_pub = nh_private.advertise<sensor_msgs::PointCloud>("goals", 1, true);
    ros::Publisher state_pub = nh_private.advertise<sensor_msgs::PointCloud>("states", 1, true);

    std::string parameterpath;
    nh_private.getParam("parameterpath", parameterpath );
    std::string uavswarminitstatepath;
    nh_private.getParam("uavswarminitstatepath", uavswarminitstatepath );
    int obstaclesconfig;
    nh_private.getParam("obstaclesconfig", obstaclesconfig); // 1 or 2


    Vec2f origin, dim;
    nh_private.param("origin_x", origin(0), 0.0);
    nh_private.param("origin_y", origin(1), -5.0);
    nh_private.param("range_x", dim(0), 10.0);
    nh_private.param("range_y", dim(1), 10.0);


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
    std::vector<PolyObsGeo2d> static_obs;
    vec_E<Polyhedron2D> static_poly_obs;


    if ( obstaclesconfig == 1)
    {
        Polyhedron2D rec1;
        rec1.add(Hyperplane2D(Vec2f(4, 0), -Vec2f::UnitX()));
        rec1.add(Hyperplane2D(Vec2f(6, 0), Vec2f::UnitX()));
        rec1.add(Hyperplane2D(Vec2f(5, -1), -Vec2f::UnitY()));
        rec1.add(Hyperplane2D(Vec2f(5, 1), Vec2f::UnitY()));

        // rec1.add(Hyperplane2D(Vec2f(4.25, 0), -Vec2f::UnitX()));
        // rec1.add(Hyperplane2D(Vec2f(5.75, 0), Vec2f::UnitX()));
        // rec1.add(Hyperplane2D(Vec2f(5, -0.75), -Vec2f::UnitY()));
        // rec1.add(Hyperplane2D(Vec2f(5, 0.75), Vec2f::UnitY()));

        static_obs.push_back(PolyObsGeo2d(rec1, Vec2f::Zero()));
        static_poly_obs.push_back(rec1);
    }
    else if ( obstaclesconfig == 2 )
    {
        Polyhedron2D rec2;
        rec2.add(Hyperplane2D(Vec2f(4, 0), -Vec2f::UnitX()));
        rec2.add(Hyperplane2D(Vec2f(6, 0), Vec2f::UnitX()));
        rec2.add(Hyperplane2D(Vec2f(5, 0.2), -Vec2f::UnitY()));
        rec2.add(Hyperplane2D(Vec2f(5, 5.5), Vec2f::UnitY()));
        static_obs.push_back(PolyObsGeo2d(rec2, Vec2f::Zero()));

        Polyhedron2D rec3;
        rec3.add(Hyperplane2D(Vec2f(4, 0), -Vec2f::UnitX()));
        rec3.add(Hyperplane2D(Vec2f(6, 0), Vec2f::UnitX()));
        rec3.add(Hyperplane2D(Vec2f(5, -5.5), -Vec2f::UnitY()));
        rec3.add(Hyperplane2D(Vec2f(5, -0.2), Vec2f::UnitY()));
        static_obs.push_back(PolyObsGeo2d(rec3, Vec2f::Zero()));

        static_poly_obs.push_back(rec2);
        static_poly_obs.push_back(rec3);
    }


    GlobalMap2d gmap( dim, resolution, origin, UAVGeoDes, ObsGeoDes, static_obs );
    // gmap.printGlobalMapInfo();


    CopterParameter2d cp( para.getCM(), para.getYawUse(), para.getU(), para.getUyaw(), para.getSampleNum(), para.getDt() );
    cp.setDynamicPara( para.getMaxVel(), para.getMaxAcc(), para.getMaxJrk(), para.getMaxYaw() );
    cp.setTolPara( para.getTolPos(), para.getTolVel(), para.getTolAcc(), para.getTolYaw() );
    cp.setCostWeightPara( para.getWJ(), para.getWT(), para.getWYaw() );
    cp.setPlannerPara( para.getMapType(), para.getHeurType(), para.getMPPenalty(), para.getYawMPPenalty() );
    cp.InitRelatedVariable();
    // cp.printCopterParameterInfo();


    UavSwarmStartGoalLoader2d uavswarmstartgoal(uavswarminitstatepath);
    if ( !uavswarmstartgoal.exist() )
    {
        std::cout << ANSI_COLOR_RED "failed to load uav_swarm_start_goal!" ANSI_COLOR_RESET << std::endl;
        return -1;
    }
    // uavswarmstartgoal.printUavSwarmStartGoalInfo();

    int uavswarmssize = uavswarmstartgoal.getUavNum();
    std::vector<CopterState2d> starts, goals;
    for( int i = 0; i < uavswarmssize; i++ )
    {
        // CopterState2d start( cp.cm, cp.yaw_use, uavswarmstartgoal.getUavSwarmStartGoal(i).start_pos, uavswarmstartgoal.getUavSwarmStartGoal(i).start_yaw );
        // CopterState2d goal( cp.cm, cp.yaw_use, uavswarmstartgoal.getUavSwarmStartGoal(i).goal_pos,  uavswarmstartgoal.getUavSwarmStartGoal(i).goal_yaw );
        CopterState2d start( cp.cm, cp.yaw_use, uavswarmstartgoal.getUavSwarmStartGoal(i).start_pos );
        CopterState2d goal( cp.cm, cp.yaw_use, uavswarmstartgoal.getUavSwarmStartGoal(i).goal_pos );
        starts.push_back(start);
        goals.push_back(goal);
    }


    SLPlanner2d slsolver( gmap, cp, starts[UAVID], goals[UAVID], true );


    Solution2d sol;
    Timer one_clock(true);
    bool res  = slsolver.search(sol);
    double spend_time = one_clock.Elapsed();
    if ( res )
    {
        sol.printSolutionInfo();
        std::cout << ANSI_COLOR_GREEN "Find the Solution: Take " <<  spend_time << "ms" ANSI_COLOR_RESET << std::endl;
    }
    else
    {
        std::cout << ANSI_COLOR_GREEN "Can Not Find the Solution: Take " <<  spend_time << "ms" ANSI_COLOR_RESET << std::endl;
        return 0;
    }

    Trajectory2d traj(sol.prs);
    OpenCVPlot plot( gmap );

    plot.drawCircle( starts[UAVID].pos, blue, UAVRadius/resolution, 2 ); 
    plot.drawCircle( goals[UAVID].pos, cyan, UAVRadius/resolution, 2 ); 

    // plot.drawPoints( gmap.getOccupiedSpace(), black ); 
    plot.drawSolidRect( Vec2f(4,1), Vec2f(6,-1), black );

    // plot.drawPoints( slsolver.getExpandStateNodeSet(), grey, 3 ); 
    plot.show( name );


#if PLOT_LEVEL == 1
    plot.drawTraj(traj, red, 2);
    plot.show( name );
    plot.save( name + ".jpg");

#elif PLOT_LEVEL == 2
    plot.drawDynamicTraj( name, traj, red, 2 );

#elif PLOT_LEVEL == 3
    plot.drawDynamicProcess( name, traj, slsolver.getExpandStates(), slsolver.getExpandPrimitive(), slsolver.getExpandedActions(),  magenta,  1);
    plot.show( name );
    plot.save( name + ".jpg");
#endif

    return 0;
}