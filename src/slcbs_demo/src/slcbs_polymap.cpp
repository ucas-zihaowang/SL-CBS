
#include <ros/ros.h>
#include "common.h"
#include "copter_common.h"
#include "cbssearch.h"

#include "load_parameter.h"
#include "load_uavs.h"

#include <decomp_ros_utils/data_ros_utils.h>
#include <planning_ros_utils/data_ros_utils.h>
#include <planning_ros_utils/primitive_ros_utils.h>

static double UAVRadius = 0.1;
static double ObsRadius = 0;
double resolution = 0.1;

int main(int argc, char **argv)
{
    std::string name = "slcbs_polymap_node";
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

        Polyhedron2D rec1_show;
        rec1_show.add(Hyperplane2D(Vec2f(4.25, 0), -Vec2f::UnitX()));
        rec1_show.add(Hyperplane2D(Vec2f(5.75, 0), Vec2f::UnitX()));
        rec1_show.add(Hyperplane2D(Vec2f(5, -0.75), -Vec2f::UnitY()));
        rec1_show.add(Hyperplane2D(Vec2f(5, 0.75), Vec2f::UnitY()));

        static_obs.push_back(PolyObsGeo2d(rec1, Vec2f::Zero()));
        static_poly_obs.push_back(rec1_show);
    }
    else if ( obstaclesconfig == 2 )
    {
        Polyhedron2D rec2;
        rec2.add(Hyperplane2D(Vec2f(4, 0), -Vec2f::UnitX()));
        rec2.add(Hyperplane2D(Vec2f(6, 0), Vec2f::UnitX()));
        rec2.add(Hyperplane2D(Vec2f(5, 0.2), -Vec2f::UnitY()));
        rec2.add(Hyperplane2D(Vec2f(5, 5.5), Vec2f::UnitY()));
        Polyhedron2D rec3;
        rec3.add(Hyperplane2D(Vec2f(4, 0), -Vec2f::UnitX()));
        rec3.add(Hyperplane2D(Vec2f(6, 0), Vec2f::UnitX()));
        rec3.add(Hyperplane2D(Vec2f(5, -5.5), -Vec2f::UnitY()));
        rec3.add(Hyperplane2D(Vec2f(5, -0.2), Vec2f::UnitY()));
        static_obs.push_back(PolyObsGeo2d(rec2, Vec2f::Zero()));
        static_obs.push_back(PolyObsGeo2d(rec3, Vec2f::Zero()));

        Polyhedron2D rec2_show;
        rec2_show.add(Hyperplane2D(Vec2f(4.25, 0), -Vec2f::UnitX()));
        rec2_show.add(Hyperplane2D(Vec2f(5.75, 0), Vec2f::UnitX()));
        rec2_show.add(Hyperplane2D(Vec2f(5, 0.45), -Vec2f::UnitY()));
        rec2_show.add(Hyperplane2D(Vec2f(5, 5.25), Vec2f::UnitY()));
        Polyhedron2D rec3_show;
        rec3_show.add(Hyperplane2D(Vec2f(4.25, 0), -Vec2f::UnitX()));
        rec3_show.add(Hyperplane2D(Vec2f(5.75, 0), Vec2f::UnitX()));
        rec3_show.add(Hyperplane2D(Vec2f(5, -5.25), -Vec2f::UnitY()));
        rec3_show.add(Hyperplane2D(Vec2f(5, -0.45), Vec2f::UnitY()));
        static_poly_obs.push_back(rec2_show);
        static_poly_obs.push_back(rec3_show);
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
    // for( int i = 0; i < 1; i++ )
    {
        // CopterState2d start( cp.cm, cp.yaw_use, uavswarmstartgoal.getUavSwarmStartGoal(i).start_pos, uavswarmstartgoal.getUavSwarmStartGoal(i).start_yaw );
        // CopterState2d goal( cp.cm, cp.yaw_use, uavswarmstartgoal.getUavSwarmStartGoal(i).goal_pos,  uavswarmstartgoal.getUavSwarmStartGoal(i).goal_yaw );
        CopterState2d start( cp.cm, cp.yaw_use, uavswarmstartgoal.getUavSwarmStartGoal(i).start_pos );
        CopterState2d goal( cp.cm, cp.yaw_use, uavswarmstartgoal.getUavSwarmStartGoal(i).goal_pos );
        starts.push_back(start);
        goals.push_back(goal);
    }


    CBSSearch2d cbssolver( gmap, cp, starts, goals, true );


    std::vector<Solution2d> sols;
    Timer one_clock(true);
    bool res  = cbssolver.search(sols);
    // bool res  = cbssolver.cbssearch_id(sols);
    double spend_time = one_clock.Elapsed();
    if ( res )
    {
        cbssolver.printSolutionsInfo( sols );
        // bool res_validate = cbssolver.revalidateSolutions( sols );
        // std::cout << "Res_Validate: " << res_validate << std::endl;

        std::cout << ANSI_COLOR_GREEN "Find the Solution: Take " <<  spend_time << "ms" ANSI_COLOR_RESET << std::endl;
    }
    else
    {
        std::cout << ANSI_COLOR_GREEN "Can Not Find the Solution: Take " <<  spend_time << "ms" ANSI_COLOR_RESET << std::endl;
        // return 0;
    }


    std_msgs::Header header;
    header.frame_id = std::string("map");


    vec_E<Polyhedron2D> bbox;
    bbox.push_back( gmap.getBoundingBox() );
    decomp_ros_msgs::PolyhedronArray bbox_msg = DecompROS::polyhedron_array_to_ros(bbox);
    bbox_msg.header = header;
    bound_pub.publish(bbox_msg);

    decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(static_poly_obs);
    poly_msg.header = header;
    poly_pub.publish(poly_msg);


    sensor_msgs::PointCloud start_msg;
    start_msg.header = header;
    for( int i = 0; i < uavswarmssize; i++ )
    {
        Vec2f start_pos_t = uavswarmstartgoal.getUavSwarmStartGoal(i).start_pos;
        geometry_msgs::Point32 pt;
        pt.x = start_pos_t(0);
        pt.y = start_pos_t(1);
        pt.z = 0;
        start_msg.points.push_back( pt );
    }
    s_pub.publish(start_msg);


    vec_E<Primitive2d> prs_array;
    for( int i = 0; i < uavswarmssize; i++ )
    {
        // if ( i == 1 )
        //     continue;
        Trajectory2d traj(sols[i].prs);
        auto prs = traj.getPrimitives();
        prs_array.insert(prs_array.end(), prs.begin(), prs.end());
    }
    auto prs_msg = toPrimitiveArrayROSMsg(prs_array);
    prs_msg.header = header;
    prs_pub.publish(prs_msg);


    ros::Time t0 = ros::Time::now();
    double t = 0;
    double dt = 0.1;
    ros::Rate loop_rate(10);
    double max_time = 0;
    for( int i = 0; i < uavswarmssize; i++ )
    {
        double tmp_t = (sols[i].states.size() - 1)*cp.dt;
        if ( tmp_t > max_time )
            max_time = tmp_t;
    }
    while ( ros::ok() )
    {
        t += dt;

        if ( t >= max_time )
            break;

        vec_Vec2f states;
        /* code */
        for( int i = 0; i < uavswarmssize; i++ )
        {
            Trajectory2d traj(sols[i].prs);
            Command2d cmd;
            traj.evaluate(t,cmd);
            states.push_back( cmd.pos );
        }

        for( int i = 0; i < uavswarmssize; i++ )
        {
            for( int j = i+1; j < uavswarmssize; j++ )
            {
                if ( (states[i] - states[j] ).template lpNorm<2>() <= 2 * UAVRadius )
                {
                    std::cout << "BIG ERROR: " << t << std::endl;
                }
                
            }
        }
        auto state_msg = vec_to_cloud(states);
        state_msg.header.frame_id = "map";
        state_msg.header.stamp = t0 + ros::Duration(t);
        state_pub.publish(state_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    // ros::spin();
    return 0;
}

    // ros::Rate loop_rate(100);
    // decimal_t update_t = 0.01;
    // decimal_t time = 0;
    // decimal_t prev_time = -1000;
    // ros::Time t0 = ros::Time::now();
    // double max_time = 0;
    // for( int i = 0; i < uavswarmssize; i++ )
    // {
    //     double tmp_t = (sols[i].states.size() - 1)*cp.dt;
    //     if ( tmp_t > max_time )
    //         max_time = tmp_t;
    // }
    // while (ros::ok()) 
    // {
    //     time += update_t;
    //     // Visualizing current status at 10 Hz
    //     if (time - prev_time >= 0.1) {
    //         vec_Vec2f states;
    //         for( int i = 0; i < uavswarmssize; i++ )
    //         {
    //             Trajectory2d traj(sols[i].prs);
    //             Command2d cmd;
    //             traj.evaluate( time, cmd);
    //             states.push_back( cmd.pos );
    //         }
    //         auto state_msg = vec_to_cloud(states);
    //         state_msg.header.frame_id = "map";
    //         state_msg.header.stamp = t0 + ros::Duration(time);
    //         state_pub.publish(state_msg);

    //         prev_time = time;
    //     }
    //     if ( time >= max_time )
    //         break;
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }
    // ROS_WARN("Total time: %f", (ros::Time::now() - t0).toSec());