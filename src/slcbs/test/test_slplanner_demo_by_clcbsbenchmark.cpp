
#include <ros/ros.h>
#include "common.h"
#include "copter_common.h"
#include "slplanner.h"

#include "load_parameter.h"
#include "opencv_plot.h"
#include <fstream>

static double Resolution = 0.1;
static double ObsRadius = 2;
static double UAVRadius = 1;
static int UAVID = 0;

#define PLOT_LEVEL 3


class InstanceLoader
{
public:
    explicit InstanceLoader(const std::string &file )
    {
        try
        {
            YAML::Node config = YAML::LoadFile(file);

            const auto &dimensions = config["map"]["dimensions"];
            double dimx = dimensions[0].as<double>();
            double dimy = dimensions[1].as<double>();
            dim = Vec2f( dimx, dimy );

            for (const auto &node : config["map"]["obstacles"])
            {
                obstacles.push_back( Vec2f(node[0].as<double>(), node[1].as<double>()) );
            }
            

            for (const auto &node : config["agents"])
            {
                const auto &start = node["start"];
                const auto &goal = node["goal"];

                starts.emplace_back( Vec3f(start[0].as<double>(), start[1].as<double>(), start[2].as<double>()) );
                goals.emplace_back( Vec3f(goal[0].as<double>(), goal[1].as<double>(), goal[2].as<double>()) );
            }

            agents_num = goals.size();

            exist_ = true;
        }
        catch (YAML::ParserException &e)
        {
            exist_ = false;
        }
    }

    bool exist() { return exist_; }
    Vec2f getDim() { return dim; }
    Vec2f getOrigin() { return origin; }
    std::vector<Vec2f> getObstacles() { return obstacles; }

    int getAgentsNum() { return agents_num; }
    Vec3f getStarts( int i ) { return starts[i]; }
    Vec3f getGoals( int i ) { return goals[i]; }

    void printInstanceInfo()
    {
        std::cout << ANSI_COLOR_YELLOW "********************Instance Info:********************" ANSI_COLOR_RESET << std::endl;
        std::cout << "Origin: (" << origin.transpose() << ")" << std::endl;
        std::cout << "Dim: (" << dim.transpose() << ")" << std::endl;
        std::cout << "Obstacle Num: " << obstacles.size()  << std::endl;
        for( auto obs: this->obstacles )
        {
            std::cout << obs.transpose() << std::endl;
        }
        std::cout << ANSI_COLOR_YELLOW << "********************End********************" << ANSI_COLOR_RESET << std::endl;
    }


private:
    bool exist_;

    Vec2f dim;
    Vec2f origin{Vec2f::Zero()};
    std::vector<Vec2f> obstacles;

    std::vector<Vec3f> starts;
    std::vector<Vec3f> goals;
    int agents_num;
};

// void writeTrajectorysToFile( std::vector<Trajectory2d> &trajs, const std::string &file )
// {
//     std::ofstream out;
//     out = std::ofstream( file );
//     out << "schedule:" << std::endl;
    
//     int sample_num = 4;
//     int trajs_num = trajs.size();
//     std::vector<vec_E<Command2d>> wss;
//     for( int i = 0; i < trajs_num; i++ )
//     {
//         int pr_num = trajs[i].getPrimitives().size();
//         vec_E<Command2d> ws = trajs[i].sample( sample_num * pr_num );
//         wss.push_back(ws);
//     }
//     for( int i = 0; i < trajs_num; i++ )
//     {
//         out << "  agent" << UAVID << ":" << std::endl;
//         for ( const auto &state : wss[i] )
//         {
//             out << "    - x: " << state.pos(0) << std::endl
//                 << "      y: " << state.pos(1) << std::endl
//                 << "      t: " << state.t << std::endl;
//         }
//     }
// }

int main( int argc, char **argv )
{
    std::string name = "test_slplanner_demo_by_clcbsbenchmark_node";
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
    InstanceLoader instance(instancepath);
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
    std::vector<Eigen::VectorXd> obstacles_kd;
    for( auto obs : instance.getObstacles() )
    {
        Eigen::VectorXd one_obs{Vec2f::Zero()};
        one_obs = obs;
        obstacles_kd.push_back( one_obs );
    }
    MyKDTree obs_kd_tree( 2, obstacles_kd, 10 );
    ObsGeo2d ObsGeoDes( instance.getObstacles(), ObsRadius, obs_kd_tree );


    GlobalMap2d gmap( instance.getDim(), Resolution, instance.getOrigin(), UAVGeoDes, ObsGeoDes );
    // gmap.printGlobalMapInfo();


    CopterParameter2d cp( para.getCM(), para.getYawUse(), para.getU(), para.getUyaw(), para.getSampleNum(), para.getDt() );
    cp.setDynamicPara( para.getMaxVel(), para.getMaxAcc(), para.getMaxJrk(), para.getMaxYaw() );
    cp.setTolPara( para.getTolPos(), para.getTolVel(), para.getTolAcc(), para.getTolYaw() );
    cp.setCostWeightPara( para.getWJ(), para.getWT(), para.getWYaw() );
    cp.setPlannerPara( para.getMapType(), para.getHeurType(), para.getMPPenalty(), para.getYawMPPenalty() );
    cp.InitRelatedVariable();
    // cp.printCopterParameterInfo();


    std::vector<CopterState2d> starts, goals;
    for( int i = 0; i < instance.getAgentsNum(); i++ )
    {
        CopterState2d start( cp.cm, cp.yaw_use, instance.getStarts(i).head<2>(),  instance.getStarts(i)(2) );
        CopterState2d goal( cp.cm, cp.yaw_use, instance.getGoals(i).head<2>(),  instance.getGoals(i)(2) );
        // std::cout << "UAV " << i << ": " << std::endl;
        // start.printCopterStateInfo("start state info: ");
        // goal.printCopterStateInfo("goal state info: ");
        starts.push_back(start);
        goals.push_back(goal);
    }


    SLPlanner2d slsolver( gmap, cp, starts[UAVID], goals[UAVID] );


    Solution2d sol;
    Timer one_clock(true);
    bool res  = slsolver.search( sol );
    double spend_time = one_clock.Elapsed();
    if ( res )
    {
        std::cout << ANSI_COLOR_GREEN "Find the Solution: Take " <<  spend_time << "ms" ANSI_COLOR_RESET << std::endl;
//        std::cout << ANSI_COLOR_GREEN "ExpandStateNodeNum: " << slsolver.getExpandStateNodeSet().size() << ANSI_COLOR_RESET << std::endl;
//        std::cout << ANSI_COLOR_GREEN "ExpandStateNum: " << slsolver.getExpandStates().size() << ANSI_COLOR_RESET << std::endl;
//        std::cout << ANSI_COLOR_GREEN "ExpandPrNum: " << slsolver.getExpandPrimitive().size() << ANSI_COLOR_RESET << std::endl;
        sol.printSolutionInfo();
    }
    else
    {
        std::cout << ANSI_COLOR_GREEN "Can Not Find the Solution: Take " <<  spend_time << "ms" ANSI_COLOR_RESET << std::endl;
        return 0;
    }


    // Trajectory2d traj(sol.prs);
    // std::vector<Trajectory2d> trajs;
    // trajs.push_back( traj );
    // writeTrajectorysToFile( trajs, outputpath );


    Trajectory2d traj(sol.prs);
    OpenCVPlot plot( gmap );

    // plot.drawPoints( gmap.getOccupiedSpace(), black ); 
    for( auto obs : instance.getObstacles() )
    {
        plot.drawSolidCircle( obs, black, ObsRadius/Resolution ); 
    }

    plot.drawCircle( starts[UAVID].pos, blue, UAVRadius/Resolution, 2); 
    plot.drawCircle( goals[UAVID].pos, cyan, UAVRadius/Resolution, 2); 

    plot.drawPoints( slsolver.getExpandStateNodeSet(), grey, 3 );


#if PLOT_LEVEL == 1
    plot.drawTraj(traj, red, 2);


    vec_E<vec_Vec2f> trias;
    const auto ws_yaw = traj.sample(20);
    Vec2f d(1.4, 0);
    for (const auto &w: ws_yaw) 
    {
        double yaw = w.yaw;
        double yaw1 = yaw + cp.max_yaw/2;
        double yaw2 = yaw - cp.max_yaw/2;
        Mat2f Ryaw1, Ryaw2;
        Ryaw1 << cos(yaw1), -sin(yaw1), sin(yaw1), cos(yaw1);
        Ryaw2 << cos(yaw2), -sin(yaw2), sin(yaw2), cos(yaw2);
        Vec2f p1 = w.pos;
        Vec2f p2 = w.pos + Ryaw1 * d;
        Vec2f p3 = w.pos + Ryaw2 * d;
        Vec2f p4 = (p2 + p3) / 2;

        vec_Vec2f tria;
        tria.push_back(p1);
        tria.push_back(p2);
        tria.push_back(p3);
        tria.push_back(p1);
        tria.push_back(p4);

        trias.push_back(tria);
    }
    plot.drawLineStrip(trias, blue, 1);

    plot.show( name );
    plot.save( name + ".jpg");

#elif PLOT_LEVEL == 2
    plot.drawDynamicTraj( name, traj, red, 2 );

#elif PLOT_LEVEL == 3
    plot.drawDynamicProcess( name, traj, slsolver.getExpandStates(), slsolver.getExpandPrimitive(), slsolver.getExpandedActions(),  magenta,  1);


    vec_E<vec_Vec2f> trias;
    const auto ws_yaw = traj.sample(20);
    Vec2f d(1.4, 0);
    for (const auto &w: ws_yaw) 
    {
        double yaw = w.yaw;
        double yaw1 = yaw + cp.max_yaw/2;
        double yaw2 = yaw - cp.max_yaw/2;
        Mat2f Ryaw1, Ryaw2;
        Ryaw1 << cos(yaw1), -sin(yaw1), sin(yaw1), cos(yaw1);
        Ryaw2 << cos(yaw2), -sin(yaw2), sin(yaw2), cos(yaw2);
        Vec2f p1 = w.pos;
        Vec2f p2 = w.pos + Ryaw1 * d;
        Vec2f p3 = w.pos + Ryaw2 * d;
        Vec2f p4 = (p2 + p3) / 2;

        vec_Vec2f tria;
        tria.push_back(p1);
        tria.push_back(p2);
        tria.push_back(p3);
        tria.push_back(p1);
        tria.push_back(p4);

        trias.push_back(tria);
    }
    plot.drawLineStrip(trias, blue, 1);

    plot.show( name );
    plot.save( name + ".jpg");
#endif

    return 0;
}