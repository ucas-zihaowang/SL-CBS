
#ifndef LOAD_UAVS_H_
#define LOAD_UAVS_H_

#include "common.h"
#include <yaml-cpp/yaml.h>

template <int Dim>
struct Uav
{
    Uav( int id, const Vecf<Dim> &start_pos, const Vecf<Dim> &goal_pos, double start_yaw = 0.0, double goal_yaw = 0.0 )
    {
        this->uav_id = id;
        this->start_pos = start_pos;
        this->goal_pos = goal_pos;

        this->start_yaw = start_yaw;
        this->goal_yaw = goal_yaw;
    }

    int uav_id{0};
    Vecf<Dim> start_pos{Vecf<Dim>::Zero()};
    Vecf<Dim> goal_pos{Vecf<Dim>::Zero()};
    double start_yaw = 0.0;
    double goal_yaw = 0.0;

    void printUavInfo()
    {
        std::cout << "UAV ID: " << uav_id << std::endl;
        std::cout << "Start Position: (" << start_pos.transpose() << ")" << std::endl;
        std::cout << "Start Yaw: " << start_yaw << std::endl; 
        std::cout << "Goal Position: (" << goal_pos.transpose() << ")"  << std::endl;
        std::cout << "Goal Yaw: " << goal_yaw << std::endl;
    }
};

typedef Uav<2> Uav2d;
typedef Uav<3> Uav3d;

template <int Dim>
class UavSwarmStartGoalLoader
{
public:
    explicit UavSwarmStartGoalLoader(const std::string &file )
    {
        try
        {
            YAML::Node config = YAML::LoadFile(file);
            for ( const auto &node : config)
            {
                const auto &id = node["uav_id"];
                const auto &start_pos = node["start_pos"];
                const auto &goal_pos = node["goal_pos"];

                int uav_i_ = id.as<int>();
                Vecf<Dim> start_pos_, goal_pos_;
                for( int i = 0; i < Dim; i++ )
                {
                    start_pos_(i) = start_pos[i].as<double>();
                    goal_pos_(i) = goal_pos[i].as<double>();
                }
                double start_yaw = start_pos[Dim].as<double>();
                double goal_yaw = goal_pos[Dim].as<double>();
                Uav<Dim> one_uav( uav_i_, start_pos_, goal_pos_, start_yaw, goal_yaw );
                uav_swarm.push_back( one_uav );

            }
            uav_num = uav_swarm.size();
            exist_ = true;
        }
        catch( YAML::ParserException &e )
        {
            exist_ = false;
        }
    }

    bool exist() { return exist_; }
    int getUavNum() { return uav_num; }
    Uav<Dim> getUavSwarmStartGoal(int i) { return uav_swarm[i]; }

    void printUavSwarmStartGoalInfo()
    {
        std::cout << ANSI_COLOR_YELLOW "********************UAV Swarm StartGoal Info:********************" ANSI_COLOR_RESET << std::endl;
        for( int i = 0; i < uav_num; i++ )
        {
            uav_swarm[i].printUavInfo();
        }
        std::cout << ANSI_COLOR_YELLOW << "********************End********************" << ANSI_COLOR_RESET << std::endl;
    }

private:
    bool exist_;
    int uav_num;
    std::vector<Uav<Dim>> uav_swarm;
};

typedef UavSwarmStartGoalLoader<2> UavSwarmStartGoalLoader2d;
typedef UavSwarmStartGoalLoader<3> UavSwarmStartGoalLoader3d;

#endif