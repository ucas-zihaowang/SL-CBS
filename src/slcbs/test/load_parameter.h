
#ifndef LOAD_PARAMETER_H_
#define LOAD_PARAMETER_H_

#include "data_struct.h"
#include <yaml-cpp/yaml.h>


class ParameterLoader
{
public:

    explicit ParameterLoader(const std::string &file )
    {
        try
        {
            YAML::Node config = YAML::LoadFile(file);

            this->cm = config[0]["ControlMode"].as<std::string>();
            this->yaw_use = config[1]["YawUse"].as<bool>();

            this->U = config[2]["U"].as<double>();
            this->U_yaw = config[3]["U_YAW"].as<double>();
            this->sample_num = config[4]["SampleNum"].as<int>();
            this->dt = config[5]["Dt"].as<double>();

            this->max_vel = config[6]["MAX_VEL"].as<double>();
            this->max_acc = config[7]["MAX_ACC"].as<double>();
            this->max_jrk = config[8]["MAX_JRK"].as<double>();
            this->max_yaw = config[9]["MAX_YAW"].as<double>();

            this->tol_pos = config[10]["TOL_POS"].as<double>();
            this->tol_vel = config[11]["TOL_VEL"].as<double>();
            this->tol_acc = config[12]["TOL_ACC"].as<double>();
            this->tol_yaw = config[13]["TOL_YAW"].as<double>();

            this->Wj = config[14]["WJ"].as<double>();
            this->Wt = config[15]["WT"].as<double>();
            this->Wyaw = config[16]["WYaw"].as<double>();

            this->map_type = config[17]["Map_Type"].as<int>();
            this->heur_type = config[18]["Heur_Type"].as<int>();
            this->mppenalty = config[19]["MP_PENALTY"].as<int>();
            this->yawmppenalty = config[20]["YAWMP_PENALTY"].as<int>();

            exist_ = true;
        }
        catch (YAML::ParserException &e)
        {
            exist_ = false;
        }
    }

    bool exist() { return exist_; }
    std::string getCM() { return cm; }
    bool getYawUse() { return yaw_use; }

    double getU() { return U; }
    double getUyaw() { return U_yaw; }
    int getSampleNum() { return sample_num; }
    double getDt() { return dt; }

    double getMaxVel() { return max_vel; }
    double getMaxAcc() { return max_acc; }
    double getMaxJrk() { return max_jrk; }
    double getMaxYaw() { return max_yaw; }

    double getTolPos() { return tol_pos; }
    double getTolVel() { return tol_vel; }
    double getTolAcc() { return tol_acc; }
    double getTolYaw() { return tol_yaw; }

    
    double getWJ() { return Wj; }
    double getWT() { return Wt; }
    double getWYaw() { return Wyaw; }

    int getMapType() { return map_type; }
    int getHeurType() { return heur_type; }
    int getMPPenalty() { return mppenalty; }
    int getYawMPPenalty() { return yawmppenalty; }

    void printParameterInfo()
    {
        std::cout << ANSI_COLOR_YELLOW "********************Parameter Info:********************" ANSI_COLOR_RESET << std::endl;
        std::cout << "ControlMode: " << cm  << std::endl;
        std::cout << "YawUse: " << yaw_use << std::endl;

        std::cout << "U: " << U << std::endl;
        std::cout << "U_YAW: " << U_yaw << std::endl;
        std::cout << "SampleNum: " << sample_num << std::endl;
        std::cout << "Dt: " << dt << std::endl;

        std::cout << "MAX_ACC: " << max_vel << std::endl;
        std::cout << "MAX_ACC: " << max_acc << std::endl;
        std::cout << "MAX_JRK: " << max_jrk << std::endl;
        std::cout << "MAX_YAW: " << max_yaw << std::endl;

        std::cout << "TOL_POS: " << tol_pos << std::endl;
        std::cout << "TOL_VEL: " << tol_vel << std::endl;
        std::cout << "TOL_ACC: " << tol_acc << std::endl;
        std::cout << "TOL_YAW: " << tol_yaw << std::endl;

        
        std::cout << "WJ: " << Wj << std::endl;
        std::cout << "WT: " << Wt << std::endl;
        std::cout << "WYaw: " << Wyaw << std::endl;

        std::cout << "Map_Type: " << map_type << std::endl;
        std::cout << "Heur_Type: " << heur_type << std::endl;
        std::cout << "MPPenalty: " << mppenalty << std::endl;
        std::cout << "YawMPPenalty: " << yawmppenalty << std::endl;

        std::cout << ANSI_COLOR_YELLOW << "********************End********************" << ANSI_COLOR_RESET << std::endl;
    }

private:
    bool exist_;

    std::string cm;
    bool yaw_use;

    double U;
    double U_yaw;
    int sample_num;
    double dt;

    double max_vel;
    double max_acc;
    double max_jrk;
    double max_yaw;

    double tol_pos;
    double tol_vel;
    double tol_acc;
    double tol_yaw;

    
    double Wj;
    double Wt;
    double Wyaw;

    int map_type;
    int heur_type;
    int mppenalty;
    int yawmppenalty;
};

#endif