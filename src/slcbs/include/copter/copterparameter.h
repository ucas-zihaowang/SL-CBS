
#ifndef COPTERPARAMETER_H_
#define COPTERPARAMETER_H_

#include "common.h"


template <int Dim>
struct CopterParameter
{

    CopterParameter( std::string cmstr, bool yaw_use, double U, double U_yaw, int sample_num, double dt )
    {
        this->cm = strtocm[cmstr];
        this->yaw_use = yaw_use;

        this->u = U;
        this->u_yaw = U_yaw;
        this->sample_num = sample_num;
        this->dt = dt;
    }


    void setDynamicPara( double max_vel, double max_acc, double max_jrk, double max_yaw )
    {
        this->max_vel = max_vel;
        this->max_acc = max_acc;
        this->max_jrk = max_jrk;
        this->max_yaw = max_yaw;
    }


    void setTolPara( double tol_pos, double tol_vel, double tol_acc, double tol_yaw )
    {
        this->tol_pos = tol_pos;
        this->tol_vel = tol_vel;
        this->tol_acc = tol_acc;
        this->tol_yaw = tol_yaw;
    }


    void setCostWeightPara( double Wj, double Wt, double Wyaw )
    {
        this->Wj = Wj;
        this->Wt = Wt;
        this->Wyaw = Wyaw;
    }


    void setPlannerPara( int map_type, int heur_type, int motion_primitive_penalty, int yaw_motion_primitive_penalty)
    {
        this->map_type = map_type;
        this->heur_type = heur_type;
        this->motion_primitive_penalty = motion_primitive_penalty;
        this->yaw_motion_primitive_penalty = yaw_motion_primitive_penalty;
    }


    ControlMode cm{ControlMode::NONE};
    bool yaw_use{false};
    double u;
    double u_yaw{0};
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

    int map_type = 1;
    int heur_type = 0;
    int motion_primitive_penalty = 0;
    int yaw_motion_primitive_penalty = 0;

    vec_E<VecDf> All_U;
    int all_u_num;
    VecDf U_Zero;

    vec_E<VecDi> move_directions;
    vec_E<VecDi> move_directions_abs;
    int move_dir_num;

    void InitRelatedVariable()
    {

        if ( this->yaw_use )
        {
            double du = this->u / this->sample_num;
            double dyaw = this->u_yaw / this->sample_num;
            if ( Dim == 2 )
            {
                U_Zero = Vec3f::Zero();
                for (double dx = -u; dx <= u; dx += du)
                    for (double dy = -u; dy <= u; dy += du)
                        for ( double dz = -u_yaw; dz <= u_yaw; dz += dyaw)
                            this->All_U.push_back(Vec3f(dx, dy, dz));
            }
            else if ( Dim == 3 )
            {
                U_Zero = Vec4f::Zero();
                for (double dx = -u; dx <= u; dx += du)
                    for (double dy = -u; dy <= u; dy += du)
                        for (double dz = -u; dz <= u; dz += du)
                            for ( double dz1 = -u_yaw; dz1 <= u_yaw; dz1 += dyaw)
                                this->All_U.push_back(Vec4f(dx, dy, dz, dz1));

            }
        }
        else
        {
            double du = this->u / this->sample_num;
            if ( Dim == 2 )
            {
                U_Zero = Vec2f::Zero();
                for (double dx = -u; dx <= u; dx += du)
                    for (double dy = -u; dy <= u; dy += du)
                        this->All_U.push_back(Vec2f(dx, dy));
            }
            else if ( Dim == 3 )
            {
                U_Zero = Vec3f::Zero();
                for (double dx = -u; dx <= u; dx += du)
                    for (double dy = -u; dy <= u; dy += du)
                        for (double dz = -u; dz <= u; dz += du)
                            this->All_U.push_back(Vec3f(dx, dy, dz));
            }
        }
        all_u_num = All_U.size();


        if ( Dim == 2 )
        {
            for( int i = -1; i <= 1; i++ )
            {
                for( int j = -1; j <= 1; j++ )
                {
                    if ( i == 0 && j == 0 )
                        continue;
                    this->move_directions.push_back( Vec2i(i,j) );
                    this->move_directions_abs.push_back( Vec2i(i,j).array().abs() );
                }
            }
        }
        else if ( Dim == 3 )
        {
            for( int i = -1; i <= 1; i++ )
            {
                for( int j = -1; j <= 1; j++ )
                {
                    for( int k = -1; k <= 1; k++ )
                    {
                        if ( i == 0 && j == 0 && k == 0 )
                            continue;
                        this->move_directions.push_back( Vec3i(i,j,k) );
                        this->move_directions_abs.push_back( Vec3i(i,j,k).array().abs() );
                    }
                }
            }
        }
        move_dir_num = move_directions.size();
    }

    void printCopterParameterInfo()
    {
        std::cout << ANSI_COLOR_YELLOW << "********************Copter Parameter Info:********************" << ANSI_COLOR_RESET << std::endl;
        std::cout << "ControlMode: " << cmtostr[cm] << std::endl;
        std::cout << "YawUse: " << yaw_use << std::endl;
        std::cout << "U: " << u << std::endl;
        if ( yaw_use )
            std::cout << "U_YAW: " << u_yaw << std::endl;
        std::cout << "SampleNum: " << sample_num << std::endl;
        std::cout << "Dt: " << dt << std::endl;

        switch( cm )
        {
            case ControlMode::VEL:
                break;
            case ControlMode::ACC:
                std::cout << "MAX VEL: " << max_vel << std::endl;
                break;
            case ControlMode::JRK:
                std::cout << "MAX VEL: " << max_vel << std::endl;
                std::cout << "MAX ACC: " << max_acc << std::endl;
                break;
            case ControlMode::SNP:
                std::cout << "MAX VEL: " << max_vel << std::endl;
                std::cout << "MAX ACC: " << max_acc << std::endl;
                std::cout << "MAX JRK: " << max_jrk << std::endl;
                break;
            default:
                break;
        }
        if ( yaw_use )
            std::cout << "MAX YAW: " << max_yaw << std::endl;

        std::cout << "TOL POS: " << tol_pos << std::endl;
        if ( tol_vel > 0 )
            std::cout << "TOL VEL: " << tol_vel << std::endl;
        if ( tol_acc > 0 )
            std::cout << "TOL ACC: " << tol_acc << std::endl;
        if ( yaw_use )
            std::cout << "TOL YAW: " << tol_yaw << std::endl;
        
        std::cout << "Energy Cost Weight: " << Wj << std::endl;
        std::cout << "Time Cost Weight: " << Wt << std::endl;
        if ( yaw_use )
            std::cout << "Yaw Effort Weight: " << Wyaw << std::endl;

        
        std::cout << "Map Type: " << map_type << std::endl;
        std::cout << "Heuristic Type: " << heur_type << std::endl;
        std::cout << "Motion Primitive Penalty: " << motion_primitive_penalty << std::endl;
        std::cout << "Yaw Motion Primitive Penalty: " << yaw_motion_primitive_penalty << std::endl;
        

        std::cout << ANSI_COLOR_YELLOW << "********************End********************" << ANSI_COLOR_RESET << std::endl;
    }
};

typedef CopterParameter<2> CopterParameter2d;
typedef CopterParameter<3> CopterParameter3d;

#endif
