
#ifndef COPTERSTATE_H_
#define COPTERSTATE_H_

#include "common.h"


template<int Dim>
struct CopterState
{
    CopterState(){}


    CopterState( ControlMode cm, bool yaw_use )
    {
        this->cm = cm;
        this->yaw_use = yaw_use;
    }


    CopterState( ControlMode cm, bool yaw_use, Vecf<Dim> pos, double yaw = 0 )
    {
        this->cm = cm;
        this->yaw_use = yaw_use;
        this->pos = pos;
        this->vel = Vecf<Dim>::Zero();
        this->acc = Vecf<Dim>::Zero();
        this->jrk = Vecf<Dim>::Zero();
        this->yaw = yaw;
        this->time_t = 0;
        this->timestamp = 0;
    }


    CopterState( const CopterState& other )
    {
        this->cm = other.cm;
        this->yaw_use = other.yaw_use;
        this->pos = other.pos;
        this->vel = other.vel;
        this->acc = other.acc;
        this->jrk = other.jrk;
        this->yaw = other.yaw;
        this->time_t = other.time_t;
        this->timestamp = other.timestamp;
    }


    ControlMode cm{ControlMode::NONE};
    bool yaw_use{false};

    Vecf<Dim> pos{Vecf<Dim>::Zero()};
    Vecf<Dim> vel{Vecf<Dim>::Zero()};
    Vecf<Dim> acc{Vecf<Dim>::Zero()};
    Vecf<Dim> jrk{Vecf<Dim>::Zero()};
    double yaw{0};
    double time_t = 0;
    int timestamp = 0;

    void printCopterStateInfo(const std::string &str = "")
    {
        if (!str.empty())
            std::cout << ANSI_COLOR_YELLOW << "********************" << str << "********************" << ANSI_COLOR_RESET << std::endl;

        switch (cm)
        {
            case ControlMode::VEL:
                std::cout << "pos: ( " << pos.transpose() << " )" << std::endl;
                break;
            case ControlMode::ACC:
                std::cout << "pos: ( " << pos.transpose() << " )" << std::endl;
                std::cout << "vel: ( " << vel.transpose() << " )" << std::endl;
                break;
            case ControlMode::JRK:
                std::cout << "pos: ( " << pos.transpose() << " )" << std::endl;
                std::cout << "vel: ( " << vel.transpose() << " )" << std::endl;
                std::cout << "acc: ( " << acc.transpose() << " )" << std::endl;
                break;
            case ControlMode::SNP:
                std::cout << "pos: ( " << pos.transpose() << " )" << std::endl;
                std::cout << "vel: ( " << vel.transpose() << " )" << std::endl;
                std::cout << "acc: ( " << acc.transpose() << " )" << std::endl;
                std::cout << "jrk: ( " << jrk.transpose() << " )" << std::endl;
                break;
        }
        if ( yaw_use )
            std::cout << "yaw: " << yaw << std::endl;
        std::cout << "time_t: " << time_t << std::endl;
        std::cout << "timestamp: " << timestamp << std::endl;
        std::cout << ANSI_COLOR_YELLOW << "********************End********************" << ANSI_COLOR_RESET << std::endl;
    }
};


template <int Dim>
std::size_t hash_value(const CopterState<Dim> &key)
{
    std::size_t val = 0;
    int id;
    for (int i = 0; i < Dim; i++)
    {
        if (key.cm == ControlMode::VEL )
        {
            id = std::round(key.pos(i) / 0.01);
            boost::hash_combine(val, id);
        }
        else if (key.cm == ControlMode::ACC )
        {
            id = std::round(key.pos(i) / 0.01);
            boost::hash_combine(val, id);
            id = std::round(key.vel(i) / 0.1);
            boost::hash_combine(val, id);
        }
        else if (key.cm == ControlMode::JRK )
        {
            id = std::round(key.pos(i) / 0.01);
            boost::hash_combine(val, id);
            id = std::round(key.vel(i) / 0.1);
            boost::hash_combine(val, id);
            id = std::round(key.acc(i) / 0.1);
            boost::hash_combine(val, id);
        }
        else if (key.cm == ControlMode::SNP  )
        {
            id = std::round(key.pos(i) / 0.01);
            boost::hash_combine(val, id);
            id = std::round(key.vel(i) / 0.1);
            boost::hash_combine(val, id);
            id = std::round(key.acc(i) / 0.1);
            boost::hash_combine(val, id);
            id = std::round(key.jrk(i) / 0.1);
            boost::hash_combine(val, id);
        }

    }
    if ( key.yaw_use )
    {
        id = std::round(key.yaw / 0.1);
        boost::hash_combine(val, id);
    }

    id = std::round( key.time_t / 0.1);
    boost::hash_combine(val, id);
    id = std::round( key.timestamp / 0.1);
    boost::hash_combine(val, id);

    return val;
}

template<int Dim>
bool operator==(const CopterState<Dim> &l, const CopterState<Dim> &r)
{
    return hash_value(l) == hash_value(r);
}

template<int Dim>
bool operator!=(const CopterState<Dim> &l, const CopterState<Dim> &r)
{
    return hash_value(l) != hash_value(r);
}

typedef CopterState<2> CopterState2d;
typedef CopterState<3> CopterState3d;

#endif
