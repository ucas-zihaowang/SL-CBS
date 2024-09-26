
#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_

#include "command.h"
#include "primitive.h"



template <int Dim>
class Trajectory
{
public:

    Trajectory(): total_t(0) {}


    Trajectory(const vec_E<Primitive<Dim>> &prs) : segs(prs)
    {
        taus.push_back(0);
        for ( const auto &pr : prs )
            taus.push_back( pr.getDT() + taus.back() );
        Ts = taus;
        total_t = taus.back();
    }

    bool evaluate(double time, Command<Dim> &p) const
    {
        double tau = time;
        if ( tau < 0) tau = 0;
        if ( tau > total_t ) tau = total_t;

        for (size_t id = 0; id < segs.size(); id++)
        {
            if ( tau >= taus[id] && tau <= taus[id + 1] )
            {
                tau -= taus[id];

                if ( tau > segs[id].getWorkTime() )
                    tau = segs[id].getWorkTime();

                for (int j = 0; j < Dim; j++)
                {
                    const auto pr = segs[id].getPr(j);
                    p.pos(j) = pr.p(tau);
                    p.vel(j) = pr.v(tau);
                    p.acc(j) = pr.a(tau);
                    p.jrk(j) = pr.j(tau);
                    p.yaw = normalize_angle( segs[id].getPrYaw().p(tau) );
                    p.yaw_dot = normalize_angle( segs[id].getPrYaw().v(tau) );
                    p.t = time;
                }
                return true;
            }
        }
        return false;
    }

    vec_E<Command<Dim>> sample(int N) const
    {
        vec_E<Command<Dim>> ps(N + 1);

        double dt = total_t / N;
        
        for (int i = 0; i <= N; i++)
            evaluate( i * dt, ps[i] );
        return ps;
    }


    vec_E<Primitive<Dim>> getPrimitives() const { return segs; }

    double getTotalTime() const { return total_t; }

private:

    vec_E<Primitive<Dim>> segs;

    double total_t;



    std::vector<double> taus;

    std::vector<double> Ts;

};

typedef Trajectory<2> Trajectory2d;
typedef Trajectory<3> Trajectory3d;

#endif
