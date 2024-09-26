
#ifndef PRIMITIVE_H_
#define PRIMITIVE_H_

#include "common.h"
#include "copterstate.h"


class Primitive1D
{
public:
    Primitive1D() = default; 
    explicit Primitive1D(Vec6f coeff) : c(std::move(coeff)) {} 


    Primitive1D(double p, double u)
    {
        c << 0, 0, 0, 0, u, p;
    }

    Primitive1D(Vec2f state, double u)
    {
        c << 0, 0, 0, u, state(1), state(0);
    }

    Primitive1D(Vec3f state, double u)
    {
        c << 0, 0, u, state(2), state(1), state(0);
    }

    Primitive1D(Vec4f state, double u)
    {
        c << 0, u, state(3), state(2), state(1), state(0);
    }

    Primitive1D(double p1, double p2, double t)
    {
        c << 0, 0, 0, 0, (p2 - p1) / t, p1;
    }

    Primitive1D(double p1, double v1, double p2, double v2, double t)
    {
        Mat4f A;
        A << 0, 0, 0, 1,
        0, 0, 1, 0,
        power(t, 3) / 6, t *t / 2, t, 1,
        t *t / 2, t, 1, 0;
        Vec4f b;
        b << p1, v1, p2, v2;
        Vec4f cc = A.inverse() * b;
        c << 0, 0, cc(0), cc(1), cc(2), cc(3);
    }

    Primitive1D(double p1, double v1, double a1, double p2, double v2, double a2, double t)
    {
        Mat6f A;
        A << 0, 0, 0, 0, 0, 1,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 1, 0, 0,
        power(t, 5) / 120, power(t, 4) / 24, power(t, 3) / 6, power(t, 2) / 2, t, 1,
        power(t, 4) / 24, power(t, 3) / 6, power(t, 2) / 2, t, 1, 0,
        power(t, 3) / 6, t *t / 2, t, 1, 0, 0;
        Vec6f b;
        b << p1, v1, a1, p2, v2, a2;
        c = A.inverse() * b;
    }


    Vec6f coeff() const
    {
        return c;
    }

    double J(double t, const ControlMode cm) const
    {
        if ( cm == ControlMode::VEL  )
        {
            return c(0) * c(0) / 5184 * power(t, 9) +
                   c(0) * c(1) / 576 * power(t, 8) +
                   (c(1) * c(1) / 252 + c(0) * c(2) / 168) * power(t, 7) +
                   (c(0) * c(3) / 72 + c(1) * c(2) / 36) * power(t, 6) +
                   (c(2) * c(2) / 20 + c(0) * c(4) / 60 + c(1) * c(3) / 15) *
                   power(t, 5) +
                   (c(2) * c(3) / 4 + c(1) * c(4) / 12) * power(t, 4) +
                   (c(3) * c(3) / 3 + c(2) * c(4) / 3) * power(t, 3) +
                   c(3) * c(4) * t * t + c(4) * c(4) * t;
        }
        else if ( cm == ControlMode::ACC  )
        {
            return c(0) * c(0) / 252 * power(t, 7) + c(0) * c(1) / 36 * power(t, 6) +
                   (c(1) * c(1) / 20 + c(0) * c(2) / 15) * power(t, 5) +
                   (c(0) * c(3) / 12 + c(1) * c(2) / 4) * power(t, 4) +
                   (c(2) * c(2) / 3 + c(1) * c(3) / 3) * power(t, 3) +
                   c(2) * c(3) * t * t + c(3) * c(3) * t;
        }
        else if ( cm == ControlMode::JRK )
        {
            return c(0) * c(0) / 20 * power(t, 5) + c(0) * c(1) / 4 * power(t, 4) +
                   (c(1) * c(1) + c(0) * c(2)) / 3 * power(t, 3) +
                   c(1) * c(2) * t * t + c(2) * c(2) * t;
        }
        else if ( cm == ControlMode::SNP )
        {
            return c(0) * c(0) / 3 * power(t, 3) + c(0) * c(1) * t * t +
                   c(1) * c(1) * t;
        }
        else
            return 0;
    }

    double p(double t) const
    {
        return c(0) / 120 * power(t, 5) + c(1) / 24 * power(t, 4) +
               c(2) / 6 * power(t, 3) + c(3) / 2 * t * t + c(4) * t + c(5);
    }

    double v(double t) const
    {
        return c(0) / 24 * power(t, 4) + c(1) / 6 * power(t, 3) + c(2) / 2 * t * t +
               c(3) * t + c(4);
    }

    double a(double t) const
    {
        return c(0) / 6 * power(t, 3) + c(1) / 2 * t * t + c(2) * t + c(3);
    }

    double j(double t) const
    {
        return c(0) / 2 * t * t + c(1) * t + c(2);
    }

    std::vector<double> extrema_v(double t) const
    {
        std::vector<double> roots = solve(0, c(0) / 6, c(1) / 2, c(2), c(3));
        std::vector<double> ts;
        for (const auto &it : roots)
        {
            if (it > 0 && it < t)
                ts.push_back(it);
            else if (it >= t)
                break;
        }
        return ts;
    }

    std::vector<double> extrema_a(double t) const
    {
        std::vector<double> roots = solve(0, 0, c(0) / 2, c(1), c(2));
        std::vector<double> ts;
        for (const auto &it : roots)
        {
            if (it > 0 && it < t)
                ts.push_back(it);
            else if (it >= t)
                break;
        }
        return ts;
    }

    std::vector<double> extrema_j(double t) const
    {
        std::vector<double> ts;
        if (c(0) != 0)
        {
            double t_sol = -c(1) * 2 / c(0);
            if (t_sol > 0 && t_sol < t) ts.push_back(t_sol);
        }
        return ts;
    }
private:

    Vec6f c{Vec6f::Zero()};
};

inline std::size_t hash_value(const Primitive1D &key)
{
    std::size_t val = 0;
    for( int i = 0; i < 6; i++)
    {
        int id = key.coeff()(i) / 0.1;
        boost::hash_combine( val, id );
    }
    return val;
}

template<int Dim>
class Primitive
{
public:
    Primitive() = default; 


    Primitive(const CopterState<Dim> &p, const VecDf &u, double dt) : cm(p.cm), yaw_use(p.yaw_use), entire_time(dt), work_time(dt), isStopPrimitive(false)
    {
        if ( cm == ControlMode::SNP )
        {
            for (int i = 0; i < Dim; i++)
            {
                prs[i] = Primitive1D(Vec4f(p.pos(i), p.vel(i), p.acc(i), p.jrk(i)), u(i));
            }
        }
        else if ( cm == ControlMode::JRK )
        {
            for (int i = 0; i < Dim; i++)
                prs[i] = Primitive1D(Vec3f(p.pos(i), p.vel(i), p.acc(i)), u(i));
        }
        else if ( cm == ControlMode::ACC )
        {
            for (int i = 0; i < Dim; i++)
                prs[i] = Primitive1D(Vec2f(p.pos(i), p.vel(i)), u(i));
        }
        else if ( cm == ControlMode::VEL )
        {
            for (int i = 0; i < Dim; i++)
                prs[i] = Primitive1D( p.pos(i), u(i) );
        }
        else {}

        if ( yaw_use )
        {
            pr_yaw = Primitive1D( p.yaw, u(Dim) ) ;
        }
    }


    Primitive(const CopterState<Dim> &p, const VecDf &u, double dt, bool flag ) : cm(p.cm), yaw_use(p.yaw_use), entire_time(dt), work_time(dt), isStopPrimitive(false)
    {
        
        for (int i = 0; i < Dim; i++)
            prs[i] = Primitive1D( p.pos(i), u(i) );
        if ( yaw_use )
        {
            pr_yaw = Primitive1D( p.yaw, u(Dim) ) ;
        }
    }


    Primitive(const CopterState<Dim> &p, const VecDf &u, double dt, double wt) : cm(p.cm), yaw_use(p.yaw_use), entire_time(dt), work_time(wt), isStopPrimitive(true)
    {
        if ( cm == ControlMode::SNP )
        {
            for (int i = 0; i < Dim; i++)
            {
                prs[i] = Primitive1D(Vec4f(p.pos(i), p.vel(i), p.acc(i), p.jrk(i)), u(i));
            }
        }
        else if ( cm == ControlMode::JRK )
        {
            for (int i = 0; i < Dim; i++)
                prs[i] = Primitive1D(Vec3f(p.pos(i), p.vel(i), p.acc(i)), u(i));
        }
        else if ( cm == ControlMode::ACC )
        {
            for (int i = 0; i < Dim; i++)
                prs[i] = Primitive1D(Vec2f(p.pos(i), p.vel(i)), u(i));
        }
        else if ( cm == ControlMode::VEL )
        {
            for (int i = 0; i < Dim; i++)
                prs[i] = Primitive1D( p.pos(i), u(i) );
        }
        else {}

        if ( yaw_use )
        {
            pr_yaw = Primitive1D( p.yaw, u(Dim) ) ;
        }

    }


    Primitive(const CopterState<Dim> &p1, const CopterState<Dim> &p2, double dt) : cm(p1.cm), yaw_use(p1.yaw_use), entire_time(dt), work_time(dt), isStopPrimitive(false)
    {

        if ( cm == ControlMode::JRK )
        {
            for (int i = 0; i < Dim; i++)
                prs[i] = Primitive1D(p1.pos(i), p1.vel(i), p1.acc(i), p2.pos(i), p2.vel(i), p2.acc(i), dt);
        }

        else if ( cm == ControlMode::ACC )
        {
            for (int i = 0; i < Dim; i++)
                prs[i] = Primitive1D(p1.pos(i), p1.vel(i), p2.pos(i), p2.vel(i), dt);
        }

        else if ( cm == ControlMode::VEL )
        {
            for (int i = 0; i < Dim; i++)
                prs[i] = Primitive1D(p1.pos(i), p2.pos(i), dt);
        }
        else {}

        if ( yaw_use )
        {
            pr_yaw = Primitive1D( p1.yaw, p2.yaw, dt );
        }

    }


    Primitive( const vec_E<Vec6f> &cs, ControlMode cm, bool yaw_use, double dt1, double dt2 ) : cm(cm), yaw_use(yaw_use), entire_time(dt1), work_time(dt2)
    {
        for (int i = 0; i < Dim; i++)
            prs[i] = Primitive1D(cs[i]);
        if ( cs.size() == Dim + 1 )
            pr_yaw = Primitive1D(cs[Dim]);
        isStopPrimitive = work_time < entire_time ? true : false;
    }


    double J()
    {
        double dt = work_time;
        double j = 0;
        for (const auto &pr : prs)
            j += pr.J( dt, cm );
        return j;
    }

    double Jyaw()
    {
        double dt = work_time;
        return pr_yaw.J( dt, ControlMode::VEL );
    }


    CopterState<Dim> evaluate(double t) const
    {
        CopterState<Dim> p( cm, yaw_use );

        if ( isStopPrimitive && t > work_time )
            t = work_time;

        if ( cm == ControlMode::SNP  )
        {
            for (int k = 0; k < Dim; k++)
            {
                p.pos(k) = prs[k].p(t);
                p.vel(k) = prs[k].v(t);
                p.acc(k) = prs[k].a(t);
                p.jrk(k) = prs[k].j(t);
            }
        }
        else if ( cm == ControlMode::JRK  )
        {
            for (int k = 0; k < Dim; k++)
            {
                p.pos(k) = prs[k].p(t);
                p.vel(k) = prs[k].v(t);
                p.acc(k) = prs[k].a(t);
            }
        }
        else if ( cm == ControlMode::ACC )
        {
            for (int k = 0; k < Dim; k++)
            {
                p.pos(k) = prs[k].p(t);
                p.vel(k) = prs[k].v(t);
            }
        }
        else if ( cm == ControlMode::VEL )
        {
            for (int k = 0; k < Dim; k++)
            {
                p.pos(k) = prs[k].p(t);
            }
        }
        if ( yaw_use )
            p.yaw = normalize_angle( pr_yaw.p(t) );
        return p;
    }


    vec_E<CopterState<Dim>> sample(int N) const
    {
        double dt = entire_time;
        vec_E<CopterState<Dim>> ps(N + 1);
        double dt_seg = dt / N;
        for (int i = 0; i <= N; i++)
            ps[i] = evaluate(i * dt_seg);
        return ps;
    }
    

    double max_vel(int k) const
    {
        double dt = work_time;
        std::vector<double> ts = prs[k].extrema_v(dt);
        double max_v = std::max(std::abs(prs[k].v(0)), std::abs(prs[k].v(dt)));
        for ( auto &it : ts )
        {
            if ( it > 0 && it < dt) 
            {
                double v = std::abs( prs[k].v(it) );
                max_v = v > max_v ? v : max_v;
            }
        }
        return max_v;
    }


    double max_acc(int k) const
    {
        double dt = work_time;
        std::vector<double> ts = prs[k].extrema_a(dt);
        double max_a = std::max(std::abs(prs[k].a(0)), std::abs(prs[k].a(dt)));
        for (const auto &it : ts)
        {
            if (it > 0 && it < dt )
            {
                double a = std::abs( prs[k].a(it) );
                max_a = a > max_a ? a : max_a;
            }
        }
        return max_a;
    }


    double max_jrk(int k) const
    {
        double dt = work_time;
        std::vector<double> ts = prs[k].extrema_j(dt);
        double max_j = std::max(std::abs(prs[k].j(0)), std::abs(prs[k].j(dt)));
        for ( const auto &it : ts )
        {
            if ( it > 0 && it < dt )
            {
                double j = std::abs( prs[k].j(it) );
                max_j = j > max_j ? j : max_j;
            }
        }
        return max_j;
    }


    ControlMode getControl() const { return cm; }

    bool getYawControl() const { return yaw_use; }

    Primitive1D getPr(int k) const { return prs[k]; }

    Primitive1D getPrYaw() const { return pr_yaw; }

    double getDT() const { return entire_time;}

    double getWorkTime() const { return work_time; }


    bool getIsStopPr() const { return isStopPrimitive; }


    void printMotionPrimitiveInfo() const
    {
        std::cout << ANSI_COLOR_YELLOW << "********************Motion Primitive Info:********************" << ANSI_COLOR_RESET << std::endl;
        for(int i = 0; i < Dim; i++ )
        {
            std::cout << prs[i].coeff().transpose() << std::endl;
        }
        if ( yaw_use )
        {
            std::cout << pr_yaw.coeff().transpose() << std::endl;
        }
        std::cout << "IsStopPrimitive: " << isStopPrimitive << std::endl;
        std::cout << "Entire Time: " << entire_time << std::endl;
        std::cout << "Work Time: " << work_time << std::endl;
        std::cout << ANSI_COLOR_YELLOW << "********************End********************" << ANSI_COLOR_RESET << std::endl;
    }

private:

    ControlMode cm;
    bool yaw_use;


    std::array<Primitive1D, Dim> prs;

    Primitive1D pr_yaw;


    double entire_time{};
    double work_time{};


    bool isStopPrimitive{false};
};


template <int Dim>
std::size_t hash_value(const Primitive<Dim> &key)
{
    std::size_t val = 0;
    boost::hash_combine(val, key.getDT() );
    boost::hash_combine(val, key.getWorkTime() );
    int id = key.getIsStopPr()? 1:0;
    boost::hash_combine(val, id);

    if ( key.getYawControl() )
        boost::hash_combine( val, key.getPrYaw() );
    
    for( int i = 0; i < Dim; i++ )
        boost::hash_combine( val, key.getPr(i) );

    return val;
}


template<int Dim>
bool operator==(const Primitive<Dim> &l, const Primitive<Dim> &r)
{
    return hash_value(l) == hash_value(r);
}


template<int Dim>
bool operator!=(const Primitive<Dim> &l, const Primitive<Dim> &r)
{
    return hash_value(l) != hash_value(r);
}


template<int Dim>
bool validate_cm(const Primitive<Dim> &pr, ControlMode cm, double mv, double ma, double mj )
{
    bool res = true;
    if ( cm == ControlMode::ACC || cm == ControlMode::JRK || cm == ControlMode::SNP )
    {
        if ( mv <= 0 )
            res = true;
        else
        {
            for( int i = 0; i < Dim; i++ )
            {
                if ( pr.max_vel(i) > mv )
                {
                    return false;
                }
            }
        }
    }
    
    if ( cm == ControlMode::JRK || cm == ControlMode::SNP )
    {
        if ( ma <= 0 )
            res = true;
        else
        {
            for( int i = 0; i < Dim; i++ )
            {
                if ( pr.max_acc(i) > ma )
                {
                    return false;
                }
            }
        }
    }

    if ( cm == ControlMode::SNP )
    {
        if ( mj <= 0 )
            res = true;
        else
        {
            for( int i = 0; i < Dim; i++ )
            {
                if ( pr.max_jrk(i) > mj )
                {
                    return false;
                }
            }
        }
    }
    return res;
}

template<int Dim>
bool validate_yaw(const Primitive<Dim> &pr, double my)
{

    if ( my <= 0 )
        return true;


    vec_E<CopterState<Dim>> ws(2);
    ws[0] = pr.evaluate( 0 );
    ws[1] = pr.evaluate( pr.getDT() );
    
    for (const auto &w : ws)
    {

        const auto v = w.vel.template topRows<2>();
        if (v(0) != 0 || v(1) != 0)    // if v is not zero
        {
            double d = v.normalized().dot(Vec2f(cos(w.yaw), sin(w.yaw)));
            if ( d < cos(my) )
                return false;
        }
    }
    return true;
}

typedef Primitive<2> Primitive2d;
typedef Primitive<3> Primitive3d;

#endif
