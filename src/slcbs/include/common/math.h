
#ifndef MATH_H_
#define MATH_H_

#include "data_struct.h"


inline double normalize_angle(double angle)
{
    while (angle > M_PI)
        angle -= 2.0 * M_PI;
    while (angle < -M_PI)
        angle += 2.0 * M_PI;
    return angle;
}


inline double power(double t, int n)
{
    double tn = 1;
    while (n > 0)
    {
        tn *= t;
        n--;
    }
    return tn;
}


inline std::vector<double> quad(double b, double c, double d)
{
    std::vector<double> dts;
    double p = c * c - 4 * b * d;
    if (p < 0)
        return dts;
    else
    {
        dts.push_back((-c - sqrt(p)) / (2 * b));
        dts.push_back((-c + sqrt(p)) / (2 * b));
        return dts;
    }
}

inline std::vector<double> cubic(double a, double b, double c, double d)
{
    std::vector<double> dts;

    double a2 = b / a;
    double a1 = c / a;
    double a0 = d / a;

    double Q = (3 * a1 - a2 * a2) / 9;
    double R = (9 * a1 * a2 - 27 * a0 - 2 * a2 * a2 * a2) / 54;
    double D = Q * Q * Q + R * R;

    if (D > 0)
    {
        double S = std::cbrt(R + sqrt(D));
        double T = std::cbrt(R - sqrt(D));
        // printf("S: %f, T: %f\n", S, T);
        dts.push_back(-a2 / 3 + (S + T));
        return dts;
    }
    else if (D == 0)
    {
        double S = std::cbrt(R);
        dts.push_back(-a2 / 3 + S + S);
        dts.push_back(-a2 / 3 - S);
        return dts;
    }
    else
    {
        double theta = acos(R / sqrt(-Q * Q * Q));
        dts.push_back(2 * sqrt(-Q) * cos(theta / 3) - a2 / 3);
        dts.push_back(2 * sqrt(-Q) * cos((theta + 2 * M_PI) / 3) - a2 / 3);
        dts.push_back(2 * sqrt(-Q) * cos((theta + 4 * M_PI) / 3) - a2 / 3);
        return dts;
    }
}

inline std::vector<double> quartic(double a, double b, double c, double d, double e)
{
    std::vector<double> dts;

    double a3 = b / a;
    double a2 = c / a;
    double a1 = d / a;
    double a0 = e / a;

    std::vector<double> ys = cubic(1, -a2, a1 * a3 - 4 * a0, 4 * a2 * a0 - a1 * a1 - a3 * a3 * a0);
    double y1 = ys.front();
    double r = a3 * a3 / 4 - a2 + y1;
    if (r < 0) return dts;

    double R = sqrt(r);
    double D, E;
    if (R != 0)
    {
        D = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 +
                 0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
        E = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 -
                 0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
    }
    else
    {
        D = sqrt(0.75 * a3 * a3 - 2 * a2 + 2 * sqrt(y1 * y1 - 4 * a0));
        E = sqrt(0.75 * a3 * a3 - 2 * a2 - 2 * sqrt(y1 * y1 - 4 * a0));
    }

    if (!std::isnan(D))
    {
        dts.push_back(-a3 / 4 + R / 2 + D / 2);
        dts.push_back(-a3 / 4 + R / 2 - D / 2);
    }
    if (!std::isnan(E))
    {
        dts.push_back(-a3 / 4 - R / 2 + E / 2);
        dts.push_back(-a3 / 4 - R / 2 - E / 2);
    }

    return dts;
}

inline std::vector<double> solve(double a, double b, double c, double d, double e)
{
    std::vector<double> ts;
    if (a != 0)
        return quartic(a, b, c, d, e);
    else if (b != 0)
        return cubic(b, c, d, e);
    else if (c != 0)
        return quad(c, d, e);
    else if (d != 0)
    {
        ts.push_back(-e / d);
        return ts;
    }
    else
        return ts;
}

inline std::vector<double> solve(double a, double b, double c, double d, double e, double f)
{
    std::vector<double> ts;
    if (a == 0)
        return solve(b, c, d, e, f);
    else
    {
        Eigen::VectorXd coeff(6);
        coeff << f, e, d, c, b, a;
        Eigen::PolynomialSolver<double, 5> solver;
        solver.compute(coeff);

        const Eigen::PolynomialSolver<double, 5>::RootsType &r = solver.roots();

        for (int i = 0; i < r.rows(); ++i)
        {
            if (r[i].imag() == 0)
            {
                ts.push_back(r[i].real());
            }
        }
        return ts;
    }
}

inline std::vector<double> solve(double a, double b, double c, double d, double e, double f, double g)
{
    std::vector<double> ts;
    if (a == 0 && b == 0)
        return solve(c, d, e, f, g);
    else
    {
        Eigen::VectorXd coeff(7);
        coeff << g, f, e, d, c, b, a;
        Eigen::PolynomialSolver<double, 6> solver;
        solver.compute(coeff);

        const Eigen::PolynomialSolver<double, 6>::RootsType &r = solver.roots();

        for (int i = 0; i < r.rows(); ++i)
        {
            if ( r[i].imag() == 0 )
            {
                ts.push_back(r[i].real());
            }
        }
        return ts;
    }
}

#endif
