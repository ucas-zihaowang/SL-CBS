
#ifndef ROOTFINDER_H
#define ROOTFINDER_H

#include "data_struct.h"

namespace RootFinderParam
{
constexpr size_t highestOrder = 64;
}

namespace RootFinderPriv
{

/*
** Modulus of u(x)/v(x)
** The leading coefficient of v, i.e., v[0], must be 1.0 or -1.0
** The length of u, v, and r are lu, lv, and lu, respectively
*/ 
inline int polyMod(double *u, double *v, double *r, int lu, int lv)
{
    int orderu = lu - 1;
    int orderv = lv - 1;

    memcpy(r, u, lu * sizeof(double));

    if (v[0] < 0.0)
    {
        // for (int i = orderv + 1; i <= orderu; i += 2)
        // {
        //     r[i] = -r[i];
        // }
        // for (int i = 0; i <= orderu - orderv; i++)
        // {
        //     for (int j = i + 1; j <= orderv + i; j++)
        //     {
        //         r[j] = -r[j] - r[i] * v[j - i];
        //     }
        // }

        for (int i = 0; i <= orderu - orderv; i++)
        {
            for (int j = i + 1; j <= orderv + i; j++)
            {
                r[j] = r[j] + r[i] * v[j - i];
            }
        }
    }
    else
    {
        for (int i = 0; i <= orderu - orderv; i++)
        {
            for (int j = i + 1; j <= orderv + i; j++)
            {
                r[j] = r[j] - r[i] * v[j - i];
            }
        }
    }

    int k = orderv - 1;
    while (k >= 0 && fabs(r[orderu - k]) < DBL_EPSILON)
    {
        r[orderu - k] = 0.0;
        k--;
    }

    return (k <= 0) ? 1 : (k + 1);
}

/*
** Evaluate the polynomial p(x), which has len coefficients
** Note: Horner scheme should not be employed here !!!
** Horner scheme has bad numerical stability despite of its efficiency.
** These errors are particularly troublesome for root-finding algorithms.
** When the polynomial is evaluated near a zero, catastrophic
** cancellation (subtracting two nearby numbers) is guaranteed to occur.
** Therefore, Horner scheme may slow down some root-finding algorithms.
*/ 
inline double polyEval(double *p, int len, double x)
{
    double retVal = 0.0;

    if (len > 0)
    {
        if (fabs(x) < DBL_EPSILON)
        {
            retVal = p[len - 1];
        }
        else if (x == 1.0)
        {
            for (int i = len - 1; i >= 0; i--)
            {
                retVal += p[i];
            }
        }
        else
        {
            double xn = 1.0;

            for (int i = len - 1; i >= 0; i--)
            {
                retVal += p[i] * xn;
                xn *= x;
            }
        }
    }

    return retVal;
}
}

namespace RootFinder
{
/*
** Calculate self-convolution of coef(x)
*/ 
inline Eigen::VectorXd polySqr(const Eigen::VectorXd &coef)
{
    int coefSize = coef.size();
    int resultSize = coefSize * 2 - 1;
    int lbound, rbound;
    Eigen::VectorXd result(resultSize);
    double temp;
    for (int i = 0; i < resultSize; i++)
    {
        temp = 0;
        lbound = i - coefSize + 1;
        lbound = lbound > 0 ? lbound : 0;
        rbound = coefSize < (i + 1) ? coefSize : (i + 1);
        rbound += lbound;
        if (rbound & 1) //faster than rbound % 2 == 1
        {
            rbound >>= 1; //faster than rbound /= 2
            temp += coef(rbound) * coef(rbound);
        }
        else
        {
            rbound >>= 1; //faster than rbound /= 2
        }

        for (int j = lbound; j < rbound; j++)
        {
            temp += 2.0 * coef(j) * coef(i - j);
        }
        result(i) = temp;
    }

    return result;
}
/*
** Count the number of distinct roots of coeffs(x) inside (l, r), leveraging Sturm theory
** Boundary values, i.e., coeffs(l) and coeffs(r), must be nonzero
*/ 
inline int countRoots(const Eigen::VectorXd &coeffs, double l, double r)
{
    int nRoots = 0;

    int originalSize = coeffs.size();
    int valid = originalSize;
    for (int i = 0; i < originalSize; i++)
    {
        if (fabs(coeffs(i)) < DBL_EPSILON)
        {
            valid--;
        }
        else
        {
            break;
        }
    }
    if (valid > 0 && fabs(coeffs(originalSize - 1)) > DBL_EPSILON)
    {
        Eigen::VectorXd monicCoeffs(valid);
        monicCoeffs << 1.0, coeffs.segment(originalSize - valid + 1, valid - 1) / coeffs(originalSize - valid);

        // Build the Sturm sequence
        int len = monicCoeffs.size();
        int order = len - 1;
        double sturmSeqs[(RootFinderParam::highestOrder + 1) * (RootFinderParam::highestOrder + 1)];
        int szSeq[RootFinderParam::highestOrder + 1] = {0}; // Explicit ini as zero (gcc may neglect this in -O3)
        int num = 0;

        for (int i = 0; i < len; i++)
        {
            sturmSeqs[i] = monicCoeffs(i);
            sturmSeqs[i + 1 + len] = (order - i) * sturmSeqs[i] / order;
        }
        szSeq[0] = len;
        szSeq[1] = len - 1;
        num += 2;

        bool remainderConstant = false;
        int idx = 0;
        while (!remainderConstant)
        {
            szSeq[idx + 2] = RootFinderPriv::polyMod(&(sturmSeqs[(idx + 1) * len - szSeq[idx]]),
                                                     &(sturmSeqs[(idx + 2) * len - szSeq[idx + 1]]),
                                                     &(sturmSeqs[(idx + 3) * len - szSeq[idx]]),
                                                     szSeq[idx], szSeq[idx + 1]);
            remainderConstant = szSeq[idx + 2] == 1;
            // if ( !remainderConstant )
            // {
            for (int i = 1; i < szSeq[idx + 2]; i++)
            {
                sturmSeqs[(idx + 3) * len - szSeq[idx + 2] + i] /= -fabs(sturmSeqs[(idx + 3) * len - szSeq[idx + 2]]);
            }
            sturmSeqs[(idx + 3) * len - szSeq[idx + 2]] /= -fabs(sturmSeqs[(idx + 3) * len - szSeq[idx + 2]]);

            // }
            num++;
            idx++;
        }
        // Count numbers of sign variations at two boundaries
        double yl, lastyl, yr, lastyr;
        lastyl = RootFinderPriv::polyEval(&(sturmSeqs[len - szSeq[0]]), szSeq[0], l);
        lastyr = RootFinderPriv::polyEval(&(sturmSeqs[len - szSeq[0]]), szSeq[0], r);
        // std::cout << "yl: " << lastyl << std::endl;
        // std::cout << "yr: " << lastyr << std::endl;
        for (int i = 1; i < num; i++)
        {
            yl = RootFinderPriv::polyEval(&(sturmSeqs[(i + 1) * len - szSeq[i]]), szSeq[i], l);
            yr = RootFinderPriv::polyEval(&(sturmSeqs[(i + 1) * len - szSeq[i]]), szSeq[i], r);
            if (lastyl == 0.0 || lastyl * yl < 0.0)
            {
                ++nRoots;
            }
            if (lastyr == 0.0 || lastyr * yr < 0.0)
            {
                --nRoots;
            }
            lastyl = yl;
            lastyr = yr;
            // std::cout << "yl: " << lastyl << std::endl;
            // std::cout << "yr: " << lastyr << std::endl;
        }
    }
    return nRoots;
}
}

#endif