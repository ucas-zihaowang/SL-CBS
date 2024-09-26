
#ifndef COPTERGEO_H_
#define COPTERGEO_H_

#include "common.h"

template <int Dim>
class CopterGeo
{
public:
    CopterGeo( double radius, double shortHeight = 0, double longHeight = 0 )
    {
        this->UAVRadius = radius;
        if ( Dim == 3 )
        {
            this->shortHeight = shortHeight;
            this->longHeight = longHeight;
        }

    }
    double getUAVRadius() { return UAVRadius; }

private:
    double UAVRadius;
    double shortHeight;
    double longHeight;
};

typedef CopterGeo<2> CopterGeo2d;
typedef CopterGeo<3> CopterGeo3d;

#endif
