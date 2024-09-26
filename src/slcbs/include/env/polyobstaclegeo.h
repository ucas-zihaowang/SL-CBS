
#ifndef POLYOBSTACLEGEO_H_
#define POLYOBSTACLEGEO_H_

#include "common.h"
#include "decomp_geometry/polyhedron.h"

template <int Dim>
class PolyObsGeo
{
public:
    PolyObsGeo(){}
    PolyObsGeo( const Polyhedron<Dim> &polyobtacle, const Vecf<Dim> &p )
    {
        this->polyobtacle = polyobtacle;
        this->p = p;
    }

    bool inside ( const Vecf<Dim> &pt ) const { return polyobtacle.inside(pt-p); }

    Polyhedron<Dim> getPolyObstacle() { return polyobtacle; }
    Vecf<Dim> getP() { return p; }
    

private:

    Polyhedron<Dim> polyobtacle;

    Vecf<Dim> p;

};

typedef PolyObsGeo<2> PolyObsGeo2d;
typedef PolyObsGeo<3> PolyObsGeo3d;

#endif
