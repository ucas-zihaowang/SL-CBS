
#ifndef GLOBALMAP_H_
#define GLOBALMAP_H_

#include "common.h"
#include "obstaclegeo.h"
#include "coptergeo.h"
#include "polyobstaclegeo.h"

#define VAL_OCC      100
#define VAL_FREE     0
#define VAL_UNKNOWN  -1

template<int Dim>
class GlobalMap
{
public:
    GlobalMap( const Veci<Dim> &grid_dim, double res, const Vecf<Dim> &origin, const std::vector<signed char> grid_map, CopterGeo<Dim> &UAVGeoDes_, ObsGeo<Dim> &ObsGeoDes_ );
    GlobalMap( const Vecf<Dim> &dim, double res, const Vecf<Dim> &origin, CopterGeo<Dim> &UAVGeoDes_, ObsGeo<Dim> &ObsGeoDes_ );
    GlobalMap( const Vecf<Dim> &dim, double res, const Vecf<Dim> &origin, CopterGeo<Dim> &UAVGeoDes_, ObsGeo<Dim> &ObsGeoDes_, std::vector<PolyObsGeo<Dim>> &PolyObsSet);
    ~GlobalMap();
    void printGlobalMapInfo();

    template <int U = Dim>
    typename std::enable_if<U == 2 || U ==3>::type setBoundingBox(const Vecf<Dim> &ori,const Vecf<Dim> &dim);
    
    Polyhedron<Dim> getBoundingBox() { return bbox; };

    Vecf<Dim> getDim() { return dim; }
    double getRes() { return res; }
    Vecf<Dim> getOrigin() { return origin; }

    Veci<Dim> getGridDim() { return grid_dim; }
    int getGridMapSize() { return grid_map_size; }
	std::vector<signed char> getGridMap() {return grid_map; }


    Veci<Dim> toGrid(const Vecf<Dim> &pt);
    Vecf<Dim> toRaw(const Veci<Dim> &pn);

    int getIndex(const Veci<Dim> &pn);
    Veci<Dim> getIndex(int index);

    bool isOutside( const Vecf<Dim> &pt );
    bool isOutside( const Veci<Dim> &pn );

    bool isOccupied( int idx );
    bool isOccupied( const Veci<Dim> &pn );

    vec_Vecf<Dim> getOccupiedSpace();

    vec_Vecf<Dim> rayTrace(const Vecf<Dim> &pt1, const Vecf<Dim> &pt2, int type ); 
    vec_Veci<Dim> rayTrace(const Vecf<Dim> &pt1, const Vecf<Dim> &pt2);

private:

    Vecf<Dim> dim;
    double res;
    Vecf<Dim> origin;


    Veci<Dim> grid_dim;
    int grid_map_size;
    std::vector<signed char> grid_map;

    Polyhedron<Dim> bbox;

public:

    CopterGeo<Dim> &UAVGeoDes;
    ObsGeo<Dim> &ObsGeoDes;

    std::vector<PolyObsGeo<Dim>> PolyObsSet;
    
    double Safe_Dis_Collision;
    double Safe_Dis_Obstacle;
};

typedef GlobalMap<2> GlobalMap2d;
typedef GlobalMap<3> GlobalMap3d;

#endif

