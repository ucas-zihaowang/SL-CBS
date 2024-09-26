
#include "globalmap.h"



template<int Dim>
GlobalMap<Dim>::GlobalMap( const Veci<Dim> &grid_dim, double res, const Vecf<Dim> &origin, const std::vector<signed char> grid_map, CopterGeo<Dim> &UAVGeoDes_, ObsGeo<Dim> &ObsGeoDes_ ):UAVGeoDes(UAVGeoDes_),ObsGeoDes(ObsGeoDes_)
{
    for(int i = 0; i < Dim; i++ )
        this->dim(i) = std::round( grid_dim(i) * res );
    this->res = res;
    this->origin = origin;

    this->grid_dim = grid_dim;
    this->grid_map = grid_map;
    this->grid_map_size = grid_map.size();

    this->Safe_Dis_Collision = UAVGeoDes.getUAVRadius() * 2;
    this->Safe_Dis_Obstacle = UAVGeoDes.getUAVRadius() + ObsGeoDes.getObsRadius();

}

template<int Dim>
GlobalMap<Dim>::GlobalMap(const Vecf<Dim> &dim, double res, const Vecf<Dim> &origin, CopterGeo<Dim> &UAVGeoDes_, ObsGeo<Dim> &ObsGeoDes_):UAVGeoDes(UAVGeoDes_),ObsGeoDes(ObsGeoDes_)
{
    this->dim = dim;
    this->res = res;
    this->origin = origin;

    this->grid_map_size = 1;
    for (int i = 0; i < Dim; i++)
    {
        this->grid_dim(i) = std::round(  dim(i) / res  );
        this->grid_map_size *= this->grid_dim(i);
    }
    this->grid_map.resize( this->grid_map_size, VAL_FREE );

    this->Safe_Dis_Collision = UAVGeoDes.getUAVRadius() * 2;
    this->Safe_Dis_Obstacle = UAVGeoDes.getUAVRadius() + ObsGeoDes.getObsRadius();
}

template<int Dim>
GlobalMap<Dim>::GlobalMap( const Vecf<Dim> &dim, double res, const Vecf<Dim> &origin, CopterGeo<Dim> &UAVGeoDes_, ObsGeo<Dim> &ObsGeoDes_, std::vector<PolyObsGeo<Dim>> &PolyObsSet ):UAVGeoDes(UAVGeoDes_),ObsGeoDes(ObsGeoDes_)
{
    this->dim = dim;
    this->res = res;
    this->origin = origin;

    this->grid_map_size = 1;
    for (int i = 0; i < Dim; i++)
    {
        this->grid_dim(i) = std::round(  dim(i) / res  );
        this->grid_map_size *= this->grid_dim(i);
    }
    this->grid_map.resize( this->grid_map_size, VAL_FREE );

    this->Safe_Dis_Collision = UAVGeoDes.getUAVRadius() * 2;
    this->Safe_Dis_Obstacle = UAVGeoDes.getUAVRadius();


    this->PolyObsSet = PolyObsSet;
    setBoundingBox( origin, dim );
}

template <int Dim>
template <int U>
typename std::enable_if<U == 2 || U ==3>::type GlobalMap<Dim>::setBoundingBox(const Vecf<Dim> &ori,const Vecf<Dim> &dim)
{
    if constexpr ( U == 2 )
    {
        Polyhedron2D Vs;
        Vs.add(Hyperplane2D(ori + Vec2f(0, dim(1) / 2), -Vec2f::UnitX()));
        Vs.add(Hyperplane2D(ori + Vec2f(dim(0) / 2, 0), -Vec2f::UnitY()));
        Vs.add(Hyperplane2D(ori + dim - Vec2f(0, dim(1) / 2), Vec2f::UnitX()));
        Vs.add(Hyperplane2D(ori + dim - Vec2f(dim(0) / 2, 0), Vec2f::UnitY()));
        bbox = Vs;
    }
    else if constexpr ( U == 3 )
    {
        Polyhedron3D Vs;
        Vs.add( Hyperplane3D(ori + Vec3f(0, dim(1) / 2, dim(2) / 2), -Vec3f::UnitX()));
        Vs.add( Hyperplane3D(ori + Vec3f(dim(0) / 2, 0, dim(2) / 2), -Vec3f::UnitY()));
        Vs.add( Hyperplane3D(ori + Vec3f(dim(0) / 2, dim(2) / 2, 0), -Vec3f::UnitZ()));
        Vs.add(Hyperplane3D(ori + dim - Vec3f(0, dim(1) / 2, dim(2) / 2), Vec3f::UnitX()));
        Vs.add(Hyperplane3D(ori + dim - Vec3f(dim(0) / 2, 0, dim(2) / 2), Vec3f::UnitY()));
        Vs.add(Hyperplane3D(ori + dim - Vec3f(dim(0) / 2, dim(1) / 2, 0), Vec3f::UnitZ()));
        bbox = Vs;
    }
}

template<int Dim>
GlobalMap<Dim>::~GlobalMap()
{

}


template<int Dim>
void GlobalMap<Dim>::printGlobalMapInfo()
{
    std::cout << ANSI_COLOR_YELLOW "********************GlobalMap Info:********************" ANSI_COLOR_RESET << std::endl;
    std::cout << "Origin: (" << origin.transpose() << ")" << std::endl;
    std::cout << "Dim: (" << dim.transpose() << ")" << std::endl;
    std::cout << "Resolution: " << res << std::endl;

    std::cout << "GRID Dim: (" << grid_dim.transpose() << ")" << std::endl;
    std::cout << "GRID Map Size: " << grid_map_size << std::endl; 

    std::cout << "Collision Distance: " << Safe_Dis_Collision << std::endl;
    std::cout << "Obstacle Distance: " << Safe_Dis_Obstacle << std::endl;

    std::cout << ANSI_COLOR_YELLOW "********************End********************" ANSI_COLOR_RESET << std::endl;
}


template<int Dim>
Veci<Dim> GlobalMap<Dim>::toGrid(const Vecf<Dim> &pt)
{
    Veci<Dim> pn;
    for (int i = 0; i < Dim; i++)
        pn(i) = std::round((pt(i) - origin(i)) / res - 0.5);
    return pn;
}


template<int Dim>
Vecf<Dim> GlobalMap<Dim>::toRaw(const Veci<Dim> &pn)
{
    return (pn.template cast<double>() + Vecf<Dim>::Constant(0.5)) * res + origin;
}


template<int Dim>
int GlobalMap<Dim>::getIndex(const Veci<Dim> &pn)
{
    if (Dim == 2)
        return pn(0) + grid_dim(0) * pn(1);
    else if (Dim == 3)
        return pn(0) + grid_dim(0) * pn(1) + grid_dim(0) * grid_dim(1) * pn(2);
    else {}
}
template<int Dim>
Veci<Dim> GlobalMap<Dim>::getIndex(int index)
{
    Veci<Dim> pn;
    if ( Dim == 2 )
    {
        int c1 = index / grid_dim(0);
        int c0 = index % grid_dim(0);
        pn(0) = c0;
        pn(1) = c1;
    }
    else if ( Dim == 3 )
    {
        int c2 = index / ( grid_dim(0) * grid_dim(1) );
        int tmp = index % ( grid_dim(0) * grid_dim(1) );
        int c1 = tmp / grid_dim(0);
        int c0 = tmp % grid_dim(0);
        pn(0) = c0;
        pn(1) = c1;
        pn(2) = c2;
    }
    return pn;
}


template <int Dim>
bool GlobalMap<Dim>::isOutside( const Vecf<Dim> &pt )
{
    double uav_radius = UAVGeoDes.getUAVRadius();
    auto realpt = pt - origin;
    for( int i = 0; i < Dim; i++ )
    {
        // if ( realpt(i) < (0+uav_radius) || realpt(i) > (dim(i)-uav_radius) ) // exp2-3
        if ( realpt(i) < 0 || realpt(i) > dim(i) ) // exp1
            return true;
    }
    return false;
}


template<int Dim>
bool GlobalMap<Dim>::isOutside(const Veci<Dim> &pn)
{
    for (int i = 0; i < Dim; i++)
        if ( pn(i) < 0 || pn(i) >= grid_dim(i) )
            return true;
    return false;
}


template<int Dim>
bool GlobalMap<Dim>::isOccupied(int idx)
{
    return this->grid_map[idx] == VAL_OCC;
}
template<int Dim>
bool GlobalMap<Dim>::isOccupied( const Veci<Dim> &pn )
{
    return this->grid_map[getIndex(pn)] == VAL_OCC;
}


template<int Dim>
vec_Vecf<Dim> GlobalMap<Dim>::getOccupiedSpace()
{
    vec_Vecf<Dim> occ_space;
    Veci<Dim> n;
    if (Dim == 2)
    {
        for (n(0) = 0; n(0) < grid_dim(0); n(0)++)
            for (n(1) = 0; n(1) < grid_dim(1); n(1)++)
                if ( isOccupied(getIndex(n)) )
                    occ_space.push_back(toRaw(n));
    }
    else if (Dim == 3)
    {

        for (n(0) = 0; n(0) < grid_dim(0); n(0)++)
            for (n(1) = 0; n(1) < grid_dim(1); n(1)++)
                for (n(2) = 0; n(2) < grid_dim(2); n(2)++)
                    if ( isOccupied(getIndex(n)) )
                        occ_space.push_back(toRaw(n));
    }
    else {}
    return occ_space;
}


template <int Dim>
vec_Vecf<Dim> GlobalMap<Dim>::rayTrace(const Vecf<Dim> &pt1, const Vecf<Dim> &pt2, int type )
{

    Vecf<Dim> diff = pt2 - pt1;

    double k = 0.5; // 0.8

    int max_diff = (diff / res) .template lpNorm<Eigen::Infinity>() / k ;

    double s = 1.0 / max_diff;
    Vecf<Dim> step = diff * s;

    vec_Vecf<Dim> pns;
    for (int n = 1; n < max_diff; n++)
    {
        Vecf<Dim> pt = pt1 + step * n;

        if ( isOutside(pt) )
            break;
        pns.push_back( pt );
    }
    return pns;
}

template <int Dim>
vec_Veci<Dim> GlobalMap<Dim>::rayTrace(const Vecf<Dim> &pt1, const Vecf<Dim> &pt2)
{

    Vecf<Dim> diff = pt2 - pt1;

    double k = 0.8;

    int max_diff = (diff / res ).template lpNorm<Eigen::Infinity>() / k;
    double s = 1.0 / max_diff;

    Vecf<Dim> step = diff * s;

    vec_Veci<Dim> pns;
    Veci<Dim> prev_pn = Veci<Dim>::Constant(-1);
    for (int n = 1; n < max_diff; n++)
    {
        Vecf<Dim> pt = pt1 + step * n;
        Veci<Dim> new_pn = toGrid(pt);

        if ( isOutside(new_pn) )
            break;

        if ( new_pn != prev_pn )
            pns.push_back(new_pn);
        prev_pn = new_pn;
    }
    return pns;
}

template class GlobalMap<2>;
template class GlobalMap<3>;
