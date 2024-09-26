
#ifndef OBSTACLEGEO_H_
#define OBSTACLEGEO_H_

#include "common.h"
using my_vector_of_vectors_t = std::vector<Eigen::VectorXd>;
typedef KDTreeVectorOfVectorsAdaptor< my_vector_of_vectors_t, double, -1, metric_L2_Simple > MyKDTree;

template <int Dim>
class ObsGeo
{
public:
    ObsGeo(MyKDTree &mykdtree):ObsKDTree(mykdtree){}
    
    ObsGeo(const std::vector<Vecf<Dim>> &obstacles, double radius, MyKDTree &mykdtree):ObsKDTree(mykdtree)
    {
        this->obstacles = obstacles;
        this->obsRadius = radius;
    }
    
    std::vector<Vecf<Dim>> radiusSearch( Vecf<Dim> &search_centre, double search_radius )
    {
        double search_point[Dim];
        for(int i = 0; i < Dim; i++ )
            search_point[i] = search_centre(i);
        
        std::vector<nanoflann::ResultItem<long unsigned int, double>> ret_matches;
        
        const size_t nMatches = ObsKDTree.index->radiusSearch( &search_point[0], search_radius, ret_matches);

        std::vector<Vecf<Dim>> ret_obs;
        for (size_t i = 0; i < nMatches; i++)
        {
            ret_obs.push_back( this->obstacles[ret_matches[i].first] );
        }
        return ret_obs;
    }

    std::vector<Vecf<Dim>> getObstacles() { return obstacles; }
    double getObsRadius() { return obsRadius; }
    
    MyKDTree& ObsKDTree;

private:

    std::vector<Vecf<Dim>> obstacles;
    double obsRadius;
};

typedef ObsGeo<2> ObsGeo2d;
typedef ObsGeo<3> ObsGeo3d;

#endif
