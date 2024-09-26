
#ifndef CBSSEARCH_H_
#define CBSSEARCH_H_

#include "common.h"
#include "copter_common.h"
#include "globalmap.h"
#include "slplanner.h"
#include "cbsnode.h"

#include <thread>
#include <future>

template <int Dim>
class CBSSearch
{
public:
    CBSSearch( GlobalMap<Dim> &gmap_, CopterParameter<Dim> &cp_, std::vector<CopterState<Dim>> &starts_, std::vector<CopterState<Dim>> &goals_, bool debug = true ):gmap(gmap_), cp(cp_), starts(starts_), goals(goals_)
    {
        this->uav_num = goals.size();
        if ( debug )
            printCBSSearchInfo();
        slplanners.resize( uav_num );
        iter_solutions.resize( uav_num );
        for( int i = 0; i < uav_num; i++ )
        {
            slplanners[i] = new SLPlanner<Dim>(gmap, cp, starts[i], goals[i], false );
        }
    }

    ~CBSSearch()
    {
        for ( int i = 0; i < uav_num; i++ )
        {
            delete slplanners[i];
        }
    }

    void printCBSSearchInfo()
    {
        gmap.printGlobalMapInfo();
        cp.printCopterParameterInfo();
        std::cout << "UAV Swarm Num: " << uav_num << std::endl;
        for( int i = 0; i < uav_num; i++ )
        {
            std::cout << "UAV ID: " << i << std::endl;
            starts[i].printCopterStateInfo("Start State Info:");
            goals[i].printCopterStateInfo("Goal State Info:");
        }
    }


    bool search(std::vector<Solution<Dim>> &sols);

    bool cbssearch_id(std::vector<Solution<Dim>> &sols);
    bool generateCBSRootNode();
    void divideGroups( std::list<Conflict<Dim>> &conflicts, std::vector<std::vector<int>> &groups );
    void search_group( std::vector<Solution<Dim>> &sols, std::vector<int> &group, int &retFlag);


    void findAllConflicts( CBSNodePtr<Dim> currCBSNodePtr, CBSNodePtr<Dim> parentCBSNodePtr = NULL, int uav_id = -1 );
    bool checkConflict( std::vector<Solution<Dim>> &sols, int one_uav, int another_uav, int ts, Conflict<Dim> &conflict);
    bool StateCollision( CopterState<Dim> &one_state, CopterState<Dim> &another_state );
    bool PrimitiveCollision( Primitive<Dim> &one_pr, Primitive<Dim> &another_pr );
    bool StatePrimitiveCollision( Primitive<Dim> &one_pr, CopterState<Dim> &another_state );


    Conflict<Dim> chooseConflict( CBSNodePtr<Dim> currCBSNode );
    void divideConstraints( Conflict<Dim> &conflict, std::map< int,Constraints<Dim> > &allcons );


    void printSolutionsInfo( std::vector<Solution<Dim>> &sols );
    void validateSolutions( std::vector<Solution<Dim>> &sols );
    // bool revalidateSolutions( std::vector<Solution<Dim>> &sols );


    // std::vector<Conflict<Dim>> getHandleConflicts() { return handle_conflicts; }
    // std::vector<Constraints<Dim>> getHandleConstraints() { return handle_constraints; }
    // std::vector<Solution<Dim>> getHandleSolutions() { return handle_solutions; }
    // std::vector<int> getHandleUAVID() { return handle_uav_id; }

private:
    GlobalMap<Dim> &gmap;
    CopterParameter<Dim> &cp;
    std::vector<CopterState<Dim>> starts;
    std::vector<CopterState<Dim>> goals;
    int uav_num;

    std::vector<SLPlanner<Dim>*> slplanners;

    CBSNode<Dim> root;
    std::vector<Solution<Dim>> iter_solutions;
    std::list< Conflict<Dim> > iter_conflicts;
public:
    int iter_conflicts_num;

    // std::vector<Conflict<Dim>> handle_conflicts;
    // std::vector<Constraints<Dim>> handle_constraints;
    // std::vector<Solution<Dim>> handle_solutions;
    // std::vector<int> handle_uav_id;

};

typedef CBSSearch<2> CBSSearch2d;
typedef CBSSearch<3> CBSSearch3d;

#endif
