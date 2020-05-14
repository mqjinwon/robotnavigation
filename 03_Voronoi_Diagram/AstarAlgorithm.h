#ifndef ASTARALGORITHM_H
#define ASTARALGORITHM_H

#include <string.h>
#include <map>
#include <queue>
#include <limits> //for max num(infinite)
#include <cmath>
#include <QDebug>

#define _MAX_NEIGHBOR_A_STAR    300

using namespace std;

typedef pair<int, int> JPoint;

struct JNODE{

    int     nX, nY;                                 // node Position
    char    szID[51];                               // node Name
    double  dCostF, dCostG;                         // dCostF: dCostG + heuristics cost, dCostG: from start to now
    char    szpNeighbor[_MAX_NEIGHBOR_A_STAR][51];  // save neighbor node
    int     nNeighbors;                             // neighbor node num

    JNODE*  opPrvious = nullptr;                    // previous Node pointer
};

class nvimap {

public:

    struct compare{
        bool operator()(const JNODE* n1, const JNODE* n2){
            return n1->dCostF >= n2->dCostF;
        }
    };

    vector<JNODE*>                                  navigationMap;  // making navigation map

    nvimap(){};
    ~nvimap(){
        Reset();
    }
    //allocate vector memory
    nvimap(int mapsize){
        navigationMap.reserve(mapsize * sizeof(JNODE));
    }

    JNODE* FindNode(const char* szID);
    bool AddNode(const int& nX, const int& nY, const char* szID);   // add node
    bool AddNeighbor(const char* szID, const char* szNeighbor);     // add neighbor node
    double Heuristic(int sX, int sY, int eX, int eY);               // two dimension euclidean distance
    JNODE* Execute(const char* st, const char* end);                // execute Astar algorithm
    bool AddNeigborTo(const char *szID);                            // add neighbor

protected:
    void Reset();

};


#endif // ASTARALGORITHM_H
