#ifndef ASTARALGORITHM_H
#define ASTARALGORITHM_H

#include <string.h>
#include <map>
#include <queue>
#include <limits> //for max num(infinite)
#include <cmath>

#define _MAX_NEIGHBOR_A_STAR    5

using namespace std;

struct KNODE{

    int     nX, nY;                                 // node Position
    char    szID[51];                               // node Name
    double  dCostF, dCostG;                         // dCostF: dCostG + heuristics cost, dCostG: from start to now
    char    szpNeighbor[_MAX_NEIGHBOR_A_STAR][51];  // save neighbor node
    int     nNeighbors;                             // neighbor node num

    KNODE*  opPrvious;                              // previous Node pointer
};

class nvimap {

public:

    struct compare{
        bool operator()(const KNODE* n1, const KNODE* n2){
            return n1->dCostF > n2->dCostF;
        }
    };

    map<string, KNODE*>                             mapID;          // for finding KNODE fast
    vector<KNODE*>                                  navigationMap;  // making navigation map

    //allocate vector memory
    nvimap(int mapsize){
        navigationMap.reserve(mapsize * sizeof(KNODE));
    }

    bool AddNode(const int& nX, const int& nY, const char* szID);   // add node
    bool AddNeighbor(const char* szID, const char* szNeighbor);     // add neighbor node
    double Heuristic(int sX, int sY, int eX, int eY);               // two dimension euclidean distance
    KNODE* Execute(const char* st, const char* end);                // execute Astar algorithm

};


#endif // ASTARALGORITHM_H
