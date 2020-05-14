#include "AstarAlgorithm.h"


JNODE* nvimap::FindNode(const char *szID)
{
    JNODE* findNode = nullptr;

    for(int i=0; i < navigationMap.size(); i++){
        if(strncmp(navigationMap[i]->szID, szID, strlen(szID)) == 0){
            findNode = navigationMap[i];
            break;
        }
    }

    return findNode;
}

bool nvimap::AddNode(const int &nX, const int &nY, const char *szID){
    JNODE*  opNode  = FindNode(szID);

    //if same ID already exist, return false
    if(opNode == nullptr){
        opNode              = new JNODE;
        opNode->nX          = nX;
        opNode->nY          = nY;
        opNode->nNeighbors  = 0;
        opNode->opPrvious   = nullptr;

        opNode->dCostG      = numeric_limits<short>::max();
        opNode->dCostF      = opNode->dCostG;

        strcpy(opNode->szID, szID);

        navigationMap.push_back(opNode);

        return true;
    }
    else{
        return false;
    }

}

bool nvimap::AddNeighbor(const char *szID, const char *szNeighbor){
    JNODE*  opNode  = FindNode(szID);
    int*    nNum    = &opNode->nNeighbors;

    if(opNode == nullptr)
        return false;

    //if same ID already exist, return false
    for(int i=0; i < *nNum; i++){
        if(strncmp(opNode->szpNeighbor[i], szNeighbor, strlen(szNeighbor)) == 0)
            return false;
    }

    //add neighbor
    strcpy(opNode->szpNeighbor[*nNum], szNeighbor);
    (*nNum)++;
    return true;
}

double nvimap::Heuristic(int sX, int sY, int eX, int eY){
    return sqrt(pow(eX-sX, 2.) + pow(eY-sY, 2.));
}

JNODE *nvimap::Execute(const char *stID, const char *endID){

    // check whether error ID input or not
    if(FindNode(stID) == nullptr || FindNode(endID) == nullptr){
        exit(1);
    }

    JNODE* endNode      = FindNode(endID);                           // end node
    int eX              = endNode->nX;  int eY = endNode->nY;

    JNODE* startNode    = FindNode(stID);
    startNode->dCostG   = 0.;
    startNode->dCostF   = startNode->dCostG + Heuristic(startNode->nX, startNode->nY, eX, eY);

    priority_queue<JNODE*, vector<JNODE*>, compare>   AstarContainer;   // use for finding shortest path
    AstarContainer.push(startNode);                                     // add startNode

    JNODE* findNode;                                                    // find node

    findNode = AstarContainer.top();
    AstarContainer.pop();

    do{

        //if find endNode
        if(strcmp(findNode->szID, endNode->szID) == 0){
            return findNode;
        }

        //add neighbor node
        for(int nNum=0; nNum < findNode->nNeighbors; nNum++){

            JNODE* neighborNODE = FindNode(findNode->szpNeighbor[nNum]);


            // dcostF,G 값을 잠시 저장한다.
            double tmpCostG = findNode->dCostG + Heuristic(findNode->nX, findNode->nY, neighborNODE->nX, neighborNODE->nY);
            double tmpCostF = tmpCostG + Heuristic(neighborNODE->nX, neighborNODE->nY, eX, eY);

            ////don't put neighborNODE's dCostF has lager than Acontainer's dCostF
            //for comparing, change prior queue to queue
            JNODE* compareNode = nullptr;
            queue<JNODE*> compareQueue;
            while(!AstarContainer.empty()){
                compareQueue.push(AstarContainer.top());
                AstarContainer.pop();
            }
            //find whether same id exist
            int qNum = compareQueue.size();
            //same id flag
            bool identityFlag = false;
            for(int i=0; i<qNum; i++){
                compareNode = compareQueue.front();
                compareQueue.pop();

                //if AstarContainer already has same ID
                if(strcmp(compareNode->szID, neighborNODE->szID) == 0){
                    if(tmpCostF < compareNode->dCostF){
                        neighborNODE->dCostF     = tmpCostF;
                        neighborNODE->dCostG     = tmpCostG;
                        neighborNODE->opPrvious  = findNode;
                        compareQueue.push(neighborNODE);
                    }
                    else{
                        compareQueue.push(compareNode);
                    }
                    identityFlag = true;
                    break;
                }
                //push previous pop node
                else{
                    compareQueue.push(compareNode);
                }
            }

            //there is not same id
            if(!identityFlag){
                if(tmpCostF < neighborNODE->dCostF){
                    neighborNODE->dCostF     = tmpCostF;
                    neighborNODE->dCostG     = tmpCostG;
                    neighborNODE->opPrvious  = findNode;
                    compareQueue.push(neighborNODE);
                }
            }

            while(!compareQueue.empty()){
                AstarContainer.push(compareQueue.front());
                compareQueue.pop();
            }
        }

        //can't reach endNode
        if(AstarContainer.empty()){
            exit(1);
        }

        findNode = AstarContainer.top();
        AstarContainer.pop();

    }while(1);


}

bool nvimap::AddNeigborTo(const char *szID){
    JNODE*  opNode  = FindNode(szID);

    // Register neigbor other node
    for(int i=0; i < navigationMap.size(); i++){
        if(navigationMap[i]->szID != opNode->szID){
            nvimap::AddNeighbor(navigationMap[i]->szID, szID);
            nvimap::AddNeighbor(szID, navigationMap[i]->szID);
        }
    }

    return false;
}

void nvimap::Reset()
{
    if(navigationMap.size() != 0){
        for(auto v : navigationMap){
            delete v;
        }
        navigationMap.clear();
    }
}


