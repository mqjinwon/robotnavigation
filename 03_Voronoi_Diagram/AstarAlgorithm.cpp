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
        opNode      = new JNODE;
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

    JNODE* startNode    = new JNODE;
    memcpy(startNode, FindNode(stID), sizeof(struct JNODE));
    startNode->dCostG   = 0.;
    startNode->dCostF   = startNode->dCostG + Heuristic(startNode->nX, startNode->nY, eX, eY);

    priority_queue<JNODE*, vector<JNODE*>, compare>   AstarContainer;   // use for finding shortest path
    AstarContainer.push(startNode);                                     // add startNode

    JNODE* findNode;                                                    // find node



    do{
        //can't reach endNode
        if(AstarContainer.empty()){
            exit(1);
        }

        findNode = AstarContainer.top();
        AstarContainer.pop();

        //if find endNode
        if(strcmp(findNode->szID, endNode->szID) == 0){
            return findNode;
        }

        //add neighbor node
        for(int nNum=0; nNum < findNode->nNeighbors; nNum++){

            JNODE* newNODE = new JNODE;
            memcpy(newNODE, FindNode(findNode->szpNeighbor[nNum]), sizeof(struct JNODE));

            // dcostF,G 값을 갱신하고, previous 노드에 추가한다

            // Add
            newNODE->opPrvious = findNode;
            newNODE->dCostG = findNode->dCostG + Heuristic(findNode->nX, findNode->nY, newNODE->nX, newNODE->nY);
            newNODE->dCostF = newNODE->dCostG + Heuristic(newNODE->nX, newNODE->nY, eX, eY);


            ////don't put newNODE's dCostF has lager than Acontainer's dCostF
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
                if(strcmp(compareNode->szID, newNODE->szID) == 0){
                    if(newNODE->dCostF < compareNode->dCostF){
                        compareQueue.push(newNODE);
                    }
                    else{
                        compareQueue.push(compareNode);
                        delete newNODE;
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
                compareQueue.push(newNODE);
            }

            while(!compareQueue.empty()){
                AstarContainer.push(compareQueue.front());
                compareQueue.pop();
            }

        }

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


