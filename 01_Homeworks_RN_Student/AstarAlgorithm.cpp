#include "AstarAlgorithm.h"


bool nvimap::AddNode(const int &nX, const int &nY, const char *szID){
    KNODE*  opNode      = new KNODE;
    opNode->nX          = nX;
    opNode->nY          = nY;
    opNode->nNeighbors  = 0;
    opNode->opPrvious   = nullptr;

    opNode->dCostG      = numeric_limits<short>::max();
    opNode->dCostF      = opNode->dCostG;

    strcpy(opNode->szID, szID);
    mapID.insert(make_pair(szID, opNode));  // add mapID

    navigationMap.push_back(opNode);

    return true;
}

bool nvimap::AddNeighbor(const char *szID, const char *szNeighbor){
    KNODE*  opNode  = mapID[szID];
    int*    nNum    = &opNode->nNeighbors;

    //if same ID already exist, return false
    for(int i=0; i < *nNum; i++){
        if(strncmp(opNode->szpNeighbor[i], szNeighbor,1) == 0)
            return false;
    }

    //add neighbor
    strcpy(opNode->szpNeighbor[(*nNum)++], szNeighbor);
    return true;
}

double nvimap::Heuristic(int sX, int sY, int eX, int eY){
    return sqrt(pow(eX-sX, 2.) + pow(eY-sY, 2.));
}

KNODE *nvimap::Execute(const char *stID, const char *endID){

    // check whether error ID input or not
    if(mapID.find(stID) == mapID.end() || mapID.find(endID) == mapID.end()){
        exit(1);
    }

    KNODE* endNode      = mapID[endID];                           // end node
    int eX              = endNode->nX;  int eY = endNode->nY;

    KNODE* startNode    = new KNODE;
    memcpy(startNode, mapID[stID], sizeof(struct KNODE));
    startNode->dCostG   = 0.;
    startNode->dCostF   = startNode->dCostG + Heuristic(startNode->nX, startNode->nY, eX, eY);

    priority_queue<KNODE*, vector<KNODE*>, compare>   AstarContainer; // use for finding shortest path
    AstarContainer.push(startNode);                                // add startNode

    KNODE* findNode;                                                 // find node



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

//        //can't reach endNode
//        if(AstarContainer.empty() && findNode != startNode){
//            exit(1);
//        }

        //add neighbor node
        for(int nNum=0; nNum < findNode->nNeighbors; nNum++){

            KNODE* newNODE = new KNODE;
            memcpy(newNODE, mapID[findNode->szpNeighbor[nNum]], sizeof(struct KNODE));

            // dcostF,G 값을 갱신하고, previous 노드에 추가한다

            // Add
            newNODE->opPrvious = findNode;
            newNODE->dCostG = findNode->dCostG + Heuristic(findNode->nX, findNode->nY, newNODE->nX, newNODE->nY);
            newNODE->dCostF = newNODE->dCostG + Heuristic(newNODE->nX, newNODE->nY, eX, eY);


            ////don't put newNODE's dCostF has lager than Acontainer's dCostF
            //for comparing, change prior queue to queue
            KNODE* compareNode;
            queue<KNODE*> compareQueue;
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


