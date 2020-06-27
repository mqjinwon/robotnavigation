#include "VisibilityGraph.h"

bool VisibilityGraph::Create(const char *szMapText)
{
    FILE* fp            = fopen(szMapText, "r");
    if(fp==NULL){
        return false;
    }

    // get map size
    if(fscanf(fp, "%d %d", &_nWidth, &_nHeight) !=2){
        fclose(fp);
        return false;
    }

    if(_plpObstacle != nullptr)
        _plpObstacle->clear();

    _plpObstacle = new vector<JPolygon*>;
    (*_plpObstacle).reserve(20 * sizeof(JNODE));


    // read polygon points
    int         nSize;
    JPolygon*   opPolygon;

    while(fscanf(fp, "%d", &nSize) == 1){
        int nX, nY;

        opPolygon = new JPolygon;

        for(int i=0; i<nSize; i++){
            fscanf(fp, "%d %d", &nX, &nY);
            opPolygon->push_back(make_pair(nX, nY));
        }

        _plpObstacle->push_back(opPolygon);
    }
    fclose(fp);

    opPolygon = new JPolygon;

    opPolygon->push_back(make_pair(0        , 0));
    opPolygon->push_back(make_pair(_nWidth-1, 0));
    opPolygon->push_back(make_pair(_nWidth-1, _nHeight-1));
    opPolygon->push_back(make_pair(0        , _nHeight-1));

    _plpObstacle->push_back(opPolygon);

    // Register each vertex and side of a polygon
    char  stID0[51], stID1[51];
    JPoint  oPos;

    for(int n=0; n<_plpObstacle->size(); n++){
        oPos.first  = (*(*_plpObstacle)[n])[0].first;
        oPos.second = (*(*_plpObstacle)[n])[0].second;
        sprintf(stID0, "%d_%d", oPos.first, oPos.second);

        nvimap::AddNode(oPos.first, oPos.second, stID0);

        for(int i=1; i < (*(*_plpObstacle)[n]).size() + 1; i++){
            oPos.first  = (*(*_plpObstacle)[n])[i % (*(*_plpObstacle)[n]).size()].first;
            oPos.second = (*(*_plpObstacle)[n])[i % (*(*_plpObstacle)[n]).size()].second;
            sprintf(stID1, "%d_%d", oPos.first, oPos.second);

            nvimap::AddNode(oPos.first, oPos.second, stID1);

            nvimap::AddNeighbor(stID0, stID1);
            nvimap::AddNeighbor(stID1, stID0);

            strcpy(stID0, stID1);
        }

    }

    // Register using visibility between vertices
    for(int i=0; i < navigationMap.size(); i++){
        for(int j=i+1; j < navigationMap.size(); j++){
            if(BelongToSamePolygon(i, j) == true){
                continue;
            }

            if(Visibility(i,j) == true){
                nvimap::AddNeighbor(navigationMap[i]->szID, navigationMap[j]->szID);
                nvimap::AddNeighbor(navigationMap[j]->szID, navigationMap[i]->szID);
            }
        }
    }

    return true;

}

bool VisibilityGraph::AddNeigborTo(const char *szID){

    JNODE*  opNode  = FindNode(szID);

    // Register using visibility between vertices
    for(int i=0; i < navigationMap.size(); i++){
        if(navigationMap[i]->szID == opNode->szID){
            for(int j=0; j < navigationMap.size(); j++){
                if( (Visibility(i,j) == true) && (i != j) ){
                    nvimap::AddNeighbor(navigationMap[i]->szID, navigationMap[j]->szID);
                    nvimap::AddNeighbor(navigationMap[j]->szID, navigationMap[i]->szID);
                }
            }

            return true;
        }
    }

    return false;
}

bool VisibilityGraph::BelongToSamePolygon(int i, int j){
    for(int k=0; k < _plpObstacle->size(); k++){
        bool bFound = false;
        for(int v=0; v < (*_plpObstacle)[k]->size(); v++){
            if( (navigationMap[i]->nX == (*(*_plpObstacle)[k])[v].first) && (navigationMap[i]->nY == (*(*_plpObstacle)[k])[v].second) ){
                bFound = true;
                break;
            }
        }

        if(bFound == false){
            continue;
        }

        for(int v=0; v < (*_plpObstacle)[k]->size(); v++){
            if( (navigationMap[j]->nX == (*(*_plpObstacle)[k])[v].first) && (navigationMap[j]->nY == (*(*_plpObstacle)[k])[v].second) ){
                return ( (v == (_plpObstacle->size()-1) ) ? false : true);
            }
        }
        return false;
    }

    return false;
}

bool VisibilityGraph::Visibility(int i, int j){

    JPoint A = make_pair(navigationMap[i]->nX, navigationMap[i]->nY);
    JPoint B = make_pair(navigationMap[j]->nX, navigationMap[j]->nY);

    for(int k=0; k <_plpObstacle->size(); k++){
        for(int v=0; v < (*_plpObstacle)[k]->size(); v++){
            JPoint oPos;

            oPos.first  = (*(*_plpObstacle)[k])[v].first;
            oPos.second = (*(*_plpObstacle)[k])[v].second;
            JPoint C = make_pair(oPos.first, oPos.second);

            oPos.first  = (*(*_plpObstacle)[k])[(v+1)%(*_plpObstacle)[k]->size()].first;
            oPos.second = (*(*_plpObstacle)[k])[(v+1)%(*_plpObstacle)[k]->size()].second;
            JPoint D = make_pair(oPos.first, oPos.second);

            if(Intersected(A, B, C, D) == true){
                return false;
            }
        }
    }

    return true;
}

int VisibilityGraph::AreaSign(const JPoint &A, const JPoint &B, const JPoint &C){
    double area2;

    area2 = ( B.first - A.first ) * (double)( C.second - A.second )
                                - ( C.first - A.first ) * (double)( B.second - A.second );

    if ( area2 > 0.5 ) return 1;
    else if ( area2 < -0.5 ) return -1;
    else return 0;
}

bool VisibilityGraph::Collinear(const JPoint &A, const JPoint &B, const JPoint &C){
return (AreaSign(A,B,C) == 0 ? true : false);
}

bool VisibilityGraph::Intersected(const JPoint &A, const JPoint &B, const JPoint &C, const JPoint &D){
    if(Collinear(A,B,C) || Collinear(A,B,D) || Collinear(C,D,A) || Collinear(C,D,B) ||
                                    Right(A,B,C) == Right(A,B,D) || Right(C,D,B) == Right(C,D,A))
        return false;
    else{
        return true;
    }

}
