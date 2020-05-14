#include "Voronoigraph.h"



VoronoiGraph::VoronoiGraph()
{

}

KImageGray VoronoiGraph::Execute(int obType, int bfType, int threshold, int scale){
    getObstacles(obType);
    brushFire(bfType, threshold);

    return scaleUp(_BFimg, scale);
}

bool VoronoiGraph::AddSENeigborTo(const char *szID)
{
    JNODE*  opNode  = FindNode(szID);
    JNODE*  shortNode;
    int     shortlength = numeric_limits<int>::max();

    // Register neigbor other node
    for(int i=0; i < navigationMap.size(); i++){
        if(navigationMap[i]->szID != opNode->szID){
            int tmplength = Heuristic(opNode->nX, opNode->nY, navigationMap[i]->nX, navigationMap[i]->nY);
            if (tmplength < shortlength){
                shortlength = tmplength;
                shortNode = navigationMap[i];
            }
        }
    }

    nvimap::AddNeighbor(shortNode->szID, szID);
    nvimap::AddNeighbor(szID, shortNode->szID);

    return true;
}

Obstacles VoronoiGraph::getObstacles(int type){
    // create obsatcle
    Obstacle    *obs            = new Obstacle;

    int         nOBJ            = (type == FOREGROUND ? 255 : 0);
    int         nBKG            = (type == FOREGROUND ? 0 : 255);
    int         nCHK            = 100;

    //1. 장애물을 만났을 때, 이것이 이미 정의가 되어있는 장애물인지 확인한다.(nCHK를 활용)
    //2. 장애물 픽셀 발견시 8방향으로 뻗어가며 장애물의 경계를 찾는다.(nOBJ를 활용)
    //3. 이때 경계를 판단하는 방법은 주변에 배경이 있는지이다.(nBKG를 활용)

    for(int row=0; row < _igMap.Row(); row++){
        for(int col=0; col < _igMap.Col(); col++){

            // 만약 이 픽셀이 이미 정의된 경계라면
            if(_igMap._ppA[row][col] == nCHK){
                for(int i = col+1; _igMap.Col(); i++){
                    if(_igMap._ppA[row][i] == nBKG){
                        col = i;
                        break;
                    }
                }
            }
            // 만약 이 픽셀이 장애물이라면
            else if(_igMap._ppA[row][col] == nOBJ){
                int     obRow = row;
                int     obCol = col;
                bool    isEnd(false);

                do{
                    obs->push_back(make_pair(obRow, obCol));
                    _igMap._ppA[obRow][obCol] = nCHK;

                    int     findRow;
                    int     findCol;


                    // 다음으로 이동할 좌표가 경계값이고, 이전에 선택하지 않은 값인지 확인
                    for(int i=0; i<8; i++){
                        findRow = obRow + eightWay[i][0];
                        findCol = obCol + eightWay[i][1];

                        if( (*obs)[0] == make_pair(findRow, findCol)){
                            isEnd = true;
                            break;
                        }

                        if(_igMap._ppA[findRow][findCol] == nOBJ){

                            bool isBoundary(false);

                            int bkgCount = 0;
                            for(int j=0; j<8; j++){
                                if(_igMap._ppA[findRow + eightWay[j][0]][findCol + eightWay[j][1]] == nBKG){
                                    bkgCount++;
                                    if(bkgCount>=2){
                                        isBoundary = true;
                                        break;
                                    }
                                }
                            }

                            if(isBoundary){
                                bool isOverlap(false);

                                for(int i=0; i < obs->size(); i++){
                                    if((*obs)[i] == make_pair(findRow, findCol)){
                                        isOverlap = true;
                                        break;
                                    }
                                }

                                if(!isOverlap){
                                    obRow = findRow;
                                    obCol = findCol;
                                    break;
                                }
                            }

                        }
                    }

                }while( !isEnd );

                _Obstacles.push_back(*obs);
                obs = new Obstacle;
            }
        }
    }

    // 이미지의 꼭지점들도 장애물로 인식함
    obs->push_back(make_pair(0,0));


    obs = new Obstacle;
    obs->push_back(make_pair(_igMap.Row()-1,0));


    obs = new Obstacle;
    obs->push_back(make_pair(0,_igMap.Col()-1));


    obs = new Obstacle;
    obs->push_back(make_pair(_igMap.Row()-1,_igMap.Col()-1));
    _Obstacles.push_back(*obs);

    return _Obstacles;
}

void VoronoiGraph::brushFire(int type,int threshold){
    // 모든 값을 0으로 하는 이미지를 만듬
    _BFimg.Create(_igMap.Row(), _igMap.Col());

    queue<brushNode*>*      BQueue = new queue<brushNode*>;
    brushNode*              BPoint = nullptr;
    Obstacle                *_obs;

    //제일 큰 값으로 맞춰준다.
    for(int i=0; i< _BFimg.Row(); i++){
        for(int j=0; j< _BFimg.Col(); j++){
            _BFimg[i][j] = 255;
        }
    }
    // 장애물 좌표로 부터 하나씩 멀어져간다.( fourway나 eightway를 선택할 수 있음)
    // 경계값의 경우 0으로 만들어주고 더이상 진행하지 않는다.
    // 점점 멀어져나가다가 이미지에 저장된 값과 점이 가지고 있는 값의 차이가 임계값을 내부에 포함되면 그만 진행한다.
    if(type == FOURWAY){
        // 장애물이 비어있지 않다면 첫번째 장애물을 이용해서 기준 이미지를 만든다.
        // 기준 이미지 : 장애물로부터 뻗어나가는 값중에서 큰값을 기준으로 brush fire 값을 정의한다.
        // 이때 장애물의 경계선에 있을 경우 queue를 삭제해버린다.
        for(int i=0; i< _Obstacles.size(); i++){
            _obs = &_Obstacles[i];

            // BQueue에 point들을 넣어놓는다.
            for(int j=0; j< _obs->size(); j++){
                BPoint = new brushNode;
                BPoint->BCodin = (*_obs)[j];
                BPoint->Bvalue = 0;
                BQueue->push(BPoint);
            }

            // 장애물에 있던 점을 하나씩 꺼내면서 맵을 만들어 나간다.
            while(!(BQueue->empty()) ){

                BPoint = BQueue->front();
                BQueue->pop();

                int findRow;
                int findCol;
                brushNode* newBPoint;

                // 4방향으로 뻗어나가면서 brush fire 진행
                for(int i=0; i<4; i++){
                    findRow = BPoint->BCodin.first    + fourWay[i][0];
                    findCol = BPoint->BCodin.second   + fourWay[i][1];

                    // 경계값이라면 0으로 초기화 해준다.
                    bool isBoundary = false;
                    for(int i=0; i< _obs->size(); i++){
                        if( (*_obs)[i] == make_pair(findRow, findCol) ){
                            isBoundary = true;
                            _BFimg[findRow][findCol] = 0;
                            break;
                        }
                    }

                    if(isBoundary) continue;
                    // 경계값이 아니라면
                    else{
                        // 이미지의 경계에 있는지 확인한다.
                        if(findRow < 0 || findRow > _igMap.Row()-1 || findCol < 0 || findCol > _igMap.Col()-1 ){
                            continue;
                        }
                        if(abs(_BFimg[findRow][findCol] - (BPoint->Bvalue + 1)) <= threshold){
                            continue;
                        }
                        else{
                            // 만약 이미지의 값보다 작거나 이미지 값이 0이라면 집어넣는다.
                            if( ( (BPoint->Bvalue + 1) < _BFimg[findRow][findCol])){
                                newBPoint = new brushNode;
                                newBPoint->BCodin = make_pair(findRow, findCol);
                                newBPoint->Bvalue = BPoint->Bvalue + 1;
                                _BFimg[findRow][findCol] = newBPoint->Bvalue;
                                BQueue->push(newBPoint);
                            }
                        }
                    }
                }
            }
        }
    }

    if(type == EIGHTWAY){
        // 장애물이 비어있지 않다면 첫번째 장애물을 이용해서 기준 이미지를 만든다.
        // 기준 이미지 : 장애물로부터 뻗어나가는 값중에서 큰값을 기준으로 brush fire 값을 정의한다.
        // 이때 장애물의 경계선에 있을 경우 queue를 삭제해버린다.
        for(int i=0; i< _Obstacles.size(); i++){
            _obs = &_Obstacles[i];

            // BQueue에 point들을 넣어놓는다.
            for(int j=0; j< _obs->size(); j++){
                BPoint = new brushNode;
                BPoint->BCodin = (*_obs)[j];
                BPoint->Bvalue = 0;
                BQueue->push(BPoint);
            }

            // 장애물에 있던 점을 하나씩 꺼내면서 맵을 만들어 나간다.
            while(!(BQueue->empty()) ){

                BPoint = BQueue->front();
                BQueue->pop();

                int findRow;
                int findCol;
                brushNode* newBPoint;

                // 8방향으로 뻗어나가면서 brush fire 진행
                for(int i=0; i<8; i++){
                    findRow = BPoint->BCodin.first    + eightWay[i][0];
                    findCol = BPoint->BCodin.second   + eightWay[i][1];

                    // 경계값이라면 0으로 초기화 해준다.
                    bool isBoundary = false;
                    for(int i=0; i< _obs->size(); i++){
                        if( (*_obs)[i] == make_pair(findRow, findCol) ){
                            isBoundary = true;
                            _BFimg[findRow][findCol] = 0;
                            break;
                        }
                    }

                    if(isBoundary) continue;
                    // 경계값이 아니라면
                    else{
                        // 이미지의 경계에 있는지 확인한다.
                        if(findRow < 0 || findRow > _igMap.Row()-1 || findCol < 0 || findCol > _igMap.Col()-1 ){
                            continue;
                        }
                        if(abs(_BFimg[findRow][findCol] - (BPoint->Bvalue + 1)) <= threshold){
                            continue;
                        }
                        else{
                            // 만약 이미지의 값보다 작거나 이미지 값이 0이라면 집어넣는다.
                            if( ( (BPoint->Bvalue + 1) < _BFimg[findRow][findCol])){
                                newBPoint = new brushNode;
                                newBPoint->BCodin = make_pair(findRow, findCol);
                                newBPoint->Bvalue = BPoint->Bvalue + 1;
                                _BFimg[findRow][findCol] = newBPoint->Bvalue;
                                BQueue->push(newBPoint);
                            }
                        }
                    }
                }
            }
        }
    }

    // 장애물 내부를 채워주기 위해서 사용
    int nBKG            = 255;
    int nCHK            = 100;

    for(int row=0; row < _igMap.Row(); row++){
        for(int col=0; col < _igMap.Col(); col++){

            if(_igMap._ppA[row][col] == nCHK){
                for(int i = col+1; _igMap.Col(); i++){
                    _BFimg._ppA[row][i] = 0;
                    if(_igMap._ppA[row][i] == nBKG){
                        col = i;
                        break;
                    }
                }
            }
        }
    }

    //--------------------------------------------------------------------------------
    // local maximum을 노드로 추가

    for(int row=0; row < _BFimg.Row(); row++){
        for(int col=0; col < _BFimg.Col(); col++){
            int isLarge = 0;
            // lcoal maximum 픽셀인지 확인

            //장애물이면 패스(장애물의 픽셀 : 0)
            if(_BFimg[row][col] == 0){
                continue;
            }

            for(int i=0; i<8; i++){
                int findRow = row + eightWay[i][0];
                int findCol = col + eightWay[i][1];
                if(findRow >=0 && findCol >=0 && findRow < _BFimg.Row() && findCol < _BFimg.Col()){
                    if(_BFimg[findRow][findCol] > _BFimg[row][col]){
                        isLarge++;
                    }
                }

            }

            // local maximum을 노드로 추가
            if(isLarge <= 1){
                char  szID[51];
                sprintf(szID, "%d_%d", col, row);
                AddNode(col, row, szID);
            }
        }
    }

    //서로간의 이웃으로 추가
    for(int i=0; i < navigationMap.size(); i++){
        for(int j=0; j < navigationMap.size(); j++){
            if(i==j)    continue;

            if( abs(navigationMap[j]->nX - navigationMap[i]->nX) + abs(navigationMap[i]->nY - navigationMap[j]->nY) <= 3){
                char* szID = navigationMap[j]->szID;

                nvimap::AddNeighbor(szID, navigationMap[i]->szID);
                nvimap::AddNeighbor(navigationMap[i]->szID, szID);
            }
        }

    }

//     local maximum을 255픽셀로 바꿔서 보기 좋게 함
    for(int i=0; i< navigationMap.size(); i++){
        _BFimg[navigationMap[i]->nY][navigationMap[i]->nX] = 255;
    }

    return;
}

KImageGray VoronoiGraph::scaleUp(const KImageGray& igMap, int scale){

    KImageGray  scaleImg;
    scaleImg.Create(igMap.Row(), igMap.Col());

    if(scale == 0){

    }
    else{
        for(int _row=0; _row<scaleImg.Row(); _row++){
            for(int _col=0; _col<scaleImg.Col(); _col++){
                if(igMap[_row][_col] * scale > 255){
                    scaleImg[_row][_col] = 255;
                }
                else{
                    scaleImg[_row][_col] = igMap[_row][_col] * scale;
                }
            }
        }
    }

    return scaleImg;
}
