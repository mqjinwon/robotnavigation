#include "potentialfield.h"

PotentialField::PotentialField()
{

}

forceVec PotentialField::attracPotential(Fpoint robotPos, Fpoint goalPos, float ATTRACTIVE_CONSTANT){
    forceVec attVec;

    attVec.first  = -ATTRACTIVE_CONSTANT * ((robotPos.first - goalPos.first) * PIXEL_SIZE);
    attVec.second = -ATTRACTIVE_CONSTANT * ((robotPos.second - goalPos.second) * PIXEL_SIZE);

    return attVec;
}

forceVec PotentialField::repulPotential(Fpoint robotPos, Fpoint obsPos, float REPULSIVE_CONSTANT, float NEAR_REPULSIVE){
    forceVec    repulVec;
    double       obsDist = sqrt( pow(( (robotPos.first - obsPos.second) * PIXEL_SIZE), 2.)
                                 + pow((robotPos.second - obsPos.first)* PIXEL_SIZE, 2.) );

    double      repulparam = 0;

    if(obsDist <= NEAR_REPULSIVE){
        repulparam      = REPULSIVE_CONSTANT * (1./obsDist - 1./NEAR_REPULSIVE) / pow(obsDist, 2.) / obsDist;

        repulVec.first  = repulparam * ( (robotPos.first - obsPos.second) * PIXEL_SIZE);
        repulVec.second = repulparam * ( (robotPos.second - obsPos.first) * PIXEL_SIZE);
    }
    else{
        repulVec.first = 0;
        repulVec.second = 0;
    }

    return repulVec;
}

Fpoint PotentialField::shortestPoint(Fpoint robotPos, Obstacle obs)
{
    Fpoint  shortPos;
    double   shortDist = numeric_limits<double>::max();

    for(auto& _obs:obs){
        double tmpDist = sqrt(pow(_obs.second * PIXEL_SIZE - robotPos.first * PIXEL_SIZE,2.) + pow(_obs.first * PIXEL_SIZE - robotPos.second * PIXEL_SIZE, 2.));
        if(tmpDist < shortDist){
            shortPos = _obs;
            shortDist = tmpDist;
        }
    }

    return shortPos;
}

forceVec** PotentialField::makePotentialMap(Fpoint endPos1, Fpoint endPos2, int borderSize, float ATTRACTIVE_CONSTANT, float REPULSIVE_CONSTANT, float NEAR_REPULSIVE)
{

    Obstacles mapObs = getObstacles(BACKGROUND, borderSize);

    forceVec**    forceMap;

    forceMap = (forceVec**) new forceVec[_igMap.Row()];

    for(int i = 0; i < _igMap.Row(); ++i) {
        forceMap[i] = new forceVec[_igMap.Col()];
        memset(forceMap[i], 0, sizeof(char)*_igMap.Col()); // 메모리 공간을 0으로 초기화
    }

    float    maxSize = 0.;
    int      nBKG    = 255;

    for(int row = borderSize + 1; row < _igMap.Row() - (borderSize + 1); row++){

        for(int col = borderSize + 1; col < _igMap.Col() - (borderSize + 1); col++){

            forceVec    sumVec(0.,0.);

            if(_igMap._ppA[row][col] == nBKG){
                forceVec    attracVec1(attracPotential(Fpoint(col, row), endPos1, ATTRACTIVE_CONSTANT));
                forceVec    attracVec2(attracPotential(Fpoint(col, row), endPos2, ATTRACTIVE_CONSTANT));
                //qDebug("att : (%f, %f)", attracVec.first, attracVec.second);
                forceVec    sumRepulsVec(0.,0.);

                // 장애물을 하나씩 꺼내서 repulsive를 계산한다.
                for(int obs=0; obs<mapObs.size(); obs++){
                    Fpoint shortObsPoint    = shortestPoint(Fpoint(col, row), _Obstacles[obs]);

                    Fpoint repulVec         = repulPotential(Fpoint(col, row) ,shortObsPoint, REPULSIVE_CONSTANT, NEAR_REPULSIVE);
                    //qDebug("repul : (%f, %f)", repulVec.first, repulVec.second);

                    sumRepulsVec.first  += repulVec.first;
                    sumRepulsVec.second += repulVec.second;

                }
                //qDebug("repul : (%f, %f)", sumRepulsVec.first, sumRepulsVec.second);

                sumVec.first    = attracVec1.first + attracVec2.first + sumRepulsVec.first;
                sumVec.second   = attracVec1.second + attracVec2.second + sumRepulsVec.second;
            }
            forceMap[row][col] = sumVec;

            float tmpDist = sqrt(pow(sumVec.first,2) + pow(sumVec.second, 2));

            if(maxSize < tmpDist)   maxSize = tmpDist;

            //qDebug("x: %d, y: %d, vector :(%f, %f)", col, row, forceMap[row][col].first, forceMap[row][col].second);
        }
    }

    for(int row = borderSize + 1; row < _igMap.Row() - (borderSize + 1); row++){
        for(int col = borderSize + 1; col < _igMap.Col() - (borderSize + 1); col++){

            //qDebug("x: %d, y: %d, vector :(%f, %f)", col, row, forceMap[row][col].first, forceMap[row][col].second);

//            if(maxSize != 0){
//                forceMap[row][col].first  *= PIXEL_RESOLUTION / (float)maxSize;
//                forceMap[row][col].second *= PIXEL_RESOLUTION / (float)maxSize;
//            }
//            else{
//                forceMap[row][col].first = 0;
//                forceMap[row][col].second = 0;
//            }


            //qDebug("x: %d, y: %d, vector :(%f, %f)", col, row, forceMap[row][col].first, forceMap[row][col].second);

        }
    }

    qDebug() << "========================================maxsize: " << maxSize;

    return forceMap;
}



