#ifndef POTENTIALFIELD_H
#define POTENTIALFIELD_H
#include "voronoigraph.h"

typedef pair<double, double> forceVec; //힘 벡터를 저장하는 변수
typedef pair<float, float> Fpoint;

class PotentialField : public VoronoiGraph{
private:
    int     PIXEL_RESOLUTION        = 2;
    float   PIXEL_SIZE              = 0.05;

public:
    forceVec** forceMap;

    PotentialField();
    PotentialField(const KImageGray& igMap) : VoronoiGraph(igMap){
    }

    forceVec attracPotential(Fpoint robotPos, Fpoint goalPos, float ATTRACTIVE_CONSTANT);
    forceVec repulPotential(Fpoint robotPos, Fpoint obsPos, float REPULSIVE_CONSTANT, float NEAR_REPULSIVE);

    Fpoint  shortestPoint(Fpoint robotPos, Obstacle obs);

    forceVec** makePotentialMap(Fpoint stPos, Fpoint endPos, int borderSize, float ATTRACTIVE_CONSTANT = 1., float REPULSIVE_CONSTANT = 1., float NEAR_REPULSIVE = 0.5);

    int getPixelResolution(){return PIXEL_RESOLUTION;}

    int width(){return _igMap.Col();}

    int height(){return _igMap.Row();}

};

#endif // POTENTIALFIELD_H
