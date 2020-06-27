#ifndef LIKELIHOODFIELD_H
#define LIKELIHOODFIELD_H
#include "voronoigraph.h"
#include <kfc.h>
#include <cmath>


class LikelihoodField : public VoronoiGraph{
private:

    double zHit;
    double zRand;
    double rhoHit;
    double zMax;
    double pixelDistance;

    KGaussian gaussian;

public:
    KImageGray      _LHFMap;


    LikelihoodField(){
        initialize();
    };

    LikelihoodField(const KImageGray& igMap) : VoronoiGraph(igMap){
        initialize();
    }

    void initialize();

    KImageGray* likeLihood_field_range_finder_model();

    double GaussianProb(double x, double mean, double sigma);


};

#endif // LIKELIHODDFIELD_H
