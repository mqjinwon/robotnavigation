#ifndef ODOMETRYMOTION_H
#define ODOMETRYMOTION_H
#include "AstarAlgorithm.h"
#include <kfc.h>
#include <vector>
#include <queue>
#include <cmath>

struct OPoint{
    double x;
    double y;
    double theta;
};
typedef vector<OPoint> OPoints;

class OdometryMotion : public nvimap{
private:
    KGaussian gaussian;
    float alpha1 = 0, alpha2 = 0, alpha3 = 0, alpha4 = 0;

public:
    OPoints odoPoints;

    OdometryMotion(int point_num, float _x, float _y, float _theta){
        odoPoints.reserve(point_num * sizeof (OPoints));
        setInitPosition(point_num, _x, _y, _theta);
        gaussian.OnRandom(100000);
    }

    void setVarianceParam(float _alpha1, float _alpha2, float _alpha3, float _alpha4);
    void setInitPosition(int point_num, float _x, float _y, float _theta);
    OPoints Prediction(float _x, float _y, float _theta);

    double gaussianGenerate(const double& dMean=0, const double& dSigma=1);
};

#endif // ODOMETRYMOTION_H
