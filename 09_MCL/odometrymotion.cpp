#include "odometrymotion.h"



void OdometryMotion::setVarianceParam(float _alpha1, float _alpha2, float _alpha3, float _alpha4){
    alpha1 = _alpha1;
    alpha2 = _alpha2;
    alpha3 = _alpha3;
    alpha4 = _alpha4;

}

void OdometryMotion::setInitPosition(int point_num, float _x, float _y, float _theta)
{
    if(!odoPoints.empty())
        odoPoints.clear();

    OPoint newPoint;
    newPoint.x     = _x;
    newPoint.y     = _y;
    newPoint.theta = _theta;

    for(int idx=0; idx < point_num; idx++){
        odoPoints.push_back(newPoint);
    }
}

OPoints OdometryMotion::Prediction(float _x, float _y, float _theta){
    float delta_rot1, delta_trans, delta_rot2;
    float GNdelta_rot1, GNdelta_trans, GNdelta_rot2;

    for(unsigned int idx=0; idx < odoPoints.size(); idx++){
        delta_rot1      = atan2(_y - odoPoints[idx].y, _x - odoPoints[idx].x) - odoPoints[idx].theta;
        delta_trans     = sqrt(pow( _x - odoPoints[idx].x, 2) + pow(_y - odoPoints[idx].y, 2));
        delta_rot2      = _theta - odoPoints[idx].theta - delta_rot1;

        // 가우시안 노이즈를 추가한 값
        GNdelta_rot1    = delta_rot1 + gaussian.Generate(0, alpha1*pow(delta_rot1, 2) + alpha2*pow(delta_trans, 2));
        GNdelta_trans   = delta_trans + gaussian.Generate(0, alpha3*pow(delta_trans, 2)  + alpha4*(pow(delta_rot1, 2)+pow(delta_rot2, 2)));
        GNdelta_rot2    = delta_rot2 + gaussian.Generate(0, alpha1*pow(delta_rot2, 2) + alpha2*pow(delta_trans, 2));

        // 위의 값을 이용해 prediction을 완료한다.
        odoPoints[idx].x        += GNdelta_trans * cos(odoPoints[idx].theta + GNdelta_rot1);
        odoPoints[idx].y        += GNdelta_trans * sin(odoPoints[idx].theta + GNdelta_rot1);
        odoPoints[idx].theta    += GNdelta_rot1 + GNdelta_rot2;
    }

    return odoPoints;
}

double OdometryMotion::gaussianGenerate(const double &dMean, const double &dSigma)
{
    return gaussian.Generate(dMean, dSigma);
}
