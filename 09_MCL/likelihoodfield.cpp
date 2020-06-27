#include "likelihoodfield.h"

void LikelihoodField::initialize()
{
    zHit            = 0.99;
    zRand           = 0.01;
    rhoHit          = 0.3;
    zMax            = 3;
    pixelDistance   = 0.05;
    gaussian.OnRandom(100000);
}

KImageGray* LikelihoodField::likeLihood_field_range_finder_model(){
    _LHFMap = _BFimg;

    double maxQ = 0;

    // likelihood probability map 생성
    for(int row=0; row < _BFimg.Row(); row++){
        for(int col=0; col< _BFimg.Col(); col++){
            double q = 1;
            q = q * (zHit * GaussianProb(_BFimg[row][col] * pixelDistance, 0, rhoHit) + zRand/zMax );

            if(maxQ < q)    maxQ = q;

            _LHFMap[row][col] = q * 255;
        }
    }
//    // 이미지 크기 정규화 과정
//    for(int row=0; row < _BFimg.Row(); row++){
//        for(int col=0; col< _BFimg.Col(); col++){
//            _LHFMap[row][col] = _LHFMap[row][col] / maxQ;
//        }
//    }

//#if 1
//    for(int i=0; i< _Obstacles.size(); i++){
//        for(int j=0; j < _Obstacles[i].size(); j++){
//            _LHFMap[_Obstacles[i][j].first][_Obstacles[i][j].second] = 0;
//        }
//    }
//#endif

    return &_LHFMap;
}

//정규분포확률 구하는 함수
double LikelihoodField::GaussianProb(double x, double mean, double sigma){
    double Probability=0;
    Probability = pow(M_E, -pow(x-mean, 2)/ pow(sigma, 2) )/(M_PI * 2 *sigma);

    return Probability;
}
