#ifndef VORONOIGRAPH_H
#define VORONOIGRAPH_H
#include "AstarAlgorithm.h"
#include <kfc.h>
#include <vector>
#include <queue>

typedef pair<int, int>      Opoint;
typedef vector<Opoint>      Obstacle;
typedef vector<Obstacle>    Obstacles;

enum TYPE{
    FOREGROUND = 0,
    BACKGROUND,
    FOURWAY,
    EIGHTWAY
};

class VoronoiGraph : public nvimap{

private:

//    Obstacle        _Obstacle;
    Obstacles       _Obstacles;
    KImageGray      _igMap;
    KImageGray      _BFimg;             //brush fire img

    //{row, col} clockwise from right
    int             fourWay[4][2]   = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};
    int             eightWay[8][2]  = {{0, 1}, {1, 1}, {1, 0}, {1, -1},
                                                {0, -1}, {-1, -1}, {-1, 0}, {-1, 1}};

    struct brushNode{
        Opoint      BCodin;
        int         Bvalue;

        // 아마 쓸일 없을듯
        brushNode*  BPrvious;
    };

public:
    VoronoiGraph();
    VoronoiGraph(const KImageGray& igMap){
        _igMap = igMap;
    }

    KImageGray Execute(int obType, int bfType, int thershold=3, int scale=3);

    /* FOREGROUND
     * - object(obstacle) : 255
     * - background       : 0
     * ELSE
     * - object(obstacle) : 0
     * - background       : 255
     */
    Obstacles   getObstacles(int type);

    /* FOURWAY
     * - find 4way method
     * EIGHTWAY
     * - find 8way method
    */
    void        brushFire(int type, int threshold=3);

    KImageGray  scaleUp(const KImageGray& igMap, int scale=3);

    KImageGray  getigMap(void)  {return _igMap;}
};

#endif // VORONOIGRAPH_H
