#ifndef VISIBILITYGRAPH_H
#define VISIBILITYGRAPH_H

#include "AstarAlgorithm.h"

typedef vector<JPoint>        JPolygon;

class VisibilityGraph : public nvimap{

    int                     _nWidth, _nHeight;
    vector<JPolygon*>*      _plpObstacle            = nullptr;

public:

    VisibilityGraph(){
    };
    VisibilityGraph(const char* szMapText, int mapsize) : nvimap(mapsize){
        Create(szMapText);
    }
    ~VisibilityGraph(){
        if(_plpObstacle){
            delete _plpObstacle;
        }
    }

    bool Create(const char* szMapText);
    int Width(){return _nWidth;}
    int Height(){return _nHeight;}
    vector<JPolygon*>& Polygons(){return *_plpObstacle;}

    bool AddNeigborTo(const char* szID);                                                                                // add neighbor from ID

protected:
    bool                BelongToSamePolygon(int i, int j);                                                              // is point i and point j belong to same polygon?
    bool                Visibility(int i, int j);                                                                       // can point i see point j?

    int                 AreaSign(const JPoint& A, const JPoint& B, const JPoint& C);
    bool                Right(const JPoint& A, const JPoint& B, const JPoint& C) { return AreaSign(A, B, C) > 0; }
    bool                Collinear(const JPoint& A, const JPoint& B, const JPoint& C);
    bool                Intersected(const JPoint& A, const JPoint& B, const JPoint& C, const JPoint& D);

};



#endif // VISIBILITYGRAPH_H
