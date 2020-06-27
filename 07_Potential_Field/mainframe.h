#ifndef MAINFRAME_H
#define MAINFRAME_H

#include <QDialog>
#include <QTime>
#include "kfc.h"
#include <queue>
#include "AstarAlgorithm.h"
#include "VisibilityGraph.h"
#include "Voronoigraph.h"
#include "odometrymotion.h"
#include "renderarea.h"
#include "likelihoodfield.h"
#include "potentialfield.h"

namespace Ui {
class MainFrame;

}

class ImageForm;
class KVoronoiGraph;
class KVisibilityGraph;
class KPotentialField;

class MainFrame : public QDialog
{
    Q_OBJECT

private:
    Ui::MainFrame *ui;

    KPtrList<ImageForm*>*           _plpImageForm;
    ImageForm*                      _q_pFormFocused;

    // defined by jinwon
    nvimap*                         opAStar         = nullptr;
    VisibilityGraph*                _opVG           = nullptr;
    VoronoiGraph*                   VRG             = nullptr;
    OdometryMotion*                 odometry        = nullptr;
    LikelihoodField*                LHF             = nullptr;
    PotentialField*                 PTF             = nullptr;
    queue<QPair<int,int>>           mousePos;
    int                             mouseClickCnt   = 0;

public:
    explicit MainFrame(QWidget *parent = 0);
    ~MainFrame();

    void            ImageFormFocused(ImageForm* q_pImageForm)
                    {   _q_pFormFocused  = q_pImageForm;   //활성화된 창의 포인터를 저장함
                        UpdateUI();                        //UI 활성화 갱신
                    }
    void            UpdateUI();
    void            CloseImageForm(ImageForm* pForm);

public:
    void            OnMousePos(const int& nX, const int& nY, ImageForm* q_pForm);

private slots:
    void on_buttonOpen_clicked();
    void on_buttonDeleteContents_clicked();        
    void on_tabWidget_currentChanged(int index);           
    void on_buttonAStar_clicked();    
    void on_buttonShowList_clicked();

    void on_visibilityMap_clicked();

    void on_visibilityAstar_clicked();

    void on_voronoiDiagram_clicked();

    void on_voronoiAstar_clicked();

    void on_odometryMotion_clicked();

    void on_likelihoodField_clicked();

    void on_PotentialField_clicked();

protected:
    void closeEvent(QCloseEvent* event);
};

#endif // MAINFRAME_H
