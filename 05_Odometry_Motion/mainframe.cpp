#include <QFileDialog>
#include <QPainter>
#include <QDebug>

#include "mainframe.h"
#include "ui_mainframe.h"
#include "imageform.h"




MainFrame::MainFrame(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::MainFrame)
{
    ui->setupUi(this);

    _plpImageForm       = new KPtrList<ImageForm*>(10,false,false);
    _q_pFormFocused     = 0;

    //객체 맴버의 초기화

    //get a current directory
    char st[100];
    GetCurrentDirectoryA(100,st);

    //리스트 출력창을 안보이게    
    ui->listWidget->setVisible(false);
    this->adjustSize();


    //UI 활성화 갱신
    UpdateUI();
}

MainFrame::~MainFrame()
{ 
    delete ui;         
    delete _plpImageForm;

}

void MainFrame::CloseImageForm(ImageForm *pForm)
{
    //ImageForm 포인터 삭제
    _plpImageForm->Remove(pForm);

    //활성화 ImageForm 초기화
    _q_pFormFocused     = 0;

    //UI 활성화 갱신
    UpdateUI();
}

void MainFrame::UpdateUI()
{    
    if(ui->tabWidget->currentIndex() == 0) //rn1
    {

    }
    else if(ui->tabWidget->currentIndex() == 1) //rn2
    {

    }

}

void MainFrame::OnMousePos(const int &nX, const int &nY, ImageForm* q_pForm)
{
    mouseClickCnt++;

    QPair<int,int> tmpPos;
    tmpPos.first                = nX;
    tmpPos.second               = nY;

    if(mousePos.size() > 10){
        mousePos.pop();
    }
    mousePos.push(tmpPos);

    qDebug() << nX << ", " << nY;


    UpdateUI();
}

void MainFrame::closeEvent(QCloseEvent* event)
{
    //생성된 ImageForm을 닫는다.
    for(int i=_plpImageForm->Count()-1; i>=0; i--)
        _plpImageForm->Item(i)->close();

    //리스트에서 삭제한다.
    _plpImageForm->RemoveAll();
}


void MainFrame::on_buttonOpen_clicked()
{
    //이미지 파일 선택
    QFileDialog::Options    q_Options   =  QFileDialog::DontResolveSymlinks  | QFileDialog::DontUseNativeDialog; // | QFileDialog::ShowDirsOnly
    QString                 q_stFile    =  QFileDialog::getOpenFileName(this, tr("Select a Image File"),  "../", "Image Files(*.bmp *.ppm *.pgm *.png)",0, q_Options);

    if(q_stFile.length() == 0)
        return;

    //이미지 출력을 위한 ImageForm 생성    
    ImageForm*              q_pForm   = new ImageForm(q_stFile, "SOURCE", this);

    _plpImageForm->Add(q_pForm);
    q_pForm->show();

    //UI 활성화 갱신
    UpdateUI();
}

void MainFrame::on_buttonDeleteContents_clicked()
{
    //생성된 ImageForm을 닫는다.
    for(int i=_plpImageForm->Count()-1; i>=0; i--)
        _plpImageForm->Item(i)->close();

    //리스트에서 삭제한다.
    _plpImageForm->RemoveAll();
}

void MainFrame::on_tabWidget_currentChanged(int index)
{
    static int nOld = -1;

    if(nOld == 0)
    {

    }
    else if(nOld == 1)
    {

    }
    nOld = index;

    //UI 활성화 갱신
    UpdateUI();
}

void MainFrame::on_buttonShowList_clicked()
{
    static int nWidthOld = ui->tabWidget->width();

    if(ui->listWidget->isVisible())
    {
        nWidthOld = ui->listWidget->width();
        ui->listWidget->hide();
        this->adjustSize();
    }
    else
    {        
        ui->listWidget->show();
        QRect q_rcWin = this->geometry();

        this->setGeometry(q_rcWin.left(), q_rcWin.top(), q_rcWin.width()+nWidthOld, q_rcWin.height());
    }
}

void MainFrame::on_buttonAStar_clicked()
{
    ui->listWidget->clear();                                   // erase all list

    QString stSource = ui->editSourceNode->text();
    QString stGoal   = ui->editGoalNode->text();

    opAStar = new nvimap(30);

    opAStar->AddNode(1, 5, "A");    opAStar->AddNode(1, 2, "B");
    opAStar->AddNode(3, 4, "C");    opAStar->AddNode(3, 6, "D");
    opAStar->AddNode(2, 7, "E");    opAStar->AddNode(3, 1, "F");
    opAStar->AddNode(4, 3, "G");    opAStar->AddNode(5, 5, "H");
    opAStar->AddNode(6, 2, "I");    opAStar->AddNode(7, 0, "J");
    opAStar->AddNode(8, 3, "K");    opAStar->AddNode(8, 5, "L");
    opAStar->AddNode(7, 7, "M");    opAStar->AddNode(9, 7, "N");
    opAStar->AddNode(10, 5, "O");   opAStar->AddNode(9, 1, "P");
    opAStar->AddNode(10, 3, "Q");

    opAStar->AddNeighbor("A", "B"); opAStar->AddNeighbor("A", "C"); opAStar->AddNeighbor("A", "E");
    opAStar->AddNeighbor("B", "F");
    opAStar->AddNeighbor("C", "B"); opAStar->AddNeighbor("C", "G"); opAStar->AddNeighbor("C", "H");
    opAStar->AddNeighbor("D", "C"); opAStar->AddNeighbor("D", "H"); opAStar->AddNeighbor("D", "M");
    opAStar->AddNeighbor("E", "D"); opAStar->AddNeighbor("E", "M");
    opAStar->AddNeighbor("F", "I"); opAStar->AddNeighbor("F", "J");
    opAStar->AddNeighbor("G", "F"); opAStar->AddNeighbor("G", "I");
    opAStar->AddNeighbor("H", "I"); opAStar->AddNeighbor("H", "K"); opAStar->AddNeighbor("H", "L");
    opAStar->AddNeighbor("I", "P"); opAStar->AddNeighbor("I", "K");
    opAStar->AddNeighbor("J", "K"); opAStar->AddNeighbor("J", "P");
    opAStar->AddNeighbor("K", "P"); opAStar->AddNeighbor("K", "O");
    opAStar->AddNeighbor("L", "K"); opAStar->AddNeighbor("L", "Q");
    opAStar->AddNeighbor("M", "L"); opAStar->AddNeighbor("M", "N");
    opAStar->AddNeighbor("N", "L"); opAStar->AddNeighbor("N", "O");
    opAStar->AddNeighbor("O", "K"); opAStar->AddNeighbor("O", "Q");
    opAStar->AddNeighbor("P", "K"); opAStar->AddNeighbor("P", "Q");

    JNODE* result = opAStar->Execute(stSource.toStdString().c_str(), stGoal.toStdString().c_str());
    string route;


    while(1){
        route += result->szID[0];
        if(result->opPrvious == nullptr){
            break;
        }

        route += " <-- ";
        result = result->opPrvious;

    }

    ui->listWidget->insertItem(0, QString().sprintf("%s ", route.c_str()));

    //UI 활성화 갱신
    UpdateUI();
}

void MainFrame::on_visibilityMap_clicked()
{
    //get file name
    QFileDialog::Options    q_Options   =  QFileDialog::DontResolveSymlinks  | QFileDialog::DontUseNativeDialog; // | QFileDialog::ShowDirsOnly
    QString                 q_stFile    =  QFileDialog::getOpenFileName(this, tr("Select a Image File"),  "../02_Visibility_Graph", "Image Files(*.txt)",0, q_Options);

    _opVG = new VisibilityGraph(q_stFile.toStdString().c_str(), 50);

    ImageForm* q_pForm = 0;
    for(int i=0; i<_plpImageForm->Count(); i++){
        if((*_plpImageForm)[i]->ID() == "VG"){
            q_pForm = (*_plpImageForm)[i];
            break;
        }
    }

    if(q_pForm == 0){
        q_pForm = new ImageForm(KImageColor(_opVG->Height(), _opVG->Width()), "VG", this);
        _plpImageForm->Add(q_pForm);
    }

    // Map 표시
    QPolygonF           q_Polygon;
    vector<JPolygon*>&  lPolygon = _opVG->Polygons();
    QColor              Q_colorBrush(255, 255, 255);

    for(int p=0; p<lPolygon.size(); p++){
        q_Polygon.clear();

        for(int i=0; i<lPolygon[p]->size(); i++){
            q_Polygon << QPointF((float)(*lPolygon[p])[i].first, (float)(*lPolygon[p])[i].second);
        }

        if(p == lPolygon.size()-1){
            q_pForm->DrawPolygon(q_Polygon, QColorConstants::White, 1);
        }
        else{
            q_pForm->DrawPolygon(q_Polygon, QColorConstants::White, 1, &Q_colorBrush);
        }
    }

    //visibility Graph 표시----------------------------------------------------------------------
    vector<JNODE*> plpGraph = _opVG->navigationMap;
    for(int i=0; i<plpGraph.size(); i++){
        JNODE* pNode = plpGraph[i];
        JNODE* pNeighbor;

        for(int n=0; n<pNode->nNeighbors; n++){
            pNeighbor = _opVG->FindNode(pNode->szpNeighbor[n]);
            q_pForm->DrawLine(pNode->nX, pNode->nY, pNeighbor->nX, pNeighbor->nY, QColorConstants::Green, 1);
        }
    }

    q_pForm->update();
    q_pForm->show();
    UpdateUI();

}

void MainFrame::on_visibilityAstar_clicked()
{
    if(mouseClickCnt >= 2){
        mouseClickCnt -= 2;

        ImageForm* q_pForm = 0;
        for(int i=0; i<_plpImageForm->Count(); i++){
            if((*_plpImageForm)[i]->ID() == "VG"){
                q_pForm = (*_plpImageForm)[i];
                break;
            }
        }

        if(q_pForm == 0){
            q_pForm = new ImageForm(KImageColor(_opVG->Height(), _opVG->Width()), "VG", this);
            _plpImageForm->Add(q_pForm);
        }

        // 시작점 끝점 그리기
        QPolygon            q_Polygon;
        QColor              Q_colorBrush(255, 255, 255);                            //White

        QPair<int,int>  startPos(mousePos.front());         mousePos.pop();
        QPair<int,int>  endPos(mousePos.front());           mousePos.pop();

        _opVG->AddNode(startPos.first, startPos.second, "start");
        _opVG->AddNeigborTo("start");

        q_Polygon << QPoint(startPos.first, startPos.second);
        q_pForm->DrawPoints(q_Polygon, QColorConstants::Green, 15);

        _opVG->AddNode(endPos.first, endPos.second, "end");
        _opVG->AddNeigborTo("end");

        q_Polygon.clear();
        q_Polygon << QPoint(endPos.first, endPos.second);
        q_pForm->DrawPoints(q_Polygon, QColorConstants::Red, 15);

        // 시작점과 끝점들의 인근 접점들을 그림으로 그려주기
        {
            JNODE* pNode = _opVG->FindNode("start");
            JNODE* pNeighbor;

            for(int n=0; n<pNode->nNeighbors; n++){
                pNeighbor = _opVG->FindNode(pNode->szpNeighbor[n]);
                q_pForm->DrawLine(pNode->nX, pNode->nY, pNeighbor->nX, pNeighbor->nY, QColorConstants::Green, 1);
            }

            pNode = _opVG->FindNode("end");

            for(int n=0; n<pNode->nNeighbors; n++){
                pNeighbor = _opVG->FindNode(pNode->szpNeighbor[n]);
                q_pForm->DrawLine(pNode->nX, pNode->nY, pNeighbor->nX, pNeighbor->nY, QColorConstants::Green, 1);
            }
        }

        // A star 알고리즘을 사용하고, 최단경로를 노란색으로 칠해주기
        JNODE* result = _opVG->Execute("start", "end");

        while(1){
            if(result->opPrvious == nullptr){
                break;
            }
            q_pForm->DrawLine(result->nX, result->nY, result->opPrvious->nX, result->opPrvious->nY, QColorConstants::Yellow, 3);
            result = result->opPrvious;
        }

        q_pForm->update();
        UpdateUI();
    }

    qDebug() << "mouse counts are: " << mouseClickCnt;


}

void MainFrame::on_voronoiDiagram_clicked()
{
    // make bmp image window
    ImageForm* q_pForm = 0;
    for(int i=0; i < _plpImageForm->Count(); i++){
        if((*_plpImageForm)[i]->ID() == "SOURCE"){
            q_pForm = (*_plpImageForm)[i];
            break;
        }
    }

    if(q_pForm == 0)
        return;

    int thershold   = 1;
    int scale       = 5;

    VRG = new VoronoiGraph(q_pForm->ImageGray());

    QTime startTime = QTime::currentTime();
    ImageForm* BFimg = 0;
    for(int i=0; i < _plpImageForm->Count(); i++){
        if((*_plpImageForm)[i]->ID() == "BRUSHFIRE"){
            BFimg = (*_plpImageForm)[i];
            break;
        }
    }

    if(BFimg == 0)
        BFimg = new ImageForm(VRG->Execute(BACKGROUND, FOURWAY, thershold, scale), "BRUSHFIRE", this);
    _plpImageForm->Add(BFimg);

    BFimg->update();
    BFimg->show();
    qDebug() << "Voronoi algorithm time:" << startTime.elapsed();



    q_pForm->update();
    q_pForm->show();
    UpdateUI();
}

void MainFrame::on_voronoiAstar_clicked()
{
    if(mouseClickCnt >= 2){

        QTime startTime = QTime::currentTime();

        qDebug() << "mouse counts are: " << mouseClickCnt;
        mouseClickCnt -= 2;

        ImageForm* BFimg = 0;
        bool isRefer(false);
        for(int i=0; i < _plpImageForm->Count(); i++){
            if((*_plpImageForm)[i]->ID() == "BRUSHFIRE"){
                BFimg = (*_plpImageForm)[i];
                isRefer = true;
                break;
            }
        }

        // 빈 이미지를 만들기 위해 사용
        if(isRefer){
            ImageForm* BFCimg = new ImageForm(BFimg->_igMain.GrayToRGB(), "BF_SHORT_PATH", this);
            _plpImageForm->Add(BFCimg);

            // 시작점 끝점 그리기
            QPolygon            q_Polygon;

            QPair<int,int>  startPos(mousePos.front());         mousePos.pop();
            QPair<int,int>  endPos(mousePos.front());           mousePos.pop();

            VRG->AddNode(startPos.first, startPos.second, "start");
            VRG->AddSENeigborTo("start");

            q_Polygon << QPoint(startPos.first, startPos.second);
            BFCimg->DrawPoints(q_Polygon, QColorConstants::Green, 7);

            VRG->AddNode(endPos.first, endPos.second, "end");
            VRG->AddSENeigborTo("end");

            q_Polygon.clear();
            q_Polygon << QPoint(endPos.first, endPos.second);
            BFCimg->DrawPoints(q_Polygon, QColorConstants::Red, 7);

            // A star 알고리즘을 사용하고, 최단경로를 노란색으로 칠해주기
            JNODE* result = VRG->nvimap::Execute("start", "end");

            while(1){
                if(result->opPrvious == nullptr){
                    break;
                }
                BFCimg->DrawLine(result->nX, result->nY, result->opPrvious->nX, result->opPrvious->nY, QColorConstants::Yellow, 3);
                result = result->opPrvious;
            }

            BFCimg->update();
            BFCimg->show();
            UpdateUI();
        }
        qDebug() << "Astar algorithm time:" << startTime.elapsed();
    }

}

void MainFrame::on_odometryMotion_clicked()
{
    RenderArea* odoImg = new RenderArea();
    odoImg->setStyleSheet("background:rgb(0,0,0)");
    odoImg->resize(500, 500);

    QPen pen;
    pen.setColor(QColor(255, 255, 255));
    pen.setWidth(1);

    odoImg->setPen(pen);

    odoImg->setShape(odoImg->Points);

    if(odometry != nullptr)
        delete odometry;

    QPolygonF q_Polygon;

    odometry = new OdometryMotion(1000, 180, 180, 0);
    odometry->setVarianceParam(0.00002,0.00002,0.0001,0.0001);

    for(int idx=0; idx<odometry->odoPoints.size(); idx++){
        q_Polygon << QPoint(odometry->odoPoints[idx].x, odometry->odoPoints[idx].y);
    }

    OPoints points;

    points = odometry->Prediction(230, 180, 0);
    for(int idx=0; idx<points.size(); idx++){
        q_Polygon << QPoint(points[idx].x, points[idx].y);
    }

    points = odometry->Prediction(280, 180, 0);
    for(int idx=0; idx<points.size(); idx++){
        q_Polygon << QPoint(points[idx].x, points[idx].y);
    }

    points = odometry->Prediction(330, 180, 0);
    for(int idx=0; idx<points.size(); idx++){
        q_Polygon << QPoint(points[idx].x, points[idx].y);
    }

    points = odometry->Prediction(330, 230, 90);
    for(int idx=0; idx<points.size(); idx++){
        q_Polygon << QPoint(points[idx].x, points[idx].y);
    }

    points = odometry->Prediction(330, 280, 90);
    for(int idx=0; idx<points.size(); idx++){
        q_Polygon << QPoint(points[idx].x, points[idx].y);
    }

    points = odometry->Prediction(330, 330, 90);
    for(int idx=0; idx<points.size(); idx++){
        q_Polygon << QPoint(points[idx].x, points[idx].y);
    }

    points = odometry->Prediction(280, 330, 180);
    for(int idx=0; idx<points.size(); idx++){
        q_Polygon << QPoint(points[idx].x, points[idx].y);
    }

    points = odometry->Prediction(230, 330, 180);
    for(int idx=0; idx<points.size(); idx++){
        q_Polygon << QPoint(points[idx].x, points[idx].y);
    }

    points = odometry->Prediction(180, 330, 180);
    for(int idx=0; idx<points.size(); idx++){
        q_Polygon << QPoint(points[idx].x, points[idx].y);
    }

    points = odometry->Prediction(130, 330, 180);
    for(int idx=0; idx<points.size(); idx++){
        q_Polygon << QPoint(points[idx].x, points[idx].y);
    }

    points = odometry->Prediction(80, 330, 180);
    for(int idx=0; idx<points.size(); idx++){
        q_Polygon << QPoint(points[idx].x, points[idx].y);
    }

    odoImg->polygon = q_Polygon;
    odoImg->update();
    odoImg->show();

}
