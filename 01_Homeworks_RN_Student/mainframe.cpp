#include <QFileDialog>
#include <QPainter>

#include "mainframe.h"
#include "ui_mainframe.h"
#include "imageform.h"
#include "AstarAlgorithm.h"


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
    QString                 q_stFile    =  QFileDialog::getOpenFileName(this, tr("Select a Image File"),  "./data", "Image Files(*.bmp *.ppm *.pgm *.png)",0, q_Options);

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

    nvimap* opAStar = new nvimap(30);

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

    KNODE* result = opAStar->Execute(stSource.toStdString().c_str(), stGoal.toStdString().c_str());
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
