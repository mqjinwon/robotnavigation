#include "imageform.h"
#include "ui_imageform.h"
#include "mainframe.h"

ImageForm::ImageForm(const QString& q_stFile, const QString& q_stID, MainFrame* owner) :  QWidget(0), ui(new Ui::ImageForm)
{
    ui->setupUi(this);

    this->setEnabled(true);

    //ImageForm의 ID
    _q_stID              = q_stID;
    setWindowTitle(KString(q_stFile.toStdString().c_str()).FileNameWithExt().Address());

    //MainForm의 주소를 저장
    _q_MainFrame    = owner;

    //이미지 읽어들임
    _q_bmpMain   = new QImage;   //디스플레이를 위한 이미지 생성
    _q_bmpMain->load(q_stFile);

    //영상처리용 메모리와 공유 설정
    if(_q_bmpMain->format() == QImage::Format_Grayscale8)
        _igMain.Create(_q_bmpMain->height(),_q_bmpMain->width(),_q_bmpMain->bits(), _LOCK);       

    else if(_q_bmpMain->format() == QImage::Format_RGB32)
        _icMain.Create(_q_bmpMain->height(),_q_bmpMain->width(), (KCOLOR32*)_q_bmpMain->bits(), _LOCK);

    //ImageForm의 출력 위치 및 크기 설정
    static int   nXo, nYo=-30;
    QRect       q_rcGeom = _q_MainFrame->geometry();

    setGeometry(q_rcGeom.left() - _q_bmpMain->width() -  (nXo+= 20), q_rcGeom.top()  + (nYo += 30),
                        _q_bmpMain->width(),_q_bmpMain->height());
    focusInEvent(0);
}

ImageForm::ImageForm(const KImageColor& icImg, const QString& q_stID, MainFrame* owner) :  QWidget(0),  ui(new Ui::ImageForm)
{
    ui->setupUi(this);

    //ImageForm의 ID
    _q_stID              = q_stID;
    setWindowTitle(_q_stID);

    //MainForm의 주소를 저장
    _q_MainFrame    = owner;

    //이미지 읽어들임
    _q_bmpMain      = new QImage(icImg.Col(), icImg.Row(), QImage::Format_RGB32);
    memcpy(_q_bmpMain->bits(), icImg.Address(), icImg.Size() * sizeof(KCOLOR32));

    //영상처리용 메모리와 공유 설정
    _icMain.Create(_q_bmpMain->height(), _q_bmpMain->width(), (KCOLOR32*)_q_bmpMain->bits(), _LOCK);

    //ImageForm의 출력 위치 및 크기 설정
    static int   nXo, nYo=-30;
    QRect       q_rcGeom = _q_MainFrame->geometry();

    setGeometry(q_rcGeom.right() +  (nXo+= 20), q_rcGeom.top()  + (nYo += 30),
                        _q_bmpMain->width(),_q_bmpMain->height());
    focusInEvent(0);
}


ImageForm::ImageForm(const KImageGray& igImg, const QString& q_stID, MainFrame* owner) :  QWidget(0),  ui(new Ui::ImageForm)
{
    ui->setupUi(this);

    //ImageForm의 ID
    _q_stID              = q_stID;
    setWindowTitle(_q_stID);

    //MainForm의 주소를 저장
    _q_MainFrame    = owner;

    //이미지 읽어들임 & 칼라 테이블 설정
    _q_bmpMain      = new QImage( igImg.Col(), igImg.Row(), QImage::Format_Indexed8);
    memcpy(_q_bmpMain->bits(), igImg.Address(), igImg.Size());
    for(int i=0; i<256; i++)
    {
        QRgb value = qRgb(i,i,i);
        _q_bmpMain->setColor(i,value);
    }

    //영상처리용 메모리와 공유 설정    
    _igMain.Create(_q_bmpMain->height(), _q_bmpMain->width(), (uchar*)_q_bmpMain->bits(), _LOCK);

    //ImageForm의 출력 위치 및 크기 설정
    static int   nXo, nYo=-30;
    QRect       q_rcGeom = _q_MainFrame->geometry();

    setGeometry(q_rcGeom.right() +  (nXo+= 20), q_rcGeom.top()  + (nYo += 30),
                        _q_bmpMain->width(),_q_bmpMain->height());
    focusInEvent(0);
}


ImageForm::~ImageForm()
{
    delete ui;
}

void ImageForm::Update(const KImageGray& igImg)
{
    //이미지 갱신
    if(_igMain.Row() != igImg.Row() || _igMain.Col() != igImg.Col())
        return;

    memcpy(_q_bmpMain->bits(), igImg.Address(), igImg.Size());

    //표시
    repaint();
}

void ImageForm::Update(const KImageColor& icImg)
{
    //이미지 갱신
    if(_icMain.Row() != icImg.Row() || _icMain.Col() != icImg.Col())
        return;

    memcpy(_q_bmpMain->bits(), icImg.Address(), icImg.Size()*sizeof(KCOLOR32));

    //표시
    repaint();
}

void ImageForm::DrawEllipse(const QPoint& q_ptCenter,  int nRx, int nRy, const QColor& q_Color, int nWidth)
{
    QPainter  painter;
    QPen       pen;

    pen.setColor(q_Color);
    pen.setWidth(nWidth);

    painter.begin(_q_bmpMain);
    painter.setPen(pen);
    painter.drawEllipse(q_ptCenter, nRx, nRy);
    painter.end();

}

void ImageForm::DrawPolygon(const QPolygonF& q_Polygon, const QColor& q_Color, int nWidth, QColor* q_pColorBrush)
{
    QPainter  painter;
    QPen      pen;
    QBrush    brush;

    pen.setColor(q_Color);
    pen.setWidth(nWidth);

    if(q_pColorBrush)
    {
        brush.setColor(*q_pColorBrush);
        brush.setStyle(Qt::SolidPattern);
    }

    painter.begin(_q_bmpMain);
    painter.setPen(pen);
    if(q_pColorBrush)
        painter.setBrush(brush);
    painter.drawPolygon(q_Polygon);
    painter.end();
}

void ImageForm::DrawLines(const QPolygon& q_Polygon, const QColor& q_Color, int nWidth)
{
    QPainter  painter;
    QPen      pen;

    pen.setColor(q_Color);
    pen.setWidth(nWidth);

    painter.begin(_q_bmpMain);
    painter.setPen(pen);

    for(int i=0; i<q_Polygon.count()-1; i++)
        painter.drawLine(q_Polygon.point(i).x(),q_Polygon.point(i).y(),q_Polygon.point(i+1).x(),q_Polygon.point(i+1).y());
    painter.end();
}

void ImageForm::DrawPoints(const QPolygon& q_Polygon, const QColor& q_Color, int nWidth)
{
    QPainter  painter;    
    QPen       pen;

    pen.setColor(q_Color);
    pen.setWidth(nWidth);

    painter.begin(_q_bmpMain);
    painter.setPen(pen);
    painter.drawPoints(q_Polygon);
    painter.end();
}

void ImageForm::DrawLine(int nXo,int nYo, int nX1, int nY1,const QColor& q_Color, int nWidth)
{
    QPainter  painter;
    QPen       pen;

    pen.setColor(q_Color);
    pen.setWidth(nWidth);

    painter.begin(_q_bmpMain);
    painter.setPen(pen);
    painter.drawLine(nXo,nYo,nX1,nY1);
    painter.end();
}


void ImageForm::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event) ;

    QPainter  painter;
    painter.begin(this);
    painter.drawImage(0, 0, *_q_bmpMain);

    painter.end();
}

void ImageForm::focusInEvent(QFocusEvent * event)
{
    Q_UNUSED(event) ;

    _q_MainFrame->ImageFormFocused(this);
}

void ImageForm::focusOutEvent(QFocusEvent * event)
{
    Q_UNUSED(event) ;
}

void ImageForm::closeEvent(QCloseEvent *event)
{
    Q_UNUSED(event) ;
    _q_MainFrame->CloseImageForm(this);
}

void ImageForm::mousePressEvent(QMouseEvent *event)
{
    int x = event->x();
    int y = event->y();

    _q_MainFrame->OnMousePos(x, y, this);
}
