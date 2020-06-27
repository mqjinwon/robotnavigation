#include "pgmimg.h"
#include "ui_pgmimg.h"

pgmImg::pgmImg(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::pgmImg)
{
    ui->setupUi(this);

}

pgmImg::~pgmImg()
{
    delete ui;
}

void pgmImg::setShape(Shape shape)
{
    this->shape = shape;
    update();
}

void pgmImg::setPen(const QPen &pen)
{
    this->pen = pen;
    update();
}

void pgmImg::setBrush(const QBrush &brush)
{
    this->brush = brush;
    update();
}

void pgmImg::setAntialiased(bool antialiased)
{
    this->antialiased = antialiased;
    update();
}

void pgmImg::setTransformed(bool transformed)
{
    this->transformed = transformed;
    update();
}

void pgmImg::paintEvent(QPaintEvent *event){
    Q_UNUSED(event);

    QPainter painter(this);
    painter.setPen(pen);
    painter.setBrush(brush);
    if (antialiased)
        painter.setRenderHint(QPainter::Antialiasing, true);

    painter.save();

    if (transformed) {
        painter.translate(50, 50);
        painter.rotate(60.0);
        painter.scale(0.6, 0.9);
        painter.translate(-50, -50);
    }

    switch (shape) {
    case Line:
        for(auto& i:lines[0]){
            pen.setColor(QColor(255, 0, 0));
            painter.setPen(pen);
            painter.drawLine(i);
        }
        for(auto& i:lines[1]){
            pen.setColor(QColor(0, 255, 0));
            painter.setPen(pen);
            painter.drawLine(i);
        }
        for(auto& i:lines[2]){
            pen.setColor(QColor(0, 255, 255));
            painter.setPen(pen);
            painter.drawLine(i);
        }
        for(auto& i:lines[3]){
            pen.setColor(QColor(255, 255, 255));
            painter.setPen(pen);
            painter.drawLine(i);
        }
        for(auto& i:lines[4]){
            pen.setColor(QColor(255, 0, 255));
            painter.setPen(pen);
            painter.drawLine(i);
        }
        break;
    case Points:
        painter.drawPoints(polygon);
        break;
    case Polyline:
        painter.drawPolyline(polygon);
        break;
    case Polygon:
        painter.drawPolygon(polygon);
        break;
    case Rect:
        for(auto& i:rect)
            painter.drawRect(i);
        break;
    case RoundedRect:
        for(auto& i:rect)
            painter.drawRoundedRect(i, 25, 25, Qt::RelativeSize);
        break;
    case Ellipse:
        for(auto& i:rect)
            painter.drawEllipse(i);
        break;
    case Arc:
        for(auto& i:rect)
            painter.drawArc(i, startAngle, arcLength);
        break;
    case Chord:
        for(auto& i:rect)
            painter.drawChord(i, startAngle, arcLength);
        break;
    case Pie:
        for(auto& i:rect)
            painter.drawPie(i, startAngle, arcLength);
        break;
    case Path:
        painter.drawPath(path);
        break;
    case Text:
        for(auto& i:rect)
            painter.drawText(i,
                         Qt::AlignCenter,
                         tr("Qt by\nThe Qt Company"));
        break;
    case Pixmap:
        painter.drawPixmap(10, 10, pixmap);
    }
    painter.restore();


    painter.setRenderHint(QPainter::Antialiasing, false);
    painter.setPen(palette().dark().color());
    painter.setBrush(Qt::NoBrush);
    painter.drawRect(QRect(0, 0, width() - 1, height() - 1));

}

void pgmImg::on_makeImg_clicked()
{
    int Row = 200, Col = 200;
    int Black = 0, White = 1;
    QImage img(QSize(Row, Col), QImage::Format_Mono);

    //initialize
    for(int r = 0; r < Row; r++){
        for(int c = 0; c < Col; c++){
            img.setPixel(c, r, White);
        }
    }

    QPainter img_painter(&img);
    QBrush brush;
    brush.setColor(Qt::black);
    brush.setStyle(Qt::SolidPattern);
    pen.setBrush(brush);
    img_painter.setPen(pen);

    // Fill polygon
    QPainterPath path;
    QRect rect;
    rect.setRect(0,0, 199, 9);
    path.addRect(rect);

    // Draw polygon
    img_painter.drawRect(rect);
    img_painter.fillPath(path, brush);

    rect.setRect(0,9, 9, 199);
    path.addRect(rect);

    // Draw polygon
    img_painter.drawRect(rect);
    img_painter.fillPath(path, brush);

    rect.setRect(10,189, 199, 199);
    path.addRect(rect);

    // Draw polygon
    img_painter.drawRect(rect);
    img_painter.fillPath(path, brush);

    rect.setRect(189,10, 199, 179);
    path.addRect(rect);

    // Draw polygon
    img_painter.drawRect(rect);
    img_painter.fillPath(path, brush);

    rect.setRect(29,29, 20, 30);
    path.addRect(rect);

    // Draw polygon
    img_painter.drawRect(rect);
    img_painter.fillPath(path, brush);

    rect.setRect(119,29, 50, 30);
    path.addRect(rect);

    // Draw polygon
    img_painter.drawRect(rect);
    img_painter.fillPath(path, brush);

    rect.setRect(69,49, 20, 80);
    path.addRect(rect);

    // Draw polygon
    img_painter.drawRect(rect);
    img_painter.fillPath(path, brush);

    rect.setRect(139,79, 20, 20);
    path.addRect(rect);

    // Draw polygon
    img_painter.drawRect(rect);
    img_painter.fillPath(path, brush);

    rect.setRect(149,99, 20, 30);
    path.addRect(rect);

    // Draw polygon
    img_painter.drawRect(rect);
    img_painter.fillPath(path, brush);

    rect.setRect(29,129, 30, 30);
    path.addEllipse(rect);

    // Draw polygon
    img_painter.drawEllipse(rect);
    img_painter.fillPath(path, brush);

    rect.setRect(119,139, 30, 30);
    path.addEllipse(rect);

    // Draw polygon
    img_painter.drawEllipse(rect);
    img_painter.fillPath(path, brush);




    img.save("hi.bmp");
    img.save("hi.pgm");

}
