#include "renderarea.h"

RenderArea::RenderArea(QWidget *parent) : QWidget(parent)
{
    shape = Polygon;
    antialiased = false;
    transformed = false;
    pixmap.load(":/images/qt-logo.png");

    setBackgroundRole(QPalette::Base);
    setAutoFillBackground(true);
}

void RenderArea::setShape(Shape shape)
{
    this->shape = shape;
    update();
}

void RenderArea::setPen(const QPen &pen)
{
    this->pen = pen;
    update();
}

void RenderArea::setBrush(const QBrush &brush)
{
    this->brush = brush;
    update();
}

void RenderArea::setAntialiased(bool antialiased)
{
    this->antialiased = antialiased;
    update();
}

void RenderArea::setTransformed(bool transformed)
{
    this->transformed = transformed;
    update();
}

void RenderArea::paintEvent(QPaintEvent *event){
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
            pen.setWidth(1);
            painter.setPen(pen);
            painter.drawLine(i);
        }
        for(auto& i:lines[1]){
            pen.setColor(QColor(0, 255, 0));
            pen.setWidth(1);
            painter.setPen(pen);
            painter.drawLine(i);
        }
        for(auto& i:lines[2]){
            pen.setColor(QColor(0, 255, 255));
            pen.setWidth(1);
            painter.setPen(pen);
            painter.drawLine(i);
        }
        for(auto& i:lines[3]){
            pen.setColor(QColor(255, 255, 255));
            pen.setWidth(1);
            painter.setPen(pen);
            painter.drawLine(i);
        }
        for(auto& i:lines[4]){
            pen.setColor(QColor(255, 0, 255));
            pen.setWidth(10);
            painter.setPen(pen);
            painter.drawLine(i);
        }

        break;
    case Points:

        pen.setColor(QColor(255, 255, 255));
        pen.setWidth(1);
        painter.setPen(pen);
        painter.drawPoints(polygon);

        pen.setColor(QColor(255, 0, 0));
        pen.setWidth(7);
        painter.setPen(pen);
        painter.drawPoints(polygon2);
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
