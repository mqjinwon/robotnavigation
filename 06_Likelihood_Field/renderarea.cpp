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
        painter.drawLine(rect.bottomLeft(), rect.topRight());
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
        painter.drawRect(rect);
        break;
    case RoundedRect:
        painter.drawRoundedRect(rect, 25, 25, Qt::RelativeSize);
        break;
    case Ellipse:
        painter.drawEllipse(rect);
        break;
    case Arc:
        painter.drawArc(rect, startAngle, arcLength);
        break;
    case Chord:
        painter.drawChord(rect, startAngle, arcLength);
        break;
    case Pie:
        painter.drawPie(rect, startAngle, arcLength);
        break;
    case Path:
        painter.drawPath(path);
        break;
    case Text:
        painter.drawText(rect,
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
