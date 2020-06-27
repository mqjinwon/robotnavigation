#ifndef RENDERAREA_H
#define RENDERAREA_H

#include <QWidget>
#include "QPainter"

class RenderArea : public QWidget
{
    Q_OBJECT

public:
    enum Shape { Line, Points, Polyline, Polygon, Rect, RoundedRect, Ellipse, Arc,
                 Chord, Pie, Path, Text, Pixmap };

    QPolygonF polygon;

    QRectF rect;

    QPainterPath path;

    int startAngle  = 0;
    int arcLength   = 0;

    explicit RenderArea(QWidget *parent = nullptr);

public slots:
    void setShape(Shape shape);
    void setPen(const QPen &pen);
    void setBrush(const QBrush &brush);
    void setAntialiased(bool antialiased);
    void setTransformed(bool transformed);

protected:
    void paintEvent(QPaintEvent *event);

private:
    Shape shape;
    QPen pen;
    QBrush brush;
    bool antialiased    = false;
    bool transformed    = false;
    QPixmap pixmap;
};

#endif // RENDERAREA_H
