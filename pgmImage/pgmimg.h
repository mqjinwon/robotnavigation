#ifndef PGMIMG_H
#define PGMIMG_H

#include <QMainWindow>
#include <Qimage>
#include <QPainter>
#include <QPen>

QT_BEGIN_NAMESPACE
namespace Ui { class pgmImg; }
QT_END_NAMESPACE

class pgmImg : public QMainWindow
{
    Q_OBJECT

public:
    pgmImg(QWidget *parent = nullptr);
    ~pgmImg();

    enum Shape { Line, Points, Polyline, Polygon, Rect, RoundedRect, Ellipse, Arc,
                 Chord, Pie, Path, Text, Pixmap };

    QPolygonF polygon;

    QVector<QRectF> rect;

    QVector<QLine> lines[5];

    QPainterPath path;

    int startAngle  = 0;
    int arcLength   = 0;

public slots:
    void setShape(Shape shape);
    void setPen(const QPen &pen);
    void setBrush(const QBrush &brush);
    void setAntialiased(bool antialiased);
    void setTransformed(bool transformed);

protected:
    void paintEvent(QPaintEvent *event);

private slots:
    void on_makeImg_clicked();

private:
    Shape shape;
    QPen pen;
    QBrush brush;
    bool antialiased    = false;
    bool transformed    = false;
    QPixmap pixmap;
    Ui::pgmImg *ui;    


};
#endif // PGMIMG_H
