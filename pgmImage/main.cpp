#include "pgmimg.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    pgmImg w;
    w.show();
    return a.exec();
}
