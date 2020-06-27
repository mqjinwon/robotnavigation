#ifndef ODESIM_H
#define ODESIM_H
#include <QThread>
#include "odefunction.h"

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif

extern dsFunctions fn;

class odeSim : public QThread{
public:
    odeSim();
    void run();
};

#endif // ODESIM_H
