#-------------------------------------------------
#
# Project created by QtCreator 2014-09-29T17:12:01
#
#-------------------------------------------------

CONFIG  += c++11
QT       += core gui
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = homework
TEMPLATE = app

#for OpenGL
#LIBS        += -lopengl32\
#               -lglu32\
#               -lgdi32\
#               -lglut32\
#               -luser32\
#               -lwinmm

SOURCES += main.cpp\
    AstarAlgorithm.cpp \
    VisibilityGraph.cpp \
    Voronoigraph.cpp \
    likelihoodfield.cpp \
        mainframe.cpp \
    imageform.cpp \
    kfc.cpp \
    odefunction.cpp \
    odesim.cpp \
    odometrymotion.cpp \
    potentialfield.cpp \
    renderarea.cpp \
    sensorBeam.cpp

HEADERS  += mainframe.h \
    AstarAlgorithm.h \
    VisibilityGraph.h \
    Voronoigraph.h \
    imageform.h \
    kfc.h \
    likelihoodfield.h \
    odefunction.h \
    odesim.h \
    odometrymotion.h \
    potentialfield.h \
    renderarea.h \
    sensorBeam.h

FORMS    += mainframe.ui \
    imageform.ui

RESOURCES += \
    images/mainframe.qrc

# For ODE Library
DEFINES += dDOUBLE
INCLUDEPATH += C:\\ode-0.13\\include
INCLUDEPATH += C:\\ode-0.13\\drawstuff
INCLUDEPATH += C:\\ode-0.13\\ode\\src
# For ODE Library
LIBS += C:\ode-0.13\lib\ReleaseDoubleLib\libode_double.a
LIBS += C:\ode-0.13\lib\ReleaseDoubleLib\libdrawstuff.a
# For OpenGL Library
LIBS += -lopengl32\
-lglu32\
-lgdi32\
-lglut32\
-luser32\
-lwinmm\

# For Windows Resource File
win32{ RC_FILE = C:\\ode-0.13\\drawstuff\\src\\resources.rc }
