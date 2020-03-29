#-------------------------------------------------
#
# Project created by QtCreator 2014-09-29T17:12:01
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = homework
TEMPLATE = app

#for OpenGL
LIBS        += -lopengl32\
               -lglu32\
               -lgdi32\
               -lglut32\
               -luser32\
               -lwinmm

SOURCES += main.cpp\
        mainframe.cpp \
    imageform.cpp \
    kfc.cpp

HEADERS  += mainframe.h \
    imageform.h \
    kfc.h

FORMS    += mainframe.ui \
    imageform.ui

RESOURCES += \
    images/mainframe.qrc

