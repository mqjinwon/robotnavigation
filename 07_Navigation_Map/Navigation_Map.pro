
TEMPLATE = app
TARGET = wheel3
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt


QT += opengl
SOURCES += main.cpp

DEFINES += dDOUBLE
INCLUDEPATH += c:/ode-0.13/include
INCLUDEPATH += c:/ode-0.13/drawstuff
INCLUDEPATH += c:/ode-0.13/ode/src

LIBS        += c:/ode-0.13/lib/ReleaseDoubleLib/libode_double.a
LIBS        += c:/ode-0.13/lib/ReleaseDoubleLib/libdrawstuff.a

win32{ RC_FILE = c:/ode-0.13/drawstuff/src/resources.rc }

#for OpenGL
LIBS        += -lopengl32\
               -lglu32\
               -lgdi32\
               -lglut32\
               -luser32\
               -lwinmm

HEADERS +=
