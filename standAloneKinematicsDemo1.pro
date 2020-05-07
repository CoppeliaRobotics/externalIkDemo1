include(config.pri)

QT -= core
QT -= gui

TARGET = standAloneKinematicsDemo1
TEMPLATE = app

DEFINES -= UNICODE
CONFIG   += console
CONFIG   -= app_bundle

#DEFINES += SIM_MATH_DOUBLE # for double-precision
DEFINES += NON_MATLAB_PARSING
DEFINES += MAX_EXT_API_CONNECTIONS=255

*-msvc* {
    QMAKE_CXXFLAGS += -O2
    QMAKE_CXXFLAGS += -W3
}
*-g++* {
    QMAKE_CXXFLAGS += -O3
    QMAKE_CXXFLAGS += -Wall
    QMAKE_CXXFLAGS += -Wno-unused-parameter
    QMAKE_CXXFLAGS += -Wno-strict-aliasing
    QMAKE_CXXFLAGS += -Wno-empty-body
    QMAKE_CXXFLAGS += -Wno-write-strings

    QMAKE_CXXFLAGS += -Wno-unused-but-set-variable
    QMAKE_CXXFLAGS += -Wno-unused-local-typedefs
    QMAKE_CXXFLAGS += -Wno-narrowing

    QMAKE_CFLAGS += -O3
    QMAKE_CFLAGS += -Wall
    QMAKE_CFLAGS += -Wno-strict-aliasing
    QMAKE_CFLAGS += -Wno-unused-parameter
    QMAKE_CFLAGS += -Wno-unused-but-set-variable
    QMAKE_CFLAGS += -Wno-unused-local-typedefs
}

INCLUDEPATH += $$BOOST_INCLUDEPATH

win32 {
    LIBS += -lwinmm
    LIBS += -lWs2_32
    DEFINES += WIN_SIM
}

macx {
    DEFINES += MAC_SIM
}

unix:!macx {
    DEFINES += LIN_SIM
    LIBS += -lrt
}

INCLUDEPATH += "../include"
INCLUDEPATH += "../remoteApi"
INCLUDEPATH += "../simMath"
INCLUDEPATH += "../coppeliaKinematicsRoutines"

unix:!symbian {
    maemo5 {
        target.path = /opt/usr/lib
    } else {
        target.path = /usr/lib
    }
    INSTALLS += target
}

HEADERS += \
    ../remoteApi/extApiPlatform.h \
    ../remoteApi/extApiInternal.h \
    ../remoteApi/extApi.h \
    ../coppeliaKinematicsRoutines/ik.h \
    ../coppeliaKinematicsRoutines/environment.h \
    ../coppeliaKinematicsRoutines/serialization.h \
    ../coppeliaKinematicsRoutines/ikRoutines.h \
    ../coppeliaKinematicsRoutines/ikGroupContainer.h \
    ../coppeliaKinematicsRoutines/ikGroup.h \
    ../coppeliaKinematicsRoutines/ikElement.h \
    ../coppeliaKinematicsRoutines/objectContainer.h \
    ../coppeliaKinematicsRoutines/sceneObject.h \
    ../coppeliaKinematicsRoutines/dummy.h \
    ../coppeliaKinematicsRoutines/joint.h \
    ../simMath/mathDefines.h \
    ../simMath/MyMath.h \
    ../simMath/3Vector.h \
    ../simMath/4Vector.h \
    ../simMath/7Vector.h \
    ../simMath/Vector.h \
    ../simMath/3X3Matrix.h \
    ../simMath/4X4Matrix.h \
    ../simMath/4X4FullMatrix.h \
    ../simMath/MMatrix.h

SOURCES += \
    ../remoteApi/extApiPlatform.c \
    ../remoteApi/extApi.c \
    ../coppeliaKinematicsRoutines/ik.cpp \
    ../coppeliaKinematicsRoutines/environment.cpp \
    ../coppeliaKinematicsRoutines/serialization.cpp \
    ../coppeliaKinematicsRoutines/ikRoutines.cpp \
    ../coppeliaKinematicsRoutines/ikGroupContainer.cpp \
    ../coppeliaKinematicsRoutines/ikGroup.cpp \
    ../coppeliaKinematicsRoutines/ikElement.cpp \
    ../coppeliaKinematicsRoutines/objectContainer.cpp \
    ../coppeliaKinematicsRoutines/sceneObject.cpp \
    ../coppeliaKinematicsRoutines/dummy.cpp \
    ../coppeliaKinematicsRoutines/joint.cpp \
    ../simMath/MyMath.cpp \
    ../simMath/3Vector.cpp \
    ../simMath/4Vector.cpp \
    ../simMath/7Vector.cpp \
    ../simMath/Vector.cpp \
    ../simMath/3X3Matrix.cpp \
    ../simMath/4X4Matrix.cpp \
    ../simMath/4X4FullMatrix.cpp \
    ../simMath/MMatrix.cpp \
    ../common/shared_memory.c \
    standAloneKinematicsDemo1.cpp

