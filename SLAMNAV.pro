QT       += core gui network websockets

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++14
CONFIG += optimize_full
CONFIG += debug

# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0
w
HOME = $$system(echo $HOME)
message($$HOME)

DEFINES += VERSION_LOG="\\\"1.7.5"\\\"
DEFINES += BUILD_DATE='"\\\"$(shell date +"%Y-%m-%d %H:%M:%S")\\\""'

SOURCES += \
    LakiBeamHTTP.cpp \
    LakiBeamUDP.cpp \
    Logger.cpp \
    autocontrol.cpp \
    cam.cpp \
    cmd_server.cpp \
    cv_to_qt.cpp \
    integrate_ui.cpp \
    ipc.cpp \
    l2c.cpp \
    lidar_2d.cpp \
    main.cpp \
    mainwindow.cpp \
    mobile.cpp \
    multicontrol.cpp \
    mygraphicsview.cpp \
    p2o.cpp \
    sim.cpp \
    slam_2d.cpp \
    submap.cpp \
    topomap.cpp \
    unimap.cpp \
    ws_client.cpp

HEADERS += \
    LakiBeamHTTP.h \
    LakiBeamUDP.h \
    Logger.h \
    autocontrol.h \
    cam.h \
    cmd_server.h \
    cv_to_qt.h \
    global_defines.h \
    integrate_ui.h \
    ipc.h \
    l2c.h \
    lidar_2d.h \
    mainwindow.h \
    mobile.h \
    multicontrol.h \
    mygraphicsview.h \
    nanoflann.hpp \
    p2o.h \
    sim.h \
    slam_2d.h \
    spline.h \
    submap.h \
    topomap.h \
    unimap.h \
    ws_client.h

FORMS += \
    mainwindow.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

# Libraries setting (for x86_64)
contains(QT_ARCH, x86_64){
    # OpenCV library all
    INCLUDEPATH += /usr/include/opencv4/
    LIBS += -L/usr/lib/x86_64-linux-gnu/
    LIBS += -lopencv_core \
            -lopencv_highgui \
            -lopencv_imgcodecs \
            -lopencv_imgproc \
            -lopencv_calib3d \
            -lopencv_features2d \
            -lopencv_flann \
            -lopencv_objdetect \
            -lopencv_photo \
            -lopencv_video \
            -lopencv_videoio \
            -lopencv_ximgproc
            -lboost_system \

    # Lakibeam lidar
    INCLUDEPATH += /usr/include/rapidjson/
    INCLUDEPATH += /usr/include/boost/
    INCLUDEPATH += /usr/include/boost/beast/
    LIBS += -L/usr/lib/x86_64-linux-gnu/
    LIBS += -lboost_system \
            -lboost_thread

    # Eigen and Sophus library
    INCLUDEPATH += /usr/include/eigen3/
    INCLUDEPATH += /usr/local/include/sophus/

    # TBB
    LIBS += -L/usr/lib/x86_64-linux-gnu/
    LIBS += -ltbb

    # OpenMP
    QMAKE_CXXFLAGS += -fopenmp
    QMAKE_CFLAGS += -fopenmp
    LIBS += -L/usr/lib/x86_64-linux-gnu/
    LIBS += -lpthread
    LIBS += -lgomp

    # rplidar sdk
    INCLUDEPATH += $$HOME/rplidar_sdk/sdk/include/
    LIBS += -L$$HOME/rplidar_sdk/output/Linux/Release/
    LIBS += -lsl_lidar_sdk

    # sick sdk
    INCLUDEPATH += /usr/local/include/sick_safetyscanners_base/
    LIBS += -L/usr/local/lib/
    LIBS += -lsick_safetyscanners_base

#    # orbbec
#    INCLUDEPATH += /usr/local/include/
#    LIBS += -L/usr/local/lib/
#    LIBS += -lOrbbecSDK

    # orbbec
    INCLUDEPATH += $$HOME/OrbbecSDK/build/install/include/
    LIBS += -L$$HOME/OrbbecSDK/build/install/lib/
    LIBS += -lOrbbecSDK
}

# Libraries setting (for aarch64)
contains(QT_ARCH, arm64) {
    # OpenCV library all
    INCLUDEPATH += /usr/include/opencv4/
    LIBS += -L/usr/lib/aarch64-linux-gnu/
    LIBS += -lopencv_core \
            -lopencv_highgui \
            -lopencv_imgcodecs \
            -lopencv_imgproc \
            -lopencv_calib3d \
            -lopencv_features2d \
            -lopencv_flann \
            -lopencv_objdetect \
            -lopencv_photo \
            -lopencv_video \
            -lopencv_videoio \
            -lboost_system \
            -lopencv_ximgproc

    # Lakibeam lidar
    INCLUDEPATH += /usr/include/boost/
    LIBS += -L/usr/lib/aarch64-linux-gnu/
    LIBS += -lboost_system \
            -lboost_thread

    INCLUDEPATH += /usr/include/boost/beast/
    INCLUDEPATH += /usr/include/rapidjson/

    # Eigen and Sophus library
    INCLUDEPATH += /usr/include/eigen3/
    INCLUDEPATH += /usr/local/include/sophus/

    # TBB
    LIBS += -L/usr/lib/aarch64-linux-gnu/
    LIBS += -ltbb

    # OpenMP
    QMAKE_CXXFLAGS += -fopenmp
    QMAKE_CFLAGS += -fopenmp
    LIBS += -L/usr/lib/aarch64-linux-gnu/
    LIBS += -lpthread
    LIBS += -lgomp

    # rplidar sdk
    INCLUDEPATH += $$HOME/rplidar_sdk/sdk/include/
    LIBS += -L$$HOME/rplidar_sdk/output/Linux/Release/
    LIBS += -lsl_lidar_sdk

    # sick sdk
    INCLUDEPATH += /usr/local/include/sick_safetyscanners_base/
    LIBS += -L/usr/local/lib/
    LIBS += -lsick_safetyscanners_base

    # orbbec
    INCLUDEPATH += /usr/local/include/
    LIBS += -L/usr/local/lib/
    LIBS += -lOrbbecSDK
}

RESOURCES += \
    img.qrc
