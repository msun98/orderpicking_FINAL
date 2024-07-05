#ifndef LIDAR_2D_H
#define LIDAR_2D_H

// defines
#include "global_defines.h"

// rplidar sdk
#include "sl_lidar.h"
#include "sl_lidar_driver.h"
#ifndef _countof
    #define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#ifdef USE_DUAL_SICK
#include <sick_safetyscanners_base/SickSafetyscanners.h>
#include <sick_safetyscanners_base/Exceptions.h>
#include <sick_safetyscanners_base/Types.h>
#include <sick_safetyscanners_base/datastructure/CommSettings.h>
#endif

#ifdef USE_DUAL_LAKI
#include "LakiBeamHTTP.h"
#include "LakiBeamUDP.h"
#endif

// stl
#include <vector>
#include <thread>
#include <atomic>

// qt
#include <QObject>

// my
#include "mobile.h"

class LIDAR_2D : public QObject
{
    Q_OBJECT
public:
    explicit LIDAR_2D(QObject *parent = nullptr);
    ~LIDAR_2D();

    void init(MOBILE *_mobile);
    tbb::concurrent_queue<LIDAR_FRM> raw_que;
    tbb::concurrent_queue<LIDAR_FRM> scan_que;

    // for dual lidar
    tbb::concurrent_queue<LIDAR_FRM> raw_que1;

    std::mutex mtx;
    std::vector<cv::Vec2d> cur_scan;
    std::vector<cv::Vec2d> get_cur_scan();

private:
    MOBILE *mobile = NULL;

    std::atomic<bool> grabFlag;
    std::thread* grabThread = NULL;    
    void grabLoop();

    // for dual rplidar
    std::atomic<bool> grabFlag1;
    std::thread* grabThread1 = NULL;
    void grabLoop1();

    std::atomic<bool> aFlag;
    std::thread* aThread = NULL;
    void aLoop();

    cv::Vec3d intpXi(double a, double b, cv::Vec3d xi0, cv::Vec3d xi1);
    cv::Vec3d intpXi(double t, double t0, double t1, cv::Vec3d xi0, cv::Vec3d xi1);
    cv::Vec3d divXi(cv::Vec3d xi0, cv::Vec3d xi1);
    cv::Vec3d mulXi(cv::Vec3d xi0, cv::Vec3d xi1);
    cv::Vec2d transform(cv::Vec3d xi, cv::Vec2d pt);

signals:

};

#endif // LIDAR_2D_H
