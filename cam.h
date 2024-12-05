#ifndef CAM_H
#define CAM_H

#include "global_defines.h"
//#include <libobsensor/ObSensor.hpp>
#include <QObject>

class CAM : public QObject
{
    Q_OBJECT
public:
    explicit CAM(QObject *parent = nullptr);
    ~CAM();

    // interface func
    void init();

    QString get_sn_l();
    QString get_sn_r();

    cv::Mat get_cur_img_l();
    cv::Mat get_cur_img_r();

    cv::Mat get_cur_img_l_color();
    cv::Mat get_cur_img_r_color();

    std::vector<cv::Vec2d> get_cur_scan_l();
    std::vector<cv::Vec2d> get_cur_scan_r();

    // util func
    Eigen::Matrix4d zyx_tranformation(double x, double y, double z, double rx, double ry, double rz);

    // loop
    std::atomic<bool> grabFlag;
    std::thread* grabThread = NULL;
    void grabLoop();

    // storage    
    std::mutex mtx_l;
    std::mutex mtx_r;
    QString sn_l;
    QString sn_r;
    cv::Mat cur_img_l;
    cv::Mat cur_img_r;
    cv::Mat cur_img_l_color;
    cv::Mat cur_img_r_color;
    double time_l = 0;
    double time_r = 0;
    std::vector<cv::Vec2d> cur_scan_l;
    std::vector<cv::Vec2d> cur_scan_r;

private:

signals:

};

#endif // CAM_H
