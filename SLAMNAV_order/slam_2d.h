#ifndef SLAM_2D_H
#define SLAM_2D_H

// defines
#include "global_defines.h"

// qt
#include <QObject>

// p2o
#include "p2o.h"

// my
#include "lidar_2d.h"
#include "mobile.h"
#include "submap.h"
#include "unimap.h"
#include "cam.h"

class SLAM_2D : public QObject
{
    Q_OBJECT
public:
    explicit SLAM_2D(QObject *parent = nullptr);
    ~SLAM_2D();

    // mapping interface
    void init(CAM *_cam, LIDAR_2D *_lidar, MOBILE *_mobile, UNIMAP *_unimap);
    void run();
    void stop();
    void save(QString path);

    // localization interface
    void set_map();
    void set_cur_scans();
    void set_initial_location(cv::Vec3d xi);
    void set_auto_initial_location_resting();
    void set_auto_initial_location_semi();
    void set_auto_initial_location_full();
    void run_loc();
    void stop_loc();

    // data interface    
    cv::Vec3d get_cur_pose();
    cv::Vec3d get_cur_scan_pose();
    std::vector<cv::Vec2d> get_cur_scan();
    std::vector<cv::Vec2d> get_cur_scan_global();
    void get_cur_scan_and_pose(std::vector<cv::Vec2d> &scan, cv::Vec3d& pose);

    // flags    
    std::atomic<bool> is_loc;
    std::atomic<bool> is_slam;
    std::atomic<bool> is_init;

    std::atomic<int> is_travel;

    // storage
    std::mutex mtx;
    std::atomic<double> cur_pose_t;
    cv::Vec3d cur_pose;
    cv::Vec3d cur_scan_pose;
    std::vector<cv::Vec2d> cur_scan;
    std::vector<cv::Vec2d> cur_scan_global;
    tbb::concurrent_queue<LOC> loc_que;
    tbb::concurrent_queue<TIME_POSE> tp_que;    
    tbb::concurrent_queue<TIME_POSE> tp_que2;
    std::vector<cv::Vec3d> locs;

    std::atomic<bool> is_manual_lc = {false};

    std::atomic<double> loc_inlier_ratio;
    std::atomic<double> loc_inlier_error;

    std::atomic<double> mapping_inlier_ratio;
    std::atomic<double> mapping_inlier_error;

    // nanoflann
    Eigen::MatrixXf tree_pts;
    using my_kd_tree_t = nanoflann::KDTreeEigenMatrixAdaptor<Eigen::MatrixXf, 2, nanoflann::metric_L2_Simple>;
    my_kd_tree_t *tree = NULL;

    // slam algorithm
    bool map_to_scan(SUBMAP &map, std::vector<cv::Vec2d> &pts, cv::Vec3d &res, double &inlier_error, double &inlier_ratio);
    bool scan_to_scan_icp(std::vector<cv::Vec2d> pts0, std::vector<cv::Vec2d> pts1, double max_dist, cv::Vec3d &res);
    bool submap_to_submap_icp2(SUBMAP &map0, SUBMAP &map1, cv::Vec3d &res);    
    bool map_to_scan_icp2(std::vector<cv::Vec2d> &pts, cv::Vec3d &res, double max_dist, double &inlier_error, double &inlier_ratio);

    // loop closing
    void pose_graph_optimization(std::vector<SUBMAP>& _submaps, std::vector<LC_INFO>& _lc_infos);

    // utils    
    double saturation(double val, double min, double max);
    std::vector<cv::Vec3d> transform(std::vector<cv::Vec3d> pts, cv::Vec3d xi);
    std::vector<cv::Vec2d> transform(std::vector<cv::Vec2d> pts, cv::Vec3d xi);
    std::vector<cv::Vec2i> transform(std::vector<cv::Vec2i> pts, cv::Vec3d xi);
    cv::Vec2d transform(cv::Vec3d xi, cv::Vec2d P);
    cv::Vec3d mulXi(cv::Vec3d xi0, cv::Vec3d xi1);
    cv::Vec3d divXi(cv::Vec3d xi0, cv::Vec3d xi1);
    cv::Vec3d invXi(cv::Vec3d xi);
    cv::Vec2d dTdR(cv::Vec3d xi0, cv::Vec3d xi1);
    cv::Vec3d intpXi(double a, double b, cv::Vec3d xi0, cv::Vec3d xi1);
    cv::Vec3d get_best_pose(double t, std::vector<TIME_POSE> &tp);
    cv::Mat resize_max_filter(cv::Mat &img, int w, int h);    



signals:
    void auto_init_finished_resting();
    void auto_init_finished_semi();
    void auto_init_finished_full();

private:
    CAM *cam = NULL;
    LIDAR_2D *lidar = NULL;
    MOBILE *mobile = NULL;
    UNIMAP *unimap = NULL;

    // storage
    std::vector<SUBMAP> submaps;

    // slam loop
    std::atomic<bool> aFlag;
    std::thread* aThread = NULL;
    void aLoop();

    // localization loop
    std::atomic<bool> locFlag;
    std::thread* locThread = NULL;
    void locLoop();

    std::atomic<bool> locFlag2;
    std::thread* locThread2 = NULL;
    void locLoop2();

    std::atomic<bool> obsFlag;
    std::thread* obsThread = NULL;
    void obsLoop();

public:
    // for bnb
    struct BNB_NODE
    {
        int cx = 0;
        int cy = 0;
        int cth = 0;
        int ch = 0;
        double score = 0;
        std::vector<cv::Vec2i> uv;

        BNB_NODE() {}
        BNB_NODE(int _cx, int _cy, int _cth, int _ch, double _score, std::vector<cv::Vec2i> &_uv)
        {
            cx = _cx;
            cy = _cy;
            cth = _cth;
            ch = _ch;
            score = _score;
            uv = _uv;
        }
        BNB_NODE(const BNB_NODE &p)
        {
            this->cx = p.cx;
            this->cy = p.cy;
            this->cth = p.cth;
            this->ch = p.ch;
            this->score = p.score;
            this->uv = p.uv;
        }
    };

    const int bnb_step[10] = { 1,2,4,8,16,32,64,128,256,512 };
    cv::Mat maxfilter(cv::Mat map, int ch);
    cv::Vec3d map_to_scan_bnb(cv::Mat _map, std::vector<cv::Vec2d> pts);
};

#endif // SLAM_2D_H
