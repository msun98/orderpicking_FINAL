#ifndef MULTICONTROL_H
#define MULTICONTROL_H

// defines
#include "global_defines.h"

// qt
#include <QObject>
#include <QTimer>

// modules
#include "mobile.h"
#include "slam_2d.h"
#include "unimap.h"
#include "ws_client.h"

// third party
#include "spline.h"

class MULTICONTROL : public QObject
{
    Q_OBJECT
public:
    explicit MULTICONTROL(QObject *parent = nullptr);
    ~MULTICONTROL();
    std::mutex mtx;

    void init(MOBILE *_mobile, SLAM_2D *_slam, UNIMAP *_unimap, WS_CLIENT *_ws);
    void run(cv::Vec3d _goal, int _preset_idx = PRESET_SPEED_NORMAL);
    void stop();
    std::vector<PATH_POINT> get_cur_path();

    double sgn(double val);
    double saturation(double val, double min, double max);

    int get_cur_idx(cv::Vec3d cur_pose, std::vector<PATH_POINT> path);
    int get_tgt_idx(int cur_idx, double ld, int path_num);

    cv::Vec3d mul_pose(cv::Vec3d xi0, cv::Vec3d xi1);
    bool is_los(cv::Mat& map, cv::Vec2i pt0, cv::Vec2i pt1);
    void fill_mask(cv::Mat&src, cv::Mat mask, float val);

    // for path
    std::vector<PATH_POINT> calc_ref_path(std::vector<cv::Vec2d> path);
    std::vector<PATH_POINT> calc_avoid_path(cv::Mat obs_map, std::vector<PATH_POINT> ref);

    std::vector<cv::Vec2i> path_finding_avoid(cv::Mat map, cv::Mat cost_map, cv::Vec2i st, cv::Vec2i ed);
    std::vector<cv::Vec2d> path_dividing(std::vector<cv::Vec2d> _path, double step);
    std::vector<cv::Vec2d> path_ccma(std::vector<cv::Vec2d> P);

    // for pp
    double calc_motion_time(double _s, double _v0, double _v1, double _acc);

    void smoothing_ref_v(std::vector<PATH_POINT>& path);
    void smoothing_obs_v(std::vector<PATH_POINT>& path);

    // for obstacle
    void marking_obstacle(int cur_idx, double dead_zone, std::vector<PATH_POINT>& path, cv::Mat& obs_map);
    bool checking_obstacle_pivot(cv::Vec2d pos, cv::Mat& obs_map);
    bool checking_obstacle_in_trajectory(std::vector<cv::Vec3d> traj, cv::Mat& obs_map);
    std::vector<cv::Vec3d> calc_trajectory(double v, double w, double dt, double predict_t, cv::Vec3d xi0);
    std::vector<cv::Vec6d> calc_trajectory(cv::Vec2d vw, double predict_time, double dt, cv::Vec3d cur_pose, cv::Vec2d cur_vw);

    bool check_resting_location(cv::Vec3d pose);
    bool check_serving_location(cv::Vec3d pose);

    int check_led_state(int cur_idx, const std::vector<PATH_POINT>& path);
    int check_everything_fine();

    // for mileage
    void write_mileage(double dist);

    std::atomic<bool> is_loc = {false};
    std::atomic<bool> is_resting = {false};
    std::atomic<int> preset_idx = {PRESET_SPEED_NORMAL};

    CTS_STATUS status;
    std::vector<PATH_POINT> cur_path;
    std::vector<PATH_POINT> ref_path;
    std::vector<PATH_POINT> avoid_path;

    // fsm
    std::atomic<int> cur_face_state = {UI_FACE_NORMAL};
    std::atomic<int> fsm_state = {STATE_AUTO_GOAL_REACHED};
    std::atomic<bool> fsm_flag = {false};
    std::thread* fsm_thread = NULL;
    void fsm_loop();

private:
    MOBILE *mobile = NULL;
    SLAM_2D *slam = NULL;
    UNIMAP *unimap = NULL;
    WS_CLIENT *ws = NULL;

    QTimer status_timer;

private slots:
    void status_loop();

    // from server
    void stc_id(QString id);
    void stc_stop();
    void stc_allow(int allow);
    void stc_path(std::vector<cv::Vec2d> path);

signals:
    void signal_cts_status(CTS_STATUS status);
    void signal_cts_confirm(std::vector<cv::Vec2d> path);

};

#endif // MULTICONTROL_H
