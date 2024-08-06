#ifndef AUTOCONTROL_H
#define AUTOCONTROL_H

// defines
#include "global_defines.h"

// qt
#include <QObject>

// modules
#include "mobile.h"
#include "slam_2d.h"
#include "unimap.h"

// third party
#include "spline.h"

class AUTOCONTROL : public QObject
{
    Q_OBJECT
public:
    explicit AUTOCONTROL(QObject *parent = nullptr);
    ~AUTOCONTROL();
    std::mutex mtx;

    void init(MOBILE *_mobile, SLAM_2D *_slam, UNIMAP *_unimap);
    void run(cv::Vec3d _goal, int _preset_idx = PRESET_SPEED_NORMAL);
    void run_pick(cv::Vec3d _goal, int _preset_idx = PRESET_SPEED_NORMAL);
    void run_ext(std::vector<cv::Vec3d> waypoints, int _preset_idx = PRESET_SPEED_NORMAL);
    void run_ext2(QString goal_node, int _preset_idx = PRESET_SPEED_NORMAL);
    void stop();
    std::vector<PATH_POINT> get_cur_path();

    double sgn(double val);
    double saturation(double val, double min, double max);

    int get_cur_idx(cv::Vec2d cur_pos, std::vector<PATH_POINT> path);
    int get_tgt_idx(int cur_idx, double ld, int path_num);

    void draw_approach_travel_line(cv::Mat travel, cv::Vec2i st, cv::Vec2i ed);
    cv::Mat travel_smoothing(cv::Mat img, double r);
    void calc_path_ref_v(std::vector<PATH_POINT> &path);

    cv::Vec3d mul_pose(cv::Vec3d xi0, cv::Vec3d xi1);
    bool is_los(cv::Mat& map, cv::Vec2i pt0, cv::Vec2i pt1);
    void fill_mask(cv::Mat&src, cv::Mat mask, float val);

    // for path
    std::vector<PATH_POINT> calc_no_travel_path(cv::Vec3d st_pose, cv::Vec3d ed_pose);
    std::vector<PATH_POINT> calc_short_path(cv::Vec3d st_pose, cv::Vec3d ed_pose);
    std::vector<PATH_POINT> calc_ref_path(cv::Vec3d st_pose, cv::Vec3d ed_pose);
    std::vector<PATH_POINT> calc_avoid_path(cv::Mat obs_map, std::vector<PATH_POINT> ref);
    std::vector<PATH_POINT> calc_topo_path(QString goal_node_name);
    std::vector<PATH_POINT> waypoints_to_path(std::vector<cv::Vec3d> waypoints);

    std::vector<cv::Vec2i> path_finding(cv::Mat map, cv::Vec2i st, cv::Vec2i ed);
    std::vector<cv::Vec2i> path_finding_avoid(cv::Mat map, cv::Mat cost_map, cv::Vec2i st, cv::Vec2i ed);
    std::vector<QString> topo_path_finding(QString goal_node_name);
    std::vector<cv::Vec2d> path_dividing(std::vector<cv::Vec2d> _path, double step);
    std::vector<cv::Vec2d> path_ccma(std::vector<cv::Vec2d> path);

    // for pp
    double calc_motion_time(double _s, double _v0, double _v1, double _acc);

    void smoothing_obs_v(std::vector<PATH_POINT>& path);
    void smoothing_ref_v(std::vector<PATH_POINT>& path);

    // for obstacle
    void marking_obstacle(int cur_idx, double dead_zone, std::vector<PATH_POINT>& path, cv::Mat& obs_map);
    bool checking_obstacle(int cur_idx, std::vector<PATH_POINT>& path, cv::Mat& obs_map);    
    bool checking_obstacle_pivot(cv::Vec2d pos, cv::Mat& obs_map);
    bool checking_obstacle_in_trajectory(std::vector<cv::Vec3d> traj, cv::Mat& obs_map);
    std::vector<cv::Vec6d> calc_trajectory(cv::Vec2d vw, double predict_time, double dt, cv::Vec3d cur_pose, cv::Vec2d cur_vw);
    std::vector<cv::Vec3d> calc_trajectory(double v, double w, double dt, double predict_t, cv::Vec3d xi0);

    bool check_resting_location(cv::Vec3d pose);
    bool check_serving_location(cv::Vec3d pose);

    int check_led_state(int cur_idx, const std::vector<PATH_POINT>& path);
    int check_everything_fine();

    // for mileage
    void write_mileage(double dist);

    // path
    std::vector<PATH_POINT> cur_path;

    // flags
    std::atomic<bool> is_loc = {false};
    std::atomic<bool> is_early_stop = {false};
    std::atomic<bool> is_ignore_safety = {false};
    std::atomic<bool> is_resting = {false};
    std::atomic<bool> is_serving = {false};
    std::atomic<int> preset_idx = {PRESET_SPEED_NORMAL};

    // obs flag
    std::atomic<int> cur_face_state = {UI_FACE_NORMAL};
    tbb::concurrent_queue<cv::Mat> local_map_plot;

    // fsm
    cv::Vec3d goal;
    std::atomic<int> fsm_state = {0};
    std::atomic<bool> fsm_flag = {false};
    std::thread* fsm_thread = NULL;
    void fsm_loop();
    void fsm_loop_pick();
    void fsm_loop_ext();
    void fsm_loop_ext2();

private:
    MOBILE *mobile = NULL;
    SLAM_2D *slam = NULL;
    UNIMAP *unimap = NULL;

};

#endif // AUTOCONTROL_H
