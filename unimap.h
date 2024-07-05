#ifndef UNIMAP_H
#define UNIMAP_H

#include "global_defines.h"
#include "submap.h"

#include <QObject>
#include <QSettings>
#include <QInputDialog>
#include <QFileInfo>
#include <QByteArray>
#include <QJsonDocument>
#include <QJsonValue>
#include <QJsonArray>
#include <QJsonObject>
#include <QDir>

class UNIMAP : public QObject
{
    Q_OBJECT
public:
    explicit UNIMAP(QObject *parent = nullptr);
    std::mutex mtx;

    void load_map(QString path);
    void save_map(QString path);

    void load_map_soft(QString path);
    void load_locations();

    void save_travel_map();
    void save_static_obs_map();

    void update_map(SUBMAP* submap);
    void update_map(cv::Mat &_map, SUBMAP* submap);
    void update_travel_map(std::vector<cv::Vec3d> poses);
    void update_dynamic_obs_map(std::vector<cv::Vec2d> &pts);

    void set_map(cv::Mat &_map);
    void clear_map_soft();
    void clear_map();
    void erase_one_dot_noise(cv::Mat& img);

    void draw_robot(cv::Mat &img, cv::Vec3d xi, cv::Scalar c, int tickness);    
    void draw_lidar(cv::Mat &img, cv::Vec3d xi, std::vector<cv::Vec2d> pts, cv::Scalar c);
    void draw_lidar_scale(cv::Mat &img, cv::Vec3d xi, std::vector<cv::Vec2d> pts, cv::Scalar c, int scale);

    void draw_path(cv::Mat &img, std::vector<PATH_POINT> path, cv::Scalar c);
    void draw_trajectory(cv::Mat &img, std::vector<cv::Vec6d> traj, cv::Scalar c, int tickness);
    void draw_travel_map(cv::Mat &img);

    void draw_obs_map(cv::Mat &img);
    void draw_velocity_map(cv::Mat &img);

    cv::Mat get_map_raw();
    cv::Mat get_map_plot();
    cv::Mat get_map_travel();
    cv::Mat get_map_avoid();

    cv::Mat get_map_static_obs();
    cv::Mat get_map_extended_static_obs();

    cv::Mat get_map_dynamic_obs();
    cv::Mat get_map_extended_dynamic_obs();

    cv::Mat get_map_obs();
    cv::Mat get_map_extended_obs();

    cv::Mat get_map_extended_dynamic_obs_near();

    std::vector<cv::Vec3d> get_locations();
    std::vector<cv::Vec3d> get_resting_locations();
    std::vector<cv::Vec3d> get_charging_locations();
    std::vector<cv::Vec3d> get_cleaning_locations();
    std::vector<std::vector<cv::Vec3d>> get_serving_locations();
    std::vector<QString> get_serving_group_names();
    std::vector<std::vector<QString>> get_serving_names();

    std::vector<cv::Vec2d> get_map_pts();
    int get_preset_idx(cv::Vec2d pt);

    // topology
    std::vector<QString> get_nodes(QString attrib);
    NODE* get_node_by_id(QString name);
    NODE* get_node_nn(cv::Vec2d pt);
    NODE* get_edge_nn(cv::Vec3d pose);

    // json interface
    QJsonArray pose_to_array(cv::Vec3d pose);
    cv::Vec3d array_to_pose(QJsonArray arr);
    QJsonArray links_to_array(std::vector<QString> linked);
    std::vector<QString> array_to_links(QJsonArray arr);
    double calc_seg_dist(cv::Vec2d _P0, cv::Vec2d _P1, cv::Vec2d _P);

    cv::Vec2i xy_uv(cv::Vec2d P, cv::Vec3d xi);
    cv::Vec2i xy_uv(cv::Vec2d P);
    cv::Vec2i xy_uv(cv::Vec2d P, int map_size, double grid_size);
    cv::Vec2i xy_uv(cv::Vec2d P, cv::Vec3d xi, int map_size, double grid_size);
    cv::Vec2d xy_uvd(cv::Vec2d P);
    cv::Vec2i xy_uv2(cv::Vec2d P);

    cv::Vec2d uv_xy(cv::Vec2i uv);
    cv::Vec2d uv_xy(cv::Vec2i uv, cv::Vec3d xi);

    // map info
    std::atomic<bool> is_loaded;
    QString map_dir;
    int map_w = 1000;
    int map_h = 1000;
    int map_ou = 500;
    int map_ov = 500;
    int map_cut_u = 0;
    int map_cut_v = 0;
    int map_pre_w = 0;
    int map_pre_h = 0;
    double map_angle = 0;
    double map_grid_width = 0.05;

    // annotated locations
    std::vector<cv::Vec3d> charging_locs;
    std::vector<cv::Vec3d> resting_locs;
    std::vector<cv::Vec3d> cleaning_locs;
    std::vector<std::vector<cv::Vec3d>> serving_locs;

    std::vector<QString> charging_names;
    std::vector<QString> resting_names;
    std::vector<QString> cleaning_names;
    std::vector<QString> serving_group_names;
    std::vector<std::vector<QString>> serving_names;

    std::vector<cv::Vec3d> locs;

    // map image
    cv::Mat raw_map;    
    cv::Mat travel_map;
    cv::Mat ui_travel_map;
    cv::Mat dynamic_cnt_map;
    cv::Mat static_obs_map;    
    cv::Mat dynamic_obs_map;
    cv::Mat extended_static_obs_map;
    cv::Mat extended_dynamic_obs_map;
    cv::Mat extended_dynamic_obs_near_map;
    cv::Mat velocity_map;
    cv::Mat avoid_map;

    // map points    
    std::vector<cv::Vec2d> map_pts;
    size_t map_pts_size = 0;

    std::vector<NODE> nodes;

private:

signals:

public slots:
};

#endif // UNIMAP_H
