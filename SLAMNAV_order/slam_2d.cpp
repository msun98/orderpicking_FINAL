#include "slam_2d.h"

SLAM_2D::SLAM_2D(QObject *parent) : QObject(parent)
{
    is_loc = false;
    is_slam = false;    
    is_init = false;
    is_travel = 0;

    cur_pose = cv::Vec3d(0, 0, 0);
    cur_scan_pose = cv::Vec3d(0, 0, 0);

    loc_inlier_ratio = 0;
    loc_inlier_error = 0;

    mapping_inlier_ratio = 0;
    mapping_inlier_error = 0;
}

SLAM_2D::~SLAM_2D()
{
    if(obsThread != NULL)
    {
        obsFlag = false;
        obsThread->join();
        obsThread = NULL;
    }

    if(aThread != NULL)
    {
        aFlag = false;
        aThread->join();
        aThread = NULL;
    }

    if(locThread != NULL)
    {
        locFlag = false;
        locThread->join();
        locThread = NULL;
    }

    if(locThread2 != NULL)
    {
        locFlag2 = false;
        locThread2->join();
        locThread2 = NULL;
    }
}

void SLAM_2D::init(CAM *_cam, LIDAR_2D *_lidar, MOBILE *_mobile, UNIMAP *_unimap)
{
    cam = _cam;
    lidar = _lidar;    
    mobile = _mobile;    
    unimap = _unimap;    
    cur_pose = cv::Vec3d(0, 0, 0);
    cur_scan_pose = cv::Vec3d(0, 0, 0);
}

cv::Vec3d SLAM_2D::get_cur_pose()
{
    mtx.lock();
    cv::Vec3d _cur_pose = cur_pose;
    mtx.unlock();

    return _cur_pose;
}

cv::Vec3d SLAM_2D::get_cur_scan_pose()
{
    mtx.lock();
    cv::Vec3d _cur_pose = cur_scan_pose;
    mtx.unlock();

    return _cur_pose;
}

std::vector<cv::Vec2d> SLAM_2D::get_cur_scan()
{
    mtx.lock();
    std::vector<cv::Vec2d> _cur_scan = cur_scan;
    mtx.unlock();

    return _cur_scan;
}

std::vector<cv::Vec2d> SLAM_2D::get_cur_scan_global()
{
    mtx.lock();
    std::vector<cv::Vec2d> _cur_scan_global = cur_scan_global;
    mtx.unlock();

    return _cur_scan_global;
}

void SLAM_2D::get_cur_scan_and_pose(std::vector<cv::Vec2d> &scan, cv::Vec3d& pose)
{
    mtx.lock();
    scan = cur_scan;
    pose = cur_scan_pose;
    mtx.unlock();
}

void SLAM_2D::run()
{
    stop();
    stop_loc();

    if (aThread == NULL)
    {
        mtx.lock();
        cur_pose = cv::Vec3d(0, 0, 0);
        mtx.unlock();

        submaps.clear();
        unimap->clear_map();

        aFlag = true;
        aThread = new std::thread(&SLAM_2D::aLoop, this);

        is_slam = true;
    }
}

void SLAM_2D::stop()
{
    if(aThread != NULL)
    {
        is_slam = false;

        aFlag = false;
        aThread->join();        
        aThread = NULL;
    }

    mapping_inlier_ratio = 0;
    mapping_inlier_error = 0;
}

void SLAM_2D::save(QString path)
{
    QString cloud_path = path + "/clouds.csv";

    QFile file(cloud_path);
    if(file.open(QIODevice::WriteOnly))
    {
        // sampling (1mm)
        std::unordered_map<std::string, cv::Vec2d> sampled;
        for(size_t p = 0; p < submaps.size(); p++)
        {
            cv::Vec3d xi = submaps[p].xi;
            for(size_t q = 0; q < submaps[p].pts_storage.size(); q++)
            {
                cv::Vec2d pt = submaps[p].pts_storage[q];
                cv::Vec2d _pt = transform(xi, pt);

                QString key;
                key.sprintf("%d,%d", (int)(_pt[0]*1000), (int)(_pt[1]*1000));
                sampled[key.toStdString()] = _pt;
            }
        }

        // writing
        for(auto& it: sampled)
        {
            cv::Vec2d pt = it.second;

            QString str;
            str.sprintf("%f,%f\n", pt[0], pt[1]);
            file.write(str.toLocal8Bit());
        }

        file.close();

        QString str;
        str.sprintf("[SLAM] save cloud num: %d\n", (int)sampled.size());
        logger.write(str, true);
    }
}

void SLAM_2D::set_map()
{
    // get map pts
    std::vector<cv::Vec2d> pts = unimap->get_map_pts();

    // set nanoflann
    if(tree != NULL)
    {
        delete tree;
    }

    tree_pts.resize(pts.size(), 2);
    for(size_t p = 0; p < pts.size(); p++)
    {
        tree_pts(p, 0) = pts[p][0];
        tree_pts(p, 1) = pts[p][1];
    }
    tree = new my_kd_tree_t(2, std::cref(tree_pts), 10);
}

void SLAM_2D::set_cur_scans()
{
    // get current single scan
    LIDAR_FRM frm;    
    while(lidar->scan_que.try_pop(frm))
    {
        mtx.lock();
        cur_scan = frm.pts;
        mtx.unlock();
    }
}

void SLAM_2D::set_initial_location(cv::Vec3d xi)
{
    // initial pose update
    mtx.lock();    
    cur_pose = xi;
    mtx.unlock();
}

void SLAM_2D::set_auto_initial_location_resting()
{
    // for new scan
    cur_scan.clear();

    int wait_cnt = 0;
    while(cur_scan.size() == 0)
    {
        wait_cnt++;
        if(wait_cnt > 30)
        {
            emit auto_init_finished_resting();
            logger.write("[AUTOINIT_RESTING] no cur_scan", true);
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // get cur scan
    std::vector<cv::Vec2d> _cur_scan = get_cur_scan();

    // resting locations
    std::vector<cv::Vec3d> _locs = unimap->get_resting_locations();
    if(_locs.size() == 0)
    {
        emit auto_init_finished_resting();
        logger.write("[AUTOINIT_RESTING] no resting locs", true);
        return;
    }

    // find best match
    cv::Vec3d min_pose = _locs[0];
    double min_cost = 99999999;
    for(size_t p = 0; p < _locs.size(); p++)
    {
        for(double th_offset = -180.0; th_offset <= 180.0; th_offset += 10.0)
        {
            cv::Vec3d xi = _locs[p];
            xi[2] += th_offset*D2R;

            double ie = 0; double ir = 0;
            if(map_to_scan_icp2(_cur_scan, xi, update_config.robot_icp_dist, ie, ir))
            {
                double cost = (ie/update_config.robot_icp_error) + (1.0 - ir);
                if(cost < min_cost)
                {
                    min_cost = cost;
                    min_pose = xi;
                    printf("[AUTOINIT_RESTING] loc_%d, x:%f, y:%f, th:%f, cost:%f\n", (int)p, xi[0], xi[1], xi[2]*R2D, cost);
                }
            }
        }
    }

    // initial pose update
    mtx.lock();
    cur_pose = min_pose;
    cur_scan_pose = min_pose;
    cur_scan = _cur_scan;
    locs = _locs;
    mtx.unlock();

    emit auto_init_finished_resting();

    logger.write("[AUTOINIT_RESTING] finished", true);
    return;
}

void SLAM_2D::set_auto_initial_location_semi()
{
    // for new scan
    cur_scan.clear();

    int wait_cnt = 0;
    while(cur_scan.size() == 0)
    {
        wait_cnt++;
        if(wait_cnt > 30)
        {
            emit auto_init_finished_semi();
            logger.write("[AUTOINIT_SEMI] no cur_scan", true);
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // get cur scan
    std::vector<cv::Vec2d> _cur_scan = get_cur_scan();

    // resting locations
    std::vector<cv::Vec3d> _locs = unimap->get_resting_locations();

    // read robot last pose
    cv::Vec3d last_pose(0,0,0);
    QString pose_path = QDir::homePath()+"/RB_MOBILE/config/robot_pose.ini";
    QFileInfo pose_info(pose_path);
    if(pose_info.exists() && pose_info.isFile())
    {
        QSettings settings(pose_path, QSettings::IniFormat);
        last_pose[0] = settings.value("POSE/robot_last_pose_x", 0.0).toDouble();
        last_pose[1] = settings.value("POSE/robot_last_pose_y", 0.0).toDouble();
        last_pose[2] = settings.value("POSE/robot_last_pose_th", 0.0).toDouble();
    }
    _locs.push_back(last_pose);

    // add cut map pos
    cv::Vec2i crop_left_top = cv::Vec2i(unimap->map_cut_u, unimap->map_cut_v);
    int old_map_size = unimap->map_pre_w;
    int new_map_size = unimap->map_w;
    double grid_size = unimap->map_grid_width;

    cv::Vec2d T;
    T[0] = (old_map_size/2 - (crop_left_top[1]+new_map_size/2))*grid_size;
    T[1] = (old_map_size/2 - (crop_left_top[0]+new_map_size/2))*grid_size;

    double angle = -unimap->map_angle;
    cv::Matx22d rot;
    rot(0, 0) = std::cos(angle);
    rot(0, 1) = -std::sin(angle);
    rot(1, 0) = std::sin(angle);
    rot(1, 1) = std::cos(angle);

    cv::Vec2d xy = rot*cv::Vec2d(last_pose[0], last_pose[1]) - T;

    cv::Vec3d cut_map_pose;
    cut_map_pose[0] = xy[0];
    cut_map_pose[1] = xy[1];
    cut_map_pose[2] = toWrap(last_pose[2] + angle);
    _locs.push_back(cut_map_pose);

    // find best match
    cv::Vec3d min_pose = _locs[0];
    double min_cost = 99999999;
    for(size_t p = 0; p < _locs.size(); p++)
    {
        for(double th_offset = -180.0; th_offset <= 180.0; th_offset += 10.0)
        {
            cv::Vec3d xi = _locs[p];
            xi[2] += th_offset*D2R;

            double ie = 0; double ir = 0;
            if(map_to_scan_icp2(_cur_scan, xi, update_config.robot_icp_dist, ie, ir))
            {
                double cost = (ie/update_config.robot_icp_error) + (1.0 - ir);
                if(cost < min_cost)
                {
                    min_cost = cost;
                    min_pose = xi;
                    printf("[AUTOINIT2] loc_%d, x:%f, y:%f, th:%f, cost:%f\n", (int)p, xi[0], xi[1], xi[2]*R2D, cost);
                }
            }
        }
    }

    // initial pose update
    mtx.lock();
    cur_pose = min_pose;
    cur_scan_pose = min_pose;
    cur_scan = _cur_scan;
    locs = _locs;
    mtx.unlock();

    emit auto_init_finished_semi();

    logger.write("[AUTOINIT_SEMI] finished", true);
    return;
}

void SLAM_2D::set_auto_initial_location_full()
{
    // for new scan
    cur_scan.clear();

    int wait_cnt = 0;
    while(cur_scan.size() == 0)
    {
        wait_cnt++;
        if(wait_cnt > 30)
        {
            emit auto_init_finished_full();
            logger.write("[AUTOINIT_FULL] no cur_scan", true);
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::vector<cv::Vec2d> _cur_scan = get_cur_scan();

    QString str;
    str.sprintf("[AUTOINIT_FULL] cur scan size: %d", (int)_cur_scan.size());
    logger.write(str, true);

    // coarse
    cv::Mat map = unimap->get_map_raw();
    cv::Vec3d xi = map_to_scan_bnb(map, _cur_scan);

    // initial pose update
    mtx.lock();
    cur_pose = xi;
    cur_scan_pose = xi;
    cur_scan = _cur_scan;
    mtx.unlock();

    emit auto_init_finished_full();

    logger.write("[AUTOINIT_FULL] finished", true);
    return;
}

void SLAM_2D::run_loc()
{
    stop();
    stop_loc();

    if (locThread == NULL)
    {
        locFlag = true;
        locThread = new std::thread(&SLAM_2D::locLoop, this);
    }

    if (locThread2 == NULL)
    {
        locFlag2 = true;
        locThread2 = new std::thread(&SLAM_2D::locLoop2, this);
    }

    if (obsThread == NULL)
    {
        obsFlag = true;
        obsThread = new std::thread(&SLAM_2D::obsLoop, this);
    }
}

void SLAM_2D::stop_loc()
{
    if(locThread != NULL)
    {
        locFlag = false;
        locThread->join();
        locThread = NULL;
    }

    if(locThread2 != NULL)
    {
        locFlag2 = false;
        locThread2->join();
        locThread2 = NULL;
    }

    if(obsThread != NULL)
    {
        obsFlag = false;
        obsThread->join();
        obsThread = NULL;
    }

    is_loc = false;

    loc_inlier_error = 0;
    loc_inlier_ratio = 0;
}

void SLAM_2D::aLoop()
{
    // clear old lidar scan
    lidar->scan_que.clear();
    LIDAR_FRM pre_frm;
    std::vector<LC_INFO> lc_infos;

    // loop start
    while(aFlag)
    {
        LIDAR_FRM frm;
        if(lidar->scan_que.try_pop(frm))
        {
            if(submaps.size() == 0)
            {
                // build initial submap
                SUBMAP _map;
                _map.id = 0;
                _map.xi = cv::Vec3d(0, 0, 0);
                submaps.push_back(_map);

                // add first data
                SUBMAP *map = &submaps[submaps.size() - 1];
                map->add_scan(frm.pts, cv::Vec3d(0, 0, 0));

                // update merge map
                unimap->update_map(map);

                // for next
                pre_frm = frm;
            }
            else
            {
                // if robot stop, No mapping is performed
                cv::Vec2d dtdr = dTdR(frm.mobile_pose, pre_frm.mobile_pose);
                if(dtdr[0] < 0.01 && dtdr[1] < 1.0*D2R && is_manual_lc == false)
                {
                    continue;
                }

                // get last submap
                SUBMAP *map = &submaps[submaps.size() - 1];

                // set initial guess from mobile
                cv::Vec3d dxi0 = divXi(frm.mobile_pose, pre_frm.mobile_pose);
                cv::Vec3d dxi = mulXi(map->dxi, dxi0);

                // map to scan pose estimation
                double ie = 0; double ir = 0;
                if(map_to_scan(*map, frm.pts, dxi, ie, ir))
                {
                    mapping_inlier_error = ie;
                    mapping_inlier_ratio = ir;

                    // update exist map
                    map->add_scan(frm.pts, dxi);

                    // update merge map
                    unimap->update_map(map);

                    // create new submap
                    if(map->num >= update_config.robot_slam_submap_cnt)
                    {
                        // loop closing
                        if(submaps.size() >= 2)
                        {
                            bool is_found = false;
                            for(size_t p = 0; p < submaps.size()-1; p++)
                            {
                                cv::Vec3d xi0 = submaps[p].xi;
                                cv::Vec3d xi1 = submaps[submaps.size()-1].xi;
                                cv::Vec3d lc_dxi = divXi(xi1, xi0);

                                double d = cv::norm(cv::Vec2d(lc_dxi[0], lc_dxi[1]));
                                if(d < update_config.robot_slam_lc_dist)
                                {
                                    if(submap_to_submap_icp2(submaps[p], submaps[submaps.size()-1], lc_dxi) || is_manual_lc == true)
                                    {
                                        is_manual_lc = false;

                                        // lc found
                                        int lc_id0 = p;
                                        int lc_id1 = submaps.size()-1;

                                        LC_INFO lc;
                                        lc.id0 = lc_id0;
                                        lc.id1 = lc_id1;
                                        lc.dxi = lc_dxi;
                                        lc_infos.push_back(lc);

                                        printf("LC id: %d-%d, dxi: %.3f, %.3f, %.1f\n", lc_id0, lc_id1, lc_dxi[0], lc_dxi[1], lc_dxi[2]*R2D);
                                        is_found = true;
                                        break;
                                    }
                                }
                            }

                            if(is_found)
                            {
                                // pose graph optimization
                                pose_graph_optimization(submaps, lc_infos);

                                // redraw merged map
                                cv::Mat merge_map(unimap->map_h, unimap->map_w, CV_8U, cv::Scalar(0));
                                for(size_t p = 0; p < submaps.size(); p++)
                                {
                                    unimap->update_map(merge_map, &submaps[p]);
                                }
                                unimap->set_map(merge_map);

                                printf("pose graph optimized, redrawing %d\n", (int)submaps.size());
                            }
                        }

                        // create new submap
                        SUBMAP _map;
                        _map.id = submaps.size();
                        _map.xi = mulXi(map->xi, map->dxi);
                        _map.add_scan(frm.pts, cv::Vec3d(0, 0, 0));
                        submaps.push_back(_map);
                    }
                }

                // set global pose and current scan
                cv::Vec3d global_pose = mulXi(map->xi, map->dxi);
                std::vector<cv::Vec2d> _cur_scan_global = transform(frm.pts, global_pose);

                mtx.lock();
                cur_pose = global_pose;
                cur_scan = frm.pts;
                cur_scan_pose = global_pose;
                cur_scan_global = _cur_scan_global;
                mtx.unlock();

                // for next
                pre_frm = frm;
            }

            // for speed
            lidar->scan_que.clear();
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void SLAM_2D::pose_graph_optimization(std::vector<SUBMAP>& _submaps, std::vector<LC_INFO>& _lc_infos)
{
    p2o::Pose2DVec nodes;
    p2o::Con2DVec edges;
    p2o::Optimizer2D optimizer;
    optimizer.setLambda(1e-6);
    optimizer.setVerbose(false);

    double robust_th = 1.0;
    optimizer.setRobustThreshold(robust_th);

    // set node
    for(size_t p = 0; p < _submaps.size(); p++)
    {
        p2o::Pose2D pose;
        pose.x = _submaps[p].xi[0];
        pose.y = _submaps[p].xi[1];
        pose.th = _submaps[p].xi[2];
        nodes.push_back(pose);
    }

    // set edge
    for(size_t p = 1; p < _submaps.size(); p++)
    {
        cv::Vec3d dxi = divXi(_submaps[p].xi, _submaps[p-1].xi);

        p2o::Con2D con;
        con.id1 = p-1;
        con.id2 = p;
        con.t = p2o::Pose2D(dxi[0], dxi[1], dxi[2]);
        con.info << 1, 0, 0,
                    0, 1, 0,
                    0, 0, 1;

        edges.push_back(con);
    }

    // set lc edge
    for(size_t p = 0; p < _lc_infos.size(); p++)
    {
        p2o::Con2D con;
        con.id1 = _lc_infos[p].id0;
        con.id2 = _lc_infos[p].id1;
        con.t = p2o::Pose2D(_lc_infos[p].dxi[0], _lc_infos[p].dxi[1], _lc_infos[p].dxi[2]);
        con.info << 1, 0, 0,
                    0, 1, 0,
                    0, 0, 1;

        edges.push_back(con);
    }

    // solve
    p2o::Pose2DVec result = optimizer.optimizePath(nodes, edges, 20, 20);
    for(size_t p = 0; p < result.size(); p++)
    {
        _submaps[p].xi[0] = result[p].x;
        _submaps[p].xi[1] = result[p].y;
        _submaps[p].xi[2] = result[p].th;
    }
}

void SLAM_2D::locLoop()
{
    lidar->scan_que.clear();    
    cv::Vec3d pre_pose(0,0,0);
    double last_time = 0;

    std::vector<cv::Vec3d> pose_storage;

    // loop start
    while(locFlag)
    {
        // if lidar scan exists
        LIDAR_FRM frm;
        if(lidar->scan_que.try_pop(frm))
        {
            if(frm.pts.size() < 5)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }

            double cur_time = get_time();

            // get cur pose
            mtx.lock();
            cv::Vec3d _cur_pose = cur_pose;
            mtx.unlock();

            // set initial guess
            cv::Vec2d dtdr = dTdR(_cur_pose, pre_pose);
            double delta_time = cur_time - last_time;
            if(dtdr[0] > update_config.robot_icp_repeat_dist || delta_time > update_config.robot_icp_repeat_time)
            {
                // icp hard initial guess
                cv::Vec3d xi = _cur_pose;

                double ie = 0; double ir = 0;
                bool res = map_to_scan_icp2(frm.pts, xi, update_config.robot_icp_dist, ie, ir);

                // update for watchdog
                loc_inlier_error = ie;
                loc_inlier_ratio = ir;

                is_loc = true;

                // icp
                if(res)
                {
                    // compensation current mobile moving
                    cv::Vec3d new_pose = xi;
                    cv::Vec3d old_pose = _cur_pose;

                    // moving average
                    double alpha = update_config.robot_icp_odometry_weight;

                    cv::Vec3d smooth_pose;
                    smooth_pose[0] = alpha*old_pose[0] + (1-alpha)*new_pose[0];
                    smooth_pose[1] = alpha*old_pose[1] + (1-alpha)*new_pose[1];

                    cv::Vec2d old_th_vec;
                    old_th_vec[0] = std::cos(old_pose[2]);
                    old_th_vec[1] = std::sin(old_pose[2]);

                    cv::Vec2d new_th_vec;
                    new_th_vec[0] = std::cos(new_pose[2]);
                    new_th_vec[1] = std::sin(new_pose[2]);

                    cv::Vec2d smooth_vec = alpha*old_th_vec + (1-alpha)*new_th_vec;
                    smooth_pose[2] = std::atan2(smooth_vec[1], smooth_vec[0]);
                    _cur_pose = smooth_pose;

                    // update pose
                    LOC loc;
                    loc.pose = _cur_pose;
                    loc.mobile_pose = frm.mobile_pose;
                    loc_que.push(loc);

                    if(is_travel != 0)
                    {
                        pose_storage.push_back(new_pose);
                        if(pose_storage.size() >= 2)
                        {
                            unimap->update_travel_map(pose_storage);
                            pose_storage.erase(pose_storage.begin());
                        }
                    }
                    else
                    {
                        pose_storage.clear();
                    }

                }
                else
                {
                    printf("[ICP] fail, pose:%.2f,%.2f,%.1f, ie:%f, ir:%f\n", _cur_pose[0], _cur_pose[1], _cur_pose[2]*R2D, ie, ir);
                }

                // for icp processing rate
                pre_pose = _cur_pose;
                last_time = get_time();
            }

            // set global scan
            std::vector<cv::Vec2d> _cur_scan_global = transform(frm.pts, _cur_pose);

            // update scan
            mtx.lock();
            cur_scan = frm.pts;
            cur_scan_pose = _cur_pose;
            cur_scan_global = _cur_scan_global;
            mtx.unlock();

            // for obs loop            
            TIME_POSE tp;
            tp.t = frm.t;
            tp.pose = _cur_pose;
            tp_que.push(tp);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void SLAM_2D::locLoop2()
{
    cv::Vec3d pre_mobile_pose = mobile->get_pose().pose;

    // loop start
    while(locFlag2)
    {
        // get cur pose
        mtx.lock();
        cv::Vec3d _cur_pose = cur_pose;
        mtx.unlock();

        // if lidar scan exists
        LOC loc;
        if(loc_que.try_pop(loc))
        {
            _cur_pose = loc.pose;
            pre_mobile_pose = loc.mobile_pose;
        }

        // add delta pose from mobile
        MOBILE_POSE mobile_pose = mobile->get_pose();
        cv::Vec3d d_mobile_pose = divXi(mobile_pose.pose, pre_mobile_pose);
        _cur_pose = mulXi(_cur_pose, d_mobile_pose);
        pre_mobile_pose = mobile_pose.pose;

        // pose update
        cur_pose_t = mobile_pose.t;

        mtx.lock();
        cur_pose = _cur_pose;
        mtx.unlock();

        // storing time pose2
        TIME_POSE tp;
        tp.t = mobile_pose.t;
        tp.pose = _cur_pose;
        tp_que2.push(tp);

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void SLAM_2D::obsLoop()
{
    std::vector<TIME_POSE> pose_storage;
    while(obsFlag)
    {
        TIME_POSE tp;
        if(tp_que.try_pop(tp))
        {
            // for cam
            TIME_POSE tp2;
            while(tp_que2.try_pop(tp2))
            {
                // pre pose storing
                pose_storage.push_back(tp2);
                if(pose_storage.size() > 500)
                {
                    pose_storage.erase(pose_storage.begin());
                }
            }

            // get global lidar data
            std::vector<cv::Vec2d> _cur_scan_global = get_cur_scan_global();

            // calc global camera scan
            cam->mtx_l.lock();
            double cam_t_l = cam->time_l;
            std::vector<cv::Vec2d> cam_scan_l = cam->cur_scan_l;
            cam->mtx_l.unlock();

            cam->mtx_r.lock();
            double cam_t_r = cam->time_r;
            std::vector<cv::Vec2d> cam_scan_r = cam->cur_scan_r;
            cam->mtx_r.unlock();

            // convert camera scan to global coordinates
            cv::Vec3d cam_pose_l = get_best_pose(cam_t_l, pose_storage);
            cv::Vec3d cam_pose_r = get_best_pose(cam_t_r, pose_storage);

            std::vector<cv::Vec2d> global_cam_scan_l = transform(cam_scan_l, cam_pose_l);
            std::vector<cv::Vec2d> global_cam_scan_r = transform(cam_scan_r, cam_pose_r);

            // update obstacle map
            std::vector<cv::Vec2d> merge_scan;
            merge_scan.insert(merge_scan.end(), _cur_scan_global.begin(), _cur_scan_global.end());
            merge_scan.insert(merge_scan.end(), global_cam_scan_l.begin(), global_cam_scan_l.end());
            merge_scan.insert(merge_scan.end(), global_cam_scan_r.begin(), global_cam_scan_r.end());

            unimap->update_dynamic_obs_map(merge_scan);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

bool SLAM_2D::map_to_scan(SUBMAP &map, std::vector<cv::Vec2d> &pts, cv::Vec3d &res, double &inlier_error, double &inlier_ratio)
{
    // set initial guess
    cv::Vec3d xi = res;

    // set variables
    int max_iter = 50;
    double lambda = 0.01;
    double pre_err = 9999;
    double first_err = 9999;    
    double convergence = 9999;
    int num = 0;
    int iter = 0;
    std::vector<double> errs;

    std::vector<cv::Vec2d> _pts;
    for(size_t p = 0; p < pts.size(); p++)
    {
        double range = cv::norm(pts[p]);
        if(range < update_config.robot_icp_near)
        {
            continue;
        }
        _pts.push_back(pts[p]);
    }

    // solve
    for (iter = 0; iter < max_iter; iter++)
    {
        std::vector<double> _residuals_sorted(_pts.size());
        std::vector<double> _residuals(_pts.size());
        std::vector<cv::Vec3d> _jacobians(_pts.size());
        std::vector<uchar> _mask(_pts.size(), 0);

        #pragma omp parallel for num_threads(3)
        for (size_t p = 0; p < _pts.size(); p++)
        {
            // get M_smooth
            cv::Vec2d P = _pts[p];
            double M = map.get_probability(P, xi);
            if (M < 0)
            {
                continue;
            }

            // residual
            double r = 1.0 - M;

            // jacobian
            double gu = 0;
            double gv = 0;
            map.get_gradient(P, xi, gu, gv);

            cv::Mat JM(1, 2, CV_64F, cv::Scalar(0));
            JM.ptr<double>(0)[0] = gu;
            JM.ptr<double>(0)[1] = gv;

            cv::Mat JU(2, 2, CV_64F, cv::Scalar(0));
            JU.ptr<double>(0)[0] = 1.0 / update_config.robot_grid_size;
            JU.ptr<double>(0)[1] = 0;
            JU.ptr<double>(1)[0] = 0;
            JU.ptr<double>(1)[1] = 1.0 / update_config.robot_grid_size;

            cv::Mat JG(2, 3, CV_64F, cv::Scalar(0));
            JG.ptr<double>(0)[0] = 1.0;
            JG.ptr<double>(0)[1] = 0;
            JG.ptr<double>(0)[2] = -std::sin(xi[2])*P[0] - std::cos(xi[2])*P[1];
            JG.ptr<double>(1)[0] = 0;
            JG.ptr<double>(1)[1] = 1.0;
            JG.ptr<double>(1)[2] = std::cos(xi[2])*P[0] - std::sin(xi[2])*P[1];

            cv::Mat _J = -JM*JU*JG;
            cv::Vec3d J;
            J[0] = _J.ptr<double>(0)[0];
            J[1] = _J.ptr<double>(0)[1];
            J[2] = _J.ptr<double>(0)[2];
            if (!isfinite(J[0]) || !isfinite(J[1]) || !isfinite(J[2]))
            {
                continue;
            }

            // store residual and jacobian
            _residuals_sorted[p] = r;
            _residuals[p] = r;
            _jacobians[p] = J;
            _mask[p] = 1;
        }

        std::vector<double> residuals_sorted;
        std::vector<double> residuals;
        std::vector<cv::Vec3d> jacobians;
        for(size_t p = 0; p < _mask.size(); p++)
        {
            if(_mask[p] == 0)
            {
                continue;
            }
            residuals_sorted.push_back(_residuals_sorted[p]);
            residuals.push_back(_residuals[p]);
            jacobians.push_back(_jacobians[p]);
        }

        // num of residuals
        num = residuals.size();
        if(num < 10)
        {
            printf("[STM-F] i:%d, not enough correspondence %d\n", iter, num);
            return false;
        }

        // student t distribution
        std::nth_element(residuals_sorted.begin(), residuals_sorted.begin() + residuals_sorted.size() / 2, residuals_sorted.end());
        double mu_d = residuals_sorted[residuals_sorted.size() / 2];

        std::vector<double> _d(residuals.size());

        #pragma omp parallel for num_threads(3)
        for (size_t p = 0; p < residuals.size(); p++)
        {
            _d[p] = residuals[p] - mu_d;
        }

        std::nth_element(_d.begin(), _d.begin() + _d.size() / 2, _d.end());
        double sigma_d = _d[_d.size() / 2];
        if (sigma_d < 0.01)
        {
            sigma_d = 0.01;
        }

        // fill matrix
        const double v0 = 30;
        cv::Mat d(num, 1, CV_64F, cv::Scalar(0));
        cv::Mat J(num, 3, CV_64F, cv::Scalar(0));

        #pragma omp parallel for num_threads(3)
        for (int p = 0; p < num; p++)
        {
            double r = residuals[p];
            double w = (v0 + 1) / (v0 + ((r - mu_d) / sigma_d)*((r - mu_d) / sigma_d));

            d.ptr<double>(p)[0] = w * residuals[p];
            J.ptr<double>(p)[0] = w * jacobians[p][0];
            J.ptr<double>(p)[1] = w * jacobians[p][1];
            J.ptr<double>(p)[2] = w * jacobians[p][2];
        }

        // calc matrix
        cv::Mat JtJ = J.t()*J;
        cv::Mat Jtd = J.t()*d;
        cv::Mat diagJtJ(3, 3, CV_64F, cv::Scalar(0));
        JtJ.copyTo(diagJtJ, cv::Mat::eye(3, 3, CV_8U));

        // calc residual error
        cv::Mat residualErr = d.t()*d;
        double err = sqrt(residualErr.ptr<double>(0)[0]) / num;
        errs.push_back(err);

        // save the first error
        if (iter == 0)
        {
            first_err = err;
        }

        // solve
        cv::Mat dxi = -(JtJ + lambda * diagJtJ).inv(cv::DecompTypes::DECOMP_CHOLESKY)*Jtd;        

        // update xi
        xi[0] = xi[0] + dxi.ptr<double>(0)[0];
        xi[1] = xi[1] + dxi.ptr<double>(1)[0];
        xi[2] = toWrap(xi[2] + dxi.ptr<double>(2)[0]);


        // update lambda
        if (err < pre_err)
        {
            lambda *= 0.1;
        }
        else
        {
            lambda *= std::pow(2, iter);
        }
        pre_err = err;

        // convergence
        double min, max;
        cv::minMaxLoc(abs(dxi), &min, &max);
        convergence = max;

        // save param
        if (iter > 0 && convergence < 1.0e-7)
        {
            break;
        }
    }

    inlier_ratio = (double)num/pts.size();
    inlier_error = std::sqrt(pre_err);

    // update pose
    printf("[STM-S] ID:%d, I:%d, NUM:%d, FE:%f, LE:%f, CG:%e\n", map.id, iter, num, first_err, pre_err, convergence);
    res = xi;
    return true;
}

bool SLAM_2D::submap_to_submap_icp2(SUBMAP &map0, SUBMAP &map1, cv::Vec3d &res)
{
    cv::Mat prob_map0 = map0.get_map_real();
    std::vector<cv::Vec2d> pts0;
    for(int i = 0; i < prob_map0.rows; i++)
    {
        for(int j = 0; j < prob_map0.cols; j++)
        {
            double val = prob_map0.ptr<double>(i)[j];
            if(val > WALL_THRESHOLD)
            {
                cv::Vec2d pt = map0.uv_xy(cv::Vec2i(j,i));
                pts0.push_back(pt);
            }
        }
    }

    cv::Mat prob_map1 = map1.get_map_real();
    std::vector<cv::Vec2d> pts1;
    for(int i = 0; i < prob_map1.rows; i++)
    {
        for(int j = 0; j < prob_map1.cols; j++)
        {
            double val = prob_map1.ptr<double>(i)[j];
            if(val > WALL_THRESHOLD)
            {
                cv::Vec2d pt = map1.uv_xy(cv::Vec2i(j,i));
                pts1.push_back(pt);
            }
        }
    }

    return scan_to_scan_icp(pts0, pts1, update_config.robot_slam_lc_icp_dist, res);
}

bool SLAM_2D::scan_to_scan_icp(std::vector<cv::Vec2d> pts0, std::vector<cv::Vec2d> pts1, double max_dist, cv::Vec3d &res)
{
    // nanoflann
    Eigen::MatrixXf _tree_pts;
    _tree_pts.resize(pts0.size(), 2);
    for(size_t p = 0; p < pts0.size(); p++)
    {
        _tree_pts(p, 0) = pts0[p][0];
        _tree_pts(p, 1) = pts0[p][1];
    }

    my_kd_tree_t *_tree = new my_kd_tree_t(2, std::cref(_tree_pts), 10);

    // set initial guess
    cv::Vec3d xi0 = res;
    cv::Vec3d inv_xi0 = invXi(xi0);

    // set variables
    int max_iter = 50;
    double lambda = 0.01;
    double last_err = 9999;
    double first_err = 9999;
    double convergence = 9999;
    int num = 0;
    int iter = 0;

    // solve
    cv::Vec3d xi(0,0,0);
    for(iter = 0; iter < max_iter; iter++)
    {
        // calc cost jacobian
        std::vector<double> weights;
        std::vector<double> residuals;
        std::vector<cv::Vec3d> jacobians;
        for(size_t p = 0; p < pts1.size(); p++)
        {
            // TF
            cv::Vec2d pt1 = pts1[p]; // for jacobian
            cv::Vec2d _pt1 = transform(xi, pts1[p]); // for cost
            cv::Vec2d __pt1 = transform(mulXi(xi0, xi), pts1[p]); // for query

            // find NN
            cv::Vec2d pt0(0,0);
            {
                std::vector<float> query(2);
                query[0] = __pt1[0];
                query[1] = __pt1[1];

                std::vector<size_t> indices(1);
                std::vector<float>  dists(1);

                nanoflann::KNNResultSet<float> resultSet(1);
                resultSet.init(&indices[0], &dists[0]);

                _tree->index_->findNeighbors(resultSet, &query[0]);

                cv::Vec2d _pt0;
                _pt0[0] = _tree_pts(indices[0], 0);
                _pt0[1] = _tree_pts(indices[0], 1);

                pt0 = transform(inv_xi0, _pt0); // for cost, jacobian
            }

            if(!isfinite(pt0[0]) || !isfinite(pt0[1]) || pt0 == cv::Vec2d(0,0))
            {
                continue;
            }

            // cost
            double r = (_pt1[0]-pt0[0])*(_pt1[0]-pt0[0]) + (_pt1[1]-pt0[1])*(_pt1[1]-pt0[1]);
            if(std::sqrt(r) > max_dist)
            {
                continue;
            }

            // jacobian
            cv::Vec3d J;
            J[0] = 2.0*(std::cos(xi[2])*pt1[0] - std::sin(xi[2])*pt1[1] + xi[0] - pt0[0]);
            J[1] = 2.0*(std::sin(xi[2])*pt1[0] + std::cos(xi[2])*pt1[1] + xi[1] - pt0[1]);
            J[2] = 2.0*(std::cos(xi[2])*pt1[0] - std::sin(xi[2])*pt1[1] + xi[0] - pt0[0])*(-std::sin(xi[2])*pt1[0] - std::cos(xi[2])*pt1[1])
                 + 2.0*(std::sin(xi[2])*pt1[0] + std::cos(xi[2])*pt1[1] + xi[1] - pt0[1])*(std::cos(xi[2])*pt1[0] - std::sin(xi[2])*pt1[1]);

            if(!isfinite(J[0]) || !isfinite(J[1]) || !isfinite(J[2]))
            {
                continue;
            }

            // weighting
            double w = 1.0;

            // storing
            residuals.push_back(r);
            jacobians.push_back(J);
            weights.push_back(w);
        }

        // num of residuals
        num = residuals.size();
        if(num < 30)
        {
            printf("[STS-F] i:%d, not enough correspondence %d\n", iter, num);
            return false;
        }

        // student t distribution
        std::vector<double> residuals_sorted = residuals;
        std::nth_element(residuals_sorted.begin(), residuals_sorted.begin() + residuals_sorted.size() / 2, residuals_sorted.end());
        double mu_d = residuals_sorted[residuals_sorted.size() / 2];

        std::vector<double> _d(residuals.size());
        for (size_t p = 0; p < residuals.size(); p++)
        {
            _d[p] = std::abs(residuals[p] - mu_d);
        }

        std::nth_element(_d.begin(), _d.begin() + _d.size() / 2, _d.end());
        double sigma_d = _d[_d.size() / 2];
        if (sigma_d < 0.001)
        {
            sigma_d = 0.001;
        }

        // fill matrix
        const double v0 = 15;
        cv::Mat d(num, 1, CV_64F, cv::Scalar(0));
        cv::Mat J(num, 3, CV_64F, cv::Scalar(0));
        for (int p = 0; p < num; p++)
        {
            double r = residuals[p];
            double w = weights[p]*((v0 + 1) / (v0 + ((r - mu_d) / sigma_d)*((r - mu_d) / sigma_d)));

            d.ptr<double>(p)[0] = w * residuals[p];
            J.ptr<double>(p)[0] = w * jacobians[p][0];
            J.ptr<double>(p)[1] = w * jacobians[p][1];
            J.ptr<double>(p)[2] = w * jacobians[p][2];
        }

        // calc matrix
        cv::Mat JtJ = J.t()*J;
        cv::Mat Jtd = J.t()*d;
        cv::Mat diagJtJ(3, 3, CV_64F, cv::Scalar(0));
        JtJ.copyTo(diagJtJ, cv::Mat::eye(3, 3, CV_8U));

        // calc residual error
        cv::Mat residualErr = d.t()*d;
        double err = sqrt(residualErr.ptr<double>(0)[0]) / num;

        // save the first error
        if (iter == 0)
        {
            first_err = err;
        }

        // solve
        cv::Mat sol = -(JtJ + lambda * diagJtJ).inv(cv::DecompTypes::DECOMP_CHOLESKY)*Jtd;

        // update xi
        cv::Vec3d dxi;
        dxi[0] = sol.ptr<double>(0)[0];
        dxi[1] = sol.ptr<double>(1)[0];
        dxi[2] = sol.ptr<double>(2)[0];
        xi = mulXi(dxi, xi);

        // update lambda
        if (err < last_err)
        {
            lambda *= 0.1;
        }
        else
        {
            lambda *= std::pow(2, iter);
        }
        last_err = err;

        // convergence
        double min, max;
        cv::minMaxLoc(abs(sol), &min, &max);
        convergence = max;
        if (iter > 0 && convergence < 1.0e-6)
        {
            break;
        }
    }

    // set result
    double ratio = (double)num/pts1.size();
    double error = std::sqrt(last_err);
    if(ratio < update_config.robot_icp_ratio || error > update_config.robot_icp_error)
    {
        printf("[STS-F] iter:%d, num:%d, fe:%f, le:%f, cg:%e\n", iter, num, first_err, last_err, convergence);
        return false;
    }

    // update pose
    // printf("[STS-S] iter:%d, num:%d/%d, fe:%f, le:%f, cg:%e\n", iter, num, (int)pts.size(), first_err, last_err, convergence);
    res = mulXi(xi0, xi);
    return true;
}

bool SLAM_2D::map_to_scan_icp2(std::vector<cv::Vec2d> &pts, cv::Vec3d &res, double max_dist, double &inlier_error, double &inlier_ratio)
{
    size_t _map_pts_size = unimap->map_pts_size;
    if(int(_map_pts_size) < 100)
    {
        QString str;
        str.sprintf("[AUTOINIT2] not enough map_pts. size:%d", int(_map_pts_size));
        logger.write(str, true);
        return false;
    }

    // set initial guess
    cv::Vec3d xi0 = res;
    cv::Vec3d inv_xi0 = invXi(xi0);

    // set variables
    int max_iter = 50;
    double lambda = 0.01;
    double last_err = 9999;
    double first_err = 9999;
    double convergence = 9999;
    int num = 0;
    int iter = 0;

    // solve
    cv::Vec3d xi(0,0,0);
    for(iter = 0; iter < max_iter; iter++)
    {
        // calc cost jacobian
        std::vector<double> weights;
        std::vector<double> residuals;
        std::vector<cv::Vec3d> jacobians;
        for(size_t p = 0; p < pts.size(); p++)
        {
            // TF
            cv::Vec2d pt1 = pts[p]; // for jacobian
            cv::Vec2d _pt1 = transform(xi, pts[p]); // for cost
            cv::Vec2d __pt1 = transform(mulXi(xi0, xi), pts[p]); // for query

            // find NN            
            cv::Vec2d pt0(0,0);
            {
                std::vector<float> query(2);
                query[0] = __pt1[0];
                query[1] = __pt1[1];

                std::vector<size_t> indices(1);
                std::vector<float>  dists(1);

                nanoflann::KNNResultSet<float> resultSet(1);
                resultSet.init(&indices[0], &dists[0]);

                tree->index_->findNeighbors(resultSet, &query[0]);

                cv::Vec2d _pt0;
                _pt0[0] = tree_pts(indices[0], 0);
                _pt0[1] = tree_pts(indices[0], 1);

                pt0 = transform(inv_xi0, _pt0); // for cost, jacobian
            }

            if(!isfinite(pt0[0]) || !isfinite(pt0[1]) || pt0 == cv::Vec2d(0,0))
            {
                continue;
            }

            // cost
            double r = (_pt1[0]-pt0[0])*(_pt1[0]-pt0[0]) + (_pt1[1]-pt0[1])*(_pt1[1]-pt0[1]);
            if(std::sqrt(r) > max_dist)
            {
                continue;
            }

            // jacobian
            cv::Vec3d J;
            J[0] = 2.0*(std::cos(xi[2])*pt1[0] - std::sin(xi[2])*pt1[1] + xi[0] - pt0[0]);
            J[1] = 2.0*(std::sin(xi[2])*pt1[0] + std::cos(xi[2])*pt1[1] + xi[1] - pt0[1]);
            J[2] = 2.0*(std::cos(xi[2])*pt1[0] - std::sin(xi[2])*pt1[1] + xi[0] - pt0[0])*(-std::sin(xi[2])*pt1[0] - std::cos(xi[2])*pt1[1])
                 + 2.0*(std::sin(xi[2])*pt1[0] + std::cos(xi[2])*pt1[1] + xi[1] - pt0[1])*(std::cos(xi[2])*pt1[0] - std::sin(xi[2])*pt1[1]);

            if(!isfinite(J[0]) || !isfinite(J[1]) || !isfinite(J[2]))
            {
                continue;
            }

            // weighting
            double w = 1.0;

            // storing
            residuals.push_back(r);
            jacobians.push_back(J);
            weights.push_back(w);
        }

        // num of residuals
        num = residuals.size();
        if(num < 30)
        {
            printf("[STM-F] i:%d, not enough correspondence %d\n", iter, num);
            return false;
        }

        // student t distribution
        std::vector<double> residuals_sorted = residuals;
        std::nth_element(residuals_sorted.begin(), residuals_sorted.begin() + residuals_sorted.size() / 2, residuals_sorted.end());
        double mu_d = residuals_sorted[residuals_sorted.size() / 2];

        std::vector<double> _d(residuals.size());
        for (size_t p = 0; p < residuals.size(); p++)
        {
            _d[p] = std::abs(residuals[p] - mu_d);
        }

        std::nth_element(_d.begin(), _d.begin() + _d.size() / 2, _d.end());
        double sigma_d = _d[_d.size() / 2];
        if (sigma_d < 0.001)
        {
            sigma_d = 0.001;
        }        

        // fill matrix
        const double v0 = 15;
        cv::Mat d(num, 1, CV_64F, cv::Scalar(0));
        cv::Mat J(num, 3, CV_64F, cv::Scalar(0));
        for (int p = 0; p < num; p++)
        {
            double r = residuals[p];
            double w = weights[p]*((v0 + 1) / (v0 + ((r - mu_d) / sigma_d)*((r - mu_d) / sigma_d)));

            d.ptr<double>(p)[0] = w * residuals[p];
            J.ptr<double>(p)[0] = w * jacobians[p][0];
            J.ptr<double>(p)[1] = w * jacobians[p][1];
            J.ptr<double>(p)[2] = w * jacobians[p][2];
        }

        // calc matrix
        cv::Mat JtJ = J.t()*J;
        cv::Mat Jtd = J.t()*d;
        cv::Mat diagJtJ(3, 3, CV_64F, cv::Scalar(0));
        JtJ.copyTo(diagJtJ, cv::Mat::eye(3, 3, CV_8U));

        // calc residual error
        cv::Mat residualErr = d.t()*d;
        double err = sqrt(residualErr.ptr<double>(0)[0]) / num;

        // save the first error
        if (iter == 0)
        {
            first_err = err;
        }

        // solve
        cv::Mat sol = -(JtJ + lambda * diagJtJ).inv(cv::DecompTypes::DECOMP_CHOLESKY)*Jtd;

        // update xi
        cv::Vec3d dxi;
        dxi[0] = sol.ptr<double>(0)[0];
        dxi[1] = sol.ptr<double>(1)[0];
        dxi[2] = sol.ptr<double>(2)[0];
        xi = mulXi(dxi, xi);

        // update lambda
        if (err < last_err)
        {
            lambda *= 0.1;
        }
        else
        {
            lambda *= std::pow(2, iter);
        }
        last_err = err;

        // convergence
        double min, max;
        cv::minMaxLoc(abs(sol), &min, &max);
        convergence = max;
        if (iter > 0 && convergence < 1.0e-6)
        {
            break;
        }
    }

    // set result
    inlier_ratio = (double)num/pts.size();
    inlier_error = std::sqrt(last_err);

    if(inlier_ratio < update_config.robot_icp_ratio || inlier_error > update_config.robot_icp_error)
    {
        printf("[STM-F] iter:%d, num:%d, fe:%f, le:%f, cg:%e\n", iter, num, first_err, last_err, convergence);
        return false;
    }

    // update pose
    // printf("[STM-S] iter:%d, num:%d/%d, fe:%f, le:%f, cg:%e\n", iter, num, (int)pts.size(), first_err, last_err, convergence);
    res = mulXi(xi0, xi);
    return true;
}

double SLAM_2D::saturation(double val, double min, double max)
{
    double _min = std::min<double>(min, max);
    double _max = std::max<double>(min, max);

    if(val <= _min)
    {
        val = _min;
    }
    else if(val >= _max)
    {
        val = _max;
    }

    return val;
}

std::vector<cv::Vec3d> SLAM_2D::transform(std::vector<cv::Vec3d> pts, cv::Vec3d xi)
{
    cv::Matx22d R;
    R(0, 0) = std::cos(xi[2]);
    R(0, 1) = -std::sin(xi[2]);
    R(1, 0) = std::sin(xi[2]);
    R(1, 1) = std::cos(xi[2]);

    cv::Vec2d T;
    T[0] = xi[0];
    T[1] = xi[1];

    std::vector<cv::Vec3d> res;
    for(size_t p = 0; p < pts.size(); p++)
    {
        cv::Vec2d _P = R * cv::Vec2d(pts[p][0], pts[p][1]) + T;
        res.push_back(cv::Vec3d(_P[0], _P[1], pts[p][2]));
    }
    return res;
}

std::vector<cv::Vec2i> SLAM_2D::transform(std::vector<cv::Vec2i> pts, cv::Vec3d xi)
{
    int cu = unimap->map_ou;
    int cv = unimap->map_ov;

    cv::Matx22d R;
    R(0, 0) = std::cos(xi[2]);
    R(0, 1) = -std::sin(xi[2]);
    R(1, 0) = std::sin(xi[2]);
    R(1, 1) = std::cos(xi[2]);

    cv::Vec2d T;
    T[0] = xi[0]-cu;
    T[1] = xi[1]-cv;

    std::vector<cv::Vec2i> res;
    for (size_t p = 0; p < pts.size(); p++)
    {
        cv::Vec2d pt(pts[p][0], pts[p][1]);
        cv::Vec2d rpt = R * pt + T;

        int u = rpt[0];
        int v = rpt[1];

        res.push_back(cv::Vec2i(u, v));
    }

    return res;
}

std::vector<cv::Vec2d> SLAM_2D::transform(std::vector<cv::Vec2d> pts, cv::Vec3d xi)
{
    cv::Matx22d R;
    R(0, 0) = std::cos(xi[2]);
    R(0, 1) = -std::sin(xi[2]);
    R(1, 0) = std::sin(xi[2]);
    R(1, 1) = std::cos(xi[2]);

    cv::Vec2d T;
    T[0] = xi[0];
    T[1] = xi[1];

    std::vector<cv::Vec2d> res;
    for(size_t p = 0; p < pts.size(); p++)
    {
        cv::Vec2d _P = R * pts[p] + T;
        res.push_back(_P);
    }
    return res;
}

cv::Vec2d SLAM_2D::transform(cv::Vec3d xi, cv::Vec2d P)
{
    cv::Matx22d R;
    R(0, 0) = std::cos(xi[2]);
    R(0, 1) = -std::sin(xi[2]);
    R(1, 0) = std::sin(xi[2]);
    R(1, 1) = std::cos(xi[2]);

    cv::Vec2d t;
    t[0] = xi[0];
    t[1] = xi[1];

    cv::Vec2d _P = R * P + t;
    return _P;
}

cv::Vec3d SLAM_2D::invXi(cv::Vec3d xi)
{
    cv::Vec3d res;

    cv::Matx22d R_inv;
    R_inv(0, 0) = std::cos(xi[2]);
    R_inv(0, 1) = std::sin(xi[2]);
    R_inv(1, 0) = -std::sin(xi[2]);
    R_inv(1, 1) = std::cos(xi[2]);

    cv::Vec2d v(xi[0], xi[1]);
    cv::Vec2d _v = -R_inv*v;

    res[0] = _v[0];
    res[1] = _v[1];
    res[2] = -xi[2];
    return res;
}

cv::Vec3d SLAM_2D::mulXi(cv::Vec3d xi0, cv::Vec3d xi1)
{
    cv::Matx22d R0;
    R0(0, 0) = std::cos(xi0[2]);
    R0(0, 1) = -std::sin(xi0[2]);
    R0(1, 0) = std::sin(xi0[2]);
    R0(1, 1) = std::cos(xi0[2]);

    cv::Vec2d t1;
    t1[0] = xi1[0];
    t1[1] = xi1[1];

    cv::Vec2d _t1 = R0 * t1;

    cv::Vec3d xi;
    xi[0] = xi0[0] + _t1[0];
    xi[1] = xi0[1] + _t1[1];
    xi[2] = toWrap(xi0[2] + xi1[2]);
    return xi;
}

cv::Vec3d SLAM_2D::divXi(cv::Vec3d xi0, cv::Vec3d xi1)
{
    cv::Matx22d R1_inv;
    R1_inv(0, 0) = std::cos(xi1[2]);
    R1_inv(0, 1) = std::sin(xi1[2]);
    R1_inv(1, 0) = -std::sin(xi1[2]);
    R1_inv(1, 1) = std::cos(xi1[2]);

    cv::Vec2d t0;
    t0[0] = xi0[0];
    t0[1] = xi0[1];

    cv::Vec2d t1;
    t1[0] = xi1[0];
    t1[1] = xi1[1];

    cv::Vec2d _t = R1_inv * (t0-t1);

    cv::Vec3d xi;
    xi[0] = _t[0];
    xi[1] = _t[1];
    xi[2] = toWrap(xi0[2] - xi1[2]);
    return xi;
}

cv::Vec3d SLAM_2D::intpXi(double a, double b, cv::Vec3d xi0, cv::Vec3d xi1)
{
    cv::Vec3d res;
    res[0] = a*xi0[0] + b*xi1[0];
    res[1] = a*xi0[1] + b*xi1[1];

    cv::Vec2d r_vec0;
    r_vec0[0] = std::cos(xi0[2]);
    r_vec0[1] = std::sin(xi0[2]);

    cv::Vec2d r_vec1;
    r_vec1[0] = std::cos(xi1[2]);
    r_vec1[1] = std::sin(xi1[2]);

    cv::Vec2d r_vec = a*r_vec0 + b*r_vec1;
    res[2] = std::atan2(r_vec[1], r_vec[0]);
    return res;
}

cv::Vec3d SLAM_2D::get_best_pose(double t, std::vector<TIME_POSE> &tp)
{
    int min_idx = 0;
    double min_t = 99999999;
    for(size_t p = 0; p < tp.size(); p++)
    {
        double dt = std::abs(t - tp[p].t);
        if(dt < min_t)
        {
            min_t = dt;
            min_idx = p;
        }
    }

    return tp[min_idx].pose;
}

cv::Vec2d SLAM_2D::dTdR(cv::Vec3d xi0, cv::Vec3d xi1)
{
    double dt = cv::norm(cv::Vec2d(xi1[0], xi1[1])-cv::Vec2d(xi0[0], xi0[1]));
    double dr = std::abs(deltaRad(xi1[2], xi0[2]));
    return cv::Vec2d(dt, dr);
}

cv::Mat SLAM_2D::maxfilter(cv::Mat map, int ch)
{
    if (ch == 0)
    {
        return map;
    }

    int w = map.cols;
    int h = map.rows;

    cv::Mat res(h, w, CV_8U, cv::Scalar(0));

    int bs = bnb_step[ch];
    for (int i = 0; i < h; i++)
    {
        for (int j = 0; j < w; j++)
        {
            double max = 0;
            for (int my = 0; my < bs; my++)
            {
                for (int mx = 0; mx < bs; mx++)
                {
                    int ii = i + my;
                    int jj = j + mx;

                    if (ii >= h || jj >= w)
                    {
                        continue;
                    }

                    uchar val = map.ptr<uchar>(ii)[jj];
                    if (val > max)
                    {
                        max = val;
                    }
                }
            }
            res.ptr<uchar>(i)[j] = max;
        }
    }
    return res;
}

cv::Mat SLAM_2D::resize_max_filter(cv::Mat &img, int w, int h)
{
    int old_w = img.cols;
    int old_h = img.cols;

    double scale_w = (double)w/old_w;
    double scale_h = (double)h/old_h;

    cv::Mat res(h, w, CV_8U, cv::Scalar(0));
    for(int i = 0; i < old_h; i++)
    {
        for(int j = 0; j < old_w; j++)
        {
            uchar val = img.ptr<uchar>(i)[j];

            int ii = i*scale_h;
            int jj = j*scale_w;
            if(ii < 0 || ii >= h || jj < 0 || jj >= w)
            {
                continue;
            }

            if(res.ptr<uchar>(ii)[jj] < val)
            {
               res.ptr<uchar>(ii)[jj] = val;
            }
        }
    }

    return res;
}

cv::Vec3d SLAM_2D::map_to_scan_bnb(cv::Mat _map, std::vector<cv::Vec2d> pts)
{
    cv::Mat map;    
    map = resize_max_filter(_map, 500, 500);
    //cv::imshow("map", map);

    double scale = 500.0/unimap->map_w;
    double grid_width = unimap->map_grid_width * (1.0/scale);

    const int bnb_fixed_height = 3;

    // max filtering
    cv::Mat map_p[bnb_fixed_height+1];
    for (int p = 0; p < bnb_fixed_height+1; p++)
    {
        map_p[p] = maxfilter(map, p);

        /*
        QString str;
        str.sprintf("map_p_%d", p);
        cv::imshow(str.toStdString(), map_p[p]);
        */
    }

    // root node expanding
    double score_threshold = (pts.size() * 255)*0.1;
    double best_score = score_threshold;

    const int w = map.cols;
    const int h = map.rows;

    // no initial guess
    cv::Vec3d xi0(0, 0, 0);

    // search resolution
    double r = grid_width;
    double dth = 1.0;

    // search range
    int wth = 180;
    int wx = w/2;
    int wy = h/2;

    int ch = bnb_fixed_height;

    std::vector<BNB_NODE> root;
    for (int cth = -wth; cth <= wth; cth++)
    {
        for (int cy = -wy; cy <= wy; cy += bnb_step[ch])
        {
            for (int cx = -wx; cx <= wx; cx += bnb_step[ch])
            {
                // center of search window, global frame
                cv::Vec3d xic;
                xic[0] = xi0[0] + cx * r;
                xic[1] = xi0[1] + cy * r;
                xic[2] = toWrap(xi0[2] + (cth * dth)*D2R);

                // get pixel coordinates on map from new position.
                cv::Matx22d R;
                R(0, 0) = std::cos(xic[2]);
                R(0, 1) = -std::sin(xic[2]);
                R(1, 0) = std::sin(xic[2]);
                R(1, 1) = std::cos(xic[2]);

                cv::Vec2d T;
                T[0] = xic[0];
                T[1] = xic[1];

                std::vector<cv::Vec2i> uv;
                for(size_t p = 0; p < pts.size(); p++)
                {
                    cv::Vec2d P = pts[p];
                    cv::Vec2d _P = R * P + T;

                    int u = std::round(_P[0]/grid_width + (w/2));
                    int v = std::round(_P[1]/grid_width + (h/2));
                    uv.push_back(cv::Vec2i(u,v));
                }

                // calc score
                double score = 0;
                for (size_t p = 0; p < uv.size(); p++)
                {
                    int u = uv[p][0];
                    int v = uv[p][1];
                    if (u < 0 || u >= w || v < 0 || v >= h)
                    {
                        continue;
                    }
                    score += map_p[ch].ptr<uchar>(v)[u];
                }

                // storing node
                if (score > best_score)
                {
                    root.push_back(BNB_NODE(cx, cy, cth, ch, score, uv));
                }
            }
        }
    }

    if(root.size() == 0)
    {
        printf("root zero\n");
        return cv::Vec3d(0, 0, 0);
    }

    // sort root node(accending order, - largest score last)
    if(root.size() >= 2)
    {
        std::sort(root.begin(), root.end(),[](const BNB_NODE & a, const BNB_NODE & b) -> bool{return a.score < b.score;});
    }

    // into stack
    std::stack<BNB_NODE> C;
    for (size_t p = 0; p < root.size(); p++)
    {
        C.push(root[p]);
    }

    // branch and bound algorithm
    while (!C.empty())
    {
        BNB_NODE c = C.top();
        C.pop();
        if (c.score > best_score)
        {
            if (c.ch == 0)
            {
                // leaf node
                cv::Vec3d xic;
                xic[0] = xi0[0] + c.cx * r;
                xic[1] = xi0[1] + c.cy * r;
                xic[2] = toWrap(xi0[2] + (c.cth * dth)*D2R);
                best_score = c.score;

                QString str;
                str.sprintf("[bnb] x:%.2f, y:%.2f, th:%.2f, score:%.2f", xic[0], xic[1], xic[2]*R2D, best_score);
                logger.write(str, true);

                return xic;
            }
            else
            {
                // branch
                std::vector<BNB_NODE> child(4);
                for (int i = 0; i < 2; i++)
                {
                    for (int j = 0; j < 2; j++)
                    {
                        // calc score
                        std::vector<cv::Vec2i> uv;
                        double score = 0;
                        for (size_t p = 0; p < c.uv.size(); p++)
                        {
                            int u = c.uv[p][0] + j * bnb_step[c.ch - 1];
                            int v = c.uv[p][1] + i * bnb_step[c.ch - 1];
                            uv.push_back(cv::Vec2i(u, v));
                            if (u < 0 || u >= w || v < 0 || v >= h)
                            {
                                continue;
                            }
                            score += map_p[c.ch - 1].ptr<uchar>(v)[u];
                        }

                        // storing child
                        child[i * 2 + j].cx = c.cx + j * bnb_step[c.ch - 1];
                        child[i * 2 + j].cy = c.cy + i * bnb_step[c.ch - 1];
                        child[i * 2 + j].cth = c.cth;
                        child[i * 2 + j].ch = c.ch - 1;
                        child[i * 2 + j].score = score;
                        child[i * 2 + j].uv = uv;
                    }
                }

                // sorting child
                if(child.size() >= 2)
                {
                    std::sort(child.begin(), child.end(), [](const BNB_NODE & a, const BNB_NODE & b) -> bool {return a.score < b.score; });
                }

                // into stack
                for (size_t p = 0; p < child.size(); p++)
                {
                    C.push(child[p]);
                }
            }
        }
    }
}
