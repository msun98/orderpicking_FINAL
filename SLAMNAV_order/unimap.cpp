#include "unimap.h"

UNIMAP::UNIMAP(QObject *parent) : QObject(parent)
{
    is_loaded = false;

    raw_map = cv::Mat(map_h, map_w, CV_8U, cv::Scalar::all(0));        
    dynamic_cnt_map = cv::Mat(map_h, map_w, CV_8U, cv::Scalar::all(0));
    static_obs_map = cv::Mat(map_h, map_w, CV_8U, cv::Scalar::all(0));
    dynamic_obs_map = cv::Mat(map_h, map_w, CV_8U, cv::Scalar::all(0));
    extended_static_obs_map = cv::Mat(map_h, map_w, CV_8U, cv::Scalar::all(0));
    extended_dynamic_obs_map = cv::Mat(map_h, map_w, CV_8U, cv::Scalar::all(0));
    extended_dynamic_obs_near_map = cv::Mat(map_h, map_w, CV_8U, cv::Scalar::all(0));
    travel_map = cv::Mat(map_h, map_w, CV_8U, cv::Scalar::all(0));    
    ui_travel_map = cv::Mat(map_h, map_w, CV_8U, cv::Scalar::all(0));
    velocity_map = cv::Mat(map_h, map_w, CV_8U, cv::Scalar::all(0));
    avoid_map = cv::Mat(map_h, map_w, CV_8U, cv::Scalar::all(0));
}

void UNIMAP::update_map(SUBMAP* submap)
{
    mtx.lock();
    cv::Mat _map = raw_map.clone();
    mtx.unlock();

    cv::Matx22d R;
    R(0, 0) = std::cos(submap->xi[2]);
    R(0, 1) = -std::sin(submap->xi[2]);
    R(1, 0) = std::sin(submap->xi[2]);
    R(1, 1) = std::cos(submap->xi[2]);

    cv::Vec2d submap_center = R * cv::Vec2d(submap->ou, submap->ov);
    cv::Vec2d global_center = xy_uvd(cv::Vec2d(submap->xi[0], submap->xi[1]));

    cv::Matx23d T;
    T(0, 0) = std::cos(submap->xi[2]);
    T(0, 1) = -std::sin(submap->xi[2]);
    T(0, 2) = std::round(global_center[0] - submap_center[0]);
    T(1, 0) = std::sin(submap->xi[2]);
    T(1, 1) = std::cos(submap->xi[2]);
    T(1, 2) = std::round(global_center[1] - submap_center[1]);

    cv::Mat src = submap->get_map_gray();
    cv::Mat dst;
    cv::warpAffine(src, dst, T, raw_map.size(), cv::InterpolationFlags::INTER_NEAREST);

    int n = map_w*map_h;
    #pragma omp parallel for num_threads(2)
    for(int p = 0; p < n; p++)
    {
        int i = p/map_w;
        int j = p%map_w;

        uchar val = dst.ptr<uchar>(i)[j];
        if(val == 0)
        {
            continue;
        }

        uchar pre_val = _map.ptr<uchar>(i)[j];
        if (pre_val == 0)
        {
            _map.ptr<uchar>(i)[j] = val;
            continue;
        }

        uchar pre_diff = std::abs(pre_val - 128);
        uchar cur_diff = std::abs(val - 128);
        if (cur_diff > pre_diff)
        {
            _map.ptr<uchar>(i)[j] = val;
        }
    }

    mtx.lock();
    raw_map = _map.clone();
    mtx.unlock();
}

void UNIMAP::update_map(cv::Mat &_map, SUBMAP* submap)
{
    cv::Matx22d R;
    R(0, 0) = std::cos(submap->xi[2]);
    R(0, 1) = -std::sin(submap->xi[2]);
    R(1, 0) = std::sin(submap->xi[2]);
    R(1, 1) = std::cos(submap->xi[2]);

    cv::Vec2d submap_center = R * cv::Vec2d(submap->ou, submap->ov);
    cv::Vec2d global_center = xy_uvd(cv::Vec2d(submap->xi[0], submap->xi[1]));

    cv::Matx23d T;
    T(0, 0) = std::cos(submap->xi[2]);
    T(0, 1) = -std::sin(submap->xi[2]);
    T(0, 2) = std::round(global_center[0] - submap_center[0]);
    T(1, 0) = std::sin(submap->xi[2]);
    T(1, 1) = std::cos(submap->xi[2]);
    T(1, 2) = std::round(global_center[1] - submap_center[1]);

    cv::Mat src = submap->get_map_gray();
    cv::Mat dst;
    cv::warpAffine(src, dst, T, raw_map.size(), cv::InterpolationFlags::INTER_NEAREST);

    int n = map_w*map_h;
    #pragma omp parallel for num_threads(2)
    for(int p = 0; p < n; p++)
    {
        int i = p/map_w;
        int j = p%map_w;

        uchar val = dst.ptr<uchar>(i)[j];
        if(val == 0)
        {
            continue;
        }

        uchar chk_val_min = 0.2*255;
        uchar chk_val_max = 0.7*255;
        if(val > chk_val_min && val < chk_val_max)
        {
            continue;
        }

        uchar pre_val = _map.ptr<uchar>(i)[j];
        if (pre_val == 0)
        {
            _map.ptr<uchar>(i)[j] = val;
            continue;
        }

        _map.ptr<uchar>(i)[j] = 0.5*pre_val + 0.5*val;

        /*
        uchar pre_diff = std::abs(pre_val - 128);
        uchar cur_diff = std::abs(val - 128);
        if (cur_diff > pre_diff)
        {
            _map.ptr<uchar>(i)[j] = val;
        }
        */
    }
}

void UNIMAP::update_dynamic_obs_map(std::vector<cv::Vec2d> &pts)
{
    // get current obs map
    mtx.lock();    
    cv::Mat _dynamic_cnt_map = dynamic_cnt_map.clone();
    cv::Mat _static_obs_map = static_obs_map.clone();        
    mtx.unlock();

    // count up
    cv::Mat update_mask(map_h, map_w, CV_8U, cv::Scalar(0));
    for(size_t p = 0; p < pts.size(); p++)
    {
        cv::Vec2i uv = xy_uv(pts[p]);
        int u = uv[0];
        int v = uv[1];
        if(u < 0 || u >= map_w || v < 0 || v >= map_h)
        {
            continue;
        }

        int cnt = _dynamic_cnt_map.ptr<uchar>(v)[u];
        cnt++;
        if(cnt > 20)
        {
            cnt = 20;
        }

        _dynamic_cnt_map.ptr<uchar>(v)[u] = cnt;
        update_mask.ptr<uchar>(v)[u] = 255;
    }

    // count down
    for(int i = 0; i < map_h; i++)
    {
        for(int j = 0; j < map_w; j++)
        {
            if(update_mask.ptr<uchar>(i)[j] == 255)
            {
                continue;
            }

            int cnt = _dynamic_cnt_map.ptr<uchar>(i)[j];
            cnt--;
            if(cnt < 0)
            {
                cnt = 0;
            }
            _dynamic_cnt_map.ptr<uchar>(i)[j] = cnt;
        }
    }

    // erase dynamic obstacle on static obs map    
    cv::Mat candidate_mask(map_h, map_w, CV_8U, cv::Scalar(0));
    for(int i = 0; i < map_h; i++)
    {
        for(int j = 0; j < map_w; j++)
        {
            if(_static_obs_map.ptr<uchar>(i)[j] == 255)
            {
                _dynamic_cnt_map.ptr<uchar>(i)[j] = 0;
                continue;
            }

            int cnt = _dynamic_cnt_map.ptr<uchar>(i)[j];
            if(cnt < setting_config.robot_obs_detect_sensitivity)
            {
                _dynamic_cnt_map.ptr<uchar>(i)[j] = 0;
                continue;
            }

            candidate_mask.ptr<uchar>(i)[j] = 255;
        }
    }

    // area filtering
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(candidate_mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE, cv::Point());

    cv::Mat _dynamic_obs_map(map_h, map_w, CV_8U, cv::Scalar(0));
    for(size_t p = 0; p< contours.size(); p++)
    {
        if (contours[p].size() >= setting_config.robot_obs_detect_area)
        {
            cv::drawContours(_dynamic_obs_map, contours, p, cv::Scalar(255), cv::FILLED, 8);
        }
    }

    // set extended dynamic obs map margin
    int r_margin = (static_config.robot_radius + setting_config.robot_dynamic_obs_margin)/map_grid_width;
    cv::Mat _extended_dynamic_obs_map;
    cv::dilate(_dynamic_obs_map, _extended_dynamic_obs_map, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(r_margin*2 + 1, r_margin*2 + 1)));

    // set extended dynamic obs near map margin
    //int r_near_margin = (static_config.robot_radius + setting_config.robot_dynamic_obs_margin + setting_config.robot_obs_near_dist)/map_grid_width;
    //cv::Mat _extended_dynamic_obs_near_map;
    //cv::dilate(_dynamic_obs_map, _extended_dynamic_obs_near_map, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(r_near_margin*2 + 1, r_near_margin*2 + 1)));

    // update obs map
    mtx.lock();
    dynamic_obs_map = _dynamic_obs_map.clone();
    extended_dynamic_obs_map = _extended_dynamic_obs_map.clone();    
    //extended_dynamic_obs_near_map = _extended_dynamic_obs_near_map.clone();
    mtx.unlock();
}

void UNIMAP::update_travel_map(std::vector<cv::Vec3d> poses)
{
    if(!is_loaded)
    {
        logger.write("[UNIMAP] map not loaded", true);
        return;
    }

    mtx.lock();
    cv::Mat _travel_map = travel_map.clone();
    mtx.unlock();

    if(poses.size() >= 2)
    {
        cv::Vec2i uv0 = xy_uv(cv::Vec2d(poses[0][0], poses[0][1]));
        cv::Vec2i uv1 = xy_uv(cv::Vec2d(poses[1][0], poses[1][1]));
        cv::line(_travel_map, cv::Point(uv0[0], uv0[1]), cv::Point(uv1[0], uv1[1]), cv::Scalar(255), 1);
    }

    mtx.lock();
    travel_map = _travel_map.clone();
    mtx.unlock();
}

void UNIMAP::set_map(cv::Mat &_map)
{
    mtx.lock();
    raw_map = _map.clone();
    mtx.unlock();
}

void UNIMAP::clear_map()
{
    is_loaded = false;

    mtx.lock();

    // clear annotated locations
    charging_locs.clear();
    resting_locs.clear();
    cleaning_locs.clear();
    charging_names.clear();
    resting_names.clear();
    cleaning_names.clear();
    serving_group_names.clear();
    locs.clear();

    for(size_t i=0; i<serving_locs.size(); i++)
    {
        serving_locs[i].clear();
    }
    serving_locs.clear();

    for(size_t i=0; i<serving_names.size(); i++)
    {
        serving_names[i].clear();
    }
    serving_names.clear();

    // clear map
    raw_map = cv::Mat(map_h, map_w, CV_8U, cv::Scalar::all(0));
    travel_map = cv::Mat(map_h, map_w, CV_8U, cv::Scalar::all(0));
    ui_travel_map = cv::Mat(map_h, map_w, CV_8U, cv::Scalar::all(0));
    dynamic_cnt_map = cv::Mat(map_h, map_w, CV_8U, cv::Scalar::all(0));
    static_obs_map = cv::Mat(map_h, map_w, CV_8U, cv::Scalar::all(0));    
    dynamic_obs_map = cv::Mat(map_h, map_w, CV_8U, cv::Scalar::all(0));
    extended_static_obs_map = cv::Mat(map_h, map_w, CV_8U, cv::Scalar::all(0));
    extended_dynamic_obs_map = cv::Mat(map_h, map_w, CV_8U, cv::Scalar::all(0));
    extended_dynamic_obs_near_map = cv::Mat(map_h, map_w, CV_8U, cv::Scalar::all(0));
    velocity_map = cv::Mat(map_h, map_w, CV_8U, cv::Scalar::all(0));
    avoid_map = cv::Mat(map_h, map_w, CV_8U, cv::Scalar::all(0));

    // clear map points
    map_pts.clear();
    map_pts_size = 0;

    mtx.unlock();
}

void UNIMAP::clear_map_soft()
{
    is_loaded = false;

    mtx.lock();

    // clear annotated locations
    charging_locs.clear();
    resting_locs.clear();
    cleaning_locs.clear();
    charging_names.clear();
    resting_names.clear();
    cleaning_names.clear();
    serving_group_names.clear();
    locs.clear();

    for(size_t i=0; i<serving_locs.size(); i++)
    {
        serving_locs[i].clear();
    }
    serving_locs.clear();

    for(size_t i=0; i<serving_names.size(); i++)
    {
        serving_names[i].clear();
    }
    serving_names.clear();

    // clear map
    travel_map = cv::Mat(map_h, map_w, CV_8U, cv::Scalar::all(0));
    ui_travel_map = cv::Mat(map_h, map_w, CV_8U, cv::Scalar::all(0));
    dynamic_cnt_map = cv::Mat(map_h, map_w, CV_8U, cv::Scalar::all(0));
    static_obs_map = cv::Mat(map_h, map_w, CV_8U, cv::Scalar::all(0));
    dynamic_obs_map = cv::Mat(map_h, map_w, CV_8U, cv::Scalar::all(0));
    extended_static_obs_map = cv::Mat(map_h, map_w, CV_8U, cv::Scalar::all(0));
    extended_dynamic_obs_map = cv::Mat(map_h, map_w, CV_8U, cv::Scalar::all(0));
    extended_dynamic_obs_near_map = cv::Mat(map_h, map_w, CV_8U, cv::Scalar::all(0));
    velocity_map = cv::Mat(map_h, map_w, CV_8U, cv::Scalar::all(0));
    avoid_map = cv::Mat(map_h, map_w, CV_8U, cv::Scalar::all(0));

    mtx.unlock();
}

void UNIMAP::erase_one_dot_noise(cv::Mat& img)
{
    cv::Mat res = img.clone();
    for(int i = 1; i < img.rows-1; i++)
    {
        for(int j = 1; j < img.cols-1; j++)
        {
            if(img.ptr<uchar>(i)[j] == 0)
            {
                continue;
            }

            int cnt = 0;
            for(int my = -1; my <= 1; my++)
            {
                for(int mx = -1; mx <= 1; mx++)
                {
                    int ii = i+my;
                    int jj = j+mx;

                    if(img.ptr<uchar>(ii)[jj] == 255)
                    {
                        cnt++;
                    }
                }
            }

            if(cnt == 1)
            {
                res.ptr<uchar>(i)[j] = 0;
            }
        }
    }

    img = res.clone();
}

// new annotation
void UNIMAP::load_map(QString path)
{
    is_loaded = false;

    double st_time = get_time();
    mtx.lock();

    // set map dir
    map_dir = path;

    // load map
    QString raw_map_path = path + "/map_raw.png";
    QFileInfo raw_map_info(raw_map_path);
    if(raw_map_info.exists() && raw_map_info.isFile())
    {
        raw_map = cv::imread(raw_map_path.toStdString(), cv::IMREAD_GRAYSCALE);

        QString map_meta_path = path + "/map_meta.ini";
        QFileInfo map_meta_info(map_meta_path);
        if(map_meta_info.exists() && map_meta_info.isFile())
        {
            QSettings settings(map_meta_path, QSettings::IniFormat);
            map_w = settings.value("map_metadata/map_w", raw_map.cols).toInt();
            map_h = settings.value("map_metadata/map_h", raw_map.rows).toInt();
            map_ou = settings.value("map_metadata/map_origin_u", raw_map.cols/2).toInt();
            map_ov = settings.value("map_metadata/map_origin_v", raw_map.rows/2).toInt();
            map_cut_u = 0;
            map_cut_v = 0;
            map_angle = 0;
            map_pre_w = map_w;
            map_pre_h = map_h;
            map_grid_width = settings.value("map_metadata/map_grid_width", update_config.robot_grid_size).toDouble();
        }

        if(raw_map.rows != map_h || raw_map.cols != map_w)
        {
            mtx.unlock();
            logger.write("[UNIMAP] map_raw.png size is different map_meta.ini, check file", true);
            return;
        }
    }
    else
    {
        mtx.unlock();
        logger.write("[UNIMAP] can not found map_raw.png, check path", true);
        return;
    }

    QString edited_map_path = path + "/map_edited.png";
    QFileInfo edited_map_info(edited_map_path);
    if(edited_map_info.exists() && edited_map_info.isFile())
    {
        raw_map = cv::imread(edited_map_path.toStdString(), cv::IMREAD_GRAYSCALE);

        QString map_meta_path = path + "/map_meta.ini";
        QFileInfo map_meta_info(map_meta_path);
        if(map_meta_info.exists() && map_meta_info.isFile())
        {
            QSettings settings(map_meta_path, QSettings::IniFormat);
            map_w = settings.value("map_metadata/map_edited_w", raw_map.cols).toInt();
            map_h = settings.value("map_metadata/map_edited_h", raw_map.rows).toInt();
            map_ou = settings.value("map_metadata/map_edited_u", raw_map.cols/2).toInt();
            map_ov = settings.value("map_metadata/map_edited_v", raw_map.rows/2).toInt();
            map_cut_u = settings.value("map_metadata/map_edited_cut_u", 0).toInt();
            map_cut_v = settings.value("map_metadata/map_edited_cut_v", 0).toInt();
            map_angle = settings.value("map_metadata/map_edited_angle", 0).toDouble()*D2R;
            map_pre_w = settings.value("map_metadata/map_w", raw_map.cols).toInt();
            map_pre_h = settings.value("map_metadata/map_h", raw_map.rows).toInt();
            map_grid_width = settings.value("map_metadata/map_grid_width", update_config.robot_grid_size).toDouble();
        }

        if(raw_map.rows != map_h || raw_map.cols != map_w)
        {
            mtx.unlock();
            logger.write("[UNIMAP] map_edited.png size is different map_meta.ini, check file", true);
            return;
        }
    }

    // load travel map
    QString travel_map_path = path + "/map_travel_line.png";
    QFileInfo travel_map_info(travel_map_path);
    if(travel_map_info.exists() && travel_map_info.isFile())
    {
        travel_map = cv::imread(travel_map_path.toStdString(), cv::IMREAD_GRAYSCALE);
        if(travel_map.rows != map_h || travel_map.cols != map_w)
        {
            mtx.unlock();
            logger.write("[UNIMAP] map_travel_line.png size is different map_meta.ini, check file", true);
            return;
        }

        int r = static_config.robot_radius/map_grid_width;
        for(int i = 0; i < map_h; i++)
        {
            for(int j = 0; j < map_w; j++)
            {
                if(travel_map.ptr<uchar>(i)[j] == 255)
                {
                    cv::circle(raw_map, cv::Point(j,i), r, cv::Scalar(127), -1);
                }
            }
        }
    }
    else
    {
        travel_map = cv::Mat(map_h, map_w, CV_8U, cv::Scalar(0));
    }

    // load UI travel map
    QString ui_travel_map_path = path + "/map_travel_line_ui.png";
    QFileInfo ui_travel_map_info(ui_travel_map_path);
    if(ui_travel_map_info.exists() && ui_travel_map_info.isFile())
    {
        ui_travel_map = cv::imread(ui_travel_map_path.toStdString(), cv::IMREAD_GRAYSCALE);
        if(ui_travel_map.rows != map_h || ui_travel_map.cols != map_w)
        {
            mtx.unlock();
            logger.write("[UNIMAP] ui_map_travel_line.png size is different map_meta.ini, check file", true);
            return;
        }

        int r = static_config.robot_radius/map_grid_width;
        for(int i = 0; i < map_h; i++)
        {
            for(int j = 0; j < map_w; j++)
            {
                if(ui_travel_map.ptr<uchar>(i)[j] == 255)
                {
                    cv::circle(raw_map, cv::Point(j,i), r, cv::Scalar(127), -1);
                }
            }
        }
    }
    else
    {
        ui_travel_map = cv::Mat(map_h, map_w, CV_8U, cv::Scalar(0));
    }
    cv::add(travel_map, ui_travel_map, travel_map);
    erase_one_dot_noise(travel_map);

    // set map points for icp
    map_pts.clear();
    QString cloud_path = path + "/clouds.csv";
    QFileInfo cloud_info(cloud_path);
    if(cloud_info.exists() && cloud_info.isFile())
    {
        // add cut map pos
        cv::Vec2i crop_left_top = cv::Vec2i(map_cut_u, map_cut_v);
        int old_map_size = map_pre_w;
        int new_map_size = map_w;
        double grid_size = map_grid_width;

        cv::Vec2d T;
        T[0] = (old_map_size/2 - (crop_left_top[1]+new_map_size/2))*grid_size;
        T[1] = (old_map_size/2 - (crop_left_top[0]+new_map_size/2))*grid_size;

        double angle = -map_angle;
        cv::Matx22d rot;
        rot(0, 0) = std::cos(angle);
        rot(0, 1) = -std::sin(angle);
        rot(1, 0) = std::sin(angle);
        rot(1, 1) = std::cos(angle);

        QFile file(cloud_path);
        if(file.open(QIODevice::ReadOnly))
        {
            QTextStream in(&file);
            while(!in.atEnd())
            {
                QString line = in.readLine();
                QStringList str = line.split(",");
                double x = str[0].toDouble();
                double y = str[1].toDouble();
                cv::Vec2d pt = rot*cv::Vec2d(x,y)-T;

                cv::Vec2i uv = xy_uv(pt);
                int u = uv[0];
                int v = uv[1];
                if(u < 0 || u >= map_w || v < 0 || v >= map_h)
                {
                    continue;
                }

                if(raw_map.ptr<uchar>(v)[u] == 255)
                {
                    map_pts.push_back(pt);
                }
            }
        }

        cv::Mat chk_cloud_map(map_h, map_w, CV_8U, cv::Scalar(0));
        for(size_t p = 0; p < map_pts.size(); p++)
        {
            cv::Vec2i uv = xy_uv(map_pts[p]);
            int u = uv[0];
            int v = uv[1];
            if(u < 0 || u >= map_w || v < 0 || v >= map_h)
            {
                continue;
            }

            chk_cloud_map.ptr<uchar>(v)[u] = 255;
        }
        QString chk_path = path + "/map_chk.png";
        cv::imwrite(chk_path.toStdString(), chk_cloud_map);

        logger.write("[UNIMAP] cloud loaded", true);
    }
    else
    {
        for(int i = 0; i < raw_map.rows; i++)
        {
            for(int j = 0; j < raw_map.cols; j++)
            {
                if(raw_map.ptr<uchar>(i)[j] == 255)
                {
                    map_pts.push_back(uv_xy(cv::Vec2i(j,i)));
                }
            }
        }
    }
    map_pts_size = map_pts.size();

    // load velocity map
    QString velocity_map_path = path + "/map_velocity.png";
    QFileInfo velocity_map_info(velocity_map_path);
    if(velocity_map_info.exists() && velocity_map_info.isFile())
    {
        velocity_map = cv::imread(velocity_map_path.toStdString(), cv::IMREAD_GRAYSCALE);
        if(velocity_map.rows != map_h || velocity_map.cols != map_w)
        {
            mtx.unlock();
            logger.write("[UNIMAP] map_velocity.png size is different map_meta.ini, check file", true);
            return;
        }
    }
    else
    {
        velocity_map = cv::Mat(map_h, map_w, CV_8U, cv::Scalar(0));
    }

    // load avoid map
    QString avoid_map_path = path + "/map_avoid.png";
    QFileInfo avoid_map_info(avoid_map_path);
    if(avoid_map_info.exists() && avoid_map_info.isFile())
    {
        avoid_map = cv::imread(avoid_map_path.toStdString(), cv::IMREAD_GRAYSCALE);
    }
    else
    {
        avoid_map = cv::Mat(map_h, map_w, CV_8U, cv::Scalar(0));
    }

    // set obs map base
    dynamic_cnt_map = cv::Mat(map_h, map_w, CV_8U, cv::Scalar::all(0));
    dynamic_obs_map = cv::Mat(map_h, map_w, CV_8U, cv::Scalar(0));
    extended_dynamic_obs_map = cv::Mat(map_h, map_w, CV_8U, cv::Scalar(0));

    // load static obs map
    QString obs_map_path = path + "/map_obs.png";
    QFileInfo obs_map_info(obs_map_path);
    if(obs_map_info.exists() && obs_map_info.isFile())
    {
        static_obs_map = cv::imread(obs_map_path.toStdString(), cv::IMREAD_GRAYSCALE);
        if(static_obs_map.rows != map_h || static_obs_map.cols != map_w)
        {
            mtx.unlock();
            logger.write("[UNIMAP] map_obs.png size is different map_meta.ini, check file", true);
            return;
        }
    }
    else
    {
        static_obs_map = cv::Mat(map_h, map_w, CV_8U, cv::Scalar(0));
    }

    // set static obs map
    for(int i = 0; i < map_h; i++)
    {
        for(int j = 0; j < map_w; j++)
        {
            if(raw_map.ptr<uchar>(i)[j] == 255)
            {
                static_obs_map.ptr<uchar>(i)[j] = 255;
            }
        }
    }

    // static map extend
    extended_static_obs_map = cv::Mat(map_h, map_w, CV_8U, cv::Scalar(0));
    int r_margin = (static_config.robot_radius+setting_config.robot_static_obs_margin)/map_grid_width;
    for(int i = 0; i < map_h; i++)
    {
        for(int j = 0; j < map_w; j++)
        {
            if(static_obs_map.ptr<uchar>(i)[j] == 255)
            {
                cv::circle(extended_static_obs_map, cv::Point(j,i), r_margin, cv::Scalar(255), -1);
            }
        }
    }

    // load topology
    nodes.clear();
    QString topo_path = map_dir + "/topo.json";
    QFileInfo topo_info(topo_path);
    if(topo_info.exists() && topo_info.isFile())
    {
        if(topo_info.exists() && topo_info.isFile())
        {
            QFile topo_file(topo_path);
            if(topo_file.open(QIODevice::ReadOnly))
            {
                QByteArray data = topo_file.readAll();
                QJsonDocument doc = QJsonDocument::fromJson(data);

                QJsonArray arr = doc.array();
                Q_FOREACH(const QJsonValue &val, arr)
                {
                    QJsonObject obj = val.toObject();

                    NODE node;
                    node.id = obj["id"].toString();
                    node.attrib = obj["attrib"].toString();
                    node.pose = array_to_pose(obj["pose"].toArray());
                    node.linked = array_to_links(obj["linked"].toArray());

                    nodes.push_back(node);
                }
                topo_file.close();

                QString str;
                str.sprintf("[UNIMAP] %s loaded size:%d", topo_path.toStdString().c_str(), (int)nodes.size());
                logger.write(str, true);
            }
        }
    }

    mtx.unlock();

    // flag set
    is_loaded = true;

    // log
    QString str;
    str.sprintf("[UNIMAP] map load, spent time: %f", get_time()-st_time);
    logger.write(str, true);
}

// new annotation
void UNIMAP::load_map_soft(QString path)
{
    is_loaded = false;

    double st_time = get_time();
    mtx.lock();

    // set map dir
    map_dir = path;

    // load travel map
    QString travel_map_path = path + "/map_travel_line.png";
    QFileInfo travel_map_info(travel_map_path);
    if(travel_map_info.exists() && travel_map_info.isFile())
    {
        travel_map = cv::imread(travel_map_path.toStdString(), cv::IMREAD_GRAYSCALE);
        if(travel_map.rows != map_h || travel_map.cols != map_w)
        {
            mtx.unlock();
            logger.write("[UNIMAP] map_travel_line.png size is different map_meta.ini, check file", true);
            return;
        }

        int r = static_config.robot_radius/map_grid_width;
        for(int i = 0; i < map_h; i++)
        {
            for(int j = 0; j < map_w; j++)
            {
                if(travel_map.ptr<uchar>(i)[j] == 255)
                {
                    cv::circle(raw_map, cv::Point(j,i), r, cv::Scalar(127), -1);
                }
            }
        }
    }
    else
    {
        travel_map = cv::Mat(map_h, map_w, CV_8U, cv::Scalar(0));
    }

    // load UI travel map
    QString ui_travel_map_path = path + "/map_travel_line_ui.png";
    QFileInfo ui_travel_map_info(ui_travel_map_path);
    if(ui_travel_map_info.exists() && ui_travel_map_info.isFile())
    {
        ui_travel_map = cv::imread(ui_travel_map_path.toStdString(), cv::IMREAD_GRAYSCALE);
        if(ui_travel_map.rows != map_h || ui_travel_map.cols != map_w)
        {
            mtx.unlock();
            logger.write("[UNIMAP] ui_map_travel_line.png size is different map_meta.ini, check file", true);
            return;
        }

        int r = static_config.robot_radius/map_grid_width;
        for(int i = 0; i < map_h; i++)
        {
            for(int j = 0; j < map_w; j++)
            {
                if(ui_travel_map.ptr<uchar>(i)[j] == 255)
                {
                    cv::circle(raw_map, cv::Point(j,i), r, cv::Scalar(127), -1);
                }
            }
        }
    }
    else
    {
        ui_travel_map = cv::Mat(map_h, map_w, CV_8U, cv::Scalar(0));
    }
    cv::add(travel_map, ui_travel_map, travel_map);
    erase_one_dot_noise(travel_map);

    // load velocity map
    QString velocity_map_path = path + "/map_velocity.png";
    QFileInfo velocity_map_info(velocity_map_path);
    if(velocity_map_info.exists() && velocity_map_info.isFile())
    {
        velocity_map = cv::imread(velocity_map_path.toStdString(), cv::IMREAD_GRAYSCALE);
        if(velocity_map.rows != map_h || velocity_map.cols != map_w)
        {
            mtx.unlock();
            logger.write("[UNIMAP] map_velocity.png size is different map_meta.ini, check file", true);
            return;
        }
    }
    else
    {
        velocity_map = cv::Mat(map_h, map_w, CV_8U, cv::Scalar(0));
    }

    // load avoid map
    QString avoid_map_path = path + "/map_avoid.png";
    QFileInfo avoid_map_info(avoid_map_path);
    if(avoid_map_info.exists() && avoid_map_info.isFile())
    {
        avoid_map = cv::imread(avoid_map_path.toStdString(), cv::IMREAD_GRAYSCALE);
    }
    else
    {
        avoid_map = cv::Mat(map_h, map_w, CV_8U, cv::Scalar(0));
    }

    // set obs map base
    dynamic_cnt_map = cv::Mat(map_h, map_w, CV_8U, cv::Scalar::all(0));
    dynamic_obs_map = cv::Mat(map_h, map_w, CV_8U, cv::Scalar(0));
    extended_dynamic_obs_map = cv::Mat(map_h, map_w, CV_8U, cv::Scalar(0));

    // load static obs map
    QString obs_map_path = path + "/map_obs.png";
    QFileInfo obs_map_info(obs_map_path);
    if(obs_map_info.exists() && obs_map_info.isFile())
    {
        static_obs_map = cv::imread(obs_map_path.toStdString(), cv::IMREAD_GRAYSCALE);
        if(static_obs_map.rows != map_h || static_obs_map.cols != map_w)
        {
            mtx.unlock();
            logger.write("[UNIMAP] map_obs.png size is different map_meta.ini, check file", true);
            return;
        }
    }
    else
    {
        static_obs_map = cv::Mat(map_h, map_w, CV_8U, cv::Scalar(0));
    }

    // set static obs map
    for(int i = 0; i < map_h; i++)
    {
        for(int j = 0; j < map_w; j++)
        {
            if(raw_map.ptr<uchar>(i)[j] == 255)
            {
                static_obs_map.ptr<uchar>(i)[j] = 255;
            }
        }
    }

    // static map extend
    extended_static_obs_map = cv::Mat(map_h, map_w, CV_8U, cv::Scalar(0));
    int r_margin = (static_config.robot_radius + setting_config.robot_static_obs_margin)/map_grid_width;
    for(int i = 0; i < map_h; i++)
    {
        for(int j = 0; j < map_w; j++)
        {
            if(static_obs_map.ptr<uchar>(i)[j] == 255)
            {
                cv::circle(extended_static_obs_map, cv::Point(j,i), r_margin, cv::Scalar(255), -1);
            }
        }
    }

    mtx.unlock();

    // flag set
    is_loaded = true;

    // log
    QString str;
    str.sprintf("[UNIMAP] map load, spent time: %f", get_time()-st_time);
    logger.write(str, true);
}

void UNIMAP::save_travel_map()
{
    mtx.lock();
    QString travel_map_path = map_dir + "/map_travel_line.png";
    cv::imwrite(travel_map_path.toStdString(), travel_map);
    mtx.unlock();
}

void UNIMAP::save_static_obs_map()
{
    mtx.lock();
    QString obs_map_path = map_dir + "/map_obs.png";
    cv::imwrite(obs_map_path.toStdString(), static_obs_map);
    mtx.unlock();
}

void UNIMAP::load_locations()
{
    if(!is_loaded)
    {
        logger.write("[UNIMAP] map not loaded", true);
        return;
    }

    mtx.lock();

    // load locations from annotation.ini
    QString annotated_loc_path = map_dir + "/annotation.ini";
    QFileInfo annotated_loc_info(annotated_loc_path);
    if(annotated_loc_info.exists() && annotated_loc_info.isFile())
    {
        QSettings settings(annotated_loc_path, QSettings::IniFormat);

        charging_locs.clear();
        int charging_num = settings.value("charging_locations/num").toInt();
        for(int p = 0; p < charging_num; p++)
        {
            QString sec;
            sec.sprintf("charging_locations/loc%d", p);

            QString str = settings.value(sec).toString();
            QStringList str_list = str.split(",");

            QString name = str_list[0];
            double x = str_list[1].toDouble();
            double y = str_list[2].toDouble();
            double th = str_list[3].toDouble();

            charging_locs.push_back(cv::Vec3d(x,y,th));
            charging_names.push_back(name);            
        }

        resting_locs.clear();
        int resting_num = settings.value("resting_locations/num").toInt();
        for(int p = 0; p < resting_num; p++)
        {
            QString sec;
            sec.sprintf("resting_locations/loc%d", p);

            QString str = settings.value(sec).toString();
            QStringList str_list = str.split(",");

            QString name = str_list[0];
            double x = str_list[1].toDouble();
            double y = str_list[2].toDouble();
            double th = str_list[3].toDouble();

            resting_locs.push_back(cv::Vec3d(x,y,th));
            resting_names.push_back(name);
        }

        cleaning_locs.clear();
        int cleaning_num = settings.value("cleaning_locations/num").toInt();
        for(int p = 0; p < cleaning_num; p++)
        {
            QString sec;
            sec.sprintf("cleaning_locations/loc%d", p);

            QString str = settings.value(sec).toString();
            QStringList str_list = str.split(",");

            QString name = str_list[0];
            double x = str_list[1].toDouble();
            double y = str_list[2].toDouble();
            double th = str_list[3].toDouble();

            cleaning_locs.push_back(cv::Vec3d(x,y,th));
            cleaning_names.push_back(name);
        }

        // load serving locations
        for(size_t i=0; i<serving_locs.size(); i++)
        {
            serving_locs[i].clear();
        }
        serving_locs.clear();

        for(size_t i=0; i<serving_names.size(); i++)
        {
            serving_names[i].clear();
        }
        serving_names.clear();

        serving_group_names.clear();

        int group_num = settings.value("serving_locations/group").toInt();
        serving_locs.resize(group_num);
        serving_names.resize(group_num);
        serving_group_names.resize(group_num);

        for(int p = 0; p < group_num; p++)
        {
            QString gtr;
            gtr.sprintf("serving_%d/name", p);

            QString group_str = settings.value(gtr).toString();
            serving_group_names[p] = group_str;

            QString str;
            str.sprintf("serving_%d/num", p);

            int loc_num = settings.value(str).toInt();
            serving_locs[p].resize(loc_num);
            serving_names[p].resize(loc_num);

            for(int q = 0; q < loc_num; q++)
            {
                QString sec;
                sec.sprintf("serving_%d/loc%d", p, q);

                QStringList str_list = settings.value(sec).toString().split(",");

                QString name = str_list[0];
                double x = str_list[1].toDouble();
                double y = str_list[2].toDouble();
                double th = str_list[3].toDouble();

                serving_locs[p][q] = cv::Vec3d(x,y,th);
                serving_names[p][q] = name;
            }
        }

        // set whole locations
        locs.clear();
        locs.insert(locs.end(), charging_locs.begin(), charging_locs.end());
        locs.insert(locs.end(), resting_locs.begin(), resting_locs.end());

        for(size_t p = 0; p < serving_locs.size(); p++)
        {
            locs.insert(locs.end(), serving_locs[p].begin(), serving_locs[p].end());
        }        
    }
    else
    {
        // clear location info
        resting_locs.clear();
        serving_locs.clear();
        charging_locs.clear();
        cleaning_locs.clear();

        resting_names.clear();
        serving_names.clear();
        charging_names.clear();
        cleaning_names.clear();
        serving_group_names.clear();

        for(size_t i=0; i<serving_locs.size(); i++)
        {
            serving_locs[i].clear();
        }
        serving_locs.clear();

        for(size_t i=0; i<serving_names.size(); i++)
        {
            serving_names[i].clear();
        }
        serving_names.clear();
    }

    mtx.unlock();
}

std::vector<cv::Vec3d> UNIMAP::get_resting_locations()
{
    std::vector<cv::Vec3d> _resting_locs = resting_locs;
    return _resting_locs;
}

std::vector<cv::Vec3d> UNIMAP::get_charging_locations()
{
    std::vector<cv::Vec3d> _charging_locs = charging_locs;
    return _charging_locs;
}

std::vector<cv::Vec3d> UNIMAP::get_cleaning_locations()
{
    std::vector<cv::Vec3d> _cleaning_locs = cleaning_locs;
    return _cleaning_locs;
}

std::vector<std::vector<cv::Vec3d>> UNIMAP::get_serving_locations()
{
    std::vector<std::vector<cv::Vec3d>> _serving_locs = serving_locs;
    return _serving_locs;
}

std::vector<QString> UNIMAP::get_serving_group_names()
{
    std::vector<QString> _serving_group_names = serving_group_names;
    return _serving_group_names;
}

std::vector<std::vector<QString>> UNIMAP::get_serving_names()
{
    std::vector<std::vector<QString>> _serving_names = serving_names;
    return _serving_names;
}

void UNIMAP::save_map(QString path)
{
    // save slam result
    mtx.lock();
    cv::Mat _map = raw_map.clone();
    mtx.unlock();

    // raw map recoloring
    for(int i = 0; i < _map.rows; i++)
    {
        for(int j = 0; j < _map.cols; j++)
        {
            if(_map.ptr<uchar>(i)[j] >= WALL_THRESHOLD*255)
            {
                // wall
                _map.ptr<uchar>(i)[j] = 255;
            }
            else if(_map.ptr<uchar>(i)[j] == 0)
            {
                // non measured
                _map.ptr<uchar>(i)[j] = 0;
            }
            else
            {
                // floor
                _map.ptr<uchar>(i)[j] = 127;
            }
        }
    }

    // raw map save
    QString raw_map_path = path + "/map_raw.png";
    cv::imwrite(raw_map_path.toStdString(), _map);

    // map meta save
    QString map_meta_path = path + "/map_meta.ini";
    QSettings settings(map_meta_path, QSettings::IniFormat);
    settings.setValue("map_metadata/map_w", map_w);
    settings.setValue("map_metadata/map_h", map_h);
    settings.setValue("map_metadata/map_origin_u", map_ou);
    settings.setValue("map_metadata/map_origin_v", map_ov);
    settings.setValue("map_metadata/map_grid_width", map_grid_width);
}

cv::Mat UNIMAP::get_map_raw()
{
    mtx.lock();
    cv::Mat img = raw_map.clone();
    mtx.unlock();

    return img;
}

cv::Mat UNIMAP::get_map_plot()
{
    mtx.lock();
    cv::Mat _map = raw_map.clone();    
    cv::Mat _travel = travel_map.clone();
    cv::Mat _obs = static_obs_map.clone();
    cv::Mat _avoid = avoid_map.clone();
    mtx.unlock();

    cv::Mat plot_img;
    cv::cvtColor(_map, plot_img, cv::COLOR_GRAY2BGR);

    int r_travel = static_config.robot_radius/map_grid_width;
    int r_obs = (static_config.robot_radius + setting_config.robot_static_obs_margin)/map_grid_width;

    cv::Mat obs_margin(map_h, map_w, CV_8U, cv::Scalar(0));
    cv::Mat travel_margin(map_h, map_w, CV_8U, cv::Scalar(0));
    for(int i = 0; i < map_h; i++)
    {
        for(int j = 0; j < map_w; j++)
        {
            if(_map.ptr<uchar>(i)[j] == 255 || _obs.ptr<uchar>(i)[j] == 255)
            {
                cv::circle(obs_margin, cv::Point(j,i), r_obs, cv::Scalar(255), -1);
            }

            if(_travel.ptr<uchar>(i)[j] == 255)
            {
                cv::circle(travel_margin, cv::Point(j,i), r_travel, cv::Scalar(255), -1);
            }
        }
    }

    for(int i = 0; i < map_h; i++)
    {
        for(int j = 0; j < map_w; j++)
        {
            if(obs_margin.ptr<uchar>(i)[j] == 255)
            {
                cv::Vec3b c = plot_img.ptr<cv::Vec3b>(i)[j];
                c[1] = 0.8*c[1] + 0.2*255;
                c[2] = 0.8*c[2] + 0.2*255;
                plot_img.ptr<cv::Vec3b>(i)[j] = c;
            }

            if(travel_margin.ptr<uchar>(i)[j] == 255)
            {
                cv::Vec3b c = plot_img.ptr<cv::Vec3b>(i)[j];
                c[0] = 0.7*c[0] + 0.3*255;
                plot_img.ptr<cv::Vec3b>(i)[j] = c;
            }

            if(_avoid.ptr<uchar>(i)[j] == 255)
            {
                cv::Vec3b c = plot_img.ptr<cv::Vec3b>(i)[j];
                c[0] = 0.7*c[0] + 0.3*255;
                c[2] = 0.7*c[2] + 0.3*255;
                plot_img.ptr<cv::Vec3b>(i)[j] = c;
            }
        }
    }

    return plot_img;
}

cv::Mat UNIMAP::get_map_travel()
{
    mtx.lock();
    cv::Mat img = travel_map.clone();
    mtx.unlock();

    return img;
}

cv::Mat UNIMAP::get_map_dynamic_obs()
{
    mtx.lock();
    cv::Mat img = dynamic_obs_map.clone();
    mtx.unlock();

    return img;
}

cv::Mat UNIMAP::get_map_static_obs()
{
    mtx.lock();
    cv::Mat img = static_obs_map.clone();
    mtx.unlock();

    return img;
}

cv::Mat UNIMAP::get_map_obs()
{
    mtx.lock();
    cv::Mat img0 = static_obs_map.clone();
    cv::Mat img1 = dynamic_obs_map.clone();
    mtx.unlock();

    cv::Mat res;
    cv::add(img0, img1, res);

    return res;
}

cv::Mat UNIMAP::get_map_avoid()
{
    mtx.lock();
    cv::Mat img = avoid_map.clone();
    mtx.unlock();

    return img;
}

cv::Mat UNIMAP::get_map_extended_dynamic_obs()
{
    mtx.lock();
    cv::Mat img = extended_dynamic_obs_map.clone();
    mtx.unlock();

    return img;
}

cv::Mat UNIMAP::get_map_extended_static_obs()
{
    mtx.lock();
    cv::Mat img = extended_static_obs_map.clone();
    mtx.unlock();

    return img;
}

cv::Mat UNIMAP::get_map_extended_dynamic_obs_near()
{
    mtx.lock();
    cv::Mat img = extended_dynamic_obs_near_map.clone();
    mtx.unlock();

    return img;
}

cv::Mat UNIMAP::get_map_extended_obs()
{
    mtx.lock();
    cv::Mat img0 = extended_dynamic_obs_map.clone();
    cv::Mat img1 = extended_static_obs_map.clone();
    mtx.unlock();

    cv::Mat res;
    cv::add(img0, img1, res);

    return res;
}

std::vector<cv::Vec3d> UNIMAP::get_locations()
{
    mtx.lock();
    std::vector<cv::Vec3d> res = locs;
    mtx.unlock();

    return res;
}

std::vector<cv::Vec2d> UNIMAP::get_map_pts()
{
    mtx.lock();
    std::vector<cv::Vec2d> _map_pts = map_pts;
    mtx.unlock();

    return _map_pts;
}

int UNIMAP::get_preset_idx(cv::Vec2d pt)
{
    cv::Vec2i uv = xy_uv(pt);

    mtx.lock();
    int val = velocity_map.ptr<uchar>(uv[1])[uv[0]];
    mtx.unlock();

    if(val == SLOWEST_ZONE)
    {
        return PRESET_SPEED_SLOWEST;
    }
    else if(val == SLOW_ZONE)
    {
        return PRESET_SPEED_SLOW;
    }
    else
    {
        return NORMAL_ZONE;
    }
}

void UNIMAP::draw_robot(cv::Mat &img, cv::Vec3d xi, cv::Scalar c, int tickness)
{
    cv::Vec2i uv0 = xy_uv(cv::Vec2d(0, 0), xi);
    cv::Vec2i uv1 = xy_uv(cv::Vec2d(static_config.robot_radius, 0), xi);

    cv::circle(img, cv::Point(uv0[0], uv0[1]), std::ceil(static_config.robot_radius/map_grid_width), c, tickness);
    cv::line(img, cv::Point(uv0[0], uv0[1]), cv::Point(uv1[0], uv1[1]), c, tickness);
}

void UNIMAP::draw_lidar(cv::Mat &img, cv::Vec3d xi, std::vector<cv::Vec2d> pts, cv::Scalar c)
{
    if(pts.size() > 0)
    {
        // draw current scan
        for (size_t p = 0; p < pts.size(); p++)
        {
            cv::Vec2i uv = xy_uv(pts[p], xi);
            if(uv[0] < 0 || uv[0] >= img.cols || uv[1] < 0 || uv[1] >= img.rows)
            {
                continue;
            }
            cv::Vec3b c_old = img.ptr<cv::Vec3b>(uv[1])[uv[0]];
            img.ptr<cv::Vec3b>(uv[1])[uv[0]] = 0.5*c_old + 0.5*cv::Vec3b(c[0], c[1], c[2]);
        }
    }
}

void UNIMAP::draw_lidar_scale(cv::Mat &img, cv::Vec3d xi, std::vector<cv::Vec2d> pts, cv::Scalar c, int scale)
{
    if(pts.size() > 0)
    {
        cv::Mat _img = img.clone();
        // draw current scan
        for (size_t p = 0; p < pts.size(); p++)
        {
            cv::Vec2i uv = xy_uv(pts[p], xi);
            if(uv[0] < 0 || uv[0] >= img.cols || uv[1] < 0 || uv[1] >= img.rows)
            {
                continue;
            }
            cv::Vec3b c_old = img.ptr<cv::Vec3b>(uv[1])[uv[0]];
            cv::Vec3b c_new = 0.5*c_old + 0.5*cv::Vec3b(c[0], c[1], c[2]);

            cv::circle(img, uv, scale, c_new, -1, cv::LINE_AA);
        }

        for(int i=0; i<img.rows; i++)
        {
            for(int j=0; j<img.cols; j++)
            {
                cv::Vec3b c_ref0 = img.ptr<cv::Vec3b>(i)[j];
                cv::Vec3b c_ref1 = _img.ptr<cv::Vec3b>(i)[j];
                cv::Vec3b c_new2 = 0.5*c_ref0 + 0.5*c_ref1;

                img.ptr<cv::Vec3b>(i)[j] = c_new2;
            }
        }
    }
}

void UNIMAP::draw_path(cv::Mat &img, std::vector<PATH_POINT> path, cv::Scalar c)
{
    if(path.size() >= 2)
    {
        for(size_t p = 0; p < path.size()-1; p++)
        {
            cv::Vec2i uv0 = xy_uv(path[p].pt);
            cv::Vec2i uv1 = xy_uv(path[p+1].pt);

            if(path[p].obs == 0)
            {
                double val = path[p].v*path[p].v;
                cv::Scalar _c(0.5*c[0] + 128*val, 0.5*c[1] + 128*val, 0.5*c[2] + 128*val);
                cv::line(img, cv::Point(uv0[0], uv0[1]), cv::Point(uv1[0], uv1[1]), _c, 1);
            }
            else
            {
                cv::line(img, cv::Point(uv0[0], uv0[1]), cv::Point(uv1[0], uv1[1]), cv::Vec3b(0,0,255), 1);
            }
        }
    }
}

void UNIMAP::draw_trajectory(cv::Mat &img, std::vector<cv::Vec6d> traj, cv::Scalar c, int tickness)
{
    if(traj.size() >= 2)
    {
        for(size_t p = 1; p < traj.size(); p++)
        {
            cv::Vec2i pt0 = xy_uv(cv::Vec2d(traj[p-1][0], traj[p-1][1]));
            cv::Vec2i pt1 = xy_uv(cv::Vec2d(traj[p][0], traj[p][1]));
            cv::line(img, cv::Point(pt0[0], pt0[1]), cv::Point(pt1[0], pt1[1]), c, tickness);
        }
    }
}

void UNIMAP::draw_travel_map(cv::Mat &img)
{    
    cv::Mat _travel_map = get_map_travel();

    for(int i = 0; i < img.rows; i++)
    {
        for(int j = 0; j < img.cols; j++)
        {
            if(_travel_map.ptr<uchar>(i)[j] == 0)
            {
                continue;
            }

            img.ptr<cv::Vec3b>(i)[j] = cv::Vec3b(255,0,0);
        }
    }
}

void UNIMAP::draw_obs_map(cv::Mat &img)
{
    cv::Mat _extended_dynamic_map = get_map_extended_dynamic_obs();

    for(int i = 0; i < img.rows; i++)
    {
        for(int j = 0; j < img.cols; j++)
        {
            if(_extended_dynamic_map.ptr<uchar>(i)[j] == 0)
            {
                continue;
            }

            cv::Vec3b c = img.ptr<cv::Vec3b>(i)[j];
            c[1] = 0.5*c[1] + 127;
            img.ptr<cv::Vec3b>(i)[j] = c;
        }
    }
}

cv::Vec2i UNIMAP::xy_uv(cv::Vec2d P, cv::Vec3d xi)
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

    int u = std::round(_P[0] / map_grid_width + map_ou);
    int v = std::round(_P[1] / map_grid_width + map_ov);
    return cv::Vec2i(u, v);
}

cv::Vec2i UNIMAP::xy_uv(cv::Vec2d P)
{
    int u = std::round(P[0] / map_grid_width + map_ou);
    int v = std::round(P[1] / map_grid_width + map_ov);
    return cv::Vec2i(u, v);
}

cv::Vec2i UNIMAP::xy_uv(cv::Vec2d P, int map_size, double grid_size)
{
    int u = std::round(P[0] / grid_size + map_size/2);
    int v = std::round(P[1] / grid_size + map_size/2);
    return cv::Vec2i(u, v);
}

cv::Vec2i UNIMAP::xy_uv(cv::Vec2d P, cv::Vec3d xi, int map_size, double grid_size)
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

    int u = std::round(_P[0] / grid_size + map_size/2);
    int v = std::round(_P[1] / grid_size + map_size/2);
    return cv::Vec2i(u, v);
}

cv::Vec2d UNIMAP::xy_uvd(cv::Vec2d P)
{
    double u = P[0] / map_grid_width + map_ou;
    double v = P[1] / map_grid_width + map_ov;
    return cv::Vec2d(u, v);
}

cv::Vec2i UNIMAP::xy_uv2(cv::Vec2d P)
{
    // uv for plot image
    int u = std::round(-P[1] / map_grid_width + map_ou);
    int v = std::round(-P[0] / map_grid_width + map_ov);
    return cv::Vec2i(u, v);
}

cv::Vec2d UNIMAP::uv_xy(cv::Vec2i uv)
{
    double x = (uv[0] - map_ou) * map_grid_width;
    double y = (uv[1] - map_ov) * map_grid_width;
    return cv::Vec2d(x, y);
}

cv::Vec2d UNIMAP::uv_xy(cv::Vec2i uv, cv::Vec3d xi)
{
    double x = (uv[0] - map_ou) * map_grid_width;
    double y = (uv[1] - map_ov) * map_grid_width;

    cv::Vec2d P(x, y);

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

std::vector<QString> UNIMAP::get_nodes(QString attrib)
{
    std::vector<QString> res;
    for(auto& it: nodes)
    {
        if(it.attrib == attrib)
        {
            res.push_back(it.id);
        }
    }
    return res;
}

NODE* UNIMAP::get_node_by_id(QString id)
{
    if(id == "")
    {
        return NULL;
    }

    NODE *node = NULL;
    if(nodes.size() != 0)
    {
        for(size_t p = 0; p < nodes.size(); p++)
        {
            if(nodes[p].id == id)
            {
                node = &nodes[p];
                break;
            }
        }
    }

    return node;
}

NODE* UNIMAP::get_node_nn(cv::Vec2d pt)
{
    if(nodes.size() == 0)
    {
        return NULL;
    }

    // find node
    int min_idx = -1;
    double min_d = 99999999;
    for(size_t p = 0; p < nodes.size(); p++)
    {
        cv::Vec2d node_pos = cv::Vec2d(nodes[p].pose[0], nodes[p].pose[1]);

        double d = cv::norm(node_pos - pt);
        if(d < min_d)
        {
            min_d = d;
            min_idx = p;
        }
    }

    if(min_idx != -1)
    {
        return &nodes[min_idx];
    }

    return NULL;
}

NODE* UNIMAP::get_edge_nn(cv::Vec3d pose)
{
    cv::Vec2d pos = cv::Vec2d(pose[0], pose[1]);

    double min_d = 99999999;
    NODE* min_node0 = NULL;
    NODE* min_node1 = NULL;
    cv::Vec2d min_pos0;
    cv::Vec2d min_pos1;
    for(auto& it: nodes)
    {
        for(size_t p = 0; p < it.linked.size(); p++)
        {
            QString id0 = it.id;
            QString id1 = it.linked[p];

            NODE* node0 = get_node_by_id(id0);
            NODE* node1 = get_node_by_id(id1);

            cv::Vec2d P0 = cv::Vec2d(node0->pose[0], node0->pose[1]);
            cv::Vec2d P1 = cv::Vec2d(node1->pose[0], node1->pose[1]);

            double d = calc_seg_dist(P0, P1, pos);
            if(d < min_d)
            {
                min_d = d;
                min_node0 = node0;
                min_node1 = node1;
                min_pos0 = P0;
                min_pos1 = P1;
            }
        }
    }

    double d0 = cv::norm(min_pos0 - pos);
    double d1 = cv::norm(min_pos1 - pos);

    if(d0 < d1)
    {
        return min_node0;
    }
    else
    {
        return min_node1;
    }
}

// json interface
QJsonArray UNIMAP::pose_to_array(cv::Vec3d pose)
{
    QJsonArray res;
    res.append(pose[0]);
    res.append(pose[1]);
    res.append(pose[2]);
    return res;
}

cv::Vec3d UNIMAP::array_to_pose(QJsonArray arr)
{
    cv::Vec3d res;
    res[0] = arr[0].toDouble();
    res[1] = arr[1].toDouble();
    res[2] = arr[2].toDouble();
    return res;
}

QJsonArray UNIMAP::links_to_array(std::vector<QString> links)
{
    QJsonArray res;
    for(size_t p = 0; p < links.size(); p++)
    {
        res.append(links[p]);
    }
    return res;
}

std::vector<QString> UNIMAP::array_to_links(QJsonArray arr)
{
    std::vector<QString> res;
    for(int p = 0; p < arr.size(); p++)
    {
        res.push_back(arr[p].toString());
    }
    return res;
}

double UNIMAP::calc_seg_dist(cv::Vec2d _P0, cv::Vec2d _P1, cv::Vec2d _P)
{
    Eigen::Vector3d P0 = Eigen::Vector3d(_P0[0], _P0[1], 0);
    Eigen::Vector3d P1 = Eigen::Vector3d(_P1[0], _P1[1], 0);
    Eigen::Vector3d P = Eigen::Vector3d(_P[0], _P[1], 0);

    Eigen::Vector3d ab = P1-P0;
    Eigen::Vector3d av = P-P0;
    if(av.dot(ab) <= 0)
    {
        return av.norm();
    }

    Eigen::Vector3d bv = P-P1;
    if(bv.dot(ab) >= 0)
    {
        return bv.norm();
    }

    return (ab.cross(av)).norm() / ab.norm();
}
