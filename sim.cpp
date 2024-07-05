#include "sim.h"

SIM::SIM(QObject *parent) : QObject(parent)
{
    obs_pt = cv::Vec2d(0,0);
    state = cv::Vec6d(0,0,0,0,0,0);
    is_pause = false;
}

SIM::~SIM()
{
    // loop destroy
    if(simThread != NULL)
    {
        simFlag = false;
        simThread->join();
        simThread = NULL;
    }
}

void SIM::init(MOBILE *_mobile, LIDAR_2D *_lidar, CAM *_cam)
{
    mobile = _mobile;
    lidar = _lidar;
    cam = _cam;

    // make lidar pattern
    lidar_pattern.clear();
    const double max_d = 40.0;
    const double d_ang = 0.391;
    for(double ang = 0; ang < 360.0; ang += d_ang)
    {
        double th = ang*D2R;
        double x = max_d*std::cos(th);
        double y = max_d*std::sin(th);
        lidar_pattern.push_back(cv::Vec2d(x, y));
    }
}

void SIM::set_sim_pose(cv::Vec3d pose)
{
    pre_mobile_pose = pose;
    state[0] = pose[0];
    state[1] = pose[1];
    state[2] = pose[2];
}

void SIM::load_map(QString path)
{
    // loop destroy
    if(simThread != NULL)
    {
        simFlag = false;
        simThread->join();
        simThread = NULL;
    }

    // load map
    QString raw_map_path = path + "/map_raw.png";
    QFileInfo raw_map_info(raw_map_path);
    if(raw_map_info.exists() && raw_map_info.isFile())
    {
        sim_map = cv::imread(raw_map_path.toStdString(), cv::IMREAD_GRAYSCALE);        
        printf("map_raw.png loaded\n");
    }
    else
    {
        printf("map_raw.png file not found\n");
    }

    QString edited_map_path = path + "/map_edited.png";
    QFileInfo edited_map_info(edited_map_path);
    if(edited_map_info.exists() && edited_map_info.isFile())
    {
        sim_map = cv::imread(edited_map_path.toStdString(), cv::IMREAD_GRAYSCALE);
        printf("map_edited.png loaded\n");
    }
    else
    {
        printf("map_edited.png not found\n");
    }

    // for simulation
    if (simThread == NULL)
    {
        simFlag = true;
        simThread = new std::thread(&SIM::simLoop, this);
    }
}

double SIM::sgn(double val)
{
    if(val < 0)
    {
        return -1;
    }
    else if(val == 0)
    {
        return 0;
    }
    else
    {
        return 1;
    }
}

double SIM::saturation(double val, double min, double max)
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

cv::Vec2i SIM::xy_uv(cv::Vec2d P)
{
    int u = std::round(P[0] / sim_grid_width + sim_map.cols/2);
    int v = std::round(P[1] / sim_grid_width + sim_map.rows/2);
    return cv::Vec2i(u, v);
}

cv::Vec2d SIM::uv_xy(cv::Vec2i uv)
{
    double x = (uv[0] - sim_map.cols/2) * sim_grid_width;
    double y = (uv[1] - sim_map.rows/2) * sim_grid_width;
    return cv::Vec2d(x, y);
}

std::vector<cv::Vec2d> SIM::calc_lidar_scan(cv::Vec3d xi)
{
    std::vector<cv::Vec2d> pts;

    cv::Matx22d R;
    R(0, 0) = std::cos(xi[2]);
    R(0, 1) = -std::sin(xi[2]);
    R(1, 0) = std::sin(xi[2]);
    R(1, 1) = std::cos(xi[2]);

    cv::Vec2d T;
    T[0] = xi[0];
    T[1] = xi[1];

    cv::Matx22d R_inv = R.t();
    cv::Vec2d T_inv = -T;

    cv::Vec2i pt0 = xy_uv(cv::Vec2d(xi[0], xi[1]));
    for(size_t i = 0; i < lidar_pattern.size(); i++)
    {
        cv::Vec2d pt1 = R*lidar_pattern[i]+T;
        std::vector<cv::Vec2i> ray = line_iterator(pt0, xy_uv(pt1));

        bool is_found = false;
        cv::Vec2d pt;
        for(size_t p = 0; p < ray.size(); p++)
        {
            int u = ray[p][0];
            int v = ray[p][1];
            if(u < 0 || u >= sim_map.cols || v < 0 || v >= sim_map.rows)
            {
                continue;
            }

            if(sim_map.ptr<uchar>(v)[u] == 255)
            {
                pt = uv_xy(cv::Vec2i(u,v));
                is_found = true;
                break;
            }
        }

        if(is_found)
        {
            cv::Vec2d _pt = R_inv*(pt+T_inv);
            if(!isfinite(_pt[0]) || !isfinite(_pt[1]))
            {
                continue;
            }
            pts.push_back(_pt);
        }
    }

    // adding noise
    for(size_t p = 0; p < pts.size(); p++)
    {
        double th = std::atan2(pts[p][1], pts[p][0]);
        double d = cv::norm(pts[p]) + (double)(rand()%10 - 5)*0.003;

        double x = std::cos(th)*d;
        double y = std::sin(th)*d;

        pts[p][0] = x;
        pts[p][1] = y;
    }

    return pts;
}

cv::Vec2d SIM::inv_transform(cv::Vec2d pt, cv::Vec3d xi)
{
    cv::Matx22d R;
    R(0, 0) = std::cos(xi[2]);
    R(0, 1) = -std::sin(xi[2]);
    R(1, 0) = std::sin(xi[2]);
    R(1, 1) = std::cos(xi[2]);

    cv::Vec2d T;
    T[0] = xi[0];
    T[1] = xi[1];

    cv::Matx22d R_inv = R.t();
    cv::Vec2d T_inv = -T;

    cv::Vec2d _pt = R_inv*(pt+T_inv);
    return _pt;
}

void SIM::set_obs_pt(cv::Vec2d pt)
{
    obs_pt = pt;
}

void SIM::simLoop()
{
    double pre_loop_time = 0;

    while(simFlag)
    {
        // get
        double v_acc = update_config.motor_limit_v_acc;
        double w_acc = update_config.motor_limit_w_acc;
        double v = mobile->last_v;
        double w = mobile->last_w;
        double v0 = mobile->pose.vw[0];
        double w0 = mobile->pose.vw[1];

        // calc state
        double sgn_v = sgn(v-v0);
        double sgn_w = sgn(w-w0);

        double _v = state[3] + sgn_v*v_acc*dt;
        if(sgn_v < 0)
        {
            if(_v <= v)
            {
                _v = v;
            }
        }
        else if(sgn_v > 0)
        {
            if(_v >= v)
            {
                _v = v;
            }
        }

        double _w = state[4] + sgn_w*w_acc*dt;
        if(sgn_w < 0)
        {
            if(_w <= w)
            {
                _w = w;
            }
        }
        else if(sgn_w > 0)
        {
            if(_w >= w)
            {
                _w = w;
            }
        }

        state[2] = toWrap(state[2] + _w*dt);
        state[0] += _v*std::cos(state[2])*dt; // x
        state[1] += _v*std::sin(state[2])*dt; // y
        state[3] = _v;
        state[4] = _w;
        state[5] = 0;

        // set mobile pose (0.01 sec)
        mobile->pose.t = sim_time;
        mobile->pose.pose[0] = state[0];
        mobile->pose.pose[1] = state[1];
        mobile->pose.pose[2] = state[2];
        mobile->pose.vw[0] = state[3];
        mobile->pose.vw[1] = state[4];
        mobile->status.is_ok = true;

        // set fake lidar (0.1 sec)
        if(sim_cnt % 2 == 0 && is_pause == false)
        {
            std::vector<cv::Vec2d> pts = calc_lidar_scan(mobile->pose.pose);

            // virtual obstacle
            if(obs_pt != cv::Vec2d(0,0))
            {
                pts.push_back(inv_transform(obs_pt, mobile->pose.pose));
                pts.push_back(inv_transform(obs_pt+cv::Vec2d(0.025, 0), mobile->pose.pose));
                pts.push_back(inv_transform(obs_pt+cv::Vec2d(0.05, 0), mobile->pose.pose));
                pts.push_back(inv_transform(obs_pt+cv::Vec2d(0.075, 0), mobile->pose.pose));
                pts.push_back(inv_transform(obs_pt+cv::Vec2d(0.1, 0), mobile->pose.pose));
            }

            std::vector<double> ts(pts.size(), sim_time);

            LIDAR_FRM frm;
            frm.t = sim_time;
            frm.t0 = sim_time;
            frm.t1 = sim_time;
            frm.ts = ts;
            frm.pts = pts;
            frm.mobile_pose = mobile->pose.pose;
            frm.pre_mobile_pose = pre_mobile_pose;
            pre_mobile_pose = mobile->pose.pose;
            lidar->scan_que.push(frm);

            lidar->mtx.lock();
            lidar->cur_scan = pts;
            lidar->mtx.unlock();

            // for memory
            if(lidar->scan_que.unsafe_size() > 50)
            {
                LIDAR_FRM temp;
                lidar->scan_que.try_pop(temp);
            }
        }

        // set fake camera (0.2 sec)
        if(sim_cnt % 4 == 0)
        {

        }

        sim_time += dt;
        sim_cnt++;

        // for real time loop
        double cur_loop_time = get_time();
        double delta_loop_time = cur_loop_time - pre_loop_time;
        if(delta_loop_time < dt)
        {
            int sleep_ms = (dt-delta_loop_time)*1000;
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
        }
        else
        {
            printf("[SIM] loop time drift\n");
        }
        pre_loop_time = get_time();
    }
}
