#include "multicontrol.h"

MULTICONTROL::MULTICONTROL(QObject *parent)
    : QObject{parent}
    , status_timer(this)
{

}

MULTICONTROL::~MULTICONTROL()
{

}

void MULTICONTROL::init(MOBILE *_mobile, SLAM_2D *_slam, UNIMAP *_unimap, WS_CLIENT *_ws)
{
    mobile = _mobile;
    slam = _slam;
    unimap = _unimap;
    ws = _ws;

    // websocket
    connect(ws, SIGNAL(signal_stc_id(QString)), this, SLOT(stc_id(QString)));
    connect(ws, SIGNAL(signal_stc_stop()), this, SLOT(stc_stop()));
    connect(ws, SIGNAL(signal_stc_allow(int)), this, SLOT(stc_allow(int)));
    connect(ws, SIGNAL(signal_stc_path(std::vector<cv::Vec2d>)), this, SLOT(stc_path(std::vector<cv::Vec2d>)));

    connect(this, SIGNAL(signal_cts_status(CTS_STATUS)), ws, SLOT(cts_status(CTS_STATUS)));
    connect(this, SIGNAL(signal_cts_confirm(std::vector<cv::Vec2d>)), ws, SLOT(cts_confirm(std::vector<cv::Vec2d>)));


    // other
    connect(&status_timer, SIGNAL(timeout()), this, SLOT(status_loop()));
    status_timer.start(100);

}

std::vector<PATH_POINT> MULTICONTROL::get_cur_path()
{
    mtx.lock();
    std::vector<PATH_POINT> path = cur_path;
    mtx.unlock();

    return path;
}

bool MULTICONTROL::check_resting_location(cv::Vec3d pose)
{
    std::vector<cv::Vec3d> _resting_locs = unimap->get_resting_locations();
    for(size_t i = 0; i < _resting_locs.size(); ++i)
    {
        double dx = _resting_locs[i][0] - pose[0];
        double dy = _resting_locs[i][1] - pose[1];

        double err_d = std::sqrt(dx*dx + dy*dy);
        double err_th = deltaRad(_resting_locs[i][2], pose[2]);
        if(err_d < 0.01 && std::abs(err_th*R2D) < 0.01)
        {
            return true;
        }
    }

    return false;
}

bool MULTICONTROL::check_serving_location(cv::Vec3d pose)
{
    std::vector<std::vector<cv::Vec3d>> _serving_locs = unimap->get_serving_locations();
    for(size_t p=0; p<_serving_locs.size(); p++)
    {
        for(size_t q=0; q<_serving_locs[p].size(); q++)
        {
            double dx = _serving_locs[p][q][0] - pose[0];
            double dy = _serving_locs[p][q][1] - pose[1];

            double err_d = std::sqrt(dx*dx + dy*dy);
            double err_th = deltaRad(_serving_locs[p][q][2], pose[2]);
            if(err_d < 0.01 && std::abs(err_th*R2D) < 0.01)
            {
                return true;
            }
        }
    }

    return false;
}

void MULTICONTROL::run(cv::Vec3d _goal, int _preset_idx)
{
    // update goal and preset
    preset_idx = _preset_idx;

    // check destination
    is_resting = check_resting_location(_goal);

    // update status
    mtx.lock();
    status.goal = _goal;
    status.state = "path_req";
    status.tick = ws->fms_tick;
    mtx.unlock();
}

void MULTICONTROL::stop()
{
    // stop
    if(fsm_thread != NULL)
    {
        fsm_flag = false;
        fsm_thread->join();
        fsm_thread = NULL;
    }

    // stop
    mobile->move(0, 0);
    mobile->led(0, LED_CYAN_DIM);

    // update
    mtx.lock();
    status.path.clear();
    status.state = "path_complete";
    mtx.unlock();
}

void MULTICONTROL::stc_id(QString id)
{
    // update
    mtx.lock();
    status.id = id;
    mtx.unlock();
}

void MULTICONTROL::stc_stop()
{
    stop();
}

void MULTICONTROL::stc_allow(int allow)
{
    if(allow == 1)
    {
        // update driving path
        mtx.lock();
        cur_path = avoid_path;
        mtx.unlock();

        // stop first
        if(fsm_thread != NULL)
        {
            fsm_flag = false;
            fsm_thread->join();
            fsm_thread = NULL;
        }

        // start fsm loop
        fsm_state = STATE_AUTO_FIRST_ALIGN;
        fsm_flag = true;
        fsm_thread = new std::thread(&MULTICONTROL::fsm_loop, this);
    }
    else
    {
        // update driving path
        mtx.lock();
        cur_path = ref_path;
        mtx.unlock();

        // stop first
        if(fsm_thread != NULL)
        {
            fsm_flag = false;
            fsm_thread->join();
            fsm_thread = NULL;
        }

        // start fsm loop
        fsm_state = STATE_AUTO_PURE_PURSUIT;
        fsm_flag = true;
        fsm_thread = new std::thread(&MULTICONTROL::fsm_loop, this);
    }
}

void MULTICONTROL::stc_path(std::vector<cv::Vec2d> path)
{
    if(path.size() > 0)
    {
        // update status
        mtx.lock();
        status.path = path;
        status.state = "path_driving";
        mtx.unlock();

        // calc ref path
        std::vector<PATH_POINT> _ref_path = calc_ref_path(path);

        // storing
        mtx.lock();
        ref_path = _ref_path;
        mtx.unlock();

        // obstacle situation
        if(fsm_state == STATE_AUTO_OBSTACLE)
        {
            // calc avoid path
            cv::Mat obs_map = unimap->get_map_extended_obs();
            std::vector<PATH_POINT> _avoid_path = calc_avoid_path(obs_map, _ref_path);

            // storing
            mtx.lock();
            avoid_path = _avoid_path;
            mtx.unlock();

            // send confirm
            std::vector<cv::Vec2d> confirm_path;
            for(size_t p = 0; p < _avoid_path.size(); p+= 10)
            {
                confirm_path.push_back(_avoid_path[p].pt);
            }
            emit signal_cts_confirm(confirm_path);
        }
        else
        {
            // update driving path
            mtx.lock();
            cur_path = _ref_path;
            mtx.unlock();

            // stop first
            if(fsm_thread != NULL)
            {
                fsm_flag = false;
                fsm_thread->join();
                fsm_thread = NULL;
            }

            // start fsm loop
            fsm_state = STATE_AUTO_PURE_PURSUIT;
            fsm_flag = true;
            fsm_thread = new std::thread(&MULTICONTROL::fsm_loop, this);
        }
    }
}

std::vector<cv::Vec2d> MULTICONTROL::path_dividing(std::vector<cv::Vec2d> _path, double step)
{
    std::vector<cv::Vec2d> precise_path;
    if (_path.empty())
    {
        return precise_path; // 입력 경로가 비어있으면 빈 벡터 반환
    }

    for(size_t i = 0; i < _path.size()-1; ++i)
    {
        const auto& start = _path[i];
        const auto& end = _path[i + 1];

        // 중복된 점을 무시합니다.
        double dist = cv::norm(end - start);
        if (dist == 0) continue; // 중복된 점은 무시

        precise_path.push_back(start);
        int numPoints = std::max(static_cast<int>(std::round(dist / step)) - 1, 0);
        cv::Vec2d direction = (end - start) / dist;

        for (int j = 1; j <= numPoints; ++j)
        {
            cv::Vec2d newPoint = start + direction * (step * j);
            precise_path.push_back(newPoint);
        }
    }

    // 마지막 점을 추가합니다. 중복된 점을 무시했기 때문에 항상 추가해야 합니다.
    precise_path.push_back(_path.back());

    return precise_path;
}

void MULTICONTROL::smoothing_ref_v(std::vector<PATH_POINT>& path)
{
    std::vector<double> list0(path.size());
    double v0 = path.front().v;
    for(int p = 0; p < (int)path.size(); p++)
    {
        double v_limit = setting_config.robot_preset_limit_v[path[p].preset_idx];
        double v_acc = setting_config.robot_preset_limit_v_acc[path[p].preset_idx]*update_config.robot_path_delta_v_acc_gain;
        double v1 = std::sqrt(2*v_acc*PATH_STEP + v0*v0);
        if(v1 > v_limit)
        {
            v1 = v_limit;
        }

        if(path[p].v > v0)
        {
            v0 = v1;
        }
        else
        {
            v0 = path[p].v;
        }
        list0[p] = v0;
    }

    std::vector<double> list1(path.size());
    v0 = path.back().v;
    for(int p = path.size()-1; p >= 0; p--)
    {
        double v_limit = setting_config.robot_preset_limit_v[path[p].preset_idx];
        double v_acc = setting_config.robot_preset_limit_v_acc[path[p].preset_idx]*update_config.robot_path_delta_v_dec_gain;
        double v1 = std::sqrt(2*v_acc*PATH_STEP + v0*v0);
        if(v1 > v_limit)
        {
            v1 = v_limit;
        }

        if(path[p].v > v0)
        {
            v0 = v1;
        }
        else
        {
            v0 = path[p].v;
        }
        list1[p] = v0;
    }

    std::vector<double> res(path.size());
    for(size_t p = 0; p < path.size(); p++)
    {
        path[p].v = std::min<double>(list0[p], list1[p]);
    }
}

void MULTICONTROL::smoothing_obs_v(std::vector<PATH_POINT>& path)
{
    std::vector<double> list1(path.size(), 0);
    double v0 = 0;
    for(int p = path.size()-1; p >= 0; p--)
    {
        double v_limit = setting_config.robot_preset_limit_v[path[p].preset_idx];
        double v_acc = setting_config.robot_preset_limit_v_acc[path[p].preset_idx]*update_config.robot_path_delta_v_dec_gain;
        double v1 = std::sqrt(2*v_acc*PATH_STEP + v0*v0);
        if(v1 > v_limit)
        {
            v1 = v_limit;
        }

        if(path[p].obs_v > v0)
        {
            v0 = v1;
        }
        else
        {
            v0 = path[p].obs_v;
        }
        list1[p] = v0;
    }

    std::vector<double> res(path.size());
    for(size_t p = 0; p < path.size(); p++)
    {
        path[p].obs_v = list1[p];
    }
}

std::vector<cv::Vec2d> MULTICONTROL::path_ccma(std::vector<cv::Vec2d> path)
{
    // init params
    const int w_ma = 10; // 5
    const int w_cc = 3; // 3
    const int w_ccma = w_ma + w_cc + 1;

    // not enough points
    if(path.size() < w_ccma)
    {
        return path;
    }
    // padding func
    auto add_padding = [](std::vector<cv::Vec3d> points, int n_padding)->std::vector<cv::Vec3d>
    {
        std::vector<cv::Vec3d> padded;

        for (int i = 0; i < n_padding; ++i)
        {
            padded.push_back(points.front());
        }

        padded.insert(padded.end(), points.begin(), points.end());

        for (int i = 0; i < n_padding; ++i)
        {
            padded.push_back(points.back());
        }

        return padded;
    };

    // calc unit vector function
    auto get_unit_vector = [](const cv::Vec3d vec)->cv::Vec3d
    {
        double norm = cv::norm(vec);
        if(norm > 0.0)
        {
            return vec/norm;
        }
        else
        {
            return vec;
        }
    };

    // calc pascal's triangle distribution
    std::function<std::vector<int>(int)> get_pascal_row = [&](int row_index)
    {
        std::vector<int> cur_row(1, 1);
        if(row_index == 0)
        {
            return cur_row;
        }

        std::vector<int> prev = get_pascal_row(row_index - 1);

        for(int idx = 1; idx < (int)prev.size(); idx++)
        {
            int cur = prev[idx-1] + prev[idx];
            cur_row.push_back(cur);
        }

        cur_row.push_back(1);
        return cur_row;
    };

    // calc weight kernels
    std::vector<std::vector<double>> weights_ma;
    for(int i = 0; i < w_ma + 1; i++)
    {
        int pascal_row_index = i * 2;
        std::vector<int> row = get_pascal_row(pascal_row_index);
        int row_sum = std::accumulate(row.begin(), row.end(), 0);

        std::vector<double> tmp;
        for(int j = 0; j < (int)row.size(); j++)
        {
            tmp.push_back((double)row[j]/row_sum);
        }

        weights_ma.push_back(tmp);
    }

    std::vector<std::vector<double>> weights_cc;
    for(int i = 0; i < w_cc + 1; i++)
    {
        int pascal_row_index = i * 2;
        std::vector<int> row = get_pascal_row(pascal_row_index);
        int row_sum = std::accumulate(row.begin(), row.end(), 0);

        std::vector<double> tmp;
        for(int j = 0; j < (int)row.size(); j++)
        {
            tmp.push_back((double)row[j]/row_sum);
        }

        weights_cc.push_back(tmp);
    }

    // convert 2d points to 3d points
    std::vector<cv::Vec3d> points;
    for(size_t p = 0; p < path.size(); p++)
    {
        points.push_back(cv::Vec3d(path[p][0], path[p][1], 0));
    }
    points = add_padding(points, w_ccma);

    // calc moving average points
    std::vector<cv::Vec3d> points_ma;
    for(int i = w_ma; i < (int)points.size() - w_ma; i++)
    {
        // convolution
        cv::Vec3d pt(0,0,0);
        for(int j = -w_ma; j <= w_ma; j++)
        {
            pt += weights_ma[w_ma][w_ma+j]*points[i+j];
        }

        points_ma.push_back(pt);
    }

    // calc curvature vector
    std::vector<cv::Vec3d> curvature_vector(points_ma.size(), cv::Vec3d(0,0,0));
    for (size_t p = 1; p < points_ma.size() - 1; p++)
    {
        cv::Vec3d p0 = points_ma[p-1];
        cv::Vec3d p1 = points_ma[p];
        cv::Vec3d p2 = points_ma[p+1];

        cv::Vec3d v1 = p1 - p0;
        cv::Vec3d v2 = p2 - p1;
        cv::Vec3d cross = v1.cross(v2);
        double cross_norm = cv::norm(cross);

        double curvature = 0.0;
        if (cross_norm > 0.0)
        {
            double radius = cv::norm(v1)*cv::norm(v2)*cv::norm(p2-p0)/(2*cross_norm);
            curvature = 1.0/radius;
        }
        else
        {
            curvature = 0;
        }

        curvature_vector[p] = curvature * get_unit_vector(cross);
    }

    // calc curvatures
    std::vector<double> curvatures(curvature_vector.size(), 0);
    for(size_t p = 0; p < curvature_vector.size(); p++)
    {
        curvatures[p] = cv::norm(curvature_vector[p]);
    }

    // calc alphas
    std::vector<double> alphas(points_ma.size(), 0);
    for (size_t p = 1; p < points_ma.size()-1; p++)
    {
        if (curvatures[p] > 0.0)
        {
            double radius = 1.0/curvatures[p];
            double dist_neighbors = cv::norm(points_ma[p+1] - points_ma[p-1]);
            alphas[p] = std::sin((dist_neighbors/2)/radius);
        }
        else
        {
            alphas[p] = 0.0;
        }
    }

    // calc radii_ma
    std::vector<double> radii_ma(alphas.size(), 0);
    for (size_t i = 1; i < alphas.size()-1; i++)
    {
        radii_ma[i] = weights_ma[w_ma][w_ma];
        for (int k = 1; k <= w_ma; k++)
        {
            radii_ma[i] += 2*std::cos(alphas[i]*k) * weights_ma[w_ma][w_ma+k];
        }

        // Apply threshold
        radii_ma[i] = std::max(0.35, radii_ma[i]);
    }

    // calc ccma
    std::vector<cv::Vec3d> points_ccma(points.size() - 2*w_ccma, cv::Vec3d(0,0,0));
    for (size_t idx = 0; idx < points_ccma.size(); idx++)
    {
        // Get tangent vector for the shifting point
        cv::Vec3d unit_tangent = get_unit_vector(points_ma[w_cc + idx + 1 + 1] - points_ma[w_cc + idx - 1 + 1]);

        // Calculate the weighted shift
        cv::Vec3d shift(0,0,0);
        for (int idx_cc = 0; idx_cc < 2*w_cc + 1; idx_cc++)
        {
            if (curvatures[idx + idx_cc + 1] > 0.0)
            {
                cv::Vec3d u = get_unit_vector(curvature_vector[idx + idx_cc + 1]);
                double weight = weights_cc[w_cc][idx_cc];
                double shift_magnitude = (1.0/curvatures[idx + idx_cc + 1]) * (1.0/radii_ma[idx + idx_cc + 1] - 1);
                shift += u * weight * shift_magnitude;
            }
        }

        // Reconstruction
        points_ccma[idx] = points_ma[idx + w_cc + 1] + unit_tangent.cross(shift);
    }

    // result
    std::vector<cv::Vec2d> res;
    res.push_back(path.front());
    for(size_t p = 1; p < points_ccma.size()-1; p++)
    {
        res.push_back(cv::Vec2d(points_ccma[p][0], points_ccma[p][1]));
    }
    res.push_back(path.back());
    return res;
}

std::vector<PATH_POINT> MULTICONTROL::calc_ref_path(std::vector<cv::Vec2d> path)
{
    // path dividing
    if(setting_config.robot_use_ccma)
    {
        std::vector<cv::Vec2d> path2 = path_dividing(path, 0.1);
        std::vector<cv::Vec2d> path3 = path_ccma(path2);
        path = path_dividing(path3, PATH_STEP);
    }
    else
    {
        path = path_dividing(path, PATH_STEP);
    }

    // set path points
    std::vector<PATH_POINT> res;
    for(size_t p = 0; p < path.size(); p++)
    {
        int _preset_idx = preset_idx;

        if(is_resting == false)
        {
            int map_preset_idx = unimap->get_preset_idx(path[p]);
            if(map_preset_idx < preset_idx)
            {
                _preset_idx = map_preset_idx;
            }
        }

        PATH_POINT pt;
        pt.pt = path[p];
        pt.preset_idx = _preset_idx;
        res.push_back(pt);
    }

    // calc dists and heading angle
    for(size_t p = 0; p < res.size()-1; p++)
    {
        double dx = res[p+1].pt[0]-res[p].pt[0];
        double dy = res[p+1].pt[1]-res[p].pt[1];
        double d = std::sqrt(dx*dx + dy*dy);
        double th = std::atan2(dy, dx);

        res[p+1].od = res[p].od + d;
        res[p].th = th;
        res[p+1].th = th;
    }

    // calc ref v
    int ld = update_config.robot_look_ahead_dist/PATH_STEP;

    res.front().v = std::max<double>(update_config.robot_st_v, mobile->last_v);
    for(size_t p = 0; p < res.size(); p++)
    {
        int _preset_idx = res[p].preset_idx;
        double v_limit = setting_config.robot_preset_limit_v[_preset_idx];
        double w_limit = setting_config.robot_preset_limit_w[_preset_idx]*update_config.robot_path_ref_v_gain;

        int cur_idx = p;
        int tgt_idx = cur_idx + ld;
        if(tgt_idx > (int)res.size()-1)
        {
            tgt_idx = res.size()-1;
        }

        double dx = res[tgt_idx].pt[0] - res[cur_idx].pt[0];
        double dy = res[tgt_idx].pt[1] - res[cur_idx].pt[1];
        double err_d = std::sqrt(dx*dx + dy*dy);
        double err_th = deltaRad(std::atan2(dy, dx), res[cur_idx].th) * 2.0;

        double t_v = std::abs(err_d)/v_limit;
        double t_w = std::abs(err_th)/w_limit;
        double t = std::max<double>(t_v, t_w);
        double v = err_d/t;
        res[cur_idx].v = v;
    }
    res.back().v = update_config.robot_goal_v;

    // smoothing speed
    smoothing_ref_v(res);
    return res;
}

bool MULTICONTROL::is_los(cv::Mat& map, cv::Vec2i pt0, cv::Vec2i pt1)
{
    std::vector<cv::Vec2i> line = line_iterator(pt0, pt1);

    bool res = true;
    for(size_t p = 0; p < line.size(); p++)
    {
        int u = line[p][0];
        int v = line[p][1];

        if(map.ptr<uchar>(v)[u] == 255)
        {
            return false;
        }
    }

    return res;
}

void MULTICONTROL::fill_mask(cv::Mat&src, cv::Mat mask, float val)
{
    for(int i = 0; i < src.rows; i++)
    {
        for(int j = 0; j < src.cols; j++)
        {
            if(mask.ptr<uchar>(i)[j] == 255)
            {
                src.ptr<float>(i)[j] = val;
            }
        }
    }
}

std::vector<cv::Vec2i> MULTICONTROL::path_finding_avoid(cv::Mat map, cv::Mat cost_map, cv::Vec2i st, cv::Vec2i ed)
{
    // no need to path planning in this case
    if(st == ed)
    {
        std::vector<cv::Vec2i> res;
        res.push_back(st);
        res.push_back(ed);
        return res;
    }

    // obs_map CV_8U(obstacle is 255)
    const int w = map.cols;
    const int h = map.rows;
    const int r_ld0 = update_config.robot_min_look_ahead_dist/unimap->map_grid_width;
    const int r_ld1 = update_config.robot_look_ahead_dist/unimap->map_grid_width;

    // cost map CV_32F(low is good)
    if(cost_map.empty())
    {
        cost_map = cv::Mat(h, w, CV_32F, cv::Scalar(0));
    }

    cv::Mat _map;
    cv::dilate(map, _map, cv::Mat());

    // make open set and close set
    std::vector<ASTAR_NODE*> open_set;
    cv::Mat close_set(h, w, CV_8U, cv::Scalar(0));

    // set end node
    ASTAR_NODE *ed_node = new ASTAR_NODE();
    ed_node->pos = ed;

    // set start node
    ASTAR_NODE *st_node = new ASTAR_NODE();
    st_node->pos = st;
    st_node->g = 0;
    st_node->h = cv::norm(ed_node->pos-st_node->pos);
    st_node->f = st_node->g + st_node->h;
    open_set.push_back(st_node);

    // result storage
    bool is_first = true;
    while(open_set.size() > 0)
    {
        // get the current nodeopen_set
        int cur_node_idx = 0;
        ASTAR_NODE *cur_node = open_set.front();
        for(size_t p = 0; p < open_set.size(); p++)
        {
            if(open_set[p]->f < cur_node->f)
            {
                cur_node = open_set[p];
                cur_node_idx = p;
            }
        }

        // pop current off open list, add to closed list
        open_set.erase(open_set.begin()+cur_node_idx);
        close_set.ptr<uchar>(cur_node->pos[1])[cur_node->pos[0]] = 255;

        // found the goal
        //if(cv::norm(ed_node->pos - cur_node->pos) <= r_near)
        if(ed_node->pos == cur_node->pos)
        {
            std::vector<cv::Vec2i> res;

            ASTAR_NODE* _cur_node = cur_node;
            while(_cur_node != NULL)
            {
                res.push_back(_cur_node->pos);
                _cur_node = _cur_node->parent;
            }

            std::reverse(res.begin(), res.end());
            if(res.back() != ed)
            {
                res.push_back(ed);
            }

            return res;
        }

        // append child nodes
        std::vector<cv::Vec2i> around_pts;

        if(is_first)
        {
            is_first = false;

            std::vector<cv::Vec2i> circle = filled_circle_iterator(st, r_ld1);
            for(size_t p = 0; p < circle.size(); p++)
            {
                int u = circle[p][0];
                int v = circle[p][1];
                if(u < 0 || u >= w || v < 0 || v >= h)
                {
                    continue;
                }

                // check min lookahead
                double d = cv::norm(cv::Vec2i(u, v) - st);
                if(d < r_ld0)
                {
                    continue;
                }

                // obstacle
                if(map.ptr<uchar>(v)[u] == 255)
                {
                    continue;
                }

                // los check
                if(is_los(map, st, circle[p]))
                {
                    around_pts.push_back(cv::Vec2i(u,v));
                }
            }

            if(around_pts.size() == 0)
            {
                printf("path finding failed, obstacle too close\n");
            }
        }
        else
        {
            int search_range = 1;
            for(int my = -search_range; my <= search_range; my++)
            {
                for(int mx = -search_range; mx <=search_range; mx++)
                {
                    if(mx == 0 && my == 0)
                    {
                        continue;
                    }

                    int u = cur_node->pos[0] + mx;
                    int v = cur_node->pos[1] + my;
                    if(u < 0 || u >= w || v < 0 || v >= h)
                    {
                        continue;
                    }

                    // obstacle
                    if(map.ptr<uchar>(v)[u] == 255)
                    {
                        continue;
                    }

                    // close set
                    if(close_set.ptr<uchar>(v)[u] == 255)
                    {
                        continue;
                    }

                    around_pts.push_back(cv::Vec2i(u,v));
                }
            }
        }

        for(size_t p = 0; p < around_pts.size(); p++)
        {
            // calc heuristics
            int u = around_pts[p][0];
            int v = around_pts[p][1];
            cv::Vec2i child_pos(u, v);

            // parents check
            ASTAR_NODE* parent_node = cur_node;
            ASTAR_NODE* chk_node = cur_node;
            while(chk_node->parent != NULL)
            {
                double d = cv::norm(child_pos - chk_node->pos);
                if(d > r_ld1)
                {
                    break;
                }

                if(is_los(_map, chk_node->pos, child_pos))
                {
                    parent_node = chk_node;
                }

                chk_node = chk_node->parent;
            }

            double weight = cost_map.ptr<float>(v)[u];
            double child_g = parent_node->g + cv::norm(child_pos - parent_node->pos);
            double child_h = cv::norm(child_pos - ed_node->pos);
            double child_f = weight*child_g + child_h;

            // check open set
            bool is_open_set = false;
            ASTAR_NODE* open_node = NULL;
            for(size_t p = 0; p < open_set.size(); p++)
            {
                if(open_set[p]->pos == child_pos)
                {
                    is_open_set = true;
                    open_node = open_set[p];
                    break;
                }
            }

            // change better parent
            if(is_open_set)
            {
                if(open_node->parent == NULL)
                {
                    continue;
                }

                if(child_g < open_node->g)
                {
                    open_node->parent = parent_node;
                    open_node->g = child_g;
                    open_node->h = child_h;
                    open_node->f = child_f;
                    continue;
                }
                else
                {
                    continue;
                }
            }

            // add new child to open set
            ASTAR_NODE *node = new ASTAR_NODE();
            node->parent = parent_node;
            node->pos = child_pos;
            node->g = child_g;
            node->h = child_h;
            node->f = child_f;
            open_set.push_back(node);
        }
    }

    // path finding failed
    return std::vector<cv::Vec2i>();
}

std::vector<PATH_POINT> MULTICONTROL::calc_avoid_path(cv::Mat obs_map, std::vector<PATH_POINT> ref)
{
    // make grid space
    const double grid_size = unimap->map_grid_width;
    const int w = unimap->map_w;
    const int h = w;

    // radius
    int r_ref = 1.5/grid_size;
    int r_chk = setting_config.robot_obs_check_range/grid_size;

    // set start, end point
    cv::Vec2i st = unimap->xy_uv(ref.front().pt);

    int ed_idx = ref.size()-1;
    for(size_t p = 0; p < ref.size(); p++)
    {
        double d = cv::norm(unimap->xy_uv(ref[p].pt) - st);
        if(d > r_chk)
        {
            cv::Vec2i _ed = unimap->xy_uv(ref[p].pt);
            int u = _ed[0];
            int v = _ed[1];
            if(obs_map.ptr<uchar>(v)[u] == 0)
            {
                ed_idx = p;
                break;
            }
        }
    }
    cv::Vec2i ed = unimap->xy_uv(ref[ed_idx].pt);

    // check goal occupied
    if(obs_map.ptr<uchar>(ed[1])[ed[0]] == 255)
    {
        printf("[MULTI] avoid_path goal occupied\n");
        return std::vector<PATH_POINT>();
    }

    // ref path map
    cv::Mat ref_map(h, w, CV_8U, cv::Scalar(0));
    cv::Mat ref_mask(h, w, CV_8U, cv::Scalar(0));
    for(size_t p = 0; p < ref.size()-1; p++)
    {
        cv::Vec2i pt0 = unimap->xy_uv(ref[p].pt);
        cv::Vec2i pt1 = unimap->xy_uv(ref[p+1].pt);
        cv::line(ref_map, pt0, pt1, cv::Scalar(255), 1);
        cv::line(ref_mask, pt0, pt1, cv::Scalar(255), r_ref*2+1);
    }

    // obs_margin
    cv::Mat obs_margin;
    cv::Mat avoid_map = unimap->get_map_avoid();
    cv::add(avoid_map, ref_map, avoid_map);
    cv::add(obs_map, ~avoid_map, obs_margin);

    // cost map
    cv::Mat ref_dist_map;
    cv::distanceTransform(~ref_map, ref_dist_map, cv::DIST_L2, 3, CV_32F);
    cv::normalize(ref_dist_map, ref_dist_map, 0.0, 1.0, cv::NORM_MINMAX, -1, ref_mask);
    fill_mask(ref_dist_map, ~ref_mask, 1.0);

    cv::Mat obs_dist_base;
    cv::subtract(ref_mask, obs_margin, obs_dist_base, ref_mask);

    cv::Mat obs_dist_map;
    cv::distanceTransform(obs_dist_base, obs_dist_map, cv::DIST_L2, 3, CV_32F);
    cv::normalize(obs_dist_map, obs_dist_map, 0.0, 1.0, cv::NORM_MINMAX, -1, ref_mask);
    cv::subtract(cv::Mat::ones(h,w,CV_32F), obs_dist_map, obs_dist_map);

    cv::Mat cost_map(h, w, CV_32F, cv::Scalar(1.0));
    for(int i = 0; i < h; i++)
    {
        for(int j = 0; j < w; j++)
        {
            float c0 = ref_dist_map.ptr<float>(i)[j];
            float c1 = obs_dist_map.ptr<float>(i)[j];
            float c = std::sqrt(c0*c0 + c1*c1);
            cost_map.ptr<float>(i)[j] = c;
        }
    }

    // path finding
    cv::Mat astar_map;
    cv::add(obs_margin, ~ref_mask, astar_map);

    std::vector<cv::Vec2i> path_uv = path_finding_avoid(astar_map, cost_map, st, ed);
    if(path_uv.size() < 2)
    {
        printf("[AUTO] error, avoid path failed\n");
        return std::vector<PATH_POINT>();
    }

    // first lookahead distance
    double st_ld = cv::norm(path_uv[1] - path_uv[0])*grid_size;

    // convert metric path
    std::vector<cv::Vec2d> path;
    for(size_t p = 0; p < path_uv.size(); p++)
    {
        path.push_back(unimap->uv_xy(path_uv[p]));
    }

    // smoothing
    if(setting_config.robot_use_ccma)
    {
        path = path_dividing(path, 0.05);
        path = path_ccma(path);
        path = path_dividing(path, PATH_STEP);
    }
    else
    {
        path = path_dividing(path, PATH_STEP);
    }

    // merge avoid path + ref
    if(ed_idx < (int)ref.size()-1)
    {
        for(size_t p = ed_idx+1; p < ref.size(); p++)
        {
            path.push_back(ref[p].pt);
        }
    }

    // set path points
    std::vector<PATH_POINT> res;
    for(size_t p = 0; p < path.size(); p++)
    {
        int _preset_idx = preset_idx;

        if(is_resting == false)
        {
            int map_preset_idx = unimap->get_preset_idx(path[p]);
            if(map_preset_idx < preset_idx)
            {
                _preset_idx = map_preset_idx;
            }
        }

        PATH_POINT pt;
        pt.pt = path[p];
        pt.preset_idx = _preset_idx;
        res.push_back(pt);
    }

    // calc dists and heading angle
    for(size_t p = 0; p < res.size()-1; p++)
    {
        double dx = res[p+1].pt[0]-res[p].pt[0];
        double dy = res[p+1].pt[1]-res[p].pt[1];
        double d = std::sqrt(dx*dx + dy*dy);
        double th = std::atan2(dy, dx);

        res[p+1].od = res[p].od + d;
        res[p].th = th;
        res[p+1].th = th;
    }

    // calc ref v
    int ld = update_config.robot_look_ahead_dist/PATH_STEP;

    res.front().v = std::max<double>(update_config.robot_st_v, mobile->last_v);
    for(size_t p = 0; p < res.size(); p++)
    {
        int _preset_idx = res[p].preset_idx;
        double v_limit = setting_config.robot_preset_limit_v[_preset_idx];
        double w_limit = setting_config.robot_preset_limit_w[_preset_idx]*update_config.robot_path_ref_v_gain;

        double d = cv::norm(path[p] - path[0]);
        if(d <= st_ld * 1.5)
        {
            v_limit = st_ld;
        }

        int cur_idx = p;
        int tgt_idx = cur_idx + ld;
        if(tgt_idx > (int)res.size()-1)
        {
            tgt_idx = res.size()-1;
        }

        double dx = res[tgt_idx].pt[0] - res[cur_idx].pt[0];
        double dy = res[tgt_idx].pt[1] - res[cur_idx].pt[1];
        double err_d = std::sqrt(dx*dx + dy*dy);
        double err_th = deltaRad(std::atan2(dy, dx), res[cur_idx].th) * 2.0;

        double t_v = std::abs(err_d)/v_limit;
        double t_w = std::abs(err_th)/w_limit;
        double t = std::max<double>(t_v, t_w);
        double v = err_d/t;
        res[cur_idx].v = v;
    }
    res.back().v = update_config.robot_goal_v;

    // smoothing speed
    smoothing_ref_v(res);
    return res;
}

// for pp
double MULTICONTROL::sgn(double val)
{
    if(val >= 0)
    {
        return 1;
    }
    else
    {
        return -1;
    }
}

double MULTICONTROL::saturation(double val, double min, double max)
{
    double _min = std::min<double>(min, max);
    double _max = std::max<double>(min, max);

    if(val < _min)
    {
        val = _min;
    }
    else if(val > _max)
    {
        val = _max;
    }

    return val;
}

int MULTICONTROL::get_cur_idx(cv::Vec3d cur_pose, std::vector<PATH_POINT> path)
{
    cv::Vec2d cur_pos(cur_pose[0], cur_pose[1]);

    int min_id = 0;
    double min_d = 99999999;
    for(size_t p = 0; p < path.size()-1; p++)
    {
        double d = cv::norm(path[p].pt - cur_pos);
        if(d < min_d)
        {
            min_d = d;
            min_id = p;
        }
    }
    return min_id;
}

int MULTICONTROL::get_tgt_idx(int cur_idx, double ld, int path_num)
{
    int idx = cur_idx + (ld/PATH_STEP);
    if(idx > path_num-1)
    {
        idx = path_num-1;
    }
    return idx;
}

double MULTICONTROL::calc_motion_time(double _s, double _v0, double _v1, double _acc)
{
    if(std::abs(_s) == 0)
    {
        return 0;
    }

    double sign = _v0*_v1;
    if(sign >= 0)
    {
        // v0, v1 same sign
        double v0 = std::abs(_v0);
        double v1 = std::abs(_v1);
        double s = std::abs(_s);
        double acc;
        if(v1 - v0 > 0)
        {
            // accel
            acc = _acc;
        }
        else if(v1 - v0 < 0)
        {
            // decel
            acc = -_acc;
        }
        else
        {
            // constant case
            double t = s/v0;
            return t;
        }

        double t0 = (v1-v0)/acc;
        double s0 = v0*t0+0.5*acc*t0*t0;
        if(s0 >= s)
        {
            double a = 0.5*acc;
            double b = v0;
            double c = -s;

            // always b^2 - 4ac > 0, because c is minus
            double tmp0 = (-b - std::sqrt(b*b - 4*a*c))/(2*a);
            double tmp1 = (-b + std::sqrt(b*b - 4*a*c))/(2*a);
            double t = std::max<double>(tmp0, tmp1);
            return t;
        }

        double t1 = (s - v0*t0 - 0.5*(v1-v0)*t0 + v1*t0)/v1;
        return t1;
    }
    else if(sign < 0)
    {
        // v0, v1 different sign
        double v0 = std::abs(_v0);
        double v1 = std::abs(_v1);
        double s = std::abs(_s);
        double acc = std::abs(_acc);

        double t0 = v0/acc;
        double t1 = t0 + v1/acc;

        double s1 = -0.5*v0*t0 + 0.5*v1*(t1-t0);
        if(s1 >= s)
        {
            v0 *= -1;
            double a = 0.5*acc;
            double b = v0;
            double c = -s;

            // always b^2 - 4ac > 0, because c is minus
            double tmp0 = (-b - std::sqrt(b*b - 4*a*c))/(2*a);
            double tmp1 = (-b + std::sqrt(b*b - 4*a*c))/(2*a);
            double t = std::max<double>(tmp0, tmp1);
            return t;
        }

        double t2 = (s + 0.5*v0*t0 - 0.5*v1*(t1-t0) + v1*t1)/v1;
        return t2;
    }
}

std::vector<cv::Vec3d> MULTICONTROL::calc_trajectory(double v, double w, double dt, double predict_t, cv::Vec3d xi0)
{
    std::vector<cv::Vec3d> res;
    res.push_back(xi0);

    double x = 0;
    double y = 0;
    double th = 0;
    for(double t = 0; t < predict_t; t += dt)
    {
        th = th + w*dt;
        x += v*std::cos(th)*dt;
        y += v*std::sin(th)*dt;

        cv::Vec3d pose(x, y, th);
        cv::Vec3d predict_pose = mul_pose(xi0, pose);
        res.push_back(predict_pose);
    }

    return res;
}

std::vector<cv::Vec6d> MULTICONTROL::calc_trajectory(cv::Vec2d vw, double predict_time, double dt, cv::Vec3d cur_pose, cv::Vec2d cur_vw)
{
    double v_acc = update_config.motor_limit_v_acc;
    double w_acc = update_config.motor_limit_w_acc;

    double v0 = cur_vw[0];
    double w0 = cur_vw[1];

    double v = vw[0];
    double w = vw[1];
    double sgn_v = sgn(v-v0);
    double sgn_w = sgn(w-w0);

    cv::Vec6d state(0, 0, 0, v0, w0, 0); // x, y, th, v, w, sum_dt

    std::vector<cv::Vec6d> traj;
    traj.push_back(state);

    for(double sum_dt = 0; sum_dt <= predict_time; sum_dt += dt)
    {
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
        state[5] = sum_dt;

        // save trajectory
        traj.push_back(state);
    }

    // trajectory global transformation
    cv::Matx22d R;
    R(0, 0) = std::cos(cur_pose[2]);
    R(0, 1) = -std::sin(cur_pose[2]);
    R(1, 0) = std::sin(cur_pose[2]);
    R(1, 1) = std::cos(cur_pose[2]);

    cv::Vec2d T;
    T[0] = cur_pose[0];
    T[1] = cur_pose[1];

    for(size_t p = 0; p < traj.size(); p++)
    {
        cv::Vec2d P(traj[p][0], traj[p][1]);
        cv::Vec2d _P = R * P + T;
        traj[p][0] = _P[0]; // x
        traj[p][1] = _P[1]; // y
        traj[p][2] = toWrap(cur_pose[2]+traj[p][2]); // th
    }

    return traj;
}

cv::Vec3d MULTICONTROL::mul_pose(cv::Vec3d xi0, cv::Vec3d xi1)
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

// status loop
void MULTICONTROL::status_loop()
{
    // update
    mtx.lock();
    cv::Vec3d pose = slam->get_cur_pose();
    status.pose = pose;

    // dynamic obstacle
    std::vector<cv::Vec2d> obs;
    cv::Mat dynamic_obs_map = unimap->get_map_dynamic_obs();

    int r = setting_config.robot_obs_check_range/unimap->map_grid_width;
    cv::Vec2i ct = unimap->xy_uv(cv::Vec2d(pose[0], pose[1]));
    std::vector<cv::Vec2i> circle = filled_circle_iterator(ct, r);
    for(size_t p = 0; p < circle.size(); p++)
    {
        cv::Vec2i uv = circle[p];
        int u = uv[0];
        int v = uv[1];
        if(u < 0 || u >= unimap->map_w || v < 0 || v >= unimap->map_h)
        {
            continue;
        }

        if(dynamic_obs_map.ptr<uchar>(v)[u] == 255)
        {
            cv::Vec2d pt = unimap->uv_xy(uv);
            obs.push_back(pt);
        }
    }
    status.obs = obs;

    // for remaining path
    if(status.path.size() >= 2)
    {
        cv::Vec2d pos(status.pose[0], status.pose[1]);

        double d0 = cv::norm(status.path[0] - pos);
        double d1 = cv::norm(status.path[1] - pos);

        int min_p;
        if(d0 < d1)
        {
            min_p = 0;
        }
        else
        {
            min_p = 1;
        }

        if(min_p != 0)
        {
            status.path.erase(status.path.begin(), status.path.begin()+min_p);
        }
    }

    // send status
    emit signal_cts_status(status);
    mtx.unlock();
}

bool MULTICONTROL::checking_obstacle_pivot(cv::Vec2d pos, cv::Mat& obs_map)
{
    cv::Mat _obs_map;
    cv::erode(obs_map, _obs_map, cv::Mat());

    cv::Vec2i uv = unimap->xy_uv(pos);
    if(_obs_map.ptr<uchar>(uv[1])[uv[0]] == 255)
    {
        return true;
    }

    return false;
}

bool MULTICONTROL::checking_obstacle_in_trajectory(std::vector<cv::Vec3d> traj, cv::Mat& obs_map)
{
    for(size_t p = 1; p < traj.size(); p++)
    {
        cv::Vec2d pt(traj[p][0], traj[p][1]);
        cv::Vec2i uv = unimap->xy_uv(pt);

        int u = uv[0];
        int v = uv[1];
        if(u < 0 || u >= unimap->map_w || v < 0 || v >= unimap->map_h)
        {
            continue;
        }

        if(obs_map.ptr<uchar>(v)[u] == 255)
        {
            return true;
        }
    }

    return false;
}


void MULTICONTROL::marking_obstacle(int cur_idx, double dead_zone, std::vector<PATH_POINT>& path, cv::Mat& obs_map)
{
    // clear
    for(int p = 0; p < (int)path.size(); p++)
    {
        path[p].obs = 0;
        path[p].obs_v = path[p].v;
    }

    cv::Mat _obs_map;
    cv::erode(obs_map, _obs_map, cv::Mat());

    int mask_range = dead_zone/PATH_STEP;
    for(int p = cur_idx; p < (int)path.size(); p++)
    {
        cv::Vec2i uv = unimap->xy_uv(path[p].pt);
        if(_obs_map.ptr<uchar>(uv[1])[uv[0]] == 255)
        {
            for(int q = -mask_range; q <= mask_range; q++)
            {
                int i = p+q;
                if(i < 0 || i >= (int)path.size())
                {
                    continue;
                }

                path[i].obs = 1;
                path[i].obs_v = 0.01;
            }
        }
    }

    // set obs v
    smoothing_obs_v(path);
}

int MULTICONTROL::check_led_state(int cur_idx, const std::vector<PATH_POINT>& path)
{
    bool is_obs = false;
    int check_range = setting_config.robot_obs_check_range/PATH_STEP;
    for(int i = cur_idx; i <= check_range; i++)
    {
        if(i < 0 || i >= (int)path.size())
        {
            continue;
        }

        if(path[i].obs != 0)
        {
            is_obs = true;
            break;
        }
    }

    if(is_obs)
    {
        return OBS_DETECTED;
    }
    else
    {
        return OBS_NONE;
    }
}

int MULTICONTROL::check_everything_fine()
{
    if(is_loc == false)
    {
        logger.write("[MULTI] auto failed (localization failed)", true);
        return 0;
    }

    if(slam->is_travel != 0)
    {
        logger.write("[MULTI] auto failed (travel drawing worked)", true);
        return 0;
    }

    MOBILE_STATUS mobile_status = mobile->get_status();
    if(mobile_status.is_ok)
    {
        if(mobile_status.charge_state == 1)
        {
            logger.write("[MULTI] auto failed (charging)", true);
            return 0;
        }

        if(mobile_status.status_m0 > 1 || mobile_status.status_m1 > 1)
        {
            logger.write("[MULTI] auto failed (motor error)", true);
            return 0;
        }

        if(mobile_status.connection_m0 != 1 || mobile_status.connection_m1 != 1)
        {
            logger.write("[MULTI] auto failed (motor not connected)", true);
            return 0;
        }

        if(mobile_status.emo_state == 0)
        {
            logger.write("[MULTI] auto failed (emo pushed)", true);
            return 1;
        }

        if(mobile_status.status_m0 == 0 && mobile_status.status_m1 == 0)
        {
            logger.write("[MULTI] auto failed (motor lock offed)", true);
            return 1;
        }
    }

    if(ws->is_connected == false)
    {
        logger.write("[MULTI] auto failed (ws disconnected)", true);
        return 0;
    }

    // everything fine
    return 2;
}

void MULTICONTROL::fsm_loop()
{
    // real time loop
    const double dt = 0.05; // 20hz
    double pre_loop_time = get_time();
    double fine_dt = 0;

    // get driving path
    mtx.lock();
    std::vector<PATH_POINT> path = cur_path;
    cv::Vec3d goal = status.goal;
    mtx.unlock();

    // params
    double min_ld = update_config.robot_min_look_ahead_dist;
    double max_ld = update_config.robot_look_ahead_dist;

    double kp_w = update_config.robot_k_w;
    double kd_w = update_config.robot_k_w*0.1;

    double kp_v = update_config.robot_k_v;
    double kd_v = update_config.robot_k_v*0.1;

    // obs
    double obs_time = get_time();
    int pre_obs = OBS_NONE;

    // loop param
    cur_face_state = UI_FACE_NORMAL;
    mobile->led(0, LED_WHITE);

    bool is_pp_first = true;
    while(fsm_flag)
    {
        // check robot status
        int is_good_everything = check_everything_fine();
        if(is_good_everything == 0)
        {
            // stop
            mobile->move(0, 0);
            mobile->led(0, LED_MAGENTA);

            // set path complete
            mtx.lock();
            status.path.clear();
            status.state = "path_complete";
            mtx.unlock();

            fsm_state = STATE_AUTO_FAILED;
            fsm_flag = false;
            logger.write("[MULTI] everything fine check failed -> FAILED", true);

            return;
        }
        else if(is_good_everything == 1)
        {
            // stop
            mobile->move(0, 0);
            mobile->led(0, LED_CYAN_DIM);

            // set path complete
            mtx.lock();
            status.path.clear();
            status.state = "path_complete";
            mtx.unlock();

            fsm_state = STATE_AUTO_GOAL_REACHED;
            fsm_flag = false;
            logger.write("[MULTI] everything fine check failed -> GOAL_REACHED", true);

            return;
        }

        // cur state update
        cv::Vec2d cur_vw(mobile->last_v, mobile->last_w);
        cv::Vec3d cur_pose = slam->get_cur_pose();
        cv::Vec2d cur_pos(cur_pose[0], cur_pose[1]);
        double cur_th = cur_pose[2];

        // set obs map
        cv::Mat obs_map = unimap->get_map_extended_obs();

        if(fsm_state == STATE_AUTO_FIRST_ALIGN)
        {
            bool is_obs = checking_obstacle_pivot(cur_pos, obs_map);
            if(is_obs)
            {
                obs_time = get_time();
                mobile->move(0, 0);
                mobile->led(0, LED_RED);

                cur_face_state = UI_FACE_CRYING;
                fsm_state = STATE_AUTO_OBSTACLE;

                QString str;
                str.sprintf("[MULTI] FIRST_ALIGN -> STATE_OBSTACLE, obs_time: %f", obs_time);
                logger.write(str, true);
                continue;
            }

            // get index from path
            int cur_idx = get_cur_idx(cur_pose, path);
            double ref_v = path[cur_idx].v;
            double ld = saturation(ref_v, min_ld, max_ld);

            // get target point
            int tgt_idx = get_tgt_idx(cur_idx, ld, path.size());
            cv::Vec2d tgt = path[tgt_idx].pt;

            // calc err
            double dx = tgt[0] - cur_pos[0];
            double dy = tgt[1] - cur_pos[1];
            double err_th = deltaRad(std::atan2(dy,dx), cur_th);

            // goal check
            if(std::abs(err_th) < update_config.robot_goal_th)
            {
                fine_dt = 0;
                mobile->move(0, 0);

                fsm_state = STATE_AUTO_PURE_PURSUIT;
                printf("[MULTI] FIRST_ALIGN -> PP, err_th:%f\n", err_th*R2D);
                continue;
            }

            // pivot control
            int path_preset_idx = path[cur_idx].preset_idx;
            double pp_limit_v_acc = setting_config.robot_preset_limit_v_acc[path_preset_idx];
            double v = std::max<double>(cur_vw[0] - 2.0*pp_limit_v_acc*dt, 0);

            double pivot_limit_w = setting_config.robot_preset_limit_pivot[path_preset_idx];
            double pivot_limit_w_acc = setting_config.robot_preset_limit_pivot_acc[path_preset_idx];
            double w = kp_w*err_th + kd_w*cur_vw[1];
            if(std::abs(w) > pivot_limit_w)
            {
                w = sgn(w)*pivot_limit_w;
            }
            w = saturation(w, 0, cur_vw[1] + sgn(w)*pivot_limit_w_acc*dt);

            // send
            if(v > 0)
            {
                mobile->move(v, 0);
            }
            else
            {
                mobile->move(0, w);
            }
        }
        else if(fsm_state == STATE_AUTO_PURE_PURSUIT)
        {
            // get cur idx
            int cur_idx = get_cur_idx(cur_pose, path);

            // check path out
            double path_d = cv::norm(path[cur_idx].pt - cur_pos);
            if(path_d > update_config.robot_path_out_dist)
            {
                mobile->move(0, 0);
                mobile->led(0, LED_MAGENTA);

                fsm_state = STATE_AUTO_FAILED;
                fsm_flag = false;

                logger.write("[MULTI] STATE_PP -> PATH_OUT", true);
                return;
            }

            // obstacle
            marking_obstacle(cur_idx, setting_config.robot_obs_deadzone, path, obs_map);
            if(path[cur_idx].obs != 0)
            {
                obs_time = get_time();
                mobile->move(0, 0);
                mobile->led(0, LED_RED);

                cur_face_state = UI_FACE_CRYING;
                fsm_state = STATE_AUTO_OBSTACLE;
                printf("[MULTI] STATE_PP -> STATE_OBSTACLE, obs_time: %f\n", obs_time);
                continue;
            }

            // change led and face
            int cur_obs = check_led_state(cur_idx, path);
            if(cur_obs != pre_obs)
            {
                if(cur_obs == OBS_DETECTED)
                {
                    mobile->led(0, LED_RED);
                }
                else
                {
                    mobile->led(0, LED_WHITE);
                }
                pre_obs = cur_obs;
            }

            // get tgt idx
            int path_preset_idx = path[cur_idx].preset_idx;
            double pp_limit_v = setting_config.robot_preset_limit_v[path_preset_idx];
            double pp_limit_v_acc = setting_config.robot_preset_limit_v_acc[path_preset_idx];

            double pp_limit_w = setting_config.robot_preset_limit_w[path_preset_idx];
            double pp_limit_w_acc = setting_config.robot_preset_limit_w_acc[path_preset_idx];

            double ref_v = path[cur_idx].v;
            double ld = saturation(ref_v, min_ld, max_ld);

            int tgt_idx = get_tgt_idx(cur_idx, ld, path.size());
            cv::Vec2d tgt = path[tgt_idx].pt;

            // calc err and control
            double dx = tgt[0] - cur_pos[0];
            double dy = tgt[1] - cur_pos[1];
            double err_d = std::sqrt(dx*dx + dy*dy);
            double err_th = deltaRad(std::atan2(dy,dx), cur_th) * 2.0;

            // goal err
            double goal_err_d = cv::norm(path.back().pt - cur_pos);

            // return to first align
            if(goal_err_d > update_config.robot_goal_dist && std::abs(err_th) > 90.0*D2R)
            {
                fsm_state = STATE_AUTO_FIRST_ALIGN;
                printf("[MULTI] PP -> FIRST_ALIGN, idx:%d, %d, err:%f, %f\n", cur_idx, tgt_idx, goal_err_d, err_th*R2D);
                continue;
            }

            double v = 0;
            double w = 0;
            //if(goal_err_d > update_config.robot_goal_near_dist)
            {
                // pp
                double v0 = cur_vw[0];
                double w0 = cur_vw[1];

                // predict obs decel
                double limit_v = setting_config.robot_preset_limit_v[path[cur_idx].preset_idx];
                double pred_v = limit_v;

                // deceleration when obstacle is detected in the robot's path.
                if(setting_config.robot_use_obs_preview == true)
                {
                    for(double vv = 0; vv <= limit_v+0.001; vv += 0.1)
                    {
                        std::vector<cv::Vec3d> traj0 = calc_trajectory(vv, 0, 0.1, setting_config.robot_obs_preview_time, cur_pose);
                        std::vector<cv::Vec3d> traj1 = calc_trajectory(vv, cur_vw[1]*2, 0.1, setting_config.robot_obs_preview_time, cur_pose);
                        std::vector<cv::Vec3d> traj = traj0;
                        traj.insert(traj.end(), traj1.begin(), traj1.end());

                        if(checking_obstacle_in_trajectory(traj, obs_map))
                        {
                            pred_v = vv;
                        }
                    }
                }

                double v1 = std::min<double>({ref_v, path[cur_idx].obs_v, pred_v});
                double w1 = sgn(err_th)*pp_limit_w;

                double gamma = saturation(ld/max_ld, 0.0, 1.0);
                w1 *= gamma;

                double lambda = saturation(err_d/ld, 0.0, 1.0);
                w1 *= lambda;

                double t_v = calc_motion_time(err_d, v0, v1, pp_limit_v_acc);
                double t_w = calc_motion_time(err_th, w0, w1, pp_limit_w_acc);
                double t = std::max<double>(t_v, t_w);

                double max_v = std::min<double>(pp_limit_v, v0 + pp_limit_v_acc*dt);
                double min_v = std::max<double>(0, v0 - (2.0*pp_limit_v_acc)*dt); // decel case, acc*2

                double max_w = std::min<double>(pp_limit_w, w0 + pp_limit_w_acc*dt);
                double min_w = std::max<double>(-pp_limit_w, w0 - pp_limit_w_acc*dt);

                v = saturation(err_d/t, min_v, max_v);
                w = saturation(err_th/t, min_w, max_w);
            }
            /*else
            {
                // pid
                double v0 = cur_vw[0];
                double w0 = cur_vw[1];

                double max_v = std::min<double>(pp_limit_v, v0 + pp_limit_v_acc*dt);
                double min_v = std::max<double>(0, v0 - (2.0*pp_limit_v_acc)*dt); // decel case, acc*2

                double max_w = std::min<double>(pp_limit_w, w0 + pp_limit_w_acc*dt);
                double min_w = std::max<double>(-pp_limit_w, w0 - pp_limit_w_acc*dt);

                v = saturation(kp_v*err_d + kd_v*v0, min_v, max_v);
                w = saturation(kp_w*err_th + kd_w*w0, min_w, max_w);
            }*/

            // goal check
            if(goal_err_d < update_config.robot_goal_dist)
            {
                //fine_dt += dt;
                //double fine_t = std::min<double>(goal_err_d/0.01, 2.0);
                //if(fine_dt > fine_t || is_pp_first)
                {
                    // check final align
                    if(cv::norm(path.back().pt - cv::Vec2d(goal[0], goal[1])) < update_config.robot_goal_dist)
                    {
                        // stop
                        fine_dt = 0;
                        mobile->move(0, 0);

                        // path complete early
                        mtx.lock();
                        if(status.state == "path_driving")
                        {
                            status.state = "path_complete";
                        }
                        mtx.unlock();

                        fsm_state = STATE_AUTO_FINAL_ALIGN;
                        QString str;
                        str.sprintf("[MULTI] PP -> FINAL_ALIGN, goal_err_d:%f, fine_t:%f\n", goal_err_d, fine_dt);
                        logger.write(str, true);
                        continue;
                    }
                    else
                    {
                        // stop
                        mobile->move(0, 0);
                        mobile->led(0, LED_CYAN_DIM);

                        // not yet goal, but path clear
                        mtx.lock();
                        status.path.clear();
                        status.state = "path_req";
                        mtx.unlock();

                        // thread flag clear, fsm state not change, for ui
                        fsm_flag = false;
                        QString str;
                        str.sprintf("[MULTI] PP -> GOAL_REACHED, goal_err_d:%f\n", goal_err_d);
                        logger.write(str, true);
                        return;
                    }
                }
            }

            // send control
            mobile->move(v, w);
            is_pp_first = false;
        }
        else if(fsm_state == STATE_AUTO_FINAL_ALIGN)
        {
            bool is_obs = checking_obstacle_pivot(cur_pos, obs_map);
            if(is_obs)
            {
                obs_time = get_time();
                mobile->move(0, 0);
                mobile->led(0, LED_RED);

                cur_face_state = UI_FACE_CRYING;
                fsm_state = STATE_AUTO_OBSTACLE;

                QString str;
                str.sprintf("[AUTO] FINAL_ALIGN -> STATE_OBSTACLE, obs_time: %f", obs_time);
                logger.write(str, true);
                continue;
            }

            // calc error
            double goal_err_th = deltaRad(goal[2], cur_th);

            // pivot control
            int path_preset_idx = path.back().preset_idx;
            double pivot_limit_w = setting_config.robot_preset_limit_pivot[path_preset_idx];
            double pivot_limit_w_acc = setting_config.robot_preset_limit_pivot_acc[path_preset_idx];
            double w = kp_w*goal_err_th + kd_w*cur_vw[1];
            if(std::abs(w) > pivot_limit_w)
            {
                w = sgn(w)*pivot_limit_w;
            }
            w = saturation(w, 0, cur_vw[1] + sgn(w)*pivot_limit_w_acc*dt);

            // goal check
            if(std::abs(goal_err_th) < update_config.robot_goal_th)
            {
                fine_dt += dt;
                double fine_t = std::min<double>(std::abs(goal_err_th)/(1.0*D2R), 2.0);
                if(fine_dt > fine_t)
                {
                    // stop
                    fine_dt = 0;
                    mobile->move(0, 0);
                    mobile->led(0, LED_CYAN_DIM);

                    // goal reached, path clear
                    mtx.lock();
                    status.path.clear();
                    mtx.unlock();

                    // thread flag clear
                    fsm_flag = false;

                    fsm_state = STATE_AUTO_GOAL_REACHED;
                    printf("[MULTI] FINAL_ALIGN -> GOAL_REACHED, goal_err_th:%f, fine_t:%f\n", goal_err_th*R2D, fine_t);
                    return;
                }
            }

            // send control
            mobile->move(0, w);
        }
        else if(fsm_state == STATE_AUTO_OBSTACLE)
        {
            mobile->move(0, 0);
            mobile->led(0, LED_RED);
            cur_face_state = UI_FACE_CRYING;

            if(get_time() > obs_time + setting_config.robot_obs_wait_time)
            {
                mobile->led(0, LED_WHITE);

                // not yet goal, but path clear
                mtx.lock();
                status.path.clear();
                status.state = "path_req";
                mtx.unlock();

                // thread flag clear, fsm state not change, for ui
                fsm_flag = false;
                logger.write("[MULTI] OBSTACLE -> REQ");
                return;
            }
        }
        else if(fsm_state == STATE_AUTO_PAUSE)
        {
            // slow down stop
            double v_acc = 0.5;
            double v = std::max<double>(cur_vw[0] - v_acc*dt, 0);

            mobile->move(v, 0);
            mobile->led(0, LED_CYAN_DIM);
        }

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
            QString str;
            str.sprintf("[MULTI] loop time drift, dt:%f\n", delta_loop_time);
            logger.write(str, true);
        }
        pre_loop_time = get_time();
    }

    // thread flag clear

    mobile->led(0, LED_CYAN_DIM);

    cur_face_state = UI_FACE_NORMAL;
    fsm_state = STATE_AUTO_GOAL_REACHED;
    fsm_flag = false;

    logger.write("[MULTI] STATE_STOPPED", true);
}
