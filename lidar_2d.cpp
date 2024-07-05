#include "lidar_2d.h"

LIDAR_2D::LIDAR_2D(QObject *parent) : QObject(parent)
{
}

LIDAR_2D::~LIDAR_2D()
{
    if(grabThread != NULL)
    {
        grabFlag = false;
        grabThread->join();
        grabThread = NULL;
    }

    if(grabThread1 != NULL)
    {
        grabFlag1 = false;
        grabThread1->join();
        grabThread1 = NULL;
    }

    if(aThread != NULL)
    {
        aFlag = false;
        aThread->join();
        aThread = NULL;
    }
}

void LIDAR_2D::init(MOBILE *_mobile)
{
    mobile = _mobile;

#ifndef USE_SIM
    // start grab loop
    if (grabThread == NULL)
    {
        grabFlag = true;
        grabThread = new std::thread(&LIDAR_2D::grabLoop, this);
    }

#if defined(USE_DUAL_S1) || defined(USE_DUAL_S3)
    if (grabThread1 == NULL)
    {
        grabFlag1 = true;
        grabThread = new std::thread(&LIDAR_2D::grabLoop1, this);
    }
#endif

    if (aThread == NULL)
    {
        aFlag = true;
        aThread = new std::thread(&LIDAR_2D::aLoop, this);
    }
#endif
}

#if defined(USE_SINGLE_S1) || defined(USE_DUAL_S1) || defined(USE_DUAL_S3)
void LIDAR_2D::grabLoop()
{
    //sudo adduser $USER dialout
    //return;

    QString port;
    int baudrate;

#ifdef USE_SINGLE_S1
    port = "/dev/ttyRP0";
    baudrate = 256000;
#endif

#ifdef USE_DUAL_S1
    port = "/dev/ttyRPF0";
    baudrate = 256000;
#endif

#ifdef USE_DUAL_S3
    port = "/dev/ttyRPF0";
    baudrate = 1000000;
#endif

    sl::ILidarDriver* drv = *sl::createLidarDriver();
    if(!drv)
    {
        logger.write("[LIDAR] driver init failed", true);
        return;
    }

    sl::IChannel* channel = (*sl::createSerialPortChannel(port.toStdString().c_str(), baudrate));
    if(!channel->open())
    {
        logger.write("[LIDAR] port open failed", true);
        return;
    }
    else
    {
        // close for next operation
        channel->close();
    }

    if(drv->connect(channel) != SL_RESULT_OK)
    {
        logger.write("[LIDAR] connection failed", true);
        return;
    }

    std::vector<sl::LidarScanMode> modes;
    drv->getAllSupportedScanModes(modes);

    sl::LidarScanMode mode;
    float min = modes[0].us_per_sample;
    for(size_t p = 0; p < modes.size(); p++)
    {
        //printf("%s[%d] us_per_sample:%f\n", modes[p].scan_mode, modes[p].id, modes[p].us_per_sample);
        if(modes[p].us_per_sample < min)
        {
            min = modes[p].us_per_sample;
            mode = modes[p];
        }
    }

    if(drv->setMotorSpeed(DEFAULT_MOTOR_SPEED) != SL_RESULT_OK)
    {
        logger.write("[LIDAR] lidar set motor speed failed", true);
        return;
    }

    if(drv->startScanExpress(0, mode.id, 0, &mode) != SL_RESULT_OK)
    {
        logger.write("[LIDAR] start scan failed", true);
        return;
    }

    QString str;
    str.sprintf("[LIDAR] lidar scan start, MODE :%s", mode.scan_mode);
    logger.write(str, true);

    MOBILE_POSE pre_mobile_pose = mobile->get_pose();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    while(grabFlag)
    {
        sl_lidar_response_measurement_node_hq_t nodes[8192];
        size_t count = _countof(nodes);
        if(drv->grabScanDataHq(nodes, count) == SL_RESULT_OK)
        {
            double lidar_t = get_time();
            drv->ascendScanData(nodes, count);

            MOBILE_POSE mobile_pose = mobile->get_pose();
            double mobile_t = mobile_pose.t;

            int wait_cnt = 0;
            while(lidar_t >= mobile_t)
            {
                mobile_pose = mobile->get_pose();
                mobile_t = mobile_pose.t;

                wait_cnt++;
                if(wait_cnt > 500)
                {
                    QString str;
                    str.sprintf("[LIDAR] mobile pose time drift, dt: %f", lidar_t-mobile_t);
                    logger.write(str, true);

                    // try resync
                    mobile->l2c.is_resync = true;
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }

            // interpolate mobile pose
            cv::Vec3d intp_pose0;
            cv::Vec3d intp_pose;
            if(pre_mobile_pose.t < mobile_pose.t)
            {
                intp_pose0 = intpXi(lidar_t - 0.1, pre_mobile_pose.t, mobile_pose.t, pre_mobile_pose.pose, mobile_pose.pose);
                intp_pose = intpXi(lidar_t, pre_mobile_pose.t, mobile_pose.t, pre_mobile_pose.pose, mobile_pose.pose);
            }
            else
            {
                pre_mobile_pose = mobile_pose;
                continue;
            }

            const double t0 = 0;
            const double t1 = 360;

            std::vector<double> ts;
            std::vector<cv::Vec2d> pts;
            for(size_t p = 0; p < count; p++)
            {
                if(nodes[p].dist_mm_q2 == 0)
                {
                    continue;
                }

                double deg =(nodes[p].angle_z_q14 * 90.0)/16384.0;
                double t = deg;
                double th = deg*D2R;
                double dist = (nodes[p].dist_mm_q2/4.0)/1000.0;
                if(dist > setting_config.robot_lidar_max_range)
                {
                    continue;
                }

                double x = std::cos(th)*dist;
                double y = std::sin(th)*dist;
                if(!isfinite(x) || !isfinite(y))
                {
                    continue;
                }

                ts.push_back(t);
                pts.push_back(cv::Vec2d(x,y));
            }

            LIDAR_FRM frm;
            frm.t = lidar_t;
            frm.t0 = t0;
            frm.t1 = t1;
            frm.ts = ts;
            frm.pts = pts;
            frm.mobile_pose = intp_pose;
            frm.pre_mobile_pose = intp_pose0;

            pre_mobile_pose = mobile_pose;

            raw_que.push(frm);
            if(raw_que.unsafe_size() > 50)
            {
                LIDAR_FRM temp;
                raw_que.try_pop(temp);
            }

        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    drv->stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    drv->setMotorSpeed(0);

    delete drv;
    drv = NULL;
}
#endif

#ifdef USE_DUAL_SICK
void LIDAR_2D::grabLoop()
{
    printf("[LIDAR] start grabLoop\n");

    // sensors
    std::string ip_str0 = "192.168.2.11";
    std::string ip_str1 = "192.168.2.10";

    sick::types::ip_address_t sensor_ip0 = boost::asio::ip::address_v4::from_string(ip_str0);
    sick::types::ip_address_t sensor_ip1 = boost::asio::ip::address_v4::from_string(ip_str1);
    sick::types::port_t tcp_port0{2122};
    sick::types::port_t tcp_port1{2122};

    // host pc
    std::string host_ip_str0 = "192.168.2.2";
    std::string host_ip_str1 = "192.168.2.2";

    sick::datastructure::CommSettings comm_settings0;
    comm_settings0.host_ip = boost::asio::ip::address_v4::from_string(host_ip_str0); // laptop(temporal)
    comm_settings0.host_udp_port = 0;
    comm_settings0.e_interface_type = 4;
    comm_settings0.publishing_frequency = 2; // 1:25 hz, 2:12.5 hz

    sick::datastructure::CommSettings comm_settings1;
    comm_settings1.host_ip = boost::asio::ip::address_v4::from_string(host_ip_str1); // laptop(temporal)
    comm_settings1.host_udp_port = 0;
    comm_settings1.e_interface_type = 4;
    comm_settings1.publishing_frequency = 2; // 1:25 hz, 2:12.5 hz

    // create instance
    auto safety_scanner0 = std::make_unique<sick::SyncSickSafetyScanner>(sensor_ip0, tcp_port0, comm_settings0);
    auto safety_scanner1 = std::make_unique<sick::SyncSickSafetyScanner>(sensor_ip1, tcp_port1, comm_settings1);

    printf("[LIDAR] run\n");

    MOBILE_POSE pre_mobile_pose0 = mobile->get_pose();
    MOBILE_POSE pre_mobile_pose1 = mobile->get_pose();

    while(grabFlag)
    {
        if(safety_scanner0->isDataAvailable())
        {
            auto timeout = boost::posix_time::seconds(1);
            sick::datastructure::Data data = safety_scanner0->receive(timeout);

            double lidar_t = get_time();
            MOBILE_POSE mobile_pose = mobile->get_pose();
            double mobile_t = mobile_pose.t;

            int wait_cnt = 0;
            while(lidar_t >= mobile_t)
            {
                mobile_pose = mobile->get_pose();
                mobile_t = mobile_pose.t;

                wait_cnt++;
                if(wait_cnt > 500)
                {
                    QString str;
                    str.sprintf("[LIDAR] mobile pose time drift, front, dt: %f", lidar_t-mobile_t);
                    logger.write(str, true);

                    // try resync
                    mobile->l2c.is_resync = true;
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }

            // interpolate mobile pose
            cv::Vec3d intp_pose0;
            cv::Vec3d intp_pose;
            if(pre_mobile_pose0.t < mobile_pose.t)
            {
                intp_pose0 = intpXi(lidar_t - 0.1, pre_mobile_pose0.t, mobile_pose.t, pre_mobile_pose0.pose, mobile_pose.pose);
                intp_pose = intpXi(lidar_t, pre_mobile_pose0.t, mobile_pose.t, pre_mobile_pose0.pose, mobile_pose.pose);

                // parsing
                std::vector<sick::ScanPoint> sp = data.getMeasurementDataPtr()->getScanPointsVector();

                std::vector<double> ts;
                std::vector<cv::Vec2d> pts;
                for(size_t p = 0; p < sp.size(); p+=3)
                {
                    double deg = sp[p].getAngle();
                    double dist = (double)sp[p].getDistance()/1000.0; // mm to meter
                    if(dist > 39.9)
                    {
                        continue;
                    }

                    double th = toWrap(deg*D2R); // deg to radian

                    double x = dist*std::cos(th);
                    double y = dist*std::sin(th);
                    double t = (double)p/(sp.size()-1);

                    ts.push_back(t);
                    pts.push_back(cv::Vec2d(x,y));

                    //printf("%d, th:%f, d:%f, x:%f, y:%f\n", p, th*R2D, dist, x, y);
                }

                LIDAR_FRM frm;
                frm.t = lidar_t;
                frm.t0 = 0;
                frm.t1 = 1.0;
                frm.ts = ts;
                frm.pts = pts;
                frm.mobile_pose = intp_pose;
                frm.pre_mobile_pose = intp_pose0;
                raw_que.push(frm);

                if(raw_que.unsafe_size() > 50)
                {
                    LIDAR_FRM temp;
                    raw_que.try_pop(temp);
                }

                pre_mobile_pose0 = mobile_pose;
            }
            else
            {
                pre_mobile_pose0 = mobile_pose;
            }
        }

        if(safety_scanner1->isDataAvailable())
        {
            auto timeout = boost::posix_time::seconds(1);
            sick::datastructure::Data data = safety_scanner1->receive(timeout);

            double lidar_t = get_time();
            MOBILE_POSE mobile_pose = mobile->get_pose();
            double mobile_t = mobile_pose.t;

            int wait_cnt = 0;
            while(lidar_t >= mobile_t)
            {
                mobile_pose = mobile->get_pose();
                mobile_t = mobile_pose.t;

                wait_cnt++;
                if(wait_cnt > 500)
                {
                    QString str;
                    str.sprintf("[LIDAR] mobile pose time drift, front, dt: %f", lidar_t-mobile_t);
                    logger.write(str, true);

                    // try resync
                    mobile->l2c.is_resync = true;
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }

            // interpolate mobile pose
            cv::Vec3d intp_pose0;
            cv::Vec3d intp_pose;
            if(pre_mobile_pose1.t < mobile_pose.t)
            {
                intp_pose0 = intpXi(lidar_t - 0.1, pre_mobile_pose1.t, mobile_pose.t, pre_mobile_pose1.pose, mobile_pose.pose);
                intp_pose = intpXi(lidar_t, pre_mobile_pose1.t, mobile_pose.t, pre_mobile_pose1.pose, mobile_pose.pose);

                // parsing
                std::vector<sick::ScanPoint> sp = data.getMeasurementDataPtr()->getScanPointsVector();

                std::vector<double> ts;
                std::vector<cv::Vec2d> pts;
                for(size_t p = 0; p < sp.size(); p+=3)
                {
                    double deg = sp[p].getAngle();
                    double dist = (double)sp[p].getDistance()/1000.0; // mm to meter
                    if(dist > 39.9)
                    {
                        continue;
                    }

                    double th = toWrap(deg*D2R); // deg to radian

                    double x = dist*std::cos(th);
                    double y = dist*std::sin(th);
                    double t = (double)p/(sp.size()-1);

                    ts.push_back(t);
                    pts.push_back(cv::Vec2d(x,y));

                    //printf("%d, th:%f, d:%f, x:%f, y:%f\n", p, th*R2D, dist, x, y);
                }

                LIDAR_FRM frm;
                frm.t = lidar_t;
                frm.t0 = 0;
                frm.t1 = 1.0;
                frm.ts = ts;
                frm.pts = pts;
                frm.mobile_pose = intp_pose;
                frm.pre_mobile_pose = intp_pose0;
                raw_que1.push(frm);

                if(raw_que1.unsafe_size() > 50)
                {
                    LIDAR_FRM temp;
                    raw_que1.try_pop(temp);
                }

                pre_mobile_pose1 = mobile_pose;
            }
            else
            {
                pre_mobile_pose1 = mobile_pose;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    printf("[LIDAR] stop\n");
}
#endif

#ifdef USE_DUAL_LAKI
void LIDAR_2D::grabLoop()
{
    printf("[LIDAR] start grabLoop\n");

//    LakiBeamUDP* lidar0 = new LakiBeamUDP("192.168.2.2", "2367", "192.168.2.10", "8888");
//    LakiBeamUDP* lidar1 = new LakiBeamUDP("192.168.2.2", "2368", "192.168.2.11", "8888");

    LakiBeamUDP* lidar1 = new LakiBeamUDP("192.168.2.2", "2367", "192.168.2.10", "8888");
    LakiBeamUDP* lidar0 = new LakiBeamUDP("192.168.2.2", "2368", "192.168.2.11", "8888");

    printf("[LIDAR] run\n");

    MOBILE_POSE pre_mobile_pose0 = mobile->get_pose();
    MOBILE_POSE pre_mobile_pose1 = mobile->get_pose();

    while(grabFlag)
    {
        repark_t temp_pack0;
        if(lidar0->get_repackedpack(temp_pack0))
        {
            double lidar_t = get_time();

//            double lidar_t = temp_pack0.dotcloud[0].timestamp * U2S;

            MOBILE_POSE mobile_pose = mobile->get_pose();
            double mobile_t = mobile_pose.t;

            int wait_cnt = 0;
            while(lidar_t >= mobile_t)
            {
                mobile_pose = mobile->get_pose();
                mobile_t = mobile_pose.t;

                wait_cnt++;
                if(wait_cnt > 500)
                {
                    QString str;
                    str.sprintf("[LIDAR] mobile pose time drift, front, dt: %f", lidar_t-mobile_t);
                    logger.write(str, true);

                    // try resync
                    mobile->l2c.is_resync = true;
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }

            // interpolate mobile pose
            cv::Vec3d intp_pose0;
            cv::Vec3d intp_pose;
            if(pre_mobile_pose0.t < mobile_pose.t)
            {
                intp_pose0 = intpXi(lidar_t - 0.1, pre_mobile_pose0.t, mobile_pose.t, pre_mobile_pose0.pose, mobile_pose.pose);
                intp_pose = intpXi(lidar_t, pre_mobile_pose0.t, mobile_pose.t, pre_mobile_pose0.pose, mobile_pose.pose);

                std::vector<double> ts;
                std::vector<cv::Vec2d> pts;
                for (int p = 0; p < temp_pack0.maxdots; p+=3)
                {
                    double t = double(p)/temp_pack0.maxdots-1;
                    double th = double(temp_pack0.dotcloud[p].angle)/100*D2R;
                    double dist = double(temp_pack0.dotcloud[p].distance)/1000;

                    if(dist > setting_config.robot_lidar_max_range)
                    {
                        continue;
                    }
                    if(th*R2D < 45.0+8.0 || th*R2D > 315.0-8.0)
                    {
                        continue;
                    }

                    double x = dist * std::cos(th);
                    double y = dist * std::sin(th);
                    if(!isfinite(x) || !isfinite(y))
                    {
                        continue;
                    }

                    ts.push_back(t);
                    pts.push_back(cv::Vec2d(x,y));
                }

                LIDAR_FRM frm;
                frm.t = lidar_t;
                frm.t0 = 0;
                frm.t1 = 1.0;
                frm.ts = ts;
                frm.pts = pts;
                frm.mobile_pose = intp_pose;
                frm.pre_mobile_pose = intp_pose0;
                raw_que.push(frm);

                if(raw_que.unsafe_size() > 50)
                {
                    LIDAR_FRM temp;
                    raw_que.try_pop(temp);
                }

                pre_mobile_pose0 = mobile_pose;
            }
            else
            {
                pre_mobile_pose0 = mobile_pose;
            }
        }

        repark_t temp_pack1;
        if(lidar1->get_repackedpack(temp_pack1))
        {
            double lidar_t = get_time();
//            double lidar_t = temp_pack1.dotcloud[0].timestamp * U2S;

            MOBILE_POSE mobile_pose = mobile->get_pose();
            double mobile_t = mobile_pose.t;

            int wait_cnt = 0;
            while(lidar_t >= mobile_t)
            {
                mobile_pose = mobile->get_pose();
                mobile_t = mobile_pose.t;

                wait_cnt++;
                if(wait_cnt > 500)
                {
                    QString str;
                    str.sprintf("[LIDAR] mobile pose time drift, back, dt: %f", lidar_t-mobile_t);
                    logger.write(str, true);

                    // try resync
                    mobile->l2c.is_resync = true;
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }

            // interpolate mobile pose
            cv::Vec3d intp_pose0;
            cv::Vec3d intp_pose;
            if(pre_mobile_pose1.t < mobile_pose.t)
            {
                intp_pose0 = intpXi(lidar_t - 0.1, pre_mobile_pose1.t, mobile_pose.t, pre_mobile_pose1.pose, mobile_pose.pose);
                intp_pose = intpXi(lidar_t, pre_mobile_pose1.t, mobile_pose.t, pre_mobile_pose1.pose, mobile_pose.pose);

                std::vector<double> ts;
                std::vector<cv::Vec2d> pts;
                for (int p = 0; p < temp_pack1.maxdots; p+=3)
                {
                    double t = double(p)/temp_pack1.maxdots-1;
                    double th = double(temp_pack1.dotcloud[p].angle)/100*D2R;
                    double dist = double(temp_pack1.dotcloud[p].distance)/1000;

                    if(dist > setting_config.robot_lidar_max_range)
                    {
                        continue;
                    }
                    if(th*R2D < 45.0+8.0 || th*R2D > 315.0-8.0)
                    {
                        continue;
                    }

                    double x = dist * std::cos(th);
                    double y = dist * std::sin(th);
                    if(!isfinite(x) || !isfinite(y))
                    {
                        continue;
                    }

                    ts.push_back(t);
                    pts.push_back(cv::Vec2d(x,y));
                }

                LIDAR_FRM frm;
                frm.t = lidar_t;
                frm.t0 = 0;
                frm.t1 = 1.0;
                frm.ts = ts;
                frm.pts = pts;
                frm.mobile_pose = intp_pose;
                frm.pre_mobile_pose = intp_pose0;
                raw_que1.push(frm);

                if(raw_que1.unsafe_size() > 50)
                {
                    LIDAR_FRM temp;
                    raw_que1.try_pop(temp);
                }

                pre_mobile_pose1 = mobile_pose;
            }
            else
            {
                pre_mobile_pose1 = mobile_pose;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    printf("[LIDAR] stop\n");
}
#endif

#if defined(USE_DUAL_S1) || defined(USE_DUAL_S3)
void LIDAR_2D::grabLoop1()
{
    //sudo adduser $USER dialout
    //return;

    int baudrate;
#ifdef USE_DUAL_S1
    baudrate = 256000;
#endif

#ifdef USE_DUAL_S3
    baudrate = 1000000;
#endif

    sl::ILidarDriver* drv = *sl::createLidarDriver();
    if(!drv)
    {
        logger.write("[LIDAR] driver init failed", true);
        return;
    }

    sl::IChannel* channel = (*sl::createSerialPortChannel("/dev/ttyRPB0", baudrate));
    if(!channel->open())
    {
        logger.write("[LIDAR] port open failed", true);
        return;
    }
    else
    {
        // close for next operation
        channel->close();
    }

    if(drv->connect(channel) != SL_RESULT_OK)
    {
        logger.write("[LIDAR] connection failed", true);
        return;
    }

    std::vector<sl::LidarScanMode> modes;
    drv->getAllSupportedScanModes(modes);

    sl::LidarScanMode mode;
    float min = modes[0].us_per_sample;
    for(size_t p = 0; p < modes.size(); p++)
    {
        //printf("%s[%d] us_per_sample:%f\n", modes[p].scan_mode, modes[p].id, modes[p].us_per_sample);
        if(modes[p].us_per_sample < min)
        {
            min = modes[p].us_per_sample;
            mode = modes[p];
        }
    }

    if(drv->setMotorSpeed(DEFAULT_MOTOR_SPEED) != SL_RESULT_OK)
    {
        logger.write("lidar set motor speed failed", true);
        return;
    }

    if(drv->startScanExpress(0, mode.id, 0, &mode) != SL_RESULT_OK)
    {
        logger.write("[LIDAR] start scan failed", true);
        return;
    }

    QString str;
    str.sprintf("[LIDAR] lidar scan start, MODE :%s", mode.scan_mode);
    logger.write(str, true);

    MOBILE_POSE pre_mobile_pose = mobile->get_pose();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    while(grabFlag)
    {
        sl_lidar_response_measurement_node_hq_t nodes[8192];
        size_t count = _countof(nodes);
        if(drv->grabScanDataHq(nodes, count) == SL_RESULT_OK)
        {
            double lidar_t = get_time();
            drv->ascendScanData(nodes, count);

            MOBILE_POSE mobile_pose = mobile->get_pose();
            double mobile_t = mobile_pose.t;

            int wait_cnt = 0;
            while(lidar_t >= mobile_t)
            {
                mobile_pose = mobile->get_pose();
                mobile_t = mobile_pose.t;

                wait_cnt++;
                if(wait_cnt > 500)
                {
                    QString str;
                    str.sprintf("[LIDAR] mobile pose time drift, dt: %f", lidar_t-mobile_t);
                    logger.write(str, true);

                    // try resync
                    mobile->l2c.is_resync = true;
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }

            // interpolate mobile pose
            cv::Vec3d intp_pose0;
            cv::Vec3d intp_pose;
            if(pre_mobile_pose.t < mobile_pose.t)
            {
                intp_pose0 = intpXi(lidar_t - 0.1, pre_mobile_pose.t, mobile_pose.t, pre_mobile_pose.pose, mobile_pose.pose);
                intp_pose = intpXi(lidar_t, pre_mobile_pose.t, mobile_pose.t, pre_mobile_pose.pose, mobile_pose.pose);
            }
            else
            {
                pre_mobile_pose = mobile_pose;
                continue;
            }

            const double t0 = 0;
            const double t1 = 360;

            std::vector<double> ts;
            std::vector<cv::Vec2d> pts;
            for(size_t p = 0; p < count; p++)
            {
                if(nodes[p].dist_mm_q2 == 0)
                {
                    continue;
                }

                double deg =(nodes[p].angle_z_q14 * 90.0)/16384.0;
                double t = deg;
                double th = deg*D2R;
                double dist = (nodes[p].dist_mm_q2/4.0)/1000.0;
                if(dist > setting_config.robot_lidar_max_range)
                {
                    continue;
                }

                double x = std::cos(th)*dist;
                double y = std::sin(th)*dist;
                if(!isfinite(x) || !isfinite(y))
                {
                    continue;
                }

                ts.push_back(t);
                pts.push_back(cv::Vec2d(x,y));
            }

            LIDAR_FRM frm;
            frm.t = lidar_t;
            frm.t0 = t0;
            frm.t1 = t1;
            frm.ts = ts;
            frm.pts = pts;
            frm.mobile_pose = intp_pose;
            frm.pre_mobile_pose = intp_pose0;
            raw_que1.push(frm);

            pre_mobile_pose = mobile_pose;
            if(raw_que1.unsafe_size() > 50)
            {
                LIDAR_FRM temp;
                raw_que1.try_pop(temp);
            }
        }

        //std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    drv->stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    drv->setMotorSpeed(0);

    delete drv;
    drv = NULL;
}
#endif


// single lidar aLoop
#ifdef USE_SINGLE_S1
void LIDAR_2D::aLoop()
{
    // params
    QStringList lidar_tf = static_config.robot_lidar_tf.split(",");
    if(lidar_tf.size() != 3)
    {
        printf("[LIDAR] tf invalid\n");
        return;
    }

    double offset_x = lidar_tf[0].toDouble();
    double offset_y = lidar_tf[1].toDouble();
    double offset_th = lidar_tf[2].toDouble()*D2R;

    cv::Vec3d xi_offset(offset_x, offset_y, offset_th);

    while(aFlag)
    {
        LIDAR_FRM frm;
        if(raw_que.try_pop(frm))
        {
            const double t0 = frm.t0;
            const double t1 = frm.t1;

            cv::Vec3d pre_mobile_pose = frm.pre_mobile_pose;
            cv::Vec3d mobile_pose = frm.mobile_pose;

            LIDAR_FRM _frm;
            _frm.t = frm.t;
            _frm.t0 = frm.t0;
            _frm.t1 = frm.t1;
            _frm.ts = frm.ts;
            _frm.mobile_pose = frm.mobile_pose;
            _frm.pre_mobile_pose = frm.pre_mobile_pose;

            for(size_t p = 0; p < frm.pts.size(); p++)
            {
                // motion distortion compensation using odometry
                double t = frm.ts[p];

                // lidar raw point
                cv::Vec2d P;
                P[0] = frm.pts[p][0];
                P[1] = frm.pts[p][1];

                // to robot origin
                P = transform(xi_offset, P);

                // motion compensation
                double a = (t-t1)/(t0-t1);
                double b = (t-t0)/(t1-t0);

                cv::Vec3d _xi = intpXi(a, b, pre_mobile_pose, mobile_pose);
                cv::Vec3d xi = divXi(_xi, mobile_pose);

                cv::Vec2d _P = transform(xi, P);
                if(!isfinite(_P[0]) || !isfinite(_P[1]))
                {
                    continue;
                }

                double length = static_config.robot_length/2;
                if(_P[0] >= -length && _P[0] <= length && _P[1] >= -length && _P[1] <= length)
                {
                    continue;
                }

                // storing
                _frm.ts.push_back(t);
                _frm.pts.push_back(_P);
            }

            scan_que.push(_frm);

            mtx.lock();
            cur_scan = _frm.pts;
            mtx.unlock();

            // for memory
            if(scan_que.unsafe_size() > 50)
            {
                LIDAR_FRM temp;
                scan_que.try_pop(temp);
            }

            continue;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}
#endif

// dual lidar aLoop
#if defined(USE_DUAL_SICK) || defined(USE_DUAL_LAKI) || defined(USE_DUAL_S1) || defined(USE_DUAL_S3)
void LIDAR_2D::aLoop()
{
    // params
    QStringList lidar_tf = static_config.robot_lidar_tf.split(",");
    if(lidar_tf.size() != 3)
    {
        printf("[LIDAR] tf invalid\n");
        return;
    }

    double offset_x = lidar_tf[0].toDouble();
    double offset_y = lidar_tf[1].toDouble();
    double offset_th = lidar_tf[2].toDouble()*D2R;

    cv::Vec3d xi_offset0(offset_x, offset_y, offset_th);
    cv::Vec3d xi_offset1(-offset_x, -offset_y, toWrap(M_PI + offset_th));

    std::vector<LIDAR_FRM> storage;
    while(aFlag)
    {
        LIDAR_FRM frm0;
        if(raw_que.try_pop(frm0))
        {
            raw_que.clear();

            LIDAR_FRM frm_ref;
            while(raw_que1.try_pop(frm_ref))
            {
                storage.push_back(frm_ref);
            }

            int min_idx = -1;
            double min_dt = 999999;
            for(size_t p = 0; p < storage.size(); p++)
            {
                double dt = std::abs(storage[p].t-frm0.t);
                if(dt < min_dt)
                {
                    min_dt = dt;
                    min_idx = p;
                }
            }

            if(min_idx >= 0)
            {
                LIDAR_FRM frm1 = storage[min_idx];

                const double t0 = frm0.t0;
                const double t1 = frm0.t1;

                cv::Vec3d pre_mobile_pose0 = frm0.pre_mobile_pose;
                cv::Vec3d mobile_pose0 = frm0.mobile_pose;

                cv::Vec3d pre_mobile_pose1 = frm1.pre_mobile_pose;
                cv::Vec3d mobile_pose1 = frm1.mobile_pose;

                LIDAR_FRM _frm0;
                _frm0.t = frm0.t;
                _frm0.t0 = frm0.t0;
                _frm0.t1 = frm0.t1;
                _frm0.ts = frm0.ts;
                _frm0.mobile_pose = frm0.mobile_pose;
                _frm0.pre_mobile_pose = frm0.pre_mobile_pose;

                LIDAR_FRM _frm1;
                _frm1.t = frm1.t;
                _frm1.t0 = frm1.t0;
                _frm1.t1 = frm1.t1;
                _frm1.ts = frm1.ts;
                _frm1.mobile_pose = frm1.mobile_pose;
                _frm1.pre_mobile_pose = frm1.pre_mobile_pose;

                for(size_t p = 0; p < frm0.pts.size(); p++)
                {
                    // motion distortion compensation using odometry
                    double t = frm0.ts[p];

                    cv::Vec2d P;
                    P[0] = frm0.pts[p][0];
                    P[1] = frm0.pts[p][1];

                    P = transform(xi_offset0, P);

                    double a = (t-t1)/(t0-t1);
                    double b = (t-t0)/(t1-t0);

                    cv::Vec3d _xi = intpXi(a, b, pre_mobile_pose0, mobile_pose0);
                    cv::Vec3d xi = divXi(_xi, mobile_pose0);

                    cv::Vec2d _P = transform(xi, P);
                    if(!isfinite(_P[0]) || !isfinite(_P[1]))
                    {
                        continue;
                    }

                    double lx = static_config.robot_length_x/2;
                    double ly = static_config.robot_length_y/2;
                    if(_P[0] >= -lx && _P[0] <= lx && _P[1] >= -ly && _P[1] <= ly)
                    {
                        continue;
                    }

                    // storing
                    _frm0.ts.push_back(t);
                    _frm0.pts.push_back(_P);
                }

                for(size_t p = 0; p < frm1.pts.size(); p++)
                {
                    // motion distortion compensation using odometry
                    double t = frm1.ts[p];

                    cv::Vec2d P;
                    P[0] = frm1.pts[p][0];
                    P[1] = frm1.pts[p][1];

                    P = transform(xi_offset1, P);

                    double a = (t-t1)/(t0-t1);
                    double b = (t-t0)/(t1-t0);

                    cv::Vec3d _xi = intpXi(a, b, pre_mobile_pose1, mobile_pose1);
                    cv::Vec3d xi = divXi(_xi, mobile_pose1);

                    cv::Vec2d _P = transform(xi, P);
                    if(!isfinite(_P[0]) || !isfinite(_P[1]))
                    {
                        continue;
                    }

                    double lx = static_config.robot_length_x/2;
                    double ly = static_config.robot_length_y/2;
                    if(_P[0] >= -lx && _P[0] <= lx && _P[1] >= -ly && _P[1] <= ly)
                    {
                        continue;
                    }

                    // storing
                    _frm1.ts.push_back(t);
                    _frm1.pts.push_back(_P);
                }

                auto del = divXi(mobile_pose1, mobile_pose0);
                for(size_t p = 0; p < _frm1.pts.size(); p++)
                {
                    cv::Vec2d _P = _frm1.pts[p];
                    cv::Vec2d P = transform(del, _P);
                    _frm1.pts[p] = P;
                }

                _frm0.ts.insert(_frm0.ts.end(), _frm1.ts.begin(), _frm1.ts.end());
                _frm0.pts.insert(_frm0.pts.end(), _frm1.pts.begin(), _frm1.pts.end());

                scan_que.push(_frm0);

                mtx.lock();
                cur_scan = _frm0.pts;
                mtx.unlock();

                // for memory
                if(scan_que.unsafe_size() > 50)
                {
                    LIDAR_FRM temp;
                    scan_que.try_pop(temp);
                }

                storage.erase(storage.begin(), storage.begin()+min_idx);
                continue;
            }

        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}
#endif

std::vector<cv::Vec2d> LIDAR_2D::get_cur_scan()
{
    mtx.lock();
    std::vector<cv::Vec2d> _cur_scan = cur_scan;
    mtx.unlock();

    return _cur_scan;
}

cv::Vec2d LIDAR_2D::transform(cv::Vec3d xi, cv::Vec2d pt)
{
    cv::Matx22d R;
    R(0, 0) = std::cos(xi[2]);
    R(0, 1) = -std::sin(xi[2]);
    R(1, 0) = std::sin(xi[2]);
    R(1, 1) = std::cos(xi[2]);

    cv::Vec2d T;
    T[0] = xi[0];
    T[1] = xi[1];

    return R*pt + T;
}

cv::Vec3d LIDAR_2D::intpXi(double a, double b, cv::Vec3d xi0, cv::Vec3d xi1)
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

cv::Vec3d LIDAR_2D::intpXi(double t, double t0, double t1, cv::Vec3d xi0, cv::Vec3d xi1)
{
    double a = (t-t1)/(t0-t1);
    double b = (t-t0)/(t1-t0);

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

cv::Vec3d LIDAR_2D::divXi(cv::Vec3d xi0, cv::Vec3d xi1)
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

cv::Vec3d LIDAR_2D::mulXi(cv::Vec3d xi0, cv::Vec3d xi1)
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
