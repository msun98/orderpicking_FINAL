#include "cam.h"

CAM::CAM(QObject *parent) : QObject(parent)
{
//    std::cout<<"CCCCCCCCAMMMMMMMMMMMMMM"<<std::endl;
}

CAM::~CAM()
{
    if(grabThread != NULL)
    {
        grabFlag = false;
        grabThread->join();
        grabThread = NULL;
    }
}

void CAM::init()
{
//    // start grab loop
//    if (grabThread == NULL)
//    {
//        grabFlag = true;
//        grabThread = new std::thread(&CAM::grabLoop, this);
//    }
}

void CAM::grabLoop()
{/*
#ifndef USE_SIM
    // check device
    ob::Context::setLoggerSeverity(OB_LOG_SEVERITY_FATAL);
    ob::Context ctx;
    auto devList = ctx.queryDeviceList();
    int devCount = devList->deviceCount();
    if(devCount != 2)
    {
        for(int p = 0; p < devCount; p++)
        {
            auto dev = devList->getDevice(p);
            auto dev_info = dev->getDeviceInfo();
            QString sn = dev_info->serialNumber();
            logger.write("[CAM] installed cam serial : " + sn, true);
        }

        logger.write("[CAM] only support two device, failed", true);
        return;
    }

    // get camera
    std::vector<std::shared_ptr<ob::Pipeline>> pipes(2);
    for(int p = 0; p < devCount; p++)
    {
        auto dev = devList->getDevice(p);
        dev->setBoolProperty(OB_PROP_COLOR_MIRROR_BOOL, false);
        dev->setBoolProperty(OB_PROP_DEPTH_MIRROR_BOOL, true);

        auto pipe = std::make_shared<ob::Pipeline>(dev);

        auto dev_info = dev->getDeviceInfo();
        QString sn = dev_info->serialNumber();
        logger.write("[CAM] installed cam serial : " + sn, true);

        if(sn == static_config.robot_cam_left_sn)
        {
            // left
            pipes[0] = pipe;
            sn_l = sn;
            logger.write("[CAM] left camera serial : " + sn, true);
        }
        else if(sn == static_config.robot_cam_right_sn)
        {
            // right
            pipes[1] = pipe;
            sn_r = sn;
            logger.write("[CAM] right camera serial : " + sn, true);
        }
        else
        {
            if(p == 0)
            {
                pipes[0] = pipe;
                sn_l = sn;
                logger.write("[CAM] left camera serial : " + sn, true);
            }
            else
            {
                pipes[1] = pipe;
                sn_r = sn;
                logger.write("[CAM] right camera serial : " + sn, true);
            }

            logger.write("[CAM] no match serial number", true);
        }
    }

    // params
    QStringList l_tf = static_config.robot_cam_left_tf.split(",");
    if(l_tf.size() != 6)
    {
        printf("cam left tf invalid\n");
        return;
    }

    QStringList r_tf = static_config.robot_cam_right_tf.split(",");
    if(r_tf.size() != 6)
    {
        printf("cam right tf invalid\n");
        return;
    }

    Eigen::Matrix4d T_l = zyx_tranformation(l_tf[0].toDouble(), l_tf[1].toDouble(), l_tf[2].toDouble(), l_tf[3].toDouble(), l_tf[4].toDouble(), l_tf[5].toDouble());
    Eigen::Matrix4d T_r = zyx_tranformation(r_tf[0].toDouble(), r_tf[1].toDouble(), r_tf[2].toDouble(), r_tf[3].toDouble(), r_tf[4].toDouble(), r_tf[5].toDouble());

    // setting
    std::vector<std::shared_ptr<ob::Config>> configs(2);
    for(size_t p = 0; p < pipes.size(); p++)
    {
        auto pipe = pipes[p];
        auto config = configs[p];
        config = std::make_shared<ob::Config>();
        config->disableAllStream();

        auto depthProfileList = pipe->getStreamProfileList(OB_SENSOR_DEPTH);
        std::shared_ptr<ob::VideoStreamProfile> depthProfile = depthProfileList->getVideoStreamProfile(640, 360, OB_FORMAT_Y11, 30);
        {
            depthProfile = std::const_pointer_cast<ob::StreamProfile>(depthProfileList->getProfile(0))->as<ob::VideoStreamProfile>();

            int w = depthProfile->width();
            int h = depthProfile->height();
            int fps = depthProfile->fps();
            printf("[CAM] depth id:%d, w:%d, h:%d, fps:%d\n", p, w, h, fps);
        }
        config->enableStream(depthProfile);

        auto colorProfileList = pipe->getStreamProfileList(OB_SENSOR_COLOR);
        std::shared_ptr<ob::VideoStreamProfile> colorProfile = colorProfileList->getVideoStreamProfile(640, 360, OB_FORMAT_RGB, 30);
        {
            colorProfile = std::const_pointer_cast<ob::StreamProfile>(colorProfileList->getProfile(0))->as<ob::VideoStreamProfile>();

            int w = colorProfile->width();
            int h = colorProfile->height();
            int fps = colorProfile->fps();
            printf("[CAM] color id:%d, w:%d, h:%d, fps:%d\n", p, w, h, fps);
        }
        config->enableStream(colorProfile);
        config->setAlignMode(ALIGN_DISABLE);
    }

    // frame callback left
    pipes[0]->start(configs[0], [&](std::shared_ptr<ob::FrameSet> frameset)
    {
        if(frameset->depthFrame() != nullptr)
        {
            uint64_t ts = frameset->depthFrame()->systemTimeStamp();
            double time = (double)ts/1000.0 - st_time_for_get_time;

            // get point cloud
            ob::PointCloudFilter pointCloud;
            auto cameraParam = pipes[0]->getCameraParam();

            pointCloud.setCameraParam(cameraParam);
            pointCloud.setCreatePointFormat(OB_FORMAT_POINT);

            auto depthValueScale = frameset->depthFrame()->getValueScale();
            pointCloud.setPositionDataScaled(depthValueScale);

            std::shared_ptr<ob::Frame> frame = pointCloud.process(frameset);
            OBPoint *point = (OBPoint *)frame->data();

            int w = 640;
            int h = 360;
            std::vector<cv::Vec3d> pts;
            for(int i = 0; i < h; i++)
            {
                for(int j = 0; j < w; j++)
                {
                    double x = -point[i*w+j].x/1000.0;
                    double y = point[i*w+j].y/1000.0;
                    double z = point[i*w+j].z/1000.0;
                    if(x != 0 || y != 0 || z != 0)
                    {
                        Eigen::Vector3d P = T_l.block(0,0,3,3)*Eigen::Vector3d(x,y,z)+T_l.block(0,3,3,1);
                        if(P[2] < setting_config.robot_obs_cam_height_min || P[2] > setting_config.robot_obs_cam_height_max)
                        {
                            continue;
                        }

                        pts.push_back(cv::Vec3d(P[0],P[1],P[2]));
                    }
                }
            }

            // sampling
            double sampling_grid = 0.02; // 2cm
            int sample_w = (setting_config.robot_obs_check_range / sampling_grid)*2;
            int sample_c = sample_w/2;

            cv::Mat sampling_mask(sample_w, sample_w, CV_32S, cv::Scalar(0));
            for(size_t p = 0; p < pts.size(); p++)
            {
                int u = pts[p][0]/sampling_grid + sample_c;
                int v = pts[p][1]/sampling_grid + sample_c;
                if(u < 0 || u >= sample_w || v < 0 || v >= sample_w)
                {
                    continue;
                }
                sampling_mask.ptr<int>(v)[u] = p;
            }

            std::vector<cv::Vec2d> sampled_scan;            
            for(int i = 0; i < sample_w; i++)
            {
                for(int j = 0; j < sample_w; j++)
                {
                    int idx = sampling_mask.ptr<int>(i)[j];
                    if(idx != 0)
                    {
                        sampled_scan.push_back(cv::Vec2d(pts[idx][0], pts[idx][1]));                        
                    }
                }
            }

            // update
            mtx_l.lock();
            time_l = time;
            cur_scan_l = sampled_scan;
            mtx_l.unlock();
        }

        if(frameset->colorFrame() != nullptr)
        {
            // get color image
            std::shared_ptr<ob::ColorFrame> colorFrame = frameset->colorFrame();
            cv::Mat img_c_raw(1, colorFrame->dataSize(), CV_8UC1, colorFrame->data());
            cv::Mat img_c = cv::imdecode(img_c_raw, 1);
            cv::flip(img_c, img_c, -1);

            // get gray image
            cv::Mat img;
            cv::cvtColor(img_c, img, cv::COLOR_BGR2GRAY);

            // update
            mtx_l.lock();
            cur_img_l = img.clone();
            cur_img_l_color = img_c.clone();
            mtx_l.unlock();
        }
    });

    // frame callback right
    pipes[1]->start(configs[1], [&](std::shared_ptr<ob::FrameSet> frameset)
    {
        if(frameset->depthFrame() != nullptr)
        {
            uint64_t ts = frameset->depthFrame()->systemTimeStamp();
            double time = (double)ts/1000.0 - st_time_for_get_time;

            // get point cloud
            ob::PointCloudFilter pointCloud;
            auto cameraParam = pipes[0]->getCameraParam();
            pointCloud.setCameraParam(cameraParam);
            pointCloud.setCreatePointFormat(OB_FORMAT_POINT);

            auto depthValueScale = frameset->depthFrame()->getValueScale();
            pointCloud.setPositionDataScaled(depthValueScale);

            std::shared_ptr<ob::Frame> frame = pointCloud.process(frameset);
            OBPoint *point = (OBPoint *)frame->data();

            int w = 640;
            int h = 360;
            std::vector<cv::Vec3d> pts;
            for(int i = 0; i < h; i++)
            {
                for(int j = 0; j < w; j++)
                {
                    double x = -point[i*w+j].x/1000.0;
                    double y = point[i*w+j].y/1000.0;
                    double z = point[i*w+j].z/1000.0;
                    if(x != 0 || y != 0 || z != 0)
                    {
                        Eigen::Vector3d P = T_r.block(0,0,3,3)*Eigen::Vector3d(x,y,z)+T_r.block(0,3,3,1);
                        if(P[2] < setting_config.robot_obs_cam_height_min || P[2] > setting_config.robot_obs_cam_height_max)
                        {
                            continue;
                        }

                        pts.push_back(cv::Vec3d(P[0],P[1],P[2]));
                    }
                }
            }

            // sampling
            double sampling_grid = 0.02; // 2cm
            int sample_w = (setting_config.robot_obs_check_range / sampling_grid)*2;
            int sample_c = sample_w/2;

            cv::Mat sampling_mask(sample_w, sample_w, CV_32S, cv::Scalar(0));
            for(size_t p = 0; p < pts.size(); p++)
            {
                int u = pts[p][0]/sampling_grid + sample_c;
                int v = pts[p][1]/sampling_grid + sample_c;
                if(u < 0 || u >= sample_w || v < 0 || v >= sample_w)
                {
                    continue;
                }
                sampling_mask.ptr<int>(v)[u] = p;
            }

            std::vector<cv::Vec2d> sampled_scan;
            for(int i = 0; i < sample_w; i++)
            {
                for(int j = 0; j < sample_w; j++)
                {
                    int idx = sampling_mask.ptr<int>(i)[j];
                    if(idx != 0)
                    {
                        sampled_scan.push_back(cv::Vec2d(pts[idx][0], pts[idx][1]));
                    }
                }
            }

            // update
            mtx_r.lock();
            time_r = time;
            cur_scan_r = sampled_scan;
            mtx_r.unlock();
        }

        if(frameset->colorFrame() != nullptr)
        {
            // get color image
            std::shared_ptr<ob::ColorFrame> colorFrame = frameset->colorFrame();
            cv::Mat img_c_raw(1, colorFrame->dataSize(), CV_8UC1, colorFrame->data());
            cv::Mat img_c = cv::imdecode(img_c_raw, 1);
            //cv::flip(img_c, img_c, -1);

            // get gray image
            cv::Mat img;
            cv::cvtColor(img_c, img, cv::COLOR_BGR2GRAY);

            // update
            mtx_r.lock();
            cur_img_r = img.clone();
            cur_img_r_color = img_c.clone();
            mtx_r.unlock();
        }
    });

    while(grabFlag)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    pipes[0]->stop();
    pipes[1]->stop();

    #endif
//*/
}

QString CAM::get_sn_l()
{
    mtx_l.lock();
    QString _sn_l = sn_l;
    mtx_l.unlock();

    return _sn_l;
}

QString CAM::get_sn_r()
{
    mtx_r.lock();
    QString _sn_r = sn_r;
    mtx_r.unlock();

    return _sn_r;
}

cv::Mat CAM::get_cur_img_l()
{
    mtx_l.lock();
    cv::Mat _cur_img_l = cur_img_l.clone();
    mtx_l.unlock();

    return _cur_img_l;
}

cv::Mat CAM::get_cur_img_r()
{
    mtx_r.lock();
    cv::Mat _cur_img_r = cur_img_r.clone();
    mtx_r.unlock();

    return _cur_img_r;
}

cv::Mat CAM::get_cur_img_l_color()
{
    mtx_l.lock();
    cv::Mat _cur_img_l_color = cur_img_l_color.clone();
    mtx_l.unlock();

    return _cur_img_l_color;
}

cv::Mat CAM::get_cur_img_r_color()
{
    mtx_r.lock();
    cv::Mat _cur_img_r_color = cur_img_r_color.clone();
    mtx_r.unlock();

    return _cur_img_r_color;
}

std::vector<cv::Vec2d> CAM::get_cur_scan_l()
{
    mtx_l.lock();
    std::vector<cv::Vec2d> res = cur_scan_l;
    mtx_l.unlock();

    return res;
}

std::vector<cv::Vec2d> CAM::get_cur_scan_r()
{
    mtx_r.lock();
    std::vector<cv::Vec2d> res = cur_scan_r;
    mtx_r.unlock();

    return res;
}

Eigen::Matrix4d CAM::zyx_tranformation(double x, double y, double z, double rx, double ry, double rz)
{
    Eigen::Matrix4d res;
    res.setIdentity();

    Eigen::AngleAxisd Rz(rz*D2R, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd Ry(ry*D2R, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd Rx(rx*D2R, Eigen::Vector3d::UnitX());

    Eigen::Quaternion<double> q = Rz * Ry * Rx;

    res.block(0,0,3,3) = q.matrix();
    res(0,3) = x;
    res(1,3) = y;
    res(2,3) = z;

    return res;
}

