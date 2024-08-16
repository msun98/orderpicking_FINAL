#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , ipc(this)
    , cam(this)
    , lidar(this)
    , mobile(this)
    , unimap(this)
    , topomap(this)
    , slam(this)
    , ctrl(this)
    , mctrl(this)
    , ws(this)
    , integrate_ui(this)
    , sim(this)
    , plot_timer(this)
    , watchdog_timer(this)
{
    ui->setupUi(this);

    // mapping
    connect(ui->bt_MapRun, SIGNAL(clicked()), this, SLOT(bt_MapRun()));
    connect(ui->bt_MapStop, SIGNAL(clicked()), this, SLOT(bt_MapStop()));
    connect(ui->bt_MapSave, SIGNAL(clicked()), this, SLOT(bt_MapSave()));
    connect(ui->bt_MapManualLc, SIGNAL(clicked()), this, SLOT(bt_MapManualLc()));

    // robot move
    connect(ui->bt_RobotMotorInit, SIGNAL(clicked()), this, SLOT(bt_RobotMotorInit()));
    connect(ui->bt_RobotMoveForward, SIGNAL(pressed()), this, SLOT(bt_RobotMoveForward_pressed()));
    connect(ui->bt_RobotMoveBackward, SIGNAL(pressed()), this, SLOT(bt_RobotMoveBackward_pressed()));
    connect(ui->bt_RobotMoveLeftTurn, SIGNAL(pressed()), this, SLOT(bt_RobotMoveLeftTurn_pressed()));
    connect(ui->bt_RobotMoveRightTurn, SIGNAL(pressed()), this, SLOT(bt_RobotMoveRightTurn_pressed()));

    connect(ui->bt_RobotMoveForward, SIGNAL(released()), this, SLOT(bt_RobotMove_released()));
    connect(ui->bt_RobotMoveBackward, SIGNAL(released()), this, SLOT(bt_RobotMove_released()));
    connect(ui->bt_RobotMoveLeftTurn, SIGNAL(released()), this, SLOT(bt_RobotMove_released()));
    connect(ui->bt_RobotMoveRightTurn, SIGNAL(released()), this, SLOT(bt_RobotMove_released()));

    // localization
    connect(ui->bt_LocalizationLoad, SIGNAL(clicked()), this, SLOT(bt_LocalizationLoad()));
    connect(ui->bt_LocalizationInit, SIGNAL(clicked()), this, SLOT(bt_LocalizationInit()));
    connect(ui->bt_LocalizationInitAuto_Resting, SIGNAL(clicked()), this, SLOT(bt_LocalizationInitAuto_Resting()));
    connect(ui->bt_LocalizationInitAuto_Semi, SIGNAL(clicked()), this, SLOT(bt_LocalizationInitAuto_Semi()));
    connect(ui->bt_LocalizationInitAuto_Full, SIGNAL(clicked()), this, SLOT(bt_LocalizationInitAuto_Full()));
    connect(ui->bt_LocalizationRun, SIGNAL(clicked()), this, SLOT(bt_LocalizationRun()));
    connect(ui->bt_LocalizationStop, SIGNAL(clicked()), this, SLOT(bt_LocalizationStop()));
    connect(&slam, SIGNAL(auto_init_finished_resting()), this, SLOT(AutoInitFinished()));
    connect(&slam, SIGNAL(auto_init_finished_semi()), this, SLOT(AutoInitFinished()));
    connect(&slam, SIGNAL(auto_init_finished_full()), this, SLOT(AutoInitFinished()));

    // manual
    connect(ui->cb_ManualServingGroup, SIGNAL(currentIndexChanged(int)), this, SLOT(cb_ManualServingGroup()));

    connect(ui->bt_ManualServingRun, SIGNAL(clicked()), this, SLOT(bt_ManualServingRun()));
    connect(ui->bt_ManualRestingRun, SIGNAL(clicked()), this, SLOT(bt_ManualRestingRun()));
    connect(ui->bt_ManualChargingRun, SIGNAL(clicked()), this, SLOT(bt_ManualChargingRun()));
    connect(ui->bt_ManualNodesRun, SIGNAL(clicked()), this, SLOT(bt_ManualNodesRun()));

    connect(ui->bt_ManualServingSet, SIGNAL(clicked()), this, SLOT(bt_ManualServingSet()));
    connect(ui->bt_ManualRestingSet, SIGNAL(clicked()), this, SLOT(bt_ManualRestingSet()));
    connect(ui->bt_ManualChargingSet, SIGNAL(clicked()), this, SLOT(bt_ManualChargingSet()));

    connect(ui->ckb_SetLocation, SIGNAL(stateChanged(int)), this, SLOT(ckb_SetLocation(int)));

    connect(ui->bt_ManualClickedRun, SIGNAL(clicked()), this, SLOT(bt_ManualClickedRun()));
    connect(ui->bt_ManualStop, SIGNAL(clicked()), this, SLOT(bt_ManualStop()));
    connect(ui->bt_ManualPause, SIGNAL(clicked()), this, SLOT(bt_ManualPause()));
    connect(ui->bt_ManualResume, SIGNAL(clicked()), this, SLOT(bt_ManualResume()));

    // temporal
    connect(ui->bt_ReloadRobotConfig, SIGNAL(clicked()), this, SLOT(bt_ReloadRobotConfig()));
    connect(ui->bt_SimInit, SIGNAL(clicked()), this, SLOT(bt_SimInit()));
    connect(ui->bt_Test, SIGNAL(clicked()), this, SLOT(bt_Test()));
    connect(ui->bt_Test2, SIGNAL(clicked()), this, SLOT(bt_Test2()));
    connect(ui->bt_Test3, SIGNAL(clicked()), this, SLOT(bt_Test3()));
    connect(ui->bt_Test4, SIGNAL(clicked()), this, SLOT(bt_Test4()));

    connect(ui->bt_SimEmoPush, SIGNAL(clicked()), this, SLOT(bt_SimEmoPush()));
    connect(ui->bt_SimEmoRelease, SIGNAL(clicked()), this, SLOT(bt_SimEmoRelease()));

    connect(ui->bt_DrawStart, SIGNAL(clicked()), this, SLOT(bt_DrawStart()));
    connect(ui->bt_DrawStop, SIGNAL(clicked()), this, SLOT(bt_DrawStop()));

    connect(ui->bt_RobotSetLed, SIGNAL(clicked()), this, SLOT(bt_RobotSetLed()));
    connect(ui->bt_Exit, SIGNAL(clicked()), this, SLOT(bt_Exit()));
    connect(ui->bt_CheckAllPath, SIGNAL(clicked()), this, SLOT(bt_CheckAllPath()));
    connect(&cmd_server, SIGNAL(process_command_signal(CMD)), this, SLOT(process_command(CMD)));

    connect(ui->gv_Screen1, SIGNAL(pose_clicked(double, double, double)), this, SLOT(make_sim_virtualObs()));

    // plot
    connect(&plot_timer, SIGNAL(timeout()), this, SLOT(plot_loop()));
    connect(&watchdog_timer, SIGNAL(timeout()), this, SLOT(watchdog_loop()));

    // re-load
    connect(ui->bt_SoftReload, SIGNAL(clicked()), this, SLOT(bt_SoftReload()));
    connect(ui->bt_HardReload, SIGNAL(clicked()), this, SLOT(bt_HardReload()));

    // send maps
    connect(ui->bt_SendMaps, SIGNAL(clicked()), this, SLOT(bt_SendMaps()));

    ui->lb_VersionInfo->setText("VS:" + QString::fromStdString(VERSION_LOG));

    // build datetime
    QString compilationTime = "BT:" + QString::fromStdString(BUILD_DATE);
    ui->lb_BuildInfo->setText(compilationTime);

    // set fms info
    ui->le_IP2Send->setText(setting_config.fms_ip);
    ui->le_ID2Send->setText(setting_config.fms_id);
    ui->le_PW2Send->setText(setting_config.fms_pw);

    // integrate ui (orderPicking)
    connect(ui->bt_TestMoveExt, SIGNAL(clicked()), this, SLOT(bt_TestMoveExt()));
    connect(ui->bt_TestMovePick, SIGNAL(clicked()), this, SLOT(bt_TestMovePick()));
    connect(&integrate_ui, SIGNAL(change_map(QString)), this, SLOT(websocket_map_changed(QString)));
    connect(&integrateUI_timer, SIGNAL(timeout()), this, SLOT(IntegrateUILoop()));
    integrateUI_timer.start(1000);

    // initialize
    init();

    logger.write("[MAIN] Initilized", true);
}

MainWindow::~MainWindow()
{
    slam.stop();
    slam.stop_loc();

    watchdog_timer.stop();
    plot_timer.stop();
    integrateUI_timer.stop();

    if(backThread != NULL)
    {
        backFlag = false;
        backThread->join();
        backThread = NULL;
    }

    delete ui;

    logger.write("[MAIN] program dead (destructor)", true);
}

void MainWindow::write_last_pose()
{
    QString pose_path = QDir::homePath()+"/RB_MOBILE/config/robot_pose.ini";
    cv::Vec3d cur_pose = slam.get_cur_pose();
    float robot_pose_x = cur_pose[0];
    float robot_pose_y = cur_pose[1];
    float robot_pose_th = cur_pose[2];

    QSettings settings(pose_path, QSettings::IniFormat);
    settings.setValue("POSE/robot_last_pose_x", QString::number(robot_pose_x));
    settings.setValue("POSE/robot_last_pose_y", QString::number(robot_pose_y));
    settings.setValue("POSE/robot_last_pose_th", QString::number(robot_pose_th));
}

void MainWindow::bt_Exit()
{
    slam.stop();
    slam.stop_loc();

    logger.write("[MAIN] program dead (bt_Exit call)", true);
    QApplication::exit();
}

void MainWindow::bt_ReloadRobotConfig()
{
    update_robot_config();
}

void MainWindow::init()
{
    // check log folder
    QString log_path = QDir::homePath() + "/RB_MOBILE/log/sn_log";
    QDir log_dir(log_path);
    if(!log_dir.exists())
    {
        log_dir.mkdir(log_path);
    }

    // variable
    ui_motor_state = UI_MOTOR_NOT_READY;
    ui_loc_state = UI_LOC_NOT_READY;
    ui_auto_state = UI_AUTO_NOT_READY;
    ui_face_state = UI_FACE_NORMAL;
    ui_velocity_preset = PRESET_SPEED_NORMAL;
    ui_motor_lock_state = UI_MOTOR_LOCK_ON;
    ui_draw_state = UI_DRAW_STOP;
    ui_multi_state = UI_MULTI_READY;
    ui_fail_state = UI_AUTO_NO_FAILED;
    pre_vwt = cv::Vec3d(0, 0, 0);

    // auto init vars
    ui_loc_inlier_ratio = 0.0;
    ui_loc_inlier_error = 0.0;

    ui_mapping_inlier_ratio = 0.0;
    ui_mapping_inlier_error = 0.0;

    // mobile init
    mobile.init();

    // camera init
    //cam.init();

    // lidar init
    lidar.init(&mobile);

    // slam init
    slam.init(&cam, &lidar, &mobile, &unimap);

    // control init
    ctrl.init(&mobile, &slam, &unimap);

    // websocket init
    ws.init();

    // topomap
    topomap.init(&unimap);

    // communicate to integrate ui init
    integrate_ui.init(&mobile, &ctrl, &slam);

    // multicontrol init
    mctrl.init(&mobile, &slam, &unimap, &ws);

    // cmd server init
    cmd_server.init();

    // simulation init
#ifdef USE_SIM
    sim.init(&mobile, &lidar, &cam);
    logger.write("[MAIN] sim mode", true);
#endif

    // auto load map
    QString path = setting_config.map_path;
    if(!path.isEmpty())
    {
        QDir dir(path);
        if(dir.exists())
        {
            // new map load
            slam.stop();
            slam.stop_loc();
            unimap.load_map(path);
            if(unimap.is_loaded)
            {
                slam.set_map();

                unimap.load_locations();
                set_ui_items_from_unimap();

                ui->gv_Screen1->map_size = unimap.map_w;
                ui->gv_Screen1->grid_size = unimap.map_grid_width;
                ui->gv_Screen1->reload_gv_screen();

                // for map name check
                ui->lb_MapName->setText(path);

#ifdef USE_SIM
                sim.sim_grid_width = unimap.map_grid_width;
                QString str;
                str.sprintf("sim.sim_grid_width %f", unimap.map_grid_width);
                logger.write(str, true);
                sim.load_map(path);
#endif
            }
        }
    }

    // run watchdog
    watchdog_timer.start(10);

    // plot
    plot_timer.start(100);

    // background loop init
    if (backThread == NULL)
    {
        backFlag = true;
        backThread = new std::thread(&MainWindow::backLoop, this);
    }

//    // not used LOC_STATUS
//    IPC::LOC_STATUS loc;
//    memset(loc.serving, 1, 255);
//    ipc.set_loc_status(loc);
}

void MainWindow::make_sim_virtualObs()
{
#ifdef USE_SIM
    if(ui->ckb_SimObs->isChecked())
    {
        cv::Vec2d pt;
        pt[0] = ui->gv_Screen1->target_pose[0];
        pt[1] = ui->gv_Screen1->target_pose[1];

        sim.set_obs_pt(pt);
    }
#endif
}

void MainWindow::cb_ManualServingGroup()
{
    int group_idx = ui->cb_ManualServingGroup->currentIndex();
    if(group_idx < 0)
    {
        return;
    }

    ui->cb_ManualServing->clear();
    for(size_t p = 0; p < unimap.serving_names[group_idx].size(); p++)
    {
        ui->cb_ManualServing->addItem(unimap.serving_names[group_idx][p]);
    }
}

void MainWindow::set_ui_items_from_unimap()
{
    printf("[MAIN] set ui item\n");

    // clear
    ui->cb_ManualCharging->clear();
    ui->cb_ManualResting->clear();
    ui->cb_ManualServingGroup->clear();

    // add
    for(size_t p = 0; p < unimap.charging_names.size(); p++)
    {
        ui->cb_ManualCharging->addItem(unimap.charging_names[p]);
    }

    for(size_t p = 0; p < unimap.resting_names.size(); p++)
    {
        ui->cb_ManualResting->addItem(unimap.resting_names[p]);
    }

    for(size_t p = 0; p < unimap.serving_names.size(); p++)
    {
        ui->cb_ManualServingGroup->addItem(QString::number(p));
    }

    for(size_t p = 0; p < unimap.nodes.size(); p++)
    {
        ui->cb_ManualNodes->addItem(unimap.nodes[p].id);
    }
}

// mapping
void MainWindow::bt_MapRun()
{
    unimap.map_w = update_config.robot_map_size;
    unimap.map_h = update_config.robot_map_size;
    unimap.map_ou = update_config.robot_map_size/2;
    unimap.map_ov = update_config.robot_map_size/2;
    unimap.map_grid_width = update_config.robot_grid_size;

    ui->gv_Screen1->map_size = update_config.robot_map_size;
    ui->gv_Screen1->grid_size = update_config.robot_grid_size;
    ui->gv_Screen1->reload_gv_screen();

    slam.run();
}

void MainWindow::bt_MapStop()
{
    slam.stop();
}

void MainWindow::bt_MapSave()
{
    QString path = QFileDialog::getExistingDirectory(this, "Select dir", QDir::homePath() + "/RB_MOBILE/maps");
    if(!path.isNull())
    {
        unimap.save_map(path);
        slam.save(path);
    }
}

void MainWindow::bt_MapManualLc()
{
    slam.is_manual_lc = true;
}

// robot move
void MainWindow::bt_RobotMotorInit()
{
    mobile.motor_init();
}

void MainWindow::bt_RobotMoveForward_pressed()
{
    double alpha = ui->spb_JogGain->value();
    double v = update_config.robot_manual_limit_v * alpha;
    double w = 0;
    mobile.move(v, w);
    printf("[JOG] v:%f, w:%f(%f)\n", v, w*R2D, w);
}

void MainWindow::bt_RobotMoveBackward_pressed()
{    
    double alpha = ui->spb_JogGain->value();
    double v = -update_config.robot_manual_limit_v * alpha;
    double w = 0;
    mobile.move(v, w);
    printf("[JOG] v:%f, w:%f(%f)\n", v, w*R2D, w);
}

void MainWindow::bt_RobotMoveLeftTurn_pressed()
{
    double alpha = ui->spb_JogGain->value();
    double v = 0;
    double w = update_config.robot_manual_limit_w * alpha;
    mobile.move(v, w);
    printf("[JOG] v:%f, w:%f(%f)\n", v, w*R2D, w);
}

void MainWindow::bt_RobotMoveRightTurn_pressed()
{
    double alpha = ui->spb_JogGain->value();
    double v = 0;
    double w = -update_config.robot_manual_limit_w * alpha;
    mobile.move(v, w);
    printf("[JOG] v:%f, w:%f(%f)\n", v, w*R2D, w);
}

void MainWindow::bt_RobotMove_released()
{
    mobile.move(0, 0);
}

// localization
void MainWindow::bt_LocalizationLoad()
{
    // dataset load
    QString path = QFileDialog::getExistingDirectory(this, "Select dir", QDir::homePath() + "/RB_MOBILE/maps");
    if (!path.isNull())
    {
        // new map load
        slam.stop();
        slam.stop_loc();

        unimap.clear_map();
        unimap.load_map(path);
        if(unimap.is_loaded)
        {
            slam.set_map();

            unimap.load_locations();
            set_ui_items_from_unimap();

            ui->gv_Screen1->map_size = unimap.map_w;
            ui->gv_Screen1->grid_size = unimap.map_grid_width;
            ui->gv_Screen1->reload_gv_screen();
            ui->lb_MapName->setText(path);

#ifdef USE_SIM
            sim.sim_grid_width = unimap.map_grid_width;

            QString str;
            str.sprintf("sim.sim_grid_width %f", unimap.map_grid_width);
            logger.write(str, true);

            sim.load_map(path);
#endif
        }
    }
}

void MainWindow::bt_LocalizationInit()
{
    loc_manual_init(ui->gv_Screen1->target_pose);
    //    std::cout<<ui->gv_Screen1->target_pose<<std::endl;
}

bool MainWindow::check_approach_travel_line(cv::Mat &img, cv::Vec2i uv0, cv::Vec2i uv1)
{
    bool is_far = false;

    cv::Vec2i nn_st = cv::Vec2i(-1, -1);
    cv::Vec2i nn_ed = cv::Vec2i(-1, -1);
    double min_st = 99999999;
    double min_ed = 99999999;
    for(int i = 0; i < img.rows; i++)
    {
        for(int j = 0; j < img.cols; j++)
        {
            if(img.ptr<uchar>(i)[j] == 0)
            {
                continue;
            }

            cv::Vec2i pt;
            pt[0] = j;
            pt[1] = i;

            double d_st = cv::norm(pt - uv0);
            if(d_st < min_st)
            {
                min_st = d_st;
                nn_st = pt;
            }

            double d_ed = cv::norm(pt - uv1);
            if(d_ed < min_ed)
            {
                min_ed = d_ed;
                nn_ed = pt;
            }
        }
    }

    const double limit_dist = 20; // 1m
    if(min_st > limit_dist || min_ed > limit_dist)
    {
        is_far = true;
    }

    if(nn_st[0] != -1 || nn_st[1] != -1 || nn_ed[0] != -1 || nn_ed[1] != -1)
    {
        // draw shortest travel line
        cv::line(img, cv::Point(uv0[0], uv0[1]), cv::Point(nn_st[0], nn_st[1]), cv::Scalar(255), 1);
        cv::line(img, cv::Point(uv1[0], uv1[1]), cv::Point(nn_ed[0], nn_ed[1]), cv::Scalar(255), 1);
    }
    return is_far;
}



void MainWindow::bt_LocalizationInitAuto_Full()
{
    loc_auto_init_full();
}

void MainWindow::bt_LocalizationInitAuto_Semi()
{
    loc_auto_init_semi();
}

void MainWindow::bt_LocalizationInitAuto_Resting()
{
    loc_auto_init_resting();
}

void MainWindow::AutoInitFinished()
{
    slam.run_loc();
}

void MainWindow::loc_manual_init(cv::Vec3d pose)
{
    if(ui_loc_state == UI_LOC_BUSY)
    {
        logger.write("[LOC] busy", true);
        return;
    }

    logger.write("[LOC] try manual init", true);
    ui_loc_state = UI_LOC_MANUAL;

    // initial location finding
    slam.set_initial_location(pose);
    slam.run_loc();
}

void MainWindow::loc_auto_init_full()
{
    if(ui_loc_state == UI_LOC_BUSY)
    {
        logger.write("[LOC] busy", true);
        return;
    }

    logger.write("[LOC] try full-auto init", true);
    slam.stop_loc();
    ui_loc_state = UI_LOC_BUSY;

    // doing thread
    if(auto_init_thread_full != NULL)
    {
        logger.write("[LOC] auto init already running", true);
        auto_init_thread_full->join();
        auto_init_thread_full = NULL;
    }

    auto_init_thread_full = new std::thread(&SLAM_2D::set_auto_initial_location_full, &slam);
}

void MainWindow::loc_auto_init_semi()
{
    if(ui_loc_state == UI_LOC_BUSY)
    {
        logger.write("[LOC] busy", true);
        return;
    }

    logger.write("[LOC] try semi-auto init", true);
    slam.stop_loc();
    ui_loc_state = UI_LOC_BUSY;

    // doing thread
    if(auto_init_thread_semi != NULL)
    {
        logger.write("[LOC] auto init already running", true);
        auto_init_thread_semi->join();
        auto_init_thread_semi = NULL;
    }

    auto_init_thread_semi = new std::thread(&SLAM_2D::set_auto_initial_location_semi, &slam);
}

void MainWindow::loc_auto_init_resting()
{
    if(ui_loc_state == UI_LOC_BUSY)
    {
        logger.write("[LOC] busy", true);
        return;
    }

    logger.write("[LOC] try resting-auto init", true);
    slam.stop_loc();
    ui_loc_state = UI_LOC_BUSY;

    // doing thread
    if(auto_init_thread_resting != NULL)
    {
        logger.write("[LOC] auto init already running", true);
        auto_init_thread_resting->join();
        auto_init_thread_resting = NULL;
    }

    auto_init_thread_resting = new std::thread(&SLAM_2D::set_auto_initial_location_resting, &slam);
}

void MainWindow::bt_LocalizationRun()
{
    slam.run_loc();

#ifdef USE_SIM
    slam.loc_inlier_error = 0;
    slam.loc_inlier_ratio = 1.0;
    ui_loc_state = UI_LOC_GOOD;
#endif
}

void MainWindow::bt_LocalizationStop()
{
    slam.stop_loc();
}

// manual
void MainWindow::bt_ManualServingRun()
{
    if(unimap.serving_locs.size() == 0)
    {
        return;
    }

    int group_idx = ui->cb_ManualServingGroup->currentIndex();
    int idx = ui->cb_ManualServing->currentIndex();
    int preset_idx = ui->cb_preset->currentIndex() + 1;

    // robot state now moving
    cv::Vec3d pose = unimap.serving_locs[group_idx][idx];
    if(setting_config.robot_use_multi)
    {
        mctrl.run(pose, preset_idx);
    }
    else
    {
        ctrl.run(pose, preset_idx);
    }

    printf("%d-serving_%d, %f, %f, %f\n", group_idx, idx, pose[0], pose[1], pose[2]*R2D);
}

void MainWindow::bt_ManualRestingRun()
{
    if(unimap.resting_locs.size() == 0)
    {
        return;
    }

    int idx = ui->cb_ManualResting->currentIndex();
    int preset_idx = ui->cb_preset->currentIndex() + 1;

    // robot state now moving
    cv::Vec3d pose = unimap.resting_locs[idx];
    if(setting_config.robot_use_multi)
    {
        mctrl.run(pose, preset_idx);
    }
    else
    {
        ctrl.run(pose, preset_idx);
    }

    printf("resting_%d, %f, %f, %f\n", idx, pose[0], pose[1], pose[2]*R2D);
}

void MainWindow::bt_ManualChargingRun()
{
    if(unimap.charging_locs.size() == 0)
    {
        return;
    }

    int idx = ui->cb_ManualCharging->currentIndex();
    int preset_idx = ui->cb_preset->currentIndex() + 1;

    // robot state now moving
    cv::Vec3d pose = unimap.charging_locs[idx];
    if(setting_config.robot_use_multi)
    {
        mctrl.run(pose, preset_idx);
    }
    else
    {
        ctrl.run(pose, preset_idx);
    }

    printf("charging_%d, %f, %f, %f\n", idx, pose[0], pose[1], pose[2]*R2D);
}

void MainWindow::bt_ManualNodesRun()
{
    if(unimap.nodes.size() == 0)
    {
        return;
    }
    QString goal_node = ui->cb_ManualNodes->currentText();
    int preset_idx = ui->cb_preset->currentIndex() + 1;

    ctrl.run_ext2(goal_node, preset_idx);
    printf("run %s\n", goal_node.toStdString());
}

void MainWindow::bt_ManualClickedRun()
{
    int preset_idx = ui->cb_preset->currentIndex() + 1;
    if(setting_config.robot_use_multi)
    {
        mctrl.run(ui->gv_Screen1->target_pose, preset_idx);
    }
    else
    {
        ctrl.run(ui->gv_Screen1->target_pose, preset_idx);
    }
}

void MainWindow::bt_ManualStop()
{
    if(setting_config.robot_use_multi)
    {
        mctrl.stop();
    }
    else
    {
        ctrl.stop();
    }
}

void MainWindow::bt_ManualPause()
{
    if(setting_config.robot_use_multi)
    {
        if(mctrl.fsm_state != STATE_AUTO_GOAL_REACHED &&
                mctrl.fsm_state != STATE_AUTO_FAILED)
        {
            mctrl.fsm_state = STATE_AUTO_PAUSE;
        }
    }
    else
    {
        if(ctrl.fsm_state != STATE_AUTO_GOAL_REACHED &&
                ctrl.fsm_state != STATE_AUTO_FAILED)
        {
            ctrl.fsm_state = STATE_AUTO_PAUSE;
        }
    }
}

void MainWindow::bt_ManualResume()
{
    if(setting_config.robot_use_multi)
    {
        if(mctrl.fsm_state == STATE_AUTO_PAUSE)
        {
            mctrl.fsm_state = STATE_AUTO_PURE_PURSUIT;
        }
    }
    else
    {
        if(ctrl.fsm_state == STATE_AUTO_PAUSE)
        {
            ctrl.fsm_state = STATE_AUTO_PATH_FINDING;
        }
    }
}

void MainWindow::bt_ManualServingSet()
{
    if(slam.is_loc == false)
    {
        logger.write("[MAINWINDOW] loc not init", true);
        return;
    }

    cv::Vec3d cur_pose = slam.get_cur_pose();

    int num = -1;
    // load locations from annotation.ini
    QString annotated_loc_path = unimap.map_dir + "/annotation.ini";
    QFileInfo annotated_loc_info(annotated_loc_path);
    if(annotated_loc_info.exists() && annotated_loc_info.isFile())
    {
        QSettings settings(annotated_loc_path, QSettings::IniFormat);

        num = settings.value("serving_0/num").toInt();

        QString sec;
        sec.sprintf("serving_0/loc%d", num);

        QString str;
        str.sprintf("Serving%d,%f,%f,%f,", num, cur_pose[0], cur_pose[1], cur_pose[2]);

        num += 1;
        settings.setValue(sec, str);
        settings.setValue("serving_0/num", num);
    }

    printf("set serving_%d, %f, %f, %f\n", num, cur_pose[0], cur_pose[1], cur_pose[2]*R2D);
}

void MainWindow::bt_ManualRestingSet()
{
    if(slam.is_loc == false)
    {
        logger.write("[MAINWINDOW] loc not init", true);
        return;
    }

    cv::Vec3d cur_pose = slam.get_cur_pose();

    int num = -1;
    // load locations from annotation.ini
    QString annotated_loc_path = unimap.map_dir + "/annotation.ini";
    QFileInfo annotated_loc_info(annotated_loc_path);
    if(annotated_loc_info.exists() && annotated_loc_info.isFile())
    {
        QSettings settings(annotated_loc_path, QSettings::IniFormat);

        num = settings.value("resting_locations/num").toInt();

        QString sec;
        sec.sprintf("resting_locations/loc%d", num);

        QString str;
        str.sprintf("Resting%d,%f,%f,%f,", num, cur_pose[0], cur_pose[1], cur_pose[2]);

        num += 1;
        settings.setValue(sec, str);
        settings.setValue("resting_locations/num", num);
    }

    printf("set resting_%d, %f, %f, %f\n", num, cur_pose[0], cur_pose[1], cur_pose[2]*R2D);
}

void MainWindow::bt_ManualChargingSet()
{
    if(slam.is_loc == false)
    {
        logger.write("[MAINWINDOW] loc not init", true);
        return;
    }

    cv::Vec3d cur_pose = slam.get_cur_pose();

    int num = -1;
    // load locations from annotation.ini
    QString annotated_loc_path = unimap.map_dir + "/annotation.ini";
    QFileInfo annotated_loc_info(annotated_loc_path);
    if(annotated_loc_info.exists() && annotated_loc_info.isFile())
    {
        QSettings settings(annotated_loc_path, QSettings::IniFormat);

        num = settings.value("charging_locations/num").toInt();

        QString sec;
        sec.sprintf("charging_locations/loc%d", num);

        QString str;
        str.sprintf("Charging%d,%f,%f,%f,", num, cur_pose[0], cur_pose[1], cur_pose[2]);

        num += 1;
        settings.setValue(sec, str);
        settings.setValue("charging_locations/num", num);
    }

    printf("set charging_%d, %f, %f, %f\n", num, cur_pose[0], cur_pose[1], cur_pose[2]*R2D);
}

void MainWindow::ckb_SetLocation(int val)
{
    if(ui->ckb_SetLocation->isChecked())
    {
        ui->bt_ManualServingSet->setDisabled(false);
        ui->bt_ManualRestingSet->setDisabled(false);
        ui->bt_ManualChargingSet->setDisabled(false);
    }
    else
    {
        ui->bt_ManualServingSet->setDisabled(true);
        ui->bt_ManualRestingSet->setDisabled(true);
        ui->bt_ManualChargingSet->setDisabled(true);
    }
}

// temporal
void MainWindow::bt_SimInit()
{
#ifdef USE_SIM
    if(unimap.resting_locs.size() == 0)
    {
        return;
    }

    int idx = ui->cb_ManualResting->currentIndex();

    // robot jump resting
    cv::Vec3d pose = unimap.resting_locs[idx];
    sim.set_sim_pose(pose);
    slam.cur_pose = pose;
    slam.cur_scan_pose = pose;
    loc_init(pose);

    // emo release
    MOBILE_STATUS status;
    status.is_ok = true;

    status.emo_state = 1;
    status.charge_state = 0;
    status.power_state = 1;

    status.connection_m0 = 1;
    status.connection_m1 = 1;

    status.status_m0 = 1;
    status.status_m1 = 1;

    mobile.status_received(status);

#endif
}

void MainWindow::bt_Test()
{

}

void MainWindow::bt_Test2()
{

}

void MainWindow::bt_Test3()
{

}

void MainWindow::bt_Test4()
{

}

void MainWindow::bt_CheckAllPath()
{
    if(unimap.is_loaded == false)
    {
        logger.write("[PATH_CHECK] unimap is not loaded", true);
        return;
    }

    logger.write("[PATH_CHECK] path check start", true);

    std::vector<cv::Vec3d> resting_locs = unimap.get_resting_locations();
    std::vector<cv::Vec3d> charging_locs = unimap.get_charging_locations();
    std::vector<cv::Vec3d> cleaning_locs = unimap.get_cleaning_locations();

    std::vector<std::vector<cv::Vec3d>> serving_locs = unimap.get_serving_locations();
    std::vector<std::vector<QString>> serving_names = unimap.get_serving_names();
    std::vector<QString> serving_group_names = unimap.get_serving_group_names();

    IPC::CHECK_TRAVEL msg;
    int idx = 0;
    for(size_t p = 0; p < resting_locs.size(); p++)
    {
        for(size_t q = 0; q < serving_locs.size(); q++)
        {
            bool is_broken = false;
            bool is_far = false;
            for(size_t r = 0; r < serving_locs[q].size(); r++)
            {
                if(idx >= 5)
                {
                    break;
                }

                cv::Mat travel_map = unimap.get_map_travel();

                cv::Vec2d pos0(resting_locs[p][0], resting_locs[p][1]);
                cv::Vec2d pos1(serving_locs[q][r][0], serving_locs[q][r][1]);

                cv::Vec2i uv0 = unimap.xy_uv(pos0);
                cv::Vec2i uv1 = unimap.xy_uv(pos1);

                is_far = check_approach_travel_line(travel_map, uv0, uv1);

                std::vector<cv::Vec2i> path = ctrl.path_finding(~travel_map, uv0, uv1);
                if(path.size() == 0)
                {
                    is_broken = true;
                }

                if(is_far == true || is_broken == true)
                {
                    if(idx < 5)
                    {
                        uint8_t _is_broken = uint8_t(is_broken);
                        uint8_t _is_far = uint8_t(is_far);
                        QString _group = serving_group_names[q];
                        QString _name = serving_names[q][r];

                        memcpy(&msg.is_broken[idx], &_is_broken, sizeof(uint8_t));
                        memcpy(&msg.is_far[idx],&_is_far, sizeof(uint8_t));
                        memcpy(&msg.group[idx], _group.toUtf8().data(), 255);
                        memcpy(&msg.name[idx], _name.toUtf8().data(), 255);
                    }
                    idx++;
                }
            }
        }
    }

    for(size_t p = 0; p < resting_locs.size(); p++)
    {
        for(size_t q = 0; q < charging_locs.size(); q++)
        {
            if(idx >= 5)
            {
                break;
            }

            bool is_broken = false;
            bool is_far = false;

            cv::Mat travel_map = unimap.get_map_travel();

            cv::Vec2d pos0(resting_locs[p][0], resting_locs[p][1]);
            cv::Vec2d pos1(charging_locs[q][0], charging_locs[q][1]);

            cv::Vec2i uv0 = unimap.xy_uv(pos0);
            cv::Vec2i uv1 = unimap.xy_uv(pos1);

            is_far = check_approach_travel_line(travel_map, uv0, uv1);

            std::vector<cv::Vec2i> path = ctrl.path_finding(~travel_map, uv0, uv1);
            if(path.size() == 0)
            {
                is_broken = true;
            }

            if(is_far == true || is_broken == true)
            {
                if(idx < 5)
                {
                    uint8_t _is_broken = uint8_t(is_broken);
                    uint8_t _is_far = uint8_t(is_far);
                    QString _group = "charging_locations";
                    QString _name = "";

                    memcpy(&msg.is_broken[idx], &_is_broken, sizeof(uint8_t));
                    memcpy(&msg.is_far[idx],&_is_far, sizeof(uint8_t));
                    memcpy(&msg.group[idx], _group.toUtf8().data(), 255);
                    memcpy(&msg.name[idx], _name.toUtf8().data(), 255);
                }
                idx++;
            }
        }
    }

    for(size_t p = 0; p < resting_locs.size(); p++)
    {
        for(size_t q = 0; q < cleaning_locs.size(); q++)
        {
            if(idx >= 5)
            {
                break;
            }

            bool is_broken = false;
            bool is_far = false;

            cv::Mat travel_map = unimap.get_map_travel();

            cv::Vec2d pos0(resting_locs[p][0], resting_locs[p][1]);
            cv::Vec2d pos1(cleaning_locs[q][0], cleaning_locs[q][1]);

            cv::Vec2i uv0 = unimap.xy_uv(pos0);
            cv::Vec2i uv1 = unimap.xy_uv(pos1);

            is_far = check_approach_travel_line(travel_map, uv0, uv1);

            std::vector<cv::Vec2i> path = ctrl.path_finding(~travel_map, uv0, uv1);
            if(path.size() == 0)
            {
                is_broken = true;
            }

            if(is_far == true || is_broken == true)
            {
                if(idx < 5)
                {
                    uint8_t _is_broken = uint8_t(is_broken);
                    uint8_t _is_far = uint8_t(is_far);
                    QString _group = "cleaning_locations";
                    QString _name = "";

                    memcpy(&msg.is_broken[idx], &_is_broken, sizeof(uint8_t));
                    memcpy(&msg.is_far[idx],&_is_far, sizeof(uint8_t));
                    memcpy(&msg.group[idx], _group.toUtf8().data(), 255);
                    memcpy(&msg.name[idx], _name.toUtf8().data(), 255);
                }
                idx++;
            }
        }
    }

    if(idx == 0)
    {
        logger.write("[PATH_CHECK] all path success", true);
    }
    else
    {
        QString str;
        str.sprintf("[PATH_CHECK] some path: %d wrong", idx);
        logger.write(str, true);
    }

    msg.num = idx;
    ipc.set_check_travel(msg);
}

void MainWindow::bt_SendMaps()
{
    QString ip = ui->le_IP2Send->text();
    QString id = ui->le_ID2Send->text();
    QString pw = ui->le_PW2Send->text();

    if(ip == "" || id == "" || pw == "")
    {
        ip = setting_config.fms_ip;
        id = setting_config.fms_id;
        pw = setting_config.fms_pw;

        if(ip == "" || id == "" || pw == "")
        {
            logger.write("[SERVER] no exist infos", true);
            return;
        }
    }

    QString send_path = QDir::homePath() + "/RB_MOBILE/maps";
//    QString send_path = QDir::homePath() + "/maps";
    QString destination_path = "/home/" + id + "/RB_MOBILE";
//    QString destination_path = "/home/" + id;

    QString cmd = "sshpass -p " + pw + " rsync -avz -e 'ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null'" + send_path + " " + id + "@" + ip + ":" + destination_path;
    int result = system(cmd.toStdString().c_str());
    std::cout<<"file transfer result : "<<result<<std::endl;
    if(result != 0)
    {
        logger.write("[SERVER] rsync command failed.", true);
    }
    else
    {
        std::cout<<"file transfer done!"<<std::endl;
        logger.write("[SERVER] successfully transferred using rsync..", true);
    }
}

// travel line drawing interface
void MainWindow::bt_DrawStart()
{
    slam.is_travel = 1;
}

void MainWindow::bt_DrawStop()
{
    slam.is_travel = 0;
    unimap.save_travel_map();
}

void MainWindow::bt_SimEmoPush()
{
    // emo pushed
#ifdef USE_SIM
    MOBILE_STATUS status;
    status.is_ok = true;

    status.emo_state = 0;
    status.charge_state = 0;
    status.power_state = 1;

    status.connection_m0 = 1;
    status.connection_m1 = 1;

    status.status_m0 = 0;
    status.status_m1 = 0;

    mobile.status_received(status);
#endif
}

void MainWindow::bt_SimEmoRelease()
{
    // emo released
#ifdef USE_SIM
    MOBILE_STATUS status;
    status.is_ok = true;

    status.emo_state = 1;
    status.charge_state = 0;
    status.power_state = 1;

    status.connection_m0 = 1;
    status.connection_m1 = 1;

    status.status_m0 = 1;
    status.status_m1 = 1;

    mobile.status_received(status);
#endif
}

void MainWindow::bt_RobotSetLed()
{
    int target = ui->spb_LedTarget->value();
    int mode = ui->spb_LedMode->value();
    mobile.led(target, mode);
}

void MainWindow::bt_SoftReload()
{
    QString path = QFileDialog::getExistingDirectory(this, "Select dir", QDir::homePath() + "/RB_MOBILE/maps");
    if (!path.isNull())
    {
        // soft map-reload
        unimap.clear_map_soft();
        unimap.load_map_soft(path);
        if(unimap.is_loaded)
        {
            unimap.load_locations();
            set_ui_items_from_unimap();

            logger.write("[CMD] soft map re-loaded", true);
        }
    }
}

void MainWindow::bt_HardReload()
{
    bt_LocalizationLoad();
    logger.write("[CMD] hard map re-loaded", true);
}

void MainWindow::process_command(CMD cur_cmd)
{
    int cmd = cur_cmd.cmd;
    uint8_t* params = cur_cmd.params;

    QString str;
    str.sprintf("[CMD] cmd:%d, accepted", cmd);
    logger.write(str, true);

    if(cmd == CMD_MOVE_TARGET0)
    {
        if(unimap.is_loaded == false)
        {
            logger.write("[CMD] ID:01, unimap is not loaded", true);
            return;
        }

        // motor init
        mobile.init();

        float x; memcpy(&x, &params[0], 4);
        float y; memcpy(&y, &params[4], 4);
        float th; memcpy(&th, &params[8], 4);
        uint8_t preset_idx; memcpy(&preset_idx, &params[12], 1);

        cv::Vec3d goal = cv::Vec3d(x, y, th);
        if(setting_config.robot_use_multi)
        {
            mctrl.run(goal, preset_idx);
        }
        else
        {
            ctrl.run(goal, preset_idx);
        }

        QString str;
        str.sprintf("[CMD] ID:01, move target0, %f, %f, %f, %d", x, y, th*R2D, preset_idx);
        logger.write(str, true);
    }
    else if(cmd == CMD_MOVE_TARGET)
    {
        if(unimap.is_loaded == false)
        {
            logger.write("[CMD] ID:02, unimap is not loaded", true);
            return;
        }

        float x; memcpy(&x, &params[0], 4);
        float y; memcpy(&y, &params[4], 4);
        float th; memcpy(&th, &params[8], 4);
        uint8_t preset_idx; memcpy(&preset_idx, &params[12], 1);

        if(get_time() - last_accepted_times[CMD_MOVE_TARGET] > 1.0)
        {
            // motor init
            mobile.init();

            cv::Vec3d goal = cv::Vec3d(x, y, th);
            if(setting_config.robot_use_multi)
            {
                mctrl.run(goal, preset_idx);
            }
            else
            {
                ctrl.run(goal, preset_idx);
            }

            QString str;
            str.sprintf("[CMD] ID:02, move target, %f, %f, %f, %d", x, y, th*R2D, preset_idx);
            logger.write(str, true);

            last_accepted_times[CMD_MOVE_TARGET] = get_time();
        }
    }
    else if(cmd == CMD_MOVE_JOYSTICK)
    {
        logger.write("[CMD] ID:03, move joystick", true);
    }
    else if(cmd == CMD_MOVE_MANUAL)
    {
        logger.write("[CMD] ID:04, move manual", true);
    }
    else if(cmd == CMD_MOVE_STOP)
    {
        bt_ManualStop();
        logger.write("[CMD] ID:05, stop", true);
    }
    else if(cmd == CMD_PAUSE)
    {
        bt_ManualPause();
        logger.write("[CMD] ID:06, pause", true);
    }
    else if(cmd == CMD_RESUME)
    {
        bt_ManualResume();
        logger.write("[CMD] ID:07, resume", true);
    }
    else if(cmd == CMD_SET_VEL)
    {
        logger.write("[CMD] ID:08, set velocity", true);
    }
    else if(cmd == CMD_RESTART)
    {
        logger.write("[CMD] ID:09, restart (kill slamnav)", true);
        bt_Exit();
    }
    else if(cmd == CMD_SET_INIT)
    {
        float x; memcpy(&x, &params[0], 4);
        float y; memcpy(&y, &params[4], 4);
        float th; memcpy(&th, &params[8], 4);

        loc_manual_init(cv::Vec3d(x,y,th));

        logger.write("[CMD] ID:10, set init", true);
    }
    else if(cmd == CMD_LOC_RUN)
    {
        slam.run_loc();

        logger.write("[CMD] ID:11, loc run", true);
    }
    else if(cmd == CMD_LOC_STOP)
    {
        slam.stop_loc();

        logger.write("[CMD] ID:12, loc stop", true);
    }
    else if(cmd == CMD_LOC_AUTO)
    {
        loc_auto_init_semi();

        logger.write("[CMD] ID:13, semi auto init", true);
    }
    else if(cmd == CMD_LOC_AUTO_FULL)
    {
        loc_auto_init_full();

        logger.write("[CMD] ID:14, full auto init", true);
    }
    else if(cmd == CMD_MAPPING_START)
    {
        if(get_time() - last_accepted_times[CMD_MAPPING_START] > 1.0)
        {
            int map_size; memcpy(&map_size, &params[0], 4);
            float grid_size; memcpy(&grid_size, &params[4], 4);

            update_config.robot_map_size = map_size;
            update_config.robot_grid_size = (double)grid_size;

            unimap.map_w = update_config.robot_map_size;
            unimap.map_h = update_config.robot_map_size;
            unimap.map_ou = update_config.robot_map_size/2;
            unimap.map_ov = update_config.robot_map_size/2;
            unimap.map_grid_width = update_config.robot_grid_size;

            ui->gv_Screen1->map_size = update_config.robot_map_size;
            ui->gv_Screen1->grid_size = update_config.robot_grid_size;
            ui->gv_Screen1->reload_gv_screen();

            slam.run();

            logger.write("[CMD] ID:15, start mapping", true);
            last_accepted_times[CMD_MAPPING_START] = get_time();
        }
    }
    else if(cmd == CMD_MAPPING_STOP)
    {
        // new map load
        slam.stop();
        slam.stop_loc();

        QString path = unimap.map_dir;
        unimap.load_map(path);
        if(unimap.is_loaded)
        {
            slam.set_map();

            unimap.load_locations();
            set_ui_items_from_unimap();

            ui->gv_Screen1->map_size = unimap.map_w;
            ui->gv_Screen1->grid_size = unimap.map_grid_width;
            ui->gv_Screen1->reload_gv_screen();
            ui->lb_MapName->setText(path);
        }

        logger.write("[CMD] ID:16, stop mapping", true);
    }
    else if(cmd == CMD_REQ_CAMERA)
    {
        logger.write("[CMD] ID:17, req camera", true);
    }
    else if(cmd == CMD_MAP_SAVE)
    {
        slam.stop();

        QByteArray buf = (char*)params;
        QString str(buf);
        QString path = QDir::homePath() + "/RB_MOBILE/maps/" + str;
        QDir dir;
        dir.mkpath(path);

        unimap.save_map(path);
        slam.save(path);

        // write robot last pose
        write_last_pose();

        logger.write("[CMD] ID:18, map save", true);
    }
    else if(cmd == CMD_OBS_START)
    {
        logger.write("[CMD] ID:19, obs start", true);
    }
    else if(cmd == CMD_OBS_STOP)
    {
        logger.write("[CMD] ID:20, obs stop", true);
    }
    else if(cmd == CMD_OBS_SAVE)
    {
        logger.write("[CMD] ID:21, obs save", true);
    }
    else if(cmd == CMD_NONE)
    {
    }
    else if(cmd == CMD_MOTOR_LOCK_ON)
    {
        double dt = get_time() - last_accepted_times[CMD_MOTOR_LOCK_ON];
        printf("[CMD] ID:23 dt:%f\n", dt);
        if(dt > 1.0)
        {
            mobile.motor_init();

            logger.write("[CMD] ID:23, motor lock on", true);
            last_accepted_times[CMD_MOTOR_LOCK_ON] = get_time();
            is_received_motor_lock_off = false;
        }
    }
    else if(cmd == CMD_DRAW_START)
    {
        bt_DrawStart();
        ui_draw_state = UI_DRAW_START;

        logger.write("[CMD] ID:24, draw start", true);
    }
    else if(cmd == CMD_DRAW_STOP_SAVE)
    {
        bt_DrawStop();
        ui_draw_state = UI_DRAW_STOP;

        logger.write("[CMD] ID:25, draw stop", true);
    }
    else if(cmd == CMD_ROBOT_MAP_HARD_RELOAD)
    {
        double dt = get_time() - last_accepted_times[CMD_ROBOT_MAP_HARD_RELOAD];
        printf("[CMD] ID:26 dt:%f\n", dt);
        if(dt > 1.0)
        {
            QByteArray buf = (char*)params;
            QString str(buf);
            QString path = QDir::homePath() + "/RB_MOBILE/maps/" + str;

            // new map load
            slam.stop();
            slam.stop_loc();

            unimap.clear_map();
            unimap.load_map(path);
            if(unimap.is_loaded)
            {
                slam.set_map();

                unimap.load_locations();
                set_ui_items_from_unimap();

                ui->gv_Screen1->map_size = unimap.map_w;
                ui->gv_Screen1->grid_size = unimap.map_grid_width;
                ui->gv_Screen1->reload_gv_screen();
                ui->lb_MapName->setText(path);

                logger.write("[CMD] ID:26, hard map re-loaded", true);
                last_accepted_times[CMD_ROBOT_MAP_HARD_RELOAD] = get_time();
            }
        }
    }
    else if(cmd == CMD_ROBOT_CONFIG_RELOAD)
    {
        update_robot_config();
        logger.write("[CMD] ID:27, robot_config re-loaded", true);
    }
    else if(cmd == CMD_FMS_SEND_MAP)
    {
        update_robot_config();
        bt_SendMaps();

        logger.write("[CMD] ID:28, fms send map", true);
    }
    else if(cmd == CMD_CHECK_TRAVEL)
    {
        bt_CheckAllPath();

        logger.write("[CMD] ID:29, check all path", true);
    }
    else if(cmd == CMD_ROBOT_MAP_SOFT_RELOAD)
    {
        QByteArray buf = (char*)params;
        QString str(buf);
        QString path = QDir::homePath() + "/RB_MOBILE/maps/" + str;

        // soft map-reload
        unimap.clear_map_soft();
        unimap.load_map_soft(path);
        if(unimap.is_loaded)
        {
            unimap.load_locations();
            set_ui_items_from_unimap();

            logger.write("[CMD] ID:30, soft map re-loaded", true);
        }
    }
    else if(cmd == CMD_MANUAL_LC)
    {
        slam.is_manual_lc = true;

        logger.write("[CMD] ID:31, try manual lc", true);
    }
    else if(cmd == CMD_LOC_AUTO_RESTING)
    {
        loc_auto_init_resting();

        logger.write("[CMD] ID:32, resting auto init", true);
    }
    else if(cmd == CMD_MOTOR_LOCK_OFF)
    {
        double dt = get_time() - last_accepted_times[CMD_MOTOR_LOCK_OFF];
        printf("[CMD] ID:33 dt:%f\n", dt);
        if(dt > 1.0)
        {
            mobile.motor_lock_off();

            logger.write("[CMD] ID:33, motor lock off", true);
            last_accepted_times[CMD_MOTOR_LOCK_OFF] = get_time();
            is_received_motor_lock_off = true;
        }
    }
    last_accepted_cmd = cmd;
    last_accepted_time = get_time();
}

void MainWindow::publish_ipc_status()
{
    cv::Vec3d cur_pose = slam.get_cur_pose();
    std::vector<cv::Vec2d> cur_scan = slam.get_cur_scan();
    if(cur_scan.size() == 0)
    {
        slam.set_cur_scans();
        cur_scan = slam.get_cur_scan();
    }

    std::vector<float> cur_scan_sampled(360, 0);
    if(cur_scan.size() > 0)
    {
        for(size_t p = 0; p < cur_scan.size(); p++)
        {
            double x = cur_scan[p][0];
            double y = cur_scan[p][1];
            double d = std::sqrt(x*x + y*y);
            double th = std::atan2(y, x);
            double deg = th*R2D;
            if(deg < 0)
            {
                deg += 360;
            }

            int idx = std::round(deg);
            if(cur_scan_sampled[idx] == 0)
            {
                cur_scan_sampled[idx] = d;
            }
            else
            {
                if(d < cur_scan_sampled[idx])
                {
                    cur_scan_sampled[idx] = d;
                }
            }
        }
    }

    MOBILE_STATUS status = mobile.get_status();

    IPC::STATUS msg;
    msg.connection_m0 = status.connection_m0;
    msg.connection_m1 = status.connection_m1;

    msg.status_m0 = status.status_m0;
    msg.status_m1 = status.status_m1;

    msg.temp_m0 = status.temp_m0;
    msg.temp_m1 = status.temp_m1;

    msg.cur_m0 = status.cur_m0;
    msg.cur_m1 = status.cur_m1;

    msg.status_charge = status.charge_state;
    msg.status_power = status.power_state;
    msg.status_emo = status.emo_state;
    msg.status_remote = status.remote_state;

    msg.bat_in = status.bat_in;
    msg.bat_out = status.bat_out;
    msg.bat_cur = status.bat_current;

    msg.power = status.power;
    msg.total_power = status.total_power;

    msg.ui_loc_state = ui_loc_state;
    msg.ui_auto_state = ui_auto_state;
    msg.ui_obs_state = 0;
    msg.ui_face_state = ui_face_state;
    msg.ui_cur_velocity_preset = ui_velocity_preset;
    msg.ui_motor_lock_state = ui_motor_lock_state;
    msg.ui_draw_state = ui_draw_state;
    msg.ui_multi_state = ui_multi_state;
    msg.ui_fail_state = ui_fail_state;

    msg.ui_loc_inlier_ratio = ui_loc_inlier_ratio;
    msg.ui_loc_inlier_error = ui_loc_inlier_error;

    msg.ui_mapping_inlier_ratio = ui_mapping_inlier_ratio;
    msg.ui_mapping_inlier_error = ui_mapping_inlier_error;

    msg.robot_pose[0] = cur_pose[0];
    msg.robot_pose[1] = cur_pose[1];
    msg.robot_pose[2] = cur_pose[2];

    memcpy(msg.robot_scan, cur_scan_sampled.data(), sizeof(float)*360);

    ipc.set_status(msg);
}

void MainWindow::publish_ipc_path()
{
    std::vector<PATH_POINT> path;
    if(setting_config.robot_use_multi)
    {
        path = mctrl.get_cur_path();
    }
    else
    {
        path = ctrl.get_cur_path();
    }

    if(path.size() == 0)
    {
        return;
    }

    // path sampling, 0.5m
    std::vector<PATH_POINT> sampled_path;
    sampled_path.push_back(path[0]);
    for(size_t p = 1; p < path.size()-1; p+=50)
    {
        sampled_path.push_back(path[p]);
    }
    sampled_path.push_back(path.back());

    IPC::PATH msg;
    msg.num = sampled_path.size();
    for(int p = 0; p < msg.num; p++)
    {
        msg.x[p] = sampled_path[p].pt[0];
        msg.y[p] = sampled_path[p].pt[1];
    }

    ipc.set_path(msg);
}

void MainWindow::publish_ipc_map()
{
    cv::Mat _map = unimap.get_map_raw();

    cv::Mat map;
    cv::resize(_map, map, cv::Size(1000,1000), 0, 0, cv::INTER_NEAREST);

    IPC::MAP msg;
    memcpy(msg.buf, map.data, map.rows*map.cols);

    ipc.set_map(msg);
}

void MainWindow::publish_ipc_cam()
{
    cv::Mat _cam0 = cam.get_cur_img_l();
    cv::Mat _cam1 = cam.get_cur_img_r();
    if(_cam0.empty() || _cam1.empty())
    {
        return;
    }

    cv::Mat cam0;
    cv::Mat cam1;

    cv::resize(_cam0, cam0, cv::Size(480, 270));
    cv::resize(_cam1, cam1, cv::Size(480, 270));

    QString sn0 = cam.get_sn_l();
    QString sn1 = cam.get_sn_r();

    IPC::IMG msg0;
    memcpy(msg0.serial, sn0.toUtf8().data(), 255);
    memcpy(msg0.buf, cam0.data, 480*270);
    ipc.set_cam0(msg0);

    IPC::IMG msg1;
    memcpy(msg1.serial, sn1.toUtf8().data(), 255);
    memcpy(msg1.buf, cam1.data, 480*270);
    ipc.set_cam1(msg1);
}

void MainWindow::publish_ipc_cam_color()
{
    cv::Mat _cam0 = cam.get_cur_img_l_color();
    cv::Mat _cam1 = cam.get_cur_img_r_color();
    if(_cam0.empty() || _cam1.empty())
    {
        return;
    }

    cv::Mat cam0;
    cv::resize(_cam0, cam0, cv::Size(480, 270));

    cv::Mat cam1;
    cv::resize(_cam1, cam1, cv::Size(480, 270));

    QString sn0 = cam.get_sn_l();
    QString sn1 = cam.get_sn_r();

    IPC::IMG_COLOR msg0;
    memcpy(msg0.serial, sn0.toUtf8().data(), 255);
    memcpy(msg0.buf, cam0.data, 480*270*3);
    ipc.set_cam_color0(msg0);

    IPC::IMG_COLOR msg1;
    memcpy(msg1.serial, sn1.toUtf8().data(), 255);
    memcpy(msg1.buf, cam1.data, 480*270*3);
    ipc.set_cam_color1(msg1);
}

void MainWindow::plot_loop()
{
    // cmd info
    if(last_accepted_cmd == CMD_MOVE_TARGET0)
    {
        ui->lb_CmdInfo->setText("CMD_MOVE_TARGET0");
    }
    else if(last_accepted_cmd == CMD_MOVE_TARGET)
    {
        ui->lb_CmdInfo->setText("CMD_MOVE_TARGET");
    }
    else if(last_accepted_cmd == CMD_MOVE_JOYSTICK)
    {
        ui->lb_CmdInfo->setText("CMD_MOVE_JOYSTICK");
    }
    else if(last_accepted_cmd == CMD_MOVE_MANUAL)
    {
        ui->lb_CmdInfo->setText("CMD_MOVE_MANUAL");
    }
    else if(last_accepted_cmd == CMD_MOVE_STOP)
    {
        ui->lb_CmdInfo->setText("CMD_MOVE_STOP");
    }
    else if(last_accepted_cmd == CMD_PAUSE)
    {
        ui->lb_CmdInfo->setText("CMD_PAUSE");
    }
    else if(last_accepted_cmd == CMD_RESUME)
    {
        ui->lb_CmdInfo->setText("CMD_RESUME");
    }
    else if(last_accepted_cmd == CMD_SET_VEL)
    {
        ui->lb_CmdInfo->setText("CMD_SET_VEL");
    }
    else if(last_accepted_cmd == CMD_RESTART)
    {
        ui->lb_CmdInfo->setText("CMD_RESTART");
    }
    else if(last_accepted_cmd == CMD_SET_INIT)
    {
        ui->lb_CmdInfo->setText("CMD_SET_INIT");
    }
    else if(last_accepted_cmd == CMD_LOC_RUN)
    {
        ui->lb_CmdInfo->setText("CMD_LOC_RUN");
    }
    else if(last_accepted_cmd == CMD_LOC_STOP)
    {
        ui->lb_CmdInfo->setText("CMD_LOC_STOP");
    }
    else if(last_accepted_cmd == CMD_LOC_AUTO)
    {
        ui->lb_CmdInfo->setText("CMD_LOC_AUTO");
    }
    else if(last_accepted_cmd == CMD_MAPPING_START)
    {
        ui->lb_CmdInfo->setText("CMD_MAPPING_START");
    }
    else if(last_accepted_cmd == CMD_MAPPING_STOP)
    {
        ui->lb_CmdInfo->setText("CMD_MAPPING_STOP");
    }
    else if(last_accepted_cmd == CMD_REQ_CAMERA)
    {
        ui->lb_CmdInfo->setText("CMD_REQ_CAMERA");
    }
    else if(last_accepted_cmd == CMD_MAP_SAVE)
    {
        ui->lb_CmdInfo->setText("CMD_MAP_SAVE");
    }
    else if(last_accepted_cmd == CMD_OBS_START)
    {
        ui->lb_CmdInfo->setText("CMD_OBS_START");
    }
    else if(last_accepted_cmd == CMD_OBS_STOP)
    {
        ui->lb_CmdInfo->setText("CMD_OBS_STOP");
    }
    else if(last_accepted_cmd == CMD_OBS_SAVE)
    {
        ui->lb_CmdInfo->setText("CMD_OBS_SAVE");
    }
    else if(last_accepted_cmd == CMD_MOTOR_LOCK_OFF)
    {
        ui->lb_CmdInfo->setText("CMD_MOTOR_LOCK_OFF");
    }
    else if(last_accepted_cmd == CMD_MOTOR_LOCK_ON)
    {
        ui->lb_CmdInfo->setText("CMD_MOTOR_LOCK_ON");
    }
    else if(last_accepted_cmd == CMD_DRAW_START)
    {
        ui->lb_CmdInfo->setText("CMD_DRAW_START");
    }
    else if(last_accepted_cmd == CMD_DRAW_STOP_SAVE)
    {
        ui->lb_CmdInfo->setText("CMD_DRAW_STOP_SAVE");
    }
    else if(last_accepted_cmd == CMD_ROBOT_MAP_HARD_RELOAD)
    {
        ui->lb_CmdInfo->setText("CMD_ROBOT_MAP_RELOAD");
    }
    else if(last_accepted_cmd == CMD_ROBOT_CONFIG_RELOAD)
    {
        ui->lb_CmdInfo->setText("CMD_ROBOT_CONFIG_RELOAD");
    }
    else if(last_accepted_cmd == CMD_FMS_SEND_MAP)
    {
        ui->lb_CmdInfo->setText("CMD_FMS_SEND_MAP");
    }
    else if(last_accepted_cmd == CMD_CHECK_TRAVEL)
    {
        ui->lb_CmdInfo->setText("CMD_CHECK_TRAVEL");
    }

    // autocontrol fsm state
    if(setting_config.robot_use_multi)
    {
        if(mctrl.fsm_state == STATE_AUTO_PATH_FINDING)
        {
            ui->lb_AutoFsmState->setText("STATE_AUTO_PATH_FINDING");
        }
        else if(mctrl.fsm_state == STATE_AUTO_FIRST_ALIGN)
        {
            ui->lb_AutoFsmState->setText("STATE_AUTO_FIRST_ALIGN");
        }
        else if(mctrl.fsm_state == STATE_AUTO_PURE_PURSUIT)
        {
            ui->lb_AutoFsmState->setText("STATE_AUTO_PURE_PURSUIT");
        }
        else if(mctrl.fsm_state == STATE_AUTO_FINAL_ALIGN)
        {
            ui->lb_AutoFsmState->setText("STATE_AUTO_FINAL_ALIGN");
        }
        else if(mctrl.fsm_state == STATE_AUTO_GOAL_REACHED)
        {
            ui->lb_AutoFsmState->setText("STATE_AUTO_GOAL_REACHED");


        }
        else if(mctrl.fsm_state == STATE_AUTO_OBSTACLE)
        {
            ui->lb_AutoFsmState->setText("STATE_AUTO_OBSTACLE");
        }
        else if(mctrl.fsm_state == STATE_AUTO_PAUSE)
        {
            ui->lb_AutoFsmState->setText("STATE_AUTO_PAUSE");
        }
        else if(mctrl.fsm_state == STATE_AUTO_FAILED)
        {
            ui->lb_AutoFsmState->setText("STATE_AUTO_FAILED");
        }
    }
    else
    {
        if(ctrl.fsm_state == STATE_AUTO_PATH_FINDING)
        {
            fsm_state=STATE_AUTO_PATH_FINDING;
            ui->lb_AutoFsmState->setText("STATE_AUTO_PATH_FINDING");
        }
        else if(ctrl.fsm_state == STATE_AUTO_FIRST_ALIGN)
        {
            fsm_state=STATE_AUTO_FIRST_ALIGN;
            ui->lb_AutoFsmState->setText("STATE_AUTO_FIRST_ALIGN");
        }
        else if(ctrl.fsm_state == STATE_AUTO_PURE_PURSUIT)
        {
            fsm_state=STATE_AUTO_PURE_PURSUIT;
            ui->lb_AutoFsmState->setText("STATE_AUTO_PURE_PURSUIT");
        }
        else if(ctrl.fsm_state == STATE_AUTO_FINAL_ALIGN)
        {
            fsm_state=STATE_AUTO_FINAL_ALIGN;
            ui->lb_AutoFsmState->setText("STATE_AUTO_FINAL_ALIGN");
        }
        else if(ctrl.fsm_state == STATE_AUTO_GOAL_REACHED)
        {
            fsm_state=STATE_AUTO_GOAL_REACHED;
            ui->lb_AutoFsmState->setText("STATE_AUTO_GOAL_REACHED");
        }
        else if(ctrl.fsm_state == STATE_AUTO_OBSTACLE)
        {
            fsm_state=STATE_AUTO_OBSTACLE;
            ui->lb_AutoFsmState->setText("STATE_AUTO_OBSTACLE");
        }
        else if(ctrl.fsm_state == STATE_AUTO_PAUSE)
        {
            fsm_state=STATE_AUTO_PAUSE;
            ui->lb_AutoFsmState->setText("STATE_AUTO_PAUSE");
        }
        else if(ctrl.fsm_state == STATE_AUTO_FAILED)
        {
            fsm_state=STATE_AUTO_FAILED;
            ui->lb_AutoFsmState->setText("STATE_AUTO_FAILED");
        }

        integrate_ui.mobile_moving_status = ctrl.fsm_state;
    }

    // plot state
    if(ui_motor_state == UI_MOTOR_NOT_READY)
    {
        ui->lb_MotorState->setText("UI_MOTOR_NOT_READY");
        //        integrate_status=UI_MOTOR_NOT_READY;
    }
    else if(ui_motor_state == UI_MOTOR_READY)
    {
        ui->lb_MotorState->setText("UI_MOTOR_READY");
        //        integrate_status=UI_MOTOR_READY;
    }
    if(ui_loc_state == UI_LOC_NOT_READY)
    {
        ui->lb_LocState->setText("UI_LOC_NOT_READY");
        integrate_status=UI_LOC_NOT_READY;
    }
    else if(ui_loc_state == UI_LOC_BUSY)
    {
        ui->lb_LocState->setText("UI_LOC_BUSY");
        integrate_status=UI_LOC_FAIL;
    }
    else if(ui_loc_state == UI_LOC_GOOD)
    {
        ui->lb_LocState->setText("UI_LOC_GOOD");
        integrate_status=UI_LOC_GOOD;
    }
    else if(ui_loc_state == UI_LOC_FAIL)
    {
        ui->lb_LocState->setText("UI_LOC_FAIL");
        integrate_status=UI_LOC_FAIL;
    }
    if(ui_auto_state == UI_AUTO_NOT_READY)
    {
        ui->lb_AutoState->setText("UI_AUTO_NOT_READY");
        //        integrate_status=UI_AUTO_NOT_READY;
    }
    else if(ui_auto_state == UI_AUTO_READY)
    {
        ui->lb_AutoState->setText("UI_AUTO_READY");
        //        integrate_status=UI_AUTO_READY;
    }
    else if(ui_auto_state == UI_AUTO_MOVING)
    {
        ui->lb_AutoState->setText("UI_AUTO_MOVING");
        //        integrate_status=UI_AUTO_MOVING;
    }
    else if(ui_auto_state == UI_AUTO_WAIT)
    {
        ui->lb_AutoState->setText("UI_AUTO_WAIT");
        //        integrate_status=UI_AUTO_WAIT;
    }
    else if(ui_auto_state == UI_AUTO_PAUSE)
    {
        ui->lb_AutoState->setText("UI_AUTO_PAUSE");
        //        integrate_status=UI_AUTO_PAUSE;
    }

    if(ui_face_state == UI_FACE_NORMAL)
    {
        ui->lb_FaceState->setText("UI_FACE_NORMAL");
    }
    else if(ui_face_state == UI_FACE_SURPRISE)
    {
        ui->lb_FaceState->setText("UI_FACE_SURPRISE");
    }
    else if(ui_face_state == UI_FACE_CRYING)
    {
        ui->lb_FaceState->setText("UI_FACE_CRYING");
    }

    if(ui_velocity_preset == PRESET_SPEED_NORMAL)
    {
        ui->lb_PresetState->setText("PRESET_SPEED_NORMAL");
    }
    else if(ui_velocity_preset == PRESET_SPEED_SLOWEST)
    {
        ui->lb_PresetState->setText("PRESET_SPEED_SLOWEST");
    }
    else if(ui_velocity_preset == PRESET_SPEED_SLOW)
    {
        ui->lb_PresetState->setText("PRESET_SPEED_SLOW");
    }
    else if(ui_velocity_preset == PRESET_SPEED_FAST)
    {
        ui->lb_PresetState->setText("PRESET_SPEED_FAST");
    }
    else if(ui_velocity_preset == PRESET_SPEED_FASTEST)
    {
        ui->lb_PresetState->setText("PRESET_SPEED_FASTEST");
    }

    if(ui_motor_lock_state == UI_MOTOR_LOCK_OFF)
    {
        ui->lb_MotorLockState->setText("UI_MOTOR_LOCK_OFF");
    }
    else if(ui_motor_lock_state == UI_MOTOR_LOCK_ON)
    {
        ui->lb_MotorLockState->setText("UI_MOTOR_LOCK_ON");
    }

    if(ui_draw_state == UI_DRAW_STOP)
    {
        ui->lb_DrawState->setText("UI_DRAW_STOP");
    }
    else if(ui_draw_state == UI_DRAW_START)
    {
        ui->lb_DrawState->setText("UI_DRAW_START");
    }

    if(ui_fail_state == UI_AUTO_NO_FAILED)
    {
        ui->lb_FailState->setText("UI_AUTO_NO_FAILED");
    }
    else if(ui_fail_state == UI_AUTO_FAILED)
    {
        ui->lb_FailState->setText("UI_AUTO_FAILED");
    }

#ifdef USE_SINGLE_S1
    // plot que info
    QString que_info_str;
    que_info_str.sprintf("raw_q:%d, scan_q:%d, l2c_q:%d\npose_q:%d, status_q:%d", (int)lidar.raw_que.unsafe_size(), (int)lidar.scan_que.unsafe_size(), (int)mobile.l2c.msg_que.unsafe_size()
                         , (int)mobile.l2c.pose_que.unsafe_size(), (int)mobile.l2c.status_que.unsafe_size());
    ui->lb_QueInfo->setText(que_info_str);
#endif

#ifndef USE_SINGLE_S1
    // plot que info
    QString que_info_str;
    que_info_str.sprintf("raw_q0:%d, raw_q1:%d, scan_q:%d, l2c_q:%d\npose_q:%d, status_q:%d", (int)lidar.raw_que.unsafe_size(), (int)lidar.raw_que1.unsafe_size(), (int)lidar.scan_que.unsafe_size(), (int)mobile.l2c.msg_que.unsafe_size()
                         , (int)mobile.l2c.pose_que.unsafe_size(), (int)mobile.l2c.status_que.unsafe_size());
    ui->lb_QueInfo->setText(que_info_str);
#endif

    // plot localization info
    QString loc_info_str;
    loc_info_str.sprintf("(loc) Inlier:%.3f, Err:%.3f", (double)slam.loc_inlier_ratio, (double)slam.loc_inlier_error);
    ui->lb_LocInfo->setText(loc_info_str);

    // plot mapping info
    QString mapping_info_str;
    mapping_info_str.sprintf("(map) Inlier:%.3f, Err:%.3f", (double)slam.mapping_inlier_ratio, (double)slam.mapping_inlier_error);
    ui->lb_MappingInfo->setText(mapping_info_str);

    // plot mobile pose
    MOBILE_POSE mobile_pose = mobile.get_pose();

    double mobile_dt = mobile_pose.t - pre_vwt[2];
    double mobile_v_acc = (mobile_pose.vw[0]-pre_vwt[0])/mobile_dt;
    double mobile_w_acc = (mobile_pose.vw[1]-pre_vwt[1])/mobile_dt;
    pre_vwt = cv::Vec3d(mobile_pose.vw[0], mobile_pose.vw[1], mobile_pose.t);

    QString mobile_pose_str;
    mobile_pose_str.sprintf("t: %f\npos: %.2f, %.2f, %.2f\nvel: %.2f, %.2f, %.2f\nvw: %.2f, %.2f\nacc: %.2f, %.2f", mobile_pose.t,
                            mobile_pose.pose[0], mobile_pose.pose[1], mobile_pose.pose[2]*R2D,
            mobile_pose.vel[0], mobile_pose.vel[1], mobile_pose.vel[2]*R2D,
            mobile_pose.vw[0], mobile_pose.vw[1]*R2D,
            mobile_v_acc, mobile_w_acc*R2D);
    ui->te_Pose->setText(mobile_pose_str);

    // plot mobile status
    MOBILE_STATUS mobile_status = mobile.get_status();
    QString mobile_status_str;
    mobile_status_str.sprintf("connection(m0, m1): %d, %d\nstatus(m0, m1): %d, %d\ntemperature(m0, m1): %d, %d\ncur(m0, m1):%.2f, %.2f\ncharge, power, emo, remote state: %d, %d, %d, %d\nBAT(in, out, cur):%.3f, %.3f, %.3f\npower: %.3f\ntotal power: %.3f",
                              mobile_status.connection_m0, mobile_status.connection_m1, mobile_status.status_m0, mobile_status.status_m1, mobile_status.temp_m0, mobile_status.temp_m1,
                              (double)mobile_status.cur_m0/10.0, (double)mobile_status.cur_m1/10.0,
                              mobile_status.charge_state, mobile_status.power_state, mobile_status.emo_state, mobile_status.remote_state,
                              mobile_status.bat_in, mobile_status.bat_out, mobile_status.bat_current,
                              mobile_status.power, mobile_status.total_power);
    ui->lb_RobotStatus->setText(mobile_status_str);

    // plot robot id
    QString fms_info_str;
    fms_info_str.sprintf("id:%s, tick:%d", ws.id.toLocal8Bit().data(), (int)ws.fms_tick);
    ui->lb_FmsInfo->setText(fms_info_str);

    // plot pose
    cv::Vec3d cur_pose = slam.get_cur_pose();
    QString pose_str;
    pose_str.sprintf("LOC: %.2f, %.2f, %.1f", cur_pose[0], cur_pose[1], cur_pose[2]*R2D);
    ui->lb_Pose->setText(pose_str);

    cv::Vec3d target_pose = ui->gv_Screen1->target_pose;
    QString target_str;
    target_str.sprintf("TARGET LOC: %.2f, %.2f, %.1f", target_pose[0], target_pose[1], target_pose[2]*R2D);
    ui->lb_Target->setText(target_str);

    QJsonObject json_output;
    json_output["MSG_TYPE"] = "MOBILE_POSE";
    UI_LOC_STATE status(integrate_status);
    AUTO_FSM_STATE fsm_status(fsm_state);

    json_output["STATUS"] = status;
    json_output["Pose_x"] = cur_pose[0];
    json_output["Pose_y"] = cur_pose[1];
    json_output["Pose_th"] = cur_pose[2]*R2D;
    json_output["battery"] = mobile_status.bat_in;
    json_output["charge_state"] = mobile_status.charge_state;
    json_output["FSM STATUS"] = fsm_status;

    QByteArray json_string = QJsonDocument(json_output).toJson(QJsonDocument::Compact);
    integrate_ui.mtx.lock();
    integrate_ui.send_mobile_status.push(json_string);
    integrate_ui.mtx.unlock();

//    /    integrate_ui.onMobileStatusSocketWrite(json_string);
//    integrate_ui.newk_AMR_status = json_string;

    // image plot
    cv::Mat plot_img;
    if(plot_que.try_pop(plot_img))
    {
        ui->gv_Screen1->map.setPixmap(QPixmap::fromImage(mat_to_qimage_cpy(plot_img)));
        plot_que.clear();
    }

    // cam plot
    if(ui->ckb_PlotEnable->isChecked())
    {
        cv::Mat img_l = cam.get_cur_img_l_color();
        if(!img_l.empty())
        {
            ui->lb_ScreenL->setPixmap(QPixmap::fromImage(mat_to_qimage_cpy(img_l)));
        }

        cv::Mat img_r = cam.get_cur_img_r_color();
        if(!img_r.empty())
        {
            ui->lb_ScreenR->setPixmap(QPixmap::fromImage(mat_to_qimage_cpy(img_r)));
        }
    }
}

void MainWindow::watchdog_loop()
{
    const double init_loc_dist = 1.0;

    if(setting_config.robot_use_multi)
    {
        // check status
        MOBILE_STATUS status = mobile.get_status();
        if(status.is_ok && pre_status.is_ok)
        {
            // motor
            if(status.status_m0 == 1 && status.status_m1 == 1 || is_received_motor_lock_off == true)
            {
                ui_motor_state = UI_MOTOR_READY;
            }
            else
            {
                ui_motor_state = UI_MOTOR_NOT_READY;
            }

            // emo release
            if(pre_status.emo_state > 0 && status.emo_state == 0)
            {
                logger.write("[WATCHDOG] EMO pushed", true);
            }
            else if(pre_status.emo_state == 0 && status.emo_state > 0)
            {
                mobile.motor_init();
                logger.write("[WATCHDOG] EMO released", true);
            }
        }
        pre_status = status;

        // ui icp params(ratio, error) update
        ui_loc_inlier_ratio = (double)slam.loc_inlier_ratio;
        ui_loc_inlier_error = (double)slam.loc_inlier_error;

        ui_mapping_inlier_ratio = (double)slam.mapping_inlier_ratio;
        ui_mapping_inlier_error = (double)slam.mapping_inlier_error;

        // check init localization
        if(slam.is_loc == false)
        {
            if(ui_loc_state != UI_LOC_BUSY && ui_loc_state != UI_LOC_MANUAL)
            {
                ui_loc_state = UI_LOC_NOT_READY;
            }
        }
        else
        {
            if(ui_loc_state == UI_LOC_BUSY)
            {
                // check cur_pose val too far
                bool is_close = false;
                cv::Vec3d cur_pose = slam.cur_pose;
                std::vector<cv::Vec3d> _locs = slam.locs;
                for(size_t p=0; p<_locs.size(); p++)
                {
                    cv::Vec2d dtdr = slam.dTdR(cur_pose, _locs[p]);
                    if(dtdr[0] < init_loc_dist)
                    {
                        is_close = true;
                        break;
                    }
                }

                if(is_close == false)
                {
                    mctrl.is_loc = false;
                    ui_loc_state = UI_LOC_FAIL;
                    loc_fail_cnt = 0;

                    QString str;
                    str.sprintf("[WATCHDOG] LOC init successed, ir:%f, ie:%f, but cur_pose is too far away", (double)slam.loc_inlier_ratio, (double)slam.loc_inlier_error);
                    logger.write(str, true);
                    return;
                }

                if(slam.loc_inlier_ratio > setting_config.robot_icp_ratio0 && slam.loc_inlier_error < setting_config.robot_icp_error0)
                {
                    mctrl.is_loc = true;
                    ui_loc_state = UI_LOC_GOOD;
                    loc_fail_cnt = 0;
                }
                else
                {
                    if(loc_fail_cnt >= 500 && ui_loc_state != UI_LOC_FAIL)
                    {
                        mctrl.is_loc = false;
                        ui_loc_state = UI_LOC_FAIL;
                        loc_fail_cnt = 0;

                        QString str;
                        str.sprintf("[WATCHDOG] LOC init failed, ir:%f, ie:%f", (double)slam.loc_inlier_ratio, (double)slam.loc_inlier_error);
                        logger.write(str, true);
                    }
                    loc_fail_cnt++;
                }
            }
            else
            {
                if(ui_loc_state == UI_LOC_GOOD || ui_loc_state == UI_LOC_MANUAL)
                {
                    if(slam.loc_inlier_ratio > update_config.robot_icp_ratio && slam.loc_inlier_error < update_config.robot_icp_error)
                    {
                        mctrl.is_loc = true;
                        ui_loc_state = UI_LOC_GOOD;

                        loc_fail_cnt = 0;
                    }
                    else
                    {
                        if(loc_fail_cnt >= 300 && ui_loc_state != UI_LOC_FAIL)
                        {
                            mctrl.is_loc = false;
                            ui_loc_state = UI_LOC_FAIL;

                            QString str;
                            str.sprintf("[WATCHDOG] LOC failed, ir:%f, ie:%f", (double)slam.loc_inlier_ratio, (double)slam.loc_inlier_error);
                            logger.write(str, true);

                            loc_fail_cnt = 0;
                        }
                        loc_fail_cnt++;
                    }
                }
            }
        }

        // set velocity preset idx
        ui_velocity_preset = (int)mctrl.preset_idx;

        // set face state
        ui_face_state = (int)mctrl.cur_face_state;

        // set fail state
        if(mctrl.fsm_state == STATE_AUTO_FAILED)
        {
            ui_fail_state = UI_AUTO_FAILED;
        }
        else if(mctrl.fsm_state == STATE_AUTO_GOAL_REACHED)
        {
            ui_fail_state = UI_AUTO_NO_FAILED;
        }

        // check running
        bool is_charging = (status.is_ok == true && status.charge_state == 1);
        if(ui_motor_state == UI_MOTOR_READY && ui_loc_state == UI_LOC_GOOD && is_charging == false)
        {
            if(mctrl.fsm_state == STATE_AUTO_GOAL_REACHED)
            {
                ui_auto_state = UI_AUTO_READY;
            }
            else if(mctrl.fsm_state == STATE_AUTO_OBSTACLE)
            {
                ui_auto_state = UI_AUTO_WAIT;
            }
            else if(mctrl.fsm_state == STATE_AUTO_PAUSE)
            {
                ui_auto_state = UI_AUTO_PAUSE;
            }
            else if(mctrl.fsm_state == STATE_AUTO_PATH_FINDING ||
                    mctrl.fsm_state == STATE_AUTO_FIRST_ALIGN  ||
                    mctrl.fsm_state == STATE_AUTO_PURE_PURSUIT ||
                    mctrl.fsm_state == STATE_AUTO_FINAL_ALIGN )
            {
                ui_auto_state = UI_AUTO_MOVING;
            }
        }
        else
        {
            ui_auto_state = UI_AUTO_NOT_READY;
        }
    }
    else
    {
        // check status
        MOBILE_STATUS status = mobile.get_status();
//        qDebug()<<"status.is_ok : "<<status.is_ok;
//        qDebug()<<"pre_status.is_ok :"<<pre_status.is_ok;
        if(status.is_ok && pre_status.is_ok)
        {
            // motor
            if(status.status_m0 == 1 && status.status_m1 == 1 || is_received_motor_lock_off == true)
            {
                ui_motor_state = UI_MOTOR_READY;
            }
            else
            {
                ui_motor_state = UI_MOTOR_NOT_READY;
            }

            // emo release
            if(pre_status.emo_state > 0 && status.emo_state == 0)
            {
                logger.write("[WATCHDOG] EMO pushed", true);
            }
            else if(pre_status.emo_state == 0 && status.emo_state > 0)
            {
                mobile.motor_init();
                logger.write("[WATCHDOG] EMO released", true);
            }
        }
        pre_status = status;

        // ui icp params(ratio, error) update
        ui_loc_inlier_ratio = (double)slam.loc_inlier_ratio;
        ui_loc_inlier_error = (double)slam.loc_inlier_error;

        ui_mapping_inlier_ratio = (double)slam.mapping_inlier_ratio;
        ui_mapping_inlier_error = (double)slam.mapping_inlier_error;

        // check init localization
        if(slam.is_loc == false)
        {
            if(ui_loc_state != UI_LOC_BUSY && ui_loc_state != UI_LOC_MANUAL)
            {
                ui_loc_state = UI_LOC_NOT_READY;
            }
        }
        else
        {
            if(ui_loc_state == UI_LOC_BUSY)
            {
                // check cur_pose val too far
                bool is_close = false;

                cv::Vec3d cur_pose = slam.cur_pose;
                std::vector<cv::Vec3d> _locs = slam.locs;
                for(size_t p=0; p<_locs.size(); p++)
                {
                    cv::Vec2d dtdr = slam.dTdR(cur_pose, _locs[p]);
                    if(dtdr[0] < init_loc_dist)
                    {
                        is_close = true;
                        break;
                    }
                }

                if(is_close == false)
                {
                    ctrl.is_loc = false;
                    ui_loc_state = UI_LOC_FAIL;
                    loc_fail_cnt = 0;

                    QString str;
                    str.sprintf("[WATCHDOG] LOC init successed, ir:%f, ie:%f, but cur_pose is too far away", (double)slam.loc_inlier_ratio, (double)slam.loc_inlier_error);
                    logger.write(str, true);
                    return;
                }

                if(slam.loc_inlier_ratio > setting_config.robot_icp_ratio0 && slam.loc_inlier_error < setting_config.robot_icp_error0)
                {
                    ctrl.is_loc = true;
                    ui_loc_state = UI_LOC_GOOD;
                    loc_fail_cnt = 0;
                }
                else
                {
                    if(loc_fail_cnt >= 500 && ui_loc_state != UI_LOC_FAIL)
                    {
                        ctrl.is_loc = false;
                        ui_loc_state = UI_LOC_FAIL;
                        loc_fail_cnt = 0;

                        QString str;
                        str.sprintf("[WATCHDOG] LOC init failed, ir:%f, ie:%f", (double)slam.loc_inlier_ratio, (double)slam.loc_inlier_error);
                        logger.write(str, true);
                    }
                    loc_fail_cnt++;
                }
            }
            else
            {
                if(ui_loc_state == UI_LOC_GOOD || ui_loc_state == UI_LOC_MANUAL)
                {
                    if(slam.loc_inlier_ratio > update_config.robot_icp_ratio && slam.loc_inlier_error < update_config.robot_icp_error)
                    {
                        ctrl.is_loc = true;
                        ui_loc_state = UI_LOC_GOOD;

                        loc_fail_cnt = 0;
                    }
                    else
                    {
                        if(loc_fail_cnt >= 300 && ui_loc_state != UI_LOC_FAIL)
                        {
                            ctrl.is_loc = false;
                            ui_loc_state = UI_LOC_FAIL;

                            QString str;
                            str.sprintf("[WATCHDOG] LOC failed, ir:%f, ie:%f", (double)slam.loc_inlier_ratio, (double)slam.loc_inlier_error);
                            logger.write(str, true);

                            loc_fail_cnt = 0;
                        }
                        loc_fail_cnt++;
                    }
                }
            }
        }

        // set velocity preset idx
        ui_velocity_preset = (int)ctrl.preset_idx;

        // set face state
        ui_face_state = (int)ctrl.cur_face_state;

        // set fail state
        if(ctrl.fsm_state == STATE_AUTO_FAILED)
        {
            ui_fail_state = UI_AUTO_FAILED;
        }
        else if(ctrl.fsm_state == STATE_AUTO_GOAL_REACHED)
        {
            ui_fail_state = UI_AUTO_NO_FAILED;
        }

        // check running
        bool is_charging = (status.is_ok == true && status.charge_state == 1);
        if(ui_motor_state == UI_MOTOR_READY && ui_loc_state == UI_LOC_GOOD && is_charging == false)
        {
            if(ctrl.fsm_state == STATE_AUTO_GOAL_REACHED)
            {
                ui_auto_state = UI_AUTO_READY;
            }
            else if(ctrl.fsm_state == STATE_AUTO_OBSTACLE)
            {
                ui_auto_state = UI_AUTO_WAIT;
            }
            else if(ctrl.fsm_state == STATE_AUTO_PAUSE)
            {
                ui_auto_state = UI_AUTO_PAUSE;
            }
            else if(ctrl.fsm_state == STATE_AUTO_PATH_FINDING ||
                    ctrl.fsm_state == STATE_AUTO_FIRST_ALIGN  ||
                    ctrl.fsm_state == STATE_AUTO_PURE_PURSUIT ||
                    ctrl.fsm_state == STATE_AUTO_FINAL_ALIGN )
            {
                ui_auto_state = UI_AUTO_MOVING;
            }
        }
        else
        {
            ui_auto_state = UI_AUTO_NOT_READY;
        }
//        qDebug()<<"ui_auto_state : "<<ui_auto_state;
    }

    integrate_ui.mobile_moving_status = (int)ui_auto_state;
}

void MainWindow::backLoop()
{
    double dt = 0.1;
    double pre_loop_time = get_time();
    double last_pub_time = get_time();

    while(backFlag)
    {
        double cur_time = get_time();

        // publish status for ui
        publish_ipc_status();

        if(cur_time - last_pub_time >= 1.0)
        {
            // publish path for ui
            publish_ipc_path();

            // publish map for ui
            if(slam.is_slam)
            {
                publish_ipc_map();
            }

            // publish cam for ui
            publish_ipc_cam_color();

            // write last pose
            write_last_pose();

            last_pub_time = cur_time;
        }

        // make plot image
        if(ui->ckb_PlotEnable->isChecked())
        {
            // plot map info
            cv::Mat plot_img = unimap.get_map_plot();
            if(!plot_img.empty())
            {
                cv::Vec3d cur_pose(0,0,0);
                cv::Vec2d cur_vw(0,0);
                std::vector<cv::Vec2d> cur_scan;
                std::vector<cv::Vec2d> cam_scan_l;
                std::vector<cv::Vec2d> cam_scan_r;

                if(slam.is_slam)
                {
                    // has location info
                    cur_pose = slam.get_cur_pose();
                    cur_scan = slam.get_cur_scan_global();
                    cur_vw = mobile.get_pose().vw;

#ifdef USE_SIM
                    cur_pose = mobile.get_pose().pose;
#endif
                }
                else if(slam.is_loc)
                {
                    // has location info
                    cur_pose = slam.get_cur_pose();
                    cur_scan = slam.get_cur_scan_global();
                    cam_scan_l = cam.get_cur_scan_l();
                    cam_scan_r = cam.get_cur_scan_r();
                    cur_vw = mobile.get_pose().vw;
                }
                else
                {
                    // no location info
                    cur_scan = lidar.get_cur_scan();
                    cam_scan_l = cam.get_cur_scan_l();
                    cam_scan_r = cam.get_cur_scan_r();
                    cur_vw = mobile.get_pose().vw;
                }

                // map data loaded
                if(unimap.is_loaded)
                {
                    // draw locations
                    for(size_t p = 0; p < unimap.charging_locs.size(); p++)
                    {
                        unimap.draw_robot(plot_img, unimap.charging_locs[p], cv::Scalar(0,0,128), 1);
                    }

                    for(size_t p = 0; p < unimap.resting_locs.size(); p++)
                    {
                        unimap.draw_robot(plot_img, unimap.resting_locs[p], cv::Scalar(255,255,255), 1);
                    }

                    for(size_t p = 0; p < unimap.cleaning_locs.size(); p++)
                    {
                        unimap.draw_robot(plot_img, unimap.cleaning_locs[p], cv::Scalar(0,128,0), 1);
                    }

                    for(size_t p = 0; p < unimap.serving_locs.size(); p++)
                    {
                        for(size_t q = 0; q < unimap.serving_locs[p].size(); q++)
                        {
                            unimap.draw_robot(plot_img, unimap.serving_locs[p][q], cv::Scalar(0,255,0), 1);
                        }
                    }
                }

                // get path
                std::vector<PATH_POINT> cur_path;
                if(setting_config.robot_use_multi)
                {
                    cur_path = mctrl.get_cur_path();
                }
                else
                {
                    cur_path = ctrl.get_cur_path();
                }

                // draw current sensing data
                unimap.draw_travel_map(plot_img);
                unimap.draw_lidar(plot_img, cur_pose, cam_scan_l, cv::Scalar(255,0,0));
                unimap.draw_lidar(plot_img, cur_pose, cam_scan_r, cv::Scalar(255,0,0));
                unimap.draw_lidar(plot_img, cv::Vec3d(0,0,0), cur_scan, cv::Scalar(0,0,255));
                unimap.draw_obs_map(plot_img);
                unimap.draw_path(plot_img, cur_path, cv::Scalar(0,255,0));
                unimap.draw_robot(plot_img, ui->gv_Screen1->target_pose, cv::Scalar(255, 255, 0), 1);
                unimap.draw_robot(plot_img, cur_pose, cv::Scalar(0,255,255), 1);

                // draw feedback trajectory
                std::vector<cv::Vec6d> traj_feed = ctrl.calc_trajectory(cur_vw, 2.0, 0.2, cur_pose, cur_vw);
                unimap.draw_trajectory(plot_img, traj_feed, cv::Scalar(255,255,255), 1);

                // draw input trajectory
                cv::Vec2d input_vw(mobile.last_v, mobile.last_w);
                std::vector<cv::Vec6d> traj_input = ctrl.calc_trajectory(input_vw, 2.0, 0.2, cur_pose, input_vw);
                unimap.draw_trajectory(plot_img, traj_input, cv::Scalar(0,0,255), 1);

                // preview trajectory
                cv::Vec2d prev_vw(mobile.last_v, 0);
                std::vector<cv::Vec6d> traj_prev = ctrl.calc_trajectory(prev_vw, setting_config.robot_obs_preview_time, 0.3, cur_pose, prev_vw);
                unimap.draw_trajectory(plot_img, traj_prev, cv::Scalar(255,0,255), 1);

                // plot image
                cv::flip(plot_img, plot_img, 0);
                cv::rotate(plot_img, plot_img, cv::ROTATE_90_COUNTERCLOCKWISE); // image north is +x axis

                // draw table name
                int r = static_config.robot_radius/unimap.map_grid_width;
                if(ui->ckb_TableName->isChecked())
                {
                    for(size_t p = 0; p < unimap.serving_names.size(); p++)
                    {
                        for(size_t q = 0; q < unimap.serving_names[p].size(); q++)
                        {
                            QString name = unimap.serving_names[p][q];
                            cv::Vec3d loc = unimap.serving_locs[p][q];
                            cv::Vec2i uv = unimap.xy_uv2(cv::Vec2d(loc[0], loc[1]));
                            cv::putText(plot_img, name.toStdString(), cv::Point(uv[0]+r, uv[1]-r), cv::FONT_HERSHEY_SIMPLEX, 0.3, CV_RGB(0, 255, 0), 1, cv::LineTypes::LINE_AA);
                        }
                    }
                }

                // robot vel plot
                cv::Vec2i cur_uv = unimap.xy_uv2(cv::Vec2d(cur_pose[0], cur_pose[1]));
                QString cur_v_str;
                cur_v_str.sprintf("%.2f, %.1f", cur_vw[0], cur_vw[1]*R2D);
                cv::putText(plot_img, cur_v_str.toStdString(), cv::Point(cur_uv[0]-r, cur_uv[1]-2*r), cv::FONT_HERSHEY_SIMPLEX, 0.3, CV_RGB(0, 255, 255), 1, cv::LineTypes::LINE_AA);

                plot_que.push(plot_img);

                cv::Mat map;
                cv::resize(plot_img,map,cv::Size(1000,1000),0,0,cv::INTER_NEAREST);
                QByteArray IMGByte((char*)(map.data),1000*1000*3);//숫자확
                // latter mutex add
                mtx.lock();
                IMGByte2 = IMGByte;
                mtx.unlock();
            }
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
            printf("[PLOT] loop time drift, dt:%f\n", delta_loop_time);
        }
        pre_loop_time = get_time();
    }
}

void MainWindow::bt_TestMoveExt()
{
    // waypoints
    std::vector<cv::Vec3d> waypoints;
    waypoints.push_back(cv::Vec3d(2, 1, -90*D2R));
    waypoints.push_back(cv::Vec3d(-0.58, 1.2, -90*D2R));
    waypoints.push_back(cv::Vec3d(-0.40, 1.11, 0*D2R));
    //waypoints.push_back(cv::Vec3d(10.7, 0, 90*D2R));
    ctrl.run_ext(waypoints);
}

void MainWindow::bt_TestMovePick()
{
    int preset_idx = ui->cb_preset->currentIndex() + 1;
    ctrl.run_pick(ui->gv_Screen1->target_pose, preset_idx);
}

void MainWindow::websocket_map_changed(QString map_name)
{
    // dataset load
    QString path = "/home/rainbow/RB_MOBILE/maps/" + map_name;
    if (!path.isNull())
    {
        // new map load
        slam.stop();
        slam.stop_loc();
        unimap.load_map(path);
        if(unimap.is_loaded)
        {
            slam.set_map();
            unimap.load_locations();
            set_ui_items_from_unimap();

            ui->gv_Screen1->map_size = unimap.map_w;
            ui->gv_Screen1->grid_size = unimap.map_grid_width;
            ui->gv_Screen1->reload_gv_screen();
            ui->lb_MapName->setText(path);

            QString config_path = "/home/rainbow/RB_MOBILE/config/setting_config.ini";
            QFileInfo config_info(config_path);
            if(config_info.exists() && config_info.isFile())
            {
                // set valueis_normal
                QSettings settings(config_path, QSettings::IniFormat);
                settings.setValue("MAP/map_name", map_name);
                settings.setValue("MAP/map_path", path);
            }

            float x=-0.65;
            float y=-0.07;
            float th=-2.4*D2R;

            yujin_pose = (cv::Vec3d(x,y,th));
            loc_manual_init(yujin_pose);

            QJsonObject json_output;

            json_output["MSG_TYPE"] = "MOBILE_MAP_CHANGE";
            QByteArray json_string = QJsonDocument(json_output).toJson(QJsonDocument::Compact);

            integrate_ui.mtx.lock();
            integrate_ui.send_mobile_status.push(json_string);
            integrate_ui.mtx.unlock();

        }
    }
}

void MainWindow::IntegrateUILoop()
{
    mtx.lock();
    QByteArray header = "map";
    header.append(IMGByte2);
    mtx.unlock();
    integrate_ui.onMapImageSocketWrite(header);

}
