#ifndef MAINWINDOW_H
#define MAINWINDOW_H

// defines
#include "global_defines.h"

// Qt
#include <QMainWindow>
#include <QTimer>
#include <QFileDialog>
#include <cstdlib>

// my
#include "ipc.h"
#include "unimap.h"
#include "topomap.h"
#include "cam.h"
#include "lidar_2d.h"
#include "mobile.h"
#include "slam_2d.h"
#include "autocontrol.h"
#include "multicontrol.h"
#include "ws_client.h"
#include "cmd_server.h"
#include "integrate_ui.h"
#include "sim.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

private:
    Ui::MainWindow *ui;

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    // ipc
    IPC ipc;

    // camera
    CAM cam;

    // lidar
    LIDAR_2D lidar;

    // platforms
    MOBILE mobile;

    // unified map
    UNIMAP unimap;

    // topology map
    TOPOMAP topomap;

    // alogorithms
    SLAM_2D slam;

    // control
    AUTOCONTROL ctrl;

    // multi control
    MULTICONTROL mctrl;

    // websocket
    WS_CLIENT ws;

    // CMD server
    CMD_SERVER cmd_server;

    // integrate ui (orderPicking)
    INTEGRATE_UI integrate_ui;

    // simulation
    SIM sim;

    // for auto init
    std::thread *auto_init_thread_resting = NULL;
    std::thread *auto_init_thread_semi = NULL;
    std::thread *auto_init_thread_full = NULL;

    // loc score
    std::atomic<double> ui_loc_inlier_ratio;
    std::atomic<double> ui_loc_inlier_error;
    std::atomic<double> ui_mapping_inlier_ratio;
    std::atomic<double> ui_mapping_inlier_error;

    // state vars    
    std::atomic<int> last_accepted_cmd;
    std::atomic<int> ui_motor_state;
    std::atomic<int> ui_loc_state;
    std::atomic<int> ui_auto_state;
    std::atomic<int> ui_face_state;
    std::atomic<int> ui_velocity_preset;
    std::atomic<int> ui_motor_lock_state;
    std::atomic<int> ui_draw_state;
    std::atomic<int> ui_multi_state;    
    std::atomic<int> ui_fail_state;

    std::atomic<bool> is_received_motor_lock_off = {false};

    MOBILE_STATUS pre_status;
    cv::Vec3d pre_vwt;

    tbb::concurrent_queue<cv::Mat> plot_que;

    // integrate ui (orderPicking)
    QTimer integrateUI_timer;
    QByteArray IMGByte2;
    enum UI_LOC_STATE integrate_status;
    enum AUTO_FSM_STATE fsm_state;

    std::mutex mtx;

    // background loop
    std::atomic<bool> backFlag;
    std::thread* backThread = NULL;
    void backLoop();

    // initialize
    void init();
    void set_ui_items_from_unimap();

    // localization
    void loc_auto_init_full();
    void loc_auto_init_semi();
    void loc_auto_init_resting();
    void loc_manual_init(cv::Vec3d pose);

    // for check path
    bool check_approach_travel_line(cv::Mat &img, cv::Vec2i uv0, cv::Vec2i uv1);

    // save last pose
    void write_last_pose();

    // for ipc
    void publish_ipc_status();
    void publish_ipc_path();
    void publish_ipc_map();
    void publish_ipc_cam();
    void publish_ipc_cam_color();

    //for set_position
    cv::Vec3d yujin_pose;

private:
    QTimer plot_timer;
    QTimer watchdog_timer;

    int loc_fail_cnt = 0;
    double last_accepted_time = get_time();
    double last_accepted_times[34] = {get_time(),};

signals:

private slots:
    // mapping
    void bt_MapRun();
    void bt_MapStop();    
    void bt_MapSave();
    void bt_MapManualLc();

    // robot move
    void bt_RobotMotorInit();    
    void bt_RobotMoveForward_pressed();
    void bt_RobotMoveBackward_pressed();
    void bt_RobotMoveLeftTurn_pressed();
    void bt_RobotMoveRightTurn_pressed();
    void bt_RobotMove_released();
    void bt_RobotSetLed();

    // localization
    void bt_LocalizationLoad();
    void bt_LocalizationInit();
    void bt_LocalizationInitAuto_Full();
    void bt_LocalizationInitAuto_Semi();
    void bt_LocalizationInitAuto_Resting();
    void bt_LocalizationRun();
    void bt_LocalizationStop();
    void AutoInitFinished();

    // manual
    void cb_ManualServingGroup();

    void bt_ManualServingRun();
    void bt_ManualRestingRun();
    void bt_ManualChargingRun();
    void bt_ManualNodesRun();
    void bt_ManualClickedRun();

    void bt_ManualServingSet();
    void bt_ManualRestingSet();
    void bt_ManualChargingSet();
    void ckb_SetLocation(int val);

    void bt_ManualStop();
    void bt_ManualPause();
    void bt_ManualResume();
    void bt_Exit();

    // utils
    void bt_ReloadRobotConfig();
    void bt_CheckAllPath();
    void bt_DrawStart();
    void bt_DrawStop();

    // re-load maps
    void bt_SoftReload();
    void bt_HardReload();

    // SIM
    void bt_SimInit();
    void bt_SimEmoPush();
    void bt_SimEmoRelease();
    void make_sim_virtualObs();

    // send maps
    void bt_SendMaps();

    // set maps
    void bt_SetMap();

    // integrate ui (orderPicking)
    void bt_TestMoveExt();
    void bt_TestMovePick();
    void websocket_map_changed(QString map_name);
    void IntegrateUILoop();

    // plot loop    
    void plot_loop();
    void watchdog_loop();
    void process_command(CMD cur_cmd);

    // test
    void bt_Test();
    void bt_Test2();
    void bt_Test3();
    void bt_Test4();
};
#endif // MAINWINDOW_H
