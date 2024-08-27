#ifndef COBOT_H
#define COBOT_H

#include <QObject>
#include <QTimer>
#include <QTcpSocket>
#include <QHostAddress>
#include <QTime>

#include "common_header.h"
#include "global_defines.h"
#include "option_writer.h"
//#include <eigen3/Eigen/Core>

class Cobot : public QObject
{
    Q_OBJECT

public:
    explicit Cobot(QObject *parent = nullptr);
    ~Cobot();

    Option_Writer settings;

    systemSTAT systemStat;
    systemCONFIG systemConfig;
    systemPOPUP  systemPopup;

    ipInfo IP_RB5_CMD;
    ipInfo IP_RB5_DATA;
    QTcpSocket cmdSocket;
    QTcpSocket dataSocket;

    QTimer* init_timer;
    bool flag_check_RB5_MODE = false;
    bool flag_check_connection_initializaion = false;

    // check socket status
    int test = 0;

    float _mslider_Spd = 0.f;

    int cmdConfirmFlag;
    int moveCmdFlag;
    int moveCmdCnt;

    int speed_changed = false;

    QByteArray recvBuf;
    QByteArray datas;

    QTimer timer;
    QTimer logicTimer;
    QTimer systemTimer;

    void write_log(Integrated_infomation& Ii, const QString& text);

    void check_socket_state();
    void MoveJoint_specific_pos(const QString& path);
    void SaveJoint_specific_pos(const QString& path);

    tcpInfo calcCalibration(xyzInfo visionPoint, const tcpInfo& v2n, const tcpInfo& n2e);
    Eigen::Matrix3f calcRoll(float x);
    Eigen::Matrix3f calcPitch(float y);
    Eigen::Matrix3f calcYaw(float z);

    bool realflag=false;

    void init();

    void RB5_Connect_and_initialzation();
    void RB5_MODE_CHANGE();

    void CobotInit();
    int Get_RB5_state();
    // Cobot Control API -------------------
    // <CobotInit>
    // : initialize Cobot
    //    void CobotInit();

    // <MoveJoint>
    // : move to target posture in joint coordinate
    // joint1~joint6 : target joint angle in deg unit
    // spd : speed parameter (0~1: user define   or   -1: default setting)
    // acc : acceleration parameter (0~1: user define   or   -1: default setting)
    void MoveJoint(float joint1, float joint2, float joint3, float joint4, float joint5, float joint6, float spd = -1, float acc = -1);

    void move_jb_clear();
    void move_jb_add(std::vector<float> myjoint);
    void move_jb_run(int a, int b) ;
    void MoveL_rel(float x, float y, float z, float rx, float ry, float rz, float spd= -1, float acc= -1);

    // <MoveTCP>
    // : move to target posture in cartesian coordinate
    // x, y, z : target TCP(tool center point) position in mm unit
    // rx, ry, rz : target TCP orientation (Yaw-Pitch-Roll Euler angle) in degree unit
    // spd : speed parameter (0~1: user define   or   -1: default setting)
    // acc : acceleration parameter (0~1: user define   or   -1: default setting)
    void MoveTCP(float x, float y, float z, float rx, float ry, float rz, float spd = -1, float acc = -1);

    // <MoveCircle_ThreePoint>
    // : move current position to final position while it follows circle trajectory
    // : the circle trajectory is derived from current, first, and final position
    // type : 0 - try to follow both input position and orientation
    //        1 - orientation will be fixed to current orientation
    //        2 - orientation will be changed perpendicularly starting from current orientation
    // x1, y1, z1 : first position in mm unit
    // rx1, ry1, rz1 : first orientation (Yaw-Pitch-Roll Euler angle) in degree unit
    // x2, y2, z2 : final position in mm unit
    // rx2, ry2, rz2 : final orientation (Yaw-Pitch-Roll Euler angle) in degree unit
    // spd : speed parameter (0~1: user define   or   -1: default setting)
    // acc : acceleration parameter (0~1: user define   or   -1: default setting)
    void MoveCircle_ThreePoint(int type, float x1, float y1, float z1, float rx1, float ry1, float rz1, float x2, float y2, float z2, float rx2, float ry2, float rz2, float spd = -1, float acc = -1);

    // <MoveCircle_Axis>
    // : move current position to final position while it follows circle trajectory
    // : the circle trajectory is derived from current position, center position, axis, and rotation angle
    // type : 0 - try to follow both input position and orientation
    //        1 - orientation will be fixed to current orientation
    //        2 - orientation will be changed perpendicularly starting from current orientation
    // cx, cy, cz : center position in mm unit
    // ax, ay, az : axis representation (norminal)
    // rot_angle: rotation angle in degree unit
    // spd : speed parameter (0~1: user define   or   -1: default setting)
    // acc : acceleration parameter (0~1: user define   or   -1: default setting)
    void MoveCircle_Axis(int type, float cx, float cy, float cz, float ax, float ay, float az, float rot_angle, float spd, float acc);

    // <MoveJointBlend_Clear>
    // : clear joint blend list
    void MoveJointBlend_Clear();

    // <MoveJointBlend_AddPoint>
    // : add point to the joint blend list
    // : only the last point's 'vel' and 'acc' will be applied
    // joint1~joint6 : target joint angle in deg unit
    // spd : speed parameter (0~1: user define   or   -1: default setting)
    // acc : acceleration parameter (0~1: user define   or   -1: default setting)
    void MoveJointBlend_AddPoint(float joint1, float joint2, float joint3, float joint4, float joint5, float joint6, float spd = -1, float acc = -1);

    // <MoveJointBlend_MovePoint>
    // : start to move all the points in the joint blend list
    void MoveJointBlend_MovePoint();

    // <MoveTCPBlend_Clear>
    // : clear TCP blend list
    void MoveTCPBlend_Clear();

    // <MoveTCPBlend_AddPoint>
    // : add point to the TCP blend list
    // : only the last point's 'vel' and 'acc' will be applied
    // radius : blend distance in mm unit
    // x, y, z : target TCP(tool center point) position in mm unit
    // rx, ry, rz : target TCP orientation (Yaw-Pitch-Roll Euler angle) in degree unit
    // spd : speed parameter (0~1: user define   or   -1: default setting)
    // acc : acceleration parameter (0~1: user define   or   -1: default setting)
    void MoveTCPBlend_AddPoint(float radius, float x, float y, float z, float rx, float ry, float rz, float spd = -1, float acc = -1);

    // <MoveTCPBlend_MovePoint>
    // : start to move all the points in the joint blend list
    void MoveTCPBlend_MovePoint();

    // <ControlBoxDigitalOut>
    // control digital out ports in control box
    // d0~d15 : digital out value (0 or 1)
    void ControlBoxDigitalOut(int d0, int d1, int d2, int d3, int d4, int d5, int d6, int d7, int d8, int d9, int d10, int d11, int d12, int d13, int d14, int d15);

    // <ControlBoxAnalogOut>
    // control analog out ports in control box
    // a0~a3 : analog out value in voltage unit (0~10)
    void ControlBoxAnalogOut(float a0, float a1, float a2, float a3);

    // <ToolOut>
    // control digital out ports and voltage level in tool flange board
    // volt : reference voltage of tool flange board in voltage unit(0, 12, 24)
    // d0, d1 : digital out value (0 or 1)
    void ToolOut(int volt, int d0, int d1);

    // <ProgramMode_Real>
    // change to 'real robot' mode -- robot will move
    void ProgramMode_Real();

    // <ProgramMode_Simulation>
    // change to 'simulation' mode -- robot will not move except teaching
    void ProgramMode_Simulation();

    // <BaseSpeedChange>
    // change base speed -- base speed will be multiplied to motion velocity
    // spd : normalized base speed (0~1)
    void BaseSpeedChange(float spd);

    // <MotionPause>
    // pause the current motion
    void MotionPause();

    // <MotionHalt>
    // halt the current motion
    // !! CAUTION : user would better escape the motion sequence
    //            : if not, the next motion will be activated immediately
    void MotionHalt();

    // <MotionResume>
    // resume the paused motion
    void MotionResume();

    // <CollisionResume>
    // resume the motion which is paused due to external collision
    void CollisionResume();

    int IsMotionIdle();

    void get_Joint_Tcp(jointInfo& js, tcpInfo& ts);

    void move_jb2_clear();
    void move_jb2_add(float joint1, float joint2, float joint3, float joint4, float joint5, float joint6, float spd, float acc, float blending_value);
    void move_jb2_run();

private:
    int initFlag;

signals:
    void SIGNAL_LOG_WRITE();
    void CobotMSG (QString msg);


public slots:
    void onSystemCheck();
    void task_intialization();

    void onReadyCmdRead();
    void onReadyDataRead();
    void requestDataPacket();

    void onCmdConnected();
    void onCmdDisconnected();
    void onDataConnected();
    void onDataDisconnected();
    void onSliderChange_RB5_Speed(int val);

};

#endif // COBOT_H

