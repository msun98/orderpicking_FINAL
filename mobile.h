#ifndef MOBILE_H
#define MOBILE_H

// defines
#include "global_defines.h"

// rb lan to can
#include "l2c.h"

// qt
#include <QObject>
#include <QTimer>

class MOBILE : public QObject
{
    Q_OBJECT
public:
    explicit MOBILE(QObject *parent = nullptr);
    ~MOBILE();

    void init();
    void move(double v, double w);
    void led(int target, int mode);        
    MOBILE_POSE get_pose();    
    MOBILE_STATUS get_status();

    std::mutex mtx;    
    MOBILE_POSE pose;
    MOBILE_STATUS status;    
    L2C l2c;

    std::atomic<double> last_v;
    std::atomic<double> last_w;

    std::atomic<bool> poolFlag;
    std::thread* poolThread = NULL;
    void poolLoop();

    std::atomic<bool> poolFlag2;
    std::thread* poolThread2 = NULL;
    void poolLoop2();

public slots:
    void connected();
    void motor_init();
    void motor_lock_off();
    void motor_gain(int Kp, int Ki, int Kd);
    void motor_cur_gain(int Kp, int Ki, int Kd);
    void motor_limit_update(float v_limit, float v_acc_limit, float w_limit, float w_acc_limit);
    void motor_mT_update(float m, float T);
    void pose_received(MOBILE_POSE mobile_pose);
    void status_received(MOBILE_STATUS mobile_status);
};

#endif // MOBILE_H
