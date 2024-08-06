#ifndef IPC_H
#define IPC_H

#include <QObject>
#include <QSharedMemory>

class IPC : public QObject
{
    Q_OBJECT

public:
    struct STATUS
    {
        uint32_t   tick = 0;
        int8_t     connection_m0 = 0;
        int8_t     connection_m1 = 0;
        int8_t     status_m0 = 0;
        int8_t     status_m1 = 0;
        int8_t     temp_m0 = 0;
        int8_t     temp_m1 = 0;
        int8_t     temp_ex_m0 = 0;
        int8_t     temp_ex_m1 = 0;
        int8_t     cur_m0 = 0;
        int8_t     cur_m1 = 0;
        int8_t     status_charge = 0;
        int8_t     status_power = 0;
        int8_t     status_emo = 0;
        int8_t     status_remote = 0;
        float      bat_in = 0;
        float      bat_out = 0;
        float      bat_cur = 0;
        float      power = 0;
        float      total_power = 0;
        int8_t     ui_loc_state = 0;
        int8_t     ui_auto_state = 0;
        int8_t     ui_obs_state = 0;
        int8_t     ui_face_state = 0;
        int8_t     ui_cur_velocity_preset = 0;
        int8_t     ui_motor_lock_state = 0;

        float      robot_pose[3] = {0,};
        float      robot_scan[360] = {0,};

        int8_t     ui_draw_state = 0;
        int8_t     ui_multi_state = 0;
        int8_t     ui_fail_state = 0;

        float      ui_loc_inlier_ratio = 0;
        float      ui_loc_inlier_error = 0;

        float      ui_mapping_inlier_ratio = 0;
        float      ui_mapping_inlier_error = 0;

        STATUS()
        {
        }
        STATUS(const STATUS& p)
        {
            tick = p.tick;
            connection_m0 = p.connection_m0;
            connection_m1 = p.connection_m1;
            status_m0 = p.status_m0;
            status_m1 = p.status_m1;
            temp_m0 = p.temp_m0;
            temp_m1 = p.temp_m1;
            temp_ex_m0 = p.temp_ex_m0;
            temp_ex_m1 = p.temp_ex_m1;
            cur_m0 = p.cur_m0;
            cur_m1 = p.cur_m1;
            status_charge = p.status_charge;
            status_power = p.status_power;
            status_emo = p.status_emo;
            status_remote = p.status_remote;
            bat_in = p.bat_in;
            bat_out = p.bat_out;
            bat_cur = p.bat_cur;
            power = p.power;
            total_power = p.total_power;
            ui_loc_state = p.ui_loc_state;
            ui_auto_state = p.ui_auto_state;
            ui_obs_state = p.ui_obs_state;
            ui_face_state = p.ui_face_state;
            ui_cur_velocity_preset = p.ui_cur_velocity_preset;
            ui_motor_lock_state = p.ui_motor_lock_state;
            ui_draw_state = p.ui_draw_state;
            ui_multi_state = p.ui_multi_state;
            ui_fail_state = p.ui_fail_state;
            ui_loc_inlier_ratio = p.ui_loc_inlier_ratio;
            ui_loc_inlier_error = p.ui_loc_inlier_error;
            ui_mapping_inlier_ratio = p.ui_mapping_inlier_ratio;
            ui_mapping_inlier_error = p.ui_mapping_inlier_error;
            memcpy(robot_pose, p.robot_pose, sizeof(float)*3);
            memcpy(robot_scan, p.robot_scan, sizeof(float)*360);
        }
    };

    struct PATH
    {
        uint32_t tick = 0;
        int32_t num = 0;
        float x[512] = {0,};
        float y[512] = {0,};

        PATH()
        {
        }
        PATH(const PATH& p)
        {
            tick = p.tick;
            num = p.num;
            memcpy(x, p.x, sizeof(float)*512);
            memcpy(y, p.y, sizeof(float)*512);
        }
    };

    struct MAP
    {
        uint32_t tick = 0;
        uint32_t width = 1000;
        uint32_t height = 1000;
        uint8_t buf[1000*1000] = {0,};

        MAP()
        {
        }
        MAP(const MAP& p)
        {
            tick = p.tick;
            width = p.width;
            height = p.height;
            memcpy(buf, p.buf, 1000*1000);
        }
    };

    struct IMG
    {
        uint32_t tick = 0;        
        uint8_t serial[255] = {0,};
        uint32_t width = 480;
        uint32_t height = 270;
        uint8_t buf[480*270] = {0,};

        IMG()
        {
        }
        IMG(const IMG& p)
        {
            tick = p.tick;
            width = p.width;
            height = p.height;
            memcpy(serial, p.serial, 255);
            memcpy(buf, p.buf, 480*270);
        }
    };

    struct IMG_COLOR
    {
        uint32_t tick = 0;
        uint8_t serial[255] = {0,};
        uint32_t width = 480;
        uint32_t height = 270;
        uint8_t buf[480*270*3] = {0,};

        IMG_COLOR()
        {
        }
        IMG_COLOR(const IMG_COLOR& p)
        {
            tick = p.tick;
            width = p.width;
            height = p.height;
            memcpy(serial, p.serial, 255);
            memcpy(buf, p.buf, 480*270*3);
        }
    };

    struct LOC_STATUS
    {
        uint32_t tick = 0;
        uint8_t serving[255] = {0,}; // array index means unique number of serving loc. occupied is 1

        LOC_STATUS()
        {
        }
        LOC_STATUS(const LOC_STATUS& p)
        {
            tick = p.tick;
            memcpy(serving, p.serving, 255);
        }
    };

    struct CALL_STATUS
    {
        uint32_t  tick = 0;
        uint8_t call_acceptable = 0;

        CALL_STATUS()
        {
        }
        CALL_STATUS(const CALL_STATUS& p)
        {
            tick = p.tick;
            call_acceptable = p.call_acceptable;
        }
    };

    struct CALL_LOC
    {
        uint32_t  tick = 0;
        int32_t loc_id = -1;

        CALL_LOC()
        {
        }
        CALL_LOC(const CALL_LOC& p)
        {
            tick = p.tick;
            loc_id = p.loc_id;
        }
    };

    struct CHECK_TRAVEL
    {
        uint32_t  tick = 0;

        uint8_t num = 0;
        uint8_t group[5][255] = {0,};
        uint8_t name[5][255] = {0,};
        uint8_t is_broken[5] = {0,};
        uint8_t is_far[5] = {0,};

        CHECK_TRAVEL()
        {
        }

        CHECK_TRAVEL(const CHECK_TRAVEL& p)
        {
            tick = p.tick;
            num = p.num;

            memcpy(group, p.group, 5*255);
            memcpy(name, p.name, 5*255);
            memcpy(is_broken, p.is_broken, 5);
            memcpy(is_far, p.is_far, 5);
        }
    };

public:
    explicit IPC(QObject *parent = nullptr);
    ~IPC();

    std::atomic<uint32_t> tick;

    QSharedMemory shm_status;
    QSharedMemory shm_path;
    QSharedMemory shm_map;
    QSharedMemory shm_obs;
    QSharedMemory shm_cam0;
    QSharedMemory shm_cam1;
    QSharedMemory shm_cam_color0;
    QSharedMemory shm_cam_color1;
    QSharedMemory shm_loc_status;
    QSharedMemory shm_call_status;
    QSharedMemory shm_call_loc;
    QSharedMemory shm_check_travel;

    STATUS get_status();
    PATH get_path();
    MAP get_map();
    MAP get_obs();
    IMG get_cam0();
    IMG get_cam1();
    IMG_COLOR get_cam_color0();
    IMG_COLOR get_cam_color1();
    LOC_STATUS get_loc_status();
    CALL_STATUS get_call_status();
    CALL_LOC get_call_loc();
    CHECK_TRAVEL get_check_travel();

    void set_status(IPC::STATUS val);
    void set_path(IPC::PATH val);
    void set_map(IPC::MAP val);
    void set_obs(IPC::MAP val);
    void set_cam0(IPC::IMG val);
    void set_cam1(IPC::IMG val);
    void set_cam_color0(IPC::IMG_COLOR val);
    void set_cam_color1(IPC::IMG_COLOR val);
    void set_loc_status(IPC::LOC_STATUS val);
    void set_call_status(IPC::CALL_STATUS val);
    void set_call_loc(IPC::CALL_LOC val);
    void set_check_travel(IPC::CHECK_TRAVEL val);

signals:

};

#endif // IPC_H
