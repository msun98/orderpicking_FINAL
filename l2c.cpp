#include "l2c.h"

L2C::L2C(QObject *parent) : QObject(parent)
{
    is_connected = false;
    is_resync = false;
}

L2C::~L2C()
{
    // send last message    
    msg_que.clear();
    SendLinearAngularVel(0,0);
    is_connected = false;

    // loop destroy
    if(commThread2 != NULL)
    {
        commFlag2 = false;
        commThread2->join();
        commThread2 = NULL;
    }

    // loop destroy
    if(commThread != NULL)
    {
        commFlag = false;
        commThread->join();
        commThread = NULL;
    }
}

void L2C::init()
{
#ifndef USE_SIM
    if (commThread == NULL)
    {
        commFlag = true;
        commThread = new std::thread(&L2C::commLoop, this);
    } 

    if (commThread2 == NULL)
    {
        commFlag2 = true;
        commThread2 = new std::thread(&L2C::commLoop2, this);
    }
#endif
}

void L2C::commLoop()
{
    logger.write("try to connect LAN2CAN 192.168.2.100:1977", true);

    // lan to can ip : 192.168.2.100, port : 1977
    sockaddr_in server_addr;
    bzero((char*)&server_addr, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = inet_addr("192.168.2.100");
    server_addr.sin_port = htons(1977);

    // connection
    fd = socket(AF_INET, SOCK_STREAM, 0);
    if(fd < 0)
    {
        logger.write("l2c socket create failed", true);
        return;
    }

    int status = ::connect(fd, (sockaddr*)&server_addr, sizeof(server_addr));
    if(status < 0)
    {
        logger.write("l2c socket connection failed", true);
        return;
    }

    // connected    
    emit connected();
    is_connected = true;    
    logger.write("[L2C] connected", true);

    // variable init
    int offset_cnt = 0;
    int packet_size = 8+126; //75

    sync_st_time = get_time();

#ifdef USE_EX_TEMP
    packet_size = 77;
#endif

    std::vector<uchar> buf;

    logger.write("l2c loop start", true);
    while(commFlag)
    {
        // recv
        std::vector<uchar> recv_buf(2000, 0);
        int num = read(fd, (char*)recv_buf.data(), recv_buf.size());
        if(num == 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        buf.insert(buf.end(), recv_buf.begin(), recv_buf.begin()+num);

        // check fine packet
        bool is_fine = false;
        if((int)buf.size() == packet_size)
        {
            is_fine = true;
        }

        // parsing
        while((int)buf.size() >= packet_size)
        {
            uchar *_buf = (uchar*)buf.data();
            if(_buf[0] == 0x24 && _buf[5] == 0xA2 && _buf[packet_size-1] == 0x25)
            {
                int index=6;
                int dlc=1;
                int dlc_f=4;

                uint32_t tick;
                memcpy(&tick, &_buf[index], dlc_f);     index=index+dlc_f;

                double mobile_t = tick*0.002;
                if((is_fine && offset_cnt % 10000 == 0) || is_resync)
                {
                    double pc_t = get_time();
                    offset_t = pc_t - mobile_t;
                    is_resync = false;
                }
                offset_cnt++;

                uint8_t connection_status_m0, connection_status_m1;
                connection_status_m0 = _buf[index];     index=index+dlc;
                connection_status_m1 = _buf[index];     index=index+dlc;

                float x_dot, y_dot, th_dot;
                memcpy(&x_dot, &_buf[index], dlc_f);     index=index+dlc_f;
                memcpy(&y_dot, &_buf[index], dlc_f);     index=index+dlc_f;
                memcpy(&th_dot, &_buf[index], dlc_f);    index=index+dlc_f;

                float x, y, th;
                memcpy(&x, &_buf[index], dlc_f);     index=index+dlc_f;
                memcpy(&y, &_buf[index], dlc_f);     index=index+dlc_f;
                memcpy(&th, &_buf[index], dlc_f);    index=index+dlc_f;

                float local_v, local_w;
                memcpy(&local_v, &_buf[index], dlc_f);     index=index+dlc_f;
                memcpy(&local_w, &_buf[index], dlc_f);     index=index+dlc_f;

                uint8_t stat_m0, stat_m1;
                stat_m0 = _buf[index];     index=index+dlc;
                stat_m1 = _buf[index];     index=index+dlc;

                uint8_t temp_m0, temp_m1;
                temp_m0 = _buf[index];     index=index+dlc;
                temp_m1 = _buf[index];     index=index+dlc;

                #ifdef USE_EX_TEMP
                uint8_t temp_ex_m0, temp_ex_m1;
                temp_ex_m0 = _buf[index];     index=index+dlc;
                temp_ex_m1 = _buf[index];     index=index+dlc;
                #endif

                uint8_t cur_m0, cur_m1;
                cur_m0 = _buf[index];     index=index+dlc;
                cur_m1 = _buf[index];     index=index+dlc;

                uint8_t charge_state, power_state, emo_state, remote_state;
                charge_state = _buf[index];     index=index+dlc;
                power_state = _buf[index];      index=index+dlc;
                emo_state = _buf[index];        index=index+dlc;
                remote_state = _buf[index];     index=index+dlc;

                float bat_in, bat_out, bat_cur, power, total_used_power;
                memcpy(&bat_in, &_buf[index], dlc_f);               index=index+dlc_f;
                memcpy(&bat_out, &_buf[index], dlc_f);              index=index+dlc_f;
                memcpy(&bat_cur, &_buf[index], dlc_f);              index=index+dlc_f;
                memcpy(&power, &_buf[index], dlc_f);                index=index+dlc_f;
                memcpy(&total_used_power, &_buf[index], dlc_f);     index=index+dlc_f;

                uint32_t recv_tick, return_time;
                memcpy(&recv_tick, &_buf[index], dlc_f);     index=index+dlc_f;
                memcpy(&return_time, &_buf[index], dlc_f);     index=index+dlc_f;

                uint8_t roller_controller_state;
                memcpy(&roller_controller_state, &_buf[index], dlc);     index=index+dlc;

                uint8_t roller_sensor0, roller_sensor1, roller_sensor2, roller_sensor3;
                memcpy(&roller_sensor0, &_buf[index], dlc);     index=index+dlc;
                memcpy(&roller_sensor1, &_buf[index], dlc);     index=index+dlc;
                memcpy(&roller_sensor2, &_buf[index], dlc);     index=index+dlc;
                memcpy(&roller_sensor3, &_buf[index], dlc);     index=index+dlc;

                uint8_t roller_manual_sw0, roller_manual_sw1;
                memcpy(&roller_manual_sw0, &_buf[index], dlc);     index=index+dlc;
                memcpy(&roller_manual_sw1, &_buf[index], dlc);     index=index+dlc;

                uint8_t roller_blocking_state0, roller_blocking_state1;
                memcpy(&roller_blocking_state0, &_buf[index], dlc);     index=index+dlc;
                memcpy(&roller_blocking_state1, &_buf[index], dlc);     index=index+dlc;

                uint8_t roller_blocking_manual_sw0, roller_blocking_manual_sw1, roller_blocking_manual_sw2, roller_blocking_manual_sw3;
                memcpy(&roller_blocking_manual_sw0, &_buf[index], dlc);     index=index+dlc;
                memcpy(&roller_blocking_manual_sw1, &_buf[index], dlc);     index=index+dlc;
                memcpy(&roller_blocking_manual_sw2, &_buf[index], dlc);     index=index+dlc;
                memcpy(&roller_blocking_manual_sw3, &_buf[index], dlc);     index=index+dlc;

                uint8_t orgo_on_init, orgo_on_run;
                memcpy(&orgo_on_init, &_buf[index], dlc);     index=index+dlc;
                memcpy(&orgo_on_run, &_buf[index], dlc);     index=index+dlc;

                uint8_t orgo_pos_state0, orgo_pos_state1;
                memcpy(&orgo_pos_state0, &_buf[index], dlc);     index=index+dlc;
                memcpy(&orgo_pos_state1, &_buf[index], dlc);     index=index+dlc;

                uint8_t orgo_manual_sw0, orgo_manual_sw1;
                memcpy(&orgo_manual_sw0, &_buf[index], dlc);     index=index+dlc;
                memcpy(&orgo_manual_sw1, &_buf[index], dlc);     index=index+dlc;

                int orgo_pos0, orgo_pos1;
                memcpy(&orgo_pos0, &_buf[index], dlc);     index=index+dlc_f;
                memcpy(&orgo_pos1, &_buf[index], dlc);     index=index+dlc_f;

                float imu_gyr_x, imu_gyr_y, imu_gyr_z;
                memcpy(&imu_gyr_x, &_buf[index], dlc_f);      index=index+dlc_f;
                memcpy(&imu_gyr_y, &_buf[index], dlc_f);      index=index+dlc_f;
                memcpy(&imu_gyr_z, &_buf[index], dlc_f);      index=index+dlc_f;

                float imu_acc_x, imu_acc_y, imu_acc_z;
                memcpy(&imu_acc_x, &_buf[index], dlc_f);      index=index+dlc_f;
                memcpy(&imu_acc_y, &_buf[index], dlc_f);      index=index+dlc_f;
                memcpy(&imu_acc_z, &_buf[index], dlc_f);      index=index+dlc_f;

                // received mobile pose update
                MOBILE_POSE mobile_pose;
                mobile_pose.t = mobile_t + offset_t;
                mobile_pose.pose = cv::Vec3d(x, y, toWrap(th));
                mobile_pose.vel = cv::Vec3d(x_dot, y_dot, th_dot);
                mobile_pose.vw = cv::Vec2d(local_v, local_w);

                #ifndef USE_SIM
                pose_que.push(mobile_pose);

                if(pose_que.unsafe_size() > 50)
                {
                    MOBILE_POSE dummy;
                    pose_que.try_pop(dummy);
                }
                #endif

                //emit pose_received(mobile_pose);

                // received mobile status update
                MOBILE_STATUS mobile_status;
                mobile_status.t = mobile_t + offset_t;

                // motor
                mobile_status.connection_m0 = connection_status_m0;
                mobile_status.connection_m1 = connection_status_m1;
                mobile_status.status_m0 = stat_m0;
                mobile_status.status_m1 = stat_m1;
                mobile_status.temp_m0 = temp_m0;
                mobile_status.temp_m1 = temp_m1;
                mobile_status.cur_m0 = cur_m0;
                mobile_status.cur_m1 = cur_m1;
                mobile_status.charge_state = charge_state;
                mobile_status.power_state = power_state;
                mobile_status.emo_state = emo_state;
                mobile_status.remote_state = remote_state;
                mobile_status.bat_in = bat_in;
                mobile_status.bat_out = bat_out;
                mobile_status.bat_current = bat_cur;
                mobile_status.power = power;
                mobile_status.total_power = total_used_power;
                mobile_status.recv_tick = recv_tick;
                mobile_status.return_time = return_time;

                // roller
                mobile_status.roller_controller_state = roller_controller_state;
                mobile_status.roller_sensor0 = roller_sensor0;
                mobile_status.roller_sensor1 = roller_sensor1;
                mobile_status.roller_sensor2 = roller_sensor2;
                mobile_status.roller_sensor3 = roller_sensor3;
                mobile_status.roller_manual_sw0 = roller_manual_sw0;
                mobile_status.roller_manual_sw1 = roller_manual_sw1;
                mobile_status.roller_blocking_state0 = roller_blocking_state0;
                mobile_status.roller_blocking_state1 = roller_blocking_state1;
                mobile_status.roller_blocking_manual_sw0 = roller_blocking_manual_sw0;
                mobile_status.roller_blocking_manual_sw1 = roller_blocking_manual_sw1;
                mobile_status.roller_blocking_manual_sw2 = roller_blocking_manual_sw2;
                mobile_status.roller_blocking_manual_sw3 = roller_blocking_manual_sw3;
                mobile_status.orgo_on_init = orgo_on_init;
                mobile_status.orgo_on_run = orgo_on_run;
                mobile_status.orgo_pos_state0 = orgo_pos_state0;
                mobile_status.orgo_pos_state1 = orgo_pos_state1;
                mobile_status.orgo_manual_sw0 = orgo_manual_sw0;
                mobile_status.orgo_manual_sw1 = orgo_manual_sw1;
                mobile_status.orgo_pos1 = orgo_pos1;
                mobile_status.orgo_pos0 = orgo_pos0;

                // imu
                mobile_status.imu_gyr_x = imu_gyr_x * D2R;
                mobile_status.imu_gyr_y = imu_gyr_y * D2R;
                mobile_status.imu_gyr_z = imu_gyr_z * D2R;
                mobile_status.imu_acc_x = imu_acc_x * ACC_G;
                mobile_status.imu_acc_y = imu_acc_y * ACC_G;
                mobile_status.imu_acc_z = imu_acc_z * ACC_G;

                #ifndef USE_SIM
                status_que.push(mobile_status);
                if(status_que.unsafe_size() > 50)
                {
                    MOBILE_STATUS dummy;
                    status_que.try_pop(dummy);
                }
                #endif

                //emit status_received(mobile_status);
            }

            buf.erase(buf.begin(), buf.begin() + packet_size);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    ::close(fd);

    /*logger.write("l2c loop start", true);
    while(commFlag)
    {
        // recv
        std::vector<uchar> recv_buf(2000, 0);
        int num = read(fd, (char*)recv_buf.data(), recv_buf.size());
        if(num == 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        buf.insert(buf.end(), recv_buf.begin(), recv_buf.begin()+num);

        // check fine packet
        bool is_fine = false;
        if((int)buf.size() == packet_size)
        {
            is_fine = true;
        }

        // parsing
        while((int)buf.size() >= packet_size)
        {
            uchar *_buf = (uchar*)buf.data();
            if(_buf[0] == 0x24 && _buf[5] == 0xA2 && _buf[packet_size-1] == 0x25)
            {
                int index=6;
                int dlc=1;
                int dlc_f=4;

                uint32_t tick;
                memcpy(&tick, &_buf[index], dlc_f);     index=index+dlc_f;

                double mobile_t = tick*0.002;
                if((is_fine && offset_cnt % 10000 == 0) || is_resync)
                {
                    double pc_t = get_time();
                    offset_t = pc_t - mobile_t;
                    is_resync = false;
                }
                offset_cnt++;

                uint8_t connection_status_m0, connection_status_m1;
                connection_status_m0 = _buf[index];     index=index+dlc;
                connection_status_m1 = _buf[index];     index=index+dlc;

                float x_dot, y_dot, th_dot;
                memcpy(&x_dot, &_buf[index], dlc_f);     index=index+dlc_f;
                memcpy(&y_dot, &_buf[index], dlc_f);     index=index+dlc_f;
                memcpy(&th_dot, &_buf[index], dlc_f);    index=index+dlc_f;

                float x, y, th;
                memcpy(&x, &_buf[index], dlc_f);     index=index+dlc_f;
                memcpy(&y, &_buf[index], dlc_f);     index=index+dlc_f;
                memcpy(&th, &_buf[index], dlc_f);    index=index+dlc_f;

                float local_v, local_w;
                memcpy(&local_v, &_buf[index], dlc_f);     index=index+dlc_f;
                memcpy(&local_w, &_buf[index], dlc_f);     index=index+dlc_f;

                uint8_t stat_m0, stat_m1;
                stat_m0 = _buf[index];     index=index+dlc;
                stat_m1 = _buf[index];     index=index+dlc;

                uint8_t temp_m0, temp_m1;
                temp_m0 = _buf[index];     index=index+dlc;
                temp_m1 = _buf[index];     index=index+dlc;

                #ifdef USE_EX_TEMP
                uint8_t temp_ex_m0, temp_ex_m1;
                temp_ex_m0 = _buf[index];     index=index+dlc;
                temp_ex_m1 = _buf[index];     index=index+dlc;
                #endif

                uint8_t cur_m0, cur_m1;
                cur_m0 = _buf[index];     index=index+dlc;
                cur_m1 = _buf[index];     index=index+dlc;

                uint8_t charge_state, power_state, emo_state, remote_state;
                charge_state = _buf[index];     index=index+dlc;
                power_state = _buf[index];      index=index+dlc;
                emo_state = _buf[index];        index=index+dlc;
                remote_state = _buf[index];     index=index+dlc;

                float bat_in, bat_out, bat_cur, power, total_used_power;
                memcpy(&bat_in, &_buf[index], dlc_f);               index=index+dlc_f;
                memcpy(&bat_out, &_buf[index], dlc_f);              index=index+dlc_f;
                memcpy(&bat_cur, &_buf[index], dlc_f);              index=index+dlc_f;
                memcpy(&power, &_buf[index], dlc_f);                index=index+dlc_f;
                memcpy(&total_used_power, &_buf[index], dlc_f);     index=index+dlc_f;

                // received mobile pose update
                MOBILE_POSE mobile_pose;
                mobile_pose.t = mobile_t + offset_t;
                mobile_pose.pose = cv::Vec3d(x, y, toWrap(th));
                mobile_pose.vel = cv::Vec3d(x_dot, y_dot, th_dot);
                mobile_pose.vw = cv::Vec2d(local_v, local_w);

                #ifndef USE_SIM
                pose_que.push(mobile_pose);

                if(pose_que.unsafe_size() > 50)
                {
                    MOBILE_POSE dummy;
                    pose_que.try_pop(dummy);
                }
                #endif

                //emit pose_received(mobile_pose);

                // received mobile status update
                MOBILE_STATUS mobile_status;
                mobile_status.is_ok = true;
                mobile_status.connection_m0 = connection_status_m0;
                mobile_status.connection_m1 = connection_status_m1;
                mobile_status.status_m0 = stat_m0;
                mobile_status.status_m1 = stat_m1;
                mobile_status.temp_m0 = temp_m0;
                mobile_status.temp_m1 = temp_m1;

                #ifdef USE_EX_TEMP
                mobile_status.temp_ex_m0 = temp_ex_m0;
                mobile_status.temp_ex_m1 = temp_ex_m1;
                #endif

                mobile_status.cur_m0 = cur_m0;
                mobile_status.cur_m1 = cur_m1;
                mobile_status.charge_state = charge_state;
                mobile_status.power_state = power_state;
                mobile_status.emo_state = emo_state;
                mobile_status.remote_state = remote_state;
                mobile_status.bat_in = bat_in;
                mobile_status.bat_out = bat_out;
                mobile_status.bat_current = bat_cur;
                mobile_status.power = power;
                mobile_status.total_power = total_used_power;

                #ifndef USE_SIM
                status_que.push(mobile_status);
                if(status_que.unsafe_size() > 50)
                {
                    MOBILE_STATUS dummy;
                    status_que.try_pop(dummy);
                }
                #endif

                //emit status_received(mobile_status);
            }

            buf.erase(buf.begin(), buf.begin() + packet_size);            
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    ::close(fd);*/
}

void L2C::commLoop2()
{
    logger.write("l2c loop2 start", true);
    while(commFlag2)
    {
        if(is_connected)
        {
            // send
            std::vector<uchar> msg;
            if(msg_que.try_pop(msg))
            {
                #ifndef USE_SIM
                ::send(fd, msg.data(), msg.size(), 0);                
                #endif
                //continue;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void L2C::SendMotorInit(float kp, float ki, float kd)
{
    std::vector<uchar> send_byte(25, 0);
    send_byte[0] = 0x24;

    uint16_t size = 6+8+8;
    memcpy(&send_byte[1], &size, 2); // size
    send_byte[3] = 0x00;
    send_byte[4] = 0x00;

    send_byte[5] = 0xA0;
    send_byte[6] = 0x00;
    send_byte[7] = 100; // cmd motor init

    memcpy(&send_byte[8], &kp, 4);
    memcpy(&send_byte[12], &ki, 4);
    memcpy(&send_byte[16], &kd, 4);
    send_byte[24] = 0x25;

    if(is_connected)
    {
        msg_que.push(send_byte);
    }
}

void L2C::SendMotorGain(float kp, float ki, float kd)
{
    std::vector<uchar> send_byte(25, 0);
    send_byte[0] = 0x24;

    uint16_t size = 6+8+8;
    memcpy(&send_byte[1], &size, 2); // size
    send_byte[3] = 0x00;
    send_byte[4] = 0x00;

    send_byte[5] = 0xA0;
    send_byte[6] = 0x00;
    send_byte[7] = 106; // cmd motor init

    memcpy(&send_byte[8], &kp, 4);
    memcpy(&send_byte[12], &ki, 4);
    memcpy(&send_byte[16], &kd, 4);
    send_byte[24] = 0x25;

    if(is_connected)
    {
        msg_que.push(send_byte);
    }
}

void L2C::SendMotorCurGain(float kp, float ki, float kd)
{
    std::vector<uchar> send_byte(25, 0);
    send_byte[0] = 0x24;

    uint16_t size = 6+8+8;
    memcpy(&send_byte[1], &size, 2); // size
    send_byte[3] = 0x00;
    send_byte[4] = 0x00;

    send_byte[5] = 0xA0;
    send_byte[6] = 0x00;
    send_byte[7] = 107; // cmd motor init

    memcpy(&send_byte[8], &kp, 4);
    memcpy(&send_byte[12], &ki, 4);
    memcpy(&send_byte[16], &kd, 4);
    send_byte[24] = 0x25;

    if(is_connected)
    {
        msg_que.push(send_byte);
    }
}

void L2C::SendMotorLimitUpdate(float v_limit, float v_acc_limit, float w_limit, float w_acc_limit)
{
    std::vector<uchar> send_byte(25, 0);
    send_byte[0] = 0x24;

    uint16_t size = 6+8+8;
    memcpy(&send_byte[1], &size, 2); // size
    send_byte[3] = 0x00;
    send_byte[4] = 0x00;

    send_byte[5] = 0xA0;
    send_byte[6] = 0x00;
    send_byte[7] = 108; // cmd error clear

    memcpy(&send_byte[8], &v_limit, 4); // m/s
    memcpy(&send_byte[12], &v_acc_limit, 4); // m/s^2
    memcpy(&send_byte[16], &w_limit, 4); // rad/s
    memcpy(&send_byte[20], &w_acc_limit, 4); // rad/s^2
    send_byte[24] = 0x25;

    if(is_connected)
    {
        msg_que.push(send_byte);
    }
}

void L2C::SendMotormTUpdate(float m, float T)
{
    std::vector<uchar> send_byte(25, 0);
    send_byte[0] = 0x24;

    uint16_t size = 6+8+8;
    memcpy(&send_byte[1], &size, 2); // size
    send_byte[3] = 0x00;
    send_byte[4] = 0x00;

    send_byte[5] = 0xA0;
    send_byte[6] = 0x00;
    send_byte[7] = 109; // cmd error clear

    memcpy(&send_byte[8], &m, 4);
    memcpy(&send_byte[12], &T, 4);
    send_byte[24] = 0x25;

    if(is_connected)
    {
        msg_que.push(send_byte);
    }
}

void L2C::SendMotorID(float id_r, float id_l)
{
    std::vector<uchar> send_byte(25, 0);
    send_byte[0] = 0x24;

    uint16_t size = 6+8+8;
    memcpy(&send_byte[1], &size, 2); // size
    send_byte[3] = 0x00;
    send_byte[4] = 0x00;

    send_byte[5] = 0xA0;
    send_byte[6] = 0x00;
    send_byte[7] = 101; // cmd motor init

    memcpy(&send_byte[8], &id_r, 4);
    memcpy(&send_byte[12], &id_l, 4);
    send_byte[24] = 0x25;

    if(is_connected)
    {
        msg_que.push(send_byte);
    }
}

void L2C::SendMotorLimitVel(float limit_v, float limit_w)
{
    std::vector<uchar> send_byte(25, 0);
    send_byte[0] = 0x24;

    uint16_t size = 6+8+8;
    memcpy(&send_byte[1], &size, 2); // size
    send_byte[3] = 0x00;
    send_byte[4] = 0x00;

    send_byte[5] = 0xA0;
    send_byte[6] = 0x00;
    send_byte[7] = 102; // cmd motor init

    memcpy(&send_byte[8], &limit_v, 4);
    memcpy(&send_byte[12], &limit_w, 4);
    send_byte[24] = 0x25;

    if(is_connected)
    {
        msg_que.push(send_byte);
    }
}

void L2C::SendMotorLimitAcc(float limit_v_acc, float limit_w_acc)
{
    std::vector<uchar> send_byte(25, 0);
    send_byte[0] = 0x24;

    uint16_t size = 6+8+8;
    memcpy(&send_byte[1], &size, 2); // size
    send_byte[3] = 0x00;
    send_byte[4] = 0x00;

    send_byte[5] = 0xA0;
    send_byte[6] = 0x00;
    send_byte[7] = 103; // cmd motor init

    memcpy(&send_byte[8], &limit_v_acc, 4);
    memcpy(&send_byte[12], &limit_w_acc, 4);
    send_byte[24] = 0x25;

    if(is_connected)
    {
        msg_que.push(send_byte);
    }
}

void L2C::SendMotorWheelSpec(float wheel_base, float wheel_radius)
{
    std::vector<uchar> send_byte(25, 0);
    send_byte[0] = 0x24;

    uint16_t size = 6+8+8;
    memcpy(&send_byte[1], &size, 2); // size
    send_byte[3] = 0x00;
    send_byte[4] = 0x00;

    send_byte[5] = 0xA0;
    send_byte[6] = 0x00;
    send_byte[7] = 104; // cmd motor init

    memcpy(&send_byte[8], &wheel_base, 4);
    memcpy(&send_byte[12], &wheel_radius, 4);
    send_byte[24] = 0x25;

    if(is_connected)
    {
        msg_que.push(send_byte);
    }
}

void L2C::SendMotorSpec(float wheel_dir, float gear_ratio)
{
    std::vector<uchar> send_byte(25, 0);
    send_byte[0] = 0x24;

    uint16_t size = 6+8+8;
    memcpy(&send_byte[1], &size, 2); // size
    send_byte[3] = 0x00;
    send_byte[4] = 0x00;

    send_byte[5] = 0xA0;
    send_byte[6] = 0x00;
    send_byte[7] = 105; // cmd motor init

    memcpy(&send_byte[8], &wheel_dir, 4);
    memcpy(&send_byte[12], &gear_ratio, 4);
    send_byte[24] = 0x25;

    if(is_connected)
    {
        msg_que.push(send_byte);
    }
}

void L2C::SendLinearAngularVel(float v, float w)
{
    std::vector<uchar> send_byte(25, 0);
    send_byte[0] = 0x24;

    uint16_t size = 6+8+8;
    memcpy(&send_byte[1], &size, 2); // size
    send_byte[3] = 0x00;
    send_byte[4] = 0x00;

    send_byte[5] = 0xA0;
    send_byte[6] = 0x00;
    send_byte[7] = 10; // cmd move

    memcpy(&send_byte[8], &v, 4); // param1 linear vel
    memcpy(&send_byte[12], &w, 4); // param2 angular vel
    send_byte[24] = 0x25;

    if(is_connected)
    {
        msg_que.push(send_byte);
    }
}

void L2C::SendLed(int target, int mode)
{
    std::vector<uchar> send_byte(25, 0);
    send_byte[0] = 0x24;

    uint16_t size = 6+8+8;
    memcpy(&send_byte[1], &size, 2); // size
    send_byte[3] = 0x00;
    send_byte[4] = 0x00;

    send_byte[5] = 0xB0;
    send_byte[6] = target; // 0~1
    send_byte[7] = mode; // cmd move

    send_byte[24] = 0x25;

    if(is_connected)
    {
        msg_que.push(send_byte);
    }
}

void L2C::SendMotorLockOff()
{
    std::vector<uchar> send_byte(25, 0);
    send_byte[0] = 0x24;

    uint16_t size = 6+8+8;
    memcpy(&send_byte[1], &size, 2); // size
    send_byte[3] = 0x00;
    send_byte[4] = 0x00;

    send_byte[5] = 0xA0;
    send_byte[6] = 0x00; // 0~1
    send_byte[7] = 120; // cmd move

    send_byte[24] = 0x25;

    if(is_connected)
    {
        msg_que.push(send_byte);
    }
}
