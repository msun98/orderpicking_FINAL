//랙과 모바일 플랫폼과의 거리는 45cm 정도를 유지해야 하며 랙 한칸의 높이는 70cm를 유지해야한다.￣

#include "mainwindow.h"
#include "ui_mainwindow.h"
Integrated_infomation Integrated_info;//헤더 클래스 사용한다고 선언.

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , md_mot(this)
    , cobot(this)
    , mb(this)

{
    ui->setupUi(this);

    //btn click rb_5 connection ctrl
    connect(ui->BTN_CONNECT_COM, SIGNAL(clicked()), this, SLOT(BTN_CONNECT_COM()));
    connect(ui->BTN_CONNECT_DATA, SIGNAL(clicked()), this, SLOT(BTN_CONNECT_DATA()));

    connect(&systemTimer, SIGNAL(timeout()), this, SLOT(onSystemCheck()));
    systemTimer.start(200);

    connect(ui->bt_connect, SIGNAL(clicked()), this, SLOT(bt_connect()));
    //    connect(ui->bt_test, SIGNAL(clicked()), this, SLOT(bt_test()));

    // position control
    connect(ui->bt_homing, SIGNAL(clicked()), this, SLOT(bt_homing()));
    connect(ui->bt_auto_homing, SIGNAL(clicked()), this, SLOT(bt_auto_homing()));
    connect(&md_mot, SIGNAL(homingEnd(QString)), this, SLOT(homeEnd(QString)));

    connect(ui->bt_lift_homimg, SIGNAL(clicked()), this, SLOT(bt_homing()));
    connect(ui->bt_move_pos, SIGNAL(clicked()), this, SLOT(bt_move_pos()));
    connect(ui->bt_lift_pos_move, SIGNAL(clicked()), this, SLOT(bt_lift_pos_move()));
    connect(ui->sb_lift_pos, SIGNAL(clicked()), this, SLOT(sb_lift_pos()));
    connect(ui->bt_set_maxVel_pos, SIGNAL(clicked()), this, SLOT(bt_set_maxVel_pos()));
    connect(ui->bt_lift_rpm_set, SIGNAL(clicked()), this, SLOT(bt_lift_maxVel_pos()));

    // RPM
    connect(ui->bt_move_rpm, SIGNAL(clicked()), this, SLOT(bt_move_rpm()));
    connect(ui->bt_rpmZero, SIGNAL(clicked()), this, SLOT(bt_rpmZero()));
    connect(ui->bt_lift_stop, SIGNAL(clicked()), this, SLOT(bt_rpmZero()));


    connect(&md_mot, SIGNAL(rev_DATA(QByteArray)), this, SLOT(rev_data(QByteArray)));
    connect(&md_mot, SIGNAL(write_log(QString)), this, SLOT(write_log(QString)));

    //mobile tcp new connection check
    connect(&mobile_timer, SIGNAL(timeout()), this, SLOT(newConnection_tcp()));
    //    connect(&mobile_timer, SIGNAL(timeout()), this, SLOT(disConnection_tcp()));
    mobile_timer.start(200);

    //    udp_socket.bind(QHostAddress(QString("10.108.1.52")), 4444);
    //ordroid go for lift(jog controll lift)


    connect(&vision, SIGNAL(msgSignal(bool)), this, SLOT(keti_showUI_msg(bool)));


    QString lift_port;
    //    QString port = QString("ttyUSB0");

    Q_FOREACH(QSerialPortInfo port, QSerialPortInfo::availablePorts())
    {
        //        qDebug()<<"port : "<<port.portName();
        bool lift = port.portName().contains("USB", Qt::CaseInsensitive);//이름이 바코드로 정해놓은 것만 들어오도록
        if (lift)
        {
            ui->cb_port->addItem(port.portName());
            lift_port = port.portName();
        }

        bool bar = port.portName().contains("ACM", Qt::CaseInsensitive);//이름이 바코드로 정해놓은 것만 들어오도록
        if (bar)
        {

            //barcode on
            //    qDebug()<<"eeeee : "<<ui->cb_usb1->currentText();
            //    ui->cb_usb_1->currentText();
            //            barcode.bt_usb1_con(port);
            /*
            ui->cb_usb1->addItem(port.portName());
            ui->cb_usb2->addItem(port.portName());*/
            ui->cb_usb1->addItem("ttyBAR0");
            ui->cb_usb2->addItem("ttyBAR1");
        }
    }

    connect(ui->BTN_CONNECT_GRIPPER, SIGNAL(clicked()), this, SLOT(BTN_CONNECT_GRIPPER()));
    //ui on -> ketich communication on!
    // rb_5 , lift
    //순서문제로 인한 불 안들어옴....

    // gripper msg
    connect(&gripper, SIGNAL(msgSignal(bool)), this, SLOT(kitech_showUI_msg(bool)));
    connect(&gripper, SIGNAL(kitech_cmd(QString)), this, SLOT(kitech_msg(QString)));

    if(gripper.connect_flag == false)
    {
        qDebug()<<"not connected";
        //GRIPPER CONNECT
    }

    //    initFlag = 1;

    //for lift motor on check
    connect(&md_mot, SIGNAL(LiftMSG(QString)), this, SLOT(UI_Connection_Update_Lift(QString)));
    if(lift_port.size() != 0)
    {
        //         connect(ui->bt_connect, SIGNAL(clicked()), this, SLOT(bt_connect()));
        md_mot.open(lift_port);
        ui->bt_connect->setEnabled(false);
    }

    connect(ui->BTN_GRIPPER_CMD_WRITE, SIGNAL(clicked()), this, SLOT(BTN_GRIPPER_CMD_WRITE()));
    connect(ui->BTN_GRIPPER_ALLFOLD, SIGNAL(clicked()), this, SLOT(BTN_GRIPPER_ALLFOLD()));
    connect(ui->BTN_GRIPPER_GRASP, SIGNAL(clicked()), this, SLOT(BTN_GRIPPER_GRASP()));
    connect(ui->BTN_GRIPPER_VISION, SIGNAL(clicked()), this, SLOT(BTN_GRIPPER_VISION()));
    connect(ui->CB_GRIPPER_GE_CMD_WRITE, SIGNAL(clicked()), this, SLOT(CB_GRIPPER_GE_CMD_WRITE()));

    connect(ui->Pinch12_FOLD, SIGNAL(clicked()), this, SLOT(Pinch12_FOLD()));

    connect(ui->bt_usb1_con, SIGNAL(clicked()), this, SLOT(bt_usb1_con()));
    connect(ui->bt_usb2_con, SIGNAL(clicked()), this, SLOT(bt_usb2_con()));

    connect(ui->bt_usb1_dis, SIGNAL(clicked()), this, SLOT(bt_usb1_dis()));
    connect(ui->bt_usb2_dis, SIGNAL(clicked()), this, SLOT(bt_usb2_dis()));

    connect(&barcode, SIGNAL(msgSignal(QByteArray)), this, SLOT(barcode1_showUI_msg(QByteArray)));
    connect(&barcode, SIGNAL(msgSignal2(QByteArray)), this, SLOT(barcode2_showUI_msg(QByteArray)));
    connect(&barcode, SIGNAL(barcode_UI(QString)), this, SLOT(barcode_ON_showUI(QString)));

    // ipc start
    connect(&timer_shared_memory, SIGNAL(timeout()), this, SLOT(timer_shared_memory_loop()));
    timer_shared_memory.start(100);

    connect(&updateTimer, SIGNAL(timeout()), this, SLOT(onUpdate()));
    updateTimer.start(100);

    connect(&cobot, SIGNAL(CobotMSG(QString)), this, SLOT(UI_Connection_Update(QString)));

    //img change for yujin
    connect(ui->bt_png_change, SIGNAL(clicked()), this, SLOT(bt_png_change()));

    //zip file
    connect(ui->bt_zip, SIGNAL(clicked()), this, SLOT(bt_zip()));

    //for websocket on check
    connect(&web, SIGNAL(msgSignal(bool)), this, SLOT(showUI_msg(bool)));
    connect(&web, SIGNAL(msgReciveSignal(QString)), this, SLOT(showUI_recive_msg(QString)));
    connect(&web, SIGNAL(msgSendSignal(QString)), this, SLOT(showUI_send_msg(QString)));

    connect(ui->btn_pump, SIGNAL(clicked()), this, SLOT(btn_pump()));
    connect(ui->bt_tcp_on, SIGNAL(clicked()), this, SLOT(bt_tcp_on()));
    //    bt_tcp_on

    connect(&liftTimer, SIGNAL(timeout()), this, SLOT(status_loop()));
    connect(&liftTimer, SIGNAL(timeout()), this, SLOT(bt_lift_status()));
    connect(&liftTimer, SIGNAL(timeout()), this, SLOT(lift_ui()));
    liftTimer.start(100);

    connect(&CobotStatusTimer, SIGNAL(timeout()), this, SLOT(print()));
    CobotStatusTimer.start(300);

    connect(ui->bt_get_map_from_mobile, SIGNAL(clicked()), this, SLOT(bt_get_map_from_mobile()));

    connect(&mb, SIGNAL(mobile_run(QString)), this, SLOT(UI_Connection_Update_mobile(QString)));
    connect(&mb, SIGNAL(mobile_status(QString)), this, SLOT(rb_pose(QString)));

    //    connect(&map_timer, SIGNAL(timeout()), this, SLOT(lb_sn_map()));
    //    map_timer.start(150);

    connect(&mb, SIGNAL(img_signal(cv::Mat)), this, SLOT(lb_sn_map(cv::Mat)));

    mb.init(&md_mot, &cobot);
    web.init(&mb, &md_mot);

    connect(ui->spb_Tx, SIGNAL(valueChanged(double)), this, SLOT(tf_any_change()));
    connect(ui->spb_Ty, SIGNAL(valueChanged(double)), this, SLOT(tf_any_change()));
    connect(ui->spb_Tz, SIGNAL(valueChanged(double)), this, SLOT(tf_any_change()));
    connect(ui->spb_Rx, SIGNAL(valueChanged(double)), this, SLOT(tf_any_change()));
    connect(ui->spb_Ry, SIGNAL(valueChanged(double)), this, SLOT(tf_any_change()));
    connect(ui->spb_Rz, SIGNAL(valueChanged(double)), this, SLOT(tf_any_change()));

    connect(ui->bt_cobot_move2object, SIGNAL(clicked()), this, SLOT(bt_cobot_move2object()));

    time_seq = new QTimer();
    connect(time_seq,SIGNAL(timeout()),this,SLOT(seqLoop()));
    time_seq->start(100);

    connect(time_seq,SIGNAL(timeout()),this,SLOT(yujin_loop()));

    double tx = ui->spb_Tx->value(); // m
    double ty = ui->spb_Ty->value();
    double tz = ui->spb_Tz->value();

    double rz = ui->spb_Rz->value(); // deg
    double ry = ui->spb_Ry->value();
    double rx = ui->spb_Rx->value();

    QString str; //qstring에 sprintf 로 한방에 집어넣을 수 있음.
    str.sprintf("%f,%f,%f,%f,%f,%f", tx, ty, tz, rz, ry, rx);

    T_sensor = string_to_TF(str); //translate matrix

    set_ui_item_from_unimap();

    connect(ui->bt_amr_motor_init, SIGNAL(clicked()), this, SLOT(bt_amr_motor_init()));

    // save auto annotation
    connect(ui->bt_save_annot, SIGNAL(clicked()), this, SLOT(bt_save_annot()));
    connect(ui->bt_save_all_annot, SIGNAL(clicked()), this, SLOT(bt_save_all_annot()));
    connect(ui->bt_delete_annot, SIGNAL(clicked()), this, SLOT(bt_delete_annot()));

    connect(ui->bt_save_annot_2, SIGNAL(clicked()), this, SLOT(bt_save_annot_2()));
    connect(ui->bt_save_all_annot_2, SIGNAL(clicked()), this, SLOT(bt_save_all_annot_2()));
    connect(ui->bt_delete_annot_2, SIGNAL(clicked()), this, SLOT(bt_delete_annot_2()));

    // order in
    connect(ui->bt_order, SIGNAL(clicked()), this, SLOT(bt_order()));

    connect(&vision, SIGNAL(keti_img_signal(cv::Mat)), this, SLOT(LE_KETI_IMG(cv::Mat)));
    connect(&vision, SIGNAL(keti_img_point(QString)), this, SLOT(lb_keti_point(QString)));

    connect(ui->BTN_GRIPPER_recog_WRITE, SIGNAL(clicked()), this, SLOT(BTN_GRIPPER_recog_WRITE()));

    connect(&vision_capture_timer,SIGNAL(timeout()),this,SLOT(vision_img_capture()));
    vision_capture_timer.start(100);

    connect(&auto_homming_timer,SIGNAL(timeout()),this,SLOT(auto_homming_seq()));
    auto_homming_timer.start(100);

    connect(&web, SIGNAL(order_pass(QJsonObject)), this, SLOT(yujin_order(QJsonObject)));

    init();

    // 기본
    vision.TCP2cam = cam_config.RB_TF_SENSOR;

//    QString aaa = QString::number(172, 16);
//    qDebug()<<aaa;
//    const char* p = aaa.toStdString().c_str();
//    qDebug()<<p;
    //    connect(&vision_trigger_timer,SIGNAL(timeout()),this,SLOT(vision_trigger()));
    //    vision_trigger_timer.start(5000);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::init()
{
    le_digital_in_on[0]  = ui->LE_D_IN_ON_0;
    le_digital_in_on[1]  = ui->LE_D_IN_ON_1;
    le_digital_in_on[2]  = ui->LE_D_IN_ON_2;
    le_digital_in_on[3]  = ui->LE_D_IN_ON_3;
    le_digital_in_on[4]  = ui->LE_D_IN_ON_4;
    le_digital_in_on[5]  = ui->LE_D_IN_ON_5;
    le_digital_in_on[6]  = ui->LE_D_IN_ON_6;
    le_digital_in_on[7]  = ui->LE_D_IN_ON_7;
    le_digital_in_on[8]  = ui->LE_D_IN_ON_8;
    le_digital_in_on[9]  = ui->LE_D_IN_ON_9;
    le_digital_in_on[10] = ui->LE_D_IN_ON_10;
    le_digital_in_on[11] = ui->LE_D_IN_ON_11;
    le_digital_in_on[12] = ui->LE_D_IN_ON_12;
    le_digital_in_on[13] = ui->LE_D_IN_ON_13;
    le_digital_in_on[14] = ui->LE_D_IN_ON_14;
    le_digital_in_on[15] = ui->LE_D_IN_ON_15;

    le_digital_in_off[0]  = ui->LE_D_IN_OFF_0;
    le_digital_in_off[1]  = ui->LE_D_IN_OFF_1;
    le_digital_in_off[2]  = ui->LE_D_IN_OFF_2;
    le_digital_in_off[3]  = ui->LE_D_IN_OFF_3;
    le_digital_in_off[4]  = ui->LE_D_IN_OFF_4;
    le_digital_in_off[5]  = ui->LE_D_IN_OFF_5;
    le_digital_in_off[6]  = ui->LE_D_IN_OFF_6;
    le_digital_in_off[7]  = ui->LE_D_IN_OFF_7;
    le_digital_in_off[8]  = ui->LE_D_IN_OFF_8;
    le_digital_in_off[9]  = ui->LE_D_IN_OFF_9;
    le_digital_in_off[10] = ui->LE_D_IN_OFF_10;
    le_digital_in_off[11] = ui->LE_D_IN_OFF_11;
    le_digital_in_off[12] = ui->LE_D_IN_OFF_12;
    le_digital_in_off[13] = ui->LE_D_IN_OFF_13;
    le_digital_in_off[14] = ui->LE_D_IN_OFF_14;
    le_digital_in_off[15] = ui->LE_D_IN_OFF_15;

    le_digital_out_on[0]  = ui->LE_D_OUT_ON_0;
    le_digital_out_on[1]  = ui->LE_D_OUT_ON_1;
    le_digital_out_on[2]  = ui->LE_D_OUT_ON_2;
    le_digital_out_on[3]  = ui->LE_D_OUT_ON_3;
    le_digital_out_on[4]  = ui->LE_D_OUT_ON_4;
    le_digital_out_on[5]  = ui->LE_D_OUT_ON_5;
    le_digital_out_on[6]  = ui->LE_D_OUT_ON_6;
    le_digital_out_on[7]  = ui->LE_D_OUT_ON_7;
    le_digital_out_on[8]  = ui->LE_D_OUT_ON_8;
    le_digital_out_on[9]  = ui->LE_D_OUT_ON_9;
    le_digital_out_on[10] = ui->LE_D_OUT_ON_10;
    le_digital_out_on[11] = ui->LE_D_OUT_ON_11;
    le_digital_out_on[12] = ui->LE_D_OUT_ON_12;
    le_digital_out_on[13] = ui->LE_D_OUT_ON_13;
    le_digital_out_on[14] = ui->LE_D_OUT_ON_14;
    le_digital_out_on[15] = ui->LE_D_OUT_ON_15;

    le_digital_out_off[0]  = ui->LE_D_OUT_OFF_0;
    le_digital_out_off[1]  = ui->LE_D_OUT_OFF_1;
    le_digital_out_off[2]  = ui->LE_D_OUT_OFF_2;
    le_digital_out_off[3]  = ui->LE_D_OUT_OFF_3;
    le_digital_out_off[4]  = ui->LE_D_OUT_OFF_4;
    le_digital_out_off[5]  = ui->LE_D_OUT_OFF_5;
    le_digital_out_off[6]  = ui->LE_D_OUT_OFF_6;
    le_digital_out_off[7]  = ui->LE_D_OUT_OFF_7;
    le_digital_out_off[8]  = ui->LE_D_OUT_OFF_8;
    le_digital_out_off[9]  = ui->LE_D_OUT_OFF_9;
    le_digital_out_off[10] = ui->LE_D_OUT_OFF_10;
    le_digital_out_off[11] = ui->LE_D_OUT_OFF_11;
    le_digital_out_off[12] = ui->LE_D_OUT_OFF_12;
    le_digital_out_off[13] = ui->LE_D_OUT_OFF_13;
    le_digital_out_off[14] = ui->LE_D_OUT_OFF_14;
    le_digital_out_off[15] = ui->LE_D_OUT_OFF_15;

    foreach(QString list, robot_config.object_list)
    {
        //do whatever you need to do
        ui->cb_get_object_id->addItem(list);
        ui->cb_get_object_id_2->addItem(list);
        ui->cb_get_object_id_vision->addItem(list);
    }

    foreach(QString list, robot_config.shelves_list)
    {
        //do whatever you need to do
        ui->cb_shelf_id->addItem(list);
        ui->CB_shelf->addItem(list);
    }

    load();
    ui->le_scenario->setStyleSheet("QLineEdit{background-color:white}");
}


void MainWindow::map_size_load()
{
    QString command = "du -hs /home/rainbow/RB_MOBILE/maps";

    QProcess process;
    process.start(command);
    process.waitForFinished(-1); // Wait until the process is finished

    QByteArray result = process.readAllStandardOutput();
    QString resultString = QString::fromUtf8(result).trimmed(); // Convert to QString and trim whitespace

    qDebug() << "Result:" << resultString;
}

void MainWindow::load()
{
    QString shelf_info_path = QDir::homePath() + "/order_json/shelf_infomation.json";

    QFile file(shelf_info_path);
    if(file.open(QIODevice::ReadOnly))
    {
        QByteArray data = file.readAll();
        QJsonDocument doc = QJsonDocument::fromJson(data);

        QJsonArray arr = doc.array();
        foreach(const QJsonValue &val, arr)
        {
            // shelf's name = original name + box left/right + lift pose
            SHELF_INFO *shelf_info = new SHELF_INFO();
            QJsonObject obj = val.toObject();
            shelf_info->shelf_id = obj["shelf_id"].toString()+"_"+ obj["lift_pose"].toString();
            shelf_info->lift_pose = obj["lift_pose"].toString().toDouble();
            shelf_info->AMR_pose = array_to_pose(obj["AMR_pose"].toArray());
            shelf_info->RB_5_pose = array_to_pose6(obj["RB_5_pose"].toArray());
            //            qDebug()<<"lift : "<<shelf_info->lift_pose;

            shelf_infos[shelf_info->shelf_id.toStdString()] = shelf_info;

            web.shelf_infos = shelf_infos;
        }

        file.close();
    }

    QString object_info_path = QDir::homePath() + "/order_json/object_infomation.json";

    QFile obj_file(object_info_path);
    if(obj_file.open(QIODevice::ReadOnly))
    {
        QByteArray data = obj_file.readAll();
        QJsonDocument doc = QJsonDocument::fromJson(data);

        QJsonArray arr = doc.array();
        foreach(const QJsonValue &val, arr)
        {
            OBJECT_INFO *object_info = new OBJECT_INFO();
            QJsonObject obj = val.toObject();
            object_info->obj_id = obj["obj_id"].toString();
            object_info->obj_grap_pose = obj["real_Gripping"].toString();
            object_info->obj_ready_grap_pose = obj["ready_Gripping"].toString();

            object_infos[object_info->obj_id.toStdString()] = object_info;
            web.object_infos = object_infos;
        }

        obj_file.close();
    }
}

void MainWindow::tf_any_change()
{
    double tx = ui->spb_Tx->value(); // m
    double ty = ui->spb_Ty->value();
    double tz = ui->spb_Tz->value();
    double rx = ui->spb_Rx->value(); // deg
    double ry = ui->spb_Ry->value();
    double rz = ui->spb_Rz->value();

    QString str; //qstring에 sprintf 로 한방에 집어넣을 수 있음.
    str.sprintf("%f,%f,%f,%f,%f,%f", tx, ty, tz, rx, ry, rz);

    T_sensor = string_to_TF(str); //translate matrix

    //    std::cout<<T_sensor<<std::endl; //cout translate metrix
    Eigen::Vector3d P(keti_x, keti_y, keti_z);
    Eigen::Vector3d _P = T_sensor.block(0,0,3,3)*P+T_sensor.block(0,3,3,1);

    //    std::cout<<_P[0]<<_P[1]<<_P[2]<<std::endl;
    QString res_x,res_y,res_z;
    res_x.sprintf("%.f",(_P[0])*1000);
    res_y.sprintf("%.f",(_P[1])*1000);
    res_z.sprintf("%.f",(_P[2])*1000);

    ui->move_x_val->setText(res_x);
    ui->move_y_val->setText(res_y);
    ui->move_z_val->setText(res_z);
}

void MainWindow::lb_sn_map(cv::Mat map_img)
{
    //    계속 업데이트 되고 있음.
    ui->lb_sn_map->map.setPixmap(QPixmap::fromImage(mat_to_qimage_cpy(map_img)));

}

void MainWindow::bt_tcp_on()
{
    mb.mobile_Connect();
}

void MainWindow::bt_get_map_from_mobile()
{
    qDebug()<<"get map from mobile robot";
    QString msg = ui->cb_get_map_data->currentText();
    mb.getData(msg);

}

void MainWindow::kitech_msg(QString msg)
{
    //이전 명령을 체크한 후 그냥 손 오므리기가 성공인지 잡은것이 성공인지 확인해야함.
    if (msg == "0")
    {
        msg = "grip fail";
        grip = false;
        ui->le_pick->setStyleSheet("QLineEdit{background-color:green}");
    }
    else if(msg == "1")
    {
        msg = "grip success";
        grip = true;
        ui->le_pick->setStyleSheet("QLineEdit{background-color:red}");
    }
    ui ->notice_kitech ->setText(msg);
}

void MainWindow::status_loop()
{
    md_mot.req_data();
}

void MainWindow::btn_pump()
{
    if (ui->btn_pump->text()=="Pump on")
    {
        cobot.ControlBoxDigitalOut(1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
        ui->btn_pump->setText("Pump off");
        ui->btn_pump_2->setText("Pump off");
        ui->le_rb5_move_status->setStyleSheet("QLineEdit{background-color:red}");
    }
    else if(ui->btn_pump->text()=="Pump off")
    {
        cobot.ControlBoxDigitalOut(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
        ui->btn_pump->setText("Pump on");
        ui->btn_pump_2->setText("Pump on");
        ui->le_rb5_move_status->setStyleSheet("QLineEdit{background-color:green}");
    }
}

void MainWindow::lift_ui()
{
    static int lift_timeout = 0;

    //    qDebug()<<"md_mot.move_poisition_flag : "<<md_mot.move_poisition_flag;
    //    qDebug()<<"lift_clicked : "<<lift_clicked;
    if(md_mot.move_poisition_flag)
    {
        lift_clicked = lift_start;
        md_mot.move_poisition_flag = false;
    }
    else
    {
        //        lift_clicked = lift_done;
    }
    //    qDebug()<<"lift_clicked : "<<lift_clicked;

    if(lift_clicked == lift_start)
    {
        web.lift_status = "working";
        qDebug()<<"lift is working";
        lift_timeout = 500/100;
        lift_clicked = lift_wait;
    }

    else if(lift_clicked == lift_wait)
    {
        if(--lift_timeout > 0)
        {
            //            qDebug()<<lift_timeout;
            lift_clicked = lift_wait;
        }
        else
        {
            lift_clicked = lift_working;
        }
    }
    else if(lift_clicked == lift_working)
    {
        if(lift_pos == old_lift_pos)
        {
            //            web.lift_status = false;
            ui -> le_lift_move_status->setStyleSheet("QLineEdit{background-color:green}");
            //        web.move_flag =false;
            web.lift_status = "not moving";
            lift_state = "done";
            lift_clicked = lift_done;
        }
        else if(lift_pos != old_lift_pos)
        {
            //            web.lift_status = true;
            ui -> le_lift_move_status->setStyleSheet("QLineEdit{background-color:red}");
            //        web.move_flag =false;
            web.lift_status = "moving";
            lift_state = "working";
            lift_clicked = lift_working;
            mb.liftState(lift_state);
        }
        old_lift_pos = lift_pos;
    }
    else if(lift_clicked == lift_done)
    {
        if(lift_pos == old_lift_pos)
        {
            //            web.lift_status = true;
            ui -> le_lift_move_status->setStyleSheet("QLineEdit{background-color:green}");
            //        web.move_flag =false;
            web.lift_status = "not moving";
            lift_state = "done";
            lift_clicked = lift_done;
            mb.liftState(lift_state);
            //            md_mot.move_poisition_flag =false;
        }
    }
}

void MainWindow::barcode_ON_showUI(QString msg)
{
    if (msg == "all usb is on")
    {
        ui->le_barcode_connection->setStyleSheet("QLineEdit{background-color:green}");
    }
    else
    {
        ui->le_barcode_connection->setStyleSheet("QLineEdit{background-color:red}");
    }
}

void MainWindow::showUI_msg(bool msg)
{
    if (msg == true)
    {
        ui->te_label->setText(QString("FMS Client Connected.."));
        ui->le_websocket_connection->setStyleSheet("QLineEdit{background-color:green}");
    }

    else
    {
        ui->te_label->setText(QString("FMS Client Disconnect.."));
        ui->le_websocket_connection->setStyleSheet("QLineEdit{background-color:red}");
    }
}

void MainWindow::showUI_recive_msg(QString message)
{
    ui->te_msg->append(QString(message));
    ui->te_msg->append("");

    ui->te_websocket->append(QString(message));
    ui->te_websocket->append("");
}

void MainWindow::showUI_send_msg(QString message)
{
    ui->te_send_msg->setText(QString(message));
}

void MainWindow::UI_Connection_Update(QString msg)
{
    if (msg == "cmd Disconnect")
    {
        ui->BTN_CONNECT_COM->setText("Connect");
        ui->le_rb5_cmd_connection->setStyleSheet("QLineEdit{background-color:red}");
    }

    else if (msg == "cmd connected")
    {
        ui->BTN_CONNECT_COM->setText("Disconnect");
        ui->le_rb5_cmd_connection->setStyleSheet("QLineEdit{background-color:green}");
    }

    else if (msg == "data Disconnect")
    {
        ui->BTN_CONNECT_DATA->setText("Connect");
        ui->le_rb5_data_connection->setStyleSheet("QLineEdit{background-color:red}");
    }

    else if (msg == "data connected")
    {
        ui->BTN_CONNECT_DATA->setText("Disconnect");
        ui->le_rb5_data_connection->setStyleSheet("QLineEdit{background-color:green}");
    }

}


void MainWindow::UI_Connection_Update_Lift(QString msg)
{

    if (msg == "false")
    {
        ui->le_lift_connection->setStyleSheet("QLineEdit{background-color:red}");
    }

    else if (msg == "true")
    {
        ui->le_lift_connection->setStyleSheet("QLineEdit{background-color:green}");
    }
}

void MainWindow::UI_Connection_Update_mobile(QString run)
{
    //    qDebug()<<"mobile run :"<<run;
    if (run == "false")
    {
        mobile_moving_flag = false;
        ui->le_mobile_move_status->setStyleSheet("QLineEdit{background-color:green}");
    }

    else if (run == "true")
    {
        mobile_moving_flag = true;
        ui->le_mobile_move_status->setStyleSheet("QLineEdit{background-color:red}");
    }
    //    qDebug()<<"uiuiuiui mobile_moving_flag : "<<mobile_moving_flag;
}

void MainWindow::rb_pose(QString pose_msg)
{
    ui->te_log->setText(pose_msg);
}


void MainWindow::onUpdate()
{
    //        qDebug() << "***************** program_mode " <<  cobot.systemStat.sdata.robot_state ;
    {
        if(cobot.systemStat.sdata.program_mode == 0){
            ui->LE_PG_MODE_REAL->setStyleSheet("QLineEdit{background-color:green}");
            //cobot 이 realmode 가 되는 순간 속도를 50%로 변경.
            if (cobot.realflag)
            {
                cobot.BaseSpeedChange(0.5);
                cobot.realflag = false;
            }
            ui->LE_PG_MODE_SIMULATION->setStyleSheet("QLineEdit{background-color:white}");
        }
        else if(cobot.systemStat.sdata.program_mode == 1)
        {
            ui->LE_PG_MODE_REAL->setStyleSheet("QLineEdit{background-color:white}");
            ui->LE_PG_MODE_SIMULATION->setStyleSheet("QLineEdit{background-color:green}");
        }
        else
        {
            ui->LE_PG_MODE_REAL->setStyleSheet("QLineEdit{background-color:white}");
            ui->LE_PG_MODE_SIMULATION->setStyleSheet("QLineEdit{background-color:white}");
        }

        if(cobot.systemStat.sdata.robot_state == 1)
        {
            //            web.move_flag = false;
            web.rb_status = "not moving";
            ui->LE_ROBOT_STATE_IDLE->setStyleSheet("QLineEdit{background-color:green}");
            ui->LE_ROBOT_STATE_MOVING->setStyleSheet("QLineEdit{background-color:white}");
            ui->le_rb5_move_status->setStyleSheet("QLineEdit{background-color:green}");

            if(cobot.systemStat.sdata.robot_state != old_robot_state)
            {
                on_pb_reload_clicked();
            }

        }
        else if(cobot.systemStat.sdata.robot_state == 2)
        {
            //            web.move_flag = false;
            web.rb_status = "not moving";
            ui->LE_ROBOT_STATE_IDLE->setStyleSheet("QLineEdit{background-color:white}");
            ui->LE_ROBOT_STATE_MOVING->setStyleSheet("QLineEdit{background-color:white}");
            ui->le_rb5_move_status->setStyleSheet("QLineEdit{background-color:green}");
        }
        else if(cobot.systemStat.sdata.robot_state == 3)
        {
            //            web.move_flag = true;
            web.rb_status = "moving";
            ui->LE_ROBOT_STATE_IDLE->setStyleSheet("QLineEdit{background-color:white}");
            ui->LE_ROBOT_STATE_MOVING->setStyleSheet("QLineEdit{background-color:green}");
            ui->le_rb5_move_status->setStyleSheet("QLineEdit{background-color:red}");
        }
        else
        {
            web.rb_status = "not moving";
            web.rb5_move_flag = false;
            ui->LE_ROBOT_STATE_IDLE->setStyleSheet("QLineEdit{background-color:white}");
            ui->LE_ROBOT_STATE_MOVING->setStyleSheet("QLineEdit{background-color:white}");
            ui->le_rb5_move_status->setStyleSheet("QLineEdit{background-color:green}");
        }


        if(cobot.systemStat.sdata.is_freedrive_mode == 1){
            ui->LE_ROBOT_STATUS_TEACHING->setStyleSheet("QLineEdit{background-color:green}");
        }else{
            ui->LE_ROBOT_STATUS_TEACHING->setStyleSheet("QLineEdit{background-color:white}");
        }
        if(cobot.systemStat.sdata.op_stat_collision_occur == 1){
            ui->LE_ROBOT_STATUS_EXT_COLLISION->setStyleSheet("QLineEdit{background-color:red}");
        }else{
            ui->LE_ROBOT_STATUS_EXT_COLLISION->setStyleSheet("QLineEdit{background-color:white}");
        }
        if(cobot.systemStat.sdata.op_stat_self_collision == 1){
            ui->LE_ROBOT_STATUS_SELF_COLLISION->setStyleSheet("QLineEdit{background-color:red}");
        }else{
            ui->LE_ROBOT_STATUS_SELF_COLLISION->setStyleSheet("QLineEdit{background-color:white}");
        }
        if(cobot.systemStat.sdata.op_stat_soft_estop_occur == 1){
            ui->LE_ROBOT_STATUS_PAUSED->setStyleSheet("QLineEdit{background-color:red}");
            web.rb_pause_status = "paused";
        }else{
            ui->LE_ROBOT_STATUS_PAUSED->setStyleSheet("QLineEdit{background-color:white}");
            web.rb_pause_status = "resume";
        }
        if(cobot.systemStat.sdata.op_stat_ems_flag != 0){
            ui->LE_ROBOT_STATUS_EMS->setStyleSheet("QLineEdit{background-color:red}");
        }else{
            ui->LE_ROBOT_STATUS_EMS->setStyleSheet("QLineEdit{background-color:white}");
        }
        if(cobot.systemStat.sdata.op_stat_sos_flag == 1){
            ui->LE_ROBOT_STATUS_SOS->setStyleSheet("QLineEdit{background-color:red}");
        }else{
            ui->LE_ROBOT_STATUS_SOS->setStyleSheet("QLineEdit{background-color:white}");
        }

        ui->LE_JNT_ENC_1->setText(QString().sprintf("%.3f", cobot.systemStat.sdata.jnt_ang[0]));
        ui->LE_JNT_ENC_2->setText(QString().sprintf("%.3f", cobot.systemStat.sdata.jnt_ang[1]));
        ui->LE_JNT_ENC_3->setText(QString().sprintf("%.3f", cobot.systemStat.sdata.jnt_ang[2]));
        ui->LE_JNT_ENC_4->setText(QString().sprintf("%.3f", cobot.systemStat.sdata.jnt_ang[3]));
        ui->LE_JNT_ENC_5->setText(QString().sprintf("%.3f", cobot.systemStat.sdata.jnt_ang[4]));
        ui->LE_JNT_ENC_6->setText(QString().sprintf("%.3f", cobot.systemStat.sdata.jnt_ang[5]));

        ui->LE_JNT_REF_1->setText(QString().sprintf("%.3f", cobot.systemStat.sdata.jnt_ref[0]));
        ui->LE_JNT_REF_2->setText(QString().sprintf("%.3f", cobot.systemStat.sdata.jnt_ref[1]));
        ui->LE_JNT_REF_3->setText(QString().sprintf("%.3f", cobot.systemStat.sdata.jnt_ref[2]));
        ui->LE_JNT_REF_4->setText(QString().sprintf("%.3f", cobot.systemStat.sdata.jnt_ref[3]));
        ui->LE_JNT_REF_5->setText(QString().sprintf("%.3f", cobot.systemStat.sdata.jnt_ref[4]));
        ui->LE_JNT_REF_6->setText(QString().sprintf("%.3f", cobot.systemStat.sdata.jnt_ref[5]));

        ui->LE_TCP_REF_X->setText(QString().sprintf("%.3f", cobot.systemStat.sdata.tcp_ref[0]));
        ui->LE_TCP_REF_Y->setText(QString().sprintf("%.3f", cobot.systemStat.sdata.tcp_ref[1]));
        ui->LE_TCP_REF_Z->setText(QString().sprintf("%.3f", cobot.systemStat.sdata.tcp_ref[2]));
        ui->LE_TCP_REF_RX->setText(QString().sprintf("%.3f",cobot.systemStat.sdata.tcp_ref[3]));
        ui->LE_TCP_REF_RY->setText(QString().sprintf("%.3f",cobot.systemStat.sdata.tcp_ref[4]));
        ui->LE_TCP_REF_RZ->setText(QString().sprintf("%.3f",cobot.systemStat.sdata.tcp_ref[5]));


        //        ui -> spb_Rz2->setValue(-1*(90+cobot.systemStat.sdata.tcp_ref[4]));

        if(first_time==1)
        {
            ui -> spb_Tx->setValue(ui->LE_TCP_REF_X->text().toFloat()/1000);
            ui -> spb_Ty->setValue(ui->LE_TCP_REF_Y->text().toFloat()/1000);
            ui -> spb_Tz->setValue(ui->LE_TCP_REF_Z->text().toFloat()/1000);

            ui -> spb_Rx->setValue(ui->LE_TCP_REF_RX->text().toFloat());
            ui -> spb_Ry->setValue(ui->LE_TCP_REF_RY->text().toFloat());
            ui -> spb_Rz->setValue(ui->LE_TCP_REF_RZ->text().toFloat());

            first_time+=1;
        }

        for(int i=0; i<16; i++){
            if(cobot.systemStat.sdata.digital_in[i] == 0){
                le_digital_in_off[i]->setStyleSheet("QLineEdit{background-color:green}");
                le_digital_in_on[i]->setStyleSheet("QLineEdit{background-color:white}");
                if(i == 12)
                {
                    ui->le_pick->setStyleSheet("QLineEdit{background-color:green}");
                    //                    bool ret = gripper.contains("red", Qt::CaseInsensitive);
                    //                    qDebug()<<ret;
                }
            }else if(cobot.systemStat.sdata.digital_in[i] == 1){
                le_digital_in_off[i]->setStyleSheet("QLineEdit{background-color:white}");
                le_digital_in_on[i]->setStyleSheet("QLineEdit{background-color:green}");
                if(i == 12)
                {
                    ui->le_pick->setStyleSheet("QLineEdit{background-color:red}");
                    //                    qDebug()<<ui->le_pick->styleSheet();
                }
            }else{
                le_digital_in_off[i]->setStyleSheet("QLineEdit{background-color:white}");
                le_digital_in_on[i]->setStyleSheet("QLineEdit{background-color:white}");
            }

            if(cobot.systemStat.sdata.digital_out[i] == 0){
                le_digital_out_off[i]->setStyleSheet("QLineEdit{background-color:green}");
                le_digital_out_on[i]->setStyleSheet("QLineEdit{background-color:white}");
                if(i == 0)
                {
                    //                    qDebug()<<"pump is not working";
                    ui->btn_pump->setText("Pump on");
                    ui->btn_pump_2->setText("Pump on");
                    //                    ui->le_rb5_move_status->setStyleSheet("QLineEdit{background-color:green}");
                    //                    qDebug()<<ui->le_pick->styleSheet();
                }

            }else if(cobot.systemStat.sdata.digital_out[i] == 1){
                le_digital_out_off[i]->setStyleSheet("QLineEdit{background-color:white}");
                le_digital_out_on[i]->setStyleSheet("QLineEdit{background-color:green}");
                if(i == 0)
                {
                    //                    qDebug()<<"pump is working";
                    ui->btn_pump->setText("Pump off");
                    ui->btn_pump_2->setText("Pump off");
                    //                    ui->le_rb5_move_status->setStyleSheet("QLineEdit{background-color:red}");
                    //                    qDebug()<<ui->le_pick->styleSheet();
                }

            }else{
                le_digital_out_off[i]->setStyleSheet("QLineEdit{background-color:white}");
                le_digital_out_on[i]->setStyleSheet("QLineEdit{background-color:white}");
            }
        }

        ui->LE_A_IN_0->setText(QString().sprintf("%.3f", cobot.systemStat.sdata.analog_in[0]));
        ui->LE_A_IN_1->setText(QString().sprintf("%.3f", cobot.systemStat.sdata.analog_in[1]));
        ui->LE_A_IN_2->setText(QString().sprintf("%.3f", cobot.systemStat.sdata.analog_in[2]));
        ui->LE_A_IN_3->setText(QString().sprintf("%.3f", cobot.systemStat.sdata.analog_in[3]));

        ui->LE_A_OUT_0->setText(QString().sprintf("%.3f", cobot.systemStat.sdata.analog_out[0]));
        ui->LE_A_OUT_1->setText(QString().sprintf("%.3f", cobot.systemStat.sdata.analog_out[1]));
        ui->LE_A_OUT_2->setText(QString().sprintf("%.3f", cobot.systemStat.sdata.analog_out[2]));
        ui->LE_A_OUT_3->setText(QString().sprintf("%.3f", cobot.systemStat.sdata.analog_out[3]));



        if(fabs(cobot.systemStat.sdata.tfb_voltage_out) < 0.01){
            ui->LE_TV_0->setStyleSheet("QLineEdit{background-color:green}");
            ui->LE_TV_12->setStyleSheet("QLineEdit{background-color:white}");
            ui->LE_TV_24->setStyleSheet("QLineEdit{background-color:white}");
        }else if(fabs(cobot.systemStat.sdata.tfb_voltage_out-12.0) < 0.01){
            ui->LE_TV_0->setStyleSheet("QLineEdit{background-color:white}");
            ui->LE_TV_12->setStyleSheet("QLineEdit{background-color:green}");
            ui->LE_TV_24->setStyleSheet("QLineEdit{background-color:white}");
        }else if(fabs(cobot.systemStat.sdata.tfb_voltage_out-24.0) < 0.01){
            ui->LE_TV_0->setStyleSheet("QLineEdit{background-color:white}");
            ui->LE_TV_12->setStyleSheet("QLineEdit{background-color:white}");
            ui->LE_TV_24->setStyleSheet("QLineEdit{background-color:green}");
        }else{
            ui->LE_TV_0->setStyleSheet("QLineEdit{background-color:white}");
            ui->LE_TV_12->setStyleSheet("QLineEdit{background-color:white}");
            ui->LE_TV_24->setStyleSheet("QLineEdit{background-color:white}");
        }


        if(cobot.systemStat.sdata.tfb_digital_out[0] == 0){
            ui->LE_TD_OUT_ON_0->setStyleSheet("QLineEdit{background-color:white}");
            ui->LE_TD_OUT_OFF_0->setStyleSheet("QLineEdit{background-color:green}");
        }else if(cobot.systemStat.sdata.tfb_digital_out[0] == 1){
            ui->LE_TD_OUT_ON_0->setStyleSheet("QLineEdit{background-color:green}");
            ui->LE_TD_OUT_OFF_0->setStyleSheet("QLineEdit{background-color:white}");
        }else{
            ui->LE_TD_OUT_ON_0->setStyleSheet("QLineEdit{background-color:white}");
            ui->LE_TD_OUT_OFF_0->setStyleSheet("QLineEdit{background-color:white}");
        }
        if(cobot.systemStat.sdata.tfb_digital_out[1] == 0){
            ui->LE_TD_OUT_ON_1->setStyleSheet("QLineEdit{background-color:white}");
            ui->LE_TD_OUT_OFF_1->setStyleSheet("QLineEdit{background-color:green}");
        }else if(cobot.systemStat.sdata.tfb_digital_out[1] == 1){
            ui->LE_TD_OUT_ON_1->setStyleSheet("QLineEdit{background-color:green}");
            ui->LE_TD_OUT_OFF_1->setStyleSheet("QLineEdit{background-color:white}");
        }else{
            ui->LE_TD_OUT_ON_1->setStyleSheet("QLineEdit{background-color:white}");
            ui->LE_TD_OUT_OFF_1->setStyleSheet("QLineEdit{background-color:white}");
        }


        if(cobot.systemStat.sdata.tfb_digital_in[0] == 0){
            ui->LE_TD_IN_ON_0->setStyleSheet("QLineEdit{background-color:white}");
            ui->LE_TD_IN_OFF_0->setStyleSheet("QLineEdit{background-color:green}");
        }else if(cobot.systemStat.sdata.tfb_digital_in[0] == 1){
            ui->LE_TD_IN_ON_0->setStyleSheet("QLineEdit{background-color:green}");
            ui->LE_TD_IN_OFF_0->setStyleSheet("QLineEdit{background-color:white}");
        }else{
            ui->LE_TD_IN_ON_0->setStyleSheet("QLineEdit{background-color:white}");
            ui->LE_TD_IN_OFF_0->setStyleSheet("QLineEdit{background-color:white}");
        }
        if(cobot.systemStat.sdata.tfb_digital_in[1] == 0){
            ui->LE_TD_IN_ON_1->setStyleSheet("QLineEdit{background-color:white}");
            ui->LE_TD_IN_OFF_1->setStyleSheet("QLineEdit{background-color:green}");
        }else if(cobot.systemStat.sdata.tfb_digital_in[1] == 1){
            ui->LE_TD_IN_ON_1->setStyleSheet("QLineEdit{background-color:green}");
            ui->LE_TD_IN_OFF_1->setStyleSheet("QLineEdit{background-color:white}");
        }else{
            ui->LE_TD_IN_ON_1->setStyleSheet("QLineEdit{background-color:white}");
            ui->LE_TD_IN_OFF_1->setStyleSheet("QLineEdit{background-color:white}");
        }
        ui->LE_TA_IN_0->setText(QString().sprintf("%.3f", cobot.systemStat.sdata.tfb_analog_in[0]));
        ui->LE_TA_IN_1->setText(QString().sprintf("%.3f", cobot.systemStat.sdata.tfb_analog_in[1]));
    }
    old_robot_state = cobot.systemStat.sdata.robot_state;
}

void MainWindow::newConnection_tcp()
{
    if (mb.connect_flag)
    {
        ui->le_mobile_connection->setStyleSheet("QLineEdit{background-color:green}");
    }
    else
    {
        ui->le_mobile_connection->setStyleSheet("QLineEdit{background-color:red}");
    }
}

//void MainWindow::readyRead_udp()
//// using joystick
//// for using odroid go -> move lift
//{
//    if(udp_socket.hasPendingDatagrams())
//    {
//        QByteArray _buf;
//        _buf.resize(udp_socket.pendingDatagramSize());

//        QHostAddress sender; quint16 senderPort;
//        udp_socket.readDatagram(_buf.data(), _buf.size(), &sender, &senderPort);

//        if(_buf.size() > 0)
//        {
//            buf.append(_buf);

//            if(buf.size() < 2)
//            {
//                return;
//            }

//            bool is_header = false;
//            for(int p = 0; p < buf.size()-1; p++)
//            {
//                // header check
//                if(buf[p] == (char)0xFF && buf[p+1] == (char)0xFD)
//                {
//                    buf.remove(0, p);
//                    is_header = true;
//                    break;
//                }
//            }

//            // 11 3*4 +8 20 25
//            const int packet_size = 28;
//            if(is_header && buf.size() >= packet_size)
//            {
//                if(buf[packet_size-2] == (char)0x01 && buf[packet_size-1] == (char)0x02)
//                {
//                    uchar* body = (uchar*)buf.data();
//                    // body parsing
//                    //uchar id = body[2];
//                    float lx, ly, rx, ry; uint8_t btn[8];

//                    memcpy(&lx, &body[2], 4);
//                    memcpy(&ly, &body[6], 4);
//                    memcpy(&rx, &body[10], 4);
//                    memcpy(&ry, &body[14], 4);
//                    memcpy(&btn, &body[18], 8);

//                    // remove current packet
//                    buf.remove(0, packet_size);

//                    if(btn[6] == 1)
//                    {
//                        md_mot.move_rpm(1000);
//                    }
//                    else if(btn[7] == 1)
//                    {
//                        md_mot.move_rpm(-1000);
//                    }
//                    else if(btn[6] == 0 && btn[7] == 0)
//                    {
//                        md_mot.move_rpm(0);
//                    }
//                }
//            }
//        }
//    }
//}

void MainWindow::bt_connect()
{
    QString port = ui->cb_port->currentText();
    if(port.size() != 0)
    {
        md_mot.open(port);
    }
}

void MainWindow::bt_homing()
{
    if(!md_mot.motor.isOpen())
    {
        printf("motor not connect.\n");
        return;
    }

    if(ui->ch_dir->isChecked())
    {
        md_mot.homing(CCW);
    }
    else
    {
        md_mot.homing(CW);
    }

    if(ui->cb_lift_dir_homing->isChecked())
    {
        md_mot.homing(CCW);
    }
    else
    {
        md_mot.homing(CW);
    }

}

void MainWindow::homeEnd(QString msg)
{
    if (msg == "homing end")
    {
        qDebug()<<"homing end";
        ui->bt_homing->setEnabled(false);
        ui->bt_lift_homimg->setEnabled(false);
    }
}

void MainWindow::bt_move_pos()
{
    int pos = ui->sb_move_pos->value();

    //pos의 단위는 mm임.
    if (pos>680){
        pos=680;
    }
    else if(pos<50){
        pos = 50;
    }

    ui->sb_lift_pos->setValue(pos);
    md_mot.move_position(pos);
}


void MainWindow::bt_lift_pos_move()
{
    int pos = ui->sb_lift_pos->value();
    //pos의 단위는 mm임.
    if (pos>620){
        pos=620;
    }
    else if(pos<0){
        pos = 0;
    }
    ui->sb_move_pos->setValue(pos);
    md_mot.move_position(pos);
}

void MainWindow::sb_lift_pos()
{
    int pos = ui->sb_lift_pos->value();
    ui->sb_move_pos->setValue(pos);
    md_mot.move_position(pos);
    //    ui->le_lift_move_status->setStyleSheet("QLineEdit{background-color:green}");
}

void MainWindow::bt_rpmZero()
{
    md_mot.move_rpm(0);
    //    ui->le_lift_move_status->setStyleSheet("QLineEdit{background-color:green}");
}

void MainWindow::bt_lift_maxVel_pos()
{
    int vel = ui->sb_lift_pos_rpm->value();
    ui->sb_max_vel_pos->setValue(vel);
    md_mot.set_maxVel_pos(vel);

}

void MainWindow::bt_set_maxVel_pos()
{
    int vel = ui->sb_max_vel_pos->value();
    ui->sb_lift_pos_rpm->setValue(vel);
    md_mot.set_maxVel_pos(vel);
}


void MainWindow::bt_move_rpm()
{
    md_mot.move_poisition_flag=false;
    int rpm = ui->sb_rpm->value();
    md_mot.move_rpm(rpm);
    //    ui->le_lift_move_status->setStyleSheet("QLineEdit{background-color:red}");
}

void MainWindow::bt_lift_status()
{
    //    md_mot.test_pid();
    MOTOR_MAIN_DATA for_show = md_mot.get_main_data();
    lift_rpm = for_show.motor_rpm;
    ui->le_rpm->setText(QString::number(for_show.motor_rpm));

    armp = for_show.motor_armp;
    ui->le_armp->setText(QString::number(armp));

    short controlType = for_show.motor_controlType;
    //    memcpy(&controlType, data.data() + 9, 1);
    if(controlType == 0)
    {
        ui->le_controlType->setText(QString("stop"));
    }
    else if(controlType == 1)
    {
        ui->le_controlType->setText(QString("speed"));
    }
    else if(controlType == 2)
    {
        ui->le_controlType->setText(QString("posi"));
    }
    else if(controlType == 3)
    {
        ui->le_controlType->setText(QString("open loop"));
    }
    else if(controlType == 4)
    {
        ui->le_controlType->setText(QString("tq"));
    }

    //    ui->le_controlType->setText(QString::number(for_show.motor_controlType));

    ui->le_rpmStd->setText(QString::number(for_show.motor_ref_rpm));

    lift_pos = for_show.motor_position*620/4700+1;
    ui->le_pos->setText(QString::number(lift_pos));

    QString le_lift_move_status = ui->le_lift_move_status->styleSheet();
    QStringList lift_color = le_lift_move_status.split(":");

    if(lift_color[1]=="red}")
    {
        ui -> sb_lift_pos->setValue(lift_pos);
    }

    ui->le_temp->setText(QString::number(for_show.motor_temperature));
}

void MainWindow::write_log(QString str)
{
    qDebug()<<str;
    //    ui->te_data->append(str);
    //    MOTOR_MAIN_DATA for_show = md_mot.get_main_data();
    //    ui->le_rpm->setText(QString::number(for_show.motor_rpm));
}

void MainWindow::onSystemCheck()
{
    // base speed change
    //        qDebug() << "************** " << systemStat.sdata.robot_state ;
    //    sendData();
    Joint_info_now[0] = cobot.systemStat.sdata.jnt_ang[0];
    Joint_info_now[1] = cobot.systemStat.sdata.jnt_ang[1];
    Joint_info_now[2] = cobot.systemStat.sdata.jnt_ang[2];
    Joint_info_now[3] = cobot.systemStat.sdata.jnt_ang[3];
    Joint_info_now[4] = cobot.systemStat.sdata.jnt_ang[4];
    Joint_info_now[5] = cobot.systemStat.sdata.jnt_ang[5];

    float spd = (float)ui->HS_BASE_SPEED->value()/100.0;
    if(fabs(spd - cobot.systemStat.sdata.default_speed) > 0.005)
    {
        if(speed_changed == true)
        {
            // value changed
            cobot.BaseSpeedChange(spd);
            speed_changed = false;
        }
        else
        {
            float spd = cobot.systemStat.sdata.default_speed;
            ui->HS_BASE_SPEED->setValue(spd*100);
            ui->sld_rb5_speed->setValue(spd*100);
            ui -> sb_rb5_speed->setValue(spd*100);
            ui->lb_rb5_speed_percentage->setText(QString::number(spd*100)+"%");
        }
    }


    // check initialize status
    if(initFlag == true)
    {
        int init_info = cobot.systemStat.sdata.init_state_info;
        //        qDebug()<<"init_info : " <<init_info;
        switch(init_info)
        {
        case INIT_STAT_INFO_VOLTAGE_CHECK:
            ui->LE_INIT_POWER->setStyleSheet("QLineEdit{background-color:yellow}");
            ui->LE_INIT_DEVICE->setStyleSheet("QLineEdit{background-color:red}");
            ui->LE_INIT_SYSTEM->setStyleSheet("QLineEdit{background-color:red}");
            ui->LE_INIT_ROBOT->setStyleSheet("QLineEdit{background-color:red}");
            ui->robot_status->setStyleSheet("QLineEdit{background-color:red}");
            break;
        case INIT_STAT_INFO_DEVICE_CHECK:
            ui->LE_INIT_POWER->setStyleSheet("QLineEdit{background-color:green}");
            ui->LE_INIT_DEVICE->setStyleSheet("QLineEdit{background-color:yellow}");
            ui->LE_INIT_SYSTEM->setStyleSheet("QLineEdit{background-color:red}");
            ui->LE_INIT_ROBOT->setStyleSheet("QLineEdit{background-color:red}");
            ui->robot_status->setStyleSheet("QLineEdit{background-color:red}");
            break;
        case INIT_STAT_INFO_FIND_HOME:
            ui->LE_INIT_POWER->setStyleSheet("QLineEdit{background-color:green}");
            ui->LE_INIT_DEVICE->setStyleSheet("QLineEdit{background-color:green}");
            ui->LE_INIT_SYSTEM->setStyleSheet("QLineEdit{background-color:yellow}");
            ui->LE_INIT_ROBOT->setStyleSheet("QLineEdit{background-color:red}");
            ui->robot_status->setStyleSheet("QLineEdit{background-color:red}");
            break;
        case INIT_STAT_INFO_VARIABLE_CHECK:
            ui->LE_INIT_POWER->setStyleSheet("QLineEdit{background-color:green}");
            ui->LE_INIT_DEVICE->setStyleSheet("QLineEdit{background-color:green}");
            ui->LE_INIT_SYSTEM->setStyleSheet("QLineEdit{background-color:yellow}");
            ui->LE_INIT_ROBOT->setStyleSheet("QLineEdit{background-color:red}");
            ui->robot_status->setStyleSheet("QLineEdit{background-color:red}");
            break;
        case INIT_STAT_INFO_COLLISION_ON:
            ui->LE_INIT_POWER->setStyleSheet("QLineEdit{background-color:green}");
            ui->LE_INIT_DEVICE->setStyleSheet("QLineEdit{background-color:green}");
            ui->LE_INIT_SYSTEM->setStyleSheet("QLineEdit{background-color:green}");
            ui->LE_INIT_ROBOT->setStyleSheet("QLineEdit{background-color:yellow}");
            ui->robot_status->setStyleSheet("QLineEdit{background-color:yellow}");
            break;
        case INIT_STAT_INFO_INIT_DONE:
            ui->LE_INIT_POWER->setStyleSheet("QLineEdit{background-color:green}");
            ui->LE_INIT_DEVICE->setStyleSheet("QLineEdit{background-color:green}");
            ui->LE_INIT_SYSTEM->setStyleSheet("QLineEdit{background-color:green}");
            ui->LE_INIT_ROBOT->setStyleSheet("QLineEdit{background-color:green}");
            ui->robot_status->setStyleSheet("QLineEdit{background-color:green}");
            initFlag = false;
            break;
        case INIT_STAT_INFO_NOACT:
        default:
            break;
        }
    }
}


void MainWindow::print()
{
    QByteArray rb_5_data = cobot.datas;
    QString string(rb_5_data);
    //    qDebug() << "datas! : "<<rb_5_data;
    if(string != old_rb5_msg)
    {
        ui->textPrint->append(string);
        QScrollBar *sb = ui->textPrint->verticalScrollBar();
        sb->setValue(sb->maximum());
    }
    old_rb5_msg = string;
}

void MainWindow::BTN_CONNECT_COM()
{
    if(ui->BTN_CONNECT_COM->text() == "Connect")
    {
        // connect
        //        qDebug() << "커넥션하려고한다";
        cobot.cmdSocket.connectToHost(QHostAddress(ui->LE_IP->text()), ui->textCmdPort->text().toInt());
        connect(&cobot, SIGNAL(readyRead()), &cobot, SLOT(onReadyCmdRead()));
    }
    else
    {
        //        qDebug() << "커넥션 안하려고한다";
        // disconnect
        cobot.cmdSocket.close();
    }
}

void MainWindow::BTN_CONNECT_DATA()
{
    if(ui->BTN_CONNECT_DATA->text() == "Connect")
    {
        // connect
        cobot.dataSocket.connectToHost(QHostAddress(ui->LE_IP->text()), ui->textDataPort->text().toInt());
        connect(&cobot, SIGNAL(readyRead()), &cobot, SLOT(onReadyDataRead()));
        //        timer.start(20);
        //        cobot.init();
    }
    else
    {
        // disconnect
        timer.stop();
        cobot.dataSocket.close();
    }
}

void MainWindow::on_BTN_COBOT_INIT_clicked()
{
    cobot.CobotInit();
    initFlag = true;
}

void MainWindow::on_BTN_MODE_REAL_clicked()
{
    cobot.ProgramMode_Real();
    ui -> bt_rb5_mode ->setText("Simul.");
}

void MainWindow::on_BTN_MODE_SIMULATION_clicked()
{
    cobot.ProgramMode_Simulation();
    ui -> bt_rb5_mode ->setText("Real.");
}

void MainWindow::on_BTN_CLEAR_clicked()
{
    ui->textPrint->clear();
}

void MainWindow::on_HS_BASE_SPEED_valueChanged(int value)
{
    float spd = (float)value/100.0;
    ui->LB_BASE_SPEED->setText(QString().sprintf("%.1f%%", spd*100.0));
    ui->lb_rb5_speed_percentage->setText(QString::number(spd*100)+"%");
    ui->sb_rb5_speed->setValue(spd*100);
    ui->sld_rb5_speed->setValue(spd*100);
    speed_changed = true;
}

void MainWindow::on_BTN_MOVE_JOINT_HIGH_clicked()
{
    qDebug()<<"최고높이에서 디텍팅중.";
    //    cobot.MoveJoint(initPos._joint_1, initPos._joint_2 , initPos._joint_3 , initPos._joint_4, initPos._joint_5, initPos._joint_6, -1, -1);
    //    cobot.MoveTCP(-372, -118, 529, 90, -90.00, 180, 2, -1);
    //     cobot.MoveJoint(2.90,29.97,-137.74,107.77, -92.92,-90.04,2, -1);
    //    cobot.MoveJoint(0,20,-76,20,-90,-90,2, -1);
    //    cobot.MoveJoint(-7.11, -3.82, -40.88, -12.63, -85.45, -83.14,2, -1);
    //    cobot.MoveJoint(0.0,75.7509994506836,-124.04100036621094,21,-90.00,-90.00, 2, -1);
    cobot.MoveJoint(0.000,29.340,-119.691,69.323,-90.000,-90.000, 2, -1);
}


void MainWindow::on_BTN_MOVE_JOINT_READY_GRIPPER_clicked()
{
    //    cobot.MoveJoint(readyGripperPos._joint_1, readyGripperPos._joint_2, readyGripperPos._joint_3, readyGripperPos._joint_4,  readyGripperPos._joint_5, readyGripperPos._joint_6, -1, -1);
}

void MainWindow::on_BTN_MOVE_JOINT_GRAPS_GRIPPER_clicked()
{
    //    cobot.MoveJoint(grapsGripperPos._joint_1, grapsGripperPos._joint_2, grapsGripperPos._joint_3, grapsGripperPos._joint_4, grapsGripperPos._joint_5, grapsGripperPos._joint_6, -1, -1);
}

void MainWindow::on_BTN_MOTION_PAUSE_clicked()
{
    cobot.MotionPause();
}

void MainWindow::on_BTN_MOTION_RESUME_clicked()
{
    cobot.MotionResume();
}

void MainWindow::on_BTN_MOTION_HALT_clicked()
{
    cobot.MotionHalt();
}

void MainWindow::on_BTN_COLLISION_RESUME_clicked()
{
    cobot.CollisionResume();
}
////////////////////////////////////////////////////////////////

void MainWindow::keti_showUI_msg(bool connect_flag)
{
    if (connect_flag)
    {
        ui->le_keti->setStyleSheet("background-color:green");
        ui->le_vision_connection->setStyleSheet("background-color:green");
        keti_connect_flag = true;
    }

    else
    {
        ui->le_keti->setStyleSheet("background-color:red");
        ui->le_vision_connection->setStyleSheet("background-color:red");
        keti_connect_flag = false;
    }

}

void MainWindow::kitech_showUI_msg(bool connect_flag)
{
    if (connect_flag)
    {
        ui->le_kitech->setStyleSheet("background-color:green");
        ui->le_gripper_connection->setStyleSheet("background-color:green");
        if(old_connect_flag != connect_flag)
        {
            vision.TCP2cam = cam_config.KITECH_TF_SENSOR;
            //            std::cout<<vision.TCP2cam<<std::endl;
        }
    }

    else
    {
        ui->le_kitech->setStyleSheet("background-color:red");
        ui->le_gripper_connection->setStyleSheet("background-color:red");
        if(old_connect_flag != connect_flag)
        {
            vision.TCP2cam = cam_config.RB_TF_SENSOR;
            //            std::cout<<vision.TCP2cam<<std::endl;
        }
    }
    old_connect_flag = connect_flag;
}


void MainWindow::BTN_GRIPPER_CMD_WRITE()
{

    //    QString txt = "S1";
    QString txt = ui->LE_GRIPPER_CMD->text();
    qDebug()<<"txt : "<<txt;
    gripper.Kitech_Client->write(txt.toStdString().c_str());
    //    // all cmd write in hand
    //    if(Integrated_info.init_gripper == true)
    //    {
    //        //        IPC::GRIPPER gripper_cmd;
    //        auto text = ui->LE_GRIPPER_CMD->text();
    //        qDebug()<<text;
    //        memcpy((uint8_t*)gripper_cmd.cmd, text.data(), 50);
    //        ipc.set_gripper(gripper_cmd); //통합 ui에 그리퍼 명령을 넘기기 위한 코드.
    //        std::string msg = text.toUtf8().constData();

    //        ::send(gripper.tmp_fd, msg.data(), msg.size(), 0);
    //        //        gripper_cmd.cmd = ;
    //        QString Log = "[Gripper] cmd : " + text;
    //        QByteArray br = text.toUtf8();

    //        //        _log.PrintLog(Log.toStdString(), ui->TE_GLOBAL_LOG, "green", true);
    //        //        gripper.Kitech_Client->write(br);
    //    }
    //    else
    //    {
    //        //        _log.PrintLog("Not connected KITECH yet.", ui->TE_GLOBAL_LOG, "red", true);
    //    }
}

void MainWindow::BTN_GRIPPER_HOLD()
{

    // preset unfold(gripper ready) pos cmd
    if(Integrated_info.init_gripper == true)
    {
        QString text("Rg0");
        qDebug()<<text;
        memcpy((uint8_t*)gripper_cmd.cmd, text.data(), 50);
        ipc.set_gripper(gripper_cmd); //통합 ui에 그리퍼 명령을 넘기기 위한 코드.
        std::string msg = text.toUtf8().constData();
        ::send(gripper.tmp_fd, msg.data(), msg.size(), 0);

        QByteArray br1 = text.toUtf8();
        //        gripper.Kitech_Client->write(br1);
    }
}

void MainWindow::BTN_GRIPPER_ALLFOLD()
{

    // preset unfold(gripper ready) pos cmd
    if(Integrated_info.init_gripper == true)
    {
        BTN_GRIPPER_HOLD();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        QString text("Gg9");
        qDebug()<<text;
        QByteArray br = text.toUtf8();
        //        std::string msg = text.toUtf8().constData();
        //        ::send(gripper.tmp_fd, msg.data(), msg.size(), 0);
        gripper.Kitech_Client->write(br);

        //        start_old = clock(); // 시간 측정 시작
        //        bool flag_old = true;

        //        //        qDebug()<<"start"<<start;

        //        while(flag_old)
        //        {

        //            if(result_old>1000000)
        //            {
        //                //                Pinch12_FOLD();
        //                QString text("Rm9");
        //                qDebug()<<text;
        //                QString Log = "[Gripper] cmd : " + text;
        //                QByteArray br = text.toUtf8();

        //                //            _log.PrintLog(Log.toStdString(), ui->TE_GLOBAL_LOG, "green", true);
        ////                gripper.Kitech_Client->write(br);

        ////                gripper.Kitech_Client->flush();//
        //                flag_old = false;
        //            }
        //            end_old = clock(); // 시간 측정 끝
        //            result_old = double(end_old-start_old);

        //        }
        //        start = clock(); // 시간 측정 시작
        //        bool flag = true;

        //        //        qDebug()<<"start"<<start;

        //        while(flag)
        //        {

        //            if(result>300)
        //            {
        //                QString text("Gg9");
        //                qDebug()<<text;
        //                QString Log = "[Gripper] cmd : " + text;
        //                QByteArray br = text.toUtf8();

        //                //            _log.PrintLog(Log.toStdString(), ui->TE_GLOBAL_LOG, "green", true);
        ////                gripper.Kitech_Client->write(br);
        //                flag = false;
        //            }
        //            end = clock(); // 시간 측정 끝
        //            result = double(end-start);
        //        }

    }
    else
    {
        //        _log.PrintLog("Not connected KITECH yet.", ui->TE_GLOBAL_LOG, "red", true);
    }
}

void MainWindow::Pinch12_FOLD()
{

    // preset unfold(gripper ready) pos cmd
    if(Integrated_info.init_gripper == true)
    {
        QString text("Gi2");
        //        qDebug()<<text;
        //        std::string msg = text.toUtf8().constData();

        //        ::send(gripper.tmp_fd, msg.data(), msg.size(), 0);
        //        QString Log = "[Gripper] cmd : " + text;


        //        _log.PrintLog(Log.toStdString(), ui->TE_GLOBAL_LOG, "green", true);
        QByteArray br = text.toUtf8();
        gripper.Kitech_Client->write(br);
    }
    else
    {
        //                _log.PrintLog("Not connected KITECH yet.", ui->TE_GLOBAL_LOG, "red", true);
    }
}


void MainWindow::BTN_GRIPPER_GRASP()
{
    // preset graps pos cmd in free grapMode & power
    if(Integrated_info.init_gripper == true)
    {
        QString grap = ui->CB_GRIPPER_GRAPS->currentText();
        QString powe = ui->CB_GRIPPER_POWER->currentText();

        QString text = "R" + grap + powe ;
        QByteArray br = text.toUtf8();
        gripper.Kitech_Client->write(br);
    }
    else
    {
        //        _log.PrintLog("Not connected KITECH yet.", ui->TE_GLOBAL_LOG, "red", true);
    }
}

void MainWindow::BTN_GRIPPER_VISION()
{
    // preset vision pos cmd
    if(Integrated_info.init_gripper == true)
    {
        QString text("Rg9");
        qDebug()<<text;
        QByteArray br = text.toUtf8();

        //        _log.PrintLog(Log.toStdString(), ui->TE_GLOBAL_LOG, "green", true);
        gripper.Kitech_Client->write(br);
    }
    else
    {
        //        _log.PrintLog("Not connected KITECH yet.", ui->TE_GLOBAL_LOG, "red", true);
    }
}

void MainWindow::CB_GRIPPER_GE_CMD_WRITE()
{
    //    ControlBoxDigitalOut(1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
    // preset gripper pos cmd in free mode & grapMode & power
    if(Integrated_info.init_gripper == true)
    {
        auto mode = ui->CB_GRIPPER_GE_MODE->currentText();
        auto grap = ui->CB_GRIPPER_GE_GRAPS->currentText();
        auto powe = ui->CB_GRIPPER_GE_POWER->currentText();

        QString text = mode  + grap + powe;
        gripper.Kitech_Client->write(text.toUtf8());
        if(ui->CB_GRIPPER_GE_SUCTION_POWER->currentText()=="ON")
        {
            cobot.ControlBoxDigitalOut(1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
        }
        else if(ui->CB_GRIPPER_GE_SUCTION_POWER->currentText()=="OFF")
        {
            cobot.ControlBoxDigitalOut(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
        }

        //        std::string msg = text.toUtf8().constData();

        //        ::send(gripper.tmp_fd, msg.data(), msg.size(), 0);
        //        //        ControlBoxDigitalOut(1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
        //        qDebug()<<text;
        //        QString Log = "[Gripper] cmd : " + text;
        //        QByteArray br = text.toUtf8();

        //        _log.PrintLog(Log.toStdString(), ui->TE_GLOBAL_LOG, "green", true);
        //        gripper.Kitech_Client->write(br);
    }
    else
    {
        //        _log.PrintLog("Not connected KITECH yet.", ui->TE_GLOBAL_LOG, "red", true);
    }
}

void MainWindow::bt_cobot_move2object()
{
    on_BTN_GRIPPER_OPEN_clicked();

    float move_x_valo = ui->move_x_val->text().toFloat();
    float move_y_valo = ui->move_y_val->text().toFloat();
    float move_z_valo = ui->move_z_val->text().toFloat();

    float move_rx_valo = ui->move_rx_val->text().toFloat();
    float move_ry_valo = ui->move_ry_val->text().toFloat();
    float move_rz_valo = ui->move_rz_val->text().toFloat();

    cobot.MoveTCP(move_x_valo,move_y_valo,move_z_valo, move_rx_valo, move_ry_valo, move_rz_valo, 0.5, -1);

    if(vision.box_cent_value == false)
    {
        cobot.ControlBoxDigitalOut(1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
    }
    else if(vision.box_cent_value)
    {
        vision.box_cent_value = false;
    }

    //    cobot.MoveTCPBlend_Clear();
    //    cobot.MoveTCPBlend_AddPoint(0.1,move_x_valo+10,move_y_valo+10,move_z_valo+10, move_rx_valo, move_ry_valo, move_rz_valo, 0.5, -1);
    //    cobot.MoveTCPBlend_AddPoint(0.1,move_x_valo,move_y_valo,move_z_valo, move_rx_valo, move_ry_valo, move_rz_valo, 0.5, -1);
    //    cobot.MoveTCPBlend_MovePoint();

}


void MainWindow::BTN_CONNECT_GRIPPER()
{
    gripper.Gripper_Connect();
}

void MainWindow::on_BTN_MOVE_JOINT_HIGH_LEFT_clicked()
{
    cobot.MoveJoint(178.86,-15.61, -120.44, 46.04, -90.00, -88.89, 2, -1);
    //    cobot.MoveJoint(putdown1Pos._joint_1, putdown1Pos._joint_2 , putdown1Pos._joint_3 , putdown1Pos._joint_4, putdown1Pos._joint_5, putdown1Pos._joint_6, -1, -1);
}
void MainWindow::on_BTN_MOVE_JOINT_INIT_3_clicked()
{
    cobot.MoveJoint(0,60,-134,56,-90,-88, -1, -1);
}

void MainWindow::on_BTN_MOVE_JOINT_INIT_4_clicked()
{
    //    cobot.MoveJoint(readyGripper2Pos._joint_1, readyGripper2Pos._joint_2 , readyGripper2Pos._joint_3 , readyGripper2Pos._joint_4, readyGripper2Pos._joint_5, readyGripper2Pos._joint_6, -1, -1);
}

void MainWindow::on_BTN_MOVE_JOINT_INIT_5_clicked()
{
    //    cobot.MoveJoint(grapsGripper2Pos._joint_1, grapsGripper2Pos._joint_2 , grapsGripper2Pos._joint_3 , grapsGripper2Pos._joint_4, grapsGripper2Pos._joint_5, grapsGripper2Pos._joint_6, -1, -1);
}

//////////barcode//////////

void MainWindow::bt_usb1_con()
{
    if (ui->cb_usb1->currentText()!=ui->cb_usb2->currentText())
    {
        barcode.bt_usb1_con(ui->cb_usb1->currentText());
        ui->bt_usb1_con->setEnabled(false);
        ui->bt_usb1_dis->setEnabled(true);
    }
    else
    {
        qDebug()<<"클릭할 수 없습니다.";
    }
}

void MainWindow::bt_usb2_con()
{
    if (ui->cb_usb1->currentText()!=ui->cb_usb2->currentText())
    {
        barcode.bt_usb2_con(ui->cb_usb2->currentText());
        ui->bt_usb2_con->setEnabled(false);
        ui->bt_usb2_dis->setEnabled(true);
    }
    else
    {
        qDebug()<<"클릭할 수 없습니다.";
    }
}

void MainWindow::bt_usb1_dis()
{
    barcode.bt_usb1_dis();
    ui->bt_usb1_con->setEnabled(true);
    ui->bt_usb1_dis->setEnabled(false);
}

void MainWindow::bt_usb2_dis()
{
    barcode.bt_usb2_dis();
    ui->bt_usb2_con->setEnabled(true);
    ui->bt_usb2_dis->setEnabled(false);
}


void MainWindow::barcode1_showUI_msg(QByteArray msg)
{
    ui->te_usb1->setText(msg);
}
void MainWindow::barcode2_showUI_msg(QByteArray msg)
{
    ui->te_usb2->setText(msg);
}

//get detect point

void MainWindow::timer_shared_memory_loop()
{
    IPC::VISION_POSE vision_pose = ipc.get_point();
    //        qDebug()<<"vision_pose.x : "<<vision_pose.x;
    //        printf("x : %f, y : %f, z : %f\n",vision_pose.x,vision_pose.y,vision_pose.z);

    QString res_str;
    res_str.sprintf("%.3f, %.3f, %.3f, %.3f, %.3f, %.3f", vision_pose.x,vision_pose.y,vision_pose.z,vision_pose.roll,vision_pose.pitch,vision_pose.yaw);

    if(vision_pose.tick!=old_tick)
    {
        ui->lb_keti_point->setText(res_str);
        if(keti_connect_flag == false)
        {
            keti_x = vision_pose.x;
            keti_y = vision_pose.y;
            keti_z = vision_pose.z;

            keti_rx = vision_pose.roll;
            keti_ry = vision_pose.pitch;
            keti_rz = vision_pose.yaw;

            //            app_x = vision_pose.x;
            //            app_y = vision_pose.y+10;
            //            app_z = vision_pose.z;
            //////////////////////////////////////////////////////
            //for approach
            double approach_ty = keti_y + 0.10;
            Eigen::Vector3d app_P(keti_x,approach_ty,keti_z);
            Eigen::Vector3d app_trance_P = T_sensor.block(0,0,3,3)*app_P+T_sensor.block(0,3,3,1);

            app_x = app_trance_P[0]*1000;
            app_y = app_trance_P[1]*1000;
            app_z = app_trance_P[2]*1000;
            //////////////////////////////////////////////////////
        }

        else
        {
            keti_x = vision.keti_x;
            keti_y = vision.keti_y;
            keti_z = vision.keti_z;

            //            keti_rx = vision_pose.roll;
            //            keti_ry = vision_pose.pitch;
            //            keti_rz = vision_pose.yaw;
        }

        Eigen::Vector3d P(keti_x, keti_y, keti_z);
        Eigen::Vector3d _P = T_sensor.block(0,0,3,3)*P+T_sensor.block(0,3,3,1);

        QString res_x,res_y,res_z;
        res_x.sprintf("%.3f",(_P[0])*1000);
        res_y.sprintf("%.3f",(_P[1])*1000);
        res_z.sprintf("%.3f",(_P[2])*1000);

        ui->move_x_val->setText(res_x);
        ui->move_y_val->setText(res_y);
        ui->move_z_val->setText(res_z);

        ui->move_rx_val->setText(QString::number(keti_rx));
        ui->move_ry_val->setText(QString::number(keti_ry));
        ui->move_rz_val->setText(QString::number(keti_rz));
    }

    // map
    IPC::COLOR_MAP color_map = ipc.get_img();

    if(color_map.tick != last_map_tick)
    {
        cv::Mat map_img(300, 500, CV_8UC3, cv::Scalar(0));
        memcpy((uint8_t*)map_img.data, color_map.buf, 500*300*3);

        //        cv::imshow("map_img",map_img);
        ui->LE_KETI_IMG->setPixmap(QPixmap::fromImage(mat_to_qimage_cpy(map_img)));
        ui->lb_vision_capture_img->setPixmap(QPixmap::fromImage(mat_to_qimage_cpy(map_img)));

        if (capture_flag)
        {
            //                        qDebug()<<"working";
            cv::Mat color_img(100, 100, CV_8UC3, cv::Scalar(0));
            cv::resize(map_img, color_img, cv::Size(545,311), 0, 0, cv::INTER_NEAREST);
            cv::imshow("color_img",color_img);
            cv::imwrite("/home/rainbow/orderpicking_img/keti capture.png",color_img);
            //            imshow("/home/rainbow/orderpicking_img/capture",map_img);


            capture_flag=false;
        }
    }

    last_map_tick = color_map.tick;

}

void MainWindow::LE_KETI_IMG(cv::Mat map_img)
{
    //    계속 업데이트 되고 있음.
    ui->LE_KETI_IMG->setPixmap(QPixmap::fromImage(mat_to_qimage_cpy(map_img)));
    ui->lb_vision_capture_img->setPixmap(QPixmap::fromImage(mat_to_qimage_cpy(map_img)));

    if (capture_flag)
    {
        //        qDebug()<<"working";
        QString filename = vision_num+"vision_img.png";
        qDebug()<<filename;
        //                cv::imwrite(filename.toStdString,map_img);
        vision_num+=vision_num;
        capture_flag =false;
    }
}

void MainWindow::bt_png_change()
{
    mb.png_change();
}

void MainWindow::bt_zip()
//유진로봇 요청 대응하기 위한 자동 압축 프로그램 만들기...
{
    mb.bt_zip();
}

void MainWindow::on_bt_rb5_mode_clicked()
{
    if(ui->bt_rb5_mode->text() =="Simul.")
    {
        cobot.ProgramMode_Simulation();
        ui -> bt_rb5_mode ->setText("Real.");
    }

    else if(ui->bt_rb5_mode->text() == "Real.")
    {
        cobot.ProgramMode_Real();
        ui -> bt_rb5_mode ->setText("Simul.");
    }
}

void MainWindow::on_sld_rb5_speed_valueChanged(int value)
{
    float spd = (float)value/100.0;
    ui->LB_BASE_SPEED->setText(QString().sprintf("%.1f%%", spd*100.0));
    ui->lb_rb5_speed_percentage->setText(QString::number(spd*100)+"%");
    ui->sb_rb5_speed->setValue(spd*100);
    ui->HS_BASE_SPEED->setValue(spd*100);
    speed_changed = true;
}

void MainWindow::on_sb_rb5_speed_valueChanged(int value)
{
    float spd = value/100.0;
    ui->LB_BASE_SPEED->setText(QString().sprintf("%.1f%%", spd*100.0));
    ui->lb_rb5_speed_percentage->setText(QString::number(spd*100)+"%");
    ui->sld_rb5_speed->setValue(spd*100);
    ui->HS_BASE_SPEED->setValue(spd*100);
    speed_changed = true;
}

void MainWindow::on_BTN_MOVE_JOINT_Rotation_clicked()
{
    cobot.MoveJoint(-2.22,-15,-120,46,-90,-90, -1);
}

void MainWindow::on_bt_cobot_push_clicked()
{
    float move_x_valo = ui->LE_TCP_REF_X->text().toFloat();
    float move_y_valo = ui->LE_TCP_REF_Y->text().toFloat();
    float move_z_valo = ui->LE_TCP_REF_Z->text().toFloat();

    float move_rx_valo = ui->LE_TCP_REF_RX->text().toFloat();
    float move_ry_valo = ui->LE_TCP_REF_RY->text().toFloat();
    float move_rz_valo = ui->LE_TCP_REF_RZ->text().toFloat();

    cobot.MoveTCP(move_x_valo-10,move_y_valo,move_z_valo-10, move_rx_valo, move_ry_valo, move_rz_valo, 0.2, 0.1);
    //    cobot.ControlBoxDigitalOut(1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
}


void MainWindow::on_btn_pump_2_clicked()
{
    if (ui->btn_pump_2->text()=="Pump on")
    {
        cobot.ControlBoxDigitalOut(1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
        ui->btn_pump_2->setText("Pump off");
        ui->le_rb5_move_status->setStyleSheet("QLineEdit{background-color:green}");
    }
    else if(ui->btn_pump_2->text()=="Pump off")
    {
        cobot.ControlBoxDigitalOut(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
        ui->btn_pump_2->setText("Pump on");
        ui->le_rb5_move_status->setStyleSheet("QLineEdit{background-color:red}");
    }
}

void MainWindow::on_bt_TCP_Blend_clicked()
{
    //    cobot.ControlBoxDigitalOut(1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
    cobot.MoveTCPBlend_Clear();
    float move_x_valo = ui->move_x_val->text().toFloat();
    float move_y_valo = ui->move_y_val->text().toFloat();
    float move_z_valo = ui->move_z_val->text().toFloat();

    float move_rx_valo = ui->LE_TCP_REF_RX->text().toFloat();
    float move_ry_valo = ui->LE_TCP_REF_RY->text().toFloat();
    float move_rz_valo = ui->LE_TCP_REF_RZ->text().toFloat();

    cobot.MoveTCPBlend_AddPoint(move_x_valo,move_y_valo,move_z_valo, move_rx_valo, move_ry_valo, move_rz_valo, 0.5, -1);
    cobot.MoveTCP(move_x_valo-20,move_y_valo,move_z_valo-20, move_rx_valo, move_ry_valo, move_rz_valo, 0.2, -1);
    cobot.MoveTCPBlend_MovePoint();
}

void MainWindow::on_BTN_MOVE_JOINT_LOW_clicked()
{
    //    cobot.MoveJointBlend_Clear();
    //    cobot.MoveJointBlend_AddPoint(150.207,-3.357,-64.440,-19.582,-89.706,-88.582, 0.2, 0.2);
    //    cobot.MoveJointBlend_AddPoint(0.000,31.222,-131.071,35.017,-90.004,-90.006, -1);
    //    cobot.MoveJointBlend_MovePoint();

    float spd = 180.0;
    float acc = 400.0;
    float blending_value = 0.2;

    cobot.move_jb2_clear();
    cobot.move_jb2_add(150.68,0.00,-97.41,-10.00,-90.00,-88.90, spd, acc, blending_value);
    cobot.move_jb2_add(0.000,31.222,-131.071,35.017,-90.004,-90.006, spd, acc, blending_value);
    cobot.move_jb2_run();

}

void MainWindow::on_BTN_MOVE_JOINT_INIT_7_clicked()
{
    cobot.MoveJointBlend_Clear();
    cobot.MoveJointBlend_AddPoint(-9.35,-85,-124,168,-82,-84, -1, -1);
    cobot.MoveJointBlend_AddPoint(22.61,-49,-132,111,-86,-87, -1, -1);
    cobot.MoveJointBlend_AddPoint(50,0,-113,26,-90,-90,-1 -1);
    cobot.MoveJointBlend_AddPoint(117,-6,-79,-12,-90,-90, -1, -1);
    cobot.MoveJointBlend_AddPoint(172,-3,-116,30,-90,-90,-1 -1);
    cobot.MoveJointBlend_MovePoint();
    //    cobot.MoveJoint(180,0,-113,15,-90,-90, -1);
}

void MainWindow::on_bt_lift_top_clicked()
{
    //    ui -> sb_lift_pos->setValue(620);
    md_mot.move_poisition_flag = true;
    md_mot.move_position(620);
}

void MainWindow::on_bt_lift_MID_clicked()
{
    //    ui -> sb_lift_pos->setValue(450);
    md_mot.move_poisition_flag = true;
    md_mot.move_position(450);
}

void MainWindow::on_bt_lift_Low_clicked()
{
    //    ui -> sb_lift_pos->setValue(0);
    md_mot.move_poisition_flag = true;
    md_mot.move_position(0);
}

void MainWindow::on_bt_low_get_out_clicked()
{
    float move_x_valo = ui->LE_TCP_REF_X->text().toFloat();
    float move_y_valo = ui->LE_TCP_REF_Y->text().toFloat();
    float move_z_valo = ui->LE_TCP_REF_Z->text().toFloat();

    float move_rx_valo = ui->LE_TCP_REF_RX->text().toFloat();
    float move_ry_valo = ui->LE_TCP_REF_RY->text().toFloat();
    float move_rz_valo = ui->LE_TCP_REF_RZ->text().toFloat();

    cobot.MoveTCP(move_x_valo+50,move_y_valo,move_z_valo+50, move_rx_valo, move_ry_valo, move_rz_valo,  0.3, 0.3);
}

void MainWindow::on_bt_vision_water_clicked()
{
    float move_x_valo = ui->LE_TCP_REF_X->text().toFloat();
    float move_y_valo = ui->LE_TCP_REF_Y->text().toFloat();
    float move_z_valo = ui->LE_TCP_REF_Z->text().toFloat();

    float move_rx_valo = ui->LE_TCP_REF_RX->text().toFloat();
    float move_ry_valo = ui->LE_TCP_REF_RY->text().toFloat();
    float move_rz_valo = ui->LE_TCP_REF_RZ->text().toFloat();

    cobot.MoveTCP(move_x_valo-20,move_y_valo,move_z_valo-20, move_rx_valo, move_ry_valo, move_rz_valo, 0.2, -1);
    cobot.ControlBoxDigitalOut(1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
}

void MainWindow::on_bt_vision_high_get_out_clicked()
{
    float move_x_valo = ui->LE_TCP_REF_X->text().toFloat();
    float move_y_valo = ui->LE_TCP_REF_Y->text().toFloat();
    float move_z_valo = ui->LE_TCP_REF_Z->text().toFloat();

    float move_rx_valo = ui->LE_TCP_REF_RX->text().toFloat();
    float move_ry_valo = ui->LE_TCP_REF_RY->text().toFloat();
    float move_rz_valo = ui->LE_TCP_REF_RZ->text().toFloat();

    cobot.MoveTCP(move_x_valo+200,move_y_valo,move_z_valo+200, move_rx_valo, move_ry_valo, move_rz_valo,  0.3, 0.3);
}

void MainWindow::on_BTN_MOVE_JOINT_MID_clicked()
{
    float spd = 180.0;
    float acc = 400.0;
    float blending_value = 0.2;

    cobot.move_jb2_clear();
    cobot.move_jb2_add(150.68,0.00,-97.41,-10.00,-90.00,-88.90, spd, acc, blending_value);
    //    cobot.move_jb2_add(95.63,11.45,-97.41,10.00,-90.00,-88.90, spd, acc, blending_value);

    cobot.move_jb2_add(25.32,26.38,-113.63,0.33,-90.00,-88.90, spd, acc, blending_value);
    cobot.move_jb2_add(-3.24,36.00,-113.62,-12.40,-90.00,-88.90, spd, acc, blending_value);
        cobot.move_jb2_add(0.228,19.684,-73.543,-15.062,-89.891,-89.646, spd, acc, blending_value);

//    //for 홍보영상
//    cobot.move_jb2_add(0.217986,-1.37101,-60.712,-37.0196,-89.9902,-89.6021, spd, acc, blending_value);

    cobot.move_jb2_run();

}

void MainWindow::on_bt_vision_rael_more_move_clicked()
{

}

void MainWindow::on_BTN_MOVE_JOINT_MID_RIGHT_clicked()
{
    cobot.MoveJoint(178.859,0.005,-97.016,10.010,-90.000,-88.892, 2, -1);
}

void MainWindow::yujin_order(QJsonObject json)
{
    QJsonDocument doc_json(json);
    QString str_json(doc_json.toJson(QJsonDocument::Indented));

    // order msg kipping
    mtx.lock();
    order_yj_json_msg.push(str_json);
    mtx.unlock();

    yujin_order_check();
    ui -> te_order -> append(str_json);
}

void MainWindow::yujin_order_check()
{
    // order msg save -> parsing -> making parsing msg to qstringlist -> que astringlist
    //    qDebug()<<order_yj_json_msg.front();

    //    QString shelf_hight = ui->le_shelf_hight->text();
    //    int count = ui -> CB_gripper_num -> currentText().toInt();
    //    QString shelf_name = ui->CB_shelf->currentText();

    for(auto& it: shelf_infos) // get from saved json
    {
        if(it.second == nullptr)
        {
            continue;
        }

        QStringList order_msg;
        QString new_msg = order_yj_json_msg.front();
        QJsonDocument doc_json = QJsonDocument::fromJson(new_msg.toUtf8());
        QJsonObject json = doc_json.object();

        QVariantMap json_pa = json.toVariantMap();
        QVariantMap json_parm = json_pa["params"].toMap();
        QVariantMap json_destination_box = json_parm["destination_box"].toMap();


        int count = json_parm["item_count"].toInt();

        float shelve_height = json_parm["shelve_height"].toDouble()*1000;

        //        shelf_info->RB_5_pose;

        if (order_yj_list_msg.size() == 0)
        {
            qDebug()<<"msg order que is 0!!!_yyyyy";
            yujin_que_time = 0;
        }

        // 명령에 맞게 리스트를 만든후 que 에 넣음. -> loop start ->

        //        qDebug()<<json_parm["shelve_name"].toString()+"_"+json_destination_box["name"].toString();
        qDebug()<<it.second->shelf_id;
        qDebug()<<json_parm["shelve_name"].toString()+"_"+QString::number(shelve_height);
        if (it.second->shelf_id == json_parm["shelve_name"].toString()+"_"+QString::number(shelve_height))
        {
            qDebug()<<it.second->shelf_id;
            // 정해진 위치에 해당하는 명령이 들어온다면.
            if (it.second->lift_pose == shelve_height)
            {
                qDebug()<<"같은 위치 찾음.";
                std::cout<<it.second->RB_5_pose[0]<<std::endl;
                QString lift_high = "lift_high,"+QString::number(shelve_height);
                order_msg.append(lift_high);//리프트 이동

                // vision robot pose -> robot pose blend
                QString robot_vision = "robot vision,"+QString::number(it.second->RB_5_pose[0])+","+QString::number(it.second->RB_5_pose[1])+","+QString::number(it.second->RB_5_pose[2])+","+QString::number(it.second->RB_5_pose[3])+","+QString::number(it.second->RB_5_pose[4])+","+QString::number(it.second->RB_5_pose[5]);
                //                qDebug()<<"robot moving :" <<robot_vision;
                order_msg.append(robot_vision);
                //                                order_msg.append("robot_high_vision");
                /*
                order_msg.append("wait");
                order_msg.append("robot vision box center");
                */
                //                order_msg.append("wait");
                //                order_msg.append("vision");
                //                order_msg.append("wait");
                //                order_msg.append("robot approach");
                //                order_msg.append("");
                //                //                order_msg.append("wait");
                //                order_msg.append("robot push");
                //                order_msg.append("wait");
                //                order_msg.append("robot pump on");
                //                order_msg.append("robot pop");
                //                //                order_msg.append("robot pump on");
                //                order_msg.append("robot mid left");

                //                QString lift_down = "lift_high,5";
                //                order_msg.append(lift_down);//리프트 이동
                //                order_msg.append("robot pump off");
                //                order_msg.append("success");

                order_msg.append("wait");
                order_msg.append("vision");
                order_msg.append("wait");
                order_msg.append("robot approach");
                order_msg.append("robot pick");
                //                order_msg.append("wait");
                order_msg.append("robot push");
                order_msg.append("wait");

                if (count!=1)
                {
                    QStringList msg;
                    for(int i=0;i<count;i++)
                    {
                        msg+=order_msg;
                    }
                    order_msg=msg;
                }
            }
            else //만약 정해진 위치가 아닌 다른 위치가 들어온다면.
            {
                qDebug()<<"위치 다름.";
                QString lift_high = "lift_high,"+QString::number(shelve_height);
                order_msg.append(lift_high);//리프트 이동
                //                order_msg.append("scene done");//리프트 이동
            }

            for(auto& it: object_infos) // get propreate gripping position hands data
            {
                if(it.second == nullptr)
                {
                    continue;
                }

                if (it.second->obj_id == json_destination_box["name"].toString())
                {

                    //                        std::cout<<it.second->RB_5_pose[0]<<std::endl;
                    qDebug()<<"grap pose : "<<it.second->obj_grap_pose;
                    qDebug()<<"ready grap pose :"<<it.second->obj_ready_grap_pose;
                    //                        order_msg.append("");
                    //                        order_msg.append("");

                    qDebug()<<"grap pose : "<<it.second->obj_grap_pose;
                    qDebug()<<"ready grap pose :"<<it.second->obj_ready_grap_pose;
                    //                        order_msg.append("");
                    //                        order_msg.append("");
                    QString grasp_ready = "grasp ready,"+it.second->obj_ready_grap_pose;
                    //                qDebug()<<"robot moving :" <<robot_vision;
                    order_msg.append(grasp_ready);
                    order_msg.append("wait");
                    order_msg.append("robot pump on");

                    QString real_grasp = "grasp real,"+it.second->obj_grap_pose;
                    //                qDebug()<<"robot moving :" <<robot_vision;
                    order_msg.append(real_grasp);
                    order_msg.append("robot pop");
                    //                order_msg.append("robot pump on");
                    order_msg.append("robot mid left");

                    QString lift_down = "lift_high,5";
                    order_msg.append(lift_down);//리프트 이동
                    order_msg.append("robot pump off");
                    order_msg.append("success");
                }
            }

            mtx.lock();
            order_yj_list_msg.push(order_msg);
            //            qDebug()<<"yujin : "<<order_msg;
            mtx.unlock();
            cur_step_yj = FMS_ROBOT_STATE_START;
            //                qDebug()<<"나는 큐의 크기를 알고싶다 : "<<order_list_msg.size();
            //                cur_step = ROBOT_STATE_START;
            flag_circle = false;
            yujin_old_msg_size = order_yj_list_msg.size();
        }
    }
    order_yj_json_msg.pop();// loop 안에서 pop해야함.
}

void MainWindow::yujin_loop()
{
    /*int seq = 0;
    static int yujin_timeout = 0;
    int cnt = 1;

    QTime time = QTime::currentTime();
    QString timeString = time.toString();

    //websocket 에서 시작을 알려줌.
    //    cur_step_yj = web.cur_step;

    int new_yujin_msg_size = order_yj_list_msg.size();
    //    qDebug()<<"order_yj_list_msg : "<<order_yj_list_msg.front();


    // 여러 메시지가 들어올 때 que 에 저장해놓고 사용하기 위함.
    if (order_yj_list_msg.size()!=0){
        if(yujin_old_msg_size - new_yujin_msg_size > 0 && yujin_que_time !=0)
        {
            yujin_scene = order_yj_list_msg.front();
            cur_step_yj = FMS_ROBOT_STATE_START;

            qDebug()<<"size check 진행중...";
            qDebug()<<"old msg size : "<<yujin_old_msg_size;
            qDebug()<<"new msg size : "<<new_yujin_msg_size;

        }
        else if(yujin_que_time == 0)
        {
            yujin_scene = order_yj_list_msg.front();
            cur_step_yj = FMS_ROBOT_STATE_START;

            qDebug()<<"처음으로 루프 진행중...";
            qDebug()<<"old msg size : "<<yujin_old_msg_size;
            qDebug()<<"new msg size : "<<new_yujin_msg_size;
        }
        yujin_old_msg_size = new_yujin_msg_size;
        yujin_que_time ++;
    }

    switch(cur_step_yj)
    {

    qDebug()<<"Yujin seqLoop START!!";

    case FMS_ROBOT_STATE_START:
    {
        yujin_scene.prepend("init");
        if (flag_circle == false)
        {
            yujin_scene.append("scene done");
        }
        qDebug()<<"scene : "<<yujin_scene;

        cur_step_yj = FMS_ROBOT_STATE_CHECK_SCENE;
        yujin_timeout = 500/100;
        break;
    }
    case FMS_ROBOT_STATE_CHECK_SCENE:
    {
        if(--yujin_timeout > 0)
            break;
        if(yujin_scene.size() > 0)
        {
            //            qDebug()<<"Sssssssssssssssssssssssssssssss";
            ui->te_status->setText(yujin_scene[0]);
            //            QString log = "["+timeString +"] " + scene[0];
            //            save_log(log);

            qDebug() << "check scene " << yujin_scene[0] << yujin_scene.size();
            if(yujin_scene[0] == "move_point_1" || yujin_scene[0] == "move_point_2"|| yujin_scene[0] == "robot origin")
            {
                qDebug() << "move point";
                cur_step_yj = FMS_ROBOT_STATE_AMR_START;
                //                web.cur_step = cur_step_yj;
            }
            else if(yujin_scene[0] == "init" || yujin_scene[0] == "robot_init"){
                //                ROBOT_STATE_CHECK_SCENE=web.cur_step;
                cur_step_yj = FMS_ROBOT_STATE_ROBOT_START;
            }
            else if(yujin_scene[0].left(4) == "lift"){
                cur_step_yj = FMS_ROBOT_STATE_LIFT_START;
            }
            else if(yujin_scene[0].left(5) == "robot"){
                cur_step_yj = FMS_ROBOT_STATE_ROBOT_START;
            }
            else if(yujin_scene[0] == "vision"){
                cur_step_yj = FMS_ROBOT_STATE_VISION_START;
            }
            else if(yujin_scene[0] == "robot_grasp_ready"){
                cur_step_yj = FMS_ROBOT_STATE_GRIPPER_START;
            }
            else if(yujin_scene[0] == "robot_grasp"){
                cur_step_yj = FMS_ROBOT_STATE_GRIPPER_START;
            }
            else if(yujin_scene[0] == "lift_down"){
                cur_step_yj = FMS_ROBOT_STATE_LIFT_START;
            }
            else if(yujin_scene[0] == "lift_mid"){
                cur_step_yj = FMS_ROBOT_STATE_LIFT_START;
            }
            else if(yujin_scene[0] == "lift_high"){
                cur_step_yj = FMS_ROBOT_STATE_LIFT_START;
            }
            else if(yujin_scene[0].left(6) == "mobile"){
                cur_step_yj = FMS_ROBOT_STATE_AMR_START;
            }
            else if(yujin_scene[0] == "wait"){
                cur_step_yj = FMS_ROBOT_STATE_WAIT;
            }
            else if(yujin_scene[0] == "success"){
                cur_step_yj = FMS_ROBOT_STATE_SUCCESS;
            }

            else{
                qDebug() << "SCENE DONE";
                web.seqLoop_done_flag = true;
                order_yj_list_msg.pop();
                cur_step_yj = FMS_ROBOT_STATE_NOT_READY;
            }

            web.cur_step = cur_step_yj;
            yujin_timeout = 500/100;
            break;
        }

        else
        {
            cnt++;
        }
    }
    case FMS_ROBOT_STATE_VISION_START:
    {
        if(--yujin_timeout > 0)
            break;
        if(yujin_scene.size() > 0){
            ui->le_scenario->setStyleSheet("QLineEdit{background-color:green}");
            if(yujin_scene[0] == "vision"){
                on_pb_shutter_clicked();
            }
        }
        else{
            ui->le_scenario->setStyleSheet("QLineEdit{background-color:red}");
            qDebug() << "SCENE DONE";
            cur_step_yj = FMS_ROBOT_STATE_NOT_READY;
            web.cur_step = cur_step_yj;
        }
        yujin_timeout = 2000/100;

        cur_step_yj = FMS_ROBOT_STATE_VISION_MOVE_CHECK;
        web.cur_step = cur_step_yj;
        break;
    }
    case FMS_ROBOT_STATE_VISION_MOVE_CHECK:
    {
        if(--yujin_timeout > 0)
            break;

        cur_step_yj = FMS_ROBOT_STATE_VISION_WAIT;
        web.cur_step = cur_step_yj;
        break;
    }

    case FMS_ROBOT_STATE_VISION_WAIT:
    {
        //내부 조건 만족하지 않을 시.

        QString vision_msg = ui->lb_keti_point->text();
        //        qDebug()<<"vision_msg!!!!!!!!!!!!!!!!:"<<vision_msg;
        if(vision_msg=="OBJ_NONE")
        {
            yujin_scene.clear();
            yujin_scene.append("robot mid left");
            yujin_scene.append("lift_down");
            yujin_scene.append("scene done");
            cur_step_yj = FMS_ROBOT_STATE_CHECK_SCENE;
        }
        else if(vision_msg == "")
        {
            // 빈 배열 메시지가 들어오면 비전 again
            qDebug()<<"비전 again";
            cur_step_yj = FMS_ROBOT_STATE_VISION_START;
        }
        else
        {
            cur_step_yj = FMS_ROBOT_STATE_VISION_DONE;
        }
        web.cur_step = cur_step_yj;
        break;
    }
    case FMS_ROBOT_STATE_VISION_DONE:
    {
        if(flag_circle)
        {
            QString last = yujin_scene[0];
            yujin_scene.pop_front();
            yujin_scene.append(last);
        }
        else
        {
            yujin_scene.pop_front();
        }
        cur_step_yj = FMS_ROBOT_STATE_CHECK_SCENE;
        web.cur_step = cur_step_yj;
        break;
    }

    case FMS_ROBOT_STATE_AMR_START:
    {
        if(--yujin_timeout > 10)
            break;
        if(yujin_scene.size() > 0){
            if(yujin_scene[0] == "mobile move1"){

                QJsonObject json_output;
                json_output["MSG_TYPE"] = "MOVE";
                json_output["POSE_x"] = 0;
                json_output["POSE_y"] = -1.13;
                json_output["POSE_theta"] = 0; ///yujin 에서는 라디안으로 줌.
                QByteArray json_string = QJsonDocument(json_output).toJson(QJsonDocument::Compact);
                mb.cmdSendData(json_string);

                ui->le_mobile_move_status->setStyleSheet("QLineEdit{background-color:red}");
                qDebug()<<"mobile working";
            }
            else if(scene[0] == "mobile move2"){

                QJsonObject json_output;
                json_output["MSG_TYPE"] = "MOVE";
                json_output["POSE_x"] = 2.16;
                json_output["POSE_y"] = 0.34;
                json_output["POSE_theta"] = 0; ///yujin 에서는 라디안으로 줌.
                QByteArray json_string = QJsonDocument(json_output).toJson(QJsonDocument::Compact);
                mb.cmdSendData(json_string);

                ui->le_mobile_move_status->setStyleSheet("QLineEdit{background-color:red}");
                qDebug()<<"mobile working";
            }
        }
        else
        {
            qDebug() << "SCENE DONE";
            cur_step_yj = FMS_ROBOT_STATE_NOT_READY;
        }
        yujin_timeout = 1000/200;
        cur_step_yj = FMS_ROBOT_STATE_AMR_MOVE_CHECK;
        break;
    }
    case FMS_ROBOT_STATE_AMR_MOVE_CHECK:
    {
        if(--yujin_timeout > 5)
            break;

        if(mobile_moving_flag){
            qDebug() << "moving";
            cur_step_yj = FMS_ROBOT_STATE_AMR_MOVE_WAIT_ARRIVE;
        }
        //        else if(abs(mb.pose_x-5.67)>0.5){
        //            qDebug() << "mobile not moving";
        ////            cur_step = ROBOT_STATE_AMR_START;4

        //            cur_step = ROBOT_STATE_AMR_START;
        //        }
        break;
    }

    case FMS_ROBOT_STATE_AMR_MOVE_WAIT_ARRIVE:
    {
        if(mobile_moving_flag){
            qDebug() << "move done check";
            cur_step_yj = FMS_ROBOT_STATE_AMR_MOVE_CHECK;
            //             ui->le_mobile_move_status->setStyleSheet("QLineEdit{background-color:green}");
        }
        else if(mobile_moving_flag==false)
        {
            qDebug() << "move done check";
            if(scene[0] == "mobile move1"){

                if(abs(mb.pose_x-0)>1.0)
                {
                    cur_step_yj = FMS_ROBOT_STATE_AMR_START;
                }
                else{
                    cur_step_yj = FMS_ROBOT_STATE_AMR_MOVE_DONE;
                }
            }
            else if(scene[0] == "mobile move2"){

                if(abs(mb.pose_x-2.16)>1.0)
                {
                    cur_step_yj = FMS_ROBOT_STATE_AMR_START;
                }
                else{
                    cur_step_yj = FMS_ROBOT_STATE_AMR_MOVE_DONE;
                }
            }

            mobile_moving_flag = false;
            ui->le_mobile_move_status->setStyleSheet("QLineEdit{background-color:green}");
        }

        break;
    }
    case FMS_ROBOT_STATE_AMR_MOVE_DONE:
    {
        if(flag_circle){
            QString last = scene[0];
            scene.pop_front();
            scene.append(last);
        }else{
            scene.pop_front();
        }
        cur_step_yj = FMS_ROBOT_STATE_CHECK_SCENE;
        break;
    }

    case FMS_ROBOT_STATE_LIFT_START:
    {
        if(--yujin_timeout > 0)
            break;

        QStringList lift = yujin_scene[0].split(",");
        lift_hight = lift[1].toDouble();

        md_mot.move_position(lift_hight);
        md_mot.move_position(lift_hight);
        md_mot.move_position(lift_hight);

        cur_step_yj = FMS_ROBOT_STATE_LIFT_MOVE_CHECK;
        web.cur_step = cur_step_yj;
        yujin_timeout = 1000/100;

        break;
    }
    case FMS_ROBOT_STATE_LIFT_MOVE_CHECK:
    {
        if(--yujin_timeout > 0)
            break;
        qDebug()<<"FMS_ROBOT_STATE_LIFT_MOVE_CHECK";
        // 리프트 높이가 요청한 높이에서 2mm이상 차이 나는 경우 대기로 넘어가기.
        if(abs(lift_pos-lift_hight)<10)
        {
            qDebug() << "mobile not moving";
            qDebug() << "lift_pos : "<<lift_pos;
            qDebug() << "lift_hight : "<<lift_hight;

            // when self or ext collision occured -> stop the e/v
            if(cobot.systemStat.sdata.op_stat_collision_occur == 1 || cobot.systemStat.sdata.op_stat_self_collision == 1)
            {
                //내부에 가둬버리기
                bt_rpmZero();
                ui->le_scenario->setStyleSheet("QLineEdit{background-color:red}");
            }
            else
            {
                cur_step_yj = FMS_ROBOT_STATE_LIFT_WAIT;
            }
        }
        else
        {
            cur_step_yj = FMS_ROBOT_STATE_LIFT_MOVE_CHECK;
        }
        web.cur_step = cur_step_yj;

        yujin_timeout = 500/100;

        break;
    }
    case FMS_ROBOT_STATE_LIFT_WAIT:
    {
        qDebug()<<"sssssssssssssssssssssssssssssssssss";
        if(lift_state == "done")
        {
            qDebug()<<"done :"<<abs(lift_pos-lift_hight);
            //                if(integrate_ui.lift_state == lift_hight){
            if (abs(lift_pos)-abs(lift_hight)<2)
            {
                qDebug() << "lift not moving";
                cur_step_yj = FMS_ROBOT_STATE_LIFT_DONE;
            }
            else
            {

                //                cur_step_yj = FMS_ROBOT_STATE_LIFT_DONE;
                //3회까지 retry 하고 실패시 fail
                if(lift_yujin_retry<3)
                {
                    lift_yujin_retry++;
                    cur_step = FMS_ROBOT_STATE_LIFT_START;
                    qDebug()<<"lift Retry";
                }
                else
                {
                    //내부에 가둬버리기
                    lift_yujin_retry = 0;
                    ui->le_scenario->setStyleSheet("QLineEdit{background-color:green}");
                    qDebug()<<"lift Error";
                }
                web.cur_step = cur_step_yj;

            }
            break;
        }
    case FMS_ROBOT_STATE_LIFT_DONE:
        {
            qDebug() << "LIFT DONE";
            if(flag_circle)
            {
                QString last = yujin_scene[0];
                yujin_scene.pop_front();
                yujin_scene.append(last);
            }
            else
            {
                yujin_scene.pop_front();
            }
            cur_step_yj = FMS_ROBOT_STATE_CHECK_SCENE;
            web.cur_step = cur_step_yj;
            break;
        }

        case FMS_ROBOT_STATE_ROBOT_START:
        {

            bool json_rb_val = yujin_scene[0].contains(",", Qt::CaseInsensitive);
            if(json_rb_val)
            {
                QStringList rb = yujin_scene[0].split(",");
                cv::Vec6d rb_pose;
                for(int a=1;a<7;a++) // get from saved json file
                {
                    rb_pose[a-1] = rb[a].toDouble();
                }

                if(rb[0] == "robot vision"){
                    qDebug()<<rb_pose[0]<<rb_pose[1]<<rb_pose[2]<<rb_pose[3]<<rb_pose[4]<<rb_pose[5];
                    cobot.MoveJoint(rb_pose[0],rb_pose[1],rb_pose[2],rb_pose[3],rb_pose[4],rb_pose[5], -1, -1);
                }

            }

            if(yujin_scene[0] == "robot init"){
                on_BTN_MOVE_JOINT_MID_RIGHT_clicked();
            }
            else if(yujin_scene[0] == "robot vision box center"){
                //            when i use handeye program
                //            on_BTN_MOVE_JOINT_BOX_CENTER_clicked();
                //            when i use keti program
                on_BTN_MOVE_JOINT_BOX_CENTER_KETI_clicked();
            }
            else if(yujin_scene[0] == "robot mid vision"){
                on_BTN_MOVE_JOINT_MID_clicked();
            }
            else if(yujin_scene[0] == "robot high vision"){
                on_BTN_MOVE_JOINT_HIGH_clicked();
            }
            else if(yujin_scene[0] == "robot low vision"){
                on_BTN_MOVE_JOINT_LOW_clicked();
            }

            else if(yujin_scene[0] == "robot init pick"){
                cobot.MoveTCP(-231,-78,617,0,0,-90,-1,-1);
            }
            else if(yujin_scene[0] == "robot move"){
                //            cobot.MoveJointBlend_Clear();
                //            cobot.MoveJointBlend_AddPoint(-90, 25.14, -143, 36,-93, 0, -1, -1);
                //            cobot.MoveJointBlend_AddPoint(-175, 25, -143, 28, -90, 3.6, -1, -1);
                //            cobot.MoveJointBlend_AddPoint(-186, -14, -121, 45, -89, 6, -1, -1);
                //            cobot.MoveJointBlend_MovePoint();

                cobot.MoveJoint(-186, -14, -121, 45, -89, 6, 0.8, 0.8);
            }
            else if(yujin_scene[0] == "robot push"){
                on_bt_cobot_push_clicked();
            }
            else if(yujin_scene[0] == "robot pop"){
                on_BTN_MOVE_JOINT_POP_clicked();
            }
            else if(yujin_scene[0] == "robot mid left"){
                on_BTN_MOVE_JOINT_MID_LEFT_clicked();
            }
            else if(yujin_scene[0] == "robot mid right"){
                on_BTN_MOVE_JOINT_MID_RIGHT_clicked();
            }
            else if(yujin_scene[0] == "robot approach"){
                on_bt_cobot_move2object_approach_clicked();
            }
            else if(yujin_scene[0] == "robot pick"){
                bt_cobot_move2object();
            }
            else if(yujin_scene[0] == "robot pump off"){
                //            cobot.MoveTCP(497, 55, 202, 90, 0, 90, 0.5, 0.5);
                cobot.ControlBoxDigitalOut(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
            }
            else if(yujin_scene[0] == "robot pump on"){
                //            cobot.MoveTCP(497, 55, 202, 90, 0, 90, 0.5, 0.5);
                cobot.ControlBoxDigitalOut(1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
            }

            else if(yujin_scene[0] == "robot high vision"){
                on_BTN_MOVE_JOINT_HIGH_clicked();

            }
            else if(yujin_scene[0] == "robot done"){
                //            cobot.MoveJointBlend_Clear();

                //            cobot.MoveJointBlend_AddPoint(-175,25,-143,28,-90,0,-1,-1);
                //            cobot.MoveJointBlend_AddPoint(-186,-14,-121,45,-90,6,-1,-1);
                //            cobot.MoveJointBlend_AddPoint(-231,-78,61,0,0,-90,-1,-1);// check

                cobot.MoveJoint(-15, 41, -128, 87, -74 ,0 , -1, -1);
            }
            cur_step_yj = FMS_ROBOT_STATE_ROBOT_MOVE_CHECK;
            web.cur_step = cur_step_yj;
            //            yujin_timeout = 1000/200;
            break;
        }
        case FMS_ROBOT_STATE_ROBOT_MOVE_CHECK:
        {

            ui->le_scenario->setStyleSheet("QLineEdit{background-color:green}");
            //        qDebug()<<"rb5 robot_state : "<<cobot.systemStat.sdata.robot_state;

            //파지 직후에 떨어진것도 잡기 위해 체크 넣음.
            if(cobot.systemStat.sdata.robot_state == 3){
                qDebug() << "robot moving check";
                if(yujin_scene[0] == "robot mid left"||yujin_scene[0] == "robot mid right")
                {
                    QString gripper = ui->le_pick->styleSheet();
                    bool gripper_state = gripper.contains("red", Qt::CaseInsensitive);
                    //                qDebug()<<"moving to basket : "<<gripper_state;

                    if(gripper_state)
                    {
                        //물체 감지가 일어난 경우 robot pop 단어 삭제. 이후 다음 과정 진행.
                        cur_step_yj = FMS_ROBOT_STATE_ROBOT_WAIT;
                    }
                    else
                    {
                        ui->le_scenario->setStyleSheet("QLineEdit{background-color:red}");
                        qDebug()<<"ERROR!!!!";
                        cur_step_yj = FMS_ROBOT_STATE_NOT_READY;
                    }
                }
                else{
                    cur_step_yj = FMS_ROBOT_STATE_ROBOT_WAIT;
                }

                yujin_timeout = 1000/100;
            }
            else{
                cur_step_yj = FMS_ROBOT_STATE_ROBOT_WAIT;
                yujin_timeout = 1000/100;
            }

            break;
        }
        case FMS_ROBOT_STATE_ROBOT_WAIT:
        {
            if(--yujin_timeout > 0)
                break;
            if(cobot.systemStat.sdata.robot_state == 1){
                qDebug() << "robot moving done check";
                cur_step_yj = FMS_ROBOT_STATE_ROBOT_DONE;
                web.cur_step = cur_step_yj;
                break;
            }
            break;
        }
        case FMS_ROBOT_STATE_ROBOT_DONE:
        {
            if(--yujin_timeout > 0)
                break;

            if(flag_circle)
            {
                QString last = yujin_scene[0];
                yujin_scene.pop_front();
                yujin_scene.append(last);
            }
            else if(yujin_scene[0] == "robot pop")
            {
                // 로봇팔이 물체를 드는 과정에서 석션, 그리퍼에 물체 감지가 되지 않는 경우 다시 비전 자세로 돌아가도록 scene에 명령을 넣어줌.
                QString gripper = ui->le_pick->styleSheet();
                bool gripper_state = gripper.contains("red", Qt::CaseInsensitive);
                qDebug()<<gripper_state;

                if(gripper_state)
                {
                    //물체 감지가 일어난 경우 robot pop 단어 삭제. 이후 다음 과정 진행.
                    yujin_scene.pop_front();
                }
                else
                {
                    if(yujin_robot_pop_retry<3)
                    {
                        //                    scene.pop_front();
                        qDebug()<<"robot retry!!!!!";

                        QStringList retry_msg;

                        retry_msg.append("wait");
                        retry_msg.append("robot vision box center");
                        retry_msg.append("wait");
                        retry_msg.append("vision");
                        retry_msg.append("wait");
                        retry_msg.append("robot pick");
                        retry_msg.append("robot push");
                        retry_msg.append("wait");
                        retry_msg.append("robot pump on");

                        yujin_scene=retry_msg+yujin_scene;

                        yujin_scene.prepend("robot pump off");

                        if (lift_pos>500)
                        {
                            yujin_scene.prepend("robot high vision");
                        }
                        else if(lift_pos<500)
                        {
                            yujin_scene.prepend("robot midvision");
                        }
                        else if(lift_pos<100)
                        {
                            yujin_scene.prepend("robot low vision");

                        }
                        else
                        {
                            yujin_scene.prepend("robot mid vision");
                        }

                        ui->le_scenario->setStyleSheet("QLineEdit{background-color:green}");
                        yujin_robot_pop_retry++;
                        qDebug()<<"Retry :"<<yujin_robot_pop_retry;
                    }
                    else
                    {
                        yujin_robot_pop_retry = 0;
                        // 현재 명령 지우기
                        yujin_scene.clear();
                        // lift 및 로봇 상태 원상복구
                        yujin_scene.prepend("lift_mid,1");
                        // lift 에 따른 로봇의 위치를 파악 후 로봇의 위치 원상복귀 시킴.
                        yujin_scene.prepend("robot mid left");
                        ui->le_scenario->setStyleSheet("QLineEdit{background-color:red}");
                    }
                }
            }

            else{
                yujin_scene.pop_front();
                ui->le_scenario->setStyleSheet("QLineEdit{background-color:green}");
            }
            cur_step_yj = FMS_ROBOT_STATE_CHECK_SCENE;
            break;
        }

        case FMS_ROBOT_STATE_GRIPPER_START:
        {
            //        if(scene[0] == "robot_grasp_ready"){
            //            integrate_ui.onSocketWrite("open gripper");
            //        }
            //        else if(scene[0] == "robot_grasp"){
            //            integrate_ui.onSocketWrite("close gripper"); //400
            //            //                scene.pop_front();
            //        }
            cur_step_yj = FMS_ROBOT_STATE_GRIPPER_MOVE_CHECK;
            yujin_timeout = 600/100;
            break;
        }
        case FMS_ROBOT_STATE_GRIPPER_MOVE_CHECK:
        {
            if(--yujin_timeout > 0)
                break;
            cur_step_yj = FMS_ROBOT_STATE_GRIPPER_DONE;
            //            if(integrate_ui.robot_state == 3){
            //                qDebug() << "robot moving check";
            //                cur_step = ROBOT_STATE_ROBOT_WAIT;
            //            }else{
            //                qDebug() << "robot not moving";

            //                cur_step = ROBOT_STATE_ROBOT_START;
            //            }

            break;
        }
        case FMS_ROBOT_STATE_GRIPPER_WAIT:
        {
            if(--yujin_timeout > 0)
                break;
            //        if(integrate_ui.robot_state == 1){
            //            qDebug() << "robot moving done check";
            //            cur_step = ROBOT_STATE_ROBOT_DONE;
            //            break;
            //        }
            break;
        }

        case FMS_ROBOT_STATE_WAIT:{
            //        scene.append("wait");
            //        qDebug()<<"wait";
            //        qDebug()<<"timeout"<<timeout;

            cur_step_yj = FMS_ROBOT_STATE_WAIT_OUT;
            yujin_timeout = 3000/200;
            break;
        }

        case FMS_ROBOT_STATE_WAIT_OUT:{
            //        qDebug()<<"wait out";
            if(--yujin_timeout > 0)
            {
                break;
            }
            //            qDebug()<<yujin_scene;
            yujin_scene.pop_front();
            //            qDebug()<<yujin_scene;
            cur_step_yj = FMS_ROBOT_STATE_CHECK_SCENE;
            break;
        }

        case FMS_ROBOT_STATE_GRIPPER_DONE:
        {
            if(flag_circle)
            {
                QString last = yujin_scene[0];
                yujin_scene.pop_front();
                yujin_scene.append(last);
            }
            else
            {
                yujin_scene.pop_front();
            }
            cur_step_yj = FMS_ROBOT_STATE_CHECK_SCENE;
            break;
        }

        case FMS_ROBOT_STATE_NOT_READY:
        {
            yujin_scene.append("not ready");
            break;
        }
        }
    }
    //     order_yj_list_msg.pop();// loop 안에서 pop해야함.*/
}

void MainWindow::seqLoop()
{
    //    /*
    int seq = 0;
    static int timeout = 0;

    QTime time = QTime::currentTime();
    QString timeString = time.toString();

    int new_msg_size = order_list_msg.size();
    //    static int old_msg_size = 0;

    // 여러 메시지가 들어올 때 que 에 저장해놓고 사용하기 위함.
    if (order_list_msg.size()!=0){
        if(old_msg_size - new_msg_size > 0 && que_time !=0)
        {
            scene = order_list_msg.front();
            cur_step = ROBOT_STATE_START;

            qDebug()<<"size check 진행중...";
            qDebug()<<"old msg size : "<<old_msg_size;
            qDebug()<<"new msg size : "<<new_msg_size;

        }
        else if(que_time == 0)
        {
            scene = order_list_msg.front();
            cur_step = ROBOT_STATE_START;

            qDebug()<<"seqloop 처음으로 루프 진행중...";
            qDebug()<<"old msg size : "<<old_msg_size;
            qDebug()<<"new msg size : "<<new_msg_size;
        }
        old_msg_size = new_msg_size;
        que_time ++;
    }

    switch(cur_step){
    qDebug()<<"seqLoop START!!";

    case ROBOT_STATE_START:{

        scene.prepend("init");
        if (flag_circle == false)
        {
            scene.append("scene done");
        }
        qDebug()<<"scene : "<<scene;

        cur_step = ROBOT_STATE_CHECK_SCENE;
        timeout = 500/100;
        break;
    }
    case ROBOT_STATE_CHECK_SCENE:{
        if(--timeout > 0)
            break;

        if(scene.size() > 0)
        {
            ui->te_status->setText(scene[0]);

            QString log = "["+timeString +"] " + scene[0];
            save_log(log);
            qDebug() << "check scene " << scene[0] << scene.size();

            ui->le_scenario->setStyleSheet("QLineEdit{background-color:green}");
            if(scene[0] == "move_point_1" || scene[0] == "move_point_2"|| scene[0] == "robot origin")
            {
                qDebug() << "move point";
                cur_step = ROBOT_STATE_AMR_START;
            }
            else if(scene[0] == "init" || scene[0] == "robot_init"){
                cur_step = ROBOT_STATE_ROBOT_START;
            }
            else if(scene[0].left(4) == "lift"){
                cur_step = ROBOT_STATE_LIFT_START;
            }
            else if(scene[0].left(5) == "robot"){
                cur_step = ROBOT_STATE_ROBOT_START;
            }
            else if(scene[0] == "vision"){
                cur_step = ROBOT_STATE_VISION_START;
            }
            else if(scene[0].left(5) == "grasp"){
                cur_step = ROBOT_STATE_GRIPPER_START;
            }
            else if(scene[0] == "lift_down"){
                cur_step = ROBOT_STATE_LIFT_START;
            }
            else if(scene[0] == "lift_mid"){
                cur_step = ROBOT_STATE_LIFT_START;
            }
            else if(scene[0] == "lift_high"){
                cur_step = ROBOT_STATE_LIFT_START;
            }
            else if(scene[0].left(6) == "mobile"){
                cur_step = ROBOT_STATE_AMR_START;
            }
            else if(scene[0] == "wait"){
                cur_step = ROBOT_STATE_WAIT;
            }
            else if(scene[0] == "success"){
                cur_step = ROBOT_STATE_SUCCESS;
            }
            else
            {
                qDebug() << "SCENE DONE";
                order_list_msg.pop();
                cur_step = ROBOT_STATE_NOT_READY;
            }
            timeout = 1000/100;
            break;
        }
    }
    case ROBOT_STATE_VISION_START:{
        if(--timeout > 0)
            break;

        if(scene.size() > 0){
            ui->le_scenario->setStyleSheet("QLineEdit{background-color:green}");
            if(scene[0] == "vision")
            {
                on_pb_shutter_clicked();
            }
        }
        else{
            ui->le_scenario->setStyleSheet("QLineEdit{background-color:red}");
            qDebug() << "SCENE DONE";
            cur_step = ROBOT_STATE_NOT_READY;
        }
        timeout = 2000/100;
        cur_step = ROBOT_STATE_VISION_MOVE_CHECK;
        break;
    }
    case ROBOT_STATE_VISION_MOVE_CHECK:{
        if(--timeout > 0)
            break;

        cur_step = ROBOT_STATE_VISION_WAIT;
        break;
    }

    case ROBOT_STATE_VISION_WAIT:{
        //내부 조건 만족하지 않을 시.

        QString vision_msg = ui->lb_keti_point->text();
        qDebug()<<"vision_msg!!!!!!!!!!!!!!!!:"<<vision_msg;
        if(vision_msg == "OBJ_NONE")
        {
            scene.clear();
            scene.append("robot mid left");
            scene.append("lift_high,1");
            scene.append("scene done");
            cur_step = ROBOT_STATE_CHECK_SCENE;
        }
        else if(vision_msg == "")
        {
            // 빈 배열 메시지가 들어오면 기다려
            cur_step = ROBOT_STATE_VISION_WAIT;
            //            scene.append("wait");
        }
        else
        {
            cur_step = ROBOT_STATE_VISION_DONE;
        }
        break;
    }
    case ROBOT_STATE_VISION_DONE:{
        if(flag_circle)
        {
            QString last = scene[0];
            scene.pop_front();
            scene.append(last);
        }
        else
        {
            scene.pop_front();
        }
        cur_step = ROBOT_STATE_CHECK_SCENE;
        break;
    }

    case ROBOT_STATE_AMR_START:{
        if(--timeout > 0)
            break;

        //        qDebug()<<"wwwwwwwwww";
        if(scene.size() > 0){
            if(scene[0] == "mobile move1"){

                QJsonObject json_output;
                json_output["MSG_TYPE"] = "MOVE";
                json_output["POSE_x"] = 4.92;
                json_output["POSE_y"] = 1.79;
                json_output["POSE_theta"] = 0.0; ///yujin 에서는 라디안으로 줌.
                QByteArray json_string = QJsonDocument(json_output).toJson(QJsonDocument::Compact);
                mb.cmdSendData(json_string);

                ui->le_mobile_move_status->setStyleSheet("QLineEdit{background-color:red}");
                ui->le_scenario->setStyleSheet("QLineEdit{background-color:red}");
                //                mobile_moving_flag=true;
                qDebug()<<"mobile working";
            }
            else if(scene[0] == "mobile move2"){

                QJsonObject json_output;
                json_output["MSG_TYPE"] = "MOVE";
                json_output["POSE_x"] = 5.09;
                json_output["POSE_y"] = 0.47;
                json_output["POSE_theta"] = 0.0; ///yujin 에서는 라디안으로 줌.
                QByteArray json_string = QJsonDocument(json_output).toJson(QJsonDocument::Compact);
                mb.cmdSendData(json_string);

                ui->le_mobile_move_status->setStyleSheet("QLineEdit{background-color:red}");
                ui->le_scenario->setStyleSheet("QLineEdit{background-color:red}");
                //                mobile_moving_flag = true;
                qDebug()<<"mobile working";
            }
            cur_step = ROBOT_STATE_AMR_MOVE_CHECK;
        }
        else
        {
            qDebug() << "SCENE DONE";
            cur_step = ROBOT_STATE_NOT_READY;
        }
        timeout = 1000/200;
        cur_step = ROBOT_STATE_AMR_MOVE_CHECK;
        break;
    }
    case ROBOT_STATE_AMR_MOVE_CHECK:{
        if(--timeout > 10)
            break;

        ui->le_scenario->setStyleSheet("QLineEdit{background-color:red}");
        if(mobile_moving_flag)
        {
            //            qDebug() << "moving";
            cur_step = ROBOT_STATE_AMR_MOVE_CHECK;
        }
        else
        {
            cur_step = ROBOT_STATE_AMR_MOVE_WAIT_ARRIVE;
        }
        break;
    }

    case ROBOT_STATE_AMR_MOVE_WAIT_ARRIVE:
    {
        qDebug()<<"mobile_moving_flag : "<<mobile_moving_flag;
        if(mobile_moving_flag){
            qDebug() << "moving";
            cur_step = ROBOT_STATE_AMR_MOVE_CHECK;
            //             ui->le_mobile_move_status->setStyleSheet("QLineEdit{background-color:green}");
        }
        else if(mobile_moving_flag == false)
        {

            qDebug() << "move done check";
            if(scene[0] == "mobile move1"){
                if(abs(mb.pose_x-4.92)>3)
                {
                    cur_step = ROBOT_STATE_AMR_START;
                }
                else{
                    cur_step = ROBOT_STATE_AMR_MOVE_DONE;
                }
            }
            else if(scene[0] == "mobile move2"){
                if(abs(mb.pose_x - 5.09)>3)
                {
                    cur_step = ROBOT_STATE_AMR_START;
                }
                else{
                    cur_step = ROBOT_STATE_AMR_MOVE_DONE;
                }
            }

            ui->le_mobile_move_status->setStyleSheet("QLineEdit{background-color:green}");
            ui->le_scenario->setStyleSheet("QLineEdit{background-color:red}");
        }

        break;
    }
    case ROBOT_STATE_AMR_MOVE_DONE:{
        if(flag_circle){
            QString last = scene[0];
            scene.pop_front();
            scene.append(last);
        }else{
            scene.pop_front();
        }
        cur_step = ROBOT_STATE_CHECK_SCENE;
        break;
    }

    case ROBOT_STATE_LIFT_START:{
        if(--timeout > 0)
            break;

        QStringList lift = scene[0].split(",");
        lift_hight = lift[1].toDouble();

        qDebug()<<"lift : "<<lift;
        qDebug()<<"lift_hight : "<<lift_hight;

        md_mot.move_position(lift_hight);
        md_mot.move_position(lift_hight);
        md_mot.move_position(lift_hight);

        cur_step = ROBOT_STATE_LIFT_MOVE_CHECK;
        timeout = 1000/100;

        break;
    }
    case ROBOT_STATE_LIFT_MOVE_CHECK:{
        if(--timeout > 0)
            break;
        qDebug()<<"ROBOT_STATE_LIFT_MOVE_CHECK";
        //        ui->le_scenario->setStyleSheet("QLineEdit{background-color:red}");

        // 리프트 높이가 요청한 높이에서 5mm이상 차이 나는 경우 대기로 넘어가기.
        if(abs(lift_pos-lift_hight)>5){
            qDebug() << "lift not moving";

            cur_step = ROBOT_STATE_LIFT_WAIT;
        }
        else
        {
            cur_step = ROBOT_STATE_LIFT_WAIT;
        }
        //cur_step = ROBOT_STATE_LIFT_WAIT;
        timeout = 300/100; //(wait 3회전)

        break;
    }
    case ROBOT_STATE_LIFT_WAIT:
    {
        qDebug()<<"ROBOT_STATE_LIFT_WAIT";
        if(lift_state == "done")
        {
            if (abs(lift_pos-lift_hight)<10)
            {
                qDebug() << "lift not moving";
                cur_step = ROBOT_STATE_LIFT_DONE;
            }
            else
            {
                //3회까지 retry 하고 실패시 fail
                if(lift_retry<3)
                {
                    lift_retry++;
                    cur_step = ROBOT_STATE_LIFT_START;
                    qDebug()<<"lift Retry";
                }
                else
                {
                    //내부에 가둬버리기
                    lift_retry = 0;
                    ui->le_scenario->setStyleSheet("QLineEdit{background-color:green}");
                    qDebug()<<"lift Error";
                    web.CMD_RESULT("faillure");
                    web.pick_item_failure_count ++;
                }
            }
        }
        break;
    }
    case ROBOT_STATE_LIFT_DONE:{
        qDebug() << "LIFT DONE";
        if(flag_circle)
        {
            QString last = scene[0];
            scene.pop_front();
            scene.append(last);
        }
        else
        {
            scene.pop_front();
            //            qDebug()<<scene;
        }
        cur_step = ROBOT_STATE_CHECK_SCENE;
        //        qDebug()<<scene;
        break;
    }

    case ROBOT_STATE_ROBOT_START:{
        qDebug() << "ROBOT_STATE_ROBOT_START";

        bool json_rb_val = scene[0].contains(",", Qt::CaseInsensitive);
        if(json_rb_val)
        {
            QStringList rb = scene[0].split(",");
            cv::Vec6d rb_pose;
            for(int a=1; a<7; a++) // get from saved json file
            {
                rb_pose[a-1] = rb[a].toDouble();
            }

            if(rb[0] == "robot vision")
            {
                qDebug()<<rb_pose[0]<<rb_pose[1]<<rb_pose[2]<<rb_pose[3]<<rb_pose[4]<<rb_pose[5];
                cobot.MoveJoint(rb_pose[0],rb_pose[1],rb_pose[2],rb_pose[3],rb_pose[4],rb_pose[5], -1, -1);
            }

        }

        if(scene[0] == "init")
        {
            on_BTN_MOVE_JOINT_MID_RIGHT_clicked();
        }
        else if(scene[0] == "robot vision box center"){
            //            when i use handeye program
            //            on_BTN_MOVE_JOINT_BOX_CENTER_clicked();
            //            when i use keti program
            on_BTN_MOVE_JOINT_BOX_CENTER_KETI_clicked();
        }
        else if(scene[0] == "robot mid vision"){
            on_BTN_MOVE_JOINT_MID_clicked();
        }
        else if(scene[0] == "robot high vision"){
            on_BTN_MOVE_JOINT_HIGH_clicked();
        }
        else if(scene[0] == "robot low vision"){
            on_BTN_MOVE_JOINT_LOW_clicked();
        }

        else if(scene[0] == "robot init pick"){
            cobot.MoveTCP(-231,-78,617,0,0,-90,-1,-1);
        }
        else if(scene[0] == "robot move"){
            //            cobot.MoveJointBlend_Clear();
            //            cobot.MoveJointBlend_AddPoint(-90, 25.14, -143, 36,-93, 0, -1, -1);
            //            cobot.MoveJointBlend_AddPoint(-175, 25, -143, 28, -90, 3.6, -1, -1);
            //            cobot.MoveJointBlend_AddPoint(-186, -14, -121, 45, -89, 6, -1, -1);
            //            cobot.MoveJointBlend_MovePoint();

            cobot.MoveJoint(-186, -14, -121, 45, -89, 6, 0.8, 0.8);
        }
        else if(scene[0] == "robot push"){
            on_bt_cobot_push_clicked();
        }
        else if(scene[0] == "robot pop"){
            on_BTN_MOVE_JOINT_POP_clicked();
        }
        else if(scene[0] == "robot mid left"){
            on_BTN_MOVE_JOINT_MID_LEFT_clicked();
        }
        else if(scene[0] == "robot mid right"){
            on_BTN_MOVE_JOINT_MID_RIGHT_clicked();
        }
        else if(scene[0] == "robot approach"){
            on_bt_cobot_move2object_approach_clicked();
        }
        else if(scene[0] == "robot pick"){
            bt_cobot_move2object();
        }
        else if(scene[0] == "robot pump off"){
            //            cobot.MoveTCP(497, 55, 202, 90, 0, 90, 0.5, 0.5);
            cobot.ControlBoxDigitalOut(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
            cur_step = ROBOT_STATE_ROBOT_DONE;
            break;
        }
        else if(scene[0] == "robot pump on"){
            //            cobot.MoveTCP(497, 55, 202, 90, 0, 90, 0.5, 0.5);
            cobot.ControlBoxDigitalOut(1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
            cur_step = ROBOT_STATE_ROBOT_DONE;
            break;
        }

        else if(scene[0] == "robot high vision"){
            on_BTN_MOVE_JOINT_HIGH_clicked();

        }
        else if(scene[0] == "robot done"){
            //            cobot.MoveJointBlend_Clear();

            //            cobot.MoveJointBlend_AddPoint(-175,25,-143,28,-90,0,-1,-1);
            //            cobot.MoveJointBlend_AddPoint(-186,-14,-121,45,-90,6,-1,-1);
            //            cobot.MoveJointBlend_AddPoint(-231,-78,61,0,0,-90,-1,-1);// check

            cobot.MoveJoint(-15, 41, -128, 87, -74 ,0 , -1, -1);
        }
        cur_step = ROBOT_STATE_ROBOT_MOVE_CHECK;
        break;
    }
    case ROBOT_STATE_ROBOT_MOVE_CHECK:{
        ui->le_scenario->setStyleSheet("QLineEdit{background-color:green}");
        //        qDebug()<<"rb5 robot_state : "<<cobot.systemStat.sdata.robot_state;

        //파지 직후에 떨어진것도 잡기 위해 체크 넣음.
        if(cobot.systemStat.sdata.robot_state == 3){
            qDebug() << "robot moving check";
            if(scene[0] == "robot mid left"||scene[0] == "robot mid right")
            {
                QString vision_msg = ui->lb_keti_point->text();
                qDebug()<<"aaaaaaaaa vision_msg : "<<vision_msg;
                if(vision_msg != "OBJ_NONE")
                {
                    QString gripper = ui->le_pick->styleSheet();
                    bool gripper_state = gripper.contains("red", Qt::CaseInsensitive);
                    //                qDebug()<<"moving to basket : "<<gripper_state;

                    if(gripper_state)
                    {
                        //물체 감지가 일어난 경우 robot pop 단어 삭제. 이후 다음 과정 진행.
                        cur_step = ROBOT_STATE_ROBOT_WAIT;
                    }
                    else
                    {
                        ui->le_scenario->setStyleSheet("QLineEdit{background-color:red}");
                        qDebug()<<"GRIPPER ERROR!!!!";
                        web.CMD_RESULT("faillure");
                        web.pick_item_failure_count ++;
                        cur_step = ROBOT_STATE_NOT_READY;
                    }
                }
                else
                {
                    cur_step = ROBOT_STATE_ROBOT_WAIT;
                }
            }
            else
            {
                cur_step = ROBOT_STATE_ROBOT_WAIT;
            }

            timeout = 1000/100;
        }
        else
        {
            if (caution_flag)
                // （비전에서 위치 이상하게 줘서 이동 못한 경우에는 비전 추가하고 스테이트 확인으로 옮기기)
            {
                scene.append("vision");
                cur_step = ROBOT_STATE_ROBOT_START;

            }
            else
            {
                cur_step = ROBOT_STATE_CHECK_SCENE;
                timeout = 1000/100;
            }
        }

        break;
    }
    case ROBOT_STATE_ROBOT_WAIT:{
        if(--timeout > 0)
            break;
        if(cobot.systemStat.sdata.robot_state == 1)
        {
            qDebug() << "robot moving done check";
            cur_step = ROBOT_STATE_ROBOT_DONE;
            break;
        }
        if(cobot.systemStat.sdata.robot_state == 3)
        {
            if(scene[0] == "robot mid left"||scene[0] == "robot mid right")
            {
                QString vision_msg = ui->lb_keti_point->text();
                if(vision_msg != "OBJ_NONE")
                {
                    QString gripper = ui->le_pick->styleSheet();
                    bool gripper_state = gripper.contains("red", Qt::CaseInsensitive);
                    //                qDebug()<<"moving to basket : "<<gripper_state;

                    if(gripper_state)
                    {
                        //물체 감지가 일어난 경우 robot pop 단어 삭제. 이후 다음 과정 진행.
                        cur_step = ROBOT_STATE_ROBOT_WAIT;
                    }
                    else
                    {
                        ui->le_scenario->setStyleSheet("QLineEdit{background-color:red}");
                        qDebug()<<"RB5 ERROR!!!!";
                        web.CMD_RESULT("faillure");
                        web.pick_item_failure_count ++;

                        cobot.MotionHalt();
                        cur_step = ROBOT_STATE_NOT_READY;
                    }
                }
                else
                {
                    cur_step = ROBOT_STATE_ROBOT_DONE;
                }
            }
        }
        break;
    }

    case ROBOT_STATE_ROBOT_DONE:
    {
        if(--timeout > 0)
            break;

        if(flag_circle){
            QString last = scene[0];
            scene.pop_front();
            scene.append(last);
            ui->le_scenario->setStyleSheet("QLineEdit{background-color:red}");
        }
        else if(scene[0] == "robot pop")
        {
            // 로봇팔이 물체를 드는 과정에서 석션, 그리퍼에 물체 감지가 되지 않는 경우 다시 비전 자세로 돌아가도록 scene에 명령을 넣어줌.
            QString gripper = ui->le_pick->styleSheet();
            bool gripper_state = gripper.contains("red", Qt::CaseInsensitive);
            qDebug()<<gripper_state;
            if(gripper_state)
            {
                //물체 감지가 일어난 경우 robot pop 단어 삭제. 이후 다음 과정 진행.
                scene.pop_front();
            }
            else
            {
                if(robot_pop_retry<3)
                {
                    //                    scene.pop_front();
                    qDebug()<<"robot retry!!!!!";

                    QStringList retry_msg;

                    if(lift_hight < 600)
                    {
                        retry_msg.append("wait");
                        retry_msg.append("robot vision box center");
                    }
                    retry_msg.append("wait");
                    retry_msg.append("vision");
                    retry_msg.append("wait");
                    retry_msg.append("robot pick");
                    retry_msg.append("robot push");
                    retry_msg.append("wait");
                    retry_msg.append("robot pump on");

                    scene = retry_msg+scene;

                    if (lift_pos>600)
                    {
                        scene.prepend("robot high vision");
                    }
                    else if(lift_pos<20)
                    {
                        scene.prepend("robot low vision");
                    }
                    else
                    {
                        scene.prepend("robot mid vision");
                    }

                    scene.prepend("robot pump off");

                    ui->le_scenario->setStyleSheet("QLineEdit{background-color:green}");
                    robot_pop_retry++;
                    qDebug()<<"Retry :"<<robot_pop_retry;
                }
                else
                {
                    robot_pop_retry = 0;
                    // 현재 명령 지우기
                    scene.clear();
                    // lift 및 로봇 상태 원상복구
                    scene.prepend("lift_mid,1");
                    // lift 에 따른 로봇의 위치를 파악 후 로봇의 위치 원상복귀 시킴.
                    scene.prepend("robot mid left");
                    ui->le_scenario->setStyleSheet("QLineEdit{background-color:red}");
                    web.CMD_RESULT("faillure");
                    web.pick_item_failure_count ++;
                }
            }
        }

        else{
            scene.pop_front();
            ui->lb_keti_point->setText("");
            ui->le_scenario->setStyleSheet("QLineEdit{background-color:green}");
        }
        cur_step = ROBOT_STATE_CHECK_SCENE;
        break;
    }

    case ROBOT_STATE_GRIPPER_START:{
        //
        if(scene[0] == "grasp ready"){
            bool json_rb_val = scene[0].contains(",", Qt::CaseInsensitive);
            if(json_rb_val)
            {
                QStringList grip = scene[0].split(",");
                //                cv::Vec2d gripper_pose;
                //                for(int a=1; a<3; a++) // get from saved json file
                //                {
                //                    gripper_pose[a-1] = gripper[a];
                //                }
                QString text = grip[1]+grip[2];
                QByteArray br = text.toUtf8();
                gripper.Kitech_Client->write(br);
            }

            //            integrate_ui.onSocketWrite("open gripper");
            //        }
            //        else if(scene[0] == "robot_grasp"){
            //            integrate_ui.onSocketWrite("close gripper"); //400
            //            //                scene.pop_front();
            //        }
            cur_step = ROBOT_STATE_GRIPPER_MOVE_CHECK;
            timeout = 600/100;
            break;
        }
    case ROBOT_STATE_GRIPPER_MOVE_CHECK:{

            if(--timeout > 0)
                break;
            cur_step = ROBOT_STATE_GRIPPER_DONE;
            //            if(integrate_ui.robot_state == 3){
            //                qDebug() << "robot moving check";
            //                cur_step = ROBOT_STATE_ROBOT_WAIT;
            //            }else{
            //                qDebug() << "robot not moving";

            //                cur_step = ROBOT_STATE_ROBOT_START;
            //            }

            break;
        }
        case ROBOT_STATE_GRIPPER_WAIT:{
            if(--timeout > 0)
                break;
            //        if(integrate_ui.robot_state == 1){
            //            qDebug() << "robot moving done check";
            //            cur_step = ROBOT_STATE_ROBOT_DONE;
            //            break;
            //        }
            break;
        }

        case ROBOT_STATE_GRIPPER_DONE:{
            if(flag_circle){
                QString last = scene[0];
                scene.pop_front();
                scene.append(last);
            }else{
                scene.pop_front();
            }
            cur_step = ROBOT_STATE_CHECK_SCENE;
            break;
        }
        case ROBOT_STATE_WAIT:{
            //        scene.append("wait");
            //        qDebug()<<"wait";
            //        qDebug()<<"timeout"<<timeout;

            cur_step = ROBOT_STATE_WAIT_OUT;
            timeout = 3000/200;
            break;
        }
        case ROBOT_STATE_WAIT_OUT:{
            //        qDebug()<<"wait out";
            if(--timeout > 0)
            {
                break;

            }
            scene.pop_front();
            cur_step = ROBOT_STATE_CHECK_SCENE;
            break;
        }

        case ROBOT_STATE_NOT_READY:{
            scene.append("not ready");
            //            qDebug()<<"not ready";
            break;
        }

        case ROBOT_STATE_SUCCESS:{
            qDebug()<<"success";
            scene.pop_front();
            web.CMD_RESULT("success");
            web.pick_item_success_count ++;
            cur_step = ROBOT_STATE_CHECK_SCENE;
            break;
        }

        }
        //*/
    }
}

void MainWindow::bt_auto_homing()
{
    auto_homing_clicked = auto_homing_start;
    //     shutter_clicked = vison_cap_vision_cmd;
}

void MainWindow::auto_homming_seq()
{
    //    qDebug()<<"auto_homing_clicked : "<<auto_homing_clicked;

    static int homming_timeout = 0;

    if(auto_homing_clicked == auto_homing_start)
    {
        bt_move_rpm();
        bt_move_rpm();
        bt_move_rpm();

        homming_timeout = 1000/100;
        auto_homing_clicked = auto_homing_lift_wait;
    }
    else if(auto_homing_clicked == auto_homing_lift_wait)
    {
        if(--homming_timeout > 0)
        {
            //            qDebug()<<homming_timeout;
            auto_homing_clicked = auto_homing_lift_wait;
        }
        else
        {
            auto_homing_clicked = auto_homing_lift_working;
        }
    }
    else if(auto_homing_clicked == auto_homing_lift_working)
    {

        QString color = ui -> le_lift_move_status->styleSheet();
        QStringList color_list = color.split(":");

        if(color_list[1] == "green}")
        {
            auto_homing_clicked=auto_homing_start;
        }
        else if(color_list[1] == "red}")
        {
            bt_rpmZero();
            bt_rpmZero();
            auto_homing_clicked = auto_homing_working;
        }
    }
    else if(auto_homing_clicked == auto_homing_working)
    {
        bt_homing();
        bt_homing();
        homming_timeout = 1000/100;
        auto_homing_clicked = auto_homing_wait;
    }
    else if(auto_homing_clicked == auto_homing_wait)
    {
        if(--homming_timeout > 0)
        {
            qDebug()<<homming_timeout;
            auto_homing_clicked=auto_homing_done;
        }
        else
        {
            auto_homing_clicked = auto_homing_done;
        }
    }
    else if(auto_homing_clicked == auto_homing_done)
    {
        //        qDebug()<<"homing done!!!";
        ui->bt_homing->setEnabled(false);
        ui->bt_lift_homimg->setEnabled(false);
        ui->bt_auto_homing->setEnabled(false);
    }
}

void MainWindow::on_pb_shutter_clicked()
{
    //REQ,VISION,DETECT
    //    BTN_GRIPPER_VISION();

    shutter_clicked = vison_cap_gripper_cmd;
    //     shutter_clicked = vison_cap_vision_cmd;
}

void MainWindow::vision_img_capture()
{
    static int vision_hand_timeout = 0;
    switch(shutter_clicked)
    {
    case vison_cap_gripper_cmd:
    {
        BTN_GRIPPER_VISION();
        vision_hand_timeout = 1000/100;
        shutter_clicked = vison_cap_gripper_wait;

        break;
    }
    case vison_cap_gripper_wait:
    {
        shutter_clicked = vison_cap_vision_cmd;

        break;
    }
    case vison_cap_vision_cmd:
    {
        if(--vision_hand_timeout > 0)
        {
            qDebug()<<vision_hand_timeout;
            break;
        }

        if(keti_box_request == false)
        {
            QString obj_cmd = ui->cb_get_object_id_vision->currentText();
            //            QString keti_cmd = "REQ,VISION,DETECT,"+obj_cmd;
            QString keti_cmd = "REQ,VISION,DETECT";
            qDebug()<<keti_cmd;
            vision.Keti_Client->write(keti_cmd.toUtf8());

            IPC::Handeye_FLAG handeye_flag;
            handeye_flag.flag = true;
            ipc.set_handeye_flag(handeye_flag);
            detecting_flag = true;
            shutter_clicked = vison_cap_done;
            break;
        }
        else if(keti_box_request)
        {
            vision.Keti_Client->write("REQ,VISION,BOX_CENT");
            shutter_clicked = vison_cap_done;
            keti_box_request = false;
        }
    }
    case vison_cap_done:
    {
        break;
    }
    }
}

void MainWindow::on_bt_vision_cmd_capture_clicked()
{
    capture_flag = true;
    qDebug()<<"keti capture";
}

void MainWindow::on_BTN_MOVE_JOINT_MID_LEFT_clicked()
{
    /*
    qDebug()<<"mmmmmmmmmiiiiiiiiiiiiiddddddddddddd";
    cobot.MoveJointBlend_Clear();
    cobot.MoveJointBlend_AddPoint(6.075,-13.372,-109.094,57.454,-92.694,-95.605, 2, -1);
    cobot.MoveJointBlend_AddPoint(150.682,0.000,-97.412,9.989,-90.000,-88.900 , 0.5, 0.5);
    cobot.MoveJointBlend_MovePoint();
    //    cobot.MoveJoint(150.682, 0.0, -97.412, 10.0, -90.0 ,-88.90 , 0.5, 0.5);*/

    float spd = 100.0;
    float acc = 70.0;
    float blending_value = 0.2;

    cobot.move_jb2_clear();
    cobot.move_jb2_add(6.075,-13.372,-109.094,57.454,-92.694,-95.605, spd, acc, blending_value);
    cobot.move_jb2_add(150.682,0.000,-97.412,9.989,-90.000,-88.900, spd, acc, blending_value);
    cobot.move_jb2_run();
    //    cobot.MoveJoint(150.682, 0.0, -97.412, 10.0, -90.0 ,-88.90 , 0.5, 0.5);
}

void MainWindow::on_ALL_STOP_clicked()
{
    bt_rpmZero();
    cobot.MotionHalt();
    qDebug()<<"mobile robot stop";

    while (!order_list_msg.empty())
    {
        order_list_msg.pop();
    }
    cobot.ControlBoxDigitalOut(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
    cur_step = ROBOT_STATE_NOT_READY;
    flag_circle = false;

    QJsonObject json_output;
    json_output["MSG_TYPE"] = "STOP";

    QByteArray json_string = QJsonDocument(json_output).toJson(QJsonDocument::Compact);
    mb.cmdSendData(json_string);
    web.CMD_RESULT("cancelled");

}

void MainWindow::on_BTN_MOVE_TOTAL_clicked()
{
    scene.clear();
    //    scene.append("robot init");
    scene.append("lift_down");//리프트 이동
    //    scene.append("mobile move1");// 모바일 이동.

    //when lift high
    scene.append("lift_high");//리프트 이동
    scene.append("robot high vision");
    scene.append("vision");
    //    scene.append("robot high vision point");
    scene.append("robot pick");
    scene.append("robot push");
    scene.append("robot pop");
    scene.append("robot left");
    scene.append("lift_down");
    scene.append("robot pump off");
    //    scene.append("robot done");

    scene.append("lift_down");//리프트 이동
    scene.append("robot mid vision");
    scene.append("vision");
    //    scene.append("robot high vision point");
    scene.append("robot pick");
    scene.append("robot push");
    scene.append("robot pop");
    scene.append("robot right");
    //    scene.append("lift_down");
    scene.append("robot pump off");

    //    scene.append("mobile rest");// 모바일 대기장소.

    cur_step = ROBOT_STATE_START;
}

void MainWindow::on_BTN_MOVE_TOTAL_2_clicked()
{
    //중간층에 물건 하나 있을 때
    scene.clear();
    scene.append("lift_down");//리프트 이동
    scene.append("robot mid vision");
    scene.append("vision");
    scene.append("robot pick");
    scene.append("robot push");
    scene.append("robot pop");
    scene.append("robot right");
    //    scene.append("lift_down");
    scene.append("robot pump off");

    scene.append("mobile rest");// 모바일 대기장소.

    cur_step = ROBOT_STATE_START;
}

void MainWindow::on_BTN_MOVE_TOTAL_3_clicked()
{
    scene.clear();
    //when lift high
    scene.append("lift_high");//리프트 이동
    scene.append("robot high vision");
    scene.append("vision");
    //    scene.append("robot high vision point");
    scene.append("robot pick");
    scene.append("robot push");
    scene.append("robot pop");
    scene.append("robot left");
    scene.append("lift_down");
    scene.append("robot pump off");

    //when lift high
    scene.append("lift_high");//리프트 이동
    scene.append("robot high vision");
    scene.append("vision");
    //    scene.append("robot high vision point");
    scene.append("robot pick");
    scene.append("robot push");
    scene.append("robot pop");
    scene.append("robot right");
    scene.append("lift_down");
    scene.append("robot pump off");

    cur_step = ROBOT_STATE_START;
}

void MainWindow::on_BTN_MOVE_TOTAL_4_clicked()
{
    scene.clear();
    scene.append("lift_mid");//리프트 이동
    scene.append("robot mid vision");
    scene.append("vision");
    //    scene.append("robot high vision point");
    scene.append("robot pick");
    scene.append("robot push");
    scene.append("robot pop");
    scene.append("robot right");
    //    scene.append("lift_down");
    scene.append("robot pump off");

    cur_step = ROBOT_STATE_START;
}

void MainWindow::on_pb_file_choose_clicked()
{

    QString filepath = QFileDialog::getOpenFileName(this,"open","../");
    QString ip = amr_config.AMR_ip;
    QString id = amr_config.AMR_id;
    QString pw = amr_config.AMR_pw;

    QString destination_path = "/home/" + id + "/RB_MOBILE";

    QString cmd = "sshpass -p " + pw + " rsync -avz -e 'ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null' " + filepath + " " + id + "@" + ip + ":" + destination_path;

    qDebug()<<cmd;
    std::system(cmd.toLocal8Bit().data());

    int result = system(cmd.toStdString().c_str());

    if(result != 0)
    {
        //            logger.write("[SERVER] rsync command failed.", true);
    }
    else
    {
        std::cout<<"file transfer done!!"<<std::endl;
    }

}

void MainWindow::on_bt_rb5_pause_clicked()
{
    cobot.MotionPause();
}

void MainWindow::on_bt_rb5_resume_clicked()
{
    cobot.MotionResume();
}

void MainWindow::on_bt_rb5_halt_clicked()
{
    cobot.MotionHalt();
}

void MainWindow::set_ui_item_from_unimap()
{
    //clear
    ui->cb_sn_cmd->clear();

    QDir dir("/home/rainbow/maps");
    foreach(QFileInfo item, dir.entryInfoList(QDir::NoDotAndDotDot | QDir::AllEntries))
    {
        if(item.isDir())
        {
            QString filelist = item.baseName();
            //            qDebug()<<filelist;
            ui->cb_sn_cmd->addItem(filelist);
        }
    }
}

void MainWindow::on_bt_sn_cmd_set_clicked()
{
    QJsonObject json_output;
    json_output["MSG_TYPE"] = "CHANGE MAP";
    json_output["map_name"] = ui->cb_sn_cmd->currentText();

    QByteArray json_string = QJsonDocument(json_output).toJson(QJsonDocument::Compact);

    mb.cmdSendData(json_string);
}

void MainWindow::bt_amr_motor_init()
{
    QJsonObject json_output;
    json_output["MSG_TYPE"] = "INIT";
    QByteArray json_string = QJsonDocument(json_output).toJson(QJsonDocument::Compact);
    mb.cmdSendData(json_string);

}

void MainWindow::on_BTN_show_clicked()
{
    scene.clear();
    scene.append("robot init");
    scene.append("lift_down");//리프트 이동
    scene.append("mobile move1");// 모바일 이동.

    //when lift high
    scene.append("lift_high");//리프트 이동
    scene.append("robot high vision");
    scene.append("vision");
    //    scene.append("robot high vision point");
    scene.append("robot pick");
    scene.append("robot push");
    scene.append("robot pop");
    scene.append("robot left");
    scene.append("lift_down");
    scene.append("robot pump off");
    //    scene.append("robot done");

    scene.append("mobile move2");// 모바일 이동.
    scene.append("lift_down");//리프트 이동
    scene.append("robot mid vision");
    scene.append("vision");
    //    scene.append("robot high vision point");
    scene.append("robot pick");
    scene.append("robot push");
    scene.append("robot pop");
    scene.append("robot right");
    //    scene.append("lift_down");
    scene.append("robot pump off");

    //    scene.append("mobile rest");// 모바일 대기장소.

    cur_step = ROBOT_STATE_START;
    flag_circle =true;

}

//QJsonArray arr;

void MainWindow::bt_save_annot()
{
    cv::Vec3d AMR_pose(mb.pose_x, mb.pose_y, mb.pose_th);
    QString id = ui->cb_shelf_id->currentText();
    QString lift = QString::number(lift_pos);
    QString obj_direction = ui->cb_obj_direction->currentText();

    std::vector<float> rb_5;
    float jnt_1 = ui->LE_JNT_REF_1->text().toFloat();
    float jnt_2 = ui->LE_JNT_REF_2->text().toFloat();
    float jnt_3 = ui->LE_JNT_REF_3->text().toFloat();

    float jnt_4 = ui->LE_JNT_REF_4->text().toFloat();
    float jnt_5 = ui->LE_JNT_REF_5->text().toFloat();
    float jnt_6 = ui->LE_JNT_REF_6->text().toFloat();

    rb_5.push_back(jnt_1);
    rb_5.push_back(jnt_2);
    rb_5.push_back(jnt_3);
    rb_5.push_back(jnt_4);
    rb_5.push_back(jnt_5);
    rb_5.push_back(jnt_6);

    QJsonObject obj;
    obj["shelf_id"] = id+"_"+obj_direction;
    obj["AMR_pose"] = pose_to_array(AMR_pose);
    //    obj["obj_direction"] = obj_direction;
    obj["lift_pose"] = lift;

    obj["RB_5_pose"] = pose_to_array(rb_5);

    QDate date = QDate::currentDate();
    QString dateString = date.toString();

    obj["dateString"] = dateString;// 날짜

    QTime time = QTime::currentTime();
    QString timeString = time.toString();
    obj["timeString"] = timeString;// time

    arr.append(obj);
    QJsonDocument doc(arr);
    QString str_json(doc.toJson(QJsonDocument::Indented));
    ui->te_scen ->setText(str_json);

}

void MainWindow::bt_save_annot_2()
{
    QString obj_id = ui->cb_get_object_id->currentText();
    QString ready_grap = ui->CB_GRIPPER_GRAPS_3->currentText()+","+ui->CB_GRIPPER_GRAPS_3->currentText();
    QString real_grap = ui->CB_GRIPPER_GE_MODE_3->currentText()+","+ui->CB_GRIPPER_GE_GRAPS_3->currentText()+","+ ui->CB_GRIPPER_GE_POWER_3->currentText()+","+ui->CB_GRIPPER_GE_SUCTION_POWER_3->currentText();

    //    QString lift = QString::number(lift_pos);


    QJsonObject obj;
    obj["obj_id"] = obj_id;
    obj["ready_Gripping"] = ready_grap;//2지, 3지, suction 등...
    obj["real_Gripping"] = real_grap;//2지, 3지, suction 등...

    QDate date = QDate::currentDate();
    QString dateString = date.toString();

    obj["dateString"] = dateString;// 날짜

    QTime time = QTime::currentTime();
    QString timeString = time.toString();
    obj["timeString"] = timeString;// time

    arr1.append(obj);
    QJsonDocument doc(arr1);
    QString str_json(doc.toJson(QJsonDocument::Indented));
    ui->te_scen_2 ->setText(str_json);
}

void MainWindow::bt_delete_annot()
{
    arr.pop_back();
    QJsonDocument doc(arr);
    QString str_json(doc.toJson(QJsonDocument::Indented));
    ui->te_scen ->setText(str_json);
}

void MainWindow::bt_save_all_annot()
{
    QString annot_path = QDir::homePath() + "/order_json/shelf_infomation.json";
    QFile file(annot_path);
    if(file.open(QIODevice::Append))
    {
        QJsonDocument doc(arr);
        file.write(doc.toJson());
        file.close();

        printf("[save_annot] %s saved\n", annot_path.toStdString().c_str());
    }
}

void MainWindow::bt_delete_annot_2()
{
    arr1.pop_back();
    QJsonDocument doc(arr1);
    QString str_json(doc.toJson(QJsonDocument::Indented));
    ui-> te_scen_2 ->setText(str_json);
}

void MainWindow::bt_save_all_annot_2()
{
    //    shelf_infomation
    QString annot_path = QDir::homePath() + "/order_json/object_infomation.json";
    QFile file(annot_path);
    if(file.open(QIODevice::Append))
    {
        QJsonDocument doc(arr1);
        file.write(doc.toJson());
        file.close();

        printf("[save_annot] %s saved\n", annot_path.toStdString().c_str());
    }
}

void MainWindow::bt_order()
{
    QJsonObject json;
    QJsonObject json_parm;
    QJsonObject json_destination_box;

    QString shelf_hight = ui->le_shelf_hight->text();
    QString count = ui -> CB_gripper_num -> currentText();
    QString shelf_name = ui->CB_shelf->currentText()+"_"+ui->cb_obj_direction_2->currentText();
    QString obj_name = ui->cb_get_object_id->currentText();

    json["msg_type"] = "command";
    json["entry"] = "manipulator";
    json["do"] = "pick_item";

    json_parm["item_count"] = count.toDouble();
    json_parm["shelve_height"] = shelf_hight.toDouble();
    json_parm["shelve_degree"] = 45;
    json_parm["shelve_name"] = shelf_name;

    //db 에 저장되어있는 내용
    json_destination_box["name"] = obj_name;
    json_destination_box["id"] = obj_name;
    json_parm["destination_box"] = json_destination_box;
    json["params"] = json_parm;

    QJsonDocument doc_json(json);
    QString str_json(doc_json.toJson(QJsonDocument::Indented));

    // order msg kipping
    mtx.lock();
    order_json_msg.push(str_json);
    mtx.unlock();

    bt_order_check();
    ui -> te_order -> append(str_json);
}

void MainWindow::bt_order_check()
{
    // order msg save -> parsing -> making parsing msg to qstringlist -> que astringlist
    //    qDebug()<<order_json_msg.front();

    //    QString shelf_hight = ui->le_shelf_hight->text();
    int count = ui -> CB_gripper_num -> currentText().toInt();
    //    QString shelf_name = ui->CB_shelf->currentText();

    for(auto& it: shelf_infos) // get from saved json
    {
        if(it.second == nullptr)
        {
            continue;
        }

        QStringList order_msg;
        QString new_msg = order_json_msg.front();
        QJsonDocument doc_json = QJsonDocument::fromJson(new_msg.toUtf8());
        QJsonObject json = doc_json.object();

        QVariantMap json_pa = json.toVariantMap();
        QVariantMap json_parm = json_pa["params"].toMap();
        QVariantMap json_destination_box = json_parm["destination_box"].toMap();

        float shelve_height = json_parm["shelve_height"].toDouble();
        //        shelf_info->RB_5_pose;

        if (order_list_msg.size() == 0)
        {
            qDebug()<<"msg order que is 0!!!";
            que_time = 0;
        }

        // 명령에 맞게 리스트를 만든후 que 에 넣음. -> loop start ->

        //        qDebug()<<json_parm["shelve_name"].toString()+"_"+json_destination_box["name"].toString();
        /*  qDebug()<<it.second->shelf_id;
        qDebug()<<json_parm["shelve_name"].toString()+"_"+QString::number(shelve_height);
        */

        if (it.second->shelf_id == json_parm["shelve_name"].toString()+"_"+QString::number(shelve_height))
        {
            //            qDebug()<<it.second->shelf_id;
            // 정해진 위치에 해당하는 명령이 들어온다면.
            if (it.second->lift_pose == shelve_height)
            {
                qDebug()<<"같은 위치 찾음.";
                std::cout<<it.second->RB_5_pose[0]<<std::endl;
                QString lift_high = "lift_high,"+QString::number(shelve_height);
                order_msg.append(lift_high);//리프트 이동

                // vision robot pose -> robot pose blend
                QString robot_vision = "robot vision,"+QString::number(it.second->RB_5_pose[0])+","+QString::number(it.second->RB_5_pose[1])+","+QString::number(it.second->RB_5_pose[2])+","+QString::number(it.second->RB_5_pose[3])+","+QString::number(it.second->RB_5_pose[4])+","+QString::number(it.second->RB_5_pose[5]);
                //                qDebug()<<"robot moving :" <<robot_vision;
                order_msg.append(robot_vision);
                //                                order_msg.append("robot_high_vision");

                //                order_msg.append("wait");
                // 물체가 박스에 있는 경우에만 사용.
                if(lift_high > 600)
                {
                    order_msg.append("robot vision box center");
                }

                order_msg.append("wait");
                order_msg.append("vision");
                order_msg.append("wait");
                order_msg.append("robot approach");
                order_msg.append("robot pick");
                //                order_msg.append("wait");
                order_msg.append("robot push");
                order_msg.append("wait");
                //                order_msg.append("robot pump on");
                //                order_msg.append("robot pop");
                //                //                order_msg.append("robot pump on");
                //                order_msg.append("robot mid left");

                //                QString lift_down = "lift_high,5";
                //                order_msg.append(lift_down);//리프트 이동
                //                order_msg.append("robot pump off");
                //                order_msg.append("success");

                if (count!=1)
                {
                    QStringList msg;
                    for(int i=0;i<count;i++)
                    {
                        msg+=order_msg;
                    }
                    qDebug()<<"msg :"<<msg;
                    order_msg=msg;
                }
            }
            else //만약 정해진 위치가 아닌 다른 위치가 들어온다면.
            {
                qDebug()<<"위치 다름.";
                QString lift_high = "lift_high,"+QString::number(shelve_height);
                order_msg.append(lift_high);//리프트 이동
                //                order_msg.append("scene done");//리프트 이동
            }

            for(auto& it: object_infos) // get propreate gripping position hands data
            {
                if(it.second == nullptr)
                {
                    continue;
                }

                if (it.second->obj_id == json_destination_box["name"].toString())
                {

                    //                        std::cout<<it.second->RB_5_pose[0]<<std::endl;
                    qDebug()<<"grap pose : "<<it.second->obj_grap_pose;
                    qDebug()<<"ready grap pose :"<<it.second->obj_ready_grap_pose;
                    //                        order_msg.append("");
                    //                        order_msg.append("");
                    QString grasp_ready = "grasp ready,"+it.second->obj_ready_grap_pose;
                    //                qDebug()<<"robot moving :" <<robot_vision;
                    order_msg.append(grasp_ready);
                    order_msg.append("wait");
                    order_msg.append("robot pump on");

                    QString real_grasp = "grasp real,"+it.second->obj_grap_pose;
                    //                qDebug()<<"robot moving :" <<robot_vision;
                    order_msg.append(real_grasp);
                    order_msg.append("robot pop");
                    //                order_msg.append("robot pump on");
                    order_msg.append("robot mid left");

                    QString lift_down = "lift_high,5";
                    order_msg.append(lift_down);//리프트 이동
                    order_msg.append("robot pump off");
                    order_msg.append("success");
                }
            }

            //            qDebug()<<"order_msg :"<<order_msg;
            mtx.lock();
            order_list_msg.push(order_msg);
            mtx.unlock();
            //                qDebug()<<"나는 큐의 크기를 알고싶다 : "<<order_list_msg.size();
            //                cur_step = ROBOT_STATE_START;
            flag_circle = false;
            old_msg_size = order_list_msg.size();

        }
    }
    order_json_msg.pop();// loop 안에서 pop해야함.
}

void MainWindow::BTN_GRIPPER_recog_WRITE()
{
    QString txt = ui->CB_GRIPPER_recog->currentText();
    qDebug()<<"txt : "<<txt;
    gripper.Kitech_Client->write(txt.toStdString().c_str());

}

void MainWindow::on_BTN_MOVE_JOINT_POP_clicked()
{
    //    cobot.MoveJoint(ui->LE_JNT_ENC_1->text().toFloat()-30,ui->LE_JNT_ENC_2->text().toFloat()+30,ui->LE_JNT_ENC_3->text().toFloat()
    //                    ,ui->LE_JNT_ENC_4->text().toFloat(),ui->LE_JNT_ENC_5->text().toFloat(),ui->LE_JNT_ENC_6->text().toFloat(), 0.2, 0.2);


    if (lift_pos>500)
    {
        cobot.MoveTCP(ui->LE_TCP_REF_X->text().toFloat()+300,ui->LE_TCP_REF_Y->text().toFloat(),ui->LE_TCP_REF_Z->text().toFloat(),
                      ui->LE_TCP_REF_RX->text().toFloat(), ui->LE_TCP_REF_RY->text().toFloat(), ui->LE_TCP_REF_RZ->text().toFloat(), 2, -1);
    }
    else
    {
        cobot.MoveTCP(ui->LE_TCP_REF_X->text().toFloat()+100,ui->LE_TCP_REF_Y->text().toFloat(),ui->LE_TCP_REF_Z->text().toFloat()+300,
                      ui->LE_TCP_REF_RX->text().toFloat(), ui->LE_TCP_REF_RY->text().toFloat(), ui->LE_TCP_REF_RZ->text().toFloat(), 2, -1);
    }
}

void MainWindow::on_BTN_MOVE_JOINT_BOX_CENTER_clicked()
{
    qDebug()<<"go to Center";
    IPC::BOX_CENTER_POSE box_center_pose = ipc.get_box_center();

    ui -> spb_Tx->setValue(ui->LE_TCP_REF_X->text().toFloat()/1000);
    ui -> spb_Ty->setValue(ui->LE_TCP_REF_Y->text().toFloat()/1000);
    ui -> spb_Tz->setValue(ui->LE_TCP_REF_Z->text().toFloat()/1000);

    ui -> spb_Rx->setValue(ui->LE_TCP_REF_RX->text().toFloat());
    ui -> spb_Ry->setValue(ui->LE_TCP_REF_RY->text().toFloat());
    ui -> spb_Rz->setValue(ui->LE_TCP_REF_RZ->text().toFloat());

    double tx = ui->spb_Tx->value(); // m
    double ty = ui->spb_Ty->value();
    double tz = ui->spb_Tz->value();

    double rz = ui->spb_Rz->value(); // deg
    double ry = ui->spb_Ry->value();
    double rx = ui->spb_Rx->value();

    QString str; //qstring에 sprintf 로 한방에 집어넣을 수 있음.
    str.sprintf("%f,%f,%f,%f,%f,%f", tx, ty, tz, rx, ry, rz);
    qDebug()<<str;

    T_sensor = string_to_TF(str); //translate matrix

    Eigen::Vector3d P(box_center_pose.x, box_center_pose.y, box_center_pose.z);
    //        std::cout<<"box : "<<P<<std::endl;
    //        std::cout <<"t_sensor : "<<T_sensor<<std::endl;
    Eigen::Vector3d _P = T_sensor.block(0,0,3,3)*P+T_sensor.block(0,3,3,1);

    //    std::cout<<_P[0]<<_P[1]<<_P[2]<<std::endl;
    QString res_x,res_y,res_z;
    res_x.sprintf("%.f",(_P[0])*1000);
    res_y.sprintf("%.f",(_P[1])*1000);
    res_z.sprintf("%.f",(_P[2])*1000);

    ui->move_x_val->setText(res_x);
    ui->move_y_val->setText(res_y);
    ui->move_z_val->setText(res_z);

    float move_rx_valo = ui->LE_TCP_REF_RX->text().toFloat();
    //    float move_ry_valo = ui->LE_TCP_REF_RY->text().toFloat();
    //    float move_ry_valo = ui->LE_TCP_REF_RY->text().toFloat();
    float move_ry_valo = -21;
    //    float move_ry_valo = -(90-35);
    float move_rz_valo = ui->LE_TCP_REF_RZ->text().toFloat();

    cobot.MoveTCP(res_x.toFloat(),res_y.toFloat(),res_z.toFloat(), move_rx_valo, move_ry_valo, move_rz_valo, 2, -1);
}

void MainWindow::on_BTN_TCP_MOTION_TEST_clicked()
{
    QString LE_TCP_LIST = ui->LE_TCP_LIST->text();
    if (LE_TCP_LIST.size()==0)
    {
        qDebug()<<"로봇이 움직일 수 없습니다.";
    }
    else
    {
        qDebug()<<"TCP 좌표계로 이동합니다.";

        QStringList list = LE_TCP_LIST.split(",");
        if(list.size()==6)
        {
            double tx = list[0].toDouble();
            double ty = list[1].toDouble();
            double tz = list[2].toDouble();

            double rx = list[3].toDouble();
            double ry = list[4].toDouble();
            double rz = list[5].toDouble();

            cobot.MoveTCP(tx,ty,tz,rx,ry,rz,2,-1);
        }
        else
        {
            qDebug()<<"이동 좌표 개수 확인";
        }
    }
}

void MainWindow::on_BTN_JOINT_MOTION_TEST_clicked()
{
    qDebug()<<"Joint 좌표계로 이동합니다.";
    QString LE_JOINT_LIST = ui->LE_JOINT_LIST->text();
    if (LE_JOINT_LIST.size()==0)
    {
        qDebug()<<"로봇이 움직일 수 없습니다.";
    }
    else
    {
        QStringList list = LE_JOINT_LIST.split(",");
        if(list.size() == 6)
        {
            double r1 = list[0].toDouble();
            double r2 = list[1].toDouble();
            double r3 = list[2].toDouble();
            double r4 = list[3].toDouble();
            double r5 = list[4].toDouble();
            double r6 = list[5].toDouble();

            cobot.MoveJoint(r1,r2,r3,r4,r5,r6,2,-1);
        }
        else
        {
            qDebug()<<"이동 좌표 개수 확인";
        }
    }
}

void MainWindow::on_pb_reload_clicked()
{
    //    qDebug()<<"reload rb_5 joint value";

    float j1 = cobot.systemStat.sdata.jnt_ang[0];
    float j2 = cobot.systemStat.sdata.jnt_ang[1];
    float j3 = cobot.systemStat.sdata.jnt_ang[2];
    float j4 = cobot.systemStat.sdata.jnt_ang[3];
    float j5 = cobot.systemStat.sdata.jnt_ang[4];
    float j6 = cobot.systemStat.sdata.jnt_ang[5];

    QString LE_JOINT_LIST = QString::number(j1)+","+QString::number(j2)+","+QString::number(j3)+","+QString::number(j4)+","+QString::number(j5)+","+QString::number(j6);

    float tcp_x = cobot.systemStat.sdata.tcp_ref[0];
    float tcp_y = cobot.systemStat.sdata.tcp_ref[1];
    float tcp_z = cobot.systemStat.sdata.tcp_ref[2];
    float tcp_rx = cobot.systemStat.sdata.tcp_ref[3];
    float tcp_ry = cobot.systemStat.sdata.tcp_ref[4];
    float tcp_rz = cobot.systemStat.sdata.tcp_ref[5];
    //LE_TCP_LIST
    QString LE_TCP_LIST = QString::number(tcp_x)+","+QString::number(tcp_y)+","+QString::number(tcp_z)+","+QString::number(tcp_rx)+","+QString::number(tcp_ry)+","+QString::number(tcp_rz);


    //    qDebug()<<LE_TCP_LIST;
    ui -> LE_TCP_LIST -> setText(LE_TCP_LIST);
    ui -> LE_JOINT_LIST -> setText(LE_JOINT_LIST);
}

void MainWindow::on_bt_scen_resume_clicked()
{
    scene.clear();
    scene.append("robot pump off");
    scene.append("scene done");
    cur_step = ROBOT_STATE_CHECK_SCENE;

    //    yujin_scene.clear();
    //    yujin_scene.append("robot pump off");
    //    yujin_scene.append("SCENE DONE");
    //    cur_step_yj = FMS_ROBOT_STATE_CHECK_SCENE;
}

void MainWindow::lb_keti_point(QString msg)
{

    if(msg!="OBJ_NONE")
    {
        ui -> spb_Tx->setValue(ui->LE_TCP_REF_X->text().toFloat()/1000);
        ui -> spb_Ty->setValue(ui->LE_TCP_REF_Y->text().toFloat()/1000);
        ui -> spb_Tz->setValue(ui->LE_TCP_REF_Z->text().toFloat()/1000);

        ui -> spb_Rx->setValue(ui->LE_TCP_REF_RX->text().toFloat());
        ui -> spb_Ry->setValue(ui->LE_TCP_REF_RY->text().toFloat());
        ui -> spb_Rz->setValue(ui->LE_TCP_REF_RZ->text().toFloat());

        ui->lb_keti_point->setText(msg);

        keti_x = vision.keti_x;
        keti_y = vision.keti_y;
        keti_z = vision.keti_z;

        //        std::cout<<"keti : "<<keti_x<<","<<keti_y<<","<<keti_z<<","<<std::endl;

        Eigen::Vector3d P(keti_x, keti_y, keti_z);
        Eigen::Vector3d _P = T_sensor.block(0,0,3,3)*P+T_sensor.block(0,3,3,1);

        QString res_x,res_y,res_z;
        res_x.sprintf("%.3f",(_P[0])*1000);
        res_y.sprintf("%.3f",(_P[1])*1000);
        res_z.sprintf("%.3f",(_P[2])*1000);

        ui->move_x_val->setText(res_x);
        ui->move_y_val->setText(res_y);
        ui->move_z_val->setText(res_z);

        //////////////////////////////////////////////////////

        //for approach

        Eigen::Vector3d app_P(vision.keti_app_x, vision.keti_app_y, vision.keti_app_z);
        Eigen::Vector3d app_trance_P = T_sensor.block(0,0,3,3)*app_P+T_sensor.block(0,3,3,1);

        app_x = app_trance_P[0]*1000;
        app_y = app_trance_P[1]*1000;
        app_z = app_trance_P[2]*1000;

        //////////////////////////////////////////////////////

        if(vision.box_cent_value)
        {
            cent_res_x = res_x;
            cent_res_y = res_y;
            cent_res_z = res_z;

            float move_rx_valo = ui->LE_TCP_REF_RX->text().toFloat();
            float move_ry_valo = ui->LE_TCP_REF_RY->text().toFloat();
            float move_rz_valo = ui->LE_TCP_REF_RZ->text().toFloat();

            ui->move_rx_val->setText(QString::number(move_rx_valo));
            ui->move_ry_val->setText(QString::number(move_ry_valo));
            ui->move_rz_val->setText(QString::number(move_rz_valo));

            bt_cobot_move2object();

            //            vision.box_cent_value = false;
        }
        else
        {
            //for normal check

            QString tcp_rot_mat = mat_zyxzy();
            QStringList val_list = tcp_rot_mat.split(",");
            //            qDebug()<<"sssssssssss:"<<tcp_rot_mat;

            ui->move_rx_val->setText(val_list[0]);
            ui->move_ry_val->setText(val_list[1]);
            ui->move_rz_val->setText(val_list[2]);

            QString res_rx,res_ry,res_rz;
            res_rx.sprintf("%.3f",val_list[0].toFloat());
            res_ry.sprintf("%.3f",val_list[1].toFloat());
            res_rz.sprintf("%.3f",val_list[2].toFloat());

            ui->move_rx_val->setText(res_rx);
            ui->move_ry_val->setText(res_ry);
            ui->move_rz_val->setText(res_rz);

            //            ui->move_rx_val->setText("89");
            //            ui->move_ry_val->setText("-45");
            //            ui->move_rz_val->setText("-179");
        }
    }
    else
    {
        ui->lb_keti_point->setText(msg);
    }
}

void MainWindow::on_BTN_MOVE_JOINT_BOX_CENTER_KETI_clicked()
{
    //    IPC::BOX_CENTER_POSE box_center_pose = ipc.get_box_center();
    //    vision.Keti_Client->write("REQ,VISION,BOX_CENT");
    //    shutter_clicked = vison_cap_gripper_cmd;
    shutter_clicked = vison_cap_gripper_cmd;
    keti_box_request = true;
}

QString MainWindow::mat_zyxzy()
{
    // rz1, ry1, rx1 -> base 기준으로 tcp 의 회전각
    keti_rx = vision.keti_rx;
    keti_ry = vision.keti_ry;
    keti_rz = vision.keti_rz;

    double rz1 = ui->LE_TCP_REF_RZ->text().toFloat()*D2R;
    double ry1 = ui->LE_TCP_REF_RY->text().toFloat()*D2R;
    double rx1 = ui->LE_TCP_REF_RX->text().toFloat()*D2R;

    int rot_sign = -1;
    double rz2,rx2;

    float dist_a = sqrt(pow(keti_rz,2)+pow(keti_ry,2)+pow(keti_rx,2));
    float dist_b = keti_rz;
    float dist_c = sqrt(pow(keti_ry,2)+pow(keti_rx,2));
    float x_rot = acos((pow(dist_b,2)-(pow(dist_a,2)+pow(dist_c,2)))/(-2*dist_a*dist_c));

    //    qDebug()<<"x_rot : "<<x_rot;

    if(keti_rx>0 && keti_rz>0)
    {
        rz2 = rot_sign*atan(keti_rx/keti_ry);
        rx2 = abs(x_rot);
    }
    else  if(keti_rx>0&& keti_rz<0)
    {
        rz2 = rot_sign*atan(keti_rx/keti_ry);
        rx2 = rot_sign*x_rot;
    }
    else if(keti_rx<0 && keti_rz<0)
    {
        rz2 = abs(atan(keti_rx/keti_ry));
        rx2 = rot_sign*x_rot;
    }
    else if(keti_rx<0 && keti_rz>0)
    {
        rz2 = abs(atan(keti_rx/keti_ry));
        rx2 = x_rot;
    }
    double ry2 = vision.res_wrist_rot.toFloat()*D2R;
    //        double ry2 = 0.0*D2R;

    //    // 리프트 위치로 손목 관절 각도 회전시킴.
    ////    int pos = ui->sb_lift_pos->value();

    //    double ry2 = vision.res_wrist_rot.toFloat()*D2R;
    //    qDebug()<<"wrist angle : "<<ry2;

    // rx, ry, rz must be radian
    Eigen::AngleAxisd Rz1(rz1, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd Ry1(ry1, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd Rx1(rx1, Eigen::Vector3d::UnitX());

    Eigen::AngleAxisd Rz2(rz2, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd Ry2(ry2, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd Rx2(rx2, Eigen::Vector3d::UnitX());

    Eigen::Quaternion<double> base2tcp_rot = Rz1 * Ry1 * Rx1;
    Eigen::Matrix4d res1;
    res1.setIdentity();
    res1.block(0,0,3,3) = base2tcp_rot.matrix();
    res1(0,3) = 0;
    res1(1,3) = 0;
    res1(2,3) = 0;
    res1(3,3) = 1;

    Eigen::Quaternion<double> tcp2obj_rot = Rz2 * Rx2 * Ry2;
    Eigen::Matrix4d res2;
    res2.setIdentity();
    res2.block(0,0,3,3) = tcp2obj_rot.matrix();
    res2(0,3) = 0;
    res2(1,3) = 0;
    res2(2,3) = 0;
    res2(3,3) = 1;

    Eigen::Matrix4d resres = res1*res2;

    Eigen::Vector6d euler = TF_to_ZYX(resres);

    double euler_rx = euler[3]*R2D;
    double euler_ry = euler[4]*R2D;
    double euler_rz = euler[5]*R2D;

    QString euler_val;
    euler_val.sprintf(" %f,%f,%f",euler_rx,euler_ry,euler_rz);
    qDebug()<<euler_val;

    //    Eigen::Quaternion<double> q = Rz1 * Ry1 * Rx1 * Rz2 * Ry2 * Rx2;

    //    Eigen::Matrix3d res;
    //    res.setIdentity();
    //    res.block(0,0,3,3) = q.matrix();

    //and must be compare with Euler angle.

    return euler_val;
}

void MainWindow::on_bt_cobot_move2object_approach_clicked()
{

    //    on_BTN_GRIPPER_SUCTION_clicked();

    ui -> spb_Tx->setValue(ui->LE_TCP_REF_X->text().toFloat()/1000);
    ui -> spb_Ty->setValue(ui->LE_TCP_REF_Y->text().toFloat()/1000);
    ui -> spb_Tz->setValue(ui->LE_TCP_REF_Z->text().toFloat()/1000);

    ui -> spb_Rx->setValue(ui->LE_TCP_REF_RX->text().toFloat());
    ui -> spb_Ry->setValue(ui->LE_TCP_REF_RY->text().toFloat());
    ui -> spb_Rz->setValue(ui->LE_TCP_REF_RZ->text().toFloat());

    QString color = ui->le_vision_connection->styleSheet();
    QStringList color_list = color.split(":");
    if(color_list[1] == "green")
    {
        float move_rx_valo = ui->move_rx_val->text().toFloat();
        float move_ry_valo = ui->move_ry_val->text().toFloat();
        float move_rz_valo = ui->move_rz_val->text().toFloat();

        if(abs(abs(ui->LE_TCP_REF_X->text().toFloat())-abs(app_x))<250)
        {
            caution_flag = false;
            ui -> la_caution->setText("");
            cobot.MoveTCP(app_x,app_y,app_z, move_rx_valo, move_ry_valo, move_rz_valo, 0.5, -1);
        }
        else
        {
            caution_flag = true;
            ui -> la_caution->setText("prohibited area!");
        }
        //        ui -> la_caution->setText("");
        //        cobot.MoveTCP(app_x,app_y,app_z, move_rx_valo, move_ry_valo, move_rz_valo, 0.5, -1);
    }
    else if(color_list[1] == "red")
    {
        float move_rx_valo = ui->move_rx_val->text().toFloat();
        float move_ry_valo = ui->move_ry_val->text().toFloat();
        float move_rz_valo = ui->move_rz_val->text().toFloat();

        cobot.MoveTCP(app_x,app_y,app_z, move_rx_valo, move_ry_valo, move_rz_valo, 0.5, -1);

    }
}

void MainWindow::on_BTN_MOVEL_rel_clicked()
{
    QString LE_TCP_LIST = ui->LE_TCP_LIST->text();

    QStringList list = LE_TCP_LIST.split(",");
    if(list.size()==6)
    {
        double tx = list[0].toDouble();
        double ty = list[1].toDouble();
        double tz = list[2].toDouble();

        double rx = list[3].toDouble();
        double ry = list[4].toDouble();
        double rz = list[5].toDouble();

        cobot.MoveL_rel(tx,ty,tz,rx,ry,rz,2,-1);
    }

}

void MainWindow::on_BTN_quick_return_clicked()
{
    cobot.MoveJoint(0.228,19.684,-73.543,-15.062,-89.891,-89.646, 2, -1);
}

void MainWindow::on_BTN_GRIPPER_SUCTION_clicked()
{
    if(Integrated_info.init_gripper == true)
    {
        QString text = "Rs";
        QByteArray br = text.toUtf8();
        gripper.Kitech_Client->write(br);
    }
    else{

    }
}

void MainWindow::on_BTN_GRIPPER_OPEN_clicked()
{
    if(Integrated_info.init_gripper == true)
    {
        QString text = "Rg5";
        QByteArray br = text.toUtf8();
        gripper.Kitech_Client->write(br);
    }
    else
    {

    }
}


void MainWindow::save_log(QString log)
{
    //    shelf_infomation


    //    QTime time = QTime::QDateTime();
    QString time_Format = "yyyy-MM-dd";
    QDateTime a = QDateTime::currentDateTime();
    QString as = a.toString(time_Format);
    //    QString timeString = time.toString();

    QString annot_path = QDir::homePath() + "/order_log/"+as+ ".ini";
    QFile file(annot_path);
    if(file.open(QIODevice::Append))
    {
        QTextStream out(&file);
        out << log<< "\n";

        //        file.write(out);
        file.close();
    }
}


void MainWindow::on_BTN_MOVE_JOINT_LOW_LEFT_clicked()
{
    cobot.MoveJointBlend_Clear();
    cobot.MoveJointBlend_AddPoint(99.666,3.839,-71.312,-20.712,-94.225,-88.891, 0.5, 0.5);
    //    cobot.MoveJointBlend_AddPoint(130.194,9.992,-83.168,-4.382,-89.682,-88.651, 0.5, 0.5);
    //    cobot.MoveJointBlend_AddPoint(130.192,0.146,-83.160,-4.391,-89.678,-88.649, 0.2, 0.2);
    cobot.MoveJointBlend_AddPoint(150.207,-3.357,-64.440,-19.582,-89.706,-88.582, 0.2, 0.2);
    //    cobot.MoveJointBlend_AddPoint(150.682,0.000,-97.412,9.989,-90.000,-88.900 , 0.2, 0.2);
    cobot.MoveJointBlend_MovePoint();
}

void MainWindow::on_BTN_RETURN_MOVE_JOINT_BOX_CENTER_clicked()
{
    float move_x_valo = cent_res_x.toFloat();
    float move_y_valo = cent_res_y.toFloat();
    float move_z_valo = cent_res_z.toFloat();

    float move_rx_valo = ui->move_rx_val->text().toFloat();
    float move_ry_valo = ui->move_ry_val->text().toFloat();
    float move_rz_valo = ui->move_rz_val->text().toFloat();
    cobot.MoveTCP(move_x_valo,move_y_valo,move_z_valo, move_rx_valo, move_ry_valo, move_rz_valo, 0.5, -1);
}


void MainWindow::vision_trigger(){
    on_pb_shutter_clicked();
}
