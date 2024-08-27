/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QCommandLinkButton>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QTextBrowser>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include <mygraphicsview.h>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QTabWidget *scenario;
    QWidget *tab;
    QGroupBox *groupBox_5;
    QLineEdit *le_rb5_data_connection;
    QLabel *label_114;
    QLineEdit *le_vision_connection;
    QLineEdit *le_lift_connection;
    QLabel *label_115;
    QLabel *label_116;
    QLineEdit *le_gripper_connection;
    QLabel *label_117;
    QLabel *label_118;
    QLineEdit *le_mobile_connection;
    QLineEdit *le_rb5_cmd_connection;
    QLabel *label_119;
    QLineEdit *le_websocket_connection;
    QLabel *label_148;
    MyGraphicsView *lb_sn_map;
    QGroupBox *GE_MOVE_INFO;
    QLineEdit *le_mobile_move_status;
    QLineEdit *le_lift_move_status;
    QLineEdit *le_rb5_move_status;
    QLabel *label_120;
    QLabel *label_136;
    QLabel *label_138;
    QLabel *label_177;
    QLineEdit *le_pick;
    QGroupBox *groupBox_7;
    QPushButton *bt_lift_homimg;
    QPushButton *bt_lift_stop;
    QPushButton *bt_lift_pos_move;
    QSpinBox *sb_lift_pos;
    QCheckBox *cb_lift_dir_homing;
    QSpinBox *sb_lift_pos_rpm;
    QPushButton *bt_lift_rpm_set;
    QWidget *layoutWidget;
    QVBoxLayout *verticalLayout;
    QPushButton *bt_lift_top;
    QPushButton *bt_lift_MID;
    QPushButton *bt_lift_Low;
    QGroupBox *groupBox_8;
    QSpinBox *sb_rb5_speed;
    QSlider *sld_rb5_speed;
    QPushButton *bt_rb5_mode;
    QPushButton *bt_rb5_pause;
    QPushButton *bt_rb5_resume;
    QPushButton *bt_rb5_halt;
    QPushButton *bt_rb5_pos_move;
    QComboBox *cb_rb5_pos;
    QLabel *lb_rb5_speed_percentage;
    QLabel *label_150;
    QLabel *label_151;
    QLineEdit *robot_status;
    QPushButton *btn_pump_2;
    QGroupBox *groupBox_9;
    QComboBox *cb_sn_cmd;
    QPushButton *bt_sn_cmd_set;
    QLabel *label_162;
    QPushButton *bt_amr_motor_init;
    QGroupBox *groupBox_10;
    QLabel *lb_vision_capture_img;
    QPushButton *bt_vision_cmd_capture;
    QLineEdit *le_gripper_cmd;
    QPushButton *bt_gripper_cmd_move;
    QGroupBox *groupBox_12;
    QPushButton *bt_png_change;
    QPushButton *bt_zip;
    QLabel *label_142;
    QPushButton *bt_get_map_from_mobile;
    QComboBox *cb_get_map_data;
    QLabel *label_161;
    QLineEdit *le_text;
    QPushButton *pb_file_choose;
    QGroupBox *groupBox_11;
    QTextEdit *te_websocket;
    QLineEdit *le_barcode_connection;
    QLabel *label_149;
    QPushButton *bt_tcp_on;
    QLineEdit *LE_INFO_MOVE_INFO_2;
    QLabel *label_141;
    QLabel *label_166;
    QWidget *layoutWidget1;
    QGridLayout *gridLayout_12;
    QLabel *label_154;
    QLabel *label_132;
    QDoubleSpinBox *spb_Ty;
    QDoubleSpinBox *spb_Ry;
    QLabel *label_160;
    QLabel *label_155;
    QLabel *label_157;
    QLabel *label_156;
    QDoubleSpinBox *spb_Tz;
    QDoubleSpinBox *spb_Tx;
    QLabel *label_158;
    QLabel *label_140;
    QDoubleSpinBox *spb_Rz;
    QDoubleSpinBox *spb_Rx;
    QWidget *layoutWidget2;
    QGridLayout *gridLayout_13;
    QLabel *move_x;
    QLineEdit *move_x_val;
    QLabel *move_y;
    QLineEdit *move_y_val;
    QLabel *move_z;
    QLineEdit *move_z_val;
    QLabel *move_rx;
    QLineEdit *move_rx_val;
    QLabel *move_ry;
    QLineEdit *move_ry_val;
    QLabel *move_rz;
    QLineEdit *move_rz_val;
    QPushButton *bt_vision_cmd_capture_4;
    QPushButton *bt_TCP_Blend;
    QWidget *layoutWidget3;
    QGridLayout *gridLayout_14;
    QLabel *label_159;
    QTextEdit *te_log;
    QPushButton *bt_low_get_out;
    QPushButton *bt_vision_water;
    QPushButton *bt_avino_low_get_out_2;
    QPushButton *bt_vision_high_get_out;
    QPushButton *bt_vision_rael_more_move;
    QPushButton *bt_low_get_out_2;
    QLineEdit *LE_INFO_MOVE_INFO_1;
    QLabel *label_139;
    QLabel *label_181;
    QLineEdit *le_scenario;
    QWidget *layoutWidget4;
    QHBoxLayout *horizontalLayout_14;
    QPushButton *bt_cobot_move2object_approach;
    QPushButton *bt_cobot_move2object;
    QPushButton *bt_cobot_push;
    QLineEdit *la_caution;
    QWidget *rb_5;
    QSlider *HS_BASE_SPEED;
    QGroupBox *GB_NETWORK;
    QLineEdit *LE_IP;
    QLabel *label_8;
    QLineEdit *textCmdPort;
    QLabel *label_10;
    QLabel *label_11;
    QLineEdit *textDataPort;
    QPushButton *BTN_CONNECT_COM;
    QPushButton *BTN_CONNECT_DATA;
    QGroupBox *groupBox_4;
    QPushButton *BTN_COBOT_INIT;
    QLineEdit *LE_INIT_POWER;
    QLabel *label_12;
    QLabel *label_13;
    QLineEdit *LE_INIT_DEVICE;
    QLabel *label_14;
    QLineEdit *LE_INIT_SYSTEM;
    QLineEdit *LE_INIT_ROBOT;
    QLabel *label_15;
    QGroupBox *groupBox_6;
    QTextEdit *textPrint;
    QPushButton *BTN_CLEAR;
    QLabel *label_16;
    QPushButton *BTN_MOTION_RESUME;
    QPushButton *BTN_MOVE_JOINT_READY_GRIPPER;
    QPushButton *BTN_MOTION_HALT;
    QFrame *FRAME_STATUS;
    QWidget *gridLayoutWidget_4;
    QGridLayout *gridLayout_4;
    QLineEdit *LE_D_IN_OFF_12;
    QLineEdit *LE_D_IN_ON_9;
    QSpacerItem *horizontalSpacer;
    QLineEdit *LE_D_IN_OFF_9;
    QLabel *label_25;
    QLineEdit *LE_D_IN_OFF_0;
    QLabel *label_36;
    QLineEdit *LE_D_IN_ON_15;
    QLabel *label_28;
    QLineEdit *LE_D_IN_OFF_5;
    QLineEdit *LE_D_IN_ON_10;
    QLabel *label_24;
    QLabel *label_37;
    QLineEdit *LE_D_IN_ON_2;
    QLineEdit *LE_D_IN_ON_5;
    QLineEdit *LE_D_IN_OFF_2;
    QLabel *label_38;
    QLabel *label_21;
    QLineEdit *LE_D_IN_ON_12;
    QLineEdit *LE_D_IN_ON_3;
    QLineEdit *LE_D_IN_OFF_11;
    QLabel *label_26;
    QLineEdit *LE_D_IN_ON_1;
    QLineEdit *LE_D_IN_OFF_15;
    QLineEdit *LE_D_IN_OFF_8;
    QLineEdit *LE_D_IN_ON_11;
    QLineEdit *LE_D_IN_ON_6;
    QLineEdit *LE_D_IN_OFF_1;
    QLineEdit *LE_D_IN_OFF_3;
    QLineEdit *LE_D_IN_OFF_13;
    QLabel *label_33;
    QLabel *label_22;
    QLineEdit *LE_D_IN_ON_4;
    QLabel *label_32;
    QLineEdit *LE_D_IN_OFF_14;
    QLineEdit *LE_D_IN_ON_7;
    QLabel *label_34;
    QLabel *label_39;
    QLineEdit *LE_D_IN_ON_13;
    QLineEdit *LE_D_IN_ON_14;
    QLabel *label_35;
    QLineEdit *LE_D_IN_ON_8;
    QLineEdit *LE_D_IN_OFF_7;
    QLabel *label_23;
    QLineEdit *LE_D_IN_OFF_6;
    QLineEdit *LE_D_IN_OFF_4;
    QLineEdit *LE_D_IN_ON_0;
    QLabel *label_31;
    QLineEdit *LE_D_IN_OFF_10;
    QLabel *label_40;
    QLabel *label_41;
    QLabel *label_42;
    QLabel *label_43;
    QLabel *label_18;
    QWidget *gridLayoutWidget;
    QGridLayout *gridLayout;
    QLabel *label_19;
    QLabel *label_20;
    QLabel *label_27;
    QLabel *label_29;
    QLabel *label_30;
    QLabel *label_44;
    QLineEdit *LE_JNT_REF_1;
    QLineEdit *LE_JNT_REF_2;
    QLineEdit *LE_JNT_REF_3;
    QLineEdit *LE_JNT_REF_4;
    QLineEdit *LE_JNT_REF_5;
    QLineEdit *LE_JNT_REF_6;
    QWidget *gridLayoutWidget_8;
    QGridLayout *gridLayout_7;
    QLabel *label_71;
    QLineEdit *LE_TA_IN_0;
    QLabel *label_74;
    QLineEdit *LE_TA_IN_1;
    QWidget *gridLayoutWidget_9;
    QGridLayout *gridLayout_9;
    QLineEdit *LE_TD_IN_OFF_0;
    QLabel *label_96;
    QLabel *label_82;
    QLineEdit *LE_TD_IN_ON_0;
    QLabel *label_93;
    QLineEdit *LE_TD_IN_OFF_1;
    QLabel *label_95;
    QLineEdit *LE_TD_IN_ON_1;
    QLabel *label_45;
    QLabel *label_101;
    QLineEdit *LE_PG_MODE_SIMULATION;
    QWidget *gridLayoutWidget_10;
    QGridLayout *gridLayout_10;
    QLineEdit *LE_TD_OUT_OFF_0;
    QLabel *label_97;
    QLabel *label_83;
    QLineEdit *LE_TD_OUT_ON_0;
    QLabel *label_94;
    QLineEdit *LE_TD_OUT_OFF_1;
    QLabel *label_98;
    QLineEdit *LE_TD_OUT_ON_1;
    QLabel *label_84;
    QLabel *label_46;
    QWidget *gridLayoutWidget_5;
    QGridLayout *gridLayout_5;
    QLineEdit *LE_D_OUT_OFF_12;
    QLineEdit *LE_D_OUT_ON_9;
    QSpacerItem *horizontalSpacer_2;
    QLineEdit *LE_D_OUT_OFF_9;
    QLabel *label_47;
    QLineEdit *LE_D_OUT_OFF_0;
    QLabel *label_48;
    QLineEdit *LE_D_OUT_ON_15;
    QLabel *label_49;
    QLineEdit *LE_D_OUT_OFF_5;
    QLineEdit *LE_D_OUT_ON_10;
    QLabel *label_50;
    QLabel *label_51;
    QLineEdit *LE_D_OUT_ON_2;
    QLineEdit *LE_D_OUT_ON_5;
    QLineEdit *LE_D_OUT_OFF_2;
    QLabel *label_52;
    QLabel *label_53;
    QLineEdit *LE_D_OUT_ON_12;
    QLineEdit *LE_D_OUT_ON_3;
    QLineEdit *LE_D_OUT_OFF_11;
    QLabel *label_54;
    QLineEdit *LE_D_OUT_ON_1;
    QLineEdit *LE_D_OUT_OFF_15;
    QLineEdit *LE_D_OUT_OFF_8;
    QLineEdit *LE_D_OUT_ON_11;
    QLineEdit *LE_D_OUT_ON_6;
    QLineEdit *LE_D_OUT_OFF_1;
    QLineEdit *LE_D_OUT_OFF_3;
    QLineEdit *LE_D_OUT_OFF_13;
    QLabel *label_55;
    QLabel *label_56;
    QLineEdit *LE_D_OUT_ON_4;
    QLabel *label_57;
    QLineEdit *LE_D_OUT_OFF_14;
    QLineEdit *LE_D_OUT_ON_7;
    QLabel *label_58;
    QLabel *label_59;
    QLineEdit *LE_D_OUT_ON_13;
    QLineEdit *LE_D_OUT_ON_14;
    QLabel *label_60;
    QLineEdit *LE_D_OUT_ON_8;
    QLineEdit *LE_D_OUT_OFF_7;
    QLabel *label_61;
    QLineEdit *LE_D_OUT_OFF_6;
    QLineEdit *LE_D_OUT_OFF_4;
    QLineEdit *LE_D_OUT_ON_0;
    QLabel *label_62;
    QLineEdit *LE_D_OUT_OFF_10;
    QLabel *label_63;
    QLabel *label_64;
    QLabel *label_65;
    QLabel *label_66;
    QLabel *label_99;
    QLineEdit *LE_ROBOT_STATUS_SOS;
    QLineEdit *LE_ROBOT_STATE_IDLE;
    QLabel *label_67;
    QLabel *label_100;
    QLabel *label_68;
    QWidget *gridLayoutWidget_3;
    QGridLayout *gridLayout_3;
    QLabel *label_69;
    QLabel *label_70;
    QLabel *label_72;
    QLabel *label_73;
    QLabel *label_75;
    QLabel *label_76;
    QLineEdit *LE_TCP_REF_X;
    QLineEdit *LE_TCP_REF_Y;
    QLineEdit *LE_TCP_REF_Z;
    QLineEdit *LE_TCP_REF_RX;
    QLineEdit *LE_TCP_REF_RY;
    QLineEdit *LE_TCP_REF_RZ;
    QLineEdit *LE_ROBOT_STATUS_EMS;
    QLineEdit *LE_ROBOT_STATE_MOVING;
    QLineEdit *LE_ROBOT_STATUS_PAUSED;
    QLabel *label_77;
    QLabel *label_78;
    QLineEdit *LE_ROBOT_STATUS_SELF_COLLISION;
    QLineEdit *LE_PG_MODE_REAL;
    QWidget *gridLayoutWidget_6;
    QGridLayout *gridLayout_6;
    QLabel *label_79;
    QLabel *label_80;
    QSpacerItem *horizontalSpacer_3;
    QLabel *label_81;
    QLineEdit *LE_A_IN_1;
    QLabel *label_85;
    QLineEdit *LE_A_IN_0;
    QLineEdit *LE_A_IN_2;
    QLineEdit *LE_A_IN_3;
    QLineEdit *LE_ROBOT_STATUS_EXT_COLLISION;
    QLabel *label_86;
    QLabel *label_87;
    QLabel *label_88;
    QLineEdit *LE_ROBOT_STATUS_TEACHING;
    QWidget *gridLayoutWidget_7;
    QGridLayout *gridLayout_8;
    QLabel *label_89;
    QLabel *label_90;
    QSpacerItem *horizontalSpacer_5;
    QLabel *label_91;
    QLineEdit *LE_A_OUT_1;
    QLabel *label_92;
    QLineEdit *LE_A_OUT_0;
    QLineEdit *LE_A_OUT_2;
    QLineEdit *LE_A_OUT_3;
    QWidget *gridLayoutWidget_2;
    QGridLayout *gridLayout_2;
    QLabel *label_102;
    QLabel *label_103;
    QLabel *label_104;
    QLabel *label_105;
    QLabel *label_106;
    QLabel *label_107;
    QLineEdit *LE_JNT_ENC_1;
    QLineEdit *LE_JNT_ENC_2;
    QLineEdit *LE_JNT_ENC_3;
    QLineEdit *LE_JNT_ENC_4;
    QLineEdit *LE_JNT_ENC_5;
    QLineEdit *LE_JNT_ENC_6;
    QWidget *gridLayoutWidget_11;
    QGridLayout *gridLayout_11;
    QLabel *label_108;
    QLabel *label_109;
    QLabel *label_110;
    QLineEdit *LE_TV_0;
    QLineEdit *LE_TV_12;
    QLineEdit *LE_TV_24;
    QPushButton *BTN_COLLISION_RESUME;
    QPushButton *BTN_MOTION_PAUSE;
    QGroupBox *GB_NETWORK_2;
    QPushButton *BTN_MODE_REAL;
    QPushButton *BTN_MODE_SIMULATION;
    QCommandLinkButton *commandLinkButton;
    QLabel *LB_BASE_SPEED;
    QPushButton *BTN_MOVE_JOINT_GRAPS_GRIPPER;
    QLabel *label_137;
    QPushButton *BTN_MOVE_JOINT_INIT_3;
    QPushButton *BTN_MOVE_JOINT_INIT_4;
    QPushButton *BTN_MOVE_JOINT_INIT_5;
    QPushButton *btn_pump;
    QPushButton *BTN_MOVE_JOINT_Rotation;
    QWidget *layoutWidget5;
    QGridLayout *gridLayout_15;
    QLabel *label_9;
    QLineEdit *LE_JOINT_LIST;
    QLineEdit *LE_TCP_LIST;
    QLabel *label_17;
    QPushButton *BTN_TCP_MOTION_TEST;
    QPushButton *BTN_JOINT_MOTION_TEST;
    QPushButton *BTN_MOVE_JOINT_POP;
    QPushButton *BTN_MOVE_JOINT_BOX_CENTER;
    QPushButton *pb_reload;
    QPushButton *BTN_MOVE_JOINT_BOX_CENTER_KETI;
    QPushButton *BTN_MOVEL_rel;
    QPushButton *BTN_quick_return;
    QWidget *layoutWidget6;
    QVBoxLayout *verticalLayout_7;
    QPushButton *BTN_MOVE_JOINT_HIGH;
    QPushButton *BTN_MOVE_JOINT_MID;
    QPushButton *BTN_MOVE_JOINT_LOW;
    QWidget *layoutWidget7;
    QHBoxLayout *horizontalLayout_15;
    QPushButton *BTN_MOVE_JOINT_MID_LEFT;
    QPushButton *BTN_MOVE_JOINT_MID_RIGHT;
    QWidget *layoutWidget_8;
    QHBoxLayout *horizontalLayout_16;
    QPushButton *BTN_MOVE_JOINT_LOW_LEFT;
    QPushButton *BTN_MOVE_JOINT_LOW_RIGHT;
    QPushButton *BTN_RETURN_MOVE_JOINT_BOX_CENTER;
    QWidget *lift_motor;
    QComboBox *cb_port;
    QGroupBox *groupBox_3;
    QWidget *layoutWidget_7;
    QVBoxLayout *verticalLayout_8;
    QHBoxLayout *horizontalLayout_13;
    QLabel *label_7;
    QSpacerItem *horizontalSpacer_10;
    QLineEdit *le_temp;
    QHBoxLayout *horizontalLayout_5;
    QLabel *label;
    QSpacerItem *horizontalSpacer_4;
    QLineEdit *le_rpm;
    QHBoxLayout *horizontalLayout_11;
    QLabel *label_4;
    QSpacerItem *horizontalSpacer_8;
    QLineEdit *le_controlType;
    QHBoxLayout *horizontalLayout_6;
    QLabel *label_6;
    QSpacerItem *horizontalSpacer_6;
    QLineEdit *le_rpmStd;
    QHBoxLayout *horizontalLayout_12;
    QLabel *label_5;
    QSpacerItem *horizontalSpacer_9;
    QLineEdit *le_armp;
    QHBoxLayout *horizontalLayout_10;
    QLabel *label_3;
    QSpacerItem *horizontalSpacer_7;
    QLineEdit *le_pos;
    QTextEdit *te_data;
    QPushButton *bt_test;
    QGroupBox *groupBox;
    QCheckBox *ch_dir;
    QPushButton *bt_homing;
    QSpinBox *sb_move_pos;
    QPushButton *bt_move_pos;
    QSpinBox *sb_max_vel_pos;
    QPushButton *bt_set_maxVel_pos;
    QPushButton *bt_auto_homing;
    QGroupBox *groupBox_2;
    QSpinBox *sb_rpm;
    QPushButton *bt_move_rpm;
    QPushButton *bt_rpmZero;
    QPushButton *bt_connect;
    QLabel *label_174;
    QWidget *tab1;
    QGroupBox *GR_KITECH;
    QLineEdit *LE_GRIPPER_CMD;
    QPushButton *BTN_GRIPPER_CMD_WRITE;
    QPushButton *BTN_GRIPPER_ALLFOLD;
    QPushButton *BTN_RB5_CALC_GRAPS_POINT;
    QPushButton *BTN_RB5_MOVE_GRAPS_POINT;
    QPushButton *BTN_GRIPPER_VISION;
    QLabel *label_127;
    QLabel *label_128;
    QLabel *label_129;
    QLabel *label_130;
    QWidget *layoutWidget_3;
    QHBoxLayout *horizontalLayout_3;
    QComboBox *CB_GRIPPER_GRAPS;
    QComboBox *CB_GRIPPER_POWER;
    QPushButton *BTN_GRIPPER_GRASP;
    QWidget *layoutWidget_4;
    QHBoxLayout *horizontalLayout_4;
    QComboBox *CB_GRIPPER_GE_MODE;
    QComboBox *CB_GRIPPER_GE_GRAPS;
    QComboBox *CB_GRIPPER_GE_POWER;
    QComboBox *CB_GRIPPER_GE_SUCTION_POWER;
    QPushButton *CB_GRIPPER_GE_CMD_WRITE;
    QPushButton *Pinch12_FOLD;
    QLabel *label_152;
    QLabel *label_153;
    QTextBrowser *textBrowser;
    QTextBrowser *textBrowser_2;
    QWidget *layoutWidget8;
    QGridLayout *gridLayout_17;
    QComboBox *CB_GRIPPER_recog;
    QPushButton *BTN_GRIPPER_recog_WRITE;
    QPushButton *BTN_GRIPPER_SUCTION;
    QPushButton *BTN_GRIPPER_OPEN;
    QTextEdit *notice_kitech;
    QLabel *label_131;
    QLineEdit *le_keti;
    QLabel *lb_keti_point;
    QLineEdit *le_kitech;
    QLabel *label_133;
    QLabel *label_134;
    QLabel *label_135;
    QPushButton *BTN_CONNECT_GRIPPER;
    QLabel *LE_KETI_IMG;
    QPushButton *pb_shutter;
    QComboBox *cb_get_object_id_vision;
    QWidget *barcode;
    QLabel *label_111;
    QLabel *label_112;
    QComboBox *cb_usb2;
    QPushButton *write;
    QTextEdit *te_usb1;
    QWidget *layoutWidget_2;
    QVBoxLayout *verticalLayout_3;
    QPushButton *bt_usb2_dis;
    QPushButton *bt_usb2_con;
    QWidget *layoutWidget9;
    QVBoxLayout *verticalLayout_4;
    QPushButton *bt_usb1_dis;
    QPushButton *bt_usb1_con;
    QLabel *label_113;
    QComboBox *cb_usb1;
    QTextEdit *te_usb2;
    QPushButton *BTN_MOVE_TOTAL;
    QPushButton *BTN_MOVE_TOTAL_2;
    QPushButton *BTN_MOVE_TOTAL_3;
    QPushButton *BTN_MOVE_TOTAL_4;
    QPushButton *BTN_show;
    QWidget *tab_2;
    QLabel *label_143;
    QLabel *te_label;
    QLabel *label_144;
    QTextEdit *te_msg;
    QLabel *label_145;
    QLabel *label_146;
    QLabel *label_147;
    QTextEdit *te_send_msg;
    QWidget *tab_3;
    QPushButton *bt_save_all_annot;
    QComboBox *cb_shelf_id;
    QComboBox *cb_get_object_id;
    QLabel *label_165;
    QPushButton *bt_save_annot;
    QTextEdit *te_scen;
    QWidget *layoutWidget_5;
    QHBoxLayout *horizontalLayout_7;
    QComboBox *CB_GRIPPER_GRAPS_3;
    QComboBox *CB_GRIPPER_POWER_3;
    QLabel *label_167;
    QLabel *label_168;
    QWidget *layoutWidget_6;
    QHBoxLayout *horizontalLayout_8;
    QComboBox *CB_GRIPPER_GE_MODE_3;
    QComboBox *CB_GRIPPER_GE_GRAPS_3;
    QComboBox *CB_GRIPPER_GE_POWER_3;
    QComboBox *CB_GRIPPER_GE_SUCTION_POWER_3;
    QLabel *label_169;
    QLabel *label_172;
    QPushButton *bt_delete_annot;
    QPushButton *bt_order;
    QLabel *label_173;
    QTextEdit *te_order;
    QWidget *layoutWidget10;
    QVBoxLayout *verticalLayout_2;
    QLabel *label_164;
    QLabel *label_170;
    QLabel *label_171;
    QComboBox *cb_obj_direction;
    QLabel *label_163;
    QTextEdit *te_scen_2;
    QPushButton *bt_save_annot_2;
    QPushButton *bt_save_all_annot_2;
    QPushButton *bt_delete_annot_2;
    QWidget *layoutWidget11;
    QVBoxLayout *verticalLayout_6;
    QVBoxLayout *verticalLayout_5;
    QHBoxLayout *horizontalLayout_9;
    QComboBox *CB_shelf;
    QComboBox *cb_obj_direction_2;
    QComboBox *cb_get_object_id_2;
    QGridLayout *gridLayout_16;
    QLabel *label_178;
    QLabel *label_176;
    QHBoxLayout *horizontalLayout_2;
    QSpacerItem *horizontalSpacer_13;
    QLineEdit *le_shelf_hight;
    QSpacerItem *horizontalSpacer_11;
    QComboBox *CB_gripper_num;
    QSpacerItem *horizontalSpacer_12;
    QPushButton *bt_scen_resume;
    QLabel *te_status;
    QPushButton *ALL_STOP;
    QLabel *label_175;
    QLabel *label_180;
    QLabel *label_179;
    QStatusBar *statusbar;
    QMenuBar *menubar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(1767, 1048);
        MainWindow->setTabShape(QTabWidget::Triangular);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        scenario = new QTabWidget(centralwidget);
        scenario->setObjectName(QString::fromUtf8("scenario"));
        scenario->setGeometry(QRect(20, 10, 1731, 981));
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(scenario->sizePolicy().hasHeightForWidth());
        scenario->setSizePolicy(sizePolicy);
        scenario->setMaximumSize(QSize(3500, 3500));
        tab = new QWidget();
        tab->setObjectName(QString::fromUtf8("tab"));
        groupBox_5 = new QGroupBox(tab);
        groupBox_5->setObjectName(QString::fromUtf8("groupBox_5"));
        groupBox_5->setGeometry(QRect(30, 49, 381, 101));
        le_rb5_data_connection = new QLineEdit(groupBox_5);
        le_rb5_data_connection->setObjectName(QString::fromUtf8("le_rb5_data_connection"));
        le_rb5_data_connection->setGeometry(QRect(114, 50, 20, 40));
        le_rb5_data_connection->setCursor(QCursor(Qt::BlankCursor));
        le_rb5_data_connection->setStyleSheet(QString::fromUtf8("    border-style: outset;\n"
"    border-width: 1px;\n"
"    border-color: black;\n"
"    background-color:rgb(255,0,0)"));
        le_rb5_data_connection->setReadOnly(true);
        label_114 = new QLabel(groupBox_5);
        label_114->setObjectName(QString::fromUtf8("label_114"));
        label_114->setGeometry(QRect(5, 30, 51, 16));
        QFont font;
        font.setBold(false);
        font.setItalic(false);
        font.setWeight(50);
        label_114->setFont(font);
        label_114->setStyleSheet(QString::fromUtf8("    font:  10px;\n"
""));
        label_114->setAlignment(Qt::AlignCenter);
        le_vision_connection = new QLineEdit(groupBox_5);
        le_vision_connection->setObjectName(QString::fromUtf8("le_vision_connection"));
        le_vision_connection->setGeometry(QRect(165, 50, 40, 40));
        le_vision_connection->setCursor(QCursor(Qt::BlankCursor));
        le_vision_connection->setStyleSheet(QString::fromUtf8("    border-style: outset;\n"
"    border-width: 1px;\n"
"    border-color: black;\n"
"    background-color:rgb(255,0,0)"));
        le_vision_connection->setReadOnly(true);
        le_lift_connection = new QLineEdit(groupBox_5);
        le_lift_connection->setObjectName(QString::fromUtf8("le_lift_connection"));
        le_lift_connection->setGeometry(QRect(63, 50, 40, 40));
        le_lift_connection->setStyleSheet(QString::fromUtf8("    border-style: outset;\n"
"    border-width: 1px;\n"
"    border-color: black;\n"
"    background-color:rgb(255,0,0)"));
        le_lift_connection->setReadOnly(true);
        label_115 = new QLabel(groupBox_5);
        label_115->setObjectName(QString::fromUtf8("label_115"));
        label_115->setGeometry(QRect(213, 30, 51, 16));
        label_115->setFont(font);
        label_115->setStyleSheet(QString::fromUtf8("    font:  10px;\n"
""));
        label_115->setAlignment(Qt::AlignCenter);
        label_116 = new QLabel(groupBox_5);
        label_116->setObjectName(QString::fromUtf8("label_116"));
        label_116->setGeometry(QRect(109, 30, 51, 16));
        label_116->setFont(font);
        label_116->setStyleSheet(QString::fromUtf8("    font:  10px;\n"
""));
        label_116->setAlignment(Qt::AlignCenter);
        le_gripper_connection = new QLineEdit(groupBox_5);
        le_gripper_connection->setObjectName(QString::fromUtf8("le_gripper_connection"));
        le_gripper_connection->setGeometry(QRect(219, 50, 40, 40));
        le_gripper_connection->setStyleSheet(QString::fromUtf8("    border-style: outset;\n"
"    border-width: 1px;\n"
"    border-color: black;\n"
"    background-color:rgb(255,255,255)"));
        le_gripper_connection->setReadOnly(true);
        label_117 = new QLabel(groupBox_5);
        label_117->setObjectName(QString::fromUtf8("label_117"));
        label_117->setGeometry(QRect(58, 30, 51, 16));
        label_117->setFont(font);
        label_117->setStyleSheet(QString::fromUtf8("    font:  10px;\n"
""));
        label_117->setAlignment(Qt::AlignCenter);
        label_118 = new QLabel(groupBox_5);
        label_118->setObjectName(QString::fromUtf8("label_118"));
        label_118->setGeometry(QRect(160, 31, 51, 16));
        label_118->setFont(font);
        label_118->setStyleSheet(QString::fromUtf8("    font:  10px;\n"
""));
        label_118->setAlignment(Qt::AlignCenter);
        le_mobile_connection = new QLineEdit(groupBox_5);
        le_mobile_connection->setObjectName(QString::fromUtf8("le_mobile_connection"));
        le_mobile_connection->setGeometry(QRect(11, 50, 40, 40));
        le_mobile_connection->setStyleSheet(QString::fromUtf8("    border-style: outset;\n"
"    border-width: 1px;\n"
"    border-color: black;\n"
"    background-color:rgb(255,0,0)"));
        le_mobile_connection->setReadOnly(true);
        le_rb5_cmd_connection = new QLineEdit(groupBox_5);
        le_rb5_cmd_connection->setObjectName(QString::fromUtf8("le_rb5_cmd_connection"));
        le_rb5_cmd_connection->setGeometry(QRect(133, 50, 20, 40));
        le_rb5_cmd_connection->setCursor(QCursor(Qt::BlankCursor));
        le_rb5_cmd_connection->setStyleSheet(QString::fromUtf8("    border-style: outset;\n"
"    border-width: 1px;\n"
"    border-color: black;\n"
"    background-color:rgb(255,0,0)"));
        le_rb5_cmd_connection->setReadOnly(true);
        label_119 = new QLabel(groupBox_5);
        label_119->setObjectName(QString::fromUtf8("label_119"));
        label_119->setGeometry(QRect(260, 30, 71, 16));
        label_119->setFont(font);
        label_119->setStyleSheet(QString::fromUtf8("    font:  10px;\n"
""));
        label_119->setAlignment(Qt::AlignCenter);
        le_websocket_connection = new QLineEdit(groupBox_5);
        le_websocket_connection->setObjectName(QString::fromUtf8("le_websocket_connection"));
        le_websocket_connection->setGeometry(QRect(275, 50, 40, 40));
        le_websocket_connection->setStyleSheet(QString::fromUtf8("    border-style: outset;\n"
"    border-width: 1px;\n"
"    border-color: black;\n"
"    background-color:rgb(255,255,255)"));
        le_websocket_connection->setReadOnly(true);
        label_148 = new QLabel(groupBox_5);
        label_148->setObjectName(QString::fromUtf8("label_148"));
        label_148->setGeometry(QRect(316, 30, 71, 16));
        label_148->setFont(font);
        label_148->setStyleSheet(QString::fromUtf8("    font:  10px;\n"
""));
        label_148->setAlignment(Qt::AlignCenter);
        lb_sn_map = new MyGraphicsView(tab);
        lb_sn_map->setObjectName(QString::fromUtf8("lb_sn_map"));
        lb_sn_map->setGeometry(QRect(30, 160, 631, 631));
        lb_sn_map->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        lb_sn_map->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        lb_sn_map->setSizeAdjustPolicy(QAbstractScrollArea::AdjustIgnored);
        GE_MOVE_INFO = new QGroupBox(tab);
        GE_MOVE_INFO->setObjectName(QString::fromUtf8("GE_MOVE_INFO"));
        GE_MOVE_INFO->setGeometry(QRect(418, 49, 191, 101));
        QFont font1;
        font1.setFamily(QString::fromUtf8("Arial"));
        GE_MOVE_INFO->setFont(font1);
        le_mobile_move_status = new QLineEdit(GE_MOVE_INFO);
        le_mobile_move_status->setObjectName(QString::fromUtf8("le_mobile_move_status"));
        le_mobile_move_status->setGeometry(QRect(6, 50, 40, 40));
        le_mobile_move_status->setStyleSheet(QString::fromUtf8("    border-style: outset;\n"
"    border-width: 1px;\n"
"    border-color: black;\n"
"    background-color:rgb(255,255,255)"));
        le_mobile_move_status->setReadOnly(true);
        le_lift_move_status = new QLineEdit(GE_MOVE_INFO);
        le_lift_move_status->setObjectName(QString::fromUtf8("le_lift_move_status"));
        le_lift_move_status->setGeometry(QRect(52, 50, 40, 40));
        le_lift_move_status->setAutoFillBackground(false);
        le_lift_move_status->setStyleSheet(QString::fromUtf8("    border-style: outset;\n"
"    border-width: 1px;\n"
"    border-color: black;\n"
"    background-color:rgb(255,255,255)"));
        le_lift_move_status->setReadOnly(true);
        le_rb5_move_status = new QLineEdit(GE_MOVE_INFO);
        le_rb5_move_status->setObjectName(QString::fromUtf8("le_rb5_move_status"));
        le_rb5_move_status->setGeometry(QRect(98, 50, 40, 40));
        le_rb5_move_status->setStyleSheet(QString::fromUtf8("    border-style: outset;\n"
"    border-width: 1px;\n"
"    border-color: black;\n"
"    background-color:rgb(255,0,0)"));
        le_rb5_move_status->setReadOnly(true);
        label_120 = new QLabel(GE_MOVE_INFO);
        label_120->setObjectName(QString::fromUtf8("label_120"));
        label_120->setGeometry(QRect(0, 30, 51, 16));
        label_120->setFont(font);
        label_120->setStyleSheet(QString::fromUtf8("    font:  10px;\n"
""));
        label_120->setAlignment(Qt::AlignCenter);
        label_136 = new QLabel(GE_MOVE_INFO);
        label_136->setObjectName(QString::fromUtf8("label_136"));
        label_136->setGeometry(QRect(46, 30, 51, 16));
        label_136->setFont(font);
        label_136->setStyleSheet(QString::fromUtf8("    font:  10px;\n"
""));
        label_136->setAlignment(Qt::AlignCenter);
        label_138 = new QLabel(GE_MOVE_INFO);
        label_138->setObjectName(QString::fromUtf8("label_138"));
        label_138->setGeometry(QRect(92, 30, 51, 16));
        label_138->setFont(font);
        label_138->setStyleSheet(QString::fromUtf8("    font:  10px;\n"
""));
        label_138->setAlignment(Qt::AlignCenter);
        label_177 = new QLabel(GE_MOVE_INFO);
        label_177->setObjectName(QString::fromUtf8("label_177"));
        label_177->setGeometry(QRect(139, 30, 51, 16));
        label_177->setFont(font);
        label_177->setStyleSheet(QString::fromUtf8("    font:  10px;\n"
""));
        label_177->setAlignment(Qt::AlignCenter);
        le_pick = new QLineEdit(GE_MOVE_INFO);
        le_pick->setObjectName(QString::fromUtf8("le_pick"));
        le_pick->setGeometry(QRect(145, 50, 40, 40));
        le_pick->setStyleSheet(QString::fromUtf8("    border-style: outset;\n"
"    border-width: 1px;\n"
"    border-color: black;\n"
"    background-color:rgb(255,0,0)"));
        le_pick->setReadOnly(true);
        groupBox_7 = new QGroupBox(tab);
        groupBox_7->setObjectName(QString::fromUtf8("groupBox_7"));
        groupBox_7->setGeometry(QRect(700, 230, 591, 141));
        bt_lift_homimg = new QPushButton(groupBox_7);
        bt_lift_homimg->setObjectName(QString::fromUtf8("bt_lift_homimg"));
        bt_lift_homimg->setGeometry(QRect(20, 70, 101, 51));
        QFont font2;
        font2.setFamily(QString::fromUtf8("Arial"));
        font2.setPointSize(11);
        bt_lift_homimg->setFont(font2);
        bt_lift_stop = new QPushButton(groupBox_7);
        bt_lift_stop->setObjectName(QString::fromUtf8("bt_lift_stop"));
        bt_lift_stop->setGeometry(QRect(467, 40, 81, 81));
        QFont font3;
        font3.setFamily(QString::fromUtf8("Arial"));
        font3.setPointSize(9);
        font3.setBold(true);
        font3.setWeight(75);
        bt_lift_stop->setFont(font3);
        bt_lift_stop->setAutoFillBackground(false);
        bt_lift_stop->setStyleSheet(QString::fromUtf8("background-color: rgb(203, 35, 35);"));
        bt_lift_pos_move = new QPushButton(groupBox_7);
        bt_lift_pos_move->setObjectName(QString::fromUtf8("bt_lift_pos_move"));
        bt_lift_pos_move->setGeometry(QRect(264, 40, 91, 31));
        sb_lift_pos = new QSpinBox(groupBox_7);
        sb_lift_pos->setObjectName(QString::fromUtf8("sb_lift_pos"));
        sb_lift_pos->setGeometry(QRect(134, 40, 121, 31));
        sb_lift_pos->setLayoutDirection(Qt::LeftToRight);
        sb_lift_pos->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        sb_lift_pos->setMinimum(0);
        sb_lift_pos->setMaximum(6000);
        cb_lift_dir_homing = new QCheckBox(groupBox_7);
        cb_lift_dir_homing->setObjectName(QString::fromUtf8("cb_lift_dir_homing"));
        cb_lift_dir_homing->setGeometry(QRect(20, 40, 71, 23));
        sb_lift_pos_rpm = new QSpinBox(groupBox_7);
        sb_lift_pos_rpm->setObjectName(QString::fromUtf8("sb_lift_pos_rpm"));
        sb_lift_pos_rpm->setGeometry(QRect(134, 90, 121, 31));
        sb_lift_pos_rpm->setLayoutDirection(Qt::LeftToRight);
        sb_lift_pos_rpm->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        sb_lift_pos_rpm->setMaximum(3000);
        bt_lift_rpm_set = new QPushButton(groupBox_7);
        bt_lift_rpm_set->setObjectName(QString::fromUtf8("bt_lift_rpm_set"));
        bt_lift_rpm_set->setGeometry(QRect(264, 90, 91, 31));
        layoutWidget = new QWidget(groupBox_7);
        layoutWidget->setObjectName(QString::fromUtf8("layoutWidget"));
        layoutWidget->setGeometry(QRect(370, 34, 87, 95));
        verticalLayout = new QVBoxLayout(layoutWidget);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        bt_lift_top = new QPushButton(layoutWidget);
        bt_lift_top->setObjectName(QString::fromUtf8("bt_lift_top"));

        verticalLayout->addWidget(bt_lift_top);

        bt_lift_MID = new QPushButton(layoutWidget);
        bt_lift_MID->setObjectName(QString::fromUtf8("bt_lift_MID"));

        verticalLayout->addWidget(bt_lift_MID);

        bt_lift_Low = new QPushButton(layoutWidget);
        bt_lift_Low->setObjectName(QString::fromUtf8("bt_lift_Low"));

        verticalLayout->addWidget(bt_lift_Low);

        groupBox_8 = new QGroupBox(tab);
        groupBox_8->setObjectName(QString::fromUtf8("groupBox_8"));
        groupBox_8->setGeometry(QRect(700, 380, 591, 141));
        sb_rb5_speed = new QSpinBox(groupBox_8);
        sb_rb5_speed->setObjectName(QString::fromUtf8("sb_rb5_speed"));
        sb_rb5_speed->setGeometry(QRect(60, 81, 141, 41));
        QFont font4;
        font4.setFamily(QString::fromUtf8("Arial"));
        font4.setPointSize(14);
        font4.setBold(true);
        font4.setWeight(75);
        sb_rb5_speed->setFont(font4);
        sb_rb5_speed->setMaximum(100);
        sld_rb5_speed = new QSlider(groupBox_8);
        sld_rb5_speed->setObjectName(QString::fromUtf8("sld_rb5_speed"));
        sld_rb5_speed->setGeometry(QRect(20, 60, 141, 16));
        sld_rb5_speed->setMaximum(100);
        sld_rb5_speed->setOrientation(Qt::Horizontal);
        bt_rb5_mode = new QPushButton(groupBox_8);
        bt_rb5_mode->setObjectName(QString::fromUtf8("bt_rb5_mode"));
        bt_rb5_mode->setGeometry(QRect(220, 60, 61, 61));
        bt_rb5_mode->setFont(font2);
        bt_rb5_pause = new QPushButton(groupBox_8);
        bt_rb5_pause->setObjectName(QString::fromUtf8("bt_rb5_pause"));
        bt_rb5_pause->setGeometry(QRect(372, 50, 61, 41));
        bt_rb5_pause->setFont(font3);
        bt_rb5_pause->setStyleSheet(QString::fromUtf8("background-color: rgb(252, 233, 79);"));
        bt_rb5_resume = new QPushButton(groupBox_8);
        bt_rb5_resume->setObjectName(QString::fromUtf8("bt_rb5_resume"));
        bt_rb5_resume->setGeometry(QRect(442, 50, 61, 41));
        bt_rb5_resume->setFont(font3);
        bt_rb5_resume->setStyleSheet(QString::fromUtf8("background-color: rgb(138, 226, 52);"));
        bt_rb5_halt = new QPushButton(groupBox_8);
        bt_rb5_halt->setObjectName(QString::fromUtf8("bt_rb5_halt"));
        bt_rb5_halt->setGeometry(QRect(512, 50, 61, 41));
        bt_rb5_halt->setFont(font3);
        bt_rb5_halt->setStyleSheet(QString::fromUtf8("background-color:red"));
        bt_rb5_pos_move = new QPushButton(groupBox_8);
        bt_rb5_pos_move->setObjectName(QString::fromUtf8("bt_rb5_pos_move"));
        bt_rb5_pos_move->setGeometry(QRect(512, 100, 61, 31));
        cb_rb5_pos = new QComboBox(groupBox_8);
        cb_rb5_pos->setObjectName(QString::fromUtf8("cb_rb5_pos"));
        cb_rb5_pos->setGeometry(QRect(372, 100, 131, 31));
        lb_rb5_speed_percentage = new QLabel(groupBox_8);
        lb_rb5_speed_percentage->setObjectName(QString::fromUtf8("lb_rb5_speed_percentage"));
        lb_rb5_speed_percentage->setGeometry(QRect(150, 57, 51, 17));
        lb_rb5_speed_percentage->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        label_150 = new QLabel(groupBox_8);
        label_150->setObjectName(QString::fromUtf8("label_150"));
        label_150->setGeometry(QRect(204, 40, 91, 17));
        QFont font5;
        font5.setPointSize(9);
        label_150->setFont(font5);
        label_150->setLayoutDirection(Qt::LeftToRight);
        label_150->setAlignment(Qt::AlignCenter);
        label_151 = new QLabel(groupBox_8);
        label_151->setObjectName(QString::fromUtf8("label_151"));
        label_151->setGeometry(QRect(20, 40, 91, 17));
        label_151->setFont(font5);
        label_151->setLayoutDirection(Qt::LeftToRight);
        label_151->setAlignment(Qt::AlignCenter);
        robot_status = new QLineEdit(groupBox_8);
        robot_status->setObjectName(QString::fromUtf8("robot_status"));
        robot_status->setGeometry(QRect(20, 86, 31, 31));
        btn_pump_2 = new QPushButton(groupBox_8);
        btn_pump_2->setObjectName(QString::fromUtf8("btn_pump_2"));
        btn_pump_2->setGeometry(QRect(290, 60, 61, 61));
        btn_pump_2->setStyleSheet(QString::fromUtf8("background-color: rgb(245, 121, 0);"));
        groupBox_9 = new QGroupBox(tab);
        groupBox_9->setObjectName(QString::fromUtf8("groupBox_9"));
        groupBox_9->setGeometry(QRect(700, 60, 241, 161));
        cb_sn_cmd = new QComboBox(groupBox_9);
        cb_sn_cmd->setObjectName(QString::fromUtf8("cb_sn_cmd"));
        cb_sn_cmd->setGeometry(QRect(10, 80, 161, 25));
        bt_sn_cmd_set = new QPushButton(groupBox_9);
        bt_sn_cmd_set->setObjectName(QString::fromUtf8("bt_sn_cmd_set"));
        bt_sn_cmd_set->setGeometry(QRect(178, 80, 51, 25));
        label_162 = new QLabel(groupBox_9);
        label_162->setObjectName(QString::fromUtf8("label_162"));
        label_162->setGeometry(QRect(10, 50, 151, 17));
        label_162->setFont(font5);
        label_162->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        bt_amr_motor_init = new QPushButton(groupBox_9);
        bt_amr_motor_init->setObjectName(QString::fromUtf8("bt_amr_motor_init"));
        bt_amr_motor_init->setGeometry(QRect(60, 120, 121, 25));
        groupBox_10 = new QGroupBox(tab);
        groupBox_10->setObjectName(QString::fromUtf8("groupBox_10"));
        groupBox_10->setGeometry(QRect(700, 540, 591, 391));
        lb_vision_capture_img = new QLabel(groupBox_10);
        lb_vision_capture_img->setObjectName(QString::fromUtf8("lb_vision_capture_img"));
        lb_vision_capture_img->setGeometry(QRect(10, 40, 500, 300));
        lb_vision_capture_img->setStyleSheet(QString::fromUtf8("    border-style: outset;\n"
"    border-width: 1px;\n"
"    border-color: black;"));
        lb_vision_capture_img->setLineWidth(1);
        bt_vision_cmd_capture = new QPushButton(groupBox_10);
        bt_vision_cmd_capture->setObjectName(QString::fromUtf8("bt_vision_cmd_capture"));
        bt_vision_cmd_capture->setGeometry(QRect(520, 190, 61, 41));
        bt_vision_cmd_capture->setContextMenuPolicy(Qt::DefaultContextMenu);
        bt_vision_cmd_capture->setAutoFillBackground(false);
        le_gripper_cmd = new QLineEdit(groupBox_10);
        le_gripper_cmd->setObjectName(QString::fromUtf8("le_gripper_cmd"));
        le_gripper_cmd->setGeometry(QRect(10, 350, 331, 31));
        bt_gripper_cmd_move = new QPushButton(groupBox_10);
        bt_gripper_cmd_move->setObjectName(QString::fromUtf8("bt_gripper_cmd_move"));
        bt_gripper_cmd_move->setGeometry(QRect(400, 350, 111, 31));
        groupBox_12 = new QGroupBox(tab);
        groupBox_12->setObjectName(QString::fromUtf8("groupBox_12"));
        groupBox_12->setGeometry(QRect(950, 60, 341, 161));
        groupBox_12->setTitle(QString::fromUtf8("Chage Map Color && zip file"));
        bt_png_change = new QPushButton(groupBox_12);
        bt_png_change->setObjectName(QString::fromUtf8("bt_png_change"));
        bt_png_change->setGeometry(QRect(20, 44, 89, 25));
        bt_zip = new QPushButton(groupBox_12);
        bt_zip->setObjectName(QString::fromUtf8("bt_zip"));
        bt_zip->setGeometry(QRect(230, 40, 101, 25));
        label_142 = new QLabel(groupBox_12);
        label_142->setObjectName(QString::fromUtf8("label_142"));
        label_142->setGeometry(QRect(20, 26, 91, 17));
        label_142->setFont(font5);
        label_142->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        bt_get_map_from_mobile = new QPushButton(groupBox_12);
        bt_get_map_from_mobile->setObjectName(QString::fromUtf8("bt_get_map_from_mobile"));
        bt_get_map_from_mobile->setGeometry(QRect(230, 76, 101, 25));
        cb_get_map_data = new QComboBox(groupBox_12);
        cb_get_map_data->addItem(QString());
        cb_get_map_data->addItem(QString());
        cb_get_map_data->setObjectName(QString::fromUtf8("cb_get_map_data"));
        cb_get_map_data->setGeometry(QRect(20, 76, 191, 25));
        label_161 = new QLabel(groupBox_12);
        label_161->setObjectName(QString::fromUtf8("label_161"));
        label_161->setGeometry(QRect(24, 109, 161, 17));
        label_161->setFont(font5);
        label_161->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        le_text = new QLineEdit(groupBox_12);
        le_text->setObjectName(QString::fromUtf8("le_text"));
        le_text->setGeometry(QRect(20, 129, 191, 25));
        pb_file_choose = new QPushButton(groupBox_12);
        pb_file_choose->setObjectName(QString::fromUtf8("pb_file_choose"));
        pb_file_choose->setGeometry(QRect(228, 124, 101, 31));
        groupBox_11 = new QGroupBox(tab);
        groupBox_11->setObjectName(QString::fromUtf8("groupBox_11"));
        groupBox_11->setGeometry(QRect(1300, 60, 361, 461));
        te_websocket = new QTextEdit(groupBox_11);
        te_websocket->setObjectName(QString::fromUtf8("te_websocket"));
        te_websocket->setGeometry(QRect(10, 30, 341, 421));
        le_barcode_connection = new QLineEdit(tab);
        le_barcode_connection->setObjectName(QString::fromUtf8("le_barcode_connection"));
        le_barcode_connection->setGeometry(QRect(360, 99, 40, 40));
        le_barcode_connection->setStyleSheet(QString::fromUtf8("    border-style: outset;\n"
"    border-width: 1px;\n"
"    border-color: black;\n"
"    background-color:rgb(255,0,0)"));
        le_barcode_connection->setReadOnly(true);
        label_149 = new QLabel(tab);
        label_149->setObjectName(QString::fromUtf8("label_149"));
        label_149->setGeometry(QRect(20, 0, 251, 41));
        QFont font6;
        font6.setFamily(QString::fromUtf8("Ubuntu"));
        font6.setPointSize(30);
        label_149->setFont(font6);
        label_149->setTextFormat(Qt::RichText);
        label_149->setTextInteractionFlags(Qt::LinksAccessibleByMouse);
        bt_tcp_on = new QPushButton(tab);
        bt_tcp_on->setObjectName(QString::fromUtf8("bt_tcp_on"));
        bt_tcp_on->setGeometry(QRect(700, 20, 141, 31));
        LE_INFO_MOVE_INFO_2 = new QLineEdit(tab);
        LE_INFO_MOVE_INFO_2->setObjectName(QString::fromUtf8("LE_INFO_MOVE_INFO_2"));
        LE_INFO_MOVE_INFO_2->setEnabled(false);
        LE_INFO_MOVE_INFO_2->setGeometry(QRect(550, 37, 27, 25));
        sizePolicy.setHeightForWidth(LE_INFO_MOVE_INFO_2->sizePolicy().hasHeightForWidth());
        LE_INFO_MOVE_INFO_2->setSizePolicy(sizePolicy);
        LE_INFO_MOVE_INFO_2->setStyleSheet(QString::fromUtf8("background-color: rgb(78, 154, 6);"));
        label_141 = new QLabel(tab);
        label_141->setObjectName(QString::fromUtf8("label_141"));
        label_141->setGeometry(QRect(580, 40, 26, 16));
        sizePolicy.setHeightForWidth(label_141->sizePolicy().hasHeightForWidth());
        label_141->setSizePolicy(sizePolicy);
        QFont font7;
        font7.setFamily(QString::fromUtf8("Arial"));
        font7.setPointSize(8);
        label_141->setFont(font7);
        label_166 = new QLabel(tab);
        label_166->setObjectName(QString::fromUtf8("label_166"));
        label_166->setGeometry(QRect(1314, 540, 67, 17));
        layoutWidget1 = new QWidget(tab);
        layoutWidget1->setObjectName(QString::fromUtf8("layoutWidget1"));
        layoutWidget1->setGeometry(QRect(901, 1, 636, 52));
        gridLayout_12 = new QGridLayout(layoutWidget1);
        gridLayout_12->setObjectName(QString::fromUtf8("gridLayout_12"));
        gridLayout_12->setContentsMargins(0, 0, 0, 0);
        label_154 = new QLabel(layoutWidget1);
        label_154->setObjectName(QString::fromUtf8("label_154"));
        label_154->setMaximumSize(QSize(83, 16777215));

        gridLayout_12->addWidget(label_154, 0, 2, 1, 1);

        label_132 = new QLabel(layoutWidget1);
        label_132->setObjectName(QString::fromUtf8("label_132"));
        label_132->setMinimumSize(QSize(30, 0));
        label_132->setMaximumSize(QSize(30, 16777215));

        gridLayout_12->addWidget(label_132, 1, 0, 1, 1);

        spb_Ty = new QDoubleSpinBox(layoutWidget1);
        spb_Ty->setObjectName(QString::fromUtf8("spb_Ty"));
        spb_Ty->setDecimals(4);
        spb_Ty->setMinimum(-1.000000000000000);
        spb_Ty->setMaximum(1.000000000000000);
        spb_Ty->setSingleStep(0.001000000000000);

        gridLayout_12->addWidget(spb_Ty, 1, 2, 1, 1);

        spb_Ry = new QDoubleSpinBox(layoutWidget1);
        spb_Ry->setObjectName(QString::fromUtf8("spb_Ry"));
        spb_Ry->setDecimals(4);
        spb_Ry->setMinimum(-180.000000000000000);
        spb_Ry->setMaximum(180.000000000000000);
        spb_Ry->setSingleStep(1.000000000000000);

        gridLayout_12->addWidget(spb_Ry, 1, 6, 1, 1);

        label_160 = new QLabel(layoutWidget1);
        label_160->setObjectName(QString::fromUtf8("label_160"));
        label_160->setMaximumSize(QSize(83, 16777215));

        gridLayout_12->addWidget(label_160, 0, 1, 1, 1);

        label_155 = new QLabel(layoutWidget1);
        label_155->setObjectName(QString::fromUtf8("label_155"));
        label_155->setMaximumSize(QSize(83, 16777215));

        gridLayout_12->addWidget(label_155, 0, 3, 1, 1);

        label_157 = new QLabel(layoutWidget1);
        label_157->setObjectName(QString::fromUtf8("label_157"));

        gridLayout_12->addWidget(label_157, 0, 6, 1, 1);

        label_156 = new QLabel(layoutWidget1);
        label_156->setObjectName(QString::fromUtf8("label_156"));

        gridLayout_12->addWidget(label_156, 0, 5, 1, 1);

        spb_Tz = new QDoubleSpinBox(layoutWidget1);
        spb_Tz->setObjectName(QString::fromUtf8("spb_Tz"));
        spb_Tz->setDecimals(4);
        spb_Tz->setMinimum(-1.000000000000000);
        spb_Tz->setMaximum(1.000000000000000);
        spb_Tz->setSingleStep(0.001000000000000);

        gridLayout_12->addWidget(spb_Tz, 1, 3, 1, 1);

        spb_Tx = new QDoubleSpinBox(layoutWidget1);
        spb_Tx->setObjectName(QString::fromUtf8("spb_Tx"));
        spb_Tx->setDecimals(4);
        spb_Tx->setMinimum(-1.000000000000000);
        spb_Tx->setMaximum(1.000000000000000);
        spb_Tx->setSingleStep(0.001000000000000);

        gridLayout_12->addWidget(spb_Tx, 1, 1, 1, 1);

        label_158 = new QLabel(layoutWidget1);
        label_158->setObjectName(QString::fromUtf8("label_158"));

        gridLayout_12->addWidget(label_158, 0, 7, 1, 1);

        label_140 = new QLabel(layoutWidget1);
        label_140->setObjectName(QString::fromUtf8("label_140"));
        label_140->setEnabled(true);
        QSizePolicy sizePolicy1(QSizePolicy::Minimum, QSizePolicy::Minimum);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(label_140->sizePolicy().hasHeightForWidth());
        label_140->setSizePolicy(sizePolicy1);
        label_140->setMaximumSize(QSize(30, 20));

        gridLayout_12->addWidget(label_140, 0, 0, 1, 1);

        spb_Rz = new QDoubleSpinBox(layoutWidget1);
        spb_Rz->setObjectName(QString::fromUtf8("spb_Rz"));
        spb_Rz->setDecimals(4);
        spb_Rz->setMinimum(-180.000000000000000);
        spb_Rz->setMaximum(180.000000000000000);
        spb_Rz->setSingleStep(1.000000000000000);

        gridLayout_12->addWidget(spb_Rz, 1, 5, 1, 1);

        spb_Rx = new QDoubleSpinBox(layoutWidget1);
        spb_Rx->setObjectName(QString::fromUtf8("spb_Rx"));
        spb_Rx->setDecimals(4);
        spb_Rx->setMinimum(-180.000000000000000);
        spb_Rx->setMaximum(180.000000000000000);
        spb_Rx->setSingleStep(1.000000000000000);

        gridLayout_12->addWidget(spb_Rx, 1, 7, 1, 1);

        layoutWidget2 = new QWidget(tab);
        layoutWidget2->setObjectName(QString::fromUtf8("layoutWidget2"));
        layoutWidget2->setGeometry(QRect(1311, 596, 171, 182));
        gridLayout_13 = new QGridLayout(layoutWidget2);
        gridLayout_13->setObjectName(QString::fromUtf8("gridLayout_13"));
        gridLayout_13->setContentsMargins(0, 0, 0, 0);
        move_x = new QLabel(layoutWidget2);
        move_x->setObjectName(QString::fromUtf8("move_x"));
        move_x->setMinimumSize(QSize(40, 0));
        move_x->setAlignment(Qt::AlignCenter);

        gridLayout_13->addWidget(move_x, 0, 0, 1, 1);

        move_x_val = new QLineEdit(layoutWidget2);
        move_x_val->setObjectName(QString::fromUtf8("move_x_val"));
        QSizePolicy sizePolicy2(QSizePolicy::Maximum, QSizePolicy::Maximum);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(move_x_val->sizePolicy().hasHeightForWidth());
        move_x_val->setSizePolicy(sizePolicy2);
        move_x_val->setMaximumSize(QSize(200, 500));

        gridLayout_13->addWidget(move_x_val, 0, 1, 1, 1);

        move_y = new QLabel(layoutWidget2);
        move_y->setObjectName(QString::fromUtf8("move_y"));
        move_y->setMinimumSize(QSize(40, 0));
        move_y->setAlignment(Qt::AlignCenter);

        gridLayout_13->addWidget(move_y, 1, 0, 1, 1);

        move_y_val = new QLineEdit(layoutWidget2);
        move_y_val->setObjectName(QString::fromUtf8("move_y_val"));
        QSizePolicy sizePolicy3(QSizePolicy::Expanding, QSizePolicy::Maximum);
        sizePolicy3.setHorizontalStretch(0);
        sizePolicy3.setVerticalStretch(0);
        sizePolicy3.setHeightForWidth(move_y_val->sizePolicy().hasHeightForWidth());
        move_y_val->setSizePolicy(sizePolicy3);
        move_y_val->setMaximumSize(QSize(200, 500));

        gridLayout_13->addWidget(move_y_val, 1, 1, 1, 1);

        move_z = new QLabel(layoutWidget2);
        move_z->setObjectName(QString::fromUtf8("move_z"));
        move_z->setMinimumSize(QSize(40, 0));
        move_z->setAlignment(Qt::AlignCenter);

        gridLayout_13->addWidget(move_z, 2, 0, 1, 1);

        move_z_val = new QLineEdit(layoutWidget2);
        move_z_val->setObjectName(QString::fromUtf8("move_z_val"));

        gridLayout_13->addWidget(move_z_val, 2, 1, 1, 1);

        move_rx = new QLabel(layoutWidget2);
        move_rx->setObjectName(QString::fromUtf8("move_rx"));
        move_rx->setMinimumSize(QSize(40, 0));
        move_rx->setAlignment(Qt::AlignCenter);

        gridLayout_13->addWidget(move_rx, 3, 0, 1, 1);

        move_rx_val = new QLineEdit(layoutWidget2);
        move_rx_val->setObjectName(QString::fromUtf8("move_rx_val"));

        gridLayout_13->addWidget(move_rx_val, 3, 1, 1, 1);

        move_ry = new QLabel(layoutWidget2);
        move_ry->setObjectName(QString::fromUtf8("move_ry"));
        move_ry->setMinimumSize(QSize(40, 0));
        move_ry->setAlignment(Qt::AlignCenter);

        gridLayout_13->addWidget(move_ry, 4, 0, 1, 1);

        move_ry_val = new QLineEdit(layoutWidget2);
        move_ry_val->setObjectName(QString::fromUtf8("move_ry_val"));

        gridLayout_13->addWidget(move_ry_val, 4, 1, 1, 1);

        move_rz = new QLabel(layoutWidget2);
        move_rz->setObjectName(QString::fromUtf8("move_rz"));
        move_rz->setMinimumSize(QSize(40, 0));
        move_rz->setAlignment(Qt::AlignCenter);

        gridLayout_13->addWidget(move_rz, 5, 0, 1, 1);

        move_rz_val = new QLineEdit(layoutWidget2);
        move_rz_val->setObjectName(QString::fromUtf8("move_rz_val"));

        gridLayout_13->addWidget(move_rz_val, 5, 1, 1, 1);

        bt_vision_cmd_capture_4 = new QPushButton(tab);
        bt_vision_cmd_capture_4->setObjectName(QString::fromUtf8("bt_vision_cmd_capture_4"));
        bt_vision_cmd_capture_4->setGeometry(QRect(1520, 700, 101, 21));
        bt_vision_cmd_capture_4->setContextMenuPolicy(Qt::DefaultContextMenu);
        bt_vision_cmd_capture_4->setAutoFillBackground(false);
        bt_TCP_Blend = new QPushButton(tab);
        bt_TCP_Blend->setObjectName(QString::fromUtf8("bt_TCP_Blend"));
        bt_TCP_Blend->setGeometry(QRect(1160, 530, 101, 21));
        bt_TCP_Blend->setContextMenuPolicy(Qt::DefaultContextMenu);
        bt_TCP_Blend->setAutoFillBackground(false);
        layoutWidget3 = new QWidget(tab);
        layoutWidget3->setObjectName(QString::fromUtf8("layoutWidget3"));
        layoutWidget3->setGeometry(QRect(30, 800, 631, 131));
        gridLayout_14 = new QGridLayout(layoutWidget3);
        gridLayout_14->setObjectName(QString::fromUtf8("gridLayout_14"));
        gridLayout_14->setContentsMargins(0, 0, 0, 0);
        label_159 = new QLabel(layoutWidget3);
        label_159->setObjectName(QString::fromUtf8("label_159"));

        gridLayout_14->addWidget(label_159, 0, 0, 1, 1);

        te_log = new QTextEdit(layoutWidget3);
        te_log->setObjectName(QString::fromUtf8("te_log"));

        gridLayout_14->addWidget(te_log, 1, 0, 1, 1);

        bt_low_get_out = new QPushButton(tab);
        bt_low_get_out->setObjectName(QString::fromUtf8("bt_low_get_out"));
        bt_low_get_out->setGeometry(QRect(1520, 740, 101, 21));
        bt_low_get_out->setContextMenuPolicy(Qt::DefaultContextMenu);
        bt_low_get_out->setAutoFillBackground(false);
        bt_vision_water = new QPushButton(tab);
        bt_vision_water->setObjectName(QString::fromUtf8("bt_vision_water"));
        bt_vision_water->setGeometry(QRect(1320, 840, 161, 21));
        bt_vision_water->setContextMenuPolicy(Qt::DefaultContextMenu);
        bt_vision_water->setAutoFillBackground(false);
        bt_avino_low_get_out_2 = new QPushButton(tab);
        bt_avino_low_get_out_2->setObjectName(QString::fromUtf8("bt_avino_low_get_out_2"));
        bt_avino_low_get_out_2->setGeometry(QRect(1490, 890, 131, 21));
        bt_avino_low_get_out_2->setContextMenuPolicy(Qt::DefaultContextMenu);
        bt_avino_low_get_out_2->setAutoFillBackground(false);
        bt_vision_high_get_out = new QPushButton(tab);
        bt_vision_high_get_out->setObjectName(QString::fromUtf8("bt_vision_high_get_out"));
        bt_vision_high_get_out->setGeometry(QRect(1320, 870, 101, 21));
        bt_vision_high_get_out->setContextMenuPolicy(Qt::DefaultContextMenu);
        bt_vision_high_get_out->setAutoFillBackground(false);
        bt_vision_rael_more_move = new QPushButton(tab);
        bt_vision_rael_more_move->setObjectName(QString::fromUtf8("bt_vision_rael_more_move"));
        bt_vision_rael_more_move->setGeometry(QRect(1320, 900, 131, 21));
        bt_vision_rael_more_move->setContextMenuPolicy(Qt::DefaultContextMenu);
        bt_vision_rael_more_move->setAutoFillBackground(false);
        bt_low_get_out_2 = new QPushButton(tab);
        bt_low_get_out_2->setObjectName(QString::fromUtf8("bt_low_get_out_2"));
        bt_low_get_out_2->setGeometry(QRect(1490, 840, 101, 21));
        bt_low_get_out_2->setContextMenuPolicy(Qt::DefaultContextMenu);
        bt_low_get_out_2->setAutoFillBackground(false);
        LE_INFO_MOVE_INFO_1 = new QLineEdit(tab);
        LE_INFO_MOVE_INFO_1->setObjectName(QString::fromUtf8("LE_INFO_MOVE_INFO_1"));
        LE_INFO_MOVE_INFO_1->setEnabled(false);
        LE_INFO_MOVE_INFO_1->setGeometry(QRect(490, 37, 27, 25));
        sizePolicy.setHeightForWidth(LE_INFO_MOVE_INFO_1->sizePolicy().hasHeightForWidth());
        LE_INFO_MOVE_INFO_1->setSizePolicy(sizePolicy);
        LE_INFO_MOVE_INFO_1->setStyleSheet(QString::fromUtf8("background-color:rgb(255,0,0)"));
        label_139 = new QLabel(tab);
        label_139->setObjectName(QString::fromUtf8("label_139"));
        label_139->setGeometry(QRect(520, 38, 31, 20));
        label_139->setFont(font7);
        label_181 = new QLabel(tab);
        label_181->setObjectName(QString::fromUtf8("label_181"));
        label_181->setGeometry(QRect(614, 76, 51, 20));
        label_181->setFont(font);
        label_181->setStyleSheet(QString::fromUtf8("    font:  10px;\n"
""));
        label_181->setAlignment(Qt::AlignCenter);
        le_scenario = new QLineEdit(tab);
        le_scenario->setObjectName(QString::fromUtf8("le_scenario"));
        le_scenario->setGeometry(QRect(620, 100, 40, 40));
        le_scenario->setStyleSheet(QString::fromUtf8("    border-style: outset;\n"
"    border-width: 1px;\n"
"    border-color: black;\n"
"    background-color:rgb(255,0,0)"));
        le_scenario->setReadOnly(true);
        layoutWidget4 = new QWidget(tab);
        layoutWidget4->setObjectName(QString::fromUtf8("layoutWidget4"));
        layoutWidget4->setGeometry(QRect(1310, 560, 259, 27));
        horizontalLayout_14 = new QHBoxLayout(layoutWidget4);
        horizontalLayout_14->setObjectName(QString::fromUtf8("horizontalLayout_14"));
        horizontalLayout_14->setContentsMargins(0, 0, 0, 0);
        bt_cobot_move2object_approach = new QPushButton(layoutWidget4);
        bt_cobot_move2object_approach->setObjectName(QString::fromUtf8("bt_cobot_move2object_approach"));
        bt_cobot_move2object_approach->setContextMenuPolicy(Qt::DefaultContextMenu);
        bt_cobot_move2object_approach->setAutoFillBackground(false);

        horizontalLayout_14->addWidget(bt_cobot_move2object_approach);

        bt_cobot_move2object = new QPushButton(layoutWidget4);
        bt_cobot_move2object->setObjectName(QString::fromUtf8("bt_cobot_move2object"));
        bt_cobot_move2object->setContextMenuPolicy(Qt::DefaultContextMenu);
        bt_cobot_move2object->setAutoFillBackground(false);

        horizontalLayout_14->addWidget(bt_cobot_move2object);

        bt_cobot_push = new QPushButton(layoutWidget4);
        bt_cobot_push->setObjectName(QString::fromUtf8("bt_cobot_push"));
        bt_cobot_push->setContextMenuPolicy(Qt::DefaultContextMenu);
        bt_cobot_push->setAutoFillBackground(false);

        horizontalLayout_14->addWidget(bt_cobot_push);

        la_caution = new QLineEdit(tab);
        la_caution->setObjectName(QString::fromUtf8("la_caution"));
        la_caution->setGeometry(QRect(1360, 538, 121, 21));
        scenario->addTab(tab, QString());
        rb_5 = new QWidget();
        rb_5->setObjectName(QString::fromUtf8("rb_5"));
        HS_BASE_SPEED = new QSlider(rb_5);
        HS_BASE_SPEED->setObjectName(QString::fromUtf8("HS_BASE_SPEED"));
        HS_BASE_SPEED->setGeometry(QRect(670, 70, 171, 31));
        HS_BASE_SPEED->setMinimum(0);
        HS_BASE_SPEED->setMaximum(100);
        HS_BASE_SPEED->setOrientation(Qt::Horizontal);
        GB_NETWORK = new QGroupBox(rb_5);
        GB_NETWORK->setObjectName(QString::fromUtf8("GB_NETWORK"));
        GB_NETWORK->setGeometry(QRect(20, 40, 191, 171));
        QFont font8;
        font8.setPointSize(11);
        GB_NETWORK->setFont(font8);
        LE_IP = new QLineEdit(GB_NETWORK);
        LE_IP->setObjectName(QString::fromUtf8("LE_IP"));
        LE_IP->setGeometry(QRect(80, 30, 101, 31));
        LE_IP->setMaximumSize(QSize(150, 16777215));
        LE_IP->setAlignment(Qt::AlignCenter);
        label_8 = new QLabel(GB_NETWORK);
        label_8->setObjectName(QString::fromUtf8("label_8"));
        label_8->setGeometry(QRect(13, 30, 61, 31));
        textCmdPort = new QLineEdit(GB_NETWORK);
        textCmdPort->setObjectName(QString::fromUtf8("textCmdPort"));
        textCmdPort->setEnabled(false);
        textCmdPort->setGeometry(QRect(17, 90, 70, 21));
        textCmdPort->setMaximumSize(QSize(100, 16777215));
        textCmdPort->setAlignment(Qt::AlignCenter);
        label_10 = new QLabel(GB_NETWORK);
        label_10->setObjectName(QString::fromUtf8("label_10"));
        label_10->setGeometry(QRect(107, 70, 70, 21));
        label_11 = new QLabel(GB_NETWORK);
        label_11->setObjectName(QString::fromUtf8("label_11"));
        label_11->setGeometry(QRect(19, 70, 70, 21));
        textDataPort = new QLineEdit(GB_NETWORK);
        textDataPort->setObjectName(QString::fromUtf8("textDataPort"));
        textDataPort->setEnabled(false);
        textDataPort->setGeometry(QRect(107, 90, 70, 21));
        textDataPort->setMaximumSize(QSize(100, 16777215));
        textDataPort->setAlignment(Qt::AlignCenter);
        BTN_CONNECT_COM = new QPushButton(GB_NETWORK);
        BTN_CONNECT_COM->setObjectName(QString::fromUtf8("BTN_CONNECT_COM"));
        BTN_CONNECT_COM->setGeometry(QRect(17, 120, 70, 40));
        BTN_CONNECT_COM->setMinimumSize(QSize(0, 40));
        BTN_CONNECT_DATA = new QPushButton(GB_NETWORK);
        BTN_CONNECT_DATA->setObjectName(QString::fromUtf8("BTN_CONNECT_DATA"));
        BTN_CONNECT_DATA->setGeometry(QRect(107, 120, 70, 41));
        BTN_CONNECT_DATA->setMinimumSize(QSize(0, 40));
        groupBox_4 = new QGroupBox(rb_5);
        groupBox_4->setObjectName(QString::fromUtf8("groupBox_4"));
        groupBox_4->setGeometry(QRect(220, 40, 311, 171));
        BTN_COBOT_INIT = new QPushButton(groupBox_4);
        BTN_COBOT_INIT->setObjectName(QString::fromUtf8("BTN_COBOT_INIT"));
        BTN_COBOT_INIT->setGeometry(QRect(10, 120, 291, 41));
        LE_INIT_POWER = new QLineEdit(groupBox_4);
        LE_INIT_POWER->setObjectName(QString::fromUtf8("LE_INIT_POWER"));
        LE_INIT_POWER->setGeometry(QRect(10, 30, 31, 31));
        label_12 = new QLabel(groupBox_4);
        label_12->setObjectName(QString::fromUtf8("label_12"));
        label_12->setGeometry(QRect(50, 29, 91, 31));
        label_13 = new QLabel(groupBox_4);
        label_13->setObjectName(QString::fromUtf8("label_13"));
        label_13->setGeometry(QRect(50, 80, 91, 31));
        LE_INIT_DEVICE = new QLineEdit(groupBox_4);
        LE_INIT_DEVICE->setObjectName(QString::fromUtf8("LE_INIT_DEVICE"));
        LE_INIT_DEVICE->setGeometry(QRect(10, 80, 31, 31));
        label_14 = new QLabel(groupBox_4);
        label_14->setObjectName(QString::fromUtf8("label_14"));
        label_14->setGeometry(QRect(200, 30, 101, 31));
        LE_INIT_SYSTEM = new QLineEdit(groupBox_4);
        LE_INIT_SYSTEM->setObjectName(QString::fromUtf8("LE_INIT_SYSTEM"));
        LE_INIT_SYSTEM->setGeometry(QRect(160, 30, 31, 31));
        LE_INIT_ROBOT = new QLineEdit(groupBox_4);
        LE_INIT_ROBOT->setObjectName(QString::fromUtf8("LE_INIT_ROBOT"));
        LE_INIT_ROBOT->setGeometry(QRect(160, 80, 31, 31));
        label_15 = new QLabel(groupBox_4);
        label_15->setObjectName(QString::fromUtf8("label_15"));
        label_15->setGeometry(QRect(200, 80, 101, 31));
        groupBox_6 = new QGroupBox(rb_5);
        groupBox_6->setObjectName(QString::fromUtf8("groupBox_6"));
        groupBox_6->setGeometry(QRect(670, 180, 401, 441));
        textPrint = new QTextEdit(groupBox_6);
        textPrint->setObjectName(QString::fromUtf8("textPrint"));
        textPrint->setGeometry(QRect(10, 70, 381, 361));
        BTN_CLEAR = new QPushButton(groupBox_6);
        BTN_CLEAR->setObjectName(QString::fromUtf8("BTN_CLEAR"));
        BTN_CLEAR->setGeometry(QRect(10, 30, 101, 31));
        label_16 = new QLabel(rb_5);
        label_16->setObjectName(QString::fromUtf8("label_16"));
        label_16->setGeometry(QRect(670, 50, 91, 16));
        BTN_MOTION_RESUME = new QPushButton(rb_5);
        BTN_MOTION_RESUME->setObjectName(QString::fromUtf8("BTN_MOTION_RESUME"));
        BTN_MOTION_RESUME->setGeometry(QRect(990, 40, 101, 61));
        BTN_MOTION_RESUME->setStyleSheet(QString::fromUtf8("background-color: rgb(138, 226, 52);"));
        BTN_MOVE_JOINT_READY_GRIPPER = new QPushButton(rb_5);
        BTN_MOVE_JOINT_READY_GRIPPER->setObjectName(QString::fromUtf8("BTN_MOVE_JOINT_READY_GRIPPER"));
        BTN_MOVE_JOINT_READY_GRIPPER->setGeometry(QRect(1480, 670, 141, 71));
        BTN_MOTION_HALT = new QPushButton(rb_5);
        BTN_MOTION_HALT->setObjectName(QString::fromUtf8("BTN_MOTION_HALT"));
        BTN_MOTION_HALT->setGeometry(QRect(1110, 40, 101, 61));
        BTN_MOTION_HALT->setStyleSheet(QString::fromUtf8("background-color: rgb(252, 175, 62);"));
        FRAME_STATUS = new QFrame(rb_5);
        FRAME_STATUS->setObjectName(QString::fromUtf8("FRAME_STATUS"));
        FRAME_STATUS->setGeometry(QRect(20, 220, 630, 700));
        FRAME_STATUS->setFrameShape(QFrame::StyledPanel);
        FRAME_STATUS->setFrameShadow(QFrame::Raised);
        gridLayoutWidget_4 = new QWidget(FRAME_STATUS);
        gridLayoutWidget_4->setObjectName(QString::fromUtf8("gridLayoutWidget_4"));
        gridLayoutWidget_4->setGeometry(QRect(10, 263, 271, 227));
        gridLayout_4 = new QGridLayout(gridLayoutWidget_4);
        gridLayout_4->setObjectName(QString::fromUtf8("gridLayout_4"));
        gridLayout_4->setContentsMargins(0, 0, 0, 0);
        LE_D_IN_OFF_12 = new QLineEdit(gridLayoutWidget_4);
        LE_D_IN_OFF_12->setObjectName(QString::fromUtf8("LE_D_IN_OFF_12"));
        LE_D_IN_OFF_12->setMinimumSize(QSize(30, 0));
        LE_D_IN_OFF_12->setMaximumSize(QSize(30, 20));

        gridLayout_4->addWidget(LE_D_IN_OFF_12, 5, 6, 1, 1);

        LE_D_IN_ON_9 = new QLineEdit(gridLayoutWidget_4);
        LE_D_IN_ON_9->setObjectName(QString::fromUtf8("LE_D_IN_ON_9"));
        LE_D_IN_ON_9->setMinimumSize(QSize(30, 0));
        LE_D_IN_ON_9->setMaximumSize(QSize(30, 20));

        gridLayout_4->addWidget(LE_D_IN_ON_9, 2, 5, 1, 1);

        horizontalSpacer = new QSpacerItem(20, 20, QSizePolicy::Fixed, QSizePolicy::Minimum);

        gridLayout_4->addItem(horizontalSpacer, 2, 3, 1, 1);

        LE_D_IN_OFF_9 = new QLineEdit(gridLayoutWidget_4);
        LE_D_IN_OFF_9->setObjectName(QString::fromUtf8("LE_D_IN_OFF_9"));
        LE_D_IN_OFF_9->setMinimumSize(QSize(30, 0));
        LE_D_IN_OFF_9->setMaximumSize(QSize(30, 20));

        gridLayout_4->addWidget(LE_D_IN_OFF_9, 2, 6, 1, 1);

        label_25 = new QLabel(gridLayoutWidget_4);
        label_25->setObjectName(QString::fromUtf8("label_25"));
        label_25->setMinimumSize(QSize(30, 0));
        label_25->setMaximumSize(QSize(30, 20));
        label_25->setAlignment(Qt::AlignCenter);

        gridLayout_4->addWidget(label_25, 4, 0, 1, 1);

        LE_D_IN_OFF_0 = new QLineEdit(gridLayoutWidget_4);
        LE_D_IN_OFF_0->setObjectName(QString::fromUtf8("LE_D_IN_OFF_0"));
        LE_D_IN_OFF_0->setMinimumSize(QSize(30, 0));
        LE_D_IN_OFF_0->setMaximumSize(QSize(30, 20));

        gridLayout_4->addWidget(LE_D_IN_OFF_0, 1, 2, 1, 1);

        label_36 = new QLabel(gridLayoutWidget_4);
        label_36->setObjectName(QString::fromUtf8("label_36"));
        label_36->setMinimumSize(QSize(30, 0));
        label_36->setMaximumSize(QSize(30, 20));
        label_36->setAlignment(Qt::AlignCenter);

        gridLayout_4->addWidget(label_36, 5, 4, 1, 1);

        LE_D_IN_ON_15 = new QLineEdit(gridLayoutWidget_4);
        LE_D_IN_ON_15->setObjectName(QString::fromUtf8("LE_D_IN_ON_15"));
        LE_D_IN_ON_15->setMinimumSize(QSize(30, 0));
        LE_D_IN_ON_15->setMaximumSize(QSize(30, 20));

        gridLayout_4->addWidget(LE_D_IN_ON_15, 8, 5, 4, 1);

        label_28 = new QLabel(gridLayoutWidget_4);
        label_28->setObjectName(QString::fromUtf8("label_28"));
        label_28->setMinimumSize(QSize(30, 0));
        label_28->setMaximumSize(QSize(30, 20));
        label_28->setAlignment(Qt::AlignCenter);

        gridLayout_4->addWidget(label_28, 7, 0, 1, 1);

        LE_D_IN_OFF_5 = new QLineEdit(gridLayoutWidget_4);
        LE_D_IN_OFF_5->setObjectName(QString::fromUtf8("LE_D_IN_OFF_5"));
        LE_D_IN_OFF_5->setMinimumSize(QSize(30, 0));
        LE_D_IN_OFF_5->setMaximumSize(QSize(30, 20));

        gridLayout_4->addWidget(LE_D_IN_OFF_5, 6, 2, 1, 1);

        LE_D_IN_ON_10 = new QLineEdit(gridLayoutWidget_4);
        LE_D_IN_ON_10->setObjectName(QString::fromUtf8("LE_D_IN_ON_10"));
        LE_D_IN_ON_10->setMinimumSize(QSize(30, 0));
        LE_D_IN_ON_10->setMaximumSize(QSize(30, 20));

        gridLayout_4->addWidget(LE_D_IN_ON_10, 3, 5, 1, 1);

        label_24 = new QLabel(gridLayoutWidget_4);
        label_24->setObjectName(QString::fromUtf8("label_24"));
        label_24->setMinimumSize(QSize(30, 0));
        label_24->setMaximumSize(QSize(30, 20));
        label_24->setAlignment(Qt::AlignCenter);

        gridLayout_4->addWidget(label_24, 2, 0, 1, 1);

        label_37 = new QLabel(gridLayoutWidget_4);
        label_37->setObjectName(QString::fromUtf8("label_37"));
        label_37->setMinimumSize(QSize(30, 0));
        label_37->setMaximumSize(QSize(30, 20));
        label_37->setAlignment(Qt::AlignCenter);

        gridLayout_4->addWidget(label_37, 6, 4, 1, 1);

        LE_D_IN_ON_2 = new QLineEdit(gridLayoutWidget_4);
        LE_D_IN_ON_2->setObjectName(QString::fromUtf8("LE_D_IN_ON_2"));
        LE_D_IN_ON_2->setMinimumSize(QSize(30, 0));
        LE_D_IN_ON_2->setMaximumSize(QSize(30, 20));

        gridLayout_4->addWidget(LE_D_IN_ON_2, 3, 1, 1, 1);

        LE_D_IN_ON_5 = new QLineEdit(gridLayoutWidget_4);
        LE_D_IN_ON_5->setObjectName(QString::fromUtf8("LE_D_IN_ON_5"));
        LE_D_IN_ON_5->setMinimumSize(QSize(30, 0));
        LE_D_IN_ON_5->setMaximumSize(QSize(30, 20));

        gridLayout_4->addWidget(LE_D_IN_ON_5, 6, 1, 1, 1);

        LE_D_IN_OFF_2 = new QLineEdit(gridLayoutWidget_4);
        LE_D_IN_OFF_2->setObjectName(QString::fromUtf8("LE_D_IN_OFF_2"));
        LE_D_IN_OFF_2->setMinimumSize(QSize(30, 0));
        LE_D_IN_OFF_2->setMaximumSize(QSize(30, 20));

        gridLayout_4->addWidget(LE_D_IN_OFF_2, 3, 2, 1, 1);

        label_38 = new QLabel(gridLayoutWidget_4);
        label_38->setObjectName(QString::fromUtf8("label_38"));
        label_38->setMinimumSize(QSize(30, 0));
        label_38->setMaximumSize(QSize(30, 20));
        label_38->setAlignment(Qt::AlignCenter);

        gridLayout_4->addWidget(label_38, 7, 4, 1, 1);

        label_21 = new QLabel(gridLayoutWidget_4);
        label_21->setObjectName(QString::fromUtf8("label_21"));
        label_21->setMinimumSize(QSize(30, 0));
        label_21->setMaximumSize(QSize(30, 20));
        label_21->setAlignment(Qt::AlignCenter);

        gridLayout_4->addWidget(label_21, 5, 0, 1, 1);

        LE_D_IN_ON_12 = new QLineEdit(gridLayoutWidget_4);
        LE_D_IN_ON_12->setObjectName(QString::fromUtf8("LE_D_IN_ON_12"));
        LE_D_IN_ON_12->setMinimumSize(QSize(30, 0));
        LE_D_IN_ON_12->setMaximumSize(QSize(30, 20));

        gridLayout_4->addWidget(LE_D_IN_ON_12, 5, 5, 1, 1);

        LE_D_IN_ON_3 = new QLineEdit(gridLayoutWidget_4);
        LE_D_IN_ON_3->setObjectName(QString::fromUtf8("LE_D_IN_ON_3"));
        LE_D_IN_ON_3->setMinimumSize(QSize(30, 0));
        LE_D_IN_ON_3->setMaximumSize(QSize(30, 20));

        gridLayout_4->addWidget(LE_D_IN_ON_3, 4, 1, 1, 1);

        LE_D_IN_OFF_11 = new QLineEdit(gridLayoutWidget_4);
        LE_D_IN_OFF_11->setObjectName(QString::fromUtf8("LE_D_IN_OFF_11"));
        LE_D_IN_OFF_11->setMinimumSize(QSize(30, 0));
        LE_D_IN_OFF_11->setMaximumSize(QSize(30, 20));

        gridLayout_4->addWidget(LE_D_IN_OFF_11, 4, 6, 1, 1);

        label_26 = new QLabel(gridLayoutWidget_4);
        label_26->setObjectName(QString::fromUtf8("label_26"));
        label_26->setMinimumSize(QSize(30, 0));
        label_26->setMaximumSize(QSize(30, 20));
        label_26->setAlignment(Qt::AlignCenter);

        gridLayout_4->addWidget(label_26, 3, 0, 1, 1);

        LE_D_IN_ON_1 = new QLineEdit(gridLayoutWidget_4);
        LE_D_IN_ON_1->setObjectName(QString::fromUtf8("LE_D_IN_ON_1"));
        LE_D_IN_ON_1->setMinimumSize(QSize(30, 0));
        LE_D_IN_ON_1->setMaximumSize(QSize(30, 20));

        gridLayout_4->addWidget(LE_D_IN_ON_1, 2, 1, 1, 1);

        LE_D_IN_OFF_15 = new QLineEdit(gridLayoutWidget_4);
        LE_D_IN_OFF_15->setObjectName(QString::fromUtf8("LE_D_IN_OFF_15"));
        LE_D_IN_OFF_15->setMinimumSize(QSize(30, 0));
        LE_D_IN_OFF_15->setMaximumSize(QSize(30, 20));

        gridLayout_4->addWidget(LE_D_IN_OFF_15, 8, 6, 4, 1);

        LE_D_IN_OFF_8 = new QLineEdit(gridLayoutWidget_4);
        LE_D_IN_OFF_8->setObjectName(QString::fromUtf8("LE_D_IN_OFF_8"));
        LE_D_IN_OFF_8->setMinimumSize(QSize(30, 0));
        LE_D_IN_OFF_8->setMaximumSize(QSize(30, 20));

        gridLayout_4->addWidget(LE_D_IN_OFF_8, 1, 6, 1, 1);

        LE_D_IN_ON_11 = new QLineEdit(gridLayoutWidget_4);
        LE_D_IN_ON_11->setObjectName(QString::fromUtf8("LE_D_IN_ON_11"));
        LE_D_IN_ON_11->setMinimumSize(QSize(30, 0));
        LE_D_IN_ON_11->setMaximumSize(QSize(30, 20));

        gridLayout_4->addWidget(LE_D_IN_ON_11, 4, 5, 1, 1);

        LE_D_IN_ON_6 = new QLineEdit(gridLayoutWidget_4);
        LE_D_IN_ON_6->setObjectName(QString::fromUtf8("LE_D_IN_ON_6"));
        LE_D_IN_ON_6->setMinimumSize(QSize(30, 0));
        LE_D_IN_ON_6->setMaximumSize(QSize(30, 20));

        gridLayout_4->addWidget(LE_D_IN_ON_6, 7, 1, 1, 1);

        LE_D_IN_OFF_1 = new QLineEdit(gridLayoutWidget_4);
        LE_D_IN_OFF_1->setObjectName(QString::fromUtf8("LE_D_IN_OFF_1"));
        LE_D_IN_OFF_1->setMinimumSize(QSize(30, 0));
        LE_D_IN_OFF_1->setMaximumSize(QSize(30, 20));

        gridLayout_4->addWidget(LE_D_IN_OFF_1, 2, 2, 1, 1);

        LE_D_IN_OFF_3 = new QLineEdit(gridLayoutWidget_4);
        LE_D_IN_OFF_3->setObjectName(QString::fromUtf8("LE_D_IN_OFF_3"));
        LE_D_IN_OFF_3->setMinimumSize(QSize(30, 0));
        LE_D_IN_OFF_3->setMaximumSize(QSize(30, 20));

        gridLayout_4->addWidget(LE_D_IN_OFF_3, 4, 2, 1, 1);

        LE_D_IN_OFF_13 = new QLineEdit(gridLayoutWidget_4);
        LE_D_IN_OFF_13->setObjectName(QString::fromUtf8("LE_D_IN_OFF_13"));
        LE_D_IN_OFF_13->setMinimumSize(QSize(30, 0));
        LE_D_IN_OFF_13->setMaximumSize(QSize(30, 20));

        gridLayout_4->addWidget(LE_D_IN_OFF_13, 6, 6, 1, 1);

        label_33 = new QLabel(gridLayoutWidget_4);
        label_33->setObjectName(QString::fromUtf8("label_33"));
        label_33->setMinimumSize(QSize(30, 0));
        label_33->setMaximumSize(QSize(30, 20));
        label_33->setAlignment(Qt::AlignCenter);

        gridLayout_4->addWidget(label_33, 2, 4, 1, 1);

        label_22 = new QLabel(gridLayoutWidget_4);
        label_22->setObjectName(QString::fromUtf8("label_22"));
        label_22->setMinimumSize(QSize(30, 0));
        label_22->setMaximumSize(QSize(30, 20));
        label_22->setAlignment(Qt::AlignCenter);

        gridLayout_4->addWidget(label_22, 6, 0, 1, 1);

        LE_D_IN_ON_4 = new QLineEdit(gridLayoutWidget_4);
        LE_D_IN_ON_4->setObjectName(QString::fromUtf8("LE_D_IN_ON_4"));
        LE_D_IN_ON_4->setMinimumSize(QSize(30, 0));
        LE_D_IN_ON_4->setMaximumSize(QSize(30, 20));

        gridLayout_4->addWidget(LE_D_IN_ON_4, 5, 1, 1, 1);

        label_32 = new QLabel(gridLayoutWidget_4);
        label_32->setObjectName(QString::fromUtf8("label_32"));
        label_32->setMinimumSize(QSize(30, 0));
        label_32->setMaximumSize(QSize(30, 20));
        label_32->setAlignment(Qt::AlignCenter);

        gridLayout_4->addWidget(label_32, 1, 4, 1, 1);

        LE_D_IN_OFF_14 = new QLineEdit(gridLayoutWidget_4);
        LE_D_IN_OFF_14->setObjectName(QString::fromUtf8("LE_D_IN_OFF_14"));
        LE_D_IN_OFF_14->setMinimumSize(QSize(30, 0));
        LE_D_IN_OFF_14->setMaximumSize(QSize(30, 20));

        gridLayout_4->addWidget(LE_D_IN_OFF_14, 7, 6, 1, 1);

        LE_D_IN_ON_7 = new QLineEdit(gridLayoutWidget_4);
        LE_D_IN_ON_7->setObjectName(QString::fromUtf8("LE_D_IN_ON_7"));
        LE_D_IN_ON_7->setMinimumSize(QSize(30, 0));
        LE_D_IN_ON_7->setMaximumSize(QSize(30, 20));

        gridLayout_4->addWidget(LE_D_IN_ON_7, 8, 1, 4, 1);

        label_34 = new QLabel(gridLayoutWidget_4);
        label_34->setObjectName(QString::fromUtf8("label_34"));
        label_34->setMinimumSize(QSize(30, 0));
        label_34->setMaximumSize(QSize(30, 20));
        label_34->setAlignment(Qt::AlignCenter);

        gridLayout_4->addWidget(label_34, 3, 4, 1, 1);

        label_39 = new QLabel(gridLayoutWidget_4);
        label_39->setObjectName(QString::fromUtf8("label_39"));
        label_39->setMinimumSize(QSize(30, 0));
        label_39->setMaximumSize(QSize(30, 20));
        label_39->setAlignment(Qt::AlignCenter);

        gridLayout_4->addWidget(label_39, 8, 4, 4, 1);

        LE_D_IN_ON_13 = new QLineEdit(gridLayoutWidget_4);
        LE_D_IN_ON_13->setObjectName(QString::fromUtf8("LE_D_IN_ON_13"));
        LE_D_IN_ON_13->setMinimumSize(QSize(30, 0));
        LE_D_IN_ON_13->setMaximumSize(QSize(30, 20));

        gridLayout_4->addWidget(LE_D_IN_ON_13, 6, 5, 1, 1);

        LE_D_IN_ON_14 = new QLineEdit(gridLayoutWidget_4);
        LE_D_IN_ON_14->setObjectName(QString::fromUtf8("LE_D_IN_ON_14"));
        LE_D_IN_ON_14->setMinimumSize(QSize(30, 0));
        LE_D_IN_ON_14->setMaximumSize(QSize(30, 20));

        gridLayout_4->addWidget(LE_D_IN_ON_14, 7, 5, 1, 1);

        label_35 = new QLabel(gridLayoutWidget_4);
        label_35->setObjectName(QString::fromUtf8("label_35"));
        label_35->setMinimumSize(QSize(30, 0));
        label_35->setMaximumSize(QSize(30, 20));
        label_35->setAlignment(Qt::AlignCenter);

        gridLayout_4->addWidget(label_35, 4, 4, 1, 1);

        LE_D_IN_ON_8 = new QLineEdit(gridLayoutWidget_4);
        LE_D_IN_ON_8->setObjectName(QString::fromUtf8("LE_D_IN_ON_8"));
        LE_D_IN_ON_8->setMinimumSize(QSize(30, 0));
        LE_D_IN_ON_8->setMaximumSize(QSize(30, 20));

        gridLayout_4->addWidget(LE_D_IN_ON_8, 1, 5, 1, 1);

        LE_D_IN_OFF_7 = new QLineEdit(gridLayoutWidget_4);
        LE_D_IN_OFF_7->setObjectName(QString::fromUtf8("LE_D_IN_OFF_7"));
        LE_D_IN_OFF_7->setMinimumSize(QSize(30, 0));
        LE_D_IN_OFF_7->setMaximumSize(QSize(30, 20));

        gridLayout_4->addWidget(LE_D_IN_OFF_7, 8, 2, 4, 1);

        label_23 = new QLabel(gridLayoutWidget_4);
        label_23->setObjectName(QString::fromUtf8("label_23"));
        label_23->setMinimumSize(QSize(30, 0));
        label_23->setMaximumSize(QSize(30, 20));
        label_23->setAlignment(Qt::AlignCenter);

        gridLayout_4->addWidget(label_23, 1, 0, 1, 1);

        LE_D_IN_OFF_6 = new QLineEdit(gridLayoutWidget_4);
        LE_D_IN_OFF_6->setObjectName(QString::fromUtf8("LE_D_IN_OFF_6"));
        LE_D_IN_OFF_6->setMinimumSize(QSize(30, 0));
        LE_D_IN_OFF_6->setMaximumSize(QSize(30, 20));

        gridLayout_4->addWidget(LE_D_IN_OFF_6, 7, 2, 1, 1);

        LE_D_IN_OFF_4 = new QLineEdit(gridLayoutWidget_4);
        LE_D_IN_OFF_4->setObjectName(QString::fromUtf8("LE_D_IN_OFF_4"));
        LE_D_IN_OFF_4->setMinimumSize(QSize(30, 0));
        LE_D_IN_OFF_4->setMaximumSize(QSize(30, 20));

        gridLayout_4->addWidget(LE_D_IN_OFF_4, 5, 2, 1, 1);

        LE_D_IN_ON_0 = new QLineEdit(gridLayoutWidget_4);
        LE_D_IN_ON_0->setObjectName(QString::fromUtf8("LE_D_IN_ON_0"));
        LE_D_IN_ON_0->setMinimumSize(QSize(30, 0));
        LE_D_IN_ON_0->setMaximumSize(QSize(30, 20));

        gridLayout_4->addWidget(LE_D_IN_ON_0, 1, 1, 1, 1);

        label_31 = new QLabel(gridLayoutWidget_4);
        label_31->setObjectName(QString::fromUtf8("label_31"));
        label_31->setMinimumSize(QSize(30, 0));
        label_31->setMaximumSize(QSize(30, 20));
        label_31->setAlignment(Qt::AlignCenter);

        gridLayout_4->addWidget(label_31, 8, 0, 4, 1);

        LE_D_IN_OFF_10 = new QLineEdit(gridLayoutWidget_4);
        LE_D_IN_OFF_10->setObjectName(QString::fromUtf8("LE_D_IN_OFF_10"));
        LE_D_IN_OFF_10->setMinimumSize(QSize(30, 0));
        LE_D_IN_OFF_10->setMaximumSize(QSize(30, 20));

        gridLayout_4->addWidget(LE_D_IN_OFF_10, 3, 6, 1, 1);

        label_40 = new QLabel(gridLayoutWidget_4);
        label_40->setObjectName(QString::fromUtf8("label_40"));
        label_40->setMinimumSize(QSize(30, 0));
        label_40->setMaximumSize(QSize(30, 20));
        label_40->setAlignment(Qt::AlignCenter);

        gridLayout_4->addWidget(label_40, 0, 1, 1, 1);

        label_41 = new QLabel(gridLayoutWidget_4);
        label_41->setObjectName(QString::fromUtf8("label_41"));
        label_41->setMinimumSize(QSize(30, 0));
        label_41->setMaximumSize(QSize(30, 20));
        label_41->setAlignment(Qt::AlignCenter);

        gridLayout_4->addWidget(label_41, 0, 2, 1, 1);

        label_42 = new QLabel(gridLayoutWidget_4);
        label_42->setObjectName(QString::fromUtf8("label_42"));
        label_42->setMinimumSize(QSize(30, 0));
        label_42->setMaximumSize(QSize(30, 20));
        label_42->setAlignment(Qt::AlignCenter);

        gridLayout_4->addWidget(label_42, 0, 5, 1, 1);

        label_43 = new QLabel(gridLayoutWidget_4);
        label_43->setObjectName(QString::fromUtf8("label_43"));
        label_43->setMinimumSize(QSize(30, 0));
        label_43->setMaximumSize(QSize(30, 20));
        label_43->setAlignment(Qt::AlignCenter);

        gridLayout_4->addWidget(label_43, 0, 6, 1, 1);

        label_18 = new QLabel(FRAME_STATUS);
        label_18->setObjectName(QString::fromUtf8("label_18"));
        label_18->setGeometry(QRect(360, 10, 121, 16));
        QFont font9;
        font9.setBold(true);
        font9.setWeight(75);
        label_18->setFont(font9);
        label_18->setAlignment(Qt::AlignCenter);
        gridLayoutWidget = new QWidget(FRAME_STATUS);
        gridLayoutWidget->setObjectName(QString::fromUtf8("gridLayoutWidget"));
        gridLayoutWidget->setGeometry(QRect(220, 30, 121, 206));
        gridLayout = new QGridLayout(gridLayoutWidget);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        gridLayout->setContentsMargins(0, 0, 0, 0);
        label_19 = new QLabel(gridLayoutWidget);
        label_19->setObjectName(QString::fromUtf8("label_19"));
        label_19->setMinimumSize(QSize(40, 0));
        label_19->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(label_19, 4, 0, 1, 1);

        label_20 = new QLabel(gridLayoutWidget);
        label_20->setObjectName(QString::fromUtf8("label_20"));
        label_20->setMinimumSize(QSize(40, 0));
        label_20->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(label_20, 5, 0, 1, 1);

        label_27 = new QLabel(gridLayoutWidget);
        label_27->setObjectName(QString::fromUtf8("label_27"));
        label_27->setMinimumSize(QSize(40, 0));
        label_27->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(label_27, 0, 0, 1, 1);

        label_29 = new QLabel(gridLayoutWidget);
        label_29->setObjectName(QString::fromUtf8("label_29"));
        label_29->setMinimumSize(QSize(40, 0));
        label_29->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(label_29, 1, 0, 1, 1);

        label_30 = new QLabel(gridLayoutWidget);
        label_30->setObjectName(QString::fromUtf8("label_30"));
        label_30->setMinimumSize(QSize(40, 0));
        label_30->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(label_30, 3, 0, 1, 1);

        label_44 = new QLabel(gridLayoutWidget);
        label_44->setObjectName(QString::fromUtf8("label_44"));
        label_44->setMinimumSize(QSize(40, 0));
        label_44->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(label_44, 2, 0, 1, 1);

        LE_JNT_REF_1 = new QLineEdit(gridLayoutWidget);
        LE_JNT_REF_1->setObjectName(QString::fromUtf8("LE_JNT_REF_1"));

        gridLayout->addWidget(LE_JNT_REF_1, 0, 1, 1, 1);

        LE_JNT_REF_2 = new QLineEdit(gridLayoutWidget);
        LE_JNT_REF_2->setObjectName(QString::fromUtf8("LE_JNT_REF_2"));

        gridLayout->addWidget(LE_JNT_REF_2, 1, 1, 1, 1);

        LE_JNT_REF_3 = new QLineEdit(gridLayoutWidget);
        LE_JNT_REF_3->setObjectName(QString::fromUtf8("LE_JNT_REF_3"));

        gridLayout->addWidget(LE_JNT_REF_3, 2, 1, 1, 1);

        LE_JNT_REF_4 = new QLineEdit(gridLayoutWidget);
        LE_JNT_REF_4->setObjectName(QString::fromUtf8("LE_JNT_REF_4"));

        gridLayout->addWidget(LE_JNT_REF_4, 3, 1, 1, 1);

        LE_JNT_REF_5 = new QLineEdit(gridLayoutWidget);
        LE_JNT_REF_5->setObjectName(QString::fromUtf8("LE_JNT_REF_5"));

        gridLayout->addWidget(LE_JNT_REF_5, 4, 1, 1, 1);

        LE_JNT_REF_6 = new QLineEdit(gridLayoutWidget);
        LE_JNT_REF_6->setObjectName(QString::fromUtf8("LE_JNT_REF_6"));

        gridLayout->addWidget(LE_JNT_REF_6, 5, 1, 1, 1);

        gridLayoutWidget_8 = new QWidget(FRAME_STATUS);
        gridLayoutWidget_8->setObjectName(QString::fromUtf8("gridLayoutWidget_8"));
        gridLayoutWidget_8->setGeometry(QRect(500, 621, 121, 66));
        gridLayout_7 = new QGridLayout(gridLayoutWidget_8);
        gridLayout_7->setObjectName(QString::fromUtf8("gridLayout_7"));
        gridLayout_7->setContentsMargins(0, 0, 0, 0);
        label_71 = new QLabel(gridLayoutWidget_8);
        label_71->setObjectName(QString::fromUtf8("label_71"));
        label_71->setMinimumSize(QSize(40, 0));
        label_71->setAlignment(Qt::AlignCenter);

        gridLayout_7->addWidget(label_71, 1, 0, 1, 1);

        LE_TA_IN_0 = new QLineEdit(gridLayoutWidget_8);
        LE_TA_IN_0->setObjectName(QString::fromUtf8("LE_TA_IN_0"));

        gridLayout_7->addWidget(LE_TA_IN_0, 0, 1, 1, 1);

        label_74 = new QLabel(gridLayoutWidget_8);
        label_74->setObjectName(QString::fromUtf8("label_74"));
        label_74->setMinimumSize(QSize(40, 0));
        label_74->setAlignment(Qt::AlignCenter);

        gridLayout_7->addWidget(label_74, 0, 0, 1, 1);

        LE_TA_IN_1 = new QLineEdit(gridLayoutWidget_8);
        LE_TA_IN_1->setObjectName(QString::fromUtf8("LE_TA_IN_1"));

        gridLayout_7->addWidget(LE_TA_IN_1, 1, 1, 1, 1);

        gridLayoutWidget_9 = new QWidget(FRAME_STATUS);
        gridLayoutWidget_9->setObjectName(QString::fromUtf8("gridLayoutWidget_9"));
        gridLayoutWidget_9->setGeometry(QRect(160, 621, 134, 71));
        gridLayout_9 = new QGridLayout(gridLayoutWidget_9);
        gridLayout_9->setObjectName(QString::fromUtf8("gridLayout_9"));
        gridLayout_9->setContentsMargins(0, 0, 0, 0);
        LE_TD_IN_OFF_0 = new QLineEdit(gridLayoutWidget_9);
        LE_TD_IN_OFF_0->setObjectName(QString::fromUtf8("LE_TD_IN_OFF_0"));
        LE_TD_IN_OFF_0->setMaximumSize(QSize(30, 20));

        gridLayout_9->addWidget(LE_TD_IN_OFF_0, 1, 2, 1, 1);

        label_96 = new QLabel(gridLayoutWidget_9);
        label_96->setObjectName(QString::fromUtf8("label_96"));
        label_96->setMinimumSize(QSize(40, 0));
        label_96->setMaximumSize(QSize(16777215, 20));
        label_96->setAlignment(Qt::AlignCenter);

        gridLayout_9->addWidget(label_96, 0, 2, 1, 1);

        label_82 = new QLabel(gridLayoutWidget_9);
        label_82->setObjectName(QString::fromUtf8("label_82"));
        label_82->setMinimumSize(QSize(40, 0));
        label_82->setMaximumSize(QSize(16777215, 20));
        label_82->setAlignment(Qt::AlignCenter);

        gridLayout_9->addWidget(label_82, 2, 0, 1, 1);

        LE_TD_IN_ON_0 = new QLineEdit(gridLayoutWidget_9);
        LE_TD_IN_ON_0->setObjectName(QString::fromUtf8("LE_TD_IN_ON_0"));
        LE_TD_IN_ON_0->setMaximumSize(QSize(30, 20));

        gridLayout_9->addWidget(LE_TD_IN_ON_0, 1, 1, 1, 1);

        label_93 = new QLabel(gridLayoutWidget_9);
        label_93->setObjectName(QString::fromUtf8("label_93"));
        label_93->setMinimumSize(QSize(40, 0));
        label_93->setMaximumSize(QSize(16777215, 20));
        label_93->setAlignment(Qt::AlignCenter);

        gridLayout_9->addWidget(label_93, 1, 0, 1, 1);

        LE_TD_IN_OFF_1 = new QLineEdit(gridLayoutWidget_9);
        LE_TD_IN_OFF_1->setObjectName(QString::fromUtf8("LE_TD_IN_OFF_1"));
        LE_TD_IN_OFF_1->setMaximumSize(QSize(30, 20));

        gridLayout_9->addWidget(LE_TD_IN_OFF_1, 2, 2, 1, 1);

        label_95 = new QLabel(gridLayoutWidget_9);
        label_95->setObjectName(QString::fromUtf8("label_95"));
        label_95->setMinimumSize(QSize(40, 0));
        label_95->setMaximumSize(QSize(16777215, 20));
        label_95->setAlignment(Qt::AlignCenter);

        gridLayout_9->addWidget(label_95, 0, 1, 1, 1);

        LE_TD_IN_ON_1 = new QLineEdit(gridLayoutWidget_9);
        LE_TD_IN_ON_1->setObjectName(QString::fromUtf8("LE_TD_IN_ON_1"));
        LE_TD_IN_ON_1->setMaximumSize(QSize(30, 20));

        gridLayout_9->addWidget(LE_TD_IN_ON_1, 2, 1, 1, 1);

        label_45 = new QLabel(FRAME_STATUS);
        label_45->setObjectName(QString::fromUtf8("label_45"));
        label_45->setGeometry(QRect(10, 100, 81, 16));
        QFont font10;
        font10.setPointSize(10);
        font10.setBold(true);
        font10.setWeight(75);
        label_45->setFont(font10);
        label_45->setAlignment(Qt::AlignCenter);
        label_101 = new QLabel(FRAME_STATUS);
        label_101->setObjectName(QString::fromUtf8("label_101"));
        label_101->setGeometry(QRect(10, 601, 131, 16));
        label_101->setFont(font10);
        label_101->setAlignment(Qt::AlignCenter);
        LE_PG_MODE_SIMULATION = new QLineEdit(FRAME_STATUS);
        LE_PG_MODE_SIMULATION->setObjectName(QString::fromUtf8("LE_PG_MODE_SIMULATION"));
        LE_PG_MODE_SIMULATION->setGeometry(QRect(10, 60, 81, 31));
        LE_PG_MODE_SIMULATION->setAlignment(Qt::AlignCenter);
        gridLayoutWidget_10 = new QWidget(FRAME_STATUS);
        gridLayoutWidget_10->setObjectName(QString::fromUtf8("gridLayoutWidget_10"));
        gridLayoutWidget_10->setGeometry(QRect(342, 621, 134, 71));
        gridLayout_10 = new QGridLayout(gridLayoutWidget_10);
        gridLayout_10->setObjectName(QString::fromUtf8("gridLayout_10"));
        gridLayout_10->setContentsMargins(0, 0, 0, 0);
        LE_TD_OUT_OFF_0 = new QLineEdit(gridLayoutWidget_10);
        LE_TD_OUT_OFF_0->setObjectName(QString::fromUtf8("LE_TD_OUT_OFF_0"));
        LE_TD_OUT_OFF_0->setMaximumSize(QSize(30, 20));

        gridLayout_10->addWidget(LE_TD_OUT_OFF_0, 1, 2, 1, 1);

        label_97 = new QLabel(gridLayoutWidget_10);
        label_97->setObjectName(QString::fromUtf8("label_97"));
        label_97->setMinimumSize(QSize(40, 0));
        label_97->setMaximumSize(QSize(16777215, 20));
        label_97->setAlignment(Qt::AlignCenter);

        gridLayout_10->addWidget(label_97, 0, 2, 1, 1);

        label_83 = new QLabel(gridLayoutWidget_10);
        label_83->setObjectName(QString::fromUtf8("label_83"));
        label_83->setMinimumSize(QSize(40, 0));
        label_83->setMaximumSize(QSize(16777215, 20));
        label_83->setAlignment(Qt::AlignCenter);

        gridLayout_10->addWidget(label_83, 2, 0, 1, 1);

        LE_TD_OUT_ON_0 = new QLineEdit(gridLayoutWidget_10);
        LE_TD_OUT_ON_0->setObjectName(QString::fromUtf8("LE_TD_OUT_ON_0"));
        LE_TD_OUT_ON_0->setMaximumSize(QSize(30, 20));

        gridLayout_10->addWidget(LE_TD_OUT_ON_0, 1, 1, 1, 1);

        label_94 = new QLabel(gridLayoutWidget_10);
        label_94->setObjectName(QString::fromUtf8("label_94"));
        label_94->setMinimumSize(QSize(40, 0));
        label_94->setMaximumSize(QSize(16777215, 20));
        label_94->setAlignment(Qt::AlignCenter);

        gridLayout_10->addWidget(label_94, 1, 0, 1, 1);

        LE_TD_OUT_OFF_1 = new QLineEdit(gridLayoutWidget_10);
        LE_TD_OUT_OFF_1->setObjectName(QString::fromUtf8("LE_TD_OUT_OFF_1"));
        LE_TD_OUT_OFF_1->setMaximumSize(QSize(30, 20));

        gridLayout_10->addWidget(LE_TD_OUT_OFF_1, 2, 2, 1, 1);

        label_98 = new QLabel(gridLayoutWidget_10);
        label_98->setObjectName(QString::fromUtf8("label_98"));
        label_98->setMinimumSize(QSize(40, 0));
        label_98->setMaximumSize(QSize(16777215, 20));
        label_98->setAlignment(Qt::AlignCenter);

        gridLayout_10->addWidget(label_98, 0, 1, 1, 1);

        LE_TD_OUT_ON_1 = new QLineEdit(gridLayoutWidget_10);
        LE_TD_OUT_ON_1->setObjectName(QString::fromUtf8("LE_TD_OUT_ON_1"));
        LE_TD_OUT_ON_1->setMaximumSize(QSize(30, 20));

        gridLayout_10->addWidget(LE_TD_OUT_ON_1, 2, 1, 1, 1);

        label_84 = new QLabel(FRAME_STATUS);
        label_84->setObjectName(QString::fromUtf8("label_84"));
        label_84->setGeometry(QRect(110, 10, 91, 16));
        label_84->setFont(font10);
        label_84->setAlignment(Qt::AlignCenter);
        label_46 = new QLabel(FRAME_STATUS);
        label_46->setObjectName(QString::fromUtf8("label_46"));
        label_46->setGeometry(QRect(350, 243, 271, 16));
        label_46->setFont(font10);
        label_46->setAlignment(Qt::AlignCenter);
        gridLayoutWidget_5 = new QWidget(FRAME_STATUS);
        gridLayoutWidget_5->setObjectName(QString::fromUtf8("gridLayoutWidget_5"));
        gridLayoutWidget_5->setGeometry(QRect(347, 263, 271, 227));
        gridLayout_5 = new QGridLayout(gridLayoutWidget_5);
        gridLayout_5->setObjectName(QString::fromUtf8("gridLayout_5"));
        gridLayout_5->setContentsMargins(0, 0, 0, 0);
        LE_D_OUT_OFF_12 = new QLineEdit(gridLayoutWidget_5);
        LE_D_OUT_OFF_12->setObjectName(QString::fromUtf8("LE_D_OUT_OFF_12"));
        LE_D_OUT_OFF_12->setMinimumSize(QSize(30, 0));
        LE_D_OUT_OFF_12->setMaximumSize(QSize(30, 20));

        gridLayout_5->addWidget(LE_D_OUT_OFF_12, 5, 6, 1, 1);

        LE_D_OUT_ON_9 = new QLineEdit(gridLayoutWidget_5);
        LE_D_OUT_ON_9->setObjectName(QString::fromUtf8("LE_D_OUT_ON_9"));
        LE_D_OUT_ON_9->setMinimumSize(QSize(30, 0));
        LE_D_OUT_ON_9->setMaximumSize(QSize(30, 20));

        gridLayout_5->addWidget(LE_D_OUT_ON_9, 2, 5, 1, 1);

        horizontalSpacer_2 = new QSpacerItem(20, 20, QSizePolicy::Fixed, QSizePolicy::Minimum);

        gridLayout_5->addItem(horizontalSpacer_2, 2, 3, 1, 1);

        LE_D_OUT_OFF_9 = new QLineEdit(gridLayoutWidget_5);
        LE_D_OUT_OFF_9->setObjectName(QString::fromUtf8("LE_D_OUT_OFF_9"));
        LE_D_OUT_OFF_9->setMinimumSize(QSize(30, 0));
        LE_D_OUT_OFF_9->setMaximumSize(QSize(30, 20));

        gridLayout_5->addWidget(LE_D_OUT_OFF_9, 2, 6, 1, 1);

        label_47 = new QLabel(gridLayoutWidget_5);
        label_47->setObjectName(QString::fromUtf8("label_47"));
        label_47->setMinimumSize(QSize(30, 0));
        label_47->setMaximumSize(QSize(30, 20));
        label_47->setAlignment(Qt::AlignCenter);

        gridLayout_5->addWidget(label_47, 4, 0, 1, 1);

        LE_D_OUT_OFF_0 = new QLineEdit(gridLayoutWidget_5);
        LE_D_OUT_OFF_0->setObjectName(QString::fromUtf8("LE_D_OUT_OFF_0"));
        LE_D_OUT_OFF_0->setMinimumSize(QSize(30, 0));
        LE_D_OUT_OFF_0->setMaximumSize(QSize(30, 20));

        gridLayout_5->addWidget(LE_D_OUT_OFF_0, 1, 2, 1, 1);

        label_48 = new QLabel(gridLayoutWidget_5);
        label_48->setObjectName(QString::fromUtf8("label_48"));
        label_48->setMinimumSize(QSize(30, 0));
        label_48->setMaximumSize(QSize(30, 20));
        label_48->setAlignment(Qt::AlignCenter);

        gridLayout_5->addWidget(label_48, 5, 4, 1, 1);

        LE_D_OUT_ON_15 = new QLineEdit(gridLayoutWidget_5);
        LE_D_OUT_ON_15->setObjectName(QString::fromUtf8("LE_D_OUT_ON_15"));
        LE_D_OUT_ON_15->setMinimumSize(QSize(30, 0));
        LE_D_OUT_ON_15->setMaximumSize(QSize(30, 20));

        gridLayout_5->addWidget(LE_D_OUT_ON_15, 8, 5, 4, 1);

        label_49 = new QLabel(gridLayoutWidget_5);
        label_49->setObjectName(QString::fromUtf8("label_49"));
        label_49->setMinimumSize(QSize(30, 0));
        label_49->setMaximumSize(QSize(30, 20));
        label_49->setAlignment(Qt::AlignCenter);

        gridLayout_5->addWidget(label_49, 7, 0, 1, 1);

        LE_D_OUT_OFF_5 = new QLineEdit(gridLayoutWidget_5);
        LE_D_OUT_OFF_5->setObjectName(QString::fromUtf8("LE_D_OUT_OFF_5"));
        LE_D_OUT_OFF_5->setMinimumSize(QSize(30, 0));
        LE_D_OUT_OFF_5->setMaximumSize(QSize(30, 20));

        gridLayout_5->addWidget(LE_D_OUT_OFF_5, 6, 2, 1, 1);

        LE_D_OUT_ON_10 = new QLineEdit(gridLayoutWidget_5);
        LE_D_OUT_ON_10->setObjectName(QString::fromUtf8("LE_D_OUT_ON_10"));
        LE_D_OUT_ON_10->setMinimumSize(QSize(30, 0));
        LE_D_OUT_ON_10->setMaximumSize(QSize(30, 20));

        gridLayout_5->addWidget(LE_D_OUT_ON_10, 3, 5, 1, 1);

        label_50 = new QLabel(gridLayoutWidget_5);
        label_50->setObjectName(QString::fromUtf8("label_50"));
        label_50->setMinimumSize(QSize(30, 0));
        label_50->setMaximumSize(QSize(30, 20));
        label_50->setAlignment(Qt::AlignCenter);

        gridLayout_5->addWidget(label_50, 2, 0, 1, 1);

        label_51 = new QLabel(gridLayoutWidget_5);
        label_51->setObjectName(QString::fromUtf8("label_51"));
        label_51->setMinimumSize(QSize(30, 0));
        label_51->setMaximumSize(QSize(30, 20));
        label_51->setAlignment(Qt::AlignCenter);

        gridLayout_5->addWidget(label_51, 6, 4, 1, 1);

        LE_D_OUT_ON_2 = new QLineEdit(gridLayoutWidget_5);
        LE_D_OUT_ON_2->setObjectName(QString::fromUtf8("LE_D_OUT_ON_2"));
        LE_D_OUT_ON_2->setMinimumSize(QSize(30, 0));
        LE_D_OUT_ON_2->setMaximumSize(QSize(30, 20));

        gridLayout_5->addWidget(LE_D_OUT_ON_2, 3, 1, 1, 1);

        LE_D_OUT_ON_5 = new QLineEdit(gridLayoutWidget_5);
        LE_D_OUT_ON_5->setObjectName(QString::fromUtf8("LE_D_OUT_ON_5"));
        LE_D_OUT_ON_5->setMinimumSize(QSize(30, 0));
        LE_D_OUT_ON_5->setMaximumSize(QSize(30, 20));

        gridLayout_5->addWidget(LE_D_OUT_ON_5, 6, 1, 1, 1);

        LE_D_OUT_OFF_2 = new QLineEdit(gridLayoutWidget_5);
        LE_D_OUT_OFF_2->setObjectName(QString::fromUtf8("LE_D_OUT_OFF_2"));
        LE_D_OUT_OFF_2->setMinimumSize(QSize(30, 0));
        LE_D_OUT_OFF_2->setMaximumSize(QSize(30, 20));

        gridLayout_5->addWidget(LE_D_OUT_OFF_2, 3, 2, 1, 1);

        label_52 = new QLabel(gridLayoutWidget_5);
        label_52->setObjectName(QString::fromUtf8("label_52"));
        label_52->setMinimumSize(QSize(30, 0));
        label_52->setMaximumSize(QSize(30, 20));
        label_52->setAlignment(Qt::AlignCenter);

        gridLayout_5->addWidget(label_52, 7, 4, 1, 1);

        label_53 = new QLabel(gridLayoutWidget_5);
        label_53->setObjectName(QString::fromUtf8("label_53"));
        label_53->setMinimumSize(QSize(30, 0));
        label_53->setMaximumSize(QSize(30, 20));
        label_53->setAlignment(Qt::AlignCenter);

        gridLayout_5->addWidget(label_53, 5, 0, 1, 1);

        LE_D_OUT_ON_12 = new QLineEdit(gridLayoutWidget_5);
        LE_D_OUT_ON_12->setObjectName(QString::fromUtf8("LE_D_OUT_ON_12"));
        LE_D_OUT_ON_12->setMinimumSize(QSize(30, 0));
        LE_D_OUT_ON_12->setMaximumSize(QSize(30, 20));

        gridLayout_5->addWidget(LE_D_OUT_ON_12, 5, 5, 1, 1);

        LE_D_OUT_ON_3 = new QLineEdit(gridLayoutWidget_5);
        LE_D_OUT_ON_3->setObjectName(QString::fromUtf8("LE_D_OUT_ON_3"));
        LE_D_OUT_ON_3->setMinimumSize(QSize(30, 0));
        LE_D_OUT_ON_3->setMaximumSize(QSize(30, 20));

        gridLayout_5->addWidget(LE_D_OUT_ON_3, 4, 1, 1, 1);

        LE_D_OUT_OFF_11 = new QLineEdit(gridLayoutWidget_5);
        LE_D_OUT_OFF_11->setObjectName(QString::fromUtf8("LE_D_OUT_OFF_11"));
        LE_D_OUT_OFF_11->setMinimumSize(QSize(30, 0));
        LE_D_OUT_OFF_11->setMaximumSize(QSize(30, 20));

        gridLayout_5->addWidget(LE_D_OUT_OFF_11, 4, 6, 1, 1);

        label_54 = new QLabel(gridLayoutWidget_5);
        label_54->setObjectName(QString::fromUtf8("label_54"));
        label_54->setMinimumSize(QSize(30, 0));
        label_54->setMaximumSize(QSize(30, 20));
        label_54->setAlignment(Qt::AlignCenter);

        gridLayout_5->addWidget(label_54, 3, 0, 1, 1);

        LE_D_OUT_ON_1 = new QLineEdit(gridLayoutWidget_5);
        LE_D_OUT_ON_1->setObjectName(QString::fromUtf8("LE_D_OUT_ON_1"));
        LE_D_OUT_ON_1->setMinimumSize(QSize(30, 0));
        LE_D_OUT_ON_1->setMaximumSize(QSize(30, 20));

        gridLayout_5->addWidget(LE_D_OUT_ON_1, 2, 1, 1, 1);

        LE_D_OUT_OFF_15 = new QLineEdit(gridLayoutWidget_5);
        LE_D_OUT_OFF_15->setObjectName(QString::fromUtf8("LE_D_OUT_OFF_15"));
        LE_D_OUT_OFF_15->setMinimumSize(QSize(30, 0));
        LE_D_OUT_OFF_15->setMaximumSize(QSize(30, 20));

        gridLayout_5->addWidget(LE_D_OUT_OFF_15, 8, 6, 4, 1);

        LE_D_OUT_OFF_8 = new QLineEdit(gridLayoutWidget_5);
        LE_D_OUT_OFF_8->setObjectName(QString::fromUtf8("LE_D_OUT_OFF_8"));
        LE_D_OUT_OFF_8->setMinimumSize(QSize(30, 0));
        LE_D_OUT_OFF_8->setMaximumSize(QSize(30, 20));

        gridLayout_5->addWidget(LE_D_OUT_OFF_8, 1, 6, 1, 1);

        LE_D_OUT_ON_11 = new QLineEdit(gridLayoutWidget_5);
        LE_D_OUT_ON_11->setObjectName(QString::fromUtf8("LE_D_OUT_ON_11"));
        LE_D_OUT_ON_11->setMinimumSize(QSize(30, 0));
        LE_D_OUT_ON_11->setMaximumSize(QSize(30, 20));

        gridLayout_5->addWidget(LE_D_OUT_ON_11, 4, 5, 1, 1);

        LE_D_OUT_ON_6 = new QLineEdit(gridLayoutWidget_5);
        LE_D_OUT_ON_6->setObjectName(QString::fromUtf8("LE_D_OUT_ON_6"));
        LE_D_OUT_ON_6->setMinimumSize(QSize(30, 0));
        LE_D_OUT_ON_6->setMaximumSize(QSize(30, 20));

        gridLayout_5->addWidget(LE_D_OUT_ON_6, 7, 1, 1, 1);

        LE_D_OUT_OFF_1 = new QLineEdit(gridLayoutWidget_5);
        LE_D_OUT_OFF_1->setObjectName(QString::fromUtf8("LE_D_OUT_OFF_1"));
        LE_D_OUT_OFF_1->setMinimumSize(QSize(30, 0));
        LE_D_OUT_OFF_1->setMaximumSize(QSize(30, 20));

        gridLayout_5->addWidget(LE_D_OUT_OFF_1, 2, 2, 1, 1);

        LE_D_OUT_OFF_3 = new QLineEdit(gridLayoutWidget_5);
        LE_D_OUT_OFF_3->setObjectName(QString::fromUtf8("LE_D_OUT_OFF_3"));
        LE_D_OUT_OFF_3->setMinimumSize(QSize(30, 0));
        LE_D_OUT_OFF_3->setMaximumSize(QSize(30, 20));

        gridLayout_5->addWidget(LE_D_OUT_OFF_3, 4, 2, 1, 1);

        LE_D_OUT_OFF_13 = new QLineEdit(gridLayoutWidget_5);
        LE_D_OUT_OFF_13->setObjectName(QString::fromUtf8("LE_D_OUT_OFF_13"));
        LE_D_OUT_OFF_13->setMinimumSize(QSize(30, 0));
        LE_D_OUT_OFF_13->setMaximumSize(QSize(30, 20));

        gridLayout_5->addWidget(LE_D_OUT_OFF_13, 6, 6, 1, 1);

        label_55 = new QLabel(gridLayoutWidget_5);
        label_55->setObjectName(QString::fromUtf8("label_55"));
        label_55->setMinimumSize(QSize(30, 0));
        label_55->setMaximumSize(QSize(30, 20));
        label_55->setAlignment(Qt::AlignCenter);

        gridLayout_5->addWidget(label_55, 2, 4, 1, 1);

        label_56 = new QLabel(gridLayoutWidget_5);
        label_56->setObjectName(QString::fromUtf8("label_56"));
        label_56->setMinimumSize(QSize(30, 0));
        label_56->setMaximumSize(QSize(30, 20));
        label_56->setAlignment(Qt::AlignCenter);

        gridLayout_5->addWidget(label_56, 6, 0, 1, 1);

        LE_D_OUT_ON_4 = new QLineEdit(gridLayoutWidget_5);
        LE_D_OUT_ON_4->setObjectName(QString::fromUtf8("LE_D_OUT_ON_4"));
        LE_D_OUT_ON_4->setMinimumSize(QSize(30, 0));
        LE_D_OUT_ON_4->setMaximumSize(QSize(30, 20));

        gridLayout_5->addWidget(LE_D_OUT_ON_4, 5, 1, 1, 1);

        label_57 = new QLabel(gridLayoutWidget_5);
        label_57->setObjectName(QString::fromUtf8("label_57"));
        label_57->setMinimumSize(QSize(30, 0));
        label_57->setMaximumSize(QSize(30, 20));
        label_57->setAlignment(Qt::AlignCenter);

        gridLayout_5->addWidget(label_57, 1, 4, 1, 1);

        LE_D_OUT_OFF_14 = new QLineEdit(gridLayoutWidget_5);
        LE_D_OUT_OFF_14->setObjectName(QString::fromUtf8("LE_D_OUT_OFF_14"));
        LE_D_OUT_OFF_14->setMinimumSize(QSize(30, 0));
        LE_D_OUT_OFF_14->setMaximumSize(QSize(30, 20));

        gridLayout_5->addWidget(LE_D_OUT_OFF_14, 7, 6, 1, 1);

        LE_D_OUT_ON_7 = new QLineEdit(gridLayoutWidget_5);
        LE_D_OUT_ON_7->setObjectName(QString::fromUtf8("LE_D_OUT_ON_7"));
        LE_D_OUT_ON_7->setMinimumSize(QSize(30, 0));
        LE_D_OUT_ON_7->setMaximumSize(QSize(30, 20));

        gridLayout_5->addWidget(LE_D_OUT_ON_7, 8, 1, 4, 1);

        label_58 = new QLabel(gridLayoutWidget_5);
        label_58->setObjectName(QString::fromUtf8("label_58"));
        label_58->setMinimumSize(QSize(30, 0));
        label_58->setMaximumSize(QSize(30, 20));
        label_58->setAlignment(Qt::AlignCenter);

        gridLayout_5->addWidget(label_58, 3, 4, 1, 1);

        label_59 = new QLabel(gridLayoutWidget_5);
        label_59->setObjectName(QString::fromUtf8("label_59"));
        label_59->setMinimumSize(QSize(30, 0));
        label_59->setMaximumSize(QSize(30, 20));
        label_59->setAlignment(Qt::AlignCenter);

        gridLayout_5->addWidget(label_59, 8, 4, 4, 1);

        LE_D_OUT_ON_13 = new QLineEdit(gridLayoutWidget_5);
        LE_D_OUT_ON_13->setObjectName(QString::fromUtf8("LE_D_OUT_ON_13"));
        LE_D_OUT_ON_13->setMinimumSize(QSize(30, 0));
        LE_D_OUT_ON_13->setMaximumSize(QSize(30, 20));

        gridLayout_5->addWidget(LE_D_OUT_ON_13, 6, 5, 1, 1);

        LE_D_OUT_ON_14 = new QLineEdit(gridLayoutWidget_5);
        LE_D_OUT_ON_14->setObjectName(QString::fromUtf8("LE_D_OUT_ON_14"));
        LE_D_OUT_ON_14->setMinimumSize(QSize(30, 0));
        LE_D_OUT_ON_14->setMaximumSize(QSize(30, 20));

        gridLayout_5->addWidget(LE_D_OUT_ON_14, 7, 5, 1, 1);

        label_60 = new QLabel(gridLayoutWidget_5);
        label_60->setObjectName(QString::fromUtf8("label_60"));
        label_60->setMinimumSize(QSize(30, 0));
        label_60->setMaximumSize(QSize(30, 20));
        label_60->setAlignment(Qt::AlignCenter);

        gridLayout_5->addWidget(label_60, 4, 4, 1, 1);

        LE_D_OUT_ON_8 = new QLineEdit(gridLayoutWidget_5);
        LE_D_OUT_ON_8->setObjectName(QString::fromUtf8("LE_D_OUT_ON_8"));
        LE_D_OUT_ON_8->setMinimumSize(QSize(30, 0));
        LE_D_OUT_ON_8->setMaximumSize(QSize(30, 20));

        gridLayout_5->addWidget(LE_D_OUT_ON_8, 1, 5, 1, 1);

        LE_D_OUT_OFF_7 = new QLineEdit(gridLayoutWidget_5);
        LE_D_OUT_OFF_7->setObjectName(QString::fromUtf8("LE_D_OUT_OFF_7"));
        LE_D_OUT_OFF_7->setMinimumSize(QSize(30, 0));
        LE_D_OUT_OFF_7->setMaximumSize(QSize(30, 20));

        gridLayout_5->addWidget(LE_D_OUT_OFF_7, 8, 2, 4, 1);

        label_61 = new QLabel(gridLayoutWidget_5);
        label_61->setObjectName(QString::fromUtf8("label_61"));
        label_61->setMinimumSize(QSize(30, 0));
        label_61->setMaximumSize(QSize(30, 20));
        label_61->setAlignment(Qt::AlignCenter);

        gridLayout_5->addWidget(label_61, 1, 0, 1, 1);

        LE_D_OUT_OFF_6 = new QLineEdit(gridLayoutWidget_5);
        LE_D_OUT_OFF_6->setObjectName(QString::fromUtf8("LE_D_OUT_OFF_6"));
        LE_D_OUT_OFF_6->setMinimumSize(QSize(30, 0));
        LE_D_OUT_OFF_6->setMaximumSize(QSize(30, 20));

        gridLayout_5->addWidget(LE_D_OUT_OFF_6, 7, 2, 1, 1);

        LE_D_OUT_OFF_4 = new QLineEdit(gridLayoutWidget_5);
        LE_D_OUT_OFF_4->setObjectName(QString::fromUtf8("LE_D_OUT_OFF_4"));
        LE_D_OUT_OFF_4->setMinimumSize(QSize(30, 0));
        LE_D_OUT_OFF_4->setMaximumSize(QSize(30, 20));

        gridLayout_5->addWidget(LE_D_OUT_OFF_4, 5, 2, 1, 1);

        LE_D_OUT_ON_0 = new QLineEdit(gridLayoutWidget_5);
        LE_D_OUT_ON_0->setObjectName(QString::fromUtf8("LE_D_OUT_ON_0"));
        LE_D_OUT_ON_0->setMinimumSize(QSize(30, 0));
        LE_D_OUT_ON_0->setMaximumSize(QSize(30, 20));

        gridLayout_5->addWidget(LE_D_OUT_ON_0, 1, 1, 1, 1);

        label_62 = new QLabel(gridLayoutWidget_5);
        label_62->setObjectName(QString::fromUtf8("label_62"));
        label_62->setMinimumSize(QSize(30, 0));
        label_62->setMaximumSize(QSize(30, 20));
        label_62->setAlignment(Qt::AlignCenter);

        gridLayout_5->addWidget(label_62, 8, 0, 4, 1);

        LE_D_OUT_OFF_10 = new QLineEdit(gridLayoutWidget_5);
        LE_D_OUT_OFF_10->setObjectName(QString::fromUtf8("LE_D_OUT_OFF_10"));
        LE_D_OUT_OFF_10->setMinimumSize(QSize(30, 0));
        LE_D_OUT_OFF_10->setMaximumSize(QSize(30, 20));

        gridLayout_5->addWidget(LE_D_OUT_OFF_10, 3, 6, 1, 1);

        label_63 = new QLabel(gridLayoutWidget_5);
        label_63->setObjectName(QString::fromUtf8("label_63"));
        label_63->setMinimumSize(QSize(30, 0));
        label_63->setMaximumSize(QSize(30, 20));
        label_63->setAlignment(Qt::AlignCenter);

        gridLayout_5->addWidget(label_63, 0, 1, 1, 1);

        label_64 = new QLabel(gridLayoutWidget_5);
        label_64->setObjectName(QString::fromUtf8("label_64"));
        label_64->setMinimumSize(QSize(30, 0));
        label_64->setMaximumSize(QSize(30, 20));
        label_64->setAlignment(Qt::AlignCenter);

        gridLayout_5->addWidget(label_64, 0, 2, 1, 1);

        label_65 = new QLabel(gridLayoutWidget_5);
        label_65->setObjectName(QString::fromUtf8("label_65"));
        label_65->setMinimumSize(QSize(30, 0));
        label_65->setMaximumSize(QSize(30, 20));
        label_65->setAlignment(Qt::AlignCenter);

        gridLayout_5->addWidget(label_65, 0, 5, 1, 1);

        label_66 = new QLabel(gridLayoutWidget_5);
        label_66->setObjectName(QString::fromUtf8("label_66"));
        label_66->setMinimumSize(QSize(30, 0));
        label_66->setMaximumSize(QSize(30, 20));
        label_66->setAlignment(Qt::AlignCenter);

        gridLayout_5->addWidget(label_66, 0, 6, 1, 1);

        label_99 = new QLabel(FRAME_STATUS);
        label_99->setObjectName(QString::fromUtf8("label_99"));
        label_99->setGeometry(QRect(160, 601, 131, 16));
        label_99->setFont(font10);
        label_99->setAlignment(Qt::AlignCenter);
        LE_ROBOT_STATUS_SOS = new QLineEdit(FRAME_STATUS);
        LE_ROBOT_STATUS_SOS->setObjectName(QString::fromUtf8("LE_ROBOT_STATUS_SOS"));
        LE_ROBOT_STATUS_SOS->setGeometry(QRect(110, 180, 91, 31));
        LE_ROBOT_STATUS_SOS->setAlignment(Qt::AlignCenter);
        LE_ROBOT_STATE_IDLE = new QLineEdit(FRAME_STATUS);
        LE_ROBOT_STATE_IDLE->setObjectName(QString::fromUtf8("LE_ROBOT_STATE_IDLE"));
        LE_ROBOT_STATE_IDLE->setGeometry(QRect(10, 120, 81, 31));
        LE_ROBOT_STATE_IDLE->setAlignment(Qt::AlignCenter);
        label_67 = new QLabel(FRAME_STATUS);
        label_67->setObjectName(QString::fromUtf8("label_67"));
        label_67->setGeometry(QRect(10, 243, 271, 16));
        label_67->setFont(font10);
        label_67->setAlignment(Qt::AlignCenter);
        label_100 = new QLabel(FRAME_STATUS);
        label_100->setObjectName(QString::fromUtf8("label_100"));
        label_100->setGeometry(QRect(342, 601, 131, 16));
        label_100->setFont(font10);
        label_100->setAlignment(Qt::AlignCenter);
        label_68 = new QLabel(FRAME_STATUS);
        label_68->setObjectName(QString::fromUtf8("label_68"));
        label_68->setGeometry(QRect(10, 10, 81, 16));
        label_68->setFont(font10);
        label_68->setAlignment(Qt::AlignCenter);
        gridLayoutWidget_3 = new QWidget(FRAME_STATUS);
        gridLayoutWidget_3->setObjectName(QString::fromUtf8("gridLayoutWidget_3"));
        gridLayoutWidget_3->setGeometry(QRect(500, 30, 121, 206));
        gridLayout_3 = new QGridLayout(gridLayoutWidget_3);
        gridLayout_3->setObjectName(QString::fromUtf8("gridLayout_3"));
        gridLayout_3->setContentsMargins(0, 0, 0, 0);
        label_69 = new QLabel(gridLayoutWidget_3);
        label_69->setObjectName(QString::fromUtf8("label_69"));
        label_69->setMinimumSize(QSize(40, 0));
        label_69->setAlignment(Qt::AlignCenter);

        gridLayout_3->addWidget(label_69, 4, 0, 1, 1);

        label_70 = new QLabel(gridLayoutWidget_3);
        label_70->setObjectName(QString::fromUtf8("label_70"));
        label_70->setMinimumSize(QSize(40, 0));
        label_70->setAlignment(Qt::AlignCenter);

        gridLayout_3->addWidget(label_70, 5, 0, 1, 1);

        label_72 = new QLabel(gridLayoutWidget_3);
        label_72->setObjectName(QString::fromUtf8("label_72"));
        label_72->setMinimumSize(QSize(40, 0));
        label_72->setAlignment(Qt::AlignCenter);

        gridLayout_3->addWidget(label_72, 0, 0, 1, 1);

        label_73 = new QLabel(gridLayoutWidget_3);
        label_73->setObjectName(QString::fromUtf8("label_73"));
        label_73->setMinimumSize(QSize(40, 0));
        label_73->setAlignment(Qt::AlignCenter);

        gridLayout_3->addWidget(label_73, 1, 0, 1, 1);

        label_75 = new QLabel(gridLayoutWidget_3);
        label_75->setObjectName(QString::fromUtf8("label_75"));
        label_75->setMinimumSize(QSize(40, 0));
        label_75->setAlignment(Qt::AlignCenter);

        gridLayout_3->addWidget(label_75, 3, 0, 1, 1);

        label_76 = new QLabel(gridLayoutWidget_3);
        label_76->setObjectName(QString::fromUtf8("label_76"));
        label_76->setMinimumSize(QSize(40, 0));
        label_76->setAlignment(Qt::AlignCenter);

        gridLayout_3->addWidget(label_76, 2, 0, 1, 1);

        LE_TCP_REF_X = new QLineEdit(gridLayoutWidget_3);
        LE_TCP_REF_X->setObjectName(QString::fromUtf8("LE_TCP_REF_X"));

        gridLayout_3->addWidget(LE_TCP_REF_X, 0, 1, 1, 1);

        LE_TCP_REF_Y = new QLineEdit(gridLayoutWidget_3);
        LE_TCP_REF_Y->setObjectName(QString::fromUtf8("LE_TCP_REF_Y"));

        gridLayout_3->addWidget(LE_TCP_REF_Y, 1, 1, 1, 1);

        LE_TCP_REF_Z = new QLineEdit(gridLayoutWidget_3);
        LE_TCP_REF_Z->setObjectName(QString::fromUtf8("LE_TCP_REF_Z"));

        gridLayout_3->addWidget(LE_TCP_REF_Z, 2, 1, 1, 1);

        LE_TCP_REF_RX = new QLineEdit(gridLayoutWidget_3);
        LE_TCP_REF_RX->setObjectName(QString::fromUtf8("LE_TCP_REF_RX"));

        gridLayout_3->addWidget(LE_TCP_REF_RX, 3, 1, 1, 1);

        LE_TCP_REF_RY = new QLineEdit(gridLayoutWidget_3);
        LE_TCP_REF_RY->setObjectName(QString::fromUtf8("LE_TCP_REF_RY"));

        gridLayout_3->addWidget(LE_TCP_REF_RY, 4, 1, 1, 1);

        LE_TCP_REF_RZ = new QLineEdit(gridLayoutWidget_3);
        LE_TCP_REF_RZ->setObjectName(QString::fromUtf8("LE_TCP_REF_RZ"));

        gridLayout_3->addWidget(LE_TCP_REF_RZ, 5, 1, 1, 1);

        LE_ROBOT_STATUS_EMS = new QLineEdit(FRAME_STATUS);
        LE_ROBOT_STATUS_EMS->setObjectName(QString::fromUtf8("LE_ROBOT_STATUS_EMS"));
        LE_ROBOT_STATUS_EMS->setGeometry(QRect(110, 150, 91, 31));
        LE_ROBOT_STATUS_EMS->setAlignment(Qt::AlignCenter);
        LE_ROBOT_STATE_MOVING = new QLineEdit(FRAME_STATUS);
        LE_ROBOT_STATE_MOVING->setObjectName(QString::fromUtf8("LE_ROBOT_STATE_MOVING"));
        LE_ROBOT_STATE_MOVING->setGeometry(QRect(10, 150, 81, 31));
        LE_ROBOT_STATE_MOVING->setAlignment(Qt::AlignCenter);
        LE_ROBOT_STATUS_PAUSED = new QLineEdit(FRAME_STATUS);
        LE_ROBOT_STATUS_PAUSED->setObjectName(QString::fromUtf8("LE_ROBOT_STATUS_PAUSED"));
        LE_ROBOT_STATUS_PAUSED->setGeometry(QRect(110, 120, 91, 31));
        LE_ROBOT_STATUS_PAUSED->setAlignment(Qt::AlignCenter);
        label_77 = new QLabel(FRAME_STATUS);
        label_77->setObjectName(QString::fromUtf8("label_77"));
        label_77->setGeometry(QRect(0, 501, 281, 16));
        label_77->setFont(font10);
        label_77->setAlignment(Qt::AlignCenter);
        label_78 = new QLabel(FRAME_STATUS);
        label_78->setObjectName(QString::fromUtf8("label_78"));
        label_78->setGeometry(QRect(500, 10, 121, 16));
        label_78->setFont(font9);
        label_78->setAlignment(Qt::AlignCenter);
        LE_ROBOT_STATUS_SELF_COLLISION = new QLineEdit(FRAME_STATUS);
        LE_ROBOT_STATUS_SELF_COLLISION->setObjectName(QString::fromUtf8("LE_ROBOT_STATUS_SELF_COLLISION"));
        LE_ROBOT_STATUS_SELF_COLLISION->setGeometry(QRect(110, 90, 91, 31));
        LE_ROBOT_STATUS_SELF_COLLISION->setAlignment(Qt::AlignCenter);
        LE_PG_MODE_REAL = new QLineEdit(FRAME_STATUS);
        LE_PG_MODE_REAL->setObjectName(QString::fromUtf8("LE_PG_MODE_REAL"));
        LE_PG_MODE_REAL->setGeometry(QRect(10, 30, 81, 31));
        LE_PG_MODE_REAL->setAlignment(Qt::AlignCenter);
        gridLayoutWidget_6 = new QWidget(FRAME_STATUS);
        gridLayoutWidget_6->setObjectName(QString::fromUtf8("gridLayoutWidget_6"));
        gridLayoutWidget_6->setGeometry(QRect(10, 521, 271, 66));
        gridLayout_6 = new QGridLayout(gridLayoutWidget_6);
        gridLayout_6->setObjectName(QString::fromUtf8("gridLayout_6"));
        gridLayout_6->setContentsMargins(0, 0, 0, 0);
        label_79 = new QLabel(gridLayoutWidget_6);
        label_79->setObjectName(QString::fromUtf8("label_79"));
        label_79->setMinimumSize(QSize(40, 0));
        label_79->setAlignment(Qt::AlignCenter);

        gridLayout_6->addWidget(label_79, 0, 3, 1, 1);

        label_80 = new QLabel(gridLayoutWidget_6);
        label_80->setObjectName(QString::fromUtf8("label_80"));
        label_80->setMinimumSize(QSize(40, 0));
        label_80->setAlignment(Qt::AlignCenter);

        gridLayout_6->addWidget(label_80, 1, 0, 1, 1);

        horizontalSpacer_3 = new QSpacerItem(20, 20, QSizePolicy::Fixed, QSizePolicy::Minimum);

        gridLayout_6->addItem(horizontalSpacer_3, 0, 2, 1, 1);

        label_81 = new QLabel(gridLayoutWidget_6);
        label_81->setObjectName(QString::fromUtf8("label_81"));
        label_81->setMinimumSize(QSize(40, 0));
        label_81->setAlignment(Qt::AlignCenter);

        gridLayout_6->addWidget(label_81, 1, 3, 1, 1);

        LE_A_IN_1 = new QLineEdit(gridLayoutWidget_6);
        LE_A_IN_1->setObjectName(QString::fromUtf8("LE_A_IN_1"));

        gridLayout_6->addWidget(LE_A_IN_1, 1, 1, 1, 1);

        label_85 = new QLabel(gridLayoutWidget_6);
        label_85->setObjectName(QString::fromUtf8("label_85"));
        label_85->setMinimumSize(QSize(40, 0));
        label_85->setAlignment(Qt::AlignCenter);

        gridLayout_6->addWidget(label_85, 0, 0, 1, 1);

        LE_A_IN_0 = new QLineEdit(gridLayoutWidget_6);
        LE_A_IN_0->setObjectName(QString::fromUtf8("LE_A_IN_0"));

        gridLayout_6->addWidget(LE_A_IN_0, 0, 1, 1, 1);

        LE_A_IN_2 = new QLineEdit(gridLayoutWidget_6);
        LE_A_IN_2->setObjectName(QString::fromUtf8("LE_A_IN_2"));

        gridLayout_6->addWidget(LE_A_IN_2, 0, 4, 1, 1);

        LE_A_IN_3 = new QLineEdit(gridLayoutWidget_6);
        LE_A_IN_3->setObjectName(QString::fromUtf8("LE_A_IN_3"));

        gridLayout_6->addWidget(LE_A_IN_3, 1, 4, 1, 1);

        LE_ROBOT_STATUS_EXT_COLLISION = new QLineEdit(FRAME_STATUS);
        LE_ROBOT_STATUS_EXT_COLLISION->setObjectName(QString::fromUtf8("LE_ROBOT_STATUS_EXT_COLLISION"));
        LE_ROBOT_STATUS_EXT_COLLISION->setGeometry(QRect(110, 60, 91, 31));
        LE_ROBOT_STATUS_EXT_COLLISION->setAlignment(Qt::AlignCenter);
        label_86 = new QLabel(FRAME_STATUS);
        label_86->setObjectName(QString::fromUtf8("label_86"));
        label_86->setGeometry(QRect(220, 10, 121, 16));
        label_86->setFont(font9);
        label_86->setAlignment(Qt::AlignCenter);
        label_87 = new QLabel(FRAME_STATUS);
        label_87->setObjectName(QString::fromUtf8("label_87"));
        label_87->setGeometry(QRect(350, 501, 271, 16));
        label_87->setFont(font10);
        label_87->setAlignment(Qt::AlignCenter);
        label_88 = new QLabel(FRAME_STATUS);
        label_88->setObjectName(QString::fromUtf8("label_88"));
        label_88->setGeometry(QRect(500, 601, 121, 20));
        label_88->setFont(font10);
        label_88->setAlignment(Qt::AlignCenter);
        LE_ROBOT_STATUS_TEACHING = new QLineEdit(FRAME_STATUS);
        LE_ROBOT_STATUS_TEACHING->setObjectName(QString::fromUtf8("LE_ROBOT_STATUS_TEACHING"));
        LE_ROBOT_STATUS_TEACHING->setGeometry(QRect(110, 30, 91, 31));
        LE_ROBOT_STATUS_TEACHING->setAlignment(Qt::AlignCenter);
        gridLayoutWidget_7 = new QWidget(FRAME_STATUS);
        gridLayoutWidget_7->setObjectName(QString::fromUtf8("gridLayoutWidget_7"));
        gridLayoutWidget_7->setGeometry(QRect(350, 521, 271, 66));
        gridLayout_8 = new QGridLayout(gridLayoutWidget_7);
        gridLayout_8->setObjectName(QString::fromUtf8("gridLayout_8"));
        gridLayout_8->setContentsMargins(0, 0, 0, 0);
        label_89 = new QLabel(gridLayoutWidget_7);
        label_89->setObjectName(QString::fromUtf8("label_89"));
        label_89->setMinimumSize(QSize(40, 0));
        label_89->setAlignment(Qt::AlignCenter);

        gridLayout_8->addWidget(label_89, 0, 3, 1, 1);

        label_90 = new QLabel(gridLayoutWidget_7);
        label_90->setObjectName(QString::fromUtf8("label_90"));
        label_90->setMinimumSize(QSize(40, 0));
        label_90->setAlignment(Qt::AlignCenter);

        gridLayout_8->addWidget(label_90, 1, 0, 1, 1);

        horizontalSpacer_5 = new QSpacerItem(20, 20, QSizePolicy::Fixed, QSizePolicy::Minimum);

        gridLayout_8->addItem(horizontalSpacer_5, 0, 2, 1, 1);

        label_91 = new QLabel(gridLayoutWidget_7);
        label_91->setObjectName(QString::fromUtf8("label_91"));
        label_91->setMinimumSize(QSize(40, 0));
        label_91->setAlignment(Qt::AlignCenter);

        gridLayout_8->addWidget(label_91, 1, 3, 1, 1);

        LE_A_OUT_1 = new QLineEdit(gridLayoutWidget_7);
        LE_A_OUT_1->setObjectName(QString::fromUtf8("LE_A_OUT_1"));

        gridLayout_8->addWidget(LE_A_OUT_1, 1, 1, 1, 1);

        label_92 = new QLabel(gridLayoutWidget_7);
        label_92->setObjectName(QString::fromUtf8("label_92"));
        label_92->setMinimumSize(QSize(40, 0));
        label_92->setAlignment(Qt::AlignCenter);

        gridLayout_8->addWidget(label_92, 0, 0, 1, 1);

        LE_A_OUT_0 = new QLineEdit(gridLayoutWidget_7);
        LE_A_OUT_0->setObjectName(QString::fromUtf8("LE_A_OUT_0"));

        gridLayout_8->addWidget(LE_A_OUT_0, 0, 1, 1, 1);

        LE_A_OUT_2 = new QLineEdit(gridLayoutWidget_7);
        LE_A_OUT_2->setObjectName(QString::fromUtf8("LE_A_OUT_2"));

        gridLayout_8->addWidget(LE_A_OUT_2, 0, 4, 1, 1);

        LE_A_OUT_3 = new QLineEdit(gridLayoutWidget_7);
        LE_A_OUT_3->setObjectName(QString::fromUtf8("LE_A_OUT_3"));

        gridLayout_8->addWidget(LE_A_OUT_3, 1, 4, 1, 1);

        gridLayoutWidget_2 = new QWidget(FRAME_STATUS);
        gridLayoutWidget_2->setObjectName(QString::fromUtf8("gridLayoutWidget_2"));
        gridLayoutWidget_2->setGeometry(QRect(360, 30, 121, 206));
        gridLayout_2 = new QGridLayout(gridLayoutWidget_2);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        gridLayout_2->setContentsMargins(0, 0, 0, 0);
        label_102 = new QLabel(gridLayoutWidget_2);
        label_102->setObjectName(QString::fromUtf8("label_102"));
        label_102->setMinimumSize(QSize(40, 0));
        label_102->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(label_102, 4, 0, 1, 1);

        label_103 = new QLabel(gridLayoutWidget_2);
        label_103->setObjectName(QString::fromUtf8("label_103"));
        label_103->setMinimumSize(QSize(40, 0));
        label_103->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(label_103, 5, 0, 1, 1);

        label_104 = new QLabel(gridLayoutWidget_2);
        label_104->setObjectName(QString::fromUtf8("label_104"));
        label_104->setMinimumSize(QSize(40, 0));
        label_104->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(label_104, 0, 0, 1, 1);

        label_105 = new QLabel(gridLayoutWidget_2);
        label_105->setObjectName(QString::fromUtf8("label_105"));
        label_105->setMinimumSize(QSize(40, 0));
        label_105->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(label_105, 1, 0, 1, 1);

        label_106 = new QLabel(gridLayoutWidget_2);
        label_106->setObjectName(QString::fromUtf8("label_106"));
        label_106->setMinimumSize(QSize(40, 0));
        label_106->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(label_106, 3, 0, 1, 1);

        label_107 = new QLabel(gridLayoutWidget_2);
        label_107->setObjectName(QString::fromUtf8("label_107"));
        label_107->setMinimumSize(QSize(40, 0));
        label_107->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(label_107, 2, 0, 1, 1);

        LE_JNT_ENC_1 = new QLineEdit(gridLayoutWidget_2);
        LE_JNT_ENC_1->setObjectName(QString::fromUtf8("LE_JNT_ENC_1"));

        gridLayout_2->addWidget(LE_JNT_ENC_1, 0, 1, 1, 1);

        LE_JNT_ENC_2 = new QLineEdit(gridLayoutWidget_2);
        LE_JNT_ENC_2->setObjectName(QString::fromUtf8("LE_JNT_ENC_2"));

        gridLayout_2->addWidget(LE_JNT_ENC_2, 1, 1, 1, 1);

        LE_JNT_ENC_3 = new QLineEdit(gridLayoutWidget_2);
        LE_JNT_ENC_3->setObjectName(QString::fromUtf8("LE_JNT_ENC_3"));

        gridLayout_2->addWidget(LE_JNT_ENC_3, 2, 1, 1, 1);

        LE_JNT_ENC_4 = new QLineEdit(gridLayoutWidget_2);
        LE_JNT_ENC_4->setObjectName(QString::fromUtf8("LE_JNT_ENC_4"));

        gridLayout_2->addWidget(LE_JNT_ENC_4, 3, 1, 1, 1);

        LE_JNT_ENC_5 = new QLineEdit(gridLayoutWidget_2);
        LE_JNT_ENC_5->setObjectName(QString::fromUtf8("LE_JNT_ENC_5"));

        gridLayout_2->addWidget(LE_JNT_ENC_5, 4, 1, 1, 1);

        LE_JNT_ENC_6 = new QLineEdit(gridLayoutWidget_2);
        LE_JNT_ENC_6->setObjectName(QString::fromUtf8("LE_JNT_ENC_6"));

        gridLayout_2->addWidget(LE_JNT_ENC_6, 5, 1, 1, 1);

        gridLayoutWidget_11 = new QWidget(FRAME_STATUS);
        gridLayoutWidget_11->setObjectName(QString::fromUtf8("gridLayoutWidget_11"));
        gridLayoutWidget_11->setGeometry(QRect(10, 621, 179, 71));
        gridLayout_11 = new QGridLayout(gridLayoutWidget_11);
        gridLayout_11->setObjectName(QString::fromUtf8("gridLayout_11"));
        gridLayout_11->setContentsMargins(0, 0, 0, 0);
        label_108 = new QLabel(gridLayoutWidget_11);
        label_108->setObjectName(QString::fromUtf8("label_108"));
        label_108->setMinimumSize(QSize(40, 0));
        label_108->setAlignment(Qt::AlignCenter);

        gridLayout_11->addWidget(label_108, 0, 0, 1, 1);

        label_109 = new QLabel(gridLayoutWidget_11);
        label_109->setObjectName(QString::fromUtf8("label_109"));
        label_109->setMinimumSize(QSize(40, 0));
        label_109->setAlignment(Qt::AlignCenter);

        gridLayout_11->addWidget(label_109, 0, 1, 1, 1);

        label_110 = new QLabel(gridLayoutWidget_11);
        label_110->setObjectName(QString::fromUtf8("label_110"));
        label_110->setMinimumSize(QSize(40, 0));
        label_110->setAlignment(Qt::AlignCenter);

        gridLayout_11->addWidget(label_110, 0, 2, 1, 1);

        LE_TV_0 = new QLineEdit(gridLayoutWidget_11);
        LE_TV_0->setObjectName(QString::fromUtf8("LE_TV_0"));
        LE_TV_0->setMinimumSize(QSize(0, 35));

        gridLayout_11->addWidget(LE_TV_0, 1, 0, 1, 1);

        LE_TV_12 = new QLineEdit(gridLayoutWidget_11);
        LE_TV_12->setObjectName(QString::fromUtf8("LE_TV_12"));
        LE_TV_12->setMinimumSize(QSize(0, 35));

        gridLayout_11->addWidget(LE_TV_12, 1, 1, 1, 1);

        LE_TV_24 = new QLineEdit(gridLayoutWidget_11);
        LE_TV_24->setObjectName(QString::fromUtf8("LE_TV_24"));
        LE_TV_24->setMinimumSize(QSize(0, 35));

        gridLayout_11->addWidget(LE_TV_24, 1, 2, 1, 1);

        BTN_COLLISION_RESUME = new QPushButton(rb_5);
        BTN_COLLISION_RESUME->setObjectName(QString::fromUtf8("BTN_COLLISION_RESUME"));
        BTN_COLLISION_RESUME->setGeometry(QRect(990, 110, 101, 61));
        BTN_COLLISION_RESUME->setStyleSheet(QString::fromUtf8("background-color: rgb(138, 226, 52);"));
        BTN_MOTION_PAUSE = new QPushButton(rb_5);
        BTN_MOTION_PAUSE->setObjectName(QString::fromUtf8("BTN_MOTION_PAUSE"));
        BTN_MOTION_PAUSE->setGeometry(QRect(870, 40, 101, 61));
        BTN_MOTION_PAUSE->setStyleSheet(QString::fromUtf8("background-color: rgb(252, 233, 79);"));
        GB_NETWORK_2 = new QGroupBox(rb_5);
        GB_NETWORK_2->setObjectName(QString::fromUtf8("GB_NETWORK_2"));
        GB_NETWORK_2->setGeometry(QRect(550, 40, 101, 171));
        GB_NETWORK_2->setFont(font8);
        BTN_MODE_REAL = new QPushButton(GB_NETWORK_2);
        BTN_MODE_REAL->setObjectName(QString::fromUtf8("BTN_MODE_REAL"));
        BTN_MODE_REAL->setGeometry(QRect(10, 30, 81, 41));
        BTN_MODE_REAL->setMinimumSize(QSize(0, 40));
        BTN_MODE_SIMULATION = new QPushButton(GB_NETWORK_2);
        BTN_MODE_SIMULATION->setObjectName(QString::fromUtf8("BTN_MODE_SIMULATION"));
        BTN_MODE_SIMULATION->setGeometry(QRect(10, 120, 81, 41));
        BTN_MODE_SIMULATION->setMinimumSize(QSize(0, 40));
        commandLinkButton = new QCommandLinkButton(GB_NETWORK_2);
        commandLinkButton->setObjectName(QString::fromUtf8("commandLinkButton"));
        commandLinkButton->setGeometry(QRect(60, 160, 177, 41));
        LB_BASE_SPEED = new QLabel(rb_5);
        LB_BASE_SPEED->setObjectName(QString::fromUtf8("LB_BASE_SPEED"));
        LB_BASE_SPEED->setGeometry(QRect(790, 50, 51, 20));
        LB_BASE_SPEED->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        BTN_MOVE_JOINT_GRAPS_GRIPPER = new QPushButton(rb_5);
        BTN_MOVE_JOINT_GRAPS_GRIPPER->setObjectName(QString::fromUtf8("BTN_MOVE_JOINT_GRAPS_GRIPPER"));
        BTN_MOVE_JOINT_GRAPS_GRIPPER->setGeometry(QRect(1480, 750, 141, 71));
        label_137 = new QLabel(rb_5);
        label_137->setObjectName(QString::fromUtf8("label_137"));
        label_137->setGeometry(QRect(20, 0, 251, 41));
        label_137->setTextFormat(Qt::RichText);
        label_137->setTextInteractionFlags(Qt::LinksAccessibleByMouse);
        BTN_MOVE_JOINT_INIT_3 = new QPushButton(rb_5);
        BTN_MOVE_JOINT_INIT_3->setObjectName(QString::fromUtf8("BTN_MOVE_JOINT_INIT_3"));
        BTN_MOVE_JOINT_INIT_3->setGeometry(QRect(1290, 550, 141, 71));
        BTN_MOVE_JOINT_INIT_4 = new QPushButton(rb_5);
        BTN_MOVE_JOINT_INIT_4->setObjectName(QString::fromUtf8("BTN_MOVE_JOINT_INIT_4"));
        BTN_MOVE_JOINT_INIT_4->setGeometry(QRect(1260, 470, 141, 71));
        BTN_MOVE_JOINT_INIT_5 = new QPushButton(rb_5);
        BTN_MOVE_JOINT_INIT_5->setObjectName(QString::fromUtf8("BTN_MOVE_JOINT_INIT_5"));
        BTN_MOVE_JOINT_INIT_5->setGeometry(QRect(1410, 470, 141, 71));
        btn_pump = new QPushButton(rb_5);
        btn_pump->setObjectName(QString::fromUtf8("btn_pump"));
        btn_pump->setGeometry(QRect(680, 110, 71, 51));
        btn_pump->setStyleSheet(QString::fromUtf8("background-color: rgb(245, 121, 0);"));
        BTN_MOVE_JOINT_Rotation = new QPushButton(rb_5);
        BTN_MOVE_JOINT_Rotation->setObjectName(QString::fromUtf8("BTN_MOVE_JOINT_Rotation"));
        BTN_MOVE_JOINT_Rotation->setGeometry(QRect(1480, 590, 141, 71));
        layoutWidget5 = new QWidget(rb_5);
        layoutWidget5->setObjectName(QString::fromUtf8("layoutWidget5"));
        layoutWidget5->setGeometry(QRect(670, 640, 571, 91));
        gridLayout_15 = new QGridLayout(layoutWidget5);
        gridLayout_15->setObjectName(QString::fromUtf8("gridLayout_15"));
        gridLayout_15->setContentsMargins(0, 0, 0, 0);
        label_9 = new QLabel(layoutWidget5);
        label_9->setObjectName(QString::fromUtf8("label_9"));

        gridLayout_15->addWidget(label_9, 0, 0, 1, 1);

        LE_JOINT_LIST = new QLineEdit(layoutWidget5);
        LE_JOINT_LIST->setObjectName(QString::fromUtf8("LE_JOINT_LIST"));

        gridLayout_15->addWidget(LE_JOINT_LIST, 0, 1, 1, 1);

        LE_TCP_LIST = new QLineEdit(layoutWidget5);
        LE_TCP_LIST->setObjectName(QString::fromUtf8("LE_TCP_LIST"));

        gridLayout_15->addWidget(LE_TCP_LIST, 1, 1, 1, 1);

        label_17 = new QLabel(layoutWidget5);
        label_17->setObjectName(QString::fromUtf8("label_17"));

        gridLayout_15->addWidget(label_17, 1, 0, 1, 1);

        BTN_TCP_MOTION_TEST = new QPushButton(layoutWidget5);
        BTN_TCP_MOTION_TEST->setObjectName(QString::fromUtf8("BTN_TCP_MOTION_TEST"));
        QSizePolicy sizePolicy4(QSizePolicy::Maximum, QSizePolicy::Minimum);
        sizePolicy4.setHorizontalStretch(0);
        sizePolicy4.setVerticalStretch(0);
        sizePolicy4.setHeightForWidth(BTN_TCP_MOTION_TEST->sizePolicy().hasHeightForWidth());
        BTN_TCP_MOTION_TEST->setSizePolicy(sizePolicy4);
        BTN_TCP_MOTION_TEST->setMinimumSize(QSize(50, 40));

        gridLayout_15->addWidget(BTN_TCP_MOTION_TEST, 1, 2, 1, 1);

        BTN_JOINT_MOTION_TEST = new QPushButton(layoutWidget5);
        BTN_JOINT_MOTION_TEST->setObjectName(QString::fromUtf8("BTN_JOINT_MOTION_TEST"));
        sizePolicy4.setHeightForWidth(BTN_JOINT_MOTION_TEST->sizePolicy().hasHeightForWidth());
        BTN_JOINT_MOTION_TEST->setSizePolicy(sizePolicy4);
        BTN_JOINT_MOTION_TEST->setMinimumSize(QSize(50, 40));

        gridLayout_15->addWidget(BTN_JOINT_MOTION_TEST, 0, 2, 1, 1);

        BTN_MOVE_JOINT_POP = new QPushButton(rb_5);
        BTN_MOVE_JOINT_POP->setObjectName(QString::fromUtf8("BTN_MOVE_JOINT_POP"));
        BTN_MOVE_JOINT_POP->setGeometry(QRect(1300, 20, 141, 71));
        BTN_MOVE_JOINT_BOX_CENTER = new QPushButton(rb_5);
        BTN_MOVE_JOINT_BOX_CENTER->setObjectName(QString::fromUtf8("BTN_MOVE_JOINT_BOX_CENTER"));
        BTN_MOVE_JOINT_BOX_CENTER->setGeometry(QRect(1470, 20, 141, 71));
        pb_reload = new QPushButton(rb_5);
        pb_reload->setObjectName(QString::fromUtf8("pb_reload"));
        pb_reload->setGeometry(QRect(1210, 610, 21, 21));
        pb_reload->setMouseTracking(false);
        pb_reload->setAutoFillBackground(false);
        pb_reload->setStyleSheet(QString::fromUtf8("border-image: url(:/img/resource/reload.png);"));
        BTN_MOVE_JOINT_BOX_CENTER_KETI = new QPushButton(rb_5);
        BTN_MOVE_JOINT_BOX_CENTER_KETI->setObjectName(QString::fromUtf8("BTN_MOVE_JOINT_BOX_CENTER_KETI"));
        BTN_MOVE_JOINT_BOX_CENTER_KETI->setGeometry(QRect(1300, 100, 141, 71));
        BTN_MOVEL_rel = new QPushButton(rb_5);
        BTN_MOVEL_rel->setObjectName(QString::fromUtf8("BTN_MOVEL_rel"));
        BTN_MOVEL_rel->setGeometry(QRect(1280, 640, 141, 71));
        BTN_quick_return = new QPushButton(rb_5);
        BTN_quick_return->setObjectName(QString::fromUtf8("BTN_quick_return"));
        BTN_quick_return->setGeometry(QRect(1090, 270, 141, 71));
        layoutWidget6 = new QWidget(rb_5);
        layoutWidget6->setObjectName(QString::fromUtf8("layoutWidget6"));
        layoutWidget6->setGeometry(QRect(1240, 200, 151, 211));
        verticalLayout_7 = new QVBoxLayout(layoutWidget6);
        verticalLayout_7->setObjectName(QString::fromUtf8("verticalLayout_7"));
        verticalLayout_7->setContentsMargins(0, 0, 0, 0);
        BTN_MOVE_JOINT_HIGH = new QPushButton(layoutWidget6);
        BTN_MOVE_JOINT_HIGH->setObjectName(QString::fromUtf8("BTN_MOVE_JOINT_HIGH"));

        verticalLayout_7->addWidget(BTN_MOVE_JOINT_HIGH);

        BTN_MOVE_JOINT_MID = new QPushButton(layoutWidget6);
        BTN_MOVE_JOINT_MID->setObjectName(QString::fromUtf8("BTN_MOVE_JOINT_MID"));

        verticalLayout_7->addWidget(BTN_MOVE_JOINT_MID);

        BTN_MOVE_JOINT_LOW = new QPushButton(layoutWidget6);
        BTN_MOVE_JOINT_LOW->setObjectName(QString::fromUtf8("BTN_MOVE_JOINT_LOW"));

        verticalLayout_7->addWidget(BTN_MOVE_JOINT_LOW);

        layoutWidget7 = new QWidget(rb_5);
        layoutWidget7->setObjectName(QString::fromUtf8("layoutWidget7"));
        layoutWidget7->setGeometry(QRect(1410, 250, 260, 61));
        horizontalLayout_15 = new QHBoxLayout(layoutWidget7);
        horizontalLayout_15->setObjectName(QString::fromUtf8("horizontalLayout_15"));
        horizontalLayout_15->setContentsMargins(0, 0, 0, 0);
        BTN_MOVE_JOINT_MID_LEFT = new QPushButton(layoutWidget7);
        BTN_MOVE_JOINT_MID_LEFT->setObjectName(QString::fromUtf8("BTN_MOVE_JOINT_MID_LEFT"));

        horizontalLayout_15->addWidget(BTN_MOVE_JOINT_MID_LEFT);

        BTN_MOVE_JOINT_MID_RIGHT = new QPushButton(layoutWidget7);
        BTN_MOVE_JOINT_MID_RIGHT->setObjectName(QString::fromUtf8("BTN_MOVE_JOINT_MID_RIGHT"));

        horizontalLayout_15->addWidget(BTN_MOVE_JOINT_MID_RIGHT);

        layoutWidget_8 = new QWidget(rb_5);
        layoutWidget_8->setObjectName(QString::fromUtf8("layoutWidget_8"));
        layoutWidget_8->setGeometry(QRect(1410, 350, 260, 61));
        horizontalLayout_16 = new QHBoxLayout(layoutWidget_8);
        horizontalLayout_16->setObjectName(QString::fromUtf8("horizontalLayout_16"));
        horizontalLayout_16->setContentsMargins(0, 0, 0, 0);
        BTN_MOVE_JOINT_LOW_LEFT = new QPushButton(layoutWidget_8);
        BTN_MOVE_JOINT_LOW_LEFT->setObjectName(QString::fromUtf8("BTN_MOVE_JOINT_LOW_LEFT"));

        horizontalLayout_16->addWidget(BTN_MOVE_JOINT_LOW_LEFT);

        BTN_MOVE_JOINT_LOW_RIGHT = new QPushButton(layoutWidget_8);
        BTN_MOVE_JOINT_LOW_RIGHT->setObjectName(QString::fromUtf8("BTN_MOVE_JOINT_LOW_RIGHT"));

        horizontalLayout_16->addWidget(BTN_MOVE_JOINT_LOW_RIGHT);

        BTN_RETURN_MOVE_JOINT_BOX_CENTER = new QPushButton(rb_5);
        BTN_RETURN_MOVE_JOINT_BOX_CENTER->setObjectName(QString::fromUtf8("BTN_RETURN_MOVE_JOINT_BOX_CENTER"));
        BTN_RETURN_MOVE_JOINT_BOX_CENTER->setGeometry(QRect(1470, 100, 141, 71));
        scenario->addTab(rb_5, QString());
        lift_motor = new QWidget();
        lift_motor->setObjectName(QString::fromUtf8("lift_motor"));
        cb_port = new QComboBox(lift_motor);
        cb_port->setObjectName(QString::fromUtf8("cb_port"));
        cb_port->setGeometry(QRect(210, 70, 111, 31));
        groupBox_3 = new QGroupBox(lift_motor);
        groupBox_3->setObjectName(QString::fromUtf8("groupBox_3"));
        groupBox_3->setGeometry(QRect(440, 110, 251, 341));
        layoutWidget_7 = new QWidget(groupBox_3);
        layoutWidget_7->setObjectName(QString::fromUtf8("layoutWidget_7"));
        layoutWidget_7->setGeometry(QRect(20, 40, 211, 271));
        verticalLayout_8 = new QVBoxLayout(layoutWidget_7);
        verticalLayout_8->setObjectName(QString::fromUtf8("verticalLayout_8"));
        verticalLayout_8->setContentsMargins(0, 0, 0, 0);
        horizontalLayout_13 = new QHBoxLayout();
        horizontalLayout_13->setObjectName(QString::fromUtf8("horizontalLayout_13"));
        label_7 = new QLabel(layoutWidget_7);
        label_7->setObjectName(QString::fromUtf8("label_7"));

        horizontalLayout_13->addWidget(label_7);

        horizontalSpacer_10 = new QSpacerItem(100, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_13->addItem(horizontalSpacer_10);

        le_temp = new QLineEdit(layoutWidget_7);
        le_temp->setObjectName(QString::fromUtf8("le_temp"));

        horizontalLayout_13->addWidget(le_temp);


        verticalLayout_8->addLayout(horizontalLayout_13);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        label = new QLabel(layoutWidget_7);
        label->setObjectName(QString::fromUtf8("label"));

        horizontalLayout_5->addWidget(label);

        horizontalSpacer_4 = new QSpacerItem(108, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_5->addItem(horizontalSpacer_4);

        le_rpm = new QLineEdit(layoutWidget_7);
        le_rpm->setObjectName(QString::fromUtf8("le_rpm"));
        QSizePolicy sizePolicy5(QSizePolicy::Maximum, QSizePolicy::Fixed);
        sizePolicy5.setHorizontalStretch(0);
        sizePolicy5.setVerticalStretch(0);
        sizePolicy5.setHeightForWidth(le_rpm->sizePolicy().hasHeightForWidth());
        le_rpm->setSizePolicy(sizePolicy5);

        horizontalLayout_5->addWidget(le_rpm);


        verticalLayout_8->addLayout(horizontalLayout_5);

        horizontalLayout_11 = new QHBoxLayout();
        horizontalLayout_11->setObjectName(QString::fromUtf8("horizontalLayout_11"));
        label_4 = new QLabel(layoutWidget_7);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        horizontalLayout_11->addWidget(label_4);

        horizontalSpacer_8 = new QSpacerItem(88, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_11->addItem(horizontalSpacer_8);

        le_controlType = new QLineEdit(layoutWidget_7);
        le_controlType->setObjectName(QString::fromUtf8("le_controlType"));

        horizontalLayout_11->addWidget(le_controlType);


        verticalLayout_8->addLayout(horizontalLayout_11);

        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setObjectName(QString::fromUtf8("horizontalLayout_6"));
        label_6 = new QLabel(layoutWidget_7);
        label_6->setObjectName(QString::fromUtf8("label_6"));

        horizontalLayout_6->addWidget(label_6);

        horizontalSpacer_6 = new QSpacerItem(72, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_6->addItem(horizontalSpacer_6);

        le_rpmStd = new QLineEdit(layoutWidget_7);
        le_rpmStd->setObjectName(QString::fromUtf8("le_rpmStd"));

        horizontalLayout_6->addWidget(le_rpmStd);


        verticalLayout_8->addLayout(horizontalLayout_6);

        horizontalLayout_12 = new QHBoxLayout();
        horizontalLayout_12->setObjectName(QString::fromUtf8("horizontalLayout_12"));
        label_5 = new QLabel(layoutWidget_7);
        label_5->setObjectName(QString::fromUtf8("label_5"));

        horizontalLayout_12->addWidget(label_5);

        horizontalSpacer_9 = new QSpacerItem(55, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_12->addItem(horizontalSpacer_9);

        le_armp = new QLineEdit(layoutWidget_7);
        le_armp->setObjectName(QString::fromUtf8("le_armp"));

        horizontalLayout_12->addWidget(le_armp);


        verticalLayout_8->addLayout(horizontalLayout_12);

        horizontalLayout_10 = new QHBoxLayout();
        horizontalLayout_10->setObjectName(QString::fromUtf8("horizontalLayout_10"));
        label_3 = new QLabel(layoutWidget_7);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        horizontalLayout_10->addWidget(label_3);

        horizontalSpacer_7 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_10->addItem(horizontalSpacer_7);

        le_pos = new QLineEdit(layoutWidget_7);
        le_pos->setObjectName(QString::fromUtf8("le_pos"));

        horizontalLayout_10->addWidget(le_pos);


        verticalLayout_8->addLayout(horizontalLayout_10);

        te_data = new QTextEdit(lift_motor);
        te_data->setObjectName(QString::fromUtf8("te_data"));
        te_data->setGeometry(QRect(30, 70, 171, 301));
        bt_test = new QPushButton(lift_motor);
        bt_test->setObjectName(QString::fromUtf8("bt_test"));
        bt_test->setGeometry(QRect(30, 380, 171, 71));
        groupBox = new QGroupBox(lift_motor);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        groupBox->setGeometry(QRect(210, 110, 221, 191));
        ch_dir = new QCheckBox(groupBox);
        ch_dir->setObjectName(QString::fromUtf8("ch_dir"));
        ch_dir->setGeometry(QRect(10, 30, 71, 23));
        bt_homing = new QPushButton(groupBox);
        bt_homing->setObjectName(QString::fromUtf8("bt_homing"));
        bt_homing->setGeometry(QRect(110, 60, 101, 41));
        sb_move_pos = new QSpinBox(groupBox);
        sb_move_pos->setObjectName(QString::fromUtf8("sb_move_pos"));
        sb_move_pos->setGeometry(QRect(10, 110, 91, 31));
        sb_move_pos->setMaximum(6000);
        bt_move_pos = new QPushButton(groupBox);
        bt_move_pos->setObjectName(QString::fromUtf8("bt_move_pos"));
        bt_move_pos->setGeometry(QRect(110, 110, 101, 31));
        sb_max_vel_pos = new QSpinBox(groupBox);
        sb_max_vel_pos->setObjectName(QString::fromUtf8("sb_max_vel_pos"));
        sb_max_vel_pos->setGeometry(QRect(10, 150, 91, 31));
        sb_max_vel_pos->setMinimum(10);
        sb_max_vel_pos->setMaximum(3000);
        sb_max_vel_pos->setValue(200);
        bt_set_maxVel_pos = new QPushButton(groupBox);
        bt_set_maxVel_pos->setObjectName(QString::fromUtf8("bt_set_maxVel_pos"));
        bt_set_maxVel_pos->setGeometry(QRect(110, 150, 101, 31));
        bt_auto_homing = new QPushButton(groupBox);
        bt_auto_homing->setObjectName(QString::fromUtf8("bt_auto_homing"));
        bt_auto_homing->setGeometry(QRect(10, 60, 101, 41));
        groupBox_2 = new QGroupBox(lift_motor);
        groupBox_2->setObjectName(QString::fromUtf8("groupBox_2"));
        groupBox_2->setGeometry(QRect(210, 310, 221, 141));
        sb_rpm = new QSpinBox(groupBox_2);
        sb_rpm->setObjectName(QString::fromUtf8("sb_rpm"));
        sb_rpm->setGeometry(QRect(10, 30, 111, 31));
        sb_rpm->setMinimum(-3000);
        sb_rpm->setMaximum(3000);
        sb_rpm->setValue(700);
        bt_move_rpm = new QPushButton(groupBox_2);
        bt_move_rpm->setObjectName(QString::fromUtf8("bt_move_rpm"));
        bt_move_rpm->setGeometry(QRect(140, 30, 71, 31));
        bt_rpmZero = new QPushButton(groupBox_2);
        bt_rpmZero->setObjectName(QString::fromUtf8("bt_rpmZero"));
        bt_rpmZero->setGeometry(QRect(10, 70, 201, 61));
        QFont font11;
        font11.setPointSize(24);
        font11.setBold(true);
        font11.setWeight(75);
        bt_rpmZero->setFont(font11);
        bt_rpmZero->setStyleSheet(QString::fromUtf8("background-color:red"));
        bt_connect = new QPushButton(lift_motor);
        bt_connect->setObjectName(QString::fromUtf8("bt_connect"));
        bt_connect->setGeometry(QRect(330, 70, 71, 31));
        label_174 = new QLabel(lift_motor);
        label_174->setObjectName(QString::fromUtf8("label_174"));
        label_174->setGeometry(QRect(30, 20, 251, 41));
        label_174->setTextFormat(Qt::RichText);
        scenario->addTab(lift_motor, QString());
        tab1 = new QWidget();
        tab1->setObjectName(QString::fromUtf8("tab1"));
        GR_KITECH = new QGroupBox(tab1);
        GR_KITECH->setObjectName(QString::fromUtf8("GR_KITECH"));
        GR_KITECH->setGeometry(QRect(576, 168, 591, 411));
        GR_KITECH->setFont(font1);
        GR_KITECH->setStyleSheet(QString::fromUtf8(""));
        LE_GRIPPER_CMD = new QLineEdit(GR_KITECH);
        LE_GRIPPER_CMD->setObjectName(QString::fromUtf8("LE_GRIPPER_CMD"));
        LE_GRIPPER_CMD->setGeometry(QRect(10, 30, 221, 20));
        BTN_GRIPPER_CMD_WRITE = new QPushButton(GR_KITECH);
        BTN_GRIPPER_CMD_WRITE->setObjectName(QString::fromUtf8("BTN_GRIPPER_CMD_WRITE"));
        BTN_GRIPPER_CMD_WRITE->setGeometry(QRect(240, 30, 51, 21));
        BTN_GRIPPER_CMD_WRITE->setFont(font7);
        BTN_GRIPPER_ALLFOLD = new QPushButton(GR_KITECH);
        BTN_GRIPPER_ALLFOLD->setObjectName(QString::fromUtf8("BTN_GRIPPER_ALLFOLD"));
        BTN_GRIPPER_ALLFOLD->setGeometry(QRect(20, 60, 81, 30));
        QFont font12;
        font12.setFamily(QString::fromUtf8("Arial"));
        font12.setBold(true);
        font12.setWeight(75);
        BTN_GRIPPER_ALLFOLD->setFont(font12);
        BTN_RB5_CALC_GRAPS_POINT = new QPushButton(GR_KITECH);
        BTN_RB5_CALC_GRAPS_POINT->setObjectName(QString::fromUtf8("BTN_RB5_CALC_GRAPS_POINT"));
        BTN_RB5_CALC_GRAPS_POINT->setGeometry(QRect(100, 330, 89, 51));
        QFont font13;
        font13.setFamily(QString::fromUtf8("Liberation Sans"));
        font13.setPointSize(8);
        BTN_RB5_CALC_GRAPS_POINT->setFont(font13);
        BTN_RB5_MOVE_GRAPS_POINT = new QPushButton(GR_KITECH);
        BTN_RB5_MOVE_GRAPS_POINT->setObjectName(QString::fromUtf8("BTN_RB5_MOVE_GRAPS_POINT"));
        BTN_RB5_MOVE_GRAPS_POINT->setGeometry(QRect(200, 330, 89, 51));
        BTN_RB5_MOVE_GRAPS_POINT->setFont(font13);
        BTN_GRIPPER_VISION = new QPushButton(GR_KITECH);
        BTN_GRIPPER_VISION->setObjectName(QString::fromUtf8("BTN_GRIPPER_VISION"));
        BTN_GRIPPER_VISION->setGeometry(QRect(110, 60, 81, 30));
        BTN_GRIPPER_VISION->setFont(font12);
        label_127 = new QLabel(GR_KITECH);
        label_127->setObjectName(QString::fromUtf8("label_127"));
        label_127->setGeometry(QRect(22, 250, 67, 17));
        label_128 = new QLabel(GR_KITECH);
        label_128->setObjectName(QString::fromUtf8("label_128"));
        label_128->setGeometry(QRect(80, 250, 67, 17));
        label_129 = new QLabel(GR_KITECH);
        label_129->setObjectName(QString::fromUtf8("label_129"));
        label_129->setGeometry(QRect(133, 250, 67, 17));
        label_130 = new QLabel(GR_KITECH);
        label_130->setObjectName(QString::fromUtf8("label_130"));
        label_130->setGeometry(QRect(186, 250, 67, 17));
        layoutWidget_3 = new QWidget(GR_KITECH);
        layoutWidget_3->setObjectName(QString::fromUtf8("layoutWidget_3"));
        layoutWidget_3->setGeometry(QRect(20, 180, 187, 30));
        horizontalLayout_3 = new QHBoxLayout(layoutWidget_3);
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        horizontalLayout_3->setContentsMargins(0, 0, 0, 0);
        CB_GRIPPER_GRAPS = new QComboBox(layoutWidget_3);
        CB_GRIPPER_GRAPS->addItem(QString());
        CB_GRIPPER_GRAPS->addItem(QString());
        CB_GRIPPER_GRAPS->addItem(QString());
        CB_GRIPPER_GRAPS->addItem(QString());
        CB_GRIPPER_GRAPS->addItem(QString());
        CB_GRIPPER_GRAPS->addItem(QString());
        CB_GRIPPER_GRAPS->setObjectName(QString::fromUtf8("CB_GRIPPER_GRAPS"));

        horizontalLayout_3->addWidget(CB_GRIPPER_GRAPS);

        CB_GRIPPER_POWER = new QComboBox(layoutWidget_3);
        CB_GRIPPER_POWER->addItem(QString());
        CB_GRIPPER_POWER->addItem(QString());
        CB_GRIPPER_POWER->addItem(QString());
        CB_GRIPPER_POWER->addItem(QString());
        CB_GRIPPER_POWER->addItem(QString());
        CB_GRIPPER_POWER->addItem(QString());
        CB_GRIPPER_POWER->addItem(QString());
        CB_GRIPPER_POWER->addItem(QString());
        CB_GRIPPER_POWER->addItem(QString());
        CB_GRIPPER_POWER->addItem(QString());
        CB_GRIPPER_POWER->setObjectName(QString::fromUtf8("CB_GRIPPER_POWER"));

        horizontalLayout_3->addWidget(CB_GRIPPER_POWER);

        BTN_GRIPPER_GRASP = new QPushButton(layoutWidget_3);
        BTN_GRIPPER_GRASP->setObjectName(QString::fromUtf8("BTN_GRIPPER_GRASP"));
        BTN_GRIPPER_GRASP->setFont(font12);

        horizontalLayout_3->addWidget(BTN_GRIPPER_GRASP);

        layoutWidget_4 = new QWidget(GR_KITECH);
        layoutWidget_4->setObjectName(QString::fromUtf8("layoutWidget_4"));
        layoutWidget_4->setGeometry(QRect(20, 270, 311, 30));
        horizontalLayout_4 = new QHBoxLayout(layoutWidget_4);
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        horizontalLayout_4->setContentsMargins(0, 0, 0, 0);
        CB_GRIPPER_GE_MODE = new QComboBox(layoutWidget_4);
        CB_GRIPPER_GE_MODE->addItem(QString());
        CB_GRIPPER_GE_MODE->addItem(QString());
        CB_GRIPPER_GE_MODE->setObjectName(QString::fromUtf8("CB_GRIPPER_GE_MODE"));

        horizontalLayout_4->addWidget(CB_GRIPPER_GE_MODE);

        CB_GRIPPER_GE_GRAPS = new QComboBox(layoutWidget_4);
        CB_GRIPPER_GE_GRAPS->addItem(QString());
        CB_GRIPPER_GE_GRAPS->addItem(QString());
        CB_GRIPPER_GE_GRAPS->addItem(QString());
        CB_GRIPPER_GE_GRAPS->addItem(QString());
        CB_GRIPPER_GE_GRAPS->addItem(QString());
        CB_GRIPPER_GE_GRAPS->addItem(QString());
        CB_GRIPPER_GE_GRAPS->setObjectName(QString::fromUtf8("CB_GRIPPER_GE_GRAPS"));

        horizontalLayout_4->addWidget(CB_GRIPPER_GE_GRAPS);

        CB_GRIPPER_GE_POWER = new QComboBox(layoutWidget_4);
        CB_GRIPPER_GE_POWER->addItem(QString());
        CB_GRIPPER_GE_POWER->addItem(QString());
        CB_GRIPPER_GE_POWER->addItem(QString());
        CB_GRIPPER_GE_POWER->addItem(QString());
        CB_GRIPPER_GE_POWER->addItem(QString());
        CB_GRIPPER_GE_POWER->addItem(QString());
        CB_GRIPPER_GE_POWER->addItem(QString());
        CB_GRIPPER_GE_POWER->addItem(QString());
        CB_GRIPPER_GE_POWER->addItem(QString());
        CB_GRIPPER_GE_POWER->addItem(QString());
        CB_GRIPPER_GE_POWER->setObjectName(QString::fromUtf8("CB_GRIPPER_GE_POWER"));

        horizontalLayout_4->addWidget(CB_GRIPPER_GE_POWER);

        CB_GRIPPER_GE_SUCTION_POWER = new QComboBox(layoutWidget_4);
        CB_GRIPPER_GE_SUCTION_POWER->addItem(QString());
        CB_GRIPPER_GE_SUCTION_POWER->addItem(QString());
        CB_GRIPPER_GE_SUCTION_POWER->setObjectName(QString::fromUtf8("CB_GRIPPER_GE_SUCTION_POWER"));

        horizontalLayout_4->addWidget(CB_GRIPPER_GE_SUCTION_POWER);

        CB_GRIPPER_GE_CMD_WRITE = new QPushButton(layoutWidget_4);
        CB_GRIPPER_GE_CMD_WRITE->setObjectName(QString::fromUtf8("CB_GRIPPER_GE_CMD_WRITE"));
        sizePolicy.setHeightForWidth(CB_GRIPPER_GE_CMD_WRITE->sizePolicy().hasHeightForWidth());
        CB_GRIPPER_GE_CMD_WRITE->setSizePolicy(sizePolicy);
        CB_GRIPPER_GE_CMD_WRITE->setMinimumSize(QSize(5, 5));
        CB_GRIPPER_GE_CMD_WRITE->setMaximumSize(QSize(100, 100));
        CB_GRIPPER_GE_CMD_WRITE->setFont(font7);

        horizontalLayout_4->addWidget(CB_GRIPPER_GE_CMD_WRITE);

        Pinch12_FOLD = new QPushButton(GR_KITECH);
        Pinch12_FOLD->setObjectName(QString::fromUtf8("Pinch12_FOLD"));
        Pinch12_FOLD->setGeometry(QRect(210, 180, 111, 30));
        Pinch12_FOLD->setFont(font12);
        label_152 = new QLabel(GR_KITECH);
        label_152->setObjectName(QString::fromUtf8("label_152"));
        label_152->setGeometry(QRect(20, 160, 141, 17));
        label_153 = new QLabel(GR_KITECH);
        label_153->setObjectName(QString::fromUtf8("label_153"));
        label_153->setGeometry(QRect(20, 230, 141, 17));
        textBrowser = new QTextBrowser(GR_KITECH);
        textBrowser->setObjectName(QString::fromUtf8("textBrowser"));
        textBrowser->setGeometry(QRect(340, 40, 231, 201));
        textBrowser_2 = new QTextBrowser(GR_KITECH);
        textBrowser_2->setObjectName(QString::fromUtf8("textBrowser_2"));
        textBrowser_2->setGeometry(QRect(340, 250, 231, 131));
        layoutWidget8 = new QWidget(GR_KITECH);
        layoutWidget8->setObjectName(QString::fromUtf8("layoutWidget8"));
        layoutWidget8->setGeometry(QRect(210, 60, 111, 30));
        gridLayout_17 = new QGridLayout(layoutWidget8);
        gridLayout_17->setObjectName(QString::fromUtf8("gridLayout_17"));
        gridLayout_17->setContentsMargins(0, 0, 0, 0);
        CB_GRIPPER_recog = new QComboBox(layoutWidget8);
        CB_GRIPPER_recog->addItem(QString());
        CB_GRIPPER_recog->addItem(QString());
        CB_GRIPPER_recog->addItem(QString());
        CB_GRIPPER_recog->addItem(QString());
        CB_GRIPPER_recog->addItem(QString());
        CB_GRIPPER_recog->setObjectName(QString::fromUtf8("CB_GRIPPER_recog"));
        sizePolicy1.setHeightForWidth(CB_GRIPPER_recog->sizePolicy().hasHeightForWidth());
        CB_GRIPPER_recog->setSizePolicy(sizePolicy1);
        CB_GRIPPER_recog->setMaximumSize(QSize(50, 16777215));

        gridLayout_17->addWidget(CB_GRIPPER_recog, 0, 0, 1, 1);

        BTN_GRIPPER_recog_WRITE = new QPushButton(layoutWidget8);
        BTN_GRIPPER_recog_WRITE->setObjectName(QString::fromUtf8("BTN_GRIPPER_recog_WRITE"));
        sizePolicy1.setHeightForWidth(BTN_GRIPPER_recog_WRITE->sizePolicy().hasHeightForWidth());
        BTN_GRIPPER_recog_WRITE->setSizePolicy(sizePolicy1);
        BTN_GRIPPER_recog_WRITE->setMinimumSize(QSize(10, 0));
        BTN_GRIPPER_recog_WRITE->setMaximumSize(QSize(50, 100));
        BTN_GRIPPER_recog_WRITE->setFont(font7);

        gridLayout_17->addWidget(BTN_GRIPPER_recog_WRITE, 0, 1, 1, 1);

        BTN_GRIPPER_SUCTION = new QPushButton(GR_KITECH);
        BTN_GRIPPER_SUCTION->setObjectName(QString::fromUtf8("BTN_GRIPPER_SUCTION"));
        BTN_GRIPPER_SUCTION->setGeometry(QRect(20, 100, 81, 30));
        BTN_GRIPPER_SUCTION->setFont(font12);
        BTN_GRIPPER_OPEN = new QPushButton(GR_KITECH);
        BTN_GRIPPER_OPEN->setObjectName(QString::fromUtf8("BTN_GRIPPER_OPEN"));
        BTN_GRIPPER_OPEN->setGeometry(QRect(110, 100, 81, 30));
        BTN_GRIPPER_OPEN->setFont(font12);
        notice_kitech = new QTextEdit(tab1);
        notice_kitech->setObjectName(QString::fromUtf8("notice_kitech"));
        notice_kitech->setGeometry(QRect(576, 120, 591, 31));
        notice_kitech->setStyleSheet(QString::fromUtf8("    border-style: outset;\n"
"    border-width: 1px;\n"
"    border-color: black;\n"
"background-color:rgb(255,255,255)"));
        label_131 = new QLabel(tab1);
        label_131->setObjectName(QString::fromUtf8("label_131"));
        label_131->setGeometry(QRect(50, 80, 67, 17));
        le_keti = new QLineEdit(tab1);
        le_keti->setObjectName(QString::fromUtf8("le_keti"));
        le_keti->setGeometry(QRect(90, 70, 31, 31));
        le_keti->setCursor(QCursor(Qt::BlankCursor));
        le_keti->setStyleSheet(QString::fromUtf8("    border-style: outset;\n"
"    border-width: 1px;\n"
"    border-color: black;"));
        lb_keti_point = new QLabel(tab1);
        lb_keti_point->setObjectName(QString::fromUtf8("lb_keti_point"));
        lb_keti_point->setGeometry(QRect(30, 120, 481, 31));
        lb_keti_point->setStyleSheet(QString::fromUtf8("    border-style: outset;\n"
"    border-width: 1px;\n"
"    border-color: black;\n"
"background-color:rgb(255,255,255)"));
        le_kitech = new QLineEdit(tab1);
        le_kitech->setObjectName(QString::fromUtf8("le_kitech"));
        le_kitech->setGeometry(QRect(630, 70, 31, 31));
        le_kitech->setCursor(QCursor(Qt::BlankCursor));
        le_kitech->setStyleSheet(QString::fromUtf8("    border-style: outset;\n"
"    border-width: 1px;\n"
"    border-color: black;"));
        label_133 = new QLabel(tab1);
        label_133->setObjectName(QString::fromUtf8("label_133"));
        label_133->setGeometry(QRect(580, 80, 67, 17));
        label_134 = new QLabel(tab1);
        label_134->setObjectName(QString::fromUtf8("label_134"));
        label_134->setGeometry(QRect(30, 20, 251, 41));
        label_134->setTextFormat(Qt::RichText);
        label_135 = new QLabel(tab1);
        label_135->setObjectName(QString::fromUtf8("label_135"));
        label_135->setGeometry(QRect(30, 160, 171, 31));
        label_135->setTextFormat(Qt::RichText);
        BTN_CONNECT_GRIPPER = new QPushButton(tab1);
        BTN_CONNECT_GRIPPER->setObjectName(QString::fromUtf8("BTN_CONNECT_GRIPPER"));
        BTN_CONNECT_GRIPPER->setGeometry(QRect(670, 70, 41, 30));
        BTN_CONNECT_GRIPPER->setFont(font7);
        LE_KETI_IMG = new QLabel(tab1);
        LE_KETI_IMG->setObjectName(QString::fromUtf8("LE_KETI_IMG"));
        LE_KETI_IMG->setGeometry(QRect(30, 200, 500, 300));
        pb_shutter = new QPushButton(tab1);
        pb_shutter->setObjectName(QString::fromUtf8("pb_shutter"));
        pb_shutter->setGeometry(QRect(350, 71, 89, 30));
        cb_get_object_id_vision = new QComboBox(tab1);
        cb_get_object_id_vision->setObjectName(QString::fromUtf8("cb_get_object_id_vision"));
        cb_get_object_id_vision->setGeometry(QRect(140, 71, 191, 30));
        scenario->addTab(tab1, QString());
        barcode = new QWidget();
        barcode->setObjectName(QString::fromUtf8("barcode"));
        label_111 = new QLabel(barcode);
        label_111->setObjectName(QString::fromUtf8("label_111"));
        label_111->setGeometry(QRect(30, 100, 61, 31));
        label_111->setTextFormat(Qt::RichText);
        label_112 = new QLabel(barcode);
        label_112->setObjectName(QString::fromUtf8("label_112"));
        label_112->setGeometry(QRect(330, 100, 61, 31));
        label_112->setTextFormat(Qt::RichText);
        cb_usb2 = new QComboBox(barcode);
        cb_usb2->setObjectName(QString::fromUtf8("cb_usb2"));
        cb_usb2->setGeometry(QRect(330, 130, 111, 25));
        write = new QPushButton(barcode);
        write->setObjectName(QString::fromUtf8("write"));
        write->setGeometry(QRect(520, 30, 89, 25));
        te_usb1 = new QTextEdit(barcode);
        te_usb1->setObjectName(QString::fromUtf8("te_usb1"));
        te_usb1->setGeometry(QRect(30, 170, 231, 351));
        layoutWidget_2 = new QWidget(barcode);
        layoutWidget_2->setObjectName(QString::fromUtf8("layoutWidget_2"));
        layoutWidget_2->setGeometry(QRect(460, 100, 92, 62));
        verticalLayout_3 = new QVBoxLayout(layoutWidget_2);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        verticalLayout_3->setContentsMargins(0, 0, 0, 0);
        bt_usb2_dis = new QPushButton(layoutWidget_2);
        bt_usb2_dis->setObjectName(QString::fromUtf8("bt_usb2_dis"));

        verticalLayout_3->addWidget(bt_usb2_dis);

        bt_usb2_con = new QPushButton(layoutWidget_2);
        bt_usb2_con->setObjectName(QString::fromUtf8("bt_usb2_con"));

        verticalLayout_3->addWidget(bt_usb2_con);

        layoutWidget9 = new QWidget(barcode);
        layoutWidget9->setObjectName(QString::fromUtf8("layoutWidget9"));
        layoutWidget9->setGeometry(QRect(160, 100, 92, 62));
        verticalLayout_4 = new QVBoxLayout(layoutWidget9);
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        verticalLayout_4->setContentsMargins(0, 0, 0, 0);
        bt_usb1_dis = new QPushButton(layoutWidget9);
        bt_usb1_dis->setObjectName(QString::fromUtf8("bt_usb1_dis"));

        verticalLayout_4->addWidget(bt_usb1_dis);

        bt_usb1_con = new QPushButton(layoutWidget9);
        bt_usb1_con->setObjectName(QString::fromUtf8("bt_usb1_con"));

        verticalLayout_4->addWidget(bt_usb1_con);

        label_113 = new QLabel(barcode);
        label_113->setObjectName(QString::fromUtf8("label_113"));
        label_113->setGeometry(QRect(30, 20, 311, 31));
        label_113->setTextFormat(Qt::RichText);
        cb_usb1 = new QComboBox(barcode);
        cb_usb1->setObjectName(QString::fromUtf8("cb_usb1"));
        cb_usb1->setGeometry(QRect(30, 130, 111, 25));
        te_usb2 = new QTextEdit(barcode);
        te_usb2->setObjectName(QString::fromUtf8("te_usb2"));
        te_usb2->setGeometry(QRect(330, 170, 231, 351));
        BTN_MOVE_TOTAL = new QPushButton(barcode);
        BTN_MOVE_TOTAL->setObjectName(QString::fromUtf8("BTN_MOVE_TOTAL"));
        BTN_MOVE_TOTAL->setGeometry(QRect(700, 90, 111, 51));
        BTN_MOVE_TOTAL_2 = new QPushButton(barcode);
        BTN_MOVE_TOTAL_2->setObjectName(QString::fromUtf8("BTN_MOVE_TOTAL_2"));
        BTN_MOVE_TOTAL_2->setGeometry(QRect(830, 90, 81, 51));
        BTN_MOVE_TOTAL_3 = new QPushButton(barcode);
        BTN_MOVE_TOTAL_3->setObjectName(QString::fromUtf8("BTN_MOVE_TOTAL_3"));
        BTN_MOVE_TOTAL_3->setGeometry(QRect(700, 160, 111, 51));
        BTN_MOVE_TOTAL_4 = new QPushButton(barcode);
        BTN_MOVE_TOTAL_4->setObjectName(QString::fromUtf8("BTN_MOVE_TOTAL_4"));
        BTN_MOVE_TOTAL_4->setGeometry(QRect(830, 160, 111, 51));
        BTN_show = new QPushButton(barcode);
        BTN_show->setObjectName(QString::fromUtf8("BTN_show"));
        BTN_show->setGeometry(QRect(990, 90, 111, 51));
        scenario->addTab(barcode, QString());
        tab_2 = new QWidget();
        tab_2->setObjectName(QString::fromUtf8("tab_2"));
        label_143 = new QLabel(tab_2);
        label_143->setObjectName(QString::fromUtf8("label_143"));
        label_143->setGeometry(QRect(240, 90, 61, 31));
        label_143->setTextFormat(Qt::RichText);
        te_label = new QLabel(tab_2);
        te_label->setObjectName(QString::fromUtf8("te_label"));
        te_label->setGeometry(QRect(230, 30, 171, 23));
        label_144 = new QLabel(tab_2);
        label_144->setObjectName(QString::fromUtf8("label_144"));
        label_144->setGeometry(QRect(800, 90, 61, 31));
        label_144->setTextFormat(Qt::RichText);
        te_msg = new QTextEdit(tab_2);
        te_msg->setObjectName(QString::fromUtf8("te_msg"));
        te_msg->setGeometry(QRect(20, 130, 471, 651));
        label_145 = new QLabel(tab_2);
        label_145->setObjectName(QString::fromUtf8("label_145"));
        label_145->setGeometry(QRect(40, 20, 201, 31));
        label_145->setTextFormat(Qt::RichText);
        label_146 = new QLabel(tab_2);
        label_146->setObjectName(QString::fromUtf8("label_146"));
        label_146->setGeometry(QRect(190, 90, 51, 31));
        label_146->setContextMenuPolicy(Qt::NoContextMenu);
        label_146->setStyleSheet(QString::fromUtf8("image: url(:/img/resource/yujin.png);"));
        label_146->setTextFormat(Qt::RichText);
        label_146->setTextInteractionFlags(Qt::NoTextInteraction);
        label_147 = new QLabel(tab_2);
        label_147->setObjectName(QString::fromUtf8("label_147"));
        label_147->setGeometry(QRect(690, 80, 111, 51));
        label_147->setContextMenuPolicy(Qt::NoContextMenu);
        label_147->setStyleSheet(QString::fromUtf8("image: url(:/img/resource/rainbow.png);"));
        label_147->setFrameShadow(QFrame::Raised);
        label_147->setTextFormat(Qt::RichText);
        te_send_msg = new QTextEdit(tab_2);
        te_send_msg->setObjectName(QString::fromUtf8("te_send_msg"));
        te_send_msg->setGeometry(QRect(530, 130, 491, 651));
        scenario->addTab(tab_2, QString());
        tab_3 = new QWidget();
        tab_3->setObjectName(QString::fromUtf8("tab_3"));
        bt_save_all_annot = new QPushButton(tab_3);
        bt_save_all_annot->setObjectName(QString::fromUtf8("bt_save_all_annot"));
        bt_save_all_annot->setGeometry(QRect(340, 70, 171, 41));
        cb_shelf_id = new QComboBox(tab_3);
        cb_shelf_id->setObjectName(QString::fromUtf8("cb_shelf_id"));
        cb_shelf_id->setGeometry(QRect(110, 129, 120, 25));
        cb_get_object_id = new QComboBox(tab_3);
        cb_get_object_id->setObjectName(QString::fromUtf8("cb_get_object_id"));
        cb_get_object_id->setGeometry(QRect(700, 40, 191, 25));
        label_165 = new QLabel(tab_3);
        label_165->setObjectName(QString::fromUtf8("label_165"));
        label_165->setGeometry(QRect(30, 26, 231, 31));
        label_165->setTextFormat(Qt::RichText);
        bt_save_annot = new QPushButton(tab_3);
        bt_save_annot->setObjectName(QString::fromUtf8("bt_save_annot"));
        bt_save_annot->setGeometry(QRect(316, 130, 101, 41));
        te_scen = new QTextEdit(tab_3);
        te_scen->setObjectName(QString::fromUtf8("te_scen"));
        te_scen->setGeometry(QRect(36, 180, 501, 571));
        layoutWidget_5 = new QWidget(tab_3);
        layoutWidget_5->setObjectName(QString::fromUtf8("layoutWidget_5"));
        layoutWidget_5->setGeometry(QRect(700, 76, 191, 31));
        horizontalLayout_7 = new QHBoxLayout(layoutWidget_5);
        horizontalLayout_7->setObjectName(QString::fromUtf8("horizontalLayout_7"));
        horizontalLayout_7->setContentsMargins(0, 0, 0, 0);
        CB_GRIPPER_GRAPS_3 = new QComboBox(layoutWidget_5);
        CB_GRIPPER_GRAPS_3->addItem(QString());
        CB_GRIPPER_GRAPS_3->addItem(QString());
        CB_GRIPPER_GRAPS_3->addItem(QString());
        CB_GRIPPER_GRAPS_3->addItem(QString());
        CB_GRIPPER_GRAPS_3->addItem(QString());
        CB_GRIPPER_GRAPS_3->addItem(QString());
        CB_GRIPPER_GRAPS_3->setObjectName(QString::fromUtf8("CB_GRIPPER_GRAPS_3"));

        horizontalLayout_7->addWidget(CB_GRIPPER_GRAPS_3);

        CB_GRIPPER_POWER_3 = new QComboBox(layoutWidget_5);
        CB_GRIPPER_POWER_3->addItem(QString());
        CB_GRIPPER_POWER_3->addItem(QString());
        CB_GRIPPER_POWER_3->addItem(QString());
        CB_GRIPPER_POWER_3->addItem(QString());
        CB_GRIPPER_POWER_3->addItem(QString());
        CB_GRIPPER_POWER_3->addItem(QString());
        CB_GRIPPER_POWER_3->addItem(QString());
        CB_GRIPPER_POWER_3->addItem(QString());
        CB_GRIPPER_POWER_3->addItem(QString());
        CB_GRIPPER_POWER_3->addItem(QString());
        CB_GRIPPER_POWER_3->setObjectName(QString::fromUtf8("CB_GRIPPER_POWER_3"));

        horizontalLayout_7->addWidget(CB_GRIPPER_POWER_3);

        label_167 = new QLabel(tab_3);
        label_167->setObjectName(QString::fromUtf8("label_167"));
        label_167->setGeometry(QRect(703, 110, 67, 17));
        label_167->setAlignment(Qt::AlignCenter);
        label_168 = new QLabel(tab_3);
        label_168->setObjectName(QString::fromUtf8("label_168"));
        label_168->setGeometry(QRect(860, 110, 67, 17));
        label_168->setAlignment(Qt::AlignCenter);
        layoutWidget_6 = new QWidget(tab_3);
        layoutWidget_6->setObjectName(QString::fromUtf8("layoutWidget_6"));
        layoutWidget_6->setGeometry(QRect(701, 130, 311, 29));
        horizontalLayout_8 = new QHBoxLayout(layoutWidget_6);
        horizontalLayout_8->setObjectName(QString::fromUtf8("horizontalLayout_8"));
        horizontalLayout_8->setContentsMargins(0, 0, 0, 0);
        CB_GRIPPER_GE_MODE_3 = new QComboBox(layoutWidget_6);
        CB_GRIPPER_GE_MODE_3->addItem(QString());
        CB_GRIPPER_GE_MODE_3->addItem(QString());
        CB_GRIPPER_GE_MODE_3->setObjectName(QString::fromUtf8("CB_GRIPPER_GE_MODE_3"));

        horizontalLayout_8->addWidget(CB_GRIPPER_GE_MODE_3);

        CB_GRIPPER_GE_GRAPS_3 = new QComboBox(layoutWidget_6);
        CB_GRIPPER_GE_GRAPS_3->addItem(QString());
        CB_GRIPPER_GE_GRAPS_3->addItem(QString());
        CB_GRIPPER_GE_GRAPS_3->addItem(QString());
        CB_GRIPPER_GE_GRAPS_3->addItem(QString());
        CB_GRIPPER_GE_GRAPS_3->addItem(QString());
        CB_GRIPPER_GE_GRAPS_3->addItem(QString());
        CB_GRIPPER_GE_GRAPS_3->setObjectName(QString::fromUtf8("CB_GRIPPER_GE_GRAPS_3"));

        horizontalLayout_8->addWidget(CB_GRIPPER_GE_GRAPS_3);

        CB_GRIPPER_GE_POWER_3 = new QComboBox(layoutWidget_6);
        CB_GRIPPER_GE_POWER_3->addItem(QString());
        CB_GRIPPER_GE_POWER_3->addItem(QString());
        CB_GRIPPER_GE_POWER_3->addItem(QString());
        CB_GRIPPER_GE_POWER_3->addItem(QString());
        CB_GRIPPER_GE_POWER_3->addItem(QString());
        CB_GRIPPER_GE_POWER_3->addItem(QString());
        CB_GRIPPER_GE_POWER_3->addItem(QString());
        CB_GRIPPER_GE_POWER_3->addItem(QString());
        CB_GRIPPER_GE_POWER_3->addItem(QString());
        CB_GRIPPER_GE_POWER_3->addItem(QString());
        CB_GRIPPER_GE_POWER_3->setObjectName(QString::fromUtf8("CB_GRIPPER_GE_POWER_3"));

        horizontalLayout_8->addWidget(CB_GRIPPER_GE_POWER_3);

        CB_GRIPPER_GE_SUCTION_POWER_3 = new QComboBox(layoutWidget_6);
        CB_GRIPPER_GE_SUCTION_POWER_3->addItem(QString());
        CB_GRIPPER_GE_SUCTION_POWER_3->addItem(QString());
        CB_GRIPPER_GE_SUCTION_POWER_3->setObjectName(QString::fromUtf8("CB_GRIPPER_GE_SUCTION_POWER_3"));

        horizontalLayout_8->addWidget(CB_GRIPPER_GE_SUCTION_POWER_3);

        label_169 = new QLabel(tab_3);
        label_169->setObjectName(QString::fromUtf8("label_169"));
        label_169->setGeometry(QRect(782, 110, 67, 17));
        label_169->setAlignment(Qt::AlignCenter);
        label_172 = new QLabel(tab_3);
        label_172->setObjectName(QString::fromUtf8("label_172"));
        label_172->setGeometry(QRect(940, 110, 67, 17));
        label_172->setAlignment(Qt::AlignCenter);
        bt_delete_annot = new QPushButton(tab_3);
        bt_delete_annot->setObjectName(QString::fromUtf8("bt_delete_annot"));
        bt_delete_annot->setGeometry(QRect(436, 130, 101, 41));
        bt_order = new QPushButton(tab_3);
        bt_order->setObjectName(QString::fromUtf8("bt_order"));
        bt_order->setGeometry(QRect(1510, 70, 71, 51));
        label_173 = new QLabel(tab_3);
        label_173->setObjectName(QString::fromUtf8("label_173"));
        label_173->setGeometry(QRect(1190, 55, 50, 21));
        te_order = new QTextEdit(tab_3);
        te_order->setObjectName(QString::fromUtf8("te_order"));
        te_order->setGeometry(QRect(1170, 180, 541, 571));
        layoutWidget10 = new QWidget(tab_3);
        layoutWidget10->setObjectName(QString::fromUtf8("layoutWidget10"));
        layoutWidget10->setGeometry(QRect(604, 30, 81, 131));
        verticalLayout_2 = new QVBoxLayout(layoutWidget10);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        verticalLayout_2->setContentsMargins(0, 0, 0, 0);
        label_164 = new QLabel(layoutWidget10);
        label_164->setObjectName(QString::fromUtf8("label_164"));
        label_164->setLayoutDirection(Qt::LeftToRight);
        label_164->setAlignment(Qt::AlignCenter);

        verticalLayout_2->addWidget(label_164);

        label_170 = new QLabel(layoutWidget10);
        label_170->setObjectName(QString::fromUtf8("label_170"));
        label_170->setAlignment(Qt::AlignCenter);

        verticalLayout_2->addWidget(label_170);

        label_171 = new QLabel(layoutWidget10);
        label_171->setObjectName(QString::fromUtf8("label_171"));
        label_171->setAlignment(Qt::AlignCenter);

        verticalLayout_2->addWidget(label_171);

        cb_obj_direction = new QComboBox(tab_3);
        cb_obj_direction->addItem(QString());
        cb_obj_direction->addItem(QString());
        cb_obj_direction->setObjectName(QString::fromUtf8("cb_obj_direction"));
        cb_obj_direction->setGeometry(QRect(236, 129, 61, 25));
        label_163 = new QLabel(tab_3);
        label_163->setObjectName(QString::fromUtf8("label_163"));
        label_163->setGeometry(QRect(30, 125, 71, 31));
        label_163->setLayoutDirection(Qt::LeftToRight);
        label_163->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        te_scen_2 = new QTextEdit(tab_3);
        te_scen_2->setObjectName(QString::fromUtf8("te_scen_2"));
        te_scen_2->setGeometry(QRect(600, 180, 501, 571));
        bt_save_annot_2 = new QPushButton(tab_3);
        bt_save_annot_2->setObjectName(QString::fromUtf8("bt_save_annot_2"));
        bt_save_annot_2->setGeometry(QRect(910, 60, 101, 41));
        bt_save_all_annot_2 = new QPushButton(tab_3);
        bt_save_all_annot_2->setObjectName(QString::fromUtf8("bt_save_all_annot_2"));
        bt_save_all_annot_2->setGeometry(QRect(930, 10, 171, 41));
        bt_delete_annot_2 = new QPushButton(tab_3);
        bt_delete_annot_2->setObjectName(QString::fromUtf8("bt_delete_annot_2"));
        bt_delete_annot_2->setGeometry(QRect(1020, 60, 101, 41));
        layoutWidget11 = new QWidget(tab_3);
        layoutWidget11->setObjectName(QString::fromUtf8("layoutWidget11"));
        layoutWidget11->setGeometry(QRect(1250, 50, 253, 93));
        verticalLayout_6 = new QVBoxLayout(layoutWidget11);
        verticalLayout_6->setObjectName(QString::fromUtf8("verticalLayout_6"));
        verticalLayout_6->setContentsMargins(0, 0, 0, 0);
        verticalLayout_5 = new QVBoxLayout();
        verticalLayout_5->setObjectName(QString::fromUtf8("verticalLayout_5"));
        horizontalLayout_9 = new QHBoxLayout();
        horizontalLayout_9->setObjectName(QString::fromUtf8("horizontalLayout_9"));
        CB_shelf = new QComboBox(layoutWidget11);
        CB_shelf->setObjectName(QString::fromUtf8("CB_shelf"));

        horizontalLayout_9->addWidget(CB_shelf);

        cb_obj_direction_2 = new QComboBox(layoutWidget11);
        cb_obj_direction_2->addItem(QString());
        cb_obj_direction_2->addItem(QString());
        cb_obj_direction_2->setObjectName(QString::fromUtf8("cb_obj_direction_2"));

        horizontalLayout_9->addWidget(cb_obj_direction_2);

        cb_get_object_id_2 = new QComboBox(layoutWidget11);
        cb_get_object_id_2->setObjectName(QString::fromUtf8("cb_get_object_id_2"));

        horizontalLayout_9->addWidget(cb_get_object_id_2);


        verticalLayout_5->addLayout(horizontalLayout_9);


        verticalLayout_6->addLayout(verticalLayout_5);

        gridLayout_16 = new QGridLayout();
        gridLayout_16->setObjectName(QString::fromUtf8("gridLayout_16"));
        label_178 = new QLabel(layoutWidget11);
        label_178->setObjectName(QString::fromUtf8("label_178"));
        label_178->setAlignment(Qt::AlignCenter);

        gridLayout_16->addWidget(label_178, 0, 0, 1, 1);

        label_176 = new QLabel(layoutWidget11);
        label_176->setObjectName(QString::fromUtf8("label_176"));
        label_176->setAlignment(Qt::AlignCenter);

        gridLayout_16->addWidget(label_176, 0, 1, 1, 1);


        verticalLayout_6->addLayout(gridLayout_16);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        horizontalSpacer_13 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer_13);

        le_shelf_hight = new QLineEdit(layoutWidget11);
        le_shelf_hight->setObjectName(QString::fromUtf8("le_shelf_hight"));
        le_shelf_hight->setEnabled(true);
        sizePolicy1.setHeightForWidth(le_shelf_hight->sizePolicy().hasHeightForWidth());
        le_shelf_hight->setSizePolicy(sizePolicy1);
        le_shelf_hight->setMinimumSize(QSize(20, 20));
        le_shelf_hight->setMaximumSize(QSize(100, 25));

        horizontalLayout_2->addWidget(le_shelf_hight);

        horizontalSpacer_11 = new QSpacerItem(70, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer_11);

        CB_gripper_num = new QComboBox(layoutWidget11);
        CB_gripper_num->addItem(QString());
        CB_gripper_num->addItem(QString());
        CB_gripper_num->addItem(QString());
        CB_gripper_num->addItem(QString());
        CB_gripper_num->addItem(QString());
        CB_gripper_num->addItem(QString());
        CB_gripper_num->addItem(QString());
        CB_gripper_num->addItem(QString());
        CB_gripper_num->addItem(QString());
        CB_gripper_num->addItem(QString());
        CB_gripper_num->setObjectName(QString::fromUtf8("CB_gripper_num"));
        QSizePolicy sizePolicy6(QSizePolicy::Preferred, QSizePolicy::Minimum);
        sizePolicy6.setHorizontalStretch(0);
        sizePolicy6.setVerticalStretch(0);
        sizePolicy6.setHeightForWidth(CB_gripper_num->sizePolicy().hasHeightForWidth());
        CB_gripper_num->setSizePolicy(sizePolicy6);
        CB_gripper_num->setMaximumSize(QSize(50, 50));

        horizontalLayout_2->addWidget(CB_gripper_num);

        horizontalSpacer_12 = new QSpacerItem(50, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer_12);


        verticalLayout_6->addLayout(horizontalLayout_2);

        bt_scen_resume = new QPushButton(tab_3);
        bt_scen_resume->setObjectName(QString::fromUtf8("bt_scen_resume"));
        bt_scen_resume->setGeometry(QRect(1590, 110, 101, 51));
        QFont font14;
        font14.setFamily(QString::fromUtf8("Ubuntu Condensed"));
        font14.setPointSize(20);
        font14.setBold(true);
        font14.setItalic(true);
        font14.setWeight(75);
        bt_scen_resume->setFont(font14);
        bt_scen_resume->setLayoutDirection(Qt::LeftToRight);
        bt_scen_resume->setStyleSheet(QString::fromUtf8("background-color: rgb(245, 121, 0);"));
        te_status = new QLabel(tab_3);
        te_status->setObjectName(QString::fromUtf8("te_status"));
        te_status->setGeometry(QRect(270, 34, 301, 23));
        ALL_STOP = new QPushButton(tab_3);
        ALL_STOP->setObjectName(QString::fromUtf8("ALL_STOP"));
        ALL_STOP->setGeometry(QRect(1600, 10, 91, 91));
        ALL_STOP->setFont(font14);
        ALL_STOP->setStyleSheet(QString::fromUtf8("background-color: rgb(239, 41, 41);"));
        label_175 = new QLabel(tab_3);
        label_175->setObjectName(QString::fromUtf8("label_175"));
        label_175->setGeometry(QRect(1251, 21, 34, 17));
        label_175->setAlignment(Qt::AlignCenter);
        label_180 = new QLabel(tab_3);
        label_180->setObjectName(QString::fromUtf8("label_180"));
        label_180->setGeometry(QRect(1300, 20, 61, 17));
        label_180->setAlignment(Qt::AlignCenter);
        label_179 = new QLabel(tab_3);
        label_179->setObjectName(QString::fromUtf8("label_179"));
        label_179->setGeometry(QRect(1390, 20, 65, 17));
        label_179->setLayoutDirection(Qt::LeftToRight);
        label_179->setAlignment(Qt::AlignCenter);
        scenario->addTab(tab_3, QString());
        MainWindow->setCentralWidget(centralwidget);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        MainWindow->setStatusBar(statusbar);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 1767, 22));
        MainWindow->setMenuBar(menubar);

        retranslateUi(MainWindow);

        scenario->setCurrentIndex(1);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "Order Picking", nullptr));
        groupBox_5->setTitle(QApplication::translate("MainWindow", "Connection", nullptr));
        label_114->setText(QApplication::translate("MainWindow", "MOBILE", nullptr));
        label_115->setText(QApplication::translate("MainWindow", "GRIPPER", nullptr));
        label_116->setText(QApplication::translate("MainWindow", "RB5", nullptr));
        label_117->setText(QApplication::translate("MainWindow", "LIFT", nullptr));
        label_118->setText(QApplication::translate("MainWindow", "VISION", nullptr));
        label_119->setText(QApplication::translate("MainWindow", "WEBSOCKET", nullptr));
        label_148->setText(QApplication::translate("MainWindow", "BARCODE", nullptr));
        GE_MOVE_INFO->setTitle(QApplication::translate("MainWindow", "Move info", nullptr));
        label_120->setText(QApplication::translate("MainWindow", "MOBILE", nullptr));
        label_136->setText(QApplication::translate("MainWindow", "LIFT", nullptr));
        label_138->setText(QApplication::translate("MainWindow", "RB5", nullptr));
        label_177->setText(QApplication::translate("MainWindow", "PICK", nullptr));
        groupBox_7->setTitle(QApplication::translate("MainWindow", "LIFT", nullptr));
        bt_lift_homimg->setText(QApplication::translate("MainWindow", "Homing.", nullptr));
        bt_lift_stop->setText(QApplication::translate("MainWindow", "STOP", nullptr));
        bt_lift_pos_move->setText(QApplication::translate("MainWindow", "[mm] move", nullptr));
        cb_lift_dir_homing->setText(QApplication::translate("MainWindow", "CCW", nullptr));
        bt_lift_rpm_set->setText(QApplication::translate("MainWindow", "vel set", nullptr));
        bt_lift_top->setText(QApplication::translate("MainWindow", "TOP", nullptr));
        bt_lift_MID->setText(QApplication::translate("MainWindow", "MID", nullptr));
        bt_lift_Low->setText(QApplication::translate("MainWindow", "LOW", nullptr));
        groupBox_8->setTitle(QApplication::translate("MainWindow", "RB5", nullptr));
        bt_rb5_mode->setText(QApplication::translate("MainWindow", "Simul.", nullptr));
        bt_rb5_pause->setText(QApplication::translate("MainWindow", "Pause", nullptr));
        bt_rb5_resume->setText(QApplication::translate("MainWindow", "Resume", nullptr));
        bt_rb5_halt->setText(QApplication::translate("MainWindow", "Halt", nullptr));
        bt_rb5_pos_move->setText(QApplication::translate("MainWindow", "move", nullptr));
        lb_rb5_speed_percentage->setText(QApplication::translate("MainWindow", "0 %", nullptr));
        label_150->setText(QApplication::translate("MainWindow", "make mode", nullptr));
        label_151->setText(QApplication::translate("MainWindow", "Robot vel", nullptr));
        btn_pump_2->setText(QApplication::translate("MainWindow", "Pump on", nullptr));
        groupBox_9->setTitle(QApplication::translate("MainWindow", "SLAMNAV", nullptr));
        bt_sn_cmd_set->setText(QApplication::translate("MainWindow", "set", nullptr));
        label_162->setText(QApplication::translate("MainWindow", "Change SLAMNAVI MAP", nullptr));
        bt_amr_motor_init->setText(QApplication::translate("MainWindow", "Motor Init", nullptr));
        groupBox_10->setTitle(QApplication::translate("MainWindow", "VISION && GRIPPER", nullptr));
        lb_vision_capture_img->setText(QString());
#ifndef QT_NO_TOOLTIP
        bt_vision_cmd_capture->setToolTip(QString());
#endif // QT_NO_TOOLTIP
        bt_vision_cmd_capture->setText(QApplication::translate("MainWindow", "capture", nullptr));
        bt_gripper_cmd_move->setText(QApplication::translate("MainWindow", "move", nullptr));
        bt_png_change->setText(QApplication::translate("MainWindow", "change", nullptr));
        bt_zip->setText(QApplication::translate("MainWindow", "zip", nullptr));
        label_142->setText(QApplication::translate("MainWindow", "color -> gray IMG", nullptr));
        bt_get_map_from_mobile->setText(QApplication::translate("MainWindow", "get map data", nullptr));
        cb_get_map_data->setItemText(0, QApplication::translate("MainWindow", "map", nullptr));
        cb_get_map_data->setItemText(1, QApplication::translate("MainWindow", "ini", nullptr));

        label_161->setText(QApplication::translate("MainWindow", "Send MAP to AMR", nullptr));
        pb_file_choose->setText(QApplication::translate("MainWindow", "\355\214\214\354\235\274 \354\204\240\355\203\235", nullptr));
        groupBox_11->setTitle(QApplication::translate("MainWindow", "WEBSOCKET", nullptr));
        label_149->setText(QApplication::translate("MainWindow", "<html><head/><body><p><span style=\" font-size:26pt; font-weight:600; color:#729fcf;\">Integrated UI</span></p></body></html>", nullptr));
        bt_tcp_on->setText(QApplication::translate("MainWindow", "mobile tcp on", nullptr));
        label_141->setText(QApplication::translate("MainWindow", "stop", nullptr));
        label_166->setText(QApplication::translate("MainWindow", "RB_5", nullptr));
        label_154->setText(QApplication::translate("MainWindow", "y", nullptr));
        label_132->setText(QApplication::translate("MainWindow", " TF : ", nullptr));
        label_160->setText(QApplication::translate("MainWindow", "x", nullptr));
        label_155->setText(QApplication::translate("MainWindow", "z", nullptr));
        label_157->setText(QApplication::translate("MainWindow", "RY", nullptr));
        label_156->setText(QApplication::translate("MainWindow", "RZ", nullptr));
        label_158->setText(QApplication::translate("MainWindow", "RX", nullptr));
        label_140->setText(QString());
        move_x->setText(QApplication::translate("MainWindow", "X", nullptr));
        move_y->setText(QApplication::translate("MainWindow", "Y", nullptr));
        move_z->setText(QApplication::translate("MainWindow", "Z", nullptr));
        move_rx->setText(QApplication::translate("MainWindow", "RX", nullptr));
        move_ry->setText(QApplication::translate("MainWindow", "RY", nullptr));
        move_rz->setText(QApplication::translate("MainWindow", "RZ", nullptr));
#ifndef QT_NO_TOOLTIP
        bt_vision_cmd_capture_4->setToolTip(QString());
#endif // QT_NO_TOOLTIP
        bt_vision_cmd_capture_4->setText(QApplication::translate("MainWindow", "high get out", nullptr));
#ifndef QT_NO_TOOLTIP
        bt_TCP_Blend->setToolTip(QString());
#endif // QT_NO_TOOLTIP
        bt_TCP_Blend->setText(QApplication::translate("MainWindow", "TCP Blend", nullptr));
        label_159->setText(QApplication::translate("MainWindow", "Mobile Status", nullptr));
#ifndef QT_NO_TOOLTIP
        bt_low_get_out->setToolTip(QString());
#endif // QT_NO_TOOLTIP
        bt_low_get_out->setText(QApplication::translate("MainWindow", "low get out", nullptr));
#ifndef QT_NO_TOOLTIP
        bt_vision_water->setToolTip(QString());
#endif // QT_NO_TOOLTIP
        bt_vision_water->setText(QApplication::translate("MainWindow", "\354\210\230\353\217\204\354\225\240 more move", nullptr));
#ifndef QT_NO_TOOLTIP
        bt_avino_low_get_out_2->setToolTip(QString());
#endif // QT_NO_TOOLTIP
        bt_avino_low_get_out_2->setText(QApplication::translate("MainWindow", "\354\225\204\353\271\204\353\205\270 low get out", nullptr));
#ifndef QT_NO_TOOLTIP
        bt_vision_high_get_out->setToolTip(QString());
#endif // QT_NO_TOOLTIP
        bt_vision_high_get_out->setText(QApplication::translate("MainWindow", "high get out", nullptr));
#ifndef QT_NO_TOOLTIP
        bt_vision_rael_more_move->setToolTip(QString());
#endif // QT_NO_TOOLTIP
        bt_vision_rael_more_move->setText(QApplication::translate("MainWindow", "\353\235\274\354\227\230 more move", nullptr));
#ifndef QT_NO_TOOLTIP
        bt_low_get_out_2->setToolTip(QString());
#endif // QT_NO_TOOLTIP
        bt_low_get_out_2->setText(QApplication::translate("MainWindow", "low get out", nullptr));
        label_139->setText(QApplication::translate("MainWindow", "move", nullptr));
        label_181->setText(QApplication::translate("MainWindow", "Scenario", nullptr));
#ifndef QT_NO_TOOLTIP
        bt_cobot_move2object_approach->setToolTip(QString());
#endif // QT_NO_TOOLTIP
        bt_cobot_move2object_approach->setText(QApplication::translate("MainWindow", "approach", nullptr));
#ifndef QT_NO_TOOLTIP
        bt_cobot_move2object->setToolTip(QString());
#endif // QT_NO_TOOLTIP
        bt_cobot_move2object->setText(QApplication::translate("MainWindow", "move", nullptr));
#ifndef QT_NO_TOOLTIP
        bt_cobot_push->setToolTip(QString());
#endif // QT_NO_TOOLTIP
        bt_cobot_push->setText(QApplication::translate("MainWindow", "more move", nullptr));
        scenario->setTabText(scenario->indexOf(tab), QApplication::translate("MainWindow", "Integrated UI", nullptr));
        GB_NETWORK->setTitle(QApplication::translate("MainWindow", "Network Connection", nullptr));
        LE_IP->setText(QApplication::translate("MainWindow", "192.168.2.125", nullptr));
        label_8->setText(QApplication::translate("MainWindow", "Address", nullptr));
        textCmdPort->setText(QApplication::translate("MainWindow", "5000", nullptr));
        label_10->setText(QApplication::translate("MainWindow", "Data Port", nullptr));
        label_11->setText(QApplication::translate("MainWindow", "Cmd Port", nullptr));
        textDataPort->setText(QApplication::translate("MainWindow", "5001", nullptr));
        BTN_CONNECT_COM->setText(QApplication::translate("MainWindow", "Connect", nullptr));
        BTN_CONNECT_DATA->setText(QApplication::translate("MainWindow", "Connect", nullptr));
        groupBox_4->setTitle(QApplication::translate("MainWindow", "Cobot Initialize", nullptr));
        BTN_COBOT_INIT->setText(QApplication::translate("MainWindow", "Cobot Init", nullptr));
        label_12->setText(QApplication::translate("MainWindow", "1. Power Set", nullptr));
        label_13->setText(QApplication::translate("MainWindow", "2. Device Set", nullptr));
        label_14->setText(QApplication::translate("MainWindow", "3. System Set", nullptr));
        label_15->setText(QApplication::translate("MainWindow", "4. Robot Init", nullptr));
        groupBox_6->setTitle(QApplication::translate("MainWindow", "Robot Data Viewer", nullptr));
        BTN_CLEAR->setText(QApplication::translate("MainWindow", "Clear", nullptr));
        label_16->setText(QApplication::translate("MainWindow", "Base Speed", nullptr));
        BTN_MOTION_RESUME->setText(QApplication::translate("MainWindow", "Motion\n"
"Resume", nullptr));
        BTN_MOVE_JOINT_READY_GRIPPER->setText(QApplication::translate("MainWindow", "Move joint\n"
"Ready to Gripper", nullptr));
        BTN_MOTION_HALT->setText(QApplication::translate("MainWindow", "Motion\n"
"Halt", nullptr));
        label_25->setText(QApplication::translate("MainWindow", "D3", nullptr));
        label_36->setText(QApplication::translate("MainWindow", "D12", nullptr));
        label_28->setText(QApplication::translate("MainWindow", "D6", nullptr));
        label_24->setText(QApplication::translate("MainWindow", "D1", nullptr));
        label_37->setText(QApplication::translate("MainWindow", "D13", nullptr));
        label_38->setText(QApplication::translate("MainWindow", "D14", nullptr));
        label_21->setText(QApplication::translate("MainWindow", "D4", nullptr));
        label_26->setText(QApplication::translate("MainWindow", "D2", nullptr));
        label_33->setText(QApplication::translate("MainWindow", "D9", nullptr));
        label_22->setText(QApplication::translate("MainWindow", "D5", nullptr));
        label_32->setText(QApplication::translate("MainWindow", "D8", nullptr));
        label_34->setText(QApplication::translate("MainWindow", "D10", nullptr));
        label_39->setText(QApplication::translate("MainWindow", "D15", nullptr));
        label_35->setText(QApplication::translate("MainWindow", "D11", nullptr));
        label_23->setText(QApplication::translate("MainWindow", "D0", nullptr));
        label_31->setText(QApplication::translate("MainWindow", "D7", nullptr));
        label_40->setText(QApplication::translate("MainWindow", "on", nullptr));
        label_41->setText(QApplication::translate("MainWindow", "off", nullptr));
        label_42->setText(QApplication::translate("MainWindow", "on", nullptr));
        label_43->setText(QApplication::translate("MainWindow", "off", nullptr));
        label_18->setText(QApplication::translate("MainWindow", "Joint Encoder", nullptr));
        label_19->setText(QApplication::translate("MainWindow", "J5", nullptr));
        label_20->setText(QApplication::translate("MainWindow", "J6", nullptr));
        label_27->setText(QApplication::translate("MainWindow", "J1", nullptr));
        label_29->setText(QApplication::translate("MainWindow", "J2", nullptr));
        label_30->setText(QApplication::translate("MainWindow", "J4", nullptr));
        label_44->setText(QApplication::translate("MainWindow", "J3", nullptr));
        label_71->setText(QApplication::translate("MainWindow", "A1", nullptr));
        label_74->setText(QApplication::translate("MainWindow", "A0", nullptr));
        label_96->setText(QApplication::translate("MainWindow", "off", nullptr));
        label_82->setText(QApplication::translate("MainWindow", "D1", nullptr));
        label_93->setText(QApplication::translate("MainWindow", "D0", nullptr));
        label_95->setText(QApplication::translate("MainWindow", "on", nullptr));
        label_45->setText(QApplication::translate("MainWindow", "robot state", nullptr));
        label_101->setText(QApplication::translate("MainWindow", "tool out voltage", nullptr));
        LE_PG_MODE_SIMULATION->setText(QApplication::translate("MainWindow", "Simulation", nullptr));
        label_97->setText(QApplication::translate("MainWindow", "off", nullptr));
        label_83->setText(QApplication::translate("MainWindow", "D1", nullptr));
        label_94->setText(QApplication::translate("MainWindow", "D0", nullptr));
        label_98->setText(QApplication::translate("MainWindow", "on", nullptr));
        label_84->setText(QApplication::translate("MainWindow", "status", nullptr));
        label_46->setText(QApplication::translate("MainWindow", "digital out", nullptr));
        label_47->setText(QApplication::translate("MainWindow", "D3", nullptr));
        label_48->setText(QApplication::translate("MainWindow", "D12", nullptr));
        label_49->setText(QApplication::translate("MainWindow", "D6", nullptr));
        label_50->setText(QApplication::translate("MainWindow", "D1", nullptr));
        label_51->setText(QApplication::translate("MainWindow", "D13", nullptr));
        label_52->setText(QApplication::translate("MainWindow", "D14", nullptr));
        label_53->setText(QApplication::translate("MainWindow", "D4", nullptr));
        label_54->setText(QApplication::translate("MainWindow", "D2", nullptr));
        label_55->setText(QApplication::translate("MainWindow", "D9", nullptr));
        label_56->setText(QApplication::translate("MainWindow", "D5", nullptr));
        label_57->setText(QApplication::translate("MainWindow", "D8", nullptr));
        label_58->setText(QApplication::translate("MainWindow", "D10", nullptr));
        label_59->setText(QApplication::translate("MainWindow", "D15", nullptr));
        label_60->setText(QApplication::translate("MainWindow", "D11", nullptr));
        label_61->setText(QApplication::translate("MainWindow", "D0", nullptr));
        label_62->setText(QApplication::translate("MainWindow", "D7", nullptr));
        label_63->setText(QApplication::translate("MainWindow", "on", nullptr));
        label_64->setText(QApplication::translate("MainWindow", "off", nullptr));
        label_65->setText(QApplication::translate("MainWindow", "on", nullptr));
        label_66->setText(QApplication::translate("MainWindow", "off", nullptr));
        label_99->setText(QApplication::translate("MainWindow", "tool digital in", nullptr));
        LE_ROBOT_STATUS_SOS->setText(QApplication::translate("MainWindow", "sos", nullptr));
        LE_ROBOT_STATE_IDLE->setText(QApplication::translate("MainWindow", "Idle", nullptr));
        label_67->setText(QApplication::translate("MainWindow", "digital in", nullptr));
        label_100->setText(QApplication::translate("MainWindow", "tool digital out", nullptr));
        label_68->setText(QApplication::translate("MainWindow", "run mode", nullptr));
        label_69->setText(QApplication::translate("MainWindow", "RY", nullptr));
        label_70->setText(QApplication::translate("MainWindow", "RZ", nullptr));
        label_72->setText(QApplication::translate("MainWindow", "X", nullptr));
        label_73->setText(QApplication::translate("MainWindow", "Y", nullptr));
        label_75->setText(QApplication::translate("MainWindow", "RX", nullptr));
        label_76->setText(QApplication::translate("MainWindow", "Z", nullptr));
        LE_ROBOT_STATUS_EMS->setText(QApplication::translate("MainWindow", "ems", nullptr));
        LE_ROBOT_STATE_MOVING->setText(QApplication::translate("MainWindow", "Moving", nullptr));
        LE_ROBOT_STATUS_PAUSED->setText(QApplication::translate("MainWindow", "paused", nullptr));
        label_77->setText(QApplication::translate("MainWindow", "analog in", nullptr));
        label_78->setText(QApplication::translate("MainWindow", "TCP Reference", nullptr));
        LE_ROBOT_STATUS_SELF_COLLISION->setText(QApplication::translate("MainWindow", "self collision", nullptr));
        LE_PG_MODE_REAL->setText(QApplication::translate("MainWindow", "Real", nullptr));
        label_79->setText(QApplication::translate("MainWindow", "A2", nullptr));
        label_80->setText(QApplication::translate("MainWindow", "A1", nullptr));
        label_81->setText(QApplication::translate("MainWindow", "A3", nullptr));
        label_85->setText(QApplication::translate("MainWindow", "A0", nullptr));
        LE_ROBOT_STATUS_EXT_COLLISION->setText(QApplication::translate("MainWindow", "ext. collision", nullptr));
        label_86->setText(QApplication::translate("MainWindow", "Joint Reference", nullptr));
        label_87->setText(QApplication::translate("MainWindow", "analog out", nullptr));
        label_88->setText(QApplication::translate("MainWindow", "tool analog in", nullptr));
        LE_ROBOT_STATUS_TEACHING->setText(QApplication::translate("MainWindow", "teaching", nullptr));
        label_89->setText(QApplication::translate("MainWindow", "A2", nullptr));
        label_90->setText(QApplication::translate("MainWindow", "A1", nullptr));
        label_91->setText(QApplication::translate("MainWindow", "A3", nullptr));
        label_92->setText(QApplication::translate("MainWindow", "A0", nullptr));
        label_102->setText(QApplication::translate("MainWindow", "J5", nullptr));
        label_103->setText(QApplication::translate("MainWindow", "J6", nullptr));
        label_104->setText(QApplication::translate("MainWindow", "J1", nullptr));
        label_105->setText(QApplication::translate("MainWindow", "J2", nullptr));
        label_106->setText(QApplication::translate("MainWindow", "J4", nullptr));
        label_107->setText(QApplication::translate("MainWindow", "J3", nullptr));
        label_108->setText(QApplication::translate("MainWindow", "0V", nullptr));
        label_109->setText(QApplication::translate("MainWindow", "12V", nullptr));
        label_110->setText(QApplication::translate("MainWindow", "24V", nullptr));
        BTN_COLLISION_RESUME->setText(QApplication::translate("MainWindow", "Collision\n"
"Resume", nullptr));
        BTN_MOTION_PAUSE->setText(QApplication::translate("MainWindow", "Motion\n"
"Pause", nullptr));
        GB_NETWORK_2->setTitle(QApplication::translate("MainWindow", "Mode Change", nullptr));
        BTN_MODE_REAL->setText(QApplication::translate("MainWindow", "Real", nullptr));
        BTN_MODE_SIMULATION->setText(QApplication::translate("MainWindow", "Simulation", nullptr));
        commandLinkButton->setText(QApplication::translate("MainWindow", "CommandLinkButton", nullptr));
        LB_BASE_SPEED->setText(QApplication::translate("MainWindow", "0%", nullptr));
        BTN_MOVE_JOINT_GRAPS_GRIPPER->setText(QApplication::translate("MainWindow", "Move joint\n"
"Graps to Gripper", nullptr));
        label_137->setText(QApplication::translate("MainWindow", "<html><head/><body><p><span style=\" font-size:24pt; font-weight:600; color:#729fcf;\">RB_5</span></p><p><br/></p></body></html>", nullptr));
        BTN_MOVE_JOINT_INIT_3->setText(QApplication::translate("MainWindow", "Move joint\n"
"adust pos", nullptr));
        BTN_MOVE_JOINT_INIT_4->setText(QApplication::translate("MainWindow", "Move joint\n"
"readyG2pos", nullptr));
        BTN_MOVE_JOINT_INIT_5->setText(QApplication::translate("MainWindow", "Move joint\n"
"G2 pos", nullptr));
        btn_pump->setText(QApplication::translate("MainWindow", "Pump on", nullptr));
        BTN_MOVE_JOINT_Rotation->setText(QApplication::translate("MainWindow", "Move joint\n"
"Rotaion", nullptr));
        label_9->setText(QApplication::translate("MainWindow", "Joint", nullptr));
        label_17->setText(QApplication::translate("MainWindow", "TCP", nullptr));
        BTN_TCP_MOTION_TEST->setText(QApplication::translate("MainWindow", "TCP\n"
"Motion Test", nullptr));
        BTN_JOINT_MOTION_TEST->setText(QApplication::translate("MainWindow", "Joint\n"
"Motion Test", nullptr));
        BTN_MOVE_JOINT_POP->setText(QApplication::translate("MainWindow", "Move joint\n"
"POP", nullptr));
        BTN_MOVE_JOINT_BOX_CENTER->setText(QApplication::translate("MainWindow", "Move box Center", nullptr));
        pb_reload->setText(QString());
        BTN_MOVE_JOINT_BOX_CENTER_KETI->setText(QApplication::translate("MainWindow", "Move box Center\n"
"KETI", nullptr));
        BTN_MOVEL_rel->setText(QApplication::translate("MainWindow", "Move l REL", nullptr));
        BTN_quick_return->setText(QApplication::translate("MainWindow", "QUICK\n"
"RETURN", nullptr));
        BTN_MOVE_JOINT_HIGH->setText(QApplication::translate("MainWindow", "Move joint\n"
"HIGH\n"
"Vision pos", nullptr));
        BTN_MOVE_JOINT_MID->setText(QApplication::translate("MainWindow", "Move joint\n"
"Mid\n"
"Vision pos", nullptr));
        BTN_MOVE_JOINT_LOW->setText(QApplication::translate("MainWindow", "Move joint\n"
"LOW\n"
"Vision pos", nullptr));
        BTN_MOVE_JOINT_MID_LEFT->setText(QApplication::translate("MainWindow", "Move joint\n"
"LEFT\n"
"putdown mid pos", nullptr));
        BTN_MOVE_JOINT_MID_RIGHT->setText(QApplication::translate("MainWindow", "Move joint\n"
"RIGHT\n"
"putdown mid pos", nullptr));
        BTN_MOVE_JOINT_LOW_LEFT->setText(QApplication::translate("MainWindow", "Move joint\n"
"LEFT\n"
"putdown low pos", nullptr));
        BTN_MOVE_JOINT_LOW_RIGHT->setText(QApplication::translate("MainWindow", "Move joint\n"
"RIGHT\n"
"putdown low pos", nullptr));
        BTN_RETURN_MOVE_JOINT_BOX_CENTER->setText(QApplication::translate("MainWindow", "RETRUN_box_cent", nullptr));
        scenario->setTabText(scenario->indexOf(rb_5), QApplication::translate("MainWindow", "rb_5", nullptr));
        groupBox_3->setTitle(QApplication::translate("MainWindow", "Status", nullptr));
        label_7->setText(QApplication::translate("MainWindow", "temp", nullptr));
        label->setText(QApplication::translate("MainWindow", "RPM", nullptr));
        label_4->setText(QApplication::translate("MainWindow", "control", nullptr));
        label_6->setText(QApplication::translate("MainWindow", "RPM (std)", nullptr));
        label_5->setText(QApplication::translate("MainWindow", "Ampere [A] ", nullptr));
        label_3->setText(QApplication::translate("MainWindow", "position [mm]", nullptr));
        te_data->setHtml(QApplication::translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Ubuntu'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /></p></body></html>", nullptr));
        bt_test->setText(QApplication::translate("MainWindow", "test", nullptr));
        groupBox->setTitle(QApplication::translate("MainWindow", "Position control", nullptr));
        ch_dir->setText(QApplication::translate("MainWindow", "CCW", nullptr));
        bt_homing->setText(QApplication::translate("MainWindow", "homing", nullptr));
        bt_move_pos->setText(QApplication::translate("MainWindow", "[mm]move", nullptr));
        bt_set_maxVel_pos->setText(QApplication::translate("MainWindow", "MAX vel set", nullptr));
        bt_auto_homing->setText(QApplication::translate("MainWindow", "AUTO\n"
"homing", nullptr));
        groupBox_2->setTitle(QApplication::translate("MainWindow", "RPM", nullptr));
        bt_move_rpm->setText(QApplication::translate("MainWindow", "move", nullptr));
        bt_rpmZero->setText(QApplication::translate("MainWindow", "EMO", nullptr));
        bt_connect->setText(QApplication::translate("MainWindow", "connect", nullptr));
        label_174->setText(QApplication::translate("MainWindow", "<html><head/><body><p><span style=\" font-size:24pt; font-weight:600; color:#729fcf;\">Lift</span></p><p><br/></p></body></html>", nullptr));
        scenario->setTabText(scenario->indexOf(lift_motor), QApplication::translate("MainWindow", "lift motor", nullptr));
        GR_KITECH->setTitle(QApplication::translate("MainWindow", "GRIPPER", nullptr));
        BTN_GRIPPER_CMD_WRITE->setText(QApplication::translate("MainWindow", "Write", nullptr));
        BTN_GRIPPER_ALLFOLD->setText(QApplication::translate("MainWindow", "Grasp", nullptr));
        BTN_RB5_CALC_GRAPS_POINT->setText(QApplication::translate("MainWindow", "calc\n"
"graps point", nullptr));
        BTN_RB5_MOVE_GRAPS_POINT->setText(QApplication::translate("MainWindow", "move\n"
"graps point", nullptr));
        BTN_GRIPPER_VISION->setText(QApplication::translate("MainWindow", "Vision", nullptr));
        label_127->setText(QApplication::translate("MainWindow", "Mode", nullptr));
        label_128->setText(QApplication::translate("MainWindow", "Type", nullptr));
        label_129->setText(QApplication::translate("MainWindow", "Value", nullptr));
        label_130->setText(QApplication::translate("MainWindow", "Suction", nullptr));
        CB_GRIPPER_GRAPS->setItemText(0, QApplication::translate("MainWindow", "g", nullptr));
        CB_GRIPPER_GRAPS->setItemText(1, QApplication::translate("MainWindow", "p", nullptr));
        CB_GRIPPER_GRAPS->setItemText(2, QApplication::translate("MainWindow", "m", nullptr));
        CB_GRIPPER_GRAPS->setItemText(3, QApplication::translate("MainWindow", "i", nullptr));
        CB_GRIPPER_GRAPS->setItemText(4, QApplication::translate("MainWindow", "t", nullptr));
        CB_GRIPPER_GRAPS->setItemText(5, QApplication::translate("MainWindow", "e", nullptr));

        CB_GRIPPER_POWER->setItemText(0, QApplication::translate("MainWindow", "0", nullptr));
        CB_GRIPPER_POWER->setItemText(1, QApplication::translate("MainWindow", "1", nullptr));
        CB_GRIPPER_POWER->setItemText(2, QApplication::translate("MainWindow", "2", nullptr));
        CB_GRIPPER_POWER->setItemText(3, QApplication::translate("MainWindow", "3", nullptr));
        CB_GRIPPER_POWER->setItemText(4, QApplication::translate("MainWindow", "4", nullptr));
        CB_GRIPPER_POWER->setItemText(5, QApplication::translate("MainWindow", "5", nullptr));
        CB_GRIPPER_POWER->setItemText(6, QApplication::translate("MainWindow", "6", nullptr));
        CB_GRIPPER_POWER->setItemText(7, QApplication::translate("MainWindow", "7", nullptr));
        CB_GRIPPER_POWER->setItemText(8, QApplication::translate("MainWindow", "8", nullptr));
        CB_GRIPPER_POWER->setItemText(9, QApplication::translate("MainWindow", "9", nullptr));

        BTN_GRIPPER_GRASP->setText(QApplication::translate("MainWindow", "only Grasp", nullptr));
        CB_GRIPPER_GE_MODE->setItemText(0, QApplication::translate("MainWindow", "G", nullptr));
        CB_GRIPPER_GE_MODE->setItemText(1, QApplication::translate("MainWindow", "R", nullptr));

        CB_GRIPPER_GE_GRAPS->setItemText(0, QApplication::translate("MainWindow", "g", nullptr));
        CB_GRIPPER_GE_GRAPS->setItemText(1, QApplication::translate("MainWindow", "p", nullptr));
        CB_GRIPPER_GE_GRAPS->setItemText(2, QApplication::translate("MainWindow", "m", nullptr));
        CB_GRIPPER_GE_GRAPS->setItemText(3, QApplication::translate("MainWindow", "i", nullptr));
        CB_GRIPPER_GE_GRAPS->setItemText(4, QApplication::translate("MainWindow", "t", nullptr));
        CB_GRIPPER_GE_GRAPS->setItemText(5, QApplication::translate("MainWindow", "e", nullptr));

        CB_GRIPPER_GE_POWER->setItemText(0, QApplication::translate("MainWindow", "0", nullptr));
        CB_GRIPPER_GE_POWER->setItemText(1, QApplication::translate("MainWindow", "1", nullptr));
        CB_GRIPPER_GE_POWER->setItemText(2, QApplication::translate("MainWindow", "2", nullptr));
        CB_GRIPPER_GE_POWER->setItemText(3, QApplication::translate("MainWindow", "3", nullptr));
        CB_GRIPPER_GE_POWER->setItemText(4, QApplication::translate("MainWindow", "4", nullptr));
        CB_GRIPPER_GE_POWER->setItemText(5, QApplication::translate("MainWindow", "5", nullptr));
        CB_GRIPPER_GE_POWER->setItemText(6, QApplication::translate("MainWindow", "6", nullptr));
        CB_GRIPPER_GE_POWER->setItemText(7, QApplication::translate("MainWindow", "7", nullptr));
        CB_GRIPPER_GE_POWER->setItemText(8, QApplication::translate("MainWindow", "8", nullptr));
        CB_GRIPPER_GE_POWER->setItemText(9, QApplication::translate("MainWindow", "9", nullptr));

        CB_GRIPPER_GE_SUCTION_POWER->setItemText(0, QApplication::translate("MainWindow", "ON", nullptr));
        CB_GRIPPER_GE_SUCTION_POWER->setItemText(1, QApplication::translate("MainWindow", "OFF", nullptr));

        CB_GRIPPER_GE_CMD_WRITE->setText(QApplication::translate("MainWindow", "Write", nullptr));
        Pinch12_FOLD->setText(QApplication::translate("MainWindow", "Pinch12_FOLD", nullptr));
        label_152->setText(QApplication::translate("MainWindow", "REDY POWER", nullptr));
        label_153->setText(QApplication::translate("MainWindow", "GRASP", nullptr));
        textBrowser->setHtml(QApplication::translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Arial'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">R : \354\244\200\353\271\204</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">G : 3\354\206\220\352\260\200\353\235\275</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">P : \355\217\211\355\226\211</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">m : 1, 3 \354\206\220\352\260\200\353\235\275 \354\202\254\354\232\251</p>\n"
"<p st"
                        "yle=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">i : 1, 2 \354\206\220\352\260\200\353\235\275 \354\202\254\354\232\251</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">t : 2, 3 \354\206\220\352\260\200\353\235\275 \354\202\254\354\232\251</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">m : \354\233\200\354\274\234\354\236\241\352\270\260</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">s : \355\235\241\354\236\205\354\236\220\354\204\270</p></body></html>", nullptr));
        textBrowser_2->setHtml(QApplication::translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Arial'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">&quot;S1&quot; : 3 finger \355\214\214\354\247\200\354\227\254\353\266\200 \355\214\220\353\213\250</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">&quot;S2&quot; : parallel \355\214\214\354\247\200\354\227\254\353\266\200 \355\214\220\353\213\250</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">&quot;S3&quot; : 1,3\353\262\210 \354\206\220\352\260\200\353\235\275 \352\270\260\354\244\200\354\234\274\353\241"
                        "\234 \355\214\220\353\213\250</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">&quot;S4&quot; : 1,2\353\262\210 \354\206\220\352\260\200\353\235\275 \352\270\260\354\244\200\354\234\274\353\241\234 \355\214\220\353\213\250</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">&quot;S5&quot; : 2,3\353\262\210 \354\206\220\352\260\200\353\235\275 \352\270\260\354\244\200\354\234\274\353\241\234 \355\214\220\353\213\250</p></body></html>", nullptr));
        CB_GRIPPER_recog->setItemText(0, QApplication::translate("MainWindow", "S1", nullptr));
        CB_GRIPPER_recog->setItemText(1, QApplication::translate("MainWindow", "S2", nullptr));
        CB_GRIPPER_recog->setItemText(2, QApplication::translate("MainWindow", "S3", nullptr));
        CB_GRIPPER_recog->setItemText(3, QApplication::translate("MainWindow", "S4", nullptr));
        CB_GRIPPER_recog->setItemText(4, QApplication::translate("MainWindow", "S5", nullptr));

        BTN_GRIPPER_recog_WRITE->setText(QApplication::translate("MainWindow", "Write", nullptr));
        BTN_GRIPPER_SUCTION->setText(QApplication::translate("MainWindow", "Suction", nullptr));
        BTN_GRIPPER_OPEN->setText(QApplication::translate("MainWindow", "Open", nullptr));
        label_131->setText(QApplication::translate("MainWindow", "keti", nullptr));
        lb_keti_point->setText(QString());
        label_133->setText(QApplication::translate("MainWindow", "kitech", nullptr));
        label_134->setText(QApplication::translate("MainWindow", "<html><head/><body><p><span style=\" font-size:24pt; font-weight:600; color:#729fcf;\">vision &amp; gripper</span></p><p><br/></p></body></html>", nullptr));
        label_135->setText(QApplication::translate("MainWindow", "<html><head/><body><p>keti \354\227\220\354\204\234 \353\260\233\354\235\204 \354\230\201\354\203\201 \353\215\260\354\235\264\355\204\260</p></body></html>", nullptr));
        BTN_CONNECT_GRIPPER->setText(QApplication::translate("MainWindow", "conn.", nullptr));
        LE_KETI_IMG->setText(QApplication::translate("MainWindow", "IMG", nullptr));
        pb_shutter->setText(QApplication::translate("MainWindow", "Shutter", nullptr));
        scenario->setTabText(scenario->indexOf(tab1), QApplication::translate("MainWindow", "vison && gripper", nullptr));
        label_111->setText(QApplication::translate("MainWindow", "<html><head/><body><p><span style=\" font-size:14pt; font-weight:600; color:#888a85;\">left</span></p></body></html>", nullptr));
        label_112->setText(QApplication::translate("MainWindow", "<html><head/><body><p><span style=\" font-size:14pt; font-weight:600; color:#888a85;\">right</span></p></body></html>", nullptr));
        write->setText(QApplication::translate("MainWindow", "write", nullptr));
        bt_usb2_dis->setText(QApplication::translate("MainWindow", "disconnect", nullptr));
        bt_usb2_con->setText(QApplication::translate("MainWindow", "connect", nullptr));
        bt_usb1_dis->setText(QApplication::translate("MainWindow", "disconnect", nullptr));
        bt_usb1_con->setText(QApplication::translate("MainWindow", "connect", nullptr));
        label_113->setText(QApplication::translate("MainWindow", "<html><head/><body><p><span style=\" font-size:24pt; font-weight:600; color:#729fcf;\">box barcode reader</span></p></body></html>", nullptr));
        BTN_MOVE_TOTAL->setText(QApplication::translate("MainWindow", "\354\203\201 / \354\244\221 task1", nullptr));
        BTN_MOVE_TOTAL_2->setText(QApplication::translate("MainWindow", "\354\244\221 task2", nullptr));
        BTN_MOVE_TOTAL_3->setText(QApplication::translate("MainWindow", "\354\203\201 / \354\203\201 task3", nullptr));
        BTN_MOVE_TOTAL_4->setText(QApplication::translate("MainWindow", "\354\244\221 / \354\244\221 task4", nullptr));
        BTN_show->setText(QApplication::translate("MainWindow", "\354\213\234\354\227\260\354\232\251 FSM", nullptr));
        scenario->setTabText(scenario->indexOf(barcode), QApplication::translate("MainWindow", "barcode", nullptr));
        label_143->setText(QApplication::translate("MainWindow", "<html><head/><body><p><span style=\" font-size:10pt; font-weight:600;\">\353\260\233\354\235\200\353\251\224\354\213\234\354\247\200</span></p></body></html>", nullptr));
        te_label->setText(QApplication::translate("MainWindow", "\355\201\264\353\235\274\354\235\264\354\226\270\355\212\270 \353\257\270\354\240\221\354\206\215", nullptr));
        label_144->setText(QApplication::translate("MainWindow", "<html><head/><body><p><span style=\" font-size:10pt; font-weight:600;\">\353\263\264\353\202\270\353\251\224\354\213\234\354\247\200</span></p></body></html>", nullptr));
        label_145->setText(QApplication::translate("MainWindow", "<html><head/><body><p><span style=\" font-size:24pt; font-weight:600; color:#729fcf;\">websocket</span></p></body></html>", nullptr));
        label_146->setText(QApplication::translate("MainWindow", "<html><head/><body><p><br/></p></body></html>", nullptr));
        label_147->setText(QApplication::translate("MainWindow", "<html><head/><body><p><img src=\":/img/resource/rainbow.png\"/></p></body></html>", nullptr));
        scenario->setTabText(scenario->indexOf(tab_2), QApplication::translate("MainWindow", "websocket", nullptr));
        bt_save_all_annot->setText(QApplication::translate("MainWindow", "save ALL annotation", nullptr));
        label_165->setText(QApplication::translate("MainWindow", "<html><head/><body><p><span style=\" font-size:24pt; font-weight:600; color:#729fcf;\">Scenario auto</span></p></body></html>", nullptr));
        bt_save_annot->setText(QApplication::translate("MainWindow", "add \n"
"annotation", nullptr));
        te_scen->setHtml(QApplication::translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Ubuntu'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /></p></body></html>", nullptr));
        CB_GRIPPER_GRAPS_3->setItemText(0, QApplication::translate("MainWindow", "g", nullptr));
        CB_GRIPPER_GRAPS_3->setItemText(1, QApplication::translate("MainWindow", "p", nullptr));
        CB_GRIPPER_GRAPS_3->setItemText(2, QApplication::translate("MainWindow", "m", nullptr));
        CB_GRIPPER_GRAPS_3->setItemText(3, QApplication::translate("MainWindow", "i", nullptr));
        CB_GRIPPER_GRAPS_3->setItemText(4, QApplication::translate("MainWindow", "t", nullptr));
        CB_GRIPPER_GRAPS_3->setItemText(5, QApplication::translate("MainWindow", "e", nullptr));

        CB_GRIPPER_POWER_3->setItemText(0, QApplication::translate("MainWindow", "0", nullptr));
        CB_GRIPPER_POWER_3->setItemText(1, QApplication::translate("MainWindow", "1", nullptr));
        CB_GRIPPER_POWER_3->setItemText(2, QApplication::translate("MainWindow", "2", nullptr));
        CB_GRIPPER_POWER_3->setItemText(3, QApplication::translate("MainWindow", "3", nullptr));
        CB_GRIPPER_POWER_3->setItemText(4, QApplication::translate("MainWindow", "4", nullptr));
        CB_GRIPPER_POWER_3->setItemText(5, QApplication::translate("MainWindow", "5", nullptr));
        CB_GRIPPER_POWER_3->setItemText(6, QApplication::translate("MainWindow", "6", nullptr));
        CB_GRIPPER_POWER_3->setItemText(7, QApplication::translate("MainWindow", "7", nullptr));
        CB_GRIPPER_POWER_3->setItemText(8, QApplication::translate("MainWindow", "8", nullptr));
        CB_GRIPPER_POWER_3->setItemText(9, QApplication::translate("MainWindow", "9", nullptr));

        label_167->setText(QApplication::translate("MainWindow", "Mode", nullptr));
        label_168->setText(QApplication::translate("MainWindow", "Value", nullptr));
        CB_GRIPPER_GE_MODE_3->setItemText(0, QApplication::translate("MainWindow", "R", nullptr));
        CB_GRIPPER_GE_MODE_3->setItemText(1, QApplication::translate("MainWindow", "G", nullptr));

        CB_GRIPPER_GE_GRAPS_3->setItemText(0, QApplication::translate("MainWindow", "g", nullptr));
        CB_GRIPPER_GE_GRAPS_3->setItemText(1, QApplication::translate("MainWindow", "p", nullptr));
        CB_GRIPPER_GE_GRAPS_3->setItemText(2, QApplication::translate("MainWindow", "m", nullptr));
        CB_GRIPPER_GE_GRAPS_3->setItemText(3, QApplication::translate("MainWindow", "i", nullptr));
        CB_GRIPPER_GE_GRAPS_3->setItemText(4, QApplication::translate("MainWindow", "t", nullptr));
        CB_GRIPPER_GE_GRAPS_3->setItemText(5, QApplication::translate("MainWindow", "e", nullptr));

        CB_GRIPPER_GE_POWER_3->setItemText(0, QApplication::translate("MainWindow", "0", nullptr));
        CB_GRIPPER_GE_POWER_3->setItemText(1, QApplication::translate("MainWindow", "1", nullptr));
        CB_GRIPPER_GE_POWER_3->setItemText(2, QApplication::translate("MainWindow", "2", nullptr));
        CB_GRIPPER_GE_POWER_3->setItemText(3, QApplication::translate("MainWindow", "3", nullptr));
        CB_GRIPPER_GE_POWER_3->setItemText(4, QApplication::translate("MainWindow", "4", nullptr));
        CB_GRIPPER_GE_POWER_3->setItemText(5, QApplication::translate("MainWindow", "5", nullptr));
        CB_GRIPPER_GE_POWER_3->setItemText(6, QApplication::translate("MainWindow", "6", nullptr));
        CB_GRIPPER_GE_POWER_3->setItemText(7, QApplication::translate("MainWindow", "7", nullptr));
        CB_GRIPPER_GE_POWER_3->setItemText(8, QApplication::translate("MainWindow", "8", nullptr));
        CB_GRIPPER_GE_POWER_3->setItemText(9, QApplication::translate("MainWindow", "9", nullptr));

        CB_GRIPPER_GE_SUCTION_POWER_3->setItemText(0, QApplication::translate("MainWindow", "ON", nullptr));
        CB_GRIPPER_GE_SUCTION_POWER_3->setItemText(1, QApplication::translate("MainWindow", "OFF", nullptr));

        label_169->setText(QApplication::translate("MainWindow", "Type", nullptr));
        label_172->setText(QApplication::translate("MainWindow", "Suction", nullptr));
        bt_delete_annot->setText(QApplication::translate("MainWindow", "erase \n"
"annotation", nullptr));
        bt_order->setText(QApplication::translate("MainWindow", "pick", nullptr));
        label_173->setText(QApplication::translate("MainWindow", "order :", nullptr));
        label_164->setText(QApplication::translate("MainWindow", "object ID : ", nullptr));
        label_170->setText(QApplication::translate("MainWindow", "REDY \n"
"GRASP :", nullptr));
        label_171->setText(QApplication::translate("MainWindow", "GRASP :", nullptr));
        cb_obj_direction->setItemText(0, QApplication::translate("MainWindow", "left", nullptr));
        cb_obj_direction->setItemText(1, QApplication::translate("MainWindow", "right", nullptr));

        label_163->setText(QApplication::translate("MainWindow", "shelf ID : ", nullptr));
        bt_save_annot_2->setText(QApplication::translate("MainWindow", "add \n"
"annotation", nullptr));
        bt_save_all_annot_2->setText(QApplication::translate("MainWindow", "save ALL annotation", nullptr));
        bt_delete_annot_2->setText(QApplication::translate("MainWindow", "erase \n"
"annotation", nullptr));
        cb_obj_direction_2->setItemText(0, QApplication::translate("MainWindow", "left", nullptr));
        cb_obj_direction_2->setItemText(1, QApplication::translate("MainWindow", "right", nullptr));

        label_178->setText(QApplication::translate("MainWindow", "hight", nullptr));
        label_176->setText(QApplication::translate("MainWindow", "number", nullptr));
        CB_gripper_num->setItemText(0, QApplication::translate("MainWindow", "1", nullptr));
        CB_gripper_num->setItemText(1, QApplication::translate("MainWindow", "2", nullptr));
        CB_gripper_num->setItemText(2, QApplication::translate("MainWindow", "3", nullptr));
        CB_gripper_num->setItemText(3, QApplication::translate("MainWindow", "4", nullptr));
        CB_gripper_num->setItemText(4, QApplication::translate("MainWindow", "5", nullptr));
        CB_gripper_num->setItemText(5, QApplication::translate("MainWindow", "6", nullptr));
        CB_gripper_num->setItemText(6, QApplication::translate("MainWindow", "7", nullptr));
        CB_gripper_num->setItemText(7, QApplication::translate("MainWindow", "8", nullptr));
        CB_gripper_num->setItemText(8, QApplication::translate("MainWindow", "9", nullptr));
        CB_gripper_num->setItemText(9, QApplication::translate("MainWindow", "10", nullptr));

        bt_scen_resume->setText(QApplication::translate("MainWindow", "RESUME", nullptr));
        te_status->setText(QApplication::translate("MainWindow", "status", nullptr));
        ALL_STOP->setText(QApplication::translate("MainWindow", "ALL STOP!!!!!!!!!!!", nullptr));
        label_175->setText(QApplication::translate("MainWindow", "shelf", nullptr));
        label_180->setText(QApplication::translate("MainWindow", "direction", nullptr));
        label_179->setText(QApplication::translate("MainWindow", "object ID ", nullptr));
        scenario->setTabText(scenario->indexOf(tab_3), QApplication::translate("MainWindow", "scenario", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
