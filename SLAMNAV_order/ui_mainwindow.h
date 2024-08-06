/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.15.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include "mygraphicsview.h"

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QHBoxLayout *horizontalLayout;
    QVBoxLayout *verticalLayout;
    MyGraphicsView *gv_Screen1;
    QFrame *frame;
    QLabel *lb_Pose;
    QPushButton *bt_RobotMoveRightTurn;
    QPushButton *bt_RobotMoveBackward;
    QPushButton *bt_RobotMoveLeftTurn;
    QPushButton *bt_RobotMoveForward;
    QDoubleSpinBox *spb_JogGain;
    QTextEdit *te_Pose;
    QLabel *label_2;
    QCheckBox *ckb_PlotEnable;
    QLabel *lb_BuildInfo;
    QLabel *lb_VersionInfo;
    QLabel *lb_Target;
    QTabWidget *tabWidget;
    QWidget *tab;
    QLabel *label;
    QPushButton *bt_MapRun;
    QPushButton *bt_MapStop;
    QPushButton *bt_MapSave;
    QPushButton *bt_MapManualLc;
    QWidget *tab_2;
    QLabel *label_4;
    QPushButton *bt_LocalizationRun;
    QLabel *label_5;
    QPushButton *bt_ManualStop;
    QPushButton *bt_LocalizationLoad;
    QLabel *lb_MapName;
    QPushButton *bt_ManualClickedRun;
    QPushButton *bt_LocalizationStop;
    QPushButton *bt_LocalizationInit;
    QComboBox *cb_ManualServing;
    QLabel *label_6;
    QLabel *label_8;
    QComboBox *cb_ManualResting;
    QLabel *label_9;
    QComboBox *cb_ManualCharging;
    QPushButton *bt_ManualServingRun;
    QPushButton *bt_ManualRestingRun;
    QPushButton *bt_ManualChargingRun;
    QPushButton *bt_Test;
    QPushButton *bt_Test2;
    QPushButton *bt_LocalizationInitAuto_Full;
    QPushButton *bt_Exit;
    QPushButton *bt_Test3;
    QPushButton *bt_ManualPause;
    QPushButton *bt_ManualResume;
    QPushButton *bt_SimEmoPush;
    QPushButton *bt_SimEmoRelease;
    QPushButton *bt_Test4;
    QPushButton *bt_LocalizationInitAuto_Semi;
    QComboBox *cb_preset;
    QLabel *label_10;
    QPushButton *bt_DrawStart;
    QPushButton *bt_DrawStop;
    QGroupBox *groupBox;
    QLabel *lb_LocState;
    QLabel *lb_AutoFsmState;
    QLabel *lb_FaceState;
    QLabel *label_24;
    QLabel *lb_PresetState;
    QLabel *lb_LocInfo;
    QLabel *lb_MotorState;
    QLabel *lb_CmdInfo;
    QLabel *lb_AutoState;
    QLabel *lb_QueInfo;
    QLabel *label_23;
    QLabel *lb_MotorLockState;
    QLabel *lb_DrawState;
    QLabel *lb_FailState;
    QLabel *lb_FmsInfo;
    QLabel *lb_MappingInfo;
    QGroupBox *groupBox_2;
    QPushButton *bt_RobotSetLed;
    QPushButton *bt_RobotMotorInit;
    QSpinBox *spb_LedTarget;
    QSpinBox *spb_LedMode;
    QLabel *lb_RobotStatus;
    QLabel *label_3;
    QPushButton *bt_ReloadRobotConfig;
    QComboBox *cb_ManualServingGroup;
    QCheckBox *ckb_TableName;
    QPushButton *bt_CheckAllPath;
    QCheckBox *ckb_SimObs;
    QPushButton *bt_SoftReload;
    QPushButton *bt_HardReload;
    QPushButton *bt_SimInit;
    QPushButton *bt_LocalizationInitAuto_Resting;
    QPushButton *bt_TestMoveExt;
    QPushButton *bt_TestMovePick;
    QComboBox *cb_ManualNodes;
    QPushButton *bt_ManualNodesRun;
    QLabel *label_15;
    QPushButton *bt_ManualChargingSet;
    QPushButton *bt_ManualRestingSet;
    QPushButton *bt_ManualServingSet;
    QCheckBox *ckb_SetLocation;
    QWidget *tab_4;
    QLineEdit *le_IP2Send;
    QLineEdit *le_ID2Send;
    QLineEdit *le_PW2Send;
    QLabel *label_7;
    QLabel *label_12;
    QLabel *label_13;
    QPushButton *bt_SendMaps;
    QLabel *label_14;
    QWidget *tab_3;
    QPushButton *bt_TopoLoad;
    QLabel *lb_ScreenL;
    QLabel *lb_ScreenR;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(1134, 650);
        MainWindow->setMinimumSize(QSize(0, 0));
        QFont font;
        font.setPointSize(9);
        MainWindow->setFont(font);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        horizontalLayout = new QHBoxLayout(centralwidget);
        horizontalLayout->setSpacing(0);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        horizontalLayout->setContentsMargins(0, 0, 0, 0);
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        gv_Screen1 = new MyGraphicsView(centralwidget);
        gv_Screen1->setObjectName(QString::fromUtf8("gv_Screen1"));
        gv_Screen1->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        gv_Screen1->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        gv_Screen1->setSceneRect(QRectF(0, 0, 2000, 2000));
        gv_Screen1->setResizeAnchor(QGraphicsView::AnchorViewCenter);

        verticalLayout->addWidget(gv_Screen1);

        frame = new QFrame(centralwidget);
        frame->setObjectName(QString::fromUtf8("frame"));
        QSizePolicy sizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(frame->sizePolicy().hasHeightForWidth());
        frame->setSizePolicy(sizePolicy);
        frame->setMinimumSize(QSize(450, 200));
        frame->setFrameShape(QFrame::NoFrame);
        frame->setFrameShadow(QFrame::Plain);
        lb_Pose = new QLabel(frame);
        lb_Pose->setObjectName(QString::fromUtf8("lb_Pose"));
        lb_Pose->setGeometry(QRect(220, 150, 231, 21));
        bt_RobotMoveRightTurn = new QPushButton(frame);
        bt_RobotMoveRightTurn->setObjectName(QString::fromUtf8("bt_RobotMoveRightTurn"));
        bt_RobotMoveRightTurn->setGeometry(QRect(130, 90, 51, 41));
        bt_RobotMoveBackward = new QPushButton(frame);
        bt_RobotMoveBackward->setObjectName(QString::fromUtf8("bt_RobotMoveBackward"));
        bt_RobotMoveBackward->setGeometry(QRect(70, 90, 51, 41));
        bt_RobotMoveLeftTurn = new QPushButton(frame);
        bt_RobotMoveLeftTurn->setObjectName(QString::fromUtf8("bt_RobotMoveLeftTurn"));
        bt_RobotMoveLeftTurn->setGeometry(QRect(10, 90, 51, 41));
        bt_RobotMoveForward = new QPushButton(frame);
        bt_RobotMoveForward->setObjectName(QString::fromUtf8("bt_RobotMoveForward"));
        bt_RobotMoveForward->setGeometry(QRect(70, 40, 51, 41));
        bt_RobotMoveForward->setAutoRepeat(false);
        bt_RobotMoveForward->setAutoRepeatDelay(300);
        spb_JogGain = new QDoubleSpinBox(frame);
        spb_JogGain->setObjectName(QString::fromUtf8("spb_JogGain"));
        spb_JogGain->setGeometry(QRect(10, 40, 51, 31));
        spb_JogGain->setDecimals(1);
        spb_JogGain->setMaximum(3.000000000000000);
        spb_JogGain->setSingleStep(0.200000000000000);
        spb_JogGain->setValue(1.000000000000000);
        te_Pose = new QTextEdit(frame);
        te_Pose->setObjectName(QString::fromUtf8("te_Pose"));
        te_Pose->setGeometry(QRect(220, 40, 221, 101));
        te_Pose->setFrameShape(QFrame::Box);
        te_Pose->setFrameShadow(QFrame::Plain);
        te_Pose->setReadOnly(true);
        label_2 = new QLabel(frame);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(220, 10, 151, 21));
        ckb_PlotEnable = new QCheckBox(frame);
        ckb_PlotEnable->setObjectName(QString::fromUtf8("ckb_PlotEnable"));
        ckb_PlotEnable->setGeometry(QRect(10, 10, 111, 22));
        ckb_PlotEnable->setChecked(false);
        lb_BuildInfo = new QLabel(frame);
        lb_BuildInfo->setObjectName(QString::fromUtf8("lb_BuildInfo"));
        lb_BuildInfo->setGeometry(QRect(10, 170, 201, 21));
        lb_VersionInfo = new QLabel(frame);
        lb_VersionInfo->setObjectName(QString::fromUtf8("lb_VersionInfo"));
        lb_VersionInfo->setGeometry(QRect(10, 150, 121, 21));
        lb_Target = new QLabel(frame);
        lb_Target->setObjectName(QString::fromUtf8("lb_Target"));
        lb_Target->setGeometry(QRect(220, 170, 231, 21));

        verticalLayout->addWidget(frame);

        verticalLayout->setStretch(0, 1);

        horizontalLayout->addLayout(verticalLayout);

        tabWidget = new QTabWidget(centralwidget);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        QSizePolicy sizePolicy1(QSizePolicy::Fixed, QSizePolicy::Expanding);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(tabWidget->sizePolicy().hasHeightForWidth());
        tabWidget->setSizePolicy(sizePolicy1);
        tabWidget->setMinimumSize(QSize(610, 0));
        tab = new QWidget();
        tab->setObjectName(QString::fromUtf8("tab"));
        label = new QLabel(tab);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(20, 10, 71, 21));
        bt_MapRun = new QPushButton(tab);
        bt_MapRun->setObjectName(QString::fromUtf8("bt_MapRun"));
        bt_MapRun->setGeometry(QRect(20, 40, 91, 31));
        bt_MapStop = new QPushButton(tab);
        bt_MapStop->setObjectName(QString::fromUtf8("bt_MapStop"));
        bt_MapStop->setGeometry(QRect(120, 40, 91, 31));
        bt_MapSave = new QPushButton(tab);
        bt_MapSave->setObjectName(QString::fromUtf8("bt_MapSave"));
        bt_MapSave->setGeometry(QRect(220, 40, 91, 31));
        bt_MapManualLc = new QPushButton(tab);
        bt_MapManualLc->setObjectName(QString::fromUtf8("bt_MapManualLc"));
        bt_MapManualLc->setGeometry(QRect(20, 90, 91, 31));
        tabWidget->addTab(tab, QString());
        tab_2 = new QWidget();
        tab_2->setObjectName(QString::fromUtf8("tab_2"));
        label_4 = new QLabel(tab_2);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setGeometry(QRect(20, 10, 91, 21));
        bt_LocalizationRun = new QPushButton(tab_2);
        bt_LocalizationRun->setObjectName(QString::fromUtf8("bt_LocalizationRun"));
        bt_LocalizationRun->setGeometry(QRect(140, 40, 61, 31));
        label_5 = new QLabel(tab_2);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        label_5->setGeometry(QRect(20, 160, 121, 21));
        bt_ManualStop = new QPushButton(tab_2);
        bt_ManualStop->setObjectName(QString::fromUtf8("bt_ManualStop"));
        bt_ManualStop->setGeometry(QRect(230, 380, 71, 31));
        QFont font1;
        font1.setPointSize(10);
        bt_ManualStop->setFont(font1);
        bt_LocalizationLoad = new QPushButton(tab_2);
        bt_LocalizationLoad->setObjectName(QString::fromUtf8("bt_LocalizationLoad"));
        bt_LocalizationLoad->setGeometry(QRect(20, 40, 61, 31));
        lb_MapName = new QLabel(tab_2);
        lb_MapName->setObjectName(QString::fromUtf8("lb_MapName"));
        lb_MapName->setGeometry(QRect(20, 130, 291, 21));
        lb_MapName->setWordWrap(true);
        bt_ManualClickedRun = new QPushButton(tab_2);
        bt_ManualClickedRun->setObjectName(QString::fromUtf8("bt_ManualClickedRun"));
        bt_ManualClickedRun->setGeometry(QRect(20, 380, 71, 31));
        bt_ManualClickedRun->setFont(font1);
        bt_LocalizationStop = new QPushButton(tab_2);
        bt_LocalizationStop->setObjectName(QString::fromUtf8("bt_LocalizationStop"));
        bt_LocalizationStop->setGeometry(QRect(200, 40, 61, 31));
        bt_LocalizationInit = new QPushButton(tab_2);
        bt_LocalizationInit->setObjectName(QString::fromUtf8("bt_LocalizationInit"));
        bt_LocalizationInit->setGeometry(QRect(80, 40, 61, 31));
        cb_ManualServing = new QComboBox(tab_2);
        cb_ManualServing->setObjectName(QString::fromUtf8("cb_ManualServing"));
        cb_ManualServing->setGeometry(QRect(130, 190, 70, 31));
        label_6 = new QLabel(tab_2);
        label_6->setObjectName(QString::fromUtf8("label_6"));
        label_6->setGeometry(QRect(20, 190, 61, 31));
        label_8 = new QLabel(tab_2);
        label_8->setObjectName(QString::fromUtf8("label_8"));
        label_8->setGeometry(QRect(20, 220, 71, 31));
        cb_ManualResting = new QComboBox(tab_2);
        cb_ManualResting->setObjectName(QString::fromUtf8("cb_ManualResting"));
        cb_ManualResting->setGeometry(QRect(80, 220, 120, 31));
        label_9 = new QLabel(tab_2);
        label_9->setObjectName(QString::fromUtf8("label_9"));
        label_9->setGeometry(QRect(20, 250, 61, 31));
        cb_ManualCharging = new QComboBox(tab_2);
        cb_ManualCharging->setObjectName(QString::fromUtf8("cb_ManualCharging"));
        cb_ManualCharging->setGeometry(QRect(80, 250, 120, 31));
        bt_ManualServingRun = new QPushButton(tab_2);
        bt_ManualServingRun->setObjectName(QString::fromUtf8("bt_ManualServingRun"));
        bt_ManualServingRun->setGeometry(QRect(199, 190, 51, 31));
        bt_ManualRestingRun = new QPushButton(tab_2);
        bt_ManualRestingRun->setObjectName(QString::fromUtf8("bt_ManualRestingRun"));
        bt_ManualRestingRun->setGeometry(QRect(199, 220, 51, 31));
        bt_ManualChargingRun = new QPushButton(tab_2);
        bt_ManualChargingRun->setObjectName(QString::fromUtf8("bt_ManualChargingRun"));
        bt_ManualChargingRun->setGeometry(QRect(199, 250, 51, 31));
        bt_Test = new QPushButton(tab_2);
        bt_Test->setObjectName(QString::fromUtf8("bt_Test"));
        bt_Test->setGeometry(QRect(20, 570, 71, 31));
        bt_Test->setFont(font1);
        bt_Test2 = new QPushButton(tab_2);
        bt_Test2->setObjectName(QString::fromUtf8("bt_Test2"));
        bt_Test2->setGeometry(QRect(90, 570, 71, 31));
        bt_Test2->setFont(font1);
        bt_LocalizationInitAuto_Full = new QPushButton(tab_2);
        bt_LocalizationInitAuto_Full->setObjectName(QString::fromUtf8("bt_LocalizationInitAuto_Full"));
        bt_LocalizationInitAuto_Full->setGeometry(QRect(20, 80, 61, 31));
        bt_Exit = new QPushButton(tab_2);
        bt_Exit->setObjectName(QString::fromUtf8("bt_Exit"));
        bt_Exit->setGeometry(QRect(230, 540, 71, 31));
        bt_Test3 = new QPushButton(tab_2);
        bt_Test3->setObjectName(QString::fromUtf8("bt_Test3"));
        bt_Test3->setGeometry(QRect(160, 570, 71, 31));
        bt_Test3->setFont(font1);
        bt_ManualPause = new QPushButton(tab_2);
        bt_ManualPause->setObjectName(QString::fromUtf8("bt_ManualPause"));
        bt_ManualPause->setGeometry(QRect(90, 380, 71, 31));
        bt_ManualPause->setFont(font1);
        bt_ManualResume = new QPushButton(tab_2);
        bt_ManualResume->setObjectName(QString::fromUtf8("bt_ManualResume"));
        bt_ManualResume->setGeometry(QRect(160, 380, 71, 31));
        bt_ManualResume->setFont(font1);
        bt_SimEmoPush = new QPushButton(tab_2);
        bt_SimEmoPush->setObjectName(QString::fromUtf8("bt_SimEmoPush"));
        bt_SimEmoPush->setGeometry(QRect(90, 540, 71, 31));
        bt_SimEmoPush->setFont(font1);
        bt_SimEmoRelease = new QPushButton(tab_2);
        bt_SimEmoRelease->setObjectName(QString::fromUtf8("bt_SimEmoRelease"));
        bt_SimEmoRelease->setGeometry(QRect(160, 540, 71, 31));
        bt_SimEmoRelease->setFont(font1);
        bt_Test4 = new QPushButton(tab_2);
        bt_Test4->setObjectName(QString::fromUtf8("bt_Test4"));
        bt_Test4->setGeometry(QRect(230, 570, 71, 31));
        bt_Test4->setFont(font1);
        bt_LocalizationInitAuto_Semi = new QPushButton(tab_2);
        bt_LocalizationInitAuto_Semi->setObjectName(QString::fromUtf8("bt_LocalizationInitAuto_Semi"));
        bt_LocalizationInitAuto_Semi->setGeometry(QRect(80, 80, 61, 31));
        cb_preset = new QComboBox(tab_2);
        cb_preset->addItem(QString());
        cb_preset->addItem(QString());
        cb_preset->addItem(QString());
        cb_preset->addItem(QString());
        cb_preset->addItem(QString());
        cb_preset->setObjectName(QString::fromUtf8("cb_preset"));
        cb_preset->setGeometry(QRect(80, 310, 81, 31));
        cb_preset->setLayoutDirection(Qt::LeftToRight);
        label_10 = new QLabel(tab_2);
        label_10->setObjectName(QString::fromUtf8("label_10"));
        label_10->setGeometry(QRect(20, 310, 61, 31));
        bt_DrawStart = new QPushButton(tab_2);
        bt_DrawStart->setObjectName(QString::fromUtf8("bt_DrawStart"));
        bt_DrawStart->setGeometry(QRect(90, 410, 71, 31));
        bt_DrawStop = new QPushButton(tab_2);
        bt_DrawStop->setObjectName(QString::fromUtf8("bt_DrawStop"));
        bt_DrawStop->setGeometry(QRect(160, 410, 71, 31));
        groupBox = new QGroupBox(tab_2);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        groupBox->setGeometry(QRect(320, 0, 281, 371));
        groupBox->setFont(font);
        lb_LocState = new QLabel(groupBox);
        lb_LocState->setObjectName(QString::fromUtf8("lb_LocState"));
        lb_LocState->setGeometry(QRect(50, 40, 181, 16));
        lb_LocState->setFont(font);
        lb_AutoFsmState = new QLabel(groupBox);
        lb_AutoFsmState->setObjectName(QString::fromUtf8("lb_AutoFsmState"));
        lb_AutoFsmState->setGeometry(QRect(50, 200, 181, 16));
        lb_AutoFsmState->setFont(font);
        lb_FaceState = new QLabel(groupBox);
        lb_FaceState->setObjectName(QString::fromUtf8("lb_FaceState"));
        lb_FaceState->setGeometry(QRect(50, 160, 181, 16));
        lb_FaceState->setFont(font);
        label_24 = new QLabel(groupBox);
        label_24->setObjectName(QString::fromUtf8("label_24"));
        label_24->setGeometry(QRect(10, 200, 31, 21));
        lb_PresetState = new QLabel(groupBox);
        lb_PresetState->setObjectName(QString::fromUtf8("lb_PresetState"));
        lb_PresetState->setGeometry(QRect(50, 140, 181, 16));
        lb_PresetState->setFont(font);
        lb_LocInfo = new QLabel(groupBox);
        lb_LocInfo->setObjectName(QString::fromUtf8("lb_LocInfo"));
        lb_LocInfo->setGeometry(QRect(50, 220, 181, 16));
        lb_LocInfo->setFont(font);
        lb_MotorState = new QLabel(groupBox);
        lb_MotorState->setObjectName(QString::fromUtf8("lb_MotorState"));
        lb_MotorState->setGeometry(QRect(50, 20, 181, 16));
        lb_MotorState->setFont(font);
        lb_CmdInfo = new QLabel(groupBox);
        lb_CmdInfo->setObjectName(QString::fromUtf8("lb_CmdInfo"));
        lb_CmdInfo->setGeometry(QRect(50, 260, 181, 16));
        lb_CmdInfo->setFont(font);
        lb_AutoState = new QLabel(groupBox);
        lb_AutoState->setObjectName(QString::fromUtf8("lb_AutoState"));
        lb_AutoState->setGeometry(QRect(50, 60, 181, 16));
        lb_AutoState->setFont(font);
        lb_QueInfo = new QLabel(groupBox);
        lb_QueInfo->setObjectName(QString::fromUtf8("lb_QueInfo"));
        lb_QueInfo->setGeometry(QRect(50, 280, 221, 41));
        lb_QueInfo->setFont(font);
        label_23 = new QLabel(groupBox);
        label_23->setObjectName(QString::fromUtf8("label_23"));
        label_23->setGeometry(QRect(10, 30, 31, 21));
        lb_MotorLockState = new QLabel(groupBox);
        lb_MotorLockState->setObjectName(QString::fromUtf8("lb_MotorLockState"));
        lb_MotorLockState->setGeometry(QRect(50, 120, 181, 16));
        lb_MotorLockState->setFont(font);
        lb_DrawState = new QLabel(groupBox);
        lb_DrawState->setObjectName(QString::fromUtf8("lb_DrawState"));
        lb_DrawState->setGeometry(QRect(50, 180, 181, 16));
        lb_DrawState->setFont(font);
        lb_FailState = new QLabel(groupBox);
        lb_FailState->setObjectName(QString::fromUtf8("lb_FailState"));
        lb_FailState->setGeometry(QRect(50, 80, 181, 16));
        lb_FailState->setFont(font);
        lb_FmsInfo = new QLabel(groupBox);
        lb_FmsInfo->setObjectName(QString::fromUtf8("lb_FmsInfo"));
        lb_FmsInfo->setGeometry(QRect(50, 330, 221, 17));
        lb_MappingInfo = new QLabel(groupBox);
        lb_MappingInfo->setObjectName(QString::fromUtf8("lb_MappingInfo"));
        lb_MappingInfo->setGeometry(QRect(50, 240, 181, 16));
        lb_MappingInfo->setFont(font);
        groupBox_2 = new QGroupBox(tab_2);
        groupBox_2->setObjectName(QString::fromUtf8("groupBox_2"));
        groupBox_2->setGeometry(QRect(320, 380, 281, 231));
        bt_RobotSetLed = new QPushButton(groupBox_2);
        bt_RobotSetLed->setObjectName(QString::fromUtf8("bt_RobotSetLed"));
        bt_RobotSetLed->setGeometry(QRect(170, 30, 51, 21));
        bt_RobotMotorInit = new QPushButton(groupBox_2);
        bt_RobotMotorInit->setObjectName(QString::fromUtf8("bt_RobotMotorInit"));
        bt_RobotMotorInit->setGeometry(QRect(10, 30, 51, 21));
        spb_LedTarget = new QSpinBox(groupBox_2);
        spb_LedTarget->setObjectName(QString::fromUtf8("spb_LedTarget"));
        spb_LedTarget->setGeometry(QRect(70, 30, 41, 21));
        QFont font2;
        font2.setPointSize(8);
        spb_LedTarget->setFont(font2);
        spb_LedTarget->setMaximum(1);
        spb_LedMode = new QSpinBox(groupBox_2);
        spb_LedMode->setObjectName(QString::fromUtf8("spb_LedMode"));
        spb_LedMode->setGeometry(QRect(120, 30, 41, 21));
        spb_LedMode->setFont(font2);
        spb_LedMode->setMaximum(5);
        lb_RobotStatus = new QLabel(groupBox_2);
        lb_RobotStatus->setObjectName(QString::fromUtf8("lb_RobotStatus"));
        lb_RobotStatus->setGeometry(QRect(10, 60, 261, 121));
        lb_RobotStatus->setFont(font1);
        label_3 = new QLabel(groupBox_2);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setGeometry(QRect(10, 190, 281, 41));
        label_3->setFont(font);
        bt_ReloadRobotConfig = new QPushButton(tab_2);
        bt_ReloadRobotConfig->setObjectName(QString::fromUtf8("bt_ReloadRobotConfig"));
        bt_ReloadRobotConfig->setGeometry(QRect(20, 440, 71, 31));
        bt_ReloadRobotConfig->setFont(font1);
        cb_ManualServingGroup = new QComboBox(tab_2);
        cb_ManualServingGroup->setObjectName(QString::fromUtf8("cb_ManualServingGroup"));
        cb_ManualServingGroup->setGeometry(QRect(80, 190, 51, 31));
        ckb_TableName = new QCheckBox(tab_2);
        ckb_TableName->setObjectName(QString::fromUtf8("ckb_TableName"));
        ckb_TableName->setGeometry(QRect(110, 160, 91, 22));
        ckb_TableName->setChecked(true);
        bt_CheckAllPath = new QPushButton(tab_2);
        bt_CheckAllPath->setObjectName(QString::fromUtf8("bt_CheckAllPath"));
        bt_CheckAllPath->setGeometry(QRect(20, 410, 71, 31));
        ckb_SimObs = new QCheckBox(tab_2);
        ckb_SimObs->setObjectName(QString::fromUtf8("ckb_SimObs"));
        ckb_SimObs->setGeometry(QRect(200, 160, 71, 22));
        ckb_SimObs->setChecked(false);
        bt_SoftReload = new QPushButton(tab_2);
        bt_SoftReload->setObjectName(QString::fromUtf8("bt_SoftReload"));
        bt_SoftReload->setGeometry(QRect(90, 440, 71, 31));
        bt_SoftReload->setFont(font1);
        bt_HardReload = new QPushButton(tab_2);
        bt_HardReload->setObjectName(QString::fromUtf8("bt_HardReload"));
        bt_HardReload->setGeometry(QRect(160, 440, 71, 31));
        bt_HardReload->setFont(font1);
        bt_SimInit = new QPushButton(tab_2);
        bt_SimInit->setObjectName(QString::fromUtf8("bt_SimInit"));
        bt_SimInit->setGeometry(QRect(20, 540, 71, 31));
        bt_SimInit->setFont(font1);
        bt_LocalizationInitAuto_Resting = new QPushButton(tab_2);
        bt_LocalizationInitAuto_Resting->setObjectName(QString::fromUtf8("bt_LocalizationInitAuto_Resting"));
        bt_LocalizationInitAuto_Resting->setGeometry(QRect(140, 80, 61, 31));
        QFont font3;
        font3.setPointSize(7);
        bt_LocalizationInitAuto_Resting->setFont(font3);
        bt_TestMoveExt = new QPushButton(tab_2);
        bt_TestMoveExt->setObjectName(QString::fromUtf8("bt_TestMoveExt"));
        bt_TestMoveExt->setGeometry(QRect(20, 490, 71, 31));
        bt_TestMoveExt->setFont(font1);
        bt_TestMovePick = new QPushButton(tab_2);
        bt_TestMovePick->setObjectName(QString::fromUtf8("bt_TestMovePick"));
        bt_TestMovePick->setGeometry(QRect(90, 490, 71, 31));
        bt_TestMovePick->setFont(font1);
        cb_ManualNodes = new QComboBox(tab_2);
        cb_ManualNodes->setObjectName(QString::fromUtf8("cb_ManualNodes"));
        cb_ManualNodes->setGeometry(QRect(80, 280, 120, 31));
        bt_ManualNodesRun = new QPushButton(tab_2);
        bt_ManualNodesRun->setObjectName(QString::fromUtf8("bt_ManualNodesRun"));
        bt_ManualNodesRun->setGeometry(QRect(199, 280, 51, 31));
        label_15 = new QLabel(tab_2);
        label_15->setObjectName(QString::fromUtf8("label_15"));
        label_15->setGeometry(QRect(20, 280, 61, 31));
        bt_ManualChargingSet = new QPushButton(tab_2);
        bt_ManualChargingSet->setObjectName(QString::fromUtf8("bt_ManualChargingSet"));
        bt_ManualChargingSet->setEnabled(false);
        bt_ManualChargingSet->setGeometry(QRect(249, 250, 51, 31));
        bt_ManualRestingSet = new QPushButton(tab_2);
        bt_ManualRestingSet->setObjectName(QString::fromUtf8("bt_ManualRestingSet"));
        bt_ManualRestingSet->setEnabled(false);
        bt_ManualRestingSet->setGeometry(QRect(249, 220, 51, 31));
        bt_ManualServingSet = new QPushButton(tab_2);
        bt_ManualServingSet->setObjectName(QString::fromUtf8("bt_ManualServingSet"));
        bt_ManualServingSet->setEnabled(false);
        bt_ManualServingSet->setGeometry(QRect(249, 190, 51, 31));
        ckb_SetLocation = new QCheckBox(tab_2);
        ckb_SetLocation->setObjectName(QString::fromUtf8("ckb_SetLocation"));
        ckb_SetLocation->setGeometry(QRect(270, 160, 41, 22));
        ckb_SetLocation->setChecked(false);
        tabWidget->addTab(tab_2, QString());
        tab_4 = new QWidget();
        tab_4->setObjectName(QString::fromUtf8("tab_4"));
        le_IP2Send = new QLineEdit(tab_4);
        le_IP2Send->setObjectName(QString::fromUtf8("le_IP2Send"));
        le_IP2Send->setGeometry(QRect(55, 38, 181, 25));
        le_ID2Send = new QLineEdit(tab_4);
        le_ID2Send->setObjectName(QString::fromUtf8("le_ID2Send"));
        le_ID2Send->setGeometry(QRect(55, 67, 181, 25));
        le_PW2Send = new QLineEdit(tab_4);
        le_PW2Send->setObjectName(QString::fromUtf8("le_PW2Send"));
        le_PW2Send->setGeometry(QRect(55, 98, 181, 25));
        label_7 = new QLabel(tab_4);
        label_7->setObjectName(QString::fromUtf8("label_7"));
        label_7->setGeometry(QRect(20, 42, 31, 17));
        QFont font4;
        font4.setPointSize(11);
        label_7->setFont(font4);
        label_12 = new QLabel(tab_4);
        label_12->setObjectName(QString::fromUtf8("label_12"));
        label_12->setGeometry(QRect(19, 70, 31, 17));
        label_12->setFont(font4);
        label_13 = new QLabel(tab_4);
        label_13->setObjectName(QString::fromUtf8("label_13"));
        label_13->setGeometry(QRect(20, 100, 41, 20));
        label_13->setFont(font4);
        bt_SendMaps = new QPushButton(tab_4);
        bt_SendMaps->setObjectName(QString::fromUtf8("bt_SendMaps"));
        bt_SendMaps->setGeometry(QRect(240, 90, 91, 31));
        label_14 = new QLabel(tab_4);
        label_14->setObjectName(QString::fromUtf8("label_14"));
        label_14->setGeometry(QRect(20, 10, 91, 21));
        tabWidget->addTab(tab_4, QString());
        tab_3 = new QWidget();
        tab_3->setObjectName(QString::fromUtf8("tab_3"));
        bt_TopoLoad = new QPushButton(tab_3);
        bt_TopoLoad->setObjectName(QString::fromUtf8("bt_TopoLoad"));
        bt_TopoLoad->setGeometry(QRect(20, 10, 51, 41));
        lb_ScreenL = new QLabel(tab_3);
        lb_ScreenL->setObjectName(QString::fromUtf8("lb_ScreenL"));
        lb_ScreenL->setGeometry(QRect(20, 70, 180, 90));
        lb_ScreenL->setBaseSize(QSize(0, 0));
        lb_ScreenL->setStyleSheet(QString::fromUtf8("background-color: rgb(0, 0, 0);"));
        lb_ScreenR = new QLabel(tab_3);
        lb_ScreenR->setObjectName(QString::fromUtf8("lb_ScreenR"));
        lb_ScreenR->setGeometry(QRect(210, 70, 180, 90));
        lb_ScreenR->setBaseSize(QSize(0, 0));
        lb_ScreenR->setStyleSheet(QString::fromUtf8("background-color: rgb(0, 0, 0);"));
        tabWidget->addTab(tab_3, QString());

        horizontalLayout->addWidget(tabWidget);

        horizontalLayout->setStretch(0, 1);
        MainWindow->setCentralWidget(centralwidget);

        retranslateUi(MainWindow);

        tabWidget->setCurrentIndex(1);
        cb_preset->setCurrentIndex(2);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "SLAMNAV_0705", nullptr));
        lb_Pose->setText(QCoreApplication::translate("MainWindow", "POSE: 0, 0, 0", nullptr));
        bt_RobotMoveRightTurn->setText(QCoreApplication::translate("MainWindow", "R", nullptr));
        bt_RobotMoveBackward->setText(QCoreApplication::translate("MainWindow", "B", nullptr));
        bt_RobotMoveLeftTurn->setText(QCoreApplication::translate("MainWindow", "L", nullptr));
        bt_RobotMoveForward->setText(QCoreApplication::translate("MainWindow", "F", nullptr));
        label_2->setText(QCoreApplication::translate("MainWindow", "Mobile platform", nullptr));
        ckb_PlotEnable->setText(QCoreApplication::translate("MainWindow", "Plot enable", nullptr));
        lb_BuildInfo->setText(QCoreApplication::translate("MainWindow", "lb_BuildInfo", nullptr));
        lb_VersionInfo->setText(QCoreApplication::translate("MainWindow", "lb_VersionInfo", nullptr));
        lb_Target->setText(QCoreApplication::translate("MainWindow", "TARGET: 0, 0, 0", nullptr));
        label->setText(QCoreApplication::translate("MainWindow", "Mapping", nullptr));
        bt_MapRun->setText(QCoreApplication::translate("MainWindow", "Run", nullptr));
        bt_MapStop->setText(QCoreApplication::translate("MainWindow", "Stop", nullptr));
        bt_MapSave->setText(QCoreApplication::translate("MainWindow", "Save", nullptr));
        bt_MapManualLc->setText(QCoreApplication::translate("MainWindow", "Loop Closing", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab), QCoreApplication::translate("MainWindow", "Management", nullptr));
        label_4->setText(QCoreApplication::translate("MainWindow", "Localization", nullptr));
        bt_LocalizationRun->setText(QCoreApplication::translate("MainWindow", "Loc Run", nullptr));
        label_5->setText(QCoreApplication::translate("MainWindow", "Manual Control", nullptr));
        bt_ManualStop->setText(QCoreApplication::translate("MainWindow", "Stop", nullptr));
        bt_LocalizationLoad->setText(QCoreApplication::translate("MainWindow", "Load", nullptr));
        lb_MapName->setText(QCoreApplication::translate("MainWindow", "Map name:", nullptr));
        bt_ManualClickedRun->setText(QCoreApplication::translate("MainWindow", "Set goal", nullptr));
        bt_LocalizationStop->setText(QCoreApplication::translate("MainWindow", "Loc Stop", nullptr));
        bt_LocalizationInit->setText(QCoreApplication::translate("MainWindow", "Set Init", nullptr));
        label_6->setText(QCoreApplication::translate("MainWindow", "Serving :", nullptr));
        label_8->setText(QCoreApplication::translate("MainWindow", "Resting :", nullptr));
        label_9->setText(QCoreApplication::translate("MainWindow", "Charging :", nullptr));
        bt_ManualServingRun->setText(QCoreApplication::translate("MainWindow", "Run", nullptr));
        bt_ManualRestingRun->setText(QCoreApplication::translate("MainWindow", "Run", nullptr));
        bt_ManualChargingRun->setText(QCoreApplication::translate("MainWindow", "Run", nullptr));
        bt_Test->setText(QCoreApplication::translate("MainWindow", "test", nullptr));
        bt_Test2->setText(QCoreApplication::translate("MainWindow", "test2", nullptr));
        bt_LocalizationInitAuto_Full->setText(QCoreApplication::translate("MainWindow", "Full auto", nullptr));
        bt_Exit->setText(QCoreApplication::translate("MainWindow", "Exit", nullptr));
        bt_Test3->setText(QCoreApplication::translate("MainWindow", "test3", nullptr));
        bt_ManualPause->setText(QCoreApplication::translate("MainWindow", "Pause", nullptr));
        bt_ManualResume->setText(QCoreApplication::translate("MainWindow", "Resume", nullptr));
        bt_SimEmoPush->setText(QCoreApplication::translate("MainWindow", "Sim EMO\n"
"push", nullptr));
        bt_SimEmoRelease->setText(QCoreApplication::translate("MainWindow", "Sim EMO\n"
"release", nullptr));
        bt_Test4->setText(QCoreApplication::translate("MainWindow", "test4", nullptr));
        bt_LocalizationInitAuto_Semi->setText(QCoreApplication::translate("MainWindow", "Semi auto", nullptr));
        cb_preset->setItemText(0, QCoreApplication::translate("MainWindow", "SLOWEST", nullptr));
        cb_preset->setItemText(1, QCoreApplication::translate("MainWindow", "SLOW", nullptr));
        cb_preset->setItemText(2, QCoreApplication::translate("MainWindow", "NORMAL", nullptr));
        cb_preset->setItemText(3, QCoreApplication::translate("MainWindow", "FAST", nullptr));
        cb_preset->setItemText(4, QCoreApplication::translate("MainWindow", "FASTEST", nullptr));

        label_10->setText(QCoreApplication::translate("MainWindow", "preset :", nullptr));
        bt_DrawStart->setText(QCoreApplication::translate("MainWindow", "Draw start", nullptr));
        bt_DrawStop->setText(QCoreApplication::translate("MainWindow", "Draw stop", nullptr));
        groupBox->setTitle(QCoreApplication::translate("MainWindow", "State", nullptr));
        lb_LocState->setText(QCoreApplication::translate("MainWindow", "LOC_STATE", nullptr));
        lb_AutoFsmState->setText(QCoreApplication::translate("MainWindow", "FSM_STATE (inner)", nullptr));
        lb_FaceState->setText(QCoreApplication::translate("MainWindow", "FACE_STATE", nullptr));
        label_24->setText(QCoreApplication::translate("MainWindow", "MY:", nullptr));
        lb_PresetState->setText(QCoreApplication::translate("MainWindow", "PRESET_STATE", nullptr));
        lb_LocInfo->setText(QCoreApplication::translate("MainWindow", "LOC_INFO", nullptr));
        lb_MotorState->setText(QCoreApplication::translate("MainWindow", "MOTOR_STATE", nullptr));
        lb_CmdInfo->setText(QCoreApplication::translate("MainWindow", "CMD_INFO", nullptr));
        lb_AutoState->setText(QCoreApplication::translate("MainWindow", "AUTO_STATE", nullptr));
        lb_QueInfo->setText(QCoreApplication::translate("MainWindow", "QUE_INFO", nullptr));
        label_23->setText(QCoreApplication::translate("MainWindow", "UI :", nullptr));
        lb_MotorLockState->setText(QCoreApplication::translate("MainWindow", "MOTOR_LOCK_STATE", nullptr));
        lb_DrawState->setText(QCoreApplication::translate("MainWindow", "DRAW_STATE", nullptr));
        lb_FailState->setText(QCoreApplication::translate("MainWindow", "FAIL_STATE", nullptr));
        lb_FmsInfo->setText(QCoreApplication::translate("MainWindow", "FMS_INFO", nullptr));
        lb_MappingInfo->setText(QCoreApplication::translate("MainWindow", "MAPPING_INFO", nullptr));
        groupBox_2->setTitle(QCoreApplication::translate("MainWindow", "Mobile", nullptr));
        bt_RobotSetLed->setText(QCoreApplication::translate("MainWindow", "LED", nullptr));
        bt_RobotMotorInit->setText(QCoreApplication::translate("MainWindow", "Init", nullptr));
        lb_RobotStatus->setText(QCoreApplication::translate("MainWindow", "robot status", nullptr));
        label_3->setText(QCoreApplication::translate("MainWindow", "MOD = 2, JAM = 4, CUR = 8, BIG = 16, \n"
"IN = 32, PS1|2 = 64, NON = 128", nullptr));
        bt_ReloadRobotConfig->setText(QCoreApplication::translate("MainWindow", "Reload\n"
"config", nullptr));
        ckb_TableName->setText(QCoreApplication::translate("MainWindow", "Table name", nullptr));
        bt_CheckAllPath->setText(QCoreApplication::translate("MainWindow", "Check Path", nullptr));
        ckb_SimObs->setText(QCoreApplication::translate("MainWindow", "Sim obs", nullptr));
        bt_SoftReload->setText(QCoreApplication::translate("MainWindow", "Soft\n"
"reload", nullptr));
        bt_HardReload->setText(QCoreApplication::translate("MainWindow", "Hard\n"
"reload", nullptr));
        bt_SimInit->setText(QCoreApplication::translate("MainWindow", "Sim\n"
"init", nullptr));
        bt_LocalizationInitAuto_Resting->setText(QCoreApplication::translate("MainWindow", "Resting auto", nullptr));
        bt_TestMoveExt->setText(QCoreApplication::translate("MainWindow", "test \n"
"move ext", nullptr));
        bt_TestMovePick->setText(QCoreApplication::translate("MainWindow", "test\n"
"move pick", nullptr));
        bt_ManualNodesRun->setText(QCoreApplication::translate("MainWindow", "Run", nullptr));
        label_15->setText(QCoreApplication::translate("MainWindow", "Node :", nullptr));
        bt_ManualChargingSet->setText(QCoreApplication::translate("MainWindow", "Set", nullptr));
        bt_ManualRestingSet->setText(QCoreApplication::translate("MainWindow", "Set", nullptr));
        bt_ManualServingSet->setText(QCoreApplication::translate("MainWindow", "Set", nullptr));
        ckb_SetLocation->setText(QCoreApplication::translate("MainWindow", "Set", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_2), QCoreApplication::translate("MainWindow", "Operation", nullptr));
        label_7->setText(QCoreApplication::translate("MainWindow", "IP:", nullptr));
        label_12->setText(QCoreApplication::translate("MainWindow", "ID:", nullptr));
        label_13->setText(QCoreApplication::translate("MainWindow", "PW:", nullptr));
        bt_SendMaps->setText(QCoreApplication::translate("MainWindow", "Send maps", nullptr));
        label_14->setText(QCoreApplication::translate("MainWindow", "Server", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_4), QCoreApplication::translate("MainWindow", "Server", nullptr));
        bt_TopoLoad->setText(QCoreApplication::translate("MainWindow", "Load", nullptr));
        lb_ScreenL->setText(QCoreApplication::translate("MainWindow", "0", nullptr));
        lb_ScreenR->setText(QCoreApplication::translate("MainWindow", "0", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_3), QCoreApplication::translate("MainWindow", "Temp", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
