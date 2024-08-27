#include "mainwindow.h"

#include <QApplication>
#include <QDebug>
#include <QtGui/QIcon>
//#include "md_motor.h"

Q_DECLARE_METATYPE(std::vector<double>)

motor_cmd MD_CMD;
ROBOT_CONFIG robot_config;
AMR_CONFIG amr_config;
CAM_CONFIG cam_config;

Eigen::Matrix4d string_to_TF(QString str);
void update_order_config();

Eigen::Matrix4d string_to_TF(QString str)
{

    // string is ZYX euler "tx,ty,tz,rx,ry,rz" (meter, deg)
    QStringList list = str.split(",");
    if(list.size() != 6)
    {
        printf("string invalid\n");
        return Eigen::Matrix4d::Identity();
    }

    double tx = list[0].toDouble();
    double ty = list[1].toDouble();
    double tz = list[2].toDouble();
    double rx = list[3].toDouble()*D2R;
    double ry = list[4].toDouble()*D2R;
    double rz = list[5].toDouble()*D2R;

    Eigen::AngleAxisd Rz(rz, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd Ry(ry, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd Rx(rx, Eigen::Vector3d::UnitX());

    Eigen::Quaternion<double> q = Rz * Ry * Rx;

    Eigen::Matrix4d res;
    res.setIdentity();

    res.block(0,0,3,3) = q.matrix();
    res(0,3) = tx;
    res(1,3) = ty;
    res(2,3) = tz;
    return res;
}

Eigen::Vector6d TF_to_ZYX(Eigen::Matrix4d TF)
{
    double tx = TF(0,3);
    double ty = TF(1,3);
    double tz = TF(2,3);

    Eigen::Matrix3d R = TF.block(0,0,3,3);
    Eigen::Vector3d euler = R.eulerAngles(2,1,0);

    double rx = euler[2];
    double ry = euler[1];
    double rz = euler[0];

    Eigen::Vector6d res;
    res[0] = tx;
    res[1] = ty;
    res[2] = tz;
    res[3] = rx;
    res[4] = ry;
    res[5] = rz;

    return res;
}

void update_order_config()
{
    // read config
    QString config_path = QDir::homePath()+"/order_document/robot_config.ini";
    QFileInfo config_info(config_path);
    if(config_info.exists() && config_info.isFile())
    {
        QSettings settings(config_path, QSettings::IniFormat);

        robot_config.shelves_name = settings.value("SHELVES/shelves").toString();
        robot_config.object = settings.value("OBJECT/object").toString();
        robot_config.shelves_list = robot_config.shelves_name.split(",");
        robot_config.object_list = robot_config.object.split(",");

        amr_config.AMR_ip = settings.value("CLIENT/AMR_ip").toString();
        amr_config.AMR_id = settings.value("CLIENT/AMR_id").toString();
        amr_config.AMR_pw = settings.value("CLIENT/AMR_pw").toString();
    }

    QString cam2tcp_tf_path = QDir::currentPath() + "/cam_config.ini";
    QFileInfo cam_config_info(cam2tcp_tf_path);
    if(cam_config_info.exists() && cam_config_info.isFile())
    {
        QSettings settings(cam2tcp_tf_path, QSettings::IniFormat);

        QString RB_TF_SENSOR;
        RB_TF_SENSOR = settings.value("CALIB/RB_TF_SENSOR").toString();
        cam_config.RB_TF_SENSOR = string_to_TF(RB_TF_SENSOR);

        QString KITECH_TF_SENSOR;
        KITECH_TF_SENSOR = settings.value("CALIB/KITECH_TF_SENSOR").toString();
        cam_config.KITECH_TF_SENSOR = string_to_TF(KITECH_TF_SENSOR);
    }
}

int main(int argc, char *argv[])
{
    qRegisterMetaType<std::vector<double>>();

    // read config
    update_order_config();

    QApplication a(argc, argv);
    MainWindow w;
    w.setWindowIcon(QIcon(":/img/resource/rb.png"));
    w.show();
    return a.exec();
}
