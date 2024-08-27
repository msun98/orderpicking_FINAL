#include "option_writer.h"


Option_Writer::Option_Writer()
{

}

//bool::Option_Writer::WriteSettingFile(const std::string& file_name, posInfo option)
//{
//    file_storage.open(file_name, cv::FileStorage::WRITE);
//    if(file_storage.isOpened())
//    {
//        file_storage << "JOINT_1" << option._joint_1;
//        file_storage << "JOINT_2" << option._joint_2;
//        file_storage << "JOINT_3" << option._joint_3;
//        file_storage << "JOINT_4" << option._joint_4;
//        file_storage << "JOINT_5" << option._joint_5;
//        file_storage << "JOINT_6" << option._joint_6;

//        file_storage << "ACC" << option._acc;
//        file_storage << "SPD" << option._spd;
//    }
//    else
//    {
//        file_storage.release();
//        return false;
//    }
//    file_storage.release();
//    return true;
//}

//bool Option_Writer::ReadSettingFile(const std::string &file_name, posInfo& option)
//{

//    if(access(file_name.c_str(), F_OK))
//    {
//        qDebug() << "no file, create New One";
//        NewFile(file_name);
//        return false;
//    }

//    file_storage.open(file_name, cv::FileStorage::READ);
//    if(file_storage.isOpened())
//    {
//        file_storage["JOINT_1"] >> option._joint_1;
//        file_storage["JOINT_2"] >> option._joint_2;
//        file_storage["JOINT_3"] >> option._joint_3;
//        file_storage["JOINT_4"] >> option._joint_4;
//        file_storage["JOINT_5"] >> option._joint_5;
//        file_storage["JOINT_6"] >> option._joint_6;

//        file_storage["ACC"] >> option._acc;
//        file_storage["SPD"] >> option._spd;
//    }
//    else
//    {
//        qDebug() << "real No file";
//        file_storage.release();
//        return false;
//    }
//    file_storage.release();
//    return true;
//}

bool Option_Writer::NewFile(const std::string &file_name)
{
    file_storage.open(file_name, cv::FileStorage::WRITE);
    if(file_storage.isOpened())
    {
        file_storage << "JOINT_1" << -1;
        file_storage << "JOINT_2" << -1;
        file_storage << "JOINT_3" << -1;
        file_storage << "JOINT_4" << -1;
        file_storage << "JOINT_5" << -1;
        file_storage << "JOINT_6" << -1;

        file_storage << "ACC" << 0;
        file_storage << "SPD" << 0;
    }
    file_storage.release();
    return true;

}


bool Option_Writer::ReadSettingFile_LIFT(const std::string& file_name, liftInfo& option_Lift)
{

    QString log_file_name_ = "_LIFT/" + QString::fromStdString(file_name);
    if(access(log_file_name_.toStdString().c_str(), F_OK))
    {
        qDebug() << "no file, create New One";
        NewFile_LIFT(log_file_name_.toStdString());
        return false;
    }

    file_storage_LIFT.open(log_file_name_.toStdString(), cv::FileStorage::READ);
    if(file_storage_LIFT.isOpened())
    {
        file_storage_LIFT["TOP"] >> option_Lift.height_top;
        file_storage_LIFT["MID"] >> option_Lift.height_mid;
        file_storage_LIFT["BOT"] >> option_Lift.height_bot;
    }
    else
    {
        qDebug() << "nothing created.";
        file_storage_LIFT.release();
        return false;
    }
    file_storage_LIFT.release();
    return true;
}

bool Option_Writer::NewFile_LIFT(const std::string& file_name)
{
    file_storage_LIFT.open(file_name, cv::FileStorage::WRITE);
    if(file_storage_LIFT.isOpened())
    {
        file_storage_LIFT << "TOP" << -1;
        file_storage_LIFT << "MID" << -1;
        file_storage_LIFT << "BOT" << -1;
    }
    file_storage_LIFT.release();
    return true;
}

bool Option_Writer::WriteSettingFile_VISION(const std::string& file_name, xyzInfo option_Vision)
{
    QString log_file_name_ = "_VISION/" + QString::fromStdString(file_name);
    file_storage_VISION.open(log_file_name_.toStdString(), cv::FileStorage::WRITE);
    if(file_storage_VISION.isOpened())
    {
        file_storage_VISION << "X" << option_Vision.x;
        file_storage_VISION << "Y" << option_Vision.y;
        file_storage_VISION << "Z" << option_Vision.z;
    }
    else
    {
        file_storage_VISION.release();
        return false;
    }
    file_storage_VISION.release();
    return true;
}

bool Option_Writer::ReadSettingFile_VISION(const std::string& file_name, xyzInfo& option_Vision)
{
    QString log_file_name_ = "_VISION/" + QString::fromStdString(file_name);
    if(access(log_file_name_.toStdString().c_str(), F_OK))
    {
        qDebug() << "no file, create New One";
        NewFile_VISION(log_file_name_.toStdString());
        return false;
    }

    file_storage_VISION.open(log_file_name_.toStdString(), cv::FileStorage::READ);
    if(file_storage_VISION.isOpened())
    {
        file_storage_VISION["X"] >> option_Vision.x;
        file_storage_VISION["Y"] >> option_Vision.y;
        file_storage_VISION["Z"] >> option_Vision.z;
    }
    else
    {
        qDebug() << "nothing created.";
        file_storage_VISION.release();
        return false;
    }
    file_storage_VISION.release();
    return true;
}

bool Option_Writer::NewFile_VISION(const std::string& file_name)
{
    file_storage_VISION.open(file_name, cv::FileStorage::WRITE);
    if(file_storage_VISION.isOpened())
    {
        file_storage_VISION << "X" << -1;
        file_storage_VISION << "Y" << -1;
        file_storage_VISION << "Z" << -1;
    }
    file_storage_VISION.release();
    return true;
}


//bool::Option_Writer::WriteSettingFile_CALIB(const std::string& file_name, tcpInfo option_Tcp)
//{

//    QString log_file_name_ = "_CALIB/" + QString::fromStdString(file_name);
//    file_storage_RB5.open(log_file_name_.toStdString(), cv::FileStorage::WRITE);
//    if(file_storage_RB5.isOpened())
//    {
//        file_storage_RB5 << "X" << option_Tcp.x;
//        file_storage_RB5 << "Y" << option_Tcp.y;
//        file_storage_RB5 << "Z" << option_Tcp.z;
//        file_storage_RB5 << "RX" << option_Tcp.rx;
//        file_storage_RB5 << "RY" << option_Tcp.ry;
//        file_storage_RB5 << "RZ" << option_Tcp.rz;

//        file_storage_RB5 << "TCP_ACC" << option_Tcp.acc;
//        file_storage_RB5 << "TCP_SPD" << option_Tcp.spd;
//    }
//    else
//    {
//        file_storage_RB5.release();
//        return false;
//    }
//    file_storage_RB5.release();
//    return true;
//}

//bool Option_Writer::ReadSettingFile_CALIB(const std::string& file_name, tcpInfo& option_Tcp)
//{

//    QString log_file_name_ = "_CALIB/" + QString::fromStdString(file_name);
//    if(access(log_file_name_.toStdString().c_str(), F_OK))
//    {
//        qDebug() << "no file, create New One";
//        NewFile_RB5(file_name);
//        return false;
//    }

//    file_storage_RB5.open(log_file_name_.toStdString(), cv::FileStorage::READ);
//    if(file_storage_RB5.isOpened())
//    {
//        file_storage_RB5["X"] >> option_Tcp.x;
//        file_storage_RB5["Y"] >> option_Tcp.y;
//        file_storage_RB5["Z"] >> option_Tcp.z;
//        file_storage_RB5["RX"] >> option_Tcp.rx;
//        file_storage_RB5["RY"] >> option_Tcp.ry;
//        file_storage_RB5["RZ"] >> option_Tcp.rz;

//        file_storage_RB5["TCP_ACC"] >> option_Tcp.acc;
//        file_storage_RB5["TCP_SPD"] >> option_Tcp.spd;
//    }
//    else
//    {
//        qDebug() << "nothing created.";
//        file_storage_RB5.release();
//        return false;
//    }
//    file_storage_RB5.release();
//    return true;
//}

//bool Option_Writer::NewFile_CALIB(const std::string &file_name)
//{
//    file_storage_RB5.open(file_name, cv::FileStorage::WRITE);
//    if(file_storage_RB5.isOpened())
//    {
//        file_storage_RB5 << "X" << -1;
//        file_storage_RB5 << "Y" << -1;
//        file_storage_RB5 << "Z" << -1;
//        file_storage_RB5 << "RX" << -1;
//        file_storage_RB5 << "RY" << -1;
//        file_storage_RB5 << "RZ" << -1;

//        file_storage_RB5 << "TCP_ACC" << 0;
//        file_storage_RB5 << "TCP_SPD" << 0;
//    }
//    file_storage_RB5.release();
//    return true;
//}
