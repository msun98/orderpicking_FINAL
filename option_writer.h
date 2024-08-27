#ifndef OPTION_WRITER_H
#define OPTION_WRITER_H

#include <unistd.h>
#include "common_header.h"

#include <opencv2/core/persistence.hpp>
#include <string>
#include <QtDebug>


class Option_Writer
{
public:
    Option_Writer();

//    bool WriteSettingFile_LIFT(const std::string& file_name, liftInfo option_Lift);
    bool ReadSettingFile_LIFT(const std::string& file_name, liftInfo& option_Lift);
    bool NewFile_LIFT(const std::string& file_name);

    bool WriteSettingFile_VISION(const std::string& file_name, xyzInfo option_Vision);
    bool ReadSettingFile_VISION(const std::string& file_name, xyzInfo& option_Vision);
    bool NewFile_VISION(const std::string& file_name);

//    bool WriteSettingFile_CALIB(const std::string& file_name, tcpInfo option_Tcp);
//    bool ReadSettingFile_CALIB(const std::string& file_name, tcpInfo& option_Tcp);
//    bool NewFile_CALIB(const std::string& file_name);


//    bool WriteSettingFile(const std::string& file_name, posInfo option);
//    bool ReadSettingFile(const std::string& file_name, posInfo& option);

    bool NewFile(const std::string& file_name);


private:
    cv::FileStorage file_storage;

    cv::FileStorage file_storage_LIFT;
    cv::FileStorage file_storage_VISION;
    cv::FileStorage file_storage_CALIB;

};

#endif // OPTION_WRITER_H
