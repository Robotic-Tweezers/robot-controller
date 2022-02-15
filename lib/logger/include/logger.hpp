#ifndef _LOGGER_HPP_
#define _LOGGER_HPP_

#include <stdio.h>
#include <stdarg.h>
#include <Arduino.h>
#include <ArduinoEigen.h>
#include <TimeLib.h>

namespace RobotTweezers
{
    class Logger
    {
    private:
        Stream *serial;
        char msg_string[256];

        void SetTimeString(void);

    public:
        Logger(Stream *serial);

        void Log(const char *fmt, ...);

        void Log(Eigen::MatrixXf &matrix);

        void Log(Eigen::VectorXf &vector);

        void Error(const char *str);
    };
}

#endif // _LOGGER_HPP_
