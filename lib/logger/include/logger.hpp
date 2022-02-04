#ifndef _LOGGER_HPP_
#define _LOGGER_HPP_

#include <stdio.h>
#include <stdarg.h>
#include <Arduino.h>

namespace RobotTweezers
{
    class Logger
    {
    private:
        
    public:
        static void Log(const char *fmt, ...);
    };
}

#endif // _LOGGER_HPP_
