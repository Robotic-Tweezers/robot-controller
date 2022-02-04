#include <logger.hpp>

void RobotTweezers::Logger::Log(const char *fmt, ...)
{
#ifdef DEBUG
    char buffer[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(fmt), fmt, args);
    Serial.println(buffer);
#endif // DEBUG
}

