#include <logger.hpp>

void RobotTweezers::Logger::SetTimeString(void)
{
    sprintf(msg_string, "%d:%d:%d ", hour(), minute(), second());
}

RobotTweezers::Logger::Logger(Stream *serial)
{
#ifdef DEBUG
    this->serial = serial;
#endif // DEBUG
}

void RobotTweezers::Logger::Log(const char *fmt, ...)
{
#ifdef DEBUG
    char buffer[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, 256, fmt, args);
    SetTimeString();
    strcat(msg_string, buffer);
    serial->println(msg_string);
#endif // DEBUG
}

void RobotTweezers::Logger::Log(Eigen::MatrixXf& matrix)
{
#ifdef DEBUG
    for (uint16_t i = 0; i < matrix.rows(); i++)
    {
        for (uint16_t j = 0; j < matrix.cols(); j++)
        {
            serial->print(matrix(i, j));
            serial->print(" ");
        }
        
        serial->println(); 
    }
#endif // DEBUG
}

void RobotTweezers::Logger::Log(Eigen::VectorXf& vector)
{
#ifdef DEBUG
    for (uint16_t i = 0; i < vector.rows(); i++)
    {
        for (uint16_t j = 0; j < vector.cols(); j++)
        {
            serial->print(vector(i, j));
            serial->print(" ");
        }
        
        serial->println(); 
    }
#endif // DEBUG
}

void RobotTweezers::Logger::Error(const char *str)
{
    Serial.println(str);
    while (true)
    {
        digitalWrite(LED_BUILTIN, LOW);
        delay(1000);
        digitalWrite(LED_BUILTIN, HIGH);
        delay(1000);
    }
}
