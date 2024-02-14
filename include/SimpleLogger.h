#ifndef THESIS_PATH_GENERATOR_SIMPLELOGGER_H
#define THESIS_PATH_GENERATOR_SIMPLELOGGER_H

#include <string>
#include <iostream>

// Simple logger class for controlled logging and being able to log debug info directly into ROS from classes that should not be dependent on ROS.
// For example, MapPolygon used to include <ros/ros.h> just for logging

namespace loggers {
    enum LoggerLevel {
        LOG_DEBUG,
        LOG_INFO,
        LOG_WARN,
        LOG_ERR,
        LOG_NONE
    };


    class SimpleLogger {
    protected:
        LoggerLevel m_log_level = LOG_DEBUG;
    public:
        void set_log_level(LoggerLevel level) { m_log_level = level; };

        virtual void log_info(const std::string &s) { if (m_log_level <= LOG_INFO) std::cout << s << std::endl; };

        virtual void log_debug(const std::string &s) { if (m_log_level <= LOG_DEBUG) std::cout << s << std::endl; };

        virtual void log_warn(const std::string &s) { if (m_log_level <= LOG_WARN) std::cout << s << std::endl; };

        virtual void log_err(const std::string &s) { if (m_log_level <= LOG_ERR) std::cout << s << std::endl; };

        virtual ~SimpleLogger() = default;
    };
}

#endif //THESIS_PATH_GENERATOR_SIMPLELOGGER_H
