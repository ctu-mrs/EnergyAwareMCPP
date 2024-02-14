#ifndef THESIS_PATH_GENERATOR_LOGGERROS_H
#define THESIS_PATH_GENERATOR_LOGGERROS_H

#include "SimpleLogger.h"
#include <ros/ros.h>

namespace loggers {
    class RosLogger: public SimpleLogger {
    public:
        void log_info(const std::string &s) override {
            if (m_log_level <= LOG_INFO)
                ROS_INFO_STREAM("[PathGenerator]: " << s);
        }
        void log_err(const std::string &s) override {
            if (m_log_level <= LOG_ERR)
                ROS_ERROR_STREAM("[PathGenerator]: " << s);
        }
        void log_debug(const std::string &s) override {
            if (m_log_level <= LOG_DEBUG)
                ROS_INFO_STREAM("[PathGenerator]: " << s);
        }
        void log_warn(const std::string &s) override {
            if (m_log_level <= LOG_WARN)
                ROS_WARN_STREAM("[PathGenerator]: " << s);
        }

        ~RosLogger() override = default;
    };
}


#endif //THESIS_PATH_GENERATOR_LOGGERROS_H
