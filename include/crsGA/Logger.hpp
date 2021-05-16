#pragma once

#include "Export.hpp"
#include "ThreadSafeQueue.hpp"

#include <string>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <ctime>
#include <map>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <condition_variable>
#include <atomic>

namespace crsGA {

    enum LogLevel
    {
        LOG_OFF = -1,
        LOG_FATAL = 0,
        LOG_WARNING = 1,
        LOG_NOTICE = 2,
        LOG_DEBUG = 3,
        LOG_VERBOSE = 4
    };

    class CRSGA_EXPORT LogHandler
    {
    public:
        LogHandler(){}
        LogHandler(const LogHandler &){}
        LogHandler& operator=(LogHandler const &){
            return *this;
        }
        virtual ~LogHandler(){}
        virtual void log(const std::chrono::time_point<std::chrono::system_clock>& /*when*/, LogLevel /*level*/, const std::string& /*str*/) = 0;
    protected:
        std::string toDateTime(const std::chrono::time_point<std::chrono::system_clock>& when);
        std::string buildOutpuMessage(const std::chrono::time_point<std::chrono::system_clock>& when, LogLevel level, const std::string& str);
    };

    /**
     * @brief The Logger class
     *
     * Multithreaded logger class which allows to save to multiple log handlers or 
     * by default to the standard output
     */
    class CRSGA_EXPORT Logger
    {
    public:
        Logger();
        virtual ~Logger();

        void log(LogLevel logLevel, const char* file, int line, const std::string& entry);


        typedef std::shared_ptr<LogHandler>           LogHandlerPtr;
        typedef std::map<std::string, LogHandlerPtr > LogHandlerMap;

        bool addLogHandler(LogHandlerPtr logHandler, const std::string &id);
        bool removeLogHandler(const std::string &id);
        void setLogLevel(LogLevel level) { _currentLogLevel = level; }

        void startProcessingMessages();
        void stopProcessingMessages();
        bool isProcessingMessages() const;

        

        #define LOGFUNC(loglevel, func) template <typename... Args> void func(const char* file, int line, const Args&... args) \
                { \
            std::ostringstream msg; \
            msg << std::setprecision(15); \
            log_recursive(loglevel, file, line, msg, args...); \
        }

        LOGFUNC(LOG_FATAL,   fatal)
        LOGFUNC(LOG_WARNING, warning)
        LOGFUNC(LOG_NOTICE,  notice)
        LOGFUNC(LOG_DEBUG,   debug)
        LOGFUNC(LOG_VERBOSE, verbose)



    protected:
        // "Recursive" variadic function
        template<typename T, typename... Args>
        void log_recursive(LogLevel logLevel, const char* file, int line, std::ostringstream& msg,
            T value, const Args&... args)
        {
            msg << value;
            log_recursive(logLevel, file, line, msg, args...);
        }

        // Terminator
        void log_recursive(LogLevel logLevel, const char* file, int line, std::ostringstream& msg)
        {
            log(logLevel, file, line, msg.str());
        }

        struct LogMessage
        {
            LogLevel                                           level;
            std::chrono::time_point<std::chrono::system_clock> when;
            const char*                                        file;
            int                                                line;
            std::string                                        msg;
        };


        void processEntries();
        void dispatchLogMessage(const LogMessage& logMessage);
    private:
        Logger(const Logger& src);
        Logger& operator =(const Logger& rhs);

        LogHandlerMap                       _logHandlersMap;
        LogLevel                            _currentLogLevel;

        std::atomic<bool>                   _exit;
        std::mutex                          _mutex;
        ThreadSafeQueue<LogMessage>         _queue;
        std::thread                         _thread;

    };

    extern CRSGA_EXPORT Logger &getLogger();
}

#define ENABLE_LOGGER 1
#if ENABLE_LOGGER
#define LOG_FATAL(...) crsGA::getLogger().fatal(__FILE__, __LINE__, __VA_ARGS__);
#define LOG_WARNING(...) crsGA::getLogger().warning(__FILE__, __LINE__, __VA_ARGS__);
#define LOG_NOTICE(...) crsGA::getLogger().notice(__FILE__, __LINE__, __VA_ARGS__);
#define LOG_DEBUG(...) crsGA::getLogger().debug(__FILE__, __LINE__, __VA_ARGS__);
#define LOG_VERBOSE(...) crsGA::getLogger().verbose(__FILE__, __LINE__, __VA_ARGS__);
#else
template<typename TF>
void write_debug_output(std::ostream & out, TF const& f) {
    out << f << std::endl;
}

template<typename TF, typename ... TR>
void write_debug_output(std::ostream & out, TF const& f, TR const& ... rest) {
    out << f << " ";
    write_debug_output(out, rest...);
}
#define LOG_FATAL(...) write_debug_output( std::cerr, __FILE__, __LINE__, __VA_ARGS__ )
#define LOG_WARNING(...) write_debug_output( std::cerr, __FILE__, __LINE__, __VA_ARGS__ )
#define LOG_NOTICE(...) write_debug_output( std::cout, __FILE__, __LINE__, __VA_ARGS__ )
#define LOG_DEBUG(...) write_debug_output( std::cout, __FILE__, __LINE__, __VA_ARGS__ )
#define LOG_VERBOSE(...) write_debug_output( std::cout, __FILE__, __LINE__, __VA_ARGS__ )
#endif

