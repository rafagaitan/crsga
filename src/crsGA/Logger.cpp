#if defined(WIN32)
#define _CRT_SECURE_NO_WARNINGS
#endif
#if defined(WIN32)
    #include "Windows.h"
#endif

#include <crsGA/Logger.hpp>

using namespace crsGA;

std::string getEnvironment(const std::string &env)
{
    std::string ret;
#ifdef WIN32
    char* envvar;
    size_t requiredSize;

    getenv_s(&requiredSize, NULL, 0, env.c_str());

    envvar = new char[requiredSize];
    if (!envvar)
    {
        LOG_FATAL("Failed allocating memory!");
        return ret;
    }
    // Get the value of the LIB environment variable.
    getenv_s(&requiredSize, envvar, requiredSize, env.c_str());
    ret = envvar;
#else
    ret = getenv(env.c_str());
#endif
    return ret;
}

std::string getRootDir()
{
    std::string ret;
#ifdef WIN32
    ret = getEnvironment("SystemDrive");
    if (ret.empty())
        ret = "c:";
    ret.append("/");
#else
    ret = "/";
#endif
    return ret;
}

std::string getTempDir()
{
#if defined(WIN32)
    TCHAR tempPath[MAX_PATH];
    if (GetTempPath(sizeof(tempPath), tempPath))
    {
        std::string thePath = tempPath;
        return thePath;
    }
    return getRootDir() + std::string("TEMP");
#else
    return std::string("/tmp/");
#endif
}

std::string LogHandler::toDateTime(const std::chrono::time_point<std::chrono::system_clock>& when)
{
    auto t = std::chrono::system_clock::to_time_t(when);
    auto dateTime = std::string(std::ctime(&t));
    dateTime = dateTime.substr(0, 24);
    return dateTime;
}

std::string LogHandler::buildOutpuMessage(const std::chrono::time_point<std::chrono::system_clock>& when, LogLevel level, const std::string& str)
{
    std::ostringstream msg;
    switch (level)
    {
    case LOG_FATAL:
        msg << "[" << toDateTime(when) << "] <FATAL>:" << str;
        break;
    case LOG_WARNING:
        msg << "[" << toDateTime(when) << "] <WARNING>: " << str;
        break;
    case LOG_NOTICE:
        msg << "[" << toDateTime(when) << "] <NOTICE>:  " << str;
        break;
    case LOG_DEBUG:
        msg << "[" << toDateTime(when) << "] <DEBUG>:   " << str;
        break;
    case LOG_VERBOSE:
        msg << "[" << toDateTime(when) << "] <VERBOSE>: " << str;
        break;
    default:
        break;
    }
    return msg.str();
}


class StandardOutputLogHandler : public LogHandler
{
public:
    StandardOutputLogHandler()
    : LogHandler()
    {
    }

    void log(const std::chrono::time_point<std::chrono::system_clock>& when, LogLevel level, const std::string& str)
    {
        auto msg = buildOutpuMessage(when, level, str)+'\n';
        switch (level)
        {
        case LOG_FATAL:
        case LOG_WARNING:
            std::fputs(msg.c_str(), stderr);
            break;
        case LOG_NOTICE:
        case LOG_DEBUG:
        case LOG_VERBOSE:
            std::fputs(msg.c_str(), stdout);
            break;
        default:
            break;
        }
    }
protected:
    StandardOutputLogHandler(const StandardOutputLogHandler &rhs);
    StandardOutputLogHandler& operator=(StandardOutputLogHandler const &rhs);
};

class DefaultFileLogHandler : public LogHandler
{
public:
    DefaultFileLogHandler()
        : LogHandler()
        , _logFile(getTempDir() + std::string("crsGA.log"))
    {
    }

    ~DefaultFileLogHandler()
    {
        _logFile.close();
    }

    void log(const std::chrono::time_point<std::chrono::system_clock>& when, LogLevel level, const std::string& str)
    {
        _logFile << buildOutpuMessage(when, level, str) << std::endl;
        _logFile.flush();
    }
protected:
    DefaultFileLogHandler(const DefaultFileLogHandler &rhs);
    DefaultFileLogHandler& operator=(DefaultFileLogHandler const &rhs);

    std::ofstream _logFile;
};



Logger &crsGA::getLogger()
{
    static Logger s_logger;
    return s_logger;
}

Logger::Logger()
    : _logHandlersMap()
    , _currentLogLevel(LOG_NOTICE)
    , _exit(false)
    , _mutex()
    , _queue()
    , _thread()
{
    addLogHandler(std::make_shared<StandardOutputLogHandler>(), std::string("StandardOutputLogHandler"));
    addLogHandler(std::make_shared<DefaultFileLogHandler>(), std::string("DefaultFileLogHandler"));
}

Logger::~Logger()
{
}

void Logger::log(LogLevel logLevel, const char* file, int line, const std::string& entry)
{
    LogMessage logMessage;
    logMessage.file = file;
    logMessage.line = line;
    logMessage.level = logLevel;
    logMessage.msg = entry;
    logMessage.when = std::chrono::system_clock::now();
    if (_thread.joinable())
        _queue.push(std::move(logMessage));
    else
        dispatchLogMessage(logMessage);
}

void Logger::dispatchLogMessage(const LogMessage& logMessage)
{
    if (logMessage.level <= _currentLogLevel)
    {
        for (auto itr = _logHandlersMap.begin();
            itr != _logHandlersMap.end();
            ++itr)
        {
            std::ostringstream msg;
            msg << logMessage.msg << " at " << logMessage.file << "(" << logMessage.line << ")";
            ((*itr).second)->log(logMessage.when, logMessage.level, msg.str());
        }
    }
}

void Logger::processEntries()
{
    while (!_exit)
    {
        LogMessage logMessage;
        if (_queue.wait_pop(logMessage))
        {
            dispatchLogMessage(logMessage);
        }
    }
}

bool Logger::addLogHandler(Logger::LogHandlerPtr logHandler, const std::string &id)
{
    std::lock_guard<std::mutex> lock(_mutex);

    _logHandlersMap[id] = logHandler;
    return true;
}

bool Logger::removeLogHandler(const std::string &id)
{
    std::lock_guard<std::mutex> lock(_mutex);

    auto itr = _logHandlersMap.find(id);
    if (itr != _logHandlersMap.end())
    {
        _logHandlersMap.erase(itr);
        return true;
    }
    return false;
}

void Logger::stopProcessingMessages()
{
    _exit = true;
    _queue.notify_and_terminate();
    if (_thread.joinable())
        _thread.join();
    // purge messages
    while (!_queue.empty())
    {
        LogMessage logMessage;
        if (_queue.pop(logMessage))
        {
            dispatchLogMessage(logMessage);
        }
    }
}

void Logger::startProcessingMessages()
{
    _exit = false;
    _thread = std::thread(&Logger::processEntries, this);
}

bool Logger::isProcessingMessages() const
{
    return _thread.joinable();
}


