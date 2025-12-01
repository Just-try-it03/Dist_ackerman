/*
 * Author: BlindZhou
 */

#ifndef SIMPLE_LOGGER_H
#define SIMPLE_LOGGER_H

#include <iostream>
#include <fstream>
#include <string>
#include <thread>
#include <mutex>
#include <deque>
#include <sys/stat.h>
#include <unistd.h>
#include <dirent.h>
#include <condition_variable>
#include <vector>
#include <algorithm>
#include <stdarg.h>
#include <ctime>

class SimpleLogger {
public:
    enum LogLevel { DEBUG, INFO, WARN, ERROR };

    static SimpleLogger& getInstance(const std::string& logDir = "", const std::string& logFilePrefix = "", size_t maxFiles = 0, size_t maxFileSize = 0) {
        static SimpleLogger instance(logDir, logFilePrefix, maxFiles, maxFileSize);
        return instance;
    }

    void log(LogLevel level, const char* format, ...)
    {
        va_list aptr;
        std::string str;
        int len = 0;
        va_start(aptr, format);
        // ret = vsprintf(nullptr, format, aptr);
        len = vsnprintf(nullptr, 0, format, aptr);
        va_end(aptr);
        ++len;
        str.resize(len);
        va_start(aptr, format);
        vsnprintf(const_cast<char*>(str.data()), len, format, aptr);
        va_end(aptr);
        log(level, str);
    }

private:
    std::string logDir;
    std::string logFilePrefix;
    size_t maxFiles;
    size_t maxFileSize;
    std::deque<std::string> writeBuffer;
    std::deque<std::string> readBuffer;
    std::mutex queueMutex;
    std::condition_variable cv;
    std::thread worker;
    bool exitFlag;

    SimpleLogger(const std::string& logDir, const std::string& logFilePrefix, size_t maxFiles, size_t maxFileSize)
        : logDir(logDir), logFilePrefix(logFilePrefix), maxFiles(maxFiles), maxFileSize(maxFileSize), exitFlag(false) {
        if (logDir.empty() || logFilePrefix.empty() || maxFiles == 0 || maxFileSize == 0) {
            throw std::runtime_error("Invalid logger configuration");
        }

        if (mkdir(logDir.c_str(), 0755) && errno != EEXIST) {
            std::cerr << "Failed to create directory: " << logDir << std::endl;
        }
        manageLogFiles();
        worker = std::thread(&SimpleLogger::processQueue, this);
    }

    ~SimpleLogger() {
        {
            std::lock_guard<std::mutex> lock(queueMutex);
            exitFlag = true;
        }
        cv.notify_all();
        if (worker.joinable()) {
            worker.join();
        }
    }

    SimpleLogger(const SimpleLogger&) = delete;
    SimpleLogger& operator=(const SimpleLogger&) = delete;

    void log(LogLevel level, const std::string& message) {
        std::string logMessage = formatMessage(level, message);
        printf("%s\n", logMessage.c_str());
        {
            std::lock_guard<std::mutex> lock(queueMutex);
            writeBuffer.push_back(logMessage);
        }
        cv.notify_all();
    }
    void processQueue() {
            std::ofstream logFile;
            std::string currentLogFileName = generateLogFileName();
            logFile.open(currentLogFileName, std::ios::out | std::ios::app);
            while (true) {
                std::unique_lock<std::mutex> lock(queueMutex);
                //cv.wait(lock, [this] { return !writeBuffer.empty() || exitFlag; });
                cv.wait_for(lock, std::chrono::milliseconds(1000), [this] { return !writeBuffer.empty() || exitFlag; });

                if (exitFlag && writeBuffer.empty()) break;

                std::swap(writeBuffer, readBuffer);
                while (!readBuffer.empty()) {
                    std::string message = readBuffer.front();
                    readBuffer.pop_front();
                    if (isFileTooLarge(currentLogFileName)) {
                        logFile.close();
                        manageLogFiles();
                        currentLogFileName = generateLogFileName();
                        logFile.open(currentLogFileName, std::ios::out | std::ios::app);
                        if (logFile.fail()) {
                            std::cerr << "Failed to open log file: " << currentLogFileName << std::endl;
                            continue;
                        }
                    }
                    logFile << message << std::endl;
                }
                lock.unlock();
            }
            if (logFile.is_open()) {
                logFile.close();
            }
            //manageLogFiles();
        }

        std::string formatMessage(LogLevel level, const std::string& message) {
            const char* levelStr;
            switch (level) {
                case DEBUG: levelStr = "DEBUG"; break;
                case INFO:  levelStr = "INFO"; break;
                case WARN:  levelStr = "WARN"; break;
                case ERROR: levelStr = "ERROR"; break;
            }
            std::time_t now = std::time(nullptr);
            static char timeBuffer[256];
            std::strftime(timeBuffer, sizeof(timeBuffer), "%Y-%m-%d %H:%M:%S", std::localtime(&now));
            const auto now_time_point = std::chrono::system_clock::now();

            std::stringstream ss;
            ss <<timeBuffer << "." << std::setw(3) << std::setfill('0') <<
                 std::chrono::duration_cast<std::chrono::milliseconds>
                 (now_time_point.time_since_epoch()).count() % 1000 <<
                 " [" << levelStr << "] " <<message << "\n";
            return ss.str();
            //return std::string(timeBuffer) + " [" + levelStr + "] " + message;
        }

        std::string generateLogFileName() {
            std::time_t now = std::time(nullptr);
            std::tm* now_tm = std::localtime(&now);
            char buffer[256];
            std::strftime(buffer, sizeof(buffer), "%Y-%m%d-%H-%M-%S", now_tm);
            return logDir + "/" + logFilePrefix + "_" + buffer + ".log";
        }

        bool isFileTooLarge(const std::string& fileName) {
            struct stat fileStat;
            if (stat(fileName.c_str(), &fileStat) != 0) {
                return false;
            }
            return fileStat.st_size > maxFileSize;
        }

        void manageLogFiles() {
                std::vector<std::string> files;
                DIR* dir = opendir(logDir.c_str());
                if (dir) {
                    struct dirent* entry;
                    while ((entry = readdir(dir)) != nullptr) {
                        std::string filename = entry->d_name;
                        if (filename.find(logFilePrefix) != std::string::npos && filename.find("back_") == std::string::npos) {
                            files.push_back(filename);
                        }
                    }
                    closedir(dir);
                }

                // Rename existing log files with "back_" prefix
                for (const std::string& file : files) {
                    std::string oldPath = logDir + "/" + file;
                    std::string newPath = logDir + "/back_" + file;
                    rename(oldPath.c_str(), newPath.c_str());
                }

                // Check if the total number of backup files exceeds the limit
                files.clear();
                dir = opendir(logDir.c_str());
                if (dir) {
                    struct dirent* entry;
                    while ((entry = readdir(dir)) != nullptr) {
                        std::string filename = entry->d_name;
                        if (filename.find("back_" + logFilePrefix) != std::string::npos) {
                            files.push_back(filename);
                        }
                    }
                    closedir(dir);
                }

                if (files.size() > maxFiles) {
                    std::sort(files.begin(), files.end(), [this](const std::string& a, const std::string& b) {
                        std::string aPath = logDir + "/" + a;
                        std::string bPath = logDir + "/" + b;
                        struct stat aStat, bStat;
                        stat(aPath.c_str(), &aStat);
                        stat(bPath.c_str(), &bStat);
                        return aStat.st_mtime < bStat.st_mtime;
                    });

                    for (size_t i = 0; i < files.size() - maxFiles; ++i) {
                        std::string filePath = logDir + "/" + files[i];
                        remove(filePath.c_str());
                    }
                }
            }
};

const constexpr char* log_dir = "/usr/local/urobot/logs";
const constexpr char* log_name = "zmq_server";
const constexpr int max_files = 7;
const constexpr int max_size = 10* 1024 * 1024;

#define PRINT_INFO(format, args...)    SimpleLogger::getInstance(log_dir, log_name, max_files, max_size).log(SimpleLogger::INFO, format,  ##args);
#define PRINT_DEBUG(format, args...)    SimpleLogger::getInstance(log_dir, log_name, max_files, max_size).log(SimpleLogger::DEBUG, format,  ##args);
#define PRINT_WARN(format, args...)    SimpleLogger::getInstance(log_dir, log_name, max_files, max_size).log(SimpleLogger::WARN, format,  ##args);
#define PRINT_ERROR(format, args...)    SimpleLogger::getInstance(log_dir, log_name, max_files, max_size).log(SimpleLogger::ERROR, format,  ##args);



#endif // SIMPLE_LOGGER_H
