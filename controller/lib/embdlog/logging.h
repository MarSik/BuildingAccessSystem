#ifndef EMBD_LOGGING_H
#define EMBD_LOGGING_H

#include "Arduino.h"

class Logger
{
public:
    Logger(Stream& stream) : stream(stream), level(ERROR) {}

    enum Level {
        TRACE = 0,
        DEBUG = 1,
        INFO = 2,
        WARN = 3,
        ERROR = 4,
        NONE = 99
    };

    template <typename T>
    void print(const Level level, const T data) const {
        if (level >= this->level) {
            stream.print(data);
        }
    }

    template <typename T>
    void print(const Level level, const T data, const int flags) const {
        if (level >= this->level) {
            stream.print(data, flags);
        }
    }

    template <typename T>
    void write(const Level level, const T data) const {
        if (level >= this->level) {
            stream.write(data);
        }
    }

    template <typename T>
    void write(const Level level, const T data, const int flags) const {
        if (level >= this->level) {
            stream.write(data, flags);
        }
    }

    template <typename T>
    void println(const Level level, const T data) const {
        if (level >= this->level) {
            stream.println(data);
        }
    }

    template <typename T>
    void println(const Level level, const T data, const int flags) const {
        if (level >= this->level) {
            stream.println(data, flags);
        }
    }

    void println(Level level, const byte* data, const uint8_t len, const uint8_t format) const {
      if (level < this->level) return;

      for (uint8_t idx = 0; idx < len; idx++) {
        stream.print(*(data + idx), format);
        stream.write(' ');
      }
      stream.println("");
    }

    void setLevel(Level level) {
        this->level = level;
    }

    Logger logger(Level level) const {
        Logger logger = Logger(this->stream);
        logger.setLevel(level);
        return logger;
    }

private:
    Stream& stream;
    Level level;
};

extern Logger RootLogger;

#endif // EMBD_LOGGING_H
