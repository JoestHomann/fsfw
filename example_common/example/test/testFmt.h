#ifndef FSFW_EXAMPLE_HOSTED_TESTFMT_H
#define FSFW_EXAMPLE_HOSTED_TESTFMT_H

#include <fmt/chrono.h>
#include <fmt/color.h>
#include <fmt/compile.h>
#include <fmt/core.h>

#include <array>
#include <cstdint>

#include "fsfw/ipc/MutexFactory.h"
#include "fsfw/ipc/MutexGuard.h"
#include "fsfw/ipc/MutexIF.h"
#include "fsfw/timemanager/Clock.h"

#define __FILENAME_REL__ (((const char *)__FILE__ + SOURCE_PATH_SIZE))
#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

void fmtTests();

namespace sif {

static std::array<char, 524> PRINT_BUF = {};

static const char INFO_PREFIX[] = "INFO";
static const char DEBUG_PREFIX[] = "DEBUG";
static const char WARNING_PREFIX[] = "WARNING";
static const char ERROR_PREFIX[] = "ERROR";

enum class LogLevel : unsigned int { DEBUG = 0, INFO = 1, WARNING = 2, ERROR = 3 };

static const char *PREFIX_ARR[4] = {DEBUG_PREFIX, INFO_PREFIX, WARNING_PREFIX, ERROR_PREFIX};

static const std::array<fmt::color, 4> LOG_COLOR_ARR = {
    fmt::color::deep_sky_blue, fmt::color::forest_green, fmt::color::orange_red, fmt::color::red};

static MutexIF *PRINT_MUTEX = MutexFactory::instance()->createMutex();

static size_t writeTypePrefix(LogLevel level) {
  auto idx = static_cast<unsigned int>(level);
  const auto result =
      fmt::format_to_n(PRINT_BUF.begin(), PRINT_BUF.size() - 1,
                       fmt::runtime(fmt::format(fg(LOG_COLOR_ARR[idx]), PREFIX_ARR[idx])));
  return result.size;
}

template <typename... T>
size_t logTraced(LogLevel level, const char *file, unsigned int line, bool timed,
                 fmt::format_string<T...> fmt, T &&...args) noexcept {
  try {
    MutexGuard mg(PRINT_MUTEX);
    size_t bufPos = writeTypePrefix(level);
    auto currentIter = PRINT_BUF.begin() + bufPos;
    if (timed) {
      Clock::TimeOfDay_t logTime;
      Clock::getDateAndTime(&logTime);
      const auto result = fmt::format_to_n(currentIter, PRINT_BUF.size() - 1 - bufPos,
                                           " | {}[l.{}] | {:02}:{:02}:{:02}.{:03} | {}", file, line,
                                           logTime.hour, logTime.minute, logTime.second,
                                           logTime.usecond / 1000, fmt::format(fmt, args...));
      *result.out = '\0';
      bufPos += result.size;
    } else {
      const auto result =
          fmt::format_to_n(currentIter, PRINT_BUF.size() - 1 - bufPos, " | {}[l.{}] | {}", file,
                           line, fmt::format(fmt, args...));
      *result.out = '\0';
      bufPos += result.size;
    }

    fmt::print(fmt::runtime(PRINT_BUF.data()));
    return bufPos;
  } catch (const fmt::v8::format_error &e) {
    fmt::print("Printing failed with error: {}\n", e.what());
    return 0;
  }
}

template <typename... T>
size_t log(LogLevel level, bool timed, fmt::format_string<T...> fmt, T &&...args) noexcept {
  try {
    MutexGuard mg(PRINT_MUTEX);
    size_t bufPos = writeTypePrefix(level);
    auto currentIter = PRINT_BUF.begin() + bufPos;
    if (timed) {
      Clock::TimeOfDay_t logTime;
      Clock::getDateAndTime(&logTime);
      const auto result = fmt::format_to_n(
          currentIter, PRINT_BUF.size() - bufPos, " | {:02}:{:02}:{:02}.{:03} | {}", logTime.hour,
          logTime.minute, logTime.second, logTime.usecond / 1000, fmt::format(fmt, args...));
      bufPos += result.size;
    }
    fmt::print(fmt::runtime(PRINT_BUF.data()));
    return bufPos;
  } catch (const fmt::v8::format_error &e) {
    fmt::print("Printing failed with error: {}\n", e.what());
    return 0;
  }
}

template <typename... T>
void fdebug(const char *file, unsigned int line, fmt::format_string<T...> fmt,
            T &&...args) noexcept {
  logTraced(LogLevel::DEBUG, file, line, false, fmt, args...);
}

template <typename... T>
void fdebug_t(const char *file, unsigned int line, fmt::format_string<T...> fmt,
              T &&...args) noexcept {
  logTraced(LogLevel::DEBUG, file, line, true, fmt, args...);
}

template <typename... T>
void finfo_t(fmt::format_string<T...> fmt, T &&...args) {
  log(LogLevel::INFO, true, fmt, args...);
}

template <typename... T>
void finfo(fmt::format_string<T...> fmt, T &&...args) {
  log(LogLevel::INFO, false, fmt, args...);
}

template <typename... T>
void fwarning(const char *file, unsigned int line, fmt::format_string<T...> fmt, T &&...args) {
  logTraced(LogLevel::WARNING, file, line, false, fmt, args...);
}

template <typename... T>
void fwarning_t(const char *file, unsigned int line, fmt::format_string<T...> fmt, T &&...args) {
  logTraced(LogLevel::WARNING, file, line, true, fmt, args...);
}

template <typename... T>
void ferror(const char *file, unsigned int line, fmt::format_string<T...> fmt, T &&...args) {
  logTraced(LogLevel::ERROR, file, line, false, fmt, args...);
}

template <typename... T>
void ferror_t(const char *file, unsigned int line, fmt::format_string<T...> fmt, T &&...args) {
  logTraced(LogLevel::ERROR, file, line, true, fmt, args...);
}

}  // namespace sif

#define FSFW_LOGI(format, ...) finfo(FMT_STRING(format), __VA_ARGS__)

#define FSFW_LOGIT(format, ...) finfo_t(FMT_STRING(format), __VA_ARGS__)

#define FSFW_LOGD(format, ...) sif::fdebug(__FILENAME__, __LINE__, FMT_STRING(format), __VA_ARGS__)

#define FSFW_LOGDT(format, ...) fdebug_t(__FILENAME__, __LINE__, FMT_STRING(format), __VA_ARGS__)

#define FSFW_LOGW(format, ...) fdebug(__FILENAME__, __LINE__, FMT_STRING(format), __VA_ARGS__)

#define FSFW_LOGWT(format, ...) fdebug_t(__FILENAME__, __LINE__, FMT_STRING(format), __VA_ARGS__)

#define FSFW_LOGE(format, ...) fdebug(__FILENAME__, __LINE__, FMT_STRING(format), __VA_ARGS__)

#define FSFW_LOGET(format, ...) fdebug_t(__FILENAME__, __LINE__, FMT_STRING(format), __VA_ARGS__)

#endif  // FSFW_EXAMPLE_HOSTED_TESTFMT_H
