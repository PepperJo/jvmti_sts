#ifndef STS_CONTROL_H
#define STS_CONTROL_H

#include <cstdint>
#include <string>
#include <chrono>

enum class CommandType {
    START,
    DUMP
};


struct StartCommand {
    template<class Rep, class Period>
    StartCommand(const std::chrono::duration<Rep, Period>& period)
        : type(CommandType::START),
        period_ms(std::chrono::duration_cast<std::chrono::milliseconds>(period).count())
    {
    }
    const CommandType type;
    std::uint32_t period_ms;
};

struct DumpCommand {
    DumpCommand(const std::string& name)
        : type(CommandType::DUMP)
    {
        auto copied = name.copy(filename,
                sizeof(filename) - 1);
        filename[copied] = '\0';
    }
    const CommandType type;
    char filename[256];
};

const std::uint32_t max_command_size =
    std::max(sizeof(StartCommand), sizeof(DumpCommand));

#endif /* STS_CONTROL_H */
