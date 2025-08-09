#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/Serial.hpp>
#include <px4_platform_common/log.h>
#include <px4_platform_common/sem.h>
#include <drivers/drv_hrt.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <time.h>

// Fixed-size buffer for passing between reader and writer tasks
struct DataBuffer {
    static constexpr size_t BUFFER_SIZE = 16 * 1024; // 16KB
    uint8_t data[BUFFER_SIZE];
    size_t length{0};
    bool in_use{false};
};

// Number of buffers in the pool
static constexpr size_t NUM_BUFFERS = 3;

class EulerNavDriver : public ModuleBase<EulerNavDriver>, public ModuleParams
{
public:
    EulerNavDriver(const char *device_name, uint32_t baud_rate = 115200);
    ~EulerNavDriver();

    static int task_spawn(int argc, char *argv[]);
    static EulerNavDriver *instantiate(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);

    int print_status() final;
    void run() final;

    // Accessor for should_exit() check in worker tasks
    bool exitRequested() const { return should_exit(); }

private:
    struct Statistics {
        uint32_t _total_bytes_received{0U};
        uint32_t _total_bytes_written{0U};
        uint32_t _write_errors{0U};
        uint32_t _buffer_overflows{0U};
        hrt_abstime _start_time{0U};
        char _log_filename[64]{0};
    };

    struct Config {
        static constexpr uint32_t TASK_STACK_SIZE{3072};
        static constexpr uint32_t SERIAL_READ_BUFFER_SIZE{1024};
        static constexpr uint32_t MIN_BYTES_TO_READ{1};
        static constexpr uint32_t SERIAL_READ_TIMEOUT_US{100};
        static constexpr const char *LOG_DIR_PATH{"/fs/microsd/log/eulernav"};
    };

    bool initialize();
    void deinitialize();
    bool createLogFile();
    bool createDirectory(const char *path);
    void generateFilename(char *buffer, size_t buffer_size);

    // Worker task entries (required signature)
    static int readerTaskEntry(int argc, char *argv[]);
    static int writerTaskEntry(int argc, char *argv[]);
    // Worker methods
    void readerTask();
    void writerTask();

    // Buffer pool and SPSC queue helpers
    DataBuffer *getAvailableBuffer();
    void queueFilledBuffer(DataBuffer *buffer);
    DataBuffer *getNextFilledBuffer();
    void releaseBuffer(DataBuffer *buffer);

    device::Serial _serial_port;
    uint8_t _serial_read_buffer[Config::SERIAL_READ_BUFFER_SIZE];
    Statistics _statistics{};
    volatile bool _is_initialized{false};
    int _log_fd{-1};
    uint32_t _baud_rate;
    char _device_name[64];

    // Multithreading state
    DataBuffer _buffer_pool[NUM_BUFFERS];
    px4_sem_t _free_buffers_sem;    // counts free buffers
    px4_sem_t _filled_buffers_sem;  // counts filled buffers ready to write

    volatile uint8_t _next_fill_index{0};

    // Filled buffer index queue (single-producer single-consumer)
    uint8_t _filled_queue[NUM_BUFFERS]{};
    volatile uint8_t _filled_head{0};
    volatile uint8_t _filled_tail{0};

    volatile int _reader_task_id{-1};
    volatile int _writer_task_id{-1};

    // Flags to track the actual running state of worker tasks
    volatile bool _reader_running{false};
    volatile bool _writer_running{false};

    // Single-instance pointer for worker task entries
    static EulerNavDriver *_instance;
};
