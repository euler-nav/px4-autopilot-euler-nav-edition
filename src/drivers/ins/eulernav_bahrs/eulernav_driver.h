#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/Serial.hpp>
#include <px4_platform_common/log.h>
#include <systemlib/err.h>
#include <Ringbuffer.hpp>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <time.h>
#include <atomic>
#include <px4_platform_common/sem.h>

// Fixed-size buffer for passing between reader and writer tasks
struct DataBuffer {
    static constexpr size_t BUFFER_SIZE = 16 * 1024; // 16KB buffer size
    uint8_t data[BUFFER_SIZE];
    size_t length{0};
    bool in_use{false};
};

// Number of buffers in the pool
static constexpr size_t NUM_BUFFERS = 3;

class EulerNavDriver : public ModuleBase<EulerNavDriver>, public ModuleParams
{
public:
    /// @brief Class constructor
    /// @param device_name Serial port to open
    /// @param baud_rate Serial baud rate
    EulerNavDriver(const char* device_name, uint32_t baud_rate = 115200);

    ~EulerNavDriver();

    /// @brief Required by ModuleBase
    static int task_spawn(int argc, char *argv[]);

    /// @brief Required by ModuleBase
    static EulerNavDriver* instantiate(int argc, char *argv[]);

    /// @brief Required by ModuleBase
    static int custom_command(int argc, char *argv[]);

    /// @brief Required by ModuleBase
    static int print_usage(const char *reason = nullptr);

    /// @brief Overload of the method from the ModuleBase
    int print_status() final;

    /// @brief The main driver task - spawns reader and writer tasks
    void run() final;

    // Accessor for should_exit() check in tasks
    bool exitRequested() const { return should_exit(); }

private:
    /// @brief Driver performance indicators
    struct Statistics
    {
        uint32_t _total_bytes_received{0U}; ///< Total number of received bytes
        uint32_t _total_bytes_written{0U};  ///< Total number of bytes written to file
        uint32_t _write_errors{0U};         ///< File write error counter
        uint32_t _buffer_overflows{0U};     ///< Count of buffer overflow events
        hrt_abstime _start_time{0U};        ///< Driver start time, [us]
        char _log_filename[64]{0};          ///< Current log file name
    };

    /// @brief Configuration constants
    struct Config
    {
        static constexpr uint32_t TASK_STACK_SIZE{3072};                 ///< Driver task stack size (increased for file I/O)
        static constexpr uint32_t SERIAL_READ_BUFFER_SIZE{1024};         ///< Buffer size for serial port read operations
        static constexpr uint32_t MIN_BYTES_TO_READ{1};                  ///< Minimum number of bytes to wait for when reading from a serial port
        static constexpr uint32_t SERIAL_READ_TIMEOUT_US{100};           ///< A timeout for serial port read operation (shorter for responsiveness)
        static constexpr const char* LOG_DIR_PATH{"/fs/microsd/log/eulernav"}; ///< Log directory path
    };

    /// @brief Perform initialization
    bool initialize();

    /// @brief De-initialize
    void deinitialize();

    /// @brief Create log file with unique name
    /// @return True if file created successfully
    bool createLogFile();

    /// @brief Create directory if it doesn't exist
    /// @param path Directory path to create
    /// @return True if directory exists or was created successfully
    bool createDirectory(const char* path);

    /// @brief Generate unique filename
    /// @param buffer Output buffer for filename
    /// @param buffer_size Size of output buffer
    void generateFilename(char* buffer, size_t buffer_size);

    static int readerTaskEntry(int argc, char *argv[]);
    void readerTask();

    /// @brief Writer task function - writes buffers to file
    static int writerTaskEntry(int argc, char *argv[]);
    void writerTask();

    /// @brief Get next available buffer for reading
    /// @return Pointer to available buffer or nullptr if none available
    DataBuffer* getAvailableBuffer();

    /// @brief Put a filled buffer into the queue for writing
    /// @param buffer Pointer to the filled buffer
    void queueFilledBuffer(DataBuffer* buffer);

    /// @brief Get next filled buffer for writing
    /// @return Pointer to next filled buffer or nullptr if none available
    DataBuffer* getNextFilledBuffer();

    /// @brief Mark a buffer as available for reuse
    /// @param buffer Pointer to the buffer to release
    void releaseBuffer(DataBuffer* buffer);

    device::Serial _serial_port;                           ///< Serial port object to read data from
    uint8_t _serial_read_buffer[Config::SERIAL_READ_BUFFER_SIZE]; ///< Temporary buffer for serial port reads
    Statistics _statistics{};                              ///< Driver performance indicators
    std::atomic<bool> _is_initialized{false};              ///< Initialization flag
    int _log_fd{-1};                                       ///< Log file descriptor
    uint32_t _baud_rate;                                   ///< Serial port baud rate
    char _device_name[64];                                 ///< Serial device name for logging

    // Multithreading related
    DataBuffer _buffer_pool[NUM_BUFFERS];                  ///< Fixed pool of data buffers
    px4_sem_t _free_buffers_sem;                           ///< Semaphore to track available buffers
    px4_sem_t _filled_buffers_sem;                         ///< Semaphore to track filled buffers
    std::atomic<uint8_t> _next_fill_index{0};              ///< Index for next buffer to fill
    std::atomic<uint8_t> _next_write_index{0};             ///< Index for next buffer to write

    // Filled buffer index queue (single-producer single-consumer)
    uint8_t _filled_queue[NUM_BUFFERS]{};
    std::atomic<uint8_t> _filled_head{0};
    std::atomic<uint8_t> _filled_tail{0};

    std::atomic<int> _reader_task_id{-1};                  ///< Reader task ID
    std::atomic<int> _writer_task_id{-1};                  ///< Writer task ID

    // Single-instance pointer for worker task entries
    static EulerNavDriver* _instance;
};
