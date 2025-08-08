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

    /// @brief The main loop of the task.
    void run() final;

private:
    /// @brief Driver performance indicators
    struct Statistics
    {
        uint32_t _total_bytes_received{0U}; ///< Total number of received bytes
        uint32_t _total_bytes_written{0U}; ///< Total number of bytes written to file
        uint32_t _write_errors{0U}; ///< File write error counter
        hrt_abstime _start_time{0U}; ///< Driver start time, [us]
        char _log_filename[64]{0}; ///< Current log file name
    };

    /// @brief Configuration constants
    struct Config
    {
        static constexpr uint32_t TASK_STACK_SIZE{3072}; ///< Driver task stack size (increased for file I/O)
        static constexpr uint32_t SERIAL_READ_BUFFER_SIZE{1024}; ///< Buffer size for serial port read operations
        static constexpr uint32_t MIN_BYTES_TO_READ{1}; ///< Minimum number of bytes to wait for when reading from a serial port
        static constexpr uint32_t SERIAL_READ_TIMEOUT_US{10000}; ///< A timeout for serial port read operation
        static constexpr uint32_t DATA_BUFFER_SIZE{16384}; ///< Size of ring buffer for storing RX data stream (increased for high baud rates)
        static constexpr uint32_t FILE_WRITE_CHUNK_SIZE{1024}; ///< Size of chunks to write to file (increased for efficiency)
        static constexpr const char* LOG_DIR_PATH{"/fs/microsd/log/eulernav"}; ///< Log directory path
    };

    /// @brief Perform initialization
    void initialize();

    /// @brief De-initialize
    void deinitialize();

    /// @brief Create log file with unique name
    /// @return True if file created successfully
    bool createLogFile();

    /// @brief Write data from ring buffer to log file
    void writeDataToFile();

    /// @brief Create directory if it doesn't exist
    /// @param path Directory path to create
    /// @return True if directory exists or was created successfully
    bool createDirectory(const char* path);

    /// @brief Generate unique filename
    /// @param buffer Output buffer for filename
    /// @param buffer_size Size of output buffer
    void generateFilename(char* buffer, size_t buffer_size);

    device::Serial _serial_port; ///< Serial port object to read data from
    Ringbuffer _data_buffer; ///< A buffer for RX data stream
    uint8_t _serial_read_buffer[Config::SERIAL_READ_BUFFER_SIZE]; ///< A buffer for serial port read operation
    uint8_t _file_write_buffer[Config::FILE_WRITE_CHUNK_SIZE]; ///< Buffer for file write operations
    Statistics _statistics{}; ///< Driver performance indicators
    bool _is_initialized{false}; ///< Initialization flag
    int _log_fd{-1}; ///< Log file descriptor
    uint32_t _baud_rate; ///< Serial port baud rate
    char _device_name[64]; ///< Serial device name for logging
};
