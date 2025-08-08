#include "eulernav_driver.h"
#include <px4_platform_common/getopt.h>
#include <errno.h>
#include <string.h>
#include <inttypes.h>

EulerNavDriver::EulerNavDriver(const char* device_name, uint32_t baud_rate)
    : ModuleParams{nullptr}
    , _serial_port{device_name, baud_rate, ByteSize::EightBits, Parity::None, StopBits::One, FlowControl::Disabled}
    , _data_buffer{}
    , _baud_rate{baud_rate}
{
    // Store device name for logging
    strncpy(_device_name, device_name, sizeof(_device_name) - 1);
    _device_name[sizeof(_device_name) - 1] = '\0';

    initialize();
}

EulerNavDriver::~EulerNavDriver()
{
    deinitialize();
}

int EulerNavDriver::task_spawn(int argc, char *argv[])
{
    int task_id = px4_task_spawn_cmd("eulernav_log", SCHED_DEFAULT, SCHED_PRIORITY_SLOW_DRIVER,
                     Config::TASK_STACK_SIZE, (px4_main_t)&run_trampoline, argv);

    if (task_id < 0)
    {
        _task_id = -1;
        PX4_ERR("Failed to spawn task.");
    }
    else
    {
        _task_id = task_id;
    }

    return (_task_id < 0) ? 1 : 0;
}

EulerNavDriver* EulerNavDriver::instantiate(int argc, char *argv[])
{
    int option_index = 1;
    const char* option_arg{nullptr};
    const char* device_name{nullptr};
    uint32_t baud_rate{115200};

    while (true)
    {
        int option{px4_getopt(argc, argv, "d:b:", &option_index, &option_arg)};

        if (EOF == option)
        {
            break;
        }

        switch (option)
        {
        case 'd':
            device_name = option_arg;
            break;
        case 'b':
            baud_rate = static_cast<uint32_t>(atoi(option_arg));
            if (baud_rate < 9600 || baud_rate > 921600) {
                PX4_WARN("Invalid baud rate %" PRIu32 ", using default 115200", baud_rate);
                baud_rate = 115200;
            }
            break;
        default:
            break;
        }
    }

    if (!device_name) {
        PX4_ERR("Device name is required");
        return nullptr;
    }

    return new EulerNavDriver(device_name, baud_rate);
}

int EulerNavDriver::custom_command(int argc, char *argv[])
{
    return print_usage("unrecognized command");
}

int EulerNavDriver::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description

Serial data logger for the EULER-NAV BAHRS device.
Logs raw serial data to binary files on SD card.

### Examples

Start logger on specified serial device with default baud rate (115200)
$ eulernav_bahrs start -d /dev/ttyS1

Start logger with custom baud rate
$ eulernav_bahrs start -d /dev/ttyS1 -b 921600

Stop logger
$ eulernav_bahrs stop
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("eulernav_bahrs", "driver");
    PRINT_MODULE_USAGE_SUBCATEGORY("ins");
    PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start data logger");
    PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, nullptr, "Serial device", true);
    PRINT_MODULE_USAGE_PARAM_INT('b', 115200, 9600, 921600, "Baud rate", false);
    PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Print logger status");
    PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop data logger");

    return PX4_OK;
}

int EulerNavDriver::print_status()
{
    if (_is_initialized)
    {
        PX4_INFO("EULER-NAV Data Logger Status:");
        PX4_INFO("Elapsed time: %llu [us]", hrt_elapsed_time(&_statistics._start_time));
        PX4_INFO("Baud rate: %" PRIu32, _baud_rate);
        PX4_INFO("Log file: %s", _statistics._log_filename);
        PX4_INFO("Total bytes received: %" PRIu32, _statistics._total_bytes_received);
        PX4_INFO("Total bytes written: %" PRIu32, _statistics._total_bytes_written);
        PX4_INFO("Write errors: %" PRIu32, _statistics._write_errors);
        PX4_INFO("Buffer usage: %zu/%zu bytes", _data_buffer.space_used(), _data_buffer.space_available() + _data_buffer.space_used());
    }
    else
    {
        PX4_INFO("Logger is not initialized or failed to start");
    }

    return PX4_OK;
}

void EulerNavDriver::run()
{
    _statistics._start_time = hrt_absolute_time();

    while (!should_exit())
    {
        if (_is_initialized && _log_fd >= 0)
        {
            // Read data from serial port
            const auto bytes_read{_serial_port.readAtLeast(_serial_read_buffer, sizeof(_serial_read_buffer),
                                       Config::MIN_BYTES_TO_READ, Config::SERIAL_READ_TIMEOUT_US)};

            if (bytes_read > 0)
            {
                _statistics._total_bytes_received += bytes_read;

                // Push to ring buffer
                if (!_data_buffer.push_back(_serial_read_buffer, bytes_read))
                {
                    PX4_WARN("Ring buffer overflow, data lost");
                }

                // Write data to file
                writeDataToFile();
            }
        }
        else
        {
            // Re-initialize if something failed
            deinitialize();
            px4_usleep(1000000); // Wait 1 second before retry
            initialize();
        }
    }
}

void EulerNavDriver::initialize()
{
    if (_is_initialized) {
        return;
    }

    // Create log directory
    if (!createDirectory(Config::LOG_DIR_PATH)) {
        PX4_ERR("Failed to create log directory");
        return;
    }

    // Open serial port
    _serial_port.open();
    if (!_serial_port.isOpen()) {
        PX4_ERR("Failed to open serial port");
        return;
    }

    // Allocate ring buffer
    if (!_data_buffer.allocate(Config::DATA_BUFFER_SIZE)) {
        PX4_ERR("Failed to allocate data buffer");
        _serial_port.close();
        return;
    }

    // Create log file
    if (!createLogFile()) {
        PX4_ERR("Failed to create log file");
        _data_buffer.deallocate();
        _serial_port.close();
        return;
    }

    _is_initialized = true;
    PX4_INFO("EULER-NAV logger initialized successfully");
    PX4_INFO("Serial port: %s at %" PRIu32 " baud", _device_name, _baud_rate);
    PX4_INFO("Log file: %s", _statistics._log_filename);
}

void EulerNavDriver::deinitialize()
{
    if (_log_fd >= 0) {
        // Flush remaining data
        writeDataToFile();
        close(_log_fd);
        _log_fd = -1;
    }

    if (_serial_port.isOpen()) {
        _serial_port.close();
    }

    _data_buffer.deallocate();
    _is_initialized = false;
}

bool EulerNavDriver::createDirectory(const char* path)
{
    struct stat st;
    if (stat(path, &st) == 0) {
        return S_ISDIR(st.st_mode);
    }

    // Try to create the directory
    if (mkdir(path, 0755) == 0) {
        return true;
    }

    // If mkdir failed, check if parent directory exists
    if (errno == ENOENT) {
        // Try to create parent directory first
        char parent_path[128];
        strncpy(parent_path, path, sizeof(parent_path) - 1);
        parent_path[sizeof(parent_path) - 1] = '\0';

        char* last_slash = strrchr(parent_path, '/');
        if (last_slash && last_slash != parent_path) {
            *last_slash = '\0';
            if (createDirectory(parent_path)) {
                return mkdir(path, 0755) == 0;
            }
        }
    }

    return false;
}

void EulerNavDriver::generateFilename(char* buffer, size_t buffer_size)
{
    struct timespec ts;
    struct tm *tm_info;

    // Try to get current time
    if (clock_gettime(CLOCK_REALTIME, &ts) == 0 && ts.tv_sec > 1000000000) {
        tm_info = localtime(&ts.tv_sec);
        snprintf(buffer, buffer_size, "%s/eulernav_log_%04d%02d%02d_%02d%02d%02d.bin",
            Config::LOG_DIR_PATH,
            tm_info->tm_year + 1900, tm_info->tm_mon + 1, tm_info->tm_mday,
            tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec);
    } else {
        // Fallback to sequential numbering
        static uint32_t file_counter = 0;
        snprintf(buffer, buffer_size, "%s/eulernav_log_%06" PRIu32 ".bin",
            Config::LOG_DIR_PATH, ++file_counter);
    }
}

bool EulerNavDriver::createLogFile()
{
    char filename[128];
    struct stat st;

    // Generate filename and ensure it's unique
    for (int attempts = 0; attempts < 1000; attempts++) {
        generateFilename(filename, sizeof(filename));

        if (stat(filename, &st) != 0) {
            // File doesn't exist, we can use this name
            break;
        }

        // File exists, wait a bit and try again
        px4_usleep(1000);
    }

    // Create and open the file
    _log_fd = open(filename, O_CREAT | O_WRONLY | O_EXCL, 0644);

    if (_log_fd < 0) {
        PX4_ERR("Failed to create log file %s: %s", filename, strerror(errno));
        return false;
    }

    // Store filename for status reporting
    strncpy(_statistics._log_filename, filename, sizeof(_statistics._log_filename) - 1);
    _statistics._log_filename[sizeof(_statistics._log_filename) - 1] = '\0';

    return true;
}

void EulerNavDriver::writeDataToFile()
{
    if (_log_fd < 0) {
        return;
    }

    // Write data in chunks to avoid blocking
    while (_data_buffer.space_used() >= Config::FILE_WRITE_CHUNK_SIZE) {
        size_t bytes_to_write = _data_buffer.pop_front(_file_write_buffer, Config::FILE_WRITE_CHUNK_SIZE);

        if (bytes_to_write > 0) {
            ssize_t bytes_written = write(_log_fd, _file_write_buffer, bytes_to_write);

            if (bytes_written > 0) {
                _statistics._total_bytes_written += bytes_written;
            } else {
                _statistics._write_errors++;
                PX4_WARN("File write error: %s", strerror(errno));
                break;
            }
        }
    }

    // Write remaining smaller chunks
    if (_data_buffer.space_used() > 0) {
        size_t remaining = _data_buffer.space_used();
        if (remaining <= Config::FILE_WRITE_CHUNK_SIZE) {
            size_t bytes_to_write = _data_buffer.pop_front(_file_write_buffer, remaining);

            if (bytes_to_write > 0) {
                ssize_t bytes_written = write(_log_fd, _file_write_buffer, bytes_to_write);

                if (bytes_written > 0) {
                    _statistics._total_bytes_written += bytes_written;
                } else {
                    _statistics._write_errors++;
                }
            }
        }
    }
}
