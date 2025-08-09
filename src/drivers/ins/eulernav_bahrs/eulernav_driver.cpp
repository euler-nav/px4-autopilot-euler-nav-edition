#include "eulernav_driver.h"
#include <px4_platform_common/getopt.h>
#include <errno.h>
#include <string.h>
#include <inttypes.h>
#include <px4_platform_common/tasks.h>
#include <algorithm> // add for std::min

EulerNavDriver* EulerNavDriver::_instance = nullptr;

EulerNavDriver::EulerNavDriver(const char* device_name, uint32_t baud_rate)
    : ModuleParams{nullptr}
    , _serial_port{device_name, baud_rate, device::SerialConfig::ByteSize::EightBits,
                   device::SerialConfig::Parity::None, device::SerialConfig::StopBits::One,
                   device::SerialConfig::FlowControl::Disabled}
    , _baud_rate{baud_rate}
{
    // ...existing code...
    _instance = this; // set global instance for worker entries

    // Initialize semaphores
    px4_sem_init(&_free_buffers_sem, 0, NUM_BUFFERS);
    px4_sem_init(&_filled_buffers_sem, 0, 0);

    // Init buffer pool
    for (size_t i = 0; i < NUM_BUFFERS; i++) {
        _buffer_pool[i].in_use = false;
        _buffer_pool[i].length = 0;
        _filled_queue[i] = 0;
    }
    _filled_head = 0;
    _filled_tail = 0;

    initialize();
}

EulerNavDriver::~EulerNavDriver()
{
    deinitialize();

    // Destroy semaphores
    px4_sem_destroy(&_free_buffers_sem);
    px4_sem_destroy(&_filled_buffers_sem);
}

int EulerNavDriver::task_spawn(int argc, char *argv[])
{
    int task_id = px4_task_spawn_cmd("eulernav_log", SCHED_DEFAULT, SCHED_PRIORITY_DEFAULT,
                                     Config::TASK_STACK_SIZE, (px4_main_t)&run_trampoline, argv);

    if (task_id < 0) {
        _task_id = -1;
        PX4_ERR("Failed to spawn task.");
    } else {
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

    while (true) {
        int option = px4_getopt(argc, argv, "d:b:", &option_index, &option_arg);

        if (EOF == option) {
            break;
        }

        switch (option) {
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
    if (_is_initialized) {
        PX4_INFO("EULER-NAV Data Logger Status:");
        PX4_INFO("Elapsed time: %llu [us]", hrt_elapsed_time(&_statistics._start_time));
        PX4_INFO("Baud rate: %" PRIu32, _baud_rate);
        PX4_INFO("Log file: %s", _statistics._log_filename);
        PX4_INFO("Total bytes received: %" PRIu32, _statistics._total_bytes_received);
        PX4_INFO("Total bytes written: %" PRIu32, _statistics._total_bytes_written);
        PX4_INFO("Write errors: %" PRIu32, _statistics._write_errors);
        PX4_INFO("Buffer overflows: %" PRIu32, _statistics._buffer_overflows);

        // Count used buffers
        int used_buffers = 0;
        for (size_t i = 0; i < NUM_BUFFERS; i++) {
            if (_buffer_pool[i].in_use) {
                used_buffers++;
            }
        }
        PX4_INFO("Buffers in use: %d/%zu", used_buffers, NUM_BUFFERS);

        if (_reader_task_id.load() > 0 && _writer_task_id.load() > 0) {
            PX4_INFO("Reader task: running (id: %d)", _reader_task_id.load());
            PX4_INFO("Writer task: running (id: %d)", _writer_task_id.load());
        } else {
            PX4_INFO("Reader task: %s", _reader_task_id.load() > 0 ? "running" : "stopped");
            PX4_INFO("Writer task: %s", _writer_task_id.load() > 0 ? "running" : "stopped");
        }
    } else {
        PX4_INFO("Logger is not initialized or failed to start");
    }

    return PX4_OK;
}

void EulerNavDriver::run()
{
    _statistics._start_time = hrt_absolute_time();

    if (!_is_initialized) {
        PX4_ERR("Driver not initialized, exiting");
        return;
    }

    // Spawn reader task (higher priority)
    _reader_task_id = px4_task_spawn_cmd("eulernav_reader",
                                         SCHED_DEFAULT,
                                         SCHED_PRIORITY_DEFAULT + 5,   // safer than MAX-5
                                         Config::TASK_STACK_SIZE,
                                         (px4_main_t)&EulerNavDriver::readerTaskEntry,
                                         nullptr);

    if (_reader_task_id < 0) {
        PX4_ERR("Failed to spawn reader task");
        deinitialize();
        return;
    }

    // Spawn writer task (lower priority)
    _writer_task_id = px4_task_spawn_cmd("eulernav_writer",
                                         SCHED_DEFAULT,
                                         SCHED_PRIORITY_DEFAULT,
                                         Config::TASK_STACK_SIZE,
                                         (px4_main_t)&EulerNavDriver::writerTaskEntry,
                                         nullptr);

    if (_writer_task_id < 0) {
        PX4_ERR("Failed to spawn writer task");
        // Let reader run down when stop is requested
    }

    // Monitor until stop requested
    while (!should_exit()) {
        px4_usleep(200000); // 200 ms
    }

    // Wait for workers to exit themselves (they check exitRequested())
    const hrt_abstime t_wait_start = hrt_absolute_time();
    while (((_reader_task_id.load() > 0) || (_writer_task_id.load() > 0))
           && (hrt_absolute_time() - t_wait_start < 2_s)) {
        px4_usleep(50000);
    }

    deinitialize();
}

void EulerNavDriver::readerTaskTrampoline(void *arg)
{
    EulerNavDriver *driver = static_cast<EulerNavDriver *>(arg);
    driver->readerTask();
}

int EulerNavDriver::readerTaskEntry(int, char**)
{
    if (!_instance) { return -1; }
    _instance->readerTask();
    return 0;
}

int EulerNavDriver::writerTaskEntry(int, char**)
{
    if (!_instance) { return -1; }
    _instance->writerTask();
    return 0;
}

void EulerNavDriver::readerTask()
{
    PX4_INFO("Reader task started");

    while (!exitRequested()) {
        DataBuffer *buffer = getAvailableBuffer();
        if (!buffer) {
            _statistics._buffer_overflows++;
            PX4_WARN("No buffers available for reading, data may be lost");
            px4_usleep(1000);
            continue;
        }

        buffer->length = 0;
        bool got_data = false;

        while (!exitRequested() && buffer->length < DataBuffer::BUFFER_SIZE) {
            const auto bytes_read = _serial_port.readAtLeast(_serial_read_buffer, sizeof(_serial_read_buffer),
                                                             Config::MIN_BYTES_TO_READ, Config::SERIAL_READ_TIMEOUT_US);
            if (bytes_read > 0) {
                const size_t bytes_to_copy = std::min(static_cast<size_t>(bytes_read),
                                                      DataBuffer::BUFFER_SIZE - buffer->length);
                memcpy(buffer->data + buffer->length, _serial_read_buffer, bytes_to_copy);
                buffer->length += bytes_to_copy;
                _statistics._total_bytes_received += bytes_to_copy;
                got_data = true;

                if (bytes_to_copy < static_cast<size_t>(bytes_read)) {
                    PX4_WARN("Buffer full, %zu bytes dropped", static_cast<size_t>(bytes_read) - bytes_to_copy);
                    _statistics._buffer_overflows++;
                    break;
                }
            } else if (got_data) {
                break;
            }
        }

        if (got_data) {
            queueFilledBuffer(buffer);
        } else {
            releaseBuffer(buffer);
        }
    }

    PX4_INFO("Reader task exiting");
    _reader_task_id = -1;
    px4_task_exit(0);
}

void EulerNavDriver::writerTaskTrampoline(void *arg)
{
    EulerNavDriver *driver = static_cast<EulerNavDriver *>(arg);
    driver->writerTask();
}

void EulerNavDriver::writerTask()
{
    PX4_INFO("Writer task started");

    hrt_abstime last_sync_time = hrt_absolute_time();
    uint32_t bytes_since_sync = 0;

    while (!exitRequested()) {
        DataBuffer *buffer = getNextFilledBuffer();
        if (!buffer) {
            px4_usleep(1000);
            continue;
        }

        if (_log_fd < 0) {
            PX4_WARN("Log file not open, discarding buffer");
            releaseBuffer(buffer);
            continue;
        }

        size_t offset = 0;
        while (offset < buffer->length && !exitRequested()) {
            ssize_t n = write(_log_fd, buffer->data + offset, buffer->length - offset);
            if (n > 0) {
                offset += n;
                _statistics._total_bytes_written += n;
                bytes_since_sync += n;
            } else if (n == 0 || (n < 0 && errno != EINTR)) {
                _statistics._write_errors++;
                PX4_WARN("File write error: %s", strerror(errno));
                close(_log_fd);
                _log_fd = -1;
                (void)createLogFile(); // best effort recovery
                break;
            }
        }

        releaseBuffer(buffer);

        const hrt_abstime now = hrt_absolute_time();
        if ((bytes_since_sync > 1048576) || (now - last_sync_time > 5000000)) {
            if (_log_fd >= 0) {
                fsync(_log_fd);
            }
            bytes_since_sync = 0;
            last_sync_time = now;
        }
    }

    if (_log_fd >= 0) {
        fsync(_log_fd);
    }

    PX4_INFO("Writer task exiting");
    _writer_task_id = -1;
    px4_task_exit(0);
}

bool EulerNavDriver::initialize()
{
    if (_is_initialized) {
        return true;
    }

    // Create log directory
    if (!createDirectory(Config::LOG_DIR_PATH)) {
        PX4_ERR("Failed to create log directory");
        return false;
    }

    // Open serial port
    _serial_port.open();
    if (!_serial_port.isOpen()) {
        PX4_ERR("Failed to open serial port");
        return false;
    }

    // Create log file
    if (!createLogFile()) {
        PX4_ERR("Failed to create log file");
        _serial_port.close();
        return false;
    }

    _is_initialized = true;
    PX4_INFO("EULER-NAV logger initialized successfully");
    PX4_INFO("Serial port: %s at %" PRIu32 " baud", _device_name, _baud_rate);
    PX4_INFO("Log file: %s", _statistics._log_filename);

    return true;
}

void EulerNavDriver::deinitialize()
{
    if (_log_fd >= 0) {
        // Ensure data is written to disk
        fsync(_log_fd);
        close(_log_fd);
        _log_fd = -1;
    }

    if (_serial_port.isOpen()) {
        _serial_port.close();
    }

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
    struct tm tm_info; // Use a local struct for the result

    // Try to get current time
    if (clock_gettime(CLOCK_REALTIME, &ts) == 0 && ts.tv_sec > 1000000000) {
        // Use thread-safe localtime_r
        if (localtime_r(&ts.tv_sec, &tm_info)) {
            snprintf(buffer, buffer_size, "%s/eulernav_log_%04d%02d%02d_%02d%02d%02d.bin",
                     Config::LOG_DIR_PATH,
                     tm_info.tm_year + 1900, tm_info.tm_mon + 1, tm_info.tm_mday,
                     tm_info.tm_hour, tm_info.tm_min, tm_info.tm_sec);
            return; // Exit after successful generation
        }
    }

    // Fallback to sequential numbering if time functions fail
    static uint32_t file_counter = 0;
    snprintf(buffer, buffer_size, "%s/eulernav_log_%06" PRIu32 ".bin",
             Config::LOG_DIR_PATH, ++file_counter);
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

    // Create and open the file with standard permissions (0644)
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

void EulerNavDriver::queueFilledBuffer(DataBuffer* buffer)
{
    if (!buffer || !buffer->in_use) {
        return;
    }
    // Push buffer index into SPSC queue
    const uint8_t idx = static_cast<uint8_t>(buffer - _buffer_pool);
    const uint8_t next_tail = static_cast<uint8_t>((_filled_tail.load() + 1) % NUM_BUFFERS);

    // Simple overflow protection: if queue is full, drop and count overflow
    if (next_tail == _filled_head.load()) {
        _statistics._buffer_overflows++;
        PX4_WARN("Filled queue overflow, dropping buffer");
        // Do not release here; writer won't see it. Release it for reuse.
        releaseBuffer(buffer);
        return;
    }

    _filled_queue[_filled_tail.load()] = idx;
    _filled_tail.store(next_tail);

    // Signal one filled buffer available
    px4_sem_post(&_filled_buffers_sem);
}

DataBuffer* EulerNavDriver::getNextFilledBuffer()
{
    if (px4_sem_timedwait(&_filled_buffers_sem, 100000) != 0) {
        return nullptr;
    }

    // Pop index from SPSC queue
    const uint8_t head = _filled_head.load();
    if (head == _filled_tail.load()) {
        // Should not happen due to semaphore, but guard anyway
        return nullptr;
    }

    const uint8_t idx = _filled_queue[head];
    _filled_head.store(static_cast<uint8_t>((head + 1) % NUM_BUFFERS));

    return &_buffer_pool[idx];
}

DataBuffer* EulerNavDriver::getAvailableBuffer()
{
    if (px4_sem_timedwait(&_free_buffers_sem, 100000) != 0) {
        return nullptr;
    }

    for (size_t i = 0; i < NUM_BUFFERS; i++) {
        uint8_t idx = (_next_fill_index.load() + i) % NUM_BUFFERS;
        if (!_buffer_pool[idx].in_use) {
            _buffer_pool[idx].in_use = true;
            _buffer_pool[idx].length = 0;
            _next_fill_index = static_cast<uint8_t>((idx + 1) % NUM_BUFFERS);
            return &_buffer_pool[idx];
        }
    }

    // Shouldn't happen due to semaphore
    px4_sem_post(&_free_buffers_sem);
    return nullptr;
}

void EulerNavDriver::releaseBuffer(DataBuffer* buffer)
{
    if (!buffer) { return; }
    buffer->in_use = false;
    buffer->length = 0;
    px4_sem_post(&_free_buffers_sem);
}
