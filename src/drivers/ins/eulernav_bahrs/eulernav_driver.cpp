#include "eulernav_driver.h"
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/tasks.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <time.h>

// Minimal helper: avoid <algorithm>
static inline size_t min_size(size_t a, size_t b) { return (a < b) ? a : b; }

// Helper to build absolute timeout from microseconds for sem_timedwait
static inline void make_abstime_us(struct timespec &abstime, uint32_t timeout_us)
{
    clock_gettime(CLOCK_REALTIME, &abstime);
    uint64_t nsec = (uint64_t)abstime.tv_nsec + (uint64_t)timeout_us * 1000ULL;
    abstime.tv_sec  += nsec / 1000000000ULL;
    abstime.tv_nsec  = nsec % 1000000000ULL;
}

EulerNavDriver *EulerNavDriver::_instance = nullptr;

EulerNavDriver::EulerNavDriver(const char *device_name, uint32_t baud_rate)
    : ModuleParams{nullptr}
    , _serial_port{device_name, baud_rate,
               device::SerialConfig::ByteSize::EightBits,
               device::SerialConfig::Parity::None,
               device::SerialConfig::StopBits::One,
               device::SerialConfig::FlowControl::Disabled}
    , _baud_rate{baud_rate}
{
    // Store device name for logging
    strncpy(_device_name, device_name, sizeof(_device_name) - 1);
    _device_name[sizeof(_device_name) - 1] = '\0';

    // Set global instance for worker entries
    _instance = this;

    // Initialize semaphores
    px4_sem_init(&_free_buffers_sem, 0, NUM_BUFFERS);
    px4_sem_init(&_filled_buffers_sem, 0, 0);

    // Init buffer pool and queue
    for (size_t i = 0; i < NUM_BUFFERS; i++) {
        _buffer_pool[i].in_use = false;
        _buffer_pool[i].length = 0;
        _filled_queue[i] = 0;
    }

    _filled_head = 0;
    _filled_tail = 0;

    (void)initialize();
}

EulerNavDriver::~EulerNavDriver()
{
    deinitialize();
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

EulerNavDriver *EulerNavDriver::instantiate(int argc, char *argv[])
{
    int myoptind = 1;
    const char *myoptarg = nullptr;

    const char *device_name{nullptr};
    uint32_t baud_rate{115200};

    int ch;

    while ((ch = px4_getopt(argc, argv, "d:b:", &myoptind, &myoptarg)) != EOF) {
        switch (ch) {
        case 'd':
            device_name = myoptarg;
            break;

        case 'b':
            baud_rate = static_cast<uint32_t>(atoi(myoptarg));

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

int EulerNavDriver::custom_command(int, char *[])
{
    return print_usage("unrecognized command");
}

int EulerNavDriver::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
Serial data logger for the EULER-NAV BAHRS device. Logs raw serial data to SD.

### Examples
eulernav_bahrs start -d /dev/ttyS1
eulernav_bahrs start -d /dev/ttyS1 -b 921600
eulernav_bahrs status
eulernav_bahrs stop
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("eulernav_bahrs", "driver");
    PRINT_MODULE_USAGE_SUBCATEGORY("ins");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, nullptr, "Serial device", true);
    PRINT_MODULE_USAGE_PARAM_INT('b', 115200, 9600, 921600, "Baud rate", false);
    PRINT_MODULE_USAGE_COMMAND("status");
    PRINT_MODULE_USAGE_COMMAND("stop");

    return PX4_OK;
}

int EulerNavDriver::print_status()
{
    if (_is_initialized) {
        PX4_INFO("EULER-NAV logger:");
        PX4_INFO("  Elapsed: %llu us", hrt_elapsed_time(&_statistics._start_time));
        PX4_INFO("  Baud: %" PRIu32, _baud_rate);
        PX4_INFO("  File: %s", _statistics._log_filename);
        PX4_INFO("  RX bytes: %" PRIu32, _statistics._total_bytes_received);
        PX4_INFO("  TX bytes: %" PRIu32, _statistics._total_bytes_written);
        PX4_INFO("  Write errors: %" PRIu32, _statistics._write_errors);
        PX4_INFO("  Buffer overflows: %" PRIu32, _statistics._buffer_overflows);

        int used = 0;

        for (size_t i = 0; i < NUM_BUFFERS; i++) {
            if (_buffer_pool[i].in_use) { used++; }
        }

        PX4_INFO("  Buffers in use: %d/%zu", used, NUM_BUFFERS);

        PX4_INFO("  Reader task: %s (id=%d)", _reader_running ? "running" : "stopped", _reader_task_id);
        PX4_INFO("  Writer task: %s (id=%d)", _writer_running ? "running" : "stopped", _writer_task_id);

    } else {
        PX4_INFO("Logger not initialized");
    }

    return PX4_OK;
}

void EulerNavDriver::run()
{
    _statistics._start_time = hrt_absolute_time();

    if (!_is_initialized) {
        PX4_ERR("Not initialized");
        return;
    }

    // Spawn reader task (higher prio)
    _reader_task_id = px4_task_spawn_cmd("eulernav_reader",
                         SCHED_DEFAULT,
                         SCHED_PRIORITY_DEFAULT + 1,
                         Config::TASK_STACK_SIZE,
                         (px4_main_t)&EulerNavDriver::readerTaskEntry,
                         nullptr);

    if (_reader_task_id < 0) {
        PX4_ERR("Failed to spawn reader task");
        _is_initialized = false;
        deinitialize();
        return;
    }

    // Spawn writer task (normal prio)
    _writer_task_id = px4_task_spawn_cmd("eulernav_writer",
                         SCHED_DEFAULT,
                         SCHED_PRIORITY_DEFAULT,
                         Config::TASK_STACK_SIZE,
                         (px4_main_t)&EulerNavDriver::writerTaskEntry,
                         nullptr);

    if (_writer_task_id < 0) {
        PX4_ERR("Failed to spawn writer task");
        px4_task_delete(_reader_task_id);
        _reader_task_id = -1;
        _is_initialized = false;
        deinitialize();
        return;
    }

    // Monitor until stop requested
    while (!should_exit()) {
        px4_usleep(200000); // 200 ms
    }

    // Give workers time to exit
    const hrt_abstime t0 = hrt_absolute_time();

    while ((hrt_absolute_time() - t0 < 2 * 1000 * 1000)) {
        // Check our internal state flags
        if (!_reader_running && !_writer_running) {
            break;
        }

        px4_usleep(50000);
    }

    // Clean up task IDs for the status command
    _reader_task_id = -1;
    _writer_task_id = -1;

    deinitialize();
}

// Worker entries (px4_task_spawn_cmd signature)
int EulerNavDriver::readerTaskEntry(int, char **)
{
    if (!_instance) { return -1; }

    _instance->readerTask();
    return 0;
}

int EulerNavDriver::writerTaskEntry(int, char **)
{
    if (!_instance) { return -1; }

    _instance->writerTask();
    return 0;
}

void EulerNavDriver::readerTask()
{
    PX4_INFO("Reader task started");
    _reader_running = true;

    while (!exitRequested()) {
        // 1. Get a buffer to fill. This will block if none are free.
        DataBuffer *buffer = getAvailableBuffer();

        if (!buffer) {
            // This case happens if the getAvailableBuffer semaphore times out.
            // It implies the writer is stuck and not releasing buffers.
            _statistics._buffer_overflows++;
            // A small sleep is good to prevent spamming if we are in a bad state.
            px4_usleep(100000);
            continue;
        }

        // 2. Attempt to read from the serial port.
        const auto bytes_read = _serial_port.readAtLeast(buffer->data, DataBuffer::BUFFER_SIZE, 12, 100000);
	PX4_INFO("=== Read result: %d bytes ===", (int)bytes_read);

        if (bytes_read > 0) {
            // 3a. If we got data, queue the buffer for the writer.
            buffer->length = bytes_read;
            _statistics._total_bytes_received += bytes_read;
            queueFilledBuffer(buffer);

        } else {
            // 3b. If we got no data (timeout), release the buffer immediately
            // so it can be used again.
            releaseBuffer(buffer);

            // IMPORTANT: Add a small delay to prevent a tight busy-loop
            // when no data is available. This yields CPU to other tasks.
            px4_usleep(1000); // 1ms sleep
        }
    }

    PX4_INFO("Reader task exiting");
    _reader_running = false;
}

void EulerNavDriver::writerTask()
{
    PX4_INFO("Writer task started");
    _writer_running = true;

    hrt_abstime last_sync_time = hrt_absolute_time();
    uint32_t bytes_since_sync = 0;

    while (!exitRequested()) {
        DataBuffer *buffer = getNextFilledBuffer();

        if (!buffer) {
            px4_usleep(1000);
            continue;
        }

        if (_log_fd < 0) {
            PX4_WARN("Log not open, discarding buffer");
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
                PX4_WARN("Write error: %s", strerror(errno));
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
    _writer_running = false;
}

bool EulerNavDriver::initialize()
{
    if (_is_initialized) {
        return true;
    }

    if (!createDirectory(Config::LOG_DIR_PATH)) {
        PX4_ERR("Failed to create log directory");
        return false;
    }

    _serial_port.open();

    if (!_serial_port.isOpen()) {
        PX4_ERR("Failed to open serial port");
        return false;
    }

    if (!createLogFile()) {
        PX4_ERR("Failed to create log file");
        _serial_port.close();
        return false;
    }

    _is_initialized = true;
    PX4_INFO("EULER-NAV logger initialized");
    PX4_INFO("Port: %s @ %" PRIu32, _device_name, _baud_rate);
    PX4_INFO("File: %s", _statistics._log_filename);
    return true;
}

void EulerNavDriver::deinitialize()
{
    if (_log_fd >= 0) {
        fsync(_log_fd);
        close(_log_fd);
        _log_fd = -1;
    }

    if (_serial_port.isOpen()) {
        _serial_port.close();
    }

    _is_initialized = false;
}

bool EulerNavDriver::createDirectory(const char *path)
{
    struct stat st{};

    if (stat(path, &st) == 0) {
        return S_ISDIR(st.st_mode);
    }

    if (mkdir(path, 0755) == 0) {
        return true;
    }

    if (errno == ENOENT) {
        char parent_path[128];
        strncpy(parent_path, path, sizeof(parent_path) - 1);
        parent_path[sizeof(parent_path) - 1] = '\0';

        char *last_slash = strrchr(parent_path, '/');

        if (last_slash && last_slash != parent_path) {
            *last_slash = '\0';

            if (createDirectory(parent_path)) {
                return mkdir(path, 0755) == 0;
            }
        }
    }

    return false;
}

void EulerNavDriver::generateFilename(char *buffer, size_t buffer_size)
{
    struct timespec ts{};
    struct tm tm_info{};

    if (clock_gettime(CLOCK_REALTIME, &ts) == 0 && ts.tv_sec > 1000000000) {
        if (localtime_r(&ts.tv_sec, &tm_info)) {
            snprintf(buffer, buffer_size, "%s/eulernav_log_%04d%02d%02d_%02d%02d%02d.bin",
                 Config::LOG_DIR_PATH,
                 tm_info.tm_year + 1900, tm_info.tm_mon + 1, tm_info.tm_mday,
                 tm_info.tm_hour, tm_info.tm_min, tm_info.tm_sec);
            return;
        }
    }

    static uint32_t file_counter = 0;
    snprintf(buffer, buffer_size, "%s/eulernav_log_%06" PRIu32 ".bin",
         Config::LOG_DIR_PATH, ++file_counter);
}

bool EulerNavDriver::createLogFile()
{
    char filename[128];
    struct stat st{};

    for (int attempts = 0; attempts < 1000; attempts++) {
        generateFilename(filename, sizeof(filename));

        if (stat(filename, &st) != 0) {
            break; // name is free
        }

        px4_usleep(1000);
    }

    _log_fd = open(filename, O_CREAT | O_WRONLY | O_EXCL, 0644);

    if (_log_fd < 0) {
        PX4_ERR("Failed to create log file %s: %s", filename, strerror(errno));
        return false;
    }

    strncpy(_statistics._log_filename, filename, sizeof(_statistics._log_filename) - 1);
    _statistics._log_filename[sizeof(_statistics._log_filename) - 1] = '\0';
    return true;
}

void EulerNavDriver::queueFilledBuffer(DataBuffer *buffer)
{
    if (!buffer || !buffer->in_use) {
        return;
    }

    const uint8_t idx = static_cast<uint8_t>(buffer - _buffer_pool);
    const uint8_t current_tail = _filled_tail;
    const uint8_t next_tail = static_cast<uint8_t>((current_tail + 1) % NUM_BUFFERS);

    if (next_tail == _filled_head) {
        _statistics._buffer_overflows++;
        PX4_WARN("Filled queue overflow, dropping buffer");
        releaseBuffer(buffer);
        return;
    }

    _filled_queue[current_tail] = idx;
    _filled_tail = next_tail;

    px4_sem_post(&_filled_buffers_sem);
}

DataBuffer *EulerNavDriver::getNextFilledBuffer()
{
    struct timespec abstime{};
    make_abstime_us(abstime, 100000); // 100 ms

    if (px4_sem_timedwait(&_filled_buffers_sem, &abstime) != 0) {
        return nullptr;
    }

    const uint8_t head = _filled_head;

    if (head == _filled_tail) {
        return nullptr; // should not happen
    }

    const uint8_t idx = _filled_queue[head];
    _filled_head = static_cast<uint8_t>((head + 1) % NUM_BUFFERS);
    return &_buffer_pool[idx];
}

DataBuffer *EulerNavDriver::getAvailableBuffer()
{
    struct timespec abstime{};
    make_abstime_us(abstime, 100000); // 100 ms

    if (px4_sem_timedwait(&_free_buffers_sem, &abstime) != 0) {
        return nullptr;
    }

    for (size_t i = 0; i < NUM_BUFFERS; i++) {
        uint8_t idx = static_cast<uint8_t>((_next_fill_index + i) % NUM_BUFFERS);

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

void EulerNavDriver::releaseBuffer(DataBuffer *buffer)
{
    if (!buffer) { return; }

    buffer->in_use = false;
    buffer->length = 0;
    px4_sem_post(&_free_buffers_sem);
}
