// Required libraries
//
// NimBLE-Arduino : https://www.arduino.cc/reference/en/libraries/nimble-arduino/
// EasyLogger : https://www.arduino.cc/reference/en/libraries/easylogger/
// arduino-RaceChrono : https://github.com/timurrrr/arduino-RaceChrono
//
// Special thanks to Timur Iskhodzhanov / timurrr. This project heavily referenced
// his RaceChronoDiyBleDevice project, as well as using his RaceChrono library.
// https://github.com/timurrrr
//

// Copy the "config.h.example" file provided to "config.h"
// and edit as needed, following the instructions provided in README.md
#include "config.h"

#include <driver/twai.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/ringbuf.h>
#include <freertos/semphr.h>
#include <esp_mac.h>
#include <EasyLogger.h>
#include <RaceChrono.h>

// bool isTwaiDriverStarted = false;
bool isBLEStarted = false;
twai_message_t message;
RingbufHandle_t bufferHandle;

using PidExtra = struct
{
    uint8_t updateRateLimiter = DEFAULT_UPDATE_RATE_LIMITER;
    uint8_t messageCount = 0;
};
RaceChronoPidMap<PidExtra> pidMap;

// Code taken from the arduino-RaceChrono example files.
class PrintRaceChronoCommands : public RaceChronoBleCanHandler
{
public:
    void allowAllPids(uint16_t updateIntervalMs)
    {
        LOG_INFO("racechrono", "Allow all PIDs.");
        pidMap.allowAllPids(updateIntervalMs);
    }

    void denyAllPids()
    {
        LOG_INFO("racechrono", "Deny all PIDs.");
        pidMap.reset();
    }

    void allowPid(uint32_t pid, uint16_t updateIntervalMs)
    {
        LOG_INFO("racechrono", "Allow PID " << pid << ".");

        if (pidMap.allowOnePid(pid, updateIntervalMs))
        {
            void *entry = pidMap.getEntryId(pid);
            PidExtra *pidExtra = pidMap.getExtra(entry);
            pidExtra->messageCount = 0;
            pidExtra->updateRateLimiter = getUpdateRateLimiter(pid);
            LOG_INFO("racechrono", "PID " << pid << " update rate limiter: " << pidExtra->updateRateLimiter << ".");
        }
        else
        {
            LOG_WARNING("racechrono", "Unable to handle this request.");
        }
    }

    void handleDisconnect()
    {
        if (!pidMap.isEmpty() || pidMap.areAllPidsAllowed())
        {
            LOG_NOTICE("racechrono", "RaceChrono disconnected.");
            LOG_INFO("racechrono", "Resetting the map.");
            this->denyAllPids();
        }
    }
} raceChronoHandler;

// Called from setup() on core 1
bool startTwaiDriver()
{
    LOG_NOTICE("twai", "Starting driver.");

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TX_PIN, (gpio_num_t)RX_PIN, TWAI_MODE_LISTEN_ONLY);
    g_config.rx_queue_len = TWAI_RX_QUEUE_LENGTH;
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    twai_driver_install(&g_config, &t_config, &f_config);

    if (twai_start() == ESP_OK)
    {
        // isTwaiDriverStarted = true;
        LOG_NOTICE("twai", "Driver started.");
        return true;
    }
    else
    {
        LOG_ERROR("twai", "Driver failed to start.");
    }

    return false;
}

// Never called, should probably be removed ...
bool stopTwaiDriver()
{
    LOG_NOTICE("twai", "Stopping driver.");

    if (twai_stop() == ESP_OK)
    {
        LOG_NOTICE("twai", "Driver stopped.");
        if (twai_driver_uninstall() == ESP_OK)
        {
            LOG_NOTICE("twai", "Driver uninstalled.");
            // isTwaiDriverStarted = false;
            return true;
        }
        else
        {
            LOG_ERROR("twai", "Driver failed to uninstall.");
        }
    }
    else
    {
        LOG_ERROR("twai", "Driver failed to stop.");
    }

    return false;
}

// Returns the Bluetooth device name to advertise
// If unspecified in the config file, returns a generic name based on the MAC address
char *getDeviceName()
{
#ifdef DEVICE_NAME
    LOG_INFO("getDeviceName", "Device name is " << DEVICE_NAME);
    return DEVICE_NAME;
#endif

    // Get Bluetooth MAC address
    // https://github.com/espressif/esp-idf/blob/master/examples/system/base_mac_address/main/base_mac_address_example_main.c
    uint8_t baseMac[6];
    static char deviceName[32];

    if (esp_read_mac(baseMac, ESP_MAC_BT) == ESP_OK)
    {
        sprintf(deviceName, "RaceChrono %x:%x:%x", baseMac[3], baseMac[4], baseMac[5]);
    }
    else
    {
        LOG_ERROR("getDeviceName", "Failed to get BT MAC address.");
        sprintf(deviceName, "RaceChrono aa:bb:cc");
    }

    LOG_INFO("getDeviceName", "Device name is " << deviceName);
    return deviceName;
}

// Called from taskManageBLEConnection() task on core 0
bool startBLEConnection()
{
    LOG_NOTICE("task_ble_manage", "Starting BLE.");

    RaceChronoBle.setUp(getDeviceName(), &raceChronoHandler);
    RaceChronoBle.startAdvertising();
    isBLEStarted = true;

    LOG_NOTICE("task_ble_manage", "BLE started.");

    return true;
}

void taskManageBLEConnection(void *)
{
    LOG_DEBUG("task_ble_manage", "Starting up on core " << xPortGetCoreID() << ".");
    bool connectedNotice = false;

    for (;;)
    {
        if (!isBLEStarted)
        {
            startBLEConnection();
        }

        if (!RaceChronoBle.isConnected())
        {
            connectedNotice = false;
            raceChronoHandler.handleDisconnect();
            LOG_INFO("task_ble_manage", "Waiting for a new connection.");
        }
        else if (!connectedNotice)
        {
            connectedNotice = true;
            LOG_NOTICE("task_ble_manage", "RaceChrono connected.");
        }

        vTaskDelay(10000);
    }
}

void taskGetTwaiMessages(void *)
{
    LOG_DEBUG("task_twai", "Starting up on core " << xPortGetCoreID() << ".");

    uint64_t taskGetTwaiMessagesTimerStart = esp_timer_get_time();
    int taskGetTwaiMessagesTimerInterval = 10000000; // 10 seconds
    uint64_t msgCountRx = 0;
    uint64_t msgCountBufferTx = 0;

    for (;;)
    {
        if (twai_receive(&message, portMAX_DELAY) == ESP_OK)
        {
            msgCountRx++;
            if (!(message.rtr))
            {
                if (!(message.data_length_code == 0))
                {
                    void *entry = pidMap.getEntryId(message.identifier);
                    if (entry != NULL)
                    {
                        PidExtra *extra = pidMap.getExtra(entry);
                        extra->messageCount++;
                        if (extra->messageCount == extra->updateRateLimiter)
                        {
                            msgCountBufferTx++;
                            if (!(xRingbufferSend(bufferHandle, &message, sizeof(message), 0)))
                            {
                                LOG_WARNING("task_twai", "Receive ring buffer overflow, dropping one message.");
                            }
                            extra->messageCount = 0;
                        }
                    }
                }
            }

            // Debug - Report messages received count from TWAI every 10 seconds
            if (LOG_LEVEL == LOG_LEVEL_DEBUG)
            {
                if ((esp_timer_get_time() - taskGetTwaiMessagesTimerStart) >= taskGetTwaiMessagesTimerInterval)
                {
                    LOG_DEBUG("task_twai", msgCountRx << " messages received from TWAI in the last 10 seconds.");
                    LOG_DEBUG("task_twai", msgCountBufferTx << " messages sent to ring buffer the last 10 seconds.");
                    msgCountRx = 0;
                    msgCountBufferTx = 0;
                    taskGetTwaiMessagesTimerStart = esp_timer_get_time();
                }
            }
        }
    }
}

void taskSendBLEMessages(void *)
{
    LOG_DEBUG("task_ble_send", "Starting up on core " << xPortGetCoreID());

    uint64_t taskSendBLEMessagesStart = esp_timer_get_time();
    int taskSendBLEMessagesInterval = 10000000; // 10 seconds
    uint64_t msgCountBufferRx = 0;

    size_t message_size;

    for (;;)
    {
        twai_message_t *message = (twai_message_t *)xRingbufferReceive(bufferHandle, &message_size, portMAX_DELAY);

        if (message != NULL)
        {
            RaceChronoBle.sendCanData(message->identifier, message->data, message->data_length_code);
            msgCountBufferRx++;

            // Debug - Report message received count from ring buffer every 10 seconds
            if (LOG_LEVEL == LOG_LEVEL_DEBUG)
            {
                if ((esp_timer_get_time() - taskSendBLEMessagesStart) >= taskSendBLEMessagesInterval)
                {
                    LOG_DEBUG("task_ble_send", msgCountBufferRx << " messages sent to RaceChronoBle in the last 10 seconds.");
                    msgCountBufferRx = 0;
                    taskSendBLEMessagesStart = esp_timer_get_time();
                }
            }
        }
        else
        {
            LOG_ERROR("task_ble_send", "Failed to receive message from receive ring buffer.");
        }

        vRingbufferReturnItem(bufferHandle, (void *)message);
    }
}

void setup()
{
    Serial.begin(SERIAL_BAUD_RATE);

    delay(2000);
    LOG_DEBUG("setup", "Starting up on core " << xPortGetCoreID());

    delay(1000);
    startTwaiDriver();

    bufferHandle = xRingbufferCreate(65536, RINGBUF_TYPE_NOSPLIT);
    if (bufferHandle == NULL)
    {
        LOG_ERROR("setup", "Failed to create ring buffer.");
    }

    // When main core is 1, assume dual-core mcu (S3) is used and map the TWAI task to it
    // Otherwise, assume single-core mcu (C3) and map all tasks to core 0
    uint mainCore = xPortGetCoreID();

    delay(1250); // Delay used to avoid multiple serial messages overlapping
    xTaskCreatePinnedToCore(taskManageBLEConnection, "taskManageBLEConnection", 4096, NULL, 1, NULL, 0);

    delay(1250); // Delay used to avoid multiple serial messages overlapping
    xTaskCreatePinnedToCore(taskGetTwaiMessages, "taskGetTwaiMessages", 4096, NULL, 5, NULL, mainCore);

    delay(1250); // Delay used to avoid multiple serial messages overlapping
    xTaskCreatePinnedToCore(taskSendBLEMessages, "taskSendBLEMessages", 4096, NULL, 5, NULL, 0);
}

// The default Arduino loop is not used, so no point in keeping the task running
void loop()
{
    vTaskDelete(NULL);
}
