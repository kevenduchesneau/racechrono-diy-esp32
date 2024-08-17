#ifndef CONFIG_H
#define CONFIG_H

// Bluetooth device name
#define DEVICE_NAME "RaceChrono DIY"

// Serial logging level, select one from the following list :
// LOG_LEVEL_EMERGENCY
// LOG_LEVEL_ALERT
// LOG_LEVEL_CRITICAL
// LOG_LEVEL_ERROR
// LOG_LEVEL_WARNING
// LOG_LEVEL_NOTICE
// LOG_LEVEL_INFO
// LOG_LEVEL_DEBUG
#define LOG_LEVEL LOG_LEVEL_INFO

// GPIO pins connected to the CAN transceiver
#define RX_PIN 11
#define TX_PIN 12

// Serial baud rate
#define SERIAL_BAUD_RATE 115200

// TWAI RX buffer size
#define TWAI_RX_QUEUE_LENGTH 128

// Bluetooth 5.0 support, 1 to enable, 0 to disable
// #define CONFIG_BT_NIMBLE_EXT_ADV 1

// Limit the rate of messages sent to RaceChrono to one every x CAN messages
// Can be further refined per PID through the list below
#define DEFAULT_UPDATE_RATE_LIMITER 10

// An optional list of PIDs and their associated rate limit.
// Reduces the update rate of the specified PID by a factor of the returned value.
uint8_t getUpdateRateLimiter(uint32_t can_id)
{
    switch (can_id)
    {
        // An example of how to configure this list is detailed in the README.

    default:
        return DEFAULT_UPDATE_RATE_LIMITER;
    }
}

#endif // CONFIG_H