/**
 * @file fanControl.h
 * @brief Fan Control Driver Module Header
 * 
 * This module provides complete fan control functionality including:
 * - PWM speed control with smooth ramping
 * - Real-time tachometer feedback (RPM monitoring)
 * - Fault detection and status reporting
 * - Thread-safe multi-core operation
 * - Integration with air purifier system
 * 
 * @author ZafarKhan - Air Purifier Control
 * @date 2025
 * @version v1.0
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

/*****************************************************************************
                       PIN DEFINITIONS - FAN CONTROL
*****************************************************************************/

// Fan Control Pins
#define FAN_PWM_PIN                     (23)    // GPIO23 - FAN_PWM (PWM Speed Control)
#define FAN_TACHO_PIN                   (4)     // GPIO4  - FAN_TACHO (Tachometer Feedback)

/*****************************************************************************
                       CONSTANTS AND TIMING CONFIGURATION
*****************************************************************************/

// PWM Configuration
#define FAN_PWM_FREQUENCY_HZ            (5000)  // 5kHz PWM frequency
#define FAN_PWM_RESOLUTION_BITS         (12)    // 12-bit resolution (0-4095)
#define FAN_PWM_MAX_DUTY                (4095)  // Maximum duty cycle value
#define FAN_PWM_MIN_DUTY                (0)     // Minimum duty cycle value

// Speed Control
#define FAN_MIN_SPEED_PERCENT           (0)     // 0% minimum speed
#define FAN_MAX_SPEED_PERCENT           (100)   // 100% maximum speed
#define FAN_STARTUP_SPEED_PERCENT       (30)    // Default startup speed

// Ramping Configuration
#define FAN_RAMP_STEP_PERCENT           (5)     // 5% speed change per ramp step
#define FAN_RAMP_DELAY_MS               (100)   // 100ms delay between ramp steps
#define FAN_SMOOTH_RAMP_STEPS           (20)    // Steps for very smooth ramping

// Tachometer Configuration
#define FAN_TACHO_SAMPLE_PERIOD_MS      (1000)  // 1 second sampling period for RPM
#define FAN_TACHO_PULSES_PER_REVOLUTION (2)     // Typically 2 pulses per revolution
#define FAN_RPM_INVALID_THRESHOLD       (50)    // Below 50 RPM considered stopped/invalid

// Task Configuration
#define FAN_TASK_STACK_SIZE             (4096)  // 4KB stack size
#define FAN_TASK_PRIORITY               (4)     // Medium priority
#define FAN_TASK_CORE_ID                (1)     // Run on Core 1 (Application Core)
#define FAN_TASK_NAME                   "fan_ctrl"

// Queue Configuration
#define FAN_STATUS_QUEUE_SIZE           (8)     // Status message queue depth
#define FAN_MUTEX_TIMEOUT_MS            (100)   // Mutex timeout in milliseconds

// Health Monitoring
#define FAN_STALL_DETECT_COUNT          (3)     // Consecutive zero RPM readings for stall
#define FAN_RPM_VARIANCE_THRESHOLD      (20)    // ±20% RPM variance tolerance

/*****************************************************************************
                       ENUMERATION AND TYPE DEFINITIONS
*****************************************************************************/

/**
 * @brief Fan operation mode enumeration
 */
typedef enum {
    FAN_MODE_OFF = 0,           /**< Fan is OFF */
    FAN_MODE_MANUAL,            /**< Manual speed control */
    FAN_MODE_AUTO,              /**< Automatic speed control based on system state */
    FAN_MODE_RAMP_UP,           /**< Currently ramping up to target speed */
    FAN_MODE_RAMP_DOWN,         /**< Currently ramping down to target speed */
    FAN_MODE_FAULT              /**< Fan in fault state */
} fan_mode_t;

/**
 * @brief Fan status enumeration for health monitoring
 */
typedef enum {
    FAN_STATUS_UNKNOWN = 0,             /**< Status not yet determined */
    FAN_STATUS_OFF_OK,                  /**< Fan is OFF and RPM confirms it */
    FAN_STATUS_RUNNING_OK,              /**< Fan running normally */
    FAN_STATUS_RAMPING_UP,              /**< Fan speed increasing */
    FAN_STATUS_RAMPING_DOWN,            /**< Fan speed decreasing */
    FAN_STATUS_FAULT_STALLED,           /**< Fan stalled (no RPM when expected) */
    FAN_STATUS_FAULT_SPEED_ERROR,       /**< Significant speed deviation from target */
    FAN_STATUS_FAULT_NO_TACHOMETER      /**< Tachometer signal missing */
} fan_status_t;

/**
 * @brief Comprehensive fan feedback data structure
 */
typedef struct {
    uint32_t current_speed_percent;     /**< Current speed setting (0-100%) */
    uint32_t target_speed_percent;      /**< Target speed for ramping (0-100%) */
    uint32_t current_duty_cycle;        /**< Current PWM duty cycle (0-4095) */
    uint32_t measured_rpm;              /**< Measured RPM from tachometer */
    uint32_t expected_rpm;              /**< Expected RPM based on speed setting */
    fan_status_t status;                /**< Current fan status */
    fan_mode_t mode;                    /**< Current operation mode */
    uint32_t timestamp_ms;              /**< When this reading was taken */
    uint16_t consecutive_faults;        /**< Number of consecutive fault readings */
    uint32_t total_runtime_seconds;     /**< Total fan runtime in seconds */
    bool is_ramping;                    /**< True if currently ramping speed */
} fan_feedback_t;

/**
 * @brief Fan configuration structure
 */
typedef struct {
    uint32_t pwm_frequency_hz;          /**< PWM frequency in Hz */
    uint32_t ramp_step_percent;         /**< Speed change per ramp step */
    uint32_t ramp_delay_ms;             /**< Delay between ramp steps */
    uint32_t tacho_sample_period_ms;    /**< Tachometer sampling period */
    bool enable_smooth_ramping;         /**< Enable smooth speed transitions */
} fan_config_t;

/*****************************************************************************
                         PUBLIC INTERFACE DECLARATION
*****************************************************************************/

/**
 * @brief Initialize the fan control module
 * 
 * This function must be called before any other fan control functions.
 * It initializes PWM channel, tachometer counter, creates mutexes and queues.
 * 
 * @return true if initialization successful, false otherwise
 */
bool FanControlInit(void);

/**
 * @brief Create and start the fan control task
 * 
 * Creates a FreeRTOS task that runs on Core 1 and handles:
 * - PWM speed control with ramping
 * - Tachometer monitoring
 * - Status updates and fault detection
 * 
 * @return pdPASS if task created successfully, pdFAIL otherwise
 */
BaseType_t FanControlTaskCreate(void);

/**
 * @brief Set fan speed with smooth ramping
 * 
 * @param speed_percent Target speed as percentage (0-100%)
 * @param enable_ramping true to enable smooth ramping, false for immediate change
 * @return true if successful, false on error
 */
bool FanControlSetSpeed(uint32_t speed_percent, bool enable_ramping);

/**
 * @brief Set fan speed immediately without ramping
 * 
 * @param speed_percent Target speed as percentage (0-100%)
 * @return true if successful, false on error
 */
bool FanControlSetSpeedImmediate(uint32_t speed_percent);

/**
 * @brief Get the latest fan feedback data (thread-safe)
 * 
 * @param feedback Pointer to store feedback data
 * @return true if data retrieved successfully, false on error or timeout
 */
bool FanControlGetFeedback(fan_feedback_t* feedback);

/**
 * @brief Get current measured RPM
 * 
 * @return Current RPM reading, 0 if not available or fan stopped
 */
uint32_t FanControlGetRPM(void);

/**
 * @brief Get current speed percentage
 * 
 * @return Current speed setting (0-100%)
 */
uint32_t FanControlGetCurrentSpeed(void);

/**
 * @brief Check if fan is currently ramping speed
 * 
 * @return true if ramping in progress, false otherwise
 */
bool FanControlIsRamping(void);

/**
 * @brief Set fan operation mode
 * 
 * @param mode Operation mode (off, manual, auto, etc.)
 */
void FanControlSetMode(fan_mode_t mode);

/**
 * @brief Get current operation mode
 * 
 * @return Current operation mode
 */
fan_mode_t FanControlGetMode(void);

/**
 * @brief Stop fan with smooth ramp down
 * 
 * @return true if successful, false on error
 */
bool FanControlStop(void);

/**
 * @brief Emergency stop - immediate fan shutdown
 * 
 * @return true if successful, false on error
 */
bool FanControlEmergencyStop(void);

/**
 * @brief Get queue handle for receiving fan status updates
 * 
 * Other tasks can use this queue to receive real-time status updates.
 * Queue contains fan_feedback_t structures.
 * 
 * @return Queue handle for fan feedback messages, or NULL if not initialized
 */
QueueHandle_t FanControlGetStatusQueue(void);

/**
 * @brief Get fan system health summary
 * 
 * @param total_runtime_seconds Pointer to store total runtime
 * @param fault_count Pointer to store total fault count
 * @return true if fan is healthy, false if faults detected
 */
bool FanControlGetSystemHealth(uint32_t* total_runtime_seconds, uint32_t* fault_count);

/**
 * @brief Convert fan status enum to human-readable string
 * 
 * @param status Fan status to convert
 * @return Pointer to status string (do not free)
 */
const char* FanControlStatusToString(fan_status_t status);

/**
 * @brief Convert fan mode enum to human-readable string
 * 
 * @param mode Fan mode to convert
 * @return Pointer to mode string (do not free)
 */
const char* FanControlModeToString(fan_mode_t mode);

/**
 * @brief Get detailed task information for debugging
 * 
 * @param task_handle Pointer to store task handle (can be NULL)
 * @param core_id Pointer to store core ID where task is running (can be NULL)
 * @param high_water_mark Pointer to store minimum free stack (can be NULL)
 * @return true if task is running, false if task not created or stopped
 */
bool FanControlGetTaskInfo(TaskHandle_t* task_handle, uint32_t* core_id, uint32_t* high_water_mark);

/**
 * @brief Configure fan parameters
 * 
 * @param config Pointer to configuration structure
 * @return true if configuration applied successfully, false on error
 */
bool FanControlConfigure(const fan_config_t* config);

/**
 * @brief Force immediate tachometer reading update
 * 
 * Useful for testing or when immediate feedback is needed.
 * Normally tachometer is read automatically every second.
 * 
 * @return true if reading successful, false on error
 */
bool FanControlForceRPMRead(void);