/**
 * @file ballastControl.h
 * @brief UV Ballast Control Driver Module Header
 * 
 * This module provides complete UV ballast control functionality including:
 * - Dual ballast control with relay switching
 * - Real-time ADC feedback monitoring
 * - Fault detection and status reporting
 * - Thread-safe multi-core operation
 * - Automatic and manual control modes
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
                       PIN DEFINITIONS - UV BALLAST CONTROL
*****************************************************************************/

// UV Ballast Control Pins (Relay Control)
#define UV_BALLAST_A_CONTROL_PIN        (22)    // GPIO22 - REL1 (Ballast A On/Off)
#define UV_BALLAST_B_CONTROL_PIN        (19)    // GPIO19 - REL2 (Ballast B On/Off)

// UV Ballast Feedback Pins (ADC for status monitoring)
#define UV_BALLAST_A_FEEDBACK_PIN       (34)    // GPIO34 - UV1_ADC (Ballast A Feedback)
#define UV_BALLAST_B_FEEDBACK_PIN       (35)    // GPIO35 - UV2_ADC (Ballast B Feedback)

/*****************************************************************************
                       CONSTANTS AND TIMING CONFIGURATION
*****************************************************************************/

// Timing Parameters
#define BALLAST_SWITCH_DELAY_MS         (10000) // 10 seconds - automatic switching interval
#define BALLAST_ADC_READ_DELAY_MS       (1000)  // 1 second - ADC monitoring interval
#define BALLAST_STARTUP_DELAY_MS        (100)   // Initial startup delay

// Control States
#define BALLAST_ON                      (1)     // Logic HIGH to turn ON
#define BALLAST_OFF                     (0)     // Logic LOW to turn OFF

// ADC Voltage Thresholds (in millivolts)
#define BALLAST_OFF_MIN_VOLTAGE         (1800)  // 1.8V - Minimum voltage for OFF state
#define BALLAST_OFF_MAX_VOLTAGE         (2200)  // 2.2V - Maximum voltage for OFF state
#define BALLAST_ON_MIN_VOLTAGE          (2700)  // 2.7V - Minimum voltage for ON state
#define BALLAST_ON_MAX_VOLTAGE          (3100)  // 3.1V - Maximum voltage for ON state
#define BALLAST_VOLTAGE_TOLERANCE       (100)   // 100mV tolerance for readings

// ADC Configuration
#define BALLAST_ADC_SAMPLES             (10)    // Number of ADC samples to average
#define BALLAST_MAX_VOLTAGE_MV          (3300)  // Maximum expected voltage (3.3V)

// Task Configuration
#define BALLAST_TASK_STACK_SIZE         (4096)  // 4KB stack size
#define BALLAST_TASK_PRIORITY           (5)     // Medium-high priority
#define BALLAST_TASK_CORE_ID            (1)     // Run on Core 1 (Application Core)
#define BALLAST_TASK_NAME               "ballast_ctrl"

// Queue Configuration
#define BALLAST_STATUS_QUEUE_SIZE       (10)    // Status message queue depth
#define BALLAST_MUTEX_TIMEOUT_MS        (100)   // Mutex timeout in milliseconds

/*****************************************************************************
                       ENUMERATION AND TYPE DEFINITIONS
*****************************************************************************/

/**
 * @brief UV Ballast identifier enumeration
 */
typedef enum {
    UV_BALLAST_A = 0,    /**< UV Ballast A (first ballast) */
    UV_BALLAST_B,        /**< UV Ballast B (second ballast) */
    UV_BALLAST_COUNT     /**< Total number of ballasts */
} uv_ballast_t;

/**
 * @brief System ballast state enumeration
 */
typedef enum {
    BALLAST_STATE_A_ON_B_OFF = 0,    /**< Ballast A is ON, Ballast B is OFF */
    BALLAST_STATE_A_OFF_B_ON         /**< Ballast A is OFF, Ballast B is ON */
} ballast_state_t;

/**
 * @brief Ballast status enumeration for health monitoring
 */
typedef enum {
    BALLAST_STATUS_UNKNOWN = 0,              /**< Status not yet determined */
    BALLAST_STATUS_OFF_OK,                   /**< Ballast is OFF and ADC confirms it */
    BALLAST_STATUS_ON_OK,                    /**< Ballast is ON and ADC confirms it */
    BALLAST_STATUS_FAULT_NO_FEEDBACK,        /**< Expected ON but ADC shows OFF voltage */
    BALLAST_STATUS_FAULT_UNEXPECTED_ON,      /**< Expected OFF but ADC shows ON voltage */
    BALLAST_STATUS_FAULT_VOLTAGE_OUT_OF_RANGE /**< ADC voltage outside expected ranges */
} ballast_status_t;

/**
 * @brief Comprehensive ballast feedback data structure
 */
typedef struct {
    uv_ballast_t ballast_id;        /**< Which ballast this data refers to */
    uint32_t voltage_mv;            /**< Measured voltage in millivolts */
    ballast_status_t status;        /**< Current ballast status */
    bool control_state;             /**< What we commanded (true=ON, false=OFF) */
    bool feedback_state;            /**< What ADC indicates (true=ON, false=OFF) */
    uint32_t timestamp_ms;          /**< When this reading was taken (system uptime) */
    uint16_t consecutive_faults;    /**< Number of consecutive fault readings */
    uint32_t raw_adc_value;         /**< Raw ADC reading for debugging */
} ballast_feedback_t;

/**
 * @brief System operation mode enumeration
 */
typedef enum {
    BALLAST_MODE_AUTO = 0,          /**< Automatic switching every 10 seconds */
    BALLAST_MODE_MANUAL,            /**< Manual control only */
    BALLAST_MODE_DISABLED           /**< All ballasts OFF, monitoring only */
} ballast_mode_t;

/*****************************************************************************
                         PUBLIC INTERFACE DECLARATION
*****************************************************************************/

/**
 * @brief Initialize the ballast control module
 * 
 * This function must be called before any other ballast control functions.
 * It initializes GPIO pins, ADC channels, creates mutexes and queues.
 * 
 * @return true if initialization successful, false otherwise
 */
bool BallastControlInit(void);

/**
 * @brief Create and start the ballast control task
 * 
 * Creates a FreeRTOS task that runs on Core 1 and handles:
 * - Automatic ballast switching
 * - ADC monitoring
 * - Status updates
 * 
 * @return pdPASS if task created successfully, pdFAIL otherwise
 */
BaseType_t BallastControlTaskCreate(void);

/**
 * @brief Get the latest ballast feedback data (thread-safe)
 * 
 * @param ballast Which ballast (UV_BALLAST_A or UV_BALLAST_B)
 * @param feedback Pointer to store feedback data
 * @return true if data retrieved successfully, false on error or timeout
 */
bool BallastControlGetFeedback(uv_ballast_t ballast, ballast_feedback_t* feedback);

/**
 * @brief Manually control a specific ballast state
 * 
 * Note: This works in both auto and manual modes. In auto mode,
 * the automatic switching will override this command after the next cycle.
 * 
 * @param ballast Which ballast to control (UV_BALLAST_A or UV_BALLAST_B)
 * @param state true for ON, false for OFF
 * @return true if successful, false on error
 */
bool BallastControlSetState(uv_ballast_t ballast, bool state);

/**
 * @brief Get current system ballast state
 * 
 * @return Current ballast state (which ballast is ON)
 */
ballast_state_t BallastControlGetCurrentState(void);

/**
 * @brief Set the system operation mode
 * 
 * @param mode Operation mode (auto, manual, or disabled)
 */
void BallastControlSetMode(ballast_mode_t mode);

/**
 * @brief Get current operation mode
 * 
 * @return Current operation mode
 */
ballast_mode_t BallastControlGetMode(void);

/**
 * @brief Enable/disable automatic ballast switching (legacy function)
 * 
 * @param enable true to enable automatic switching, false for manual mode
 * @deprecated Use BallastControlSetMode() instead
 */
void BallastControlSetAutoMode(bool enable);

/**
 * @brief Get queue handle for receiving ballast status updates
 * 
 * Other tasks can use this queue to receive real-time status updates.
 * Queue contains ballast_feedback_t structures.
 * 
 * @return Queue handle for ballast feedback messages, or NULL if not initialized
 */
QueueHandle_t BallastControlGetStatusQueue(void);

/**
 * @brief Get system health summary
 * 
 * @param total_faults Pointer to store total fault count since startup
 * @param uptime_ms Pointer to store system uptime in milliseconds
 * @return true if both ballasts are healthy, false if any faults detected
 */
bool BallastControlGetSystemHealth(uint32_t* total_faults, uint32_t* uptime_ms);

/**
 * @brief Force immediate ADC reading update
 * 
 * Useful for testing or when immediate feedback is needed.
 * Normally ADC is read automatically every 1 second.
 * 
 * @param ballast Which ballast to read (UV_BALLAST_A or UV_BALLAST_B)
 * @return true if reading successful, false on error
 */
bool BallastControlForceADCRead(uv_ballast_t ballast);

/**
 * @brief Convert ballast status enum to human-readable string
 * 
 * @param status Ballast status to convert
 * @return Pointer to status string (do not free)
 */
const char* BallastControlStatusToString(ballast_status_t status);

/**
 * @brief Get detailed task information for debugging
 * 
 * @param task_handle Pointer to store task handle (can be NULL)
 * @param core_id Pointer to store core ID where task is running (can be NULL)
 * @param high_water_mark Pointer to store minimum free stack (can be NULL)
 * @return true if task is running, false if task not created or stopped
 */
bool BallastControlGetTaskInfo(TaskHandle_t* task_handle, uint32_t* core_id, uint32_t* high_water_mark);