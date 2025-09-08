/**
 * @file ballastControl.c
 * @brief UV Ballast Control Driver Module Implementation
 * 
 * Complete implementation of UV ballast control with:
 * - Dual-core task architecture
 * - Real-time ADC monitoring with fault detection
 * - Thread-safe data access with mutex protection
 * - Comprehensive status reporting and health monitoring
 * 
 * @author ZafarKhan - Air Purifier Control
 * @date 2025
 * @version v1.0
 */

#include "ballastControl.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <inttypes.h> 

/*****************************************************************************
                          PRIVATE DEFINES / MACROS
*****************************************************************************/

#define GPIO_OUTPUT_BALLAST_PIN_MASK    ((1ULL << UV_BALLAST_A_CONTROL_PIN) | \
                                         (1ULL << UV_BALLAST_B_CONTROL_PIN))

// ADC Configuration
#define ADC_WIDTH                       ADC_WIDTH_BIT_12
#define ADC_ATTEN                       ADC_ATTEN_DB_12
#define ADC_UNIT                        ADC_UNIT_1

// Fault detection thresholds
#define MAX_CONSECUTIVE_FAULTS          (5)     // Max faults before major alarm
#define FAULT_RESET_THRESHOLD           (3)     // Consecutive good readings to clear fault

// Task timing
#define TASK_LOOP_DELAY_MS             (100)   // Main task loop delay
#define SYSTEM_STARTUP_DELAY_MS        (500)   // Wait for system stabilization

/*****************************************************************************
                     PRIVATE STRUCTS / ENUMS / VARIABLES
*****************************************************************************/

static const char* TAG = "BALLAST_CTRL";

// Task management
static TaskHandle_t ballast_task_handle = NULL;

// Current state tracking
static ballast_state_t current_state = BALLAST_STATE_A_ON_B_OFF;
static ballast_mode_t operation_mode = BALLAST_MODE_AUTO;

// ADC calibration
static esp_adc_cal_characteristics_t adc_chars;

// Ballast feedback data (protected by mutex)
static ballast_feedback_t ballast_feedback[UV_BALLAST_COUNT];

// System health tracking
static uint32_t total_system_faults = 0;
static uint32_t system_start_time = 0;

// Synchronization primitives
static SemaphoreHandle_t feedback_mutex = NULL;

// Inter-task communication
static QueueHandle_t status_queue = NULL;

// ADC channel mapping
static const adc1_channel_t ballast_adc_channels[UV_BALLAST_COUNT] = {
    ADC1_CHANNEL_6,  // GPIO34 - Ballast A
    ADC1_CHANNEL_7   // GPIO35 - Ballast B
};

/*****************************************************************************
                         PRIVATE FUNCTION DECLARATIONS
*****************************************************************************/

// Initialization functions
static bool ballast_gpio_init(void);
static bool ballast_adc_init(void);
static bool ballast_sync_init(void);

// Core task function
static void ballast_control_task(void* parameters);

// Control functions
static void set_ballast_state_internal(uv_ballast_t ballast, uint32_t level);
static void switch_ballast_state_auto(void);
static void initialize_ballast_feedback_data(void);

// ADC and monitoring functions
static uint32_t read_ballast_voltage(uv_ballast_t ballast);
static ballast_status_t analyze_ballast_status(uv_ballast_t ballast, uint32_t voltage_mv, bool control_state);
static void update_ballast_feedback(uv_ballast_t ballast);
static bool validate_adc_reading(uint32_t voltage_mv);

// Status and communication functions
static void send_status_update(const ballast_feedback_t* feedback);
static void handle_fault_detection(uv_ballast_t ballast, ballast_feedback_t* feedback);
static void log_ballast_status(const ballast_feedback_t* feedback);

// Utility functions
static const char* ballast_name_string(uv_ballast_t ballast);
static const char* mode_to_string(ballast_mode_t mode);

/*****************************************************************************
                           PUBLIC INTERFACE IMPLEMENTATION
*****************************************************************************/

bool BallastControlInit(void)
{
    ESP_LOGI(TAG, "Initializing UV Ballast Control Module v1.0");
    
    // Record system start time
    system_start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    // Initialize synchronization primitives
    if (!ballast_sync_init()) {
        ESP_LOGE(TAG, "Failed to initialize synchronization primitives");
        return false;
    }
    
    // Initialize GPIO pins
    if (!ballast_gpio_init()) {
        ESP_LOGE(TAG, "Failed to initialize GPIO pins");
        return false;
    }
    
    // Initialize ADC
    if (!ballast_adc_init()) {
        ESP_LOGE(TAG, "Failed to initialize ADC");
        return false;
    }
    
    // Initialize feedback data structures
    initialize_ballast_feedback_data();
    
    ESP_LOGI(TAG, "UV Ballast Control Module initialized successfully");
    ESP_LOGI(TAG, "Configuration: Auto-switch=%ds, ADC-monitor=%ds, Samples=%d", 
             BALLAST_SWITCH_DELAY_MS/1000, BALLAST_ADC_READ_DELAY_MS/1000, BALLAST_ADC_SAMPLES);
    
    return true;
}

BaseType_t BallastControlTaskCreate(void)
{
    if (ballast_task_handle != NULL) {
        ESP_LOGW(TAG, "Ballast control task already exists");
        return pdFAIL;
    }
    
    BaseType_t result = xTaskCreatePinnedToCore(
        ballast_control_task,           // Task function
        BALLAST_TASK_NAME,              // Task name
        BALLAST_TASK_STACK_SIZE,        // Stack size
        NULL,                           // Parameters
        BALLAST_TASK_PRIORITY,          // Priority
        &ballast_task_handle,           // Task handle
        BALLAST_TASK_CORE_ID           // Core ID (Core 1 - Application Core)
    );
    
    if (result == pdPASS) {
        ESP_LOGI(TAG, "Ballast control task created successfully on Core %d", BALLAST_TASK_CORE_ID);
        ESP_LOGI(TAG, "Task: %s, Priority: %d, Stack: %d bytes", 
                 BALLAST_TASK_NAME, BALLAST_TASK_PRIORITY, BALLAST_TASK_STACK_SIZE);
    } else {
        ESP_LOGE(TAG, "Failed to create ballast control task (result: %d)", result);
        ESP_LOGE(TAG, "Free heap: %lu bytes", esp_get_free_heap_size());
    }
    
    return result;
}

bool BallastControlGetFeedback(uv_ballast_t ballast, ballast_feedback_t* feedback)
{
    if (ballast >= UV_BALLAST_COUNT || feedback == NULL) {
        ESP_LOGW(TAG, "Invalid parameters: ballast=%d, feedback=%p", ballast, (void*)feedback);
        return false;
    }
    
    if (xSemaphoreTake(feedback_mutex, pdMS_TO_TICKS(BALLAST_MUTEX_TIMEOUT_MS)) == pdTRUE) {
        *feedback = ballast_feedback[ballast];
        xSemaphoreGive(feedback_mutex);
        return true;
    } else {
        ESP_LOGW(TAG, "Mutex timeout getting feedback for Ballast %s", ballast_name_string(ballast));
        return false;
    }
}

bool BallastControlSetState(uv_ballast_t ballast, bool state)
{
    if (ballast >= UV_BALLAST_COUNT) {
        ESP_LOGW(TAG, "Invalid ballast number: %d", ballast);
        return false;
    }
    
    ESP_LOGI(TAG, "Manual control: Setting Ballast %s to %s", 
             ballast_name_string(ballast), state ? "ON" : "OFF");
    
    set_ballast_state_internal(ballast, state ? BALLAST_ON : BALLAST_OFF);
    return true;
}

ballast_state_t BallastControlGetCurrentState(void)
{
    return current_state;
}

void BallastControlSetMode(ballast_mode_t mode)
{
    ballast_mode_t old_mode = operation_mode;
    operation_mode = mode;
    
    ESP_LOGI(TAG, "Operation mode changed: %s → %s", 
             mode_to_string(old_mode), mode_to_string(mode));
    
    // Handle mode transitions
    if (mode == BALLAST_MODE_DISABLED) {
        ESP_LOGI(TAG, "Disabling all ballasts due to DISABLED mode");
        set_ballast_state_internal(UV_BALLAST_A, BALLAST_OFF);
        set_ballast_state_internal(UV_BALLAST_B, BALLAST_OFF);
    }
}

ballast_mode_t BallastControlGetMode(void)
{
    return operation_mode;
}

void BallastControlSetAutoMode(bool enable)
{
    // Legacy function - convert to new mode system
    BallastControlSetMode(enable ? BALLAST_MODE_AUTO : BALLAST_MODE_MANUAL);
}

QueueHandle_t BallastControlGetStatusQueue(void)
{
    return status_queue;
}

bool BallastControlGetSystemHealth(uint32_t* total_faults, uint32_t* uptime_ms)
{
    if (total_faults != NULL) {
        *total_faults = total_system_faults;
    }
    
    if (uptime_ms != NULL) {
        *uptime_ms = (xTaskGetTickCount() * portTICK_PERIOD_MS) - system_start_time;
    }
    
    // Check if both ballasts are healthy
    bool system_healthy = true;
    if (xSemaphoreTake(feedback_mutex, pdMS_TO_TICKS(BALLAST_MUTEX_TIMEOUT_MS)) == pdTRUE) {
        for (int i = 0; i < UV_BALLAST_COUNT; i++) {
            if (ballast_feedback[i].status == BALLAST_STATUS_FAULT_NO_FEEDBACK ||
                ballast_feedback[i].status == BALLAST_STATUS_FAULT_UNEXPECTED_ON ||
                ballast_feedback[i].status == BALLAST_STATUS_FAULT_VOLTAGE_OUT_OF_RANGE) {
                system_healthy = false;
                break;
            }
        }
        xSemaphoreGive(feedback_mutex);
    } else {
        system_healthy = false; // Mutex timeout indicates system issues
    }
    
    return system_healthy;
}

bool BallastControlForceADCRead(uv_ballast_t ballast)
{
    if (ballast >= UV_BALLAST_COUNT) {
        ESP_LOGW(TAG, "Invalid ballast for forced ADC read: %d", ballast);
        return false;
    }
    
    ESP_LOGI(TAG, "Forcing ADC read for Ballast %s", ballast_name_string(ballast));
    update_ballast_feedback(ballast);
    return true;
}

const char* BallastControlStatusToString(ballast_status_t status)
{
    switch (status) {
        case BALLAST_STATUS_UNKNOWN:                return "UNKNOWN";
        case BALLAST_STATUS_OFF_OK:                 return "OFF_OK";
        case BALLAST_STATUS_ON_OK:                  return "ON_OK";
        case BALLAST_STATUS_FAULT_NO_FEEDBACK:      return "FAULT_NO_FEEDBACK";
        case BALLAST_STATUS_FAULT_UNEXPECTED_ON:    return "FAULT_UNEXPECTED_ON";
        case BALLAST_STATUS_FAULT_VOLTAGE_OUT_OF_RANGE: return "FAULT_VOLTAGE_OUT_OF_RANGE";
        default:                                    return "INVALID_STATUS";
    }
}

bool BallastControlGetTaskInfo(TaskHandle_t* task_handle, uint32_t* core_id, uint32_t* high_water_mark)
{
    if (ballast_task_handle == NULL) {
        return false;
    }
    
    if (task_handle != NULL) {
        *task_handle = ballast_task_handle;
    }
    
    if (core_id != NULL) {
        *core_id = BALLAST_TASK_CORE_ID;
    }
    
    if (high_water_mark != NULL) {
        *high_water_mark = uxTaskGetStackHighWaterMark(ballast_task_handle);
    }
    
    return true;
}

/*****************************************************************************
                         PRIVATE FUNCTION IMPLEMENTATIONS
*****************************************************************************/

static bool ballast_gpio_init(void)
{
    gpio_config_t io_conf = {0};
    
    ESP_LOGI(TAG, "Configuring GPIO pins: %d, %d", UV_BALLAST_A_CONTROL_PIN, UV_BALLAST_B_CONTROL_PIN);
    
    // Configure ballast control pins as outputs
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_OUTPUT_BALLAST_PIN_MASK;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIO pins: %s", esp_err_to_name(ret));
        return false;
    }
    
    // Initialize both ballasts to OFF state for safety
    gpio_set_level(UV_BALLAST_A_CONTROL_PIN, BALLAST_OFF);
    gpio_set_level(UV_BALLAST_B_CONTROL_PIN, BALLAST_OFF);
    
    ESP_LOGI(TAG, "GPIO pins initialized successfully - both ballasts OFF");
    return true;
}

static bool ballast_adc_init(void)
{
    ESP_LOGI(TAG, "Initializing ADC for ballast feedback monitoring");
    
    // Configure ADC1 width (pins 34, 35 are on ADC1)
    esp_err_t ret = adc1_config_width(ADC_WIDTH);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure ADC width: %s", esp_err_to_name(ret));
        return false;
    }
    
    // Configure ADC channels for both ballasts
    for (int i = 0; i < UV_BALLAST_COUNT; i++) {
        ret = adc1_config_channel_atten(ballast_adc_channels[i], ADC_ATTEN);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure ADC channel %d: %s", 
                     ballast_adc_channels[i], esp_err_to_name(ret));
            return false;
        }
    }
    
    // Characterize ADC for accurate voltage conversion
    esp_adc_cal_characterize(ADC_UNIT, ADC_ATTEN, ADC_WIDTH, 1100, &adc_chars);
    
    // Verify ADC calibration
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        ESP_LOGI(TAG, "ADC calibrated using eFuse Vref");
    } else {
        ESP_LOGW(TAG, "ADC using default calibration");
    }
    
    ESP_LOGI(TAG, "ADC initialized successfully (Width: %d-bit, Attenuation: %d)", 
             ADC_WIDTH + 9, ADC_ATTEN);
    return true;
}

static bool ballast_sync_init(void)
{
    // Create mutex for feedback data protection
    feedback_mutex = xSemaphoreCreateMutex();
    if (feedback_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create feedback mutex");
        return false;
    }
    
    // Create status queue for inter-task communication
    status_queue = xQueueCreate(BALLAST_STATUS_QUEUE_SIZE, sizeof(ballast_feedback_t));
    if (status_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create status queue");
        vSemaphoreDelete(feedback_mutex);
        return false;
    }
    
    ESP_LOGI(TAG, "Synchronization primitives created (Mutex + Queue size: %d)", 
             BALLAST_STATUS_QUEUE_SIZE);
    return true;
}

static void ballast_control_task(void* parameters)
{
    TickType_t last_switch_time = xTaskGetTickCount();
    TickType_t last_adc_time = xTaskGetTickCount();
    
    ESP_LOGI(TAG, "Ballast control task started on Core %d", xPortGetCoreID());
    ESP_LOGI(TAG, "Task configuration: Priority=%d, Stack=%d, Core=%d", 
             BALLAST_TASK_PRIORITY, BALLAST_TASK_STACK_SIZE, BALLAST_TASK_CORE_ID);
    
    // Initial system stabilization delay
    vTaskDelay(pdMS_TO_TICKS(SYSTEM_STARTUP_DELAY_MS));
    
    // Set safe initial state
    set_ballast_state_internal(UV_BALLAST_A, BALLAST_ON);
    set_ballast_state_internal(UV_BALLAST_B, BALLAST_OFF);
    current_state = BALLAST_STATE_A_ON_B_OFF;
    
    ESP_LOGI(TAG, "Initial state set: Ballast A=ON, Ballast B=OFF");
    
    // Main task loop
    while (1) {
        TickType_t current_time = xTaskGetTickCount();
        
        // Periodic ADC readings for both ballasts
        if ((current_time - last_adc_time) >= pdMS_TO_TICKS(BALLAST_ADC_READ_DELAY_MS)) {
            update_ballast_feedback(UV_BALLAST_A);
            update_ballast_feedback(UV_BALLAST_B);
            last_adc_time = current_time;
        }
        
        // Automatic ballast switching (only in AUTO mode)
        if (operation_mode == BALLAST_MODE_AUTO && 
            (current_time - last_switch_time) >= pdMS_TO_TICKS(BALLAST_SWITCH_DELAY_MS)) {
            
            ESP_LOGI(TAG, "Auto-switching ballast states (mode: %s)", mode_to_string(operation_mode));
            switch_ballast_state_auto();
            last_switch_time = current_time;
        }
        
        // Task scheduling delay
        vTaskDelay(pdMS_TO_TICKS(TASK_LOOP_DELAY_MS));
    }
}

static void set_ballast_state_internal(uv_ballast_t ballast, uint32_t level)
{
    if (ballast >= UV_BALLAST_COUNT) {
        ESP_LOGE(TAG, "Invalid ballast number: %d", ballast);
        return;
    }
    
    gpio_num_t pin = (ballast == UV_BALLAST_A) ? UV_BALLAST_A_CONTROL_PIN : UV_BALLAST_B_CONTROL_PIN;
    
    esp_err_t ret = gpio_set_level(pin, level);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set Ballast %s level: %s", 
                 ballast_name_string(ballast), esp_err_to_name(ret));
        return;
    }
    
    // Update control state in feedback data (thread-safe)
    if (xSemaphoreTake(feedback_mutex, pdMS_TO_TICKS(BALLAST_MUTEX_TIMEOUT_MS)) == pdTRUE) {
        ballast_feedback[ballast].control_state = (level == BALLAST_ON);
        xSemaphoreGive(feedback_mutex);
    }
    
    ESP_LOGI(TAG, "UV Ballast %s: %s", ballast_name_string(ballast), 
             (level == BALLAST_ON) ? "ON" : "OFF");
}

static void switch_ballast_state_auto(void)
{
    switch (current_state) {
        case BALLAST_STATE_A_ON_B_OFF:
            set_ballast_state_internal(UV_BALLAST_A, BALLAST_OFF);
            set_ballast_state_internal(UV_BALLAST_B, BALLAST_ON);
            current_state = BALLAST_STATE_A_OFF_B_ON;
            ESP_LOGI(TAG, "Switched to: Ballast A=OFF, Ballast B=ON");
            break;
            
        case BALLAST_STATE_A_OFF_B_ON:
            set_ballast_state_internal(UV_BALLAST_A, BALLAST_ON);
            set_ballast_state_internal(UV_BALLAST_B, BALLAST_OFF);
            current_state = BALLAST_STATE_A_ON_B_OFF;
            ESP_LOGI(TAG, "Switched to: Ballast A=ON, Ballast B=OFF");
            break;
            
        default:
            ESP_LOGE(TAG, "Invalid ballast state during auto-switch: %d", current_state);
            // Recover to known state
            current_state = BALLAST_STATE_A_ON_B_OFF;
            set_ballast_state_internal(UV_BALLAST_A, BALLAST_ON);
            set_ballast_state_internal(UV_BALLAST_B, BALLAST_OFF);
            break;
    }
}

static void initialize_ballast_feedback_data(void)
{
    for (int i = 0; i < UV_BALLAST_COUNT; i++) {
        ballast_feedback[i].ballast_id = i;
        ballast_feedback[i].voltage_mv = 0;
        ballast_feedback[i].status = BALLAST_STATUS_UNKNOWN;
        ballast_feedback[i].control_state = false;
        ballast_feedback[i].feedback_state = false;
        ballast_feedback[i].timestamp_ms = 0;
        ballast_feedback[i].consecutive_faults = 0;
        ballast_feedback[i].raw_adc_value = 0;
    }
    ESP_LOGI(TAG, "Ballast feedback data structures initialized");
}

static uint32_t read_ballast_voltage(uv_ballast_t ballast)
{
    if (ballast >= UV_BALLAST_COUNT) {
        ESP_LOGE(TAG, "Invalid ballast for ADC reading: %d", ballast);
        return 0;
    }
    
    adc1_channel_t channel = ballast_adc_channels[ballast];
    
    // Take multiple samples and average for noise reduction
    uint32_t adc_reading = 0;
    for (int i = 0; i < BALLAST_ADC_SAMPLES; i++) {
        adc_reading += adc1_get_raw(channel);
        vTaskDelay(pdMS_TO_TICKS(1)); // Small delay between samples
    }
    adc_reading /= BALLAST_ADC_SAMPLES;
    
    // Convert to voltage in millivolts
    uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, &adc_chars);
    
    // Store raw ADC value for debugging
    if (xSemaphoreTake(feedback_mutex, pdMS_TO_TICKS(BALLAST_MUTEX_TIMEOUT_MS)) == pdTRUE) {
        ballast_feedback[ballast].raw_adc_value = adc_reading;
        xSemaphoreGive(feedback_mutex);
    }
    
    return voltage;
}

static ballast_status_t analyze_ballast_status(uv_ballast_t ballast, uint32_t voltage_mv, bool control_state)
{
    // Check for voltage out of range
    if (!validate_adc_reading(voltage_mv)) {
        return BALLAST_STATUS_FAULT_VOLTAGE_OUT_OF_RANGE;
    }
    
    // Determine what the ADC reading indicates
    bool feedback_indicates_on = (voltage_mv >= BALLAST_ON_MIN_VOLTAGE && 
                                  voltage_mv <= BALLAST_ON_MAX_VOLTAGE);
    bool feedback_indicates_off = (voltage_mv >= BALLAST_OFF_MIN_VOLTAGE && 
                                   voltage_mv <= BALLAST_OFF_MAX_VOLTAGE);
    
    // Compare control state with feedback
    if (control_state) {  // We commanded ON
        if (feedback_indicates_on) {
            return BALLAST_STATUS_ON_OK;
        } else {
            return BALLAST_STATUS_FAULT_NO_FEEDBACK;
        }
    } else {  // We commanded OFF
        if (feedback_indicates_off) {
            return BALLAST_STATUS_OFF_OK;
        } else if (feedback_indicates_on) {
            return BALLAST_STATUS_FAULT_UNEXPECTED_ON;
        } else {
            return BALLAST_STATUS_UNKNOWN;
        }
    }
}

static void update_ballast_feedback(uv_ballast_t ballast)
{
    if (ballast >= UV_BALLAST_COUNT) {
        return;
    }
    
    uint32_t voltage = read_ballast_voltage(ballast);
    
    if (xSemaphoreTake(feedback_mutex, pdMS_TO_TICKS(BALLAST_MUTEX_TIMEOUT_MS)) == pdTRUE) {
        ballast_feedback_t* feedback = &ballast_feedback[ballast];
        
        // Update basic readings
        feedback->voltage_mv = voltage;
        feedback->timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
        feedback->feedback_state = (voltage >= BALLAST_ON_MIN_VOLTAGE && 
                                   voltage <= BALLAST_ON_MAX_VOLTAGE);
        
        // Analyze status
        ballast_status_t new_status = analyze_ballast_status(ballast, voltage, feedback->control_state);
        ballast_status_t old_status = feedback->status;
        feedback->status = new_status;
        
        // Handle fault detection and counting
        handle_fault_detection(ballast, feedback);
        
        // Send status update if changed or if it's a fault
        if (new_status != old_status || 
            new_status == BALLAST_STATUS_FAULT_NO_FEEDBACK || 
            new_status == BALLAST_STATUS_FAULT_UNEXPECTED_ON ||
            new_status == BALLAST_STATUS_FAULT_VOLTAGE_OUT_OF_RANGE) {
            send_status_update(feedback);
        }
        
        // Log status periodically or on changes
        log_ballast_status(feedback);
        
        xSemaphoreGive(feedback_mutex);
    } else {
        ESP_LOGW(TAG, "Mutex timeout updating feedback for Ballast %s", ballast_name_string(ballast));
    }
}

static bool validate_adc_reading(uint32_t voltage_mv)
{
    return (voltage_mv <= BALLAST_MAX_VOLTAGE_MV);
}

static void send_status_update(const ballast_feedback_t* feedback)
{
    if (status_queue != NULL) {
        if (xQueueSend(status_queue, feedback, 0) != pdTRUE) {
            ESP_LOGD(TAG, "Status queue full, dropping update for Ballast %s", 
                     ballast_name_string(feedback->ballast_id));
        }
    }
}

static void handle_fault_detection(uv_ballast_t ballast, ballast_feedback_t* feedback)
{
    bool is_fault = (feedback->status == BALLAST_STATUS_FAULT_NO_FEEDBACK ||
                     feedback->status == BALLAST_STATUS_FAULT_UNEXPECTED_ON ||
                     feedback->status == BALLAST_STATUS_FAULT_VOLTAGE_OUT_OF_RANGE);
    
    if (is_fault) {
        feedback->consecutive_faults++;
        if (feedback->consecutive_faults >= MAX_CONSECUTIVE_FAULTS) {
            total_system_faults++;
            ESP_LOGE(TAG, "CRITICAL: Ballast %s has %d consecutive faults!", 
                     ballast_name_string(ballast), feedback->consecutive_faults);
        }
    } else {
        // Reset fault counter on good readings
        if (feedback->consecutive_faults >= FAULT_RESET_THRESHOLD) {
            ESP_LOGI(TAG, "Ballast %s fault condition cleared after %d consecutive good readings", 
                     ballast_name_string(ballast), FAULT_RESET_THRESHOLD);
        }
        feedback->consecutive_faults = 0;
    }
}

static void log_ballast_status(const ballast_feedback_t* feedback)
{
    // Log faults immediately
    if (feedback->status == BALLAST_STATUS_FAULT_NO_FEEDBACK || 
        feedback->status == BALLAST_STATUS_FAULT_UNEXPECTED_ON ||
        feedback->status == BALLAST_STATUS_FAULT_VOLTAGE_OUT_OF_RANGE) {
        ESP_LOGW(TAG, "Ballast %s FAULT: %s (Voltage: %"PRIu32" mV, Raw ADC: %"PRIu32", Faults: %"PRIu16")", 
                 ballast_name_string(feedback->ballast_id),
                 BallastControlStatusToString(feedback->status),
                 feedback->voltage_mv,
                 feedback->raw_adc_value,
                 feedback->consecutive_faults);
    } else {
        // Log normal status at debug level
        ESP_LOGD(TAG, "Ballast %s: %s (Voltage: %"PRIu32" mV, Control: %s, Feedback: %s)", 
                 ballast_name_string(feedback->ballast_id),
                 BallastControlStatusToString(feedback->status),
                 feedback->voltage_mv,
                 feedback->control_state ? "ON" : "OFF",
                 feedback->feedback_state ? "ON" : "OFF");
    }
}

static const char* ballast_name_string(uv_ballast_t ballast)
{
    switch (ballast) {
        case UV_BALLAST_A: return "A";
        case UV_BALLAST_B: return "B";
        default: return "INVALID";
    }
}

static const char* mode_to_string(ballast_mode_t mode)
{
    switch (mode) {
        case BALLAST_MODE_AUTO: return "AUTO";
        case BALLAST_MODE_MANUAL: return "MANUAL";
        case BALLAST_MODE_DISABLED: return "DISABLED";
        default: return "UNKNOWN";
    }
}