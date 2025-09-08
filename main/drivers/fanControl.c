/**
 * @file fanControl.c
 * @brief Minimal Fan Control Driver for ESP-IDF 5.4.1
 */

#include "fanControl.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <inttypes.h>
#include <string.h>

// Simple defines
#define FAN_LEDC_TIMER              LEDC_TIMER_1
#define FAN_LEDC_MODE               LEDC_LOW_SPEED_MODE
#define FAN_LEDC_CHANNEL            LEDC_CHANNEL_0
#define FAN_LEDC_DUTY_RESOLUTION    LEDC_TIMER_12_BIT
#define PERCENT_TO_DUTY(percent)    ((percent * FAN_PWM_MAX_DUTY) / 100)

// Static variables
static const char* TAG = "FAN_CTRL";
static TaskHandle_t fan_task_handle = NULL;
static fan_feedback_t fan_feedback = {0};
static SemaphoreHandle_t feedback_mutex = NULL;
static QueueHandle_t status_queue = NULL;
static pcnt_unit_handle_t pcnt_unit_handle = NULL;

// Forward declarations
static bool fan_pwm_init(void);
static bool fan_tachometer_init(void);
static void fan_control_task(void* parameters);
static bool set_pwm_duty_cycle(uint32_t duty_cycle);

// Public interface implementation
bool FanControlInit(void)
{
    ESP_LOGI(TAG, "Initializing Fan Control Module v1.1");
    
    // Create mutex
    feedback_mutex = xSemaphoreCreateMutex();
    if (feedback_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create feedback mutex");
        return false;
    }
    
    // Create status queue
    status_queue = xQueueCreate(FAN_STATUS_QUEUE_SIZE, sizeof(fan_feedback_t));
    if (status_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create status queue");
        vSemaphoreDelete(feedback_mutex);
        return false;
    }
    
    // Initialize PWM
    if (!fan_pwm_init()) {
        ESP_LOGE(TAG, "Failed to initialize PWM");
        return false;
    }
    
    // Initialize tachometer
    if (!fan_tachometer_init()) {
        ESP_LOGE(TAG, "Failed to initialize tachometer");
        return false;
    }
    
    // Initialize feedback data
    memset(&fan_feedback, 0, sizeof(fan_feedback_t));
    fan_feedback.mode = FAN_MODE_OFF;
    fan_feedback.status = FAN_STATUS_OFF_OK;
    
    ESP_LOGI(TAG, "Fan Control Module initialized successfully");
    return true;
}

BaseType_t FanControlTaskCreate(void)
{
    if (fan_task_handle != NULL) {
        ESP_LOGW(TAG, "Fan control task already exists");
        return pdFAIL;
    }
    
    BaseType_t result = xTaskCreatePinnedToCore(
        fan_control_task,
        FAN_TASK_NAME,
        FAN_TASK_STACK_SIZE,
        NULL,
        FAN_TASK_PRIORITY,
        &fan_task_handle,
        FAN_TASK_CORE_ID
    );
    
    if (result == pdPASS) {
        ESP_LOGI(TAG, "Fan control task created successfully on Core %d", FAN_TASK_CORE_ID);
    } else {
        ESP_LOGE(TAG, "Failed to create fan control task");
    }
    
    return result;
}

bool FanControlSetSpeed(uint32_t speed_percent, bool enable_ramping)
{
    if (speed_percent > 100) {
        ESP_LOGW(TAG, "Invalid speed percentage: %"PRIu32"%%", speed_percent);
        return false;
    }
    
    if (xSemaphoreTake(feedback_mutex, pdMS_TO_TICKS(FAN_MUTEX_TIMEOUT_MS)) == pdTRUE) {
        fan_feedback.target_speed_percent = speed_percent;
        fan_feedback.current_speed_percent = speed_percent;
        fan_feedback.current_duty_cycle = PERCENT_TO_DUTY(speed_percent);
        fan_feedback.mode = (speed_percent > 0) ? FAN_MODE_MANUAL : FAN_MODE_OFF;
        fan_feedback.is_ramping = false;
        
        set_pwm_duty_cycle(fan_feedback.current_duty_cycle);
        ESP_LOGI(TAG, "Set fan speed: %"PRIu32"%%", speed_percent);
        
        xSemaphoreGive(feedback_mutex);
        return true;
    }
    return false;
}

bool FanControlSetSpeedImmediate(uint32_t speed_percent)
{
    return FanControlSetSpeed(speed_percent, false);
}

uint32_t FanControlGetCurrentSpeed(void)
{
    uint32_t speed = 0;
    if (xSemaphoreTake(feedback_mutex, pdMS_TO_TICKS(FAN_MUTEX_TIMEOUT_MS)) == pdTRUE) {
        speed = fan_feedback.current_speed_percent;
        xSemaphoreGive(feedback_mutex);
    }
    return speed;
}

uint32_t FanControlGetRPM(void)
{
    uint32_t rpm = 0;
    if (xSemaphoreTake(feedback_mutex, pdMS_TO_TICKS(FAN_MUTEX_TIMEOUT_MS)) == pdTRUE) {
        rpm = fan_feedback.measured_rpm;
        xSemaphoreGive(feedback_mutex);
    }
    return rpm;
}

bool FanControlIsRamping(void)
{
    bool ramping = false;
    if (xSemaphoreTake(feedback_mutex, pdMS_TO_TICKS(FAN_MUTEX_TIMEOUT_MS)) == pdTRUE) {
        ramping = fan_feedback.is_ramping;
        xSemaphoreGive(feedback_mutex);
    }
    return ramping;
}

bool FanControlStop(void)
{
    ESP_LOGI(TAG, "Stopping fan");
    return FanControlSetSpeed(0, true);
}

bool FanControlEmergencyStop(void)
{
    ESP_LOGW(TAG, "Emergency stop");
    return FanControlSetSpeed(0, false);
}

QueueHandle_t FanControlGetStatusQueue(void)
{
    return status_queue;
}

const char* FanControlStatusToString(fan_status_t status)
{
    switch (status) {
        case FAN_STATUS_UNKNOWN:            return "UNKNOWN";
        case FAN_STATUS_OFF_OK:             return "OFF_OK";
        case FAN_STATUS_RUNNING_OK:         return "RUNNING_OK";
        case FAN_STATUS_RAMPING_UP:         return "RAMPING_UP";
        case FAN_STATUS_RAMPING_DOWN:       return "RAMPING_DOWN";
        case FAN_STATUS_FAULT_STALLED:      return "FAULT_STALLED";
        case FAN_STATUS_FAULT_SPEED_ERROR:  return "FAULT_SPEED_ERROR";
        case FAN_STATUS_FAULT_NO_TACHOMETER: return "FAULT_NO_TACHOMETER";
        default:                            return "INVALID_STATUS";
    }
}

const char* FanControlModeToString(fan_mode_t mode)
{
    switch (mode) {
        case FAN_MODE_OFF:          return "OFF";
        case FAN_MODE_MANUAL:       return "MANUAL";
        case FAN_MODE_AUTO:         return "AUTO";
        case FAN_MODE_RAMP_UP:      return "RAMP_UP";
        case FAN_MODE_RAMP_DOWN:    return "RAMP_DOWN";
        case FAN_MODE_FAULT:        return "FAULT";
        default:                    return "UNKNOWN";
    }
}

// Private function implementations
static bool fan_pwm_init(void)
{
    ESP_LOGI(TAG, "Initializing PWM on GPIO %d", FAN_PWM_PIN);
    
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = FAN_LEDC_DUTY_RESOLUTION,
        .freq_hz = FAN_PWM_FREQUENCY_HZ,
        .speed_mode = FAN_LEDC_MODE,
        .timer_num = FAN_LEDC_TIMER,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    
    esp_err_t ret = ledc_timer_config(&ledc_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LEDC timer: %s", esp_err_to_name(ret));
        return false;
    }
    
    ledc_channel_config_t ledc_channel = {
        .channel = FAN_LEDC_CHANNEL,
        .duty = 0,
        .gpio_num = FAN_PWM_PIN,
        .speed_mode = FAN_LEDC_MODE,
        .hpoint = 0,
        .timer_sel = FAN_LEDC_TIMER
    };
    
    ret = ledc_channel_config(&ledc_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LEDC channel: %s", esp_err_to_name(ret));
        return false;
    }
    
    ESP_LOGI(TAG, "PWM initialized successfully");
    return true;
}

static bool fan_tachometer_init(void)
{
    ESP_LOGI(TAG, "Initializing tachometer on GPIO %d", FAN_TACHO_PIN);
    
    pcnt_unit_config_t unit_config = {
        .high_limit = 10000,
        .low_limit = -10000,
    };
    
    esp_err_t ret = pcnt_new_unit(&unit_config, &pcnt_unit_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create PCNT unit: %s", esp_err_to_name(ret));
        return false;
    }
    
    pcnt_chan_config_t chan_config = {
        .edge_gpio_num = FAN_TACHO_PIN,
        .level_gpio_num = -1,
    };
    
    pcnt_channel_handle_t pcnt_chan = NULL;
    ret = pcnt_new_channel(pcnt_unit_handle, &chan_config, &pcnt_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create PCNT channel: %s", esp_err_to_name(ret));
        return false;
    }
    
    ret = pcnt_channel_set_edge_action(pcnt_chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set PCNT edge action: %s", esp_err_to_name(ret));
        return false;
    }
    
    ret = pcnt_unit_enable(pcnt_unit_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable PCNT unit: %s", esp_err_to_name(ret));
        return false;
    }
    
    ret = pcnt_unit_clear_count(pcnt_unit_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to clear PCNT count: %s", esp_err_to_name(ret));
        return false;
    }
    
    ret = pcnt_unit_start(pcnt_unit_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start PCNT unit: %s", esp_err_to_name(ret));
        return false;
    }
    
    ESP_LOGI(TAG, "Tachometer initialized successfully");
    return true;
}

static void fan_control_task(void* parameters)
{
    ESP_LOGI(TAG, "Fan control task started on Core %d", xPortGetCoreID());
    
    while (1) {
        // Read RPM if we have a valid handle
        if (pcnt_unit_handle != NULL) {
            int pulse_count;
            if (pcnt_unit_get_count(pcnt_unit_handle, &pulse_count) == ESP_OK) {
                pcnt_unit_clear_count(pcnt_unit_handle);
                
                if (xSemaphoreTake(feedback_mutex, pdMS_TO_TICKS(FAN_MUTEX_TIMEOUT_MS)) == pdTRUE) {
                    // Simple RPM calculation (adjust based on your fan)
                    fan_feedback.measured_rpm = pulse_count * 30; // Simplified
                    fan_feedback.timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                    
                    // Update status
                    if (fan_feedback.current_speed_percent == 0) {
                        fan_feedback.status = FAN_STATUS_OFF_OK;
                    } else {
                        fan_feedback.status = FAN_STATUS_RUNNING_OK;
                    }
                    
                    xSemaphoreGive(feedback_mutex);
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000)); // 1 second delay
    }
}

static bool set_pwm_duty_cycle(uint32_t duty_cycle)
{
    if (duty_cycle > FAN_PWM_MAX_DUTY) {
        duty_cycle = FAN_PWM_MAX_DUTY;
    }
    
    esp_err_t ret = ledc_set_duty(FAN_LEDC_MODE, FAN_LEDC_CHANNEL, duty_cycle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set PWM duty: %s", esp_err_to_name(ret));
        return false;
    }
    
    ret = ledc_update_duty(FAN_LEDC_MODE, FAN_LEDC_CHANNEL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to update PWM duty: %s", esp_err_to_name(ret));
        return false;
    }
    
    return true;
}

// Stub implementations for remaining functions
void FanControlSetMode(fan_mode_t mode) { }
fan_mode_t FanControlGetMode(void) { return FAN_MODE_MANUAL; }
bool FanControlGetFeedback(fan_feedback_t* feedback) { if(feedback) *feedback = fan_feedback; return true; }
bool FanControlGetSystemHealth(uint32_t* runtime, uint32_t* faults) { return true; }
bool FanControlGetTaskInfo(TaskHandle_t* h, uint32_t* c, uint32_t* s) { return true; }
bool FanControlConfigure(const fan_config_t* config) { return true; }
bool FanControlForceRPMRead(void) { return true; }