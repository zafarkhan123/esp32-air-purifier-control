/**
 * @file main.c
 * @brief ESP32 Air Purifier Main Control Program with Fan Integration
 * 
 * Main application with coordinated fan and ballast control:
 * - Sequence: Fan 30% → Ballast ON → 10s → Ballast OFF → 5s → Fan OFF → Repeat
 * - Dual-core task architecture (Fan+Ballast Control on Core 1, System Monitor on Core 0)
 * - Real-time monitoring with fault detection for both fan and ballast systems
 * - Thread-safe inter-task communication via queues and mutex
 * - Comprehensive system health monitoring and status reporting
 * 
 * @author ZafarKhan - Air Purifier Control
 * @date 2025
 * @version v1.1
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_chip_info.h"
#include "esp_flash.h"        
#include <inttypes.h>         

// Include our driver modules
#include "ballastControl.h"
#include "fanControl.h"

/*****************************************************************************
                       SYSTEM MONITOR TASK CONFIGURATION
*****************************************************************************/

#define SYSTEM_MONITOR_TASK_STACK_SIZE  (3072)
#define SYSTEM_MONITOR_TASK_PRIORITY    (3)
#define SYSTEM_MONITOR_TASK_CORE_ID     (0)     // Run on Core 0 (System Core)
#define SYSTEM_MONITOR_TASK_NAME        "system_monitor"

// Timing configuration for air purifier sequence
#define FAN_SPEED_PERCENT               (30)    // 30% fan speed during operation
#define BALLAST_ON_TIME_MS              (10000) // 10 seconds ballast ON time
#define BALLAST_OFF_TIME_MS             (5000)  // 5 seconds between ballast OFF and fan OFF
#define CYCLE_COMPLETE_DELAY_MS         (2000)  // 2 seconds between cycles

// System monitoring timing
#define SYSTEM_STATUS_CHECK_DELAY_MS    (5000)  // Print system status every 5 seconds
#define STATUS_QUEUE_TIMEOUT_MS         (50)    // Timeout for queue operations
#define MAIN_HEARTBEAT_DELAY_MS         (15000) // Main task heartbeat every 15 seconds

// Health monitoring thresholds
#define MIN_FREE_HEAP_THRESHOLD         (8192)  // Minimum free heap (8KB)
#define MAX_SYSTEM_FAULTS_THRESHOLD     (15)    // Maximum allowed total system faults

/*****************************************************************************
                       STATIC VARIABLES
*****************************************************************************/

static const char* TAG = "MAIN";
static TaskHandle_t system_monitor_task_handle = NULL;

// System health tracking
static uint32_t system_status_counter = 0;
static uint32_t total_ballast_status_updates = 0;
static uint32_t total_fan_status_updates = 0;
static uint32_t air_purifier_cycle_count = 0;

// Air purifier sequence state
typedef enum {
    PURIFIER_STATE_IDLE = 0,
    PURIFIER_STATE_FAN_STARTING,
    PURIFIER_STATE_BALLAST_ON,
    PURIFIER_STATE_BALLAST_OFF_WAIT,
    PURIFIER_STATE_FAN_STOPPING,
    PURIFIER_STATE_CYCLE_COMPLETE
} purifier_state_t;

static purifier_state_t current_purifier_state = PURIFIER_STATE_IDLE;

/*****************************************************************************
                       FUNCTION DECLARATIONS
*****************************************************************************/

// Core system functions
static void print_chip_information(void);
static void print_system_banner(void);

// System monitoring
static void system_monitor_task(void* parameters);
static void print_system_status(void);
static void handle_ballast_status_updates(void);
static void handle_fan_status_updates(void);
static void monitor_system_health(void);

// Air purifier sequence control
static void air_purifier_sequence_task(void* parameters);
static const char* purifier_state_to_string(purifier_state_t state);

// Air purifier sequence control
static void air_purifier_sequence_task(void* parameters);
static const char* purifier_state_to_string(purifier_state_t state);

/*****************************************************************************
                       FUNCTION IMPLEMENTATIONS
*****************************************************************************/

/**
 * @brief Print ESP32 chip information for debugging
 */
static void print_chip_information(void)
{
    uint32_t flash_size = 0;

    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    
    esp_flash_get_size(NULL, &flash_size);

    ESP_LOGI(TAG, "ESP32 Chip Information:");
    ESP_LOGI(TAG, "  Model: ESP32");
    ESP_LOGI(TAG, "  Cores: %d", chip_info.cores);
    ESP_LOGI(TAG, "  Features: WiFi%s%s", 
             (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
             (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");
    ESP_LOGI(TAG, "  Silicon revision: v%d.%d", chip_info.revision / 100, chip_info.revision % 100);
    ESP_LOGI(TAG, "  Flash: %"PRIu32"MB %s",flash_size / (1024 * 1024),(chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
    ESP_LOGI(TAG, "  Free heap: %"PRIu32" bytes", esp_get_free_heap_size());
}

/**
 * @brief Print system startup banner
 */
static void print_system_banner(void)
{
    ESP_LOGI(TAG, "=======================================================");
    ESP_LOGI(TAG, " ESP32 AIR PURIFIER CONTROL SYSTEM v1.1 - WITH FAN");
    ESP_LOGI(TAG, "=======================================================");
    ESP_LOGI(TAG, "Features:");
    ESP_LOGI(TAG, "  ✓ Dual UV Ballast Control");
    ESP_LOGI(TAG, "  ✓ PWM Fan Control with RPM Monitoring");
    ESP_LOGI(TAG, "  ✓ Coordinated Fan+Ballast Sequence");
    ESP_LOGI(TAG, "  ✓ Real-time Fault Detection");
    ESP_LOGI(TAG, "  ✓ Multi-core Task Architecture");
    ESP_LOGI(TAG, "  ✓ Thread-safe Communication");
    ESP_LOGI(TAG, "  ✓ Smooth Speed Ramping");
    ESP_LOGI(TAG, "=======================================================");
    ESP_LOGI(TAG, "Sequence: Fan %d%% → Ballast ON → %ds → Ballast OFF → %ds → Fan OFF", 
             FAN_SPEED_PERCENT, BALLAST_ON_TIME_MS/1000, BALLAST_OFF_TIME_MS/1000);
    ESP_LOGI(TAG, "=======================================================");
}

/**
 * @brief Air purifier sequence control task - runs on Core 1
 * 
 * This task handles the main air purifier sequence:
 * 1. Fan 30% (with ramp up)
 * 2. Ballast ON 
 * 3. Wait 10 seconds
 * 4. Ballast OFF
 * 5. Wait 5 seconds
 * 6. Fan OFF (with ramp down)
 * 7. Repeat cycle
 */
static void air_purifier_sequence_task(void* parameters)
{
    ESP_LOGI(TAG, "Air purifier sequence task started on Core %d", xPortGetCoreID());
    
    TickType_t state_start_time = xTaskGetTickCount();
    
    while (1) {
        TickType_t current_time = xTaskGetTickCount();
        TickType_t elapsed_time = (current_time - state_start_time) * portTICK_PERIOD_MS;
        
        switch (current_purifier_state) {
            case PURIFIER_STATE_IDLE:
                // Start new cycle
                air_purifier_cycle_count++;
                ESP_LOGI(TAG, "🔄 Starting Air Purifier Cycle #%"PRIu32"", air_purifier_cycle_count);
                
                // Start fan with smooth ramp up to 30%
                ESP_LOGI(TAG, "💨 Starting fan with smooth ramp to %d%%", FAN_SPEED_PERCENT);
                if (FanControlSetSpeed(FAN_SPEED_PERCENT, true)) {
                    current_purifier_state = PURIFIER_STATE_FAN_STARTING;
                    state_start_time = current_time;
                } else {
                    ESP_LOGE(TAG, "❌ Failed to start fan - retrying in 2 seconds");
                    vTaskDelay(pdMS_TO_TICKS(2000));
                }
                break;
                
            case PURIFIER_STATE_FAN_STARTING:
                // Wait for fan to finish ramping up
                if (!FanControlIsRamping()) {
                    ESP_LOGI(TAG, "💨 Fan ramp complete - current speed: %"PRIu32"%%", FanControlGetCurrentSpeed());
                    ESP_LOGI(TAG, "🟡 Turning ON UV ballasts");
                    
                    // Set ballasts to manual mode and turn on
                    BallastControlSetMode(BALLAST_MODE_MANUAL);
                    vTaskDelay(pdMS_TO_TICKS(100)); // Small delay for mode change
                    
                    // Turn on both ballasts (or use the existing alternating system)
                    BallastControlSetState(UV_BALLAST_A, true);
                    BallastControlSetState(UV_BALLAST_B, false); // Start with A only, or modify as needed
                    
                    current_purifier_state = PURIFIER_STATE_BALLAST_ON;
                    state_start_time = current_time;
                } else if (elapsed_time > 10000) {  // 10 second timeout for fan ramp
                    ESP_LOGW(TAG, "⚠️  Fan ramp timeout - proceeding anyway");
                    current_purifier_state = PURIFIER_STATE_BALLAST_ON;
                    state_start_time = current_time;
                }
                break;
                
            case PURIFIER_STATE_BALLAST_ON:
                // Wait for ballast ON time (10 seconds)
                if (elapsed_time >= BALLAST_ON_TIME_MS) {
                    ESP_LOGI(TAG, "🔴 Turning OFF UV ballasts after %"PRIu32"ms", elapsed_time);
                    
                    // Turn off ballasts
                    BallastControlSetState(UV_BALLAST_A, false);
                    BallastControlSetState(UV_BALLAST_B, false);
                    
                    current_purifier_state = PURIFIER_STATE_BALLAST_OFF_WAIT;
                    state_start_time = current_time;
                }
                break;
                
            case PURIFIER_STATE_BALLAST_OFF_WAIT:
                // Wait 5 seconds after ballast OFF
                if (elapsed_time >= BALLAST_OFF_TIME_MS) {
                    ESP_LOGI(TAG, "💨 Stopping fan with smooth ramp down");
                    
                    // Stop fan with smooth ramp down
                    if (FanControlStop()) {
                        current_purifier_state = PURIFIER_STATE_FAN_STOPPING;
                        state_start_time = current_time;
                    } else {
                        ESP_LOGE(TAG, "❌ Failed to stop fan - emergency stop");
                        FanControlEmergencyStop();
                        current_purifier_state = PURIFIER_STATE_CYCLE_COMPLETE;
                        state_start_time = current_time;
                    }
                }
                break;
                
            case PURIFIER_STATE_FAN_STOPPING:
                // Wait for fan to finish ramping down
                if (!FanControlIsRamping() && FanControlGetCurrentSpeed() == 0) {
                    ESP_LOGI(TAG, "💨 Fan stopped - current speed: %"PRIu32"%%", FanControlGetCurrentSpeed());
                    current_purifier_state = PURIFIER_STATE_CYCLE_COMPLETE;
                    state_start_time = current_time;
                } else if (elapsed_time > 10000) {  // 10 second timeout for fan ramp down
                    ESP_LOGW(TAG, "⚠️  Fan ramp down timeout - emergency stop");
                    FanControlEmergencyStop();
                    current_purifier_state = PURIFIER_STATE_CYCLE_COMPLETE;
                    state_start_time = current_time;
                }
                break;
                
            case PURIFIER_STATE_CYCLE_COMPLETE:
                // Wait before starting next cycle
                if (elapsed_time >= CYCLE_COMPLETE_DELAY_MS) {
                    ESP_LOGI(TAG, "✅ Cycle #%"PRIu32" complete - total time: %"PRIu32"ms", 
                             air_purifier_cycle_count, 
                             (current_time * portTICK_PERIOD_MS) - (state_start_time * portTICK_PERIOD_MS) + elapsed_time);
                    current_purifier_state = PURIFIER_STATE_IDLE;
                    state_start_time = current_time;
                }
                break;
                
            default:
                ESP_LOGE(TAG, "❌ Invalid purifier state: %d", current_purifier_state);
                current_purifier_state = PURIFIER_STATE_IDLE;
                state_start_time = current_time;
                break;
        }
        
        // Task scheduling delay
        vTaskDelay(pdMS_TO_TICKS(200)); // Check state every 200ms
    }
}

/**
 * @brief System monitoring task - runs on Core 0
 * 
 * This task handles:
 * - Processing ballast and fan status updates from queues
 * - Periodic system health monitoring
 * - Status reporting and logging
 * - Future integration point for WiFi, display, etc.
 */
static void system_monitor_task(void* parameters)
{
    ESP_LOGI(TAG, "System monitor task started on Core %d", xPortGetCoreID());
    
    TickType_t last_status_time = xTaskGetTickCount();
    
    while (1) {
        TickType_t current_time = xTaskGetTickCount();
        
        // Handle incoming status updates from both systems
        handle_ballast_status_updates();
        handle_fan_status_updates();
        
        // Monitor overall system health
        monitor_system_health();
        
        // Print comprehensive system status periodically
        if ((current_time - last_status_time) >= pdMS_TO_TICKS(SYSTEM_STATUS_CHECK_DELAY_MS)) {
            print_system_status();
            last_status_time = current_time;
            system_status_counter++;
        }
        
        // Task scheduling delay
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/**
 * @brief Handle incoming ballast status updates from the queue
 */
static void handle_ballast_status_updates(void)
{
    ballast_feedback_t feedback;
    QueueHandle_t status_queue = BallastControlGetStatusQueue();
    
    if (status_queue == NULL) {
        return;
    }
    
    // Process all available status updates (non-blocking)
    while (xQueueReceive(status_queue, &feedback, 0) == pdTRUE) {
        total_ballast_status_updates++;
        
        // Log significant ballast events
        ESP_LOGD(TAG, "Ballast %c Update #%"PRIu32": %s, %"PRIu32" mV, Faults:%"PRIu16"",
                 'A' + feedback.ballast_id,
                 total_ballast_status_updates,
                 BallastControlStatusToString(feedback.status),
                 feedback.voltage_mv,
                 feedback.consecutive_faults);
        
        // Handle specific fault conditions
        if (feedback.status == BALLAST_STATUS_FAULT_NO_FEEDBACK ||
            feedback.status == BALLAST_STATUS_FAULT_UNEXPECTED_ON ||
            feedback.status == BALLAST_STATUS_FAULT_VOLTAGE_OUT_OF_RANGE) {
            ESP_LOGW(TAG, "⚠️  BALLAST %c FAULT: %s (V:%"PRIu32"mV)", 
                     'A' + feedback.ballast_id, 
                     BallastControlStatusToString(feedback.status),
                     feedback.voltage_mv);
        }
    }
}

/**
 * @brief Handle incoming fan status updates from the queue
 */
static void handle_fan_status_updates(void)
{
    fan_feedback_t feedback;
    QueueHandle_t status_queue = FanControlGetStatusQueue();
    
    if (status_queue == NULL) {
        return;
    }
    
    // Process all available status updates (non-blocking)
    while (xQueueReceive(status_queue, &feedback, 0) == pdTRUE) {
        total_fan_status_updates++;
        
        // Log significant fan events
        ESP_LOGD(TAG, "Fan Update #%"PRIu32": %s, Speed:%"PRIu32"%%, RPM:%"PRIu32", Faults:%"PRIu16"",
                 total_fan_status_updates,
                 FanControlStatusToString(feedback.status),
                 feedback.current_speed_percent,
                 feedback.measured_rpm,
                 feedback.consecutive_faults);
        
        // Handle specific fault conditions
        if (feedback.status == FAN_STATUS_FAULT_STALLED ||
            feedback.status == FAN_STATUS_FAULT_SPEED_ERROR ||
            feedback.status == FAN_STATUS_FAULT_NO_TACHOMETER) {
            ESP_LOGW(TAG, "⚠️  FAN FAULT: %s (Speed:%"PRIu32"%%, RPM:%"PRIu32"/%"PRIu32")", 
                     FanControlStatusToString(feedback.status),
                     feedback.current_speed_percent,
                     feedback.measured_rpm,
                     feedback.expected_rpm);
        }
    }
}

/**
 * @brief Monitor overall system health and performance
 */
static void monitor_system_health(void)
{
    // Check free heap memory
    uint32_t free_heap = esp_get_free_heap_size();
    if (free_heap < MIN_FREE_HEAP_THRESHOLD) {
        ESP_LOGW(TAG, "⚠️  LOW MEMORY WARNING: Free heap = %"PRIu32" bytes (threshold: %d)", 
                 free_heap, MIN_FREE_HEAP_THRESHOLD);
    }
    
    // Check ballast system health
    uint32_t ballast_faults, ballast_uptime;
    bool ballast_healthy = BallastControlGetSystemHealth(&ballast_faults, &ballast_uptime);
    
    // Check fan system health
    uint32_t fan_runtime, fan_faults;
    bool fan_healthy = FanControlGetSystemHealth(&fan_runtime, &fan_faults);
    
    uint32_t total_system_faults = ballast_faults + fan_faults;
    
    if (!ballast_healthy || !fan_healthy) {
        ESP_LOGW(TAG, "⚠️  SYSTEM HEALTH WARNING: Ballast:%s, Fan:%s", 
                 ballast_healthy ? "OK" : "FAULT", 
                 fan_healthy ? "OK" : "FAULT");
    }
    
    if (total_system_faults > MAX_SYSTEM_FAULTS_THRESHOLD) {
        ESP_LOGE(TAG, "🚨 CRITICAL: Total system faults (%"PRIu32") exceeded threshold (%d)", 
                 total_system_faults, MAX_SYSTEM_FAULTS_THRESHOLD);
        // TODO: Implement emergency procedures
    }
}

/**
 * @brief Print comprehensive system status
 */
static void print_system_status(void)
{
    // Get system health information
    uint32_t ballast_faults, ballast_uptime;
    bool ballast_healthy = BallastControlGetSystemHealth(&ballast_faults, &ballast_uptime);
    
    uint32_t fan_runtime, fan_faults;
    bool fan_healthy = FanControlGetSystemHealth(&fan_runtime, &fan_faults);
    
    // Get current states
    ballast_mode_t ballast_mode = BallastControlGetMode();
    fan_mode_t fan_mode = FanControlGetMode();
    uint32_t fan_speed = FanControlGetCurrentSpeed();
    uint32_t fan_rpm = FanControlGetRPM();
    
    ESP_LOGI(TAG, "================== SYSTEM STATUS #%"PRIu32" ==================", system_status_counter);
    ESP_LOGI(TAG, "Purifier State:    %s", purifier_state_to_string(current_purifier_state));
    ESP_LOGI(TAG, "Cycle Count:       %"PRIu32"", air_purifier_cycle_count);
    ESP_LOGI(TAG, "System Health:     %s", (ballast_healthy && fan_healthy) ? "✅ HEALTHY" : "⚠️  DEGRADED");
    ESP_LOGI(TAG, "Uptime:            %"PRIu32" seconds", ballast_uptime / 1000);
    ESP_LOGI(TAG, "Total Faults:      B:%"PRIu32", F:%"PRIu32" (Total:%"PRIu32")", ballast_faults, fan_faults, ballast_faults + fan_faults);
    ESP_LOGI(TAG, "Status Updates:    B:%"PRIu32", F:%"PRIu32"", total_ballast_status_updates, total_fan_status_updates);
    ESP_LOGI(TAG, "Ballast Mode:      %s", 
             (ballast_mode == BALLAST_MODE_AUTO) ? "AUTO" :
             (ballast_mode == BALLAST_MODE_MANUAL) ? "MANUAL" : "DISABLED");
    ESP_LOGI(TAG, "Fan Status:        %s, %"PRIu32"%%, %"PRIu32" RPM", 
             FanControlModeToString(fan_mode), fan_speed, fan_rpm);
    ESP_LOGI(TAG, "Free Heap:         %"PRIu32" bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "========================================================");
}

/**
 * @brief Convert purifier state enum to human-readable string
 */
static const char* purifier_state_to_string(purifier_state_t state)
{
    switch (state) {
        case PURIFIER_STATE_IDLE:           return "IDLE";
        case PURIFIER_STATE_FAN_STARTING:   return "FAN_STARTING";
        case PURIFIER_STATE_BALLAST_ON:     return "BALLAST_ON";
        case PURIFIER_STATE_BALLAST_OFF_WAIT: return "BALLAST_OFF_WAIT";
        case PURIFIER_STATE_FAN_STOPPING:   return "FAN_STOPPING";
        case PURIFIER_STATE_CYCLE_COMPLETE: return "CYCLE_COMPLETE";
        default:                            return "UNKNOWN";
    }
}

/*****************************************************************************
                       MAIN APPLICATION ENTRY POINT
*****************************************************************************/

/**
 * @brief Main application entry point
 */
void app_main(void)
{
    // Print system banner and information
    print_system_banner();
    print_chip_information();
    
    // Check initial system health
    uint32_t initial_free_heap = esp_get_free_heap_size();
    if (initial_free_heap < MIN_FREE_HEAP_THRESHOLD) {
        ESP_LOGW(TAG, "⚠️  WARNING: Low initial free heap: %"PRIu32" bytes", initial_free_heap);
    }
    
    // Initialize ballast control module
    ESP_LOGI(TAG, "🔧 Initializing Ballast Control Module...");
    if (!BallastControlInit()) {
        ESP_LOGE(TAG, "❌ CRITICAL: Failed to initialize Ballast Control Module");
        ESP_LOGE(TAG, "❌ SYSTEM HALTED - Check hardware connections and power supply");
        return;
    }
    ESP_LOGI(TAG, "✅ Ballast Control Module initialized successfully");
    
    // Initialize fan control module
    ESP_LOGI(TAG, "🔧 Initializing Fan Control Module...");
    if (!FanControlInit()) {
        ESP_LOGE(TAG, "❌ CRITICAL: Failed to initialize Fan Control Module");
        ESP_LOGE(TAG, "❌ SYSTEM HALTED - Check PWM and tachometer connections");
        return;
    }
    ESP_LOGI(TAG, "✅ Fan Control Module initialized successfully");
    
    // Create ballast control task (runs on Core 1 - Application Core)
    ESP_LOGI(TAG, "🔧 Creating Ballast Control Task...");
    if (BallastControlTaskCreate() != pdPASS) {
        ESP_LOGE(TAG, "❌ CRITICAL: Failed to create Ballast Control Task");
        ESP_LOGE(TAG, "❌ Free heap: %"PRIu32" bytes", esp_get_free_heap_size());
        ESP_LOGE(TAG, "❌ SYSTEM HALTED - Insufficient memory or task creation failure");
        return;
    }
    ESP_LOGI(TAG, "✅ Ballast Control Task created successfully on Core 1");
    
    // Create fan control task (runs on Core 1 - Application Core)
    ESP_LOGI(TAG, "🔧 Creating Fan Control Task...");
    if (FanControlTaskCreate() != pdPASS) {
        ESP_LOGE(TAG, "❌ CRITICAL: Failed to create Fan Control Task");
        ESP_LOGE(TAG, "❌ Free heap: %"PRIu32" bytes", esp_get_free_heap_size());
        ESP_LOGE(TAG, "❌ SYSTEM HALTED - Insufficient memory or task creation failure");
        return;
    }
    ESP_LOGI(TAG, "✅ Fan Control Task created successfully on Core 1");
    
    // Create system monitor task (runs on Core 0 - System Core)
    ESP_LOGI(TAG, "🔧 Creating System Monitor Task...");
    BaseType_t result = xTaskCreatePinnedToCore(
        system_monitor_task,                // Task function
        SYSTEM_MONITOR_TASK_NAME,           // Task name
        SYSTEM_MONITOR_TASK_STACK_SIZE,     // Stack size
        NULL,                               // Parameters
        SYSTEM_MONITOR_TASK_PRIORITY,       // Priority
        &system_monitor_task_handle,        // Task handle
        SYSTEM_MONITOR_TASK_CORE_ID        // Core ID (Core 0 - System Core)
    );
    
    if (result != pdPASS) {
        ESP_LOGE(TAG, "❌ CRITICAL: Failed to create System Monitor Task");
        ESP_LOGE(TAG, "❌ Free heap: %"PRIu32" bytes", esp_get_free_heap_size());
        return;
    }
    ESP_LOGI(TAG, "✅ System Monitor Task created successfully on Core 0");
    
    // Create air purifier sequence task (runs on Core 1 - Application Core)
    ESP_LOGI(TAG, "🔧 Creating Air Purifier Sequence Task...");
    TaskHandle_t sequence_task_handle = NULL;
    result = xTaskCreatePinnedToCore(
        air_purifier_sequence_task,         // Task function
        "purifier_sequence",                // Task name
        4096,                               // Stack size
        NULL,                               // Parameters
        5,                                  // Priority (higher than individual control tasks)
        &sequence_task_handle,              // Task handle
        1                                   // Core ID (Core 1 - Application Core)
    );
    
    if (result != pdPASS) {
        ESP_LOGE(TAG, "❌ CRITICAL: Failed to create Air Purifier Sequence Task");
        ESP_LOGE(TAG, "❌ Free heap: %"PRIu32" bytes", esp_get_free_heap_size());
        return;
    }
    ESP_LOGI(TAG, "✅ Air Purifier Sequence Task created successfully on Core 1");
    
    // System startup complete
    ESP_LOGI(TAG, "=======================================================");
    ESP_LOGI(TAG, "🚀 AIR PURIFIER SYSTEM STARTUP COMPLETE");
    ESP_LOGI(TAG, "=======================================================");
    ESP_LOGI(TAG, "Architecture Summary:");
    ESP_LOGI(TAG, "  📋 Core 0 (System):     System Monitor Task");
    ESP_LOGI(TAG, "  ⚙️  Core 1 (Application): Ballast + Fan + Sequence Tasks");
    ESP_LOGI(TAG, "  🔄 Sequence:            Fan %d%% → Ballast → %ds → OFF → %ds → Repeat", 
             FAN_SPEED_PERCENT, BALLAST_ON_TIME_MS/1000, BALLAST_OFF_TIME_MS/1000);
    ESP_LOGI(TAG, "  💨 Fan Control:         PWM + RPM monitoring with smooth ramping");
    ESP_LOGI(TAG, "  🟡 Ballast Control:     Dual UV with ADC feedback");
    ESP_LOGI(TAG, "  🧵 Thread-safe design:  Mutex + Queue communication");
    ESP_LOGI(TAG, "=======================================================");
    ESP_LOGI(TAG, "💡 Ready for operation - air purifier sequence starting...");
    
    // Main task heartbeat loop
    uint32_t heartbeat_counter = 0;
    while (1) {
        // Main task heartbeat
        vTaskDelay(pdMS_TO_TICKS(MAIN_HEARTBEAT_DELAY_MS));
        heartbeat_counter++;
        
        ESP_LOGI(TAG, "💓 Main heartbeat #%"PRIu32" - Cycles: %"PRIu32", State: %s", 
                 heartbeat_counter, air_purifier_cycle_count, purifier_state_to_string(current_purifier_state));
        ESP_LOGI(TAG, "📈 Free heap: %"PRIu32" bytes, Fan: %"PRIu32"%% @ %"PRIu32" RPM", 
                 esp_get_free_heap_size(), 
                 FanControlGetCurrentSpeed(),
                 FanControlGetRPM());
    }
}