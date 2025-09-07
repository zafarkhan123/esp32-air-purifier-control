/**
 * @file main.c
 * @brief ESP32 Air Purifier Main Control Program
 * 
 * Main application that demonstrates the modular ballast control system with:
 * - Dual-core task architecture (Ballast Control on Core 1, System Monitor on Core 0)
 * - Real-time ADC feedback monitoring with fault detection
 * - Thread-safe inter-task communication via queues and mutex
 * - Comprehensive system health monitoring and status reporting
 * - Extensible design for additional air purifier components
 * 
 * @author ZafarKhan - Air Purifier Control
 * idf.py version 
 * @date 2025
 * @version v1.0
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
// Include our ballast control driver module
#include "ballastControl.h"

/*****************************************************************************
                       ADDITIONAL PIN DEFINITIONS (For Future Extension)
*****************************************************************************/

// Fan Control Pins
#define FAN_PWM_PIN                     (23)    // GPIO23 - FAN_PWM
#define FAN_TACHO_PIN                   (4)     // GPIO4  - FAN_TACHO

// RGB LED Pin
#define RGB_LED_DATA_PIN                (17)    // GPIO17 - SLED_DATA

// Touch Interface
#define TOUCH_INTERRUPT_PIN             (36)    // GPIO36 - TP_INT

// I2C Communication
#define I2C_SDA_PIN                     (26)    // GPIO26 - I2C_SDA
#define I2C_SCL_PIN                     (27)    // GPIO27 - I2C_SCL

// SPI Communication
#define SPI_SCLK_PIN                    (25)    // GPIO25 - SPI_SCL
#define SPI_MOSI_PIN                    (33)    // GPIO33 - SPI_MOSI
#define SPI_MISO_PIN                    (32)    // GPIO32 - SPI_MISO

// Ethernet Control
#define ETHERNET_CS_PIN                 (18)    // GPIO18 - ETH_CS
#define ETHERNET_INT_PIN                (39)    // GPIO39 - ETH_INT
#define ETHERNET_RST_PIN                (21)    // GPIO21 - ETH_RST

// External Flash
#define EXTERNAL_FLASH_CS_PIN           (5)     // GPIO5  - FLASH_CS
#define EXTERNAL_FLASH_RST_PIN          (0)     // GPIO0  - FLASH_RST

// GPIO Expander
#define GPIO_EXPANDER_INT_PIN           (16)    // GPIO16 - EXP_INT

/*****************************************************************************
                       SYSTEM MONITOR TASK CONFIGURATION
*****************************************************************************/

#define SYSTEM_MONITOR_TASK_STACK_SIZE  (3072)
#define SYSTEM_MONITOR_TASK_PRIORITY    (3)
#define SYSTEM_MONITOR_TASK_CORE_ID     (0)     // Run on Core 0 (System Core)
#define SYSTEM_MONITOR_TASK_NAME        "system_monitor"

// Timing configuration
#define SYSTEM_STATUS_CHECK_DELAY_MS    (5000)  // Print system status every 5 seconds
#define STATUS_QUEUE_TIMEOUT_MS         (100)   // Timeout for queue operations
#define MAIN_HEARTBEAT_DELAY_MS         (30000) // Main task heartbeat every 30 seconds

// Health monitoring thresholds
#define MIN_FREE_HEAP_THRESHOLD         (8192)  // Minimum free heap (8KB)
#define MAX_BALLAST_FAULTS_THRESHOLD    (10)    // Maximum allowed system faults

/*****************************************************************************
                       STATIC VARIABLES
*****************************************************************************/

static const char* TAG = "MAIN";
static TaskHandle_t system_monitor_task_handle = NULL;

// System health tracking
static uint32_t system_status_counter = 0;
static uint32_t total_status_updates_received = 0;
static bool demo_mode_enabled = false;

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
static void monitor_system_health(void);

// Future extension placeholders
static void initialize_other_peripherals(void);

// Demo and testing functions
static void demo_manual_control(void);

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
    ESP_LOGI(TAG, "  Free heap: %lu bytes", esp_get_free_heap_size());
}

/**
 * @brief Print system startup banner
 */
static void print_system_banner(void)
{
    ESP_LOGI(TAG, "=================================================");
    ESP_LOGI(TAG, "   ESP32 AIR PURIFIER CONTROL SYSTEM v1.0");
    ESP_LOGI(TAG, "=================================================");
    ESP_LOGI(TAG, "Features:");
    ESP_LOGI(TAG, "  ✓ Dual UV Ballast Control");
    ESP_LOGI(TAG, "  ✓ Real-time ADC Monitoring");
    ESP_LOGI(TAG, "  ✓ Fault Detection & Recovery");
    ESP_LOGI(TAG, "  ✓ Multi-core Task Architecture");
    ESP_LOGI(TAG, "  ✓ Thread-safe Communication");
    ESP_LOGI(TAG, "  ✓ Modular Extension Ready");
    ESP_LOGI(TAG, "=================================================");
}

/**
 * @brief System monitoring task - runs on Core 0
 * 
 * This task handles:
 * - Processing ballast status updates from the queue
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
        
        // Handle incoming ballast status updates
        handle_ballast_status_updates();
        
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
        total_status_updates_received++;
        
        // Log the status update with detailed information
        ESP_LOGI(TAG, "Ballast %c Update #%lu: %s, %lu mV, Ctrl:%s, FB:%s, Faults:%d",
                 'A' + feedback.ballast_id,
                 total_status_updates_received,
                 BallastControlStatusToString(feedback.status),
                 feedback.voltage_mv,
                 feedback.control_state ? "ON" : "OFF",
                 feedback.feedback_state ? "ON" : "OFF",
                 feedback.consecutive_faults);
        
        // Handle specific fault conditions with appropriate responses
        switch (feedback.status) {
            case BALLAST_STATUS_FAULT_NO_FEEDBACK:
                ESP_LOGW(TAG, "⚠️  BALLAST %c FAULT: No feedback detected", 'A' + feedback.ballast_id);
                ESP_LOGW(TAG, "   Possible causes: Ballast disconnected, relay failure, wiring issue");
                // TODO: Add fault recovery actions, notifications, safety shutdowns
                break;
                
            case BALLAST_STATUS_FAULT_UNEXPECTED_ON:
                ESP_LOGW(TAG, "⚠️  BALLAST %c FAULT: Unexpected ON state", 'A' + feedback.ballast_id);
                ESP_LOGW(TAG, "   Possible causes: Relay stuck, control signal failure");
                // TODO: Add emergency shutdown procedures
                break;
                
            case BALLAST_STATUS_FAULT_VOLTAGE_OUT_OF_RANGE:
                ESP_LOGW(TAG, "⚠️  BALLAST %c FAULT: Voltage out of range (%lu mV)", 
                         'A' + feedback.ballast_id, feedback.voltage_mv);
                ESP_LOGW(TAG, "   Possible causes: Power supply issue, ADC malfunction");
                break;
                
            case BALLAST_STATUS_ON_OK:
            case BALLAST_STATUS_OFF_OK:
                // Normal operation - log at debug level only
                ESP_LOGD(TAG, "Ballast %c operating normally: %s", 
                         'A' + feedback.ballast_id, BallastControlStatusToString(feedback.status));
                break;
                
            default:
                ESP_LOGD(TAG, "Ballast %c status: %s", 
                         'A' + feedback.ballast_id, BallastControlStatusToString(feedback.status));
                break;
        }
        
        // TODO: Add integration points for:
        // - WiFi status reporting to cloud/mobile app
        // - Display updates (LCD/OLED)
        // - Touch panel status indicators
        // - Data logging to external flash
        // - Email/SMS notifications for critical faults
        // - Integration with home automation systems
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
        ESP_LOGW(TAG, "⚠️  LOW MEMORY WARNING: Free heap = %lu bytes (threshold: %d)", 
                 free_heap, MIN_FREE_HEAP_THRESHOLD);
    }
    
    // Check ballast system health
    uint32_t total_faults, uptime_ms;
    bool system_healthy = BallastControlGetSystemHealth(&total_faults, &uptime_ms);
    
    if (!system_healthy) {
        ESP_LOGW(TAG, "⚠️  BALLAST SYSTEM HEALTH WARNING");
    }
    
    if (total_faults > MAX_BALLAST_FAULTS_THRESHOLD) {
        ESP_LOGE(TAG, "🚨 CRITICAL: Total system faults (%lu) exceeded threshold (%d)", 
                 total_faults, MAX_BALLAST_FAULTS_THRESHOLD);
        // TODO: Implement emergency procedures
    }
    
    // Check task health
    TaskHandle_t ballast_task;
    uint32_t core_id, stack_high_water;
    if (BallastControlGetTaskInfo(&ballast_task, &core_id, &stack_high_water)) {
        if (stack_high_water < 512) {  // Less than 512 bytes free stack
            ESP_LOGW(TAG, "⚠️  BALLAST TASK LOW STACK: %lu bytes remaining", stack_high_water);
        }
    }
}

/**
 * @brief Print comprehensive system status
 */
static void print_system_status(void)
{
    uint32_t total_faults, uptime_ms;
    bool system_healthy = BallastControlGetSystemHealth(&total_faults, &uptime_ms);
    
    // Get current ballast states
    ballast_state_t current_state = BallastControlGetCurrentState();
    ballast_mode_t operation_mode = BallastControlGetMode();
    
    // Get individual ballast feedback
    ballast_feedback_t feedback_a, feedback_b;
    bool got_a = BallastControlGetFeedback(UV_BALLAST_A, &feedback_a);
    bool got_b = BallastControlGetFeedback(UV_BALLAST_B, &feedback_b);
    
    ESP_LOGI(TAG, "================== SYSTEM STATUS #%lu ==================", system_status_counter);
    ESP_LOGI(TAG, "System Health:     %s", system_healthy ? "✅ HEALTHY" : "⚠️  DEGRADED");
    ESP_LOGI(TAG, "Uptime:           %lu seconds", uptime_ms / 1000);
    ESP_LOGI(TAG, "Total Faults:     %lu", total_faults);
    ESP_LOGI(TAG, "Status Updates:   %lu received", total_status_updates_received);
    ESP_LOGI(TAG, "Operation Mode:   %s", 
             (operation_mode == BALLAST_MODE_AUTO) ? "AUTO" :
             (operation_mode == BALLAST_MODE_MANUAL) ? "MANUAL" : "DISABLED");
    ESP_LOGI(TAG, "Current State:    %s", 
             (current_state == BALLAST_STATE_A_ON_B_OFF) ? "A=ON, B=OFF" : "A=OFF, B=ON");
    
    if (got_a) {
        ESP_LOGI(TAG, "Ballast A:        %s, %lu mV, Faults: %d", 
                 BallastControlStatusToString(feedback_a.status),
                 feedback_a.voltage_mv, feedback_a.consecutive_faults);
    }
    
    if (got_b) {
        ESP_LOGI(TAG, "Ballast B:        %s, %lu mV, Faults: %d", 
                 BallastControlStatusToString(feedback_b.status),
                 feedback_b.voltage_mv, feedback_b.consecutive_faults);
    }
    
    ESP_LOGI(TAG, "Free Heap:        %lu bytes", esp_get_free_heap_size());
    
    // Task information
    TaskHandle_t ballast_task;
    uint32_t core_id, stack_high_water;
    if (BallastControlGetTaskInfo(&ballast_task, &core_id, &stack_high_water)) {
        ESP_LOGI(TAG, "Ballast Task:     Core %lu, Stack: %lu bytes free", core_id, stack_high_water);
    }
    
    ESP_LOGI(TAG, "========================================================");
}

/**
 * @brief Initialize other air purifier peripherals (placeholder for future expansion)
 */
static void initialize_other_peripherals(void)
{
    ESP_LOGI(TAG, "Initializing additional peripherals...");
    
    // TODO: Future peripheral initialization
    // - Fan control module
    // - RGB LED strip control
    // - Touch interface
    // - WiFi connectivity
    // - I2C sensors (temperature, humidity, air quality)
    // - SPI devices (Ethernet, external flash)
    // - Display interface (LCD/OLED)
    // - Audio feedback/alerts
    
    ESP_LOGI(TAG, "Additional peripherals initialization placeholder complete");
}

/**
 * @brief Demonstrate manual ballast control (for testing)
 */
static void demo_manual_control(void)
{
    if (!demo_mode_enabled) {
        return;
    }
    
    ESP_LOGI(TAG, "🧪 DEMO: Starting manual control demonstration...");
    
    // Switch to manual mode
    BallastControlSetMode(BALLAST_MODE_MANUAL);
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Manual control sequence
    ESP_LOGI(TAG, "🧪 DEMO: Manual ON - Ballast A");
    BallastControlSetState(UV_BALLAST_A, true);
    BallastControlSetState(UV_BALLAST_B, false);
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    ESP_LOGI(TAG, "🧪 DEMO: Manual ON - Ballast B");
    BallastControlSetState(UV_BALLAST_A, false);
    BallastControlSetState(UV_BALLAST_B, true);
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    ESP_LOGI(TAG, "🧪 DEMO: Manual OFF - Both ballasts");
    BallastControlSetState(UV_BALLAST_A, false);
    BallastControlSetState(UV_BALLAST_B, false);
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Return to automatic mode
    ESP_LOGI(TAG, "🧪 DEMO: Returning to automatic mode");
    BallastControlSetMode(BALLAST_MODE_AUTO);
    
    ESP_LOGI(TAG, "🧪 DEMO: Manual control demonstration complete");
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
        ESP_LOGW(TAG, "⚠️  WARNING: Low initial free heap: %lu bytes", initial_free_heap);
    }
    
    // Initialize ballast control module
    ESP_LOGI(TAG, "🔧 Initializing Ballast Control Module...");
    if (!BallastControlInit()) {
        ESP_LOGE(TAG, "❌ CRITICAL: Failed to initialize Ballast Control Module");
        ESP_LOGE(TAG, "❌ SYSTEM HALTED - Check hardware connections and power supply");
        return;
    }
    ESP_LOGI(TAG, "✅ Ballast Control Module initialized successfully");
    
    // Initialize other peripherals (placeholder for future expansion)
    initialize_other_peripherals();
    
    // Create ballast control task (runs on Core 1 - Application Core)
    ESP_LOGI(TAG, "🔧 Creating Ballast Control Task...");
    if (BallastControlTaskCreate() != pdPASS) {
        ESP_LOGE(TAG, "❌ CRITICAL: Failed to create Ballast Control Task");
        ESP_LOGE(TAG, "❌ Free heap: %lu bytes", esp_get_free_heap_size());
        ESP_LOGE(TAG, "❌ SYSTEM HALTED - Insufficient memory or task creation failure");
        return;
    }
    ESP_LOGI(TAG, "✅ Ballast Control Task created successfully on Core 1");
    
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
        ESP_LOGE(TAG, "❌ Free heap: %lu bytes", esp_get_free_heap_size());
        return;
    }
    ESP_LOGI(TAG, "✅ System Monitor Task created successfully on Core 0");
    
    // System startup complete
    ESP_LOGI(TAG, "=================================================");
    ESP_LOGI(TAG, "🚀 SYSTEM STARTUP COMPLETE");
    ESP_LOGI(TAG, "=================================================");
    ESP_LOGI(TAG, "Architecture Summary:");
    ESP_LOGI(TAG, "  📋 Core 0 (System):     System Monitor Task");
    ESP_LOGI(TAG, "  ⚙️  Core 1 (Application): Ballast Control Task");
    ESP_LOGI(TAG, "  🔄 Auto switching:      Every 10 seconds");
    ESP_LOGI(TAG, "  📊 ADC monitoring:      Every 1 second");
    ESP_LOGI(TAG, "  🔍 Status reporting:    Every 5 seconds");
    ESP_LOGI(TAG, "  🧵 Thread-safe design:  Mutex + Queue communication");
    ESP_LOGI(TAG, "=================================================");
    ESP_LOGI(TAG, "💡 Ready for operation - monitoring started");
    
    // Enable demo mode for testing (comment out for production)
    // demo_mode_enabled = true;
    
    // Main task heartbeat loop
    uint32_t heartbeat_counter = 0;
    while (1) {
        // Main task heartbeat
        vTaskDelay(pdMS_TO_TICKS(MAIN_HEARTBEAT_DELAY_MS));
        heartbeat_counter++;
        
        ESP_LOGI(TAG, "💓 Main task heartbeat #%lu - System running normally", heartbeat_counter);
        ESP_LOGI(TAG, "📈 Free heap: %"PRIu32" bytes, Uptime: %"PRIu32" seconds", 
                 esp_get_free_heap_size(), 
                 (xTaskGetTickCount() * portTICK_PERIOD_MS) / 1000);
        
        // Demonstrate manual control periodically (if demo mode enabled)
        if (demo_mode_enabled && (heartbeat_counter % 3 == 0)) {  // Every 3rd heartbeat
            demo_manual_control();
        }
        
        // TODO: Add main task responsibilities:
        // - Watchdog feeding
        // - System-wide error recovery
        // - Configuration updates from external sources
        // - OTA update handling
        // - Power management decisions
    }
}