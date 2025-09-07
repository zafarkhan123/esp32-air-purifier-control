# ESP32 Air Purifier UV Ballast Control System

## 📋 What is this Software?

This is a **modular UV ballast control system** for an air purifier built on the ESP32-WROOM-32UE microcontroller using ESP-IDF framework. The system controls two UV ballasts alternately with real-time ADC feedback monitoring for fault detection and system health reporting.

### Key Features:
- ✅ **Dual UV Ballast Control** - Alternates between Ballast A and B every 10 seconds
- ✅ **Real-time ADC Monitoring** - Voltage feedback every 1 second  
- ✅ **Fault Detection** - Detects ballast failures and unexpected states
- ✅ **Multi-core Task Architecture** - Optimized for ESP32 dual-core system
- ✅ **Thread-safe Communication** - Mutex-protected data sharing
- ✅ **Modular Design** - Easy to extend with fan, WiFi, touch controls

---

## 🔧 How Does It Work?

### System Architecture:

```
┌─────────────────┐    ┌─────────────────┐
│   CORE 0        │    │   CORE 1        │
│  (System)       │    │ (Application)   │
├─────────────────┤    ├─────────────────┤
│ System Monitor  │◄──►│ Ballast Control │
│ - Status Queue  │    │ - GPIO Control  │
│ - Fault Handler │    │ - ADC Reading   │
│ - Health Check  │    │ - State Machine │
└─────────────────┘    └─────────────────┘
         ▲                       ▲
         │                       │
    ┌────▼────┐             ┌────▼────┐
    │ Status  │             │ Ballast │
    │ Reports │             │Hardware │
    └─────────┘             └─────────┘
```

### Control Flow:
1. **Initialization** → GPIO + ADC setup → Task creation
2. **Ballast Control Task** (Core 1) → Controls relays + reads ADC
3. **System Monitor Task** (Core 0) → Processes status + handles faults
4. **Continuous Operation** → 10s switching + 1s monitoring

### Hardware Interface:
```
ESP32 GPIO22 ──→ Relay 1 ──→ UV Ballast A
ESP32 GPIO19 ──→ Relay 2 ──→ UV Ballast B
ESP32 GPIO34 ◄── ADC ◄────── Ballast A Feedback
ESP32 GPIO35 ◄── ADC ◄────── Ballast B Feedback
```

---

## 📖 Important Definitions

### Pin Definitions:
| Pin | Function | Direction | Description |
|-----|----------|-----------|-------------|
| GPIO22 | UV_BALLAST_A_CONTROL_PIN | Output | Relay control for Ballast A |
| GPIO19 | UV_BALLAST_B_CONTROL_PIN | Output | Relay control for Ballast B |
| GPIO34 | UV_BALLAST_A_FEEDBACK_PIN | Input (ADC) | Voltage feedback from Ballast A |
| GPIO35 | UV_BALLAST_B_FEEDBACK_PIN | Input (ADC) | Voltage feedback from Ballast B |

### Voltage Thresholds:
```c
#define BALLAST_OFF_MIN_VOLTAGE    (1800)  // 1.8V - Ballast confirmed OFF
#define BALLAST_OFF_MAX_VOLTAGE    (2200)  // 2.2V
#define BALLAST_ON_MIN_VOLTAGE     (2700)  // 2.7V - Ballast confirmed ON  
#define BALLAST_ON_MAX_VOLTAGE     (3100)  // 3.1V
```

### Task Configuration:
```c
#define BALLAST_TASK_CORE_ID       (1)     // Application Core
#define BALLAST_TASK_PRIORITY      (5)     // Medium-high priority
#define BALLAST_TASK_STACK_SIZE    (4096)  // 4KB stack
```

### Status Types:
- **BALLAST_STATUS_OFF_OK** - Ballast OFF and ADC confirms
- **BALLAST_STATUS_ON_OK** - Ballast ON and ADC confirms  
- **BALLAST_STATUS_FAULT_NO_FEEDBACK** - Should be ON but reads OFF
- **BALLAST_STATUS_FAULT_UNEXPECTED_ON** - Should be OFF but reads ON

---

## 🔄 How Subroutines are Called

### Initialization Sequence:
```c
app_main()
├── BallastControlInit()          // Initialize module
│   ├── ballast_gpio_init()       // Setup GPIO pins
│   ├── ballast_adc_init()        // Setup ADC channels
│   └── Create mutex & queue      // Thread synchronization
├── BallastControlTaskCreate()    // Create control task
└── system_monitor_task()         // Create monitor task
```

### Runtime Call Flow:
```
ballast_control_task() [Core 1]        system_monitor_task() [Core 0]
├─ Every 100ms:                        ├─ Every 500ms:
│  ├─ update_ballast_feedback()        │  ├─ handle_ballast_status_updates()
│  │  ├─ read_ballast_voltage()        │  │  └─ xQueueReceive() ← Status data
│  │  ├─ analyze_ballast_status()      │  └─ print_system_status()
│  │  └─ send_status_update()          └─ Every 5s: System health check
│  └─ Every 10s: switch_ballast_state()
```

### API Call Examples:
```c
// Get current status
ballast_feedback_t feedback;
bool success = BallastControlGetFeedback(UV_BALLAST_A, &feedback);

// Manual control
BallastControlSetState(UV_BALLAST_A, true);  // Turn ON Ballast A

// Enable/disable auto mode  
BallastControlSetAutoMode(false);  // Stop automatic switching
```

---

## 🔒 Why Mutex is Used?

### Thread Safety Problem:
```
Core 1 (Ballast Task)     Core 0 (Monitor Task)
├─ Writing feedback ──────┼──── Reading feedback ← RACE CONDITION!
└─ Updating voltage       └──── Displaying status
```

### Mutex Solution:
```c
// Writer (Core 1) - Ballast Control Task
if (xSemaphoreTake(feedback_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    ballast_feedback[ballast].voltage_mv = voltage;      // ← Protected write
    ballast_feedback[ballast].status = new_status;      // ← Protected write
    xSemaphoreGive(feedback_mutex);
}

// Reader (Core 0) - System Monitor Task  
if (xSemaphoreTake(feedback_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    *feedback = ballast_feedback[ballast];               // ← Protected read
    xSemaphoreGive(feedback_mutex);
}
```

### Why It's Critical:
- **Data Corruption Prevention** - Ensures atomic read/write operations
- **Consistent State** - Prevents reading partially updated data
- **ESP32 Dual-Core Safety** - Both cores can access data safely
- **100ms Timeout** - Prevents deadlocks if mutex holder crashes

---

## 🧪 How to Test the System

### 1. Build and Flash:
```bash
# Set target for ESP32
idf.py set-target esp32

# Build the project
idf.py build

# Flash to ESP32 on COM6
idf.py -p COM6 flash

# Monitor serial output
idf.py -p COM6 monitor
```

### 2. Expected Normal Output:
```
I (xxx) MAIN: AIR PURIFIER CONTROL SYSTEM STARTING
I (xxx) BALLAST_CTRL: UV ballast GPIO pins initialized successfully
I (xxx) BALLAST_CTRL: ADC initialized successfully  
I (xxx) BALLAST_CTRL: Ballast control task started on Core 1
I (xxx) MAIN: System monitor task started on Core 0
I (xxx) BALLAST_CTRL: UV Ballast A: ON
I (xxx) BALLAST_CTRL: UV Ballast B: OFF
I (xxx) MAIN: Ballast A Status: ON_OK, Voltage: 2850 mV
I (xxx) MAIN: Ballast B Status: OFF_OK, Voltage: 1950 mV

// After 10 seconds:
I (xxx) BALLAST_CTRL: Auto switching ballast states...
I (xxx) BALLAST_CTRL: UV Ballast A: OFF  
I (xxx) BALLAST_CTRL: UV Ballast B: ON
```

### 3. Hardware Testing:

#### Test 1: GPIO Control
```bash
# Connect LED to GPIO22 and GPIO19
# You should see LEDs alternating every 10 seconds
```

#### Test 2: ADC Feedback  
```bash
# Connect voltage divider to GPIO34 and GPIO35:
# 3.3V ──[10kΩ]──┬──[GPIO34]
#                 └──[10kΩ]──GND
# Expected voltage: ~1.65V (should show BALLAST_STATUS_UNKNOWN)
```

#### Test 3: Manual Control
```c
// Uncomment in main.c:
BallastControlSetAutoMode(false);               // Stop auto switching
BallastControlSetState(UV_BALLAST_A, true);     // Manual ON
vTaskDelay(pdMS_TO_TICKS(5000));                // Wait 5 seconds
BallastControlSetState(UV_BALLAST_A, false);    // Manual OFF
BallastControlSetAutoMode(true);                // Resume auto mode
```

### 4. Fault Injection Testing:

#### Test 4: Simulate Ballast Failure
```bash
# Disconnect ADC feedback while ballast should be ON
# Expected: "BALLAST A FAULT: No feedback - ballast may be disconnected"
```

#### Test 5: Simulate Stuck Relay
```bash
# Apply 3V to ADC pin while ballast should be OFF  
# Expected: "BALLAST A FAULT: Unexpected ON state - relay may be stuck"
```

---

## ⚠️ Important Failure Points & Remedies

### 1. **Task Creation Failures**

**Symptoms:**
```
E (xxx) MAIN: ❌ Failed to create Ballast Control Task. System halted.
```

**Possible Causes:**
- Insufficient heap memory
- Stack size too large
- Invalid core ID

**Remedies:**
```c
// Reduce stack size
#define BALLAST_TASK_STACK_SIZE    (3072)  // Instead of 4096

// Check free heap
ESP_LOGI(TAG, "Free heap before task creation: %lu", esp_get_free_heap_size());

// Use default core (don't pin to specific core)
xTaskCreate() instead of xTaskCreatePinnedToCore()
```

### 2. **GPIO Configuration Failures**

**Symptoms:**
```
E (xxx) BALLAST_CTRL: Failed to configure GPIO pins: Invalid argument
```

**Possible Causes:**
- Invalid pin numbers
- Pin already in use
- Incorrect pin mask

**Remedies:**
```c
// Verify pin assignments
ESP_LOGI(TAG, "Configuring GPIO %d and %d", UV_BALLAST_A_CONTROL_PIN, UV_BALLAST_B_CONTROL_PIN);

// Check pin availability in ESP32 documentation
// GPIO 6-11 are connected to flash - avoid using them
```

### 3. **ADC Initialization Failures**

**Symptoms:**
```
E (xxx) BALLAST_CTRL: Failed to configure ADC1_CHANNEL_6: Invalid state
```

**Possible Causes:**
- ADC already configured by another component
- Invalid channel for pin
- Wrong ADC unit

**Remedies:**
```c
// Check ADC channel mapping:
// GPIO34 = ADC1_CHANNEL_6 ✅
// GPIO35 = ADC1_CHANNEL_7 ✅

// Verify ADC unit:
adc1_config_width(ADC_WIDTH_BIT_12);  // Use ADC1, not ADC2
```

### 4. **Mutex/Queue Failures**

**Symptoms:**
```
E (xxx) BALLAST_CTRL: Failed to create feedback mutex
W (xxx) BALLAST_CTRL: Failed to take mutex within timeout
```

**Possible Causes:**
- Insufficient heap memory
- Deadlock condition
- Task holding mutex crashed

**Remedies:**
```c
// Check heap before creating mutex
if (esp_get_free_heap_size() < 8192) {
    ESP_LOGW(TAG, "Low heap memory: %lu bytes", esp_get_free_heap_size());
}

// Implement mutex timeout handling
if (xSemaphoreTake(feedback_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
    ESP_LOGW(TAG, "Mutex timeout - using cached data");
    return cached_feedback;
}
```

### 5. **ADC Reading Anomalies**

**Symptoms:**
```
W (xxx) BALLAST_CTRL: Ballast A FAULT: FAULT_NO_FEEDBACK (Voltage: 0 mV)
W (xxx) BALLAST_CTRL: Ballast B readings inconsistent: 4095, 0, 2048, 0
```

**Possible Causes:**
- Loose connections
- ADC noise
- Power supply issues
- Incorrect voltage divider

**Remedies:**
```c
// Increase sampling
#define ADC_SAMPLES    20    // Instead of 10

// Add ADC calibration check
esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF);

// Add voltage range validation
if (voltage > 3300) {  // Should not exceed 3.3V
    ESP_LOGW(TAG, "ADC voltage out of range: %lu mV", voltage);
    return 0;  // Invalid reading
}
```

### 6. **System Performance Issues**

**Symptoms:**
```
W (xxx) MAIN: Task watchdog timeout - system_monitor_task
W (xxx) MAIN: High CPU usage detected
```

**Possible Causes:**
- Task delays too short
- Infinite loops
- High interrupt load

**Remedies:**
```c
// Increase task delays
vTaskDelay(pdMS_TO_TICKS(200));  // Instead of 100ms

// Monitor task CPU usage
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    ESP_LOGE(TAG, "Stack overflow in task: %s", pcTaskName);
}

// Enable watchdog monitoring
esp_task_wdt_add(NULL);  // Add current task to watchdog
```

---

## 🏗️ Project Structure

```
project/
├── CMakeLists.txt                 # Project level CMake (no changes needed)
├── main/
│   ├── CMakeLists.txt            # Updated component CMake
│   ├── main.c                    # Main application
│   ├── drivers/                  # ← NEW FOLDER
│   │   ├── ballastControl.h      # Ballast driver header
│   │   └── ballastControl.c      # Ballast driver implementation
│   └── README.md                 # This documentation
└── build/                        # Generated build files
```

### No CMakeLists.txt needed in drivers/ folder
The drivers folder is part of the main component, so it doesn't need its own CMakeLists.txt file.

---

## 🚀 Quick Start

1. **Clone/Copy** files to your ESP-IDF project
2. **Build:** `idf.py build`
3. **Flash:** `idf.py -p COM6 flash monitor`
4. **Observe:** Ballasts alternating every 10 seconds
5. **Test:** Try manual control examples in code

---

## 📞 Support & Extension

This modular design allows easy extension:
- **Add Fan Control:** Create `drivers/fanControl.c`
- **Add WiFi Status:** Include WiFi reporting in system monitor
- **Add Touch Interface:** Create touch event handlers
- **Add Display:** Integrate with LCD/OLED for status display

For issues or extensions, check the failure points section above first!

* =============================================================================
 * BUILD ENVIRONMENT & TOOLCHAIN INFORMATION
 * =============================================================================
 * ESP-IDF Version:    v5.4.1 (tested and compatible)
 * Target Platform:    ESP32-WROOM-32UE
 * Build Target:       esp32
 * Flash Size:         4MB
 * Minimum IDF:        v5.0 or later
 * 
 * =============================================================================
 * PARTITION TABLE LAYOUT (4MB Flash)
 * =============================================================================
 * Partition Name    | Type     | SubType  | Offset   | Size     | Purpose
 * ------------------|----------|----------|----------|----------|------------
 * nvs               | data     | nvs      | 0x9000   | 24KB     | Non-volatile storage
 * phy_init          | data     | phy      | 0xF000   | 4KB      | RF calibration data
 * factory           | app      | factory  | 0x10000  | 1MB      | Main application
 * storage           | data     | fat      | 0x110000 | ~3MB     | File system (optional)
 * 
 * Note: For larger applications requiring >1MB, modify partition table in:
 * - partitions.csv (create custom partition table)
 * - sdkconfig (CONFIG_PARTITION_TABLE_CUSTOM=y)
 * 
 * =============================================================================
 * BUILD COMMANDS
 * =============================================================================
 * Standard Build Process:
 *   idf.py set-target esp32
 *   idf.py menuconfig          # Optional: configure project
 *   idf.py build
 *   idf.py -p COM6 flash monitor
 * 
 * Clean Build (recommended after major changes):
 *   idf.py fullclean
 *   idf.py build
 *   idf.py -p COM6 flash monitor
 * 
 * =============================================================================
 * ADVANCED ESPTOOL COMMANDS FOR LARGE APPLICATIONS
 * =============================================================================
 * 
 * Complete Flash Erase + Program (Production Ready):
 *   esptool.py --chip esp32 --port COM6 --baud 921600 erase_flash
 *   esptool.py --chip esp32 --port COM6 --baud 921600 write_flash \
 *     --flash_mode dio --flash_freq 40m --flash_size 4MB \
 *     0x1000 build/bootloader/bootloader.bin \
 *     0x8000 build/partition_table/partition-table.bin \
 *     0x10000 build/app.bin
 * 
 * Fast Development Flash (App Only - Preserves NVS):
 *   esptool.py --chip esp32 --port COM6 --baud 921600 write_flash \
 *     0x10000 build/app.bin
 * 
 * Custom Partition Flash (For Modified Partition Table):
 *   esptool.py --chip esp32 --port COM6 --baud 921600 write_flash \
 *     --flash_mode dio --flash_freq 40m --flash_size 4MB \
 *     0x1000 build/bootloader/bootloader.bin \
 *     0x8000 build/partition_table/partition-table.bin \
 *     0x10000 build/app.bin \
 *     0x110000 build/storage.bin
 * 
 * Verify Flash Contents:
 *   esptool.py --chip esp32 --port COM6 verify_flash \
 *     0x10000 build/app.bin
 * 
 * =============================================================================
 * LARGE APPLICATION CONSIDERATIONS
 * =============================================================================
 * For applications approaching 1MB+ size:
 * 
 * 1. Partition Table Modification:
 *    - Create custom partitions.csv with larger app partition
 *    - Enable CONFIG_PARTITION_TABLE_CUSTOM in menuconfig
 *    - Consider OTA partitions for firmware updates
 * 
 * 2. Memory Optimization:
 *    - Enable CONFIG_COMPILER_OPTIMIZATION_SIZE
 *    - Use CONFIG_SPIRAM for external PSRAM if available
 *    - Optimize task stack sizes in each module
 * 
 * 3. Build Optimization Flags:
 *    - CMAKE_BUILD_TYPE=MinSizeRel for smallest binary
 *    - Enable LTO (Link Time Optimization) in advanced settings
 *    - Remove unused components in sdkconfig
 * 
 * 4. Flash Configuration:
 *    - Use DIO mode for maximum compatibility
 *    - 40MHz frequency for stable operation
 *    - Consider QIO mode for faster flash access (if supported)
 * 
 * 5. OTA (Over-The-Air) Updates Setup:
 *    Example partition table for OTA:
 *    factory,app,factory,0x10000,1M
 *    ota_0,app,ota_0,0x110000,1M
 *    ota_1,app,ota_1,0x210000,1M
 *    ota_data,data,ota,0x310000,8K
 * 
 * =============================================================================
 * DEBUGGING AND MONITORING
 * =============================================================================
 * Serial Monitor:
 *   idf.py -p COM6 monitor
 *   Exit: Ctrl+] (Windows) or Ctrl+T, Ctrl+X (Linux/Mac)
 * 
 * GDB Debugging:
 *   idf.py -p COM6 openocd-gdb
 * 
 * Core Dump Analysis:
 *   idf.py -p COM6 coredump-info
 * 
 * Performance Monitoring:
 *   Enable CONFIG_FREERTOS_USE_TRACE_FACILITY
 *   Use vTaskGetRunTimeStats() for task analysis
 * 
 * =============================================================================
 */
