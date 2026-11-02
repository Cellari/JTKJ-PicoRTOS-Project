#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <tkjhat/sdk.h> //added

#include <pico/stdlib.h>
#include <pico/stdio.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h" //added

#include <tusb.h>
#include "usbSerialDebug/helper.h"

#define BUFFER_SIZE     30
#define TEMP_MIN        0
#define TEMP_MAX        40
#define LUX_MIN         100
#define LUX_MAX         1500
#define CDC_ITF_TX      1
#define MORSE_QUEUE_LENGTH 128
#define USER_INPUT_DELAY 400
#define SENSOR_DELAY    50
static QueueHandle_t morseTxQueue = NULL;  


/*#if   CFG_TUSB_OS == OPT_OS_FREERTOS
#pragma message "TinyUSB: FreeRTOS"
#elif CFG_TUSB_OS == OPT_OS_PICO
#pragma message("TinyUSB: OS PICO")
#elif CFG_TUSB_OS == OPT_OS_NONE
#pragma message("TinyUSB: sin RTOS")
#endif*/

#if CFG_TUSB_OS != OPT_OS_FREERTOS
#error "This should be using FREERTOS but the CFG_TUSB_OS is not OPT_OS_FREERTOS"
#endif

static int rand_in_range(int min, int max) {
    return min + rand() % (max - min + 1);
}

// ---- Task generating sensor data----
static void sendTask(void *arg)
{
    (void)arg;
    char ch;

    // Wait until the USB device is mounted.
    while (!tud_mounted()) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    while (1) {
        // Wait until a Morse character is available.
        if (xQueueReceive(morseTxQueue, &ch, portMAX_DELAY) == pdTRUE) {

            // Wait until CDC1 is connected.
            while (!tud_cdc_n_connected(CDC_ITF_TX)) {
                vTaskDelay(pdMS_TO_TICKS(50));
            }

            // Send one character to the workstation.
            tud_cdc_n_write(CDC_ITF_TX, &ch, 1);
            tud_cdc_n_write_flush(CDC_ITF_TX);

            // Use buzzer to indicate a succesful morse signal
            buzzer_play_tone(110, 50);
        }
    }
}

static void testMorseTask(void *arg)
{
    (void)arg;

    const char *message = "... --- ...\n";

    while (1) {
        // Put each character into the queue for sendTask.
        for (const char *p = message; *p != '\0'; p++) {
            xQueueSend(morseTxQueue, p, portMAX_DELAY);
        }

        // Wait before sending the test message again.
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

// ---- Task running USB stack ----
static void usbTask(void *arg) {
    (void)arg;
    while (1) {
        tud_task();              // With FreeRTOS wait for events
                                 // Do not add vTaskDelay. 
    }
}

/// @brief A shorthand function to implement a repeated value range comparison
/// @param value Pointer to the value to be evaluated
/// @param low The low non-reachable boundary for the value
/// @param high The high non-reachable boundary for the value
/// @return true (1) if the value is between the boundary values, otherwise false (0)
static inline int inBetween(const float *value, float low, float high) {
    if (*value > low && *value < high) return 1;
    else return 0;
}

/// @brief Listens to the IMU-sensor values to add morse code to the queue
/// @param pvParameters Unused
static void imu_task(void *pvParameters) {
    (void)pvParameters;

    float ax, ay, az, gx, gy, gz, t;
    // Setting up the sensor. 
    if (init_ICM42670() == 0) {
        usb_serial_print("ICM-42670P initialized successfully!\n");
        if (ICM42670_start_with_default_values() != 0){
            usb_serial_print("ICM-42670P could not initialize accelerometer or gyroscope");
        }
    } else {
        usb_serial_print("Failed to initialize ICM-42670P.\n");
    }
    
    // Start device orientation decodings for sending morse code. Infinite loop. 
    uint8_t buf[BUFFER_SIZE];
    while (1)
    {
        char c = 0; // Local variable, sent to queue via address
        if (ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &t) == 0) {
            if (inBetween(&ax, -0.3, 0.3) && inBetween(&ay, -0.3, 0.3) && !inBetween(&az, -0.7, 0.7)) {
                sprintf(buf, "Flat\n");
                c = '-';
            } else if (inBetween(&ax, -0.3, 0.3) && !inBetween(&ay, -0.7, 0.7) && inBetween(&az, -0.3, 0.3)) {
                sprintf(buf, "Stand\n");
                c = '.';
            }

            // Send results
            if (c == 0)
                sprintf(buf,"Accel: X=%.2f, Y=%.2f, Z=%.2f | Gyro: X=%.2f, Y=%.2f, Z=%.2f| Temp: %2.2f°C\n", ax, ay, az, gx, gy, gz, t);
            else
                xQueueSend(morseTxQueue, &c, portMAX_DELAY);
            usb_serial_print(buf);
            usb_serial_flush();
        } else {
            usb_serial_print("Failed to read imu data\n");
        }

        // Adjust software for faster reactivity and user lag
        if(c == 0)
            vTaskDelay(pdMS_TO_TICKS(SENSOR_DELAY));
        else
            vTaskDelay(pdMS_TO_TICKS(USER_INPUT_DELAY));
    }
}

static void btn_fxn(uint gpio, uint32_t eventMask) {
    char c = ' ';
    xQueueSend(morseTxQueue, &c, portMAX_DELAY);
}

int main (void) {
    //Generating random seed
    srand((unsigned)time(NULL));
    // Initialize the JTKJ Hat and OLED display.
    init_hat_sdk();
    init_display();
    clear_display();
    write_text("Morse Ready!");

    // Set the control button
    init_sw1();
    gpio_set_irq_enabled_with_callback(SW1_PIN, GPIO_IRQ_EDGE_FALL, true, btn_fxn);

    // Initialize Buzzer
    init_buzzer();

    // Queue for outgoing Morse characters.
    morseTxQueue = xQueueCreate(MORSE_QUEUE_LENGTH, sizeof(char));

    if (morseTxQueue == NULL) {
        while (1) {
            // Stop if the queue could not be created.
        }
    }
    
    // Create tasks
    TaskHandle_t hUsb   = NULL;
    xTaskCreate(usbTask, "usb", 1024, NULL, 3, &hUsb);
    xTaskCreate(sendTask, "send", 1024, NULL, 2, NULL);
    // xTaskCreate(testMorseTask, "test", 1024, NULL, 1, NULL);
    xTaskCreate(imu_task, "imu", 1024, NULL, 1, NULL); // listenTask

    #if (configNUMBER_OF_CORES > 1)
        vTaskCoreAffinitySet(hUsb, 1u << 0);
    #endif
    
    // VERY IMPORTANT, THIS SHOULD GO JUST BEFORE vTaskStartSheduler
    // WITHOUT ANY DELAYS. OTHERWISE, THE TinyUSB stack wont recognize
    // the device.
    // Initialize TinyUSB 
    tusb_init();
    //Initialize helper library to write in CDC0)
    usb_serial_init();
    vTaskStartScheduler();
    return 0;
}

// Called by the USB task when a CDC interface receives data.
void tud_cdc_rx_cb(uint8_t itf) {
    // Reserve two extra bytes for the newline and string terminator.
    char buf[CFG_TUD_CDC_RX_BUFSIZE + 2];
    uint32_t count = tud_cdc_n_read(itf, buf, sizeof(buf) - 2);
    buf[count] = '\n';
    buf[count + 1] = '\0';

    // Also read CDC0 so that its receive buffer does not fill up.
    if (itf == 1 && count > 0) {
        if (tud_cdc_n_connected(0)) {
            // Nonblocking writes: do not use the waiting logger here.
            // Meaning, do not use usb_serial_print() or usb_serial_flush() here.
            const char *prefix = "\nReceived on CDC 1:";
            tud_cdc_n_write(0, prefix, strlen(prefix));
            // Include the appended newline, but not the null terminator.
            tud_cdc_n_write(0, buf, count + 1);
            tud_cdc_n_write_flush(0);
        }

        if (tud_cdc_n_connected(1)) {
            tud_cdc_n_write(1, "OK\n", 3);
            tud_cdc_n_write_flush(1);
        }
    }
}
