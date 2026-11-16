#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <tkjhat/sdk.h>

#include <pico/stdlib.h>
#include <pico/stdio.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "pico/cyw43_arch.h"
#define AP_SSID "panoulu"
#define AP_PSWD NULL
#define AP_AUTH_TYPE CYW43_AUTH_OPEN

#define MQTT_IP "server.server.com"
#define MQTT_PORT 8000
#define MQTT_USERNAME "user"
#define MQTT_PASSWORD "supersecretpassword"

#include <tusb.h>
#include "usbSerialDebug/helper.h"

#include "morse_utils.h"
#include "utils.h"

#define LOG_BUFFER_SIZE 128
#define TEMP_MIN 0
#define TEMP_MAX 40
#define LUX_MIN 100
#define LUX_MAX 1500

// ports
#define CDC_ITF_DATA 1
#define CDC_ITF_DEBUG 0

// size of serial input buffer

#define RX_QUEUE_LENGTH 128
#define TX_QUEUE_LENGTH 128
#define USER_INPUT_DELAY 200
#define SENSOR_DELAY 40
#define SCAN_DELAY 1000

#define MAX_OLED_WIDTH 21
#define MAX_OLED_HEIGHT 7     // given that 1 line ~ 9px in height
#define MAX_OLED_CHAR_NUM 147 // 21 * 7
#define CHAR_OLED_WIDTH 6
#define CHAR_OLED_HEIGHT 7

#define MAX_RX_LENGTH 256

#define MOTION_EDGE 80

// standard task stacksize
#define STACKSIZE 1024

// see state machine diagram for description of each state
typedef enum
{
    IDLE,
    RECEIVE,
    UPDATE_LCD,
    SEND
} programState;
// variable holding program state
programState currentState;

// initializes task handles
TaskHandle_t ReadSerialTaskHandle = NULL;
TaskHandle_t HandleLcdTaskHandle = NULL;
TaskHandle_t SendTaskHandle = NULL;
TaskHandle_t UsbTaskHandle = NULL;
TaskHandle_t ListenImuTaskHandle = NULL;
TaskHandle_t WirelessHandle = NULL;

// initializes semaphores
SemaphoreHandle_t serialSemaphore = NULL;
SemaphoreHandle_t btnSemaphore = NULL;

QueueHandle_t morseRxQueue = NULL;
QueueHandle_t morseTxQueue = NULL;

/**
 * @brief Runs the TinyUSB device service continuously.
 *
 * TinyUSB requires tud_task() to be called regularly so that USB events,
 * connection changes and CDC communication callbacks can be processed.
 *
 * This task runs independently of the application tasks, allowing USB
 * communication events to be handled while other operations are running.
 */

void usbTask(void *arg)
{
    (void)arg;
    while (true)
    {
        tud_task();
    }
}

/**
 * @brief TinyUSB callback triggered when data arrives through a CDC interface.
 *
 * The callback checks that the incoming data belongs to the communication
 * interface rather than the debugging interface.
 *
 * When the device is not transmitting, the serial semaphore is released.
 * This wakes ReadSerialTask, which then reads the available USB data.
 *
 * @param itf CDC interface that received the data.
 */
void tud_cdc_rx_cb(uint8_t itf)
{
    // condition ensures program only processes data from CDC1
    // incoming data is ignored while the device is on SEND mode
    if (itf == CDC_ITF_DATA && currentState != SEND)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        // gives serialSemaphore required by ReadSerialTask
        xSemaphoreGiveFromISR(serialSemaphore, &xHigherPriorityTaskWoken);
        // no portYIELD_FROM_ISR() since all tasks have equal priority, no preemption needed
    }
}

/// @brief Use buzzer to indicate a succesful morse signal
/// @param ch Accepts either '.', '-', ' ' or '\n'
void char_to_buzz(char ch)
{
    switch (ch)
    {
    case '.':
        buzzer_play_tone(110, 50);
        break;
    case '-':
        buzzer_play_tone(330, 50);
        break;
    case ' ':
        buzzer_play_tone(220, 150);
        break;
    case '\n':
        buzzer_play_tone(440, 200);
        break;
    default:
        break;
    }
}

/// @brief Function used for the button interrupt handler
/// @param gpio Unused
/// @param events Unused
void btnFxn(uint gpio, uint32_t events)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    // if program is in IDLE, button press switches state to SEND
    if (currentState == IDLE)
    {
        // switch the program state to SEND
        currentState = SEND;
        // indicates that there's no need to switch to another task right away
        // gives btnSemaphore for send_task
        xSemaphoreGiveFromISR(btnSemaphore, &xHigherPriorityTaskWoken);
    } // if program is in SEND, button inputs spaces
    else if (currentState == SEND)
    {
        char c = ' ';
        xQueueSendFromISR(morseTxQueue, &c, &xHigherPriorityTaskWoken);
    }
}

/**
 * @brief Reads incoming USB serial data and places it in the receive queue.
 *
 * This task remains blocked until serialSemaphore is released by the
 * TinyUSB receive callback.
 *
 * Once notified, it:
 * 1. Changes the program state to RECEIVE.
 * 2. Reads available characters from the USB CDC communication interface.
 * 3. Transfers each received character into morseRxQueue.
 *
 * The queue separates USB data acquisition from message processing and
 * display updates, allowing the two operations to run independently.
 */

void ReadSerialTask(void *pvParameters)
{
    char buf[CFG_TUD_CDC_RX_BUFSIZE + 2];

    while (true)
    {
        // Task is blocked indefinitely until serialSemaphore is given
        if (xSemaphoreTake(serialSemaphore, portMAX_DELAY) == pdTRUE)
        {
            // state switched to RECEIVE while reading serial data
            currentState = RECEIVE;

            // logging
            char log_buf[LOG_BUFFER_SIZE];
            snprintf(log_buf, sizeof(log_buf), "[ReadSerialTask] State = %d; Task activated, reading usb buffer...\n\n", currentState);
            usb_serial_print(log_buf);
            usb_serial_flush();
            // logging

            // drains the usb buffer (cdc 1)
            uint32_t count = tud_cdc_n_read(CDC_ITF_DATA, buf, sizeof(buf) - 2);
            // puts read characters in queue
            if (count > 0)
            {
                for (uint32_t i = 0; i < count; i++)
                {
                    xQueueSend(morseRxQueue, &buf[i], portMAX_DELAY);
                }
            }

            // logging
            snprintf(log_buf, sizeof(log_buf), "[ReadSerialTask] State = %d; Usb buffer drained\n\n", currentState);
            usb_serial_print(log_buf);
            usb_serial_flush();
            // logging
        }
    }
}

/**
 * @brief Displays a null-terminated string while managing OLED screen limits.
 *
 * The function converts character positions into pixel coordinates and
 * displays the supplied text one character at a time.
 *
 * When the current line is full, the cursor moves to the next line.
 * When the entire display is full, the screen is cleared and the cursor
 * returns to the top-left corner.
 *
 * The cursor coordinates are passed by reference so their updated positions
 * are retained by the calling function.
 *
 * @param x_ind Pointer to the current horizontal character position.
 * @param y_ind Pointer to the current vertical character position.
 * @param txt Null-terminated string to display.
 */

void smart_display_text(uint32_t *x_ind, uint32_t *y_ind, char *txt)
{
    for (uint32_t i = 0; txt[i] != '\0'; i++)
    {

        char str[2] = {txt[i], '\0'};
        write_text_xy(*x_ind * CHAR_OLED_WIDTH, *y_ind * CHAR_OLED_HEIGHT, str);
        *x_ind += 1;

        // screen line limit reached, switches to next line
        if (*x_ind >= MAX_OLED_WIDTH)
        {
            *x_ind = 0;
            *y_ind += 1;
        }

        // screen reached capacity limit, clear the screen
        if (*y_ind >= MAX_OLED_HEIGHT)
        {
            *x_ind = 0;
            *y_ind = 0;
            clear_display();
        }
    }
}

// is activated when characters are added to RTOS Queue

/**
 * @brief Processes received Morse characters and displays the result.
 *
 * This task is responsible for the main receiving workflow after USB data
 * has been placed in morseRxQueue.
 *
 * It processes incoming characters individually and displays the Morse
 * sequence on the OLED while providing buzzer feedback.
 *
 * A newline character marks the end of a message. At that point, the complete
 * Morse sequence is passed to morse_to_text(), which converts it into
 * readable text. The translated message is then displayed on the OLED.
 */
void HandleLcdTask(void *pvParameters)
{
    char queue_char;

    char morse_msg[MAX_RX_LENGTH];
    char msg_translation[MAX_RX_LENGTH];

    uint32_t ind = 0;
    uint32_t x_ind = 0;
    uint32_t y_ind = 0;

    while (true)
    {
        // stays blocked until queue is updated
        // condition is satisfied as long as the queue is not empty
        if (xQueueReceive(morseRxQueue, &queue_char, portMAX_DELAY) == pdTRUE)
        {
            // updates the state to UPDATE_LCD while working with display
            currentState = UPDATE_LCD;

            // Ignore carriage return. (we need it?)
            if (queue_char == '\r')
            {
                // currentState = IDLE;
                continue;
            }

            // reached EOM
            // clears display, displays translation, switches state to IDLE
            if (queue_char == '\n')
            {
                char for_display[2] = {queue_char, '\0'};
                smart_display_text(&x_ind, &y_ind, for_display);

                char_to_buzz('\n');
                
                morse_msg[ind] = '\0';

                // logging
                char log_buf[LOG_BUFFER_SIZE];
                snprintf(log_buf, sizeof(log_buf),
                         "[HandleLcdTask] State = %d; EOM reached, clearing display, initiating translation; Morse message: %s\n\n",
                         currentState, morse_msg);
                usb_serial_print(log_buf);
                usb_serial_flush();
                // logging

                clear_display();
                ind = 0;
                x_ind = 0;
                y_ind = 0;

                // translates morse message, puts result in msg_translation
                morse_to_text(morse_msg, msg_translation, MAX_RX_LENGTH);
                smart_display_text(&x_ind, &y_ind, msg_translation);

                // resets the msg buffer
                memset(morse_msg, 0, sizeof(morse_msg));

                currentState = IDLE;

                // logging
                snprintf(log_buf, sizeof(log_buf), "[HandleLcdTask] State = %d; Finished displaying translation: %s\n\n",
                         currentState, msg_translation);
                usb_serial_print(log_buf);
                usb_serial_flush();
                // logging
            }

            else
            {
                morse_msg[ind] = queue_char;
                // write_text_xy function requires a null-terminated str as a parameter

                // if this is a new message, clear display first
                if (ind == 0)
                    clear_display();

                char for_display[2] = {queue_char, '\0'};
                smart_display_text(&x_ind, &y_ind, for_display);

                char_to_buzz(queue_char);

                ind += 1;
            }
        }
    }
}
/**
 * @brief Transmits generated Morse symbols through USB CDC and MQTT.
 *
 * This task consumes characters from morseTxQueue and sends them to the
 * connected computer through the USB communication interface.
 *
 * The task waits for USB to be mounted before beginning normal operation.
 * During transmission, it checks that the CDC interface is connected before
 * sending each character.
 *
 * Three consecutive spaces are interpreted as the end of a message.
 * They are replaced with a newline character, which acts as the message
 * terminator on the receiving side.
 *
 * Each transmitted symbol is also sent to the buzzer for audio feedback.
 */
void SendTask(void *pvParameters)
{
    char queue_char;
    uint8_t conseq_spaces = 0;

    char morse_buf[MAX_RX_LENGTH];
    uint16_t ind = 0;

    while (!tud_mounted())
    {
        // logging
        char log_buf[LOG_BUFFER_SIZE];
        snprintf(log_buf, sizeof(log_buf), "[SendTask] State = %d; tud not mounted\n\n", currentState);
        usb_serial_print(log_buf);
        usb_serial_flush();
        // logging

        vTaskDelay(pdMS_TO_TICKS(50));
    }

    while (true)
    {
        if (xQueueReceive(morseTxQueue, &queue_char, portMAX_DELAY) == pdTRUE)
        {
            if (currentState == SEND)
            {
                while (!tud_cdc_n_connected(CDC_ITF_DATA))
                {
                    // logging
                    char log_buf[LOG_BUFFER_SIZE];
                    snprintf(log_buf, sizeof(log_buf), "[SendTask] State = %d; tud cdc 1 not connected\n\n", currentState);
                    usb_serial_print(log_buf);
                    usb_serial_flush();
                    // logging
                    vTaskDelay(pdMS_TO_TICKS(50));
                }

                // checks if 3 space were received consequtively (EOM condition)
                if (queue_char == ' ')
                {
                    if (conseq_spaces == 2)
                    {
                        queue_char = '\n';
                        conseq_spaces = 0;
                        currentState = IDLE;

                        morse_buf[ind] = '\0';

                        // logging
                        char log_buf[LOG_BUFFER_SIZE];
                        // log last character
                        snprintf(log_buf, sizeof(log_buf),
                                 "[SendTask] State = %d; Symbol sent: \"\\n\"\n\n", currentState, queue_char);
                        usb_serial_print(log_buf);
                        usb_serial_flush();
                        // log the entire message
                        snprintf(log_buf, sizeof(log_buf), "[SendTask] State = %d; EOM generated; Morse generated: \"%s\\n\"\n\n", currentState, morse_buf);
                        usb_serial_print(log_buf);
                        usb_serial_flush();
                        // logging

                        // send \n over serial
                        tud_cdc_n_write(CDC_ITF_DATA, &queue_char, 1);
                        tud_cdc_n_write_flush(CDC_ITF_DATA);

                        char_to_buzz('\n');

                        // reset buffer (every value is 0)
                        memset(morse_buf, 0, sizeof(morse_buf));
                        // reset buffer index
                        ind = 0;

                        // skip the rest of the loop code
                        continue;
                    }
                    else
                        conseq_spaces += 1;
                }
                else
                    conseq_spaces = 0;

                morse_buf[ind] = queue_char;
                ind += 1;

                // USB
                tud_cdc_n_write(CDC_ITF_DATA, &queue_char, 1);
                tud_cdc_n_write_flush(CDC_ITF_DATA);

                // MQTT
                // if(server_connected))
                //     wireless_send_message(_msg, strlen(_msg)}

                // logging
                char log_buf[LOG_BUFFER_SIZE];
                snprintf(log_buf, sizeof(log_buf),
                         "[SendTask] State = %d; Symbol sent: \"%c\"\n\n", currentState, queue_char);
                usb_serial_print(log_buf);
                usb_serial_flush();
                // logging

                char_to_buzz(queue_char);
            }
        }
    }
}

/// @brief Listens to the IMU-sensor values to add morse code to the queue via device movement
/// @param pvParameters Unused
static void listenMovementTask(void *pvParameters)
{
    (void)pvParameters;

    float ax, ay, az, gx, gy, gz, t;

    // Start device movement decodings for sending morse code. Infinite loop.
    char buf[LOG_BUFFER_SIZE];
    char prev_c;
    while (true)
    {
        if (xSemaphoreTake(btnSemaphore, portMAX_DELAY) == pdTRUE)
        {
            clear_display();
            write_text("MODE: SEND");

            prev_c = 0;
            while (currentState == SEND)
            {
                char c = 0; // Local variable for each loop, sent to queue via address
                if (ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &t) == 0)
                {
                    // Detection for horizontal movement
                    if (inBetween(&gx, -MOTION_EDGE, MOTION_EDGE) && !inBetween(&gy, -MOTION_EDGE * 0.75, MOTION_EDGE * 0.75) && !inBetween(&gz, -MOTION_EDGE * 0.75, MOTION_EDGE * 0.75))
                    {
                        c = '-';
                    }
                    // Detection for vertical movement
                    else if (!inBetween(&gx, -MOTION_EDGE * 1.25, MOTION_EDGE * 1.25) && (inBetween(&gy, -MOTION_EDGE * 0.75, MOTION_EDGE * 0.75) || inBetween(&gz, -MOTION_EDGE * 0.75, MOTION_EDGE * 0.75)))
                    {
                        c = '.';
                    }

                    // Detect the input is a new user action
                    if (c != 0 && prev_c != c)
                    {
                        // Send character to the queue
                        xQueueSend(morseTxQueue, &c, portMAX_DELAY);

                        // Print the accepted action values
                        sprintf(buf, "[listenMovementTask] Generated: %c; Accel: X=%.2f, Y=%.2f, Z=%.2f | Gyro: X=%.2f, Y=%.2f, Z=%.2f\n\n", c, ax, ay, az, gx, gy, gz);
                        usb_serial_print(buf);
                        usb_serial_flush();
                    }
                }
                else
                {
                    usb_serial_print("Failed to read imu data\n");
                }

                // Remember the previous user action outcome
                prev_c = c;

                // Adjust software responsiveness
                if (c == 0)
                    vTaskDelay(pdMS_TO_TICKS(SENSOR_DELAY));
                else
                    vTaskDelay(pdMS_TO_TICKS(USER_INPUT_DELAY));
            }

            clear_display();
            write_text("IDLE");
        }
    }
}

/// @brief Initialises wifi and mqtt connection, and makes communications
/// @param pvParameters Unused
static void wirelessTask(void *pvParameters) {
    (void)pvParameters;

    // Initialize wifi
    if (cyw43_arch_init()) {
        usb_serial_print("failed to initialise\n");
        return;
    }
     
    cyw43_arch_enable_sta_mode();
    cyw43_arch_poll();

    char _msg[30];
    int w_code = cyw43_arch_wifi_connect_async(AP_SSID, AP_PSWD, AP_AUTH_TYPE);
    bool server_connected = false;
    while(w_code == 0) {
        if (cyw43_wifi_scan_active(&cyw43_state) 
        || cyw43_wifi_link_status(&cyw43_state, CYW43_ITF_STA) < CYW43_LINK_JOIN) {
            server_connected = false;
        } else if(server_connected == false) {
            // int result = wireless_init(MQTT_IP, MQTT_PORT, MQTT_USERNAME, MQTT_PASSWORD);
            // if(result == 0) server_connected = true;
        }
        
        vTaskDelay(pdMS_TO_TICKS(SCAN_DELAY));
    }
}

int main()
{
    // Generating random seed
    srand((unsigned)time(NULL));
    // Initialize the JTKJ Hat and OLED display.
    init_hat_sdk();
    init_display();
    clear_display();
    write_text("IDLE");

    // Set the control button
    init_sw1();
    gpio_set_irq_enabled_with_callback(SW1_PIN, GPIO_IRQ_EDGE_FALL, true, btnFxn);

    // Initialize Buzzer
    init_buzzer();

    // Setting up the imu-sensor.
    if (init_ICM42670() == 0)
    {
        usb_serial_print("ICM-42670P initialized successfully!\n");
        if (ICM42670_start_with_default_values() != 0)
        {
            usb_serial_print("ICM-42670P could not initialize accelerometer or gyroscope");
        }
    }
    else
    {
        usb_serial_print("Failed to initialize ICM-42670P.\n");
    }

    currentState = IDLE;

    // creates binary semaphores
    serialSemaphore = xSemaphoreCreateBinary();
    btnSemaphore = xSemaphoreCreateBinary();

    // create queue
    morseRxQueue = xQueueCreate(RX_QUEUE_LENGTH, sizeof(char));
    morseTxQueue = xQueueCreate(TX_QUEUE_LENGTH, sizeof(char));

    BaseType_t result_read_serial = xTaskCreate(
        ReadSerialTask, "READ_SERIAL", STACKSIZE, (void *)127, 1, &ReadSerialTaskHandle);

    BaseType_t result_handle_lcd = xTaskCreate(
        HandleLcdTask, "UPDATE_LCD", STACKSIZE, (void *)127, 1, &HandleLcdTaskHandle);

    BaseType_t result_send = xTaskCreate(
        SendTask, "SEND", STACKSIZE, (void *)127, 1, &SendTaskHandle);

    BaseType_t result_usb = xTaskCreate(
        usbTask, "TinyUSB", STACKSIZE, (void *)127, 2, &UsbTaskHandle);

    BaseType_t result_imu = xTaskCreate(
        listenMovementTask, "imu", STACKSIZE, (void *)127, 1, &ListenImuTaskHandle);

    BaseType_t result_wifi = xTaskCreate(
        wirelessTask, "wifi", STACKSIZE, (void *)127, 1, &WirelessHandle);

    tusb_init();

    usb_serial_init();
    vTaskStartScheduler();

    return 0;
}