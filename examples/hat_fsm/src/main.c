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

// initializes semaphores
SemaphoreHandle_t serialSemaphore = NULL;
SemaphoreHandle_t btnSemaphore = NULL;

QueueHandle_t morseRxQueue = NULL;
QueueHandle_t morseTxQueue = NULL;

// runs tud_task() responsilbe for TinyUSB handling USB connection
// activates on hardware usb interrupt, activates callback functions
// needs to preempt any other task
void usbTask(void *arg)
{
    (void)arg;
    while (true)
    {
        tud_task();
    }
}

// callback executed automatically by TinyUSB
// itf indicates which port (CDC0 or CDC1) received data
void tud_cdc_rx_cb(uint8_t itf)
{
    // condition ensures program only processes data from CDC1
    if (itf == CDC_ITF_DATA && currentState != SEND)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        // gives serialSemaphore required by ReadSerialTask
        xSemaphoreGiveFromISR(serialSemaphore, &xHigherPriorityTaskWoken);
        // no portYIELD_FROM_ISR() since all tasks have equal priority, no preemption needed
    }
}

// button interrupt handler
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
    } // if program is in SEND, button inputs spaces/EOM
    else if (currentState == SEND)
    {
        char c = ' ';
        xQueueSendFromISR(morseTxQueue, &c, &xHigherPriorityTaskWoken);
    }
}

// takes the binary semaphore serialSemaphore which in turn is given by serial ISR
// drains the serial buffer and adds read characters to RTOS Queue
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

// function that displays text on oled, takes care of display constraints
// txt - null-terminated string
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
                // logging
                char log_buf[LOG_BUFFER_SIZE];
                snprintf(log_buf, sizeof(log_buf),
                         "[HandleLcdTask] State = %d; EOM reached, clearing display, initiating translation; Morse message: %s\n\n",
                         currentState, morse_msg);
                usb_serial_print(log_buf);
                usb_serial_flush();
                // logging

                morse_msg[ind] = '\0';

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
                char for_display[2] = {queue_char, '\0'};
                smart_display_text(&x_ind, &y_ind, for_display);

                // Use buzzer to indicate a succesful morse signal
                switch (queue_char)
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
                default:
                    break;
                }

                ind += 1;
            }
        }
    }
}

// activated with a button interrupt
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
                        snprintf(log_buf, sizeof(log_buf), "[SendTask] State = %d; EOM generated; Morse generated: \"%s\"\n\n", currentState, morse_buf);
                        usb_serial_print(log_buf);
                        usb_serial_flush();
                        // logging

                        memset(morse_buf, 0, sizeof(morse_buf));
                        ind = 0;
                    }
                    else
                        conseq_spaces += 1;
                }
                else
                    conseq_spaces = 0;

                morse_buf[ind] = queue_char;
                ind += 1;

                tud_cdc_n_write(CDC_ITF_DATA, &queue_char, 1);
                tud_cdc_n_write_flush(CDC_ITF_DATA);

                // logging
                char log_buf[LOG_BUFFER_SIZE];
                snprintf(log_buf, sizeof(log_buf),
                         "[SendTask] State = %d; Symbol sent: \"%c\"\n\n", currentState, queue_char);
                usb_serial_print(log_buf);
                usb_serial_flush();
                // logging

                switch (queue_char)
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
                }
            }
        }
    }
}

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
            prev_c = 0;
            while (currentState == SEND)
            {
                char c = 0; // Local variable for each loop, sent to queue via address
                if (ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &t) == 0)
                {
                    if (inBetween(&gx, -MOTION_EDGE, MOTION_EDGE) && !inBetween(&gy, -MOTION_EDGE * 0.75, MOTION_EDGE * 0.75) && !inBetween(&gz, -MOTION_EDGE * 0.75, MOTION_EDGE * 0.75))
                    {
                        c = '-';
                    }
                    else if (!inBetween(&gx, -MOTION_EDGE * 1.25, MOTION_EDGE * 1.25) && (inBetween(&gy, -MOTION_EDGE * 0.75, MOTION_EDGE * 0.75) || inBetween(&gz, -MOTION_EDGE * 0.75, MOTION_EDGE * 0.75)))
                    {
                        c = '.';
                    }

                    // Send results
                    if (c != 0 && prev_c != c)
                    { // New user action
                        xQueueSend(morseTxQueue, &c, portMAX_DELAY);

                        // Print the accepted action
                        sprintf(buf, "[listenMovementTask] Generated: %c; Accel: X=%.2f, Y=%.2f, Z=%.2f | Gyro: X=%.2f, Y=%.2f, Z=%.2f\n\n", c, ax, ay, az, gx, gy, gz);
                        usb_serial_print(buf);
                        usb_serial_flush();
                    }
                }

                else
                {
                    usb_serial_print("Failed to read imu data\n");
                }

                prev_c = c;

                // Adjust software responsiveness
                if (c == 0)
                    vTaskDelay(pdMS_TO_TICKS(SENSOR_DELAY));
                else
                    vTaskDelay(pdMS_TO_TICKS(USER_INPUT_DELAY));
            }
        }
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
    write_text("Working");

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
        listenMovementTask, "imu", 1024, (void *)127, 1, &ListenImuTaskHandle);

    tusb_init();
    usb_serial_init();
    vTaskStartScheduler();

    return 0;
}