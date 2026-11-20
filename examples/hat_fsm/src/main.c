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

// BUFFERS
#define LOG_BUFFER_SIZE 128
#define RX_TX_BUFFER_SIZE 256

// PORTS
#define CDC_ITF_DATA 1
#define CDC_ITF_DEBUG 0

// IMU
#define RX_QUEUE_LENGTH 128
#define TX_QUEUE_LENGTH 128
#define USER_INPUT_DELAY 200
#define SENSOR_DELAY 40
#define MOTION_EDGE 80

// OLED
#define MAX_OLED_WIDTH 21
#define MAX_OLED_HEIGHT 7
#define MAX_OLED_CHAR_NUM 147
#define CHAR_OLED_WIDTH 6
#define CHAR_OLED_HEIGHT 7

// RTOS
#define STACKSIZE 1024

typedef enum
{
    IDLE = 0,
    SEND
} coreState;

typedef enum
{
    SHOW_USB_MORSE = 0,
    SHOW_UART_MORSE,
    SHOW_USB_TRANSLATION,
    SHOW_UART_TRANSLATION
} displayState;

typedef enum
{
    CONNECTED = 0,
    DISCONNECTED
} uartState;

typedef enum
{
    CONNECTED = 0,
    DISCONNECTED
} usbState;

typedef struct
{
    coreState core_state;
    displayState display_state;
    uartState uart_state;
    usbState usb_state;
    uint8_t conseq_spaces_count;
} programState;

#define NUM_OLED_STATES 4

typedef enum
{
    BTN1_PRESSED = 0,
    BTN2_PRESSED,
    USB_CONNECTED,
    USB_DISCONNECTED,
    UART_CONNECTED,
    UART_DISCONNECTED
} eventType;

typedef struct
{
    eventType event_type;
    char *msg;
} programEvent;

programState program_state;

// initializes task handles
TaskHandle_t ReadSerialTaskHandle = NULL;
TaskHandle_t HandleLcdTaskHandle = NULL;
TaskHandle_t SendTaskHandle = NULL;
TaskHandle_t UsbTaskHandle = NULL;
TaskHandle_t ListenImuTaskHandle = NULL;
TaskHandle_t ControllerTaskHandle = NULL;

// initializes semaphores
SemaphoreHandle_t usbReadSemaphore = NULL;
SemaphoreHandle_t uartReadSemaphore = NULL;
SemaphoreHandle_t imuSemaphore = NULL;

QueueHandle_t uartRxQueue = NULL;
QueueHandle_t uartTxQueue = NULL;

QueueHandle_t usbRxQueue = NULL;
QueueHandle_t usbTxQueue = NULL;

QueueHandle_t eventQueue = NULL;

void controller_task(void *pvParameters)
{
    programEvent cur_event;

    while (true)
    {
        if (xQueueReceive(eventQueue, &cur_event, portMAX_DELAY))
        {
            if (cur_event.event_type == BTN1_PRESSED)
            {
                // btn1 + IDLE -> switch to SEND mode (IMU enabled)
                if (program_state.core_state == IDLE)
                {
                    program_state.conseq_spaces_count = 0;
                    program_state.core_state = SEND;
                    xSemaphoreGive(imuSemaphore);
                }
                // btn1 + SEND -> add space to TxQueue
                else if (program_state.core_state == SEND)
                {
                    char to_send;
                    program_state.conseq_spaces_count += 1;

                    // EOM input sequence
                    if (program_state.conseq_spaces_count == 3)
                    {
                        program_state.conseq_spaces_count = 0;
                        to_send = '\n';

                        program_state.core_state = IDLE;
                    }
                    else
                    {
                        to_send = ' ';
                    }

                    // send character to Tx queues
                    if (program_state.uart_state == CONNECTED)
                        xQueueSend(uartTxQueue, &to_send, portMAX_DELAY);

                    if (program_state.usb_state == CONNECTED)
                        xQueueSend(usbTxQueue, &to_send, portMAX_DELAY);
                }
            }
            else if (cur_event.event_type == BTN2_PRESSED)
            {
                program_state.display_state = (program_state.display_state + 1) % NUM_OLED_STATES;
            }
        }
    }
}

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

void tud_cdc_rx_cb(uint8_t itf)
{
    // condition ensures program only processes data from CDC1
    // incoming data is ignored while the device is on SEND mode
    if (itf == CDC_ITF_DATA)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        // gives usbReadSemaphore required by rx_usb_task
        xSemaphoreGiveFromISR(usbReadSemaphore, &xHigherPriorityTaskWoken);
        // no portYIELD_FROM_ISR() since all tasks have equal priority, no preemption needed
    }
}

// UART interrupt
void on_uart_rx(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(uartReadSemaphore, &xHigherPriorityTaskWoken);
}

void btn1Fxn(uint gpio, uint32_t events)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    programEvent event = {BTN1_PRESSED, "BTN1 PRESSED"};
    xQueueSendFromISR(eventQueue, &event, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void btn2Fxn(uint gpio, uint32_t events)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    programEvent event = {BTN2_PRESSED, "BTN2 PRESSED"};
    xQueueSendFromISR(eventQueue, &event, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
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

void btn1Fxn(uint gpio, uint32_t events)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    programEvent event = {BTN1_PRESSED, "BTN1 PRESSED"};
    xQueueSendFromISR(eventQueue, &event, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void btn2Fxn(uint gpio, uint32_t events)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    programEvent event = {BTN2_PRESSED, "BTN2 PRESSED"};
    xQueueSendFromISR(eventQueue, &event, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void rx_usb_task(void *pvParameters)
{
    char buf[CFG_TUD_CDC_RX_BUFSIZE];

    while (true)
    {
        // Task is blocked indefinitely until usbReadSemaphore is given
        if (xSemaphoreTake(usbReadSemaphore, portMAX_DELAY) == pdTRUE)
        {
            // state switched to RECEIVE while reading serial data

            // logging
            char log_buf[LOG_BUFFER_SIZE];
            snprintf(log_buf, sizeof(log_buf), "[rx_usb_task] State = %d; Task activated, reading usb buffer...\n\n", program_state.core_state);
            usb_serial_print(log_buf);
            usb_serial_flush();
            // logging

            // drains the usb buffer (cdc 1)
            uint32_t count = tud_cdc_n_read(CDC_ITF_DATA, buf, sizeof(buf));
            // puts read characters in queue
            if (count > 0)
            {
                for (uint32_t i = 0; i < count; i++)
                {
                    xQueueSend(usbRxQueue, &buf[i], portMAX_DELAY);
                }
            }

            // logging
            snprintf(log_buf, sizeof(log_buf), "[ReadSerialTask] State = %d; Usb buffer drained\n\n", program_state.core_state);
            usb_serial_print(log_buf);
            usb_serial_flush();
            // logging
        }
    }
}

void rx_uart_task(void *pvParameters)
{
    // takes uartReadSemaphore and drains uart buffer
}

void tx_uart_task(void *pvParameters)
{
    // sends characters over UART from uartTxQueue
}

/**
 * @brief Displays a null-terminated string while managing OLED screen limits.
 *
 * The function converts character positions into pixel on-screen coordinates and
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

// clears the screen, displays given text on the screen
// can write out of bounds if no vertical space left
void dumb_display_text(char *txt)
{
    uint32_t x_ind = 0;
    uint32_t y_ind = 0;

    clear_display();

    for (uint32_t i = 0; txt[i] != '\0'; i++)
    {

        char str[2] = {txt[i], '\0'};
        write_text_xy(x_ind * CHAR_OLED_WIDTH, y_ind * CHAR_OLED_HEIGHT, str);
        x_ind += 1;

        // screen line limit reached, switches to next line
        if (x_ind >= MAX_OLED_WIDTH)
        {
            x_ind = 0;
            y_ind += 1;
        }
    }
}

bool drain_queue_to_buffer(QueueHandle_t queue, char *buffer, uint16_t *cur_buffer_ind)
{
    char c;
    bool new_data = false;

    while (xQueueReceive(queue, &c, 0) == pdTRUE)
    {
        if (c == '\r')
            continue;

        // overflow guard
        if(*cur_buffer_ind < RX_TX_BUFFER_SIZE - 1)
        {
            // puts a character from queue to buffer
            buffer[*cur_buffer_ind] = c;
            *cur_buffer_ind += 1;
            new_data = true;
        }
    }

    return new_data;
}

void display_task(void *pvParameters)
{
    displayState prev_display_state = SHOW_USB_MORSE;
    displayState new_display_state;

    char morse_usb_msg[RX_TX_BUFFER_SIZE];
    char morse_uart_msg[RX_TX_BUFFER_SIZE];
    char usb_msg_translation[RX_TX_BUFFER_SIZE];
    char uart_msg_translation[RX_TX_BUFFER_SIZE];

    uint16_t morse_usb_ind = 0;
    uint16_t morse_uart_ind = 0;

    while (true)
    {
        new_display_state = program_state.display_state;

        // drain respective buffers
        bool usb_morse_updated = drain_queue_to_buffer(usbRxQueue, morse_usb_msg, &morse_usb_ind);
        bool uart_morse_updated = drain_queue_to_buffer(uartRxQueue, morse_uart_msg, &morse_uart_ind);

        // put null-terminator at the end of what was drained
        // needed for displaying text on the screen
        morse_usb_msg[morse_usb_ind] = '\0';
        morse_uart_msg[morse_uart_ind] = '\0';

        // has display mode change?
        bool display_state_updated = (new_display_state != prev_display_state);
        // is there new data availabe for current display mode? (translation doesn't have live update)
        bool new_relevant_data = (usb_morse_updated && (new_display_state == SHOW_USB_MORSE)) || (uart_morse_updated && (new_display_state == SHOW_UART_MORSE));

        // redraws the display if there are any relevant changes
        if (display_state_updated || new_relevant_data)
        {
            switch (new_display_state)
            {
            case SHOW_USB_MORSE:
                dumb_display_text(morse_usb_msg);
                break;
            case SHOW_UART_MORSE:
                dumb_display_text(morse_uart_msg);
                break;
            case SHOW_USB_TRANSLATION:
                morse_to_text(morse_usb_msg, usb_msg_translation, RX_TX_BUFFER_SIZE);
                dumb_display_text(usb_msg_translation);
                break;
            case SHOW_UART_TRANSLATION:
                morse_to_text(morse_uart_msg, uart_msg_translation, RX_TX_BUFFER_SIZE);
                dumb_display_text(uart_msg_translation);
                break;
            default:
                break;
            }
        }

        prev_display_state = new_display_state;

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/**
 * @brief Transmits generated Morse symbols through USB CDC.
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

    char morse_buf[RX_TX_BUFFER_SIZE];
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

                tud_cdc_n_write(CDC_ITF_DATA, &queue_char, 1);
                tud_cdc_n_write_flush(CDC_ITF_DATA);

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
    gpio_set_irq_enabled_with_callback(SW1_PIN, GPIO_IRQ_EDGE_FALL, true, btn1Fxn);
    gpio_set_irq_enabled_with_callback(SW2_PIN, GPIO_IRQ_EDGE_FALL, true, btn2Fxn);

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
        display_task, "UPDATE_LCD", STACKSIZE, (void *)127, 1, &HandleLcdTaskHandle);

    BaseType_t result_send = xTaskCreate(
        SendTask, "SEND", STACKSIZE, (void *)127, 1, &SendTaskHandle);

    BaseType_t result_usb = xTaskCreate(
        usbTask, "TinyUSB", STACKSIZE, (void *)127, 2, &UsbTaskHandle);

    BaseType_t result_imu = xTaskCreate(
        listenMovementTask, "imu", STACKSIZE, (void *)127, 1, &ListenImuTaskHandle);

    BaseType_t result_controller = xTaskCreate(
        controller_task, "controller", STACKSIZE, NULL, 2, &ControllerTaskHandle);

    tusb_init();

    usb_serial_init();
    vTaskStartScheduler();

    return 0;
}