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

// QUEUES
#define EVENT_QUEUE_LENGTH 32
#define RX_QUEUE_LENGTH 128
#define TX_QUEUE_LENGTH 128

// IMU
#define USER_INPUT_DELAY 200
#define SENSOR_DELAY 40
#define MOTION_EDGE 80

// OLED
#define MAX_OLED_WIDTH 21
#define MAX_OLED_HEIGHT 7
#define MAX_OLED_CHAR_NUM 147
#define CHAR_OLED_WIDTH 6
#define CHAR_OLED_HEIGHT 7

// TASKS
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
    UART_CONNECTED = 0,
    UART_DISCONNECTED
} uartState;

typedef enum
{
    USB_CONNECTED = 0,
    USB_DISCONNECTED
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
    EVT_BTN1_PRESSED = 0,
    EVT_BTN2_PRESSED,
    EVT_USB_CONNECTED,
    EVT_USB_DISCONNECTED,
    EVT_UART_CONNECTED,
    EVT_UART_DISCONNECTED,
    EVT_IMU_INPUT
} eventType;

typedef struct
{
    eventType event_type;
    char msg;
} programEvent;

programState program_state;

// initializes task handles
TaskHandle_t RxUsbTaskHandle = NULL;
TaskHandle_t TxUsbTaskHandle = NULL;
TaskHandle_t DisplayTaskHandle = NULL;
TaskHandle_t ListenImuTaskHandle = NULL;
TaskHandle_t UsbTaskHandle = NULL;
TaskHandle_t ControllerTaskHandle = NULL;

// initializes semaphores
SemaphoreHandle_t usbReadSemaphore = NULL;
SemaphoreHandle_t uartReadSemaphore = NULL;
SemaphoreHandle_t imuSemaphore = NULL;

QueueHandle_t eventQueue = NULL;
QueueHandle_t uartRxQueue = NULL;
QueueHandle_t uartTxQueue = NULL;
QueueHandle_t usbRxQueue = NULL;
QueueHandle_t usbTxQueue = NULL;

// helper function for logging
const char *event_type_to_str(eventType e)
{
    switch (e)
    {
    case EVT_BTN1_PRESSED:
        return "EVT_BTN1_PRESSED";
    case EVT_BTN2_PRESSED:
        return "EVT_BTN2_PRESSED";
    case EVT_USB_CONNECTED:
        return "EVT_USB_CONNECTED";
    case EVT_USB_DISCONNECTED:
        return "EVT_USB_DISCONNECTED";
    case EVT_UART_CONNECTED:
        return "EVT_UART_CONNECTED";
    case EVT_UART_DISCONNECTED:
        return "EVT_UART_DISCONNECTED";
    case EVT_IMU_INPUT:
        return "EVT_IMU_INPUT";
    default:
        return "UNDEFINED";
    }
}

const char *display_state_to_str(displayState s)
{
    switch (s)
    {
    case SHOW_USB_MORSE:
        return "SHOW_USB_MORSE";
    case SHOW_UART_MORSE:
        return "SHOW_UART_MORSE";
    case SHOW_USB_TRANSLATION:
        return "SHOW_USB_TRANSLATION";
    case SHOW_UART_TRANSLATION:
        return "SHOW_UART_TRANSLATION";
    default:
        return "UNDEFINED";
    }
}

const char *core_state_to_str(coreState s)
{
    switch (s)
    {
    case IDLE:
        return "IDLE";
    case SEND:
        return "SEND";
    default:
        return "UNDEFINED";
    }
}

void controller_task(void *pvParameters)
{
    programEvent cur_event;
    char to_send;

    while (true)
    {
        if (xQueueReceive(eventQueue, &cur_event, portMAX_DELAY))
        {

            // logging
            char log_buf[LOG_BUFFER_SIZE];
            snprintf(log_buf, sizeof(log_buf), "[controller_task] State = %s; Event received: %s\n\n",
                     core_state_to_str(program_state.core_state), event_type_to_str(cur_event.event_type));
            usb_serial_print(log_buf);
            usb_serial_flush();
            // logging

            if (cur_event.event_type == EVT_BTN1_PRESSED)
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
                    if (program_state.uart_state == UART_CONNECTED)
                        xQueueSend(uartTxQueue, &to_send, portMAX_DELAY);

                    if (program_state.usb_state == USB_CONNECTED)
                        xQueueSend(usbTxQueue, &to_send, portMAX_DELAY);
                }
            }
            else if (cur_event.event_type == EVT_BTN2_PRESSED)
            {
                if (program_state.core_state == IDLE)
                    program_state.display_state = (program_state.display_state + 1) % NUM_OLED_STATES;
                else
                    continue;
            }
            else if (cur_event.event_type == EVT_IMU_INPUT)
            {
                if (program_state.core_state == SEND)
                {
                    // resets conseq spaces counter on IMU input
                    program_state.conseq_spaces_count = 0;

                    to_send = cur_event.msg;

                    if (program_state.uart_state == UART_CONNECTED)
                        xQueueSend(uartTxQueue, &to_send, portMAX_DELAY);

                    if (program_state.usb_state == USB_CONNECTED)
                        xQueueSend(usbTxQueue, &to_send, portMAX_DELAY);
                }
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

void usb_task(void *arg)
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

// // UART interrupt
// void on_uart_rx(void)
// {
//     BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//     xSemaphoreGiveFromISR(uartReadSemaphore, &xHigherPriorityTaskWoken);
// }

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

void btnFxn(uint gpio, uint32_t events)
{
    static uint32_t last_pressed[2];
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint8_t btn_ind = 0;

    if (gpio == SW2_PIN)
        btn_ind = 1;
    uint32_t now = to_ms_since_boot(get_absolute_time());
    if (now - last_pressed[btn_ind] < 50)
        return;

    last_pressed[btn_ind] = now;

    if (gpio == SW1_PIN)
    {
        programEvent event = {EVT_BTN1_PRESSED, 0};
        xQueueSendFromISR(eventQueue, &event, &xHigherPriorityTaskWoken);
    }
    else if (gpio == SW2_PIN)
    {
        programEvent event = {EVT_BTN2_PRESSED, 0};
        xQueueSendFromISR(eventQueue, &event, &xHigherPriorityTaskWoken);
    }

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
            snprintf(log_buf, sizeof(log_buf), "[rx_usb_task] State = %s; Task activated, reading usb buffer...\n\n",
                     core_state_to_str(program_state.core_state));
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
                    char_to_buzz(buf[i]);
                    xQueueSend(usbRxQueue, &buf[i], portMAX_DELAY);
                }
            }

            // logging
            snprintf(log_buf, sizeof(log_buf), "[rx_usb_task] State = %s; Usb buffer drained\n\n",
                     core_state_to_str(program_state.core_state));
            usb_serial_print(log_buf);
            usb_serial_flush();
            // logging
        }
    }
}

// void rx_uart_task(void *pvParameters)
// {
//     // takes uartReadSemaphore and drains uart buffer
// }

// void tx_uart_task(void *pvParameters)
// {
//     // sends characters over UART from uartTxQueue
// }

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

bool drain_queue_to_buffer(QueueHandle_t queue, char *buffer, uint16_t *cur_buffer_ind, bool *eom_encountered)
{
    char c;
    bool new_data = false;

    while (xQueueReceive(queue, &c, 0) == pdTRUE)
    {
        // first symbol of a new message is detected
        if (*eom_encountered)
        {
            *cur_buffer_ind = 0;
            *eom_encountered = false;
        }

        if (c == '\r')
            continue;

        if (c == '\n')
        {
            *eom_encountered = true;
            continue;
        }

        // overflow guard
        if (*cur_buffer_ind < RX_TX_BUFFER_SIZE - 1)
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

    // positions in respective morse buffers
    // e.g. read 3 symbols from usb -> morse_usb_ind shifted by 3
    uint16_t morse_usb_ind = 0;
    uint16_t morse_uart_ind = 0;

    bool eom_uart_encountered = false;
    bool eom_usb_encountered = false;

    while (true)
    {
        new_display_state = program_state.display_state;

        // drain respective buffers
        bool usb_morse_updated = drain_queue_to_buffer(usbRxQueue, morse_usb_msg, &morse_usb_ind, &eom_usb_encountered);
        bool uart_morse_updated = drain_queue_to_buffer(uartRxQueue, morse_uart_msg, &morse_uart_ind, &eom_uart_encountered);

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
            // logging
            char log_buf[LOG_BUFFER_SIZE];
            snprintf(log_buf, sizeof(log_buf), "[display_task] State = %s; New display state: %s; New relevant data: %d; Redrawing display;\n\n",
                     core_state_to_str(program_state.core_state), display_state_to_str(new_display_state), new_relevant_data);
            usb_serial_print(log_buf);
            usb_serial_flush();
            // logging

            switch (new_display_state)
            {
            case SHOW_USB_MORSE:

                // logging
                snprintf(log_buf, sizeof(log_buf), "[display_task] State = %s; USB morse to draw: %s;\n\n",
                         core_state_to_str(program_state.core_state), morse_usb_msg);
                usb_serial_print(log_buf);
                usb_serial_flush();
                // logging

                dumb_display_text(morse_usb_msg);
                break;
            case SHOW_UART_MORSE:

                // logging
                snprintf(log_buf, sizeof(log_buf), "[display_task] State = %s; UART morse to draw: %s;\n\n",
                         core_state_to_str(program_state.core_state), morse_uart_msg);
                usb_serial_print(log_buf);
                usb_serial_flush();
                // logging

                dumb_display_text(morse_uart_msg);
                break;
            case SHOW_USB_TRANSLATION:
                morse_to_text(morse_usb_msg, usb_msg_translation, RX_TX_BUFFER_SIZE);

                // logging
                snprintf(log_buf, sizeof(log_buf), "[display_task] State = %s; USB Translation to draw: %s;\n\n",
                         core_state_to_str(program_state.core_state), usb_msg_translation);
                usb_serial_print(log_buf);
                usb_serial_flush();
                // logging

                dumb_display_text(usb_msg_translation);
                break;
            case SHOW_UART_TRANSLATION:
                morse_to_text(morse_uart_msg, uart_msg_translation, RX_TX_BUFFER_SIZE);

                // logging
                char log_buf[LOG_BUFFER_SIZE];
                snprintf(log_buf, sizeof(log_buf), "[display_task] State = %s; UART Translation to draw: %s;\n\n",
                         core_state_to_str(program_state.core_state), uart_msg_translation);
                usb_serial_print(log_buf);
                usb_serial_flush();
                // logging

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

void tx_usb_task(void *pvParameters)
{
    char queue_char;

    while (!tud_mounted())
    {
        // logging
        char log_buf[LOG_BUFFER_SIZE];
        snprintf(log_buf, sizeof(log_buf), "[tx_usb_task] State = %s; tud not mounted\n\n",
                 core_state_to_str(program_state.core_state));
        usb_serial_print(log_buf);
        usb_serial_flush();
        // logging

        vTaskDelay(pdMS_TO_TICKS(50));
    }

    while (true)
    {
        if (xQueueReceive(usbTxQueue, &queue_char, portMAX_DELAY) == pdTRUE)
        {
            // usb connection guard
            while (!tud_cdc_n_connected(CDC_ITF_DATA))
            {
                // logging
                char log_buf[LOG_BUFFER_SIZE];
                snprintf(log_buf, sizeof(log_buf), "[tx_usb_task] State = %s; tud cdc 1 not connected\n\n",
                         core_state_to_str(program_state.core_state));
                usb_serial_print(log_buf);
                usb_serial_flush();
                // logging
                vTaskDelay(pdMS_TO_TICKS(50));
            }

            tud_cdc_n_write(CDC_ITF_DATA, &queue_char, 1);
            tud_cdc_n_write_flush(CDC_ITF_DATA);

            // logging
            char log_buf[LOG_BUFFER_SIZE];
            snprintf(log_buf, sizeof(log_buf),
                     "[SendTask] State = %s; Symbol sent: \"%c\"\n\n",
                     core_state_to_str(program_state.core_state), queue_char);
            usb_serial_print(log_buf);
            usb_serial_flush();
            // logging

            char_to_buzz(queue_char);
        }
    }
}

/// @brief Listens to the IMU-sensor values to add morse code to the queue via device movement
/// @param pvParameters Unused
static void imu_task(void *pvParameters)
{
    (void)pvParameters;

    float ax, ay, az, gx, gy, gz, t;

    // Start device movement decodings for sending morse code. Infinite loop.
    char buf[LOG_BUFFER_SIZE];
    char prev_c;
    while (true)
    {
        if (xSemaphoreTake(imuSemaphore, portMAX_DELAY) == pdTRUE)
        {
            prev_c = 0;
            while (program_state.core_state == SEND)
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
                        // Send event to queue
                        programEvent event = {EVT_IMU_INPUT, c};
                        xQueueSend(eventQueue, &event, portMAX_DELAY);

                        // Print the accepted action values
                        sprintf(buf, "[imu_task] Generated: %c; Accel: X=%.2f, Y=%.2f, Z=%.2f | Gyro: X=%.2f, Y=%.2f, Z=%.2f\n\n", c, ax, ay, az, gx, gy, gz);
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
    write_text("HI!");

    // Set the buttons
    init_sw1();
    init_sw2();

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

    // program_state initial config
    program_state.core_state = IDLE;
    program_state.display_state = SHOW_USB_MORSE;
    // for testing!!!
    program_state.uart_state = UART_DISCONNECTED;
    program_state.usb_state = USB_CONNECTED;

    // creates binary semaphores
    usbReadSemaphore = xSemaphoreCreateBinary();
    uartReadSemaphore = xSemaphoreCreateBinary();
    imuSemaphore = xSemaphoreCreateBinary();

    // create queues
    eventQueue = xQueueCreate(EVENT_QUEUE_LENGTH, sizeof(programEvent));
    usbRxQueue = xQueueCreate(RX_QUEUE_LENGTH, sizeof(char));
    usbTxQueue = xQueueCreate(TX_QUEUE_LENGTH, sizeof(char));
    uartRxQueue = xQueueCreate(RX_QUEUE_LENGTH, sizeof(char));
    uartTxQueue = xQueueCreate(TX_QUEUE_LENGTH, sizeof(char));

    BaseType_t result_rx_usb = xTaskCreate(
        rx_usb_task, "RX_USB", STACKSIZE, NULL, 1, &RxUsbTaskHandle);

    BaseType_t result_tx_usb = xTaskCreate(
        tx_usb_task, "TX_USB", STACKSIZE, NULL, 1, &TxUsbTaskHandle);

    BaseType_t result_handle_lcd = xTaskCreate(
        display_task, "DISPLAY", STACKSIZE, NULL, 1, &DisplayTaskHandle);

    BaseType_t result_usb = xTaskCreate(
        usb_task, "TinyUSB", STACKSIZE, NULL, 2, &UsbTaskHandle);

    BaseType_t result_imu = xTaskCreate(
        imu_task, "IMU", STACKSIZE, NULL, 1, &ListenImuTaskHandle);

    BaseType_t result_controller = xTaskCreate(
        controller_task, "controller", STACKSIZE, NULL, 2, &ControllerTaskHandle);

    // Set buttons interrupt callback
    gpio_set_irq_enabled_with_callback(SW1_PIN, GPIO_IRQ_EDGE_FALL, true, btnFxn);
    gpio_set_irq_enabled(SW2_PIN, GPIO_IRQ_EDGE_FALL, true);

    tusb_init();

    usb_serial_init();
    vTaskStartScheduler();

    return 0;
}