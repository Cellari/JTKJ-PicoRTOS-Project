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
#define MAX_OLED_WIDTH 21     // max number of chars oled can display horizontally
#define MAX_OLED_HEIGHT 7     // max number of chars oled can display vertically
#define MAX_OLED_CHAR_NUM 147 // max number of chars oled can display in total
#define CHAR_OLED_WIDTH 6     // oled char width (px)
#define CHAR_OLED_HEIGHT 7    // oled char height (px)
#define NUM_OLED_STATES 4     // how many states display has

// TASKS
#define STACKSIZE 1024

// BUTTONS
#define BUTTON_DEBONCE_THRESHOLD_MS 50

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

// general program state struct needed for the state machine
// includes info about display, physical connections, core program state
typedef struct
{
    coreState core_state;
    displayState display_state;
    uartState uart_state;
    usbState usb_state;
    uint8_t conseq_spaces_count; // how many conseq. spaces were put in tx queues
} programState;

// types of events in the program (e.g. button 1 was pressed)
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

// program event struct, used for eventQueue that is read by controller_task
typedef struct
{
    eventType event_type;
    char msg;
} programEvent;

// global var for holding the current state of the program
programState program_state;

// global var that stores text currently written on display
char on_display[MAX_OLED_CHAR_NUM + 1];
// global bool flag var that tells the program if the screen was updated at least once
// needed because we put "Hi!" message on program initialization ("Hi!" is not stored in on_display buffer)
bool screen_updated = false;

// task handles
TaskHandle_t RxUsbTaskHandle = NULL;
TaskHandle_t TxUsbTaskHandle = NULL;
TaskHandle_t DisplayTaskHandle = NULL;
TaskHandle_t ListenImuTaskHandle = NULL;
TaskHandle_t UsbTaskHandle = NULL;
TaskHandle_t ControllerTaskHandle = NULL;

// semaphores
SemaphoreHandle_t usbReadSemaphore = NULL;
SemaphoreHandle_t uartReadSemaphore = NULL;
SemaphoreHandle_t imuSemaphore = NULL;

// queues
QueueHandle_t eventQueue = NULL;
QueueHandle_t uartRxQueue = NULL;
QueueHandle_t uartTxQueue = NULL;
QueueHandle_t usbRxQueue = NULL;
QueueHandle_t usbTxQueue = NULL;

// helper function for logging
// returns name of event type as str
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

// helper function for logging
// returns display state as str
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

// helper function for logging
// returns core state name as str
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

/**
 * @brief Task that controls state transitions and responds to events
 *
 * Reads eventQueue to which events are sent by other tasks and reacts according
 * to the type of the event and current program state
 *
 * e.g. If button 1 is pressed, the interrupt callback function puts corresponding event
 * in eventQueue, controller_task reads it, checks the program state; if program is in SEND,
 * a space is put in transmission queue, but if the program is in IDLE, imu_task is given
 * a semaphore, program state is switched to SEND and user may start generating symbols
 *
 * @param pvParameters
 */
void controller_task(void *pvParameters)
{
    programEvent cur_event;
    // char that is sent to transmission queues if needed
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
                    // reset conseq. spaces counter
                    program_state.conseq_spaces_count = 0;
                    // switch program state to SEND
                    program_state.core_state = SEND;
                    // give seamaphore to IMU to activate the imu_task
                    xSemaphoreGive(imuSemaphore);
                }
                // btn1 + SEND -> add space to tx queues
                else if (program_state.core_state == SEND)
                {
                    // space was generated, update the counter
                    program_state.conseq_spaces_count += 1;

                    // 3 conseq. spaces = EOM sequence
                    if (program_state.conseq_spaces_count == 3)
                    {
                        // reset conseq. spaces counter
                        program_state.conseq_spaces_count = 0;
                        // send \n instead of space
                        to_send = '\n';
                        // symbol generation ended, switch state back to IDLE
                        program_state.core_state = IDLE;
                    }
                    else // otherwise send space on button 1 press
                    {
                        to_send = ' ';
                    }

                    // send character to tx queues
                    if (program_state.uart_state == UART_CONNECTED)
                        xQueueSend(uartTxQueue, &to_send, portMAX_DELAY);

                    if (program_state.usb_state == USB_CONNECTED)
                        xQueueSend(usbTxQueue, &to_send, portMAX_DELAY);
                }
            }
            else if (cur_event.event_type == EVT_BTN2_PRESSED)
            {
                // mode switching is allowed only in IDLE (not SEND) to reduce cross task interference
                if (program_state.core_state == IDLE)
                    program_state.display_state = (program_state.display_state + 1) % NUM_OLED_STATES;
                else
                    continue;
            }
            else if (cur_event.event_type == EVT_IMU_INPUT)
            {
                // checks if program is in SEND, otherwise discard symbol
                if (program_state.core_state == SEND)
                {
                    // resets conseq spaces counter on IMU input
                    program_state.conseq_spaces_count = 0;

                    to_send = cur_event.msg;

                    // send character to tx queues
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

/**
 * @brief USB interrupt call back function, notifies rx_usb_task about new data
 *
 * @param itf
 */
void tud_cdc_rx_cb(uint8_t itf)
{
    // program only processes data from CDC1 port
    if (itf == CDC_ITF_DATA)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        // gives usbReadSemaphore required by rx_usb_task
        xSemaphoreGiveFromISR(usbReadSemaphore, &xHigherPriorityTaskWoken);
    }
}

// // UART interrupt
// void on_uart_rx(void)
// {
//     BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//     xSemaphoreGiveFromISR(uartReadSemaphore, &xHigherPriorityTaskWoken);
// }

/**
 * @brief Transforms char into a buzzer signal. Used for both rx, tx operations
 *
 * @param ch
 */
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

/**
 * @brief Button interrupt callback
 *
 * Checks from which pin (button) interrupt came from, sends coresponding
 * event to eventQueue
 *
 * Has basic debounce guard
 *
 * @param gpio - interrupt pin
 * @param events - not used
 */
void btnFxn(uint gpio, uint32_t events)
{
    // holds times when each button was pressed last
    static uint32_t last_pressed[2];
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // ind of button that was pressed (0 for btn1, 1 for btn2)
    // 0 by default, changed if btn 2 was pressed
    // used for accessing last_pressed correctly
    uint8_t btn_ind = 0;

    // checks if btn2 was pressed, changes btn_ind if so
    if (gpio == SW2_PIN)
        btn_ind = 1;

    // time stamp when the current button press happened (since boot)
    uint32_t now = to_ms_since_boot(get_absolute_time());
    // if less than BUTTON_DEBONCE_THRESHOLD_MS has passed between button presses (of the same button)
    // rejects the press (most likely a debounce)
    if (now - last_pressed[btn_ind] < BUTTON_DEBONCE_THRESHOLD_MS)
        return;

    last_pressed[btn_ind] = now;

    // sends corresponding events to eventQueue
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

    // makes sure FreeRTOS switches to controller_task on callback exit
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief Reads received usb characters
 *
 * Takes semaphore, drains usb buffer, puts characters in usbRxQueue
 *
 * @param pvParameters
 */
void rx_usb_task(void *pvParameters)
{
    char buf[CFG_TUD_CDC_RX_BUFSIZE];

    while (true)
    {
        // Task is blocked until usbReadSemaphore is given
        if (xSemaphoreTake(usbReadSemaphore, portMAX_DELAY) == pdTRUE)
        {
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
 * @brief Helper function that drains character from passed queue to passed buffer
 *
 * Is called by display_task to drain RX queues to respective morse buffers
 * Checks if EOM was encountered - used by display_task to track when to reset the buffers
 * for new message
 *
 * @param queue - queue to drain
 * @param buffer - buffer to fill with queue chars
 * @param cur_buffer_ind - where to start writing to buffer
 * @param eom_encountered - pointer to bool flag for checking if EOM was encountered
 * @return true - if new chars were read
 * @return false - if no new chars were read
 */
bool drain_queue_to_buffer(QueueHandle_t queue, char *buffer, uint16_t *cur_buffer_ind, bool *eom_encountered)
{
    char c;
    // flag that switches to true if any new chars were drained
    bool new_data = false;

    // non-blocking (xTicksToWait = 0) queue read
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

        // EOM encountered
        if (c == '\n')
        {
            // set flag to true
            *eom_encountered = true;
            // skip writing \n to buffer
            continue;
        }

        // overflow guard
        if (*cur_buffer_ind < RX_TX_BUFFER_SIZE - 1)
        {
            // puts a character from queue to buffer
            buffer[*cur_buffer_ind] = c;
            // updates the buffer index
            *cur_buffer_ind += 1;
            // marks that new data was received
            new_data = true;
        }
    }

    return new_data;
}

/**
 * @brief Helper function for printing char on given position on display
 *
 * @param display_ind - position on display to print
 * @param c - char to print
 */
void put_char_on_display(uint16_t display_ind, char c)
{
    char str[2] = {c, '\0'};
    // calculates column
    uint8_t x_ind = display_ind % MAX_OLED_WIDTH;
    // calculates row
    uint8_t y_ind = display_ind / MAX_OLED_WIDTH;
    write_text_xy(x_ind * CHAR_OLED_WIDTH, y_ind * CHAR_OLED_HEIGHT, str);
}

/**
 * @brief Helper function used by display_task for appending/redrawing on display
 *
 * Checks if the operation is appending (part of the text is already displayed -> no need to redraw everything)
 * and draws only the new part; or if text is completely new and redraws the screen
 *
 * @param txt - str to be displayed
 */
void update_display_txt(char *txt)
{
    uint16_t txt_len = strlen(txt);
    uint16_t on_display_len = strlen(on_display);
    // checks if text on display is a prefix of txt
    // strncmp returns 0 if if the first (on_display_len) num of chars from both strings match
    bool is_display_txt_prefix = (strncmp(on_display, txt, on_display_len) == 0);
    // checks if it's an appending operation
    bool is_append = screen_updated && is_display_txt_prefix;

    // operation is not an append, need to redraw screen
    if (is_append == false)
    {
        clear_display();
        // resets on screen position
        on_display_len = 0;
    }

    // draws new characters starting at on_display_len
    // if on_display_len = 0 (screen needs to be redrawn), starts from top left corner
    // if on_display_len is not zero (appending), starts writing after the current text
    for (uint16_t i = on_display_len; i < txt_len; i++)
    {
        put_char_on_display(i, txt[i]);
    }

    // copies txt contents to on_display
    strcpy(on_display, txt);

    screen_updated = true;
}

/**
 * @brief Task that handles display updates. Wakes up every 20 ms
 *
 * Reads tx queues, checks if display mode was switched, updates the display if necessary
 * Stores current message buffers
 *
 * @param pvParameters
 */
void display_task(void *pvParameters)
{
    // var used for storing prev display state
    // used to decide if full redrawn is needed
    // e.g. btn was pressed and state was switched from SHOW_USB_MORSE to SHOW_UART_MORSE
    displayState prev_display_state = SHOW_USB_MORSE;

    // var used for storing the current display state
    displayState new_display_state;

    // buffers for storing current messages and their translations
    char morse_usb_msg[RX_TX_BUFFER_SIZE];
    char morse_uart_msg[RX_TX_BUFFER_SIZE];
    char usb_msg_translation[RX_TX_BUFFER_SIZE];
    char uart_msg_translation[RX_TX_BUFFER_SIZE];

    // positions in respective morse buffers
    // e.g. read 3 symbols from usb -> morse_usb_ind shifted by 3
    uint16_t morse_usb_ind = 0;
    uint16_t morse_uart_ind = 0;

    // flags updated on EOM encounters
    bool eom_uart_encountered = false;
    bool eom_usb_encountered = false;

    while (true)
    {
        // update new_display_state to store the current display state
        new_display_state = program_state.display_state;

        // drain usb and uart buffers
        // stores true if buffer was updated
        bool usb_morse_updated = drain_queue_to_buffer(usbRxQueue, morse_usb_msg, &morse_usb_ind, &eom_usb_encountered);
        bool uart_morse_updated = drain_queue_to_buffer(uartRxQueue, morse_uart_msg, &morse_uart_ind, &eom_uart_encountered);

        // put null-terminator at the end of what was drained
        // needed for displaying text on the screen and translation
        morse_usb_msg[morse_usb_ind] = '\0';
        morse_uart_msg[morse_uart_ind] = '\0';

        // checks if display state changed since last task wake up
        bool display_state_updated = (new_display_state != prev_display_state);
        // checks if new data is available for the current display state (translation doesn't have live update)
        bool new_relevant_data = (usb_morse_updated && (new_display_state == SHOW_USB_MORSE)) || (uart_morse_updated && (new_display_state == SHOW_UART_MORSE));

        // updates display if display state was switched or new relevant data arrived
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

                update_display_txt(morse_usb_msg);
                break;
            case SHOW_UART_MORSE:

                // logging
                snprintf(log_buf, sizeof(log_buf), "[display_task] State = %s; UART morse to draw: %s;\n\n",
                         core_state_to_str(program_state.core_state), morse_uart_msg);
                usb_serial_print(log_buf);
                usb_serial_flush();
                // logging

                update_display_txt(morse_uart_msg);
                break;
            case SHOW_USB_TRANSLATION:
                morse_to_text(morse_usb_msg, usb_msg_translation, RX_TX_BUFFER_SIZE);

                // logging
                snprintf(log_buf, sizeof(log_buf), "[display_task] State = %s; USB Translation to draw: %s;\n\n",
                         core_state_to_str(program_state.core_state), usb_msg_translation);
                usb_serial_print(log_buf);
                usb_serial_flush();
                // logging

                update_display_txt(usb_msg_translation);
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

                update_display_txt(uart_msg_translation);
                break;
            default:
                break;
            }
        }

        // update prev_display_state var
        prev_display_state = new_display_state;

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/**
 * @brief Sends chars from usbTxQueue to the workstation
 *
 * @param pvParameters
 */
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

/**
 * @brief Listens to the IMU-sensor values to generate morse code
 *
 * Takes semaphore, detects movements and sends events to eventQueue (generated symbol stored in event.msg)
 *
 * @param pvParameters
 */
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

    // creates queues
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