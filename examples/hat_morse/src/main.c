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

#define LOG_BUFFER_SIZE 30
#define TEMP_MIN 0
#define TEMP_MAX 40
#define LUX_MIN 100
#define LUX_MAX 1500
#define CDC_ITF_DATA 1
#define RX_QUEUE_LENGTH 128
#define USER_INPUT_DELAY 200
#define SENSOR_DELAY    40
static QueueHandle_t morseTxQueue = NULL;  
static QueueHandle_t morseRxQueue = NULL; 

// Morse dictionary: each letter or number has a Morse code.
typedef struct {
    char character;
    const char *code;
} MorseEntry;

// Morse code dictionary from:
// https://www.programmersranch.com/2013/05/c-basics-morse-code-converter-using.html
// Credit to the original author for the dictionary used below.
static const MorseEntry morseDictionary[] = {
    {'A', ".-"},
    {'B', "-..."},
    {'C', "-.-."},
    {'D', "-.."},
    {'E', "."},
    {'F', "..-."},
    {'G', "--."},
    {'H', "...."},
    {'I', ".."},
    {'J', ".---"},
    {'K', "-.-"},
    {'L', ".-.."},
    {'M', "--"},
    {'N', "-."},
    {'O', "---"},
    {'P', ".--."},
    {'Q', "--.-"},
    {'R', ".-."},
    {'S', "..."},
    {'T', "-"},
    {'U', "..-"},
    {'V', "...-"},
    {'W', ".--"},
    {'X', "-..-"},
    {'Y', "-.--"},
    {'Z', "--.."},

    {'0', "-----"},
    {'1', ".----"},
    {'2', "..---"},
    {'3', "...--"},
    {'4', "....-"},
    {'5', "....."},
    {'6', "-...."},
    {'7', "--..."},
    {'8', "---.."},
    {'9', "----."}
};

#define MORSE_DICTIONARY_SIZE \
    (sizeof(morseDictionary) / sizeof(morseDictionary[0]))

static int text_to_morse(const char *text, char *output, size_t output_size)
{
    size_t output_index = 0;
    int first_letter = 1;
    int new_word = 0;

    if (text == NULL || output == NULL || output_size == 0) {
        return 0;
    }

    output[0] = '\0';

    for (size_t i = 0; text[i] != '\0'; i++) {

        char letter = text[i];

        // Convert lowercase letters into uppercase.
        if (letter >= 'a' && letter <= 'z') {
            letter = letter - ('a' - 'A');
        }

        // Remember when a new word starts.
        if (letter == ' ') {
            if (!first_letter) {
                new_word = 1;
            }
            continue;
        }

        // Find the character in the Morse dictionary.
        const char *morse_code = NULL;

        for (size_t j = 0; j < MORSE_DICTIONARY_SIZE; j++) {
            if (morseDictionary[j].character == letter) {
                morse_code = morseDictionary[j].code;
                break;
            }
        }

        // Character was not found in the dictionary.
        if (morse_code == NULL) {
            output[0] = '\0';
            return 0;
        }

        // Add a separator before the next Morse letter.
        if (!first_letter) {

            if (new_word) {
        // Two spaces between words.
                if (output_index + 2 >= output_size) {
            output[0] = '\0';
            return 0;
        }
            output[output_index++] = ' ';
            output[output_index++] = ' ';
        }
            else {
                // One space between letters.
                if (output_index + 1 >= output_size) {
                    output[0] = '\0';
                    return 0;
                }

                output[output_index++] = ' ';
    }
}
        // Check whether the output buffer has enough space.
        size_t code_length = strlen(morse_code);

        if (output_index + code_length >= output_size) {
            output[0] = '\0';
            return 0;
        }

        // Copy the Morse code into the output.
        for (size_t k = 0; k < code_length; k++) {
            output[output_index++] = morse_code[k];
        }

        first_letter = 0;
        new_word = 0;
    }

    output[output_index] = '\0';

    return 1;
}

static int morse_to_text(const char *morse, char *output, size_t output_size)
{
    size_t output_index = 0;
    size_t i = 0;

    if (morse == NULL || output == NULL || output_size == 0) {
        return 0;
    }

    output[0] = '\0';

    while (morse[i] != '\0') {

        // Check whether spaces separate letters or words.
        if (morse[i] == ' ') {
            int space_count = 0;

            while (morse[i] == ' ') {
                space_count++;
                i++;
            }

    // Two spaces mean a new word.
    if (space_count >= 2 && output_index > 0) {
        if (output_index + 1 >= output_size) {
            output[0] = '\0';
            return 0;
        }

        output[output_index++] = ' ';
    }
    continue;
}
        // Read one complete Morse symbol.
        char symbol[8];
        size_t symbol_index = 0;

        while (morse[i] != '\0' && morse[i] != ' ' && morse[i] != '/') {

            if (symbol_index >= sizeof(symbol) - 1) {
                output[0] = '\0';
                return 0;
            }

            if (morse[i] != '.' && morse[i] != '-') {
                output[0] = '\0';
                return 0;
            }

            symbol[symbol_index++] = morse[i];
            i++;
        }

        symbol[symbol_index] = '\0';

        // Find the matching letter in the dictionary.
        int found = 0;

        for (size_t j = 0; j < MORSE_DICTIONARY_SIZE; j++) {

            if (strcmp(morseDictionary[j].code, symbol) == 0) {

                if (output_index + 1 >= output_size) {
                    output[0] = '\0';
                    return 0;
                }

                output[output_index++] = morseDictionary[j].character;
                found = 1;
                break;
            }
        }

        // Invalid Morse sequence.
        if (!found) {
            output[0] = '\0';
            return 0;
        }
    }

    output[output_index] = '\0';

    return 1;
}

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

static int rand_in_range(int min, int max)
{
    return min + rand() % (max - min + 1);
}

// ---- Task generating sensor data----
static void sendTask(void *arg)
{
    (void)arg;
    char ch;

    // Wait until the USB device is mounted.
    while (!tud_mounted())
    {
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    while (1)
    {
        // Wait until a Morse character is available.
        if (xQueueReceive(morseTxQueue, &ch, portMAX_DELAY) == pdTRUE)
        {

            // Wait until CDC1 is connected.
            if (!tud_cdc_n_connected(CDC_ITF_DATA))
            {
                vTaskDelay(pdMS_TO_TICKS(50));
                continue;
            }

            // Send one character to the workstation.
            tud_cdc_n_write(CDC_ITF_DATA, &ch, 1);
            tud_cdc_n_write_flush(CDC_ITF_DATA);

            // Use buzzer to indicate a succesful morse signal
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
            }
        }
    }
}

static void testMorseTask(void *arg)
{
    (void)arg;

    const char *message = "... --- ...\n";

    while (1)
    {
        // Put each character into the queue for sendTask.
        for (const char *p = message; *p != '\0'; p++)
        {
            xQueueSend(morseTxQueue, p, portMAX_DELAY);
        }

        // Wait before sending the test message again.
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
// Receives messages from the workstation, translates them,
// displays the result on the OLED and sends it back.
static void receiveTask(void *arg)
{
    (void)arg;

    char ch;
    char message[128];
    char translated[256];
    uint32_t index = 0;

    while (1) {

        if (xQueueReceive(morseRxQueue, &ch, portMAX_DELAY) == pdTRUE) {

            // Ignore carriage return.
            if (ch == '\r') {
                continue;
            }

            // A newline means the complete message has arrived.
            if (ch == '\n') {

                message[index] = '\0';

                if (index >= 2) {

                    int success = 0;

                    // D: means display the original text without translating it.
                    if (message[0] == 'D' && message[1] == ':') {
                        
                        // Copy the original message without the D: prefix.
                        strcpy(translated, message + 2);
                        
                        success = 1;
                    }

                    // T: means text-to-Morse conversion.
                    else if (message[0] == 'T' && message[1] == ':') {

                        success = text_to_morse(
                            message + 2,
                            translated,
                            sizeof(translated)
                        );
                    }

                    // M: means Morse-to-text conversion.
                    else if (message[0] == 'M' && message[1] == ':') {

                        success = morse_to_text(
                            message + 2,
                            translated,
                            sizeof(translated)
                        );
                    }

                    // Show and send the translated result.
                    if (success) {

                        clear_display();
                        write_text(translated);

                        // Send every translated character through the existing USB sending task.
                        for (size_t i = 0; translated[i] != '\0'; i++) {
                            xQueueSend(
                                morseTxQueue,
                                &translated[i],
                                portMAX_DELAY
                            );
                        }

                        // Send newline to finish the response.
                        char newline = '\n';
                        xQueueSend(
                            morseTxQueue,
                            &newline,
                            portMAX_DELAY
                        );
                    }
                    else {
                        clear_display();
                        write_text("Invalid input");
                    }
                }

                // Reset the message buffer for the next message.
                index = 0;
            }

            // Store incoming characters while there is space.
            else if (index < sizeof(message) - 1) {
                message[index++] = ch;
            }
        }
    }
}
// ---- Task running USB stack ----
static void usbTask(void *arg)
{
    (void)arg;
    while (1)
    {
        tud_task(); // With FreeRTOS wait for events
                    // Do not add vTaskDelay.
    }
}

/// @brief A shorthand function to implement a repeated value range comparison
/// @param value Pointer to the value to be evaluated
/// @param low The low non-reachable boundary for the value
/// @param high The high non-reachable boundary for the value
/// @return true (1) if the value is between the boundary values, otherwise false (0)
static inline int inBetween(const float *value, float low, float high)
{
    if (*value > low && *value < high)
        return 1;
    else
        return 0;
}

static void listenMovementTask(void *pvParameters)
{
    (void)pvParameters;

    float ax, ay, az, gx, gy, gz, t;

    // Start device movement decodings for sending morse code. Infinite loop.
    uint8_t buf[LOG_BUFFER_SIZE];
    char prev_c;
    while (1)
    {
        char c = 0; // Local variable for each loop, sent to queue via address
        if (ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &t) == 0)
        {
#define MOTION_EDGE 80 // Used for ease of modification
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
                sprintf(buf, "Accel: X=%.2f, Y=%.2f, Z=%.2f | Gyro: X=%.2f, Y=%.2f, Z=%.2f| Temp: %2.2f°C\n", ax, ay, az, gx, gy, gz, t);
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

/// @brief Listens to the IMU-sensor values to add morse code to the queue via device orientation
/// @param pvParameters Unused
static void listenOrientationTask(void *pvParameters)
{
    (void)pvParameters;

    float ax, ay, az, gx, gy, gz, t;

    // Start device orientation decodings for sending morse code. Infinite loop.
    uint8_t buf[LOG_BUFFER_SIZE];
    char prev_c;
    while (1)
    {
        char c = 0; // Local variable for each loop, sent to queue via address
        if (ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &t) == 0)
        {
            if (inBetween(&ax, -0.4, 0.4) && inBetween(&ay, -0.4, 0.4) && !inBetween(&az, -0.6, 0.6) && inBetween(&gx, -60, 60))
            {
                c = '-';
            }
            else if (inBetween(&ax, -0.4, 0.4) && !inBetween(&ay, -0.6, 0.6) && inBetween(&az, -0.4, 0.4) && inBetween(&gx, -60, 60))
            {
                c = '.';
            }

            // Send results
            if (c == 0)
            {
                sprintf(buf, "Accel: X=%.2f, Y=%.2f, Z=%.2f | Gyro: X=%.2f, Y=%.2f, Z=%.2f| Temp: %2.2f°C\n", ax, ay, az, gx, gy, gz, t);
                usb_serial_print(buf);
                usb_serial_flush();
            }
            else if (prev_c != c)
            { // New user action
                xQueueSend(morseTxQueue, &c, portMAX_DELAY);
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

/// @brief Button caller to control the space input to the workstation
/// @param gpio Unused
/// @param eventMask Unused
static void btn_fxn(uint gpio, uint32_t eventMask)
{
    char c = ' ';
    xQueueSend(morseTxQueue, &c, portMAX_DELAY);
}

int main(void)
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
    gpio_set_irq_enabled_with_callback(SW1_PIN, GPIO_IRQ_EDGE_FALL, true, btn_fxn);

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

    // Queue for outgoing Morse characters.
    morseTxQueue = xQueueCreate(RX_QUEUE_LENGTH, sizeof(char));

    if (morseTxQueue == NULL)
    {
        while (1)
        {
            // Stop if the queue could not be created.
        }
    }
    // Queue for incoming Morse characters.
    morseRxQueue = xQueueCreate(RX_QUEUE_LENGTH, sizeof(char));

    if (morseRxQueue == NULL)
    {
        while (1)
        {
            // Stop if the queue could not be created.
        }
    }
    // Create tasks
    TaskHandle_t hUsb = NULL;
    xTaskCreate(usbTask, "usb", 1024, NULL, 3, &hUsb);
    xTaskCreate(sendTask, "send", 1024, NULL, 2, NULL);

    xTaskCreate(receiveTask, "receive", 1024, NULL, 2, NULL);
    // xTaskCreate(testMorseTask, "test", 1024, NULL, 1, NULL);
    // xTaskCreate(listenOrientationTask, "imu", 1024, NULL, 1, NULL);
    xTaskCreate(listenMovementTask, "imu", 1024, NULL, 1, NULL);

#if (configNUMBER_OF_CORES > 1)
    vTaskCoreAffinitySet(hUsb, 1u << 0);
#endif

    // VERY IMPORTANT, THIS SHOULD GO JUST BEFORE vTaskStartSheduler
    // WITHOUT ANY DELAYS. OTHERWISE, THE TinyUSB stack wont recognize
    // the device.
    // Initialize TinyUSB
    tusb_init();
    // Initialize helper library to write in CDC0)
    usb_serial_init();
    vTaskStartScheduler();
    return 0;
}

// Called by the USB task when a CDC interface receives data.
void tud_cdc_rx_cb(uint8_t itf)
{
    // Reserve two extra bytes for the newline and string terminator.
    char buf[CFG_TUD_CDC_RX_BUFSIZE + 2];
    uint32_t count = tud_cdc_n_read(itf, buf, sizeof(buf) - 2);
    // Put incoming Morse characters into the receiving queue.
    if (itf == 1 && count > 0)
    {
        for (uint32_t i = 0; i < count; i++)
        {
            xQueueSend(morseRxQueue, &buf[i], 0);
        }
    }
    buf[count] = '\n';
    buf[count + 1] = '\0';

    // Also read CDC0 so that its receive buffer does not fill up.
    if (itf == 1 && count > 0)
    {
        if (tud_cdc_n_connected(0))
        {
            // Nonblocking writes: do not use the waiting logger here.
            // Meaning, do not use usb_serial_print() or usb_serial_flush() here.
            const char *prefix = "\nReceived on CDC 1:";
            tud_cdc_n_write(0, prefix, strlen(prefix));
            // Include the appended newline, but not the null terminator.
            tud_cdc_n_write(0, buf, count + 1);
            tud_cdc_n_write_flush(0);
        }

        if (tud_cdc_n_connected(1))
        {
            tud_cdc_n_write(1, "OK\n", 3);
            tud_cdc_n_write_flush(1);
        }
    }
}
