#include "pico/stdlib.h"
#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>
#include <semphr.h>
#include <tkjhat/sdk.h>


// standard task stacksize
#define STACKSIZE 512

// see state machine diagram for description of each state
typedef enum {IDLE, RECEIVE, UPDATE_LCD, SEND} programState;
// variable holding program state
programState currentState;

// initializes task handles
TaskHandle_t ReadSerialTaskHandle = NULL;
TaskHandle_t HandleLcdTaskHandle = NULL;
TaskHandle_t SendTaskHandle = NULL;

// initializes semaphores
SemaphoreHandle_t serialSemaphore = NULL;
SemaphoreHandle_t btnSemaphore = NULL;

// initializes queue
QueueHandle_t serial_queue = NULL;

// serial isr, triggered when new data arrives over serial
// gives the binary semaphore serialSemaphore required by ReadSerialTask
void uartFxn(void){
    // indicates that there's no need to switch to another task right away
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    // gives semaphore
    xSemaphoreGiveFromISR(serialSemaphore, &xHigherPriorityTaskWoken);
}

// button interrupt handler
void btnFxn(void){
    // if program is in IDLE, button press switches state to SEND
    if(currentState == IDLE)
    {
        // switch the program state to SEND
        currentState = SEND;
        // indicates that there's no need to switch to another task right away
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        // gives semaphore for send_task
        xSemaphoreGiveFromISR(btnSemaphore, &xHigherPriorityTaskWoken);
    } // if program is in SEND, button inputs spaces/EOM
    else if (currentState == SEND)
    {
        //...read space or EOM...
    }
    
}

// takes the binary semaphore serialSemaphore which in turn is given by serial ISR
// drains the serial buffer and adds read characters to RTOS Queue
void ReadSerialTask(void * pvParameters){
    while(true){
        // Task is blocked indefinitely until serialSemaphore is given
        if(xSemaphoreTake(serialSemaphore, portMAX_DELAY) == pdTRUE)
        {
            // ... buffer drain ...
        }
    }
}

// is activated when characters are added to RTOS Queue
// prints morse code on the 1st line of LCD
// prints translation on the 2nd line of LCD
void HandleLcdTask(void * pvParameters){
    char queue_char;

    while(true){
        // stays blocked until queue is updated
        if(xQueueReceive(serial_queue, &queue_char, portMAX_DELAY) == pdTRUE){
            // ... lcd magick ...
        }
    }
}

// activated with a button interrupt
void SendTask(void * pvParameters){
    while(true){
        // blocked until give btnSemaphore
        if(xSemaphoreTake(btnSemaphore, portMAX_DELAY) == pdTRUE)
        {
            // ... IMU magic ...
        }
    }
}

int main() {
    stdio_init_all();

    currentState = IDLE;

    // creates binary semaphores
    serialSemaphore = xSemaphoreCreateBinary();
    btnSemaphore = xSemaphoreCreateBinary();

    // create queue
    serial_queue = xQueueCreate(10, sizeof(char));

    BaseType_t result_read_serial = xTaskCreate(
        ReadSerialTask,
        "READ_SERIAL",
        STACKSIZE,
        (void *) 127,
        2,
        &ReadSerialTaskHandle
    );

    BaseType_t result_handle_lcd = xTaskCreate(
        HandleLcdTask,
        "UPDATE_LCD",
        STACKSIZE,
        (void *) 127,
        2,
        &HandleLcdTaskHandle
    );

    BaseType_t result_send = xTaskCreate(
        SendTask,
        "SEND",
        STACKSIZE,
        (void *) 127,
        2,
        &SendTaskHandle
    );

    vTaskStartScheduler();

    return 0;
}