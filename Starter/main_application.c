/* STANDARD INCLUDES */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

/* KERNEL INCLUDES */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"
#include "extint.h"

/* HARDWARE SIMULATOR UTILITY FUNCTIONS */
#include "HW_access.h"

/* SERIAL CHANNELS */
#define COM_CH_0 (0)       /* senzor osvjetljenja */
#define COM_CH_1 (1)       /* senzor vrata + poruke prema PC */
#define COM_CH_2 (2)       /* PC komande */

/* TASK PRIORITIES */
#define TASK_SERIAL_REC_PRI (tskIDLE_PRIORITY + 5)
#define TASK_PC_REC_PRI     (tskIDLE_PRIORITY + 5)
#define TASK_LED_PRI        (tskIDLE_PRIORITY + 4)
#define TASK_DATA_PROC_PRI  (tskIDLE_PRIORITY + 3)
#define TASK_PC_SEND_PRI    (tskIDLE_PRIORITY + 2)
#define TASK_TRIGGER_PRI    (tskIDLE_PRIORITY + 2)
#define TASK_LCD_PRI        (tskIDLE_PRIORITY + 1)

/* CONSTANTS */
#define R_BUF_SIZE                  (32)
#define LIGHT_QUEUE_SIZE            (10)
#define DOOR_QUEUE_SIZE             (10)
#define LIGHT_AVG_SAMPLES           (10U)
#define SENSOR_PERIOD_MS            (200)
#define PC_SEND_PERIOD_MS           (2000)
#define DISPLAY_PERIOD_MS           (1500)
#define DISPLAY_DIGIT_PERIOD_MS     (167)

/* TASK FORWARD DECLARATIONS */
void main_demo(void);

static void SensorLightReceive_Task(void* pvParameters);
static void SensorDoorReceive_Task(void* pvParameters);
static void SensorTrigger_Task(void* pvParameters);
static void PCReceive_Task(void* pvParameters);
static void PCSend_Task(void* pvParameters);
static void DataProcessing_Task(void* pvParameters);
static void LCDDisplay_Task(void* pvParameters);
static void LEDBar_Task(void* pvParameters);

/* GLOBAL OS HANDLES */
static SemaphoreHandle_t LED_INT_BinarySemaphore;
static SemaphoreHandle_t TBE_BinarySemaphore0;
static SemaphoreHandle_t TBE_BinarySemaphore1;
static SemaphoreHandle_t TBE_BinarySemaphore2;
static SemaphoreHandle_t RXC_BinarySemaphore0;
static SemaphoreHandle_t RXC_BinarySemaphore1;
static SemaphoreHandle_t RXC_BinarySemaphore2;
static SemaphoreHandle_t Trigger_BinarySemaphore;
static SemaphoreHandle_t Blink_BinarySemaphore;

/* MUTEX ZA ZASTITU COM_CH_1 */
static SemaphoreHandle_t COM1_Mutex;

static QueueHandle_t Light_Queue;
static QueueHandle_t Door_Queue;

/* GLOBAL DATA */
static uint16_t current_illumination = 0U;
static uint8_t illumination_valid = 0U;
static uint16_t average_illumination = 0U;
static uint16_t illumination_threshold = 500U;

/* 0 = AUTOMATSKI, 1 = MANUELNI */
static uint8_t current_mode = 0U;

/* 0 = zatvorena, 1 = otvorena */
static uint8_t door_state = 0U;
static uint8_t blink_state = 0U;
static uint8_t kratka_timer_count = 0U;
static uint8_t kratka_timer_active = 0U;

/* RECEPTION BUFFERS */
static uint8_t drl_state = 0U;
static uint8_t kratka_state = 0U;

/* 7-SEGMENT */
static const uint8_t hexnum[] = { 0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F,0x77,0x7C,0x39,0x5E,0x79,0x71 };
static uint8_t display_memory[9] = { 0U };

typedef struct
{
    uint8_t broj_stuba;
    uint8_t broj_diode;
} LEDBarData;

/* INTERRUPTS */
static uint32_t OnLED_ChangeInterrupt(void)
{
    BaseType_t xHigherPTW = pdFALSE;

    xSemaphoreGiveFromISR(LED_INT_BinarySemaphore, &xHigherPTW);
    portYIELD_FROM_ISR(xHigherPTW);
}

static uint32_t prvProcessTBEInterrupt(void)
{
    BaseType_t xHigherPTW = pdFALSE;

    if (get_TBE_status(COM_CH_0) != 0) { xSemaphoreGiveFromISR(TBE_BinarySemaphore0, &xHigherPTW); }
    if (get_TBE_status(COM_CH_1) != 0) { xSemaphoreGiveFromISR(TBE_BinarySemaphore1, &xHigherPTW); }
    if (get_TBE_status(COM_CH_2) != 0) { xSemaphoreGiveFromISR(TBE_BinarySemaphore2, &xHigherPTW); }

    portYIELD_FROM_ISR(xHigherPTW);
}

static uint32_t prvProcessRXCInterrupt(void)
{
    BaseType_t xHigherPTW = pdFALSE;

    if (get_RXC_status(COM_CH_0) != 0) { xSemaphoreGiveFromISR(RXC_BinarySemaphore0, &xHigherPTW); }
    if (get_RXC_status(COM_CH_1) != 0) { xSemaphoreGiveFromISR(RXC_BinarySemaphore1, &xHigherPTW); }
    if (get_RXC_status(COM_CH_2) != 0) { xSemaphoreGiveFromISR(RXC_BinarySemaphore2, &xHigherPTW); }

    portYIELD_FROM_ISR(xHigherPTW);
}

/* TIMER CALLBACKS */
static void TimerCallback(TimerHandle_t xTimer)
{
    (void)xTimer;
    xSemaphoreGive(Trigger_BinarySemaphore);
}

static void BlinkTimerCallback(TimerHandle_t xTimer)
{
    (void)xTimer;

    if (blink_state == 0U)
    {
        blink_state = 1U;
    }
    else
    {
        blink_state = 0U;
    }

    if (kratka_timer_active != 0U)
    {
        kratka_timer_count++;

        if (kratka_timer_count >= 10U)
        {
            kratka_timer_count = 0U;
            kratka_timer_active = 0U;
            kratka_state = 0U;
        }
    }

    xSemaphoreGive(Blink_BinarySemaphore);
}

static void DisplayTimerCallback(TimerHandle_t xTimer)
{
    static uint8_t display_digit = 0U;

    (void)xTimer;

    select_7seg_digit(display_digit);
    set_7seg_digit(hexnum[display_memory[display_digit]]);

    display_digit++;

    if (display_digit >= 9U)
    {
        display_digit = 0U;
    }
}

/* HELPER FUNCTIONS */
static uint8_t LEDBar_DiodeToMask(uint8_t broj_diode)
{
    uint8_t mask = 0U;

    if ((broj_diode >= 1U) && (broj_diode <= 8U))
    {
        mask = (uint8_t)(1U << (broj_diode - 1U));
    }

    return mask;
}

/* MAIN */
void main_demo(void)
{
    TimerHandle_t per_TimerHandle;
    TimerHandle_t blink_TimerHandle;
    TimerHandle_t display_TimerHandle;
    const BaseType_t task_create_pass = (BaseType_t)1;

    /* PERIPHERALS */
    init_LED_comm();
    set_LED_BAR(1U, 0U);
    init_7seg_comm();

    init_serial_uplink(COM_CH_0);
    init_serial_downlink(COM_CH_0);

    init_serial_uplink(COM_CH_1);
    init_serial_downlink(COM_CH_1);

    init_serial_uplink(COM_CH_2);
    init_serial_downlink(COM_CH_2);

    select_7seg_digit(4);
    set_7seg_digit(hexnum[0]);

    /* INTERRUPTS */
    vPortSetInterruptHandler(portINTERRUPT_SRL_OIC, OnLED_ChangeInterrupt);
    vPortSetInterruptHandler(portINTERRUPT_SRL_TBE, prvProcessTBEInterrupt);
    vPortSetInterruptHandler(portINTERRUPT_SRL_RXC, prvProcessRXCInterrupt);

    /* SEMAPHORES & MUTEX */
    LED_INT_BinarySemaphore = xSemaphoreCreateBinary();
    TBE_BinarySemaphore0 = xSemaphoreCreateBinary();
    TBE_BinarySemaphore1 = xSemaphoreCreateBinary();
    TBE_BinarySemaphore2 = xSemaphoreCreateBinary();
    RXC_BinarySemaphore0 = xSemaphoreCreateBinary();
    RXC_BinarySemaphore1 = xSemaphoreCreateBinary();
    RXC_BinarySemaphore2 = xSemaphoreCreateBinary();
    Trigger_BinarySemaphore = xSemaphoreCreateBinary();
    Blink_BinarySemaphore = xSemaphoreCreateBinary();

    COM1_Mutex = xSemaphoreCreateMutex();

    /* QUEUES */
    Light_Queue = xQueueCreate(LIGHT_QUEUE_SIZE, sizeof(uint16_t));
    Door_Queue = xQueueCreate(DOOR_QUEUE_SIZE, sizeof(uint8_t));

    /* CHECK SEMAPHORES & MUTEX */
    if (LED_INT_BinarySemaphore == NULL) { for (;;) {} }
    if (TBE_BinarySemaphore0 == NULL) { for (;;) {} }
    if (TBE_BinarySemaphore1 == NULL) { for (;;) {} }
    if (TBE_BinarySemaphore2 == NULL) { for (;;) {} }

    if (RXC_BinarySemaphore0 == NULL) { for (;;) {} }
    if (RXC_BinarySemaphore1 == NULL) { for (;;) {} }
    if (RXC_BinarySemaphore2 == NULL) { for (;;) {} }

    if (Trigger_BinarySemaphore == NULL) { for (;;) {} }
    if (Blink_BinarySemaphore == NULL) { for (;;) {} }
    if (COM1_Mutex == NULL) { for (;;) {} }

    if (Light_Queue == NULL) { for (;;) {} }
    if (Door_Queue == NULL) { for (;;) {} }

    /* TAJMERI */
    per_TimerHandle = xTimerCreate("SensorTimer", pdMS_TO_TICKS(SENSOR_PERIOD_MS), pdTRUE, NULL, TimerCallback);
    if (per_TimerHandle == NULL) { for (;;) {} }
    (void)xTimerStart(per_TimerHandle, 0U);

    blink_TimerHandle = xTimerCreate("BlinkTimer", pdMS_TO_TICKS(500U), pdTRUE, NULL, BlinkTimerCallback);
    if (blink_TimerHandle == NULL) { for (;;) {} }
    (void)xTimerStart(blink_TimerHandle, 0U);

    display_TimerHandle = xTimerCreate("DisplayTimer", pdMS_TO_TICKS(DISPLAY_DIGIT_PERIOD_MS), pdTRUE, NULL, DisplayTimerCallback);
    if (display_TimerHandle == NULL) { for (;;) {} }
    (void)xTimerStart(display_TimerHandle, 0U);

    /* TASKOVI */
    if (xTaskCreate(SensorLightReceive_Task, "LightRx", configMINIMAL_STACK_SIZE, NULL, TASK_SERIAL_REC_PRI, NULL) != task_create_pass) { for (;;) {} }
    if (xTaskCreate(SensorDoorReceive_Task, "DoorRx", configMINIMAL_STACK_SIZE, NULL, TASK_SERIAL_REC_PRI, NULL) != task_create_pass) { for (;;) {} }
    if (xTaskCreate(SensorTrigger_Task, "Trigger", configMINIMAL_STACK_SIZE, NULL, TASK_TRIGGER_PRI, NULL) != task_create_pass) { for (;;) {} }
    if (xTaskCreate(PCReceive_Task, "PCRx", configMINIMAL_STACK_SIZE, NULL, TASK_PC_REC_PRI, NULL) != task_create_pass) { for (;;) {} }
    if (xTaskCreate(PCSend_Task, "PCTx", configMINIMAL_STACK_SIZE, NULL, TASK_PC_SEND_PRI, NULL) != task_create_pass) { for (;;) {} }
    if (xTaskCreate(DataProcessing_Task, "Processing", configMINIMAL_STACK_SIZE, NULL, TASK_DATA_PROC_PRI, NULL) != task_create_pass) { for (;;) {} }
    if (xTaskCreate(LCDDisplay_Task, "Display", configMINIMAL_STACK_SIZE, NULL, TASK_LCD_PRI, NULL) != task_create_pass) { for (;;) {} }
    if (xTaskCreate(LEDBar_Task, "LEDBar", configMINIMAL_STACK_SIZE, NULL, TASK_LED_PRI, NULL) != task_create_pass) { for (;;) {} }

    /* START SCHEDULER */
    vTaskStartScheduler();

    for (;;) {}
}

/* SENSOR LIGHT RECEIVE TASK */
static void SensorLightReceive_Task(void* pvParameters)
{
    uint8_t cc = 0U;
    uint16_t illumination = 0U;
    uint8_t light_point = 0U;
    uint8_t light_buffer[R_BUF_SIZE] = { 0U };
    const uint8_t buffer_limit = 31U;

    (void)pvParameters;

    memset(light_buffer, 0, R_BUF_SIZE);
    light_point = 0U;

    for (;;)
    {
        xSemaphoreTake(RXC_BinarySemaphore0, (TickType_t)portMAX_DELAY);

        if (get_serial_character(COM_CH_0, &cc) == 0)
        {
            if (cc == 0x0DU)
            {
                light_buffer[light_point] = (uint8_t)'\0';
                illumination = (uint16_t)atoi((const char*)light_buffer);

                if (illumination > 1000U) { illumination = 1000U; }

                xQueueSend(Light_Queue, &illumination, 0U);

                light_point = 0U;
                memset(light_buffer, 0, R_BUF_SIZE);
            }
            else if (light_point < buffer_limit)
            {
                light_buffer[light_point] = cc;
                light_point++;
            }
            else
            {
                light_point = 0U;
            }
        }
    }
}

/* SENSOR DOOR RECEIVE TASK */
static void SensorDoorReceive_Task(void* pvParameters)
{
    uint8_t cc = 0U;
    uint8_t door = 0U;
    uint8_t door_point = 0U;
    uint8_t door_buffer[R_BUF_SIZE] = { 0U };
    const uint8_t buffer_limit = 31U;

    (void)pvParameters;

    memset(door_buffer, 0, R_BUF_SIZE);
    door_point = 0U;

    for (;;)
    {
        xSemaphoreTake(RXC_BinarySemaphore1, (TickType_t)portMAX_DELAY);

        if (get_serial_character(COM_CH_1, &cc) == 0)
        {
            if (cc == 0x0DU)
            {
                door_buffer[door_point] = (uint8_t)'\0';
                door = (uint8_t)atoi((const char*)door_buffer);

                if (door != 0U) { door = 1U; }

                xQueueSend(Door_Queue, &door, 0U);

                door_point = 0U;
                memset(door_buffer, 0, R_BUF_SIZE);
            }
            else if (door_point < buffer_limit)
            {
                door_buffer[door_point] = cc;
                door_point++;
            }
            else
            {
                door_point = 0U;
            }
        }
    }
}

/* SENSOR TRIGGER TASK */
static void SensorTrigger_Task(void* pvParameters)
{
    static const char trigger[] = "t";
    uint8_t i = 0U;

    (void)pvParameters;

    for (;;)
    {
        xSemaphoreTake(Trigger_BinarySemaphore, (TickType_t)portMAX_DELAY);

        /* CHANNEL 0 - SVJETLO */
        for (i = 0U; i < (uint8_t)(sizeof(trigger) - 1U); i++)
        {
            send_serial_character(COM_CH_0, (uint8_t)trigger[i]);
            xSemaphoreTake(TBE_BinarySemaphore0, (TickType_t)portMAX_DELAY);
        }

        /* CHANNEL 1 - VRATA (Zasticeno Mutex-om) */
        if (xSemaphoreTake(COM1_Mutex, (TickType_t)portMAX_DELAY) == pdTRUE)
        {
            for (i = 0U; i < (uint8_t)(sizeof(trigger) - 1U); i++)
            {
                send_serial_character(COM_CH_1, (uint8_t)trigger[i]);
                xSemaphoreTake(TBE_BinarySemaphore1, (TickType_t)portMAX_DELAY);
            }
            xSemaphoreGive(COM1_Mutex);
        }
    }
}

/* PC RECEIVE TASK */
static void PCReceive_Task(void* pvParameters)
{
    uint8_t cc = 0U;
    uint8_t pc_point = 0U;
    uint8_t pc_buffer[R_BUF_SIZE] = { 0U };
    const uint8_t buffer_limit = 31U;

    (void)pvParameters;

    memset(pc_buffer, 0, R_BUF_SIZE);
    pc_point = 0U;

    for (;;)
    {
        xSemaphoreTake(RXC_BinarySemaphore2, (TickType_t)portMAX_DELAY);

        if (get_serial_character(COM_CH_2, &cc) == 0)
        {
            if (cc == 0x0DU)
            {
                pc_buffer[pc_point] = (uint8_t)'\0';

                /* PRAG */
                if (strncmp((const char*)pc_buffer, "PRAG", 4U) == 0)
                {
                    illumination_threshold = (uint16_t)atoi((const char*)&pc_buffer[4]);
                    if (illumination_threshold > 1000U) { illumination_threshold = 1000U; }
                }
                /* MANUELNO */
                else if (strcmp((const char*)pc_buffer, "MANUELNO") == 0)
                {
                    current_mode = 1U;

                    send_serial_character(COM_CH_2, (uint8_t)'O');
                    xSemaphoreTake(TBE_BinarySemaphore2, (TickType_t)portMAX_DELAY);

                    send_serial_character(COM_CH_2, (uint8_t)'K');
                    xSemaphoreTake(TBE_BinarySemaphore2, (TickType_t)portMAX_DELAY);
                }
                /* AUTOMATSKI */
                else if (strcmp((const char*)pc_buffer, "AUTOMATSKI") == 0)
                {
                    current_mode = 0U;

                    send_serial_character(COM_CH_2, (uint8_t)'O');
                    xSemaphoreTake(TBE_BinarySemaphore2, (TickType_t)portMAX_DELAY);

                    send_serial_character(COM_CH_2, (uint8_t)'K');
                    xSemaphoreTake(TBE_BinarySemaphore2, (TickType_t)portMAX_DELAY);
                }

                pc_point = 0U;
                memset(pc_buffer, 0, R_BUF_SIZE);
            }
            else if (pc_point < buffer_limit)
            {
                pc_buffer[pc_point] = cc;
                pc_point++;
            }
            else
            {
                pc_point = 0U;
            }
        }
    }
}

/* PC SEND TASK */
static void PCSend_Task(void* pvParameters)
{
    char message[64];
    uint8_t i;
    uint16_t length = 0U;

    (void)pvParameters;

    for (;;)
    {
        length = (uint16_t)sprintf(message, "OSVJETLJENJE:%u MOD:%u\r\n", (uint32_t)average_illumination, (uint32_t)current_mode);

        /* Zasticeno Mutex-om zbog dijeljenja COM_CH_1 sa SensorTrigger_Task */
        if (xSemaphoreTake(COM1_Mutex, (TickType_t)portMAX_DELAY) == pdTRUE)
        {
            for (i = 0U; i < (uint8_t)length; i++)
            {
                send_serial_character(COM_CH_1, (uint8_t)message[i]);
                xSemaphoreTake(TBE_BinarySemaphore1, (TickType_t)portMAX_DELAY);
            }
            xSemaphoreGive(COM1_Mutex);
        }

        vTaskDelay(pdMS_TO_TICKS(PC_SEND_PERIOD_MS));
    }
}

/* DATA PROCESSING TASK */
static void DataProcessing_Task(void* pvParameters)
{
    uint16_t new_illumination = 0U;
    uint8_t new_door_state = 0U;
    uint16_t illumination_samples[LIGHT_AVG_SAMPLES];
    uint8_t sample_index = 0U;
    uint8_t sample_count = 0U;
    uint32_t sum = 0U;
    uint8_t i;

    (void)pvParameters;

    for (i = 0U; i < (uint8_t)LIGHT_AVG_SAMPLES; i++) { illumination_samples[i] = 0U; }

    for (;;)
    {
        if (xQueueReceive(Light_Queue, &new_illumination, pdMS_TO_TICKS(10)) == pdPASS)
        {
            current_illumination = new_illumination;
            illumination_valid = 1U;

            illumination_samples[sample_index] = new_illumination;
            sample_index++;

            if (sample_index >= (uint8_t)LIGHT_AVG_SAMPLES) { sample_index = 0U; }
            if (sample_count < (uint8_t)LIGHT_AVG_SAMPLES) { sample_count++; }

            sum = 0U;
            for (i = 0U; i < sample_count; i++) { sum += illumination_samples[i]; }

            if (sample_count > 0U)
            {
                average_illumination = (uint16_t)(sum / sample_count);
            }

            if (current_mode == 0U)
            {
                if (average_illumination > illumination_threshold)
                {
                    drl_state = 1U;

                    if ((kratka_state != 0U) && (kratka_timer_active == 0U))
                    {
                        kratka_timer_count = 0U;
                        kratka_timer_active = 1U;
                    }
                }
                else
                {
                    drl_state = 0U;
                    kratka_state = 1U;
                    kratka_timer_count = 0U;
                    kratka_timer_active = 0U;
                }
            }
        }

        if (xQueueReceive(Door_Queue, &new_door_state, 0U) == pdPASS)
        {
            door_state = new_door_state;
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

/* DISPLAY TASK */
static void LCDDisplay_Task(void* pvParameters)
{
    uint16_t value = 0U;
    static uint16_t min_illumination = 1000U;
    static uint16_t max_illumination = 0U;
    uint8_t d = 0U;
    uint16_t value_for_min_max = 0U;

    (void)pvParameters;

    for (;;)
    {
        if (illumination_valid != 0U)
        {
            if (current_illumination < min_illumination)
            {
                min_illumination = current_illumination;
            }

            if (current_illumination > max_illumination)
            {
                max_illumination = current_illumination;
            }
        }

        value = current_illumination;

        (void)get_LED_BAR(0U, &d);
        if ((d & 0x80U) != 0U)
        {
            value_for_min_max = max_illumination;
        }
        else
        {
            value_for_min_max = min_illumination;
        }

        display_memory[0] = (uint8_t)((value / 1000U) % 10U);
        display_memory[1] = (uint8_t)((value / 100U) % 10U);
        display_memory[2] = (uint8_t)((value / 10U) % 10U);
        display_memory[3] = (uint8_t)(value % 10U);
        display_memory[4] = current_mode;

        display_memory[5] = (uint8_t)((value_for_min_max / 1000U) % 10U);
        display_memory[6] = (uint8_t)((value_for_min_max / 100U) % 10U);
        display_memory[7] = (uint8_t)((value_for_min_max / 10U) % 10U);
        display_memory[8] = (uint8_t)(value_for_min_max % 10U);

        vTaskDelay(pdMS_TO_TICKS(DISPLAY_PERIOD_MS));
    }
}

/* LED BAR TASK */
static void LEDBar_Task(void* pvParameters)
{
    const LEDBarData DRL_input = { 0U, 1U };
    const LEDBarData DRL_output = { 1U, 8U };
    const LEDBarData kratka_input = { 0U, 2U };
    const LEDBarData kratka_output = { 1U, 7U };
    const LEDBarData duga_input = { 0U, 3U };
    const LEDBarData duga_output = { 1U, 6U };
    const LEDBarData lijevi_input = { 0U, 4U };
    const LEDBarData lijevi_output = { 1U, 5U };
    const LEDBarData desni_input = { 0U, 5U };
    const LEDBarData desni_output = { 1U, 4U };
    const LEDBarData kabina_input = { 0U, 7U };
    const LEDBarData kabina_output = { 1U, 1U };
    uint8_t d = 0U;
    uint8_t output = 0U;

    (void)pvParameters;

    for (;;)
    {
        (void)xSemaphoreTake(LED_INT_BinarySemaphore, 0U);
        (void)xSemaphoreTake(Blink_BinarySemaphore, 0U);

        (void)get_LED_BAR(DRL_input.broj_stuba, &d);
        output = 0U;

        if (current_mode == 1U)
        {
            if ((d & LEDBar_DiodeToMask(DRL_input.broj_diode)) != 0U)
            {
                output |= LEDBar_DiodeToMask(DRL_output.broj_diode);
            }

            if ((d & LEDBar_DiodeToMask(kratka_input.broj_diode)) != 0U)
            {
                output |= LEDBar_DiodeToMask(kratka_output.broj_diode);
            }

            if ((d & LEDBar_DiodeToMask(duga_input.broj_diode)) != 0U)
            {
                output |= LEDBar_DiodeToMask(duga_output.broj_diode);
            }

            if ((d & LEDBar_DiodeToMask(lijevi_input.broj_diode)) != 0U)
            {
                if (blink_state != 0U)
                {
                    output |= LEDBar_DiodeToMask(lijevi_output.broj_diode);
                }
            }

            if ((d & LEDBar_DiodeToMask(desni_input.broj_diode)) != 0U)
            {
                if (blink_state != 0U)
                {
                    output |= LEDBar_DiodeToMask(desni_output.broj_diode);
                }
            }

            if ((d & LEDBar_DiodeToMask(kabina_input.broj_diode)) != 0U)
            {
                output |= LEDBar_DiodeToMask(kabina_output.broj_diode);
            }
        }
        else
        {
            if (drl_state != 0U)
            {
                output |= LEDBar_DiodeToMask(DRL_output.broj_diode);
            }

            if (kratka_state != 0U)
            {
                output |= LEDBar_DiodeToMask(kratka_output.broj_diode);
            }

            if ((d & LEDBar_DiodeToMask(duga_input.broj_diode)) != 0U)
            {
                output |= LEDBar_DiodeToMask(duga_output.broj_diode);
            }

            if ((d & LEDBar_DiodeToMask(lijevi_input.broj_diode)) != 0U)
            {
                if (blink_state != 0U)
                {
                    output |= LEDBar_DiodeToMask(lijevi_output.broj_diode);
                }
            }

            if ((d & LEDBar_DiodeToMask(desni_input.broj_diode)) != 0U)
            {
                if (blink_state != 0U)
                {
                    output |= LEDBar_DiodeToMask(desni_output.broj_diode);
                }
            }

            if (door_state != 0U)
            {
                output |= LEDBar_DiodeToMask(kabina_output.broj_diode);
            }
        }

        (void)set_LED_BAR(DRL_output.broj_stuba, output);

        vTaskDelay(pdMS_TO_TICKS(10U));
    }
}