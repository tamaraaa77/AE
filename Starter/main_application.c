/* STANDARD INCLUDES */
#include <stdio.h>
#include <conio.h>
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

#define R_BUF_SIZE          (32)
#define LIGHT_QUEUE_SIZE    (10)
#define DOOR_QUEUE_SIZE     (10)
#define LIGHT_AVG_SAMPLES   (10)
#define SENSOR_PERIOD_MS    (200)
#define PC_SEND_PERIOD_MS   (2000)
#define DISPLAY_PERIOD_MS   (1500)

const char trigger[] = "t";


/* TASK FORWARD DECLARATIONS */

void SensorLightReceive_Task(void* pvParameters);
void SensorDoorReceive_Task(void* pvParameters);
void SensorTrigger_Task(void* pvParameters);
void PCReceive_Task(void* pvParameters);
void PCSend_Task(void* pvParameters);
void DataProcessing_Task(void* pvParameters);
void LCDDisplay_Task(void* pvParameters);
void LEDBar_Task(void* pvParameters);


/* GLOBAL OS HANDLES */

SemaphoreHandle_t LED_INT_BinarySemaphore;
SemaphoreHandle_t TBE_BinarySemaphore0;
SemaphoreHandle_t TBE_BinarySemaphore1;
SemaphoreHandle_t TBE_BinarySemaphore2;
SemaphoreHandle_t RXC_BinarySemaphore0;
SemaphoreHandle_t RXC_BinarySemaphore1;
SemaphoreHandle_t RXC_BinarySemaphore2;
SemaphoreHandle_t Trigger_BinarySemaphore;
SemaphoreHandle_t Blink_BinarySemaphore;

QueueHandle_t Light_Queue;
QueueHandle_t Door_Queue;

TimerHandle_t per_TimerHandle;

/* GLOBAL DATA */

static uint16_t current_illumination = 0U;
static uint16_t average_illumination = 0U;
static uint16_t minimum_illumination = 1000U;
static uint16_t maximum_illumination = 0U;
static uint16_t illumination_threshold = 500U;

/* 0 = AUTOMATSKI, 1 = MANUELNI */
static uint8_t current_mode = 0U;

/* 0 = zatvorena, 1 = otvorena */
static uint8_t door_state = 0U;
static uint8_t blink_state = 0U;
static TimerHandle_t blink_TimerHandle;

/* RECEPTION BUFFERS */

static uint8_t light_buffer[R_BUF_SIZE];
static uint8_t door_buffer[R_BUF_SIZE];
static uint8_t pc_buffer[R_BUF_SIZE];
static uint8_t light_point = 0U;
static uint8_t door_point = 0U;
static uint8_t pc_point = 0U;
static uint8_t drl_state = 0U;
static uint8_t kratka_state = 0U;

/* 7-SEGMENT */

static const uint8_t hexnum[] = { 0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F,0x77,0x7C,0x39,0x5E,0x79,0x71 };


typedef struct
{
    uint8_t broj_stuba;
    uint8_t broj_diode;
} LEDBarData;

LEDBarData DRL_input;
LEDBarData DRL_output;
LEDBarData kratka_input;
LEDBarData kratka_output;
LEDBarData duga_input;
LEDBarData duga_output;
LEDBarData lijevi_input;
LEDBarData lijevi_output;
LEDBarData desni_input;
LEDBarData desni_output;
LEDBarData kabina_input;
LEDBarData kabina_output;

LEDBarData DRL_input = { 0U, 1U };
LEDBarData DRL_output = { 1U, 8U };

LEDBarData kratka_input = { 0U, 2U };
LEDBarData kratka_output = { 1U, 7U };

LEDBarData duga_input = { 0U, 3U };
LEDBarData duga_output = { 1U, 6U };

LEDBarData lijevi_input = { 0U, 4U };
LEDBarData lijevi_output = { 1U, 5U };

LEDBarData desni_input = { 0U, 5U };
LEDBarData desni_output = { 1U, 4U };

LEDBarData kabina_input = { 0U, 7U };
LEDBarData kabina_output = { 1U, 1U };

/* INTERRUPTS */

static uint32_t OnLED_ChangeInterrupt(void)
{
    BaseType_t xHigherPTW = pdFALSE;

    xSemaphoreGiveFromISR(LED_INT_BinarySemaphore, &xHigherPTW);
    portYIELD_FROM_ISR(xHigherPTW);

    return 0U;
}


static uint32_t prvProcessTBEInterrupt(void)
{
    BaseType_t xHigherPTW = pdFALSE;

    if (get_TBE_status(COM_CH_0) != 0) { xSemaphoreGiveFromISR(TBE_BinarySemaphore0, &xHigherPTW); }
    if (get_TBE_status(COM_CH_1) != 0) { xSemaphoreGiveFromISR(TBE_BinarySemaphore1, &xHigherPTW); }
    if (get_TBE_status(COM_CH_2) != 0) { xSemaphoreGiveFromISR(TBE_BinarySemaphore2, &xHigherPTW); }

    portYIELD_FROM_ISR(xHigherPTW);

    return 0U;
}


static uint32_t prvProcessRXCInterrupt(void)
{
    BaseType_t xHigherPTW = pdFALSE;

    if (get_RXC_status(COM_CH_0) != 0) { xSemaphoreGiveFromISR(RXC_BinarySemaphore0, &xHigherPTW); }
    if (get_RXC_status(COM_CH_1) != 0) { xSemaphoreGiveFromISR(RXC_BinarySemaphore1, &xHigherPTW); }
    if (get_RXC_status(COM_CH_2) != 0) { xSemaphoreGiveFromISR(RXC_BinarySemaphore2, &xHigherPTW); }

    portYIELD_FROM_ISR(xHigherPTW);

    return 0U;
}


/* TIMER CALLBACK */

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

    xSemaphoreGive(Blink_BinarySemaphore);
}


/* MAIN */
static uint8_t LEDBar_DiodeToMask(uint8_t broj_diode)
{
    if ((broj_diode >= 1U) && (broj_diode <= 8U))
    {
        return (uint8_t)(1U << (broj_diode - 1U));
    }

    return 0U;
}

void main_demo(void)
{
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


    /* SEMAPHORES */

    LED_INT_BinarySemaphore = xSemaphoreCreateBinary();
    TBE_BinarySemaphore0 = xSemaphoreCreateBinary();
    TBE_BinarySemaphore1 = xSemaphoreCreateBinary();
    TBE_BinarySemaphore2 = xSemaphoreCreateBinary();
    RXC_BinarySemaphore0 = xSemaphoreCreateBinary();
    RXC_BinarySemaphore1 = xSemaphoreCreateBinary();
    RXC_BinarySemaphore2 = xSemaphoreCreateBinary();
    Trigger_BinarySemaphore = xSemaphoreCreateBinary();
    Blink_BinarySemaphore = xSemaphoreCreateBinary();

    /* QUEUES */

    Light_Queue = xQueueCreate(LIGHT_QUEUE_SIZE, sizeof(uint16_t));
    Door_Queue = xQueueCreate(DOOR_QUEUE_SIZE, sizeof(uint8_t));


    /* CHECK SEMAPHORES */

    if (LED_INT_BinarySemaphore == NULL) { while (1); }
    if (TBE_BinarySemaphore0 == NULL) { while (1); }
    if (TBE_BinarySemaphore1 == NULL) { while (1); }
    if (TBE_BinarySemaphore2 == NULL) { while (1); }
    if (RXC_BinarySemaphore0 == NULL) { while (1); }
    if (RXC_BinarySemaphore1 == NULL) { while (1); }
    if (RXC_BinarySemaphore2 == NULL) { while (1); }
    if (Trigger_BinarySemaphore == NULL) { while (1); }
    if (Blink_BinarySemaphore == NULL) { while (1); }

    /* CHECK QUEUES */

    if (Light_Queue == NULL) { while (1); }
    if (Door_Queue == NULL) { while (1); }


    /* TAJMERI */

    per_TimerHandle = xTimerCreate("SensorTimer", pdMS_TO_TICKS(SENSOR_PERIOD_MS), pdTRUE, NULL, TimerCallback);
    if (per_TimerHandle == NULL) { while (1); }
    xTimerStart(per_TimerHandle, 0U);
   
    blink_TimerHandle = xTimerCreate("BlinkTimer", pdMS_TO_TICKS(500U), pdTRUE, NULL, BlinkTimerCallback);
    if (blink_TimerHandle == NULL) { while (1); }
    xTimerStart(blink_TimerHandle, 0U);

    /* TASKOVI */

    if (xTaskCreate(SensorLightReceive_Task, "LightRx", configMINIMAL_STACK_SIZE, NULL, TASK_SERIAL_REC_PRI, NULL) != pdPASS)
    { while (1); }
    if (xTaskCreate(SensorDoorReceive_Task, "DoorRx", configMINIMAL_STACK_SIZE, NULL, TASK_SERIAL_REC_PRI, NULL) != pdPASS) 
    { while (1); }
    if (xTaskCreate(SensorTrigger_Task, "Trigger", configMINIMAL_STACK_SIZE, NULL, TASK_TRIGGER_PRI, NULL) != pdPASS)
    { while (1); }
    if (xTaskCreate(PCReceive_Task, "PCRx", configMINIMAL_STACK_SIZE, NULL, TASK_PC_REC_PRI, NULL) != pdPASS)
    { while (1); }
    if (xTaskCreate(PCSend_Task, "PCTx", configMINIMAL_STACK_SIZE, NULL, TASK_PC_SEND_PRI, NULL) != pdPASS)
    { while (1); }
    if (xTaskCreate(DataProcessing_Task, "Processing", configMINIMAL_STACK_SIZE, NULL, TASK_DATA_PROC_PRI, NULL) != pdPASS)
    { while (1); }
    if (xTaskCreate(LCDDisplay_Task, "Display", configMINIMAL_STACK_SIZE, NULL, TASK_LCD_PRI, NULL) != pdPASS) 
    { while (1); }
    if (xTaskCreate(LEDBar_Task, "LEDBar", configMINIMAL_STACK_SIZE, NULL, TASK_LED_PRI, NULL) != pdPASS) 
    { while (1); }


    /* START SCHEDULER */

    vTaskStartScheduler();

    while (1) {}
}


/* SENSOR LIGHT RECEIVE TASK */

void SensorLightReceive_Task(void* pvParameters)
{
    uint8_t cc = 0U;
    uint16_t illumination = 0U;

    (void)pvParameters;

    memset(light_buffer, 0, R_BUF_SIZE);
    light_point = 0U;

    while (1)
    {
        xSemaphoreTake(RXC_BinarySemaphore0, portMAX_DELAY);

        if (get_serial_character(COM_CH_0, &cc) == 0)
        {
            if (cc == 0x0DU)
            {
                light_buffer[light_point] = '\0';

                illumination = (uint16_t)atoi((const char*)light_buffer);

                if (illumination > 1000U) { illumination = 1000U; }

                printf("SVJETLO: %u\n", (unsigned)illumination);

                xQueueSend(Light_Queue, &illumination, 0U);

                light_point = 0U;
                memset(light_buffer, 0, R_BUF_SIZE);
            }
            else if (light_point < (uint8_t)(R_BUF_SIZE - 1U))
            {
                light_buffer[light_point] = cc;
                light_point++;
            }
        }
    }
}


/* SENSOR DOOR RECEIVE TASK */

void SensorDoorReceive_Task(void* pvParameters)
{
    uint8_t cc = 0U;
    uint8_t door = 0U;

    (void)pvParameters;

    memset(door_buffer, 0, R_BUF_SIZE);
    door_point = 0U;

    while (1)
    {
        xSemaphoreTake(RXC_BinarySemaphore1, portMAX_DELAY);

        if (get_serial_character(COM_CH_1, &cc) == 0)
        {
            if (cc == 0x0DU)
            {
                door_buffer[door_point] = '\0';

                door = (uint8_t)atoi((const char*)door_buffer);

                if (door != 0U) { door = 1U; }

                xQueueSend(Door_Queue, &door, 0U);

                door_point = 0U;
                memset(door_buffer, 0, R_BUF_SIZE);
            }
            else if (door_point < (uint8_t)(R_BUF_SIZE - 1U))
            {
                door_buffer[door_point] = cc;
                door_point++;
            }
        }
    }
}


/* SENSOR TRIGGER TASK */

void SensorTrigger_Task(void* pvParameters)
{
    uint8_t i = 0U;

    (void)pvParameters;

    while (1)
    {
        xSemaphoreTake(Trigger_BinarySemaphore, portMAX_DELAY);

        /* CHANNEL 0 - SVJETLO */

        for (i = 0U; i < (uint8_t)(sizeof(trigger) - 1U); i++)
        {
            send_serial_character(COM_CH_0, (uint8_t)trigger[i]);
            xSemaphoreTake(TBE_BinarySemaphore0, portMAX_DELAY);
        }

        /* CHANNEL 1 - VRATA */

        for (i = 0U; i < (uint8_t)(sizeof(trigger) - 1U); i++)
        {
            send_serial_character(COM_CH_1, (uint8_t)trigger[i]);
            xSemaphoreTake(TBE_BinarySemaphore1, portMAX_DELAY);
        }
    }
}


/* PC RECEIVE TASK */

void PCReceive_Task(void* pvParameters)
{
    uint8_t cc = 0U;

    (void)pvParameters;

    memset(pc_buffer, 0, R_BUF_SIZE);
    pc_point = 0U;

    while (1)
    {
        xSemaphoreTake(RXC_BinarySemaphore2, portMAX_DELAY);

        if (get_serial_character(COM_CH_2, &cc) == 0)
        {
            if (cc == 0x0DU)
            {
                pc_buffer[pc_point] = '\0';

                printf("PC komanda: %s\n", (char*)pc_buffer);

                /* PRAG */

                if (strncmp((const char*)pc_buffer, "PRAG", 4U) == 0)
                {
                    illumination_threshold = (uint16_t)atoi((const char*)&pc_buffer[4]);

                    if (illumination_threshold > 1000U) { illumination_threshold = 1000U; }

                    printf("Novi prag: %u\n", (unsigned)illumination_threshold);
                }

                /* MANUELNO */

                else if (strcmp((const char*)pc_buffer, "MANUELNO") == 0)
                {
                    current_mode = 1U;

                    printf("MANUELNI MOD\n");

                    send_serial_character(COM_CH_2, (uint8_t)'O');
                    xSemaphoreTake(TBE_BinarySemaphore2, portMAX_DELAY);

                    send_serial_character(COM_CH_2, (uint8_t)'K');
                    xSemaphoreTake(TBE_BinarySemaphore2, portMAX_DELAY);
                }

                /* AUTOMATSKI */

                else if (strcmp((const char*)pc_buffer, "AUTOMATSKI") == 0)
                {
                    current_mode = 0U;

                    printf("AUTOMATSKI MOD\n");

                    send_serial_character(COM_CH_2, (uint8_t)'O');
                    xSemaphoreTake(TBE_BinarySemaphore2, portMAX_DELAY);

                    send_serial_character(COM_CH_2, (uint8_t)'K');
                    xSemaphoreTake(TBE_BinarySemaphore2, portMAX_DELAY);
                }

                pc_point = 0U;
                memset(pc_buffer, 0, R_BUF_SIZE);
            }
            else if (pc_point < (uint8_t)(R_BUF_SIZE - 1U))
            {
                pc_buffer[pc_point] = cc;
                pc_point++;
            }
        }
    }
}


/* PC SEND TASK */

void PCSend_Task(void* pvParameters)
{
    char message[64];
    uint8_t i = 0U;
    int length = 0;

    (void)pvParameters;

    while (1)
    {
        length = sprintf(message, "OSVJETLJENJE:%u MOD:%u\r\n", (unsigned)average_illumination, (unsigned)current_mode);

        for (i = 0U; i < (uint8_t)length; i++)
        {
            send_serial_character(COM_CH_1, (uint8_t)message[i]);
            xSemaphoreTake(TBE_BinarySemaphore1, portMAX_DELAY);
        }

        vTaskDelay(pdMS_TO_TICKS(PC_SEND_PERIOD_MS));
    }
}


/* DATA PROCESSING TASK */

void DataProcessing_Task(void* pvParameters)
{
    uint16_t new_illumination = 0U;
    uint8_t new_door_state = 0U;
    uint16_t illumination_samples[LIGHT_AVG_SAMPLES];
    uint8_t sample_index = 0U;
    uint8_t sample_count = 0U;
    uint32_t sum = 0U;
    uint8_t i = 0U;

    (void)pvParameters;

    for (i = 0U; i < LIGHT_AVG_SAMPLES; i++) { illumination_samples[i] = 0U; }

    while (1)
    {
        if (xQueueReceive(Light_Queue, &new_illumination, pdMS_TO_TICKS(10)) == pdPASS)
        {
            current_illumination = new_illumination;

            if (new_illumination < minimum_illumination) { minimum_illumination = new_illumination; }
            if (new_illumination > maximum_illumination) { maximum_illumination = new_illumination; }

            illumination_samples[sample_index] = new_illumination;
            sample_index++;

            if (sample_index >= LIGHT_AVG_SAMPLES) { sample_index = 0U; }
            if (sample_count < LIGHT_AVG_SAMPLES) { sample_count++; }

            sum = 0U;

            for (i = 0U; i < sample_count; i++) { sum += illumination_samples[i]; }

            if (sample_count > 0U)
            {
                average_illumination = (uint16_t)(sum / sample_count);
                printf("AVG: %u\n", (unsigned)average_illumination);
            }
            if (current_mode == 0U)
            {
                if (average_illumination > illumination_threshold)
                {
                    drl_state = 1U;
                    kratka_state = 0U;
                }
                else
                {
                    drl_state = 0U;
                    kratka_state = 1U;
                }
            }
        }

        if (xQueueReceive(Door_Queue, &new_door_state, 0U) == pdPASS)
        {
            door_state = new_door_state;
            printf("Vrata: %u\n", (unsigned)door_state);
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}


/* DISPLAY TASK */

void LCDDisplay_Task(void* pvParameters)
{
    uint16_t value = 0U;
    uint8_t digit0 = 0U;
    uint8_t digit1 = 0U;
    uint8_t digit2 = 0U;
    uint8_t digit3 = 0U;

    (void)pvParameters;

    while (1)
    {
        value = average_illumination;

        digit0 = (uint8_t)(value % 10U);
        digit1 = (uint8_t)((value / 10U) % 10U);
        digit2 = (uint8_t)((value / 100U) % 10U);
        digit3 = (uint8_t)((value / 1000U) % 10U);

        select_7seg_digit(1);
        set_7seg_digit(hexnum[digit0]);
        vTaskDelay(pdMS_TO_TICKS(3));

        select_7seg_digit(2);
        set_7seg_digit(hexnum[digit1]);
        vTaskDelay(pdMS_TO_TICKS(3));

        select_7seg_digit(3);
        set_7seg_digit(hexnum[digit2]);
        vTaskDelay(pdMS_TO_TICKS(3));

        select_7seg_digit(4);
        set_7seg_digit(hexnum[digit3]);

        vTaskDelay(pdMS_TO_TICKS(DISPLAY_PERIOD_MS));
    }
}


/* LED BAR TASK */
void LEDBar_Task(void* pvParameters)
{
    uint8_t d = 0U;
    uint8_t output = 0U;

    (void)pvParameters;

    while (1)
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
        }

        (void)set_LED_BAR(DRL_output.broj_stuba, output);

        vTaskDelay(pdMS_TO_TICKS(10U));
    }
}