/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
//#include "main.h"
//#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
#include "app_include.h"


/* Private typedef -----------------------------------------------------------*/


/* Private define ------------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/
osMutexId mutex_iic0Handle;
osMutexId mutex_iic1Handle;
osMutexId mutex_printfHandle;
osMutexId mtx_batctrl;
osMutexId mtx_batchg;
osMutexId mtx_canbuf;
osSemaphoreId sem_consoleHandle;
osSemaphoreId sem_canprint;
osSemaphoreId csemHandle;
QueueHandle_t q_canmsg;
QueueHandle_t q_uartmsg;
// TimerHandle_t timer_fastboot;
// TimerHandle_t timer_gcsleep;
TimerHandle_t timer_pump1_start;
TimerHandle_t timer_pump1_stop;
TimerHandle_t timer_pump2_start;
TimerHandle_t timer_pump2_stop;
TimerHandle_t timer_pvalve_start;
TimerHandle_t timer_pvalve_stop;
TimerHandle_t timer_heater;

uint32_t seconds = 0;
uint32_t minutes = 0;

uint8_t monitor_sw = false;


/* Private function prototypes -----------------------------------------------*/
void StartDefaultTask(void const *argument);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */
void DebugTask(void *argument);
void Thread_UartCmd(void *argument);

/* Pre/Post sleep processing prototypes */
void PreSleepProcessing(uint32_t *ulExpectedIdleTime);
void PostSleepProcessing(uint32_t *ulExpectedIdleTime);

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);
void vApplicationIdleHook(void);
void vApplicationTickHook(void);
void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);
void vApplicationDaemonTaskStartupHook(void);

/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{
}

__weak unsigned long getRunTimeCounterValue(void)
{
    return 0;
}

__weak void vApplicationIdleHook(void)
{
    /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
    to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
    task. It is essential that code added to this hook function never attempts
    to block in any way (for example, call xQueueReceive() with a block time
    specified, or call vTaskDelay()). If the application makes use of the
    vTaskDelete() API function (as this demo application does) then it is also
    important that vApplicationIdleHook() is permitted to return to its calling
    function, because it is the responsibility of the idle task to clean up
    memory allocated by the kernel to any task that has since been deleted. */
}

__weak void vApplicationTickHook(void)
{
    /* This function will be called by each tick interrupt if
    configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h. User code can be
    added here, but the tick hook is called from an interrupt context, so
    code must not attempt to block, and only the interrupt safe FreeRTOS API
    functions can be used (those that end in FromISR()). */
    static uint32_t tick;

    tick++;
    if (tick >= 1000) {
        seconds++;
        tick = 0;
        //HAL_GPIO_TogglePin(LED_WHITE_GPIO_Port, LED_WHITE_Pin);
        //HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
        //HAL_IWDG_Refresh(&hiwdg);
        if (seconds % 60 == 0)
            minutes++;
    }
}

__weak void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char *pcTaskName)
{
    /* Run time stack overflow checking is performed if
    configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
    called if a stack overflow is detected. */
    xprintf("stack overflow: %s\n\r", pcTaskName);
}

__weak void vApplicationMallocFailedHook(void)
{
    /* vApplicationMallocFailedHook() will only be called if
    configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
    function that will get called if a call to pvPortMalloc() fails.
    pvPortMalloc() is called internally by the kernel whenever a task, queue,
    timer or semaphore is created. It is also called by various parts of the
    demo application. If heap_1.c or heap_2.c are used, then the size of the
    heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
    FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
    to query the size of free heap space that remains (although it does not
    provide information on how the remaining heap might be fragmented). */
    TaskHandle_t tid = xTaskGetCurrentTaskHandle();
    char *s = pcTaskGetName(tid);
    xprintf("malloc fail, %s\n\r", s);
}

void vApplicationDaemonTaskStartupHook(void)
{
    Pump_1.timer_start = timer_pump1_start;
    Pump_1.timer_stop = timer_pump1_stop;
    Pump_2.timer_start = timer_pump2_start;
    Pump_2.timer_stop = timer_pump2_stop;
    PValve.timer_start = timer_pvalve_start;
    PValve.timer_stop = timer_pvalve_stop;

    ADC_Config();
    Sdadc_Config();
    CAN_Config();

    Battery_HwInit();
    CAN_Listen();

    // xTimerStop(timer_pump1_start, portMAX_DELAY);
    // xTimerStop(timer_pump1_stop, portMAX_DELAY);
    // xTimerStop(timer_pump2_start, portMAX_DELAY);
    // xTimerStop(timer_pump2_stop, portMAX_DELAY);
    // xTimerStop(timer_pvalve_start, portMAX_DELAY);
    // xTimerStop(timer_pvalve_stop, portMAX_DELAY);
    xTimerStart(timer_heater, portMAX_DELAY);

    xTaskCreate(thread_adaptor, "adaptor", 216, NULL, osPriorityNormal, NULL);
    xTaskCreate(Thread_FieldcaseInfoUpdate, "fieldcase", 216, NULL, osPriorityNormal, NULL);
    xTaskCreate(Thread_GcPwrCntl, "gc_pwr", 216, NULL, osPriorityNormal, NULL);
    xTaskCreate(Thread_CbPwrCntl, "cb_pwr", 216, NULL, osPriorityNormal, NULL);
    xTaskCreate((TaskFunction_t)Thread_BatteryMonitor, "batmon1", 216, &Battery_1, osPriorityNormal, NULL);
    xTaskCreate((TaskFunction_t)Thread_BatteryMonitor, "batmon2", 216, &Battery_2, osPriorityNormal, NULL);
    xTaskCreate((TaskFunction_t)Thread_BatteryCharge, "batchg1", 216, &Battery_1, osPriorityNormal, NULL);
    xTaskCreate((TaskFunction_t)Thread_BatteryCharge, "batchg2", 216, &Battery_2, osPriorityNormal, NULL);
    xTaskCreate((TaskFunction_t)Thread_BatterySupply, "batsply1", 216, &Battery_1, osPriorityNormal, NULL);
    xTaskCreate((TaskFunction_t)Thread_BatterySupply, "batsply2", 216, &Battery_2, osPriorityNormal, NULL);
    xTaskCreate((TaskFunction_t)Thread_BatteryPredict, "batpredict1", 128, &Battery_1, osPriorityNormal, NULL);
    xTaskCreate((TaskFunction_t)Thread_BatteryPredict, "batpredict2", 128, &Battery_2, osPriorityNormal, NULL);
    xTaskCreate(DebugTask, "debubprint", 216, NULL, osPriorityNormal, NULL);
    xTaskCreate(BatteryDataLog, "batlog", 216, NULL, osPriorityNormal, NULL);
    xTaskCreate(Thread_CANComm, "CANbus", 216, NULL, osPriorityAboveNormal, NULL);
    xTaskCreate(Thread_Console, "console", 216, NULL, osPriorityAboveNormal, NULL);
    xTaskCreate(Thread_Led_Blink, "ledblink", 64, NULL, osPriorityNormal, NULL);
    xTaskCreate((void (*)(void *))Thread_CANPrint, "CAN_print", 128, NULL, osPriorityNormal, NULL);
    ValveDataLoad();
    xprintf("os start...\n\r");
}

__weak void PreSleepProcessing(uint32_t *ulExpectedIdleTime)
{
    /* place for user code */
}

__weak void PostSleepProcessing(uint32_t *ulExpectedIdleTime)
{
    /* place for user code */
}

uint32_t GetSecond(void)
{
    return seconds;
}

uint32_t GetMinute(void)
{
    return minutes;
}

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void)
{
    printf("************************************************\n\r"
           "********    xRTOS v%.2f  2019-09-11     ********\n\r"
           "************************************************\n\r", fw_version);

    /* Create the mutex(es) */
    /* definition and creation of mutex_iic0 */
    osMutexDef(mutex_iic0);
    mutex_iic0Handle = osMutexCreate(osMutex(mutex_iic0));

    /* definition and creation of mutex_iic1 */
    osMutexDef(mutex_iic1);
    mutex_iic1Handle = osMutexCreate(osMutex(mutex_iic1));

    /* definition and creation of mutex_printf */
    osMutexDef(mutex_printf);
    mutex_printfHandle = osMutexCreate(osMutex(mutex_printf));

    /* add mutexes, ... */
    mtx_batctrl = xSemaphoreCreateMutex();
    mtx_batchg = xSemaphoreCreateMutex();
    mtx_canbuf = xSemaphoreCreateMutex();
    /* Create the semaphores(s) */
    /* definition and creation of sem_console */
    // osSemaphoreDef(sem_console);
    // sem_consoleHandle = osSemaphoreCreate(osSemaphore(sem_console), 1);
    sem_consoleHandle = xSemaphoreCreateBinary();      // vSemaphoreCreateBinary(sem): create sem with default value 1
    sem_canprint = xSemaphoreCreateBinary();          // xSemaphoreCreateBinary(): create sem with default value 0

    /* definition and creation of csem */
    osSemaphoreDef(csem);
    csemHandle = osSemaphoreCreate(osSemaphore(csem), 8);

    /* Create the timer(s) */
    // timer_fastboot = xTimerCreate("fastBoot", 0, pdFALSE, (void *)1, TurnOffGc);
    // timer_gcsleep = xTimerCreate("gcSleep", 0, pdFALSE, (void *)1, TurnOffGc);
    timer_pump1_start = xTimerCreate("pump1start", 1000/portTICK_PERIOD_MS, pdFALSE, (void *)&Pump_1, (void (*)(void *))PulseOutOn);
    timer_pump1_stop = xTimerCreate("pump1stop", 1000/portTICK_PERIOD_MS, pdFALSE, (void *)&Pump_1, (void (*)(void *))PulseOutOff);
    timer_pump2_start = xTimerCreate("pump2start", 1000/portTICK_PERIOD_MS, pdFALSE, (void *)&Pump_2, (void (*)(void *))PulseOutOn);
    timer_pump2_stop = xTimerCreate("pump2start", 1000/portTICK_PERIOD_MS, pdFALSE, (void *)&Pump_2, (void (*)(void *))PulseOutOff);
    timer_pvalve_start = xTimerCreate("pvstart", 1000/portTICK_PERIOD_MS, pdFALSE, (void *)&PValve, (void (*)(void *))PulseOutOn);
    timer_pvalve_stop = xTimerCreate("pvstart", 1000/portTICK_PERIOD_MS, pdFALSE, (void *)&PValve, (void (*)(void *))PulseOutOff);
    timer_heater = xTimerCreate("heater", 20/portTICK_PERIOD_MS, pdTRUE, (void *)&heater, TimerCallBack_Heater);

    /* Create the queue(s) */
    /* definition and creation of q_canmsg */
    // osMessageQDef(q_canmsg, 32, uint64_t);
    // q_canmsgHandle = osMessageCreate(osMessageQ(q_canmsg), NULL);

    /* add queues, ... */
    q_canmsg = xQueueCreate(32, sizeof(stCanPacket));

    /* Create the thread(s) */
    // all threads and init-app are created in daemon task
}

/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
void StartDefaultTask(void const *argument)
{
    /* Infinite loop */
    for (;;)
    {
        osDelay(1);
    }
}

/**
* @brief Function implementing the adaptor thread.
* @param argument: Not used
* @retval None
*/


/* Private application code --------------------------------------------------*/
void DebugTask(void *argument)
{
    void *pWriteBuffer;
    stBattery *bat = &Battery_1;
    uint32_t second;
    uint32_t recv_data = 0;
    uint32_t interval = 0;

    /* USER CODE BEGIN StartDefaultTask */
    while (1)
    {
        if (pdTRUE == xTaskNotifyWait(0x00,          /* Don't clear any notification bits on entry. */
                                      0xffffffffu,   /* Reset the notification value to 0 on exit. */
                                      &recv_data,    /* Notified value pass out in
                                                        ulNotifiedValue. */
                                      5/portTICK_PERIOD_MS ))  /* Block indefinitely. */
            interval = (recv_data < 5000 && recv_data != 0) ? 5000 : recv_data;

        if (interval != 0 && (pWriteBuffer = pvPortMalloc(600)) != NULL) {
            osMutexWait(mutex_printfHandle, osWaitForever);
            second = GetSecond();
            printf("runtime: %ds\n\r", second);
            vTaskList(pWriteBuffer);
            printf("task_name task_state priority stack task_num\n\r");        // use 'debug()' here would cause i2c comm fail, i2c_isr_busy set
            printf("%s", (char *)pWriteBuffer);
            vPortFree(pWriteBuffer);
            bat = &Battery_1;
            HAL_UART_Transmit(UARTHANDLE, "==========================================================================================================================\n\r", 126, 0xffff);
            HAL_UART_Transmit(UARTHANDLE, "|BATTERY| mode\t| status| used\t| level\t| V\t| I\t| T\t| cap.\t| remain| err\t| Iset\t| fcode\t| scale\t| ng\t|\n\r", 102, 0xffff);
            HAL_UART_Transmit(UARTHANDLE, "--------------------------------------------------------------------------------------------------------------------------\n\r", 126, 0xffff);
            printf("|bat%d\t| %d\t| %d\t| %d\t| %.0f\t| %.2f\t| %.2f\t| %.2f\t| %.0f\t| %.0f\t| 0x%02x\t| %.2f\t| %d\t| %d\t| %.2f\t|\n\r", bat->index, bat->mode, bat->status, bat->mux_on, bat->gauge->level, bat->voltage, bat->current, bat->temperature, bat->capacity, bat->remain_time, bat->err_code, bat->charge_iset, bat->finish_code, bat->scale_flag, bat->ng);
            bat = &Battery_2;
            HAL_UART_Transmit(UARTHANDLE, "--------------------------------------------------------------------------------------------------------------------------\n\r", 126, 0xffff);
            printf("|bat%d\t| %d\t| %d\t| %d\t| %.0f\t| %.2f\t| %.2f\t| %.2f\t| %.0f\t| %.0f\t| 0x%02x\t| %.2f\t| %d\t| %d\t| %.2f\t|\n\r", bat->index, bat->mode, bat->status, bat->mux_on, bat->gauge->level, bat->voltage, bat->current, bat->temperature, bat->capacity, bat->remain_time, bat->err_code, bat->charge_iset, bat->finish_code, bat->scale_flag, bat->ng);
            HAL_UART_Transmit(UARTHANDLE, "==========================================================================================================================\n\r", 126, 0xffff);
            HAL_UART_Transmit(UARTHANDLE, "|HEATER\t| status| mode\t| T\t| sp\t| duty\t|\n\r", 42, 0xffff);
            HAL_UART_Transmit(UARTHANDLE, "--------------------------------------------------------------------------------------------------------------------------\n\r", 126, 0xffff);
            printf("|heater\t| %d\t| %d\t| %.3f\t| %.2f\t| %.3f\t|\n\r", heater.pwm.status, heater.mode, heater.temperature, heater.setpoint, heater.pwm.duty);
            HAL_UART_Transmit(UARTHANDLE, "==========================================================================================================================\n\r", 126, 0xffff);
            HAL_UART_Transmit(UARTHANDLE, "|Fdcase\t| v_pwr\t| i_pwr\t| cover\t| gc_sw\t| p_gas1| p_gas2| GcSts\t| lopwr\t|canintr|msghandle|\n\r", 93, 0xffff);
            HAL_UART_Transmit(UARTHANDLE, "--------------------------------------------------------------------------------------------------------------------------\n\r", 126, 0xffff);
            printf("|fdcase\t| %.2f\t| %.2f\t| %d\t| %d\t| %.1f\t| %.1f\t| %d\t| %d\t| %d\t| %d\t|\n\r", FieldCase.v_syspwr, FieldCase.consumption, FieldCase.is_covered, FieldCase.is_switchon, FieldCase.gas_1_pressure, FieldCase.gas_2_pressure, gc_status, low_power_time, can_intr_cnt, msg_total_recv);
            HAL_UART_Transmit(UARTHANDLE, "==========================================================================================================================\n\r", 126, 0xffff);
            HAL_UART_Transmit(UARTHANDLE, "|Adapter| sts\t| v\t| connecttime\t| disconntime\t|\n\r", 49, 0xffff);
            HAL_UART_Transmit(UARTHANDLE, "--------------------------------------------------------------------------------------------------------------------------\n\r", 126, 0xffff);
            printf("|Adapter| %d\t| %.2f\t| %8d\t| %8d\t|\n\r", Adaptor.status, Adaptor.voltage, Adaptor.connect_time, Adaptor.disconnect_time);
            printf("==========================================================================================================================\n\r");
            HAL_UART_Transmit(UARTHANDLE, "\n\n\r", 3, 0xffff);
            osMutexRelease(mutex_printfHandle);
            osDelay(interval);
        }
    }
}


