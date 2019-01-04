/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId tid_vlist;
osThreadId tid_fieldcase;
osThreadId tid_adaptor;
osThreadId tid_gc_cb;
osThreadId tid_bat1_mon;
osThreadId tid_bat2_mon;
osThreadId tid_bat_ctrl;
osThreadId tid_canmsg;

osMutexId mutex_i2c0;
osMutexId mutex_i2c1;
osMutexId mutex_print;

osMessageQId q_canmsg;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);
void vApplicationIdleHook(void);
void vApplicationTickHook(void);
void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char *pcTaskName);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{
}

__weak unsigned long getRunTimeCounterValue(void)
{
    return 0;
}
/* USER CODE END 1 */

/* USER CODE BEGIN 2 */
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
/* USER CODE END 2 */

/* USER CODE BEGIN 3 */
__weak void vApplicationTickHook(void)
{
    /* This function will be called by each tick interrupt if
    configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h. User code can be
    added here, but the tick hook is called from an interrupt context, so
    code must not attempt to block, and only the interrupt safe FreeRTOS API
    functions can be used (those that end in FromISR()). */
}
/* USER CODE END 3 */

/* USER CODE BEGIN 4 */
__weak void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char *pcTaskName)
{
    /* Run time stack overflow checking is performed if
    configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
    called if a stack overflow is detected. */
    printf("stack overflow: %s\n\r", pcTaskName);
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
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
    vprintf("malloc fail, %s\n\r", s);
}
/* USER CODE END 5 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void)
{
    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Create the mutex(es) */
    /* definition and creation of mutex_i2c0 */
    osMutexDef(i2c0);
    mutex_i2c0 = osMutexCreate(osMutex(i2c0));

    /* definition and creation of mutex_i2c1 */
    osMutexDef(i2c1);
    mutex_i2c1 = osMutexCreate(osMutex(i2c1));

    /* definition and creation of mutex_print */
    osMutexDef(print);
    mutex_print = osMutexCreate(osMutex(print));

    /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
    /* USER CODE END RTOS_MUTEX */

    /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
    /* USER CODE END RTOS_SEMAPHORES */

    /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
    /* USER CODE END RTOS_TIMERS */

    /* Create the thread(s) */
    /* definition and creation of defaultTask */
    osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 200);
    defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    osThreadDef(fieldcase, Thread_FieldCase, osPriorityNormal, 0, 120);
    tid_fieldcase = osThreadCreate(osThread(fieldcase), &tid_fieldcase);
    
    osThreadDef(adaptor, Thread_Adaptor, osPriorityNormal, 0, 150);
    tid_adaptor = osThreadCreate(osThread(adaptor), NULL);
    
    osThreadDef(gc_cb, Thread_Gc_CB_Pwr, osPriorityNormal, 0, 150);
    tid_gc_cb = osThreadCreate(osThread(gc_cb), NULL);

    osThreadDef(bat1_mon, Thread_BatteryMonitor, osPriorityNormal, 0, 280);
    tid_bat1_mon = osThreadCreate(osThread(bat1_mon), &Battery_1);

    osThreadDef(bat2_mon, Thread_BatteryMonitor, osPriorityNormal, 0, 280);
    tid_bat2_mon = osThreadCreate(osThread(bat2_mon), &Battery_2);

    osThreadDef(canmsg, MsgHandler, osPriorityAboveNormal, 0, 200);
    tid_canmsg = osThreadCreate(osThread(canmsg), NULL);
    
    /* USER CODE END RTOS_THREADS */

    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    q_canmsg = xQueueCreate(10, sizeof(CANMsg_t));
    /* USER CODE END RTOS_QUEUES */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const *argument)
{
    void *pWriteBuffer;
    stBattery *bat = &Battery_1;
    UBaseType_t tick;
    
    /* USER CODE BEGIN StartDefaultTask */   
    while (1)
    {
        tick = xTaskGetTickCount();
        vprintf("runtime: %d\n\r", tick);
        pWriteBuffer = pvPortMalloc(500);
        vTaskList(pWriteBuffer);
        vprintf("task_name task_state priority stack task_num\n\r");        // use 'debug()' here would cause i2c comm fail, i2c_isr_busy set
        vprintf("%s", (char *)pWriteBuffer);
        vPortFree(pWriteBuffer);
        bat = &Battery_1;
        vprintf("==========================================================================================================================\n\r");
        vprintf("|BATTERY| mode\t| status| used\t| level\t| V\t| I\t| T\t| cap.\t| remain| err\t| Iset\t| fcode\t| scale\t| ofuf\t|\n\r");
        vprintf("--------------------------------------------------------------------------------------------------------------------------\n\r");
        vprintf("|bat%d\t| %d\t| %d\t| %d\t| %.0f\t| %.2f\t| %.2f\t| %.2f\t| %.0f\t| %.0f\t| 0x%02x\t| %.2f\t| %d\t| %d\t| %d\t|\n\r", bat->index, bat->mode, bat->status, bat->mux_on, bat->gauge->level, bat->voltage, bat->current, bat->temperature, bat->capacity, bat->remain_time, bat->err_code, bat->charge_iset, bat->finish_code, bat->scale_flag, bat->gauge->acr_ofuf);
        vprintf("--------------------------------------------------------------------------------------------------------------------------\n\r");
        bat = &Battery_2;
        vprintf("|bat%d\t| %d\t| %d\t| %d\t| %.0f\t| %.2f\t| %.2f\t| %.2f\t| %.0f\t| %.0f\t| 0x%02x\t| %.2f\t| %d\t| %d\t| %d\t|\n\r", bat->index, bat->mode, bat->status, bat->mux_on, bat->gauge->level, bat->voltage, bat->current, bat->temperature, bat->capacity, bat->remain_time, bat->err_code, bat->charge_iset, bat->finish_code, bat->scale_flag, bat->gauge->acr_ofuf);
        vprintf("==========================================================================================================================\n\r");
        vprintf("|HEATER\t| status| mode\t| T\t| sp\t| kp\t| ki\t| kd\t| duty\t|\n\r");
        vprintf("--------------------------------------------------------------------------------------------------------------------------\n\r");
        vprintf("|heater\t| %d\t| %d\t| %.3f\t| %.2f\t| %d\t| %d\t| %d\t| %.3f\t|\n\r", Heater.pwm.status, Heater.mode, Heater.temperature, Heater.setpoint, Heater.kp, Heater.ki, Heater.kd, Heater.pwm.duty);
        vprintf("==========================================================================================================================\n\r");
        vprintf("|Fdcase\t| v_pwr\t| i_pwr\t| cover\t| gc_sw\t| p_gas1| p_gas2| GcSts\t| Gcrq\t| lopwr\t|\n\r");
        vprintf("--------------------------------------------------------------------------------------------------------------------------\n\r");
        vprintf("|fdcase\t| %.2f\t| %.2f\t| %d\t| %d\t| %.1f\t| %.1f\t| %d\t| %d\t| %d\t|\n\r", FieldCase.v_syspwr, FieldCase.consumption, FieldCase.is_covered, FieldCase.is_switchon, FieldCase.gas_1_pressure, FieldCase.gas_2_pressure, gc_status, boot_request, lo_pwr_trigger);
        vprintf("==========================================================================================================================\n\r");
        vprintf("|Adapter| sts\t| v\t| connecttime\t| disconntime\t|\n\r");
        vprintf("--------------------------------------------------------------------------------------------------------------------------\n\r");
        vprintf("|Adapter| %d\t| %.2f\t| %8d\t| %8d\t|\n\r", Adaptor.status, Adaptor.voltage, Adaptor.connect_time, Adaptor.disconnect_time);
        vprintf("==========================================================================================================================\n\r");
        vprintf("\n\n\r");

        osDelay(5000);
    }
    /* USER CODE END StartDefaultTask */
}



/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
