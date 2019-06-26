/**
  ******************************************************************************
  * @file    Project/src/command.c
  * @author
  * @version V0.00
  * @date
  * @brief   usart console to support command line
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "app_include.h"

/* Private macro -------------------------------------------------------------*/
#define CMD_SIZE 64

/* Private typedef -----------------------------------------------------------*/
typedef enum {
    kConsoleIdle = 0,
    kConsoleRecv,
    kConsoleCplt
} UartSts_t;

typedef struct {
    const char *s;
    void (*pfn)(void *);
} CmdTable_t;


/* Private function prototypes -----------------------------------------------*/
static void WelcomePrint(void *);
static void ResetBattery(void *arg);
static void ToggleRed(void *arg);
static void ChargeBat(void *arg);
static void SupplyBat(void *arg);
static void ResetMB(void *arg);
static void ResetCCB(void *arg);
static void Monitor(void *arg);

extern uint8_t monitor_sw;


/* Private variables ---------------------------------------------------------*/
bool uart_listen = false;

static uint8_t cmd_msg[CMD_SIZE] = {0};
//static uint8_t cmd_dlc = 0;
static uint8_t cmd_widx = 0;
static uint8_t RxByte;
//static bool dlf = 0;        // double link dlf

static CmdTable_t cmd_table[]= {
    {"fwver", WelcomePrint},
    {"resetbattery", ResetBattery},
    {"togglered", ToggleRed},
    {"chargebat", ChargeBat},
    {"supplybat", SupplyBat},
    {"resetmb", ResetMB},
    {"resetccb", ResetCCB},
    {"reset", (void (*)(void*))NVIC_SystemReset},
    {"monitor", Monitor},
    {"canmonitor", (void (*)(void*))CAN_MonitorSwitch},
    {"can", (void (*)(void*))CAN_ManualSend},
    {NULL, NULL},
};


/* Code begin ----------------------------------------------------------------*/
static void WelcomePrint(void *arg)
{
    xprintf("hummingbird charge board v%.2f 2019-06-13\n\r", fw_version);
    // xprintf("size of size_t: %d\n\r", sizeof(size_t));                           // answer is '4'
}

static void ResetBattery(void *arg)
{
    BatteryDataClear(&Battery_1);
    BatteryDataClear(&Battery_2);
    Battery_HwInit();
}

static void ToggleRed(void *arg)
{
    HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
    led_red_lock_time = GetSecond();
}

static void ChargeBat(void *arg)
{
    uint8_t *argument = (uint8_t *)arg;
    stBattery *bat = NULL;

    if (*argument == '1')
        bat = &Battery_1;
    else if (*argument == '2')
        bat = &Battery_2;

    argument = (uint8_t *)arg + 2;

    if (*argument == '0')
        ChargeCmdhandler(bat, OFF);
    else if (*argument == '1')
        ChargeCmdhandler(bat, ON);
}

static void SupplyBat(void *arg)
{
    uint8_t *argument = (uint8_t *)arg;
    stBattery *bat = NULL;

    if (*argument == '1')
        bat = &Battery_1;
    else if (*argument == '2')
        bat = &Battery_2;

    argument = (uint8_t *)arg + 2;

    if (*argument == '0')
        BatterySupplyCmdHandler(bat, OFF);
    else if (*argument == '1')
        BatterySupplyCmdHandler(bat, ON);
}

static void Monitor(void *arg)
{
    uint8_t *argument = (uint8_t *)arg;

    monitor_sw = *argument == '0' ? 0 : 1;
}

static void Console_Listen(void)
{
    uart_listen = HAL_UART_Receive_IT(&huart1, &RxByte, 1) == HAL_OK;
}

static void ResetMB(void *arg)
{
    DioSetTo(&MB_Pwr, 0);
    osDelay(1000);
    DioSetTo(&MB_Pwr, 1);
}

static void ResetCCB(void *arg)
{
    DioSetTo(&CCB_Pwr, 0);
    osDelay(1000);
    DioSetTo(&CCB_Pwr, 1);
}

#if 0
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    switch (uart_sts) {
    case kConsoleIdle:
        if (RxByte == 0xAA) {
            uart_sts = kConsoleRecv;
            cmd_widx = 0;
        }
        break;
    case kConsoleRecv:
        if (dlf == true) {
            if (RxByte != 0xAA) {
                uart_sts = kConsoleCplt;
            } else {
                cmd_msg[cmd_widx++] = RxByte;
            }
            dlf = false;
        } else {
            if (RxByte == 0xAA) {
                dlf = true;
            } else {
                cmd_msg[cmd_widx++] = RxByte;
            }
        }
        break;
    case kConsoleCplt:      // 'lock' status. in main loop it handle the cmd and unlock the sts to 'idle'
        break;
    default : break;
    }

    Console_Listen();
}
#endif

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    portBASE_TYPE pxHigherPriorityTaskWoken;

    xSemaphoreGiveFromISR(sem_consoleHandle, &pxHigherPriorityTaskWoken);
    Console_Listen();

    if (pxHigherPriorityTaskWoken == pdTRUE)
        portYIELD_FROM_ISR(pdTRUE);
}

void Thread_Console(void *param)
{
    uint8_t i = 0, *p, *q, *arg;
    UartSts_t uart_sts = kConsoleIdle;

    while (1) {

        if (uart_listen == false)
            Console_Listen();

        if (xSemaphoreTake(sem_consoleHandle, 100 / portTICK_PERIOD_MS) == pdTRUE) {

            switch (uart_sts) {
            case kConsoleIdle:
                if (RxByte != '\r' && RxByte != '\n' && RxByte != '\0') {
                    cmd_msg[cmd_widx++] = RxByte;
                    uart_sts = kConsoleRecv;
                }
                break;
            case kConsoleRecv:
                if (RxByte == '\r' || RxByte == '\n') {
                        uart_sts = kConsoleCplt;
                        cmd_widx = 0;
                } else {
                    cmd_msg[cmd_widx++] = RxByte;
                    if (cmd_widx >= CMD_SIZE) {
                        cmd_widx = 0;
                        uart_sts = kConsoleCplt;
                    }
                }
                break;
            case kConsoleCplt:      // 'lock' status. in main loop it handle the cmd and unlock the sts to 'idle'
                break;
            default : break;
            }
        }

        if (uart_sts == kConsoleCplt) {
            printf("console receive: %s\n\r", cmd_msg);
            i = 0;
            while (cmd_table[i].s != NULL) {
                p = cmd_msg;
                q = (uint8_t *)cmd_table[i].s;
                while (*q == *p) {
                    p++;
                    q++;
                    if (*q == '\0') {
                        arg = p+1;
                        cmd_table[i].pfn(arg);
                        goto FINISH;
                    }
                }

                i++;
            }
FINISH:
            uart_sts = kConsoleIdle;
            memset(cmd_msg, 0, CMD_SIZE);
        }
    }
}


