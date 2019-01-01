/**
  ******************************************************************************
  * @file    Project/src/FieldCase.c
  * @author
  * @version V0.00
  * @date
  * @brief   FieldCase.c
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "app.h"

/* Private macro -------------------------------------------------------------*/
#define BUF_SIZE 32

#if DEBUG == 0
  #define SW_OFF_TIME	    1800
  #define COVER_OFF_TIME    300
  #define FASTBOOT_TIME     300
#elif DEBUG == 1
  #define SW_OFF_TIME	    1
  #define COVER_OFF_TIME    0
  #define FASTBOOT_TIME     10
#endif

#define DUAL_CUR            4
#define SINGLE_CUR          2

/* Private typedef -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
stFieldCase FieldCase = {
    .consumption = 0,
    .v_syspwr = 0,
    .is_covered = false,
    .is_switchon = false,
//    .bat_used = NULL,
    .switchoff_time = 0,
    .switchon_time = 0,
    .lid_on_time = 0,
    .lid_off_time = 0,
    .env_temperature = 0,
    .i_transit_up = 0,
    .i_transit_down = 0
};

bool                need_dual_bat   = false;
static uint32_t     dual_start_time = 0;
static bool         back_cur_detect = false;
static uint32_t     back_cur_time   = 0;
static uint32_t     high_cur_time   = 0;
bool                lo_pwr_trigger  = false;     // to prevent battery rebound

eBootMode           boot_request    = kGcOff;
eBootMode           gc_status       = kGcOff;

static uint32_t     tick            = 0;


/* Private function prototypes -----------------------------------------------*/


/* Code begin ----------------------------------------------------------------*/

void Thread_FieldCase(void const *tid)
{

    float adc_code = 0, consumption = 0;
    
    while (1)
    {
        // check cover hall sensor
        if (HAL_GPIO_ReadPin(HALL_INTR_GPIO_Port, HALL_INTR_Pin) == GPIO_PIN_RESET && FieldCase.is_covered == false) {
            vTaskDelay(100/portTICK_PERIOD_MS);
            if (HAL_GPIO_ReadPin(HALL_INTR_GPIO_Port, HALL_INTR_Pin) == GPIO_PIN_RESET) {
                FieldCase.is_covered = true;
                FieldCase.lid_off_time = GetSecond();
            }
        } else if ((HAL_GPIO_ReadPin(HALL_INTR_GPIO_Port, HALL_INTR_Pin) == GPIO_PIN_SET) && FieldCase.is_covered == true) {
            vTaskDelay(100/portTICK_PERIOD_MS);
            if (HAL_GPIO_ReadPin(HALL_INTR_GPIO_Port, HALL_INTR_Pin) == GPIO_PIN_SET) {
                FieldCase.is_covered = false;
                FieldCase.lid_on_time = GetSecond();
            }
        }

        // check GC power switch
        if (HAL_GPIO_ReadPin(BAT_SW_MON_GPIO_Port, BAT_SW_MON_Pin) == GPIO_PIN_SET && FieldCase.is_switchon == false) {
            vTaskDelay(100/portTICK_PERIOD_MS);
            if (HAL_GPIO_ReadPin(BAT_SW_MON_GPIO_Port, BAT_SW_MON_Pin) == GPIO_PIN_SET) {
                FieldCase.is_switchon = true;
                FieldCase.switchon_time = GetSecond();
            }
        } else if ((HAL_GPIO_ReadPin(BAT_SW_MON_GPIO_Port, BAT_SW_MON_Pin) == GPIO_PIN_RESET) && FieldCase.is_switchon == true) {
            vTaskDelay(100/portTICK_PERIOD_MS);
            if (HAL_GPIO_ReadPin(BAT_SW_MON_GPIO_Port, BAT_SW_MON_Pin) == GPIO_PIN_RESET) {
                FieldCase.is_switchon = false;
                FieldCase.switchoff_time = GetSecond();
            }
        }

        FieldCase.v_syspwr = ADCvalue[ADC_SYSPWR] * 0.003742;

        adc_code = ADCvalue[ADC_4236IMON] - ADCvalue[ADC_OFFSET1];
        consumption = adc_code * 0.0053724;                  // adc_code / 4095 * 3.3 / 100 / 0.0015;
        if (FieldCase.consumption - consumption > 3)
            FieldCase.i_transit_down = GetSecond();
        else if (consumption - FieldCase.consumption > 3)
            FieldCase.i_transit_up = GetSecond();
        FieldCase.consumption = consumption;

        if (Adaptor.status == kAdaptorSupplying) {
            if (need_dual_bat == true) {
                need_dual_bat = false;
                high_cur_time = 0;
                printf("FieldCaseInofUpdate(): adaptor connected, dualbat mode off\n\r");
            }
        } else if (FieldCase.consumption >= DUAL_CUR) {                             // turn on dual_bat mode as soon as detecting high consumption
            if (need_dual_bat == false && back_cur_detect == false) {
                need_dual_bat = true;
                dual_start_time = GetSecond();
                printf("FieldCaseInofUpdate(): consumption is %.3f, dualbat mode on\n\r", FieldCase.consumption);
            } else if (need_dual_bat == true) {
                if (((Battery_1.current > 0.1 && Battery_1.mux_on == true) || (Battery_2.current > 0.1 && Battery_2.mux_on == true)) &&
                        GetSecond() - dual_start_time > 3) {
                    printf("FieldCaseInfoUpdate(): back current detect, dualbat mode off\n\r");
                    need_dual_bat = false;
                    back_cur_detect = true;
                    back_cur_time = GetMinute();
                }
            }
            high_cur_time = GetSecond();
        } else if (FieldCase.consumption <= SINGLE_CUR && high_cur_time != 0) {     // turn off dual_bat mode when low current last for 3s
            if (GetSecond() - high_cur_time > 10) {
                high_cur_time = 0;
                if (need_dual_bat == true) {
                    need_dual_bat = false;
                    printf("FieldCaseInfoUpdate(): consumption is %.3f, dualbat mode off\n\r", FieldCase.consumption);
                }
            }
        }

        if (back_cur_detect == true && GetMinute() - back_cur_time > 5) {
            back_cur_detect = false;
            printf("FieldCaseInfoUpdate(): clear back_cur_detect\n\r");
        }

        adc_code = sd1_gain_coe * (sdadc1_code[SDADC1_CHNL_GAS1] + 32767);
        FieldCase.gas_1_pressure = (adc_code / 65535 * 3.3 * 1.681 - 1) * 40;
        adc_code = sd1_gain_coe * (sdadc1_code[SDADC1_CHNL_GAS2] + 32767);
        FieldCase.gas_2_pressure = (adc_code / 65535 * 3.3 * 1.681 - 1) * 40;

        if (Battery_1.status != kstsNotExist)
            FieldCase.env_temperature = Battery_1.gauge->temperature;
        else if (Battery_2.status != kstsNotExist)
            FieldCase.env_temperature = Battery_2.gauge->temperature;

        if (Battery_1.mux_on)
            FieldCase.bat_used = &Battery_1;
        else if (Battery_2.mux_on)
            FieldCase.bat_used = &Battery_2;
        else
            FieldCase.bat_used = NULL;
        
        tick = xTaskGetTickCount();
        printf("FieldCase: tick %d, watermark %d\r", tick, uxTaskGetStackHighWaterMark(*(osThreadId *)tid_fieldcase));
        vTaskDelay(100/portTICK_PERIOD_MS);
    }   // end while (1)
}

/**
  * @brief  turn on GC with 'normal' mode
  * @param
  * @retval None
  */
static void TurnOnGc(void)
{
    DioSetTo(&MB_Pwr, 1);
    DioSetTo(&CCB_Pwr, 1);
    gc_status = kGcRun;
    printf("TurnOnGc(): turn on GC\n\r");
}

/**
  * @brief  turn on GC with 'fastboot' mode
  * @param
  * @retval None
  */
static void FastBootGc(void)
{
    DioSetTo(&MB_Pwr, 1);
    DioSetTo(&CCB_Pwr, 0);
    gc_status = kGcFastBoot;
    printf("FastBootGc(): \n\r");
}

/**
  * @brief  turn off GC
  * @param
  * @retval None
  */
static void TurnOffGc(void)
{
    DioSetTo(&MB_Pwr, 0);
    DioSetTo(&CCB_Pwr, 0);
    DioSetTo(&Fan, 0);
    DioSetTo(&Valve_1, 0);
    DioSetTo(&Valve_2, 0);
    gc_status = kGcOff;
    printf("TurnOffGc(): turn off GC\n\r");
}

void Thread_Gc_Pwr(void const *param)
{
    bool is_covered = false;
    bool is_switchon = false;
    uint32_t cover_on_time = 0;
    uint32_t cover_off_time = 0;

    while (1) {
        if (gc_status == kGcOff) {
            if (FieldCase.is_switchon == true && is_switchon == false) {
                if (lo_pwr_trigger == false || Adaptor.status == kAdaptorSupplying) {
                    is_switchon = true;
                    TurnOnGc();
                }
            } else if (FieldCase.is_covered == false && is_covered == true) {
                if (lo_pwr_trigger == false || Adaptor.status == kAdaptorSupplying) {
                    is_covered = false;
                    cover_on_time = GetSecond();
                    FastBootGc();
                }
            } 
            
        } else if (gc_status == kGcRun) {
            if (FieldCase.is_switchon == false && is_switchon == true) {
                is_switchon = false;
                TurnOffGc();
            } else if (FieldCase.is_covered == false && is_covered == true) {
                is_covered = false;
                cover_off_time = GetSecond();
            } else if (GetSecond() - cover_off_time >= 300) {
                TurnOffGc();
            }
        } else if (gc_status == kGcFastBoot) {
            if (FieldCase.is_switchon == true && is_switchon == false) {
                if (lo_pwr_trigger == false) {
                    is_switchon = true;
                    TurnOnGc();
                }
            } else if (FieldCase.is_covered == true && is_covered == false) {
                is_covered = true;
                TurnOffGc();
            } else if (GetSecond() - cover_on_time >= 300) {
                TurnOffGc();
            }
        }
    }
}

void Thread_CB_Pwr(void const *param)
{
    uint32_t cb_drop_time = 0;
    uint32_t second = 0;

    while (1) {
        if (Adaptor.status != kAdaptorNotExist) {                                  // adaptor connected
            HAL_GPIO_WritePin(BAT_SW_MCU_GPIO_Port, BAT_SW_MCU_Pin, GPIO_PIN_SET);
            cb_drop_time = 0;
        } else {                                                                    // adaptor not connected
            if (FieldCase.is_switchon == true) {                                    // sw_button is on
                HAL_GPIO_WritePin(BAT_SW_MCU_GPIO_Port, BAT_SW_MCU_Pin, GPIO_PIN_SET);
            } else {                                                                // sw_button is off
                if (FieldCase.v_syspwr <= CB_STOP_TH_VOLT) {
                    if (cb_drop_time == 0) {
                        cb_drop_time = GetSecond();
                    } else if (GetSecond() - cb_drop_time >= 7) {
                        printf("CB_PwrCntl(): low cb power, turn off cb\n\r");
                        HAL_GPIO_WritePin(BAT_SW_MCU_GPIO_Port, BAT_SW_MCU_Pin, GPIO_PIN_RESET);
                    }
                } else {
                    cb_drop_time = 0;
                }

                if (FieldCase.is_covered == false) {                                // lid is open
                    second = Adaptor.disconnect_time > FieldCase.switchoff_time ? Adaptor.disconnect_time : FieldCase.switchoff_time;
                    if (GetSecond() - second >= SW_OFF_TIME) {
                        printf("CB_PwrCntl(): SW_OFF_TIME timeout, turn off cb\n\r");
                        HAL_GPIO_WritePin(BAT_SW_MCU_GPIO_Port, BAT_SW_MCU_Pin, GPIO_PIN_RESET);
                    }
                } else {                                                            // lid is closed
                    if (GetSecond() - FieldCase.lid_off_time >= COVER_OFF_TIME) {
                        printf("CB_PwrCntl(): COVER_OFF_TIME timeout, turn off cb\n\r");
                        HAL_GPIO_WritePin(BAT_SW_MCU_GPIO_Port, BAT_SW_MCU_Pin, GPIO_PIN_RESET);
                    }
                }
            }
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void Self_Check(void)
{
    if (OpenDetect(&MB_Pwr))
        printf("MBFAN not connected\n\r");
    if (OpenDetect(&Valve_1))
        printf("Valve_1 not connected\n\r");
    if (OpenDetect(&Valve_2))
        printf("Valve_2 not connected\n\r");
    if (OpenDetect(&Fan))
        printf("GCFAN not connected\n\r");
//    if (CheckFaulty(&Pump_1))
//        printf("Pump_1 not connected\n\r");
//    if (CheckFaulty(&Pump_2))
//        printf("Pump_2 not connected\n\r");
//    if (CheckFaulty(&PValve))
//        printf("PValve not connected\n\r");
}



