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
#include "app_include.h"

/* Private macro -------------------------------------------------------------*/
#define BUF_SIZE 32

#if DEBUG == 0
  #define SW_OFF_TIME	    900
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
    .bat_used = NULL,
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
bool                back_cur_detect = false;
static uint32_t     back_cur_time   = 0;
static uint32_t     high_cur_time   = 0;
static uint32_t     cb_drop_time    = 0;
uint32_t            low_power_time  = 0;
bool                low_power_flag  = false;
eBootMode           gc_status       = kGcOff;
int32_t             total_bat_time  = 0;
int32_t             total_bat_percent = 0;


/* Private function prototypes -----------------------------------------------*/

/* Code begin ----------------------------------------------------------------*/
/**
  * @brief  turn on GC with 'normal' mode
  * @param
  * @retval None
  */
static void TurnOnGc(void)
{
    DioSetTo(&MB_Pwr, 1);
    DioSetTo(&CCB_Pwr, 1);
    DioSetTo(&Fan, 1);
    gc_status = kGcRun;
    xprintf("TurnOnGc(): turn on GC\n\r");
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
    xprintf("FastBootGc(): \n\r");
}

/**
  * @brief  turn off GC
  * @param
  * @retval None
  */
void TurnOffGc(void *p)
{
    DioSetTo(&MB_Pwr, 0);
    DioSetTo(&CCB_Pwr, 0);
    DioSetTo(&Fan, 0);
    gc_status = kGcOff;
    xprintf("TurnOffGc(): turn off GC\n\r");
}

void Thread_FieldcaseInfoUpdate(void *p)
{
    float adc_code, consumption;
    float cap1, cap2;

    xprintf("thread_fieldcase start\n\r");

    while (1)
    {
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

        if (HAL_GPIO_ReadPin(HALL_INTR_GPIO_Port, HALL_INTR_Pin) == GPIO_PIN_RESET && FieldCase.is_covered == false) {               // hall sensor 'L' detect
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

        FieldCase.v_syspwr = ADCvalue[ADC_SYSPWR] * 0.003742;

        adc_code = ADCvalue[ADC_4236IMON] - ADCvalue[ADC_OFFSET1];
        consumption = adc_code * 0.0053724;                  // adc_code / 4095 * 3.3 / 100 / 0.0015;
        if (FieldCase.consumption - consumption > 3)
            FieldCase.i_transit_down = GetSecond();
        else if (consumption - FieldCase.consumption > 3)
            FieldCase.i_transit_up = GetSecond();
        FieldCase.consumption = consumption;

        if (Adaptor.status != kAdaptorNotExist) {
            if (need_dual_bat == true) {
                need_dual_bat = false;
                high_cur_time = 0;
                xprintf("FieldCaseInofUpdate(): adaptor connected, dualbat mode off\n\r");
            }
            low_power_flag = false;
        } else if (FieldCase.consumption >= DUAL_CUR) {                             // turn on dual_bat mode as soon as detecting high consumption
            if (need_dual_bat == false && back_cur_detect == false) {
                need_dual_bat = true;
                dual_start_time = GetSecond();
                xprintf("FieldCaseInofUpdate(): consumption is %.3f, dualbat mode on\n\r", FieldCase.consumption);
            } else if (need_dual_bat == true) {
                if (((Battery_1.current > 0.1 && Battery_1.mux_on == true) || (Battery_2.current > 0.1 && Battery_2.mux_on == true)) &&
                        GetSecond() - dual_start_time > 3) {
                    xprintf("FieldCaseInfoUpdate(): back current detect, dualbat mode off\n\r");
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
                    xprintf("FieldCaseInfoUpdate(): consumption is %.3f, dualbat mode off\n\r", FieldCase.consumption);
                }
            }
        }

        if (back_cur_detect == true && GetMinute() - back_cur_time > 5) {
            back_cur_detect = false;
            xprintf("FieldCaseInfoUpdate(): clear back_cur_detect\n\r");
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

        if (Battery_1.status == kstsFastCharge)
            cap1 = (Battery_1.level - Battery_1.q_start_charge) * Battery_1.ng + Battery_1.q_start_charge;
        else
            cap1 = Battery_1.level > Battery_1.capacity ? Battery_1.capacity : Battery_1.level;

        if (Battery_2.status == kstsFastCharge)
            cap2 = (Battery_2.level - Battery_2.q_start_charge) * Battery_2.ng + Battery_2.q_start_charge;
        else
            cap2 = Battery_2.level > Battery_2.capacity ? Battery_2.capacity : Battery_2.level;

        cap1 = cap1 < 0 ? 0 : cap1;
        cap1 = cap1 > Battery_1.capacity ? Battery_1.capacity : cap1;
        cap2 = cap2 < 0 ? 0 : cap2;
        cap2 = cap2 > Battery_2.capacity ? Battery_2.capacity : cap2;

        total_bat_percent = (int32_t)((cap1 + cap2) / (Battery_1.capacity + Battery_2.capacity) * 100);
        total_bat_time = (int32_t)(Battery_1.remain_time + Battery_2.remain_time);
        total_bat_time = total_bat_time > 960 ? 960 : total_bat_time;

        vTaskDelay(50/portTICK_PERIOD_MS);
    }
}

void Thread_GcPwrCntl(void *p)
{
    bool power_button = OFF;
    bool cover = OFF;

    xprintf("thread_gcpwrcntl start\n\r");

    while(1) {
        if (Adaptor.status != kAdaptorNotExist)
            low_power_flag = false;

        if (gc_status == kGcOff) {
            if (power_button == OFF && FieldCase.is_switchon) {
                if ((Battery_1.remain_time >= 0 || Battery_2.remain_time >= 0) && low_power_flag == false) {
                    TurnOnGc();
                }
                power_button = ON;
            } else if (power_button == ON && !FieldCase.is_switchon)
                power_button = OFF;

            if (cover == OFF && FieldCase.is_covered == false) {
                if ((Battery_1.remain_time >= 0 || Battery_2.remain_time >= 0) && low_power_flag == false) {
                    if (power_button == ON)
                        TurnOnGc();
                    else
                        FastBootGc();
                }
                cover = ON;
            } else if (cover == ON && FieldCase.is_covered )
                cover = OFF;
        }

        if (gc_status == kGcRun) {
            if (FieldCase.v_syspwr <= GC_STOP_TH_VOLT) {
                if (low_power_time == 0) {
                    low_power_time = GetSecond();
                } else if (GetSecond() - low_power_time >= 7) {
                    TurnOffGc(NULL);
                    low_power_flag = true;
                }
            } else
                low_power_time = 0;

            if (cover == ON && FieldCase.is_covered == true)
                cover = OFF;

            if (cover == OFF && GetSecond() - FieldCase.lid_off_time >= 300)
                TurnOffGc(NULL);

            if (power_button == ON && FieldCase.is_switchon == false) {
                TurnOffGc(NULL);
                power_button = OFF;
            }
        }

        if (gc_status == kGcFastBoot) {
            if (GetSecond() - FieldCase.lid_on_time >= 300)
                TurnOffGc(NULL);

            if (power_button == OFF && FieldCase.is_switchon == true && low_power_flag == false) {
                TurnOnGc();
                power_button = ON;
            }

            if (cover == ON && FieldCase.is_covered == true) {
                TurnOffGc(NULL);
                cover = OFF;
            }

            if (FieldCase.v_syspwr <= GC_STOP_TH_VOLT) {
                if (low_power_time == 0) {
                    low_power_time = GetSecond();
                } else if (GetSecond() - low_power_time >= 7) {
                    TurnOffGc(NULL);
                    low_power_flag = true;
                }
            } else
                low_power_time = 0;
        }
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
}

void Thread_CbPwrCntl(void *p)
{
    xprintf("thread_cbpwrcntl start\n\r");

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
                        xprintf("CB_PwrCntl(): low cb power, turn off cb\n\r");
                        HAL_GPIO_WritePin(BAT_SW_MCU_GPIO_Port, BAT_SW_MCU_Pin, GPIO_PIN_RESET);
                    }
                } else {
                    cb_drop_time = 0;
                }

                if (FieldCase.is_covered == false) {                                // lid is open
                    uint32_t second = Adaptor.disconnect_time > FieldCase.switchoff_time ? Adaptor.disconnect_time : FieldCase.switchoff_time;
                    if (GetSecond() - second >= SW_OFF_TIME) {
                        xprintf("CB_PwrCntl(): SW_OFF_TIME timeout, turn off cb\n\r");
                        HAL_GPIO_WritePin(BAT_SW_MCU_GPIO_Port, BAT_SW_MCU_Pin, GPIO_PIN_RESET);
                    }
                } else {                                                            // lid is closed
                    if (GetSecond() - FieldCase.lid_off_time >= COVER_OFF_TIME) {
                        xprintf("CB_PwrCntl(): COVER_OFF_TIME timeout, turn off cb\n\r");
                        HAL_GPIO_WritePin(BAT_SW_MCU_GPIO_Port, BAT_SW_MCU_Pin, GPIO_PIN_RESET);
                    }
                }
            }
        }
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
}

// void Self_Check(void)
// {
//    if (MB_Pwr.CheckFaulty())
//        xprintf("MBFAN not connected\n\r");
//    if (Valve_1.CheckFaulty())
//        xprintf("Valve_1 not connected\n\r");
//    if (Valve_2.CheckFaulty())
//        xprintf("Valve_2 not connected\n\r");
//    if (Fan.CheckFaulty())
//        xprintf("GCFAN not connected\n\r");
//    if (Pump_1.CheckFaulty())
//        xprintf("Pump_1 not connected\n\r");
//    if (Pump_2.CheckFaulty())
//        xprintf("Pump_2 not connected\n\r");
// }

