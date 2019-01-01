/**
  ******************************************************************************
  * @file    Project/src/adc_app.c
  * @author
  * @version V0.00
  * @date
  * @brief   battery.c
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "app.h"

/* Private macro -------------------------------------------------------------*/
#define TMAX_5A             343                                                 // max fast charge time limit
#define TMAX_DYNAMIC        463                                                 // max fast charge time limit
#define MAX_CHARGE_AMPS     5.0F
#define DYNAMIC_CHARGE_AMPS 3.7F
#define PRECHARGE_AMPS      1                                                   // 1amps precharge current
#define TRICKLE_AMPS        0.5                                                 // 0.5amps trickle current
#define CONSTANT_AMPS       2.0F                                                // 1amps constant current for 'california mode'
#define VSAMPLE_INTV        64                                                  // 64ms
#define CHECKCHARGE_INTV    64                                                  // 64ms
#define HEAVY_TRANSIT_TH    2                                                   // record heavy_transit when load current step increase > this amps
#define I_CAL_TH            -2.5F                                               // calibrate battery when reach bottom and supply current < this

#define LVL_OFFSET_0C       300.0F
#define LVL_OFFSET_50C      300.0F

#if DEBUG == 1
  #define PRECHARGE_TIME  10                                                    // tmax/12 in normal operation, 1min for debug convenience
  #define TRICKLE_TIME    10                                                    // tmax/3 in normal operation, 1min for debug convenience
#elif DEBUG == 0
  #define PRECHARGE_TIME  30                                                    // tmax/12 in normal operation, 1min for debug convenience
  #define TRICKLE_TIME    100                                                   // tmax/3 in normal operation, 1min for debug convenience
#endif


/* Private typedef -----------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static void SetChargeCurrent(stBattery *, float);
static void Calibrate(stBattery *bat, eCheckMode mode);
static void ErrorHandler(stBattery *bat, uint16_t err_code);

/* Private variables ---------------------------------------------------------*/
stBattery Battery_1 = {
    .index = 1,
    .status = kstsNotExist,
    .mux_on = true,
    .gauge = &Gauge1,
    .hi2c = &hi2c1,
    .ntc_adc = &ADCvalue[ADC_NTC1],
    .charge_port = CHARGE_1_EN_GPIO_Port,
    .charge_pin = CHARGE_1_EN_Pin,
    .supply_port = BAT_1_EN_GPIO_Port,
    .supply_pin = BAT_1_EN_Pin,
    .connect_port = BAT_1_INTR_GPIO_Port,
    .connect_pin = BAT_1_INTR_Pin,
    .substitute = &Battery_2,
};

stBattery Battery_2 = {
    .index = 2,
    .status = kstsNotExist,
    .mux_on = true,
    .gauge = &Gauge2,
    .hi2c = &hi2c2,
    .ntc_adc = &ADCvalue[ADC_NTC2],
    .charge_port = CHARGE_2_EN_GPIO_Port,
    .charge_pin = CHARGE_2_EN_Pin,
    .supply_port = BAT_2_EN_GPIO_Port,
    .supply_pin = BAT_2_EN_Pin,
    .connect_port = BAT_2_INTR_GPIO_Port,
    .connect_pin = BAT_2_INTR_Pin,
    .substitute = &Battery_1,
};

float bat_1_t_coef;
float bat_2_t_coef;

bool lo_pwr_beep = false;


/* Code begin ----------------------------------------------------------------*/
static void
ChargeOn(stBattery *battery, eChargeMode mode)
{
    if (battery->status == kstsError || battery->status == kstsFinish)
        return;

    battery->peak_volt = 0;
    battery->peak_time = 0;
    //battery->scale_flag &= ~REACHTOP;
    battery->tick_gaugesample = 0;                                              // to prevent spurious CheckFaulty()
    battery->tick_chargecheck = 0;
    memset(battery->v_ringbuf.data, 0, RING_BUF_SIZE * sizeof(battery->v_ringbuf.data[0]));
    battery->v_ringbuf.read_index = 0;
    battery->v_ringbuf.write_index = 0;
    battery->v_ringbuf.clear = false;
    battery->finish_code = 0;

    printf("charge on bat_%d, phase=%d, status %d\n\r", battery->index, mode, battery->status);
    if (mode == kFast) {
        if (FieldCase.consumption >= 10)
            SetChargeCurrent(battery, DYNAMIC_CHARGE_AMPS);
        else
            SetChargeCurrent(battery, MAX_CHARGE_AMPS);
        battery->fast_start_time = battery->chgtmr_update = battery->chgdisp_tmr_update = GetMinute();
        battery->fastcharge_timer = (battery->charge_iset == MAX_CHARGE_AMPS) ? TMAX_5A : TMAX_DYNAMIC;       // 5 or 3.7 amps
        battery->chgdisp_timer = (BAT_MAX_MAH - battery->level) / 1000 / battery->charge_iset * 60 / battery->ng ;
        HAL_GPIO_WritePin(battery->charge_port, battery->charge_pin, GPIO_PIN_SET);
        battery->status = kstsFastCharge;
    } else if (mode == kPreCharge) {
        SetChargeCurrent(battery, PRECHARGE_AMPS);
        battery->pre_start_time = GetMinute();
        battery->chgdisp_timer = 30 + 352;                                      // precharge 30 min, charge 352 min
        HAL_GPIO_WritePin(battery->charge_port, battery->charge_pin, GPIO_PIN_SET);
        battery->status = kstsPreCharge;
    } else if (mode == kTrickle) {
        SetChargeCurrent(battery, TRICKLE_AMPS);
        battery->trickle_start_time = GetMinute();
        HAL_GPIO_WritePin(battery->charge_port, battery->charge_pin, GPIO_PIN_SET);
        battery->status = kstsTrickle;
    } else if (mode == kConstant) {
        SetChargeCurrent(battery, CONSTANT_AMPS);
        battery->tick_constcharge = GetMinute();
//        if (battery->scale_flag & FIRSTCHARGE && battery->level < 14666) {      // equal to 13V bat
//            battery->constant_timer = 660;
//            battery->scale_flag &= ~FIRSTCHARGE;
//        } else
//            battery->constant_timer = (BAT_MAX_MAH - battery->level) / CONSTANT_AMPS / 1000 / 0.9 * 60;  // unit minute
        HAL_GPIO_WritePin(battery->charge_port, battery->charge_pin, GPIO_PIN_SET);
        battery->status = kstsConstant;
    }
}

/**
  * @brief
  * @param  bat: battery to operate
  * @param  option: 0: charge complete, 1: charge terminate
  * @retval None
  */
static void
ChargeOff(stBattery *battery, uint8_t option)
{
    HAL_GPIO_WritePin(battery->charge_port, battery->charge_pin, GPIO_PIN_RESET);

    printf("charge off bat_%d, status %d\n\r", battery->index, battery->status);
    if (battery->status == kstsFastCharge) {
        battery->charge_finish_time = GetMinute();
        if (battery->charge_finish_time - battery->fast_start_time >= 120)      // effective charge time
            battery->charge_times++;
    } else if (battery->status == kstsTrickle) {
        battery->charge_finish_time = GetMinute();
    } else if (battery->status == kstsConstant) {
        battery->constant_timer = 0;
    }

    battery->status = (battery->status == kstsTrickle) ? kstsFinish : kstsFloat;
    battery->peak_volt = 0;
    battery->peak_time = 0;
    battery->fastcharge_timer = 0;
    if (option == 0) {                                                          // charge complete
        battery->scale_flag |= REACHTOP;
        Calibrate(battery, kReachTop);
        battery->scale_flag &= ~REACHBOTTOM;
    } else if (option == 1) {                                                   // charge terminate
        battery->scale_flag = 0x00;
        if (battery->level > battery->capacity) {
            battery->level = battery->capacity;
            if (LTC2943_Write_mAh(battery->gauge, battery->capacity)) {         // uplimit of max electron quantity
                battery->status = kstsError;
                battery->err_code = GAUGE_FAIL;
            }
        }
    }

    memset(battery->v_ringbuf.data, 0, sizeof(battery->v_ringbuf.data[0]));
    battery->v_ringbuf.read_index = 0;
    battery->v_ringbuf.write_index = 0;
    battery->v_ringbuf.clear = false;
}


static HAL_StatusTypeDef
SupplyOn(stBattery *bat)
{
    if (bat->status <= kstsError)
        return HAL_ERROR;

    printf("supply on bat_%d, status %d\n\r", bat->index, bat->status);
    HAL_GPIO_WritePin(bat->supply_port, bat->supply_pin, GPIO_PIN_RESET);
    bat->mux_on = true;
    return HAL_OK;
}

static HAL_StatusTypeDef
SupplyOff(stBattery *bat)
{
    printf("supply off bat_%d, status %d\n\r", bat->index, bat->status);
    HAL_GPIO_WritePin(bat->supply_port, bat->supply_pin, GPIO_PIN_SET);
    bat->mux_on = false;
    return HAL_OK;
}

static bool
CheckConnection(stBattery *bat)
{
    return HAL_GPIO_ReadPin(bat->connect_port, bat->connect_pin);
}

static void
CurveLearning(stBattery *bat)
{
    int8_t cur = bat->current > 0 ? 0 : (int8_t)fabs(bat->current);
    int8_t t = bat->temperature < 0 ? 0 : (int8_t)bat->temperature;
    t = t > 68 ? 22 : t/3;
    bat->red_line[t][cur] = (int8_t)(bat->red_line[t][cur] * 0.5 + bat->level * 0.5);
}

/**
  * @brief  everytime start a charge, clear 'REACHTOP'
            everytime fastcharge finished, set 'REACHTOP, clear 'REACHBOTTOM'
            everytime drop to GC_STOP_TH_VOLT, set 'REACHBOTTOM', clear 'REACHTOP'
            when both 'REACHTOP' and 'REACHBOTTOM' are set, capacity is rescaled  and
            aging is judged at the moment 'finish charge' or 'drop to GC_STOP_TH_VOLT'
  * @param  bat: battery to be checked
  * @param  checkmode: indicate it's a charge cycle or discharge cycle
  * @retval None
  */
static void
Calibrate(stBattery *bat, eCheckMode mode)
{
    bat->gauge->acr_ofuf = 0;
    if (LTC2943_Write_mAh(bat->gauge, bat->level)) {
        ErrorHandler(bat, GAUGE_FAIL);
        return;
    }
    printf("Calibrate(): mode %d, bat_%d scale_flag 0x%02x, capacity %.0f, ng %.3f\n\r", mode, bat->index, bat->scale_flag, bat->capacity, bat->ng);

    // aging detect
    if (bat->is_aged == true)
        return;

    if (mode == kReachTop) {                                                    // check after reach level_top
        if (bat->capacity <= BAT_AGED_MAH_TH) {
            bat->aging_cnt[0]++;
            if (bat->aging_cnt[0] >= 3)
                bat->is_aged = true;
        }
    } else if (mode == kReachBottom) {                                          // check after reach level_bottom
        if (bat->capacity <= BAT_AGED_MAH_TH) {
            bat->aging_cnt[1]++;
            if (bat->aging_cnt[1] >= 3)
                bat->is_aged = true;
        }
    }

    if (bat->charge_times >= 500)
        bat->is_aged = true;
}

static void
RemainTimePredict(stBattery *bat)
{
    uint32_t interval = fabs(bat->current) > 0.2 ? 30 : 300;
    uint32_t current_second = GetSecond();
    uint32_t current_minute = GetMinute();
    int8_t temp = bat->temperature >= 68 ? 68 : (int8_t)bat->temperature;
    int8_t cur = (int8_t)fabs(bat->current);

    if (bat->status >= kstsPreCharge) {                                         // battery is charging
        if (bat->status == kstsConstant) {
            bat->remain_time = bat->constant_timer;
        } else if (bat->status >= kstsPreCharge) {
            if (bat->chgdisp_tmr_update != current_minute) {
                bat->chgdisp_timer -= (current_minute - bat->chgdisp_tmr_update);
                bat->chgdisp_tmr_update = current_minute;
            }
            bat->remain_time = bat->chgdisp_timer;
        }
    } else if (Adaptor.status != kAdaptorNotExist) {                            // battery is floating
        bat->remain_time = 480;
        bat->last_voltage = 0;
        bat->last_level = 0;
    } else if (bat->status == kstsFloat && bat->mux_on == true && bat->remain_time > 0) {        // battery is supplying
        if (bat->voltage > 11 || bat->level > 5000) {                           // calculate based on coulomb
            if (bat->last_level == 0) {
                bat->last_level = bat->level;
                bat->tick_predict = current_second;
            } else {
                if (current_second - bat->tick_predict >= 30) {
                    temp = temp <= 0 ? 0 : temp;
                    temp = temp / 3;
                    cur = cur > 9 ? 9 : cur;
                    cur = cur < 0 ? 0 : cur;
                    if (bat->level < bat->red_line[temp][cur])
                        goto VOLT_PREDICT;
                    bat->remain_time = (bat->level - bat->red_line[temp][cur]) / (bat->last_level - bat->level + 1e-3) * (current_second - bat->tick_predict) / 60;
                    bat->remain_time = bat->remain_time > 0 ? bat->remain_time : 0;             // when GC is off and battery power is low, the level is lower than 3000 but voltage is higher than 11V
                    bat->last_level = bat->level;
                    bat->tick_predict = current_second;
                }
            }
        } else {                                                                // calculate based on voltage
VOLT_PREDICT:
            if (bat->remain_time < 0) {
                // do nothing
            } else if (bat->last_voltage == 0) {                                // means battery just start to use, haven't record last sample voltage
                bat->last_voltage = bat->voltage;
                bat->tick_predict = GetSecond();
            } else if (bat->i_transit_up >= bat->tick_predict || bat->i_transit_down >= bat->tick_predict) {     // this judgement cannot be implement when load transit happens
                bat->last_voltage = bat->voltage;
                bat->tick_predict = current_second;
            } else if (current_second - bat->tick_predict >= interval) {
                float drop = bat->last_voltage - bat->voltage;
                if (bat->voltage <= GC_STOP_TH_VOLT)
                    bat->remain_time = -1;
                else if (drop > 0)
                    bat->remain_time = (bat->voltage - GC_STOP_TH_VOLT) / drop * (current_second - bat->tick_predict) / 60;
                bat->tick_predict = current_second;
                bat->last_voltage = bat->voltage;
            }
        }
    }
}

static bool
CheckFaulty(stBattery *bat)
{
    if (bat->status >= kstsPreCharge) {                                         // only check when charging
        if (bat->current < 0.2) {
            if (bat->cell_open_time == 0) {
                bat->cell_open_time = GetSecond();
            } else if (GetSecond() - bat->cell_open_time >= 10) {
                printf("CheckFaulty(): battery_%d current: %.2f, voltage: %.2f\n\r", bat->index, bat->current, bat->voltage);
                ErrorHandler(bat, CELL_OPEN);
            }
        } else {
            bat->cell_open_time = 0;
        }
        if (bat->peak_volt - bat->voltage >= DROP_VOLTAGE) {
            printf("CheckFaulty(): battery_%d peak_v: %.2f, v: %.2f, i_transit_down time: %d, current second: %d\n\r", bat->index, bat->peak_volt, bat->voltage, bat->i_transit_down, GetSecond());
            ErrorHandler(bat, CELL_SHORT);
        }
    }
    if (bat->current > BAT_ERR_CHARGE_CURRENT || bat->current < BAT_ERR_SUPPLY_CURRENT) {
        printf("CheckFaulty(): battery_%d current: %.2f, voltage: %.2f\n\r", bat->index, bat->current, bat->voltage);
        ErrorHandler(bat, OVER_CURRENT);
    }
    if (bat->temperature > BAT_ERROR_TH_TEMP) {
        if ((bat->mux_on && FieldCase.consumption >= 3 && Adaptor.status == kAdaptorNotExist) ||
                bat->status >= kstsPreCharge) {
            printf("CheckFaulty(): battery_%d temp: %.2f\n\r", bat->index, bat->temperature);
            ErrorHandler(bat, OVER_HEAT);
        }
    }

    if (bat->temperature < -100) {
        if (bat->ntc_open_time == 0) {
            bat->ntc_open_time = GetSecond();
        } else if (GetSecond() - bat->ntc_open_time >= 5)
            ErrorHandler(bat, NTC_OPEN);
    } else
        bat->ntc_open_time = 0;

    return (bat->err_code != 0);
}

static int8_t
CanBeCharged(stBattery *bat)
{
    if (bat->status <= kstsError || bat->status == kstsFinish)
        return 0;

    if ((Adaptor.status != kAdaptorSupplying) || (Adaptor.status == kAdaptorSupplying && GetSecond() - Adaptor.connect_time < 2))
        return 0;

    if (bat->cmdchgoff_timer != -1 && GetMinute() - bat->cmdchgoff_timer < 15)
        return 0;

	if (bat->level <= 0.9 * bat->capacity && bat->voltage <= CHARGE_START_TH_VOLT && bat->temperature < CHARGE_STOP_TH_TEMP - 5) {
        if ((MB_Pwr.status || CCB_Pwr.status) && bat->substitute->status == kstsFastCharge)           // GC current > dual_charge_threshold
            return 1;
        else                                                                                                    // GC current < dual_charge_threshold
            return 2;
    }

	return 0;
}

static int8_t
CheckFastChargeStop(stBattery *bat)
{
    if (bat->voltage > CHARGE_STOP_TH_VOLT + (25 - FieldCase.env_temperature) * 0.01)
        return 2;

    if (bat->fastcharge_timer <= 0)
        return 3;

    if (bat->temperature >= CHARGE_STOP_TH_TEMP)
        return 4;

    if (Adaptor.status != kAdaptorSupplying)
        return 10;

    if (bat->substitute->status == kstsFastCharge && (MB_Pwr.status || CCB_Pwr.status))
        return 11;

    uint32_t current_tick = HAL_GetTick();

    if ((bat->tick_chargecheck < 4294967231 && current_tick - bat->tick_chargecheck >= CHECKCHARGE_INTV) ||     // 2^32 - 1 = 4294967295, to make filter have effect
            (bat->tick_chargecheck > 4294967231 && (4294967295 - bat->tick_chargecheck) + current_tick >= CHECKCHARGE_INTV)) {

        bat->tick_chargecheck = current_tick;

        if (GetSecond() - bat->i_transit_down <= 1) {                           // i_transit_down just happens
            if (bat->v_ringbuf.clear == false) {
                printf("CheckFastChargeStop(): bat_%d clear v_ring_buf\n\r", bat->index);
                for (int i = 0; i < RING_BUF_SIZE; i++)
                    bat->v_ringbuf.data[i] = bat->voltage;
                bat->peak_volt = bat->voltage;
                bat->v_ringbuf.clear = true;
            }
        } else {
            float max = 0;
            bat->v_ringbuf.clear = false;

            bat->v_ringbuf.data[bat->v_ringbuf.write_index++] = bat->voltage;
            bat->v_ringbuf.write_index = bat->v_ringbuf.write_index >= RING_BUF_SIZE ? 0 : bat->v_ringbuf.write_index;

            for (int i = 0; i < RING_BUF_SIZE; i++) {
                max = max < bat->v_ringbuf.data[i] ? bat->v_ringbuf.data[i] : max;
            }

            if (max > bat->peak_volt) {
                bat->peak_volt = max;
                bat->peak_time = GetSecond();
            } else if (bat->peak_volt - max >= DELTA_V) {
                if (GetSecond() - bat->peak_time <= 300) {                      // detect step drop which is spurious ��V
                    // bat->peak_volt = bat->voltage;
                    // bat->peak_time = GetSecond();
                    // do nothing, filter out it
                } else if (bat->level >= bat->capacity * 0.5 && Adaptor.status == kAdaptorSupplying) {
                    return 1;
                }
            }
        }
    }

    return 0;
}

static void
SetChargeCurrent(stBattery *bat, float amps)
{
    uint8_t value = 0;
    if (amps >= 5) {
        bat->charge_iset = 5;
        value = 255;
    } else {
        bat->charge_iset = amps < 0 ? 0 : amps;
        value = (uint8_t)(bat->charge_iset * 10.24);                            // amps / 5A *20e3OHM / 100e3OHM * 256digits
    }
    AD5241_Write((I2C_HandleTypeDef *)bat->hi2c, value);
}

static void
BatteryInfoDeInit(stBattery *battery)
{
    // battery->mode = kGlobal;
    battery->status = kstsNotExist;
    battery->mux_on = false;
    battery->voltage = 0;
    battery->peak_volt = 0;
    battery->last_voltage = 0;
    battery->last_level = 0;
    battery->remain_time = 480;
    battery->current = 0;
    battery->ng = 0.8;
    battery->level = 0;
    battery->capacity = BAT_MAX_MAH;
    battery->scale_flag = 0;
    battery->v_ringbuf.read_index = 0;
    battery->v_ringbuf.write_index = 0;
    battery->v_ringbuf.clear = false;
    memset(battery->v_ringbuf.data, 0, RING_BUF_SIZE * sizeof(battery->v_ringbuf.data[0]));
    battery->temperature = 0;
    battery->is_aged = false;
    memset(battery->aging_cnt, 0, sizeof(uint8_t));
    battery->charge_iset = 0;
    battery->i_transit_up = 0;
    battery->i_transit_down = 0;
    battery->tick_gaugesample = 0;
    battery->tick_predict = 0;
    battery->tick_chargecheck = 0;
    battery->tick_constcharge = 0;
    battery->pre_start_time = 0;
    battery->fast_start_time = 0;
    battery->trickle_start_time = 0;
    battery->charge_finish_time = 0;
    battery->peak_time = 0;
    battery->ntc_open_time = 0;
    battery->cell_open_time = 0;
    battery->fastcharge_timer = 0;
    battery->chgdisp_timer = 0;
    battery->constant_timer = 0;
    battery->cmdchgoff_timer = -1;
    battery->chgtmr_update = 0;
    battery->chgdisp_tmr_update = 0;
    battery->charge_times = 0;
    battery->q_acm_charge = 0;
    battery->q_acm_discharge = 0;
    battery->err_code = 0;
    battery->finish_code = 0;

    battery->gauge->acr_ofuf = 0;
}

static HAL_StatusTypeDef
BatteryInfoInit(stBattery *battery)
{
    int8_t i = 0, j = 0;
    HAL_StatusTypeDef ret = HAL_ERROR;
    BatteryInfoDeInit(battery);
    battery->status = kstsFloat;
    if (battery->mode == kCalifornia) {
//        battery->scale_flag |= FIRSTCHARGE;
    }
    if ((ret = Init_LTC2943(battery->gauge)) != HAL_OK) {
        ErrorHandler(battery, GAUGE_FAIL);
        return ret;
    }

    HAL_Delay(64);                                                              // wait for one conv after init

    ret = HAL_ERROR;
    for (i = 0; i < 3 && ret != HAL_OK; i++)
        ret = Get_Gauge_Information(battery->gauge);
    if (i == 3) {
        ErrorHandler(battery, GAUGE_FAIL);
        return ret;
    }

    battery->voltage = battery->gauge->voltage;

    battery->current = battery->gauge->current;
    
    printf("BatteryInfoInit(): load battery_%d data from eeprom...\n\r", battery->index);
    if (BatteryDataLoad(battery) == HAL_OK) {                                   // information is stored in EEPROM and load successfully
        printf("ok\n\r");
        if (battery->level > 24000 || battery->level < -2000) {
            printf("BatteryInfoInit(): load bat_%d level %.0f is abnormal\n\r", battery->index, battery->level);
            goto INITIALIZE_LEVEL;
        }
        ret = LTC2943_Write_mAh(battery->gauge, battery->level);
        if (ret != HAL_OK) {
            battery->status = kstsError;
            battery->err_code = GAUGE_FAIL;
        }

        return ret;
    } else
        goto INITIALIZE;

INITIALIZE:
    printf("BatteryInfoInit(): INITIALIZE bat_%d matrix\n\r", battery->index);
    for (j = 0; j < RL_DEP; j++) {
        for (i = 0; i < RL_WID; i++) {                                          // -600 is to make red_line be 0 when current is 0 in 25C (9th position in depth, 9*50=450)
            battery->red_line[j][i] = 50*j + 200*i - 450;                       // j is temperature, +50 every 3C, i is current, +200 every 1A
        }                                                                       // note: in test i find v increase when t decrease, which means the lower the temp, the longer the bat can work until 10V -> the red_line is down
    }

INITIALIZE_LEVEL:                                                               // calculate capacity based on voltage
    if (battery->voltage > 14) {
        battery->level = BAT_MAX_MAH;
    } else if (battery->voltage < 11) {
        battery->level = 0;
    } else {
        battery->level = 22e3 * (battery->voltage - 11) / 3;	                // plan to probably over-estimate capacity
    }
    printf("BatteryInfoInit(): INITIALIZE_LEVEL bat_%d level to %.0f with voltage: %.2f, ofuf: %d\n\r", battery->index, battery->level, battery->voltage, battery->gauge->acr_ofuf);
    battery->gauge->acr_ofuf = 0;
    ret = LTC2943_Write_mAh(battery->gauge, battery->level);
    if (ret != HAL_OK) {
        battery->err_code = GAUGE_FAIL;
        battery->status = kstsError;
    }
    return ret;
}

static void
ErrorHandler(stBattery *bat, uint16_t err_code)
{
    bat->status = kstsError;
    bat->err_code |= err_code;
    printf("ErrorHandler(): battery_%d error!! err_code: %2x\n\r", bat->index, err_code);
    BatteryErrorLog(bat);
}

static void
BatteryInfoUpdate(stBattery *battery)
{
    int8_t acr_ofuf;
    uint16_t adccode;
    if (battery->index == 0 || battery->index > 2)                              // index number is 1 or 2 for 2 batteries
        return;

    if (!CheckConnection(battery)) {
        if (battery->status != kstsNotExist) {
            printf("bat_%d disconnect\n\r", battery->index);
            BatteryInfoDeInit(battery);
        }
        return;
    } else if (battery->status == kstsNotExist) {                               // initial for new connection
        printf("bat_% connect\n\r", battery->index);
        BatteryInfoInit(battery);
        return;
    }

    adccode = *(battery->ntc_adc);
    battery->temperature = -33.95 * log((float)adccode) + 269.94;

    acr_ofuf = battery->gauge->acr_ofuf;
    if (Get_Gauge_Information(battery->gauge)) {
        ErrorHandler(battery, GAUGE_FAIL);
        return;
    } else {
        if (acr_ofuf != battery->gauge->acr_ofuf) {
            printf("BatteryInfoUpdate(): bat_%d acr_ofuf is %d, level is %.0f\n\r", battery->index, battery->gauge->acr_ofuf, battery->gauge->level);
        }
        if (battery->status == kstsError && battery->err_code == 0x40) {    // self-recover from gauge communication fail, ONLY if error is GAUGE_FAIL
            battery->status = kstsFloat;
        }
    }

    // get current before voltage for impedance algorithm
    if (battery->gauge->current - battery->current >= 2) {                  // current transient up
        battery->i_transit_up = GetSecond();
    } else if (battery->current - battery->gauge->current >= 2) {           // current transient down
        battery->i_transit_down = GetSecond();
    }
    battery->current = battery->gauge->current;

    // get voltage
    battery->voltage = battery->gauge->voltage;

    battery->level = battery->gauge->level >= 0 ? battery->gauge->level : 0;
    RemainTimePredict(battery);
    CheckFaulty(battery);
}

static void
BatteryChargeControl(stBattery *battery)
{
    int8_t ret;
    if (battery->index == 0 || battery->index > 2)                              // index number is 1 or 2 for 2 batteries
        return;

    switch (battery->status) {
    case kstsNotExist:
    case kstsError:
        HAL_GPIO_WritePin(battery->charge_port, battery->charge_pin, GPIO_PIN_RESET);
        break;
    case kstsFloat:
        if (CanBeCharged(battery) == 2) {
            if (battery->mode == kGlobal) {
                if (battery->voltage <= 9) {
                    ChargeOn(battery, kPreCharge);
                } else {
                    ChargeOn(battery, kFast);
                }
            } else if (battery->mode == kCalifornia) {
                ChargeOn(battery, kConstant);
            }
        }
        break;
    case kstsPreCharge:
        if (Adaptor.status != kAdaptorSupplying) {
            ChargeOff(battery, 1);
        } else if (GetMinute() - battery->pre_start_time >= PRECHARGE_TIME) {    // pre-charge time over
            ChargeOn(battery, kFast);
        }
        break;
    case kstsFastCharge:
        if (ret = CheckFastChargeStop(battery)) {
            if (ret > 1) {
                printf("ChargeControl(): terminate bat_%d fast-charge because of condition_%d\n\r", battery->index, ret);
                ChargeOff(battery, 1);
            } else if (ret == 1) {
                printf("ChargeControl(): finish bat_%d fast-charge\n\r", battery->index);
                ChargeOff(battery, 0);
                ChargeOn(battery, kTrickle);                                    // set 0.1C trickle charge
            }
            battery->finish_code = ret;
        } else {
            if (FieldCase.consumption >= 10 && battery->charge_iset != DYNAMIC_CHARGE_AMPS) {
                printf("ChargeControl(): FastCharge change bat_%d charge_iset to %.1f\n\r", battery->index, DYNAMIC_CHARGE_AMPS);
                SetChargeCurrent(battery, DYNAMIC_CHARGE_AMPS);                 // set new charge current
                battery->fastcharge_timer *= MAX_CHARGE_AMPS / DYNAMIC_CHARGE_AMPS;
                battery->chgdisp_timer *= MAX_CHARGE_AMPS / DYNAMIC_CHARGE_AMPS;
            } else if (FieldCase.consumption <= 7 && GetSecond() - FieldCase.i_transit_down > 5 && battery->charge_iset != MAX_CHARGE_AMPS) {
                printf("ChargeControl(): FastCharge change bat_%d charge_iset to %.1f\n\r", battery->index, MAX_CHARGE_AMPS);
                SetChargeCurrent(battery, MAX_CHARGE_AMPS);
                battery->fastcharge_timer *= DYNAMIC_CHARGE_AMPS / MAX_CHARGE_AMPS;
                battery->chgdisp_timer *= DYNAMIC_CHARGE_AMPS / MAX_CHARGE_AMPS;
            }
            if (battery->chgtmr_update != GetMinute()) {
                battery->fastcharge_timer -= GetMinute() - battery->chgtmr_update;
                battery->chgtmr_update = GetMinute();
            }
        }
        break;
    case kstsTrickle:
        if (Adaptor.status != kAdaptorSupplying) {
            ChargeOff(battery, 1);
//            battery->scale_flag |= REACHTOP;
        } else if (battery->voltage >= CHARGE_STOP_TH_VOLT || GetMinute() - battery->trickle_start_time >= TRICKLE_TIME) {    // trickle charge time over
            ChargeOff(battery, 1);
//            battery->scale_flag |= REACHTOP;
        }                                                                       // do nothing for the rest condition, wait for trickle timeout
        break;
    case kstsConstant:
        if (Adaptor.status == kAdaptorNotExist) {
            ChargeOff(battery, 1);
        } else if (battery->voltage >= CHARGE_STOP_TH_VOLT) {
            ChargeOff(battery, 0);
            battery->finish_code = 0x11;
        } else if (GetMinute() != battery->tick_constcharge) {
            battery->constant_timer -= (GetMinute() - battery->tick_constcharge);
            battery->tick_constcharge = GetMinute();
            if (battery->constant_timer <= 0) {
                ChargeOff(battery, 0);
                battery->finish_code = 0x10;
            }
            if (fabs(battery->current - CONSTANT_AMPS) >= 0.05) {               // close-loop charge-current control
                if (fabs(battery->current - CONSTANT_AMPS) > 1)
                    printf("ChargeControl(): constant charge abnormal\n\r");
                battery->charge_iset += CONSTANT_AMPS - battery->current;
                SetChargeCurrent(battery, battery->charge_iset);
                printf("ChargeControl(): ConstantCharge change bat_%d charge_iset to %.2f\n\r", battery->index, battery->charge_iset);
            }
        }
    case kstsFinish:                                                            // use to inform, and prevent charge again
        if ((MB_Pwr.status || CCB_Pwr.status) && battery->mux_on && Adaptor.status == kAdaptorNotExist)
            battery->status = kstsFloat;
        break;
    default : break;
    }
}

static void
BatterySupplyControl(stBattery *bat)
{
    if (bat->index == 0 || bat->index > 2)                                      // index number is 1 or 2 for 2 batteries
        return;

    if (bat->mux_on == true) {                                                  // battery is supplying
        if (bat->status <= kstsError) {
            if (bat->substitute->status > kstsError) {
                SupplyOn(bat->substitute);
                HAL_Delay(1);
            }
            SupplyOff(bat);
        } else {
            if (bat->voltage <= (GC_STOP_TH_VOLT + (bat->voltage - FieldCase.v_syspwr + 0.1)) && GetSecond() - bat->i_transit_down > 5) {     // capacity is low, and prevent spurious trigger
                if (bat->substitute->status <= kstsError) {
                    if (lo_pwr_beep == false) {
                        lo_pwr_beep = true;
                        printf("BatterySupplyControl(): battery_%d  capacity is low\n\r", bat->index);
                    }
                } else if (bat->substitute->remain_time < 0 || bat->substitute->mux_on == true) {   // another bat is unavailable or dual-mode is on
                    if (lo_pwr_beep == false) {
                        lo_pwr_beep = true;
                        printf("BatterySupplyControl(): battery_%d both battery capacity are low\n\r", bat->index);
                    }
                } else {                                                        // another is available
                    SupplyOn(bat->substitute);
                    HAL_Delay(1);
                    SupplyOff(bat);
                    printf("BatterySupplyControl(): battery_%d capacity is low, switch to battery_%d\n\r", bat->index, bat->substitute->index);
                }
                if (bat->remain_time >= 0) {
                    bat->remain_time = -1;
//                    bat->scale_flag |= REACHBOTTOM;
                    Calibrate(bat, kReachBottom);
//                    bat->scale_flag &= ~REACHTOP;
                }
            } else if (need_dual_bat == false && bat->substitute->mux_on == true) {
                if (bat->level <= bat->substitute->level){
                    printf("BatterySupplyControl(): turn off bat_%d for single mode\n\r", bat->index);
                    SupplyOff(bat);
                }
            }
        }
    } else if (bat->status > kstsError) {                                       // battery mux_on == false
        if (bat->substitute->status <= kstsError) {                             // turn on this battery prior to faulty another
            SupplyOn(bat);
        } else if (need_dual_bat == true) {
            if (bat->remain_time != -1) {
                printf("BatterySupplyControl(): turn on bat_%d for dual mode\n\r", bat->index);
                SupplyOn(bat);
            }
        } else {
            if (bat->substitute->mux_on == false &&                             // no one is supplying
                    (bat->level > bat->substitute->level || bat->substitute->remain_time == -1))
                SupplyOn(bat);
        }
    }

    if (Adaptor.status == kAdaptorSupplying)
        lo_pwr_beep = false;
}

void
Battery_HwInit(void)
{
    // BatteryDataClear(&Battery_1);
    // BatteryDataClear(&Battery_2);
    // ChargeOff(&Battery_1);
    // ChargeOff(&Battery_2);

    Battery_1.mode = kGlobal;
    Battery_2.mode = kGlobal;

    if (CheckConnection(&Battery_1)) {
        BatteryInfoInit(&Battery_1);
        CheckFaulty(&Battery_1);
    } else {
        if (BatteryDataClear(&Battery_1) == HAL_OK)
            printf("Battery_HwInit(): clear battery_1 data\n\r");
    }
    if (CheckConnection(&Battery_2)) {
        BatteryInfoInit(&Battery_2);
        CheckFaulty(&Battery_2);
    } else {
        if (BatteryDataClear(&Battery_2) == HAL_OK)
            printf("Battery_HwInit(): clear battery_2 data\n\r");
    }

    if (Battery_1.status > kstsError && Battery_2.status > kstsError) {
        if (Battery_1.level >= Battery_2.level) {
            SupplyOff(&Battery_2);
            SupplyOn(&Battery_1);
        } else {
            SupplyOff(&Battery_1);
            SupplyOn(&Battery_2);
        }
    } else if (Battery_1.status > kstsError) {
        SupplyOff(&Battery_2);
        SupplyOn(&Battery_1);
    } else if (Battery_2.status > kstsError) {
        SupplyOff(&Battery_1);
        SupplyOn(&Battery_2);
    } else {
        SupplyOff(&Battery_1);
        SupplyOff(&Battery_2);
    }
}

void
ChargeCmdhandler(stBattery *bat, bool request)
{
    if (request == ON) {
        bat->cmdchgoff_timer = -1;
        if (bat->status == kstsFloat) {
            int8_t ret = CanBeCharged(bat);
            if (ret == 0)
                return;
            if (ret == 1)
                ChargeOff(bat->substitute, 1);
            if (bat->mode == kGlobal) {
                if (bat->voltage <= 9)
                    ChargeOn(bat, kPreCharge);
                else
                    ChargeOn(bat, kFast);
            } else if (bat->mode == kCalifornia) {
                ChargeOn(bat, kConstant);
            }
        }
    } else if (request == OFF) {
        bat->cmdchgoff_timer = GetMinute();
        ChargeOff(bat, 1);
    }
}

void
BatterySupplyCmdHandler(stBattery *bat, bool request)
{
    if (bat->index == 0 || bat->index > 2)                                      // index number is 1 or 2 for 2 batteries
        return;

    if (bat->mux_on == true && request == OFF) {
        if (bat->substitute->status > kstsError && bat->substitute->remain_time != -1) {
            SupplyOn(bat->substitute);
            HAL_Delay(1);
        }
        SupplyOff(bat);
    } else if (bat->mux_on == false && request == ON) {
        SupplyOn(bat);
        if (need_dual_bat == false) {
            HAL_Delay(1);
            SupplyOff(bat->substitute);
        }
    }
}

void Thread_BatteryControl(void const *param)
{
    while (1) {
        BatteryChargeControl(&Battery_1);
        BatteryChargeControl(&Battery_2);
        BatterySupplyControl(&Battery_1);
        BatterySupplyControl(&Battery_2);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void Thread_BatteryMonitor(void const *param)
{
    while (1) {

        if (xSemaphoreTake(((stBattery*)param)->mutex_i2c, 5000) == pdTRUE) {
            BatteryInfoUpdate(((stBattery*)param));
            xSemaphoreGive(((stBattery*)param)->mutex_i2c);
        } else {
            printf("Thread_BatteryMonitor: i2c blocked, battery_%d\n\r", ((stBattery*)param)->index);
        }
        xSemaphoreGive(((stBattery*)param)->sem_monitor);

        vTaskDelay(64 / portTICK_PERIOD_MS);
    }
}


