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
#include "app_include.h"

/* Private macro -------------------------------------------------------------*/
#define TMAX_5A             343                                                 // max fast charge time limit
#define TMAX_DYNAMIC        463                                                 // max fast charge time limit
#define MAX_CHARGE_AMPS     5.0F
#define DYNAMIC_CHARGE_AMPS 3.7F
#define PRECHARGE_AMPS      1                                                   // 1amps precharge current
#define TRICKLE_AMPS        1.0                                                 // 0.5amps trickle current
#define CONSTANT_AMPS       2.0F                                                // 1amps constant current for 'california mode'
#define VSAMPLE_INTV        64                                                  // 64ms
#define CHECKCHARGE_INTV    64                                                  // 64ms
#define HEAVY_TRANSIT_TH    2                                                   // record heavy_transit when load current step increase > this amps
#define I_CAL_TH            -2.5F                                               // calibrate battery when reach bottom and supply current < this

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

bool lo_pwr_beep = false;


/* Code begin ----------------------------------------------------------------*/
static void ChargeOn(stBattery *battery, eChargeMode mode)
{
    if (battery->status == kstsError || battery->status == kstsFinish)
        return;

    battery->peak_volt = 0;
    battery->peak_time = 0;
    battery->scale_flag &= ~REACHTOP;
    memset(battery->v_ringbuf.data, 0, RING_BUF_SIZE * sizeof(battery->v_ringbuf.data[0]));
    battery->v_ringbuf.read_index = 0;
    battery->v_ringbuf.write_index = 0;
    battery->v_ringbuf.clear = false;
    battery->finish_code = 0;
    battery->cell_open_time = 0;
    battery->cell_short_time = 0;
    battery->ntc_open_time = 0;

    xprintf("charge on bat_%d, mode=%d, status %d\n\r", battery->index, mode, battery->status);
    if (mode == kFast) {
        if (FieldCase.consumption >= 10)
            SetChargeCurrent(battery, DYNAMIC_CHARGE_AMPS);
        else
            SetChargeCurrent(battery, MAX_CHARGE_AMPS);
        battery->fast_start_time = battery->chgtmr_update = GetMinute();
        battery->fastcharge_timer = (battery->charge_iset == MAX_CHARGE_AMPS) ? TMAX_5A : TMAX_DYNAMIC;       // 5 or 3.7 amps
        HAL_GPIO_WritePin(battery->charge_port, battery->charge_pin, GPIO_PIN_SET);
        battery->status = kstsFastCharge;
        battery->q_start_charge = battery->level;
    } else if (mode == kPreCharge) {
        SetChargeCurrent(battery, PRECHARGE_AMPS);
        battery->pre_start_time = GetMinute();
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
        if (battery->scale_flag & FIRSTCHARGE && battery->level < 14666) {      // equal to 13V bat
            battery->constant_timer = 660;
            battery->scale_flag &= ~FIRSTCHARGE;
        } else {
            battery->constant_timer = battery->voltage >= 12 ?
                                      (BAT_MAX_MAH - battery->level) / CONSTANT_AMPS / 1000 / 0.9 * 60 :
                                      660;
        }

        HAL_GPIO_WritePin(battery->charge_port, battery->charge_pin, GPIO_PIN_SET);
        battery->status = kstsConstant;
    }
}

/**
  * @brief
  * @param  bat: battery to operate
  * @param  option: 0: charge complete, 1: terminate at fast-charge,
  * @retval None
  */
static void ChargeOff(stBattery *battery, uint8_t option)
{
    HAL_GPIO_WritePin(battery->charge_port, battery->charge_pin, GPIO_PIN_RESET);
    SetChargeCurrent(battery, 0);

    printf("charge off bat_%d, status %d\n\r", battery->index, battery->status);
    if (battery->status == kstsFastCharge) {
        battery->charge_finish_time = GetMinute();
        battery->peak_volt = 0;
        battery->peak_time = 0;
        battery->fastcharge_timer = 0;
        if (battery->charge_finish_time - battery->fast_start_time >= 120)      // effective charge time
            battery->charge_times++;
        battery->fast_start_time = 0;
    } else if (battery->status == kstsTrickle) {
        battery->charge_finish_time = GetMinute();
    } else if (battery->status == kstsConstant) {
        battery->constant_timer = 0;
        battery->trickle_start_time = 0;
    }

    if (option == 0) {                                                          // charge complete
        battery->scale_flag |= REACHTOP;
        Calibrate(battery, kReachTop);
        battery->scale_flag &= ~REACHBOTTOM;
    } else if (option == 1) {                                                   // charge terminate
        if (battery->status == kstsFastCharge)
            battery->level = (battery->level - battery->q_start_charge ) * battery->ng + battery->q_start_charge;
        battery->scale_flag = 0x00;
        if (battery->level > battery->capacity) {
            battery->level = battery->capacity;
        }
        if (LTC2943_Write_mAh(battery->gauge, battery->level, battery->iic_mutex)) {         // uplimit of max electron quantity
            battery->status = kstsError;
            battery->err_code = GAUGE_FAIL;
        }
    }
    memset(battery->v_ringbuf.data, 0, sizeof(battery->v_ringbuf.data[0]));
    battery->v_ringbuf.read_index = 0;
    battery->v_ringbuf.write_index = 0;
    battery->v_ringbuf.clear = false;
    battery->status = option == 0 ? kstsFinish : kstsFloat;
}


static HAL_StatusTypeDef SupplyOn(stBattery *bat)
{
    if (bat->status <= kstsError)
        return HAL_ERROR;

    xprintf("supply on bat_%d, status %d\n\r", bat->index, bat->status);
    HAL_GPIO_WritePin(bat->supply_port, bat->supply_pin, GPIO_PIN_RESET);
    bat->mux_on = true;
    bat->i_transit_down = GetSecond();
    return HAL_OK;
}

static HAL_StatusTypeDef SupplyOff(stBattery *bat)
{
    xprintf("supply off bat_%d, status %d\n\r", bat->index, bat->status);
    HAL_GPIO_WritePin(bat->supply_port, bat->supply_pin, GPIO_PIN_SET);
    bat->mux_on = false;
    return HAL_OK;
}

static bool CheckConnection(stBattery *bat)
{
    return HAL_GPIO_ReadPin(bat->connect_port, bat->connect_pin);
}

static void CurveLearning(stBattery *bat)
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
static void Calibrate(stBattery *bat, eCheckMode mode)
{
    // level and capacity rescale
    if (mode == kReachTop) {
        if (bat->scale_flag & 0x03 == 0x03)
            bat->ng = 0.5 * bat->ng + 0.5 * bat->q_acm_discharge / (bat->level - bat->capacity + bat->q_acm_discharge);
        bat->level = bat->gauge->level = bat->capacity;
    } else if (mode == kReachBottom) {
        if ((bat->scale_flag & 0x03) == 0x03) {
            bat->q_acm_discharge = bat->capacity - bat->level;
            if (bat->current > -3 && bat->current < 0)
                bat->capacity = 0.5 * bat->capacity + 0.5 * (bat->capacity - bat->gauge->level);
            CurveLearning(bat);
        }
        bat->level = bat->gauge->level = 0;
    }
    bat->gauge->acr_ofuf = 0;
    if (LTC2943_Write_mAh(bat->gauge, bat->level, bat->iic_mutex)) {
        ErrorHandler(bat, GAUGE_FAIL);
        return;
    }
    xprintf("Calibrate(): mode %d, bat_%d scale_flag 0x%02x, capacity %.0f, ng %.3f\n\r", mode, bat->index, bat->scale_flag, bat->capacity, bat->ng);

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

static void RemainTimePredict(stBattery *bat)
{
    uint32_t interval = fabs(bat->current) > 0.2 ? 30 : 300;
    uint32_t current_second = GetSecond();
    uint32_t current_minute = GetMinute();
    int8_t temp = bat->temperature >= 68 ? 68 : (int8_t)bat->temperature;
    int8_t cur = (int8_t)fabs(bat->current);
    float v_remain_time = bat->remain_time;
    float l_remain_time = bat->remain_time;

    if ((bat->status >= kstsPreCharge) || bat->status <= kstsError) {           // battery is not supplying
        bat->remain_time = 480;
        return;
    }
    if (Adaptor.status != kAdaptorNotExist) {                                   // battery is floating
        bat->last_voltage = 0;
        bat->last_level = 0;
        bat->remain_time = bat->remain_time < 0 ? 0 : bat->remain_time;
        return;
    }

    if (bat->status == kstsFloat && bat->mux_on == true) {                      // battery is supplying
        // volt predict
        if (bat->last_voltage == 0 ||                                                                       // means battery just start to use, haven't record last sample voltage
                (bat->i_transit_up >= bat->tick_vpredict || bat->i_transit_down >= bat->tick_vpredict)) {   // this judgement cannot be implement when load transit happens
            bat->last_voltage = bat->voltage;
            bat->tick_vpredict = current_second;
        } else if (current_second - bat->tick_vpredict >= interval) {
            float drop = bat->last_voltage - bat->voltage;
            if (bat->voltage <= GC_STOP_TH_VOLT)
                v_remain_time = -1;
            else if (drop > 0) {
                v_remain_time = (bat->voltage - GC_STOP_TH_VOLT) / drop * (current_second - bat->tick_vpredict) / 60;
                v_remain_time = v_remain_time > 0 ? v_remain_time : 0;
            }
            bat->tick_vpredict = current_second;
            bat->last_voltage = bat->voltage;
        }

        // coulomb predict
        if (bat->last_level == 0) {
            bat->last_level = bat->level;
            bat->tick_lpredict = current_second;
        } else {
            if (current_second - bat->tick_lpredict >= 30) {
                bat->coulomb_per_prd = (bat->last_level - bat->level + 1e-3) / (current_second - bat->tick_lpredict);
                temp = temp <= 0 ? 0 : temp / 3;
                cur = cur > 9 ? 9 : cur;
                cur = cur < 0 ? 0 : cur;
                if (bat->level < bat->red_line[temp][cur])
                    l_remain_time = v_remain_time;
                else {
                    l_remain_time = (bat->level - bat->red_line[temp][cur]) / bat->coulomb_per_prd / 60;
                    l_remain_time = l_remain_time > 0 ? l_remain_time : 0;                                  // when GC is off and battery power is low, the level is lower than 3000 but voltage is higher than 11V
                    bat->last_level = bat->level;
                    bat->tick_lpredict = current_second;
                }
            }
        }
        // update remain time
        if (bat->remain_time >= 0) {
            bat->remain_time  = (bat->voltage > 11 && bat->level > 5000) ? l_remain_time : v_remain_time;
            if (bat->remain_time < 0)
                BatteryDataSave(bat);
        }
    } else {                                                                                                // bat is not used but to merge remain-time
        if (bat->remain_time > 0 && bat->substitute->coulomb_per_prd != 0) {
            temp = temp <= 0 ? 0 : temp / 3;
            cur = cur > 9 ? 9 : cur;
            cur = cur < 0 ? 0 : cur;
            bat->remain_time = (bat->level - bat->red_line[temp][cur]) / bat->substitute->coulomb_per_prd / 60;
            bat->remain_time = bat->remain_time >= 0 ? bat->remain_time : 0;
        }
    }
}

static bool CheckFaulty(stBattery *bat)
{
    if (bat->status >= kstsPreCharge && GetSecond() - bat->i_transit_down > 3 && GetSecond() - Adaptor.disconnect_time >= 3) {  // only check when charging

        if (bat->current < 0.2) {
            if (bat->cell_open_time == 0) {
                bat->cell_open_time = GetSecond();
            } else if (GetSecond() - bat->cell_open_time >= 10) {
                xprintf("CheckFaulty(): battery_%d current: %.2f, voltage: %.2f\n\r", bat->index, bat->current, bat->voltage);
                ErrorHandler(bat, CELL_OPEN);
            }
        } else {
            bat->cell_open_time = 0;
        }
        if (bat->peak_volt - bat->voltage >= DROP_VOLTAGE) {
            if (bat->cell_short_time == 0) {
                bat->cell_short_time = GetSecond();
            } else if (GetSecond() - bat->cell_short_time > 3) {
                xprintf("CheckFaulty(): battery_%d peak_v: %.2f, v: %.2f, i_transit_down time: %d, current second: %d\n\r", bat->index, bat->peak_volt, bat->voltage, bat->i_transit_down, GetSecond());
                ErrorHandler(bat, CELL_SHORT);
            }
        } else
            bat->cell_short_time = 0;
    }

    if (bat->current > BAT_ERR_CHARGE_CURRENT || bat->current < BAT_ERR_SUPPLY_CURRENT) {
        xprintf("CheckFaulty(): battery_%d current: %.2f, voltage: %.2f\n\r", bat->index, bat->current, bat->voltage);
        ErrorHandler(bat, OVER_CURRENT);
    }

    if (bat->temperature > BAT_ERROR_TH_TEMP) {
        if ((bat->mux_on && FieldCase.consumption >= 3 && Adaptor.status == kAdaptorNotExist) ||
                bat->status >= kstsPreCharge) {
            xprintf("CheckFaulty(): battery_%d temp: %.2f\n\r", bat->index, bat->temperature);
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

static int8_t CanBeCharged(stBattery *bat)
{
    if (bat->status <= kstsError || bat->status == kstsFinish)
        return 0;

    if ((Adaptor.status != kAdaptorSupplying) || (Adaptor.status == kAdaptorSupplying && GetSecond() - Adaptor.connect_time < 2))
        return 0;

    if (bat->cmdchgoff_timer != -1 && GetMinute() - bat->cmdchgoff_timer < 15)
        return 0;

    if (bat->temperature >= CHARGE_START_TH_TEMP) {
        bat->finish_code = 20;
        return 0;
    }

	if (bat->level <= 0.9 * bat->capacity && bat->voltage <= CHARGE_START_TH_VOLT && bat->temperature < CHARGE_STOP_TH_TEMP - 5) {
        if ((MB_Pwr.status || CCB_Pwr.status) && bat->substitute->status == kstsFastCharge)           // GC current > dual_charge_threshold
            return 1;
        else                                                                                          // GC current < dual_charge_threshold
            return 2;
    }

	return 0;
}

static int8_t CheckFastChargeStop(stBattery *bat)
{
    // protection related
    if (bat->voltage > CHARGE_STOP_TH_VOLT + (25 - FieldCase.env_temperature) * 0.01)
        return 2;

    if (bat->fastcharge_timer <= 0)
        return 3;

    if (bat->temperature >= CHARGE_STOP_TH_TEMP)
        return 4;

    // control logic related
    if (Adaptor.status != kAdaptorSupplying)
        return 10;

    if (bat->substitute->status == kstsFastCharge && (MB_Pwr.status || CCB_Pwr.status))
        return 11;

    if (bat->mode != kGlobal)
        return 12;

    uint32_t current_tick = HAL_GetTick();

    if (GetSecond() - bat->i_transit_down <= 1) {                               // i_transit_down just happens
        if (bat->v_ringbuf.clear == false) {
            xprintf("CheckFastChargeStop(): bat_%d clear v_ring_buf\n\r", bat->index);
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
            if (GetSecond() - bat->peak_time <= 300) {                          // detect step drop which is spurious ��V
                // bat->peak_volt = bat->voltage;
                // bat->peak_time = GetSecond();
                // do nothing, filter out it
            } else if (bat->level >= bat->capacity * 0.5 && Adaptor.status == kAdaptorSupplying) {
                return 1;
            }
        }
    }

    return 0;
}

static void SetChargeCurrent(stBattery *bat, float amps)
{
    uint8_t value = 0;
    if (amps >= 5) {
        bat->charge_iset = 5;
        value = 255;
    } else {
        bat->charge_iset = amps < 0 ? 0 : amps;
        value = (uint8_t)(bat->charge_iset * 10.24);                            // amps / 5A *20e3OHM / 100e3OHM * 256digits
    }
    AD5241_Write((I2C_HandleTypeDef *)bat->hi2c, value, bat->iic_mutex);
}

static void BatteryInfoDeInit(stBattery *battery)
{
    if (battery->index == 0 || battery->index > 2)
        return;

    // battery->mode = kGlobal;
    battery->status = kstsNotExist;
    battery->mux_on = false;
    battery->voltage = 0;
    battery->peak_volt = 0;
    battery->volt_sample = 0;
    battery->last_voltage = 0;
    battery->last_level = 0;
    battery->coulomb_per_prd = 0;
    battery->remain_time = 480;
    battery->current = 0;
    battery->ng = 0.8;
    battery->level = 0;
    battery->capacity = 0;
    battery->scale_flag = FIRSTCHARGE;
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
    battery->tick_vpredict = 0;
    battery->tick_lpredict = 0;
    battery->tick_constcharge = 0;
    battery->tick_volt_sample = 0;
    battery->pre_start_time = 0;
    battery->fast_start_time = 0;
    battery->trickle_start_time = 0;
    battery->charge_finish_time = 0;
    battery->peak_time = 0;
    battery->ntc_open_time = 0;
    battery->cell_open_time = 0;
    battery->cell_short_time = 0;
    battery->fastcharge_timer = 0;
    battery->constant_timer = 0;
    battery->cmdchgoff_timer = -1;
    battery->chgtmr_update = 0;
    battery->charge_times = 0;
    battery->q_acm_discharge = 0;
    battery->q_start_charge = 0;
    battery->q_estimate = 0;
    battery->err_code = 0;
    battery->finish_code = 0;
    battery->null_dev = NULL;

    battery->gauge->acr_ofuf = 0;
}

static HAL_StatusTypeDef BatteryInfoInit(stBattery *battery)
{
    int8_t i = 0, j = 0;
    HAL_StatusTypeDef ret = HAL_ERROR;

    if (battery->index == 0 || battery->index > 2)
        return HAL_ERROR;

    BatteryInfoDeInit(battery);
    battery->status = kstsFloat;

    if ((ret = Init_LTC2943(battery->gauge, battery->iic_mutex)) != HAL_OK) {
        ErrorHandler(battery, GAUGE_FAIL);
        return ret;
    }

    osDelay(64);                                                                // wait for one conv after init

    ret = HAL_ERROR;
    for (i = 0; i < 3 && ret != HAL_OK; i++)
        ret = Get_Gauge_Information(battery->gauge, battery->iic_mutex);
    if (i == 3) {
        ErrorHandler(battery, GAUGE_FAIL);
        return ret;
    }

    battery->voltage = battery->gauge->voltage;

    battery->current = battery->gauge->current;

    xprintf("BatteryInfoInit(): load battery_%d data from eeprom...\n\r", battery->index);
    if (BatteryDataLoad(battery) == HAL_OK) {                                   // information is stored in EEPROM and load successfully
        xprintf("ok\n\r");
        if (battery->level > 24000 || battery->level < -2000) {
            xprintf("BatteryInfoInit(): load bat_%d level %.0f is abnormal\n\r", battery->index, battery->level);
            goto INITIALIZE_LEVEL;
        }
        ret = LTC2943_Write_mAh(battery->gauge, battery->level, battery->iic_mutex);
        if (ret != HAL_OK) {
            battery->status = kstsError;
            battery->err_code = GAUGE_FAIL;
        }

        return ret;
    } else
        goto INITIALIZE;

INITIALIZE:
    xprintf("BatteryInfoInit(): INITIALIZE bat_%d\n\r", battery->index);
    for (j = 0; j < RL_DEP; j++) {
        for (i = 0; i < RL_WID; i++) {                                          // -600 is to make red_line be 0 when current is 0 in 25C (9th position in depth, 9*50=450)
            battery->red_line[j][i] = 50*j + 200*i - 450;                       // j is temperature, +50 every 3C, i is current, +200 every 1A
        }                                                                       // note: in test i find v increase when t decrease, which means the lower the temp, the longer the bat can work until 10V -> the red_line is down
    }
    battery->capacity = BAT_MAX_MAH;

INITIALIZE_LEVEL:                                                               // calculate capacity based on voltage
    if (battery->voltage > 14) {
        battery->level = battery->capacity;
    } else if (battery->voltage < 11) {
        battery->level = 0;
    } else {
        battery->level = 22e3 * (battery->voltage - 11) / 3;	                // plan to probably over-estimate capacity
    }
    xprintf("BatteryInfoInit(): INITIALIZE_LEVEL bat_%d level to %.0f with voltage: %.2f, ofuf: %d\n\r", battery->index, battery->level, battery->voltage, battery->gauge->acr_ofuf);
    battery->gauge->acr_ofuf = 0;
    ret = LTC2943_Write_mAh(battery->gauge, battery->level, battery->iic_mutex);
    if (ret != HAL_OK) {
        battery->err_code = GAUGE_FAIL;
        battery->status = kstsError;
    }
    return ret;
}

static void ErrorHandler(stBattery *bat, uint16_t err_code)
{
    TaskHandle_t tid = xTaskGetCurrentTaskHandle();
    char *s = pcTaskGetName(tid);
    bat->status = kstsError;
    bat->err_code |= err_code;
    xprintf("battery_%d error!! code: %2x, task: %s\n\r", bat->index, err_code, s);
    BatteryErrorLog(bat);
}

void Thread_BatteryChargeControl(stBattery *battery)
{
    uint32_t current_minute;
    int8_t ret = 0;

    xprintf("thread_bat%dchg start\n\r", battery->index);

    while (1) {
        current_minute = GetMinute();

        if (battery->index == 0 || battery->index > 2) {                        // index number is 1 or 2 for 2 batteries
            xprintf("bat_chargecontrol:wrong bat index%d\n\r", battery->index);
            vTaskDelete(NULL);
        }

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
            } else if (current_minute - battery->pre_start_time >= PRECHARGE_TIME) {    // pre-charge time over
                ChargeOn(battery, kFast);
            }
            break;
        case kstsFastCharge:
            if (ret = CheckFastChargeStop(battery)) {
                if (ret > 1) {
                    xprintf("ChargeControl(): terminate bat_%d fast-charge because of condition_%d\n\r", battery->index, ret);
                    ChargeOff(battery, 1);
                } else if (ret == 1) {
                    xprintf("ChargeControl(): finish bat_%d fast-charge\n\r", battery->index);
                    ChargeOff(battery, 0);
                    ChargeOn(battery, kTrickle);                                        // set 0.1C trickle charge
                }
                battery->finish_code = ret;
            } else {
                if (FieldCase.consumption >= 10 && battery->charge_iset != DYNAMIC_CHARGE_AMPS) {
                    xprintf("ChargeControl(): FastCharge change bat_%d charge_iset to %.1f\n\r", battery->index, DYNAMIC_CHARGE_AMPS);
                    SetChargeCurrent(battery, DYNAMIC_CHARGE_AMPS);                     // set new charge current
                    battery->fastcharge_timer *= MAX_CHARGE_AMPS / DYNAMIC_CHARGE_AMPS;
                } else if (FieldCase.consumption <= 7 && GetSecond() - FieldCase.i_transit_down > 5 && battery->charge_iset != MAX_CHARGE_AMPS) {
                    xprintf("ChargeControl(): FastCharge change bat_%d charge_iset to %.1f\n\r", battery->index, MAX_CHARGE_AMPS);
                    SetChargeCurrent(battery, MAX_CHARGE_AMPS);
                    battery->fastcharge_timer *= DYNAMIC_CHARGE_AMPS / MAX_CHARGE_AMPS;
                }
                if (battery->chgtmr_update != current_minute) {
                    battery->fastcharge_timer -= current_minute - battery->chgtmr_update;
                    battery->chgtmr_update = current_minute;
                }
            }
            break;
        case kstsTrickle:
            if (Adaptor.status != kAdaptorSupplying ||
                    (battery->voltage >= CHARGE_STOP_TH_VOLT || current_minute - battery->trickle_start_time >= TRICKLE_TIME)) {    // trickle charge time over
                ChargeOff(battery, 0);
                battery->scale_flag |= REACHTOP;
            }                                                                   // do nothing for the rest condition, wait for trickle timeout
            break;
        case kstsConstant:
            if (Adaptor.status == kAdaptorNotExist) {
                ChargeOff(battery, 1);
                battery->finish_code = 0;
            } else if (battery->voltage >= CHARGE_STOP_TH_VOLT) {
                ChargeOff(battery, 0);
                battery->finish_code = 0x11;
            } else if (current_minute != battery->tick_constcharge) {
                battery->constant_timer -= (current_minute - battery->tick_constcharge);
                battery->tick_constcharge = current_minute;
                if (battery->constant_timer <= 0) {
                    ChargeOff(battery, 0);
                    battery->finish_code = 0x10;
                }
                if (fabs(battery->current - CONSTANT_AMPS) >= 0.05) {           // close-loop charge-current control
                    if (fabs(battery->current - CONSTANT_AMPS) > 1)
                        xprintf("ChargeControl(): bat%d constant charge abnormal: %.2f\n\r", battery->index, battery->current);
                    battery->charge_iset += CONSTANT_AMPS - battery->current;
                    SetChargeCurrent(battery, battery->charge_iset);
                    xprintf("ChargeControl(): ConstantCharge change bat_%d charge_iset to %.2f\n\r", battery->index, battery->charge_iset);
                }
            }
        case kstsFinish:                                                        // use to inform, and prevent charge again
            if ((MB_Pwr.status || CCB_Pwr.status) && battery->mux_on && Adaptor.status == kAdaptorNotExist)
                battery->status = kstsFloat;
            break;
        default : break;
        }
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
}

void Battery_HwInit(void)
{
    Battery_1.iic_mutex = mutex_iic0Handle;
    Battery_2.iic_mutex = mutex_iic1Handle;

    if (CheckConnection(&Battery_1)) {
        BatteryInfoInit(&Battery_1);
        CheckFaulty(&Battery_1);
    } else {
        if (BatteryDataClear(&Battery_1) == HAL_OK)
            xprintf("Battery_HwInit(): clear battery_1 data\n\r");
    }
    if (CheckConnection(&Battery_2)) {
        BatteryInfoInit(&Battery_2);
        CheckFaulty(&Battery_2);
    } else {
        if (BatteryDataClear(&Battery_2) == HAL_OK)
            xprintf("Battery_HwInit(): clear battery_2 data\n\r");
    }

    if (Battery_1.remain_time > 0 && Battery_2.remain_time < 0) {
        SupplyOn(&Battery_1);
        SupplyOff(&Battery_2);
        return;
    }

    if (Battery_2.remain_time > 0 && Battery_1.remain_time < 0) {
        SupplyOn(&Battery_2);
        SupplyOff(&Battery_1);
        return;
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

void Thread_BatteryInfoUpdate(stBattery *battery)
{
    uint32_t current_second;
    uint16_t adccode;
    int8_t acr_ofuf;

    uint32_t tick_volt_sample = 0;  // tick to sample volt every 1s

    xprintf("thread_bat%dinfo start\n\r", battery->index);

    while (1) {
        current_second = GetSecond();

        if (battery->index == 0 || battery->index > 2) {
            xprintf("bat_infoupdate:wrong index %d\n\r", battery->index);
            vTaskDelete(NULL);
        }

        if (!CheckConnection(battery)) {
            if (battery->status != kstsNotExist) {
                xprintf("bat_%d disconnect\n\r", battery->index);
                BatteryInfoDeInit(battery);
            }
            continue;
        } else if (battery->status == kstsNotExist) {                           // initial for new connection
            xprintf("bat_%d connect\n\r", battery->index);
            BatteryInfoInit(battery);
            continue;
        }

        adccode = *(battery->ntc_adc);
        battery->temperature = -33.95 * log((float)adccode) + 269.94;

        // get v & i every 64ms
        vTaskDelay(VSAMPLE_INTV/portTICK_PERIOD_MS);

        acr_ofuf = battery->gauge->acr_ofuf;
        if (Get_Gauge_Information(battery->gauge, battery->iic_mutex)) {
            ErrorHandler(battery, GAUGE_FAIL);
            continue;
        } else {
            if (acr_ofuf != battery->gauge->acr_ofuf) {
                xprintf("BatteryInfoUpdate(): bat_%d acr_ofuf is %d, level is %.0f\n\r", battery->index, battery->gauge->acr_ofuf, battery->gauge->level);
            }
            if (battery->status == kstsError && battery->err_code == 0x40) {    // self-recover from gauge communication fail, ONLY if error is GAUGE_FAIL
                battery->status = kstsFloat;
            }
            if (battery->status == kstsError && battery->err_code == GAUGE_FAIL)
                battery->status = kstsFloat;
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

        if (battery->volt_sample == 0) {                                        // means battery just start to use, haven't record last sample voltage
            battery->volt_sample = battery->voltage;
            tick_volt_sample = current_second;
        } else if (battery->i_transit_up >= tick_volt_sample || battery->i_transit_down >= tick_volt_sample) {    // this judgement cannot be implement when load transit happens
            battery->volt_sample = 0;
        } else if (current_second - tick_volt_sample >= 5) {
            tick_volt_sample = current_second;
            battery->volt_sample = battery->voltage;
        }
    }
}

void Thread_BatterySupplyControl(stBattery *bat)
{
    xprintf("thread_bat%dsply start\n\r", bat->index);

    while (1) {
        if (bat->index == 0 || bat->index > 2) {
            xprintf("bat_supplycontrol:wrong index %d\n\r", bat->index);
            vTaskDelete(NULL);
        }

        if (bat->mux_on == true) {                                              // battery is supplying
            if (bat->status <= kstsError) {
                if (bat->substitute->status > kstsError) {
                    SupplyOn(bat->substitute);
                    osDelay(2);
                }
                SupplyOff(bat);
            } else {
                if (bat->voltage <= (GC_STOP_TH_VOLT + (bat->voltage - FieldCase.v_syspwr)) && GetSecond() - bat->i_transit_down > 5) {     // capacity is low, and prevent spurious trigger
                    if (bat->substitute->status <= kstsError) {                                         // only one bat is used
                        if (lo_pwr_beep == false) {
                            lo_pwr_beep = true;
                            xprintf("BatterySupplyControl(): battery_%d  capacity is low\n\r", bat->index);
                        }
                    } else if (bat->substitute->remain_time < 0 || bat->substitute->mux_on == true) {   // another bat is low power or dual-mode is on
                        if (lo_pwr_beep == false) {
                            lo_pwr_beep = true;
                            xprintf("BatterySupplyControl(): battery_%d both battery capacity are low\n\r", bat->index);
                        }
                    } else {                                                        // another is available
                        SupplyOn(bat->substitute);
                        osDelay(2);
                        SupplyOff(bat);
                        // todo: set flag at i_transit_down
                        xprintf("BatterySupplyControl(): battery_%d capacity is low, switch to battery_%d\n\r", bat->index, bat->substitute->index);
                    }
                    if (bat->volt_sample - bat->voltage >= 0.2) {
                        printf("BatterySupplyControl(): rockerswitch off detect\n\r");
                    } else if (bat->remain_time >= 0) {
                        bat->remain_time = -1;
                        bat->scale_flag |= REACHBOTTOM;
                        Calibrate(bat, kReachBottom);
                        bat->scale_flag &= ~REACHTOP;
                        BatteryDataSave(bat);
                    }
                } else if (need_dual_bat == false && bat->substitute->mux_on == true) {
                    if (bat->current >= bat->substitute->current) {
                        xprintf("BatterySupplyControl(): turn off bat_%d for single mode\n\r", bat->index);
                        SupplyOff(bat);
                    }
                }
            }
        } else if (bat->status > kstsError) {                                   // battery mux_on == false
            if (bat->substitute->status <= kstsError) {                         // turn on this battery prior to faulty another
                SupplyOn(bat);
            } else if (need_dual_bat == true) {
                if (bat->remain_time != -1) {
                    xprintf("BatterySupplyControl(): turn on bat_%d for dual mode\n\r", bat->index);
                    SupplyOn(bat);
                }
            } else {
                if (bat->substitute->mux_on == false &&                         // no one is supplying
                        (bat->level > bat->substitute->level || bat->substitute->remain_time == -1))
                    SupplyOn(bat);
            }
        }

        if (Adaptor.status == kAdaptorSupplying)
            lo_pwr_beep = false;

        vTaskDelay(100/portTICK_PERIOD_MS);
    }
}

void ChargeCmdhandler(stBattery *bat, bool request)
{
    if (bat->index == 0 || bat->index > 2)
        return;

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

void BatterySupplyCmdHandler(stBattery *bat, bool request)
{
    if (bat->index == 0 || bat->index > 2)
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

