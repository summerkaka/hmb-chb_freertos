/**
  ******************************************************************************
  * @file    Project/src/can_app.c
  * @author
  * @version V0.00
  * @date
  * @brief   application for can bus communication
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "app_include.h"

/* Private macro -------------------------------------------------------------*/


/* Private typedef -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
CAN_TxHeaderTypeDef             TxHeader;
uint8_t                         TxData[8];
uint32_t                        TxMailBox;
CANMsg_t                        rx_msg;
bool                            CANListening = false;
bool                            CANMonitoring = false;
uint32_t                        msg_total_recv = 0;
uint32_t                        self_recover_cnt = 0;
uint32_t                        can_intr_cnt = 0;
static uint32_t                 ccb_rst_time = 0;
uint8_t                         ccb_rst_times = 0;

uint32_t                        led_white_lock_time = 0;
uint32_t                        led_red_lock_time = 0;

stVariableTable bat_var_table[2][25] = {
    {
        {&Battery_1.status,              kUInt8},
        {&Battery_1.err_code,            kUInt16},
        {&Battery_1.voltage,             kFloat},
        {&Battery_1.current,             kFloat},
        {&Battery_1.temperature,         kFloat},
        {&Battery_1.level,               kFloat},
        {&Battery_1.capacity,            kFloat},
        {&Battery_1.mux_on,              kUInt8},
        {&Battery_1.remain_time,         kFloat},
        {&Battery_1.charge_iset,         kFloat},
        {&Battery_1.fastcharge_timer,    kFloat},
        {&Battery_1.pre_start_time,      kUInt32},
        {&Battery_1.fast_start_time,     kUInt32},
        {&Battery_1.trickle_start_time,  kUInt32},
        {&Battery_1.charge_finish_time,  kUInt32},
        {&Battery_1.charge_times,        kUInt16},
        {&Battery_1.scale_flag,          kUInt8},
        {&Battery_1.is_aged,             kUInt8},
        {&Battery_1.ng,                  kFloat},
        {&Battery_1.finish_code,         kUInt8},
        {&Battery_1.i_transit_up,        kUInt32},
        {&Battery_1.i_transit_down,      kUInt32},
        {&Battery_1.mode,                kUInt8},
        {&Battery_1.null_dev,            kUInt32},
        {&Battery_1.constant_timer,      kUInt8},
    },
    {
        {&Battery_2.status,              kUInt8},
        {&Battery_2.err_code,            kUInt16},
        {&Battery_2.voltage,             kFloat},
        {&Battery_2.current,             kFloat},
        {&Battery_2.temperature,         kFloat},
        {&Battery_2.level,               kFloat},
        {&Battery_2.capacity,            kFloat},
        {&Battery_2.mux_on,              kUInt8},
        {&Battery_2.remain_time,         kFloat},
        {&Battery_2.charge_iset,         kFloat},
        {&Battery_2.fastcharge_timer,    kFloat},
        {&Battery_2.pre_start_time,      kUInt32},
        {&Battery_2.fast_start_time,     kUInt32},
        {&Battery_2.trickle_start_time,  kUInt32},
        {&Battery_2.charge_finish_time,  kUInt32},
        {&Battery_2.charge_times,        kUInt16},
        {&Battery_2.scale_flag,          kUInt8},
        {&Battery_2.is_aged,             kUInt8},
        {&Battery_2.ng,                  kFloat},
        {&Battery_2.finish_code,         kUInt8},
        {&Battery_2.i_transit_up,        kUInt32},
        {&Battery_2.i_transit_down,      kUInt32},
        {&Battery_2.mode,                kUInt8},
        {&Battery_2.null_dev,            kUInt32},
        {&Battery_2.constant_timer,      kUInt8},
    }
};

/* Private function prototypes -----------------------------------------------*/

/* Code begin ----------------------------------------------------------------*/
static void
VariableWrite(const void *pdata, stVariableTable table)
{
    switch (table.dtype) {
    case kUInt8:
    case kInt8:
        *(uint8_t*)table.pdata = *(uint8_t*)pdata;
        break;
    case kUInt16:
    case kInt16:
        *(uint16_t*)table.pdata = *(uint16_t*)pdata;
        break;
    case kUInt32:
    case kInt32:
        *(uint32_t*)table.pdata = *(uint32_t*)pdata;
        break;
    case kFloat:
        *(float*)table.pdata = *(float*)pdata;
        break;
    default : break;
    }
}

static void
AIOReadHandler(const stCanPacket *pcmd)
{
    int32_t value = 0;
    TxHeader.ExtId = pcmd->id.all;
    ((stCanId *)(&TxHeader.ExtId))->Src = LOCAL_ID;
    ((stCanId *)(&TxHeader.ExtId))->Target = pcmd->id.field.Src;
    TxData[0] = pcmd->data[0];

    switch (pcmd->data[0]) {
    case MUX_CG_ADAPTOR_VOL:
        value = (int32_t)(Adaptor.voltage * 1000);
        break;
    case MUX_CG_BAT_1_VOL:
        value = (int32_t)(Battery_1.voltage * 1000);
        break;
    case MUX_CG_BAT_2_VOL:
        value = (int32_t)(Battery_2.voltage * 1000);
        break;
    case MUX_CG_BAT_1_CUR:
        value = (int32_t)(Battery_1.current * 1000);
        break;
    case MUX_CG_BAT_2_CUR:
        value = (int32_t)(Battery_2.current * 1000);
        break;
    case MUX_CG_BAT_1_TEMP:
        value = (int32_t)(Battery_1.temperature * 1000);
        break;
    case MUX_CG_BAT_2_TEMP:
        value = (int32_t)(Battery_2.temperature * 1000);
        break;
    case MUX_CG_BAT_1_LVL:
        value = Battery_1.status == kstsNotExist ? 0 : (int32_t)(Battery_1.level);
        break;
    case MUX_CG_BAT_2_LVL:
        value = Battery_2.status == kstsNotExist ? 0 : (int32_t)(Battery_2.level);
        break;
    case MUX_CG_BAT_1_CAP:
        value = Battery_1.status == kstsNotExist ? 0 : (int32_t)(Battery_1.capacity);
        break;
    case MUX_CG_BAT_2_CAP:
        value = Battery_2.status == kstsNotExist ? 0 : (int32_t)(Battery_2.capacity);
        break;
    case MUX_CG_BAT_1_REMAIN_TIME:
        if (Battery_1.status != kstsNotExist)
            value = Battery_1.remain_time < 0 ? 0 : (int32_t)Battery_1.remain_time;
        else
            value = 0;
        break;
    case MUX_CG_BAT_2_REMAIN_TIME:
        if (Battery_2.status != kstsNotExist)
            value = Battery_2.remain_time < 0 ? 0 : (int32_t)Battery_2.remain_time;
        else
            value = 0;
        break;
    case MUX_CG_BAT_1_CHARGE_TIMES:
        value = (int32_t)Battery_1.charge_times;
        break;
    case MUX_CG_BAT_2_CHARGE_TIMES:
        value = (int32_t)Battery_2.charge_times;
        break;
    case MUX_CG_BAT_TOTAL_PERCENT:
        value = total_bat_percent;
        break;
    case MUX_CG_BAT_TOTAL_REMAIN:
        value = total_bat_time;
        break;
    case MUX_CG_HEATER_TEMP:
        value = (int32_t)(heater.temperature * 1000);
        break;
    case MUX_CG_HEATER_DUTY:
        value = (int32_t)(heater.pwm.duty * 10000);
        break;
    case MUX_CG_OUTPUT_VOL:
        value = (int32_t)(FieldCase.v_syspwr * 1000);
        break;
    case MUX_CG_OUTPUT_CUR:
        value = (int32_t)(FieldCase.consumption * 1000);
        break;
    case MUX_CG_PRESSURE_1:
        value = (int32_t)(FieldCase.gas_1_pressure * 1000);
        value = value <= 30000 ? 0 : value;
        break;
    case MUX_CG_PRESSURE_2:
        value = (int32_t)(FieldCase.gas_2_pressure * 1000);
        value = value <= 30000 ? 0 : value;
        break;
    default : break;
    }
    WriteWordL(&TxData[1], value);
    TxHeader.DLC = 4;
    HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailBox);
}

static void
IDReadHandler(const stCanPacket *pcmd)
{
	TxHeader.ExtId = pcmd->id.all;
	((stCanId *)(&TxHeader.ExtId))->Src = LOCAL_ID;
	((stCanId *)(&TxHeader.ExtId))->Target = pcmd->id.field.Src;
	TxData[1] = (uint8_t)fw_version;
	TxData[0] = (uint8_t)((fw_version - (uint8_t)fw_version) * 100);
	TxHeader.DLC = 2;
	HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailBox);
    xprintf("answer 'read_id'\n\r");
}

static void
DIOReadHandler(const stCanPacket *pcmd)
{
    uint16_t value = 0, mask = 0, value_b = 0;
    uint8_t cmd_num = pcmd->id.field.CmdNum;
    uint8_t block_id = 0;

	TxHeader.ExtId = pcmd->id.all;
	((stCanId *)(&TxHeader.ExtId))->Src = LOCAL_ID;
	((stCanId *)(&TxHeader.ExtId))->Target = pcmd->id.field.Src;

    if (cmd_num == CMD_DETECT_ADC) {
        mask = 0xffff;
        block_id = BLOCK_A;
    } else if (cmd_num == CMD_READ_DIO) {
	    mask = (uint16_t)pcmd->data[2] << 8 | pcmd->data[1];
        block_id = pcmd->data[0];
    }

    if ((cmd_num == CMD_READ_DIO && block_id == BLOCK_A) || cmd_num == CMD_DETECT_ADC) {
        if ((mask & DIO_ADAPTOR_SUPPLY) && Adaptor.status != kAdaptorNotExist)
            value |= DIO_ADAPTOR_SUPPLY;
        else
            value &= ~DIO_ADAPTOR_SUPPLY;

        if ((mask & DIO_BAT1_SUPPLY) && Battery_1.mux_on == true)
            value |= DIO_BAT1_SUPPLY;
        else
            value &= ~DIO_BAT1_SUPPLY;

        if ((mask & DIO_BAT2_SUPPLY) && Battery_2.mux_on == true)
            value |= DIO_BAT2_SUPPLY;
        else
            value &= ~DIO_BAT2_SUPPLY;

        if ((mask & DIO_GCFAN) && Fan.status != kLoadOff)
            value |= DIO_GCFAN;
        else
            value &= ~DIO_GCFAN;

        if ((mask & DIO_HEATER) && heater.mode != kHeaterOff)
            value |= DIO_HEATER;
        else
            value &= ~DIO_HEATER;

        if ((mask & DIO_BAT1_CHARGE) && Battery_1.status >= kstsPreCharge && Battery_1.status < kstsFinish)
            value |= DIO_BAT1_CHARGE;
        else
            value &= ~DIO_BAT1_CHARGE;

        if ((mask & DIO_BAT2_CHARGE) && Battery_2.status >= kstsPreCharge && Battery_2.status < kstsFinish)
            value |= DIO_BAT2_CHARGE;
        else
            value &= ~DIO_BAT2_CHARGE;

        if ((mask & DIO_BAT1_AGED) && (Battery_1.is_aged == true))
            value |= DIO_BAT1_AGED;
        else
            value &= ~DIO_BAT1_AGED;

        if ((mask & DIO_BAT2_AGED) && (Battery_2.is_aged == true))
            value |= DIO_BAT2_AGED;
        else
            value &= ~DIO_BAT2_AGED;

        if ((mask & DIO_VALVE1_SW) && Valve_1.status == kLoadOn )
            value |= DIO_VALVE1_SW;
        else
            value &= ~DIO_VALVE1_SW;

        if ((mask & DIO_VALVE2_SW) && Valve_2.status == kLoadOn)
            value |= DIO_VALVE2_SW;
        else
            value &= ~DIO_VALVE2_SW;

        if ((mask & DIO_PUMP) && Pump_1.dio.status == kLoadOn)
            value |= DIO_PUMP;
        else
            value &= ~DIO_PUMP;

        if ((mask & DIO_PUMP_VALVE) && PValve.dio.status == kLoadOn)
            value |= DIO_PUMP_VALVE;
        else
            value &= ~DIO_PUMP_VALVE;

        if ((mask & DIO_BOOT_MODE) && gc_status == kGcFastBoot)
            value &= ~DIO_BOOT_MODE;
        else
            value |= DIO_BOOT_MODE;
    }

    if ((cmd_num == CMD_READ_DIO && block_id == BLOCK_B) || cmd_num == CMD_DETECT_ADC) {
        if ((mask & DIO_CALIFORNIA) && Battery_1.mode == kCalifornia)
            value_b |= DIO_CALIFORNIA;
        else
            value_b &= ~DIO_CALIFORNIA;

        if ((mask & DIO_LED_RED) && HAL_GPIO_ReadPin(LED_RED_GPIO_Port, LED_RED_Pin))
            value_b |= DIO_LED_RED;
        else
            value_b &= ~DIO_LED_RED;

        if ((mask & DIO_LED_WHITE) && HAL_GPIO_ReadPin(LED_WHITE_GPIO_Port, LED_WHITE_Pin))
            value_b |= DIO_LED_WHITE;
        else
            value_b &= ~DIO_LED_WHITE;

        if ((mask & DIO_HALL_SENSOR) && !FieldCase.is_covered)
            value_b |= DIO_HALL_SENSOR;
        else
            value_b &= ~DIO_HALL_SENSOR;
    }

    if (cmd_num == CMD_READ_DIO) {
        WriteShortL(TxData, value);
        TxHeader.DLC = 2;
    } else if (cmd_num == CMD_DETECT_ADC) {
        WriteShortL(TxData, value);
        WriteShortL(TxData+2, value_b);
        TxHeader.DLC = 4;
    }
    HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailBox);
}

static void
DIOWriteHandler(const stCanPacket *pcmd)
{
    uint16_t dio_mask = (uint16_t)pcmd->data[2] << 8 | pcmd->data[1];
    uint16_t dio_value = (uint16_t)pcmd->data[4] << 8 | pcmd->data[3];;

    xprintf("DioWriteHandler(): block_%d, mask %x, value %x,\n\r", pcmd->data[0], dio_mask, dio_value);

    switch (pcmd->data[0]) {
    case BLOCK_A:

        if (dio_mask & DIO_BAT1_SUPPLY) {
            BatterySupplyCmdHandler(&Battery_1, (dio_value & DIO_BAT1_SUPPLY) == DIO_BAT1_SUPPLY);
        }
        if (dio_mask & DIO_BAT2_SUPPLY) {
            BatterySupplyCmdHandler(&Battery_2, (dio_value & DIO_BAT2_SUPPLY) == DIO_BAT2_SUPPLY);
        }
        if (dio_mask & DIO_BAT1_CHARGE) {
            ChargeCmdhandler(&Battery_1, (dio_value & DIO_BAT1_CHARGE) == DIO_BAT1_CHARGE);
        }
        if (dio_mask & DIO_BAT2_CHARGE) {
            ChargeCmdhandler(&Battery_2, (dio_value & DIO_BAT2_CHARGE) == DIO_BAT2_CHARGE);
        }
        if (dio_mask & DIO_GCFAN) {
            DioSetTo(&Fan, (dio_value & DIO_GCFAN) != 0);
        }
        if (dio_mask & DIO_HEATER) {
            heater.mode = (eHeaterMode)((dio_value & DIO_HEATER) != 0);
        }
        if (dio_mask & DIO_VALVE1_SW) {
            DioSetTo(&Valve_1, (dio_value & DIO_VALVE1_SW) != 0);
            ValveDataSave(0, (dio_value & DIO_VALVE1_SW) != 0);
        }
        if (dio_mask & DIO_VALVE2_SW) {
            DioSetTo(&Valve_2, (dio_value & DIO_VALVE2_SW) != 0);
            ValveDataSave(1, (dio_value & DIO_VALVE2_SW) != 0);
        }
        if (dio_mask & (DIO_PUMP)) {
            DioSetTo(&Pump_1.dio, (dio_value & DIO_PUMP) != 0);
            DioSetTo(&Pump_2.dio, (dio_value & DIO_PUMP) != 0);
        }
        if (dio_mask & DIO_PUMP_VALVE) {
            DioSetTo(&PValve.dio, (dio_value & DIO_PUMP_VALVE) != 0);
        }
        break;
    case BLOCK_B:
        if (dio_mask & DIO_LED_RED) {
            led_red_lock_time = GetSecond();
            HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, (GPIO_PinState)((dio_value & DIO_LED_RED) != 0));
        }
        if (dio_mask & DIO_LED_WHITE) {
            led_white_lock_time = GetSecond();
            HAL_GPIO_WritePin(LED_WHITE_GPIO_Port, LED_WHITE_Pin, (GPIO_PinState)((dio_value & DIO_LED_WHITE) != 0));
        }
        if (dio_mask & DIO_CALIFORNIA) {
            eRegionMode mode = dio_value & DIO_CALIFORNIA ? kCalifornia : kGlobal;
            BatteryModeSave(mode);
        }
		break;
    case BLOCK_C:
        if (dio_mask & DIO_CCB) {
            if (dio_value & DIO_CCB) {
                DioSetTo(&CCB_Pwr, 0);
                ccb_rst_time = GetSecond();
                ccb_rst_times++;
            }
        }
        break;
    default : break;
    }
}

static void
UpdateHandler(const stCanPacket *pcmd)
{
	TxHeader.ExtId = pcmd->id.all;
	((stCanId *)(&TxHeader.ExtId))->Src = CANID_CHARGE;
	((stCanId *)(&TxHeader.ExtId))->Target = pcmd->id.field.Src;

    switch (pcmd->id.field.CmdNum) {
    case CMD_PING:
        TxData[0] = 0x01;
        TxHeader.DLC = 1;
        HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailBox);
        break;
    case CMD_REQUEST:
        break;
    case CMD_RESET:
        NVIC_SystemReset();
        break;
    case CMD_JUMPTOBL:
        if (((*(__IO uint32_t *)BL_ADDRESS) & 0x2FFE0000) == 0x20000000) {
            update_request = 0x55555555;
            TxData[0] = 0x00;
            TxHeader.DLC = 1;
            HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailBox);
            xprintf("jumping to BL...\n\r");
            // __set_MSP(*(__IO uint32_t*)BL_ADDRESS);
            // ((void(*)(void))(*(__IO uint32_t*)(BL_ADDRESS + 4)))();
            NVIC_SystemReset();
        } else {
            TxData[0] = 0x01;
            TxHeader.DLC = 1;
            HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailBox);
            xprintf("refuse to jump to BL, BL_ADDRESS not right\n\r");
        }
        break;
	default : break;
    }
	return;
}

static void
BatteryRenewHandler(const stCanPacket *pcmd)
{
    TxHeader.ExtId = pcmd->id.all;
	((stCanId *)(&TxHeader.ExtId))->Src = CANID_CHARGE;
    ((stCanId *)(&TxHeader.ExtId))->Target = pcmd->id.field.Src;

    if (pcmd->data[0] == 0x01) {
        TxData[0] = BatteryDataClear(&Battery_1);
    } else if (pcmd->data[0] == 0x02) {
        TxData[0] = BatteryDataClear(&Battery_2);
    }
    TxHeader.DLC = 1;
    HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailBox);
}

static void
ZoneRdHandler(const stCanPacket *pcmd)
{
    int32_t value_long = 0;
    int16_t value_short = 0;

    TxHeader.ExtId = pcmd->id.all;
	((stCanId *)(&TxHeader.ExtId))->Src = CANID_CHARGE;
    ((stCanId *)(&TxHeader.ExtId))->Target = pcmd->id.field.Src;
    TxData[0] = pcmd->data[0];

    switch (pcmd->data[0]) {
    case 0x01:  // bat 1
        TxData[1] = Battery_1.status;
        TxData[2] = Battery_1.status == kstsError ? Battery_1.err_code : 0;
        TxData[3] = Battery_1.finish_code;
        TxHeader.DLC = 4;
        HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailBox);
        break;
    case 0x02:  // bat 2
        TxData[1] = Battery_2.status;
        TxData[2] = Battery_2.status == kstsError ? Battery_2.err_code : 0;
        TxData[3] = Battery_2.finish_code;
        TxHeader.DLC = 4;
        HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailBox);
        break;
    case 0x03:  // heater
        TxData[1] = heater.mode << 4;
        value_long = (int32_t)(heater.setpoint * 1000);
        WriteWordL(&TxData[2], value_long);
        value_short = (int16_t)(heater.pwm.duty * 10000);
        WriteShortL(&TxData[5], value_short);
        TxHeader.DLC = 7;
        HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailBox);
        break;
    case 0x04:  // GC fan
        TxData[1] = Fan.status == kLoadError;
        TxHeader.DLC = 2;
        HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailBox);
        break;
    case 0x05:  // MB fan
        TxData[1] = MB_Pwr.status == kLoadError;
        TxHeader.DLC = 2;
        HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailBox);
        break;
    case 0x06:  // pump 1
        TxData[1] = Pump_1.dio.status == kLoadError;
        TxHeader.DLC = 2;
        HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailBox);
        break;
    case 0x07:  // pump 2
        TxData[1] = Pump_2.dio.status == kLoadError;
        TxHeader.DLC = 2;
        HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailBox);
        break;
    case 0x08:  // Gas valve 1
        TxData[1] = Valve_1.status == kLoadError;
        TxHeader.DLC = 2;
        HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailBox);
        break;
    case 0x09:  // Gas valve 2
        TxData[1] = Valve_2.status == kLoadError;
        TxHeader.DLC = 2;
        HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailBox);
        break;
    default : break;
    }
}

static void
SetPointWrHandler(const stCanPacket *pcmd)
{
    int32_t setpoint = 0;
    float sp = 0;
    uint16_t duty = 0;
    int16_t pid[3] = {0};

    TxHeader.ExtId = pcmd->id.all;
    ((stCanId *)(&TxHeader.ExtId))->Src = CANID_CHARGE;
    ((stCanId *)(&TxHeader.ExtId))->Target = pcmd->id.field.Src;
    TxData[0] = pcmd->data[0];
    TxHeader.DLC = 2;

    switch(pcmd->data[0]) {
    case 0x01:  // battery 1
    case 0x02:  // battery 2
        if (pcmd->data[1] == 0x03) {    // battery renew
            BatteryRenewHandler(pcmd);
            TxData[1] = 0x00;
        } else
            TxData[1] = 0x01;
        break;
    case 0x03:  // heater
        switch (pcmd->data[1]) {
        case 0x00:  // turn off
            heater.mode = kHeaterOff;
            TxData[1] = 0x00;
            break;
        case 0x01:  // turn on and set setpoint
            setpoint = GetWordL(&pcmd->data[2]) & 0x00ffffff;
            setpoint = (setpoint & 0x00800000) ? (setpoint | 0xff000000) : (setpoint & 0x00ffffff);
            sp = (float)setpoint / 100;
            heater.setpoint = sp;
            heater.mode = kHeaterPID;
            TxData[1] = 0x00;
            HeaterContactRelease();
            break;
        case 0x02:  // turn on and set fix duty cycle
            duty = GetShortL(&pcmd->data[2]);
            HeaterSetFixPwm(&heater, duty / 10000);
            TxData[1] = 0x00;
            break;
        case 0x03:  // reset
            heater.mode = kHeaterOff;
            pid[0] = DEFAULT_KP;
            pid[1] = DEFAULT_KI;
            pid[2] = DEFAULT_KD;
            HeaterSetPID(&heater, (int16_t (*)[3])pid);
            PwmSetTo(&heater.pwm, 0);
            heater.setpoint = 0;
            TxData[1] = 0x00;
            break;
        default :
            TxData[1] = 0x01;
            break;
        }
        break;
    default :
        TxData[1] = 0x01;
        break;
    }
    HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailBox);
}

static void
PulseWriteHandler(const stCanPacket *pcmd)
{
    PulseOut *pulseout;
    uint16_t mask = GetShortL(&pcmd->data[1]);
    uint32_t time = 0;

    switch (mask) {
    case DIO_PUMP:
        pulseout = &Pump_1;
        break;
    case DIO_PUMP_VALVE:
        pulseout = &PValve;
        break;
    default: break;
    }
SET_TIME:
    switch(pcmd->data[3]) {
    case 0x00:
        time = GetWordL(&pcmd->data[5]);
        time &= 0x00ffffff;
        PulseOutSetStartTime(pulseout->timer_start, time);
        break;
    case 0x01:
        time = GetWordL(&pcmd->data[5]);
        time &= 0x00ffffff;
        PulseOutSetStopTime(pulseout->timer_stop, time);
        break;
    default : break;
    }

    if (pulseout == &Pump_1) {
        pulseout = &Pump_2;
        goto SET_TIME;
    } else
        return;

}

static void
StartRunHandler(void)
{
    PulseOutStartRun(&Pump_1);
    PulseOutStartRun(&Pump_2);
    PulseOutStartRun(&PValve);
}

static void
StopRunHandler(void)
{
    PulseOutStopRun(&Pump_1);
    PulseOutStopRun(&Pump_2);
    PulseOutStopRun(&PValve);
}

static void
ContactHandler(const stCanPacket *pcmd)
{
    HeaterContactHandler(pcmd->data[0]);
}

static void
UnknownCmdHandler(const stCanPacket *pcmd)
{
    printf("unkonw cmd_num : %02x", pcmd->id.field.CmdNum);
}

static void
DebugRdHandler(const stCanPacket *pcmd)
{
    int16_t value = 0;
    int16_t pid[3];
    tuType32 tu_value;
    TxHeader.ExtId = pcmd->id.all;
	((stCanId *)(&TxHeader.ExtId))->Src = CANID_CHARGE;
    ((stCanId *)(&TxHeader.ExtId))->Target = pcmd->id.field.Src;
    TxData[0] = pcmd->data[0];
    TxData[1] = pcmd->data[1];
    uint8_t row = 0, col = 0;

    switch (pcmd->data[0]) {
    case 0x01:      // battery
    case 0x02:
        row = pcmd->data[0] - 1;
        col = pcmd->data[1];
        switch (bat_var_table[row][col].dtype) {
        case kUInt8:
        case kInt8:
        {
            uint8_t data = *(uint8_t*)bat_var_table[row][col].pdata;
            TxData[2] = data;
            TxHeader.DLC = 3;
            break;
        }
        case kUInt16:
        case kInt16:
        {
            uint16_t data = *(uint16_t*)bat_var_table[row][col].pdata;
            WriteShortL(&TxData[2], data);
            TxHeader.DLC = 4;
            break;
        }
        case kUInt32:
        case kInt32:
        {
            uint32_t data = *(uint32_t*)bat_var_table[row][col].pdata;
            WriteWordL(&TxData[2], data);
            TxHeader.DLC = 6;
            break;
        }
        case kFloat:
        {
            tuType32 tu_value;
            tu_value.tfloat = *(float*)bat_var_table[row][col].pdata;
            WriteWordL(&TxData[2], tu_value.tint32);
            TxHeader.DLC = 6;
            break;
        }
        default : break;
        }
        break;
    case 0x03:      // heater
        switch (pcmd->data[1]) {
        case 0x00:  // mode
            TxData[2] = heater.mode;
            TxHeader.DLC = 3;
            break;
        case 0x01:  // temperature
            tu_value.tfloat = heater.temperature;
            WriteWordL(&TxData[2], tu_value.tint32);
            TxHeader.DLC = 6;
            break;
        case 0x02:  // set point
            tu_value.tfloat = heater.setpoint;
            WriteWordL(&TxData[2], tu_value.tint32);
            TxHeader.DLC = 6;
            break;
        case 0x03:  // duty
            tu_value.tfloat = heater.pwm.duty;
            WriteWordL(&TxData[2], tu_value.tint32);
            TxHeader.DLC = 6;
            break;
        case 0x04:  // kp
            HeaterGetPID(&heater, (int16_t (*)[3])pid);
            WriteShortL(&TxData[2], pid[0]);
            TxHeader.DLC = 4;
            break;
        case 0x05:  // ki
            HeaterGetPID(&heater, (int16_t (*)[3])pid);
            WriteShortL(&TxData[2], pid[1]);
            TxHeader.DLC = 4;
            break;
        case 0x06:  // kd
            HeaterGetPID(&heater, (int16_t (*)[3])pid);
            WriteShortL(&TxData[2], pid[2]);
            TxHeader.DLC = 4;
            break;
        case 0x07:  // pt100_adccode
            value = sdadc1_code[1];
            WriteShortL(&TxData[2], value);
            TxHeader.DLC = 4;
            break;
        default : break;
        }
        break;
    case 0x04:      // field case
        switch (pcmd->data[1]) {
        case 0x00:  // cover
            TxData[2] = !FieldCase.is_covered;
            TxHeader.DLC = 3;
            break;
        case 0x01:  // power button
            TxData[2] = FieldCase.is_switchon;
            TxHeader.DLC = 3;
            break;
        case 0x02:  // v_adaptor
            tu_value.tfloat = Adaptor.voltage;
            WriteWordL(&TxData[2], tu_value.tint32);
            TxHeader.DLC = 6;
            break;
        case 0x03:  // gc pwr
            tu_value.tfloat = FieldCase.v_syspwr;
            WriteWordL(&TxData[2], tu_value.tint32);
            TxHeader.DLC = 6;
            break;
        case 0x04:  // gc consumption
            tu_value.tfloat = FieldCase.consumption;
            WriteWordL(&TxData[2], tu_value.tint32);
            TxHeader.DLC = 6;
            break;
        case 0x05:  // runtime_min
            WriteWordL(&TxData[2], GetMinute());
            TxHeader.DLC = 6;
            break;
        case 0x06:  // runtime_sec
            WriteWordL(&TxData[2], GetSecond());
            TxHeader.DLC = 6;
            break;
        case 0x07:  // switchon time
            WriteWordL(&TxData[2], FieldCase.switchon_time);
            TxHeader.DLC = 6;
            break;
        case 0x08:  // switchoff time
            WriteWordL(&TxData[2], FieldCase.switchoff_time);
            TxHeader.DLC = 6;
            break;
        case 0x09:  // mb ccb status
            TxData[2] = MB_Pwr.status;
            TxHeader.DLC = 3;
            break;
        default :break;
        }
        break;
    default : break;
    }
    HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailBox);
}

static void
DebugWrHandler(const stCanPacket *pcmd)
{
    int16_t pid[3];
    tuType32 tu_value;

    switch (pcmd->data[0]) {
    case 0x01:      // battery
    case 0x02:
    {
        uint8_t row = pcmd->data[0] - 1;
        uint8_t col = pcmd->data[1];
        VariableWrite(&pcmd->data[2], bat_var_table[row][col]);
        break;
    }
    case 0x03:      // heater
        switch (pcmd->data[1]) {
        case 0x00:  // mode
            heater.mode = (eHeaterMode)pcmd->data[2];
            break;
        case 0x01:  // temperature:
            break;
        case 0x02:  // setpoint
            tu_value.tint32 = GetWordL(&pcmd->data[2]);
            heater.setpoint = tu_value.tfloat;
            break;
        case 0x03:  // duty
            tu_value.tint32 = GetWordL(&pcmd->data[2]);
            PwmSetDuty(&heater.pwm, tu_value.tfloat);
            break;
        case 0x04:  // kp
            HeaterGetPID(&heater, (int16_t (*)[3])pid);
            pid[0] = GetShortL(&pcmd->data[2]);
            HeaterSetPID(&heater, (int16_t (*)[3])pid);
            break;
        case 0x05:  // ki
            HeaterGetPID(&heater, (int16_t (*)[3])pid);
            pid[1] = GetShortL(&pcmd->data[2]);
            HeaterSetPID(&heater, (int16_t (*)[3])pid);
            break;
        case 0x06:  // kd
            HeaterGetPID(&heater, (int16_t (*)[3])pid);
            pid[2] = GetShortL(&pcmd->data[2]);
            HeaterSetPID(&heater, (int16_t (*)[3])pid);
            break;
        case 0x07:  // pt100_code
            break;
        default : break;
        }
    default : break;
    }
}

static void
CmdHandler(const stCanPacket *pcmd)
{
	if (pcmd->id.field.Target != LOCAL_ID && pcmd->id.field.Target != CANID_BROADCAST)
        return;
    switch (pcmd->id.field.CmdNum) {
	case CMD_READ_AIO:
		AIOReadHandler(pcmd);
        break;
    case CMD_READ_ZONE:
        ZoneRdHandler(pcmd);
        break;
	case CMD_READ_ID:
		IDReadHandler(pcmd);
		break;
    case CMD_DETECT_ADC:
	case CMD_READ_DIO:
		DIOReadHandler(pcmd);
		break;
	case CMD_WRITE_DIO:
		DIOWriteHandler(pcmd);
		break;
    case CMD_PING:
    case CMD_ASK_APPAREA:
    case CMD_JUMPTOBL:
        UpdateHandler(pcmd);
        break;
    case CMD_BAT_RENEW:
        BatteryRenewHandler(pcmd);
        break;
    case CMD_WRITE_PULSE:
        PulseWriteHandler(pcmd);
        break;
    case CMD_START_RUN:
        StartRunHandler();
        break;
    case CMD_STOP_RUN:
        StopRunHandler();
        break;
    case CMD_WRITE_SP:
        SetPointWrHandler(pcmd);
        break;
    case CMD_CONTACT_OFF:
        ContactHandler(pcmd);
        break;
    case CMD_DEBUG_RD:
        DebugRdHandler(pcmd);
        break;
    case CMD_DEBUG_WR:
        DebugWrHandler(pcmd);
        break;
	default :
        UnknownCmdHandler(pcmd);
        break;
	}
    //HAL_GPIO_TogglePin(LED_WHITE_GPIO_Port, LED_WHITE_Pin);
    msg_total_recv++;
}

void CAN_Config(void)
{
    CAN_FilterTypeDef  sFilterConfig;
    uint32_t id = LOCAL_ID << 8;
    uint32_t mask = 0xff << 8;

    if (hcan.State == HAL_CAN_STATE_LISTENING && HAL_CAN_Stop(&hcan) != HAL_OK)
        Error_Handler();

    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = ((id << 3) & 0xFFFF0000) >> 16;
    sFilterConfig.FilterIdLow = ((id << 3) | CAN_ID_EXT | CAN_RTR_DATA) & 0xFFFF;
    sFilterConfig.FilterMaskIdHigh = ((mask << 3) & 0xFFFF0000) >> 16;
    sFilterConfig.FilterMaskIdLow = ((mask << 3) | CAN_ID_EXT | CAN_RTR_DATA) & 0xFFFF;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
        Error_Handler();

    id = CANID_BROADCAST << 8;
    sFilterConfig.FilterBank = 2;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = ((id << 3) & 0xFFFF0000) >> 16;
    sFilterConfig.FilterIdLow = ((id << 3) | CAN_ID_EXT | CAN_RTR_DATA) & 0xFFFF;
    sFilterConfig.FilterMaskIdHigh = ((mask << 3) & 0xFFFF0000) >> 16;
    sFilterConfig.FilterMaskIdLow = ((mask << 3) | CAN_ID_EXT | CAN_RTR_DATA) & 0xFFFF;
    sFilterConfig.FilterFIFOAssignment = 0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
        Error_Handler();

    if (HAL_CAN_Start(&hcan) != HAL_OK) {
        HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
        Error_Handler();
    }

    TxHeader.StdId = 0x321;
    TxHeader.ExtId = 0x01;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_EXT;
    TxHeader.DLC = 2;
    TxHeader.TransmitGlobalTime = DISABLE;
}

void CAN_DeconfigFilter(void)
{
    CAN_FilterTypeDef  sFilterConfig;

    if (HAL_CAN_Stop(&hcan) != HAL_OK)
        Error_Handler();

    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0;
    sFilterConfig.FilterIdLow = 0;
    sFilterConfig.FilterMaskIdHigh = 0;
    sFilterConfig.FilterMaskIdLow = 0;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
        Error_Handler();

    sFilterConfig.FilterBank = 2;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0;
    sFilterConfig.FilterIdLow = 0;
    sFilterConfig.FilterMaskIdHigh = 0;
    sFilterConfig.FilterMaskIdLow = 0;
    sFilterConfig.FilterFIFOAssignment = 0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
        Error_Handler();

    if (HAL_CAN_Start(&hcan) != HAL_OK) {
        HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
        Error_Handler();
    }
    CAN_Listen();
}

void CAN_Listen(void)
{
    uint8_t result = HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

    if (result == HAL_OK)
        CANListening = true;
    else
        CANListening = false;
}

/**
  * @brief  Transmission  complete callback in non blocking mode
  * @param  hcan: pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    portBASE_TYPE pxHigherPriorityTaskWoken;

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_msg.header, rx_msg.data) != HAL_OK)
    {
        Error_Handler();
    }

    can_intr_cnt++;
    CAN_Listen();
    xQueueSendFromISR(q_canmsg, &rx_msg, &pxHigherPriorityTaskWoken);
    if (pxHigherPriorityTaskWoken == pdTRUE)
        portYIELD_FROM_ISR(pdTRUE);

}

void Thread_CANComm(void *p)
{
    CANMsg_t msg;
    stCanPacket pkt;
    char buf[32] = {'c','a','n',':',' ',};
    int8_t i = 0, j = 0;
    uint32_t latest_time_stamp;

    xprintf("thread_can start\n\r");

    while (1) {
        if (xQueueReceive(q_canmsg, &msg, 5/portTICK_PERIOD_MS) == pdPASS) {
            pkt.id.all = msg.header.ExtId;
            pkt.dlc = msg.header.DLC;
            memcpy(pkt.data, msg.data, 8);
            CmdHandler((const stCanPacket *)&pkt);
            if (CANMonitoring == ON) {
                i = 5;
                sprintf(&buf[i], "%08x", pkt.id.all);
                i+= 8;
                buf[i++] = ' ';
                sprintf(&buf[i++], "%01u", pkt.dlc);
                buf[i++] = ' ';
                for (j=0; j<pkt.dlc; j++) {
                    sprintf(&buf[i], "%02x", pkt.data[j]);
                    i += 2;
                    buf[i++] = ' ';
                }
                xprintf("%s\n\r", buf);
                memset(&buf[5], 0, 27);
            }
            latest_time_stamp = GetSecond();
        }

        if (GetSecond() - latest_time_stamp >= 15) {
            HeaterContactHandler(0);
        }

        if (CANListening == false)
            CAN_Listen();

        if (ccb_rst_time != 0) {
            if (GetSecond() - ccb_rst_time >= 3) {
                ccb_rst_time = 0;
                DioSetTo(&CCB_Pwr, 1);
            }
        }

        if (__HAL_CAN_GET_FLAG(&hcan, CAN_FLAG_FF0) || __HAL_CAN_GET_FLAG(&hcan, CAN_FLAG_FOV0)) {

            __HAL_CAN_CLEAR_FLAG(&hcan, CAN_FLAG_FF0);
            hcan.Instance->RF0R |= CAN_RF0R_RFOM0;                                  //__HAL_CAN_FIFO_RELEASE(&hcan, CAN_RX_FIFO0);
            __HAL_CAN_ENABLE_IT(&hcan, CAN_IT_RX_FIFO0_OVERRUN | CAN_IT_RX_FIFO0_MSG_PENDING);

            TxHeader.ExtId = 0;
            TxHeader.DLC = 8;
            memset(TxData, 1, 8);
            HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailBox);

            xprintf("CANbus self recover, times: %d, can_intr_cnt: %d\n\r", ++self_recover_cnt, can_intr_cnt);
        }
    }
}

void CAN_MonitorSwitch(uint8_t *p)
{
    if (*p == '0') {              // CAN monitor off, CAN run in normal mode with filter
        CANMonitoring = false;
        CAN_Config();
    } else if (*p == '1') {                    // CAN monitor on, CAN run in monitor mode with no filter
        CANMonitoring = true;
        CAN_DeconfigFilter();
    }
}


/* console format:
 * can src target cmdnum dlc |payload0 payload1 ......|   --all in hex format with no '0x'
 * eg: can 00 0d 80 00
 * 'can' is command prefix, argument pointer 'p' starts from 'src'
 */
void CAN_ManualSend(uint8_t *p)
{
    CAN_TxHeaderTypeDef txheader;   // for manual send from console
    uint8_t txdata[8];  // for manual send from console
    uint8_t i = 0;

    sscanf((const char *)p, "%02hhx", txdata);                                  //hh means length is char
    ((stCanId *)(&txheader.ExtId))->Src = txdata[0];
    p += 3;

    sscanf((const char *)p, "%02hhx", txdata);
    ((stCanId *)(&txheader.ExtId))->Target = txdata[0];
    p += 3;

    sscanf((const char *)p, "%02hhx", txdata);
    ((stCanId *)(&txheader.ExtId))->CmdNum = txdata[0];
    p += 3;

    sscanf((const char *)p, "%02hhx", txdata);
    txheader.DLC = txdata[0];
    p += 3;

    for (i=0; i<txheader.DLC; i++) {
        sscanf((const char *)p, "%02hhx", &txdata[i]);
        p += 3;
    }
    HAL_CAN_AddTxMessage(&hcan, &txheader, txdata, &TxMailBox);
}



