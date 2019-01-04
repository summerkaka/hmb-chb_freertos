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
#include "app.h"

/* Private macro -------------------------------------------------------------*/
#define POOL_SIZE 5
/* Private typedef -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
CANMsg_t                        rx_msg;
CAN_TxHeaderTypeDef             TxHeader;
CAN_RxHeaderTypeDef             RxHeader;
uint8_t                         TxData[8];
uint8_t                         RxData[8];
uint32_t                        TxMailBox;
bool                            CANListening = false;
uint32_t                        msg_total_recv = 0;

uint32_t                        led_white_lock_time = 0;
uint32_t                        led_red_lock_time = 0;

stVariableTable bat_var_table[2][25] = {
    {
        {&Battery_1.status,              kUInt8_t},
        {&Battery_1.err_code,            kUInt16_t},
        {&Battery_1.voltage,             kFloat},
        {&Battery_1.current,             kFloat},
        {&Battery_1.temperature,         kFloat},
        {&Battery_1.level,               kFloat},
        {&Battery_1.capacity,            kFloat},
        {&Battery_1.mux_on,              kUInt8_t},
        {&Battery_1.remain_time,         kFloat},
        {&Battery_1.charge_iset,         kFloat},
        {&Battery_1.fastcharge_timer,    kFloat},
        {&Battery_1.pre_start_time,      kUInt32_t},
        {&Battery_1.fast_start_time,     kUInt32_t},
        {&Battery_1.trickle_start_time,  kUInt32_t},
        {&Battery_1.charge_finish_time,  kUInt32_t},
        {&Battery_1.charge_times,        kUInt16_t},
        {&Battery_1.scale_flag,          kUInt8_t},
        {&Battery_1.is_aged,             kUInt8_t},
        {&Battery_1.ng,                  kFloat},
        {&Battery_1.finish_code,         kUInt8_t},
        {&Battery_1.i_transit_up,        kUInt32_t},
        {&Battery_1.i_transit_down,      kUInt32_t},
        {&Battery_1.mode,                kUInt8_t},
        {&Battery_1.null_dev,            kUInt32_t},
        {&Battery_1.constant_timer,      kUInt8_t},
    },
    {
        {&Battery_2.status,              kUInt8_t},
        {&Battery_2.err_code,            kUInt16_t},
        {&Battery_2.voltage,             kFloat},
        {&Battery_2.current,             kFloat},
        {&Battery_2.temperature,         kFloat},
        {&Battery_2.level,               kFloat},
        {&Battery_2.capacity,            kFloat},
        {&Battery_2.mux_on,              kUInt8_t},
        {&Battery_2.remain_time,         kFloat},
        {&Battery_2.charge_iset,         kFloat},
        {&Battery_2.fastcharge_timer,    kFloat},
        {&Battery_2.pre_start_time,      kUInt32_t},
        {&Battery_2.fast_start_time,     kUInt32_t},
        {&Battery_2.trickle_start_time,  kUInt32_t},
        {&Battery_2.charge_finish_time,  kUInt32_t},
        {&Battery_2.charge_times,        kUInt16_t},
        {&Battery_2.scale_flag,          kUInt8_t},
        {&Battery_2.is_aged,             kUInt8_t},
        {&Battery_2.ng,                  kFloat},
        {&Battery_2.finish_code,         kUInt8_t},
        {&Battery_2.i_transit_up,        kUInt32_t},
        {&Battery_2.i_transit_down,      kUInt32_t},
        {&Battery_2.mode,                kUInt8_t},
        {&Battery_2.null_dev,            kUInt32_t},
        {&Battery_2.constant_timer,      kUInt8_t},
    }
};

/* Private function prototypes -----------------------------------------------*/

/* Code begin ----------------------------------------------------------------*/
static void
VariableWrite(const void *pdata, stVariableTable table)
{
    switch (table.dtype) {
    case kUInt8_t:
    case kInt8_t:
        *(uint8_t*)table.pdata = *(uint8_t*)pdata;
        break;
    case kUInt16_t:
    case kInt16_t:
        *(uint16_t*)table.pdata = *(uint16_t*)pdata;
        break;
    case kUInt32_t:
    case kInt32_t:
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
    int32_t value_long = 0;
    TxHeader.ExtId = pcmd->id.all;
    ((stCanId *)(&TxHeader.ExtId))->Src = LOCAL_ID;
    ((stCanId *)(&TxHeader.ExtId))->Target = pcmd->id.field.Src;
    TxData[0] = pcmd->data[0];

    switch (pcmd->data[0]) {
    case MUX_CG_ADAPTOR_VOL:
        value_long = (int32_t)(Adaptor.voltage * 1000);
        break;
    case MUX_CG_BAT_1_VOL:
        value_long = (int32_t)(Battery_1.voltage * 1000);
        break;
    case MUX_CG_BAT_2_VOL:
        value_long = (int32_t)(Battery_2.voltage * 1000);
        break;
    case MUX_CG_BAT_1_CUR:
        value_long = (int32_t)(Battery_1.current * 1000);
        break;
    case MUX_CG_BAT_2_CUR:
        value_long = (int32_t)(Battery_2.current * 1000);
        break;
    case MUX_CG_BAT_1_TEMP:
        value_long = (int32_t)(Battery_1.temperature * 1000);
        break;
    case MUX_CG_BAT_2_TEMP:
        value_long = (int32_t)(Battery_2.temperature * 1000);
        break;
    case MUX_CG_BAT_1_LVL:
        value_long = Battery_1.status == kstsNotExist ? 0 : (int32_t)(Battery_1.level);
        break;
    case MUX_CG_BAT_2_LVL:
        value_long = Battery_2.status == kstsNotExist ? 0 : (int32_t)(Battery_2.level);
        break;
    case MUX_CG_BAT_1_CAP:
        value_long = Battery_1.status == kstsNotExist ? 0 : (int32_t)(Battery_1.capacity);
        break;
    case MUX_CG_BAT_2_CAP:
        value_long = Battery_2.status == kstsNotExist ? 0 : (int32_t)(Battery_2.capacity);
        break;
    case MUX_CG_BAT_1_REMAIN_TIME:
        if (Battery_1.status != kstsNotExist)
            value_long = Battery_1.remain_time < 0 ? 0 : (int32_t)Battery_1.remain_time;
        else
            value_long = 0;
        break;
    case MUX_CG_BAT_2_REMAIN_TIME:
        if (Battery_2.status != kstsNotExist)
            value_long = Battery_2.remain_time < 0 ? 0 : (int32_t)Battery_2.remain_time;
        else
            value_long = 0;
        break;
    case MUX_CG_BAT_1_CHARGE_TIMES:
        value_long = (int32_t)Battery_1.charge_times;
        break;
    case MUX_CG_BAT_2_CHARGE_TIMES:
        value_long = (int32_t)Battery_2.charge_times;
        break;
    case MUX_CG_HEATER_TEMP:
        value_long = (int32_t)(Heater.temperature * 1000);
        break;
    case MUX_CG_HEATER_DUTY:
        value_long = (int32_t)(Heater.pwm.duty * 10000);
        break;
    case MUX_CG_OUTPUT_VOL:
        value_long = (int32_t)(FieldCase.v_syspwr * 1000);
        break;
    case MUX_CG_OUTPUT_CUR:
        value_long = (int32_t)(FieldCase.consumption * 1000);
        break;
    case MUX_CG_PRESSURE_1:
        value_long = (int32_t)(FieldCase.gas_1_pressure * 1000);
        break;
    case MUX_CG_PRESSURE_2:
        value_long = (int32_t)(FieldCase.gas_2_pressure * 1000);
        break;
    default : break;
    }
    WriteWordL(&TxData[1], value_long);
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
    printf("answer 'read_id'\n\r");
}

static void
DetectADCReadHandler(const stCanPacket *pcmd)
{
    uint16_t value = 0;
    TxHeader.ExtId = pcmd->id.all;
	((stCanId *)(&TxHeader.ExtId))->Src = LOCAL_ID;
	((stCanId *)(&TxHeader.ExtId))->Target = pcmd->id.field.Src;
    
    // block A
    if (Adaptor.status != kAdaptorNotExist)
        value |= DIO_ADAPTOR_SUPPLY;
    else
        value &= ~DIO_ADAPTOR_SUPPLY;

    if (Battery_1.mux_on == true)
        value |= DIO_BAT1_SUPPLY;
    else
        value &= ~DIO_BAT1_SUPPLY;

    if (Battery_2.mux_on == true)
        value |= DIO_BAT2_SUPPLY;
    else
        value &= ~DIO_BAT2_SUPPLY;

    if (Fan.status != kLoadOff)
        value |= DIO_GCFAN;
    else
        value &= ~DIO_GCFAN;

    if (Heater.mode != kHeaterOff)
        value |= DIO_HEATER;
    else
        value &= ~DIO_HEATER;

    if (Battery_1.status >= kstsPreCharge && Battery_1.status < kstsFinish)
        value |= DIO_BAT1_CHARGE;
    else
        value &= ~DIO_BAT1_CHARGE;

    if (Battery_2.status >= kstsPreCharge && Battery_2.status < kstsFinish)
        value |= DIO_BAT2_CHARGE;
    else
        value &= ~DIO_BAT2_CHARGE;

    if (Battery_1.is_aged == true)
        value |= DIO_BAT1_AGED;
    else
        value &= ~DIO_BAT1_AGED;

    if (Battery_2.is_aged == true)
        value |= DIO_BAT2_AGED;
    else
        value &= ~DIO_BAT2_AGED;

    if (Valve_1.status == kLoadOn )
        value |= DIO_VALVE1_SW;
    else
        value &= ~DIO_VALVE1_SW;

    if (Valve_2.status == kLoadOn)
        value |= DIO_VALVE2_SW;
    else
        value &= ~DIO_VALVE2_SW;

    if (Pump_1.dio.status == kLoadOn)
        value |= DIO_PUMP;
    else
        value &= ~DIO_PUMP;

    if (PValve.dio.status == kLoadOn)
        value |= DIO_PUMP_VALVE;
    else
        value &= ~DIO_PUMP_VALVE;

    if (boot_request == kGcFastBoot) {
        value &= ~DIO_BOOT_MODE;
        //printf("answer boot mode: 'fastboot', %d\n\r", boot_request);
    } else {
        value |= DIO_BOOT_MODE;
        //printf("anser boot mode: 'normal', %d\n\r", boot_request);
    }
    
    WriteShortL(TxData, value);

    // block B
    value = 0;

    if (HAL_GPIO_ReadPin(LED_RED_GPIO_Port, LED_RED_Pin)) {
        value |= DIO_LED_RED;
    } else {
        value &= ~DIO_LED_RED;
    }
    
    if (HAL_GPIO_ReadPin(LED_WHITE_GPIO_Port, LED_WHITE_Pin)) {
        value |= DIO_LED_WHITE;
    } else {
        value &= ~DIO_LED_WHITE;
    }
    
    if (!FieldCase.is_covered)
        value |= DIO_HALL_SENSOR;
    else
        value &= ~DIO_HALL_SENSOR;

    WriteShortL(TxData+2, value);
    TxHeader.DLC = 4;
    HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailBox);
}

static void
DIOReadHandler(const stCanPacket *pcmd)
{
    uint16_t value = 0, mask = 0;
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

    switch (block_id) {
    case BLOCK_A:
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

        if ((mask & DIO_HEATER) && Heater.mode != kHeaterOff)
            value |= DIO_HEATER;
        else
            value &= ~DIO_HEATER;

        if ((mask & DIO_BAT1_CHARGE) && (Battery_1.status >= kstsPreCharge && Battery_1.status < kstsFinish))
            value |= DIO_BAT1_CHARGE;
        else
            value &= ~DIO_BAT1_CHARGE;

        if ((mask & DIO_BAT2_CHARGE) && (Battery_2.status >= kstsPreCharge && Battery_2.status < kstsFinish))
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

        if (mask & DIO_BOOT_MODE) {
            if (boot_request == kGcFastBoot) {
                value &= ~DIO_BOOT_MODE;
                //printf("answer boot mode: 'fastboot', %d\n\r", boot_request);
            } else {
                value |= DIO_BOOT_MODE;
                //printf("anser boot mode: 'normal', %d\n\r", boot_request);
            }
        }
        break;
    case BLOCK_B:
        if (mask & DIO_LED_RED) {
            if (HAL_GPIO_ReadPin(LED_RED_GPIO_Port, LED_RED_Pin)) {
                value |= DIO_LED_RED;
            } else {
                value &= ~DIO_LED_RED;
            }
        }

        if (mask & DIO_LED_WHITE) {
            if (HAL_GPIO_ReadPin(LED_WHITE_GPIO_Port, LED_WHITE_Pin)) {
                value |= DIO_LED_WHITE;
            } else {
                value &= ~DIO_LED_WHITE;
            }
        }

        if ((mask & DIO_HALL_SENSOR) && !FieldCase.is_covered)
            value |= DIO_HALL_SENSOR;
        else
            value &= ~DIO_HALL_SENSOR;
        break;
    case BLOCK_C:
        break;
    default : break;
    }

    WriteShortL(TxData, value);
    TxHeader.DLC = 2;
    HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailBox);
}

static void
DIOWriteHandler(const stCanPacket *pcmd)
{
    uint16_t dio_mask = 0;
    uint16_t dio_value = 0;

    printf("DioWriteHandler(): block_%d, mask %x, value %x,\n\r", pcmd->data[0], dio_mask, dio_value);
        dio_value = (uint16_t)pcmd->data[4] << 8 | pcmd->data[3];
        dio_mask = (uint16_t)pcmd->data[2] << 8 | pcmd->data[1];
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
            if (dio_value & DIO_GCFAN)
                DioSetTo(&Fan, 1);
            else
                DioSetTo(&Fan, 0);
        }
        if (dio_mask & DIO_HEATER) {
            if (dio_value & DIO_HEATER)
                Heater.mode = kHeaterPID;
            else
                Heater.mode = kHeaterOff;
        }
        if (dio_mask & DIO_VALVE1_SW) {
            if (dio_value & DIO_VALVE1_SW)
                DioSetTo(&Valve_1, 1);
            else
                DioSetTo(&Valve_2, 0);
        }
        if (dio_mask & DIO_VALVE2_SW) {
            if (dio_value & DIO_VALVE2_SW)
                DioSetTo(&Valve_2, 1);
            else
                DioSetTo(&Valve_2, 0);
        }
        if (dio_mask & (DIO_PUMP)) {
            if (dio_value & DIO_PUMP) {
                DioSetTo(&Pump_1.dio, 1);
            } else {
                DioSetTo(&Pump_1.dio, 0);
            }
        }
        if (dio_mask & DIO_PUMP_VALVE) {
            if (dio_value & DIO_PUMP_VALVE )
                DioSetTo(&PValve.dio, 1);
            else
                DioSetTo(&PValve.dio, 0);
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
            printf("jumping to BL...\n\r");
            // __set_MSP(*(__IO uint32_t*)BL_ADDRESS);
            // ((void(*)(void))(*(__IO uint32_t*)(BL_ADDRESS + 4)))();
            NVIC_SystemReset();
        } else {
            TxData[0] = 0x01;
            TxHeader.DLC = 1;
            HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailBox);
            printf("refuse to jump to BL, BL_ADDRESS not right\n\r");
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
        TxData[2] = Battery_1.err_code;
        TxData[3] = Battery_1.finish_code;
        TxHeader.DLC = 4;
        HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailBox);
        break;
    case 0x02:  // bat 2
        TxData[1] = Battery_2.status;
        TxData[2] = Battery_2.err_code;
        TxData[3] = Battery_2.finish_code;
        TxHeader.DLC = 4;
        HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailBox);
        break;
    case 0x03:  // Heater
        TxData[1] = Heater.mode << 4;
        value_long = (int32_t)(Heater.setpoint * 1000);
        WriteWordL(&TxData[2], value_long);
        value_short = (int16_t)(Heater.pwm.duty * 10000);
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
    case 0x03:  // Heater
        switch (pcmd->data[1]) {
        case 0x00:  // turn off
            Heater.mode = kHeaterOff;
            TxData[1] = 0x00;
            break;
        case 0x01:  // turn on and set setpoint
            setpoint = GetWordL(&pcmd->data[2]) & 0x00ffffff;
            setpoint = (setpoint & 0x00800000) ? (setpoint | 0xff000000) : (setpoint & 0x00ffffff);
            sp = (float)setpoint / 100;
            Heater.setpoint = sp;
            Heater.mode = kHeaterPID;
            TxData[1] = 0x00;
            break;
        case 0x02:  // turn on and set fix duty cycle
            duty = GetShortL(&pcmd->data[2]);
            Heater.pwm.duty = duty / 10000;
            TxData[1] = 0x00;
            break;
        case 0x03:  // reset
            Heater.mode = kHeaterOff;
            Heater.kp = DEFAULT_KP;
            Heater.ki = DEFAULT_KI;
            Heater.kd = DEFAULT_KD;
            PwmSetTo(&Heater.pwm, 0);
            Heater.setpoint = 0;
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
    PulseDevice_t *pulseout;
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

    switch(pcmd->data[3]) {
    case 0x00:
        time = GetWordL(&pcmd->data[5]);
        time &= 0x00ffffff;
        pulseout->start_time = time;
        break;
    case 0x01:
        time = GetWordL(&pcmd->data[5]);
        time &= 0x00ffffff;
        pulseout->stop_time = time;
        break;
    default : break;
    }
}

static void
StartRunHandler(void)
{
//    Pump_1.StartRun();
//    PValve.StartRun();
    Pump_1.sm_state = 1;
    Pump_1.origin = xTaskGetTickCount();

    PValve.sm_state = 1;
    PValve.origin = xTaskGetTickCount();
}

static void
StopRunHandler(void)
{
//    Pump_1.StopRun();
//    PValve.StopRun();
    Pump_1.sm_state = 0;
    DioSetTo(&Pump_1.dio, 0);
    
    PValve.sm_state = 0;
    DioSetTo(&PValve.dio, 0);
}

static void
ContactHandler(const stCanPacket *pcmd)
{
//    Heater::ContactHandler(pcmd->data[0]);
    PwmSetTo(&Heater.pwm, 0);
}

static void
UnknownCmdHandler(const stCanPacket *pcmd)
{
    // TxHeader.ExtId = 0;
    // TxHeader.DLC = 8;
    // memset(TxData, 0xff, 8);
    // HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailBox);
}

static void
DebugRdHandler(const stCanPacket *pcmd)
{
    int16_t value = 0;
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
        case kUInt8_t:
        case kInt8_t:
        {
            uint8_t data = *(uint8_t*)bat_var_table[row][col].pdata;
            TxData[2] = data;
            TxHeader.DLC = 3;
            break;
        }
        case kUInt16_t:
        case kInt16_t:
        {
            uint16_t data = *(uint16_t*)bat_var_table[row][col].pdata;
            WriteShortL(&TxData[2], data);
            TxHeader.DLC = 4;
            break;
        }
        case kUInt32_t:
        case kInt32_t:
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
    case 0x03:      // Heater
        switch (pcmd->data[1]) {
        case 0x00:  // mode
            TxData[2] = Heater.mode;
            TxHeader.DLC = 3;
            break;
        case 0x01:  // temperature
            tu_value.tfloat = Heater.temperature;
            WriteWordL(&TxData[2], tu_value.tint32);
            TxHeader.DLC = 6;
            break;
        case 0x02:  // set point
            tu_value.tfloat = Heater.setpoint;
            WriteWordL(&TxData[2], tu_value.tint32);
            TxHeader.DLC = 6;
            break;
        case 0x03:  // duty
            tu_value.tfloat = Heater.pwm.duty;
            WriteWordL(&TxData[2], tu_value.tint32);
            TxHeader.DLC = 6;
            break;
        case 0x04:  // kp
            WriteShortL(&TxData[2], Heater.kp);
            TxHeader.DLC = 4;
            break;
        case 0x05:  // ki
            WriteShortL(&TxData[2], Heater.ki);
            TxHeader.DLC = 4;
            break;
        case 0x06:  // kd
            WriteShortL(&TxData[2], Heater.kd);
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
    case 0x03:      // Heater
        switch (pcmd->data[1]) {
        case 0x00:  // mode
            Heater.mode = pcmd->data[2];
            break;
        case 0x01:  // temperature:
            break;
        case 0x02:  // setpoint
            tu_value.tint32 = GetWordL(&pcmd->data[2]);
            Heater.setpoint = tu_value.tfloat;
            break;
        case 0x03:  // duty
            tu_value.tint32 = GetWordL(&pcmd->data[2]);
            Heater.pwm.duty = tu_value.tfloat;
            break;
        case 0x04:  // kp
            Heater.kp = GetShortL(&pcmd->data[2]);
            break;
        case 0x05:  // ki
            Heater.ki = GetShortL(&pcmd->data[2]);
            break;
        case 0x06:  // kd
            Heater.kd = GetShortL(&pcmd->data[2]);
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
        DetectADCReadHandler(pcmd);
        break;
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

    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = ((id << 3) & 0xFFFF0000) >> 16;
    sFilterConfig.FilterIdLow = ((id << 3) | CAN_ID_EXT | CAN_RTR_DATA) & 0xFFFF;
    sFilterConfig.FilterMaskIdHigh = ((mask << 3) & 0xFFFF0000) >> 16;;
    sFilterConfig.FilterMaskIdLow = ((mask << 3) | CAN_ID_EXT | CAN_RTR_DATA) & 0xFFFF;;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    id = CANID_BROADCAST << 8;
    sFilterConfig.FilterBank = 2;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = ((id << 3) & 0xFFFF0000) >> 16;
    sFilterConfig.FilterIdLow = ((id << 3) | CAN_ID_EXT | CAN_RTR_DATA) & 0xFFFF;
    sFilterConfig.FilterMaskIdHigh = ((mask << 3) & 0xFFFF0000) >> 16;;
    sFilterConfig.FilterMaskIdLow = ((mask << 3) | CAN_ID_EXT | CAN_RTR_DATA) & 0xFFFF;;
    sFilterConfig.FilterFIFOAssignment = 0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_CAN_Start(&hcan) != HAL_OK)
    {
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

    xQueueSendToBackFromISR(q_canmsg, &rx_msg, &pxHigherPriorityTaskWoken);
    CAN_Listen();
    if (pxHigherPriorityTaskWoken == pdTRUE)
        portYIELD_FROM_ISR(pdTRUE);
}

void MsgHandler(const void *param)
{
    // uint8_t i = 0;
    // int8_t index = 0;
    // static uint32_t old_intr_cnt;
    // static uint32_t old_fetch_cnt;
    static CANMsg_t msg;
    static stCanPacket pkt;

    while (1) {
        xQueueReceive(q_canmsg, &msg, portMAX_DELAY);
        pkt.id.all = msg.header.ExtId;
        pkt.dlc = msg.header.DLC;
        memcpy(pkt.data, msg.data, 8);
        CmdHandler((const stCanPacket *)&pkt);
    }
}


