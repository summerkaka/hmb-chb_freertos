/**
  ******************************************************************************
  * @file    Project/src/eeprom.c
  * @author
  * @version V0.00
  * @date
  * @brief   eeprom operation
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "app_include.h"

/* Private macro -------------------------------------------------------------*/
#define	INITIAL_FLAG	    0
#define	BAT_ERR_CODE	    1
#define	BAT_CAPACITY	    2
#define	BAT_LEVEL	        3
#define	BAT_CHARGE_TIMES	4
#define	BAT_EFFICIENCY	    5
#define	BAT_IMPEDANCE	    6
#define	BAT_IS_AGED	        7
#define BAT_RL_ARRAY        8
#define BAT_REMAIN_TIME     9
#define BAT_MODE            10          // always the last to prevent from clear

#define TABLE_ROW           11          // table depth
#define TABLE_COL           3
#define BAT2_OFFSET         512

#define EROM_I2CID          0x50        // 7 bit eeprom address
#define PAGE_SIZE           16          // 24LC08B page buffer size
#define WRITE_DELAY         3           // millisecond
#define MAX_ADDR            1023        // eeprom byte size
#define p_hi2c              &hi2c1      // i2c bus instance of mcu

#define RECORD_INTERVAL     1           // minutes

#define UINT8               0
#define INT8                1


/* Private typedef -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static const uint16_t IndexTable[TABLE_ROW][TABLE_COL] = {
    //DTYPE               ADDR    LENGTH
    kUInt8	            ,	0	,	1	,   //INITIAL_FLAG
    kUInt16     	    ,	1	,	1	,   //BAT_ERR_CODE
    kFloat	            ,	2	,	4	,   //BAT_CAPACITY
    kFloat  	        ,	6	,	4	,   //BAT_LEVEL
    kUInt16         	,	10	,	2	,   //BAT_CHARGE_TIMES
    kFloat  	        ,	12	,	4	,   //BAT_EFFICIENCY
    kFloat      	    ,	16	,	4	,   //BAT_IMPEDANCE
    kBool   	        ,	20	,	1	,   //BAT_IS_AGED
    kInt16              ,   21  ,   460 ,   //BAT_RED_LINE
    kFloat              ,   481 ,   4   ,   //BAT_REMAIN_TIME
    kUInt8              ,   511 ,   1       //BAT_MODE
};


/* Private function prototypes -----------------------------------------------*/

/* Code begin ----------------------------------------------------------------*/
static HAL_StatusTypeDef EepromPageWrite(stEromOps *erom)
{
	uint16_t length = 0;
    uint8_t data_addr = 0;
    uint8_t id_addr = 0;
    uint8_t i = 0;
	HAL_StatusTypeDef ret = HAL_ERROR;

    // if(__HAL_I2C_GET_FLAG(p_hi2c, I2C_FLAG_BUSY) == SET)
    //     return HAL_BUSY;
	if (erom->data_addr > MAX_ADDR)
		return HAL_ERROR;

	while (erom->len > 0) {
        data_addr = erom->data_addr & 0xff;
        if ((data_addr & 0x0f) + erom->len > 16)
            length = 16 - (data_addr & 0x0f);
        else
            length = erom->len;
        id_addr = (EROM_I2CID | (erom->data_addr >> 8)) << 1;
        for (i=0; i<3; i++) {       // eeprom has write interval, try 3 times
            if ((ret = HAL_I2C_Mem_Write(p_hi2c, id_addr, data_addr, I2C_MEMADD_SIZE_8BIT, (uint8_t *)erom->pdata, length, erom->time_out)) != HAL_OK)
                osDelay(WRITE_DELAY);
            else
                break;
        }
		if (i == 3) {
            xprintf("EepromPageWrite(): Write Fail! id_addr: 0x%02x, data_addr: 0x%02x\n\r", id_addr, data_addr);
			break;
        }
		erom->data_addr += length;
		erom->len -= length;
		erom->pdata += length;
	}
	return ret;
}

static HAL_StatusTypeDef EepromRead(stEromOps *erom)
{
	HAL_StatusTypeDef ret = HAL_ERROR;

    if (erom->data_addr > MAX_ADDR)
		return HAL_ERROR;

    uint8_t data_addr = erom->data_addr & 0xff;
    uint8_t id_addr = (EROM_I2CID | (erom->data_addr >> 8)) << 1;

	ret = HAL_I2C_Mem_Read(p_hi2c, id_addr, data_addr, I2C_MEMADD_SIZE_8BIT, erom->pdata, erom->len, erom->time_out);

	return ret;
}

// index means descriptor index
static HAL_StatusTypeDef DescriptorSave(uint8_t table_offset, uint8_t index, void *data)
{
    stEromOps erom;

    if (table_offset != 0 && table_offset != 1)
        return HAL_ERROR;

    erom.data_addr = table_offset * BAT2_OFFSET + IndexTable[index][1];
    erom.len = IndexTable[index][2];
    erom.pdata = (uint8_t *)data;
    erom.time_out = 15;
    return EepromPageWrite(&erom);
}

static HAL_StatusTypeDef DescriptorLoad(uint8_t table_offset, uint8_t index, void *data)
{
    stEromOps erom;

    if (table_offset != 0 && table_offset != 1)
        return HAL_ERROR;

    erom.data_addr = table_offset * BAT2_OFFSET + IndexTable[index][1];
    erom.len = IndexTable[index][2];
    erom.pdata = (uint8_t *)data;
    erom.time_out = 15;
    return EepromRead(&erom);
}

HAL_StatusTypeDef BatteryDataSave(stBattery *bat)
{
    HAL_StatusTypeDef ret = HAL_ERROR;
    uint8_t init_flag = 0x01;
    uint16_t i = 0, j = 0;
    int16_t *pWriteBuffer;
    tuType32 tu_value;

    if (osMutexWait(bat->iic_mutex, 1000) != osOK)
        return HAL_BUSY;

    if (bat->status == kstsNotExist)
        return HAL_ERROR;

    if (bat->index != 1 && bat->index != 2)
        return HAL_ERROR;

    if ((pWriteBuffer = pvPortMalloc(RL_DEP*RL_WID*sizeof(int16_t))) == NULL) {
        xprintf("BatteryDataSave fail to malloc mem\n\r");
        return HAL_ERROR;
    }

    // save error code
    if (ret = DescriptorSave(bat->index-1, BAT_ERR_CODE, &bat->err_code))
        goto end;

    // save total capacity
    tu_value.tfloat = bat->capacity;
    if (ret = DescriptorSave(bat->index-1, BAT_CAPACITY, &tu_value.tint32))
        goto end;

    // save current level
    tu_value.tfloat = bat->level;
    if (ret = DescriptorSave(bat->index-1, BAT_LEVEL, &tu_value.tint32))
        goto end;

    // save charge times
    if (ret = DescriptorSave(bat->index-1, BAT_CHARGE_TIMES, &bat->charge_times))
        goto end;

    // save efficiency
    tu_value.tfloat = bat->ng;
    if (ret = DescriptorSave(bat->index-1, BAT_EFFICIENCY, &tu_value.tint32))
        goto end;

    // save aging flag
    if (ret = DescriptorSave(bat->index-1, BAT_IS_AGED, &bat->is_aged))
        goto end;

    // save red_line_array
    for (i = 0; i < RL_DEP; i++) {
        for (j = 0; j < RL_WID; j++) {
            pWriteBuffer[i*10 + j] = bat->red_line[i][j];
        }
    }
    if (ret = DescriptorSave(bat->index-1, BAT_RL_ARRAY, pWriteBuffer))
        goto end;

    // save remain_time
    if (ret = DescriptorSave(bat->index-1, BAT_REMAIN_TIME, &bat->remain_time))
        goto end;

    // set initial flag
    ret = DescriptorSave(bat->index-1, INITIAL_FLAG, &init_flag);
end:
    vPortFree(pWriteBuffer);
    osMutexRelease(bat->iic_mutex);
    return ret;
}

HAL_StatusTypeDef BatteryDataClear(stBattery *bat)
{
    uint8_t i = 0;
    uint32_t data = 0;
    uint8_t  depth = TABLE_ROW - 1;         // don't clear battery mode
    HAL_StatusTypeDef ret = HAL_ERROR;

    if (osMutexWait(bat->iic_mutex, 1000) != osOK)
        return HAL_BUSY;


    for (i = 0; i < depth; i++) {
        if (ret = DescriptorSave(bat->index-1, i, &data))
            break;
    }
    osMutexRelease(bat->iic_mutex);
    return ret;
}

HAL_StatusTypeDef BatteryDataLoad(stBattery *bat)
{
    HAL_StatusTypeDef ret = HAL_ERROR;
    uint8_t init_flag = 0;
    uint8_t i = 0, j = 0;
    int16_t *buf;
    tuType32 tu_value;

    if (osMutexWait(bat->iic_mutex, 1000) != osOK)
        return HAL_BUSY;

    if ((buf = pvPortMalloc(RL_DEP*RL_WID*sizeof(int16_t))) == NULL) {
        xprintf("BatteryDataLoad fail to malloc mem\n\r");
        return HAL_ERROR;
    }

    // load charge mode
    if (ret = DescriptorLoad(bat->index-1, BAT_MODE, &i)) {
        goto end;
    } else {
        bat->mode = i == 1 ? kCalifornia : kGlobal;
        if (i != 2 && i != 1)
            xprintf("Battery_%d set to default mode\n\r", bat->index);
    }

    if (ret = DescriptorLoad(bat->index-1, INITIAL_FLAG, &init_flag))
        goto end;
    if (init_flag != 0x01) {
        ret = HAL_ERROR;
        goto end;
    }

    // load error code
    if (ret = DescriptorLoad(bat->index-1, BAT_ERR_CODE, &bat->err_code))
        goto end;

    // load capacity
    if (ret = DescriptorLoad(bat->index-1, BAT_CAPACITY, &tu_value.tint32))
        goto end;
    bat->capacity = tu_value.tfloat;

    // load level
    if (ret = DescriptorLoad(bat->index-1, BAT_LEVEL, &tu_value.tint32))
        goto end;
    bat->level = bat->gauge->level = tu_value.tfloat;

    // load charge_times
    if (ret = DescriptorLoad(bat->index-1, BAT_CHARGE_TIMES, &bat->charge_times))
        goto end;

    // load ng
    if (ret = DescriptorLoad(bat->index-1, BAT_EFFICIENCY, &tu_value.tint32))
        goto end;
    bat->ng = tu_value.tfloat;

    // load red_line_array
    if (ret = DescriptorLoad(bat->index-1, BAT_RL_ARRAY, buf))
        goto end;
    for (i = 0; i < RL_DEP; i++) {
        for (j = 0; j < RL_WID; j++) {
            bat->red_line[i][j] = buf[i*10 + j];
        }
    }

    // load remain_time
    if (ret = DescriptorLoad(bat->index-1, BAT_REMAIN_TIME, &tu_value.tint32))
        goto end;
    bat->remain_time = isnormal(tu_value.tfloat) ? tu_value.tfloat : 0;

    // load aging flag
    ret = DescriptorLoad(bat->index-1, BAT_IS_AGED, &bat->is_aged);
end:
    vPortFree(buf);
    osMutexRelease(bat->iic_mutex);
    return ret;
}

void BatteryDataLog(void *p)
{
    xprintf("thread_batlog start\n\r");

    while (1)
    {
        osDelay(RECORD_INTERVAL * 60 * 1000);

        if (Adaptor.status > kAdaptorNotExist && Battery_1.status < kstsPreCharge && Battery_2.status < kstsPreCharge)
            continue;

        xprintf("bat log start\n\r");
        if (Battery_1.status != kstsNotExist) {
            BatteryDataSave(&Battery_1);
        }
        if (Battery_2.status != kstsNotExist) {
            BatteryDataSave(&Battery_2);
        }
        xprintf("bat log finish\n\r");
    }
}

void BatteryErrorLog(stBattery *bat)
{
    if (osMutexWait(bat->iic_mutex, 1000) == osOK) {
        DescriptorSave(bat->index-1, BAT_ERR_CODE, &bat->err_code);
        osMutexRelease(bat->iic_mutex);
    }
}

void BatteryModeSave(eRegionMode mode)
{
    if (osMutexWait(Battery_1.iic_mutex, 1000) == osOK) {
        DescriptorSave(Battery_1.index-1, BAT_MODE, &mode);
        osMutexRelease(Battery_1.iic_mutex);
    }

    if (osMutexWait(Battery_2.iic_mutex, 1000) == osOK) {
        DescriptorSave(Battery_2.index-1, BAT_MODE, &mode);
        osMutexRelease(Battery_2.iic_mutex);
    }
    Battery_1.mode = mode;
    Battery_2.mode = mode;
}

void ValveDataLoad(void)
{
    uint8_t data = 0;
    stEromOps erom;

    erom.len = 1;
    erom.data_addr = 510;
    erom.pdata = &data;
    erom.time_out = 10;

    if (osMutexWait(mutex_iic0Handle, 1000) == osOK) {
        EepromRead(&erom);
        osMutexRelease(mutex_iic0Handle);
        DioSetTo(&Valve_1, data == 1);
    }

    erom.data_addr = 1022;
    if (osMutexWait(mutex_iic1Handle, 1000) == osOK) {
        EepromRead(&erom);
        osMutexRelease(mutex_iic1Handle);
        DioSetTo(&Valve_2, data == 1);
    }
}

void ValveDataSave(uint8_t valve, uint8_t data)
{
    stEromOps erom;
    osMutexId mutex;


    if (valve != 0 && valve != 1)
        return;
    erom.len = 1;
    erom.data_addr = valve == 0 ? 510 : 1022;
    mutex = valve == 0 ? mutex_iic0Handle : mutex_iic1Handle;
    erom.pdata = &data;
    erom.time_out = 10;

    if (osMutexWait(mutex, 1000) == osOK) {
        EepromPageWrite(&erom);
        osMutexRelease(mutex);
    }

}
