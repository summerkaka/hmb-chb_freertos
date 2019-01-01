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
#include "app.h"

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
#define BAT_MODE            9

#define TABLE_ROW           10     // table depth
#define TABLE_COL           3
#define BAT2_OFFSET         512

#define EROM_I2CID          0x50   // 7 bit eeprom address
#define PAGE_SIZE           16     // 24LC08B page buffer size
#define WRITE_DELAY         6      // millisecond    
#define MAX_ADDR            1023   // eeprom byte size
#define p_hi2c              &hi2c1 // i2c bus instance of mcu

#define RECORD_INTERVAL     1   // minutes

/* Private typedef -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static const uint16_t IndexTable[TABLE_ROW][TABLE_COL] = {
    //INDEX                 ADDR    LENGTH
    INITIAL_FLAG	    ,	0	,	1	,
    BAT_ERR_CODE	    ,	1	,	1	,
    BAT_CAPACITY	    ,	2	,	4	,
    BAT_LEVEL	        ,	6	,	4	,
    BAT_CHARGE_TIMES	,	10	,	2	,
    BAT_EFFICIENCY	    ,	12	,	4	,
    BAT_IMPEDANCE	    ,	16	,	4	,
    BAT_IS_AGED	        ,	20	,	1	,
    BAT_RL_ARRAY        ,   21  ,   460 ,
    BAT_MODE            ,  481  ,   1   
};


/* Private function prototypes -----------------------------------------------*/

/* Code begin ----------------------------------------------------------------*/
static HAL_StatusTypeDef
EepromPageWrite(stEromOps *erom)
{
	uint16_t length = 0;
    uint8_t data_addr = 0;
    uint8_t id_addr = 0;
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
		ret = HAL_I2C_Mem_Write(p_hi2c, id_addr, data_addr, I2C_MEMADD_SIZE_8BIT, (uint8_t *)erom->pdata, length, erom->time_out);
		if (ret != HAL_OK) {
            printf("EepromPageWrite(): Write Fail! id_addr: 0x%02x, data_addr: 0x%02x\n\r", id_addr, data_addr);
			break;
        }
		erom->data_addr += length;
		erom->len -= length;
		erom->pdata += length;
		vTaskDelay(5 / portTICK_PERIOD_MS);
	}
	return ret;
}

static HAL_StatusTypeDef
EepromRead(stEromOps *erom)
{
	HAL_StatusTypeDef ret = HAL_ERROR;
    uint8_t data_addr = erom->data_addr & 0xff;
    uint8_t id_addr = (EROM_I2CID | (erom->data_addr >> 8)) << 1;

	ret = HAL_I2C_Mem_Read(p_hi2c, id_addr, data_addr, I2C_MEMADD_SIZE_8BIT, erom->pdata, erom->len, erom->time_out);

	return ret;
}

// index means descriptor index
static HAL_StatusTypeDef
BatteryDescriptorSave(stBattery *bat, uint8_t index, void *data)
{
    stEromOps erom;

    erom.data_addr = (bat->index - 1) * BAT2_OFFSET + IndexTable[index][1];
    erom.len = IndexTable[index][2];
    erom.pdata = (uint8_t *)data;
    erom.time_out = 15;
    return EepromPageWrite(&erom);
}

static HAL_StatusTypeDef
BatteryDescriptorLoad(stBattery *bat, uint8_t index, void *data)
{
    stEromOps erom;

    erom.data_addr = (bat->index - 1) * BAT2_OFFSET + IndexTable[index][1];
    erom.len = IndexTable[index][2];
    erom.pdata = (uint8_t *)data;
    erom.time_out = 15;
    return EepromRead(&erom);
}

static HAL_StatusTypeDef
BatteryDataSave(stBattery *bat)
{
    HAL_StatusTypeDef ret = HAL_ERROR;
    uint8_t init_flag = 0x01;
    uint16_t i = 0, j = 0;
    int16_t level[230] = {0,};
    tuType32 tu_value;

    if (bat->status == kstsNotExist)
        return HAL_ERROR;

    // save error code
    if (ret = BatteryDescriptorSave(bat, BAT_ERR_CODE, &bat->err_code))
        return ret;
    
    // save total capacity
    tu_value.tfloat = bat->capacity;
    if (ret = BatteryDescriptorSave(bat, BAT_CAPACITY, &tu_value.tint32))
        return ret;
    
    // save current level
    tu_value.tfloat = bat->level;
    if (ret = BatteryDescriptorSave(bat, BAT_LEVEL, &tu_value.tint32))
        return ret;
    
    // save charge times
    if (ret = BatteryDescriptorSave(bat, BAT_CHARGE_TIMES, &bat->charge_times))
        return ret;
    
    // save efficiency
    tu_value.tfloat = bat->ng;
    if (ret = BatteryDescriptorSave(bat, BAT_EFFICIENCY, &tu_value.tint32))
        return ret;
    
    // save aging flag
    if (ret = BatteryDescriptorSave(bat, BAT_IS_AGED, &bat->is_aged))
        return ret;

    // save red_line_array
    for (i = 0; i < RL_DEP; i++) {
        for (j = 0; j < RL_WID; j++) {
            level[i*10 + j] = bat->red_line[i][j];
        }
    }
    if (ret = BatteryDescriptorSave(bat, BAT_RL_ARRAY, level))
        return ret;
    
    // set initial flag
    ret = BatteryDescriptorSave(bat, INITIAL_FLAG, &init_flag);
        return ret;
}

HAL_StatusTypeDef
BatteryDataClear(stBattery *bat)
{
    uint8_t i = 0;
    uint32_t data = 0;
    HAL_StatusTypeDef ret = HAL_ERROR;

    xSemaphoreTake(bat->mutex_i2c, portMAX_DELAY);
    
    for (i = 0; i < TABLE_ROW; i++) {
        if (ret = BatteryDescriptorSave(bat, i, &data))
            goto end;
    }
    
end:
    xSemaphoreGive(bat->mutex_i2c);
    return ret;
}

HAL_StatusTypeDef
BatteryDataLoad(stBattery *bat)
{
    HAL_StatusTypeDef ret = HAL_ERROR;
    uint8_t init_flag = 0;
    uint8_t i = 0, j = 0;
    int16_t buf[230] = {0};
    tuType32 tu_value;

    xSemaphoreTake(bat->mutex_i2c, portMAX_DELAY);

    if (ret = BatteryDescriptorLoad(bat, INITIAL_FLAG, &init_flag))
        goto end;
    if (init_flag != 0x01) {
        ret = HAL_ERROR;
        goto end;
    }

    // load error code
    if (ret = BatteryDescriptorLoad(bat, BAT_ERR_CODE, &bat->err_code))
        goto end;

    // load capacity
    if (ret = BatteryDescriptorLoad(bat, BAT_CAPACITY, &tu_value.tint32))
        goto end;
    bat->capacity = tu_value.tfloat;

    // load level
    if (ret = BatteryDescriptorLoad(bat, BAT_LEVEL, &tu_value.tint32))
        goto end;
    bat->level = bat->gauge->level = tu_value.tfloat;

    // load charge_times
    if (ret = BatteryDescriptorLoad(bat, BAT_CHARGE_TIMES, &bat->charge_times))
        goto end;

    // load ng
    if (ret = BatteryDescriptorLoad(bat, BAT_EFFICIENCY, &tu_value.tint32))
        goto end;
    bat->ng = tu_value.tfloat;

    // load red_line_array
    if (ret = BatteryDescriptorLoad(bat, BAT_RL_ARRAY, buf))
        goto end; 
    for (i = 0; i < RL_DEP; i++) {
        for (j = 0; j < RL_WID; j++) {
            bat->red_line[i][j] = buf[i*10 + j];
        }
    }

    // load charge mode
    if (ret = BatteryDescriptorLoad(bat, BAT_MODE, buf)) {
        goto end;
    } else {
        bat->mode = buf[0] == 1 ? kCalifornia : kGlobal;
        if (buf[0] != 2 && buf[0] != 1)
            printf("Battery_%d set to default mode\n\r", bat->index);
    }

    // load aging flag
    if (ret = BatteryDescriptorLoad(bat, BAT_IS_AGED, &bat->is_aged))
        goto end;
end:
    xSemaphoreGive(bat->mutex_i2c);
    return ret;
}

void
Thread_BatteryDataLog(void)
{
    while (1) {
        if (xSemaphoreTake(mutex_i2c0, 1000) == pdTRUE) {
            BatteryDataSave(&Battery_1);
            xSemaphoreGive(mutex_i2c0);
        }
        if (xSemaphoreTake(mutex_i2c1, 1000) == pdTRUE) {
            BatteryDataSave(&Battery_2);
            xSemaphoreGive(mutex_i2c1);
        }

        vTaskDelay(RECORD_INTERVAL * 60 * 1000 / portTICK_PERIOD_MS);
    }
}

void
BatteryErrorLog(stBattery *bat)
{
    if (xSemaphoreTake(bat->mutex_i2c, 1000) == pdTRUE) {
        BatteryDescriptorSave(bat, BAT_ERR_CODE, &bat->err_code);
        xSemaphoreGive(bat->mutex_i2c);
    }
}

void 
BatteryModeSave(eRegionMode mode)
{
    xSemaphoreTake(mutex_i2c0, portMAX_DELAY);
    BatteryDescriptorSave(&Battery_1, BAT_MODE, &mode);
    xSemaphoreGive(mutex_i2c0);
    
    xSemaphoreTake(mutex_i2c1, portMAX_DELAY);
    BatteryDescriptorSave(&Battery_2, BAT_MODE, &mode);
    xSemaphoreGive(mutex_i2c1);
}


