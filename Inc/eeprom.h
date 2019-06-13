/**
  ******************************************************************************
  * @file    Project/src/eeprom.h
  * @author
  * @version V0.00
  * @date
  * @brief   eeprom.h
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __EEPROM_H
#define __EEPROM_H
//#ifdef __cplusplus
// extern "C" {
//#endif


/* Includes ------------------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef struct {
	uint16_t data_addr;
	uint16_t len;
	uint8_t *pdata;
    uint8_t time_out;
} stEromOps;

typedef enum {
    kUInt8 = 1,
    kUInt16 = 2,
    kUInt32 = 3,
    kUInt64 = 4,
    kInt8 = 11,
    kInt16 = 12,
    kInt32 = 13,
    kInt64 = 14,
    kFloat = 20,
    kDouble = 21,
    kBool = 30
} eDType;

typedef struct {
    void *pdata;
    eDType dtype;
} stVariableTable;

/* Exported constants --------------------------------------------------------*/

/* Exported variables ------------------------------------------------------- */

/* Exported functions ------------------------------------------------------- */
HAL_StatusTypeDef BatteryDataLoad(stBattery *bat);
HAL_StatusTypeDef BatteryDataSave(stBattery *bat);
HAL_StatusTypeDef BatteryDataClear(stBattery *bat);
void BatteryDataLog(void *);
void BatteryErrorLog(stBattery *bat);
void BatteryModeSave(eRegionMode mode);
void ValveDataLoad(void);
void ValveDataSave(uint8_t valve, uint8_t data);


//#ifdef __cplusplus
//}
//#endif
#endif /*__EEPROM_H */

