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
    kUInt8_t = 1,
    kUInt16_t = 2,
    kUInt32_t = 3,
    kUInt64_t = 4,
    kInt8_t = 11,
    kInt16_t = 12,
    kInt32_t = 13,
    kInt64_t = 14,
    kFloat = 20,
    kDouble = 21,
} eDType;

typedef struct {
    void *pdata;
    eDType dtype;
} stVariableTable;

/* Exported constants --------------------------------------------------------*/

/* Exported variables ------------------------------------------------------- */

/* Exported functions ------------------------------------------------------- */
HAL_StatusTypeDef BatteryDataLoad(stBattery *bat);
HAL_StatusTypeDef BatteryDataClear(stBattery *bat);
void BatteryDataLog(void);
void BatteryErrorLog(stBattery *bat);
void BatteryModeSave(eRegionMode mode);


//#ifdef __cplusplus
//}
//#endif
#endif /*__EEPROM_H */

