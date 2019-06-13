/**
  ******************************************************************************
  * @file    Project/src/FieldCase.h
  * @author
  * @version V0.00
  * @date
  * @brief   FieldCase.h
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FIELDCASE_H
#define __FIELDCASE_H
//#ifdef __cplusplus
// extern "C" {
//#endif


/* Includes ------------------------------------------------------------------*/
#include "battery.h"


/* Exported macro ------------------------------------------------------------*/


/* Exported types ------------------------------------------------------------*/
typedef enum {
	kNoBat = 0,
	kBat1,
	kBat2,
    kBatBoth
} eBatChannel;

typedef enum ButtonLedMode {
    kLedOff = 0,
    kLedOn,
    kLedBlink
} eLedMode;

typedef struct {
    eLedMode mode;
    uint32_t tick_blink;
    GPIO_TypeDef    * const led_port;
    uint16_t        const led_pin;
} stBlinkLed;

typedef enum {
    kGcOff = 0,
    kGcFastBoot,
    kGcRun
} eBootMode;

typedef struct {
	float       consumption;
	float       v_syspwr;
	bool        is_covered;
    bool        is_switchon;
    stBattery   *bat_used;
    uint32_t    switchoff_time; // seconds
    uint32_t    switchon_time;  // seconds, used to prevent spurious lo_pwr_trigger
    uint32_t    lid_off_time;   // seconds
    uint32_t    lid_on_time;    // seconds
    float       env_temperature;
    float       gas_1_pressure;
    float       gas_2_pressure;
    uint32_t    i_transit_up;
    uint32_t    i_transit_down;
} stFieldCase;


/* Exported constants --------------------------------------------------------*/

/* Exported variables ------------------------------------------------------- */
extern stFieldCase FieldCase;
extern bool need_dual_bat;
extern bool back_cur_detect;
extern bool low_power_flag;
extern uint32_t low_power_time;
extern eBootMode gc_status;
extern int32_t total_bat_time;
extern int32_t total_bat_percent;

/* Exported functions ------------------------------------------------------- */
void TurnOffGc(void *);
void Thread_FieldcaseInfoUpdate(void *p);
void Thread_GcPwrCntl(void *p);
void Thread_CbPwrCntl(void *p);


//#ifdef __cplusplus
//}
//#endif
#endif /*__FIELDCASE_H */

