/**
  ******************************************************************************
  * @file    Project/src/Battery.h
  * @author
  * @version V0.00
  * @date
  * @brief   Battery.h
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BATTERY_H
#define __BATTERY_H
//#ifdef __cplusplus
// extern "C" {
//#endif


/* Includes ------------------------------------------------------------------*/
#include "LTC2943.h"

/* Exported macro ------------------------------------------------------------*/
#define CHARGE_START_TH_VOLT 	14.5    // charge would not start if volt > this
#define CHARGE_STOP_TH_VOLT 	15.3    // force charge stop if volt > this
#define CHARGE_STOP_TH_TEMP     70      // force charge stop if temp > this
#define DROP_VOLTAGE            1       // force charge stop if voltage step drop > this

#define GC_STOP_TH_VOLT 	    10.0    // GC stop if volt < this
#define CB_STOP_TH_VOLT         8.5     // charge board stop if volt < this

#define BAT_ERR_CHARGE_CURRENT	6.5     // battery error if its charge current > this
#define BAT_ERR_SUPPLY_CURRENT  -14     // battery error if its suppy current < this (abs > this)
#define BAT_ERROR_TH_TEMP       85      // battery over-heat threshold

#define BAT_MAX_MAH				22000   // spec of new battery capacity
#define BAT_AGED_MAH_TH         15000   // defined aging battery capacity

#define DELTA_V                 0.05    // δV judgement for charge finish, this value influence charge efficiency certification

#define REACHTOP                0x02    // flag used when rescaling capacity
#define REACHBOTTOM             0x01    // flag used when rescaling capacity
#define FIRSTCHARGE             0x04    // for const-charge mode 1st time charge, this time would charge with 11 hour

#define OVER_HEAT               0x0001
#define OVER_CURRENT            0x0002
#define OVER_VOLTAGE            0X0004
#define CELL_SHORT              0x0008
#define CELL_OPEN               0x0010
#define NTC_OPEN                0x0020
#define GAUGE_FAIL              0x0040
#define CODE_BUG                0xffff

#define FILT_BUF_SIZE           45
#define RING_BUF_SIZE           160
#define RL_DEP                  23
#define RL_WID                  10

#define ON                      true
#define OFF                     false


/* Exported types ------------------------------------------------------------*/
typedef enum {
	kstsNotExist = 0,   // not initialized
	kstsError,          // in wrong status
	kstsFloat,          // for bat: not charging and not supplying, for adaptor: not supplying to GC, could be charging bat or not
    kstsFinish,         // charge finish, only use to inform
	kstsPreCharge,      // trickle current after over discharge
	kstsFastCharge,     // 5A charging
	kstsTrickle,        // Trickle charge mode
    kstsConstant        // constant 1A current charge for 'california mode'
} eBatStatus;

typedef enum {
    kCalifornia = 1,    // this is default mode, charge would be constant 1A
    kGlobal = 2         // this is smart mode, charge includes 'pre-charge', 'fast-charge', 'trickle-charge'
} eRegionMode;

typedef enum {
    kNotCharge = 0,
    kPreCharge,
    kFast,
    kTrickle,
    kConstant
} eChargeMode;

typedef enum {
    kReachBottom = 1,
    kReachTop = 2,
} eCheckMode;

typedef struct {
    uint8_t read_index;
    uint8_t write_index;
    bool    clear;
    float   data[RING_BUF_SIZE];
} stRingBuffer;

struct StNTC {
    uint16_t        * const ntc_adc;
    GPIO_TypeDef    * const ntc_open_port;
    uint16_t        const ntc_open_pin;
    int32_t         ntc_open_time;
};

struct StBattery {
    uint8_t         const index;
    GPIO_TypeDef    * const charge_port;
    uint16_t        const charge_pin;
    GPIO_TypeDef    * const supply_port;
    uint16_t        const supply_pin;
    GPIO_TypeDef    * const connect_port;
    uint16_t        const connect_pin;
    uint16_t        * const ntc_adc;
    I2C_HandleTypeDef * const hi2c;
    stGauge         * const gauge;
    struct StBattery * const substitute;
    SemaphoreHandle_t   mutex_i2c;
    SemaphoreHandle_t   sem_monitor;
    eRegionMode     mode;
    eBatStatus      status;
    bool            mux_on;                     // ideal-diode switch status
    float           voltage;
    float           peak_volt;                  // to judge delta-v with dynamic charge
    float           last_voltage;               // to monitor voltage drop
    float           last_level;                 // for calculate remain time
    float           remain_time;                // battery remain time to reach 10V alarm_line
    float           current;
    float           ng;                         // 0~1 , battery charge efficiency
    float           level;                      // current capacity
    float           capacity;                   // record last full cycle discharge coulomb
    uint8_t	        scale_flag;	                // 0b00x1: level_bottom record, 0b001x: level_top record
    stRingBuffer    v_ringbuf;                  // to get max value of bat voltage
    float           temperature;
    bool            is_aged;
    uint8_t         aging_cnt[2];               // [0] for reach top, [1] for reach bottom
    float           charge_iset;                // dynamic setting for fast-charge
    uint32_t        i_transit_up;               // current transient happen time second
    uint32_t        i_transit_down;             // current transient happen time second
    uint32_t        tick_gaugesample;           // sample gauge voltage every 64ms
    uint32_t        tick_predict;               // tick to record battery vdrop, unit is second
    uint32_t        tick_chargecheck;           // tick to check fast-charge-finish, every 64ms
    uint32_t        tick_constcharge;           // tick to adjust const i_charge, and check const-charge-timer
    uint32_t        pre_start_time;             // pre-charge start moment
    uint32_t        fast_start_time;            // fast-charge start moment
    uint32_t        trickle_start_time;         // trickle-charge start moment
    uint32_t        charge_finish_time;	        // last one cycle charge finsh moment (minutes)
    uint32_t        peak_time;                  // charge reach peak voltage time (second)
    uint32_t        ntc_open_time;              // to avoid spurious error log for ntc-open
    uint32_t        cell_open_time;             // to avoid spurious error log for cell-open
    float           fastcharge_timer;           // fast charge time out protection
    float           chgdisp_timer;              // fast charge remain time display
    float           constant_timer;             // constant charge time out
    int32_t         cmdchgoff_timer;            // CAN command force charge off timer, after this charge could start if it need
    uint32_t        chgtmr_update;              // for fastcharge_timer update
    uint32_t        chgdisp_tmr_update;         // tick for charge time display calc
    uint16_t        charge_times;               // total charge times of this battery
    float           q_acm_charge;               // total charge coulomb
    float           q_acm_discharge;            // total discharge coulomb
    int16_t         red_line[RL_DEP][RL_WID];   // system empty level matrix [temperature every 3C 0~66C][load_cunsumption 1A ~ 10A]
    float           alpha;
    float           beta;
    float           gamma;
    uint16_t        err_code;
    uint8_t         finish_code;
    uint32_t        null_dev;
};
typedef struct StBattery stBattery;

/* Exported constants --------------------------------------------------------*/

/* Exported variables ------------------------------------------------------- */
extern stBattery Battery_1, Battery_2;

/* Exported functions ------------------------------------------------------- */
void BatteryHwInit(void);
void BatterySupplyCmdHandler(stBattery *bat, bool request);
void ChargeCmdhandler(stBattery *bat, bool request);
void Thread_BatteryMonitor(const void *);
void Thread_BatteryControl(void const *param);

//#ifdef __cplusplus
//}
//#endif
#endif /*__BATTERY_H */

