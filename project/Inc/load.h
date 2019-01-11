/**
  ******************************************************************************
  * @file    Project/src/Load.h 
  * @author  
  * @version V0.00
  * @date    
  * @brief   Load.h
  ******************************************************************************
  */  

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __Load_H
#define __Load_H
#ifdef __cplusplus
 extern "C" {
#endif


/* Includes ------------------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
#define DEFAULT_KP  1000
#define DEFAULT_KI  4
#define DEFAULT_KD  0

/* Exported types ------------------------------------------------------------*/
typedef enum {
    kLoadOff = 0,
    kLoadOn,
    kLoadError
} eBasicStatus;

typedef enum {
    kHeaterOff,
    kHeaterPID,
    kHeaterFixPWM,
} eHeaterMode;

typedef struct {
	eBasicStatus                status;
	GPIO_TypeDef *         		port;
	uint16_t                    const pin;
    GPIO_TypeDef *         		open_detect_port;
	uint16_t                    const open_detect_pin;
    int8_t                      err_code;
} DioDevice_t;

typedef struct {
    eBasicStatus                status;
    TIM_HandleTypeDef *   		tim;
    const uint32_t              tim_channel;
    const uint32_t              period;
    float                       duty;               // 0 or 1 for dio type, 0~1 float value for pwm type
} PwmDevice_t;

typedef struct {
    PwmDevice_t     pwm;
    eHeaterMode     mode;
    float           temperature;
    float           setpoint;
    int16_t         kp;
    int16_t         ki;
    int16_t         kd;
    int8_t          err_code;
} Heater_t;

typedef struct {
    DioDevice_t     dio;
    int8_t          err_code;
    TimerHandle_t * timer_on;
    TimerHandle_t * timer_off;
} PulseDevice_t;

/* Exported constants --------------------------------------------------------*/

/* Exported variables ------------------------------------------------------- */
extern DioDevice_t Fan;
extern DioDevice_t Valve_1;
extern DioDevice_t Valve_2;
extern DioDevice_t MB_Pwr;
extern DioDevice_t CCB_Pwr;
extern PulseDevice_t Pump_1;
extern PulseDevice_t Pump_2;
extern PulseDevice_t PValve;
extern Heater_t Heater;
	 
/* Exported functions ------------------------------------------------------- */
void DioSetTo(DioDevice_t *dev, uint8_t val);
void PwmSetTo(PwmDevice_t *dev, float duty);
bool OpenDetect(DioDevice_t *load);
void timer_pulse_on_callback(TimerHandle_t id);
void timer_pulse_off_callback(TimerHandle_t id);

#ifdef __cplusplus
}
#endif
#endif /*__Load_H */

