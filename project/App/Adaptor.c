/**
  ******************************************************************************
  * @file    Project/src/Adaptor.c
  * @author
  * @version V0.00
  * @date
  * @brief   Adaptor.c
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "app.h"

/* Private macro -------------------------------------------------------------*/
#define ADAPT_CONNN_TH      12
#define ADAPT_DISCONN_TH    9
#define ADAPT_OVERLOAD_TH   11.5
#define HIGH_LOAD_AMPS      9
#define LOW_LOAD_AMPS       6

/* Private typedef -----------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
stAdaptor Adaptor = {
    .voltage = 0,
    .status = kAdaptorNotExist,
    .connect_time = -1,
    .disconnect_time = 0
};

static uint32_t second_11v5 = 0;


/* Code begin ----------------------------------------------------------------*/
void
Thread_Adaptor(const void *param)
{
//    uint32_t tick = 0;
//    uint32_t water_mark = 0;
    while (1) 
    {
        Adaptor.voltage = (float)ADCvalue[ADC_ADAPTOR] * 0.003742;             // adc / 100 / 0.0015;
        
        if (Adaptor.status == kAdaptorNotExist && Adaptor.voltage > ADAPT_CONNN_TH) {
            Adaptor.status = kAdaptorSupplying;
            Adaptor.connect_time = GetSecond();
            Adaptor.disconnect_time = -1;
            vprintf("AdaptorInfoUpdate(): connect, time: %d\n\r", Adaptor.connect_time);
        } else if (Adaptor.status == kAdaptorSupplying) {
            if (Adaptor.voltage <= ADAPT_DISCONN_TH) {
                Adaptor.status = kAdaptorNotExist;
                second_11v5 = 0;
                Adaptor.disconnect_time = GetSecond();
                Adaptor.connect_time = -1;
                vprintf("AdaptorInfoUpdate(): disconnect, time: %d\n\r", Adaptor.disconnect_time);
            } else if (Adaptor.voltage <= ADAPT_OVERLOAD_TH) {
                if (second_11v5 == 0) {
                    second_11v5 = GetSecond();
                } else if (GetSecond() - second_11v5 >= 3) {
                    Adaptor.status = kAdaptorOverLoad;
                    vprintf("AdaptorInfoUpdate(): overload, time: %d\n\r", second_11v5);
                }
            } else {
                second_11v5 = 0;
            }
        } else if (Adaptor.status == kAdaptorOverLoad) {
            // lock this flag
            if (Adaptor.voltage > 12)
                Adaptor.status = kAdaptorSupplying;
        }
        
//        tick = xTaskGetTickCount();
//        water_mark = uxTaskGetStackHighWaterMark(tid_adaptor);
//        vprintf("Adaptor: tick %d, watermark %d\n\r", tick, water_mark);
        vTaskDelay(50 / portTICK_PERIOD_MS);    // sleep 50ms
    }

    
}


