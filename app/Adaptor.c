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
#include "app_include.h"

/* Private macro -------------------------------------------------------------*/
#define ADAPT_CONN_TH       12
#define ADAPT_DISCONN_TH    11
#define ADAPT_OVERLOAD_TH   11.8
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


/* Code begin ----------------------------------------------------------------*/
void thread_adaptor(void *p)
{
    uint32_t current_second;

    xprintf("thread_adaptor start...\n\r");

    while (1)
    {
        Adaptor.voltage = (float)ADCvalue[ADC_ADAPTOR] * 0.003742;             // adc / 100 / 0.0015;
        current_second = GetSecond();

        if (Adaptor.status == kAdaptorNotExist && Adaptor.voltage > ADAPT_CONN_TH) {
            Adaptor.status = kAdaptorSupplying;
            Adaptor.connect_time = current_second;
            Adaptor.disconnect_time = -1;
            xprintf("Adaptor: connect, time: %d\n\r", Adaptor.connect_time);
        } else if (Adaptor.status == kAdaptorSupplying) {
            if (Adaptor.voltage <= ADAPT_DISCONN_TH) {
                Adaptor.status = kAdaptorNotExist;
                Adaptor.disconnect_time = current_second;
                Adaptor.connect_time = -1;
                xprintf("Adaptor: disconnect, time: %d\n\r", Adaptor.disconnect_time);
            } else if (Adaptor.voltage <= ADAPT_OVERLOAD_TH) {
                osDelay(1000);
                Adaptor.voltage = (float)ADCvalue[ADC_ADAPTOR] * 0.003742;
                if (Adaptor.voltage <= ADAPT_DISCONN_TH) {
                    Adaptor.status = kAdaptorNotExist;
                    xprintf("Adaptor: disconnect, time: %d\n\r", Adaptor.disconnect_time);
                } else if (Adaptor.voltage <= ADAPT_OVERLOAD_TH) {
                    Adaptor.status = kAdaptorOverLoad;
                    xprintf("Adaptor: overload, time: %d\n\r", current_second);
                } 
            }
        } else if (Adaptor.status == kAdaptorOverLoad) {
            // lock this flag
            if (FieldCase.consumption < 6 && Adaptor.voltage >= ADAPT_CONN_TH)
                Adaptor.status = kAdaptorSupplying;
            else if (Adaptor.voltage <= ADAPT_DISCONN_TH) {
                osDelay(1000);
                Adaptor.voltage = (float)ADCvalue[ADC_ADAPTOR] * 0.003742;
                if (Adaptor.voltage <= ADAPT_DISCONN_TH) {
                    Adaptor.status = kAdaptorNotExist;
                    Adaptor.disconnect_time = current_second;
                    Adaptor.connect_time = -1;
                    xprintf("Adaptor: disconnect, time: %d\n\r", Adaptor.disconnect_time);
                }
            }
        }
    }
}


