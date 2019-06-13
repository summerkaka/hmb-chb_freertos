/**
  ******************************************************************************
  * @file    Project/src/Adaptor.h
  * @author
  * @version V0.00
  * @date
  * @brief   Adaptor.h
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ADAPTOR_H
#define __ADAPTOR_H
//#ifdef __cplusplus
// extern "C" {
//#endif


/* Includes ------------------------------------------------------------------*/
#include "Battery.h"

/* Exported macro ------------------------------------------------------------*/


/* Exported types ------------------------------------------------------------*/
typedef enum {
    kAdaptorNotExist = 0,
    kAdaptorSupplying,
    kAdaptorOverLoad,
} eAdaptorStatus;

typedef struct {
    eAdaptorStatus	status;
    float           voltage;
    int32_t        connect_time;       // unit second
    int32_t        disconnect_time;    // unit second
} stAdaptor;

/* Exported constants --------------------------------------------------------*/

/* Exported variables ------------------------------------------------------- */
extern stAdaptor Adaptor;

/* Exported functions ------------------------------------------------------- */
void thread_adaptor(void *p);

//#ifdef __cplusplus
//}
//#endif
#endif /*__ADAPTOR_H */