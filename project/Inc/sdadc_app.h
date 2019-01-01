/**
  ******************************************************************************
  * @file    Project/src/sdadc_app.h
  * @author
  * @version V0.00
  * @date
  * @brief   sdadc_app.h
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SDADC_APP_H
#define __SDADC_APP_H
//#ifdef __cplusplus
// extern "C" {
//#endif


/* Includes ------------------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
#define SDADC1_CHN_NUM  4
// #define SDADC3_CHN_NUM  1
#define SDADC1_CHNL_REF      0
#define SDADC1_CHNL_PT100    1
#define SDADC1_CHNL_GAS1     2
#define SDADC1_CHNL_GAS2     3


/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported variables ------------------------------------------------------- */
extern int16_t sdadc1_code[SDADC1_CHN_NUM];
// extern int16_t sdadc3_code;
extern float sd1_gain_coe;

/* Exported functions ------------------------------------------------------- */
void Sdadc_Config(void);


//#ifdef __cplusplus
//}
//#endif
#endif /*__SDADC_APP_H */