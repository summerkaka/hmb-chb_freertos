/**
  ******************************************************************************
  * @file    Project/src/adc_app.h
  * @author
  * @version V0.00
  * @date
  * @brief   adc_app.h
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ADC_APP_H
#define __ADC_APP_H
//#ifdef __cplusplus
// extern "C" {
//#endif


/* Includes ------------------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
#define ADC_4236IMON    0
#define ADC_NTC1		1
#define ADC_NTC2		2
#define ADC_ADAPTOR	    3
#define ADC_SYSPWR		4
#define ADC_OFFSET1     5
#define ADC_OFFSET2     6
#define ADC_OFFSET3     7


/* Exported types ------------------------------------------------------------*/


/* Exported constants --------------------------------------------------------*/
#define ADCDMA_BUF_SIZE 8


/* Exported variables ------------------------------------------------------- */
extern uint16_t ADCvalue[ADCDMA_BUF_SIZE];


/* Exported functions ------------------------------------------------------- */
void ADC_Config(void);

//#ifdef __cplusplus
//}
//#endif

#endif /*__ADC_APP_H */


