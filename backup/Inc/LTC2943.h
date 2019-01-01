/*!
LTC2943: Multicell Battery Gas Gauge with Temperature, Voltage and Current Measurement.
LTC2943-1: Multicell Battery Gas Gauge with Temperature, Voltage and Current Measurement.

@verbatim

The LTC2943 measures battery charge state, battery voltage,
battery current and its own temperature in portable
product applications. The wide input voltage range allows
use with multicell batteries up to 20V. A precision coulomb
counter integrates current through a sense resistor between
the battery’s positive terminal and the load or charger.
Voltage, current and temperature are measured with an
internal 14-bit No Latency ΔΣ™ ADC. The measurements
are stored in internal registers accessible via the onboard
I2C/SMBus Interface.

I2C DATA FORMAT (MSB FIRST):

Data Out:
Byte #1                                    Byte #2                     Byte #3

START  SA6 SA5 SA4 SA3 SA2 SA1 SA0 W SACK  C7  C6 C5 C4 C3 C2 C1 C0 SACK D7 D6 D5 D4 D3 D2 D1 D0 SACK  STOP

Data In:
Byte #1                                    Byte #2                                    Byte #3

START  SA6 SA5 SA4 SA3 SA2 SA1 SA0 W SACK  C7  C6  C5 C4 C3 C2 C1 C0 SACK  Repeat Start SA6 SA5 SA4 SA3 SA2 SA1 SA0 R SACK

Byte #4                                   Byte #5
MSB                                       LSB
D15 D14  D13  D12  D11  D10  D9 D8 MACK   D7 D6 D5 D4 D3  D2  D1  D0  MNACK  STOP

START       : I2C Start
Repeat Start: I2c Repeat Start
STOP        : I2C Stop
SAx         : I2C Address
SACK        : I2C Slave Generated Acknowledge (Active Low)
MACK        : I2C Master Generated Acknowledge (Active Low)
MNACK       : I2c Master Generated Not Acknowledge
W           : I2C Write (0)
R           : I2C Read  (1)
Cx          : Command Code
Dx          : Data Bits
X           : Don't care



Example Code:

Read charge, current, and voltage.

    adc_command = LTC2943_SENSE_MONITOR | LTC2943_AUTOMATIC_MODE; // Builds commands to set LTC2943 to automatic mode
    ack |= LTC2943_write(LTC2943_I2C_ADDRESS, LTC2943_CONTROL_REG, adc_command);   // Sets the LTC2943 to automatic mode

    resistor = .1; // Resistor Value On Demo Board

    ack |= LTC2943_read_16_bits(LTC2943_I2C_ADDRESS, LTC2943_CHARGE_MSB_REG, &charge_code);  // Reads the ADC registers that contains charge value
    charge = LTC2943_code_to_coulombs(charge_code, resistor, prescalarValue); // Calculates charge from charge code, resistor and prescalar

    ack |= LTC2943_read_16_bits(LTC2943_I2C_ADDRESS, LTC2943_CURRENT_MSB_REG, &current_code); // Reads the voltage code across sense resistor
    current = LTC2943_code_to_current(current_code, resistor); // Calculates current from current code, resistor value.

    ack |= LTC2943_read_16_bits(LTC2943_I2C_ADDRESS, LTC2943_VOLTAGE_MSB_REG, &voltage_code);   // Reads voltage voltage code
    VIN = LTC2943_VIN_code_to_voltage(voltage_code);  // Calculates VIN voltage from VIN code and lsb


@endverbatim

http://www.linear.com/product/LTC2943
http://www.linear.com/product/LTC2943-1

http://www.linear.com/product/LTC2943#demoboards
http://www.linear.com/product/LTC2943-1#demoboards

*/

/*! @file
    @ingroup LTC2943
    Header for LTC2943: Multicell Battery Gas Gauge with Temperature, Voltage and Current Measurement
*/


#ifndef LTC2943_H
#define LTC2943_H

//#ifdef __cplusplus
//extern "C" {
//#endif

/* Includes ------------------------------------------------------------------*/
#include "stdbool.h"
#include "stdint.h"
#include "stm32f3xx_hal_i2c.h"
/* Exported macro ------------------------------------------------------------*/

#define LTC2943_DEVADDR 0xC8

#define LTC2943_STATUS_REG                          0x00
#define LTC2943_CONTROL_REG                         0x01
#define LTC2943_ACCUM_CHARGE_MSB_REG                0x02
#define LTC2943_ACCUM_CHARGE_LSB_REG                0x03
#define LTC2943_CHARGE_THRESH_HIGH_MSB_REG          0x04
#define LTC2943_CHARGE_THRESH_HIGH_LSB_REG          0x05
#define LTC2943_CHARGE_THRESH_LOW_MSB_REG           0x06
#define LTC2943_CHARGE_THRESH_LOW_LSB_REG           0x07
#define LTC2943_VOLTAGE_MSB_REG                     0x08
#define LTC2943_VOLTAGE_LSB_REG                     0x09
#define LTC2943_VOLTAGE_THRESH_HIGH_MSB_REG         0x0A
#define LTC2943_VOLTAGE_THRESH_HIGH_LSB_REG         0x0B
#define LTC2943_VOLTAGE_THRESH_LOW_MSB_REG          0x0C
#define LTC2943_VOLTAGE_THRESH_LOW_LSB_REG          0x0D
#define LTC2943_CURRENT_MSB_REG                     0x0E
#define LTC2943_CURRENT_LSB_REG                     0x0F
#define LTC2943_CURRENT_THRESH_HIGH_MSB_REG         0x10
#define LTC2943_CURRENT_THRESH_HIGH_LSB_REG         0x11
#define LTC2943_CURRENT_THRESH_LOW_MSB_REG          0x12
#define LTC2943_CURRENT_THRESH_LOW_LSB_REG          0x13
#define LTC2943_TEMPERATURE_MSB_REG                 0x14
#define LTC2943_TEMPERATURE_LSB_REG                 0x15
#define LTC2943_TEMPERATURE_THRESH_HIGH_REG         0x16
#define LTC2943_TEMPERATURE_THRESH_LOW_REG          0x17

#define LTC2943_AUTOMATIC_MODE                  0xC0
#define LTC2943_SCAN_MODE                       0x80
#define LTC2943_MANUAL_MODE                     0x40
#define LTC2943_SLEEP_MODE                      0x00

#define LTC2943_ACR_OVF_UDF 0x20
#define LTC2943_ALERT_H     0x08
#define LTC2943_ALERT_L     0x04

#define LTC2943_PRESCALAR_M_1                   0x00
#define LTC2943_PRESCALAR_M_4                   0x08
#define LTC2943_PRESCALAR_M_16                  0x10
#define LTC2943_PRESCALAR_M_64                  0x18
#define LTC2943_PRESCALAR_M_256                 0x20
#define LTC2943_PRESCALAR_M_1024                0x28
#define LTC2943_PRESCALAR_M_4096                0x30
#define LTC2943_PRESCALAR_M_4096_2              0x38

#define LTC2943_ALERT_MODE                      0x04
#define LTC2943_CHARGE_COMPLETE_MODE            0x02

#define LTC2943_DISABLE_ALCC_PIN                0x00
#define LTC2943_SHUTDOWN_MODE                   0x01


#define LTC2943_CHARGE_lsb 		0.34E-3f
#define LTC2943_VOLTAGE_lsb		1.44E-3f
#define LTC2943_CURRENT_lsb		29.3E-6f
#define LTC2943_TEMPERATURE_lsb		0.25f
#define LTC2943_FULLSCALE_VOLTAGE	23.6f
#define LTC2943_FULLSCALE_CURRENT	60E-3f
#define LTC2943_FULLSCALE_TEMPERATURE	510

#define R_SENSE        0.003
#define PRESCALAR      1024

/* Exported types ------------------------------------------------------------*/
typedef struct
{
    unsigned char status;
    unsigned char control;
    float level;
    float voltage;
    float current;
    float temperature;
    int8_t  acr_ofuf;   // positive: overflow times, negative: underflow times, default: 0
    I2C_HandleTypeDef *hi2c;
} stGauge;

/* Exported variables ------------------------------------------------------- */
extern stGauge Gauge1;
extern stGauge Gauge2;

/* Exported functions ------------------------------------------------------- */
HAL_StatusTypeDef Init_LTC2943(stGauge *const gauge);
HAL_StatusTypeDef LTC2943_Write_mAh(stGauge *gauge,float level);
HAL_StatusTypeDef Get_Gauge_Information(stGauge *gauge);


//#ifdef __cplusplus
//}
//#endif

#endif  // LTC2943_H
