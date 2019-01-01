/**
  ******************************************************************************
  * @file    Project/src/LTC2943.c
  * @author
  * @version V0.00
  * @date
  * @brief
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "app.h"

/* Private macro -------------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
stGauge Gauge1 = {
	.status = 0,
  	.control = 0,
  	.level = 0,
	.voltage = 0,
  	.current = 0,
	.temperature = 0,
	.hi2c = &hi2c1
};

stGauge Gauge2 = {
	.status = 0,
  	.control = 0,
  	.level = 0,
	.voltage = 0,
  	.current = 0,
	.temperature = 0,
	.hi2c = &hi2c2
};

/* Code begin ----------------------------------------------------------------*/
static HAL_StatusTypeDef
Read_LTC2943_Byte(const stGauge *gauge, uint8_t reg_addr, uint8_t *data)
{// The function returns the state of the acknowledge bit after the I2C address write. 0=acknowledge, 1=no acknowledge.
	return HAL_I2C_Mem_Read(gauge->hi2c, LTC2943_DEVADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, data, 1, 5);
}

static HAL_StatusTypeDef
Read_LTC2943_Word(const stGauge *gauge, uint8_t reg_addr_high, uint16_t *data)
{// The function returns the state of the acknowledge bit after the I2C address write. 0=acknowledge, 1=no acknowledge.
	uint8_t buffer[2] = {0};
	HAL_StatusTypeDef ret;

	ret = HAL_I2C_Mem_Read(gauge->hi2c, LTC2943_DEVADDR, reg_addr_high, I2C_MEMADD_SIZE_8BIT, buffer, 2, 5);
	if (ret != HAL_OK)
		return ret;
	*data = (uint16_t)buffer[1] | ((uint16_t)buffer[0] << 8);
	return ret;
}

static HAL_StatusTypeDef
Write_LTC2943_Byte(const stGauge *gauge, uint8_t reg_addr, uint8_t data)
{
	return HAL_I2C_Mem_Write(gauge->hi2c, LTC2943_DEVADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, &data, 1, 5);
}

static HAL_StatusTypeDef
Write_LTC2943_Word(const stGauge *gauge, uint8_t reg_addr_high, uint16_t data)
{
	uint8_t buffer[2];
	buffer[1] = data & 0xff;
	buffer[0] = (data >> 8);

	return HAL_I2C_Mem_Write(gauge->hi2c, LTC2943_DEVADDR, reg_addr_high, I2C_MEMADD_SIZE_8BIT, buffer, 2, 5);
}

HAL_StatusTypeDef
Init_LTC2943(stGauge *const gauge)
{
    //initialize Control register
    return Write_LTC2943_Byte(gauge,LTC2943_CONTROL_REG, LTC2943_AUTOMATIC_MODE | LTC2943_PRESCALAR_M_1024 |LTC2943_DISABLE_ALCC_PIN );
}

HAL_StatusTypeDef
LTC2943_Write_mAh(stGauge *gauge,float level)
{
    // uint16_t wdata = (uint16_t)(level / LTC2943_CHARGE_lsb / 50 * R_SENSE * 1000 / PRESCALAR * 4096);  // inv of code_to_mAh
    uint16_t wdata = (uint16_t)(level / 1.41666667f);  // inv of code_to_mAh
    if (level < 0 && wdata != 0) {
        gauge->acr_ofuf = -1;
    } else {
        gauge->acr_ofuf = 0;
    }
    printf("LTC2943_Write_mAh(): write level: %f, wdata: %d, acr_ofuf: %d\n\r", level, wdata, gauge->acr_ofuf);

    return Write_LTC2943_Word(gauge,LTC2943_ACCUM_CHARGE_MSB_REG, wdata);
}

float LTC2943_code_to_coulombs(uint16_t adc_code, float resistor, uint16_t prescalar)
// The function converts the 16-bit RAW adc_code to Coulombs
{
	float coulomb_charge;
	coulomb_charge =  1000*(float)(adc_code*LTC2943_CHARGE_lsb*prescalar*50E-3f)/(resistor*4096);
	coulomb_charge = coulomb_charge*3.6f;
	return (coulomb_charge);
}

float LTC2943_code_to_mAh(int32_t adc_code, float resistor, uint16_t prescalar )
// The function converts the 16-bit RAW adc_code to mAh
{
	float mAh_charge;
	// mAh_charge = 1000*(float)(adc_code*LTC2943_CHARGE_lsb*(int16_t)prescalar*0.05)/(resistor*4096);
    mAh_charge = (float)adc_code * 1.41666667f;
	return (mAh_charge);
}

float LTC2943_code_to_voltage(uint16_t adc_code)
// The function converts the 16-bit RAW adc_code to Volts
{
	float voltage;
	voltage = ((float)adc_code/(65535))*LTC2943_FULLSCALE_VOLTAGE;
	return (voltage);
}

float LTC2943_code_to_current(uint16_t adc_code, float resistor)
// The function converts the 16-bit RAW adc_code to Amperes
{
	float current;
	current = (((float)adc_code-32767)/(32767))*((float)(LTC2943_FULLSCALE_CURRENT)/resistor);
	return (current);
}

float LTC2943_code_to_kelvin_temperature(uint16_t adc_code)
// The function converts the 16-bit RAW adc_code to Kelvin
{
	float temperature;
	temperature = adc_code*((float)(LTC2943_FULLSCALE_TEMPERATURE)/65535);
	return (temperature);
}

float LTC2943_code_to_celcius_temperature(uint16_t adc_code)
// The function converts the 16-bit RAW adc_code to Celcius
{
	float temperature;
	temperature = adc_code*((float)(LTC2943_FULLSCALE_TEMPERATURE)/65535) - 273.15;
	return (temperature);
}

HAL_StatusTypeDef
Get_Gauge_Information(stGauge *gauge)
{
    uint16_t data_word=0;
    uint8_t data_byte=0;
    int32_t data_long = 0;
    HAL_StatusTypeDef ret = HAL_OK;

    //read Status register
    ret = Read_LTC2943_Byte(gauge, LTC2943_STATUS_REG, &data_byte);
    if (ret != HAL_OK)
        return ret;
    gauge->status = data_byte;

    //read control register
    ret = Read_LTC2943_Byte(gauge, LTC2943_CONTROL_REG, &data_byte);
    if (ret != HAL_OK)
        return ret;
    gauge->control = data_byte;

    //read Accumulated Charge register
    ret = Read_LTC2943_Word(gauge, LTC2943_ACCUM_CHARGE_MSB_REG, &data_word);
    if (ret != HAL_OK)
        return ret;
    if (gauge->status & LTC2943_ACR_OVF_UDF) {
        printf("Get_Gauge_Information(): acr_OVF_UDF is detect, sts_reg is 0x%02x, acr reading is %d, origin level is %.0f\n\r", gauge->status, data_word, gauge->level);
        if (data_word > 0xffe0) {       // under flow
            gauge->acr_ofuf--;
        } else if (data_word < 0x0020) { // over flow
            gauge->acr_ofuf++;
        }
    }
    if (gauge->acr_ofuf < 0) {
        data_long = gauge->acr_ofuf * 65536l + (int32_t)data_word;
    } else if (gauge->acr_ofuf > 0) {
        data_long = gauge->acr_ofuf * 65535l + (int32_t)data_word;
    } else {
        data_long = data_word;
    }
    gauge->level = LTC2943_code_to_mAh(data_long, R_SENSE, PRESCALAR);

    //read voltage register
    ret = Read_LTC2943_Word(gauge,LTC2943_VOLTAGE_MSB_REG, &data_word);
    if (ret != HAL_OK)
        return ret;
    gauge->voltage = LTC2943_code_to_voltage(data_word);

    //read current register
    ret = Read_LTC2943_Word(gauge,LTC2943_CURRENT_MSB_REG, &data_word);
    if (ret != HAL_OK)
        return ret;
    gauge->current = (data_word == 0) ? 0 : LTC2943_code_to_current(data_word, R_SENSE);

    //read on-board temperature
    ret = Read_LTC2943_Word(gauge,LTC2943_TEMPERATURE_MSB_REG, &data_word);
    if (ret != HAL_OK)
        return ret;
    gauge->temperature = (data_word == 0) ? 0 : LTC2943_code_to_celcius_temperature(data_word);

    return ret;
}


