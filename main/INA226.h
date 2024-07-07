#ifndef _INA226_H_
#define _INA226_H_

#include <stdio.h>                  //C/C++标准输入输出库
#include "esp_log.h"                //打印输出log库
#include <math.h>                   //数学运算函数库（包含宏定义)
#include "bsp_i2c.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
/*
* INA226 Address Pins and Slave Addresses
* INA226 A1:GND A0:GND 1000000 0x40
* INA226 A1:GND A0:VS  1000001 0x41
* INA226 A1:GND A0:SDA 1000010 0x42
* INA226 A1:GND A0:SCL 1000011 0x43
* INA226 A1:VS  A0:GND 1000100 0x44
* INA226 A1:VS  A0:VS  1000101 0x45
* INA226 A1:VS  A0:SDA 1000110 0x46
* INA226 A1:VS  A0:SCL 1000111 0x47
* INA226 A1:SDA A0:GND 1001000 0x48
* INA226 A1:SDA A0:VS  1001001 0x49
* INA226 A1:SDA A0:SDA 1001010 0x4A
* INA226 A1:SDA A0:SCL 1001011 0x4B
* INA226 A1:SCL A0:GND 1001100 0x4C
* INA226 A1:SCL A0:VS  1001101 0x4D
* INA226 A1:SCL A0:SDA 1001110 0x4E
* INA226 A1:SCL A0:SCL 1001111 0x4F
*/
#define INA226_ADDR_GND_GND 0x40 //!< I2C address, A1 pin - GND, A0 pin - GND
#define INA226_ADDR_GND_VS  0x41 //!< I2C address, A1 pin - GND, A0 pin - VS+
#define INA226_ADDR_GND_SDA 0x42 //!< I2C address, A1 pin - GND, A0 pin - SDA
#define INA226_ADDR_GND_SCL 0x43 //!< I2C address, A1 pin - GND, A0 pin - SCL
#define INA226_ADDR_VS_GND  0x44 //!< I2C address, A1 pin - VS+, A0 pin - GND
#define INA226_ADDR_VS_VS   0x45 //!< I2C address, A1 pin - VS+, A0 pin - VS+
#define INA226_ADDR_VS_SDA  0x46 //!< I2C address, A1 pin - VS+, A0 pin - SDA
#define INA226_ADDR_VS_SCL  0x47 //!< I2C address, A1 pin - VS+, A0 pin - SCL
#define INA226_ADDR_SDA_GND 0x48 //!< I2C address, A1 pin - SDA, A0 pin - GND
#define INA226_ADDR_SDA_VS  0x49 //!< I2C address, A1 pin - SDA, A0 pin - VS+
#define INA226_ADDR_SDA_SDA 0x4A //!< I2C address, A1 pin - SDA, A0 pin - SDA
#define INA226_ADDR_SDA_SCL 0x4B //!< I2C address, A1 pin - SDA, A0 pin - SCL
#define INA226_ADDR_SCL_GND 0x4C //!< I2C address, A1 pin - SCL, A0 pin - GND
#define INA226_ADDR_SCL_VS  0x4D //!< I2C address, A1 pin - SCL, A0 pin - VS+
#define INA226_ADDR_SCL_SDA 0x4E //!< I2C address, A1 pin - SCL, A0 pin - SDA
#define INA226_ADDR_SCL_SCL 0x4F //!< I2C address, A1 pin - SCL, A0 pin - SCL

/*
* Register Set Summary
* 00h   Configuration Register
* 01h   Shunt Voltage Register
* 02h   Bus Voltage Register
* 03h   Power Register
* 04h   Current Register
* 05h   Calibration Register
* 06h   Mask/Enable Register
* 07h   Alert Limit Register
* FEh   Manufacturer ID Register
* FFh   Die ID Register
*/
#define INA226_REG_CONFIG           (0x00)
#define INA226_REG_SHUNTVOLTAGE     (0x01)
#define INA226_REG_BUSVOLTAGE       (0x02)
#define INA226_REG_POWER            (0x03)
#define INA226_REG_CURRENT          (0x04)
#define INA226_REG_CALIBRATION      (0x05)
#define INA226_REG_MASKENABLE       (0x06)
#define INA226_REG_ALERTLIMIT       (0x07)
#define INA226_REG_MANUFACTURERID   (0xFE)
#define INA226_REG_DIEID            (0xFF)


#define INA226_BIT_SOL              (0x8000)
#define INA226_BIT_SUL              (0x4000)
#define INA226_BIT_BOL              (0x2000)
#define INA226_BIT_BUL              (0x1000)
#define INA226_BIT_POL              (0x0800)
#define INA226_BIT_CNVR             (0x0400)
#define INA226_BIT_AFF              (0x0010)
#define INA226_BIT_CVRF             (0x0008)
#define INA226_BIT_OVF              (0x0004)
#define INA226_BIT_APOL             (0x0002)
#define INA226_BIT_LEN              (0x0001)

// const char *TAG_INA226 = "INA226-test";    //字符型指针 存储 项目名字，不可修改

/*
* 第9~11bit register 控制芯片电压结果平均次数
* VBUSCT[11:9] D11 D10 D9
* 枚举类型 typedef enum
* 标记名(别名)   ina226_averages_t
* default value 000
*/
typedef enum
{
    INA226_AVERAGES_1             = 0b000,
    INA226_AVERAGES_4             = 0b001,
    INA226_AVERAGES_16            = 0b010,
    INA226_AVERAGES_64            = 0b011,
    INA226_AVERAGES_128           = 0b100,
    INA226_AVERAGES_256           = 0b101,
    INA226_AVERAGES_512           = 0b110,
    INA226_AVERAGES_1024          = 0b111
} ina226_averages_t;

/*
* 第6~8bit register 控制芯片电压采样结果转换时长
* VBUSCT[8:6] D8 D7 D6
* 枚举类型 typedef enum
* 标记名(别名)   ina226_busConvTime_t
* default value 000
*/
typedef enum
{
    INA226_BUS_CONV_TIME_140US    = 0b000,
    INA226_BUS_CONV_TIME_204US    = 0b001,
    INA226_BUS_CONV_TIME_332US    = 0b010,
    INA226_BUS_CONV_TIME_588US    = 0b011,
    INA226_BUS_CONV_TIME_1100US   = 0b100,
    INA226_BUS_CONV_TIME_2116US   = 0b101,
    INA226_BUS_CONV_TIME_4156US   = 0b110,
    INA226_BUS_CONV_TIME_8244US   = 0b111
} ina226_busConvTime_t;

/*
* 第3~5bit register 控制芯片电压差/电流的采样结果转换时长
* VSHCT[5:3] D5 D4 D3
* 枚举类型 typedef enum
* 标记名(别名)   ina226_shuntConvTime_t
* default value 100
*/
typedef enum
{
    INA226_SHUNT_CONV_TIME_140US   = 0b000,
    INA226_SHUNT_CONV_TIME_204US   = 0b001,
    INA226_SHUNT_CONV_TIME_332US   = 0b010,
    INA226_SHUNT_CONV_TIME_588US   = 0b011,
    INA226_SHUNT_CONV_TIME_1100US  = 0b100,
    INA226_SHUNT_CONV_TIME_2116US  = 0b101,
    INA226_SHUNT_CONV_TIME_4156US  = 0b110,
    INA226_SHUNT_CONV_TIME_8244US  = 0b111
} ina226_shuntConvTime_t;

/*
* 第0~2bit register Operating Mode setting
* VBUSCT[2:0] D2 D1 D0
* 枚举类型 typedef enum
* 标记名(别名)   ina226_mode_t
* default value 111
*/
typedef enum
{
    INA226_MODE_POWER_DOWN      = 0b000,
    INA226_MODE_SHUNT_TRIG      = 0b001,
    INA226_MODE_BUS_TRIG        = 0b010,
    INA226_MODE_SHUNT_BUS_TRIG  = 0b011,
    INA226_MODE_ADC_OFF         = 0b100,
    INA226_MODE_SHUNT_CONT      = 0b101,
    INA226_MODE_BUS_CONT        = 0b110,
    INA226_MODE_SHUNT_BUS_CONT  = 0b111,
} ina226_mode_t;

/*
* 结构体类型 typedef struct
* 标记名(别名)   INA226
*/
typedef struct {
    float currentLSB;
    float powerLSB;
    float vShuntMax;
    float vBusMax;
    float rShunt;
} INA226;

static esp_err_t INA226_register_read(uint8_t reg_addr, uint8_t *data, size_t len);
static esp_err_t INA226_register_write_2_bytes(uint8_t reg_addr, uint16_t value);
static esp_err_t INA226_register_write_byte(uint8_t reg_addr, uint8_t data);

void INA226_configure(INA226 *ina, ina226_averages_t avg, ina226_busConvTime_t busConvTime, ina226_shuntConvTime_t shuntConvTime, ina226_mode_t mode);
void INA226_calibrate(INA226 *ina, float rShuntValue, float iMaxExpected);

/*Fuction declaration*/
float INA226_readBusVoltage(INA226 *ina);           
float INA226_readBusPower(INA226 *ina);
float INA226_readShuntCurrent(INA226 *ina);
float INA226_readShuntVoltage(INA226 *ina);
void INA226_Get_Data(float *BusPowerVoltage,float *ShuntCurrent);

#endif /*INA226_ESP_IDF_INA226_H*/
