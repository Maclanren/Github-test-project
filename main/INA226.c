#include "INA226.h"

const char *TAG_INA226 = "INA226-test";    //字符型指针 存储 项目名字，不可修改

// extern esp_err_t Device_i2c_init(void);

/*INA226读主机写入的数据
* @param[name]  I2C_MASTER_NUM          :
* @param[name]  INA226_ADDR_GND_GND          :设备的7位地址 0x40
* @param[name]  reg_addr                :
* @param[name]  *data                   :
* @param[name]  len                     :
* @param[name]  I2C_MASTER_TIMEOUT_MS   :
* @param[name]  portTICK_RATE_MS        :
*/

static esp_err_t INA226_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, INA226_ADDR_GND_GND, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
}

static esp_err_t INA226_register_write_2_bytes(uint8_t reg_addr, uint16_t value)
{
    int ret;
    uint8_t write_buf[3] = {reg_addr, value >> 8, value & 0xFF};

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, INA226_ADDR_GND_GND, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);

    return ret;
} 

static esp_err_t INA226_register_write_byte(uint8_t reg_addr, uint8_t value)
{
    int ret;
    uint8_t write_buf[2] = {reg_addr, value};
    ret = i2c_master_write_to_device(I2C_MASTER_NUM, INA226_ADDR_GND_GND, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);

    return ret;
}

/*
* *ina          INA226成员的地址
* avg           采样精度 16bit
* busConvTime   BUS电压的采样转换时长
* shuntConvTime 电压差/电流的采样转换时长
* mode          芯片工作模式设置
*/

void INA226_configure(INA226 *ina, ina226_averages_t avg, ina226_busConvTime_t busConvTime, ina226_shuntConvTime_t shuntConvTime, ina226_mode_t mode)
{
    uint16_t config = 0;

    /*avg:D11-D9 busConvTime:D8-D6 shuntConvTime:D5-D3 mode:D2-D0*/
    config |= (avg << 9 | busConvTime << 6 | shuntConvTime << 3 | mode);

    ina->vBusMax = 36;                      //VBUS端输入电压 <= 36V
    ina->vShuntMax = 0.08192f;              //采样电阻两端之间的压差 <= 81.92mV

    // ESP_ERROR_CHECK(INA226_register_write_byte(INA226_REG_CONFIG, config));
    ESP_ERROR_CHECK(INA226_register_write_2_bytes(INA226_REG_CONFIG, config));

    ESP_LOGI(TAG_INA226, "Configured with config %d", config);

    uint8_t data[2];
    INA226_register_read(INA226_REG_CONFIG, data, 2);
    int16_t cfg = (data[0] << 8 | data[1]);
    ESP_LOGI(TAG_INA226, "Checking config value %d", cfg);
}

/*INA226 校准函数*/
void INA226_calibrate(INA226 *ina, float rShuntValue, float iMaxExpected)
{
    ESP_LOGI(TAG_INA226, "Going to calibrate...");
    ina->rShunt = rShuntValue;

    /* */
    float iMaxPossible = ina->vShuntMax / ina->rShunt;

    float minimumLSB = iMaxExpected / 32767;

    ina->currentLSB = (uint16_t)(minimumLSB * 100000000);
    ina->currentLSB /= 100000000;
    ina->currentLSB /= 0.0001;
    ina->currentLSB = ceil(ina->currentLSB);
    ina->currentLSB *= 0.0001;

    ina->powerLSB = ina->currentLSB * 25;

    uint16_t calibrationValue = (uint16_t)((0.00512) / (ina->currentLSB * ina->rShunt));
    /* */

    /* * /
    ina->currentLSB = iMaxExpected / 32768.0f;
    uint16_t calibrationValue = (uint16_t) (0.00512 / (ina->currentLSB * rShuntValue));
    / * */

    INA226_register_write_2_bytes(INA226_REG_CALIBRATION, calibrationValue);
    ESP_LOGI(TAG_INA226, "currentLSB is %f, calibration value is %d", ina->currentLSB, calibrationValue);
    ESP_LOGI(TAG_INA226, "Calibrated...");
}

/*
 * INA226读取总线电压算法
 * @param[in]       pina                    :INA226结构体变量名
 * @call function   INA226_register_read    :读取寄存器调用函数API
 * @call_f[one]     INA226_REG_BUSVOLTAGE   :地址 0x02(02h)
 * @call_f[two]     data  		            :需要校验的数据长度
 * @call_f[three]   2                       :数据长度
 * @retval          float                   :校验值
 */
float INA226_readBusVoltage(INA226 *ina)
{
    uint8_t data[2];
    INA226_register_read(INA226_REG_BUSVOLTAGE, data, 2);
    int16_t voltage = (data[0] << 8 | data[1]);

    return (float)voltage * 0.00125f;
}


float INA226_readBusPower(INA226 *ina)
{
    uint8_t data[2];
    INA226_register_read(INA226_REG_POWER, data, 2);
    int16_t power = (data[0] << 8 | data[1]);
    return ((float)power * ina->powerLSB);
}

float INA226_readShuntCurrent(INA226 *ina)
{
    uint8_t data[2];
    INA226_register_read(INA226_REG_CURRENT, data, 2);
    int current = (data[0] << 8 | data[1]);
    // ESP_LOGI(TAG, "Raw current value read: %d", current);
    // For some reason it needs to be divided by 2...
    return ((float)current * ina->currentLSB / 2.0f);
}

float INA226_readShuntVoltage(INA226 *ina)
{
    uint8_t data[2];
    INA226_register_read(INA226_REG_SHUNTVOLTAGE, data, 2);
    int16_t voltage = (data[0] << 8 | data[1]);

    return (float)voltage * 2.5e-6f; // fixed to 2.5 uV 2.5e-6=2.5*10^-6=0.0000025
}

void INA226_Get_Data(float *BusPowerVoltage,float *ShuntCurrent)
{
    INA226 ina;                                  		//定义一个INA226结构的变量名
    INA226_configure(&ina,                              //INA226成员的地址
                     INA226_AVERAGES_16,                //采样精度 16bit
                     INA226_BUS_CONV_TIME_1100US,       //BUS电压的采样转换时长1.1ms
                     INA226_SHUNT_CONV_TIME_1100US,     //电压差/电流的采样转换时长1.1ms
                     INA226_MODE_SHUNT_BUS_CONT);       //BUS电压和 电压差持续采样模式
    INA226_calibrate(&ina, 0.002f, 10);

    *BusPowerVoltage = (float) INA226_readBusVoltage(&ina);
    *ShuntCurrent = (float) INA226_readShuntCurrent(&ina);
    vTaskDelay(500 / portTICK_RATE_MS);
}