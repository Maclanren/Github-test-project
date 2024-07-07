
#include "bsp_i2c.h"

const char *TAG_Sensor = "Sensor:  ";              //静态字符型指针 存储 项目名字，不可修改

// /**
//  * ****************************************************************************************       
//  * @funcname    i2c_master_read_slave
//  * @functional  Read slave device datas.
//  * @param       
//  * @return      ret
//  * _______________________________________________________________________________________
//  * | start | slave_addr + rd_bit +ack | read n-1 bytes + ack | read 1 byte + nack | stop |
//  * --------|--------------------------|----------------------|--------------------|------|
//  *
//  * @note cannot use master write slave on esp32c3 because there is only one i2c controller on esp32c3（ESP32C3只有一路I2C)
//  * esp_err_t 错误处理机制，成功返回ESP_OK(0); 失败返回(非零的esp_err_t的值),用来指示发生了哪种类型的错误
//  * __attribute__((unused)) 为GCC编译器特定扩展属性，用来避免定义的函数未进行使用，编译忽略未使用警告。
//  */
esp_err_t __attribute__((unused)) i2c_master_read_slave(i2c_port_t i2c_num, uint8_t ESP_SLAVE_ADDR, uint8_t *data_rd, size_t size)
{
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ESP_SLAVE_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// /**
//  * ****************************************************************************************       
//  * @funcname    i2c_master_write_slave
//  * @functional  Write slave device datas.
//  * @param       
//  * @return      ret
//  * ___________________________________________________________________
//  * | start | slave_addr + wr_bit + ack | write n bytes + ack  | stop |
//  * --------|---------------------------|----------------------|------|
//  *
//  * @note cannot use master write slave on esp32c3 because there is only one i2c controller on esp32c3（ESP32C3只有一路I2C)
//  * esp_err_t 错误处理机制，成功返回ESP_OK(0); 失败返回(非零的esp_err_t的值),用来指示发生了哪种类型的错误
//  * __attribute__((unused)) 为GCC编译器特定扩展属性，用来避免定义的函数未进行使用，编译忽略未使用警告。
//  */
esp_err_t __attribute__((unused)) i2c_master_write_slave(i2c_port_t i2c_num, uint8_t ESP_SLAVE_ADDR, uint8_t *data_wr, size_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ESP_SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief i2c master initialization
 * esp_err_t 错误处理机制，成功返回ESP_OK(0); 失败返回(非零的esp_err_t的值),用来指示发生了哪种类型的错误
 */
esp_err_t Sensor_i2c_master_init(void)
{
    
    int i2c_master_port = I2C_MASTER_NUM;               // I2C主机端口号配置

    i2c_config_t conf = {                               // i2c配置结构体
        .mode = I2C_MODE_MASTER,                        // I2C模式 主机master
        .sda_io_num = I2C_MASTER_SDA_IO,                // I2C SDA IO映射
        .sda_pullup_en = GPIO_PULLUP_ENABLE,            // I2C SCL IO映射
        .scl_io_num = I2C_MASTER_SCL_IO,                // I2C SDA 上拉使能
        .scl_pullup_en = GPIO_PULLUP_ENABLE,            // I2C SCL 上拉使能
        .master.clk_speed = I2C_MASTER_FREQ_HZ,         // I2C CLK 频率设置
        // .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
    };

    /* 初始化I2C端口配置 I2C_param_config(端口号,模式参数配置); 
    * @success  ESP_OK
    * @error    ESP_ERR_INVALID_ARG Parameter 
    */ 
    esp_err_t err = i2c_param_config(i2c_master_port, &conf);

    //检查I2C初始化返回值
    if (err != ESP_OK) {
        return err;
    }
    //安装I2C驱动程序,也可以不加return，如果用到ESP_ERROR_CHECK(Sensor_i2c_init());则需要加 return ****;
    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}
